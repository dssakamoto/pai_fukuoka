# TASK-0011 テストケース定義: IK(逆運動学 `solve_ik`)

**機能名**: ik
**タスクID**: TASK-0011
**要件名**: so101-vlm-pickplace
**出力ファイル**: `docs/implements/so101-vlm-pickplace/TASK-0011/ik-testcases.md`
**作成日**: 2026-10-20
**入力資料**: `ik-requirements.md`(本書の要件 ID「要件 T1〜T14」「3.x 節」はこの文書を指す)、`reachability-survey.md`、`note.md`、`docs/tasks/so101-vlm-pickplace/TASK-0011.md`、`tests/robot/test_sim.py`(書き方の参考)
**テストファイル**:
- `tests/policy/test_ik.py`(T1〜T35。単体テスト本体)
- `tests/policy/test_ik_survey.py`(T36〜T37。到達性の回帰。調査の格子から選んだ代表点)

**信頼性の凡例**: 🔵 要件定義書・設計文書・タスクファイルにほぼそのまま書いてある / 🟡 要件から妥当に推測、または使い捨て実行で数値を確認済み / 🔴 要件にない仮定

> **数値の確認方法**: 本書の期待値のうち「確認済み」と書いたものは、`ik-requirements.md` の仕様(方式 B: 下向き成分 1 行、wrist_roll 固定の 4 関節、候補 (0,10,20,30,45,60)、許容 ±5°、`POS_TOL_M`=1 mm、λ=0.01、更新上限 0.2 rad、300 反復、`ORIENTATION_WEIGHT`=0.1、`ctrlrange` でクリップ、seed + 固定の追加シード 9 個(うち wrist_flex 1.60〜1.65 rad を 3 個))どおりに書いた使い捨ての参照ソルバで確認した(プロジェクト外、`/private/tmp/.../scratchpad/ik_survey/tc/ref_ik.py`、`check1.py`〜`check4.py`。リポジトリには置かない)。実装の細部(追加シードの値など)が違っても結果が変わりにくい点だけを判定に使う。

---

## 0. 共通の前提

### 0.1 テストファイルの名前付き定数(テストファイル先頭)

| 定数 | 値 | 意味 | 信頼性 |
|---|---|---|---|
| `POS_TOL_TEST_M` | 0.002 | テストの位置誤差しきい値(ソルバの 1 mm に余裕) | 🔵(TASK-0011 テストケース 1) |
| `PITCH_TOL_TEST_DEG` | 5.0 | 候補ピッチとの差の許容 | 🟡(要件 3.1) |
| `DEFAULT_CANDIDATES` | (0.0, 10.0, 20.0, 30.0, 45.0, 60.0) | 既定の候補 | 🟡(要件 3.1) |
| `SEED_ZERO` | `np.zeros(6)` | 基本の seed | 🟡 |
| `GRASP_CENTER` | (0.20, 0.0, 0.015) | 作業範囲内の把持点(候補 0° で解ける) | 🟡(確認済み) |
| `PREGRASP` | (0.25, 0.05, 0.095) | 接近点(0° 不可、10° で解ける) | 🟡(確認済み) |
| `FAR_TARGET` | (1.0, 0.0, 0.2) | 到達球の外 | 🔵(note 6.6) |
| `NEAR_HIGH_TARGET` | (0.10, 0.0, 0.095) | 球内だが届かない(土台の近くで高い) | 🟡(確認済み) |
| `BEHIND_TARGET` | (−0.15, 0.0, 0.015) | 球内だが届かない(土台の真後ろ。shoulder_pan の可動域 ±110° の外) | 🟡(確認済み) |
| `TCP_OFFSET` | (−0.0075, 0.0, 0.015) | 把持中心の暫定オフセット(gripperframe 局所座標、m) | 🔴(要件 D5 の暫定範囲の中央) |
| `PAN_AXIS_POINT` | (0.0388, 0.0, 0.1166) | shoulder_pan 軸上で shoulder_lift 軸と同じ高さの点(m) | 🟡(実測: pan 軸 xy = (0.03884, 0)、lift アンカー z = 0.1166) |
| `ROBUST_OUT_DIST_M` | 0.452 | `PAN_AXIS_POINT` からこの距離の点はどの姿勢でも届かない | 🟡(下記 T20) |

### 0.2 fixture

- `scene_model`(`scope="module"`): `mujoco.MjModel.from_xml_path(str(PICKPLACE_SCENE_PATH))` を 1 回だけ読む。🟡(要件 7 節)
- `ctrlrange`(`scope="module"`): `scene_model.actuator_ctrlrange` を `JOINT_NAMES` 順に名前で引いた (6, 2) 配列のコピー。🔵(design 4.5)
- `grasp_solution`(`scope="module"`): `solve_ik(scene_model, GRASP_CENTER, SEED_ZERO)` の結果。T1・T4・T7 などで共有して呼び出し回数を減らす。🟡
- `near_high_error`(`scope="module"`): `NEAR_HIGH_TARGET` の `IkError` と所要時間 `(exc, elapsed_s)`。T13 と T35 で共有(収束しない呼び出しは重いので 1 回にする)。🟡
- 補助関数 `fk_point(model, q, offset=(0,0,0))`: **IK と独立した FK**。新しい `mujoco.MjData(model)` に `q` を関節名で `qpos` に入れ、`mj_kinematics` を呼び、`site_xpos["gripperframe"] + site_xmat @ offset` と、ピッチ `degrees(arccos(−site_xmat[2, 0]))`(site +x 軸と世界 −z のなす角)を返す。🟡(要件 2.2)

### 0.3 テスト時間の目安

| 対象 | 目安 | 根拠 | 信頼性 |
|---|---|---|---|
| `tests/policy/test_ik.py` 全体 | **約 3 秒以下**(目標)。最悪でも 8 秒 | 参照ソルバの実測: 候補 0° で解ける点 2〜5 ms、候補 0° が失敗して 10°/20°/30° で解ける点 0.27/0.5/0.8 s、収束しない点 1.6〜1.8 s(全候補 × 10 シード × 300 反復)。停滞打ち切り(25 反復で改善 0.1 mm 未満なら次のシードへ)を入れると、収束しない点が 0.25〜0.35 s、1 呼び出しの最大が 0.33 s になり、9×9×2 格子 + 箱の結果は変わらなかった | 🟡(確認済み) |
| `tests/policy/test_ik_survey.py` | 約 5 秒以下(打ち切りなしの見積もり約 4 秒) | 8 点のうち 5 点が候補 20°/30° | 🟡(確認済み) |

- 3 秒の目標は、Green の実装に「停滞打ち切り」を入れれば届く(未決事項 O1)。打ち切りがない場合でも、T33〜T35 の性能判定は満たす。

---

## 1. 正常系テストケース

### T1: 作業範囲中央の把持点で、真下に近い姿勢で収束する 🟡

- **テスト名**: `test_reachable_grasp_point_converges_within_2mm_near_vertical`
  - **何をテストするか**: 到達可能な把持点で解が得られ、位置誤差と傾きが要件を満たすこと
  - **期待される動作**: 候補 0° で収束した `IkSolution` を返す
- **入力値**: `target_pos=GRASP_CENTER (0.20, 0.0, 0.015)`、`seed_q=SEED_ZERO`、他は既定
  - **入力データの意味**: ブロック中心の高さ(`SCENE_SPEC.block_rest_z`)で、作業範囲の中央。調査では 0° の余裕が大きい(wrist_flex の上限まで約 0.45 rad)
- **期待される結果**:
  - `sol.pos_error <= POS_TOL_TEST_M`(参照ソルバ: 0.016 mm)
  - `sol.q.shape == (6,)`、`sol.q.dtype == np.float64`
  - `sol.pitch_candidate_deg == 0.0`、`sol.pitch_deg <= 5.0`(参照: 4.8°)
  - **期待結果の理由**: TASK-0011 テストケース 1(2 mm)、要件 4.1「把持点は候補 0°」
- **テストの目的**: 完了条件「到達可能な目標で収束する」
  - **確認ポイント**: `pitch_deg` は 5° の境界付近(4.7〜5.0°)に止まる(許容内に入った時点で止めるため)。`<= 5.0` で判定し、`< 4` のような厳しい値は使わない
- 🟡 要件 T1。数値は確認済み

### T2: 接近点は 0° では届かず、傾いた候補で収束する 🟡

- **テスト名**: `test_pregrasp_point_uses_tilted_candidate`
  - **何をテストするか**: 候補 0° が不可(wrist_flex の上限)の点で、次の候補に進んで解くこと
  - **期待される動作**: 候補 10°(または 20°)で収束する
- **入力値**: `PREGRASP (0.25, 0.05, 0.095)`、`SEED_ZERO`
  - **入力データの意味**: 把持点の 8 cm 上の接近点。調査で真下の最大高さ(x=0.25 で z 0.085)を超える
- **期待される結果**: `pos_error <= 0.002`、`pitch_candidate_deg in (10.0, 20.0)`、`abs(pitch_deg - pitch_candidate_deg) <= 5.0`(参照: 候補 10°、傾き 14.5°、0.47 mm)
  - **期待結果の理由**: 要件 4.1、調査 2.5 節
- **テストの目的**: ピッチ候補を真下に近い順に試す動作の確認
  - **確認ポイント**: `pitch_candidate_deg > 0`(0° の解を誤って返さない)
- 🟡 要件 T2。確認済み

### T3: 到達可能な複数の点で、返す q が全要素 ctrlrange 内 🔵

- **テスト名**: `test_solution_within_ctrlrange`(`@pytest.mark.parametrize` で 4 点)
  - **何をテストするか**: 解の 6 関節(gripper・wrist_roll を含む)が全て `actuator_ctrlrange` 内
  - **期待される動作**: `lo <= q <= hi`(等号を含む)
- **入力値**: `GRASP_CENTER`、`(0.15, -0.10, 0.015)`(手前の角、0°)、`(0.20, 0.19, 0.06)`(箱の解放点、0°)、`(0.35, 0.10, 0.015)`(奥の角、20°。wrist_flex 等が範囲内に収まる必要がある)
  - **入力データの意味**: 作業範囲の中央・手前の角・奥の角と箱。wrist_flex が上限 1.658 に近い解(箱 1.578)を含む
- **期待される結果**: `np.all(sol.q >= ctrlrange[:, 0]) and np.all(sol.q <= ctrlrange[:, 1])`
  - **期待結果の理由**: FR-06、design 4.5「ctrlrange でクリップ」
- **テストの目的**: 完了条件「結果が可動域内にある」
  - **確認ポイント**: 範囲の比較は `ctrlrange`(`jnt_range` ではない)。この模型では両者は一致するが、テストは `actuator_ctrlrange` を名前で引く
- 🔵 要件 T5、TASK-0011 テストケース 3

### T4: `IkSolution` は frozen で、`q` は書き込み不可 🟡

- **テスト名**: `test_solution_is_frozen_and_q_readonly`
  - **何をテストするか**: 戻り値の不変性(グローバルルール: 不変性)
  - **期待される動作**: 属性への代入と `q` の要素への代入がどちらも例外
- **入力値**: fixture `grasp_solution`
- **期待される結果**:
  - `sol.pitch_deg = 0.0` → `dataclasses.FrozenInstanceError`
  - `sol.q.flags.writeable is False`、`sol.q[0] = 1.0` → `ValueError`
  - `isinstance(sol.pos_error, float)`、`isinstance(sol.pitch_deg, float)`、`isinstance(sol.pitch_candidate_deg, float)`
  - **期待結果の理由**: 要件 2.1(frozen dataclass、`writeable=False`)、`RobotState` と同じ方式
- **テストの目的**: 呼び出し側(TASK-0012)が解を誤って書き換えないことの保証
- 🟡 要件 T12

### T5: gripper と wrist_roll は seed の値をそのまま返す 🟡

- **テスト名**: `test_gripper_and_wrist_roll_copied_from_seed`
  - **何をテストするか**: 解く変数が 4 関節で、q[4]・q[5] が seed のコピーであること
  - **期待される動作**: q[4] = 0.7、q[5] = 1.0。かつ解は収束している
- **入力値**: `GRASP_CENTER`、`seed_q=(0.1, 0.2, 0.3, 0.4, 0.7, 1.0)`
  - **入力データの意味**: roll と gripper を 0 以外の範囲内の値にする(0 のままでは「コピー」と「0 に固定」を区別できない)。roll 0.7 で手先の位置は約 8 mm の腕で回るが、残り 4 関節で補える
- **期待される結果**: `sol.q[4] == 0.7`、`sol.q[5] == 1.0`(完全一致)、`sol.pos_error <= 0.002`(参照: 候補 0°、0.016 mm)
  - **期待結果の理由**: 要件 2.1(q[4]・q[5] は seed から)
- **テストの目的**: 顎の向き(roll)を TASK-0012 が seed で決められることの保証
- 🟡 要件 T9。確認済み

### T6: `tcp_offset` を指定すると、オフセット点が目標に一致する 🟡

- **テスト名**: `test_tcp_offset_point_lands_on_target`
  - **何をテストするか**: `tcp_offset`(gripperframe 局所座標)を加えた点が目標に乗ること、かつオフセットが実際に効いていること
  - **期待される動作**: 独立 FK で `p_site + R_site @ TCP_OFFSET` が目標に 2 mm 以内、`p_site` 自体は目標から約 |offset| = 16.8 mm 離れる
- **入力値**: `GRASP_CENTER`、`SEED_ZERO`、`tcp_offset=TCP_OFFSET (−0.0075, 0.0, 0.015)`
  - **入力データの意味**: 把持中心のずれの暫定値(要件 D5)。x と z の両成分を持ち、向き(R の掛け方、局所か世界か)の誤りを検出できる
- **期待される結果**:
  - `norm(fk_point(model, sol.q, TCP_OFFSET) - target) <= 0.002`(参照: 0.016 mm)
  - `norm(fk_point(model, sol.q) - target) > 0.010`(参照: 16.79 mm)
  - `sol.pos_error <= 0.002`
  - **期待結果の理由**: 要件 4.2「gripperframe の位置 + R·tcp_offset が目標に 1 mm 以内」
- **テストの目的**: オフセットを局所座標で回して足していることの確認
  - **確認ポイント**: 2 つ目の判定がないと、オフセットを無視した実装でも 1 つ目だけで誤って通る場合がある(オフセットが小さいとき)。オフセットを世界座標で足す誤りは、傾いた姿勢で 1 つ目の判定が約 19 mm ずれて検出される(下の T6b)
- 🟡 要件 T13。確認済み

### T6b: 傾いた姿勢でも `tcp_offset` が局所座標で効く 🟡

- **テスト名**: `test_tcp_offset_in_local_frame_at_tilted_pose`
- **入力値**: `(0.35, 0.10, 0.015)`(候補 20°)、`tcp_offset=TCP_OFFSET`
  - **入力データの意味**: 傾き 20° 以上の姿勢では、局所と世界の違いが mm 単位で現れる(R が単位行列から離れる)
- **期待される結果**: 独立 FK でオフセット点の誤差 ≤ 2 mm、`pitch_candidate_deg >= 10.0`(参照: 候補 20°、傾き 22.4°、0.73 mm。オフセットを世界座標で足した場合の誤差は 19.2 mm で、確実に検出できる)
  - **期待結果の理由**: 要件 2.1 の `tcp_offset` の定義
- 🟡 確認済み

### T7: 返した q を独立 FK にかけると、目標と報告値が一致する 🟡

- **テスト名**: `test_independent_fk_matches_target_and_reported_values`
  - **何をテストするか**: `IkSolution` の `pos_error`・`pitch_deg` が自己申告でなく、q から再現できること
  - **期待される動作**: 別の `MjData` + `mj_kinematics` で求めた手先位置・傾きが、目標・報告値と一致
- **入力値**: `grasp_solution` と、`PREGRASP` の解
- **期待される結果**:
  - `norm(p_fk - target) <= 0.002`
  - `abs(norm(p_fk - target) - sol.pos_error) <= 1e-6`
  - `abs(pitch_fk - sol.pitch_deg) <= 1e-4`
  - `abs(pitch_fk - sol.pitch_candidate_deg) <= 5.0`
  - **期待結果の理由**: 要件 2.1、2.2(ピッチの定義 = site +x と世界 −z のなす角)
- **テストの目的**: 接近軸を +x として扱っていること(−z ではないこと)の確認
  - **確認ポイント**: 接近軸を −z と取り違えた実装では、`pitch_fk`(+x で計算)と `sol.pitch_deg` が大きくずれて検出される
- 🟡

### T8: 候補を 1 つだけ渡すと、その候補の帯の解だけを返す 🟡

- **テスト名**: `test_single_candidate_restricts_pitch_band`
- **入力値**: `(0.25, 0.0, 0.14)`、`SEED_ZERO`、`pitch_candidates_deg=(20,)`
  - **入力データの意味**: TASK-0012 が接近点と把持点を同じピッチで解く使い方(要件 5.1)。この点の最小傾きは約 20°(`t6_check.py`)
- **期待される結果**: `pitch_candidate_deg == 20.0`、`15.0 <= pitch_deg <= 25.0`(参照: 22.1°)、`pos_error <= 0.002`
- **テストの目的**: 候補 1 個の境界(候補列の最小の長さ)
- 🟡 要件 4.1。確認済み

### T9: 候補は渡した順に試す(並べ替えない) 🟡

- **テスト名**: `test_candidates_tried_in_given_order`
- **入力値**: `GRASP_CENTER`、`pitch_candidates_deg=(30, 0)`
  - **入力データの意味**: 0° でも 30° でも解ける点で、30° を先に渡す
- **期待される結果**: `pitch_candidate_deg == 30.0`、`25.0 <= pitch_deg <= 35.0`(参照: 30.2°)
  - **期待結果の理由**: 要件 2.1「指定した順に試す」
- **テストの目的**: 実装が内部で昇順に並べ替えていないことの確認
- 🟡 確認済み

### T10: list・tuple の入力も ndarray と同じ結果になる 🟡

- **テスト名**: `test_accepts_array_like_inputs`
- **入力値**: `target_pos=[0.20, 0.0, 0.015]`(list)、`seed_q=(0,)*6`(tuple、int)、`tcp_offset=[0, 0, 0]`、`pitch_candidates_deg=[0, 10, 20, 30, 45, 60]`(list)
- **期待される結果**: `np.array_equal(sol.q, grasp_solution.q)`、他のフィールドも一致
  - **期待結果の理由**: 要件 2.1 の型(`ArrayLike`、`Sequence[float]`)と決定性
- 🟡

### T11: 接近点の解を seed にして把持点を解ける(ウォームスタート) 🟡

- **テスト名**: `test_warm_start_from_pregrasp_solution`
- **入力値**: `q_a = solve_ik(model, PREGRASP, SEED_ZERO).q`、次に `solve_ik(model, (0.25, 0.05, 0.015), q_a)`
  - **入力データの意味**: TASK-0012 の経由点の使い方。`q_a` は読み取り専用配列なので、読み取り専用の seed を受け付けることも同時に確認できる
- **期待される結果**: 解ける(`pos_error <= 0.002`)、`q[4] == q_a[4]`、`q[5] == q_a[5]`(参照: 候補 0°、関節の最大変化 0.32 rad)
- 🟡 要件 4.1。確認済み

---

## 2. 異常系テストケース

### T12: 到達球の外の目標は、反復せずに `IkError(out_of_reach)` 🔵

- **テスト名**: `test_far_target_raises_out_of_reach`
  - **エラーケースの概要**: 腕の長さより遠い点
  - **エラー処理の重要性**: 解のない目標で動作を実行しない(FR-06)
- **入力値**: `FAR_TARGET (1.0, 0.0, 0.2)`、`SEED_ZERO`
  - **不正な理由**: shoulder_lift アンカーから約 0.93 m。到達半径 0.4115 m を大きく超える
  - **実際の発生シナリオ**: VLM の検出ミスや座標変換の誤りで、机の外の座標が来る
- **期待される結果**:
  - `pytest.raises(IkError)`、`isinstance(exc, RuntimeError)`
  - `exc.reason == "out_of_reach"`
  - `exc.target == (1.0, 0.0, 0.2)`(tuple、float)
  - `exc.tried_pitches_deg == ()`、`exc.best_pitch_deg is None`
  - `exc.best_pos_error > 0`(推定の距離超過分。参照の計算では約 0.5 m)
  - `"1.0"` と `"0.2"` が `str(exc)` に含まれる(メッセージに目標)
  - **システムの安全性**: 例外を出すだけで、何も状態を変えない
- **テストの目的**: 完了条件「到達できない目標で IkError」
- 🔵 要件 T3、TASK-0011 テストケース 2、FR-06

### T13: 球の内側だが届かない点で `IkError(not_converged)` 🟡

- **テスト名**: `test_unreachable_target_inside_sphere_raises_not_converged`
- **入力値**: fixture `near_high_error`(`NEAR_HIGH_TARGET (0.10, 0.0, 0.095)`、既定の候補)
  - **不正な理由**: 位置だけでも届かない(z=0.095 の内側の限界は r ≥ 0.101 m、この点は r = 0.061 m。調査 2.6 節)。到達球の判定は通るため、反復して失敗する
  - **実際の発生シナリオ**: 土台の近くの高い経由点
- **期待される結果**:
  - `exc.reason == "not_converged"`
  - `exc.best_pos_error > 0.001`(参照: 37 mm)
  - `exc.tried_pitches_deg == DEFAULT_CANDIDATES`
  - `isinstance(exc.best_pitch_deg, float)`
  - メッセージに目標と最良誤差(mm 表記)を含む: `"0.1"` と `"mm"` が `str(exc)` に含まれる
  - **エラーメッセージの内容**: 利用者が「どの目標で、どれだけ届かなかったか」を知れる
- 🟡 要件 T4。確認済み

### T14: 土台の真後ろの点で `IkError(not_converged)` 🟡

- **テスト名**: `test_target_behind_base_raises_not_converged`
- **入力値**: `BEHIND_TARGET (−0.15, 0.0, 0.015)`
  - **不正な理由**: 方位 180° は shoulder_pan の可動域(±1.92 rad = ±110°)の外。到達球(半径 0.41 m)の内側
- **期待される結果**: `reason == "not_converged"`、`best_pos_error > 0.05`(参照: 177 mm)
- **テストの目的**: 可動域の制約で届かない点を、可動域外の解で「解けた」ことにしない
- 🟡 確認済み(依頼者の案「床の下」は不採用。下記 T24 のとおり、床の下 (0.20, 0, −0.05) は運動学的に届き、要件 4.2 により解を返す)

### T15: 真下だけを許すと高い点は `not_converged`(可動域外の解を返さない) 🟡

- **テスト名**: `test_vertical_only_candidate_fails_for_high_point`
- **入力値**: `(0.25, 0.0, 0.14)`、`pitch_candidates_deg=(0,)`
  - **不正な理由**: 真下では wrist_flex が上限 1.658 rad を超える必要がある(最小傾き約 20°)
- **期待される結果**: `reason == "not_converged"`、`tried_pitches_deg == (0.0,)`、`best_pitch_deg` は float(参照: 最良時の傾き 20.1°、位置 1.8 mm)
  - **確認ポイント**: `best_pos_error` の大きさは判定しない(定義次第で 1 mm 前後になるため)
- 🟡 要件 T6。確認済み。(0.25, 0, 0.095) は最小傾き約 5° で許容の境界にあたるため使わない

### T16: 不正な `target_pos` は `ValueError`(`IkError` ではない) 🟡

- **テスト名**: `test_invalid_target_raises_value_error`(parametrize)
- **入力値**: shape (2,) `[0.2, 0.0]`、shape (4,)、shape (3, 1)、`[nan, 0, 0.015]`、`[0.2, inf, 0.015]`
  - **不正な理由**: 3 次元の有限値でない
- **期待される結果**: `pytest.raises(ValueError)`、かつ `not isinstance(exc, IkError)`(`IkError` は `RuntimeError` の子で `ValueError` ではない)
  - **エラーメッセージの内容**: `"target"` を含む
- 🟡 要件 T11、3.5 節

### T17: 不正な `seed_q` は `ValueError` 🟡

- **テスト名**: `test_invalid_seed_raises_value_error`(parametrize)
- **入力値**: 長さ 5、長さ 7、`nan` を含む 6 要素、`-inf` を含む 6 要素
- **期待される結果**: `ValueError`(`validate_q_target` を流用した場合のメッセージ `"q_target"` でもよいので、メッセージの文言は判定しない)
- 🟡 要件 3.5 節

### T18: 不正な `pitch_candidates_deg` は `ValueError` 🔴

- **テスト名**: `test_invalid_pitch_candidates_raise_value_error`(parametrize)
- **入力値**: `()`、`(-5,)`、`(95,)`、`(float("nan"),)`、`(0, 120)`
  - **不正な理由**: 空、または [0, 90] の外(NaN は範囲判定で外)
- **期待される結果**: `ValueError`
- 🔴 要件 3.5 節(範囲 [0, 90] は要件で決めた仮定)

### T19: 不正な `tcp_offset` は `ValueError` 🟡

- **テスト名**: `test_invalid_tcp_offset_raises_value_error`(parametrize)
- **入力値**: shape (2,)、shape (4,)、`[nan, 0, 0]`、`[0, 0, inf]`
- **期待される結果**: `ValueError`
- 🟡 要件 3.5 節

---

## 3. 境界値テストケース

### T20: 到達限界のすぐ外は、反復せずに `out_of_reach` 🟡

- **テスト名**: `test_just_outside_reach_raises_out_of_reach_quickly`(parametrize で 3 方向)
  - **境界値の意味**: 到達球の判定が「本当に届かない点」を確実に弾くこと
- **入力値**: `PAN_AXIS_POINT + ROBUST_OUT_DIST_M * u`。u = (1,0,0)、(0,1,0)、(0.6,0.3,−0.2)/‖·‖。具体値は (0.4908, 0.0, 0.1166)、(0.0388, 0.452, 0.1166)、(0.4262, 0.1937, −0.0125)
  - **境界値選択の根拠**: shoulder_lift アンカーは shoulder_pan とともに pan 軸のまわりを回る(pan 軸からの水平距離 0.0355 m、うち横方向 0.0183 m)。pan 軸上の点 `PAN_AXIS_POINT` から手先までの距離の上限は √((0.4115+0.0304)² + 0.0183²) ≈ 0.4423 m(乱数 2 万姿勢の最大 0.4408 m)。0.452 m はこれより約 1 cm 外で、どの姿勢でも届かない
- **期待される結果**: `reason == "out_of_reach"`、1 回の呼び出しが 20 ms 未満(参照: 0.1〜0.9 ms)
  - **一貫した動作**: 限界の外は方向によらず同じ扱い
- **確認ポイント**: 0.4423〜0.452 m の点は判定しない(実装の到達球の取り方で、`out_of_reach` と `not_converged` のどちらにもなりうる)
- 🟡 確認済み

### T21: 到達限界の内側の届く点を `out_of_reach` と誤判定しない 🟡

- **テスト名**: `test_reachable_point_near_outer_limit_is_solved`
  - **境界値の意味**: 到達球の判定が保守的であること(要件 3.5「本当は届く点を外と判定しない」)
- **入力値**: `(0.42, 0.0, 0.015)`、`SEED_ZERO`
  - **境界値選択の根拠**: 作業範囲(x ≤ 0.35)より外で、傾き 45° 以上が必要な外縁の点(調査 2.7 節: x=0.425 で最小 40°)
- **期待される結果**: 解ける、`pitch_candidate_deg in (45.0, 60.0)`(参照: 45°、45.4°、0.31 mm)、`pos_error <= 0.002`、q が ctrlrange 内
- **テストの目的**: 予備の候補 45°・60° が使われる経路の確認
- 🟡 確認済み

### T22: ctrlrange の外の seed はクリップして使う(例外にしない) 🔴

- **テスト名**: `test_seed_outside_ctrlrange_is_clipped`(parametrize で上側・下側)
- **入力値**:
  - 上側: `seed_q=(0, 0, 0, 0, 3.0, 2.0)`(roll 上限 2.84121、gripper 上限 1.74533 を超える)
  - 下側: `seed_q=(0, 0, 0, 0, -3.0, -0.5)`(roll 下限 −2.74385、gripper 下限 −0.17453 を下回る)
  - 目標は `GRASP_CENTER`
  - **実際の使用場面**: sim の関節状態は ctrlrange をわずかに超えることがある(要件 3.5)
- **期待される結果**: 例外なし。`q[4] == ctrlrange[4, 1]`、`q[5] == ctrlrange[5, 1]`(上側)、下側は下限と一致(`np.isclose`、atol 1e-12)。`pos_error <= 0.002`。参照: 上側 (2.84121, 1.74533)、下側 (−2.74385, −0.17453)、どちらも候補 0° で解ける
- 🔴 要件 3.5 節(seed のクリップは要件で決めた仮定)。数値は確認済み

### T23: 候補の端の値 0 と 90 は受け付ける 🔴

- **テスト名**: `test_candidate_bounds_0_and_90_are_accepted`
- **入力値**: `GRASP_CENTER`、`pitch_candidates_deg=(90,)` と `(0,)`
  - **境界値選択の根拠**: 範囲 [0, 90] の両端(閉区間)
- **期待される結果**: `ValueError` を出さない。`(0,)` は解を返す。`(90,)` は解または `IkError` のどちらでもよい(結果の種類は判定しない)
- 🔴 要件 3.5 節

### T24: 床より下の目標でも、運動学的に届けば解を返す 🔴

- **テスト名**: `test_target_below_floor_is_solved_kinematically`
- **入力値**: `(0.20, 0.0, -0.05)`
  - **境界値の意味**: IK は衝突・床を見ない(拒否は安全フィルタ TASK-0010 の役割)
- **期待される結果**: 解を返す、`pos_error <= 0.002`(参照: 候補 0°)
- 🔴 要件 4.2(仮定)。確認済み

### T25: 名前付き定数の値 🟡

- **テスト名**: `test_module_constants`
- **期待される結果**:
  - `PITCH_CANDIDATES_DEG == (0, 10, 20, 30, 45, 60)`(数値として比較)
  - `PITCH_TOL_DEG == 5.0`、`POS_TOL_M == 0.001`、`POS_TOL_M < POS_TOL_TEST_M`
  - `DAMPING > 0`(減衰 0 は特異姿勢で J Jᵀ が特異になるため不可)
  - `APPROACH_AXIS_LOCAL == (1.0, 0.0, 0.0)`(接近軸は +x。note 6.2 の −z は誤り)
  - `solve_ik` の `pitch_candidates_deg` の既定値が `PITCH_CANDIDATES_DEG`(`inspect.signature` で確認)
- 🟡 要件 3.1

### T26: 既定の候補で得た解は、どれも許容帯の中 🟡

- **テスト名**: T1・T2・T3・T8・T9・T21 の中で共通の補助 `_assert_valid_solution(sol, target, ctrlrange)` を使い、`abs(pitch_deg - pitch_candidate_deg) <= 5.0` と `pitch_candidate_deg in 渡した候補` を毎回確認する(独立したテスト関数にはしない)
- 🟡 要件 3.2 の収束判定

---

## 4. 副作用・不変性のテストケース

### T27: 入力を変更しない(読み取り専用の入力も受け付ける) 🟡

- **テスト名**: `test_inputs_not_mutated`
- **入力値**: `target = np.array([0.25, 0.05, 0.095])`、`seed = np.array([0.1, 0.2, 0.3, 0.4, 0.7, 1.0])`、`offset = np.array([-0.0075, 0.0, 0.015])`、`cands = [0, 10, 20]`(list)。呼び出し前に `copy()` を取る
- **期待される結果**: 呼び出し後に 4 つとも `np.array_equal`(list は `==`)で不変。さらに `writeable=False` にした同じ配列で呼んでも例外にならない(内部で入力に書き込んでいれば `ValueError` になる)
- 🟡 要件 T7、3.3 節

### T28: 同じモデルの別の `MjData`(実行中のシミュレーション相当)を変更しない 🟡

- **テスト名**: `test_live_mjdata_untouched`
- **Given**: `data = mujoco.MjData(scene_model)`。`qpos`(腕の 6 関節を (0.3, −0.2, 0.4, 0.5, 0.6, 0.2) に)、`ctrl` を 0 以外、`data.time = 1.234` に設定して `mj_forward`。`qpos`・`qvel`・`ctrl`・`time`・`site_xpos` のコピーを取る
- **When**: `solve_ik(scene_model, PREGRASP, SEED_ZERO)`
- **Then**: 5 つとも呼び出し前と完全一致(`np.array_equal`、`time` は `==`)
- **確認ポイント**: IK が「モデルに紐づく既存の MjData」を使い回していないこと(専用 MjData)
- 🟡 要件 3.3 節

### T29: `SimRobot` の状態が IK の前後で変わらない 🟡

- **テスト名**: `test_sim_robot_state_unchanged`
- **Given**: `SimRobot(PICKPLACE_SCENE_PATH)` に `send_command((0.2, -0.3, 0.4, 0.5, 0.1, 0.3))` を 5 回送り、0 でない状態にする。`before = robot.get_state()`
- **When**: `solve_ik(scene_model, GRASP_CENTER, before.q)`(seed に現在の状態を使う、実際の使い方)
- **Then**: `after = robot.get_state()`。`q`・`dq` が `np.array_equal`、`timestamp` が `==`。最後に `robot.close()`
- 🟡 要件 T8、TASK-0011 実装詳細 4

### T30: モデルを変更しない 🟡

- **テスト名**: `test_model_not_mutated`
- **Given**: `scene_model` の `qpos0`、`jnt_range`、`actuator_ctrlrange`、`body_pos`、`body_quat`、`site_pos`、`site_quat`、`opt.timestep` のコピー
- **When**: `solve_ik` を 2 回(解ける点 1 つ、`tcp_offset` 付き 1 つ)
- **Then**: 全て完全一致
- 🟡 要件 3.3 節

---

## 5. 決定性のテストケース

### T31: 同じ入力で 2 回呼ぶと完全に同じ結果 🟡

- **テスト名**: `test_deterministic_same_inputs`
- **入力値**: `PREGRASP`、`SEED_ZERO`(候補 0° が失敗してから 10° で解ける、追加シードを通る経路)
- **期待される結果**: `np.array_equal(s1.q, s2.q)`、`s1.pitch_deg == s2.pitch_deg`、`s1.pitch_candidate_deg == s2.pitch_candidate_deg`、`s1.pos_error == s2.pos_error`(許容なしの完全一致)
- 🟡 要件 T10、3.4 節

### T32: グローバルな乱数の状態に依存せず、乱数を消費もしない 🟡

- **テスト名**: `test_independent_of_global_rng`
- **Given/When**: `np.random.seed(0)` → 解く → `np.random.seed(12345)`、`random.seed(7)` → 解く。2 回目の前に `np.random.get_state()` と `random.getstate()` を保存
- **Then**: 2 つの解が完全一致。2 回目の呼び出しの前後でグローバル乱数の状態が同じ(`get_state()` の配列部分が `np.array_equal`)
- 🟡 要件 3.4 節(追加シードは固定値)

---

## 6. 性能のテストケース

`time.perf_counter()` で測る。CI のばらつきに備え、しきい値は実測の 10 倍以上の余裕を持たせる。

### T33: `out_of_reach` は即座 🟡

- **テスト名**: `test_out_of_reach_is_fast`
- **入力値**: `FAR_TARGET`
- **期待される結果**: 1 回の呼び出し(例外まで)が 20 ms 未満(参照: 0.2 ms)
- 🟡 要件 3.6 節

### T34: 候補 0° で解ける点は 50 ms 未満 🟡

- **テスト名**: `test_typical_solve_under_50ms`
- **入力値**: `GRASP_CENTER`。3 回呼んで最小値を使う(初回の遅延・GC の影響を除く)
- **期待される結果**: `min(times) < 0.050`(参照: 2〜5 ms)
- **確認ポイント**: 0° が失敗して 10° 以上で解ける点(参照: 0.27〜0.8 s、打ち切りありで ≤ 0.33 s)には 50 ms を適用しない(要件 3.6「通常」の解釈。未決事項 O2)
- 🟡

### T35: 収束しない点でも 3 秒以内に `IkError` 🟡

- **テスト名**: `test_not_converged_within_3s`
- **入力値**: fixture `near_high_error` の所要時間(T13 と共有)
- **期待される結果**: `elapsed_s < 3.0`(参照: 打ち切りなし 1.8 s、打ち切りあり 0.25 s)
- 🟡 要件 3.6 節

---

## 7. 到達性の回帰テスト(`tests/policy/test_ik_survey.py`)

### T36: 作業範囲と箱の代表点は、既定の候補で必ず解ける 🟡

- **テスト名**: `test_survey_points_solvable_with_default_candidates`(parametrize、`ids` に点の名前)
  - **何をテストするか**: 到達性調査の結論(作業範囲・箱の全点が 30° 以下で届く)が、実装でも保たれること。シーンや定数を変えたときの検出
- **入力値**(全て `SEED_ZERO`、既定の候補。参照ソルバの結果を併記。**判定は「解ける」と「候補 ≤ 30°」だけで、候補の値そのものは判定しない**):

| id | 目標 (m) | 意味 | 参照: 候補 / 傾き | 参照: wrist roll を変えた seed (−1.5, 0.7, 1.5) と離れた seed 2 種 |
|---|---|---|---|---|
| `pregrasp_center_high` | (0.25, 0.00, 0.14) | 高い接近点(真下 0° 不可の代表) | 20° / 22.1° | 全て 20° |
| `box_release` | (0.20, 0.19, 0.06) | 箱の解放点 | 0° / 4.9° | 全て 0° |
| `box_preplace` | (0.20, 0.19, 0.14) | 箱の上の接近点 | 20° / 22.6° | 全て 20° |
| `far_corner_grasp` | (0.35, −0.10, 0.015) | 奥の角・把持(x=0.35 の境界) | 20° / 23.2° | 全て 20° |
| `far_corner_approach` | (0.35, 0.10, 0.095) | 奥の角・接近(作業範囲で最も厳しい点) | 30° / 31.8° | 全て 30° |
| `far_center_approach` | (0.35, 0.00, 0.095) | 奥の中央・接近 | 30° / 31.6° | — |
| `near_corner_grasp` | (0.15, −0.10, 0.015) | 手前の角・把持 | 0° / 5.0° | 全て 0° |
| `near_corner_approach` | (0.15, 0.10, 0.095) | 手前の角・接近 | 0° / 4.9° | — |

- **期待される結果**: 各点で `IkError` を出さない、`pos_error <= 0.002`、`pitch_candidate_deg <= 30.0`、q が ctrlrange 内(T26 の補助で確認)
  - **期待結果の理由**: 調査 3 節(i)「作業範囲は 30° 以下で全点届く」。参照ソルバでは 9×9 格子 × 2 高さ + 箱 3 点の 165 点が全て既定の候補で解け、候補の内訳は把持 0°:63 / 10°:9 / 20°:9、接近 0°:29 / 10°:30 / 20°:13 / 30°:9 だった
- **確認ポイント**: シーン定数との整合のため、`box_release`・`box_preplace` の xy は `SCENE_SPEC.box_center_xy` から組み立てる。`near_corner_approach` の傾きは 4.9° で 0° の帯の端にあるが、判定は「≤ 30°」なので影響しない
- 🟡 要件 T14(9×9 格子の全点は時間がかかるため、代表 8 点に間引く)。確認済み

### T37(判定しない記録): シード次第で候補が変わる境界の点 🟡

- **対象**: `(0.25, 0.0, 0.095)`(最小傾き約 5°、許容 ±5° の境界)、把持高さ x = 0.25〜0.30 の 0°/10° の境界の点(調査 2.4 節)
- **扱い**: **テストでは判定しない**。`test_ik_survey.py` の先頭コメントに「候補は 0° または 10° のどちらでも仕様どおり(参照ソルバでは (0.25, 0, 0.095) は候補 10°、傾き 14.0°)」と記録するだけにする。解けること自体は堅いが、候補の値を判定すると追加シードの選び方で結果が揺れるため
- 🟡 要件 4.2

---

## 8. 開発言語・フレームワーク

- **プログラミング言語**: Python 3.12
  - **言語選択の理由**: プロジェクトの既定(`pyproject.toml` の `requires-python`)。MuJoCo の Python バインディングと numpy を直接使える
  - **テストに適した機能**: `np.array_equal` による完全一致、`dataclasses.FrozenInstanceError`、配列の `flags.writeable`
- **テストフレームワーク**: pytest(+ pytest-cov)
  - **フレームワーク選択の理由**: 既存テスト(`tests/robot/test_sim.py`)と同じ。`scope="module"` の fixture でモデル読み込みを 1 回にし、`parametrize` で点・不正入力を列挙する
  - **テスト実行環境**: `uv run pytest tests/policy/`。マーカーは使わない(`pyproject.toml` に `slow` マーカーの登録がない。時間が予算を超えたら未決事項 O3)
  - **ファイル**: `tests/policy/__init__.py` は既存の `tests/robot/` に合わせる(`tests/robot/` に `__init__.py` が無ければ作らない)
- 🔵 note.md 1 節、CLAUDE.md

---

## 9. 実装時の日本語コメント指針(Python)

既存テストに合わせ、モジュール docstring にテストケース定義ファイルへのパスと信頼性の凡例を書く。各テストは AAA(Given / When / Then)。

```python
def test_reachable_grasp_point_converges_within_2mm_near_vertical(grasp_solution):
    # 【テスト目的】: 到達可能な把持点で、2 mm 以内・候補 0° で収束すること (T1)
    # 【期待される動作】: IkSolution を返し、pitch_deg <= 5
    # 🟡 数値は参照ソルバで確認済み
    # Given: 【テストデータ準備】作業範囲中央の把持点、seed は 0 姿勢(fixture で解済み)
    sol = grasp_solution
    # Then: 【結果検証】位置誤差・形状・採用ピッチ
    assert sol.pos_error <= POS_TOL_TEST_M  # 【確認内容】完了条件「収束する」
    assert sol.q.shape == (6,)              # 【確認内容】send_command と同じ 6 要素
    assert sol.pitch_candidate_deg == 0.0   # 【確認内容】真下に近い候補から採用
```

- fixture には「【テスト前準備】モデル読み込みは重いのでモジュールで 1 回」のように理由を書く
- 各 `assert` の行末に「【確認内容】」を書く(test_sim.py と同じ密度でよい)

---

## 10. 要件定義との対応関係

| テスト | 要件(ik-requirements.md) | タスク / EARS | 信頼性 |
|---|---|---|---|
| T1 | 要件 T1、4.1 | TASK-0011 テストケース 1、完了条件 1、FR-05 | 🟡 |
| T2 | 要件 T2、4.1 | TASK-0011 テストケース 4 | 🟡 |
| T3 | 要件 T5、3.2 | テストケース 3、完了条件 3、FR-06 | 🔵 |
| T4 | 要件 T12、2.1 | — | 🟡 |
| T5 | 要件 T9、2.1 | — | 🟡 |
| T6, T6b | 要件 T13、2.1、4.2 | — | 🟡 |
| T7 | 要件 2.1、2.2 | テストケース 1・4 | 🟡 |
| T8 | 4.1(候補 1 個)、5.1 | — | 🟡 |
| T9 | 2.1(指定順) | — | 🟡 |
| T10 | 2.1(ArrayLike) | — | 🟡 |
| T11 | 4.1(ウォームスタート) | — | 🟡 |
| T12 | 要件 T3、3.5 | テストケース 2、完了条件 2、FR-06 | 🔵 |
| T13 | 要件 T4、3.5 | 完了条件 2、FR-06 | 🟡 |
| T14 | 3.5 | FR-06 | 🟡 |
| T15 | 要件 T6、4.3 | FR-06 | 🟡 |
| T16〜T19 | 要件 T11、3.5 | — | 🟡(T18 は 🔴) |
| T20, T21 | 3.1(到達球)、3.5(保守的) | — | 🟡 |
| T22 | 3.5(seed のクリップ) | — | 🔴 |
| T23 | 3.5(候補の範囲) | — | 🔴 |
| T24 | 4.2(床の下) | — | 🔴 |
| T25 | 3.1 | — | 🟡 |
| T26 | 3.2(収束判定) | テストケース 4 | 🟡 |
| T27 | 要件 T7、3.3 | — | 🟡 |
| T28, T29 | 要件 T8、3.3 | TASK-0011 実装詳細 4 | 🟡 |
| T30 | 3.3 | — | 🟡 |
| T31, T32 | 要件 T10、3.4 | — | 🟡 |
| T33〜T35 | 3.6 | NFR-06 | 🟡 |
| T36 | 要件 T14、調査 3 節 | — | 🟡 |
| T37 | 4.2(境界の点) | — | 🟡 |

- **参照した機能概要**: 要件 1 節
- **参照した入力・出力仕様**: 要件 2 節(`IkSolution`、`IkError`、`solve_ik` の引数)
- **参照した制約条件**: 要件 3 節(定数、解法、副作用、決定性、エラー、性能)
- **参照した使用例**: 要件 4 節(基本・エッジ・エラー)、reachability-survey.md 2・3 節

---

## 11. 件数と信頼性の分布

| 分類 | テスト | 件数(関数) | parametrize を展開したケース数 |
|---|---|---|---|
| 正常系 | T1〜T11(T6b を含む) | 12 | 15 |
| 異常系 | T12〜T19 | 8 | 22 |
| 境界値 | T20〜T25(T26 は補助) | 6 | 9 |
| 副作用・不変性 | T27〜T30 | 4 | 4 |
| 決定性 | T31〜T32 | 2 | 2 |
| 性能 | T33〜T35 | 3 | 3 |
| 到達性の回帰 | T36(T37 は記録のみ) | 1 | 8 |
| **合計** | | **36** | **63** |

- 信頼性(T1〜T37 と T6b の 38 項目 + 開発言語 1 項目): 🔵 3(T3, T12, 開発言語) / 🟡 32 / 🔴 4(T18, T22, T23, T24)。🟡 のうち 20 項目は使い捨て実行で数値を確認済み。

## 12. 未決事項(Red / Green で判断)

| ID | 内容 | 既定 |
|---|---|---|
| O1 | 停滞打ち切り(DLS が N 反復で改善しなければ次のシードへ)を Green で入れるか。入れると `test_ik.py` が約 3 秒以内に収まる。参照ソルバで N=25、改善 0.1 mm 未満で打ち切っても 165 点の結果は不変 | 入れる(要件 3.1 の定数に `STALL_ITERS` を追加する形。要件の変更ではなく実装の詳細) |
| O2 | 要件 3.6「通常 50 ms 以下」は、候補 0° で解ける点に限って判定する(10° 以上の候補で解ける点は、前の候補の失敗分で 0.3〜0.8 s かかる) | T34 のとおり 0° の点だけ判定 |
| O3 | `test_ik_survey.py` が 5 秒を超える場合、`pyproject.toml` に `slow` マーカーを登録して既定の実行から外すか | 超えるまでは登録しない |
| O4 | `tcp_offset` の値は暫定(要件 D5)。TASK-0012/0013 で実測値に変わっても、T6・T6b はテスト専用の定数 `TCP_OFFSET` を使うので変更不要 | 変更しない |

## 13. 品質判定

| 観点 | 評価 |
|---|---|
| テストケース分類 | 正常系・異常系・境界値に加え、副作用・決定性・性能・到達性の回帰を網羅 |
| 期待値定義 | 全ケースで具体値(しきい値、例外の種類・属性、参照値)を定義。境界の揺れる点は判定から外し、理由を記録 |
| 技術選択 | Python 3.12 + pytest、module fixture、独立 FK(`mj_kinematics`) |
| 実装可能性 | 要件どおりの参照ソルバで、判定に使う数値をすべて再現済み |
| 信頼性レベル | 🟡 が多いが、大半は実測で確認済み。🔴 は要件で仮定とした 4 項目のみ |

**判定: ✅ 高品質**

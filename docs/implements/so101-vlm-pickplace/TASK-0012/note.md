# TASK-0012 タスクノート: 把持・運搬の軌道生成

**機能名**: trajectory(把持・運搬の軌道生成 / `IkPolicy`)
**タスクID**: TASK-0012
**要件名**: so101-vlm-pickplace
**作成日**: 2026-10-21
**作成方法**: tsumiki:tdd-tasknote(ユーザー確認なしでデフォルトを選び、仮定を 🔴 で記録した)
**出力ファイル**: `docs/implements/so101-vlm-pickplace/TASK-0012/note.md`

**信頼性の凡例**: 🔵 要件定義書・設計文書・既存コード・前タスクの記録どおり / 🟡 文書から妥当に推測 / 🔴 文書にない。本ノートで決めた仮定(要確認)

---

## 0. 収集した情報の範囲

| 種別 | 読んだもの |
|---|---|
| タスク | `docs/tasks/so101-vlm-pickplace/TASK-0012.md`、`TASK-0013.md`(後続)、`TASK-0024.md`、`overview.md` |
| 仕様 | `docs/spec/so101-vlm-pickplace/note.md`、`docs/requirements.md`(FR-05, FR-06, FR-11, NFR-02, NFR-05, NFR-07) |
| 設計 | `docs/design.md`(4.1 Robot、4.2 Policy、4.5 IkPolicy、4.6 安全フィルタ) |
| 前タスクの記録 | `docs/implements/so101-vlm-pickplace/TASK-0011/ik-requirements.md`、`ik-verify-complete.md`、`reachability-survey.md`、`TASK-0010/safety-requirements.md`、`safety-verify-complete.md` |
| 既存コード | `src/so101_vlm/policy/ik.py`、`policy/ik_types.py`、`safety/limits.py`、`safety/config.py`、`safety/workspace.py`、`robot/base.py`、`robot/sim.py`、`eval/scene_spec.py`、`configs/motion.yaml` |
| テスト | `tests/policy/test_ik.py`、`tests/safety/conftest.py`、`tests/safety/safety_constants.py` |

- 追加ルール: プロジェクトルートに `AGENTS.md` と `docs/rule/` は存在しない。適用するのはユーザー側の共通ルール(`~/.claude/rules/ecc/` の common と web。不変性、ファイル 800 行以下、関数 50 行未満、名前付き定数、単体テスト 80% 以上)。
- 未実施: `docs/spec/so101-vlm-pickplace/` 配下の `*-user-stories.md` / `*-acceptance-criteria.md` は存在しない(要件は `docs/requirements.md` が正)。
- 未実施: Explore エージェントによる探索は行わなかった(本セッションの指示でエージェントを起動しないため、直接読んだ)。

---

## 1. 技術スタック

- 🔵 Python 3.12(`pyproject.toml` の `requires-python = ">=3.12,<3.13"`)。パッケージ管理は uv。テストは pytest(`[tool.pytest.ini_options] testpaths = ["tests"]`)。カバレッジは `coverage`(`source = ["so101_vlm"]`)。
- 🔵 MuJoCo(`mujoco`)による順運動学・ヤコビアン。TASK-0011 の `solve_ik` を内部部品として使う。
- 🔵 numpy、PyYAML(`configs/motion.yaml` の読み込み。`safety/config.py` で `yaml.safe_load` を使用)。
- 🔵 アーキテクチャ: `Policy` Protocol(design 4.2)の背後に `IkPolicy` を置き、将来の `SmolVlaPolicy` と差し替える(FR-11)。
  - 参照元: `docs/design.md`、`src/so101_vlm/robot/base.py`(`Robot` Protocol と `RobotState` の書き方を真似る)
- 🔵 純粋関数の層(`trajectory.py`)と、MuJoCo とモデルに触れる層(`ik_policy.py`)を分ける。`limits.py` / `ik_types.py` と同じ方針。
  - 参照元: `src/so101_vlm/safety/limits.py`、`src/so101_vlm/policy/ik_types.py`

---

## 2. 開発ルール

- 🔵 不変性: 入力を変更しない。結果型は frozen dataclass とし、配列は生成時にコピーして `writeable=False` にする(`IkSolution` / `RobotState` と同じ方式)。
- 🔵 名前付き定数を使う。数値は `configs/motion.yaml` の `trajectory:` セクションに置き、コードに直書きしない。
- 🔵 ファイル 800 行以下、関数 50 行未満。
- 🔵 安全判定はフラグや例外で明示する。黙って丸めない・握りつぶさない(共通ルール「Never silently swallow errors」)。
- 🔵 `configs/motion.yaml` の `safety:` セクションは既存の `safety/config.py` が厳密に検査する(未知のキーは `ValueError`)。TASK-0012 の値は **新しい最上位キー `trajectory:`** に入れる。`safety:` の中には追加しない(`safety/config.py` は `safety:` 以外の最上位キーを無視する)。
  - 参照元: `configs/motion.yaml`、`src/so101_vlm/safety/config.py`
- 🔵 TDD: 先にテストを書く(Red → Green → Refactor)。
  - 参照元: `docs/tasks/so101-vlm-pickplace/TASK-0012.md` の実装手順

---

## 3. 関連実装

### 3.1 前提となる既存の資産(TASK-0011 / TASK-0010 の記録どおり)

| 資産 | 場所 | 使い方 |
|---|---|---|
| `solve_ik(model, target_pos, seed_q, *, pitch_candidates_deg, tcp_offset) -> IkSolution` | `src/so101_vlm/policy/ik.py`(`solve_ik` は 263 行目付近) | 経由点ごとに呼ぶ。`IkSolution.q` は (6,) で読み取り専用 |
| `IkSolution(q, pitch_deg, pitch_candidate_deg, pos_error)` | `src/so101_vlm/policy/ik_types.py` | 採用ピッチ・誤差をメタデータに載せる |
| `IkError(target, reason, best_pos_error, best_pitch_deg, tried_pitches_deg, message)` | `src/so101_vlm/policy/ik_types.py` | `REASON_OUT_OF_REACH` / `REASON_NOT_CONVERGED` を使う |
| `PITCH_CANDIDATES_DEG = (0, 10, 20, 30, 45, 60)`、`APPROACH_AXIS_LOCAL = (1, 0, 0)` | `src/so101_vlm/policy/ik.py` | 共通ピッチの候補として使う |
| `apply_safety(q_current, q_target, *, lower, upper, last_state_t, now, config, target_base=None) -> SafetyResult` | `src/so101_vlm/safety/limits.py` | TASK-0013 で毎周期通す。TASK-0012 のテストでも使う |
| `is_in_workspace(p_base, box) -> bool`、`WorkspaceBox` | `src/so101_vlm/safety/workspace.py`、`safety/config.py` | 目標点を IK の前に検査する |
| `load_safety_config()`、`limits_from_model(model)` | `src/so101_vlm/safety/config.py`、`safety/limits.py` | 関節の可動域・速度上限の取得 |
| `Robot` Protocol、`RobotState(q, dq, timestamp)`、`validate_q_target` | `src/so101_vlm/robot/base.py` | `Observation` の形を揃える。`JOINT_NAMES`(shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper) |
| `SimRobot` | `src/so101_vlm/robot/sim.py` | TASK-0013 で使う。`MjData` は外に出さない |
| `SCENE_SPEC` | `src/so101_vlm/eval/scene_spec.py` | ブロック半サイズ 0.015、床上の静止中心高さ 0.015、箱中心 (0.20, 0.19)、内寸半幅 0.04、壁厚 0.005、床上面 0.005、縁の高さ 0.04 |

### 3.2 新規に作るファイル(提案)

| ファイル | 役割 | 信頼性 |
|---|---|---|
| `src/so101_vlm/policy/base.py` | `Observation`、`Goal`、`Policy` Protocol(design 4.2)。`JointTrajectory` は `trajectory.py` に置き、ここから参照 | 🟡 |
| `src/so101_vlm/policy/trajectory.py` | 純粋層: 経由点の定義、時間配分、関節空間補間、`JointTrajectory`、`TrajectoryError` | 🟡 |
| `src/so101_vlm/policy/ik_policy.py` | `IkPolicy`: `act(obs, goal) -> JointTrajectory`。MuJoCo モデルと `solve_ik` を使う | 🔴 TASK-0012.md が `ik.py` と書いているが、`ik.py` は IK 単体(TASK-0011)なので分ける |
| `configs/motion.yaml` | 新しい `trajectory:` セクションを追加 | 🔵 |
| `tests/policy/test_trajectory.py` | TASK-0012.md のテストケース 1〜3 | 🔵 |
| `tests/policy/test_ik_policy.py` | `IkPolicy.act` の結合テスト(MuJoCo モデル使用) | 🟡 |

- 🔴 `Observation` と `Goal` の中身は本ノート 7.1 節で決める。

---

## 4. 設計文書(要点)

### 4.1 Policy 抽象(design 4.2)
- 🔵 `class Policy(Protocol): def act(self, obs: Observation, goal: Goal) -> JointTrajectory: ...`
- 🔵 `IkPolicy`(v1)は、目標 3 次元位置から接近・把持・持ち上げ・移動・解放の軌道を作る。`SmolVlaPolicy`(v2)は同じ Protocol で差し替える(FR-11)。

### 4.2 IkPolicy と軌道(design 4.5)
- 🔵 目標は「`gripperframe` を目標位置に置き、手先を下向きに近づける」。5 自由度のため姿勢は弱く拘束(TASK-0011 で実装済み)。
- 🔵 経由点: `pre_grasp`(目標の真上 8 cm)→ `grasp` → 閉じる → `lift` → `pre_place`(箱の真上)→ `place` → 開く。
- 🔵 区間は関節空間で線形補間し、最大速度で区間時間を決める。
- 🔵 関節可動域は `ctrlrange` でクリップし、収束しなければ `IkError`(FR-06)。
- 🔵 数値(接近高さ、速度、グリッパーの開閉量)は `configs/motion.yaml` に置き、実測で調整する。

### 4.3 安全フィルタ(design 4.6、TASK-0010 の記録)
- 🔵 `apply_safety` は、前回の指令を速度制限の基準に使うのが推奨(追従遅れで上限ちょうどの軌道が毎周期縮むのを防ぐ)。
- 🔵 「軌道が速度・可動域を守る」の定義(TASK-0010 の記録): 連続する指令の差が各関節で `vmax × speed_scale × control_dt` 以下、全値が `ctrlrange` 内、全 Cartesian 目標が作業範囲内。
- 🔵 TASK-0010 のテスト方法: 軌道を前回指令を基準とした `apply_safety` に通し、全フラグが False、出力が入力と等しいことを確認する。
- 🔵 TASK-0010 の推奨: 時間配分の安全係数 **0.99**。

### 4.4 シーン(scene_spec、TASK-0005 / design 4.7)
- 🔵 箱の中心 (0.20, 0.19)、内寸 8 cm(内寸半幅 0.04)、縁の高さ 0.04(`SCENE_SPEC.box_rim_z`)。
- 🔵 タスク点(把持): 作業範囲 x 0.15〜0.35、y -0.10〜0.10(`SCENE_SPEC.work_area_x` / `work_area_y`)。
- 🔵 ブロック: 一辺 3 cm(半サイズ 0.015)。床上の静止中心高さ 0.015。

### 4.5 要件
- 🔵 FR-05: 接近・把持・持ち上げ・移動・解放の関節軌道を生成して実行する。
- 🔵 FR-06: IK が解を得られない、または可動域を超える場合は、動作せずエラーを返す。
- 🔵 FR-11: 実行部分は `Policy` の背後に置き、差し替え可能にする。
- 🔵 NFR-02: 関節角・速度に上限。実機では範囲外の指令を送らない。
- 🔵 NFR-05: 純粋ロジック(IK・軌道など)に単体テストを書く。

---

## 5. テスト関連情報

- 🔵 テスト実行: `uv run pytest -q`(現状 979 件 passed、ユーザー指定)。
- 🔵 テストの配置: `tests/policy/`(既存: `test_ik.py`、`test_ik_survey.py`)、`tests/safety/`(既存: `conftest.py`、`safety_constants.py`、`test_apply_safety.py`、`test_config.py`、`test_limits.py`)。
- 🔵 共有 fixture: `tests/safety/conftest.py` の `task_points`(SCENE_SPEC から 9×9 の把持点、172 点)と、`PICKPLACE_SCENE_PATH` / `SO101_SCENE_PATH` の読み込み。TASK-0012 のテストでも同じ点を使うと、IK の到達性の前提が揃う。
  - 注意: `task_points` の生成式は `grasp_z = SCENE_SPEC.block_rest_z`(0.015)。
- 🔵 共有の定数・ヘルパ: `tests/safety/safety_constants.py` の `VMAX`、`DT`、`LOWER_SYN`、`UPPER_SYN`。TASK-0010 の Refactor で重複を集約した方式に倣い、TASK-0012 でも共通化するなら `tests/policy/` 側に置く(安全テストから import しない)。
- 🔵 単体テストの対象: `trajectory.py` は MuJoCo 不要の純粋関数としてテストする(NFR-05)。`IkPolicy` は実モデルを使う結合テスト。
- 🟡 E2E: 本タスクでは不要。TASK-0013 の固定位置 pick & place で確認する。
- 🟡 視覚回帰・ブラウザ試験: 対象外(Web 画面なし)。

---

## 6. 注意事項

### 6.1 TASK-0011 の記録から引き継ぐ制約(必ず反映する)

1. 🔵 **接近点を先に解き、その採用ピッチ p を把持点の候補に固定する。** `solve_ik(..., pitch_candidates_deg=(p,))` で把持点を解く。理由: 接近点と把持点を別々に既定候補で解くと、グリッドの 81 点中 52 点で候補が食い違う。単一の共通 p は 81 点すべてに存在する(`reachability-survey.md`)。
2. 🔵 **接近点の解を把持点のシード(ウォームスタート)に使う。** 関節の連続性を保つ。
3. 🔵 **`tcp_offset` は暫定値 (-0.0075, 0.0, 0.015)。** `gripperframe` は把持中心から約 1.7 cm ずれている。実測で確定させる(6.3 節 A)。
4. 🔵 **IK は衝突(床・箱・ブロック・自己干渉)を見ない。** 床と箱の壁への干渉は、本タスクでサンプリングした FK で確認する(6.2 節)。
5. 🔵 **IK は関節空間の直線を保証しない。** 関節空間の補間では手先が直線の経路から外れる。接近 8 cm の高さなら許容範囲とみなす。
6. 🔵 **wrist_roll と gripper は IK が動かさない。** 解いた後の値は seed のまま。`IkSolution.q[4]`、`q[5]` は seed の値。
7. 🔵 **傾きの許容は ±5°。** 候補と実際の傾きの差がこの範囲に入ること。
8. 🟡 候補 0° は wrist_flex の上限のため、高い点では解けない。接近点は候補 10° または 20° で解けることが多い(`ik-requirements.md` 4.1 節)。

### 6.2 軌道の干渉チェック(FK サンプリング)— 本タスクの要件として提案

IK は衝突を見ないため、補間後の経路を FK で確認する要件を置く。

- 🔴 **方法**: 補間済みの関節列を一定間隔(例: 5 制御周期ごと)でサンプリングし、各点で `mujoco.mj_forward` を呼んで `gripperframe` と TCP(= gripperframe + R·tcp_offset)の位置を求める。
- 🔴 **床の条件**: すべての点で TCP の z ≥ 把持中心の床高さ(ブロックの半サイズ 0.015 以上)。指先の最低点が z ≥ 0 を割らないこと。数値のマージンは実測で決める。
- 🔴 **箱の条件**: 手先の水平位置が箱の外周(内寸 + 壁厚 + ブロック半サイズ + 余裕)の内側にある間は、TCP の z ≥ 縁の高さ 0.04 + ブロック半サイズ 0.015 + 余裕。
  - 箱の外周の中心は `SCENE_SPEC.box_center_xy`、内寸半幅 0.04、壁厚 0.005。
- 🔴 **失敗時**: `TrajectoryError`(失敗した区間の名前と最低点の値を含む)。軌道は作らない(部分的な軌道を返さない)。
- 🔴 この確認は純粋な関数として `trajectory.py` に置き、FK を渡す形にする(モデルへの依存を `ik_policy.py` に閉じる)。

### 6.3 実測が必要な数値(throwaway survey を要件定義の段階で行う)

以下は文書から決めず、要件定義(TASK-0012 の tdd-requirements)の段階で、短い使い捨ての調査スクリプトで測る。調査コードは本体に残さない。

| # | 項目 | 測る方法 | 決める値 |
|---|---|---|---|
| A | `tcp_offset`(把持中心の `gripperframe` 局所座標) | 把持中の位置で、ブロックの中心と `gripperframe` の差を `gripperframe` の回転で局所座標に直す。暫定値 (-0.0075, 0, 0.015) と比べる | `trajectory.tcp_offset` |
| B | グリッパーの開き角・閉じ角(関節 `gripper`、ctrlrange -0.1745〜1.745 rad) | 開く方向(どちらが開きか)を、角度を変えた時の顎の間隔で確認。閉じ角は、ブロックに当たる直前の角度を求める。空の閉じ角と、3 cm のブロックを挟む角度を分けて記録 | `trajectory.gripper_open`、`trajectory.gripper_closed` |
| C | 運搬の高さ(持ち上げ・移動の z) | 箱の上で FK サンプリングし、6.2 節の条件を満たす最低の z を求め、余裕を足す。sim でブロックが縁に当たらないことを確認 | `trajectory.carry_height` |
| D | 置く高さ(`place` の TCP z) | 箱の底(床上面 0.005)+ ブロック半サイズ 0.015 = 0.02 を基準に、解放時の落下を sim で確認 | `trajectory.place_height` |

---

## 7. 本タスクの設計判断(ユーザー確認なし。デフォルトを選んだ仮定)

### 7.1 型(`policy/base.py`)🔴

- `Observation`: `q: (6,)`(現在の関節角)、`timestamp: float`。`RobotState` から作れる形にする。画像は含めない(IkPolicy は使わない。SmolVLA 用の拡張は v2)。
- `Goal`: `grasp_pos: (3,)`(把持中心 = ブロック中心の目標、baseframe の m)、`place_pos: (3,)`(置く位置。ブロック中心の目標)。箱の中心 xy は `SCENE_SPEC.box_center_xy` を既定に使い、`place_pos` の既定値も作る関数を用意する。
- 配列の値は生成時にコピーし、読み取り専用にする(不変性)。

### 7.2 目標の意味(把持点)🔵/🟡
- 🔵 把持の目標は **ブロック中心**(知覚の出力から変換する点。TASK-0008 の記録どおり、知覚が出すのは表面の点で、ブロック中心 = 表面の z − 0.015。**加算ではなく減算**)。
- 🟡 `IkPolicy` は受け取ったブロック中心をそのまま使う。知覚側の変換は TASK-0008 / TASK-0013 の責任。
- 🟡 `solve_ik` の `target_pos` は `gripperframe + R·tcp_offset`(TCP = ブロック中心)に合わせる。すなわち `tcp_offset` は「ブロック中心の `gripperframe` 局所座標」。`IkPolicy` は `goal.grasp_pos` を `target_pos` に、`trajectory.tcp_offset` を `tcp_offset` に渡す。
- 🔴 すべての経由点の目標は TCP(ブロック中心)で表す。`pre_grasp`、`lift`、`pre_place` も TCP 基準。

### 7.3 経由点の定義と座標(TCP 基準)🔴
| 経由点 | TCP の目標 | 備考 |
|---|---|---|
| `pre_grasp` | `grasp_pos + (0, 0, approach_height)`、approach_height = 0.08 | 真上 8 cm(design 4.5) |
| `grasp` | `grasp_pos` | 閉じる前の位置 |
| (`close`) | 腕は `grasp` のまま、グリッパーだけ `gripper_closed` へ | 区間は腕を止める |
| `lift` | `(grasp_pos.x, grasp_pos.y, carry_height)` | 真上へ持ち上げる |
| `pre_place` | `(box.x, box.y, carry_height)` | 箱の真上の運搬高さ。箱中心は `SCENE_SPEC.box_center_xy` |
| `place` | `(box.x, box.y, place_height)` | 箱の底へ置く |
| (`open`) | 腕は `place` のまま、グリッパーだけ `gripper_open` へ | 区間は腕を止める |

- 🟡 `carry_height` は暫定 0.10(ブロック中心)。根拠: 縁の 0.04 + ブロック半サイズ 0.015 + 余裕 0.02 = 0.075 を下回らないこと。実測(6.3 節 C)で確定。
- 🟡 `place_height` は暫定 0.02(底の上面 0.005 + 半サイズ 0.015)。6.3 節 D で確定。

### 7.4 ピッチの選び方(共通ピッチ)🔵/🟡
1. 🔵 接近点 `pre_grasp` を既定の候補 `PITCH_CANDIDATES_DEG` で解く。採用候補 p を得る(`IkSolution.pitch_candidate_deg`)。
2. 🔵 把持点 `grasp` は `pitch_candidates_deg=(p,)` で解く。シードは接近点の解。
3. 🟡 `lift` と `pre_place` は、`pre_grasp` の p と同じ p で解く(運搬中のピッチを一定に保つため)。`pre_place` が p で解けなければ、既定の候補で解いて新しい p' を使い、その区間の前後を同じ p' で揃える。
   - 🔴 簡単化の案(デフォルト): 軌道全体で **単一の p** を使う。接近点で p を決め、残りのすべての経由点を p で解く。解けない場合は `TrajectoryError`。
4. 🟡 `place` は `pre_place` と同じ p で解く。
5. 🟡 採用した p は `JointTrajectory.metadata` に入れる。

### 7.5 時間配分と補間(安全係数 0.99)🔵/🟡
- 🟡 各区間(経由点から次の経由点へ)の長さ: 関節ごとの `|Δq_j| / (vmax_j × speed_scale)` の最大値を `T` とし、制御周期 `dt` の列 `N = max(1, ceil(T / (dt × 0.99)))` に分ける。
- 🔵 安全係数 0.99(TASK-0010 の推奨)。1 ステップの変化量は `|Δq_j| / N ≤ vmax_j × speed_scale × dt × 0.99 < vmax_j × speed_scale × dt`。`apply_safety` の速度上限を必ず下回る。
- 🟡 補間は関節空間の線形補間。時刻 `t_k = t_start + k × dt`(k = 0..N)。`times` は単調増加で、区間の長さは正。
- 🟡 グリッパーの区間(閉じる・開く)は、腕の関節を変えずにグリッパーだけ動かす区間として扱う。グリッパーの速度上限(vmax_6)で時間を決める。その後に `gripper_dwell_s` の保持区間を入れる(🔴 既定 0.5 s。実機の動きの確認後に調整)。
- 🔴 `speed_scale` は `configs/motion.yaml` の `safety.speed_scale` を読む(sim は 1.0)。二重定義しない。

### 7.6 `JointTrajectory`(`trajectory.py`、frozen dataclass)🟡
- `times: np.ndarray (N,)`: 秒、単調増加。
- `q: np.ndarray (N, 6)`: 関節角。gripper は `q[:, 5]` として一緒に持つ。読み取り専用。
- `waypoint_labels: tuple[str, ...]`: 各サンプルがどの経由点・区間に属するかのラベル(`"pre_grasp"`、`"grasp"`、`"close"`、`"lift"`、`"pre_place"`、`"place"`、`"open"` など)。長さは N。
- `waypoints: tuple[WaypointRecord, ...]`(任意): 名前、TCP 目標、IK の位置誤差、採用ピッチ。テストで順序を確かめるため。
- `metadata: Mapping[str, object]`: 採用ピッチ `pitch_candidate_deg`、`dt`、`tcp_offset`、`carry_height` など。

### 7.7 失敗の扱い 🔴
- 🔴 **順序**: (1) 目標の入力検証 → (2) 作業範囲の検査(`is_in_workspace`、TCP 目標点について、IK の前) → (3) 各経由点の IK → (4) 補間 → (5) FK による干渉確認。どの段階でも、失敗は即座に例外。
- 🔴 **作業範囲外**: `TrajectoryError(reason="target_out_of_workspace", waypoint=...)`。IK は呼ばない。
- 🔴 **IK 失敗**: `IkError`(`out_of_reach` / `not_converged`)を `TrajectoryError(waypoint=..., reason=..., cause=IkError)` に包んで送出する。経由点の名前を含めるため。`TrajectoryError` は `IkError` を `__cause__` に入れる。
- 🔴 **干渉**: `TrajectoryError(reason="floor_clearance" | "box_clearance", ...)`。
- 🔴 `TrajectoryError` は `RuntimeError` の派生(`IkError` と同じ)。`ValueError` は入力の型・shape・NaN の誤りだけ(TASK-0010 と同じ方針)。
- 🔴 `FR-06`: 解が無い・可動域を超える軌道は、動作を実行しない(例外で止める)。

### 7.8 wrist_roll(手首の回転)🔴
- 🔴 **判断: 軌道の全区間で wrist_roll は現在の値(開始時の `obs.q[4]`)のまま。** 理由: IK が roll を変数に含めない(TASK-0011 の判断)。ブロックの向き(yaw)は本タスクの入力にない。yaw を合わせる機能は作らない(YAGNI)。
- 🔴 ブロックの向きの扱いは TASK-0013 の固定位置の実験で確認する。ブロックの yaw が 0 の配置のみが対象。yaw を合わせる必要が出たら、別タスクで `wrist_roll` を `Goal` に加える。
- 🟡 TASK-0011 の記録どおり、roll が可動域の端へ漂う問題を避けるため、解いた後の roll は変えない。

### 7.9 グリッパーの開閉の値 🔴
- 🔴 値は**未定**(6.3 節 B で測る)。決めるまで `configs/motion.yaml` の `gripper_open` / `gripper_closed` は書かない(テストが値に依存しないように、テストでは `obs` の値を使う)。

### 7.10 箱の位置の注意 🟡
- 🟡 `SCENE_SPEC.work_area_y` は (-0.10, 0.10) だが、箱の中心 y は 0.19 で、この範囲の外にある。作業範囲(safety)は y -0.13〜0.26 で箱を含む。TASK-0013 と本タスクは、この違いを前提にすること。
- 🟡 `SCENE_SPEC.work_area_x` は (0.15, 0.35)。箱の中心 x 0.20 は範囲内。

---

## 8. 他タスクとの関係

| タスク | 関係 |
|---|---|
| TASK-0010(安全フィルタ) | 前提。`apply_safety` と作業範囲を使う。TASK-0012 のテストで、生成した軌道を `apply_safety`(前回指令を基準)に通し、全フラグ False、出力 = 入力を確認する |
| TASK-0011(IK) | 前提。`solve_ik` を使う。引き継ぎ事項は 6.1 節 |
| TASK-0013(固定位置 pick & place、VLM なし) | 後続。`IkPolicy` の軌道を `SimRobot` で実行する。毎制御周期 `apply_safety` を通す。成功判定は TASK-0015 の判定器(judge)で行う |
| TASK-0024 | `docs/tasks/so101-vlm-pickplace/TASK-0024.md`。本ノートでは内容を確認していない(関連が不明) |

---

## 9. 未決事項(要件定義で決める)

1. 6.3 節 A〜D の実測値(`tcp_offset`、グリッパーの開閉角、運搬高さ、置く高さ)
2. 6.2 節の FK 確認の数値(床・箱の余裕)とサンプリング間隔
3. `gripper_dwell_s` の値(0.5 s は仮)
4. `Goal` の形(7.1 節)と、箱の位置を `Goal` で渡すか `SCENE_SPEC` から取るか
5. TASK-0012 の単体テストの一覧(TASK-0012.md の 3 件を、経由点の順序・単調な時刻・区間の長さ・制限を守る・FK 干渉・IK 失敗の伝播に具体化する)

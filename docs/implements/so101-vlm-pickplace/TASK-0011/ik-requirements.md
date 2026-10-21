# TASK-0011 TDD 要件定義書: IK(逆運動学)

**機能名**: ik(姿勢制約付き逆運動学 `solve_ik`)
**タスクID**: TASK-0011
**要件名**: so101-vlm-pickplace
**出力ファイル**: `docs/implements/so101-vlm-pickplace/TASK-0011/ik-requirements.md`
**作成日**: 2026-10-20
**前提資料**:
- `docs/implements/so101-vlm-pickplace/TASK-0011/note.md`(タスクノート)
- `docs/implements/so101-vlm-pickplace/TASK-0011/reachability-survey.md`(到達性調査。本書の数値の根拠)
- `docs/requirements.md`(FR-05, FR-06, NFR-05)、`docs/design.md`(1, 4.5, 4.7, 7, 9 章)
- `docs/tasks/so101-vlm-pickplace/TASK-0011.md`、`TASK-0012.md`
- `src/so101_vlm/eval/scene_spec.py`、`assets/scene_pickplace.xml`、`assets/so101/so101_new_calib.xml`

**信頼性の凡例**: 🔵 要件定義書・設計文書にほぼそのまま書いてある / 🟡 文書や到達性調査の実測から妥当に推測 / 🔴 文書になく、本書で決めた(ユーザー確認なし。仮定として記録)

> **ノートからの訂正(重要)**: note.md 6.2 は「接近方向 = `gripperframe` の −z 軸」を仮定していたが、実測で誤りと分かった。**接近方向(指の向き)は `gripperframe` の +x 軸**。note.md 0 節の予備調査の表もこの誤りの上で計算されており、`reachability-survey.md` で置き換える。

---

## 1. 機能の概要

- 🔵 **何をするか**: 目標の 3 次元位置(ワールド座標、m)に手先 `gripperframe` を置く腕の関節角を、MuJoCo の `mj_jacSite` によるヤコビアンを使った減衰最小二乗法(DLS)で求める。5 自由度の腕なので、姿勢は「手先の向きの下向き成分」だけを弱く拘束し、ピッチ(傾き)の候補を真下に近い順に試して、最初に収束したものを採用する(design 4.5)。
- 🔵 **解決する問題**: VLM と座標変換で得た 3 次元位置から、把持・運搬の関節軌道を作るための関節目標が必要(FR-05)。解が無い・可動域を超える場合は、動かさずにエラーにする(FR-06)。
- 🟡 **想定する利用者**: 直接の呼び出し元は TASK-0012 の軌道生成(`IkPolicy` の経由点 `pre_grasp` / `grasp` / `lift` / `pre_place` / `place`)。開発者はテストと調査スクリプトからも使う。
- 🔵 **システム内の位置**: `src/so101_vlm/policy/ik.py`。`Policy` 実装(`IkPolicy`)の内部部品。TASK-0011 の範囲は `solve_ik`・`IkSolution`・`IkError` と定数まで。`IkPolicy.act()` と軌道生成は TASK-0012、安全フィルタは TASK-0010。
- **参照した EARS 要件**: FR-05, FR-06, NFR-05
- **参照した設計文書**: design.md 1(5 自由度 + グリッパー、IK は位置 + 下向き)、4.5(IkPolicy)、7(テスト方針: ik は単体)、9(下向き把持のリスク)

---

## 2. 入力・出力の仕様

### 2.1 公開 API(`src/so101_vlm/policy/ik.py`)

```python
@dataclass(frozen=True)
class IkSolution:
    q: np.ndarray            # shape (6,)、float64、読み取り専用(writeable=False)。JOINT_NAMES の順
    pitch_deg: float         # 解での実際の傾き(接近軸と世界 -z のなす角、度)
    pitch_candidate_deg: float  # 採用した候補ピッチ(度)
    pos_error: float         # 解での位置誤差(m)

class IkError(RuntimeError):
    target: tuple[float, float, float]      # 目標(m)
    reason: str                              # "out_of_reach" | "not_converged"
    best_pos_error: float                    # 全候補・全シードで最良の位置誤差(m)。out_of_reach では推定距離超過分
    best_pitch_deg: float | None             # そのときの傾き(度)。反復していなければ None
    tried_pitches_deg: tuple[float, ...]     # 試した候補(out_of_reach では空)

def solve_ik(
    model: mujoco.MjModel,
    target_pos: ArrayLike,                   # shape (3,)
    seed_q: ArrayLike,                       # shape (6,)
    *,
    pitch_candidates_deg: Sequence[float] = PITCH_CANDIDATES_DEG,
    tcp_offset: ArrayLike = (0.0, 0.0, 0.0), # gripperframe 座標での目標点のずれ(m)
) -> IkSolution: ...
```

| 項目 | 内容 | 信頼性 |
|---|---|---|
| 戻り値を素の配列ではなく frozen dataclass `IkSolution` にする | 採用したピッチと誤差を呼び出し側(TASK-0012 の「接近点と把持点を同じピッチで解く」処理、ログ)が使うため。q は 6 要素で `Robot.send_command` にそのまま渡せる | 🟡 |
| `q` の 6 要素 | `JOINT_NAMES` の順(shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper) | 🔵(design 4.1) |
| `q[5]`(gripper) | seed の値をコピー(可動域外なら範囲内にクリップした値)。IK は gripper を動かさない | 🟡(note 6.1) |
| `q[4]`(wrist_roll) | **seed の値をコピー(解く変数に含めない)**。理由は 3.2 節 | 🟡(調査 2.4 節) |
| 解く変数 | shoulder_pan, shoulder_lift, elbow_flex, wrist_flex の 4 関節 | 🟡(調査 2.4 節) |
| `model` | 呼び出し側が渡す `MjModel`(通常は `PICKPLACE_SCENE_PATH` を読んだもの)。読み取りのみ | 🟡 |
| `target_pos` | ワールド座標(= `baseframe`。原点 (0,0,0))の `gripperframe`(+ `tcp_offset`)の目標位置、m | 🔵(design 4.5) |
| `seed_q` | 初期値。通常は現在の関節状態か、直前の経由点の解(ウォームスタート) | 🟡 |
| `pitch_candidates_deg` | 試すピッチの列。**指定した順に**試す。既定は `PITCH_CANDIDATES_DEG` | 🟡 |
| `tcp_offset` | `gripperframe` の局所座標でのずれ。既定 0(`gripperframe` そのものを目標に合わせる)。把持中心を合わせたい TASK-0012 が使う | 🟡(調査 2.1 節) |

### 2.2 用語と定義

- 🟡 **接近軸** = `gripperframe` の局所 +x 軸(指の向き、手首から指先へ)。実測で、gripper body → site のベクトル、wrist_roll の回転軸と平行(調査 2.1 節)。定数 `APPROACH_AXIS_LOCAL = (1, 0, 0)` として `ik.py` の先頭に置く。
- 🟡 **ピッチ(傾き)** = 接近軸と世界の (0, 0, −1) のなす角(度)。0° が真下。`pitch = degrees(arccos(−a_z))`。傾く向き(土台から外向き・内向き)は問わない。
- 🟡 **`gripperframe` の位置** = 固定顎の先端の内側の角(指先より約 6 mm 手前、固定顎の内面上)。両顎の中央ではない。顎は局所 z 方向に閉じ、可動顎は +z 側(調査 2.1 節)。

### 2.3 入出力の関係とデータフロー

```
target_pos (3,), seed_q (6,)
   │ 入力検証(ValueError) / seed を ctrlrange にクリップしたコピー
   ▼
到達球の事前判定 ── 外 ──▶ IkError(reason="out_of_reach")
   │ 内
   ▼
for pitch in pitch_candidates_deg(指定順):
    for start in [seed, 固定の追加シード...]:
        DLS 反復(専用 MjData、4 関節、毎反復 ctrlrange でクリップ)
        収束(位置 ≤ 1 mm かつ |傾き − pitch| ≤ 5°)→ IkSolution を返す
   ▼ どれも収束しない
IkError(reason="not_converged", best_pos_error, best_pitch_deg, tried_pitches_deg)
```

- 🔵 目標 → 関節角 → (TASK-0012)経由点の列 → 補間 → `Robot.send_command`(design 3, 4.5)。

**参照した EARS 要件**: FR-05, FR-06 / **参照した設計文書**: design.md 4.1(`send_command` は 6 要素)、4.5

---

## 3. 制約条件

### 3.1 数値の定数(`ik.py` 先頭の名前付き定数)

| 定数 | 値 | 根拠 | 信頼性 |
|---|---|---|---|
| `PITCH_CANDIDATES_DEG` | (0, 10, 20, 30, 45, 60) | 調査 3 節(i)。作業範囲は 30° 以下で全点届く。45・60 は予備。真下に近い順 | 🟡 |
| `PITCH_TOL_DEG` | 5.0 | 候補の間を切れ目なく覆う(0〜35°、40〜65°) | 🟡 |
| `POS_TOL_M` | 0.001 | ソルバの収束判定。テストは 2 mm で判定(余裕を持たせる) | 🟡(TASK-0011 テストケース 1 の 2 mm より厳しく) |
| `DAMPING` (λ) | 0.01 | 調査で使い、収束を確認した値 | 🟡 |
| `MAX_STEP_RAD` | 0.2 | 1 反復の更新量のノルム上限 | 🟡 |
| `MAX_ITERS` | 300(1 シード・1 候補あたり) | 調査で使った値。到達可能な点は約 2 ms で収束 | 🟡 |
| `ORIENTATION_WEIGHT` | 0.1(傾きの残差 1 ↔ 位置 0.1 m) | 設計の「重みを下げて拘束」。調査で使った値 | 🟡 |
| `EXTRA_SEEDS` | 固定の 4 関節シード 8 個程度。shoulder_pan は目標の方位角に置き換える。wrist_flex を上限付近(1.55〜1.65 rad)に置いたものを 2 個以上含める | 調査 2.4 節(境界でのシード依存性) | 🟡 |
| `APPROACH_AXIS_LOCAL` | (1, 0, 0) | 調査 2.1 節 | 🟡 |
| 到達球の半径 | shoulder_lift 軸アンカーから `gripperframe` までの最大距離 = 0.4115 m(リンク長の和)。モデルから実行時に計算する | 調査 2.6 節 | 🟡 |

### 3.2 解法

- 🔵 DLS: `dq = Jᵀ (J Jᵀ + λ² I)⁻¹ e`。ヤコビアンは `mj_jacSite`(`tcp_offset` が 0 でないときは同じ body 上の点のヤコビアン `mj_jac`)。
- 🔵 姿勢は下向き成分の 1 行だけを拘束する: 残差 `−cos(pitch) − a_z`、行 `(−[a]× J_rot)` の z 成分 × `ORIENTATION_WEIGHT`。位置 3 行 + 傾き 1 行 = 4 行に対し、変数 4 関節。
- 🟡 **wrist_roll を変数から外す理由**: wrist_roll の回転軸は接近軸と平行で、傾きを変えず、位置も 8 mm の腕で回すだけ(調査 2.4 節)。変数に入れると解の roll が可動域の端へ漂い(例 2.82 rad、上限 2.84)、顎の閉じる向きが予測できなくなる。roll は TASK-0012 が seed で指定する(ブロックの面に顎を合わせる)。代償として、境界付近の到達数が少し減る(把持 15° で 81 → 73 点)が、候補 20°・30° で全点届く。
- 🔵 関節の上下限は **アクチュエータの `ctrlrange`**(design 4.5)。毎反復クリップする。返す `q` は全要素が `ctrlrange` 内(gripper・wrist_roll を含む)。
- 🟡 収束判定は「位置誤差 ≤ `POS_TOL_M` かつ |傾き − 候補| ≤ `PITCH_TOL_DEG`」。両方を満たした時点で止める。
- 🟡 ピッチの候補ごとに、`seed_q` → `EXTRA_SEEDS` の順で試す。最初に収束した解を返す(候補の順が優先、シードの順が次)。

### 3.3 シミュレーションと入力を変えない

- 🟡 IK は呼び出しごとに **専用の `mujoco.MjData(model)`** を作る(または IK 専用にキャッシュする)。`SimRobot` の `MjData` は受け取らない・触らない。
- 🟡 `model` のフィールドを書き換えない。
- 🟡 入力の `target_pos`・`seed_q`・`tcp_offset`・`pitch_candidates_deg` を変更しない。
- 🔴 スレッド安全性は要求しない(専用 `MjData` を呼び出しごとに作るなら自然に満たす)。

### 3.4 決定性

- 🟡 同じ `model`・入力なら、同じ `IkSolution` を返す(乱数を使わない。追加シードは固定値)。

### 3.5 エラー

| 条件 | 例外 | 内容 | 信頼性 |
|---|---|---|---|
| `target_pos` の shape が (3,) でない、NaN/inf | `ValueError` | 入力の誤り(IK の失敗ではない) | 🟡 |
| `seed_q` の shape が (6,) でない、NaN/inf | `ValueError` | `validate_q_target` を流用 | 🟡 |
| `tcp_offset` の shape が (3,) でない、NaN/inf | `ValueError` | | 🟡 |
| `pitch_candidates_deg` が空、または値が [0, 90] の外 | `ValueError` | | 🔴 |
| 目標が到達球の外(保守的な判定。本当は届く点を外と判定しない) | `IkError(reason="out_of_reach")` | 反復せずに即座に返す。メッセージに目標、肩からの距離、到達半径 | 🟡 |
| 全候補・全シードで収束しない | `IkError(reason="not_converged")` | メッセージに目標、最良の位置誤差(mm)、そのときの傾き、試した候補 | 🔵(FR-06) |
| `seed_q` が `ctrlrange` の外 | 例外にしない。範囲内にクリップしたコピーを使う | sim の状態はわずかに範囲外になりうるため | 🔴 |

- 🔵 解を返す場合、`q` は必ず `ctrlrange` 内(FR-06「可動域を超えるとき、動作を実行せずエラー」)。
- 🟡 IK は衝突(床・箱・ブロック・自己干渉)を見ない。作業範囲・床の高さのチェックは安全フィルタ(TASK-0010)、干渉の確認はシミュレーション(TASK-0013)。

### 3.6 性能

- 🟡 到達可能な目標: 1 回の呼び出しが通常 50 ms 以下(調査では約 2 ms)。
- 🟡 収束しない目標: 3 秒以下(候補 6 × シード 9 × 300 反復の最悪値の目安。NFR-06 の 1 試行 3 分に対して十分小さい)。到達球の外は即座。

### 3.7 コーディング

- 🔵 関数 50 行未満、ファイル 800 行以下、名前付き定数、入力を変更しない、frozen dataclass(note 2 節、グローバルルール)。
- 🔵 純粋ロジックとして単体テストを書く(NFR-05)。カバレッジ 80% 以上。

**参照した EARS 要件**: FR-06, NFR-05, NFR-06 / **参照した設計文書**: design.md 4.5, 7

---

## 4. 想定される使用例

### 4.1 基本パターン

- 🟡 **把持点**: 目標 (0.20, 0.00, 0.015)、seed = 0 姿勢 → 候補 0° で収束(調査: wrist_flex の余裕 0.45 rad)。`pos_error ≤ 1 mm`、`pitch_deg ≤ 5`。
- 🟡 **接近点(8 cm 上)**: 目標 (0.25, 0.05, 0.095) → 候補 0° は不可(wrist_flex の上限)、10° または 20° で収束。`pitch_candidate_deg ∈ {10, 20}`。
- 🟡 **作業範囲の角**: (0.35, ±0.10, 0.015) → 15〜30° で収束。(0.35, ±0.10, 0.095) → 30° で収束。
- 🟡 **箱**: 解放点 (0.20, 0.19, 0.06) → 0〜10°。箱の上の接近点 (0.20, 0.19, 0.14) → 20〜30°。
- 🟡 **ピッチを 1 つに固定**: `pitch_candidates_deg=(20,)` で呼ぶと、20°±5° の解だけを探す(TASK-0012 が接近点と把持点を同じピッチで解くときに使う)。
- 🟡 **ウォームスタート**: 接近点の解を seed にして把持点を解くと、関節の連続性が保たれる。

### 4.2 エッジケース

- 🟡 真下 0° がちょうど境界にある点(把持の x = 0.25〜0.30): シード次第で 0° または 10° になる。どちらでも仕様どおり(候補 10° で必ず届く)。テストではこの領域を「0° であること」の判定に使わない。
- 🟡 傾きの帯が単調でない: x = 0.15 付近では 45°・60° の方が届かない。候補を大きい順に試すと失敗が増えるため、順序は真下に近い順。
- 🟡 `tcp_offset` が 0 でない: 返した姿勢で `gripperframe` の位置 + R·`tcp_offset` が目標に 1 mm 以内。
- 🟡 seed の gripper が開いている(例 1.0 rad): 返す `q[5]` も 1.0。
- 🔴 目標が床の下(z < 0)でも、運動学的に届けば解を返す(拒否は安全フィルタの役割)。

### 4.3 エラーケース

- 🔵 腕の長さより遠い点 (1.0, 0.0, 0.2): `IkError(reason="out_of_reach")`、メッセージに目標を含む。
- 🟡 到達球の内側だが届かない点 (0.10, 0.0, 0.095)(土台の近くで高い。調査 2.6 節で位置だけでも不可): `IkError(reason="not_converged")`、`best_pos_error > 0.001`。
- 🟡 `pitch_candidates_deg=(0,)` で高い点 (0.25, 0.0, 0.14): 真下は wrist_flex の上限で不可(最小傾き約 20°、`t6_check.py`)→ `IkError(reason="not_converged")`。可動域の外の解を返さないことの確認にも使う。(0.25, 0.0, 0.095) は最小傾きが約 5° で許容 ±5° の境界にあたるため、この判定には使わない。
- 🟡 shape (2,) の目標、NaN を含む seed: `ValueError`。

**参照した EARS 要件**: FR-05, FR-06 / **参照した設計文書**: design.md 4.5、reachability-survey.md 2 節

---

## 5. 他タスク・シーンへの影響

### 5.1 TASK-0012(軌道生成)への影響

- 🟡 `solve_ik` は `IkSolution` を返す。軌道生成は `.q` を経由点に使い、`.pitch_candidate_deg` をログに残す。
- 🟡 **接近点と把持点は同じピッチで解く**ことを推奨する。候補を順に、`pitch_candidates_deg=(p,)` で両方を解き、両方が収束した最初の p を使う。作業範囲では p ≤ 30° で全点見つかる(調査 2.4 節、方式 B・roll 固定)。同じピッチなら、下降は接近軸に沿った移動に近くなる。箱側(`pre_place` z ≈ 0.14、`place` z ≈ 0.06)も同様に 20〜30°。
- 🟡 seed の wrist_roll で顎の向きを決める(roll 軸 ≈ 接近軸。顎は局所 z 方向に閉じ、真下の姿勢では半径方向に閉じる)。ブロックの面に顎を合わせる計算は TASK-0012 の範囲。
- 🟡 `gripperframe` は把持中心ではない。ブロック中心を掴むには `tcp_offset`(局所座標、暫定の目安 (−0.005〜−0.010, 0, +0.015) m)を使うか、目標をずらす。値は TASK-0012/0013 で実測して `configs/motion.yaml` に置く。
- 🟡 ピッチ候補は `ik.py` の定数を既定とし、TASK-0012 が `configs/motion.yaml` から上書きできるようにしてよい(引数で渡せる)。
- 🟡 関節空間の線形補間では、接近点 → 把持点の間で手先が直線に動くとは限らない。干渉は TASK-0013 のシミュレーションで確認する。

### 5.2 TASK-0009(sim 用キャリブレーション)への影響

- 🟡 TASK-0005 の verify-report は「姿勢制約付き IK の到達性は TASK-0009 で確認」としていたが、**到達性の確認は本調査(reachability-survey.md)で済んだ**。TASK-0009 の完了条件に IK の到達性を加える必要はない。シーンを変えた場合だけ、調査スクリプトを再実行する。最終確認は TASK-0013(固定位置での pick & place)で行う。

### 5.3 シーンへの影響と未決事項(ユーザー判断。本書では適用しない)

**到達性の観点では、シーンの変更は不要**(推奨の既定)。以下は、より厳しい条件を望む場合の選択肢。

| ID | 未決事項 | 選択肢 | 推奨の既定 | 信頼性 |
|---|---|---|---|---|
| D1 | 把持は真下(0°)を必須にするか | (a) 必須にしない。0〜30° を許す / (b) 必須にし、作業範囲の x を 0.15〜0.35 → 0.10〜0.30 に移す(`scene_spec.py` の `work_area_x`、TASK-0006 の配置、テストの照合値) | **(a)**。作業範囲を変えると、カメラの画角・箱との距離の確認がやり直しになる。傾きの影響は TASK-0013 で確認する | 🟡 |
| D2 | 接近の高さ(把持点の 8 cm 上)を変えるか | (a) 8 cm のまま、接近点で 10〜30° 傾く / (b) 4〜6 cm に下げる(真下が 6/9 点で届くが、x = 0.35 列は不可のまま) | **(a)** | 🟡 |
| D3 | 箱の位置 (0.20, 0.19) を変えるか | (a) そのまま(解放点 0〜10°、箱の上 20〜30° で届く) / (b) 肩に近づける | **(a)** | 🟡 |
| D4 | ブロックの寸法(一辺 3 cm) | 到達性には影響しない。把持の安定性(TASK-0013)で判断 | 変更しない | 🟡 |
| D5 | 把持中心のオフセット `tcp_offset` の値 | TASK-0012/0013 で実測して決める | 暫定 (−0.005〜−0.010, 0, +0.015) m | 🔴 |

### 5.4 文書の更新が必要な箇所(別タスクで対応。本ステップでは変更しない)

- 🟡 `docs/implements/so101-vlm-pickplace/TASK-0011/note.md` 6.2 節の「接近方向 = −z 軸」と 0 節の予備調査の表は誤り(本書と reachability-survey.md が優先)。
- 🟡 `docs/tasks/so101-vlm-pickplace/TASK-0011.md` のピッチ候補・テストケース 4 の許容角は、本書の値(候補 (0, 10, 20, 30, 45, 60)、許容 ±5°)で読み替える。

---

## 6. EARS 要件・設計文書との対応関係

- **参照したユーザストーリー**: 指示した色のブロックを箱に入れる(requirements.md の v1 の目的)
- **参照した機能要件**: FR-05(IK による軌道生成)、FR-06(解なし・可動域超えでエラー、動かさない)
- **参照した非機能要件**: NFR-05(IK に単体テスト)、NFR-06(1 試行 3 分以内)
- **参照した Edge ケース**: FR-06 の「解が得られない」「可動域を超える」
- **参照した受け入れ基準**: TASK-0011 完了条件(到達可能で収束、到達不可で `IkError`、可動域内)、TASK-0011 テストケース 1〜4
- **参照した設計文書**:
  - **アーキテクチャ**: design.md 1(5 自由度 + グリッパー)、4.1(`send_command` は 6 要素)、4.2(Policy)、4.5(IkPolicy)、9(下向き把持のリスク)
  - **データフロー**: design.md 3(目標位置 → IK → 軌道 → Robot)
  - **型定義**: `src/so101_vlm/robot/base.py`(`JOINT_NAMES`, `N_JOINTS`, `validate_q_target`)、`src/so101_vlm/robot/model_paths.py`(`SITE_NAMES`, `PICKPLACE_SCENE_PATH`)
  - **シーン**: `src/so101_vlm/eval/scene_spec.py`(作業範囲、箱の中心、`block_rest_z`、`box_rim_z`)、`assets/scene_pickplace.xml`、`assets/so101/so101_new_calib.xml`(関節の可動域、`ctrlrange`、`gripperframe`)
  - **データベース / API 仕様**: 該当なし

---

## 7. テストへの引き継ぎ(tdd-testcases 用の要点)

| ID | 内容 | 判定 | 信頼性 |
|---|---|---|---|
| T1 | 到達可能な把持点 (0.20, 0.0, 0.015) | 位置誤差 ≤ 2 mm、`q.shape == (6,)`、`pitch_deg ≤ 5` | 🟡 |
| T2 | 到達可能な接近点 (0.25, 0.05, 0.095) | 位置誤差 ≤ 2 mm、`pitch_candidate_deg > 0`、`|pitch_deg − pitch_candidate_deg| ≤ 5` | 🟡 |
| T3 | 遠い点 (1.0, 0.0, 0.2) | `IkError`、`reason == "out_of_reach"`、メッセージに目標 | 🔵 |
| T4 | 球内だが届かない点 (0.10, 0.0, 0.095) | `IkError`、`reason == "not_converged"`、`best_pos_error > 0.001` | 🟡 |
| T5 | 到達可能な複数点(作業範囲の角と中央、箱) | 全関節が `ctrlrange` 内 | 🔵 |
| T6 | `pitch_candidates_deg=(0,)` で (0.25, 0.0, 0.14) | `IkError`(可動域外の解を返さない) | 🟡 |
| T7 | 入力配列 | 呼び出し前後で `target_pos`・`seed_q` が不変 | 🟡 |
| T8 | `SimRobot` と同じシーンのモデル | IK の前後で `SimRobot.get_state()` が一致 | 🟡 |
| T9 | seed の gripper・wrist_roll | `q[5]`・`q[4]` が seed と一致 | 🟡 |
| T10 | 同じ入力で 2 回 | 結果が完全に一致 | 🟡 |
| T11 | 不正な入力(shape、NaN、空の候補、範囲外の候補) | `ValueError` | 🟡 |
| T12 | `IkSolution` | frozen(代入で例外)、`q` が書き込み不可 | 🟡 |
| T13 | `tcp_offset` が 0 でない | `gripperframe` + R·offset が目標に 2 mm 以内 | 🟡 |
| T14 | 作業範囲 9×9 格子(把持・接近)を既定の候補で解く(遅ければ `slow` マーカー、または 3×3 に間引く) | 全点で解が得られる | 🟡 |

- テストのモデルは `mujoco.MjModel.from_xml_path(str(PICKPLACE_SCENE_PATH))` をモジュール単位の fixture で 1 回読む。
- 判定しきい値(2 mm、5°)はテストファイルの名前付き定数にする。

---

## 8. 品質判定

| 観点 | 評価 |
|---|---|
| 要件の曖昧さ | 小。接近軸・ピッチ・許容・ロールの扱いを実測で確定した。残る未決は D1〜D5(既定あり) |
| 入出力定義 | 完全(型、shape、例外の種類と属性) |
| 制約条件 | 明確(数値は名前付き定数、根拠は調査) |
| 実装可能性 | 確実(調査のソルバで、同じ方式の解が作業範囲の全点で得られている) |
| 信頼性レベル | 🔵 13 / 🟡 約 75 / 🔴 6。🟡 の大半は調査の実測に基づく決定 |

**判定: ✅ 高品質(条件付き)**。🟡 が多いが、ほとんどは文書の推測ではなく実測に基づく。🔴(seed のクリップ、候補値の範囲検証、z<0 を拒否しない、スレッド安全性、`tcp_offset` の暫定値)はユーザー確認なしの仮定として記録した。

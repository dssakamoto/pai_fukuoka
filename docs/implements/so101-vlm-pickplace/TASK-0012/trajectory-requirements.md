# TASK-0012 要件定義: 把持・運搬の軌道生成(trajectory / IkPolicy)

**機能名**: trajectory(把持・運搬の軌道生成)
**タスクID**: TASK-0012
**要件名**: so101-vlm-pickplace
**作成日**: 2026-10-21
**作成方法**: tsumiki:tdd-requirements(ユーザー確認なし。既定値を選び、仮定は 🔴 / 🟡 で記録した。決めてほしい点は 8 節)
**前提資料**: `docs/implements/so101-vlm-pickplace/TASK-0012/note.md`、`trajectory-survey.md`(本タスクの実測。数値の根拠)

**信頼性の凡例**: 🔵 要件定義書・設計文書・既存コード・前タスクの記録・本タスクの実測どおり / 🟡 それらから妥当に推測 / 🔴 資料にない。本書で決めた仮定

---

## 0. 実測の要点(trajectory-survey.md)

| 項目 | 値 | 信頼性 |
|---|---|---|
| `tcp_offset`(把持中心の gripperframe 局所座標) | (0.004, 0.0, 0.030) m。暫定値 (−0.0075, 0, 0.015) は sim の固定顎(凸包)に 10 mm めり込むため採らない | 🔵 実測 |
| 顎の閉じる軸 | 局所 z(固定顎 z≈0 側、可動顎 +z 側)。gripper 角が大きいほど開く | 🔵 実測 |
| 開き / 閉じ指令 | 0.6 rad / 0.15 rad(sim で 3 cm のブロックに当たるのは約 0.37〜0.44 rad) | 🔵 実測 |
| 運搬の高さ | `lift` = `pre_grasp`(把持点の 8 cm 上)。下限 0.079 m に対し 0.095 m | 🔵 実測 |
| 置く高さ | 置く面 + 0.030 m(箱の底 0.005 → TCP z 0.035) | 🔵 実測 |
| 箱の上のピッチ | 全配置で 10°(最小の共通 p) | 🔵 実測 |
| 所要時間 | 平均 5.48 s、最大 6.48 s(20 配置 × 3 ブロック) | 🔵 実測 |
| 補間のずれ | 降下 最大 6.7 mm、運搬 最大 63 mm(高さは保つ) | 🔵 実測 |
| sim での成否 | 59/59(IK が解けた全件)。1 件は IK が解けず動作前に拒否 | 🔵 実測 |
| **sim の物理設定** | 既定(pyramidal / impratio 1)では把持が保てない(0/29)。elliptic + impratio 10 で 29/29 | 🔵 実測。対処は TASK-0013/0014(8 節 D1) |

---

## 1. 機能の概要

- 🔵 **何をするか**: 掴む位置(ブロック中心)と置く位置(置く面の点)から、`pre_grasp`(真上 8 cm)→ `grasp` → 閉じる → `lift` → `pre_place`(箱の真上)→ `place` → 開く → `retreat` の経由点を IK で解き、関節空間で線形補間して、制御周期ごとの関節目標の列(`JointTrajectory`)を返す。
  - 参照: design.md 4.5、TASK-0012.md「タスク概要」「経由点」
  - 🟡 `retreat`(開いた後に箱の真上へ戻る)は設計にない追加。解放後に手を箱から抜き、成功判定(解放後 1 秒)を乱さないため。
- 🔵 **解決する問題**: 3 次元位置から、可動域と速度上限を守る pick & place の動作を作る(FR-05)。解けない・干渉する場合は動作を作らずエラーにする(FR-06)。
- 🔵 **位置づけ**: `Policy` Protocol(design 4.2)の v1 実装 `IkPolicy`。v2 の `SmolVlaPolicy` と差し替える(FR-11)。出力は `motion`(安全フィルタ)へ渡る(design 3 節のデータフロー: `policy` → `joint_trajectory` → `motion`)。
- 🔵 **利用者**: TASK-0013(固定位置の pick & place の実行スクリプト)、TASK-0024(VLM → 座標変換 → IK の 1 試行)、TASK-0026(複数試行)。
- **参照した要件**: FR-05、FR-06、FR-11、NFR-02、NFR-05、NFR-07(3)
- **参照した設計**: design.md 4.2(Policy 抽象)、4.5(IkPolicy)、4.6(安全フィルタ)、5 節(`motion.yaml`)

---

## 2. 入力・出力の仕様

### 2.1 ファイル構成 🟡

| ファイル | 内容 | 層 |
|---|---|---|
| `src/so101_vlm/policy/base.py` | `Observation`、`Goal`、`Policy` Protocol | 型のみ(MuJoCo 不要) |
| `src/so101_vlm/policy/trajectory.py` | `JointTrajectory`、`TrajectoryError`、`JointWaypoint`、時間配分・補間・指令列の検査(純粋関数) | 純粋(MuJoCo 不要) |
| `src/so101_vlm/policy/trajectory_config.py` | `TrajectoryConfig`、`ClearanceConfig`、`load_trajectory_config()` | 設定 |
| `src/so101_vlm/policy/ik_policy.py` | `IkPolicy`(経由点の目標、IK、FK のすき間確認) | MuJoCo を使う |
| `configs/motion.yaml` | 最上位キー `trajectory:` を追加(`safety:` は変えない) | 設定 |

- 🔴 TASK-0012.md は `IkPolicy` の実装先を `policy/ik.py` としているが、`ik.py` は TASK-0011 の IK 単体なので `ik_policy.py` に分ける(note.md 3.2 節)。
- 🟡 各ファイル 800 行以下、関数 50 行未満(共通ルール)。

### 2.2 `policy/base.py` 🟡

```python
@dataclass(frozen=True)
class Observation:
    q: np.ndarray          # (6,) 現在の関節角 [rad]、JOINT_NAMES 順。生成時にコピーして読み取り専用
    timestamp: float       # 状態の時刻 [s]

    @classmethod
    def from_state(cls, state: RobotState) -> "Observation": ...

@dataclass(frozen=True)
class Goal:
    pick_target_base: np.ndarray   # (3,) 掴むブロックの「中心」[m]、baseframe。コピーして読み取り専用
    place_target_base: np.ndarray  # (3,) 置く面の点 [m](箱の底板上面の中心)、baseframe
    instruction: str = ""          # 自然言語の指示(IkPolicy は使わない。記録・v2 用)

class Policy(Protocol):
    def act(self, obs: Observation, goal: Goal) -> JointTrajectory: ...
```

- 🔵 `Policy.act(obs, goal) -> JointTrajectory`(design 4.2)。
- 🔴 `Observation` は `q` と `timestamp` のみ。画像(rgb / depth)は v2(SmolVLA)で追加する。IkPolicy は使わない(YAGNI)。
- 🟡 `pick_target_base` は**ブロックの中心**。知覚は表面の点を出すので、呼び出し側(TASK-0013 は真値、TASK-0024 は知覚)が `z − 0.015` を引いて作る(TASK-0008 の記録: 加算ではなく減算)。
- 🔴 `place_target_base` は**置く面の点**(箱なら底板上面の中心、sim では (0.20, 0.19, 0.005))。知覚が箱の底を見て得る点をそのまま使えるようにするため。ブロックを離す TCP の高さは `place_target_base.z + place_height`。
- 🟡 入力検証(`ValueError`): shape が (6,) / (3,) でない、NaN / inf、bool、`instruction` が str でない。値の範囲(作業範囲)は `TrajectoryError`(2.6 節)。

### 2.3 `JointTrajectory`(`trajectory.py`、frozen dataclass)🟡

| フィールド | 型 | 内容 |
|---|---|---|
| `times` | `np.ndarray (N,)` | 秒。`times[0] = 0.0`、`times[k] = k × control_dt`。厳密に単調増加。読み取り専用 |
| `q` | `np.ndarray (N, 6)` | 関節目標 [rad]、JOINT_NAMES 順。`q[0]` は開始時の `obs.q`(そのままのコピー)。`q[1:]` が送る指令。読み取り専用 |
| `labels` | `tuple[str, ...]`(長さ N) | 各サンプルの区間名。`labels[0] = "start"` |
| `segments` | `tuple[str, ...]` | 区間名の順序(重複なし)。下表 |
| `metadata` | `Mapping[str, object]`(読み取り専用の `MappingProxyType`) | 下表 |

区間(`segments` の値、この順):

| 区間名 | 終点の関節角 | gripper | 備考 |
|---|---|---|---|
| `start` | `obs.q`(1 サンプル) | 現在値 | 送らない |
| `pre_grasp` | `pre_grasp` の IK 解 | `gripper_open` | 移動しながら開く |
| `grasp` | `grasp` の IK 解 | `gripper_open` | 真下へ 8 cm |
| `close` | `grasp` と同じ腕 | `gripper_closed` | グリッパーだけ動く |
| `close_hold` | 同上 | 同上 | `close_hold_s` の間、同じ値を繰り返す |
| `lift` | `pre_grasp` と同じ腕 | `gripper_closed` | IK を解き直さない |
| `pre_place` | `pre_place` の IK 解 | `gripper_closed` | 箱の真上、高さ = `lift` の TCP z |
| `place` | `place` の IK 解 | `gripper_closed` | |
| `open` | `place` と同じ腕 | `gripper_open` | |
| `open_hold` | 同上 | 同上 | `open_hold_s` の間 |
| `retreat` | `pre_place` と同じ腕 | `gripper_open` | 最終姿勢 |

`metadata` のキー(🟡):

| キー | 内容 |
|---|---|
| `control_dt`、`vmax`、`speed_scale`、`time_safety_factor` | 時間配分に使った値 |
| `tcp_offset` | 使った TCP オフセット |
| `pick_pitch_deg` / `place_pitch_deg` | 採用ピッチ候補 p1(把持側)/ p2(置く側) |
| `waypoints` | `tuple[WaypointRecord, ...]`: 名前、TCP 目標 (3,)、関節角 (6,)、IK の位置誤差、実際の傾き [度] |
| `min_obstacle_clearance_m` / `min_self_clearance_m` | FK 確認で得た最小すき間 |
| `duration_s` | `times[-1]` |

- 🔵 不変性: 生成時にコピーし `writeable=False`(`IkSolution` / `RobotState` と同じ)。
- 🟡 生成時の検証(`__post_init__`): `times` と `q` と `labels` の長さが一致、`q` の shape (N, 6)、N ≥ 2、`times` が厳密に単調増加、有限値。違反は `ValueError`。

### 2.4 `IkPolicy`(`ik_policy.py`)🟡

```python
class IkPolicy:
    def __init__(self, model: mujoco.MjModel, *,
                 config: TrajectoryConfig | None = None,     # None なら load_trajectory_config()
                 safety: SafetyConfig | None = None) -> None # None なら load_safety_config()
    def act(self, obs: Observation, goal: Goal) -> JointTrajectory
```

- 🟡 構築時に、モデルから可動域(`limits_from_model`)、ロボットの衝突 geom(`base` body の子孫で contype / conaffinity が 0 でないもの)、障害物の geom(`clearance.obstacle_geoms` の名前)、自己干渉を調べる組(親子以外)を引く。名前が無ければ `ValueError`(構築時に失敗させる)。
- 🔵 入力のモデル・`obs`・`goal` を変更しない。専用の `MjData` を使う(`solve_ik` と同じ方針)。乱数を使わない(同じ入力に同じ出力)。
- 🟡 `act` の処理手順(どこで失敗しても例外。部分的な軌道は返さない):
  1. 入力検証(`ValueError`)。
  2. TCP 目標を作る(2.5 節)。
  3. 作業範囲の検査: 6 つの TCP 目標(`pre_grasp`、`grasp`、`lift`、`pre_place`、`place`、`retreat`)を `is_in_workspace` で確認。外なら `TrajectoryError(reason="target_out_of_workspace", waypoint=名前)`。**IK は呼ばない**。
  4. 運搬の高さの検査: `lift` の z < `min_carry_z` なら `TrajectoryError(reason="carry_too_low")`。
  5. IK(2.5 節の順)。`IkError` は `TrajectoryError(reason=IkError.reason, waypoint=名前)` に包み、`raise ... from err`。
  6. 経由点の関節角を組み立て、時間配分・補間(2.7 節)。
  7. 指令列の検査(2.7 節の定義)。違反は `TrajectoryError(reason="limit_violation")`(実装の誤りを止める防御)。
  8. TCP 経路の作業範囲の検査: `pre_grasp` 区間の終点より後の全サンプルの FK による TCP が作業範囲内。外なら `TrajectoryError(reason="path_out_of_workspace", waypoint=区間名)`。🟡 最初の区間(開始姿勢 → `pre_grasp`)は対象外(ホームの TCP (0.395, −0.002, 0.256) が範囲外のため。trajectory-survey.md 2.3 節)。
  9. FK のすき間確認(2.8 節)。
  10. `JointTrajectory` を返す。

### 2.5 経由点(TCP 目標)とピッチ 🔵/🟡

`P = goal.pick_target_base`、`S = goal.place_target_base`、TCP = `gripperframe + R · tcp_offset`。

| 経由点 | TCP 目標 | IK の呼び方 | 信頼性 |
|---|---|---|---|
| `pre_grasp` | `P + (0, 0, approach_height)` | 既定の候補 `PITCH_CANDIDATES_DEG`、シード = `obs.q`(gripper = `gripper_open`、roll = 開始値)。採用候補を p1 とする | 🔵 TASK-0011 の引き継ぎ |
| `grasp` | `P` | `pitch_candidates_deg=(p1,)`、シード = `pre_grasp` の解 | 🔵 TASK-0011 の引き継ぎ |
| `lift` | `pre_grasp` と同じ | IK なし(`pre_grasp` の解を使う) | 🔵 実測(別の高さだと `(p1,)` で解けない点がある) |
| `pre_place` | `(S.x, S.y, lift の z)` | 既定の候補、シード = `pre_grasp` の解。採用候補を p2 | 🟡 実測で p2 = 10° 全件 |
| `place` | `(S.x, S.y, S.z + place_height)` | `(p2,)`、シード = `pre_place` の解 | 🟡 |
| `retreat` | `pre_place` と同じ | IK なし | 🔴 |

- 🔵 IK の `tcp_offset` には `trajectory.tcp_offset` を渡す。全経由点で同じオフセット(運搬中の保持位置とのずれで、箱の中のずれは最大 20.6 mm。内寸の半幅 40 mm に収まる)。
- 🔴 ピッチは把持側(p1)と置く側(p2)の 2 つ。軌道全体で 1 つの p にはしない(把持側の p1 は 0° が多いが、箱の上の z 0.095 は 0° で解けない)。
- 🔴 すき間確認(2.8 節)で失敗しても、別のピッチで解き直さない(8 節 D4)。

### 2.6 `TrajectoryError` 🟡

```python
class TrajectoryError(RuntimeError):
    reason: str              # 下表
    waypoint: str | None     # 失敗した経由点・区間の名前
    detail: Mapping[str, object]  # 目標点、最小すき間、IkError の属性など
```

| reason | いつ | 信頼性 |
|---|---|---|
| `target_out_of_workspace` | TCP 目標が作業範囲外(IK の前) | 🔵 NFR-07(3)、TASK-0010 |
| `carry_too_low` | `lift` の z < `min_carry_z` | 🔴 |
| `out_of_reach` / `not_converged` | IK の失敗(`IkError.reason` をそのまま使う。`__cause__` に `IkError`) | 🔵 FR-06 |
| `limit_violation` | 生成した指令列が 2.7 節の定義を破る(防御) | 🟡 |
| `path_out_of_workspace` | 補間した TCP 経路が作業範囲外 | 🟡 |
| `obstacle_clearance` | ロボットと障害物(床・箱)のすき間 < しきい値 | 🟡 実測 |
| `self_collision` | 自己干渉のすき間 < しきい値 | 🔵 実測(失敗例を検出) |

- 🔵 `RuntimeError` の派生(`IkError` と同じ)。`ValueError` は入力の型・shape・NaN の誤りだけ(TASK-0010 / 0011 と同じ方針)。
- 🔵 例外のメッセージは日本語で、経由点の名前と値を含む(既存の `IkError` と同じ)。

### 2.7 時間配分・補間・指令列の定義 🔵/🟡

区間 a → b(関節角 `q_a`、`q_b`)について:

- 🔵 刻み数 `N = max(1, ceil(max_j |q_b,j − q_a,j| / (vmax_j × speed_scale × control_dt × time_safety_factor)), ceil(min_segment_s / control_dt))`。
- 🔵 補間: `q_k = q_a + (k / N)(q_b − q_a)`、k = 1..N(関節空間の線形補間。design 4.5)。
- 🟡 保持区間(`close_hold`、`open_hold`): `round(hold_s / control_dt)` 個、同じ値を繰り返す。
- 🔵 `vmax`、`speed_scale`、`control_dt` は `SafetyConfig`(`configs/motion.yaml` の `safety:`)から読む。二重に定義しない。
- 🔵 **指令列の定義(TASK-0010 の「軌道が速度・可動域を守る」)**: 連続する 2 サンプル(`q[0]` と `q[1]` を含む)の差が全関節で `vmax_j × speed_scale × control_dt` 以下、`q[1:]` の全値が `ctrlrange` 内、全 TCP 目標が作業範囲内。係数 0.99 により 1 周期の変化は上限の 99% 以下。
- 🟡 wrist_roll: 全サンプルで `clip(obs.q[4], ctrlrange)`。開始値が範囲外の場合だけ最初の区間で範囲内へ移る。
- 🟡 gripper: 経由点の値(`gripper_open` / `gripper_closed`)を他の関節と同じく補間する。区間の時間はグリッパーも含めた最大で決まる。

### 2.8 FK のすき間確認 🟡

- 🟡 方法: `q` の**全サンプル**で専用 `MjData` に関節角を入れ、`mj_kinematics` → `mj_comPos` → `mj_geomDistance`。実測の計算時間は全サンプルで平均 0.08 s、最大 0.16 s。
- 🟡 障害物: `clearance.obstacle_geoms`(既定 `floor`、`box_floor`、`box_wall_px`、`box_wall_nx`、`box_wall_py`、`box_wall_ny`)。ロボットの全衝突 geom との距離 ≥ `clearance.obstacle_m`(既定 0.001 m)。実測の最小 2.2 mm(床、把持時)。
- 🔵 自己干渉: 親子でない body の衝突 geom の組の距離 ≥ `clearance.self_m`(既定 0.002 m)。正常時 ≥ 8.3 mm、IK が肘を折り畳んだ失敗例は −10.3 mm(trajectory-survey.md 2.5 節)。
- 🔴 対象外: 掴むブロック(わざと触る)、他のブロック(`Goal` に位置がない)。開いた顎が隣のブロックに最大 4.5 mm 重なる計画があるが、sim では 2.6 mm 押されるだけだった。
- 🟡 違反時: 最小すき間が最も小さいサンプルの区間名・値を `TrajectoryError.detail` に入れる。
- 🟡 確認の本体(距離の計算)は `ik_policy.py` に置く。「サンプルごとの距離の列から違反区間を見つける」部分は純粋関数として `trajectory.py` に置き、MuJoCo なしでテストする。

### 2.9 設定 `configs/motion.yaml` の `trajectory:` 🔵/🟡

```yaml
trajectory:
  tcp_offset: [0.004, 0.0, 0.030]   # 把持中心の gripperframe 局所座標 [m]
  approach_height: 0.08             # pre_grasp は把持点の何 m 上か
  min_carry_z: 0.08                 # lift / pre_place の TCP z の下限 [m]
  place_height: 0.030               # 置く面から、ブロックを離す TCP までの高さ [m]
  gripper_open: 0.6                 # [rad] 大きいほど開く
  gripper_closed: 0.15              # [rad] 3 cm のブロックに当たる角度(sim 約 0.37〜0.44)より奥
  close_hold_s: 0.5
  open_hold_s: 0.3
  min_segment_s: 0.2
  time_safety_factor: 0.99
  clearance:
    obstacle_m: 0.001
    self_m: 0.002
    obstacle_geoms: [floor, box_floor, box_wall_px, box_wall_nx, box_wall_py, box_wall_ny]
```

- 🔵 `safety:` セクションは変更しない(`safety/config.py` は `safety:` 以外の最上位キーを無視する)。
- 🟡 ローダー `load_trajectory_config(path=DEFAULT_MOTION_CONFIG_PATH) -> TrajectoryConfig`: `yaml.safe_load`、`trajectory:` と `clearance:` のキーは過不足なく一致(未知・欠落は `ValueError`)。`safety/config.py` と同じ方式。
- 🟡 値の検証(`ValueError`): 有限の数、bool 不可。`tcp_offset` は長さ 3。`approach_height`、`min_carry_z`、`place_height`、`min_segment_s` は > 0。hold は ≥ 0。`time_safety_factor` は (0, 1]。`gripper_open > gripper_closed`。`clearance.*_m` は ≥ 0。`obstacle_geoms` は空でない文字列の列。gripper の値が `ctrlrange` 内かはモデルが要るので `IkPolicy` の構築時に確認(範囲外は `ValueError`)。
- 🟡 `TrajectoryConfig` / `ClearanceConfig` は frozen dataclass(列は tuple)。
- 各値の根拠は YAML のコメントに `trajectory-survey.md` の節を書く(`safety:` と同じ書き方)。

---

## 3. 制約条件

### 3.1 安全・性能

- 🔵 NFR-02: 生成した全指令が可動域内、1 周期の変化が速度上限以下(2.7 節)。テストで `apply_safety`(前回指令を基準)に通し、全フラグ False・出力 = 入力を確認する(TASK-0012.md テストケース 2、TASK-0010 の 6.1 節)。
- 🔵 NFR-07(3): 作業範囲外の TCP 目標は IK の前に拒否。
- 🔵 FR-06: IK が解けない、可動域・すき間を守れない場合は軌道を返さない(例外)。
- 🟡 計算時間: `act` 1 回 0.5 s 以下を目安(実測 IK 約 0.1 s + FK 確認約 0.1 s。IK の未収束は 1 経由点あたり最大約 0.65 s かかりうる)。NFR-06(1 試行 3 分)に対して十分小さい。
- 🔵 所要時間(実行): sim で平均 5.5 s、最大 6.5 s(safety.yaml のコメントの想定 5〜7 s に収まる)。

### 3.2 アーキテクチャ

- 🔵 純粋層(`trajectory.py`、`trajectory_config.py` の検証)と MuJoCo 層(`ik_policy.py`)を分ける。
- 🔵 不変性(入力を変更しない、結果は読み取り専用)。乱数・時計を使わない。
- 🔵 名前付き定数。数値は `configs/motion.yaml`。
- 🔴 `IkPolicy` は「他のブロック」「物理的な把持の成否」を見ない(実行と成功判定は TASK-0013 / TASK-0015)。

---

## 4. 想定される使用例

### 4.1 基本 🔵

```python
model = mujoco.MjModel.from_xml_path(str(PICKPLACE_SCENE_PATH))
policy = IkPolicy(model)
obs = Observation.from_state(robot.get_state())
goal = Goal(pick_target_base=(0.25, 0.0, 0.015), place_target_base=(0.20, 0.19, 0.005))
traj = policy.act(obs, goal)
assert traj.segments == ("start", "pre_grasp", "grasp", "close", "close_hold",
                         "lift", "pre_place", "place", "open", "open_hold", "retreat")
```

### 4.2 エッジケース・エラー 🟡

| ケース | 期待 |
|---|---|
| 作業範囲外の掴む点(例 x = 0.45) | `TrajectoryError(target_out_of_workspace, waypoint="pre_grasp" または "grasp")`、IK を呼ばない |
| 置く面の点が高すぎ `place` / `pre_place` が範囲外(z > 0.17) | `target_out_of_workspace` |
| `lift` の z < `min_carry_z`(既定値では掴む点の z < 0 となり作業範囲の検査が先に拒否する。`approach_height` = 0.05 の設定で z = 0.015 → 0.065 など) | `carry_too_low` |
| 届かない点(肩の近く x = 0.161 など、範囲内でも解けない) | `TrajectoryError(not_converged または out_of_reach)`、`__cause__` が `IkError` |
| 自己干渉する解(IK が肘を折り畳んだ解) | `self_collision` |
| 床にめり込む解 | `obstacle_clearance` |
| `obs.q` が NaN / shape 違い、`Goal` が NaN | `ValueError` |
| `obs.q` が既に `pre_grasp` と同じ | 最初の区間は `min_segment_s`(10 サンプル)で、時刻は単調増加のまま |
| 同じ入力で 2 回呼ぶ | 同じ軌道(決定的) |

### 4.3 データフロー 🔵

`真値 or 知覚(表面の点)` → `Goal`(中心へ変換は呼び出し側)→ `IkPolicy.act` → `JointTrajectory` → 実行ループ(毎周期 `apply_safety` → `Robot.send_command`)。

---

## 5. 単体テストの観点(tdd-testcases への申し送り)🟡

TASK-0012.md の 3 件を具体化する:

1. 経由点の順序: `segments` が 2.3 節の順。`close`・`open` 区間で腕の 5 関節が変わらない。`lift` の関節角 = `pre_grasp`、`retreat` = `pre_place`。gripper の値が区間どおり。
2. 制限を守る: 前回指令を基準に `apply_safety` へ通し、全サンプルで全フラグ False・出力 = 入力。連続差 ≤ `vmax × scale × dt`。`speed_scale` = 0.3 の設定でも同じ(時間が約 3.3 倍)。
3. 区間の時間: `times` が厳密に単調増加、`times[k] = k × dt`、各区間が 1 サンプル以上、保持区間の長さ = hold_s / dt。
4. 純粋関数: 刻み数の計算(境界: ちょうど割り切れる、Δ = 0、係数 0.99)、補間の端点一致、`JointTrajectory` の検証と読み取り専用。
5. 設定ローダー: 既定ファイルが読める、キーの過不足・型・範囲で `ValueError`、`safety:` の読み込みに影響しない。
6. 失敗: 作業範囲外(IK を呼ばないことをモックで確認)、IK 失敗の包み(`waypoint`、`__cause__`)、すき間違反の検出(距離の列を与える純粋関数のテスト + 実モデルで肘を折り畳んだ姿勢を与える結合テスト)。
7. 結合(実モデル、`tests/safety/conftest.py` と同じ点の一部): 代表点で軌道が作れ、TCP が目標に 2 mm 以内(独立 FK)、metadata のピッチが p1 / p2 の規則どおり。sim での実行(把持の成否)は TASK-0013 で確認する(物理設定の変更が要るため)。

---

## 6. 後続タスクへの影響

### 6.1 TASK-0013(固定位置の pick & place)🔵/🟡

- 🔵 **sim の物理設定を変えないと把持が保てない**(既定 0/29、`cone="elliptic"` + `impratio="10"` で 29/29)。`assets/scene_pickplace.xml` に `<option cone="elliptic" impratio="10"/>` を加えるかを TASK-0013 で決める(8 節 D1)。`tests/test_scene.py` との整合も確認する。
- 🔵 実行: `q[1:]` を 1 周期に 1 つ送る。毎周期 `apply_safety(q_current=前回送った指令, ...)`。最初の基準は `q[0]`(= 開始時の計測値)。計測値を基準にすると、追従遅れ(TASK-0010)とグリッパーの停止(ブロックに当たって指令角に届かない場合)で `rate_limited` が立つ。実測では前回指令を基準にして全 59 件・全周期でフラグ 0。
- 🟡 `apply_safety` の `target_base` には、その周期が属する区間の終点の TCP 目標(`metadata["waypoints"]`)を渡す。サンプルごとの FK の TCP を渡すと、最初の区間(ホームは範囲外)で拒否される。
- 🟡 成功判定(FR-07)は軌道の終了後に 1 秒以上待ってから行う(`open_hold` 0.3 s + `retreat` 約 0.3 s の後に、さらに待つ)。
- 🟡 隣のブロックが最大 2.6 mm 押されることがある(開き 0.6)。判定には影響しない。

### 6.2 TASK-0024(VLM → 座標変換 → IK)🔵/🟡

- 🔵 `pick_target_base` = 知覚の表面の点 − (0, 0, 0.015)(ブロック中心)。
- 🟡 水平誤差 5 mm まではほぼ成功(−x 方向で 9/10)、8 mm で 7〜10/10、12 mm で 3〜10/10。VLM の輪郭 bbox の中心をそのまま使うと 7〜10 mm ずれる(TASK-0008)ので、上面中心を使う補正を入れる。
- 🔴 `place_target_base` = 箱の底の点。知覚で得るか、v1 では箱が固定なので `SCENE_SPEC`(箱中心 (0.20, 0.19)、底 0.005)から作るかを TASK-0024 で決める(8 節 D6)。
- 🔵 `TrajectoryError` は「IK / 軌道の失敗」として記録し、指令を送らない(TASK-0024.md の「失敗時は動かない」)。`IkError` は `TrajectoryError` に包まれて届く(`__cause__` で元の IkError を参照できる)。

---

## 7. EARS 要件・設計文書との対応

- **参照した機能要件**: FR-05(軌道の生成と実行)、FR-06(解けない・可動域外は動かない)、FR-11(Policy の差し替え)、FR-07(成功判定の時刻、6.1 節)、FR-12(配置範囲)
- **参照した非機能要件**: NFR-02(関節角・速度の上限)、NFR-05(純粋ロジックの単体テスト)、NFR-06(時間)、NFR-07(3)(作業範囲外の拒否)
- **参照した設計**: design.md 3 節(データフロー)、4.2(Policy 抽象)、4.5(IkPolicy)、4.6(安全フィルタ)、4.7(シーン)、5 節(`motion.yaml`)、9 節(把持が不安定な場合の対策)
- **参照した前タスクの記録**: TASK-0011 `ik-requirements.md`、`ik-verify-complete.md`(共通ピッチ、ウォームスタート、tcp_offset は暫定)、`reachability-survey.md`(接近軸、wrist_flex の上限);TASK-0010 `safety-requirements.md` 6.1 節(軌道の定義、係数 0.99、前回指令の基準)、`safety-verify-complete.md`;TASK-0008 `projection-verify-complete.md`(表面の点 − 0.015、bbox の偏り)
- **参照した既存コード**: `policy/ik.py`、`policy/ik_types.py`、`safety/limits.py`、`safety/config.py`、`safety/workspace.py`、`robot/base.py`、`robot/sim.py`、`eval/scene_spec.py`、`eval/layout.py`、`assets/scene_pickplace.xml`、`assets/so101/so101_new_calib.xml`

---

## 8. 未決事項(ユーザーに決めてほしい点と推奨の既定値)

| # | 内容 | 推奨(本書の既定) | 信頼性 |
|---|---|---|---|
| D1 | sim の把持を保つための物理設定の変更(`scene_pickplace.xml` に `<option cone="elliptic" impratio="10"/>`) | TASK-0013 で入れる(本タスクでは assets を変えない)。代案: 摩擦・`forcerange`・ブロック質量の調整(design 9 節)、固定顎の衝突形状を凸包でなく箱型の geom に置き換え(TASK-0014) | 🔵 実測 |
| D2 | `tcp_offset` を 1 つにするか、運搬側に保持位置のオフセット (0.009, 0, 0.0155) を別に持つか | 1 つ(箱の中のずれ 最大 20.6 mm で十分)。分けると最大 11.4 mm | 🟡 |
| D3 | 最終姿勢: 箱の真上で止める(`retreat`)か、さらにホームへ戻るか | `retreat` で止める(ホームへ戻るのは呼び出し側が別の軌道で行う) | 🔴 |
| D4 | すき間確認に失敗したとき、次のピッチ候補で解き直すか | 解き直さない(失敗は 60 件中 1 件。その点は最終の値では IK が先に失敗する) | 🟡 |
| D5 | 開き角 0.6 rad(箱の壁 4.7 mm、隣のブロックへの重なり小)か、0.8 rad(知覚誤差への余裕は同等、壁 2.1 mm) | 0.6 | 🔵 実測 |
| D6 | `place_target_base` を知覚で得るか、`SCENE_SPEC` から作るか(TASK-0024) | v1 は `SCENE_SPEC`(箱は固定、FR-12) | 🔴 |
| D7 | 保持時間 0.5 s / 0.3 s を短くするか(所要時間 約 0.8 s 減) | 当面このまま(短くした場合の調査はしていない) | 🔴 |
| D8 | 実機の値(顎はメッシュどおりなので当たる角度 約 0.2 rad、把持中心も変わる) | 実機用の `trajectory:` の値は TASK-0035 で測り直し、`configs/real.yaml` 側で上書きする方式を検討 | 🟡 |

---

## 9. 品質判定

| 観点 | 評価 |
|---|---|
| 要件の曖昧さ | ほぼなし。数値は実測で決めた(trajectory-survey.md)。残る判断は 8 節に列挙 |
| 入出力の定義 | 完全(`Observation`、`Goal`、`JointTrajectory`、`TrajectoryError`、設定のキー) |
| 制約条件 | 明確(TASK-0010 の指令列の定義、作業範囲、すき間のしきい値) |
| 実装可能性 | 確実(試作で 59/59 を sim で確認。ただし sim の物理設定の変更が TASK-0013 で必要) |
| 信頼性の分布 | 🔵 約 45%、🟡 約 40%、🔴 約 15%(🔴 は `retreat`、`place_target_base` の意味、ピッチを 2 つに分ける、解き直さない、Observation の中身など設計判断) |

**判定: ✅ 高品質**(ただし D1 の物理設定は TASK-0013 の前提条件として扱う)。

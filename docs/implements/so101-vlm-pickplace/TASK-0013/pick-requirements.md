# TASK-0013 要件定義書: 固定位置での pick & place(VLM なし)

**機能名**: fixed-pick(真の位置から軌道を作り、安全フィルタを通して sim で実行し、判定と動画保存を行う)
**タスクID**: TASK-0013
**要件名**: so101-vlm-pickplace
**作成日**: 2026-10-21
**作成方法**: tsumiki:tdd-requirements(ユーザーへの質問なし。既定値を選び、仮定は 🔴 で記録した。エージェントは起動していない)
**前提資料**: `docs/implements/so101-vlm-pickplace/TASK-0013/note.md`(Q1〜Q5)、`pick-survey.md`(本タスクの実測。数値の根拠)、TASK-0012 の `trajectory-requirements.md` / `trajectory-survey.md` / `trajectory-verify-complete.md`

**信頼性の凡例**: 🔵 要件定義書・設計文書・既存コード・実測(`pick-survey.md`)どおり / 🟡 文書から妥当に推測 / 🔴 文書にない。本書で決めた仮定(要確認)

---

## 0. 実測の要点(`pick-survey.md`)

| 項目 | 値 | 信頼性 |
|---|---|---|
| 物理設定を直したシーン(`<option cone="elliptic" impratio="10"/>`)の成功率 | 赤 20 seed: 20/20。3 色 20 seed: 計画失敗 2、実行 58/58。3 色 100 seed: 計画失敗 18(6.0%)、実行 282/282 | 🔵 実測 |
| 現在のシーン(既定の物理) | 実行 0/58(全件 grasp_slip) | 🔵 実測 |
| 90% に必要な追加調整 | なし(摩擦・solref・質量・保持時間は変えない) | 🔵 実測 |
| 計画失敗の位置 | x = 0.151〜0.169 m のみ(x < 0.17 の配置の 60%) | 🔵 実測 |
| 安全フィルタの介入 | 0 回(全試行・全周期) | 🔵 実測 |
| 既存テスト | シーンを使う既存テスト 1090 件が、物理を変えたシーンで全て通る | 🔵 実測 |
| 1 試行の時間 | 動画なし 約 0.4 s、動画あり 約 8.7 s | 🔵 実測 |

---

## 1. 機能の概要

- 🔵 **何をするか**: ブロックの真の位置(sim の `data.xpos`)から `IkPolicy` で軌道を作り、1 制御周期ごとに `apply_safety` を通して `SimRobot` に送り、ブロックを箱に入れる。終わったら成功判定をし、失敗なら分類する。実行の様子を mp4 に保存できる。
- 🔵 **解決する問題**: VLM・知覚の誤差を入れずに、制御部分(軌道・安全フィルタ・実行・物理)だけで pick & place が成り立つことを先に確かめる(TASK-0013 の目的)。TASK-0024(VLM → 座標 → IK)の土台になる。
- 🔵 **想定ユーザー**: 開発者本人(sim での動作確認、デモ動画、後続タスクの回帰確認)。
- 🔵 **システム内の位置**: design.md 3 節のデータフローの `policy → motion(安全フィルタ)→ robot` の部分を、ノードを使わずに 1 プロセスで通す。判定(evaluator)の最小版を含む。
- **参照した要件**: FR-05、FR-06、FR-07、FR-10、FR-12、NFR-02、NFR-05、NFR-06
- **参照した設計文書**: design.md 4.1(Robot)、4.2(Policy)、4.5(IkPolicy)、4.6(安全フィルタ)、4.7(シーン)、4.8(Evaluator)、7(テスト方針「シミュレーション 統合: 固定位置のブロックを IK で箱に入れる」)

### 1.1 本タスクの範囲

| 含む | 含まない(担当) |
|---|---|
| シーンの物理設定の変更(2 節) | 摩擦・質量・`forcerange` などの把持の調整(TASK-0014。実測で不要と分かった。8 節 O2) |
| 軌道の実行器 `run_trajectory`(3 節) | 実機での実行(TASK-0034/0035) |
| 最小の成功判定 `judge_in_box` と設定(4 節) | 判定の正式化・4 状況のテストの網羅(TASK-0015 が同じファイルを拡張) |
| 失敗の分類(5 節) | 複数試行の集計・CSV(TASK-0026) |
| 1 試行の組み立て `run_fixed_pick`(6 節)と CLI `scripts/run_fixed_pick.py`(7 節)、動画(8 節) | 動画の字幕・注記(TASK-0028)、VLM・知覚(TASK-0024) |

---

## 2. シーンの物理設定の変更

### 2.1 変更内容 🔵

`assets/scene_pickplace.xml` の `<include file="so101/so101_new_calib.xml"/>` の直後(`<compiler>` の前)に 1 行を加える。

```xml
  <!-- 【物理設定】(TASK-0013) 既定 (pyramidal / impratio 1) では持ち上げ中にブロックが抜ける (0/58)。
       楕円の摩擦錐と impratio 10 で 58/58 (docs/implements/so101-vlm-pickplace/TASK-0013/pick-survey.md) -->
  <option cone="elliptic" impratio="10"/>
```

- 🔵 `timestep` は書かない(既定 0.002 s のまま。`SimRobot` の `n_substeps` 10 = 0.02 s の意味は変わらない)。
- 🔵 include 先 `assets/so101/so101_new_calib.xml` に `<option>` は無い(重複しない)。上流ファイル `assets/so101/` は変更しない。
- 🔵 摩擦(`friction="1.0 0.005 0.0001"`)、`condim="4"`、質量、`solref` / `solimp` は変えない(実測で不要)。
- 🟡 `impratio` は 5 でも 58/58 だったが、TASK-0012 の記録と同じ 10 にする。

### 2.2 テストへの影響 🔵

- 既存テストは変更不要(シーンを使う 1090 件が通ることを実測)。静止時の z 誤差 0.108 mm(許容 2 mm)、配置後のずれ 0。
- 追加: `tests/test_scene.py` に、モデルの物理設定を確かめるテストを 1 件加える。
  - `model.opt.cone == mujoco.mjtCone.mjCONE_ELLIPTIC` かつ `model.opt.impratio == 10.0` かつ `model.opt.timestep == 0.002`。
- 🟡 TASK-0012 の `IkPolicy` の結合テスト(`tests/policy/test_ik_policy.py`)は FK・距離だけを使うため、物理の変更の影響を受けない(実測で全て通る)。

---

## 3. 実行器 `run_trajectory`

**ファイル**: `src/so101_vlm/policy/executor.py`(🟡 軌道を送る側の責務。MuJoCo に依存しない。`Robot` Protocol だけを使う)

### 3.1 API 🟡

```python
def run_trajectory(
    robot: Robot,
    trajectory: JointTrajectory,
    safety_config: SafetyConfig,
    *,
    lower: np.ndarray,            # (6,) 可動域の下限 (limits_from_model)
    upper: np.ndarray,            # (6,) 上限
    settle_s: float = 1.0,        # 最後の指令を保持して待つ時間 [s]。0 以上の有限値
    record: bool = False,         # True なら毎周期の RobotState を ExecutionResult.states に残す
    on_step: Callable[[int, str], None] | None = None,  # 送信の後に (周期の番号, 区間名) で呼ぶ。待ちの周期は "settle"
) -> ExecutionResult: ...
```

- 🟡 `lower` / `upper` を引数で受ける(`Robot` Protocol は可動域を持たない。実機でも同じ API で使える)。
- 🔴 `on_step` は動画の描画・ブロックの追跡のための入口(実行器は MuJoCo を知らない)。例外は呼び出し側へそのまま伝える。

### 3.2 ループの仕様 🔵(TASK-0012 の引き継ぎ)

1. 🔴 開始時に `robot.get_state().q` と `trajectory.q[0]` の差の最大が `START_TOLERANCE_RAD = 0.05` 以上なら `ValueError`(別の姿勢から作った軌道を送らない)。
2. `prev = trajectory.q[0]`。`trajectory.q[1:]` の各行 `row`(区間名 `label`)について:
   1. `state = robot.get_state()`
   2. `result = apply_safety(prev, row, lower=lower, upper=upper, last_state_t=state.timestamp, now=state.timestamp, config=safety_config, target_base=区間の終点の TCP 目標)`
   3. `robot.send_command(result.q_command)`、`prev = result.q_command`、フラグを数える、`on_step(i, label)`
3. 🔵 `target_base`: 区間名 → 経由点の対応(下表)で `trajectory.metadata["waypoints"]` の `target` を引く。🟡 `metadata` に `waypoints` が無い軌道(手で作った軌道)は `target_base=None`(作業範囲の確認なし)とし、`ExecutionResult.workspace_checked = False` にする。

| 区間名 | 経由点 |
|---|---|
| `pre_grasp` | pre_grasp |
| `grasp`、`close`、`close_hold` | grasp |
| `lift` | lift |
| `pre_place` | pre_place |
| `place`、`open`、`open_hold` | place |
| `retreat` | retreat |

   - 🟡 この対応は `executor.py` に定数として置く(`SEGMENT_TABLE` の区間名と一致することを単体テストで確かめる)。未知の区間名は `ValueError`。
4. 🟡 `rejected` または `stopped` が立った周期で送信を打ち切る(その周期の保持指令は送る)。残りの行は送らず、待ちへ進む。`aborted=True`、`abort_reason=result.reason`、`abort_label=label` を記録する。`clipped` / `rate_limited` は数えて続ける。
5. 🔵 待ち: `ceil(settle_s / safety_config.control_dt)` 周期、`prev` を送り続ける(既定 1.0 s = 50 周期)。待ちの周期も `apply_safety` を通す(🟡 値は変わらないがフラグを数える)。
6. 入力(`trajectory`、`lower`、`upper`)を変更しない。

### 3.3 `ExecutionResult`(frozen dataclass)🟡

| フィールド | 型 | 内容 |
|---|---|---|
| `n_commands` | int | 送った軌道の行数(打ち切りなしなら `len(q) - 1`) |
| `settle_steps` | int | 待ちの周期数 |
| `flag_counts` | Mapping[str, int](読み取り専用) | `clipped` / `rate_limited` / `rejected` / `stopped` の立った周期数 |
| `aborted` / `abort_reason` / `abort_label` | bool / str \| None / str \| None | 打ち切りの有無と理由・区間 |
| `workspace_checked` | bool | `target_base` を渡したか |
| `final_command` | np.ndarray (6,)、読み取り専用 | 最後に送った指令 |
| `final_state` | RobotState | 待ちの後の状態 |
| `final_joint_error` | float [rad] | `max |final_state.q − final_command|`(実測 0.0005 rad) |
| `duration_s` | float [s] | `(n_commands + settle_steps) × control_dt`(制御周期で数えた時間) |
| `wall_time_s` | float [s] | 実時間(`time.perf_counter` の差。記録のみ) |
| `states` | tuple[RobotState, ...] \| None | `record=True` のときだけ |

- 🟡 プロパティ `has_safety_flags`: `flag_counts` のどれかが 1 以上。
- 🔵 計画の情報(`TrajectoryError`、計画時間)は実行器に入れない。6 節の `FixedPickOutcome` が持つ。

---

## 4. 成功判定(interim)`judge_in_box`

**ファイル**: `src/so101_vlm/eval/judge.py`(🔵 design.md 6 節の配置。TASK-0015 が同じファイルを拡張する。docstring に「TASK-0013 の最小版。TASK-0015 で正式化」と書く)

### 4.1 API 🟡

```python
class JudgeReason(str, Enum):
    OUTSIDE_INNER = "outside_inner"   # (1) 中心が内寸の外 (margin 込み)
    ABOVE_RIM = "above_rim"           # (2) 中心が縁以上
    MOVING = "moving"                 # (3) 速さがしきい値を超える

@dataclass(frozen=True)
class JudgeResult:
    success: bool
    reasons: tuple[JudgeReason, ...]  # 失敗した条件を (1)(2)(3) の順に全部。成功なら空
    offset_xy: tuple[float, float]    # 箱中心からのずれ [m]
    height: float                     # 中心の z [m]
    speed: float                      # 並進の速さ [m/s]

def judge_in_box(
    centre: ArrayLike,            # (3,) ブロック中心 [m] (ワールド = baseframe、sim)
    linear_velocity: ArrayLike,   # (3,) 並進速度 [m/s]
    config: JudgeConfig,
    spec: SceneSpec = SCENE_SPEC,
) -> JudgeResult: ...
```

- 🔵 条件(FR-07): (1) `|dx| < inner_half − margin` かつ `|dy| < inner_half − margin`(dx, dy は `spec.box_center_xy` からのずれ)、(2) `z < spec.box_rim_z`、(3) `speed ≤ speed_max`。
- 🟡 境界: (1)(2) はちょうどで失敗(厳密な不等号)、(3) はちょうどで成功(FR-07 の「以下」)。
- 🟡 純粋関数。NaN / inf / shape 違い / bool は `ValueError`(既存の `as_vector` を使う)。
- 🔴 ブロックの傾きは判定に使わない(5 節の分類で使う。TASK-0015 で判断)。

### 4.2 設定(`configs/eval.yaml`)🟡

```yaml
# 成功判定 (FR-07)。TASK-0013 の最小版。TASK-0015 で正式化する。
judge:
  margin_m: 0.0         # 内寸からの内側の余裕 [m]。FR-07 どおり 0 (中心が内寸の内側)
  speed_max_mps: 0.01   # 静止とみなす速さ [m/s] (FR-07 の暫定 1 cm/s)
  settle_s: 1.0         # 軌道の終わりから判定までの待ち [s] (FR-07 の「解放 1 秒後」以上)
# 失敗の分類 (TASK-0013 の初期値。数値の根拠は pick-survey.md)
failure:
  lift_min_m: 0.03          # lift の終わりで静止高さから最低これだけ上がっていること [m]
  carry_separation_m: 0.02  # 運搬中のブロック中心と TCP の距離の上限 [m]
  other_move_m: 0.005       # 他のブロックの xy 移動の上限 [m]
  tilt_max_deg: 20.0        # 最終の傾きの上限 [deg]
```

- 🔵 **読み込みの変更が必要**: `load_eval_config` は未知キーを拒否する(`require_exact_keys`)。次を同時に変える。
  - `src/so101_vlm/eval/eval_config.py`: `_TOP_KEYS` に `judge` と `failure` を追加。`JudgeConfig`(`margin_m` ≥ 0、`speed_max_mps` > 0、`settle_s` ≥ 0)、`FailureConfig`(全て > 0、`tilt_max_deg` は (0, 90))を frozen dataclass で追加し、`EvalConfig.judge` / `EvalConfig.failure` に持たせる。
  - `tests/eval/test_eval_config.py`: `VALID` に `judge` / `failure` を加える(加えないと既存の正常系が「必須キーがありません」で失敗する)。TC-C-01 にリポジトリの値の確認を足す。欠落・未知キー・値域のパラメータに新しいキーを足す。
  - 🟡 `margin_m` の上限: `box_inner_half_size` 未満(超えると内側が空になる)。
- 🟡 `settle_s` の判定時点: 軌道の最後(解放 + `open_hold` 0.3 s + `retreat` 約 1 s)から 1 s 後。FR-07 の「解放 1 秒後」より遅いが、実測で解放 1 s 後の速さも最大 3.3e-5 m/s(1 cm/s の 0.3%)なので結果は変わらない。解放時刻に合わせる正式な扱いは TASK-0015 で決める。

---

## 5. 失敗の分類

**ファイル**: `src/so101_vlm/eval/failure.py`(🟡 純粋関数。TASK-0026 の集計で同じ型を使う)

### 5.1 列挙型 🟡

```python
class FailureKind(str, Enum):
    PLANNING_FAILURE = "planning_failure"  # TrajectoryError (動作前に拒否)
    GRASP_SLIP = "grasp_slip"              # 持ち上がらない / 持ち上げ・運搬中に抜けた
    DROP = "drop"                          # place 区間 (解放前) に離れた
    COLLISION_OTHER = "collision_other"    # 他のブロックを動かした
    KNOCKED_OVER = "knocked_over"          # 最終の傾きが大きい
    NOT_IN_BOX = "not_in_box"              # 上のどれでもなく判定が失敗
```

- 🔵 design.md 4.8 の「IK 失敗 / 把持失敗 / 配置失敗」との対応: IK 失敗 = `planning_failure`、把持失敗 = `grasp_slip` / `drop`、配置失敗 = `collision_other` / `knocked_over` / `not_in_box`。VLM 失敗は TASK-0024 以降で追加する。
- 🟡 安全フィルタの打ち切り(`aborted`)は分類に入れず、`ExecutionResult` の値で別に報告する(実測 0 回)。

### 5.2 追跡の要約と分類関数 🟡

```python
@dataclass(frozen=True)
class PickTrace:
    lift_end_rise: float          # lift 区間の最後の周期でのブロック中心 z − 静止高さ [m]
    max_separation_carry: float   # lift / pre_place 区間のブロック中心と TCP の距離の最大 [m]
    max_separation_place: float   # place 区間 (open の前) の同じ距離の最大 [m]
    max_other_move: float         # 他のブロックの xy 移動の最大 (全周期) [m]
    final_tilt_deg: float         # 最終の傾き [deg] (最も近い面の法線と鉛直の角度)

def classify_failure(trace: PickTrace, judge: JudgeResult, config: FailureConfig) -> FailureKind | None: ...
```

- 🟡 判定が成功なら `None`。失敗なら上から順に最初に当てはまるもの:
  1. `lift_end_rise < lift_min_m` または `max_separation_carry > carry_separation_m` → `GRASP_SLIP`
  2. `max_separation_place > carry_separation_m` → `DROP`
  3. `max_other_move > other_move_m` → `COLLISION_OTHER`
  4. `final_tilt_deg > tilt_max_deg` → `KNOCKED_OVER`
  5. それ以外 → `NOT_IN_BOX`
- 🟡 判定が成功でも `max_other_move > other_move_m` なら `FixedPickOutcome.other_block_moved = True` を記録する(成功は取り消さない。FR-07 は対象ブロックだけを見る)。実測の最大は 4.2 mm(しきい値 5 mm に近い。8 節 O5)。
- 🔵 TCP の位置 = `gripperframe` の位置 + 回転 × `tcp_offset`(TASK-0012 と同じ定義)。
- 🔴 しきい値(3 cm、2 cm、5 mm、20°)は初期値。実測で `grasp_slip` が付くこと(既定の物理で 58/58、閉じ指令 0.30 で 50 件)だけを確かめた。他の分類の実例はない。

---

## 6. 1 試行の組み立て `run_fixed_pick`

**ファイル**: `src/so101_vlm/eval/fixed_pick.py`(🟡 統合テストと CLI の両方から呼ぶため `src/` に置く。スクリプトの import は避ける)

```python
@dataclass(frozen=True)
class FixedPickOutcome:
    seed: int
    colour: str                         # "red" / "blue" / "green"
    pick_target: tuple[float, float, float]   # 真の位置 (ブロック中心)
    planning_error: str | None          # TrajectoryError.reason (計画失敗のとき)
    planning_waypoint: str | None
    plan_time_s: float
    execution: ExecutionResult | None   # 計画失敗なら None
    judge: JudgeResult | None
    trace: PickTrace | None
    failure: FailureKind | None         # 成功なら None
    other_block_moved: bool

    @property
    def success(self) -> bool: ...      # judge が成功

def run_fixed_pick(
    seed: int,
    colour: str = "red",
    *,
    scene_path: Path = PICKPLACE_SCENE_PATH,
    eval_config: EvalConfig | None = None,       # None なら load_eval_config()
    safety_config: SafetyConfig | None = None,   # None なら load_safety_config()
    trajectory_config: TrajectoryConfig | None = None,
    frame_sink: Callable[[mujoco.MjModel, mujoco.MjData, int, str], None] | None = None,
) -> FixedPickOutcome: ...
```

### 6.1 手順 🔵(実測と同じ)

1. `SimRobot(scene_path)` を `with` で使う。`place_blocks(model, data, sample_layout(seed, n, area, min_gap, max_attempts))`(配置の設定は `eval_config.placement`)。事前の静置はしない(ブロックは静止高さに置かれる。実測で問題なし)。
2. `Goal(pick_target_base=対象ブロックの body の xpos, place_target_base=(box_center_x, box_center_y, box_floor_top_z))`(🔵 TASK-0012 の D6: v1 は `SCENE_SPEC` から作る)。sim では baseframe = ワールド。
3. `IkPolicy(model, config=trajectory_config, safety=safety_config).act(Observation.from_state(robot.get_state()), goal)`。`TrajectoryError` は捕まえて `planning_error` に入れ、実行しない(🔵 FR-06)。
4. `run_trajectory(..., settle_s=eval_config.judge.settle_s, on_step=追跡と frame_sink)`。
5. 追跡(`PickTrace` の値)を毎周期更新し、終わりに `judge_in_box` と `classify_failure`。
- 🔵 `colour` は `red` / `blue` / `green` だけ(`COLOUR_TO_BODY` の定数。他は `ValueError`)。`seed` は 0 以上の整数(`require_int`)。

### 6.2 SimRobot の MuJoCo の扱い 🔴

`place_blocks`・真の位置・描画には `MjModel` / `MjData` が要るが、`SimRobot` は `MjData` を外に出さない(TASK-0004 の方針)。

- **採用(🔴)**: `SimRobot` に sim 専用のメソッド `mujoco_handles() -> tuple[MjModel, MjData]` を加える。docstring に「sim 専用。シーンの準備(ブロックの配置)、真値の読み取り、描画に使う。呼び出し側は `mj_step` を呼ばない。`Robot` Protocol には入れない」と書く。閉じた後は `RuntimeError`。
- 代案: `SimRobot` に `place_blocks` / `body_pose` / `render` を個別に生やす(API が増える)。または `run_fixed_pick` が `SimRobot._model` を読む(非公開属性への依存。不採用)。
- テスト: `mujoco_handles` が同じオブジェクトを返すこと、閉じた後に例外。

---

## 7. CLI `scripts/run_fixed_pick.py`

### 7.1 引数 🟡

| 引数 | 既定 | 内容 |
|---|---|---|
| `--seed N` | 0 | 最初の seed(0 以上) |
| `--n-seeds K` | 1 | seed N..N+K−1 を順に実行(20 seed の掃引は `--n-seeds 20`) |
| `--colour {red,blue,green,all}` | red | 対象。`all` は 3 色全部 |
| `--video PATH` | なし | 1 試行のときだけ有効。mp4 の保存先(省略時は `--out-dir` の既定名) |
| `--no-video` | なし | 動画を書かない(掃引の既定。1 試行の既定は書く) |
| `--headless` | — | 受け付けるだけ(🔴 本タスクは常にオフスクリーン。`view_so101.py` と引数の形を揃えるため)。対話ビューアは作らない |
| `--out-dir DIR` | `media/fixed_pick` | 動画・運搬中の PNG・結果 JSON の保存先(`media/` は `.gitignore` 済み) |

- 起動例: `uv run python scripts/run_fixed_pick.py --seed 0 --colour red --headless --video media/fixed_pick_red_seed0.mp4`
- 掃引例: `uv run python scripts/run_fixed_pick.py --seed 0 --n-seeds 20 --colour all --no-video`(実測 約 20 s)
- 🟡 出力:
  - 標準出力: 1 試行 1 行(seed、色、x, y、成否、失敗の分類、計画失敗の理由、フラグ数、計画時間、実行時間)と最後の集計(試行数、計画失敗数、計画可能な配置での成功数と率)。
  - ファイル: `<out-dir>/fixed_pick_<colour>_seed<seed>.mp4`、`..._carry.png`(`pre_place` 区間の中点の 1 フレーム)、`<out-dir>/fixed_pick_summary.json`(全試行の結果)。
- 🟡 終了コード: 計画可能な試行が全て成功なら 0、実行の失敗が 1 件でもあれば 1、引数の誤りは 2(argparse)。計画失敗だけでは 1 にしない(🔴 FR-06 どおり動作前の拒否は正常動作として扱う)。
- 🔵 関数は 50 行未満に分ける。ファイルは 800 行以下。

### 7.2 動画 🟡

- 🔵 `mujoco.Renderer`(オフスクリーン)と `imageio.v2.get_writer(path, fps=25, codec="libx264", macro_block_size=1)`(`scripts/view_so101.py` と同じ方式)。
- 🟡 カメラ: free camera(`mujoco.MjvCamera`、`type = mjCAMERA_FREE`)。`lookat = (0.22, 0.08, 0.05)`、`distance = 0.62`、`azimuth = 215`、`elevation = −28`。作業エリア・箱・腕が全て入ることを実測の画像で確認した(`pick-survey.md` 2.5 節)。値はスクリプトの名前付き定数。XML は変えない。知覚用の `cam_top` は変えない。
- 🟡 フレーム: 640×480、制御周期 2 回に 1 フレーム(50 Hz → 25 fps)。軌道 4.8〜6.5 s + 待ち 1 s で 約 150〜190 フレーム(6〜8 s の動画)。1 試行 約 8.7 s。
- 🔵 字幕・判定結果の注記は入れない(TASK-0028)。

---

## 8. 受け入れ基準

### 8.1 TASK-0013 の完了条件 🔵

1. **真の位置でブロックを箱に入れられる**: 赤・seed 0 で `run_fixed_pick(0, "red").success` が True。加えて赤・seed 0〜2 も成功(実測 20/20)。
2. **実行の様子を動画で保存できる**: `scripts/run_fixed_pick.py --seed 0 --colour red --headless --video <path>` で mp4 ができ、`imageio` で読めてフレーム数 > 0、サイズ 640×480。運搬中の PNG を開発者が目視で確認する(ブロックが顎に挟まれて運ばれている)。

### 8.2 単体テスト(MuJoCo 不要、または小さなモデル)🟡

| ファイル | 内容 |
|---|---|
| `tests/eval/test_judge.py` | 箱の中で静止 → 成功、内寸の外 → `outside_inner`、縁以上 → `above_rim`、動いている → `moving`、複数の理由、境界(ちょうど内寸−margin は失敗、ちょうど speed_max は成功)、margin の効果、入力検証、入力を変更しない |
| `tests/eval/test_failure.py` | 5 種の分類と優先順位、成功時は None、しきい値ちょうど |
| `tests/eval/test_eval_config.py`(変更) | `judge` / `failure` の読み込み、欠落・未知キー・値域 |
| `tests/policy/test_executor.py` | フェイクの Robot(送った指令を記録し、`get_state` で返す)で: 送る行数 = `len(q) − 1`、指令が `apply_safety` の出力と一致、`target_base` が区間ごとの終点、待ちの周期数、フラグの数え方(速度上限を超える行を含む軌道で `rate_limited`)、範囲外の目標で `rejected` → 打ち切り、開始姿勢の不一致 → `ValueError`、`record=True` で states、`on_step` の呼ばれ方、入力を変更しない、区間の対応表が `SEGMENT_TABLE` と一致 |
| `tests/robot/`(追加) | `SimRobot.mujoco_handles()` |
| `tests/test_scene.py`(追加) | 物理設定(2.2 節) |

### 8.3 統合テスト(sim)🔵

**ファイル**: `tests/eval/test_fixed_pick.py`(🟡 既存に `tests/integration/` は無いため `tests/eval/` に置く)

- seed 0、1、2 × 赤(全て計画可能。実測)で `run_fixed_pick` を実行し、次を全て確かめる:
  - `planning_error is None`、`success`、`failure is None`
  - `execution.has_safety_flags is False`、`execution.aborted is False`
  - `execution.final_joint_error < 0.01`(実測 0.0005 rad)
  - `other_block_moved is False`
- 🟡 計画失敗の経路: 計画失敗する既知の配置(青 seed 16、x = 0.169、`self_collision`)で `planning_error == "self_collision"`、`execution is None`、`failure == PLANNING_FAILURE`。
- 🔵 実行時間 約 1.5 s(3 件)。`slow` マーカーは付けない。
- 🔵 20 seed の掃引はテストにしない。`scripts/run_fixed_pick.py --n-seeds 20 --colour all --no-video` を手動で実行し、結果を `tdd-verify-complete` の記録に残す。**期待値: 60 件中 計画失敗 2(青 16、緑 5)、計画可能な 58 件で 58/58 成功**(実測)。

### 8.4 全体 🔵

- `uv run pytest -q` が全て通る(基準線 1243 件 + 追加分)。追加コードのカバレッジ 80% 以上。
- 既存のファイルで変えるもの: `assets/scene_pickplace.xml`(1 行)、`configs/eval.yaml`(`judge` / `failure`)、`src/so101_vlm/eval/eval_config.py`、`src/so101_vlm/robot/sim.py`(`mujoco_handles`)、`tests/eval/test_eval_config.py`、`tests/test_scene.py`。それ以外の既存コードは変えない。

---

## 9. 制約条件

- 🔵 NFR-02: 送る全指令は `apply_safety` の出力(毎周期)。フラグを捨てない(`ExecutionResult` に数える)。
- 🔵 NFR-05: 判定・分類・実行器の純粋部分に単体テスト。
- 🔵 NFR-06: 1 試行 約 0.4 s(動画なし)/ 約 8.7 s(動画あり)。3 分に対し十分小さい。
- 🔵 FR-10 / FR-12: 実行器は `Robot` Protocol だけを使う(実機に差し替えられる)。`mujoco_handles` は sim 専用の組み立て層(`eval/fixed_pick.py`)だけが使う。
- 🔵 不変性: 結果は frozen dataclass、配列は読み取り専用、入力を変更しない。
- 🔵 設定値は YAML(`configs/eval.yaml`)、動画のカメラ・フレームの値はスクリプトの名前付き定数。
- 🔵 セキュリティ: 外部入力は CLI 引数と YAML だけ(`yaml.safe_load`、厳格なキー検査)。出力先は `--out-dir` の下だけに書く。

---

## 10. 想定される使用例

### 10.1 基本

```python
outcome = run_fixed_pick(0, "red")
assert outcome.success and outcome.failure is None
print(outcome.execution.duration_s, outcome.judge.offset_xy)   # 約 6.4 s、(-0.002, -0.004)
```

### 10.2 データフロー

`seed` → `sample_layout` → `place_blocks` → 真の位置 → `Goal` → `IkPolicy.act` → `JointTrajectory` → `run_trajectory`(毎周期 `apply_safety` → `SimRobot.send_command`、`on_step` で追跡・描画)→ 待ち 1 s → `judge_in_box` → `classify_failure` → `FixedPickOutcome` → 標準出力・JSON・mp4

### 10.3 エッジケース・エラー

| 状況 | 振る舞い | 信頼性 |
|---|---|---|
| 肩の近く(x < 0.17)で IK が解けない | `planning_failure`(理由付き)、実行しない、終了コードは 0 | 🔵 実測 6.0% |
| 軌道に可動域外・速度超過の行がある(手で作った軌道) | `clipped` / `rate_limited` を数えて続ける。統合テストでは 0 を要求 | 🟡 |
| 目標が作業範囲の外 | `rejected` → 打ち切り、`aborted=True` | 🟡 |
| ロボットの姿勢が軌道の開始と違う | `ValueError` | 🔴 |
| 未知の色、負の seed、`settle_s < 0` | `ValueError` / argparse のエラー(終了コード 2) | 🟡 |
| 把持が抜ける(物理設定なしのシーン) | `grasp_slip` | 🔵 実測 |
| 動画の保存先のフォルダが無い | 作る(`mkdir(parents=True, exist_ok=True)`) | 🟡 |

---

## 11. note.md の Q1〜Q5 の決定

| # | 項目 | 決定 | 根拠 | 信頼性 |
|---|---|---|---|---|
| Q1 | 配置領域の x 下限 0.15 | 変えない。計画失敗として報告し、成功率の分母から外す。統合テストは計画可能な seed(赤 0〜2)を使う | 実測で計画失敗 6.0%(x < 0.17 のみ)。layout の変更は TASK-0006 の範囲 | 🔴(8 節 O1 でユーザー判断) |
| Q2 | `<option cone="elliptic" impratio="10"/>` | 入れる | 0/58 → 58/58、既存テスト 1090 件は全て通る | 🔵 実測 |
| Q3 | 判定の置き場所 | `src/so101_vlm/eval/judge.py` に最小 API、しきい値は `configs/eval.yaml` の `judge:` | design.md 6 節の配置。TASK-0015 が拡張 | 🟡 |
| Q4 | 動画のカメラ | free camera(7.2 節の値)、XML は変えない | 実測の画像で構図を確認 | 🟡 |
| Q5 | 分類のしきい値(5 mm、20°ほか) | 初期値のまま `configs/eval.yaml` の `failure:` に置く | 実測で誤検出 0(ただし他のブロックの最大移動 4.2 mm が 5 mm に近い) | 🔴 |

---

## 12. 未決事項(ユーザーの判断が必要)

| # | 項目 | 本書の仮定 | 選択肢 |
|---|---|---|---|
| O1 | 配置領域の x 下限(TASK-0012 O1 の継続) | 0.15 のまま。計画失敗 6.0%(100 seed × 3 色で 18/300、全て x 0.151〜0.169)を許容し、分母から外す | x 下限を 0.17 に上げる(実測では計画失敗 0 になる見込み。`sample_layout` の領域は作業範囲の中心から決まるため、TASK-0006 の領域の決め方の変更が要る) |
| O2 | TASK-0014(把持の調整)の扱い | 物理設定 1 行で 100% のため、追加の調整は不要。TASK-0014 は「調整なし」の記録(`docs/notes/grasp-tuning.md` に本調査の感度: 閉じ指令の幅 0.10〜0.20 rad、impratio 5 でも可、保持時間は短縮可)だけにしてよいか | TASK-0014 で余裕を広げる調整(閉じ指令 0.0 / 0.25 で落ちる原因の調査、顎の衝突形状の箱化)を続ける |
| O3 | `SimRobot.mujoco_handles()` を加えてよいか(MjData を外に出さない方針の例外) | 加える(sim 専用、Protocol 外) | 個別のメソッド(`place_blocks` / `body_pose` / `render`)を `SimRobot` に加える |
| O4 | 判定の `margin_m` | 0.0(FR-07 どおり) | 0.005(note.md の初期案)。実測の最大ずれ 15.2 mm に対しどちらでも結果は同じ |
| O5 | `other_move_m` 5 mm(他のブロックを動かした扱い) | 成功は取り消さず `other_block_moved` を記録 | しきい値を上げる、または成功の条件に入れる(FR-07 にはない) |
| O6 | 判定の時点 | 軌道の終わりから `settle_s` 後(解放から約 2.3 s 後) | 解放(`open` 区間の開始)からちょうど 1 s 後に速さを測る(TASK-0015 で決める。実測ではどちらも 1 cm/s を大きく下回る) |

---

## 13. EARS 要件・設計文書との対応関係

- **参照したユーザストーリー**: 該当ファイルなし(`docs/requirements.md` が正)。TASK-0013.md の概要「VLM を使わずに、制御部分だけを先に確かめる」
- **参照した機能要件**: FR-05(軌道を生成して実行)、FR-06(解けなければ動作しない)、FR-07(成功判定の 3 条件)、FR-10(動画保存)、FR-12(配置の乱数・箱固定)
- **参照した非機能要件**: NFR-02(関節角・速度の上限)、NFR-05(純粋ロジックの単体テスト)、NFR-06(1 試行 3 分以内)
- **参照した Edge ケース**: TASK-0012 の計画失敗(x < 0.17)、既定の物理での把持の抜け(D1)
- **参照した受け入れ基準**: TASK-0013.md の完了条件 2 件と統合テスト 1、TASK-0014.md の「10 回中 8 回以上」(目安)、TASK-0015.md の判定の 3 条件
- **参照した設計文書**:
  - **アーキテクチャ**: design.md 2〜3 節(ノードとデータフロー)、4.1、4.2、4.5、4.6、4.7、4.8、6(`eval/judge.py`)、7(テスト方針)
  - **型定義**: `policy/base.py`(`Observation`、`Goal`)、`policy/trajectory.py`(`JointTrajectory`、`TrajectoryError`)、`safety/limits.py`(`apply_safety`、`SafetyResult`)、`robot/base.py`(`Robot`、`RobotState`)、`eval/scene_spec.py`(`SCENE_SPEC`)、`eval/eval_config.py`(`EvalConfig`)
  - **データベース / API**: 該当なし
- **参照した既存スクリプト**: `scripts/view_so101.py`(mp4 の書き方)、`scripts/capture_scene.py`(`--out-dir`)

---

## 14. 品質判定

| 観点 | 評価 |
|---|---|
| 要件の曖昧さ | 少ない。数値は実測で裏付けた。未決は 12 節の 6 件(既定値で進められる) |
| 入出力の定義 | 完全(`run_trajectory`、`ExecutionResult`、`judge_in_box`、`JudgeResult`、`classify_failure`、`FixedPickOutcome`、CLI、設定) |
| 制約条件 | 明確(安全フィルタ、Robot Protocol、不変性、設定の厳格な読み込み) |
| 実装可能性 | 確実(使い捨ての実行器で同じ手順を 300 件実行済み) |
| 信頼性の分布 | 🔵 約 50%、🟡 約 38%、🔴 約 12%(🔴: 開始姿勢の確認、`on_step`、`mujoco_handles`、`--headless` の扱い、終了コード、分類のしきい値、x 下限) |

**判定: ✅ 高品質**(12 節の未決事項は既定値で実装を進められる。O1・O2・O3 はユーザーの確認を推奨)

# TASK-0013 タスクノート: 固定位置での pick & place(VLM なし)

**機能名**: fixed-pick(真の位置を使う把持・運搬・解放の実行と動画保存)
**タスクID**: TASK-0013
**要件名**: so101-vlm-pickplace
**作成日**: 2026-10-21
**作成方法**: tsumiki:tdd-tasknote(ユーザー確認なしでデフォルトを選び、仮定を 🔴 で記録した。エージェントは起動していない)
**出力ファイル**: `docs/implements/so101-vlm-pickplace/TASK-0013/note.md`

**信頼性の凡例**: 🔵 要件定義書・設計文書・既存コード・前タスク(TASK-0012)の記録どおり / 🟡 文書から妥当に推測 / 🔴 文書にない。本ノートで決めた仮定(要確認)

---

## 0. 収集した情報の範囲

| 種別 | 読んだもの |
|---|---|
| タスク | `docs/tasks/so101-vlm-pickplace/TASK-0013.md`、`TASK-0014.md`(後続: 把持の調整、10 回中 8 回以上)、`TASK-0015.md`(成功判定、未実装)、`overview.md` |
| 仕様・設計 | `docs/spec/so101-vlm-pickplace/note.md`、`docs/requirements.md`(FR-05, FR-07, FR-10, FR-12)、`docs/design.md`(4.5, 4.7, 4.8) |
| 前タスクの記録 | `docs/implements/so101-vlm-pickplace/TASK-0012/note.md`、`trajectory-requirements.md`、`trajectory-survey.md`(実測の根拠)、`trajectory-verify-complete.md`(引き継ぎ D1・O1) |
| 既存コード | `src/so101_vlm/policy/{base.py, ik_policy.py, trajectory.py, trajectory_config.py}`、`src/so101_vlm/safety/{limits.py, config.py}`、`src/so101_vlm/robot/{base.py, sim.py, model_paths.py}`、`src/so101_vlm/eval/{layout.py, scene_spec.py, eval_config.py}`、`src/so101_vlm/nodes/camera.py`、`assets/scene_pickplace.xml` |
| テスト・スクリプト | `tests/test_scene.py`、`scripts/view_so101.py`(オフスクリーン動画の書き方)、`scripts/capture_scene.py` |
| 設定 | `configs/motion.yaml`(`safety:` と `trajectory:`)、`configs/eval.yaml` |

- 追加ルール: プロジェクトルートに `AGENTS.md` と `docs/rule/` は存在しない。適用するのはユーザー側の共通ルール(`~/.claude/rules/ecc/` の common と web。不変性、ファイル 800 行以下、関数 50 行未満、名前付き定数、設定値は YAML に置く、単体テスト 80% 以上、画面を伴う成果物は目視確認)。
- 未存在: `docs/spec/so101-vlm-pickplace/*-user-stories.md` / `*-acceptance-criteria.md`(要件は `docs/requirements.md` が正)。
- 未存在: `docs/notes/grasp-tuning.md`(TASK-0014 で作る。本タスクでは作らない)。
- 未実施: Explore エージェントによる探索(指示により起動しない。直接読んだ)。
- 🔵 テスト基準線: `uv run pytest -q` = 1243 passed(ユーザー指定)。

---

## 1. 技術スタック

- 🔵 Python 3.12、uv、pytest(`testpaths = ["tests"]`)。カバレッジは `coverage`(`source = ["so101_vlm"]`)。
- 🔵 MuJoCo による物理シミュレーション。`SimRobot` が `send_command` 1 回で `n_substeps = 10` 回の `mj_step` を進める(制御周期 0.02 s、sim timestep 0.002 s)。参照元: `src/so101_vlm/robot/sim.py`、`configs/motion.yaml`(`control_dt: 0.02`)
- 🔵 動画・画像: `imageio`(`imageio.v2`)+ `mujoco.Renderer`(オフスクリーン)。参照元: `scripts/view_so101.py`(`run_headless`)、`scripts/capture_scene.py`
- 🔵 アーキテクチャ: 純粋層(`policy/trajectory.py`、`safety/limits.py`、`eval/layout.py`)と MuJoCo に触れる層(`policy/ik_policy.py`、`robot/sim.py`)を分ける。`Robot` Protocol(`get_state` / `send_command` / `close`)の背後で sim と実機を差し替える(FR-10 / FR-12 の方針)。
- 🔵 `Policy` Protocol: `act(obs: Observation, goal: Goal) -> JointTrajectory`。`IkPolicy` が実装(v1)。参照元: `src/so101_vlm/policy/base.py`、`src/so101_vlm/policy/ik_policy.py`

### 1.1 TASK-0012 から使える API(実装済み・変更しない)

| 資産 | 場所 | TASK-0013 での使い方 |
|---|---|---|
| `IkPolicy(...).act(obs, goal) -> JointTrajectory` | `src/so101_vlm/policy/ik_policy.py` | 真の位置 `Goal.pick_target_base`(ブロック中心)と `Goal.place_target_base`(箱の底板上面の中心)から軌道を作る |
| `Observation(q, timestamp)`、`Observation.from_state(state)` | `src/so101_vlm/policy/base.py` | 開始状態 |
| `Goal(pick_target_base, place_target_base, instruction="")` | `src/so101_vlm/policy/base.py` | 目標。`instruction` は v1 では使わない |
| `JointTrajectory`(`q`、区間の情報、`WaypointRecord`) | `src/so101_vlm/policy/trajectory.py` | 実行する行列。1 行が 1 制御周期 |
| `TrajectoryError(reason, waypoint, detail)` | `src/so101_vlm/policy/trajectory.py` | 計画失敗。部分的な軌道は返らない |
| `load_trajectory_config()`(`tcp_offset` ほか) | `src/so101_vlm/policy/trajectory_config.py` | 設定の読み込み |
| `apply_safety(q_current, q_target, *, lower, upper, last_state_t, now, config, target_base=None) -> SafetyResult` | `src/so101_vlm/safety/limits.py` | 毎周期の安全フィルタ |
| `load_safety_config()`、`limits_from_model(model) -> (lower, upper)` | `src/so101_vlm/safety/config.py`、`src/so101_vlm/safety/limits.py` | 可動域と速度上限 |
| `SimRobot(scene_path=SO101_SCENE_PATH, n_substeps=10)` | `src/so101_vlm/robot/sim.py` | 実行対象。`MjData` は外に出ない |
| `SCENE_SPEC` | `src/so101_vlm/eval/scene_spec.py` | 箱・ブロックの寸法と名前(下の 1.2 節) |
| `sample_layout(seed, n=3, area=0.20, min_gap=0.05, max_attempts=10000)` | `src/so101_vlm/eval/layout.py` | 乱数配置。`place_blocks(model, data, layout, block_names)` で書き込む |
| `render_rgbd(model, data, camera_name, width, height)` | `src/so101_vlm/nodes/camera.py` | 固定カメラ `cam_top` の RGB(知覚用。動画は別カメラ可、3 節) |
| `PICKPLACE_SCENE_PATH` | `src/so101_vlm/robot/model_paths.py` | シーンの場所 |

### 1.2 シーンの数値(`SCENE_SPEC` と `assets/scene_pickplace.xml` の一致)

- 🔵 ブロック: 一辺 3 cm(半サイズ 0.015)、質量 20 g、名前 `block_red` / `block_blue` / `block_green`、摩擦 `1.0 0.005 0.0001`、`condim="4"`。
- 🔵 床の上で静止したブロック中心の高さ `block_rest_z = 0.015`。
- 🔵 箱: 中心 (0.20, 0.19)、内寸 8 cm(内寸半幅 0.04)、底板の上面 z = 0.005、縁の高さ `box_rim_z = 0.04`、壁厚 0.005。
- 🔵 作業範囲 x 0.15〜0.35、y −0.10〜0.10(`work_area_x` / `work_area_y`)。
- 🔵 カメラ `cam_top`(640×480)。

---

## 2. 開発ルール

- 🔵 不変性: 入力を変更しない。結果は frozen dataclass、配列は読み取り専用(既存の `Observation` / `Goal` / `JointTrajectory` と同じ方式)。
- 🔵 名前付き定数を使う。しきい値・時間・倍率は YAML に置く(TASK-0013 の判定しきい値は後述の方針で置く)。
- 🔵 安全判定はフラグや例外で明示する。`apply_safety` の `SafetyResult` のフラグを黙って捨てない。
- 🔵 ファイル 800 行以下、関数 50 行未満。
- 🔵 TDD の順序: テストを先に書く(Red → Green → Refactor)。ただし TASK-0013.md は「単体テスト要件はない」としている。本ノートでは 2 節の方針で、純粋関数(判定・実行の結果集計)だけ単体テストを書く(🟡 デフォルトの選択)。
- 🔵 シーン XML の変更は TASK-0013 の責務(TASK-0012 の引き継ぎ D1)。`assets/` の上流ファイル `assets/so101/` は改変しない(`scene_pickplace.xml` のコメントの方針どおり)。

---

## 3. 関連実装

### 3.1 既存の類似コード

- 🔵 `scripts/view_so101.py` の `run_headless(out, seconds)`: `mujoco.Renderer` と `imageio.get_writer` で mp4 を書く。`--headless` / `--out` / `--seconds` の引数の形を真似る。
- 🔵 `scripts/capture_scene.py`: `SETTLE_SECONDS = 1.0` の静置後に `render_rgbd` を呼び、PNG を保存する。`--out-dir` の形を真似る。
- 🔵 `tests/safety/conftest.py`: `task_points`(SCENE_SPEC の 9×9 グリッドの把持点、172 点)。TASK-0012 の IK の到達性検証で使われた。

### 3.2 TASK-0012 の実測(本タスクの前提として使う値)

出典: `docs/implements/so101-vlm-pickplace/TASK-0012/trajectory-survey.md`、`trajectory-verify-complete.md`。

- 🔵 `tcp_offset = (0.004, 0.0, 0.030)`、`gripper_open = 0.6`、`gripper_closed = 0.15`、`approach_height = 0.08`、`min_carry_z = 0.08`、`place_height = 0.030`、`close_hold_s = 0.5`、`open_hold_s = 0.3`、`time_safety_factor = 0.99`(`configs/motion.yaml` の `trajectory:` に入っている)。
- 🔵 所要時間: 59 件の平均 5.48 s、最大 6.48 s、最小 4.92 s(約 250〜325 サンプル)。
- 🔵 計画の計算時間: IK 5 回を含め平均 0.07 s、最大 0.095 s。
- 🔵 TASK-0012 の検証: 成功 141 / 失敗 9(失敗率 6.0%)。失敗の x は 0.151〜0.169 m(肩の近く)のみ。失敗の内訳は `self_collision`(grasp) 6、`not_converged`(pre_grasp) 2、`self_collision`(pre_grasp) 1。
- 🔵 sim の 59 件の実行: 59/59 で箱に入った(IK が解けた全件)。`apply_safety`(前回指令を基準)の介入は 0 回。
- 🔴 **未解決(ユーザー判断)**: 配置領域の x 下限 0.15 は変更しない(TASK-0013 はレイアウトの範囲を変えない)。計画失敗の扱いは 4.4 節。

---

## 4. 設計文書・要件の要点

### 4.1 要件

- 🔵 FR-05: 接近・把持・持ち上げ・移動・解放の関節軌道を生成して実行する。
- 🔵 FR-06: IK が解けない・可動域を超える場合は動作せず、エラーを返す(`TrajectoryError`)。
- 🔵 FR-07: 成功判定(箱の内側、縁より低い、解放 1 秒後の速さ 1 cm/s 以下)。TASK-0015 の責務。
- 🔵 FR-10 / FR-12: 実行は `Robot` の背後に置き、sim と実機を差し替え可能にする(本タスクは sim のみ)。
- 🔵 NFR-02: 関節角・速度に上限。範囲外の指令を送らない(`apply_safety` で毎周期)。
- 🔵 NFR-05: 純粋ロジックに単体テストを書く。
- 🔵 TASK-0013 の完了条件: (a) 真の位置を使ってブロックを箱に入れられる、(b) 実行の様子を動画で保存できる。
- 🔵 TASK-0014(後続)の完了条件: 固定位置で 10 回中 8 回以上成功、パラメータと結果を記録する(`docs/notes/grasp-tuning.md`)。

### 4.2 物理設定の変更(本タスクの前提、TASK-0012 の D1)

- 🔵 実測(TASK-0012 survey 2.1 (5)): 既定の物理設定(摩擦錐 pyramidal、impratio 1)では持ち上げ中にブロックが抜ける(0/29)。`elliptic` かつ `impratio=10` で 29/29、`impratio=5` でも 29/29。
- 🔵 現在の `assets/scene_pickplace.xml` には `<option>` が無い(既定値)。

**方針(デフォルトとして採用)**:
1. `assets/scene_pickplace.xml` の `<mujoco>` 直下に `<option cone="elliptic" impratio="10"/>` を入れる(🟡 `<include>` より後ろに置く。MuJoCo で `option` 要素の重複が許されるかは実装時に確認する。重複が不可なら include 先の値を確認して 1 つにまとめる。🔴 include 先 `assets/so101/so101_new_calib.xml` に `<option>` が既にあるかは未確認)。
2. 摩擦係数・`solref` は変えない(まず `<option>` だけで 3 節の目標を満たすかを見る)。満たさない場合のみ、ブロックの `friction` を上げる(TASK-0014 の範囲。本タスクでは行わない)。
3. 変更の影響は次の既存テストで確認する(全て緑を維持):
   - `tests/test_scene.py`: `REST_TOL = 0.002`(静止時の z 0.015 ± 2 mm)、`SETTLE_STEPS` の後に床に静止していること、箱・ブロックの寸法が `SCENE_SPEC` と一致すること、`test_simrobot_works_on_pickplace_scene_without_disturbing_blocks`。
   - TASK-0006 の統合テスト(配置の既存テスト `tests/eval/test_place_blocks.py`、`tests/eval/test_layout.py`)。
   - `tests/policy/` の IK / 軌道テスト(TASK-0012 の `IkPolicy` の結合テストは sim のモデルを使うため、物理変更の影響を受けうる。失敗したら値の再測定が必要)。
4. TASK-0012 の `trajectory-survey.md` の表は `elliptic/impratio 10` の値であり、変更後のシーンで再計測する(要件 4.3 の結果に含める)。

### 4.3 TASK-0013 の確認方法(sim の成功率)

- 🔵 対象: seed 0〜19(`configs/eval.yaml` の `n_trials: 20`、`base_seed: 0`)。
- 🟡 指標: (a) 赤ブロックのみ、(b) 3 色すべて、それぞれについて「計画可能な配置」の中での成功率。計画失敗は分母から外し、別に数える。
- 🟡 合格ライン: 本タスクでは定めない。ただし TASK-0014 の 10 回中 8 回以上(80%)を目安として報告する。

---

## 5. テスト関連情報

### 5.1 既存テスト(TASK-0013 で増減・変更の対象)

- 🔵 テスト実行: `uv run pytest -q`(基準線 1243 passed)。
- 🔵 配置: `tests/`(`eval/`、`perception/`、`policy/`、`robot/`、`safety/`、直下の `test_scene.py`、`test_assets.py`、`test_package.py`)。
- 🔵 `tests/test_scene.py`: `settled` フィクスチャ(`SETTLE_STEPS` 回の `mj_step`)、`rgbd` フィクスチャ(`render_rgbd` で `cam_top`)。
- 🔵 `tests/eval/`: `test_eval_config.py`、`test_layout.py`、`test_place_blocks.py`。

### 5.2 新規テストの配置(提案)

| 対象 | テストファイル | 種別 |
|---|---|---|
| 成功判定の純粋関数(4.4 節) | `tests/eval/test_judge.py` | 単体(MuJoCo 不要) |
| 実行器の純粋部分(結果の集計、フラグ数、最終誤差) | `tests/policy/test_executor.py` | 単体(フェイク Robot を使う) |
| 固定位置の pick & place(seed 固定) | `tests/integration/test_fixed_pick.py`(🟡 既存に `integration` 用のフォルダは無い。`tests/policy/` に置いてもよい) | 統合(sim、数 seed) |

- 🟡 統合テストの seed: 3 個(例: seed 0、1、2 の赤)。全 20 件の掃引は `scripts/run_fixed_pick.py` の手動実行で行い、テストにはしない(実行時間のため)。
- 🟡 実行時間: 1 件の sim は約 300 制御周期 × 10 `mj_step` ≒ 3,000 `mj_step`。TASK-0012 の試作では 5 件が速く終わっている。統合テストは 3 件程度に抑え、`pytest` のマーカー(例: `slow`)を付けるかは `pyproject.toml` のマーカー設定を見てから決める(🔴 未確認)。

### 5.3 E2E・UI

- 🔵 Web 画面はない。E2E・視覚回帰・ブラウザ試験は対象外。
- 🔵 人が見る出力は動画(mp4)と、キャリア中の 1 フレーム画像(目視確認用)。

---

## 6. 実装の方針

### 6.1 実行器(executor)

**推奨**: `src/so101_vlm/policy/executor.py`(🟡 ``eval/`` ではなく ``policy/`` に置く。軌道を送る側の責務のため。TASK-0026 の集計は `eval/` 側で使う)。

- 🟡 API:
  - `run_trajectory(robot: Robot, trajectory: JointTrajectory, safety_config: SafetyConfig, *, lower, upper, settle_s) -> ExecutionResult`
  - `ExecutionResult`(frozen): `states`(各周期の関節角の記録)、`n_limit_flags` / `n_speed_flags` などの `SafetyResult` のフラグ別件数、`final_q_error`(最後の指令と実測の差の最大値)、`duration_s`(送信した制御周期数 × `control_dt`)、`settle_steps`。
- 🔵 ループ(1 制御周期ごと):
  1. `state = robot.get_state()`(`RobotState`、`timestamp` は sim の `d.time`)。
  2. `result = apply_safety(q_current=prev_command, q_target=row, lower=lower, upper=upper, last_state_t=state.timestamp, now=state.timestamp, config=safety_config, target_base=segment_end_tcp)`。`q_current` は **前回送った指令**(TASK-0010 の推奨。追従遅れで速度上限が縮むのを防ぐ)。
  3. `robot.send_command(result.q)`、`prev_command = result.q`。
  4. 行は `trajectory.q[1:]`(先頭 `q[0]` は開始値)。
- 🔵 `target_base` は区間ごとの終点の TCP 目標(TASK-0012 の引き継ぎ)。区間の途中の行は、その区間の終点の TCP 目標を使う。
- 🟡 `apply_safety` の `now` と `last_state_t` は sim では同じ時刻を入れる(state_timeout の判定が必ず通る)。`state_timeout` の 0.2 s は sim では満たす。実機の扱いは TASK-0035。
- 🔵 送信後、最後の行で `settle_s` 以上(**1 s = 50 制御周期以上**、TASK-0012 の引き継ぎ)、最後の指令を保持して待つ。待ち時間の後に判定する。
- 🟡 フラグが 1 回でも立ったら、その試行は「安全フィルタ介入あり」と記録する(成功扱いにはしない。TASK-0012 の記録では 0 回)。

### 6.2 成功判定(interim)

TASK-0015 が正式な判定を作る。TASK-0013 で必要なのは最低限の判定だけ。

- 🔵 判定の 3 条件(FR-07): (1) ブロック中心が箱の内寸の内側(margin 付き)、(2) 中心が縁より低い、(3) 解放 1 秒後の速さ ≤ 1 cm/s。
- 🟡 入力は真の位置(`data.xpos` の `block_red` の body 中心と、freejoint の線速度 `qvel[0:3]`)。知覚は使わない。
- 🟡 **trade-off と推奨**:
  - 案 A(推奨): `src/so101_vlm/eval/judge.py` に最小 API を先に作る。`judge_in_box(block_centre, block_linear_velocity, *, box_center_xy, inner_half_size, rim_z, margin, speed_max) -> JudgeResult(success: bool, reasons: tuple[str, ...])` のような純粋関数。TASK-0015 はこのファイルを拡張する(4 状況のテスト、しきい値の設定化)。利点: TASK-0013 と TASK-0015 の判定が同じ関数になる。欠点: TASK-0015 の API 設計が先に固まる。
  - 案 B: 実行スクリプトの中に非公開関数として書き、docstring に「TASK-0015 で移す」と書く。利点: TASK-0013 の範囲が小さい。欠点: 後で移す手間と、判定が二重になる危険。
  - **採用: 案 A**。ただし `judge.py` は最小の 1 関数 + 結果型だけにし、docstring に「TASK-0015 で置き換える(interim)」と書く。
- 🟡 しきい値は `configs/eval.yaml` に置く(`judge:` セクションを追加)。値(初期): margin 0.005 m(ブロック中心の箱内寸からの内側余裕)、`speed_max` 0.01 m/s、解放後の待ち 1.0 s。TASK-0015 の値と一致させる。
- 🔴 判定の「箱の中」: 箱の内寸に対する中心の位置。ブロックが斜めに落ちた場合の扱い(ブロック角度)は TASK-0015 で決める。TASK-0013 では角度は見ない(動画で目視)。

### 6.3 計画失敗と実行失敗の分け方

- 🔵 計画失敗: `IkPolicy.act` が `TrajectoryError` を送出(`reason` が `out_of_reach` / `not_converged` / `self_collision` / `obstacle_clearance` / `carry_too_low` 等)。動作前に拒否。
- 🔵 実行失敗: 計画が出た後の結果(把持・運搬・解放の失敗)。
- 🟡 TASK-0013 のスクリプトは、seed ごとに `planning_failure`(TrajectoryError の reason 付き)を記録し、実行へ進まない。実行の成功率の分母は計画できた件数だけ。
- 🟡 **テスト配置の扱い**: 配置の x 下限 0.15 は変えない(TASK-0012 の未決事項 O1)。代わりに、計画失敗が出た seed は「計画失敗」として報告し、統合テストの seed は計画できる seed(0、1、2 は TASK-0012 の 20 配置の検証で計画可能だったものを優先し、実行前に確認する)から選ぶ。🔴 seed 0〜2 が計画可能かは実行時に確認すること。

### 6.4 失敗の分類(TASK-0026 の集計と共通。本タスクで記録する)

| 分類 | 判定(🟡 初期値) |
|---|---|
| `planning_failure` | `TrajectoryError`(reason を保存) |
| `grasp_slip` | 閉じた後の保持中(持ち上げ途中)に、ブロックの中心と手先(`gripperframe`)の距離が閾値を超える、またはブロックが運搬中に落下する前の高さ変化 |
| `drop` | 運搬中にブロックの z が床・箱の縁より下がる、または箱の外に落ちる |
| `collision_other` | 他のブロック(`block_blue` / `block_green`)の位置が初期から 5 mm 以上動く |
| `knocked_over` | ブロックの上軸 z 成分が傾きの閾値(初期値 cos 20° 相当)を下回る |
| `not_in_box` | 成功判定 6.2 の理由(箱の外・縁に乗る・まだ動く) |

- 🔴 閾値(5 mm、20°)は仮定。実測で調整する。
- 🔵 TASK-0013 では分類を記録するところまで。集計は TASK-0026。

---

## 7. 動画・画像の保存

### 7.1 スクリプト `scripts/run_fixed_pick.py`

- 🔵 引数: `--seed`(既定 0)、`--colour`(`red` / `blue` / `green`、既定 `red`)、`--video PATH`(既定 `media/fixed_pick_<colour>_seed<seed>.mp4`)、`--headless`(動画のみ書き出して終了。省略時は対話表示は作らない(🟡 TASK-0013 の範囲外))。
- 🔵 起動例: `uv run python scripts/run_fixed_pick.py --seed 0 --colour red --video media/fixed_pick_red_seed0.mp4 --headless`
- 🟡 出力: (a) mp4、(b) 運搬中の 1 フレームの PNG(`media/fixed_pick_<colour>_seed<seed>_carry.png`、人が目視で確認する)、(c) 標準出力に計画結果・成否・フラグ件数・判定の理由。

### 7.2 カメラ(動画用)

- 🔵 知覚用の `cam_top`(`render_rgbd`)は変えない。
- 🟡 **デフォルト**: 動画は `mujoco.MjvCamera`(free camera)で、ロボットの斜め上から作業エリアと箱を写す(XML は変えない)。`cam_top` は見切れるため、動画の既定にはしない。`cam_top` で撮る選択肢は `--camera cam_top` で切り替える(🔴 引数の追加はしてもしなくてもよい)。
- 🔵 動画の注記(指示、判定結果、verdict の字幕)は TASK-0028 の責務。TASK-0013 は素の mp4 でよい。

### 7.3 フレーム間隔

- 🟡 動画は 1 制御周期(0.02 s)ごとに 1 フレーム、または 50 Hz を 25 fps に間引く(約 12 s の動画)。

---

## 8. 注意事項

### 8.1 技術的制約

- 🔵 IK は衝突を見ない。床・箱・自己干渉は FK サンプリングで確認済み(TASK-0012)。
- 🔵 `tcp_offset` は sim の固定顎の凸包に基づく(TASK-0012 survey 2.1)。実機の値は TASK-0035 で測り直す。
- 🔵 開始姿勢は `q = 0`(ホーム)。TASK-0012 の軌道はこの前提。
- 🔵 ブロックの初期姿勢は世界軸に平行(`place_blocks`)。
- 🔴 `apply_safety` の `target_base` を区間の途中でどこまで使うかは、実装時に `limits.py` の仕様を読んで確認する(区間終点の TCP のみで足りるかを含む)。

### 8.2 セキュリティ・パフォーマンス

- 🔵 セキュリティ: 外部入力はない(sim のみ)。設定値は `safety_config` の検証(`safety/config.py`)を通す。
- 🟡 パフォーマンス: 1 試行 約 3〜5 s(計画 0.07 s + 300 周期)。20 seed の掃引は数分。

### 8.3 完了条件の確認

- 🔵 (a) 真の位置を使ってブロックを箱に入れられる: 赤ブロック、seed 0 を含む計画可能な seed で、判定が成功すること。
- 🔵 (b) 実行の様子を動画で保存できる: `media/` に mp4 が出ること。
- 🟡 記録: 計画可能な 20 seed の掃引結果(赤、3 色)を `docs/implements/so101-vlm-pickplace/TASK-0013/` のレポートまたは TASK-0014 の記録へ残す。

---

## 9. 未決事項(ユーザー判断が必要。本ノートでは仮定として進める)

| # | 項目 | 仮定 | 🔴 |
|---|---|---|---|
| Q1 | 配置領域の x 下限 0.15(TASK-0012 O1) | 変更しない。計画失敗として報告する | 🔴 |
| Q2 | 物理設定 `<option cone="elliptic" impratio="10"/>` を入れてよいか(TASK-0012 D1) | 入れる(TASK-0013 の前提) | 🔴 |
| Q3 | 判定の interim 置き場所(judge.py / スクリプト内) | `judge.py` に最小 API | 🔴 |
| Q4 | 動画のカメラ | free camera(XML は変えない) | 🔴 |
| Q5 | 失敗分類の閾値(5 mm、20°) | 初期値で実測して調整 | 🔴 |

---

## 10. 次の工程への引き継ぎ

- `tsumiki:tdd-requirements` で、上の 6.1〜6.4 と 7 節を要件に落とす(要件の番号と受け入れ基準を作る)。
- 先にテストを書く対象: `tests/eval/test_judge.py`(4 状況)、`tests/policy/test_executor.py`(フラグ件数、最終誤差、settle の長さ)。
- 物理設定の変更は、テストが全部緑になることを確認してから次へ進む(`uv run pytest -q` で 1243 件以上が通ること)。

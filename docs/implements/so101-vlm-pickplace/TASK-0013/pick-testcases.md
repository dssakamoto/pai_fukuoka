# TASK-0013 テストケース定義: 固定位置での pick & place(VLM なし)

**機能名**: fixed-pick
**タスクID**: TASK-0013
**要件名**: so101-vlm-pickplace
**作成日**: 2026-10-21
**作成方法**: tsumiki:tdd-testcases(ユーザーへの質問なし。エージェントは起動していない。数値は使い捨ての実行器で確認した)
**前提資料**: `note.md`、`pick-requirements.md`(以下「要件」)、`pick-survey.md`(以下「実測」)、`docs/tasks/so101-vlm-pickplace/TASK-0013.md` / `TASK-0014.md` / `TASK-0015.md`

**信頼性の凡例**: 🔵 要件・既存コード・実測どおり / 🟡 要件から妥当に推測 / 🔴 要件にない。本書で決めた仮定

---

## 0. 共通事項

### 0.1 確認に使った使い捨ての実行(リポジトリは変更していない)

場所: `/private/tmp/claude-501/-Users-takuya-project-20-pai-homework/b14af3ef-30ce-4f56-a63b-8ac03b5a9e70/scratchpad/pick_survey/tc_check/`(`check1.py`〜`check4.py`。シーンは survey の `scenes/fix.xml` = 物理設定を加えた一時コピー)

| 確認した値 | 結果 | 使うテスト |
|---|---|---|
| 赤 seed 0 / 1 / 2 の把持位置 (x, y) | (0.2774, −0.0460) / (0.2524, 0.0901) / (0.2023, −0.0403) | TC-FP-01 |
| 同 送った軌道の行数 `len(q) − 1` | 273 / 258 / 296(+ 待ち 50 周期) | TC-FP-02、TC-EXI-01 |
| 同 `duration_s` = (行数 + 50) × 0.02 | 6.46 / 6.16 / 6.92 s | TC-FP-02 |
| 同 安全フィルタのフラグ | 全て 0 | TC-FP-01 |
| 同 最終の関節誤差 | 0.00044 / 0.00047 / 0.00043 rad | TC-FP-01 |
| 同 `lift` 終わりの上昇量 | 0.0736 / 0.0760 / 0.0753 m(しきい値 0.03) | TC-FP-03 |
| 同 運搬中の距離の最大 / place 区間の距離の最大 | 0.0158 / 0.0153 / 0.0160 m、0.0160 / 0.0153 / 0.0162 m(しきい値 0.02) | TC-FP-03 |
| 同 他のブロックの移動 | 2e-10 m 程度(しきい値 0.005) | TC-FP-01 |
| 同 置いた位置 (dx, dy)、z、速さ、傾き | (−0.0023, −0.0043) / (−0.0057, −0.0022) / (−0.0007, −0.0059)、z 0.0199、≤ 5e-11 m/s、≤ 3e-6° | TC-FP-04 |
| 同じ seed を 2 回 | 最終位置・全指令・関節誤差がビット単位で一致 | TC-FP-08 |
| 青 seed 16 | `self_collision`、経由点 `grasp`、x = 0.1688 | TC-FP-05 |
| 緑 seed 5 | `not_converged`、経由点 `pre_grasp`、x = 0.1608 | TC-FP-05 |
| 青 seed 0 / 緑 seed 0 | 成功(324 行 / 257 行、フラグ 0) | TC-CLI-03 |
| `apply_safety`: 0 → 0.05 rad を 3 行 | 指令 0.02、0.04、0.05、`rate_limited` 2 回 | TC-EX-03 |
| `apply_safety`: 0.99 → 1.01(上限 1.0) | 指令 1.0、`clipped` のみ | TC-EX-04 |
| `apply_safety`: 時刻 NaN | `stopped`、`stale_state` | TC-EX-06 |
| `apply_safety`: 目標 (0.5, 0, 0.05) | `rejected`、`target_out_of_workspace`、保持指令 | TC-EX-05 |
| `ceil(settle_s / 0.02)` | 1.0→50、0.5→25、0.03→2、0.02→1、0.01→1(`round` なら 0)、0→0 | TC-EX-09 |
| SimRobot で shoulder_pan 0 → 0.3 rad を 11 行 + 待ち 50 | `rate_limited` 11 回、最後の指令 0.22、関節誤差 0.00054 rad、時刻 1.22 s | TC-EXI-02 |
| 動画(赤 seed 0、640×480、25 fps) | 161 フレーム(= floor(323 / 2))、6.44 s、265 KB、1 試行 10.3 s | TC-CLI-08 |
| 動画(160×120) | 161 フレーム、52 KB、1 試行 7.9 s | 縮小の効果の判断(0.4 節) |
| 描画とエンコード | 描画 41 ms/フレーム(640×480 と 160×120 で同じ)、エンコード 6.3 / 1.3 ms/フレーム | 0.4 節 |

### 0.2 テストファイルと件数

| ファイル | 新規/変更 | 正常系 | 異常系 | 境界値 | 計 |
|---|---|---|---|---|---|
| `tests/test_scene.py` | 追加 | 3 | 0 | 0 | 3 |
| `tests/eval/test_eval_config.py` | 変更 | 3 | 4 | 2 | 9 |
| `tests/policy/test_executor.py`(単体: フェイク Robot) | 新規 | 14 | 6 | 4 | 24 |
| `tests/policy/test_executor.py`(統合: SimRobot) | 新規 | 3 | 0 | 0 | 3 |
| `tests/eval/test_judge.py` | 新規 | 6 | 2 | 4 | 12 |
| `tests/eval/test_failure.py` | 新規 | 11 | 1 | 2 | 14 |
| `tests/robot/test_sim.py` | 追加 | 4 | 1 | 0 | 5 |
| `tests/eval/test_fixed_pick.py` | 新規 | 10 | 2 | 0 | 12 |
| `tests/test_run_fixed_pick_script.py` | 新規 | 8 | 2 | 0 | 10 |
| 手動(20 seed の掃引) | — | 1 | 0 | 0 | 1 |
| **計** | | **63** | **18** | **12** | **93** |

(件数は TC の数。`parametrize` で pytest の項目数はこれより多くなる)

### 0.3 実行時間の予算 🔵(実測)

| ファイル | 予算 | 内訳 |
|---|---|---|
| `tests/test_scene.py` 追加分 | < 0.1 s | 既存の `settled` フィクスチャのモデルを使う |
| `tests/eval/test_eval_config.py` 追加分 | < 0.2 s | YAML のみ |
| `tests/policy/test_executor.py` | < 1.5 s | 単体 < 0.2 s(MuJoCo なし)、統合: 計画 1 回 + 実行 約 0.4 s、手書き軌道 0.1 s |
| `tests/eval/test_judge.py` / `test_failure.py` | < 0.1 s | 純粋関数 |
| `tests/robot/test_sim.py` 追加分 | < 0.1 s | 既存の小モデル `tiny_scene_path` |
| `tests/eval/test_fixed_pick.py` | < 4 s | 赤 seed 0〜2(module scope、約 1.5 s)、計画失敗 2 件(約 0.5 s)、決定性(2 回、約 0.8 s)、margin の注入(約 0.4 s) |
| `tests/test_run_fixed_pick_script.py` | < 14 s | 動画なし 4 回(計 約 3 s)、**動画あり 1 回(約 10 s)** |
| **追加の合計** | **< 20 s** | |

### 0.4 動画テストの扱い(決定)🟡

- `pyproject.toml` に `markers` の登録はない(`[tool.pytest.ini_options]` は `testpaths` のみ)。**`slow` マーカーは付けない**(未登録のマーカーは警告になる。マーカーを登録する変更は本タスクの範囲外)。
- 縮小の効果: 描画は 1 フレーム 約 41 ms で、画像サイズにほとんど依らない(640×480 と 160×120 で同じ)。160×120 にしても 10.3 s → 7.9 s にしかならず、受け入れ基準の「640×480」も確かめられなくなる。**サイズを下げる引数は作らない**。フレーム数を減らす引数(`--max-seconds` など)は動画の中身を変えるため作らない(YAGNI)。
- 結論: 動画のテストは **1 件だけ**、640×480 のまま実行する(約 10 s)。mp4、運搬中の PNG、結果 JSON の確認をこの 1 回の実行(module scope のフィクスチャ)にまとめ、他のテストは全て `--no-video` にする。費用はこのファイルの docstring に書く。

### 0.5 共通の定数(テスト側)

| 名前 | 値 | 根拠 |
|---|---|---|
| `DT` | 0.02 s | `configs/motion.yaml` の `control_dt` 🔵 |
| `SETTLE_STEPS_DEFAULT` | 50 | `ceil(1.0 / 0.02)` 🔵 |
| `JOINT_ERR_TOL` | 0.01 rad | 要件 8.3(実測 0.0005) 🔵 |
| `START_TOLERANCE_RAD` | 0.05 rad | 要件 3.2 / 指示 🔵 |
| `PLANNABLE_RED_SEEDS` | (0, 1, 2) | 実測 🔵 |
| `PLAN_FAIL_CASES` | (16, "blue", "self_collision", "grasp")、(5, "green", "not_converged", "pre_grasp") | 実測 🔵 |
| `UNIT_LOWER` / `UNIT_UPPER` | `-np.ones(6)` / `np.ones(6)` | フェイク Robot 用の可動域 🟡 |

### 0.6 フェイク Robot(単体テスト用、`tests/policy/test_executor.py` 内)🟡

- `FakeRobot(q0=np.zeros(6), dt=0.02, nan_time_calls=())`: `joint_names = JOINT_NAMES`。`get_state()` は「最後に送った指令(なければ `q0`)」を `q`、ゼロを `dq`、現在の時刻を `timestamp` とする `RobotState` を返す。`nan_time_calls` に含まれる回数目(1 始まり)の `get_state()` だけ `timestamp = NaN` を返す。`send_command(q)` は `np.array(q, copy=True)` を `sent` に追加し、時刻を `dt` 進める。`close()` は `closed = True`。
- MuJoCo を使わない。`isinstance(FakeRobot(), Robot)` が True(`Robot` は `runtime_checkable`)。
- 軌道の作り方: `make_traj(rows, labels, metadata=None)` = `JointTrajectory(times=np.arange(N) * 0.02, q=rows, labels=labels, segments=区間名の順, metadata=metadata or {})`。
- 作業範囲つきの軌道: `metadata={"waypoints": (WaypointRecord(name, target, np.zeros(6), 0.0, 0.0, 0.0) for name in WAYPOINT_ORDER)}`。内側の目標は (0.25, 0.0, 0.05)(作業範囲 x 0.12〜0.38、y −0.13〜0.26、z 0〜0.17 の内側)。

---

## 1. `tests/test_scene.py`(追加)

既存の module scope フィクスチャ `settled`(`PICKPLACE_SCENE_PATH` のモデル)を使う。既存の 15 関数は変更しない(実測: 物理設定を加えたシーンでシーンを使う既存テスト 1090 件が全て通る。静止時の z 誤差 0.108 mm < `REST_TOL` 2 mm)。

### 1.1 正常系

- **TC-SC-01 物理設定が入っている** 🔵
  - **何をテストするか**: シーンに `<option cone="elliptic" impratio="10"/>` が入り、timestep が変わらないこと(要件 2.1、2.2)。
  - **入力値**: `settled` のモデル。
  - **期待される結果**: `model.opt.cone == mujoco.mjtCone.mjCONE_ELLIPTIC`、`model.opt.impratio == 10.0`、`model.opt.timestep == 0.002`。
  - **確認ポイント**: timestep 0.002 s が変わると `SimRobot` の 1 指令 = 0.02 s の意味が変わる。

- **TC-SC-02 上流のシーンは変わらない** 🟡
  - **何をテストするか**: 物理設定はピック&プレースのシーンだけに入り、`SO101_SCENE_PATH`(上流の scene.xml)は既定のままであること(要件 2.1「上流ファイルは変更しない」)。
  - **入力値**: `mujoco.MjModel.from_xml_path(str(SO101_SCENE_PATH))`。
  - **期待される結果**: `opt.cone == mjCONE_PYRAMIDAL`、`opt.impratio == 1.0`。
  - **確認ポイント**: include 先(`assets/so101/so101_new_calib.xml`)に `<option>` を書いていないこと。

- **TC-SC-03 `<option>` は 1 回だけ、include の直後** 🟡
  - **何をテストするか**: XML の文字列に `<option` がちょうど 1 回あり、`<include file="so101/so101_new_calib.xml"/>` より後、`<compiler` より前にあること(要件 2.1)。
  - **入力値**: `PICKPLACE_SCENE_PATH.read_text()`。
  - **期待される結果**: `text.count("<option") == 1`、`text.index("<include") < text.index("<option") < text.index("<compiler")`。
  - **確認ポイント**: コメントの中の文字 `<option` を数えないよう、コメントには `<option` と書かない(実装への注意)。🔴 コメントの書き方に依存するため、壊れやすければ TC-SC-01 だけにしてよい。

---

## 2. `tests/eval/test_eval_config.py`(変更)

### 2.0 `VALID` の変更 🔵

```python
VALID = {
    "n_trials": 20,
    "base_seed": 0,
    "placement": {"n_blocks": 3, "area": 0.20, "min_gap": 0.05, "max_attempts": 10000},
    "judge": {"margin_m": 0.0, "speed_max_mps": 0.01, "settle_s": 1.0},
    "failure": {"lift_min_m": 0.03, "carry_separation_m": 0.02, "other_move_m": 0.005, "tilt_max_deg": 20.0},
}
```

- 加えないと既存の TC-C-04 / TC-C-B01(dict を直接書くもの)も含めて「必須キーがありません」で失敗する。TC-C-04 と TC-C-B01 の dict にも `judge` / `failure` を加える(既存の検証内容は変えない)。
- `configs/eval.yaml` のコメント「判定しきい値は TASK-0015 で追加する」は要件 4.2 のコメントに置き換える。

### 2.1 正常系

- **TC-C-01(変更) リポジトリの値** 🔵: 既存の確認に加え、`cfg.judge == JudgeConfig(0.0, 0.01, 1.0)`、`cfg.failure == FailureConfig(0.03, 0.02, 0.005, 20.0)`、`isinstance(cfg.judge, JudgeConfig)`、`isinstance(cfg.failure, FailureConfig)`。
- **TC-C-03(変更) frozen** 🔵: `cfg.judge.margin_m = 0.1` と `cfg.failure.tilt_max_deg = 1.0` で `FrozenInstanceError`。
- **TC-C-05(変更) int を float に正規化** 🔴: `judge.settle_s: 1`、`failure.tilt_max_deg: 20` を書くと `type(...) is float`、値 1.0 / 20.0。

### 2.2 異常系

- **TC-C-E02(変更) 未知のキー** 🔵: パラメータに `(_with(["judge", "margin"], 0.0), "margin")`、`(_with(["failure", "tilt_deg"], 20), "tilt_deg")` を加える → `ValueError`、キー名入り。
- **TC-C-E03(変更) 必須キーの欠落** 🔵: パラメータに `["judge"]`、`["failure"]`、`["judge", "margin_m"]`、`["judge", "speed_max_mps"]`、`["judge", "settle_s"]`、`["failure", "lift_min_m"]`、`["failure", "carry_separation_m"]`、`["failure", "other_move_m"]`、`["failure", "tilt_max_deg"]` を加える → `ValueError`、欠けたキー名入り。
- **TC-C-E04(変更) 型・値域** 🟡: パラメータに次を加える → `ValueError`、キー名入り。

| path | 値 | 理由 |
|---|---|---|
| `judge.margin_m` | −0.001 | 0 以上 |
| `judge.margin_m` | 0.04 | `box_inner_half_size` 未満(内側が空になる) |
| `judge.margin_m` | "0" | 数値でない |
| `judge.speed_max_mps` | 0 | 0 より大きい |
| `judge.speed_max_mps` | True | bool は不可 |
| `judge.settle_s` | −0.1 | 0 以上 |
| `judge.settle_s` | nan | 有限 |
| `failure.lift_min_m` | 0 | 0 より大きい |
| `failure.carry_separation_m` | −0.02 | 0 より大きい |
| `failure.other_move_m` | inf | 有限 |
| `failure.tilt_max_deg` | 0 | (0, 90) |
| `failure.tilt_max_deg` | 90 | (0, 90) |

- **TC-C-E05(変更) mapping でない** 🔴: `judge: [1, 2]`、`failure:`(null)→ `ValueError`(`AttributeError` / `TypeError` を漏らさない)。

### 2.3 境界値

- **TC-C-B01(変更) 下限ちょうど** 🟡: `judge.margin_m = 0`、`judge.settle_s = 0` は受け入れる。
- **TC-C-B02(新規) 上限の直前** 🟡: `judge.margin_m = 0.0399`、`failure.tilt_max_deg = 89.9` は受け入れる(0.04、90 は TC-C-E04 で拒否)。

---

## 3. `tests/policy/test_executor.py`(新規)

対象: `src/so101_vlm/policy/executor.py` の `run_trajectory`、`ExecutionResult`、区間 → 経由点の定数(名前は実装で決める。本書では `SEGMENT_WAYPOINT` と書く)、`START_TOLERANCE_RAD`。
前半はフェイク Robot(0.6 節、MuJoCo なし)、後半は SimRobot。

### 3.1 正常系(フェイク Robot)

- **TC-EX-01 基本: 1 行 1 周期で `q[1:]` を送り、待ちを足す** 🔵
  - **入力値**: `q0 = 0`、行 k = 1..5 は shoulder_pan だけ 0.01·k(1 周期の上限 0.02 rad 未満)、ラベル `("start",) + ("pre_grasp",) * 5`、`settle_s=1.0`、`metadata` なし。
  - **期待される結果**: `len(robot.sent) == 55`。`sent[0:5]` は各行と完全一致、`sent[5:55]` は全て 5 行目と一致。`n_commands == 5`、`settle_steps == 50`、`duration_s == pytest.approx(55 * 0.02)`、`flag_counts == {"clipped": 0, "rate_limited": 0, "rejected": 0, "stopped": 0}`、`has_safety_flags is False`、`aborted is False`、`abort_reason is None`、`abort_label is None`、`workspace_checked is False`、`final_command` = 5 行目、`final_joint_error == 0.0`、`final_state.timestamp == pytest.approx(1.1)`、`states is None`。
  - **確認ポイント**: 先頭 `q[0]` は送らない。

- **TC-EX-02 送った指令は `apply_safety` の出力と一致する** 🔵
  - **入力値**: TC-EX-03 の軌道。期待値はテスト内で `apply_safety(prev, row, ...)` を順に呼んで作る(`prev` は前回の出力、最初は `q[0]`)。
  - **期待される結果**: `sent[:n_commands]` が期待値の列と `np.array_equal`。
  - **確認ポイント**: `q_current` が「前回の指令」であること(実測の状態ではない)。NFR-02。

- **TC-EX-03 速度上限を超える行は `rate_limited` を数えて続ける** 🔵
  - **入力値**: `q0 = 0`、行 3 つとも shoulder_pan = 0.05、ラベル `pre_grasp`。
  - **期待される結果**: `sent[0..2][0] == [0.02, 0.04, 0.05]`(`pytest.approx(abs=1e-12)`)、`flag_counts["rate_limited"] == 2`、他は 0、`has_safety_flags is True`、`aborted is False`、`n_commands == 3`、`final_command[0] == approx(0.05)`。

- **TC-EX-04 可動域外の行は `clipped` を数えて続ける** 🔵
  - **入力値**: `q0 = 0.99`(全関節)、行 2 つとも 1.01、`lower/upper = ∓1`。
  - **期待される結果**: 送った指令は全関節 1.0、`flag_counts["clipped"] == 2`、`rate_limited == 0`、`aborted is False`。

- **TC-EX-05 作業範囲外の目標で `rejected` → 打ち切り** 🟡
  - **入力値**: ラベル `start, pre_grasp×2, grasp, lift×3, pre_place×2`(各行は前の行 + 0.01 rad)。`metadata["waypoints"]` の `lift` の target だけ (0.5, 0.0, 0.05)、他は (0.25, 0.0, 0.05)。`settle_s=1.0`。
  - **期待される結果**: `aborted is True`、`abort_reason == "target_out_of_workspace"`、`abort_label == "lift"`、`flag_counts["rejected"] == 1`、`n_commands == 4`(打ち切った周期の保持指令を含む)、4 回目の指令 == 3 回目の指令(保持)、待ち 50 周期は保持指令、`len(sent) == 54`、`workspace_checked is True`。`pre_place` の行は送らない(`on_step` に `pre_place` が来ない)。
  - **確認ポイント**: 🔴 `n_commands` が「打ち切った周期を含む」こと、待ちの周期は `target_base=None` で `rejected` が増えないことは本書の決定(要件 3.2 の 4 と 5 から推測)。

- **TC-EX-06 状態の時刻が途絶えたら `stopped` → 打ち切り** 🟡
  - **入力値**: `FakeRobot(nan_time_calls=(3,))`(3 回目の `get_state` だけ時刻 NaN)、TC-EX-01 の軌道。
  - **期待される結果**: `aborted is True`、`abort_reason == "stale_state"`、`abort_label == "pre_grasp"`、`flag_counts["stopped"] == 1`、`n_commands == 3`、3 回目の指令 == 2 回目の指令、その後は待ち 50 周期だけ。
  - **確認ポイント**: executor は `last_state_t = now = state.timestamp` を渡すため、`stopped` は時刻が非有限のときにだけ起きる(`check_state_fresh` の仕様)。monkeypatch は使わない。

- **TC-EX-07 `record=True` で毎周期の状態を残す** 🟡
  - **期待される結果**: TC-EX-01 の軌道で `len(states) == 55`、全要素が `RobotState`、`type(states) is tuple`、`states[-1].q` == `final_state.q`、`states[k].q` == `sent[k]`(フェイクは指令どおりに動く)。`record=False`(既定)は `states is None`。

- **TC-EX-08 `on_step` の呼ばれ方** 🔴
  - **期待される結果**: TC-EX-01 の軌道で呼び出しの列 == `[(1, "pre_grasp"), ..., (5, "pre_grasp"), (6, "settle"), ..., (55, "settle")]`。周期の番号は 1 始まりの通し番号(軌道の部分は `trajectory.labels[i]` と一致)。呼ばれた時点で `len(robot.sent) == i`(送信の後に呼ぶ)。

- **TC-EX-09 区間ごとの `target_base`** 🔵
  - **入力値**: 10 区間全てを含む軌道(各区間 1 行、各行 +0.001 rad)と 6 つの経由点(target は全て作業範囲の内側で互いに違う値)。`executor_module.apply_safety` を「本物を呼び、`target_base` を記録する」包みに monkeypatch する。
  - **期待される結果**: 区間 `close` / `close_hold` の `target_base` == `grasp` の target、`open` / `open_hold` == `place` の target、他は同名の経由点。待ちの周期は `None`。`workspace_checked is True`。

- **TC-EX-10 区間の対応表が `SEGMENT_TABLE` と一致する** 🔵
  - **期待される結果**: `set(SEGMENT_WAYPOINT) == {name for name, _, _ in ik_policy.SEGMENT_TABLE}`、全ての値が `ik_policy.WAYPOINT_ORDER` に含まれる、表の中身 == 要件 3.2 の表(10 行)。

- **TC-EX-11 フェイク Robot は `Robot` を満たし、executor は `close` を呼ばない** 🟡
  - **期待される結果**: `isinstance(FakeRobot(), Robot)`、実行後 `robot.closed is False`(Robot の寿命は呼び出し側が持つ)。

### 3.2 正常系(性質)

- **TC-EX-12 入力を変更しない / 結果は変更できない** 🔵
  - **期待される結果**: 実行前後で `lower` / `upper` のコピーと `np.array_equal`、`trajectory.q` は書き込み不可のまま値も同じ。`result.n_commands = 1` で `FrozenInstanceError`、`result.flag_counts["clipped"] = 9` で `TypeError`、`result.final_command[0] = 9` で `ValueError`(読み取り専用)。
- **TC-EX-13 決定性** 🔵: 新しいフェイク 2 台に同じ軌道 → `sent` が全て一致、結果のフィールドが `wall_time_s` 以外全て一致。
- **TC-EX-14 `wall_time_s` は記録のみ** 🟡: `wall_time_s >= 0` で有限。`duration_s` は周期数から決まり、`wall_time_s` と無関係(TC-EX-01 の値で確認済み)。

### 3.3 異常系

- **TC-EX-E01 開始姿勢の不一致 → `ValueError`** 🔵: Robot の q = 0、軌道 `q[0]` の elbow_flex = 0.05(および −0.05)→ `ValueError`(メッセージに「開始」を含む)。`robot.sent` は空。
- **TC-EX-E02 未知の区間名 → `ValueError`** 🟡: ラベル `("start", "dance")` → `ValueError`(メッセージに `dance`)。何も送らない(送る前に全ラベルを確かめる)。
- **TC-EX-E03 `settle_s` の不正 → `ValueError`** 🟡: −0.01、`nan`、`inf`、`True` → `ValueError`。何も送らない。
- **TC-EX-E04 可動域の shape 違い → `ValueError`** 🟡: `lower = -np.ones(5)` → `ValueError`(`apply_safety` の検証)。何も送らない(最初の周期の送信前に失敗する)。
- **TC-EX-E05 `on_step` の例外はそのまま伝わる** 🔴: 2 回目の呼び出しで `RuntimeError("boom")` → `pytest.raises(RuntimeError, match="boom")`、`len(robot.sent) == 2`(その後は送らない)。
- **TC-EX-E06 軌道の型違い → 例外** 🟡: `trajectory` に `np.ndarray` を渡す → `TypeError` または `ValueError`(`pytest.raises((TypeError, ValueError))`)。

### 3.4 境界値

- **TC-EX-B01 開始姿勢の差がちょうど許容** 🔵: 差 0.0499 rad は実行できる、0.05 rad は `ValueError`(指示どおり「0.05 以上で拒否」)。
- **TC-EX-B02 待ちの周期数は `ceil`** 🔵: `settle_s` 0 → 0 周期(`len(sent) == n_commands`)、0.01 → 1(`round` なら 0 になる値)、0.02 → 1、0.03 → 2、0.5 → 25、1.0 → 50。`duration_s == (n_commands + settle_steps) * 0.02`。
- **TC-EX-B03 最短の軌道(2 行)** 🟡: `q` 2 行 → `n_commands == 1`、`len(sent) == 1 + settle_steps`。
- **TC-EX-B04 `waypoints` なしの軌道は作業範囲を見ない** 🟡: `metadata={}` で、本来なら範囲外の動きでも `rejected == 0`、`workspace_checked is False`。

### 3.5 統合(SimRobot、`PICKPLACE_SCENE_PATH`)

module scope のフィクスチャ `seed0_plan`: `SimRobot` → `mujoco_handles()` → `place_blocks(model, data, sample_layout(0))` → 赤ブロックの `xpos` で `IkPolicy.act`(約 0.4 s)。

- **TC-EXI-01 IkPolicy の軌道を実行してフラグ 0** 🔵: `n_commands == len(trajectory.q) - 1`(実測 273)、`settle_steps == 50`、`has_safety_flags is False`、`aborted is False`、`workspace_checked is True`、`final_joint_error < 0.01`(実測 0.00044)、`duration_s == approx(6.46, abs=0.02)`。
- **TC-EXI-02 手書きの速度超過の軌道** 🔵: 新しい `SimRobot`、`q` 12 行(先頭 0、残り 11 行は shoulder_pan 0.3)、ラベル `pre_grasp`、`metadata` なし → `flag_counts["rate_limited"] == 11`、`final_command[0] == approx(0.22, abs=1e-9)`、`final_joint_error < 0.01`(実測 0.00054)、`final_state.timestamp == approx(1.22, abs=1e-9)`。
- **TC-EXI-03 実機と同じ API で SimRobot を使える** 🔵: 上の 2 件で `run_trajectory` が `SimRobot` を `Robot` としてだけ使う(`mujoco_handles` を呼ばない)ことを、`SimRobot` の `mujoco_handles` を「呼ばれたら失敗」に monkeypatch して確かめる。

---

## 4. `tests/eval/test_judge.py`(新規)

対象: `judge_in_box(centre, linear_velocity, config, spec=SCENE_SPEC) -> JudgeResult`、`JudgeReason`。
既定の `CFG = JudgeConfig(margin_m=0.0, speed_max_mps=0.01, settle_s=1.0)`。箱中心 (0.20, 0.19)、内寸半幅 0.04、縁 0.04。
**注意(実測)**: 0.20 + 0.04 = 0.24000000000000002、|0.24 − 0.20| = 0.03999999999999998 のように、10 進の値ではちょうどの境界を作れない。境界のテストは 2 進で正確な値の `SPEC_EXACT = dataclasses.replace(SCENE_SPEC, box_center_xy=(0.25, 0.125), box_inner_half_size=0.0625, box_rim_z=0.0625)` を使う。

### 4.1 正常系

- **TC-JG-01 箱の中で静止 → 成功** 🔵(TASK-0015 テストケース 1): centre (0.198, 0.186, 0.0199)(実測の seed 0)、速度 0 → `success is True`、`reasons == ()`、`offset_xy == approx((-0.002, -0.004))`、`height == approx(0.0199)`、`speed == 0.0`。
- **TC-JG-02 内寸の外 → `OUTSIDE_INNER`** 🔵(ケース 2): 各辺の外 4 通り (0.25, 0.19)、(0.15, 0.19)、(0.20, 0.24 + 0.01)、(0.20, 0.13)、z 0.02、速度 0 → `reasons == (JudgeReason.OUTSIDE_INNER,)`。
- **TC-JG-03 縁以上 → `ABOVE_RIM`** 🔵(ケース 3): (0.20, 0.19, 0.055)(縁に乗ったブロックの中心 = 0.04 + 0.015)→ `reasons == (ABOVE_RIM,)`。
- **TC-JG-04 動いている → `MOVING`** 🔵(ケース 4): 箱の中、速度 (0.0, 0.0, −0.05) → `reasons == (MOVING,)`、`speed == approx(0.05)`。
- **TC-JG-05 複数の理由は (1)(2)(3) の順に全部** 🟡: (0.30, 0.19, 0.10)、速度 (0.1, 0, 0) → `reasons == (OUTSIDE_INNER, ABOVE_RIM, MOVING)`。`JudgeReason` の値は `"outside_inner"`、`"above_rim"`、`"moving"`(`str` の Enum)。

### 4.2 異常系

- **TC-JG-E01 NaN / inf / shape 違い / bool → `ValueError`** 🟡: centre `(nan, 0.19, 0.02)`、`(0.2, 0.19)`(2 要素)、`(inf, 0, 0)`、`True`;velocity `(0, 0, nan)`、`(0, 0)` → `ValueError`。
- **TC-JG-E02 config の型違い → `ValueError`** 🔴: `config=None`、`config={"margin_m": 0}` → `ValueError`(または `TypeError`。実装で `ValueError` に統一する)。

### 4.3 境界値(`SPEC_EXACT`)

- **TC-JG-B01 ちょうど内寸の半幅は失敗(厳密な不等号)** 🟡: centre (0.3125, 0.125, 0.03125)(dx = 0.0625)→ `OUTSIDE_INNER`。`np.nextafter(0.3125, 0)` は成功。y 側も同じ。
- **TC-JG-B02 ちょうど縁は失敗** 🟡: z = 0.0625 → `ABOVE_RIM`。`np.nextafter(0.0625, 0)` は成功。
- **TC-JG-B03 ちょうど `speed_max` は成功(「以下」)** 🟡: 速度 (0.01, 0, 0)、`speed_max_mps=0.01` → 成功。`np.nextafter(0.01, 1)` は `MOVING`。
- **TC-JG-B04 margin の効果** 🟡: 既定の SPEC で centre (0.237, 0.19, 0.02)(dx ≈ 0.037)→ margin 0 は成功、margin 0.005 は `OUTSIDE_INNER`。

### 4.4 性質

- **TC-JG-06 純粋・不変**(正常系に数える)🔵: 入力の `np.ndarray` は呼び出し後も同じ値、`JudgeResult` は frozen(`FrozenInstanceError`)、`reasons` は tuple、同じ入力で同じ結果。

---

## 5. `tests/eval/test_failure.py`(新規)

対象: `FailureKind`、`PickTrace`、`classify_failure(trace, judge, config)`。
`CFG = FailureConfig(lift_min_m=0.03, carry_separation_m=0.02, other_move_m=0.005, tilt_max_deg=20.0)`。
`NOMINAL = PickTrace(lift_end_rise=0.0736, max_separation_carry=0.0158, max_separation_place=0.0160, max_other_move=2e-10, final_tilt_deg=0.0)`(実測の赤 seed 0)。
`FAILED_JUDGE = JudgeResult(False, (JudgeReason.OUTSIDE_INNER,), (0.1, 0.0), 0.015, 0.0)`、`OK_JUDGE = JudgeResult(True, (), (-0.002, -0.004), 0.0199, 0.0)`。合成のトレースは `dataclasses.replace(NOMINAL, ...)` で作る。

### 5.1 正常系(1 種類 1 件)

| ID | トレース | 判定 | 期待 | 信頼性 |
|---|---|---|---|---|
| TC-FL-01 | NOMINAL | OK_JUDGE | `None` | 🔵 |
| TC-FL-02 | `max_other_move=0.01`、`final_tilt_deg=45` | OK_JUDGE | `None`(成功は分類しない。O5) | 🟡 |
| TC-FL-03 | `lift_end_rise=0.0`(持ち上がらない。実測の既定物理の例) | FAILED | `GRASP_SLIP` | 🔵 |
| TC-FL-04 | `max_separation_carry=0.05`(運搬中に抜けた) | FAILED | `GRASP_SLIP` | 🔵 |
| TC-FL-05 | `max_separation_place=0.05` | FAILED | `DROP` | 🟡 |
| TC-FL-06 | `max_other_move=0.006` | FAILED | `COLLISION_OTHER` | 🟡 |
| TC-FL-07 | `final_tilt_deg=45.0` | FAILED | `KNOCKED_OVER` | 🟡 |
| TC-FL-08 | NOMINAL | FAILED | `NOT_IN_BOX` | 🟡 |

- `PLANNING_FAILURE` は `classify_failure` からは返らない(計画失敗は `run_fixed_pick` が付ける。TC-FP-05 で確認)。列挙の確認: `{k.value for k in FailureKind} == {"planning_failure", "grasp_slip", "drop", "collision_other", "knocked_over", "not_in_box"}`、`FailureKind` は `str` の Enum(TC-FL-08 に含める)。🔵

### 5.2 優先順位(正常系)

| ID | トレース | 期待 | 信頼性 |
|---|---|---|---|
| TC-FL-09 | slip + drop + other + tilt(`lift_end_rise=0`、place 0.05、other 0.01、tilt 45) | `GRASP_SLIP` | 🟡 |
| TC-FL-10 | drop + other + tilt | `DROP` | 🟡 |
| TC-FL-11 | other + tilt | `COLLISION_OTHER` | 🟡 |

- 実装では 5.1 と 5.2 を `parametrize` 1 本にまとめてよい。

### 5.3 異常系

- **TC-FL-E01 トレースの値の不正 → `ValueError`** 🔴: `PickTrace(lift_end_rise=nan, ...)`、`max_separation_carry=-0.01`(距離は 0 以上)、`final_tilt_deg=inf` → 生成時に `ValueError`。`lift_end_rise` は負を許す(床にめり込む・落ちる場合がある)。

### 5.4 境界値(しきい値ちょうどは「当てはまらない」)

- **TC-FL-B01** 🟡: `lift_end_rise=0.03` は slip にならない(厳密な `<`)、`max_separation_carry=0.02` / `max_separation_place=0.02` / `max_other_move=0.005` / `final_tilt_deg=20.0` はそれぞれ当てはまらない → 全て `NOT_IN_BOX`(判定は FAILED)。
- **TC-FL-B02** 🟡: 各しきい値の `np.nextafter` で外側の値 → それぞれの種類になる。

---

## 6. `tests/robot/test_sim.py`(追加): `SimRobot.mujoco_handles()`

既存の小モデル `tiny_scene_path` / `tiny_robot` を使う(読み込み 約 2 ms)。

- **TC-MH-01 自分の model / data を返す** 🔴: `model, data = robot.mujoco_handles()` → `isinstance(model, mujoco.MjModel)`、`isinstance(data, mujoco.MjData)`。2 回呼ぶと同じオブジェクト(`is`)。
- **TC-MH-02 ロボットの状態と同じ data** 🔴: `send_command` を 3 回 → `data.time == robot.get_state().timestamp`(0.06 s)。外から `data.qpos` を書くと次の `get_state().q` に表れる(同じ実体であることの確認。ブロックの配置に使うため)。
- **TC-MH-03 別のロボットとは共有しない** 🔴: 2 台の `SimRobot` で `data1 is not data2`、片方の `send_command` で他方の `data.time` は 0 のまま。
- **TC-MH-04 `Robot` Protocol に入れない** 🔵(要件 6.2): `not hasattr(Robot, "mujoco_handles")`。`mujoco_handles` を持たないフェイク(`joint_names` / `get_state` / `send_command` / `close` のみ)も `isinstance(fake, Robot)`。
- **TC-MH-E01 閉じた後は `RuntimeError`** 🔵: `close()` の後 `mujoco_handles()` → `RuntimeError`(既存の `"SimRobot is closed"` と同じメッセージ)。

---

## 7. `tests/eval/test_fixed_pick.py`(新規、統合)

対象: `run_fixed_pick`、`FixedPickOutcome`。シーンは `PICKPLACE_SCENE_PATH`(物理設定あり)。
module scope のフィクスチャ:
- `red_outcomes`: `{seed: run_fixed_pick(seed, "red", frame_sink=記録) for seed in (0, 1, 2)}`(約 1.5 s)。`frame_sink` の呼び出し `(model, data, i, label)` の `i` と `label` を記録する。

### 7.1 正常系

- **TC-FP-01 赤 seed 0〜2 で成功、フラグ 0、関節誤差 < 0.01** 🔵(要件 8.1 / 8.3、TASK-0013 統合テスト 1)
  - **期待される結果(各 seed)**: `planning_error is None`、`planning_waypoint is None`、`success is True`、`failure is None`、`judge.reasons == ()`、`execution.has_safety_flags is False`、`execution.aborted is False`、`execution.final_joint_error < 0.01`、`other_block_moved is False`、`colour == "red"`、`seed` が一致。
- **TC-FP-02 実行の数値** 🔵: `execution.settle_steps == 50`、`execution.duration_s == approx((n_commands + 50) * 0.02)`、`240 <= n_commands <= 330`(実測 273 / 258 / 296、TASK-0012 の 250〜325 サンプル)、`plan_time_s > 0`。
- **TC-FP-03 追跡の要約(PickTrace)** 🔵: `trace.lift_end_rise > 0.03`(実測 0.074〜0.076)、`trace.max_separation_carry < 0.02`(実測 ≤ 0.0160)、`trace.max_separation_place < 0.02`(≤ 0.0162)、`trace.max_other_move < 0.005`(実測 2e-10)、`trace.final_tilt_deg < 1.0`。
- **TC-FP-04 判定の値と真の位置** 🔵: `pick_target[:2] == approx(sample_layout(seed, 3, 0.20, 0.05, 10000)[0], abs=1e-9)`(配置は既存の `sample_layout` / `place_blocks` のまま。乱数配置の経路を変えていない)、`pick_target[2] == approx(0.015, abs=0.002)`、`max(abs(judge.offset_xy)) < 0.016`(実測の最大 15.2 mm)、`judge.height == approx(0.0199, abs=0.001)`、`judge.speed <= 0.01`。
- **TC-FP-05 計画失敗は実行しない** 🔵(FR-06、要件 6.1 の 3)
  - **入力値**: `(16, "blue", "self_collision", "grasp")`、`(5, "green", "not_converged", "pre_grasp")` の parametrize。`fixed_pick_module.run_trajectory` を「呼ばれたら `AssertionError`」に monkeypatch。
  - **期待される結果**: `planning_error == reason`、`planning_waypoint == waypoint`、`execution is None`、`judge is None`、`trace is None`、`failure == FailureKind.PLANNING_FAILURE`、`success is False`、`other_block_moved is False`、`plan_time_s > 0`、`pick_target[0] < 0.17`(実測 0.1688 / 0.1608)、`frame_sink` は呼ばれない。
- **TC-FP-06 計画と実行の結果を分けて持つ** 🟡: 成功の結果では計画側(`planning_error`、`planning_waypoint`)が None で実行側(`execution`、`judge`、`trace`)が全て値を持つ。計画失敗の結果ではその逆(TC-FP-05)。`ExecutionResult` に計画の情報のフィールドがない(`not hasattr(execution, "planning_error")`、`not hasattr(execution, "plan_time_s")`)。
- **TC-FP-07 `frame_sink` は毎周期呼ばれる** 🟡: seed 0 で呼び出し数 == `n_commands + settle_steps`(実測 323)、`i` は 1..323 の通し番号、最後の 50 回の label は `"settle"`、引数の model / data は `mujoco.MjModel` / `mujoco.MjData`。
- **TC-FP-08 決定性** 🔵: `run_fixed_pick(0, "red")` を 2 回 → `judge` が等しい、`trace` が等しい、`execution.n_commands` が等しい、`np.array_equal(execution.final_command)`、`execution.final_joint_error` が等しい(実測でビット単位一致)。`plan_time_s` と `wall_time_s` は比べない。
- **TC-FP-09 設定の注入で判定が変わる(分類の経路)** 🟡: `eval_config = dataclasses.replace(load_eval_config(), judge=JudgeConfig(margin_m=0.039, speed_max_mps=0.01, settle_s=1.0))` で seed 0 赤 → 内側の限界 1 mm に対し実測のずれ (−2.3, −4.3) mm → `success is False`、`judge.reasons == (OUTSIDE_INNER,)`、`failure == FailureKind.NOT_IN_BOX`(トレースは正常値)、`execution.has_safety_flags is False`。

- TC-FP-05 は計画失敗の「正常な拒否」(FR-06)なので正常系に数える。

### 7.2 異常系

- **TC-FP-E01 色の不正 → `ValueError`** 🔵: `"purple"`、`"RED"`、`""` → `ValueError`(`SimRobot` を作る前に失敗する。`fixed_pick_module.SimRobot` を「呼ばれたら失敗」に monkeypatch して確認)。
- **TC-FP-E02 seed の不正 → `ValueError`** 🔵: `-1`、`True`、`1.5`、`"0"` → `ValueError`(`require_int`)。

### 7.3 不変性(正常系に含める)

- **TC-FP-10 結果は変更できない** 🔵: `outcome.seed = 1`、`outcome.trace.lift_end_rise = 0`、`outcome.execution.n_commands = 0` で `FrozenInstanceError`、`outcome.execution.final_command.flags.writeable is False`、`pick_target` は tuple。

---

## 8. `tests/test_run_fixed_pick_script.py`(新規)

`scripts/run_fixed_pick.py` を `importlib.util.spec_from_file_location` で読み込み、`main(argv: list[str]) -> int` を直接呼ぶ(既存の `tests/perception/test_generate_calibration.py` と同じ方式)。出力先は必ず `tmp_path` の `--out-dir` を渡す(`media/` を汚さない)。
動画のテストは 1 件だけ(0.4 節)。他は全て `--no-video`。

### 8.1 正常系

- **TC-CLI-01 1 試行・動画なし** 🟡
  - **入力値**: `["--seed", "0", "--colour", "red", "--no-video", "--out-dir", tmp]`
  - **期待される結果**: 戻り値 0。標準出力(`capsys`)の試行の行は 1 行で、`seed=0`、`red`、`成功` を含む。最後の集計行に `成功 1/1` と `計画失敗 0` を含む。`tmp/fixed_pick_summary.json` があり、要素 1 個で `seed == 0`、`colour == "red"`、`success is True`、`failure is None`、`planning_error is None`、`flag_counts` の値が全て 0。`*.mp4` と `*_carry.png` は作らない。
  - 🔴 表示の文言(`成功` / `計画失敗` / `成功 1/1`)は本書の決定。実装で文言を変えるなら本書を直す。
- **TC-CLI-02 `--n-seeds`** 🟡: `["--seed", "0", "--n-seeds", "2", "--no-video", "--out-dir", tmp]` → 0、試行の行が 2 行(seed 0、1)、JSON の要素 2 個、`成功 2/2`。
- **TC-CLI-03 `--colour all` と計画失敗(終了コード 0)** 🔵: `["--seed", "16", "--colour", "all", "--no-video", "--out-dir", tmp]` → 戻り値 0(計画失敗だけでは 1 にしない)、試行 3 件(赤・青・緑の順)、青の行に `self_collision` を含む、集計に `計画失敗 1` と `成功 2/2`。JSON の青の要素は `planning_error == "self_collision"`、`failure == "planning_failure"`。
  - 🟡 赤 / 緑 seed 16 が計画可能で成功することは survey の 20 seed × 3 色の結果(計画失敗は青 16 と緑 5 のみ)による。
- **TC-CLI-04 `--headless` を受け付ける** 🔴: `["--seed", "0", "--headless", "--no-video", "--out-dir", tmp]` → 0(動作は TC-CLI-01 と同じ)。TC-CLI-01 と同じ試行のため、実装では TC-CLI-01 を `parametrize` で `--headless` の有無の 2 通りにしてよい(0.4 s 増)。
- **TC-CLI-05 実行の失敗は終了コード 1** 🟡: `script.run_fixed_pick` を「実際の seed 0 の結果を `dataclasses.replace(..., judge=失敗の JudgeResult, failure=FailureKind.NOT_IN_BOX)` にしたもの」を返す関数に monkeypatch → `main([... "--no-video" ...]) == 1`、行に `not_in_box` を含む。
- **TC-CLI-06 `--out-dir` のフォルダを作る** 🟡: `tmp/a/b`(存在しない)→ 0、`tmp/a/b/fixed_pick_summary.json` ができる。`--out-dir` の外には書かない(実行の前後で `tmp` 直下のファイル一覧が `a` だけ)。
- **TC-CLI-07 動画の既定のファイル名** 🟡: 純粋な補助関数(例 `default_video_path(out_dir, colour, seed)`)が `out_dir / "fixed_pick_red_seed0.mp4"` を返し、運搬中の PNG の名前が `fixed_pick_red_seed0_carry.png`。描画しない(動画のテストを 1 件に抑えるため、名前の規則はここで確かめる)。
- **TC-CLI-08 動画を保存できる(1 件のみ、約 10 s)** 🔵(TASK-0013 完了条件 2、要件 8.1 の 2)
  - **入力値**: `["--seed", "0", "--colour", "red", "--headless", "--video", str(tmp / "v" / "red0.mp4"), "--out-dir", str(tmp / "out")]`(module scope のフィクスチャで 1 回だけ実行)。
  - **期待される結果**: 戻り値 0。`red0.mp4` があり `size > 0`(実測 265 KB)。`imageio.v2.get_reader` で開け、1 フレーム目の shape == (480, 640, 3)、`count_frames()` が 140〜200(実測 161 = floor(323 / 2))、`get_meta_data()["fps"] == 25`。`tmp/out/fixed_pick_red_seed0_carry.png` があり、読むと (480, 640, 3)。`tmp/out/fixed_pick_summary.json` の 1 件が成功。
  - **確認ポイント**: 運搬中の PNG の目視(ブロックが顎に挟まれている)はテストではなく `tdd-verify-complete` で開発者が行う。

### 8.2 異常系

- **TC-CLI-E01 引数の誤り → 終了コード 2** 🔵: 次の各 argv で `pytest.raises(SystemExit)` かつ `code == 2`(argparse)。

| argv | 理由 |
|---|---|
| `--colour purple` | 選択肢外 |
| `--seed -1` | 0 以上 |
| `--seed abc` | 整数でない |
| `--n-seeds 0` | 1 以上 |
| `--video x.mp4 --no-video` | 同時に指定できない 🟡 |
| `--video x.mp4 --n-seeds 2` | 動画は 1 試行のときだけ 🟡 |

- **TC-CLI-E02 `--help` はサブプロセスで動く** 🟡: `subprocess.run([sys.executable, SCRIPT_PATH, "--help"])` の戻り値 0、出力に `--seed` と `--out-dir` を含む(スクリプトとして実行できることの確認。約 1 s)。

---

## 9. 手動の確認: 20 seed の掃引(テストにしない)🔵

- **MANUAL-01**: `uv run python scripts/run_fixed_pick.py --seed 0 --n-seeds 20 --colour all --no-video --out-dir media/fixed_pick_sweep`
  - **期待される結果**: 60 試行、計画失敗 2(青 seed 16: `self_collision` / grasp、緑 seed 5: `not_converged` / pre_grasp)、計画可能な 58 件で **58/58 成功**、安全フィルタのフラグ 0、終了コード 0。所要 約 20 s。
  - 結果(集計の行と JSON)を `tdd-verify-complete` の記録に残す。TASK-0014 の「10 回中 8 回以上」の確認にも使う。
  - 掃引をテストにしない理由: 約 20 s かかり、実測で 100 seed × 3 色まで確認済み。

---

## 10. 開発言語・フレームワーク

- **プログラミング言語**: Python 3.12 🔵
  - 既存コード・MuJoCo の Python バインディングと同じ。frozen dataclass と読み取り専用の numpy 配列で不変性を表せる。
- **テストフレームワーク**: pytest(`uv run pytest`、`testpaths = ["tests"]`)、カバレッジは pytest-cov 🔵
  - module scope のフィクスチャで sim の実行を共有する(既存の `tests/policy/test_ik_policy.py` と同じ)。`monkeypatch` でモジュールの大域名(`apply_safety`、`run_trajectory`、`SimRobot`、`run_fixed_pick`)を差し替える。
  - スクリプトは `importlib` で読み込んで `main(argv)` を呼ぶ(既存の方式)。
  - マーカーは使わない(未登録。0.4 節)。
- **実行環境**: macOS、MuJoCo 3.16.0、オフスクリーン描画(`mujoco.Renderer`)、`imageio` + `imageio-ffmpeg`(libx264)。🔵

---

## 11. テスト実装時の日本語コメント指針

既存のテスト(`tests/policy/test_ik_policy.py`、`tests/eval/test_eval_config.py`)に合わせる。

- モジュールの docstring: 対象モジュール、本書へのパス(`docs/implements/so101-vlm-pickplace/TASK-0013/pick-testcases.md` の節番号)、実行時間の予算、信頼性の凡例。
- 各テスト関数の docstring: `"""TC-EX-03 速度上限を超える行は rate_limited を数えて続ける 🔵"""` の形(TC の ID、日本語の名前、信頼性)。
- 本文は Given / When / Then の順にコメントを付ける。

```python
def test_rate_limited_rows_are_counted_and_execution_continues():
    """TC-EX-03 速度上限を超える行は rate_limited を数えて続ける 🔵"""
    # 【テストデータ準備】: 1 周期の上限 0.02 rad を超える 0.05 rad の目標を 3 行続ける
    robot = FakeRobot()
    traj = make_traj(rows_with_pan([0.0, 0.05, 0.05, 0.05]), ("start",) + ("pre_grasp",) * 3)
    # 【実際の処理実行】: 実行器で送る (待ちなし)
    result = run_trajectory(robot, traj, SAFETY, lower=UNIT_LOWER, upper=UNIT_UPPER, settle_s=0.0)
    # 【結果検証】: 指令は 0.02 ずつ近づき、超えた 2 周期だけ数える
    assert [c[0] for c in robot.sent] == pytest.approx([0.02, 0.04, 0.05], abs=1e-12)  # 【確認内容】: 速度制限の結果をそのまま送る
    assert result.flag_counts["rate_limited"] == 2  # 【確認内容】: フラグを捨てない (NFR-02)
    assert not result.aborted  # 【確認内容】: rate_limited では打ち切らない
```

- フィクスチャ: `# 【テスト前準備】`(何を一度だけ作るか、所要時間)、`# 【環境初期化】`(新しい SimRobot を使う理由)。

---

## 12. 要件定義との対応関係

| 要件の節 | テスト |
|---|---|
| 2 物理設定 | TC-SC-01〜03、既存のシーンのテスト全件 |
| 3.1〜3.2 実行器のループ | TC-EX-01〜11、TC-EX-E01〜E06、TC-EX-B01〜B04、TC-EXI-01〜03 |
| 3.3 `ExecutionResult` | TC-EX-01、07、12〜14 |
| 4.1 判定 | TC-JG-01〜06、TC-JG-E01〜E02、TC-JG-B01〜B04 |
| 4.2 設定 | TC-C-01、03、05、E02〜E05、B01、B02 |
| 5 失敗の分類 | TC-FL-01〜11、TC-FL-E01、TC-FL-B01〜B02、TC-FP-05、TC-FP-09 |
| 6 `run_fixed_pick` | TC-FP-01〜10、TC-FP-E01〜E02 |
| 6.2 `mujoco_handles` | TC-MH-01〜04、TC-MH-E01 |
| 7 CLI・動画 | TC-CLI-01〜08、TC-CLI-E01〜E02 |
| 8.1 完了条件 | TC-FP-01(箱に入れる)、TC-CLI-08(動画) |
| 8.3 掃引 | MANUAL-01 |
| 9 制約(NFR-02、FR-10/12、不変性) | TC-EX-02、TC-EXI-03、TC-EX-12、TC-FP-10 |

- **参照した機能概要**: 要件 1 節、TASK-0013.md の概要と完了条件
- **参照した入力・出力仕様**: 要件 3.1、3.3、4.1、5.2、6、7.1
- **参照した制約条件**: 要件 9 節、note.md 2 節
- **参照した使用例**: 要件 10 節(10.3 のエッジケースは TC-EX-05、TC-EX-E01、TC-FP-05、TC-FP-E01/E02、TC-CLI-E01、TC-CLI-06 に対応)
- **TASK-0015 との関係**: TC-JG-01〜04 は TASK-0015 のテストケース 1〜4、TC-JG-B01〜B03 はテストケース 5(境界値)に当たる。TASK-0015 はこのファイルを拡張する。

---

## 13. 本書で決めたこと・未決事項

| # | 内容 | 決定 | 信頼性 |
|---|---|---|---|
| T1 | 開始姿勢の許容: 要件 3.2 は「0.05 を超えたら」、指示は「0.05 以上で拒否」 | 指示に従い **0.05 以上で `ValueError`**(TC-EX-B01)。要件 3.2 の文言を Green で合わせる | 🔴 |
| T2 | 打ち切ったときの `n_commands` | 打ち切った周期の保持指令を含めて数える(TC-EX-05/06) | 🔴 |
| T3 | 待ちの周期の `target_base` | `None`(打ち切りの後に `rejected` が増えない) | 🔴 |
| T4 | `on_step` の周期の番号 | 1 始まりの通し番号(軌道の部分は `trajectory.labels[i]` と一致) | 🔴 |
| T5 | 動画のテスト | 1 件のみ、640×480 のまま(約 10 s)。縮小の引数は作らない(描画 41 ms/フレームはサイズに依らない) | 🟡 |
| T6 | CLI の表示の文言と JSON のキー | `成功` / `計画失敗` / `成功 a/b`、JSON は `seed`、`colour`、`success`、`failure`、`planning_error`、`flag_counts` を必ず含む | 🔴 |
| T7 | `--video` と `--no-video` の同時指定、`--video` と複数 seed | 引数の誤り(終了コード 2) | 🟡 |
| T8 | `PickTrace` の検証 | 有限、距離と傾きは 0 以上、`lift_end_rise` は負を許す | 🔴 |
| 注 | 運搬中の距離の実測最大 0.0177 m(青 seed 0)は `carry_separation_m` 0.02 に近い | 分類は判定が失敗のときだけ使うため、成功を取り消すことはない。TASK-0015 / 0026 で見直す | 🔵 実測 |

---

## 14. 品質判定

| 観点 | 評価 |
|---|---|
| テストケース分類 | 正常系 63、異常系 18、境界値 12(計 93)。判定・分類・実行器・設定は 3 分類とも揃う。統合と CLI は正常系と異常系(境界値は単体側で確認) |
| 期待値定義 | 数値は全て具体値(実測値と許容幅)。浮動小数の境界は 2 進で正確な値を使う |
| 技術選択 | Python 3.12 + pytest、importlib による CLI 呼び出し、マーカーなし(確定) |
| 実装可能性 | 統合部分は使い捨ての実行器で同じ手順・同じ数値を確認済み。追加の実行時間 < 20 s |
| 信頼性の分布 | 🔵 約 45%、🟡 約 40%、🔴 約 15%(🔴 は T1〜T8 の API の細部と表示の文言) |

**判定: ✅ 高品質**(🔴 は実装上の細部の決定で、要件の範囲を変えない。T1 は指示と要件の文言の差のため、Green で要件側の文言を合わせる)

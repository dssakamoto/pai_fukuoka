# TASK-0013 Green フェーズ記録: 固定位置での pick & place(VLM なし)

**要件名**: so101-vlm-pickplace / **タスクID**: TASK-0013 / **日付**: 2026-10-21
**方針**: Red で書いたテストは 1 件も変更していない。layout の範囲、安全フィルタ、軌道の動作も変更していない。ユーザーへの質問なし、エージェント起動なし、git commit なし。

## 1. 変更したファイル

| ファイル | 内容 | 行数 |
|---|---|---|
| `assets/scene_pickplace.xml` | include の直後に `<option cone="elliptic" impratio="10"/>` と日本語コメント(sim で把持を保つため) | +6 |
| `configs/eval.yaml` | `judge:`(margin_m 0.0 / speed_max_mps 0.01 / settle_s 1.0)、`failure:`(lift_min_m 0.03 / carry_separation_m 0.02 / other_move_m 0.005 / tilt_max_deg 20) | +12 |
| `src/so101_vlm/eval/eval_config.py` | frozen の `JudgeConfig` / `FailureConfig`(`__post_init__` で厳格に検証、int は float にそろえる)、`_TOP_KEYS` に judge / failure、`EvalConfig.judge` / `.failure`。既存のメッセージ・動作は不変 | 約 140 |
| `src/so101_vlm/robot/sim.py` | `SimRobot.mujoco_handles()`(sim 専用、閉じた後は RuntimeError、Robot Protocol には入れない) | +10 |
| `src/so101_vlm/policy/executor.py`(新規) | `run_trajectory`、`ExecutionResult`、`SEGMENT_WAYPOINT`、`START_TOLERANCE_RAD`、内部の `_Stepper` | 220 |
| `src/so101_vlm/eval/judge.py`(新規) | `judge_in_box`、`JudgeResult`、`JudgeReason` | 約 70 |
| `src/so101_vlm/eval/failure.py`(新規) | `FailureKind`(6 種)、`PickTrace`、`classify_failure`(固定の優先順位) | 約 95 |
| `src/so101_vlm/eval/pick_tracker.py`(新規) | `PickTracker`(毎周期の真値の追跡)、`tcp_position`、`tilt_degrees`、`block_linear_velocity` | 約 100 |
| `src/so101_vlm/eval/fixed_pick.py`(新規) | `run_fixed_pick`、`FixedPickOutcome`、`COLOUR_TO_BODY`。`SimRobot` と `run_trajectory` はモジュール大域名から引く | 175 |
| `scripts/run_fixed_pick.py`(新規) | `main(argv)`、`default_video_path`、`default_carry_png_path`、`VideoRecorder`(frame_sink、25 fps、640x480、free camera) | 251 |
| `docs/.../pick-requirements.md` | 3.2 の開始姿勢の許容を「超えたら」から「以上なら」に修正(T1) | 1 語 |

`pick_tracker.py` は要件にない分割で、追跡(MuJoCo の真値を読む部分)を `fixed_pick.py` から切り出した(関数 50 行未満・責務の分離のため)。

## 2. 実装方針と判断

- 🔵 Red 記録の「使い捨て最小実装」(scratchpad の repo_copy)と pick_survey の使い捨て実行器の手順を、クリーンな本番コードとして書き直した(scratchpad からの import はない)。
- 🔵 `run_trajectory`: 入力検査(型・未知の区間名・settle_s)と開始姿勢の確認を何も送る前に行う。`rejected` / `stopped` の周期で打ち切り(その周期の保持指令は送る)、待ちは `target_base=None`。
- 🟡 `FixedPickOutcome.pick_target` は tuple、`ExecutionResult.flag_counts` は読み取り専用 Mapping、`final_command` は読み取り専用配列。
- 🟡 動画: 最初の frame_sink 呼び出しでレンダラと writer を作る(計画失敗なら何も書かない)。運搬中の PNG は pre_place 区間の描画済みフレームの中央。
- 🔴 JSON の `flag_counts` は計画失敗(未実行)のとき全て 0。試行の行は計画失敗のとき「計画失敗: 理由 (経由点 …)」で「成功」の語を含めない。
- モック・スタブは実装コードに含まない。グローバル状態・グローバル RNG なし。

## 3. テスト結果

- `uv run pytest -q`: **1399 passed, 0 failed**(実行 37 s)。Red 時点の 57 failed(26 の既存 eval_config + 31 新規)と収集できなかった新規 5 ファイルが全て通る。
- テストの変更: **なし**。数値の期待が実装と食い違う箇所もなかった。
- 新規・変更テストファイルごとの実行時間(単独実行):

| ファイル | 件数 | 時間 |
|---|---|---|
| `tests/policy/test_executor.py` | 36 | 1.25 s |
| `tests/eval/test_judge.py` | 22 | 0.20 s |
| `tests/eval/test_failure.py` | 22 | 0.17 s |
| `tests/eval/test_fixed_pick.py` | 26 | 2.73 s |
| `tests/test_run_fixed_pick_script.py` | 15 | 12.6 s(動画 1 件の setup 約 8 s) |
| `tests/eval/test_eval_config.py` | 62 | 0.25 s |
| `tests/test_scene.py` | 25 | 1.09 s |
| `tests/robot/test_sim.py` | 46 | 0.98 s |

- カバレッジ(`uv run pytest --cov=so101_vlm`): `executor.py`、`judge.py`、`failure.py`、`pick_tracker.py`、`fixed_pick.py`、`eval_config.py`、`robot/sim.py` は全て 100%、パッケージ全体 100%(1679 行)。`scripts/run_fixed_pick.py` は測定の対象外(パッケージ外)だが、15 件のテストが main の全経路を通る。

## 4. MANUAL-01: 20 seed の掃引

`uv run python scripts/run_fixed_pick.py --colour all --n-seeds 20 --seed 0 --no-video`(実行 約 26 s):

- 試行 60 件、計画失敗 2(青 seed 16: `self_collision`@grasp、緑 seed 5: `not_converged`@pre_grasp)、計画可能な 58 件で **58/58 成功(100.0%)**。終了コード 0。期待値どおり。

## 5. 品質判定

✅ 高品質: 全テスト成功、実装はテストの要求に沿う最小限、ファイルは全て 800 行未満(最大 251 行)、実装コードにモックなし、コンパイルエラーなし。

## 6. Refactor への課題

1. `executor._build_result` の引数が 9 個(`_Stepper` に結果の組み立てを寄せる、または開始時刻と設定を持たせる)。
2. `run_fixed_pick` が約 45 行(本体の組み立てを `_finish_outcome` に分ける余地)。`_execute` の引数が 9 個。
3. `outcome_to_dict` / `format_trial_line` は `FixedPickOutcome` 側(または `eval/` の小モジュール)に移すと TASK-0026 の集計と共有できる。
4. `PickTracker` の区間名(lift / pre_place / place)は executor の `SEGMENT_WAYPOINT` と重複している(定数の共有を検討)。
5. 判定の速度は `qvel` 読み取りを 1 回だけ使っている(mj_step 直後の `xpos` は 1 ステップ古い可能性。実測では影響なし。TASK-0015 で判定の時点とあわせて扱う)。
6. `VideoRecorder` は pre_place のフレーム(約 25 枚、約 23 MB)をメモリに保持している。中点だけ撮るなら軌道の長さを渡す方法もある。
7. 運搬中の PNG の目視確認(完了条件 2)は verify-complete で行う。

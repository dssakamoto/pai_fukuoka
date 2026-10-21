# TASK-0013 Red フェーズ記録: 固定位置での pick & place(VLM なし)

**機能名**: fixed-pick / **タスクID**: TASK-0013 / **要件名**: so101-vlm-pickplace
**作成日**: 2026-10-21
**前提資料**: `pick-requirements.md`、`pick-testcases.md`(93 件)、`pick-survey.md`、`note.md`
**方針**: 失敗するテストだけを書いた。`src/`、`scripts/`、`assets/scene_pickplace.xml`、`configs/eval.yaml` は変更していない。ユーザーへの質問なし、エージェント起動なし、git commit なし。
**信頼性**: 🔵 要件・実測どおり / 🟡 妥当な推測 / 🔴 要件にない決定(テスト内の各 docstring に記載)

---

## 1. 作成・変更したテストファイル

| ファイル | 種別 | 関数数 | 対応する TC |
|---|---|---|---|
| `tests/test_scene.py` | 追加(既存 15 関数は不変) | +3 | TC-SC-01〜03 |
| `tests/eval/test_eval_config.py` | 変更(`VALID` に judge/failure、パラメータ追加) | 既存 9 を拡張、+2 関数 | TC-C-01, 03, 04, 05, E02〜E05, B01, B02 |
| `tests/policy/test_executor.py` | 新規 | 27(統合 3 を含む) | TC-EX-01〜14, E01〜E06, B01〜B04, TC-EXI-01〜03 |
| `tests/eval/test_judge.py` | 新規 | 12 | TC-JG-01〜06, E01, E02, B01〜B04 |
| `tests/eval/test_failure.py` | 新規 | 6(parametrize で FL-01〜11, B01, B02 を含む) | TC-FL-01〜11, E01, B01, B02 |
| `tests/robot/test_sim.py` | 追加(既存は不変) | +5 | TC-MH-01〜04, E01 |
| `tests/eval/test_fixed_pick.py` | 新規 | 12 | TC-FP-01〜10, E01, E02 |
| `tests/test_run_fixed_pick_script.py` | 新規 | 9(動画は 1 件のみ) | TC-CLI-01〜08, E01, E02(CLI-04 は CLI-01 の parametrize) |

- TC の網羅: 93 件のうち自動テスト 92 件を実装。残り 1 件 MANUAL-01(20 seed の掃引)は手動(`tdd-verify-complete` で実施)。
- TC-C-05 は既存の `TC-C-05` を変えず、同 ID に「(TASK-0013 追加)」の関数を足した(既存の assertion は不変)。
- 実行時間の見込み: 追加分 約 20 s 未満(動画 1 件 約 10 s を含む)。`slow` マーカーは登録がないため使っていない。

## 2. 失敗の理由(新規テストファイル)

実行コマンド: `uv run pytest -q <file>`

| ファイル | 結果 | 理由 |
|---|---|---|
| `tests/policy/test_executor.py` | 収集エラー | `ModuleNotFoundError: so101_vlm.policy.executor` |
| `tests/eval/test_judge.py` | 収集エラー | `ImportError: cannot import name 'JudgeConfig' from so101_vlm.eval.eval_config`(`so101_vlm.eval.judge` も未存在) |
| `tests/eval/test_failure.py` | 収集エラー | `ImportError: cannot import name 'FailureConfig'`(`so101_vlm.eval.failure` も未存在) |
| `tests/eval/test_fixed_pick.py` | 収集エラー | `ModuleNotFoundError: so101_vlm.eval.fixed_pick` |
| `tests/test_run_fixed_pick_script.py` | 1 failed + 14 errors | `scripts/run_fixed_pick.py` が存在しない(フィクスチャの読み込み失敗、`--help` は終了コード 2) |

既存ファイルへの追加分の失敗理由:

| テスト | 理由 |
|---|---|
| `tests/test_scene.py::test_physics_option_is_elliptic_cone_with_impratio`(TC-SC-01) | シーンに `<option>` がない(cone は pyramidal) |
| `tests/test_scene.py::test_option_appears_once_between_include_and_compiler`(TC-SC-03) | `<option` が 0 回 |
| `tests/robot/test_sim.py::test_mujoco_handles_*` 4 件(TC-MH-01〜03, E01) | `AttributeError: 'SimRobot' object has no attribute 'mujoco_handles'` |
| `tests/eval/test_eval_config.py` の新規・変更分 | `EvalConfig` に `judge` / `failure` がない、または未知キー `judge` / `failure` を拒否 |

次の 2 件は現状でも通る(守りのテスト。Green で壊さないことを確認する): TC-SC-02(上流 scene.xml は既定の物理のまま)、TC-MH-04(`Robot` Protocol に `mujoco_handles` を入れない)。

## 3. 既存テストのうち、この編集で通らなくなったもの(意図的)

編集前は全て通っていた `tests/eval/test_eval_config.py` の 26 件。原因は `VALID` に `judge` / `failure` を足したため(現在のローダーは未知キー `judge` / `failure` を拒否し、`configs/eval.yaml` にもまだない)と、TC-C-01 / C-03 / C-04 / C-B01 に judge / failure の assertion を足したこと。**Green で `EvalConfig` / ローダー / `configs/eval.yaml` を更新すると全て通る。**

```text
tests/eval/test_eval_config.py::test_config_is_frozen
tests/eval/test_eval_config.py::test_load_eval_config_accepts_lower_bounds
tests/eval/test_eval_config.py::test_load_eval_config_normalizes_int_to_float
tests/eval/test_eval_config.py::test_load_eval_config_reads_repository_file
tests/eval/test_eval_config.py::test_load_eval_config_reads_values_from_given_path[False]
tests/eval/test_eval_config.py::test_load_eval_config_reads_values_from_given_path[True]
tests/eval/test_eval_config.py::test_load_eval_config_rejects_invalid_value[path_keys0-0]
tests/eval/test_eval_config.py::test_load_eval_config_rejects_invalid_value[path_keys1-20]
tests/eval/test_eval_config.py::test_load_eval_config_rejects_invalid_value[path_keys10--0.1]
tests/eval/test_eval_config.py::test_load_eval_config_rejects_invalid_value[path_keys11-0]
tests/eval/test_eval_config.py::test_load_eval_config_rejects_invalid_value[path_keys2-1.5]
tests/eval/test_eval_config.py::test_load_eval_config_rejects_invalid_value[path_keys3-True]
tests/eval/test_eval_config.py::test_load_eval_config_rejects_invalid_value[path_keys4--1]
tests/eval/test_eval_config.py::test_load_eval_config_rejects_invalid_value[path_keys5-True]
tests/eval/test_eval_config.py::test_load_eval_config_rejects_invalid_value[path_keys6-0]
tests/eval/test_eval_config.py::test_load_eval_config_rejects_invalid_value[path_keys7-0.2]
tests/eval/test_eval_config.py::test_load_eval_config_rejects_invalid_value[path_keys8-0.3]
tests/eval/test_eval_config.py::test_load_eval_config_rejects_invalid_value[path_keys9-nan]
tests/eval/test_eval_config.py::test_load_eval_config_rejects_missing_required_key[path_keys0]
tests/eval/test_eval_config.py::test_load_eval_config_rejects_missing_required_key[path_keys1]
tests/eval/test_eval_config.py::test_load_eval_config_rejects_missing_required_key[path_keys2]
tests/eval/test_eval_config.py::test_load_eval_config_rejects_missing_required_key[path_keys3]
tests/eval/test_eval_config.py::test_load_eval_config_rejects_missing_required_key[path_keys4]
tests/eval/test_eval_config.py::test_load_eval_config_rejects_missing_required_key[path_keys5]
tests/eval/test_eval_config.py::test_load_eval_config_rejects_missing_required_key[path_keys6]
tests/eval/test_eval_config.py::test_load_eval_config_rejects_unknown_key[obj1-min_gapp]
```

- 通ったままの既存 9 件: `test_default_path_is_repository_configs_eval_yaml`、`test_load_eval_config_missing_file_raises_file_not_found`、`..._rejects_non_mapping[empty/list/placement-list/placement-null/scalar]`、`..._rejects_unknown_key[obj0-n_trial]`、`..._uses_safe_load`。
- `tests/eval/test_eval_config.py` 以外の既存テストは 1 件も失敗しない(`test_scene.py` の失敗 2 件、`test_sim.py` の失敗 4 件は全て今回追加した新規テスト)。
- 新規ファイル 5 本を除外した全体: **57 failed, 1221 passed**。内訳は上の 26 件(既存)+ `test_eval_config.py` の新規・追加パラメータ 25 件 + `test_scene.py` 2 件 + `test_sim.py` 4 件。編集前に通っていた 1243 件のうち通らなくなったのは上の 26 件のみ(残り 1217 件は通る)。1221 passed には、今回追加して現状でも通るテスト(TC-SC-02、TC-MH-04 など)が含まれる。
- 配慮: `JudgeConfig` / `FailureConfig` は `test_eval_config.py` ではモジュール経由(`eval_config_module.JudgeConfig`)で参照し、import 時に全滅しないようにした。

## 4. テストロジックの事前確認

Red の段階で実装は書かないが、テスト自体の誤りを避けるため、使い捨ての最小実装(リポジトリの外のコピー `.../scratchpad/repo_copy`。executor、judge、failure、`EvalConfig`、`mujoco_handles`、シーンの `<option>`)に対して `test_executor.py` / `test_judge.py` / `test_failure.py` / `test_eval_config.py` / `test_scene.py` / `test_sim.py` を実行し、213 件が全て通ることを確認した。`test_fixed_pick.py` と `test_run_fixed_pick_script.py` は実装がないため未確認(数値は `pick-testcases.md` の実測による)。リポジトリ本体は変更していない。

## 5. 未決事項の決定

| # | 内容 | 決定 | 信頼性 |
|---|---|---|---|
| T1 | 開始姿勢の許容 | 差が 0.05 rad **以上**で `ValueError`(メッセージに「開始」を含む)。TC-EX-B01 は 0.0499 を許し 0.05 を拒否。Green で `pick-requirements.md` 3.2 の「超えたら」を「以上」に直す | 🔴 指示どおり |
| T2 | 打ち切り時の `n_commands` | 打ち切った周期の保持指令を含めて数える(TC-EX-05: 4、TC-EX-06: 3) | 🔴 既定値 |
| T3 | 待ちの周期の `target_base` | `None`(打ち切りの後に `rejected` が増えない。TC-EX-09 は spy で確認) | 🔴 既定値 |
| T4 | `on_step` の周期番号 | 1 始まりの通し番号、送信の後に呼ぶ、待ちの区間名は `"settle"` | 🔴 既定値 |
| T5 | 動画テスト | 1 件のみ、640×480、`--no-video` 以外は動画なし | 🟡 既定値 |
| T6 | CLI の文言と JSON | 試行の行に `seed=N`、色名、`成功`、失敗の分類/計画失敗の理由。集計行に `成功 a/b` と `計画失敗 n`。JSON は `seed`、`colour`、`success`、`failure`、`planning_error`、`flag_counts`(4 キー)を必ず含む | 🔴 既定値 |
| T7 | `--video` と `--no-video` の同時指定、`--video` と `--n-seeds 2` | 終了コード 2(`SystemExit`) | 🟡 既定値 |
| T8 | `PickTrace` の検証 | 有限、距離と傾きは 0 以上、`lift_end_rise` は負を許す | 🔴 既定値 |
| T9(新) | TC-EX-06 のフェイクの NaN の指定 | 「N 回目の `get_state`」ではなく「送信済みの指令数が N のときの `get_state`」(`FakeRobot(nan_time_when_sent=(2,))`)。executor が開始時に何回状態を読んでも結果が変わらないようにするため | 🔴 テスト設計 |
| T10(新) | スクリプトの補助関数名 | `default_video_path(out_dir, colour, seed)` と `default_carry_png_path(out_dir, colour, seed)` を公開関数にする。`run_fixed_pick` はスクリプトのモジュール大域名(monkeypatch 可能)にする | 🔴 TC-CLI-05, 07 のため |
| T11(新) | executor の公開名 | `SEGMENT_WAYPOINT`(区間 → 経由点の Mapping)、`START_TOLERANCE_RAD`、`ExecutionResult`(`n_commands`、`settle_steps`、`flag_counts`、`aborted`、`abort_reason`、`abort_label`、`workspace_checked`、`final_command`、`final_state`、`final_joint_error`、`duration_s`、`wall_time_s`、`states`、`has_safety_flags`)。`executor.py` は `apply_safety` をモジュール大域名から引く(TC-EX-09 の spy) | 🟡 要件 3.3 |
| T12(新) | `fixed_pick.py` の大域名 | `SimRobot` と `run_trajectory` をモジュール大域名から引く(TC-FP-05, E01 の monkeypatch)。`pick_target` は tuple、`FixedPickOutcome` は `seed`、`colour`、`pick_target`、`planning_error`、`planning_waypoint`、`plan_time_s`、`execution`、`judge`、`trace`、`failure`、`other_block_moved`、`success` | 🟡 要件 6 |
| T13(新) | `run_trajectory` の引数の検証順 | 未知の区間名・不正な `settle_s`・軌道の型違いは、状態を読む前または最初の送信前に `ValueError`/`TypeError` にする(何も送らない) | 🟡 TC-EX-E02〜E06 |

## 6. Green フェーズで実装すること

1. `assets/scene_pickplace.xml`: include の直後に `<option cone="elliptic" impratio="10"/>`(コメントに `<option` と書かない)。
2. `SimRobot.mujoco_handles()`(閉じた後は `RuntimeError("SimRobot is closed")`、`Robot` Protocol には入れない)。
3. `eval_config.py`: `JudgeConfig`(margin ≥ 0 かつ < `box_inner_half_size`、speed > 0、settle ≥ 0)、`FailureConfig`(全て > 0、tilt は (0, 90))、`_TOP_KEYS` に `judge` / `failure`。`configs/eval.yaml` に値とコメントを追加。
4. `src/so101_vlm/eval/judge.py`、`failure.py`、`policy/executor.py`、`eval/fixed_pick.py`、`scripts/run_fixed_pick.py`(動画: `mujoco.Renderer` 640×480、制御 2 周期に 1 フレーム、25 fps、free camera)。
5. `pick-requirements.md` 3.2 の開始姿勢の許容の文言を「0.05 以上」に合わせる。
6. 確認: 新規 5 ファイルが通ること、既存の `test_eval_config.py` 26 件が通ること、全体で失敗なし、`uv run python scripts/run_fixed_pick.py --seed 0 --n-seeds 20 --colour all --no-video` で 58/58 と計画失敗 2 件(MANUAL-01)。

## 7. 品質判定

- テスト実行: 新規ファイルは全て期待した理由で失敗(収集エラー/ファイルなし)、既存ファイルへの追加分も機能不在で失敗することを確認済み。
- 期待値: 実測値と許容幅を明記。浮動小数の境界は `SPEC_EXACT` と `np.nextafter` を使用。
- 信頼性の分布: 🔵 約 45%、🟡 約 40%、🔴 約 15%(`pick-testcases.md` と同じ)。
- 判定: **✅ 高品質**(fixed_pick とスクリプトのテストは実装なしでは論理を確認できていない点を除く)。

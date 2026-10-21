# TASK-0013 Refactor フェーズ記録 (pick)

## 結果
- テスト: 1399 passed (前後同数)。全体 約 37-50 s (環境の揺れの範囲、前後で差なし)。
- カバレッジ: executor / segments / fixed_pick / fixed_pick_report / pick_tracker / judge すべて 100%、全体 100%。
- 20 seed x 3 色の掃引: 試行 60、計画失敗 2、計画可能な配置での成功 58/58 (リファクタ前と同一)。

## 変更点
1. 🔵 引数の削減: `executor._build_result` を 9 -> 6 引数 (`_Progress` frozen dataclass、`control_dt`、`workspace_checked`)。`fixed_pick._execute` を 9 -> 5 引数、`_plan` を 5 -> 4 引数 (`_Configs` frozen dataclass で eval/safety/trajectory 設定をまとめ、model/data は `robot.mujoco_handles()` から取得)。`run_fixed_pick` 本体は約 28 行 + docstring に短縮、判定は `_judge_block` に分離。
2. 🔵 `outcome_to_dict` / `format_trial_line` / `format_summary` を `src/so101_vlm/eval/fixed_pick_report.py` へ移動 (TASK-0026 と共有)。スクリプトは import するだけで、JSON キー・出力文言は不変。
3. 🔵 区間名の単一の定義元 `src/so101_vlm/policy/segments.py` (`SEGMENT_WAYPOINT`, `SEGMENT_LIFT/PRE_PLACE/PLACE`)。標準ライブラリのみ依存で循環なし。executor は import (従来名 `executor.SEGMENT_WAYPOINT` は維持)、pick_tracker は定数をここから導出。フラグ名は `executor.FLAG_NAMES` に公開し、レポートと共有 (スクリプト側の重複定義を削除)。
4. 🟡 `VideoRecorder`: pre_place のフレームを生配列 (約 23 MB) ではなく PNG バイト列 (数百 KB) で保持し、中点のバイト列をそのまま書く。符号化は imageio の imwrite と同じなので PNG・動画の出力は不変。
5. 🟡 `fixed_pick._judge_block` に注記: `data.xpos` は mj_step が積分前に計算するため最後の 1 サブステップ (約 2 ms) 古い可能性がある (qvel は積分後)。静止後は無視できるので挙動は変えず、TASK-0015 で厳密化 (mj_forward 等) を検討すること。

## 品質・セキュリティ・性能
- 入力の変更なし、グローバル状態・乱数・秘密情報なし。出力は `--out-dir` / `--video` で指定されたパス (と親ディレクトリ作成) のみ。既存ファイルの上書きは指定パスに限る。
- 関数は全て 50 行未満、ファイルは 800 行未満 (スクリプト 約 230 行)。レイアウト境界・安全・軌道・シーン・設定値は無変更。

## 意図的に残したもの
- `--video` 指定時の carry PNG は `--out-dir` 側に出る (従来どおり)。
- ruff は環境に未導入のため未実行。

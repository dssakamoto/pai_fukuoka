# TASK-0009 Green フェーズ記録 (calibration)

**実装日**: 2026-10-21 / **要件名**: so101-vlm-pickplace / **タスク**: TASK-0009

## 実装したファイル
- `src/so101_vlm/perception/calibration.py` (279 行): camera_from_mujoco / t_base_cam_from_mujoco / Calibration / calibration_from_mujoco / calibration_max_abs_diff / calibrations_close / save_calibration / load_calibration / DEFAULT_CALIBRATION_PATH。分割はしていない (300 行未満)。
- `scripts/generate_calibration.py` (96 行): `main(argv)`、`--out` / `--check` / `--force`。終了コード 0/1/2。
- `configs/calibration.yaml`: スクリプトで生成した sim の値 (要件 2.6 の実測値と一致)。

## 実装方針・判断 (Green 時点)
- 検証は `eval/validation.py` の `require_exact_keys` / `require_int` / `require_number` を再利用。
- 固定カメラ判定: `model.cam_mode` は `mjtCamLight` の値 (FIXED=0)。`mjtCamera` ではない (最初に取り違えて失敗、修正済み)。
- 行列の行だけ flow 形式にするため `SafeDumper` サブクラス + `list` サブクラス `_FlowRow`。`width=10**9` で折り返しを防止、`ignore_aliases` で anchor を防止。
- `calibrations_close` は `calibration_max_abs_diff <= atol` (離散項目の不一致は inf)。スクリプトの差の表示にも同関数を使う。
- 追加した公開関数 `calibration_max_abs_diff` は要件にない (🔴)。--check の差の最大値出力のため。

## テスト結果
- 新規 3 ファイル: 252 passed (test_calibration 221 / test_calibration_scene 15 / test_generate_calibration 16)。
- 実行時間 (uv 起動込み): test_calibration 約 2.6 s、test_calibration_scene 約 4.4 s、test_generate_calibration 約 3.0 s。3 ファイル合計 8.2 s。
- `tests/perception`: 410 passed 7.5 s。全体 `uv run pytest -q`: 976 passed 約 15 s。
- カバレッジ: calibration.py 98% (未通過は 277-278 行 = `yaml.YAMLError` の ValueError 変換)。全体 99%。
- R-02 / G-01 (バイト一致) は安定して通過。末尾の桁の揺れによる失敗は発生せず、atol 比較への縮小は不要。

## テストの変更・削除
- なし。T-E05 (別モデル data) は `cam_xpos.shape[0] != model.ncam` の検査で、C-E06 (fovy 範囲外) は `0 < fovy < 180` で実装でき、削除・弱体化していない。
- `tests/perception/test_projection_scene.py` は未変更。

## 課題・Refactor への申し送り
- `test_projection_scene.py` のテスト専用ヘルパ (`camera_and_T_base_cam`) を `calibration_from_mujoco` に置き換える。
- 未カバー: YAML 構文エラー (`yaml.YAMLError` -> ValueError) のテストがない。追加するとよい。
- `save_calibration` は非アトミック書き込み (テキストを先に作ってから `write_text`)。必要なら一時ファイル経由にする。
- スクリプトのメッセージ行が 100 桁超の箇所がある (整形)。ruff は環境に未導入で lint 未実施。
- 重複キーは検出しない (PyYAML の制約。要件 8.1-11)。

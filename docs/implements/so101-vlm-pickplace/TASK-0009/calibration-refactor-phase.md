# TASK-0009 Refactor フェーズ記録 (calibration)

**実施日**: 2026-10-21 / **要件名**: so101-vlm-pickplace / **タスク**: TASK-0009 / 振る舞いの変更なし

## 実施した改善
| # | 内容 | 信頼性 |
|---|------|--------|
| 1 | `tests/perception/test_projection_scene.py` のテスト専用ヘルパ `camera_and_T_base_cam` と `_transform` を削除し、`calib` フィクスチャを製品コード `calibration_from_mujoco` に置換。S-01 の docstring とモジュール docstring の記述を修正。assertion は無変更、15 passed のまま (1 mm 等の数値期待も通過) | 🔵 |
| 2 | YAML 構文エラー (`yaml.YAMLError` -> `ValueError`) のテスト `test_load_calibration_wraps_yaml_syntax_error_in_value_error` を新規追加 (3 パラメタ)。calibration.py のカバレッジ 98% -> 100% | 🔵 |
| 3 | `save_calibration` のアトミック書き込みは見送り (下記) | 🟡 |
| 4 | `scripts/generate_calibration.py`: 100 桁超の行を折り返し、`_build_parser()` を分離、`main` に docstring を追加。ruff E501 (100) クリア | 🔵 |
| 5 | `test_calibration.py` (1080 行) を純粋移動で分割 (下記) | 🔵 |
| 6 | 名前・docstring・定数・型ヒント・関数長 (<50 行) を確認。src 側は変更不要 | 🔵 |

## テスト分割 (純粋移動)
- `tests/perception/test_calibration.py` (505 行): camera / t_base_cam / dataclass / calibrations_close / それらの異常系
- `tests/perception/test_calibration_io.py` (537 行): save / load / ドリフトガード / save・load の異常系 / 境界値 (+ 新規 1 テスト)
- `tests/perception/conftest.py` (62 行): 共有フィクスチャ (scene_model, scene_data, tiny_model, tiny_data, tiny_xml_path, sim_calibration, committed, valid_doc)
- `tests/perception/calibration_support.py` (48 行): 共有定数・ヘルパ (W, H, FY_60_480, SIM_T_ROWS, TINY_XML, forwarded, golden_calibration)。`from conftest import` は tests/safety/conftest.py と衝突しうるため別モジュールにした
- テスト本体・docstring は無変更。変更はヘルパ名の改名のみ (`_forwarded` -> `forwarded`, `_golden_calibration` -> `golden_calibration`、モジュール間 import のため)。
- 収集数: 分割前 410 -> 分割後 410 (同数) -> 新規 3 パラメタ追加で 413。

## 見送り: save_calibration のアトミック書き込み
一時ファイル + `os.replace` は、(a) `mkstemp` が 0600 で作るためファイル権限が `write_text` の 0644 から変わる (chmod 追加が必要)、(b) シンボリックリンク先への書き込みが置換に変わる、など振る舞いが変わる。用途が手元の小さな設定ファイルで、書き込み途中の破損リスクは低い。単純さを保てないので現状維持とした。

## セキュリティレビュー
- 読み込みは `yaml.safe_load` のみ、書き出しは `SafeDumper` 派生のみ。任意オブジェクトの復元なし。
- 入力は `require_exact_keys` / `require_int` / `require_number` で厳格に検証 (未知キー・bool・非有限値を拒否)。
- 入力の変更なし (配列はコピーして書き込み不可)。グローバル状態なし。秘密情報なし。重大な指摘なし。

## パフォーマンスレビュー
行列は 4x4、YAML は数十行で計算量は無視できる。重大な課題なし。

## テスト結果
- `uv run pytest -q`: 979 passed (12〜17 s)。分割前 976 + 新規 3。
- tests/perception: 413 passed (約 10 s)。
- `uv run pytest --cov -q`: calibration.py 100% (131 文、未カバー 0)、全体 100%。

## 品質判定: 高品質
テスト全成功 / セキュリティ問題なし / 性能課題なし / 全ファイル 800 行未満 / ドキュメント完成。

次のお勧めステップ: `/tsumiki:tdd-verify-complete`

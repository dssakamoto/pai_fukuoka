# TASK-0006 layout Green フェーズ記録

**実施日**: 2026-10-20

## 実装ファイル
- `configs/eval.yaml` (新規): n_trials=20, base_seed=0, placement(n_blocks=3, area=0.20, min_gap=0.05, max_attempts=10000)
- `src/so101_vlm/eval/validation.py` (新規, 58 行): 型・値域検証の共有ヘルパ (require_int / require_number / require_area / require_min_gap)。area の上限は作業範囲の幅 + 1e-9 🔵
- `src/so101_vlm/eval/eval_config.py` (新規, 74 行): frozen dataclass PlacementConfig / EvalConfig、load_eval_config (yaml.safe_load、未知/欠落キー・非 mapping・型違反は ValueError でキー名入り、ファイルなしは FileNotFoundError)、DEFAULT_EVAL_CONFIG_PATH
- `src/so101_vlm/eval/layout.py` (新規, 96 行): sample_layout (default_rng(seed)、棄却サンプリング、引き数の合計が max_attempts、対角線事前判定、key=value 形式のエラー)、place_blocks (全検証後に freejoint の qpos/qvel を書き込み、mj_forward)

## 方針
- 実装コードの全文は上記ファイルを参照 (日本語コメントと信頼性レベル付き)。
- validation.py は要件にないが、layout と eval_config で同一規則を共有するための最小分割 (DRY) 🟡。
- テストの変更: なし。

## テスト結果
- `uv run pytest -q`: 247 passed (2.5 秒)
- `uv run pytest --cov -q`: 全体 100%。eval_config.py / layout.py / validation.py すべて 100%
- tests/eval: 166 passed, 0.79 秒 (最遅は setup 0.25 秒)

## 品質判定: 高品質
- 関数 < 50 行、ファイル < 800 行、モック不使用

## Refactor 課題
- 検証メッセージの言語・書式の統一
- sample_layout の長さ (約 35 行) と領域計算の分離余地
- place_blocks の _resolve_free_joint で jntnum != 1 の扱い (複数関節 body) の仕様確認

# TASK-0006 ブロック初期配置の乱数生成 TDD 検証記録

**検証日**: 2026-10-20
**判定**: OK (完全実装済み、スコープ外失敗なし)

## テスト状況
- `uv run pytest -q`: 247 passed (2.85 s)
- `uv run pytest --cov -q`: 247 passed、カバレッジ 100% (268 文 / 未実行 0)
- スコープ内 (`tests/eval/`): 166 passed (1.00 s)。遅い (2 秒以上) テストなし
- スコープ外の失敗: なし

## テストケース対応
- layout-testcases.md の TC 50 件すべてに対応するテストあり (50/50)。TC id はテストの docstring に記載
  - `tests/eval/test_layout.py`: TC-L-01..10, E01..08, B01..06
  - `tests/eval/test_eval_config.py`: TC-C-01..05, E01..06, B01
  - `tests/eval/test_place_blocks.py`: TC-P-01..06, E01..04, TC-I-01..04
- 未実装テストケース: 0

## 完了条件
| 条件 | 結果 | 根拠 |
|---|---|---|
| 同じシードで同じ配置 | 達成 | TC-L-01/02/04、別プロセス比較 |
| 範囲内かつ 5 cm 以上 | 達成 | TC-L-05/06 (シード 0..99)、独立確認 (0..999) |
| 単体テストが通る | 達成 | 247 passed |
| 統合: 配置をシーンに適用し物理を進める | 達成 | TC-I-01..04 (シード 0..19 で 500 step。静止、重なりなし、箱と非交差) |

## FR-12 / NFR-01
- FR-12: 20 cm 四方、間隔 5 cm 以上を満たす。`configs/eval.yaml` に範囲・間隔を設定
- NFR-01: `default_rng(seed)` のみ使用。20 試行 (シード 0..19) は TC-L-08 で成功・全て異なることを確認

## 独立スポットチェック (`uv run python`)
- シード 0..999: 範囲外 0 点、最小中心間距離 0.050026 m (>= 0.05)
- 2 つの別プロセス (PYTHONHASHSEED=1, 2) で全 1000 配置のハッシュが一致 (9db89deed7f7a4d5)
- シード 7 の配置も両プロセスで同一

## 要件網羅
要件定義書の項目 (入力検証、出力型、決定性、棄却サンプリングと上限エラー、設定読み込み、place_blocks) はすべてテスト済み。網羅率 100%。

## 更新したファイル
- docs/tasks/so101-vlm-pickplace/TASK-0006.md (ステータス、完了日 2026-10-20、チェックボックス)
- docs/tasks/so101-vlm-pickplace/overview.md

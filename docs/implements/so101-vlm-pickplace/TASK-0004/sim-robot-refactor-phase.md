# TASK-0004 Refactor フェーズ記録

## 変更
- `src/so101_vlm/robot/base.py`: `N_JOINTS` 定数と `validate_q_target()`(shape (6,)・有限性検証 + float64 コピー、入力非変更)を追加。実機 Robot(TASK-0034)から再利用可能。🟡
- `src/so101_vlm/robot/sim.py`: `send_command` の検証を `validate_q_target` に委譲、`_N_JOINTS` を廃止、`DEFAULT_N_SUBSTEPS` 定数化、`get_state` のコメントを明確化(防御コピーは `RobotState` 側の 1 回のみ)。
- `close()` は冪等フラグのみ維持(MjData は GC に任せる)。参照解放は get/send が RuntimeError となるため効果が薄く見送り。

## レビュー
- セキュリティ: 秘密情報なし、入力非変更、NaN/inf・shape を境界で検証。
- パフォーマンス: 毎ステップのコピーは 6 要素のみ。問題なし。
- 規模: base.py 約 70 行、sim.py 約 105 行、全関数 50 行未満。

## テスト
- `uv run pytest -q`: 59 passed (約 1.2 s)。テスト変更なし。
- カバレッジ: base.py 100%、sim.py 100%。

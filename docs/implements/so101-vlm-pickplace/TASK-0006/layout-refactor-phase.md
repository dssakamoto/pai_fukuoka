# TASK-0006 layout Refactor フェーズ記録

**実施日**: 2026-10-20

## 変更内容 (振る舞い不変)
- `src/so101_vlm/eval/layout.py`: 領域計算を純粋関数 `_sampling_bounds(area)` に分離し、`sample_layout` を短縮 (108 行) 🔵
- `_resolve_free_joint` の docstring に仕様を明記: body は関節をちょうど 1 つ (freejoint) 持つ必要がある。0 個・複数個は qpos 位置が一意に定まらず誤書き込みの恐れがあるため拒否 (要件定義書 2.3 に沿う。挙動は変更なし) 🟡
- テストの変更: なし

## 確認結果
- 作業範囲の数値は `scene_spec.py` のみに定義。`validation.py` のコメント内の数値 (0.35-0.15) は説明用で実コードの重複ではない 🔵
- `configs/eval.yaml` の area=0.20 は作業範囲の幅 0.20 以下で整合 (許容差 `WIDTH_TOLERANCE`=1e-9 は定数化済み)
- 検証メッセージは全て日本語 + `key=value` 形式で統一済みのため変更不要
- セキュリティ: `yaml.safe_load` のみ使用、秘密情報なし、入力の破壊的変更なし (place_blocks は検証後に data へ書き込み)
- 品質: 関数 < 50 行、ファイル < 500 行、モック不使用

## テスト結果
- `uv run pytest -q`: 247 passed (約 2.1 秒、tests/eval は約 0.8 秒)
- カバレッジ: eval_config.py / layout.py / validation.py / scene_spec.py すべて 100% (全体 100%)

## 品質判定: 高品質

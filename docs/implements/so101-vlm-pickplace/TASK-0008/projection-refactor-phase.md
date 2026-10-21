# TASK-0008 Refactor フェーズ記録 (projection)

## 変更内容
1. `_depth_with_clipped_bbox` を追加。代表深度とクリップ済み bbox を一度の検証で返す。`representative_depth` と `bbox_to_base_point` の `_clipped_bbox` 二重呼び出しを解消 (公開 API・検証順序・例外は不変) 🔵
2. 定数 `DEFAULT_SHRINK = 0.5` を追加し、既定値のハードコードを置換 🟡
3. 私的ヘルパー (`_is_bool`, `_positive_int`, `_vec3`, `_check_shrink`) に日本語 docstring を追加 🟡
4. 未到達分岐の網羅: 到達不能ではないため削除せず、`tests/perception/test_projection_extra.py` を追加
   - 42-43: `float()` が TypeError を出す入力 (None, list, object, complex)
   - 91-92: 不整形配列で `np.asarray` が ValueError
   - 196: shrink が極小だと mid±half が丸めで mid に潰れ、mid が整数のとき stop==start になる (到達可能。最低 1 画素の保証として必要)

## 見送り
- `_validation.py` への分離: 利用箇所は本ファイル内のみで、TASK-0009 の要件が未確定のため重複は未発生。再利用が必要になった時点で `_finite_float` / `_real_array` / `_vec3` を移す。

## レビュー
- セキュリティ: 入力は変更せず新しい float64 配列を返す。グローバル状態なし。mujoco import なし。秘密情報なし。
- パフォーマンス: bbox 検証の重複 1 回分を削減。他は O(領域画素数)。
- ファイルサイズ: projection.py 約 270 行、関数はすべて 50 行未満。

## 結果
- `uv run pytest -q`: 469 passed (既存 463 + 追加 6)、約 5.8 s
- tests/perception: 158 passed (約 2.8 s)
- projection.py カバレッジ: 100%

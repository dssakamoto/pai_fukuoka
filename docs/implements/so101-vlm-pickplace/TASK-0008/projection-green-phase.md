# TASK-0008 Green フェーズ記録: projection

**実施日**: 2026-10-20
**実装ファイル**: `src/so101_vlm/perception/projection.py` (249 行、numpy と標準ライブラリのみ。モック・MuJoCo import なし)

## 実装方針
- 要件定義書の規約をそのまま実装 (OpenCV 規約、z 深度、主点 (w-1)/2・(h-1)/2、bbox は半開区間)。🔵
- 公開: `Camera`(frozen dataclass), `camera_matrix`, `mujoco_cam_to_opencv`, `pixel_to_camera`, `camera_to_base`, `project_base_to_pixel`, `validate_T_base_cam`, `representative_depth`, `bbox_to_base_point`。
- 内部ヘルパ: `_finite_float`(bool/文字列/非有限を弾く), `_positive_int`, `_real_array`(dtype kind が i/u/f のみ許可), `_vec3`, `_clipped_bbox`, `_center_range`(floor/ceil、最低 1 画素), `_check_shrink`。
- 不正入力は全て `ValueError`。戻り値は新しい float64 配列(`project_base_to_pixel` は Python float の tuple)。入力は変更しない。
- 代表深度の領域: クリップ後 bbox の中心から shrink 倍、`[floor, ceil)`、クリップ後 bbox 内に収め最低 1x1。有効深度(有限かつ>0)の中央値、0 個なら `ValueError`。
- 表面の点を返す旨と、呼び出し側が z から 0.015 を引く旨を docstring に明記。

## テスト結果
- `uv run pytest -q`: 463 passed (7.2 s)
- `uv run pytest tests/perception -q --durations=5`: 152 passed (1.86 s)。最遅は統合テストのフィクスチャ setup 0.81 s。
- `uv run pytest --cov -q`: `projection.py` 96% (未カバー: 42-43, 91-92, 196 = 防御的な例外分岐)、TOTAL 99%。
- テストの変更: なし。

## 品質判定
✅ 高品質。全テスト成功、ファイル 249 行、全関数 50 行未満、モック無し。

## 課題・Refactor 候補
- 未カバーの防御分岐(`_finite_float` の float() 失敗、`_real_array` の変換失敗、`_clipped_bbox` の非イテラブル)にテスト追加を検討。
- `_finite_float` の bool 判定などを将来 TASK-0009 の calibration と共有するか検討。
- `bbox_to_base_point` で `_clipped_bbox` を 2 回呼んでいる(`representative_depth` 内と中心計算用)。内部関数に分けて重複を解消可能。

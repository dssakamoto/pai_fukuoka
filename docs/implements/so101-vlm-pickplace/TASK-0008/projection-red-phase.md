# TASK-0008 Red フェーズ記録: projection(画像座標 → 3 次元位置)

**作成日**: 2026-10-20
**要件名**: so101-vlm-pickplace / **タスクID**: TASK-0008 / **機能名**: projection
**テストケース定義**: `docs/implements/so101-vlm-pickplace/TASK-0008/projection-testcases.md`
**要件定義**: `docs/implements/so101-vlm-pickplace/TASK-0008/projection-requirements.md`

## 1. 作成したテストファイル

| ファイル | テスト関数数 | 対応 ID | 内容 |
|---|---|---|---|
| `tests/perception/test_projection.py` | 59 | U-01〜U-59 | 合成データの単体テスト(MuJoCo 不使用)。705 行 |
| `tests/perception/test_projection_scene.py` | 9 | S-01〜S-09 | MuJoCo 統合テスト(module スコープのフィクスチャ)。258 行 |

- 関数数は定義書の 68 件と一致。`parametrize` 展開後の件数は Green フェーズで `--collect-only` により確認する。
- `tests/perception/` に `__init__.py` は置かない(他のテストディレクトリと同じ)。
- テスト専用ヘルパ `camera_and_T_base_cam(model, data, camera_name, width, height)` は統合テストファイル内に実装した(src には置かない)。fovy から `fy=(h/2)/tan(fovy/2)`、`cx=(w-1)/2`、`cy=(h-1)/2`、`cam_xpos/cam_xmat` と `baseframe` サイトから `inv(T_world_base) @ T_world_cam_mj` を作り、`mujoco_cam_to_opencv` を通す。
- ブロック上面の bbox は定義書どおりセグメンテーション描画(`enable_segmentation_rendering`)で取る(S-06 の前提確認、S-02/S-08 の床判定、S-09 の外接 bbox)。S-05 の上面 bbox は上面 4 隅の投影から作る。

## 2. 未決事項の決定(O-1〜O-6)

| ID | 決定 |
|---|---|
| O-1 | `camera_to_base` は T を `validate_T_base_cam` で完全に検証する(U-35 で形状・NaN・det=-1 を `ValueError` として固定)。 |
| O-2 | 整数型の深度画像(uint16 など)は受け付けるかをテストしない。bool・複素数・文字列 dtype は U-47 で `ValueError`。 |
| O-3 | `p` は形状 `(3,)` のみ。`(2,)`, `(4,)`, `(3,1)` は `ValueError`(U-34)。`(1,3)` は個別にはテストしない。 |
| O-4 | エラーメッセージの文言はアサートしない(`pytest.raises(ValueError)` のみ)。 |
| O-5 | 統合テストはセグメンテーション描画をテスト内のフィクスチャだけで使う。 |
| O-6 | S-09 の特性記録は水平誤差 5〜15 mm、z 誤差は負、として固定。範囲外れ時は survey と TASK-0024 の記録を更新してから直す。 |

## 3. 定義書からの軽微な調整

- U-36 の「カメラと同一平面の点」: `T_OBLIQUE` の `eye + 0.1·x軸` はカメラ座標の z が浮動小数の丸めで厳密に 0 にならず、実装の判定次第で不安定になる。同一平面の検査は軸平行の `T_DOWN`(z が厳密に 0)で行い、「後ろの点」は `T_OBLIQUE` のままにした。
- U-25 の「入力と共有しない」検査は、配列を返す関数の代表入力(`p`, `T`, `D`)に対して行った。
- U-55 の「1+1e-7 倍は通る」は、直交誤差約 2e-7 と det 誤差約 3e-7 がともに 1e-6 未満であることに依存する。

## 4. 実行結果(Red 確認)

- `uv run pytest tests/perception -q` → 2 ファイルとも収集時に `ModuleNotFoundError: No module named 'so101_vlm.perception.projection'`(`2 errors in 0.53s`)。実装がまだ無いという正しい理由で失敗している。
- `uv run pytest -q --ignore=tests/perception` → `311 passed`(既存テストは影響なし)。
- 実装は作成していない(`src/so101_vlm/perception/projection.py` は未作成。スタブも置いていない)。

## 5. 信頼性レベルの分布(おおよそ)

🔵 約 22 / 🟡 約 33 / 🔴 約 13(定義書 8 章と同じ。各テストの docstring 末尾に記号を付けた)。🔴 は bbox の細かい規約・bool の扱い・形状の厳しさ・特性記録 S-09 に対応する。

## 6. 品質判定: 高品質

- テスト実行: 実行可能で、期待した理由(モジュール未実装)で失敗することを確認済み。
- 期待値: 解析解(合成データ)または参照実装で確認済みの実測値。許容誤差は定数(`ROUNDTRIP_TOL_M` 等)にまとめた。
- 注意: テスト自体の正しさ(参照実装との突き合わせ)は Green フェーズで実装を入れて初めて確認できる。Green で落ちたテストは、実装と定義書のどちらが誤りかを先に切り分けること。
- `test_projection.py` が 705 行(目標の 500 行を超えるが、グローバル規約の上限 800 行以内)。定義書でファイル名が固定されているため分割しなかった。必要なら Refactor フェーズで異常系を別ファイルに分ける。

## 7. Green フェーズで実装すべき内容(`src/so101_vlm/perception/projection.py`)

- `numpy` と標準ライブラリだけを import(U-28)。
- `Camera`(frozen dataclass、`__post_init__` 検証。bool/文字列/float のサイズは `ValueError`)、`camera_matrix`、`pixel_to_camera`(Camera を第 4 引数)、`camera_to_base`(T を完全検証、p は `(3,)` 有限)、`project_base_to_pixel`(`(u, v, z)` の float tuple、`z<=0` は `ValueError`)、`validate_T_base_cam`、`mujoco_cam_to_opencv`(入力検証 + `T @ diag(1,-1,-1,1)`)、`representative_depth`(半開区間 bbox・クリップ・中央領域 floor/ceil・最低 1 画素・有効深度の中央値・`float` 返却)、`bbox_to_base_point`(クリップ後の中心画素 `((x1+x2-1)/2, (y1+y2-1)/2)`、表面の点を返し補正はしない)。
- 不正入力はすべて `ValueError`(`TypeError` を出さない)。入力は変更せず、戻り値は新しい書き込み可能な float64 配列。
- 実行: `uv run pytest tests/perception -q`、`uv run pytest --durations=10`、`uv run pytest --cov`。

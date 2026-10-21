# TASK-0008 開発コンテキストノート: 画像座標から3次元位置への変換

**作成日**: 2026-10-20
**要件名**: so101-vlm-pickplace
**タスクID**: TASK-0008
**機能名(英語・ケバブ)**: pixel-to-base-projection
**タスク文書**: docs/tasks/so101-vlm-pickplace/TASK-0008.md

---

## 1. 技術スタック

- 言語: Python 3.12(`requires-python = ">=3.12,<3.13"`)
- パッケージ管理: uv(`uv sync`, `uv run pytest`)
- 数値計算: numpy(配列は float64 で扱う)
- シミュレータ: MuJoCo(カメラ描画・深度取得・レイキャストは `mujoco` パッケージ)
- テスト: pytest、pytest-cov(`--cov` でカバレッジ)
- アーキテクチャ: 純粋関数を `src/so101_vlm/perception/` に置き、ノード(`src/so101_vlm/nodes/`)や sim 依存と分離する。`perception` は MuJoCo に依存しない。統合テストだけ MuJoCo を使う。
- 座標系の前提
  - 画像: u = 列(右が正)、v = 行(下が正)、原点は左上の画素中心。
  - カメラ(本タスクの変換関数の出力): OpenCV 規約(x 右、y 下、z 前方、光軸 = +z)。
  - MuJoCo のカメラ: x 右、y 上、z 後方(光軸 = -z)。
  - ロボット座標(base): `T_base_cam` の移動先。
- 参照元:
  - pyproject.toml
  - CLAUDE.md
  - src/so101_vlm/nodes/camera.py
  - src/so101_vlm/eval/scene_spec.py

## 2. 開発ルール

### ユーザーのグローバルルール(~/.claude/rules/ecc/ 配下)より
- TDD: テストを先に書く(RED → GREEN → REFACTOR)。カバレッジ 80% 以上。
- 不変性: データは frozen な dataclass とし、配列は書き換えず新しく返す。入力配列を変更しない。
- 小さいファイル(200〜400 行目安、最大 800 行)、小さい関数(50 行未満)。
- エラーは握りつぶさず、呼び出し側が対処できるメッセージで ValueError を投げる。
- コミットは conventional commits(feat, fix, ...)。

### プロジェクト(CLAUDE.md・pyproject.toml)より
- テストは `uv run pytest -q`(現状 311 passed)。`uv run pytest --cov` でカバレッジ確認。
- `[tool.pytest.ini_options] testpaths = ["tests"]`。
- `[tool.coverage.run] source = ["so101_vlm"]`。
- アプリの起動・ビルド・DB 操作は該当なし。

### 追加ルールファイル
- AGENTS.md: 存在しない(プロジェクトルート)。
- docs/rule/: 存在しない(tdd サブディレクトリも無し)。
- 参照元: CLAUDE.md, pyproject.toml

### 本タスクで決めた事項(実装の決定)
- 座標変換の入出力は、常に OpenCV 規約のカメラ座標系で扱う。
- MuJoCo 規約との変換は、1 つの名前付き関数(`mujoco_cam_to_opencv`)に集約する。他の場所で符号を書かない。
- `T_base_cam` は OpenCV 規約のカメラ座標系から base への変換として定義する。MuJoCo 由来の行列を使う場合(TASK-0009)は、先に `mujoco_cam_to_opencv` を通す。

## 3. 関連実装

### 既存の類似実装
- カメラ描画: `src/so101_vlm/nodes/camera.py`
  - `render_rgbd(model, data, camera_name, width=640, height=480) -> (rgb uint8 HxWx3, depth float32 HxW [m])`
  - 深度は光軸方向の距離(z 深度)。遠方(何も写らない画素)は far 平面の値になる。
  - 戻り値は `.copy()` / `astype(..., copy=True)` で新しい配列を返す。
  - 引数検証: カメラ名が無い場合は `ValueError`。Renderer は `try/finally` で close する。
- シーン定数: `src/so101_vlm/eval/scene_spec.py`
  - `SCENE_SPEC`(frozen dataclass)。`block_half_size = 0.015`、`block_rest_z = 0.015`、`box_rim_z = 0.04`、`camera_name = "cam_top"`、`image_width = 640`、`image_height = 480` など。
  - 成功判定(TASK-0015)・初期配置(TASK-0006)・本タスクが共通で使う。
- 検証済みの投影: `docs/implements/so101-vlm-pickplace/TASK-0005/verify-report.md`
  - 既知の 3 次元点を投影した画素の深度が、描画深度と 1 mm 以内で一致することを確認済み。

### 本タスクで作るファイル(TDD の対象)
- `src/so101_vlm/perception/projection.py`(新規)
- `tests/perception/test_projection.py`(新規・単体テスト)
- `tests/perception/test_projection_scene.py`(新規・MuJoCo を使う統合テスト。TASK-0005 の実測値を再利用)

### 後続タスクから見た利用側
- TASK-0009(`src/so101_vlm/perception/calibration.py` を新規作成): `Camera` と `T_base_cam` を作り、`configs/calibration.yaml` に書き出す。
- TASK-0024: VLM の bbox → `bbox_to_base_point` → IK の結合。
- 参照元: docs/tasks/so101-vlm-pickplace/TASK-0009.md, docs/tasks/so101-vlm-pickplace/TASK-0024.md

### 関数の仕様(実装の目安)

| 関数 | 入力 | 出力 | 備考 |
|---|---|---|---|
| `camera_matrix(cam)` | `Camera` | 3x3 float64 K | `[[fx,0,cx],[0,fy,cy],[0,0,1]]` |
| `pixel_to_camera(u, v, depth, K)` | 画素 (float)、z 深度 [m]、K(3x3) | (3,) カメラ座標 | `x=(u-cx)*z/fx`, `y=(v-cy)*z/fy`, `z=depth`。depth は有限かつ > 0 |
| `camera_to_base(p_cam, T_base_cam)` | (3,)、4x4 | (3,) base 座標 | 同次座標で `T @ [p,1]` |
| `project_base_to_pixel(p_base, T_base_cam, cam)` | (3,)、4x4、Camera | (u, v, depth) | テスト用の順投影。z <= 0(カメラの後ろ)は ValueError |
| `mujoco_cam_to_opencv(T_base_cam_mj)` | 4x4 | 4x4 | 右から `diag(1,-1,-1,1)` を掛ける |
| `validate_T_base_cam(T)` | 任意 | 4x4 float64 のコピー | 下記「T の検証」 |
| `representative_depth(depth_image, bbox, *, shrink=0.5)` | HxW float、bbox | float [m] | 中央領域の有効深度の中央値 |
| `bbox_to_base_point(bbox, depth_image, cam, T_base_cam)` | bbox、深度画像、Camera、4x4 | (3,) base 座標 | 上 2 つの組み合わせ |

- 数値検証: 1 mm 以下が完了条件。往復テストは浮動小数の画素座標で行い、画素の四捨五入をしない。

## 4. 設計文書

### design.md 4.4 Perception(要点)
1. バウンディングボックスの中心画素 `(u, v)` を取る。外れ値を避けるため、ボックス内の深度の中央値を使う。
2. 内部パラメータ `fx, fy, cx, cy` で、カメラ座標系の点 `P_cam` を計算する。
3. 外部パラメータ `T_base_cam`(カメラ → baseframe の同次変換)で、ロボット座標系に変換する。
4. 変換行列は `configs/calibration.yaml` に置く。sim ではシーン定義から自動計算し、real では計測値を入れる。
5. このモジュールは純粋関数にして、合成データでテストする。
- 参照元: docs/design.md(4.4)

### requirements.md(関連)
- FR-04: 対象位置を受け取ったとき、深度とカメラパラメータからロボット座標系の 3 次元位置を算出する。
- NFR-05: 座標変換・IK・成功判定・JSON 検証などの純粋ロジックに単体テストを書く。
- 参照元: docs/requirements.md(FR-04, NFR-05)

### カメラ内部パラメータ(シーン)
- カメラ `cam_top`: 位置 (0.45, 0.30, 0.55)、(0.24, 0.10, 0) を注視、`fovy = 60°`、画像 640x480。
- `fy = (h/2) / tan(fovy/2)` = 240 / tan(30°) ≒ 415.7 px。正方画素なので `fx = fy`。
- 主点: `cx = (w-1)/2 = 319.5`、`cy = (h-1)/2 = 239.5`(画素中心を整数座標とする規約)。
  - 決定の理由: 描画画素の index は整数座標の中心を指す。`w/2`(=320) を使うと 0.5 px ずれ、0.6 m で約 0.7 mm の誤差になる。TASK-0009 でも同じ規約を使う。
- 参照元: assets/scene_pickplace.xml(67 行目付近の camera 定義)、src/so101_vlm/eval/scene_spec.py

### 座標系の変換(MuJoCo → OpenCV)
- MuJoCo: 光軸は -z。OpenCV: 光軸は +z。
- `P_cv = diag(1, -1, -1) @ P_mj`(x は同じ、y と z の符号を反転)。
- 描画の深度は z 深度(光軸方向)なので、`P_cv.z = depth` で成り立つ。
- 参照元: src/so101_vlm/nodes/camera.py(深度の定義)、docs/design.md(4.4)

## 5. テスト関連情報

### テストフレームワーク・設定
- `pyproject.toml`: pytest(`testpaths = ["tests"]`)、pytest-cov(`source = ["so101_vlm"]`)
- 実行: `uv run pytest -q`、`uv run pytest --cov`
- 参照元: pyproject.toml

### 既存テストの構成・命名
- `tests/test_scene.py`: シーン(TASK-0005)。MuJoCo を `from_xml_path(str(PICKPLACE_SCENE_PATH))` で読み込み、`SETTLE_STEPS` 回の `mj_step` で静定させる。定数名は大文字(`REST_TOL = 0.002`)。
- `tests/eval/`, `tests/policy/`, `tests/robot/`: 各モジュールごとのサブディレクトリ。
- 命名: `test_<対象>` 形式。`tests/perception/` は新規作成する。
- テストの書き方: AAA 形式(Given / When / Then のコメント)。
- 参照元: tests/test_scene.py, tests/eval/, tests/policy/, tests/robot/

### モック・ユーティリティ
- モックは使わず、合成データ(既知の T と点)で検証する。
- 統合テストでは実際の MuJoCo 描画を使う(`nodes/camera.py::render_rgbd`)。
- 参照元: src/so101_vlm/nodes/camera.py, src/so101_vlm/eval/scene_spec.py

### E2E テスト
- 該当なし(Web 画面なし、UI なし)。

### 再利用する実測値(TASK-0005 verify-report)
- 深度画像: 480x640、全画素有限・正。min 0.378 / max 0.893 / median 0.600 m。
- 赤ブロック画素 `(u=264, v=154)` の深度: 描画 0.6595 m、真値 0.6603 m、差 0.8 mm。
- 青ブロックの深度: 0.6068 / 真値 0.6072。緑ブロック: 0.6116 / 真値 0.6117。
- ブロック(一辺 3 cm、中心の高さ 0.015)の上面は z = 0.030 m。ブロック中心は各 (x, y, 0.015)(red (0.20, -0.06)、blue (0.30, 0.00)、green (0.22, 0.07))。
- 箱の中心 (0.20, 0.19)、内寸 8 cm、縁の高さ 0.04 m。
- 参照元: docs/implements/so101-vlm-pickplace/TASK-0005/verify-report.md、assets/scene_pickplace.xml

## 6. 注意事項

### 技術的制約
- 深度は top 面を見る。ブロックの 3 次元位置を `bbox_to_base_point` から得ると、その点は**上面**の点になる。
  - 把持の目標(ブロック中心)は、呼び出し側で `z - SCENE_SPEC.block_half_size(0.015 m)` と半辺を引いて得る(上面 z=0.030 → 中心 z=0.015。TASK-0012 / TASK-0024 の責務)。**訂正済み 2026-10-20**: 旧記載「`z + block_half_size` を足す」は符号が逆だった。
  - `projection.py` にはこの補正を入れない。理由: 投影は汎用の純粋関数で、ブロックの寸法はシーンの性質なので、呼び出し側に任せる。関数の docstring に明記する。
- 中心画素の深度: 箱の中心が背景(台)に当たる場合、深度は台の値になりうる。中央領域の中央値で影響を下げるが、完全には防げない。VLM の bbox 精度に依存する。
- 画素の量子化: 整数画素で深度を読むと、横方向に最大約 0.5 px × z / f ≒ 0.7 mm(0.6 m のとき)の誤差が出る。統合テストの許容差はこれを考慮する。

### 代表深度の決定(TDD 対象)
- 対象領域: bbox の中心を中心とし、幅・高さをそれぞれ `shrink`(既定 0.5)倍した領域。画素は `[x1, x2)` と `[y1, y2)` で整数化し、最低 1 画素を保証する。
- 有効深度: `np.isfinite(d) & (d > 0)`。
- 有効画素が 0 個なら `ValueError`(「bbox 内に有効な深度がありません」)。外側へのフォールバックはしない(隠れた挙動を避ける)。
- 中央値は `np.median`。入力配列は変更しない。

### bbox の検証
- `bbox_px = (x1, y1, x2, y2)`(画像画素、TASK-0017 で定義予定)。
- 検証: 4 つとも有限、`x1 < x2`、`y1 < y2`。違反は `ValueError`。
- 画像外へはみ出す場合は画像範囲にクリップする。クリップ後に空なら `ValueError`。
- 深度画像の形状が `(cam.height, cam.width)` と一致しない場合は `ValueError`。

### T_base_cam の検証(`validate_T_base_cam`)
- 形状が (4, 4)。すべて有限。
- 最下行が `[0, 0, 0, 1]`(許容誤差 1e-9)。
- 回転部 `R = T[:3, :3]` が直交: `R^T R ≈ I`(許容誤差 1e-6)。
- `det(R) ≈ +1`(鏡映を弾く)。
- 違反は `ValueError`。戻り値は float64 のコピー。

### Camera の検証
- `fx, fy > 0`、有限。`width, height` は正の整数。
- `frozen=True` の dataclass。

### 入力の扱い
- 関数は引数を変更しない。配列は新しく作って返す。
- NaN・inf を含む入力座標は `ValueError`。

### 完了条件と数値
- 往復誤差 ≤ 1 mm(TASK-0008 の完了条件)。
  - 単体テスト(合成データ、浮動小数の画素座標): 誤差は数値誤差レベル(1e-9 m 程度)を目標にし、許容は 1 mm。
  - 統合テスト(MuJoCo 描画): 整数画素の量子化があるので、許容は 2 mm と決める(RED 段階で値を再確認する)。
- 外れ値: 枠の一部に遠い深度を混ぜたとき、中央値の結果が真値から 1 mm 以内に収まる。

### セキュリティ・パフォーマンス
- 該当なし(外部入力は bbox と深度画像。ファイル・ネットワークは扱わない)。
- 深度の変換は numpy のベクトル演算で行い、ループを避ける。

### 参照元
- docs/tasks/so101-vlm-pickplace/TASK-0008.md
- docs/tasks/so101-vlm-pickplace/TASK-0009.md
- docs/tasks/so101-vlm-pickplace/TASK-0024.md
- docs/design.md
- docs/requirements.md
- docs/spec/so101-vlm-pickplace/note.md
- docs/implements/so101-vlm-pickplace/TASK-0005/verify-report.md
- src/so101_vlm/nodes/camera.py
- src/so101_vlm/eval/scene_spec.py
- assets/scene_pickplace.xml
- configs/calibration.yaml(TASK-0009 で作成予定。本タスクでは作らない)

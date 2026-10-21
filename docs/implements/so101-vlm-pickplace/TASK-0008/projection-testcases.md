# TASK-0008 テストケース定義書: 画像座標から3次元位置への変換(projection)

**作成日**: 2026-10-20
**要件名**: so101-vlm-pickplace
**タスクID**: TASK-0008
**機能名**: projection
**要件定義書**: `docs/implements/so101-vlm-pickplace/TASK-0008/projection-requirements.md`
**実測調査**: `docs/implements/so101-vlm-pickplace/TASK-0008/projection-survey.md`
**テストファイル(Red フェーズで作成)**:
- `tests/perception/test_projection.py`(単体。MuJoCo を使わない)
- `tests/perception/test_projection_scene.py`(MuJoCo 統合。module スコープのフィクスチャ)

信頼性: 🔵 資料にほぼそのまま書かれている / 🟡 資料からの妥当な推測、または実測で確定 / 🔴 資料にない決定

本書の数値のうち「実測」と書いたものは、リポジトリ外の使い捨てスクリプトで参照実装を動かして確かめた値(2026-10-20、MuJoCo 3.16.0)。

---

## 0. 共通の定数・ヘルパ(テストファイル内)

### 0.1 定数

| 名前 | 値 | 根拠 |
|---|---|---|
| `W`, `H` | 640, 480 | SCENE_SPEC 🔵 |
| `FOVY_DEG` | 60.0 | scene_pickplace.xml 🔵 |
| `FY` (= `FX`) | `240 / tan(30°)` = 415.6921938165307 | survey (a) 🟡 |
| `CX`, `CY` | 319.5, 239.5(`(w-1)/2`, `(h-1)/2`) | survey (a) 🟡 |
| `ROUNDTRIP_TOL_M` | 1e-3 | TASK-0008 完了条件 🔵 |
| `EXACT_TOL_M` | 1e-9 | 合成データの数値誤差の上限 🟡 |
| `SCENE_TOL_M` | 2e-3 | 整数画素の量子化(survey (d) 最大 1.34 mm)🟡 |
| `FLOOR_TOL_M` | 1e-4 | 主点規約の回帰(規約A 0.0076 mm / 規約B 0.50 mm)🟡 |
| `EUCLID_MIN_ERR_M` | 0.1 | z 深度/ユークリッド距離の判別(実測 184〜345 mm)🟡 |
| `SILHOUETTE_BIAS_RANGE_M` | (0.005, 0.015) | 特性記録用のゆるい範囲(実測 6.9〜10.2 mm)🔴 |

### 0.2 合成データのヘルパ(単体テスト)

- `CAMERA = Camera(fx=FY, fy=FY, cx=CX, cy=CY, width=W, height=H)`
- `T_OBLIQUE = look_at(eye=(0.45, 0.30, 0.55), target=(0.24, 0.10, 0.0), up=(0, 0, 1))`
  - OpenCV 規約: z = 正規化した (target − eye)、x = normalize(z × up)、y = z × x。シーンの `cam_top` と同じ位置・向き(実測で survey (b) の R と 1e-6 以内で一致)。
  - target は光軸上 → `project_base_to_pixel((0.24, 0.10, 0), T_OBLIQUE, CAMERA)` = `(319.5, 239.5, 0.6217716622683926)`(実測)。
- `T_DOWN`: R = `diag(1, -1, -1)`(真下を向く)、t = `(0.25, 0.0, 0.60)`。
- `plane_depth(T, d0)`: 平面 `z = d0`(base)を写した z 深度画像 (H, W) を解析的に作る。
  画素 (u, v) の光線 `r = R @ ((u-CX)/FY, (v-CY)/FY, 1)` に対し `depth = (d0 - t_z) / r_z`。
  - `plane_depth(T_DOWN, 0.03)` は全画素 0.57。`plane_depth(T_OBLIQUE, 0.0)` は 0.4769〜0.8931 m(実測)。
- `ray_plane_point(u, v, T, d0)`: 画素 (u, v) の光線と平面 z = d0 の交点(期待値の計算用。被テスト関数を使わない)。

### 0.3 統合テストのフィクスチャ(module スコープ)

- `settled`: `MjModel.from_xml_path(PICKPLACE_SCENE_PATH)` → `mj_step` を `SETTLE_STEPS = 500` 回(tests/test_scene.py と同じ)。
- `rgbd`: `render_rgbd(model, data, "cam_top", 640, 480)`。深度は float32。
- `segmentation`: `mujoco.Renderer.enable_segmentation_rendering()` で画素ごとの geom ID (H, W)。Renderer は `try/finally` で close。
- `calib`: テスト専用ヘルパ(要件定義書 4 章)で `(Camera, T_base_cam)` を作る。
  - `fy = (h/2)/tan(radians(model.cam_fovy[id])/2)`、`fx = fy`、`cx = (w-1)/2`、`cy = (h-1)/2`
  - `T_world_cam_mj` = `data.cam_xpos`, `data.cam_xmat`、`T_world_base` = `baseframe` サイトの `site_xpos`, `site_xmat`
  - `T_base_cam = mujoco_cam_to_opencv(inv(T_world_base) @ T_world_cam_mj)`
- 真値: ブロックの上面中心 = `data.geom_xpos[<color>_geom] + (0, 0, 0.015)`。静定後の中心 z は 0.01493 m(実測。0.07 mm 沈む)なので、XML の値ではなく静定後の値を使う。

### 0.4 実行時間の予算

| ファイル | 予算 | 実測の内訳 |
|---|---|---|
| `test_projection.py` | 合計 < 0.5 s | 1000 点の往復を含めても数十 ms 程度の見込み |
| `test_projection_scene.py` | 合計 < 3 s | モデル読込+静定 0.59 s、RGB-D 描画 0.67 s、セグメンテーション描画 0.16 s、計算 < 0.1 s(合計 1.44 s) |

描画はフィクスチャで 1 回ずつに限る。時間はアサートせず、`uv run pytest --durations=10` で確認する。

---

## 1. 正常系テストケース

### 1.1 `tests/perception/test_projection.py`

#### U-01 Camera を作って値を保持する 🟡
- **Given**: `fx=FY, fy=FY, cx=319.5, cy=239.5, width=640, height=480`
- **When**: `Camera(...)` を作る
- **Then**: 各フィールドが入力値と等しい。`camera.fx = 1.0` の代入は `dataclasses.FrozenInstanceError`
- **目的**: frozen dataclass であること(要件 2.1)。

#### U-02 camera_matrix の値(fovy 60°, 640x480) 🔵
- **Given**: `CAMERA`
- **When**: `K = camera_matrix(CAMERA)`
- **Then**: `K.shape == (3, 3)`、`K.dtype == float64`、`K == [[FY,0,319.5],[0,FY,239.5],[0,0,1]]`(1e-12)。`K[1,1] ≈ 415.6922`(abs 1e-4)
- **目的**: 式と主点規約 `(w-1)/2` の固定(要件 2.2、survey (a))。

#### U-03 主点の画素は光軸上 🟡
- **Given**: `u=CX, v=CY, depth=0.6`
- **When**: `pixel_to_camera(u, v, depth, CAMERA)`
- **Then**: `(0.0, 0.0, 0.6)`(1e-12)

#### U-04 pixel_to_camera の式(浮動小数の画素) 🔵
- **Given**: (a) `u=CX+FY, v=CY-FY/2, depth=2.0`、(b) `u=100.25, v=50.75, depth=0.5`
- **When**: `pixel_to_camera`
- **Then**: (a) `(2.0, -1.0, 2.0)`、(b) `((100.25-319.5)*0.5/FY, (50.75-239.5)*0.5/FY, 0.5)`(1e-12)
- **目的**: u = 列 → x、v = 行 → y の対応と、画素の丸めをしないこと。

#### U-05 camera_to_base の同次変換 🔵
- **Given**: (a) `T = I` と `p=(0.1,-0.2,0.3)`、(b) 平行移動のみ `t=(1,2,3)`、(c) `T_OBLIQUE` と `p=(0,0,0.6217716622683926)`
- **When**: `camera_to_base(p, T)`
- **Then**: (a) `p` と同じ、(b) `p + t`、(c) `(0.24, 0.10, 0.0)`(いずれも `EXACT_TOL_M`)

#### U-06 project_base_to_pixel(光軸上の点) 🟡
- **Given**: `p_base=(0.24, 0.10, 0.0)`、`T_OBLIQUE`、`CAMERA`
- **When**: `project_base_to_pixel(...)`
- **Then**: `(319.5, 239.5, 0.6217716622683926)`(画素 1e-9、深度 1e-12)。戻り値は長さ 3 の tuple で各要素が Python の `float`
- **目的**: 順投影の式と戻り値の型(要件 2.2)。

#### U-07 往復(画素 → base → 画素)、乱数 1000 点 + 隅 🔵
- **Given**: `np.random.default_rng(0)`。`u ~ U[0, 639]`、`v ~ U[0, 479]`、`depth ~ U[0.3, 0.9]` を 1000 点。加えて `(0,0)`, `(639,0)`, `(0,479)`, `(639,479)`, `(319.5,239.5)`, `(-0.5,-0.5)`, `(639.5,479.5)` × depth `{0.3, 0.9}`。`T_OBLIQUE`
- **When**: `P = camera_to_base(pixel_to_camera(u, v, d))` → `(u2, v2, d2) = project_base_to_pixel(P)` → `P2 = camera_to_base(pixel_to_camera(u2, v2, d2))`
- **Then**: 全点で `‖P2 − P‖ ≤ 1 mm`(アサート)。補助として `|u2−u|, |v2−v| ≤ 1e-6 px`、`|d2−d| ≤ 1e-9 m`。実測の最大誤差 1.4e-13
- **目的**: 完了条件「往復誤差 1 mm 以下」。浮動小数の画素のまま往復する(丸めない)。

#### U-08 往復(base → 画素 → base)、乱数 1000 点 🔵
- **Given**: `rng(1)`。`x ~ U[0.10, 0.40]`、`y ~ U[-0.15, 0.25]`、`z ~ U[0.0, 0.10]`(作業領域と箱を含む)。`T_OBLIQUE`
- **When**: `project_base_to_pixel` → `pixel_to_camera` → `camera_to_base`
- **Then**: `‖P' − P‖ ≤ 1 mm`(実測最大 2.2e-16 m)

#### U-09 往復(非正方画素・主点が中心でないカメラ) 🟡
- **Given**: `Camera(fx=500, fy=480, cx=300.3, cy=250.7, width=800, height=600)`、`T_OBLIQUE`、U-07 と同じ方法で 200 点
- **Then**: `‖P2 − P‖ ≤ 1 mm`、`|u2−u|, |v2−v| ≤ 1e-6`
- **目的**: `fx`/`fy`、`cx`/`cy` の取り違えを検出する(シーンのカメラは fx=fy なので U-07 では見えない)。

#### U-10 mujoco_cam_to_opencv: 単位行列 🟡
- **Given**: `np.eye(4)`
- **Then**: `diag(1, -1, -1, 1)`

#### U-11 mujoco_cam_to_opencv: 2 回で元に戻る・平行移動は不変 🟡
- **Given**: `T_mj = T_OBLIQUE @ diag(1,-1,-1,1)`
- **When**: `T_cv = mujoco_cam_to_opencv(T_mj)`
- **Then**: `T_cv ≈ T_OBLIQUE`(1e-12)、`mujoco_cam_to_opencv(T_cv) ≈ T_mj`、`T_cv[:3,3] == T_mj[:3,3]`

#### U-12 mujoco_cam_to_opencv: 前方 1 m の点 🟡
- **Given**: U-11 の `T_mj`
- **Then**: `T_mj @ (0, 0, -1, 1)`(MuJoCo の前方)と `T_cv @ (0, 0, 1, 1)`(OpenCV の前方)が一致(1e-12)
- **目的**: 光軸の符号(要件 5.3、survey (b))。

#### U-13 validate_T_base_cam: 正しい T を float64 のコピーで返す 🟡
- **Given**: (a) `T_OBLIQUE`、(b) `np.eye(4, dtype=int)`、(c) `T_OBLIQUE.tolist()`(ネストしたリスト)
- **Then**: 値が入力と等しい、`dtype == float64`、`shape == (4, 4)`、(a) で `np.shares_memory(out, T_OBLIQUE) is False`

#### U-14 representative_depth: 中央領域だけを読む 🔵/🟡
- **Given**: `D = np.full((480, 640), 9.0)`、`D[105:115, 105:115] = 0.6`、`bbox = (100, 100, 120, 120)`、`shrink=0.5`
  (中央領域: 中心 110、半幅 5 → 行・列 `[105, 115)` の 100 画素)
- **When**: `representative_depth(D, bbox)`
- **Then**: `0.6`(厳密に一致)。枠の外周(9.0)が混ざらない
- **目的**: 中央領域の決め方(要件 2.5)。

#### U-15 representative_depth: 外れ値に強い(中央値) 🔵
- **Given**: U-14 の中央領域 100 画素のうち 49 画素を 5.0 m に置き換える
- **Then**: `|result − 0.6| ≤ 1 mm`(実測 0.6 ちょうど)
- **補足(同じテスト内で特性を記録)**: 50 画素を置き換えると 2.8(= (0.6+5.0)/2)。中央値が守れるのは「半分未満」までであることを示す。🟡

#### U-16 representative_depth: NaN・inf・0・負を無視する 🔵
- **Given**: 中央領域 100 画素のうち 33 画素 NaN、33 画素 0.0、33 画素 −1.0、1 画素 0.6
- **Then**: `0.6`。さらに残り 1 画素以外に `+inf` を混ぜても 0.6

#### U-17 representative_depth: shrink=1.0 で bbox 全体 🟡
- **Given**: U-14 の画像、`shrink=1.0`(領域は `[100,120)` の 400 画素。0.6 が 100、9.0 が 300)
- **Then**: `9.0`
- **目的**: `shrink` が領域に効くこと。上限 1.0 を含むこと。

#### U-18 representative_depth: 1x1 bbox は (行 v, 列 u) を読む 🔵
- **Given**: `D = full(…, 9.0)`、`D[20, 10] = 0.5`、`D[10, 20] = 0.7`、`bbox = (10, 20, 11, 21)`
- **Then**: `0.5`
- **目的**: 行と列の取り違え(`D[u, v]`)の検出。半開区間の bbox で 1 画素を指定できること。

#### U-19 bbox_to_base_point: 真下を向くカメラ・平面 z=0.03(厳密) 🔵
- **Given**: `T_DOWN`、`D = plane_depth(T_DOWN, 0.03)`(全画素 0.57)
- **When/Then**:
  - `bbox = (300, 220, 340, 260)` → 中心画素 `(319.5, 239.5)` → `(0.25, 0.0, 0.03)`(`EXACT_TOL_M`)
  - `bbox = (100, 50, 121, 71)` → 中心画素 `(110, 60)` → `(-0.03726784, 0.24613164, 0.03)`(`EXACT_TOL_M`。期待値は `ray_plane_point(110, 60, T_DOWN, 0.03)` で計算)
- **目的**: 中心画素 `((x1+x2-1)/2, (y1+y2-1)/2)` の規約と、合成の流れの正しさ。

#### U-20 bbox_to_base_point: 表面の点を返し、ブロック中心への補正をしない 🔵
- **Given**: U-19 の 1 つ目
- **Then**: `result[2] ≈ 0.03`(上面の高さ)。`0.015`(中心)や `0.045` にならない
- **目的**: 補正は呼び出し側の責務(要件 2.6。`p[2] - block_half_size`)。

#### U-21 bbox_to_base_point: 斜めのカメラ・平面 z=0 🟡
- **Given**: `T_OBLIQUE`、`D = plane_depth(T_OBLIQUE, 0.0)`
- **When/Then**:
  - `bbox = (300, 220, 340, 260)` → `(0.24, 0.10, 0.0)` から ≤ 1 mm(実測 0.0003 mm)
  - `bbox = (0, 0, 40, 40)` → `ray_plane_point(19.5, 19.5, T_OBLIQUE, 0)` から ≤ 1 mm(実測 0.0009 mm)
  - `bbox = (600, 440, 640, 480)` → `ray_plane_point(619.5, 459.5, …)` から ≤ 1 mm(実測 0.0002 mm)
- **目的**: 傾いた面でも中央値の深度で 1 mm 以内に収まること。(深度が場所で変わるので厳密一致ではない。)

#### U-22 bbox_to_base_point: shrink を渡せる 🟡
- **Given**: U-14 の画像(中央 0.6、外周 9.0)、`T_DOWN`、bbox `(100,100,120,120)`
- **Then**: `shrink=0.5` の結果の z 成分は深度 0.6 に、`shrink=1.0` では深度 9.0 に対応する(`result` を `pixel_to_camera(109.5, 109.5, d)` → `camera_to_base` と比較、`EXACT_TOL_M`)

### 1.2 不変性・決定性・依存(`test_projection.py`)

#### U-23 入力を変更しない 🔵
- **Given**: 深度画像、`T_OBLIQUE`、`p`(ndarray)、bbox(list)。呼ぶ前に `copy.deepcopy` を取る
- **When**: 全公開関数(`camera_matrix`、`pixel_to_camera`、`camera_to_base`、`project_base_to_pixel`、`validate_T_base_cam`、`mujoco_cam_to_opencv`、`representative_depth`、`bbox_to_base_point`)を呼ぶ
- **Then**: すべての入力がコピーと `array_equal`(NaN を含む深度画像は `equal_nan=True`)。`flags.writeable` も変わらない

#### U-24 読み取り専用の入力を受け付ける 🟡
- **Given**: `D.setflags(write=False)`、`T.setflags(write=False)`、`p.setflags(write=False)`
- **Then**: 全関数が例外なく動く(内部でその場の書き換えをしていない)

#### U-25 戻り値は新しい書き込み可能な float64 配列 🟡
- **Then**: `camera_matrix`、`pixel_to_camera`、`camera_to_base`、`validate_T_base_cam`、`mujoco_cam_to_opencv`、`bbox_to_base_point` の戻り値が `np.ndarray`、`dtype == float64`、`flags.writeable`、入力と `np.shares_memory` しない。`camera_matrix` を 2 回呼ぶと別オブジェクトで、片方を書き換えても次の呼び出しに影響しない。`representative_depth` は Python の `float`

#### U-26 決定性 🔵
- **Then**: 同じ入力で 2 回呼ぶと、配列はビット単位で一致(`np.array_equal`)

#### U-27 float32 の深度画像を受け付ける 🟡
- **Given**: U-14 の画像を `astype(np.float32)`(`render_rgbd` の出力型)
- **Then**: `representative_depth` が `float` を返し、`float(np.float32(0.6))` と一致。`bbox_to_base_point` の戻り値は float64

#### U-28 projection.py は MuJoCo に依存しない 🔵
- **Given**: `src/so101_vlm/perception/projection.py` のソース
- **When**: `ast` で import 文を集める
- **Then**: トップレベルのモジュール名が `{"numpy", "math", "dataclasses", "typing", "__future__", "collections", "numbers"}` の部分集合。`mujoco` を含まない
- **目的**: design.md 4.4「純粋関数」。

---

## 2. 異常系テストケース(すべて `ValueError`)

`TypeError` は使わない(要件 2.7)。メッセージの文言はアサートしない(要件で決めていないため。🔴 は付けず、未決事項 O-4 に記録)。

| ID | 対象 | 入力 | 理由 | 信頼性 |
|---|---|---|---|---|
| U-29 | `Camera` | `fx` または `fy` が `0`, `-1.0`, `nan`, `inf` | 焦点距離は有限かつ正 | 🟡 |
| U-30 | `Camera` | `width` または `height` が `0`, `-640`, `640.0`(float), `True`, `"640"` | 正の整数。bool・float は不可 | 🔴 |
| U-31 | `Camera` | `fx=True`、`cx=nan`、`cy=inf` | bool 不可、主点は有限 | 🔴 |
| U-32 | `pixel_to_camera` | `depth` が `0.0`, `-0.1`, `nan`, `inf`, `-inf` | 完了条件「不正な深度でエラー」 | 🔵 |
| U-33 | `pixel_to_camera` | `u` または `v` が `nan`, `inf`、`u=True` | 非有限の画素・bool | 🟡 / bool 🔴 |
| U-34 | `camera_to_base` | `p` の形状 `(2,)`, `(4,)`, `(3,1)`、要素に `nan`/`inf` | `(3,)` の有限ベクトル | 🔴(形状)/ 🟡(有限) |
| U-35 | `camera_to_base` | `T` の形状 `(3,4)`、`T` に `nan`、`T` の回転部 `diag(1,1,-1)` | T の検証(`validate_T_base_cam` を通す) | 🔴(O-1) |
| U-36 | `project_base_to_pixel` | カメラの後ろの点 `eye − 0.1·z軸`、カメラと同一平面の点 `eye + 0.1·x軸`(カメラ座標で z = 0) | `z <= 0` は投影できない | 🟡 |
| U-37 | `project_base_to_pixel` | `p_base` に `nan` | 非有限 | 🟡 |
| U-38 | `validate_T_base_cam` | 形状 `(3,3)`, `(4,3)`, `(3,4)`, `(16,)`, `(4,4,1)`, スカラー | 形状 (4,4) | 🟡 |
| U-39 | `validate_T_base_cam` | 要素に `nan` / `inf`、文字列の配列(`np.full((4,4), "a")`)、複素数の配列 | 有限の実数 | 🟡 / 複素数 🔴 |
| U-40 | `validate_T_base_cam` | 回転部 `diag(1, 1, -1)`(det = −1、鏡映) | `det(R) ≈ +1` | 🟡 |
| U-41 | `mujoco_cam_to_opencv` | det = −1 の T、形状 `(3,3)`。呼んだ後も入力が変わらない | 入力は `validate_T_base_cam` で検証 | 🟡 |
| U-42 | `representative_depth` | 中央領域の 100 画素がすべて無効(NaN / 0 / −1 / inf の混在)。外周には有効な 9.0 がある | 有効画素 0 個はエラー。外側へフォールバックしない | 🔵 / フォールバックなし 🟡 |
| U-43 | `representative_depth` | bbox が `(10,10,10,20)`(x1==x2)、`(10,10,20,10)`(y1==y2)、`(20,10,5,30)`(反転)、`(10,30,20,5)` | 面積 0・反転 | 🔴 |
| U-44 | `representative_depth` | bbox が完全に画像外 `(700,10,710,20)`、`(-20,-20,-5,-5)`、端に接するだけ `(640,0,650,10)`、`(-5,0,0,10)` | クリップ後に面積 0 | 🔴 |
| U-45 | `representative_depth` | bbox の要素数 3・5、要素に `nan`/`inf`、`(True, 0, 10, 10)`、`("0", 0, 10, 10)`、`None` | 長さ 4 の有限の実数。bool 不可 | 🔴 |
| U-46 | `representative_depth` | `shrink` が `0`, `-0.5`, `1.0000001`, `1.5`, `nan`, `inf`, `True` | `0 < shrink <= 1`。`True` は 1 と等しいが bool として弾く | 🔴 |
| U-47 | `representative_depth` | 深度画像が 1 次元 `(640,)`、3 次元 `(480,640,1)`、空 `(0,0)`、`dtype=bool`、複素数、文字列 | 2 次元・実数・1x1 以上 | 🟡 / dtype 🔴 |
| U-48 | `bbox_to_base_point` | 深度画像の形状 `(480, 639)`、転置 `(640, 480)` | `(camera.height, camera.width)` と一致 | 🟡 |
| U-49 | `bbox_to_base_point` | `T` の回転部 det = −1 | `validate_T_base_cam` を通す | 🔵 |
| U-50 | `bbox_to_base_point` | 中央領域の深度がすべて NaN | `representative_depth` のエラーが伝わる | 🔵 |

---

## 3. 境界値テストケース

#### U-51 Camera の境界 🔴
- `fx = fy = 1e-9`(小さい正の値)は受け付ける。`cx = -100`、`cy = 1000`(画像外の主点)も受け付ける(実機キャリブレーションでありうる。要件 2.1)。`width = np.int64(640)` は受け付ける。

#### U-52 pixel_to_camera の境界 🟡
- `depth = 1e-9`(小さい正)は受け付ける。`u = -100.0`、`u = 1000.0`(画像外)も値を返す(範囲判定は呼び出し側。`project_base_to_pixel` と対称)。🔴(画像外の扱い)

#### U-53 project_base_to_pixel: 画像外の点も値を返す 🟡
- `T_OBLIQUE` で、カメラ前方だが画面外の点 `(0.24, 0.10, 0.0) + 2.0·x軸` → `u > 640` の値を例外なしで返す。

#### U-54 validate_T_base_cam: 最下行の許容誤差 1e-9 🟡
- `[0,0,0,1+1e-10]` は通る。`[0,0,0,1+1e-8]`、`[1e-8,0,0,1]` は `ValueError`。

#### U-55 validate_T_base_cam: 直交性の許容誤差 1e-6 🟡
- 回転部を `(1+1e-7)` 倍(`max|RᵀR−I| ≈ 2e-7`)は通る。`(1+1e-5)` 倍(≈ 2e-5)は `ValueError`。せん断 `R[0,1] += 1e-3` は `ValueError`。

#### U-56 validate_T_base_cam: 変換の掛け忘れは弾けない(制限の記録) 🟡
- `T_OBLIQUE @ diag(1,-1,-1,1)`(MuJoCo 規約のまま)は det = +1 なので**通る**。テスト名に「limitation」を入れ、docstring に「規約の取り違えは T の検証では検出できない。統合テスト S-02 で検出する」と書く(要件 2.3)。

#### U-57 representative_depth: 中央領域の画素範囲(表) 🟡
各行について、(i) 期待領域だけ NaN・他は 0.5 の画像で `ValueError`(領域 ⊆ 期待)、(ii) 期待領域の 4 隅の画素を 1 つずつ 0.5・他は全部 NaN の画像で 0.5 を返す(領域 ⊇ 4 隅)。両方で領域が期待どおりと確定する。

| bbox | shrink | 期待領域 行 `[r0,r1)` × 列 `[c0,c1)` | 中心画素 `(u_c, v_c)` | 意味 | 信頼性 |
|---|---|---|---|---|---|
| `(100,100,120,120)` | 0.5 | `[105,115)` × `[105,115)` | (109.5, 109.5) | 基本 | 🟡 |
| `(0,0,4,4)` | 0.5 | `[1,3)` × `[1,3)` | (1.5, 1.5) | 小さい bbox・画像の角 | 🟡 |
| `(10,20,11,21)` | 0.5 | `[20,21)` × `[10,11)` | (10, 20) | 1x1(最低 1 画素) | 🟡 |
| `(100,100,121,121)` | 1e-6 | `[110,111)` × `[110,111)` | (110, 110) | 極小 shrink・奇数幅 → 1 画素 | 🟡 |
| `(100,100,120,120)` | 1e-6 | `[109,111)` × `[109,111)` | (109.5, 109.5) | 極小 shrink・偶数幅 → 2x2 | 🟡 |
| `(100,100,120,120)` | 1.0 | `[100,120)` × `[100,120)` | (109.5, 109.5) | 上限 | 🟡 |
| `(10.2,20.7,30.4,40.1)` | 0.5 | `[25,36)` × `[15,26)` | (19.8, 29.9) | 浮動小数の bbox(floor/ceil) | 🔴 |
| `(-10,-10,10,10)` | 0.5 | `[2,8)` × `[2,8)` | (4.5, 4.5) | 左上にはみ出し → クリップ後 `(0,0,10,10)` | 🔴 |
| `(630,470,650,490)` | 0.5 | `[472,478)` × `[632,638)` | (634.5, 474.5) | 右下にはみ出し → クリップ後 `(630,470,640,480)` | 🔴 |

(期待値はすべて参照実装で確認済み。)

#### U-58 bbox_to_base_point: クリップ後の中心を使う 🔴
- **Given**: `T_OBLIQUE`、`plane_depth(T_OBLIQUE, 0.0)`、`bbox = (-10, -10, 30, 30)`
- **Then**: `ray_plane_point(14.5, 14.5, T_OBLIQUE, 0)`(クリップ後 `(0,0,30,30)` の中心)から ≤ 1 mm(実測 0.0009 mm)。クリップ前の中心 (9.5, 9.5) ではない(両者の差は数 mm 以上あることも確認する)

#### U-59 bbox の画像全体 🟡
- `bbox = (0, 0, 640, 480)` は受け付ける(クリップなし。中心 (319.5, 239.5))。`T_DOWN` の平面で `(0.25, 0.0, 0.03)`(`EXACT_TOL_M`)。

---

## 4. 統合テスト(`tests/perception/test_projection_scene.py`)

すべて module スコープのフィクスチャ(0.3)を共有する。合計 < 3 s(実測 1.44 s)。

#### S-01 テスト用ヘルパの K と T 🟡
- **Then**: `Camera` が `(FY, FY, 319.5, 239.5, 640, 480)`(FY は 1e-9)。`T_base_cam` が `validate_T_base_cam` を通り、平行移動が `(0.45, 0.30, 0.55)`(1e-9)。`baseframe` サイトがワールド原点・単位回転(実測)。

#### S-02 床の逆投影(主点規約の回帰) 🟡
- **Given**: 8 px 間隔の画素格子(80x60 = 4800 点)のうち、セグメンテーションで geom `floor` の画素(実測 4374 点)。整数画素と描画深度
- **When**: `camera_to_base(pixel_to_camera(u, v, depth[v, u], camera), T_base_cam)`
- **Then**: `max |z| ≤ FLOOR_TOL_M = 0.1 mm`(実測 0.0076 mm)
- **目的**: `cx=(w-1)/2` 規約と z 深度・座標変換の符号をまとめて守る。

#### S-03 負の対照: `w/2` 規約なら S-02 は落ちる 🟡
- **Given**: S-02 と同じ点。ただし `Camera(cx=320.0, cy=240.0, …)`
- **Then**: `max |z| > FLOOR_TOL_M`(実測 0.50 mm)
- **目的**: S-02 の許容誤差が規約の違いを検出できる感度を持つことの証明。テスト内のコメントに規約A 0.0076 mm / 規約B 0.50 mm を書く。

#### S-04 3 ブロックの上面中心(整数画素) 🟡
- **Given**: 各ブロックの上面中心(0.3 の真値)を `project_base_to_pixel` で投影し、四捨五入した画素 `(u_i, v_i)` を 1x1 の bbox `(u_i, v_i, u_i+1, v_i+1)` にする。画素は red (264,153)、blue (242,214)、green (314,209)(実測)
- **When**: `bbox_to_base_point(bbox, depth, camera, T_base_cam)`
- **Then**: 真値から ≤ `SCENE_TOL_M = 2 mm`(実測 red 0.285 / blue 0.986 / green 0.274 mm)。`parametrize` で 3 色

#### S-05 上面だけの bbox の精度(要件として 2 mm) 🟡
- **Given**: 上面の 4 隅 `(±0.015, ±0.015, 0)` を投影し、`x1 = round(min u)`、`y1 = round(min v)`、`x2 = round(max u) + 1`、`y2 = round(max v) + 1`(半開区間)。実測 red (251,143,278,165)、blue (227,202,257,228)、green (300,197,330,222)
- **When**: `bbox_to_base_point(bbox, …, shrink=0.5)`
- **Then**: 上面中心の真値から ≤ 2 mm(実測 0.761 / 0.142 / 0.615 mm)
- **目的**: 「bbox が上面を正しく囲めば 2 mm 以内」という TASK-0024 の前提を固定する。

#### S-06 箱の底板の中心 🟡
- **Given**: `box_floor` の上面中心 = `geom_xpos(box_floor) + (0,0,0.0025)` = (0.20, 0.19, 0.005)。投影 (383.56, 258.11) → 画素 (384, 258)。その画素がセグメンテーションで `box_floor` であることを前提として確認する
- **Then**: `bbox_to_base_point((384,258,385,259), …)` が真値から ≤ 2 mm(実測 0.674 mm)

#### S-07 把持中心への補正の符号(上面 → 中心は引く) 🟡
- **Given**: S-04 の結果 `p_surface`
- **Then**: `p_surface[2] - SCENE_SPEC.block_half_size` がブロック中心 z(`geom_xpos[2]` ≈ 0.01493)から ≤ 2 mm。`+` にすると 30 mm ずれる
- **目的**: 呼び出し側の補正規約(要件 2.6、note.md の訂正)を例として固定する。補正は projection.py に入れない。

#### S-08 深度は z 深度(ユークリッド距離ではない) 🟡
- (a) S-02 の床の点で、描画深度と「光線と平面 z=0 の交点の解析的な z 深度」の差の最大が ≤ 0.1 mm(survey 1.24e-5 m)
- (b) 画像の隅の床画素 (0,0), (639,0), (0,479), (639,479) で、深度をユークリッド距離と解釈した点(`depth · ray/‖ray‖`)と正しい点の差が > `EUCLID_MIN_ERR_M = 100 mm`(実測 345.3 / 345.3 / 184.4 / 184.4 mm)。前提として 4 画素が床であることを確認する
- **目的**: z 深度の規約の取り違えを検出する。画像中心付近では差が小さい(中心 0 mm)ので隅で判定する。

#### S-09 【特性記録】見えている輪郭全体の bbox は上面中心から 7〜10 mm ずれる 🔴
- **位置づけ**: **要件ではない**。TASK-0008 の変換は正しく、「bbox の中心 = 上面の中心」という仮定の誤差(survey 補足)。TASK-0024 への警告として値を記録する。テスト名は `test_characterize_silhouette_bbox_bias_for_task0024`、docstring 冒頭に「特性記録(要件ではない)」と書く。
- **Given**: セグメンテーションで各ブロックの geom の全画素を取り、その外接 bbox(半開区間)。実測 red (251,144,279,176)、blue (228,203,259,237)、green (300,197,329,231)
- **When**: `bbox_to_base_point(bbox, …, shrink=0.5)`
- **Then**: 上面中心との**水平**誤差が `5 mm ≤ e_xy ≤ 15 mm`(実測 red 10.17 / blue 7.43 / green 6.92 mm)。z 誤差は負(実測 −3.3 / −2.4 / −1.7 mm。側面の深度が混ざり低く出る)
- **意図**: 下限があるので、シーンやカメラが変わって偏りが消えたときにもテストが落ち、記録を更新するきっかけになる。上限は暴走の検出。S-05(上面の bbox は 2 mm 以内)と対で読む。

---

## 5. 開発言語・フレームワーク

- **プログラミング言語**: Python 3.12 🔵(pyproject.toml)
  - numpy のベクトル演算で、往復 1000 点や画素格子を一括で検証できる。
- **テストフレームワーク**: pytest(`testpaths = ["tests"]`)、pytest-cov 🔵
  - `parametrize` で 3 色・異常入力の表を回す。統合テストは module スコープのフィクスチャで描画を 1 回にする。
  - 実行: `uv run pytest tests/perception -q`、`uv run pytest --cov`。
  - 統合テストは `mujoco.Renderer`(オフスクリーン)を使う。既存の `tests/test_scene.py` と同じ環境で動く。
- モックは使わない。合成データ(解析的な平面の深度画像)と実際の MuJoCo 描画で検証する。🔵(note.md 5 章)

---

## 6. テストケース実装時の日本語コメント指針

既存テスト(`tests/policy/test_ik.py`、`tests/test_scene.py`)に合わせる。

- モジュール docstring: 対象・TASK 番号・本書へのパス・信頼性記号の凡例。
- 各テストの docstring に ID と要約: `"""U-07: 浮動小数画素の往復誤差が 1 mm 以下。🔵"""`。
- 本文は Given / When / Then のコメントで区切る。

```python
def test_roundtrip_pixel_base_pixel_within_1mm():
    """U-07: 乱数 1000 点 + 隅の往復誤差が 1 mm 以下。🔵"""
    # Given: 浮動小数の画素と深度 0.3〜0.9 m(丸めない)
    # When: 画素 → カメラ → base → 画素 → カメラ → base
    # Then: 3 次元の誤差 ≤ 1 mm(実測 1e-13 m 台)
```

- 特性記録(S-09)は docstring の 1 行目で「特性記録(要件ではない)」と明記し、実測値と TASK-0024 への申し送りを書く。
- 負の対照(S-03)はコメントで「このテストが落ちたら S-02 の許容誤差が甘すぎる」と書く。
- 定数は大文字でファイル冒頭にまとめる(`ROUNDTRIP_TOL_M` など)。

---

## 7. 要件定義との対応関係

| 要件(projection-requirements.md) | テストケース |
|---|---|
| 1 章 座標規約(画素中心・OpenCV・z 深度) | U-02, U-03, U-04, U-12, U-18, S-02, S-03, S-08 |
| 2.1 `Camera` | U-01, U-29〜U-31, U-51 |
| 2.2 `camera_matrix` / `pixel_to_camera` / `camera_to_base` / `project_base_to_pixel` | U-02〜U-09, U-32〜U-37, U-52, U-53 |
| 2.2 `mujoco_cam_to_opencv` | U-10〜U-12, U-41 |
| 2.3 `validate_T_base_cam` | U-13, U-38〜U-40, U-54〜U-56 |
| 2.4 bbox(半開区間・中心画素・クリップ) | U-18, U-19, U-43〜U-45, U-57〜U-59 |
| 2.5 `representative_depth` | U-14〜U-17, U-42, U-46, U-47, U-57 |
| 2.6 `bbox_to_base_point`(表面の点・補正は呼び出し側) | U-19〜U-22, U-48〜U-50, U-58, S-04〜S-07 |
| 2.7 不変性・決定性・float64・ValueError | U-23〜U-27、2 章全体 |
| 3 章 精度(往復 1 mm、統合 2 mm、床 0.1 mm、外れ値 1 mm) | U-07〜U-09, U-15, U-21, S-02, S-04〜S-06 |
| 3 章 純粋関数・MuJoCo 非依存 | U-28 |
| 4 章 統合テストのヘルパ | 0.3, S-01 |
| 6 章 範囲外(輪郭 bbox の誤差) | S-09(特性記録) |
| TASK-0008 完了条件 | 往復 1 mm: U-07, U-08 / 中央値: U-14〜U-16 / 不正な深度でエラー: U-32, U-42 |

- **参照した機能概要**: 要件定義書 1 章、design.md 4.4
- **参照した入力・出力仕様**: 要件定義書 2 章
- **参照した制約条件**: 要件定義書 3 章、survey (a)〜(d)
- **参照した使用例**: 要件定義書 5 章(5.3, 5.4)

---

## 8. 件数と信頼性の分布

| 分類 | 件数 | ID |
|---|---|---|
| 正常系(単体) | 22 | U-01〜U-22 |
| 不変性・決定性・依存(単体) | 6 | U-23〜U-28 |
| 異常系(単体) | 22 | U-29〜U-50 |
| 境界値(単体) | 9 | U-51〜U-59 |
| 統合・正常系 | 7 | S-01, S-02, S-04〜S-08 |
| 統合・負の対照 | 1 | S-03 |
| 統合・特性記録 | 1 | S-09 |
| **合計** | **68** | (parametrize 展開後はおよそ 150) |

信頼性(主な記号で数えた概数): 🔵 22 / 🟡 33 / 🔴 13。🔴 は bbox の細かい規約・bool の扱い・形状の厳しさなど、要件定義書 8 章で「ユーザー確認なしで決めた」項目に対応する。

---

## 9. 未決事項(Red フェーズで決める・確認する)

- **O-1** `camera_to_base` が T を `validate_T_base_cam` で完全に検証するか(U-35)。本書は「検証する」を推奨(`bbox_to_base_point` と一貫させる)。🔴
- **O-2** 整数型(例: uint16 の mm 単位)の深度画像を受け付けるか。要件定義書は「実数型」とだけ書く。本書ではテストしない。
- **O-3** `p` の形状 `(3,1)` / `(1,3)` を弾くか(U-34)。本書は「`(3,)` のみ」とした。🔴
- **O-4** エラーメッセージの文言(どの条件かを含める)は要件にあるが、文言は未定なのでアサートしない。必要なら Green 後に `match=` を追加する。
- **O-5** 統合テストのセグメンテーション描画(`enable_segmentation_rendering`)はテスト内でだけ使う。描画 1 回で 0.16 s。
- **O-6** S-09 の範囲 5〜15 mm はシーン・カメラの変更で外れうる。その場合は survey と TASK-0024 の記録を更新してから範囲を直す。

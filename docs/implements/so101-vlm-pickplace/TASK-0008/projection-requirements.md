# TASK-0008 要件定義書: 画像座標から3次元位置への変換(projection)

**作成日**: 2026-10-20
**要件名**: so101-vlm-pickplace
**タスクID**: TASK-0008
**機能名**: projection(画像座標 → カメラ座標 → ロボット座標)
**実装ファイル**: `src/so101_vlm/perception/projection.py`(新規)
**テストファイル**: `tests/perception/test_projection.py`(単体)、`tests/perception/test_projection_scene.py`(MuJoCo 統合)
**関連文書**: `docs/requirements.md`(FR-04, NFR-05, AC-05, AC-07)、`docs/design.md`(4.3, 4.4, 5)、
`docs/tasks/so101-vlm-pickplace/TASK-0008.md` / `TASK-0009.md` / `TASK-0024.md`、
`docs/implements/so101-vlm-pickplace/TASK-0008/note.md`、`docs/implements/so101-vlm-pickplace/TASK-0008/projection-survey.md`

信頼性: 🔵 資料にほぼそのまま書かれている / 🟡 資料からの妥当な推測(または実測で確定) / 🔴 資料にない決定

---

## 1. 機能の概要

- 🔵 **何をするか**: VLM が返すバウンディングボックス(`bbox_px`)と深度画像から、ロボット座標系(`baseframe`)の 3 次元位置を求める純粋関数群(FR-04、design.md 4.4)。
- 🔵 **解決する問題**: 画像上の検出結果を IK の目標位置にする(FR-04 → FR-05)。
- 🔵 **利用者**: 後続モジュール。TASK-0009(キャリブレーション生成)、TASK-0024(VLM → 座標変換 → IK の結合)、AC-05(検出誤差 mm の算出)。
- 🔵 **位置づけ**: `src/so101_vlm/perception/` の純粋関数。MuJoCo・ROS・ファイル I/O に依存しない。合成データで単体テストする(design.md 4.4、NFR-05)。
- 🟡 **座標規約**(projection-survey.md で実測確定):
  - 画像: `u` = 列(右が正)、`v` = 行(下が正)。整数の画素座標は**画素中心**を指す(画素 `(i, j)` の中心が `u=i, v=j`)。
  - カメラ: **OpenCV 規約**(x 右、y 下、光軸 +z)。本モジュールの入出力はすべてこの規約。
  - 深度: **z 深度**(光軸方向の距離)。ユークリッド距離ではない。
  - base: `baseframe` サイトの座標系。本シーンではワールド原点・回転なしと実測で一致。
- **参照したEARS要件**: FR-04, NFR-05
- **参照した設計文書**: design.md 4.4 Perception

---

## 2. 入力・出力の仕様

### 2.1 `Camera`(frozen dataclass)🟡

| フィールド | 型 | 制約 |
|---|---|---|
| `fx`, `fy` | float | 有限かつ > 0 |
| `cx`, `cy` | float | 有限(画像外の値も許す。実機キャリブレーションでありうるため)🔴 |
| `width`, `height` | int | 正の整数(`bool` は不可)🔴 |

- `@dataclass(frozen=True)`。`__post_init__` で検証し、違反は `ValueError`。🟡
- sim の値(TASK-0009 で計算): `fx = fy = (h/2)/tan(fovy/2) = 415.6922`、`cx = (w-1)/2 = 319.5`、`cy = (h-1)/2 = 239.5`(640x480、fovy 60°)。🟡(survey (a))
- `Camera` を MuJoCo から作るファクトリは本タスクに**含めない**(4 章「分担」参照)。🔴

### 2.2 関数一覧

| 関数 | 入力 | 出力 | 信頼性 |
|---|---|---|---|
| `camera_matrix(camera)` | `Camera` | 新しい `np.ndarray` (3,3) float64 `[[fx,0,cx],[0,fy,cy],[0,0,1]]` | 🔵 |
| `pixel_to_camera(u, v, depth, camera)` | 画素 `u, v`(有限の実数。浮動小数可)、`depth`(z 深度 [m]、有限かつ > 0)、`Camera` | (3,) float64 `[(u-cx)·z/fx, (v-cy)·z/fy, z]` | 🔵 式 / 🟡 第 4 引数を K ではなく `Camera` にした |
| `camera_to_base(p_cam, T_base_cam)` | (3,) 有限、4x4 | (3,) float64 `R @ p + t` | 🔵 |
| `project_base_to_pixel(p_base, T_base_cam, camera)` | (3,) 有限、4x4、`Camera` | `tuple[float, float, float]` = `(u, v, z)`。画像外でも値を返す(範囲判定は呼び出し側)。`z <= 0`(カメラの後ろ・同一平面)は `ValueError` | 🟡 |
| `validate_T_base_cam(T)` | 任意(array-like) | 検証済みの 4x4 float64 の新しい配列 | 🟡 |
| `mujoco_cam_to_opencv(T_base_cam_mj)` | MuJoCo 規約のカメラ姿勢 4x4 | OpenCV 規約の 4x4(`T @ diag(1,-1,-1,1)`)。入力は `validate_T_base_cam` で検証 | 🟡(survey (b)) |
| `representative_depth(depth_image, bbox, *, shrink=0.5)` | HxW 実数配列、`bbox`、`0 < shrink <= 1` | float [m](中央領域の有効深度の中央値) | 🔵 中央値 / 🟡 領域の決め方 |
| `bbox_to_base_point(bbox, depth_image, camera, T_base_cam, *, shrink=0.5)` | `bbox`、深度画像 (camera.height, camera.width)、`Camera`、4x4 | (3,) float64 base 座標の**表面の点** | 🔵 |

### 2.3 `validate_T_base_cam` の条件 🟡

- 形状 (4, 4)、数値に変換でき、すべて有限。
- 最下行が `[0, 0, 0, 1]`(各要素の絶対誤差 ≤ 1e-9)。
- 回転部 `R = T[:3,:3]` が直交: `max|R^T R - I| ≤ 1e-6`。
- `|det(R) - 1| ≤ 1e-6`(鏡映 det=-1 を弾く。`diag(1,-1,-1,1)` を掛け忘れた/二重に掛けた行列はここでは弾けない点に注意 — det は +1 のまま)。🔴
- 違反は `ValueError`(どの条件かをメッセージに含める)。
- 申し送り(TASK-0009): 1e-6 の直交判定を通すため、`calibration.yaml` には丸めずに float の全桁を書く。MuJoCo 由来の行列の直交誤差は 2.2e-16(survey (b))。🔴

### 2.4 bbox の仕様 🔴(design.md 4.3 の `bbox_px: tuple[int,int,int,int]  # x1, y1, x2, y2` を具体化)

- 形式 `(x1, y1, x2, y2)`、画素単位。長さ 4 の列で、各要素は有限の実数(int / float / numpy の数値。`bool` は不可)。
- **`x2`, `y2` は含まない端**(numpy のスライスと同じ半開区間)。整数の bbox `(x1, y1, x2, y2)` は画素 index `x1..x2-1`, `y1..y2-1` を覆う。
- 中心画素(画素中心規約): `u_c = (x1 + x2 - 1) / 2`、`v_c = (y1 + y2 - 1) / 2`。例: `(10, 20, 11, 21)` → `(10, 20)`。
- `x1 < x2` かつ `y1 < y2` でなければ `ValueError`(面積 0・反転)。
- 画像からはみ出す部分は `[0, width] x [0, height]` にクリップする。クリップ後に面積が 0(完全に画像外)なら `ValueError`。
- `bbox_to_base_point` の中心画素は**クリップ後の** bbox から計算する(深度を読む領域と一致させるため)。🔴

### 2.5 `representative_depth` の領域と有効深度 🟡

1. bbox を検証・クリップする(2.4)。
2. 中央領域: 中心 `((x1+x2)/2, (y1+y2)/2)`(端の座標)、幅・高さをそれぞれ `shrink` 倍。画素範囲は `[floor(左), ceil(右))`、`[floor(上), ceil(下))` とし、クリップ後の bbox 内に収める。最低 1x1 画素を保証する。
3. 有効深度: `np.isfinite(d) & (d > 0)`。
4. 有効画素が 0 個なら `ValueError`(外側へのフォールバックはしない)。
5. `float(np.median(有効深度))` を返す。
- 深度画像の検証: 2 次元、実数型、少なくとも 1x1。違反は `ValueError`。`shrink` が `(0, 1]` の外・非有限なら `ValueError`。
- far 平面の値(何も写らない画素)の除外はしない(本シーンでは発生しない。survey: 深度 0.378〜0.893 m)。🔴

### 2.6 `bbox_to_base_point` 🔵

- `depth_image.shape == (camera.height, camera.width)` でなければ `ValueError`。🟡
- `z = representative_depth(...)`、`(u_c, v_c)` = クリップ後の bbox の中心画素、`p_cam = pixel_to_camera(u_c, v_c, z, camera)`、`return camera_to_base(p_cam, validate_T_base_cam(T_base_cam))`。
- 戻り値は物体の**表面の点**(ブロックなら上面付近)。把持目標(ブロック中心)は、呼び出し側が**上面から半辺だけ下げる**補正をする(base の z は上向きなので `p[2] - SCENE_SPEC.block_half_size` = `p[2] - 0.015`)。補正ヘルパ(`surface_to_block_center` 等)は**本モジュールに入れない**。docstring に明記する。🔴
  - 注: note.md の「`z + block_half_size` を足す」は誤り(上面 z=0.030 → 中心 z=0.015 なので引く)。本書で訂正する。

### 2.7 戻り値・入力の扱い(共通)🟡

- 戻り値の配列は毎回新しく作る float64 の `np.ndarray`(書き込み可。内部状態を共有しないので読み取り専用にはしない)。🔴
- 入力(配列・bbox)は変更しない。テストで入力のコピーと比較して確認する。🔵(コーディング規約)
- 同じ入力には同じ出力(乱数・グローバル状態なし)。🔵
- 不正入力は `ValueError`: NaN / inf / 負または 0 の深度 / 形状違い / 非数値。`TypeError` は使わず `ValueError` に統一する。🟡
- ベクトル演算で実装し、画素ループを避ける。🟡

- **参照したEARS要件**: FR-04
- **参照した設計文書**: design.md 4.3(`Detection.bbox_px`)、4.4

---

## 3. 制約条件

- 🔵 純粋関数・MuJoCo 非依存(`projection.py` は `numpy` と標準ライブラリだけを import)。design.md 4.4。
- 🔵 単体テスト・カバレッジ 80% 以上(NFR-05、AC-07)。
- 🟡 精度(TASK-0008 完了条件 + survey (d)):
  - 往復(既知の 3 次元点 → `project_base_to_pixel` → 浮動小数画素のまま `pixel_to_camera` → `camera_to_base`)の誤差 ≤ 1 mm(実際は 1e-9 m 程度)。
  - シーン統合(整数画素、描画深度): ≤ 2 mm(実測最大 0.96 mm、乱数評価の最大 1.34 mm)。
  - 床の逆投影 `|z| ≤ 0.1 mm`(規約A 0.012 mm。規約B `w/2` なら 0.50 mm で落ちる → 主点規約の回帰テストになる)。
  - 外れ値: 中央領域の半分未満を遠い深度(例 5 m)や 0/NaN に置き換えても、代表深度が真値から 1 mm 以内。
- 🟡 座標規約: OpenCV 規約、z 深度、主点 `(w-1)/2, (h-1)/2`(survey (a)(b)(c))。
- 🔵 ファイルサイズ・関数サイズ: 800 行 / 50 行未満(グローバル規約)。
- セキュリティ・DB・API: 該当なし。

- **参照したEARS要件**: FR-04, NFR-05, AC-07
- **参照した設計文書**: design.md 4.4、5(configs/calibration.yaml)

---

## 4. TASK-0008 / TASK-0009 の分担 🔴

| 項目 | 担当 |
|---|---|
| `Camera`、投影・逆投影、`T` の検証、MuJoCo→OpenCV 変換、代表深度、`bbox_to_base_point` | **TASK-0008**(`projection.py`、純粋関数) |
| `camera_from_mujoco(model, camera_name, width, height) -> Camera` | **TASK-0009**(`calibration.py`) |
| `t_base_cam_from_mujoco(model, data, camera_name, base_site="baseframe")` | **TASK-0009** |
| `configs/calibration.yaml` の書き出し・読み込み | **TASK-0009** |

- TASK-0008 の統合テスト(`tests/perception/test_projection_scene.py`)は、**テストファイル内の小さなヘルパ**で K と `T_base_cam` を計算する:
  - `fy = (h/2)/tan(radians(model.cam_fovy[id])/2)`、`fx = fy`、`cx=(w-1)/2`、`cy=(h-1)/2`
  - `T_world_cam_mj` = `data.cam_xpos`, `data.cam_xmat`(`mj_forward` 後)、`T_world_base` = `baseframe` サイトの `site_xpos`, `site_xmat`
  - `T_base_cam = mujoco_cam_to_opencv(inv(T_world_base) @ T_world_cam_mj)`
- TASK-0009 はこのヘルパを製品コードに移し、同じ統合テストの値を再利用する。

---

## 5. 想定される使用例

### 5.1 基本パターン 🔵

```python
camera = Camera(fx=415.69, fy=415.69, cx=319.5, cy=239.5, width=640, height=480)
p_surface = bbox_to_base_point((251, 144, 279, 176), depth, camera, T_base_cam)
p_grasp = p_surface - np.array([0.0, 0.0, SCENE_SPEC.block_half_size])  # 呼び出し側(TASK-0012/0024)
```

### 5.2 データフロー 🔵

`Detection.bbox_px`(TASK-0017)→ `representative_depth` → 中心画素 + 深度 → `pixel_to_camera` → `camera_to_base` → IK 目標(TASK-0012 / 0024)。

### 5.3 テストで確認する場面

| 場面 | 期待 | 信頼性 |
|---|---|---|
| 既知の 3 次元点の往復(画像の中心・四隅付近・深度 0.3〜1.0 m) | 誤差 ≤ 1 mm | 🔵 |
| 主点の点 `(cx, cy)` は光軸上 | `p_cam = (0, 0, z)` | 🟡 |
| `mujoco_cam_to_opencv` を 2 回掛けると元に戻る | 一致 | 🟡 |
| MuJoCo 規約の点 `(0,0,-1)`(前方 1 m) | OpenCV 規約で `(0,0,1)` | 🟡 |
| bbox の一部に遠い深度・0・NaN が混ざる(中央領域の半分未満) | 中央値は真値から 1 mm 以内 | 🔵 |
| bbox が画像から一部はみ出す | クリップして計算できる | 🔴 |
| 1x1 の bbox、`shrink` が小さく領域が 1 画素未満 | 最低 1 画素で計算できる | 🟡 |
| シーン: 3 ブロックの上面中心(整数画素 + 描画深度) | 真値 (x, y, 0.030) から ≤ 2 mm | 🟡 |
| シーン: 床の画素の逆投影 | `|z| ≤ 0.1 mm` | 🟡 |

### 5.4 エラーケース(すべて `ValueError`)

| 入力 | 信頼性 |
|---|---|
| 深度が 0 / 負 / NaN / inf(`pixel_to_camera`) | 🔵 |
| bbox 内の深度がすべて 0 または NaN(`representative_depth`) | 🔵 |
| bbox: 要素数 ≠ 4、非有限、`x1 >= x2` / `y1 >= y2`、完全に画像外、`bool` | 🔴 |
| 深度画像: 2 次元でない、形状が `(height, width)` と違う | 🟡 |
| `shrink` ≤ 0、> 1、非有限 | 🔴 |
| `T`: 形状 ≠ (4,4)、非有限、最下行 ≠ `[0,0,0,1]`、非直交、det = -1 | 🟡 |
| `Camera`: `fx`/`fy` ≤ 0・非有限、`width`/`height` ≤ 0・非整数 | 🟡 |
| `project_base_to_pixel`: カメラ座標で `z <= 0` | 🟡 |
| 画素 `u, v` が NaN / inf | 🟡 |

- **参照したEARS要件**: FR-04、FR-03(VLM 失敗時は動かない → 変換失敗も例外で伝える)
- **参照した設計文書**: design.md 4.4

---

## 6. 範囲外(申し送り)

- 🔴 bbox の中心が上面の中心と一致しない誤差(輪郭 bbox だと 7〜10 mm、survey 補足)。TASK-0024 / AC-05 で記録し、必要なら「既知の上面の高さとの光線交差」補正を検討する。
- 🔴 far 平面の深度の除外、深度の上限。必要になったら追加する(YAGNI)。
- 🔵 `configs/calibration.yaml` の作成(TASK-0009)。

---

## 7. EARS要件・設計文書との対応関係

- **参照したユーザストーリー**: 指示した色のブロックを箱に入れる(docs/requirements.md)
- **参照した機能要件**: FR-04(主)、FR-03、FR-05
- **参照した非機能要件**: NFR-05
- **参照した受け入れ基準**: AC-05(検出誤差 mm)、AC-07(カバレッジ 80%)、TASK-0008 完了条件(往復 ≤ 1 mm、中央値、不正深度でエラー)
- **参照した設計文書**:
  - **アーキテクチャ**: design.md 4.4 Perception
  - **型定義**: design.md 4.3 `Detection.bbox_px`
  - **設定**: design.md 5 `configs/calibration.yaml`(TASK-0009)
  - **実測**: projection-survey.md、TASK-0005 verify-report.md
  - データベース・API: 該当なし

---

## 8. 決定と仮定の一覧(ユーザー確認なしで決めたもの)

1. 主点 `(w-1)/2, (h-1)/2`(実測で確定)。
2. `pixel_to_camera` の第 4 引数は K 行列ではなく `Camera`(K の検証を不要にするため)。
3. bbox は半開区間 `[x1, x2) x [y1, y2)`、中心画素 `((x1+x2-1)/2, (y1+y2-1)/2)`。
4. bbox はクリップしてから使い、中心もクリップ後から取る。
5. 戻り値は書き込み可能な新しい float64 配列(読み取り専用にしない)。
6. 表面 → ブロック中心の補正ヘルパは入れない。補正は「z から半辺を引く」(note.md の「足す」を訂正)。
7. 不正入力は全て `ValueError`。
8. `Camera` / `T_base_cam` の MuJoCo からの生成は TASK-0009。TASK-0008 の統合テストはテスト内ヘルパで計算する。
9. 許容誤差: 往復 1 mm、統合 2 mm、床 0.1 mm、T の直交 1e-6、最下行 1e-9。

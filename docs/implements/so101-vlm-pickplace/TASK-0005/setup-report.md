# TASK-0005 設定作業実行

## 作業概要

- **タスクID**: TASK-0005(作業台・ブロック・箱・カメラのシーン)
- **内容**: `assets/scene_pickplace.xml` の作成、RGB-D 取得ヘルパー、撮影スクリプト、テスト
- **参照**: docs/design.md 4.4 / 4.7、docs/requirements.md FR-01 / FR-12、TASK-0005.md

## 作成・変更したファイル

| ファイル | 内容 |
|---|---|
| assets/scene_pickplace.xml | 新規。ブロック 3、箱、カメラ `cam_top`、床・照明 |
| src/so101_vlm/robot/model_paths.py | `PICKPLACE_SCENE_PATH` を追加(既存定数は維持) |
| src/so101_vlm/nodes/camera.py | `render_rgbd(model, data, camera_name, width, height)` |
| src/so101_vlm/eval/scene_spec.py | `SceneSpec`(frozen)/`SCENE_SPEC`: 寸法・名前の単一の情報源 |
| scripts/capture_scene.py | 1 秒静定 → 撮影 → media/ に png/npy 保存 |
| tests/test_scene.py | 22 件(約 1.8 秒) |
| README.md | 「シーンの確認(TASK-0005)」を追加 |

`assets/so101/*` は未改変。

## 設計判断

- **include**: 上流 `scene.xml` は include できない(入れ子 include と `meshdir` が最上位ファイル基準で解決され、`assets/assets/so101/...` を探して失敗)。`so101_new_calib.xml` を直接 include し、末尾に `<compiler meshdir="so101/assets" strippath="true"/>` を置く(include 由来の `so101/` 接頭辞を strippath で落とす)。床・照明・空は上流 scene.xml と同内容を書き写した。🔵(動作確認済み)
- **作業面**: 作業台は置かず、ロボット土台と同じ床 (z=0) を作業面とする。🟡
- **ブロック**: 3 cm 立方体、20 g、摩擦 1.0(condim 4)、freejoint。赤 (0.20,-0.06)、青 (0.30,0.00)、緑 (0.22,0.07)。互いに 5 cm 以上離れる。TASK-0006 が上書きする。🟡
- **箱**: body `box`、底板 + 壁 4 枚(上開放)。内寸 8x8 cm、壁厚 5 mm、縁の高さ 4 cm(z=0.04)、底板上面 z=0.005、中心 (0.20, 0.19)。作業エリア(y<=0.10)と重ならない。🟡
- **作業エリア**: x[0.15,0.35] y[-0.10,0.10]。到達性は関節範囲の乱数サンプル 6 万点(位置のみ・姿勢は問わない)で、4 隅と箱中心の半径 2 cm・高さ 0〜5 cm に手先が到達する点が存在することを確認(各 5〜19 点)。姿勢制約付き IK での到達性は TASK-0009 で要確認。🟡
- **カメラ** `cam_top`: pos (0.45, 0.30, 0.55)、(0.24, 0.10, 0) を注視、fovy 60°。正面 (0.52, 0.10, 0.48) からだと home 姿勢 (q=0) の腕が赤ブロックを隠した(深度が 0.32 m となり検出)ため、腕に隠れない左前上方に変更。作業エリア全体 90 点と箱 9 点で遮蔽 0 を `mj_ray` で確認。🟡
- **深度**: `Renderer` の深度は光軸方向の z 深度(m)。実行時に「ARB_clip_control unavailable」警告が出るが、精度は下記のとおり問題なし。

## 確認結果

投影チェック(ブロック上面中心 / 箱の底板中心をカメラ行列で画素へ投影し、深度を比較):

| 対象 | 画素 (u,v) | 画像内 | 真の z 深度 [m] | 画像の深度 [m] | 差 [mm] |
|---|---|---|---|---|---|
| block_red | (264.4, 153.6) | はい | 0.6602 | 0.6595 | -0.75 |
| block_blue | (242.1, 215.0) | はい | 0.6071 | 0.6068 | -0.33 |
| block_green | (314.6, 209.3) | はい | 0.6116 | 0.6116 | -0.04 |
| box | (384.1, 258.6) | はい | 0.6019 | 0.6013 | -0.65 |

作業エリア 4 隅の画素は u 179〜360、v 136〜290 で画像内。

- `uv run pytest -q`: 81 passed(既存 59 + 新規 22)
- `uv run pytest --cov -q`: 81 passed、全体 100%
- SimRobot は名前から `jnt_qposadr` / `jnt_dofadr` で添字を引くため、ブロックの freejoint が加わっても動く(テストで確認)。

## 次のステップ

- `/tsumiki:direct-verify`
- 画像の目視確認(media/scene_rgb.png)

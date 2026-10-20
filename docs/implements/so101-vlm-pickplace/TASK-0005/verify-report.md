# TASK-0005 設定確認・動作テスト

- 実行日: 2026-10-19 / 対象: pick-and-place シーン

## 結果サマリー: 全項目合格(修正なし)

| # | 確認 | 結果 |
|---|---|---|
| 1 | `uv run pytest -q` | 81 passed。`--cov` も 81 passed、全体 100%(camera.py / scene_spec.py も 100%) |
| 2 | 上流ファイル未改変 | `git diff --stat` は README.md と model_paths.py のみ。assets/so101 に差分なし。`fetch_so101_model.py`(--force なし)は skip で正常。固定コミットから取得した so101_new_calib.xml / scene.xml と `cmp` 一致 |
| 3 | capture_scene.py | scene_rgb.png / scene_depth.npy / scene_depth.png を生成。深度 480x640、全画素有限・正。min 0.378 / max 0.893 / median 0.600 m。赤ブロック画素 (264,154) の深度 0.6595 m(真値 0.6603 m、差 0.8 mm)。青 0.6068/0.6072、緑 0.6116/0.6117 |
| 4 | 静定・SimRobot | 3 秒後の z=0.01493 m(3 個とも)、xy ドリフト 0.0 mm。SimRobot で q=(0.3,0.3,0.5,-0.3,0.5,0.5) を 300 回指令後も指令に追従し、ブロックの移動 0.0 mm(腕は遠方) |
| 5 | 遮蔽(情報) | カメラからブロック中心・箱底板中心への mj_ray は、q=0 と q=(0,1.0,-1.0,0,0,0) の両方で最初にそのブロック/箱自身に当たる(腕による遮蔽なし) |
| 6 | コード品質 | camera.py 32 行 / scene_spec.py 30 行 / capture_scene.py 47 行 / test_scene.py 148 行。関数は全て 50 行未満。SceneSpec は frozen dataclass、グローバルな可変状態なし、Renderer は try/finally で close。新しい配列を返す。定数は名前付き。XML と SCENE_SPEC の照合テスト (`test_box_geometry_matches_spec`, `test_block_geometry_matches_spec`) あり |
| 7 | media/ の gitignore | `git check-ignore media/scene_rgb.png` で無視を確認 |
| 8 | 完了条件 | 下記 |

## 完了条件
- [x] シーンを読み込める(テスト・capture_scene で確認)
- [x] RGB と深度を取得し保存できる(png / npy を生成)
- [x] ブロック 3 個と箱が画像内に写る(投影画素が画像内、深度が真値と一致、遮蔽なし。目視でも確認済み)

## 修正
なし。

## 注意・残課題
- 実行時に「ARB_clip_control unavailable」警告が出るが、深度精度は 1 mm 未満で問題なし。
- 姿勢制約付き IK での到達性は TASK-0009 で確認が必要(setup-report 記載どおり)。
- 腕がブロック・箱の上へ動く場合の遮蔽は動作中に変わるため、VLM 呼び出し時の姿勢(ホーム)で撮る前提。
- CLAUDE.md は変更していない(指示による)。

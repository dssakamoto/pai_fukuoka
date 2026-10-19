# TASK-0003 設定確認・動作テスト

- **実行日**: 2026-10-18 / **実行者**: Claude (direct-verify)

## 結果サマリー

| # | 確認 | 結果 |
|---|------|------|
| 1 | `uv run pytest -q` / `--cov` | 合格: 18 passed、カバレッジ 100%(src/so101_vlm, 25 stmts) |
| 2 | ヘッドレス 12 秒の動画 | 合格: 640x480、30fps、360 フレーム(imageio で確認) |
| 2' | 6 関節が順に動く(qpos 検証) | 合格: 下表 |
| 3 | 非 mjpython での起動 | 合格: 案内メッセージを stderr に出し exit=1。`.venv/bin/mjpython` 存在 |
| 4 | コード品質 | 合格(1 件修正) |
| 5 | media/・*.mp4 が .gitignore、検証用動画の削除 | 合格(`media/verify_sweep.mp4` 削除済み) |
| 6 | JOINT_NAMES の単一定義 | 合格: 定義は `model_paths.py:9` のみ。`base.py` は再エクスポート |
| 7 | GUI ビューア表示 | 要手動確認(`uv run mjpython scripts/view_so101.py`) |

## 2. 6 関節が順に動く(再現した sweep ループの data.qpos、各 2 秒ウィンドウ=60 フレーム)

| 窓(秒) | 関節 | qpos min | qpos max | ctrlrange | 可動域に対する振れ幅 |
|---|---|---|---|---|---|
| 0-2 | shoulder_pan | -0.960 | 0.960 | [-1.92, 1.92] | 50.0% |
| 2-4 | shoulder_lift | -0.873 | 0.664 | [-1.745, 1.745] | 44.0% |
| 4-6 | elbow_flex | -0.845 | 0.845 | [-1.69, 1.69] | 50.0% |
| 6-8 | wrist_flex | -0.829 | 0.829 | [-1.658, 1.658] | 50.0% |
| 8-10 | wrist_roll | -1.346 | 1.445 | [-2.744, 2.841] | 50.0% |
| 10-12 | gripper | 0.306 | 1.265 | [-0.175, 1.745] | 50.0% |

各関節が自分の窓で可動域の約 44〜50% 動く(アクティブ関節以外は指令が中心固定。他関節の窓内変動 0.2〜0.8 rad は、開始時の初期姿勢から中心への収束と重力・関節間の連成による追従であり、指令は 1 関節のみ。指令が 1 関節のみであることは tests/robot/test_sweep.py で検証済み)。shoulder_lift の最大値が小さいのは重力負荷による追従遅れ。

## 4. コード品質

- sweep.py(32 行)、view_so101.py(87 行)とも 800 行未満、関数は 50 行未満。入力非変更(新配列を返す)、定数は名前付き(FPS, WIDTH, HEIGHT, MJPYTHON_COMMAND)。
- **修正 1 件**: `--seconds` に 0 以下を渡すと 0 フレームの動画を無言で出力していた。`parser.error` で終了(exit=2)するよう `scripts/view_so101.py` に 2 行追加。修正後 `--seconds -1` は exit=2、`--seconds 1` は正常、pytest 18 passed。
- 非数値(`abc`)は argparse が拒否する。

## 7. 完了条件

- [ ] ビューアで 6 関節が順に動く: 要手動確認(`uv run mjpython scripts/view_so101.py`)。同一の駆動ロジックの qpos で 6 関節が順に動くことは確認済み
- [x] `JOINT_NAMES` が定義されている(`robot/base.py` から参照可能、定義元 `model_paths.py`)
- [x] 画面なしでも動画に書き出して確認できる(実行時に出力先の絶対パスを表示)

## 推奨・次のステップ

- 手元で mjpython のビューア表示を一度目視する。
- CLAUDE.md は変更しない指示のため未更新(README に確認手順は記載済み)。

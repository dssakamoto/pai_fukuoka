# TASK-0003 設定作業実行

## 作業概要

- **タスクID**: TASK-0003
- **作業内容**: MuJoCo で SO-101 を表示し、6 関節を駆動する確認スクリプトと関節順定数の整備
- **実行日時**: 2026-10-18
- **実行者**: Claude (direct-setup)

## 設計文書参照

- docs/tasks/so101-vlm-pickplace/TASK-0003.md、docs/spec/so101-vlm-pickplace/note.md、docs/design.md 1・4.1章

## 実行した作業

1. `uv add imageio imageio-ffmpeg`(動画書き出し用。pillow も依存で追加)
2. `src/so101_vlm/robot/base.py`: `JOINT_NAMES` を `model_paths` から再エクスポート(定義の重複を避け単一の情報源にする)。Robot Protocol は TASK-0004
3. `src/so101_vlm/robot/sweep.py`: `sweep_targets(ctrlrange, t, period=2.0, amplitude_fraction=0.5)`。関節を period 秒ずつ順に正弦波で駆動、他は可動域中心。新しい配列を返し、常に ctrlrange 内
4. `scripts/view_so101.py`: `--headless --out --seconds`(mujoco.Renderer 640x480、30fps、mp4)と、既定のパッシブビューア。darwin で mjpython 以外なら `uv run mjpython scripts/view_so101.py` を案内して終了(`mujoco.viewer._MJPYTHON is None` で判定)
5. `tests/robot/test_sweep.py`(範囲内、同時に動くのは 1 関節、入力非変更、形状、不正引数、JOINT_NAMES 単一定義)
6. README に「動作確認(TASK-0003)」節を追加

## 作業結果

- [x] `uv run pytest -q`: 18 passed
- [x] `uv run pytest --cov -q`: 18 passed、カバレッジ 100%(src/so101_vlm)
- [x] ヘッドレス 6 秒: `media/so101_sweep.mp4`(320,529 bytes、180 フレーム、640x480)。media/ と *.mp4 は .gitignore 済み(`git check-ignore` は git 管理外のため .gitignore 行で確認)
- [x] フレーム確認(平均画素値、黒画面でない。フレーム 0 との平均絶対差 3〜8 で動いている)
  - frame_010: 76.14 / frame_090: 76.12 / frame_175: 76.05
  - 保存先: /private/tmp/claude-501/-Users-takuya-project-20-pai-homework/b14af3ef-30ce-4f56-a63b-8ac03b5a9e70/scratchpad/task0003/
- [x] 非 mjpython でのビューア起動: 案内メッセージを表示して終了を確認
- [ ] mjpython でのビューア実表示は未確認(GUI のため。`.venv/bin/mjpython` の存在は確認済み)

## 遭遇した問題と解決方法

- mjpython 判定: `hasattr(mujoco.viewer, "_MJPYTHON")` は常に True のため、`getattr(..., None) is None` に修正
- 「ctrlrange と関節 range の一致」テストは tests/test_assets.py に既存のため重複して作らない
- 6 秒の動画では 3 関節までしか動かない(1 関節 2 秒)。全 6 関節は既定の 12 秒で一巡する

## 次のステップ

- `/tsumiki:direct-verify` を実行

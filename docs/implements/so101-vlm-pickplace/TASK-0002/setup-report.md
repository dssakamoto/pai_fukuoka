# TASK-0002 設定作業実行

## 作業概要

- **タスクID**: TASK-0002
- **作業内容**: SO-101 MuJoCo モデルの取得と配置
- **実行日時**: 2026-10-18
- **実行者**: Claude (direct-setup)

## 設計文書参照

- docs/tasks/so101-vlm-pickplace/TASK-0002.md、docs/spec/so101-vlm-pickplace/note.md、docs/design.md 1・6章、README.md

## 実行した作業

1. GitHub API で TheRobotStudio/SO-ARM100 の main の SHA を解決し固定: `a758567c3978dfeefe282ede0500085a48fe8f78`(2026-10-06 のコミット)
2. `scripts/fetch_so101_model.py` を作成(標準ライブラリのみ。xml からメッシュと include を解析して取得、既存はスキップ、`--force` で上書き、取得後に mujoco で検証)
3. `uv run python scripts/fetch_so101_model.py` を実行。`so101_new_calib.xml`、`scene.xml`、メッシュ 13 個(assets/)、`LICENSE` を `assets/so101/` に配置。`joints_properties.xml` はどの xml からも参照されないため取得していない
4. サイズ 約 15 MB(20 MB 未満)のため git 管理に含める(.gitignore は変更なし)
5. `assets/so101/SOURCE.md` を作成(リポジトリ・パス・SHA・取得日 2026-10-18・無改変を記録)
6. README.md のライセンス/クレジット TODO を、SO-ARM100(Apache-2.0)の表示と取得コマンドに置換
7. `src/so101_vlm/robot/model_paths.py`(SO101_SCENE_PATH、JOINT_NAMES、SITE_NAMES)と `tests/test_assets.py` を作成

## 作業結果

- [x] scene.xml が mujoco で読み込める(njnt=6, nu=6)
- [x] `uv run pytest -q`: 12 passed
- [x] `uv run pytest --cov -q`: 12 passed、カバレッジ 100%(対象は src/so101_vlm)
- ctrlrange と関節 range は 1e-3 rad 以内で一致(不一致なし)

## 遭遇した問題と解決方法

- 特になし(`gh` コマンドが無かったため、SHA は curl で GitHub API から取得)

## 次のステップ

- `/tsumiki:direct-verify` を実行

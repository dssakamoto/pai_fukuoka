# TASK-0002 設定確認・動作テスト

- タスクID: TASK-0002（SO-101 モデルの取得と配置）
- 実行日: 2026-10-18
- 方針: setup-report を信用せず、現状から独立に再確認した。修正は行っていない（欠陥なし）。

## 確認結果

| # | 項目 | 結果 | 根拠 |
|---|---|---|---|
| 1 | pytest / coverage | 合格 | `12 passed`。`--cov` も 12 passed、TOTAL 100%（model_paths.py 6 行すべて） |
| 2 | scene.xml の読み込み | 合格 | njnt=6, nu=6。関節・アクチュエータとも shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper。site baseframe(id 0)・gripperframe(id 1) あり。mj_step 10 回が正常（time=0.02） |
| 3 | 取得スクリプトの冪等性 | 合格 | 再実行で全 15 ファイルが `skip`、続けて `OK: scene.xml を読み込めた (njnt=6, nu=6)`、rc=0。import は標準ライブラリのみ＋検証時の mujoco（関数内）。HTTPError / URLError を SystemExit で明示処理、timeout 60 秒 |
| 4 | メッシュの存在 | 合格 | so101_new_calib.xml の mesh 13 個がすべて assets/so101/assets に存在 |
| 5 | LICENSE / SOURCE.md | 合格 | LICENSE は Apache License 2.0 の全文（201 行）。SOURCE.md にリポジトリ URL・パス・SHA・取得日 2026-10-18・「改変: なし」あり。SHA は a758567c…8f78 でスクリプトと一致 |
| 6 | 完全性 | 合格 | 固定 SHA の raw を一時ディレクトリに取得して比較。xml 2 つ・LICENSE は diff 一致、STL 13 個すべて cmp 一致（無改変） |
| 7 | README | 合格 | SO-ARM100、Apache-2.0、取得コマンドの記載あり。リンク先 LICENSE・SOURCE.md は実在 |
| 8 | サイズ / ignore | 合格 | assets/so101 は 15 MB。`git check-ignore` で除外されない（rc=1）。.gitignore に assets/stl の記載なし |

## 完了条件（TASK-0002.md）

- [x] so101_new_calib.xml と scene.xml、メッシュがそろっている: 合格（4, 6）
- [x] MuJoCo で MJCF を読み込める: 合格（2、tests/test_assets.py も通過）
- [x] ライセンス表示（Apache-2.0）と取得元・コミットが記録されている: 合格（5, 7）

## 発見された問題と対処

なし。修正は行っていない。

## 推奨事項（欠陥ではない）

- 初回コミット時に 15 MB のバイナリが履歴に入る。許容範囲だが、増える場合は Git LFS を検討する。
- joints_properties.xml は参照されないため取得していない（setup-report のとおり）。

## CLAUDE.md への記録

ルートの CLAUDE.md にテスト実行コマンドが既にあるため、変更していない（指示により CLAUDE.md は変更しない）。

## 判定: 合格

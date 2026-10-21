# TASK-0012 Refactor フェーズ記録 (軌道生成)

## 変更内容 (振る舞いは不変)
1. 🔵 `_to_float_tuple` を `src/so101_vlm/safety/_validation.py` の `to_float_tuple` へ移動 (エラーメッセージ同一)。`safety/config.py` と `policy/trajectory_config.py` が共有。private 名の import を解消。safety の公開 API は不変。
2. 🔵 `src/so101_vlm/policy/ik_policy.py` の `_interpolate` を表駆動化 (`SEGMENT_TABLE`: 区間名, q_wp キー, 保持時間設定名)。
3. 🔵 `_ik` / `_records` / `_finish` の引数に型注釈を追加。
4. 🟡 未カバー分岐のテストを `tests/policy/test_trajectory_extra.py` に追加 (11 件: hold_s 不正, n 不正, 0 秒保持, clearance 型検証)。既存テストは未変更。
5. 🔵 `ClearanceChecker.tcp_positions` / `min_distances` を `sample()` に統合し、サンプルごとの FK を 1 回にした。経路確認とすき間確認が同じ結果を共有。代表 8 配置で q・最小すき間が変更前と完全一致 (配列 equal)。
6. 命名・日本語 docstring・関数 50 行未満・ファイル 500 行未満を確認 (policy/ 最大 299 行)。

## 品質・セキュリティ
- 入力を変更しない (読み取り専用コピー、MappingProxyType)。グローバル状態・乱数なし。`yaml.safe_load` のみ。秘密情報なし。
- 実行時間: act() 代表 8 配置は約 1.0 s 前後 (前 0.9-1.0 s / 後 1.1-1.3 s, 計測は環境負荷でばらつく)。支配的なのは mj_geomDistance で、FK の一本化による短縮は誤差の範囲。コスト増ではないが速度改善も確認できていない。

## O5 (0 秒保持区間を segments に残さない) についての所見
- 現行挙動: 保持 0 秒の区間は行・labels・segments のいずれにも残らない。テストで固定 (`test_zero_hold_segment_is_not_kept_in_segments`)。変更はしていない。
- 影響: close_hold_s=0 の設定では `segments` の長さが固定 10 区間+start にならない。`SEGMENT_TABLE` の長さと segments を突き合わせる利用側は、可変長として扱う必要がある。区間を残す案 (空区間を許す) は JointTrajectory の「labels と segments の対応」を崩すため、現行の方が整合する。

## 結果
- `uv run pytest -q`: 1243 passed (変更前 1232 + 追加 11)。tests/policy + tests/safety: 583 passed。
- カバレッジ: base / clearance / ik_policy / trajectory / trajectory_config / ik / ik_types / safety/_validation いずれも 100%、全体 100%。
- ruff は環境に未導入のため未実施。
- 品質判定: 高品質。

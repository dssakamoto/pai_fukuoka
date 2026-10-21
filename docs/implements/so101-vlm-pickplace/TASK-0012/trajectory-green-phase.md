# TASK-0012 Green フェーズ記録: 把持・運搬の軌道生成(trajectory / IkPolicy)

**実装日**: 2026-10-21 / **要件名**: so101-vlm-pickplace / **機能名**: trajectory
**信頼性の凡例**: 🔵 要件・実測どおり / 🟡 妥当な推測 / 🔴 資料にない仮定

## 1. 実装したファイル

| ファイル | 行数 | 内容 |
|---|---|---|
| `src/so101_vlm/policy/base.py` | 75 | `Observation`(+`from_state`)、`Goal`、`Policy`(runtime_checkable Protocol) |
| `src/so101_vlm/policy/trajectory.py` | 266 | `segment_steps`、`hold_steps`、`interpolate_segment`(最終行は終点そのもの)、`JointWaypoint`、`build_joint_trajectory`、`check_command_sequence`、`find_clearance_violation`、`JointTrajectory`(frozen・read-only)、`TrajectoryError`、`ClearanceViolation`、`WaypointRecord`、`REASON_*` |
| `src/so101_vlm/policy/trajectory_config.py` | 127 | `TrajectoryConfig`、`ClearanceConfig`、`load_trajectory_config`(キー過不足は ValueError、`yaml.safe_load`) |
| `src/so101_vlm/policy/clearance.py` | 90 | `ClearanceChecker`(FK の TCP、障害物・自己干渉の最小距離)。ik_policy.py を 800 行未満・関数 50 行未満に保つための分割 |
| `src/so101_vlm/policy/ik_policy.py` | 238 | `IkPolicy(model, *, config=None, safety=None).act(obs, goal)` |
| `configs/motion.yaml` | +42 | 末尾に `trajectory:` を追加(日本語コメント付き)。`safety:` 部分はバイト単位で同一(cmp で確認) |

## 2. 実装方針

- 参照実装(survey の ref_impl.py)のアルゴリズムを移植。`act` の順序は 入力検証 → TCP 目標 → 作業範囲(IK の前)→ carry_too_low → IK(pre_grasp は既定候補、grasp は同じ p1 のみ、pre_place は既定候補、place は p2 のみ。ウォームスタート)→ 補間 → 指令列検査 → 経路の作業範囲 → すき間(障害物 → 自己干渉)。🔵
- `IkError` は `TrajectoryError(err.reason, 経由点名, {"target": ...}) from err` に包む。部分的な軌道は返さない。🔵
- `solve_ik` は `ik_policy` の大域名、`segment_steps` は `trajectory` の大域名から引く(monkeypatch 可)。
- 保持 0 秒の区間は `segments` に残さない(O5 の決定。🔴)。
- 状態を持たない(act ごとに専用 MjData)。乱数・時計なし。入力は変更しない。

## 3. テスト結果

- 新規 3 ファイル: 253 passed(3.4 s、予算 < 10 s)。テストの変更は **なし**(数値の不一致も発生しなかった)。
- `uv run pytest -q`: **1232 passed**(9.6 s。Red 前 979 + 253)。
- `uv run pytest tests/policy -q --durations=10`: 317 passed(4.0 s)。最遅は TC-IP-17(1.09 s)、fixture `solved` の setup(0.52 s)。
- カバレッジ: base 100%、clearance 100%、ik_policy 100%、trajectory 98%(未通過: hold_s の検証、interpolate_segment の n の検証、保持 0 秒で区間を残さない continue)、trajectory_config 99%(clearance 型の検証 1 行)。全体 99%。

## 4. 品質判定

✅ 高品質: 全テスト成功、全ファイル 800 行未満、関数は 50 行未満、実装コードにモック・スタブなし。

## 5. Refactor への申し送り

1. `trajectory_config.py` が `safety.config._to_float_tuple`(private)を import している。共有ヘルパへ昇格するか `eval/validation.py` に移す。
2. `ik_policy.py` の `_interpolate` 内の `hold` ラムダ・`order` タプルの組み立てを、区間定義の表(データ駆動)に整理できる。
3. `_finish` / `_records` の引数型注釈(targets, q_wp)が未記入。
4. `trajectory.py` の未通過 3 分岐(n の型検証など)にテストを足すか、検証を整理する。
5. `ClearanceChecker.min_distances` は全サンプルで MjData を 1 つ使い回す。性能は十分(act 0.05 s 前後)だが、必要なら重複 FK(経路確認とすき間確認)を 1 回にまとめられる。
6. O1(配置範囲の下限 x)、D1(sim の物理設定)は本タスクでは変更していない。

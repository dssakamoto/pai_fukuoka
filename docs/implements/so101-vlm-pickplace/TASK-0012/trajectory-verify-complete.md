# 把持・運搬の軌道生成 TDD開発完了記録 (TASK-0012)

## 確認すべきドキュメント

- `docs/tasks/so101-vlm-pickplace/TASK-0012.md`
- `docs/implements/so101-vlm-pickplace/TASK-0012/trajectory-requirements.md`
- `docs/implements/so101-vlm-pickplace/TASK-0012/trajectory-testcases.md`

## 最終結果 (2026-10-22)

- **判定**: OK (完全実装済み。スコープ外の失敗なし)
- **実装率**: 100% (115/115 テストケース。docstring の `TC-xx-nn` を testcases の表と照合。欠落 0、意図的な削除 0、定義にない余剰 0)
- **テスト**: `uv run pytest -q` = 1243 passed / 0 failed (約 26 s)。`uv run pytest --cov -q` = TOTAL 1330 文、未実行 0、100%
- **今回の対象ファイル**: trajectory.py 132 文、trajectory_config.py 70 文、clearance.py 51 文、ik_policy.py 111 文、base.py 34 文。すべて 100%
- **実行時間**: 総実行約 26 s (30 s 未満)。遅いテストの推奨事項なし
- **タスクファイル**: TASK-0012.md と overview.md を完了に更新

## 完了条件の確認

| 完了条件 | 結果 | 証拠 |
|---------|------|------|
| 経由点が正しい順序で並ぶ | 達成 | `SEGMENT_TABLE` が pre_grasp -> grasp -> close -> close_hold -> lift -> pre_place -> place -> open -> open_hold -> retreat の表駆動。`metadata["waypoints"]` は `WAYPOINT_ORDER` の 6 件。TC-IP の順序テストが通過 |
| 軌道が可動域と速度上限を守る | 達成 | `check_command_sequence` が軌道生成時に q[0]->q[1] を含めて検査。独立確認: 成功 141 軌道を `apply_safety` に通してフラグ 0 件 |
| 数値が configs/motion.yaml にまとまっている | 達成 | `trajectory:` に 11 項目 + clearance を集約。`load_trajectory_config()` が検証して読む。コードに数値の直書きなし |
| FR-05 (軌道生成) | 達成 | 把持・運搬・配置・退避の 10 区間。把持 TCP 誤差 最大 0.94 mm、配置 最大 0.06 mm |
| FR-06 (失敗の通知) | 達成 | 全失敗が `TrajectoryError(reason, waypoint, detail)`。部分的な軌道は返さない |
| FR-11 (Policy の実装) | 達成 | `Policy` は `runtime_checkable` Protocol。`IkPolicy.act(obs, goal) -> JointTrajectory` が isinstance で適合 (TC-IP のテスト) |

## 独立スポットチェック (実物の部品で実行、テストの fixture は不使用)

### (1) 150 目標 (sample_layout seed 0..49 x 3 ブロック、ブロック中心、置き先は箱)

- 成功 141 / 失敗 9 (失敗率 6.0%。survey の census 約 6% と一致)
- 失敗の内訳: `self_collision`(grasp) 6、`not_converged`(pre_grasp) 2、`self_collision`(pre_grasp) 1。失敗の x は 0.151〜0.169 m (肩の近く) のみ
- 成功 141 件を前回指令を基準に `apply_safety` へ通した結果 (target_base は区間の終点 TCP 目標): clipped / rate_limited / rejected / stopped が 1 件も立たない (0)
- 独立 FK (新しい MjData、mj_kinematics、gripperframe + R*tcp_offset): 把持点の TCP とブロック中心の距離 最大 0.943 mm (平均 0.192 mm)、配置点の TCP と置き目標の距離 最大 0.061 mm (平均 0.043 mm)。いずれも 1.5 mm 以内
- 所要時間: 平均 5.50 s、最大 6.48 s、最小 4.86 s (想定の約 5.4 s と同程度)

### (2) SimRobot での実行 (seed 0..4、各 1 軌道、q[1:] を 1 行ずつ送る、ループ内で `apply_safety`)

- 安全フラグの最大 0 件 (5/5)
- 軌道終了時の関節誤差 0.0035〜0.0044 rad
- 把持開始前のブロック変位 3 個とも 0.07 mm (落ち着きによる沈み。接触なし)。接近中にアームがブロックへ当たらない
- ブロックの運搬は検証しない (物理設定 D1 が未変更のため)

### (3) 頑健性

- `act()` を 2 回呼ぶと q・times・labels が完全一致
- obs.q、goal の配列は変化なし
- 作業範囲外 (x = 0.5): `target_out_of_workspace` (pre_grasp)。IK の呼び出し 0 回
- 到達不能 (x = 0.161、y = -0.0233): `not_converged` (pre_grasp)。IK 1 回
- speed_scale 0.3: 所要時間 5.50 s -> 16.20 s (全体で 2.95 倍)。保持 0.8 s は速度に依らないため。移動部分だけでは (16.20-0.8)/(5.50-0.8) = 3.28 倍で、1/0.3 = 3.33 倍と整合

### (4) 設定

- `load_safety_config()` が通り、値が TASK-0010 の検証記録と一致: vmax 1.0 x6、control_dt 0.02、speed_scale 1.0、state_timeout 0.2、workspace (0.12, -0.13, 0.00)〜(0.38, 0.26, 0.17)
- `load_trajectory_config()` が survey の値を返す: tcp_offset (0.004, 0, 0.030)、approach_height 0.08、min_carry_z 0.08、place_height 0.03、gripper 0.6 / 0.15、保持 0.5 / 0.3 s、min_segment_s 0.2、factor 0.99、clearance 1 mm / 2 mm
- 注意: このプロジェクトは git 管理外のため `git show HEAD:` との比較はできない。`safety:` 部分がバイト単位で同一であることは、green フェーズの記録 (cmp で確認) と、値の一致で確認した。バイト比較そのものは再実行していない

## 未決事項 (既定値で進めている項目)

| ID | 内容 | 採用した既定値 | 信頼度 |
|----|------|----------------|--------|
| D1 | sim の把持が保てない (既定の摩擦錐 pyramidal・impratio 1 で 0/29) | 本タスクでは assets を変更しない。`scene_pickplace.xml` に `<option cone="elliptic" impratio="10"/>` を入れると 29/29。TASK-0013 の前提 (代案: 摩擦・forcerange・質量、顎の形状は TASK-0014) | 🔵 実測 |
| D2 | tcp_offset を 1 つにするか | 1 つ (0.004, 0, 0.030) | 🟡 |
| D3 | 最終姿勢 | retreat (箱の真上) で止める。ホームへ戻るのは呼び出し側 | 🔴 |
| D4 | すき間違反時にピッチ候補を変えて解き直すか | 解き直さない | 🟡 |
| D5 | 開き角 | 0.6 rad | 🔵 実測 |
| D6 | place_target_base の出どころ | v1 は SCENE_SPEC から作る (TASK-0024 で知覚) | 🔴 |
| D7 | 保持時間 0.5 / 0.3 s の短縮 | 当面そのまま (短縮の調査なし) | 🔴 |
| D8 | 実機の値 | TASK-0035 で測り直し、configs/real.yaml で上書きする方式を検討 | 🟡 |
| O1 | 配置範囲 (area 0.20 -> x 下限 0.15) | 変更しない。x 0.151〜0.169 で失敗 9/150 = 6.0%。呼び出し側で失敗を扱う (TASK-0013 以降)。x 下限を 0.17 前後へ上げれば解消する見込み (layout の変更は TASK-0006 の判断) | 🔴 |
| O5 | 保持 0 秒の区間を segments に残さない | 残さない | 🔴 |

そのほか、configs/motion.yaml の 🔴 仮定: vmax 1.0 rad/s、state_timeout 0.2 s (TASK-0010 由来)、min_carry_z 0.08 m。

## 重要な技術学習

### 実装パターン
- 純粋層 (trajectory.py: 時間配分・補間・検査) と MuJoCo 依存層 (ik_policy.py、clearance.py) に分け、`segment_steps` / `solve_ik` を大域名から引いてテストで差し替え可能にした
- 区間を `SEGMENT_TABLE` の表駆動にして、順序の変更を 1 か所にした
- 経路の作業範囲確認とすき間確認で FK を 1 回に共有した

### テスト設計
- 代表配置を module scope の fixture で 1 回だけ解いて共有し、IK 系のテスト時間を抑えた
- 実機器 (apply_safety・SimRobot) を使った実行スモークを加えた

### 品質保証
- 全指令列を `apply_safety` へ通して「フラグ 0 件」を確認し、完了条件の「制限を守る」を実測で担保した

## 注意点・修正が必要な項目

- 修正が必要な項目はない。上の未決事項を後続タスク (TASK-0013、0024、0035) で確認する

---
スポットチェックのスクリプト: `/private/tmp/claude-501/-Users-takuya-project-20-pai-homework/b14af3ef-30ce-4f56-a63b-8ac03b5a9e70/scratchpad/traj_verify/` (spot1.py、spot2.py、spot3.py)

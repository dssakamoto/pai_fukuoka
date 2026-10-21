# TASK-0012 テストケース定義: 把持・運搬の軌道生成(trajectory / IkPolicy)

**機能名**: trajectory / **タスクID**: TASK-0012 / **要件名**: so101-vlm-pickplace
**作成日**: 2026-10-21
**作成方法**: tsumiki:tdd-testcases(ユーザー確認なし)
**前提資料**: `note.md`、`trajectory-requirements.md`(以下「要件」)、`trajectory-survey.md`(以下「調査」)
**数値の確認**: 要件 2.4〜2.9 どおりに書いた使い捨ての参照実装で、本書の期待値を確かめた(リポジトリ外: `<scratchpad>/traj_survey/ref_impl.py`、`tc_checks.py`、`tc_checks2.py`、`census.py`、`census_fine.py`、結果 `census.json`。`<scratchpad>` はセッションの一時フォルダ)。表の「実測」はこの参照実装の値。

**信頼性の凡例**: 🔵 要件・既存コード・実測どおり / 🟡 それらから妥当に推測 / 🔴 資料にない。本書で決めた仮定

**対象外(TASK-0012 では試験しない)**:
- sim でブロックを箱へ運べるか(物理設定の変更 D1 が前提。TASK-0013)。TC-IP-S01 の実行スモークもブロックの運搬は確かめない。
- 知覚誤差への強さ(調査 2.5 節の表。TASK-0024)。
- 他のブロックとの干渉(要件 2.8 で対象外)。

---

## 0. 共通事項

### 0.1 テスト専用の定数(`tests/policy/trajectory_constants.py` に置く。`tests/safety/` からは import しない)🟡

| 名前 | 値 | 根拠 |
|---|---|---|
| `VMAX` / `DT` / `SCALE` / `FACTOR` / `MIN_SEG_S` | `np.ones(6)` / 0.02 / 1.0 / 0.99 / 0.2 | `configs/motion.yaml`、要件 2.9 🔵 |
| `STEP_UNIT` | `1.0 * 1.0 * 0.02 * 0.99`(= 0.0198。掛ける順もこのとおり) | 要件 2.7 🔵 |
| `TCP_OFFSET` | (0.004, 0.0, 0.030) | 調査 2.1 (4) 🔵 |
| `G_OPEN` / `G_CLOSED` | 0.6 / 0.15 | 調査 2.1 🔵 |
| `PICK_CENTER` | (0.25, 0.0, 0.015) | 要件 4.1 🔵 |
| `PLACE_BOX` | (0.20, 0.19, 0.005)(箱の底板上面の中心) | 要件 2.2 🔵 |
| `HOME_Q` | `np.zeros(6)` | 調査 1.2 🔵 |
| `SEGMENTS` | ("start", "pre_grasp", "grasp", "close", "close_hold", "lift", "pre_place", "place", "open", "open_hold", "retreat") | 要件 2.3 🔵 |
| `SEG_TARGET` | 区間 → TCP 目標の経由点名。`close` / `close_hold` → `grasp`、`open` / `open_hold` → `place`、それ以外は同じ名前 | 要件 6.1 🟡 |
| `FK_TOL_M` | 0.002 | 要件 5 節 7 🔵(実測の最大は 0.97 mm) |
| `PITCH_TOL_DEG` | 5.0 | TASK-0011 🔵 |
| `NEAR_SHOULDER` | (0.16078614047633127, -0.023326223842896357, 0.015) | 調査 2.5 節の失敗例(`sample_layout(5)[2]`、青)🔵 |

### 0.2 合成の経由点(純粋層のテスト用。MuJoCo 不要)🟡

- 開始: `Z = zeros(6)`
- `PG = (0.3, −0.5, 0.4, 1.2, 0, 0.6)`、`GR = (0.3, −0.3, 0.6, 1.0, 0, 0.6)`
- `CL` = `GR` の gripper を 0.15 にしたもの、`LI` = `PG` の gripper を 0.15 にしたもの
- `PP = (1.0, −0.4, 0.5, 1.2, 0, 0.15)`、`PL = (1.0, −0.2, 0.7, 1.0, 0, 0.15)`
- `OP` = `PL` の gripper を 0.6 にしたもの、`RT` = `PP` の gripper を 0.6 にしたもの
- 保持: close 0.5 s、open 0.3 s

刻み数の実測(式どおりの計算。`VMAX` = 1、`DT` = 0.02、係数 0.99、`MIN_SEG_S` = 0.2):

| 区間 | pre_grasp | grasp | close | close_hold | lift | pre_place | place | open | open_hold | retreat | 合計サンプル(start 含む) | 所要時間 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| scale 1.0 | 61 | 11 | 23 | 25 | 11 | 36 | 11 | 23 | 15 | 11 | **228** | **4.54 s** |
| scale 0.3 | 203 | 34 | 76 | 25 | 34 | 118 | 34 | 76 | 15 | 34 | **650** | **12.98 s** |

### 0.3 純粋層の API(要件 2.1 / 2.3 / 2.6〜2.8 に関数名を付けたもの。Red で確定する)🔴

`src/so101_vlm/policy/trajectory.py`:
- `segment_steps(q_a, q_b, *, vmax, speed_scale, control_dt, safety_factor, min_segment_s) -> int`
- `hold_steps(hold_s, control_dt) -> int`(`round(hold_s / control_dt)`)
- `interpolate_segment(q_a, q_b, n) -> np.ndarray`(shape (n, 6)、k = 1..n)。**最後の行は `q_b` と完全に一致させる**。式 `q_a + (n/n)(q_b − q_a)` をそのまま使うと 1 ulp ずれ、`lift` = `pre_grasp` の完全一致の確認が通らなかった(実測)。
- `JointWaypoint(name, q, hold_s=None)`: `hold_s` を与えると保持区間(`q` は直前の終点と同じでなければならない)
- `build_joint_trajectory(q_start, waypoints, *, vmax, speed_scale, control_dt, safety_factor, min_segment_s, metadata=None) -> JointTrajectory`
- `check_command_sequence(q, labels, *, lower, upper, vmax, speed_scale, control_dt) -> None`(違反は `TrajectoryError("limit_violation")`)
- `find_clearance_violation(distances, labels, threshold) -> ClearanceViolation | None`(index、label、value)
- `JointTrajectory`、`TrajectoryError(reason, waypoint=None, detail=None)`、理由の定数 `REASON_*`

その他のファイル:
- `src/so101_vlm/policy/base.py`: `Observation`、`Goal`、`Policy`(`@runtime_checkable` を付ける)
- `src/so101_vlm/policy/trajectory_config.py`: `TrajectoryConfig`、`ClearanceConfig`、`load_trajectory_config`
- `src/so101_vlm/policy/ik_policy.py`: `IkPolicy`
  - **`solve_ik` はモジュールの名前空間に import する**(`from so101_vlm.policy.ik import solve_ik`)。テストで `monkeypatch.setattr(ik_policy, "solve_ik", spy)` を使うため。
  - 時間配分は `trajectory.build_joint_trajectory` に任せる。その中で `segment_steps` をモジュールの大域名として引く(TC-IP-E09 で差し替えるため)。

### 0.4 代表の配置(結合テスト)🔵 実測

開始 `HOME_Q`、置く点 `PLACE_BOX`、z = 0.015(ブロック中心)。

| 名前 | 掴む点 (x, y) | 種別 | N | 所要時間 | p1 / p2 | 最小すき間(障害物 / 自己干渉) |
|---|---|---|---|---|---|---|
| `center` | (0.25, 0.00) | **検証** | 276 | 5.50 s | 0 / 10 | 7.0 mm(place)/ 9.6 mm |
| `corner_xlo_ylo` | (0.15, −0.10) | **検証** | 329 | 6.56 s | 10 / 10 | 6.2 mm / 9.6 mm |
| `corner_xlo_yhi` | (0.15, 0.10) | **検証** | 290 | 5.78 s | 10 / 10 | 6.2 mm / 9.6 mm |
| `corner_xhi_ylo` | (0.35, −0.10) | **検証** | 287 | 5.72 s | 20 / 10 | 2.9 mm(grasp)/ 9.6 mm |
| `corner_xhi_yhi` | (0.35, 0.10) | **検証** | 257 | 5.12 s | 20 / 10 | 2.9 mm(grasp)/ 9.6 mm |
| `seed0_red` | `sample_layout(0)[0]` = (0.2774, −0.0460) | **検証** | 274 | 5.46 s | 10 / 10 | 6.5 mm / 9.6 mm |
| `seed0_blue` | `sample_layout(0)[1]` = (0.1582, −0.0967) | **検証**(肩に近い側でも解ける例) | 325 | 6.48 s | 10 / 10 | 6.4 mm / 9.6 mm |
| `seed10_red` | `sample_layout(10)[0]` = (0.3412, −0.0585) | **検証**(p1 = 20、床とのすき間 2.4 mm) | 272 | 5.42 s | 20 / 10 | 2.4 mm(grasp)/ 9.6 mm |
| seed 3 / 7 / 11 の 3 ブロック | | 記録のみ | 247〜315 | 4.92〜6.28 s | p1 0〜10 / p2 10 | ≥ 6.3 mm / 9.6 mm |

- N・所要時間・p1 の具体値は **記録のみ**(IK の解のわずかな違いで変わりうる)。テストで等号を求めるのは次の 3 つだけ:
  - p2 = 10(census の 409 件すべて 10)
  - 保持区間と開閉区間の刻み数(25 / 15 / 23)
  - 規則(metadata の p1 が `pre_grasp` の採用候補と一致する)
- 4 隅は **すべて解ける**(実測)。解けない領域は 4 隅ではなく、肩に近い x ≤ 0.1725・|y| ≤ 0.07 の帯にある(4 節の census)。
- 配置のシードは、census で解けないブロックを含まないものを選んだ(seed 0、3、7、10、11)。seed 5 は青ブロックが解けない(TC-IP-E06 で使う)。
- `act` 1 回の計算時間の実測は 0.11〜0.31 s(census 441 件の平均 0.13 s)。

### 0.5 実行時間の予算 🟡

| ファイル | 予算 | 内訳(実測) |
|---|---|---|
| `tests/policy/test_trajectory.py` | < 1.0 s | 純粋関数のみ |
| `tests/policy/test_trajectory_config.py` | < 0.5 s | YAML の読み込みのみ |
| `tests/policy/test_ik_policy.py` | < 8 s | 代表 8 配置を **module scope で 1 回だけ** 解く(約 1.8 s)、20 配置の所要時間(約 3.2 s)、異常系 約 12 回(1 回 0.1〜0.3 s)、scale 0.3 の 1 回、実行スモーク 0.4 s |
| 合計 | **< 10 s** | 20 配置の確認(3.2 s)を残したまま 6 s 以内にするのは難しい(9 節 O3) |

- IK を使うテストは fixture `solved`(名前 → `JointTrajectory` の dict、module scope)を共有する。同じ配置を何度も解かない。
- モデルは `PICKPLACE_SCENE_PATH` を module scope で 1 回だけ読む(`tests/policy/test_ik.py` と同じ)。

### 0.6 書き方(既存テストに合わせる)🔵

- docstring は日本語で `"""TC-xx-nn 名前 🔵"""`。本文に `# Given` / `# When` / `# Then` と `【確認内容】` のコメントを書く(6 節)。
- 本書の ID とテスト関数を 1 対 1 にする。パラメータ化は `pytest.mark.parametrize(ids=...)` を使う。

---

## 1. `tests/policy/test_trajectory.py`(純粋層。MuJoCo 不要)

### 1.1 正常系

| ID | テスト名 | Given | When | Then(期待値) | 信頼性 | 要件 |
|---|---|---|---|---|---|---|
| TC-TJ-01 | 刻み数: 大きな移動 | Δ = (1.0, 0, 0, 0, 0, 0)、0.1 節の値 | `segment_steps` | 51(= ceil(1.0 / 0.0198)) | 🔵 | 要件 2.7 |
| TC-TJ-02 | 刻み数: 最も大きく動く関節で決まる | Δ = (0.1, −0.5, 0.2, 0, 0, 0) | 同上 | 26(0.5 / 0.0198 = 25.25) | 🔵 | 2.7 |
| TC-TJ-03 | 刻み数: 関節ごとの vmax | vmax = (1, 1, 1, 1, 1, 0.5)、gripper の Δ 0.45 | 同上 | 46(0.45 / 0.0099 = 45.45) | 🔵 | 2.7 |
| TC-TJ-04 | 刻み数: 負の向きも同じ | Δ = (−0.45, 0, …) | 同上 | 23 | 🔵 | 2.7 |
| TC-TJ-05 | 補間: 端点と等間隔 | `Z` → `PG`、n = 61 | `interpolate_segment` | shape (61, 6)。最後の行が `PG` と **完全一致**(`np.array_equal`)。k 行目(k < 61)は `Z + (k/61)(PG − Z)`(atol 1e-12) | 🔵(式)/ 🟡(完全一致) | 2.7 |
| TC-TJ-06 | 補間: 1 刻みの変化が上限の 99% 以下 | TC-TJ-01〜04 の各入力 | `segment_steps` → `interpolate_segment` | `max |Δq_k| ≤ 0.99 × vmax × scale × dt + 1e-12` | 🔵 | 2.7、TASK-0010 |
| TC-TJ-07 | 保持の刻み数 | (0.5, 0.02)、(0.3, 0.02) | `hold_steps` | 25、15(浮動小数の割り算が 25.0 / 15.0 になることを確認済み) | 🔵 | 2.7 |
| TC-TJ-08 | 合成の軌道: 区間の順と長さ | 0.2 節の経由点(10 区間) | `build_joint_trajectory` | `segments == SEGMENTS`。各区間のサンプル数が 0.2 節の表どおり(61, 11, 23, 25, 11, 36, 11, 23, 15, 11)。N = 228 | 🔵 | 2.3、2.7、TASK-0012 テストケース 1 |
| TC-TJ-09 | 時刻 | TC-TJ-08 の結果 | `times` を見る | `times[0] == 0.0`。`times` ≈ `arange(228) × 0.02`(atol 1e-12)。`np.all(np.diff(times) > 0)`。`metadata["duration_s"]` = 4.54 ± 1e-9 | 🔵 | 2.3、TASK-0012 テストケース 3 |
| TC-TJ-10 | ラベル | 同上 | `labels` を見る | 長さ N。`labels[0] == "start"`。ラベルが `segments` の順に並ぶ(区間の添字が単調非減少)。各区間に 1 サンプル以上 | 🔵 | 2.3 |
| TC-TJ-11 | 開始値 | 同上 | `q[0]` を見る | `q[0]` が `q_start` と完全一致し、コピーである(`q_start` を後で書き換えても `q[0]` は変わらない) | 🔵 | 2.3 |
| TC-TJ-12 | 指令列の速度(前回指令が基準) | 同上 | `np.abs(np.diff(q, axis=0))`(`q[0]` → `q[1]` を含む) | 全要素が `VMAX × SCALE × DT`(= 0.02)以下。最大は 0.0198 以下 | 🔵 | 2.7、NFR-02 |
| TC-TJ-13 | 保持・開閉の区間で腕が止まる | 同上 | `close`、`close_hold`、`open`、`open_hold` の行 | 腕の 5 関節が直前の経由点と完全一致。`close_hold` / `open_hold` の行はすべて同じ値 | 🔵 | 2.3 |
| TC-TJ-14 | 経由点の終点 | 同上 | 各区間の最後の行 | 経由点の `q` と完全一致(`lift` の終点 = `LI`、`retreat` の終点 = `RT`) | 🔵 | 2.3 |
| TC-TJ-15 | 速度倍率 0.3 | 0.2 節、scale 0.3 | `build_joint_trajectory` | 区間の刻み数が 0.2 節の表どおり(203, 34, 76, 25, 34, 118, 34, 76, 15, 34)、N = 650。保持区間は倍率に依存しない(25 / 15)。連続差は 1.0 × 0.3 × 0.02 以下 | 🔵 | 2.7、NFR-07(2) |
| TC-TJ-16 | 速度倍率 0.3 で 3.3 倍以上 | Δ = (1.0, 0, …) | `segment_steps` を scale 1.0 / 0.3 で呼ぶ | 51 → 169(169 / 51 = 3.31 ≥ 3.3) | 🔵 | 2.7 |
| TC-TJ-17 | JointTrajectory は変更できない | TC-TJ-08 の結果 | `traj.q[0, 0] = 1`、`traj.times[0] = 1`、`traj.labels = ()`、`traj.metadata["x"] = 1` | 順に `ValueError`(writeable=False)、`ValueError`、`FrozenInstanceError`、`TypeError`(MappingProxyType) | 🔵 | 2.3 |
| TC-TJ-18 | 入力を変更しない | `q_start` と経由点の配列(書き込み可能なもの) | `build_joint_trajectory` | 呼ぶ前に取ったコピーと完全一致 | 🔵 | 3.2 |
| TC-TJ-19 | 決定的 | 0.2 節 | 2 回呼ぶ | `times`、`q`、`labels`、`segments` が完全一致 | 🔵 | 3.2 |
| TC-TJ-20 | 指令列の検査: 正常 | TC-TJ-08 の `q`、可動域 −2.0〜2.0(gripper は −0.2〜1.8) | `check_command_sequence` | 例外が出ない | 🔵 | 2.7 |
| TC-TJ-21 | すき間: 違反なし | 距離 (0.010, 0.005, 0.0022)、しきい値 0.001 | `find_clearance_violation` | `None` | 🟡 | 2.8 |
| TC-TJ-22 | すき間: 最小のサンプルを返す | 距離 (0.010, −0.003, −0.010, 0.004)、ラベル (pre_grasp, grasp, grasp, close)、しきい値 0.002 | 同上 | index 2、label "grasp"、value −0.010 | 🟡 | 2.8 |
| TC-TJ-23 | Observation の作成 | `RobotState(q, dq, timestamp=1.5)` | `Observation.from_state` | `q` は値が等しいコピーで読み取り専用。`timestamp == 1.5` | 🟡 | 2.2 |
| TC-TJ-24 | Goal の作成 | tuple の `PICK_CENTER`、`PLACE_BOX` | `Goal(...)` | (3,) の float64 配列で読み取り専用。元の list を変えても影響しない。`instruction` の既定は "" | 🟡 | 2.2 |
| TC-TJ-25 | TrajectoryError の属性 | `TrajectoryError("self_collision", waypoint="grasp", detail={"min": -0.01})` | 属性を見る | `RuntimeError` の派生。`reason` / `waypoint` / `detail` を持ち、`detail` は変更できない。メッセージに経由点の名前を含む | 🟡 | 2.6 |

### 1.2 異常系

| ID | テスト名 | Given | When | Then | 信頼性 | 要件 |
|---|---|---|---|---|---|---|
| TC-TJ-E01 | JointTrajectory: 長さが合わない | times 5 個、q (4, 6)、labels 5 個 | 生成 | `ValueError` | 🟡 | 2.3 |
| TC-TJ-E02 | JointTrajectory: q の shape | q (5, 5) | 生成 | `ValueError` | 🟡 | 2.3 |
| TC-TJ-E03 | JointTrajectory: N < 2 | N = 1 | 生成 | `ValueError` | 🟡 | 2.3 |
| TC-TJ-E04 | JointTrajectory: 時刻が増えない | times (0, 0.02, 0.02) | 生成 | `ValueError` | 🟡 | 2.3 |
| TC-TJ-E05 | JointTrajectory: 有限でない値 | q に NaN、times に inf(parametrize) | 生成 | `ValueError` | 🟡 | 2.3 |
| TC-TJ-E06 | 指令列: 速度の超過 | 2 行目の差だけ 0.0201(上限 0.02 を 0.5% 超える) | `check_command_sequence` | `TrajectoryError`、`reason == "limit_violation"`、`waypoint` はその行のラベル | 🟡 | 2.6、2.7 |
| TC-TJ-E07 | 指令列: 可動域の外 | `q[1:]` の 1 つの値が upper + 1e-6 | 同上 | `limit_violation` | 🟡 | 2.7 |
| TC-TJ-E08 | 指令列: q[0] は可動域を検査しない | `q[0]` だけ範囲外(開始時の計測値)。他は範囲内で、差も上限内 | 同上 | 例外が出ない | 🟡 | 2.7(検査は `q[1:]`) |
| TC-TJ-E09 | 保持区間の q が直前と違う | `JointWaypoint("close_hold", q≠CL, hold_s=0.5)` | `build_joint_trajectory` | `ValueError` | 🔴 | 0.3 節の API |
| TC-TJ-E10 | Observation の検証 | q の shape (5,)、NaN、inf、bool の配列(parametrize) | `Observation(q, 0.0)` | `ValueError` | 🟡 | 2.2 |
| TC-TJ-E11 | Goal の検証 | 長さ 2、NaN、inf、`True`、`instruction=1`(parametrize) | `Goal(...)` | `ValueError` | 🟡 | 2.2 |
| TC-TJ-E12 | 刻み数の引数 | `speed_scale` が 0 / 負、`control_dt` が 0、`safety_factor` が 1.5、Δ に NaN | `segment_steps` | `ValueError` | 🟡 | 2.7(入力の誤り) |

### 1.3 境界値

| ID | テスト名 | Given | When | Then | 信頼性 | 要件 |
|---|---|---|---|---|---|---|
| TC-TJ-B01 | ちょうど割り切れる | Δ = 23 × `STEP_UNIT`(比が浮動小数でも 23.0 ちょうどになることを確認済み) | `segment_steps`(min_segment_s = 0.02) | 23(24 にならない) | 🔵 | 2.7 |
| TC-TJ-B02 | わずかに超える | Δ = 23 × `STEP_UNIT` + 1e-9 | 同上 | 24 | 🔵 | 2.7 |
| TC-TJ-B03 | 動かない区間は最短時間になる | Δ = 0、min_segment_s 0.2 | 同上 | 10(ceil(0.2 / 0.02) が 11 にならないことを確認済み) | 🔵 | 2.7 |
| TC-TJ-B04 | 最短時間が 1 刻み以下 | Δ = 0、min_segment_s 0.02 | 同上 | 1(`max(1, …)`) | 🔵 | 2.7 |
| TC-TJ-B05 | 最短時間が移動時間より長い | Δ = 0.1(6 刻み分)、min_segment_s 0.2 | 同上 | 10 | 🔵 | 2.7 |
| TC-TJ-B06 | 保持 0 秒 | `hold_steps(0.0, 0.02)` | | 0。区間が 0 サンプルになったとき `segments` に残すかは Red で決める(9 節 O5)。本テストは `hold_steps` だけを確かめる | 🔴 | 2.7 |
| TC-TJ-B07 | 係数 1.0 | Δ = 23 × 0.02、係数 1.0 | `segment_steps` | 23。連続差は上限ちょうど(0.02)で、TC-TJ-12 の判定(≤)を満たす | 🟡 | 2.7 |
| TC-TJ-B08 | 速度の上限ちょうど | 差がちょうど 0.02 の列 | `check_command_sequence` | 例外が出ない(≤ は合格。`apply_safety` と同じ) | 🟡 | TASK-0010 |
| TC-TJ-B09 | すき間がしきい値ちょうど | 距離 (0.001,)、しきい値 0.001 | `find_clearance_violation` | `None`(≥ は合格) | 🟡 | 2.8 |

---

## 2. `tests/policy/test_trajectory_config.py`

書き方は `tests/safety/test_config.py` と同じ(`VALID` の dict を `_with` / `_without` で変えて `tmp_path` に書く)。`VALID` には `safety:` と `trajectory:` の両方を入れる。

### 2.1 正常系

| ID | テスト名 | Given | When | Then | 信頼性 | 要件 |
|---|---|---|---|---|---|---|
| TC-TC-01 | 既定のファイルを読める | `configs/motion.yaml`(`trajectory:` を追加したもの) | `load_trajectory_config()` | 例外なく `TrajectoryConfig` を返す | 🔵 | 2.9 |
| TC-TC-02 | 既定値 = 調査の推奨値 | 同上 | 各値を見る | `tcp_offset == (0.004, 0.0, 0.030)`、`approach_height` 0.08、`min_carry_z` 0.08、`place_height` 0.030、`gripper_open` 0.6、`gripper_closed` 0.15、`close_hold_s` 0.5、`open_hold_s` 0.3、`min_segment_s` 0.2、`time_safety_factor` 0.99、`clearance.obstacle_m` 0.001、`clearance.self_m` 0.002、`obstacle_geoms == ("floor", "box_floor", "box_wall_px", "box_wall_nx", "box_wall_py", "box_wall_ny")` | 🔵 | 調査 3 節 |
| TC-TC-03 | `safety:` は変わらず読める | 既定のファイル | `load_safety_config()` | TASK-0010 の値のまま(vmax 1.0 × 6、dt 0.02、scale 1.0、timeout 0.2、workspace (0.12, −0.13, 0)〜(0.38, 0.26, 0.17)) | 🔵 | 2.9、note 2 節 |
| TC-TC-04 | `trajectory:` は `safety:` の中にない | 既定のファイルを `yaml.safe_load` | キーを見る | 最上位のキーは {"safety", "trajectory"}。`safety` のキーは TASK-0010 と同じ 5 個 | 🔵 | 2.9 |
| TC-TC-05 | frozen と tuple | 読んだ設定 | 代入する、型を見る | 代入は `FrozenInstanceError`。`tcp_offset`、`obstacle_geoms` は tuple。`clearance` は `ClearanceConfig` | 🟡 | 2.9 |
| TC-TC-06 | list で渡しても tuple にそろう | `TrajectoryConfig(tcp_offset=[0.004, 0, 0.03], ...)` | 生成 | float の tuple | 🟡 | 2.9 |
| TC-TC-07 | `dataclasses.replace` で変えられる | 既定の設定 | `replace(cfg, approach_height=0.05)` | 新しいオブジェクトを返し、元は 0.08 のまま(TC-IP-E04 で使う) | 🟡 | 3.2 |

### 2.2 異常系(`TC-TC-E01` 以外は `ValueError`。メッセージにキーの名前を含む)

| ID | テスト名 | 入力 | 信頼性 |
|---|---|---|---|
| TC-TC-E01 | ファイルがない | 存在しないパス → `FileNotFoundError` | 🔵(safety と同じ) |
| TC-TC-E02 | `trajectory:` がない | `safety:` だけのファイル | 🟡 |
| TC-TC-E03 | 最上位が mapping でない | `"- 1\n- 2"` | 🟡 |
| TC-TC-E04 | 未知のキー | `trajectory.carry_height: 0.1` | 🟡 |
| TC-TC-E05 | キーの欠落 | `trajectory.gripper_open` を消す、`clearance.self_m` を消す(parametrize) | 🟡 |
| TC-TC-E06 | `clearance:` の未知のキー | `clearance.block_m: 0.001` | 🟡 |
| TC-TC-E07 | bool / 文字列 / NaN / inf | 数値のキーに `True`、`"0.1"`、`.nan`、`.inf`(parametrize) | 🟡 |
| TC-TC-E08 | `tcp_offset` の長さ | 長さ 2、長さ 4、スカラー | 🟡 |
| TC-TC-E09 | 正であるべき値が 0 以下 | `approach_height`、`min_carry_z`、`place_height`、`min_segment_s` に 0 と −0.01 | 🟡 |
| TC-TC-E10 | 保持時間が負 | `close_hold_s: -0.1` | 🟡 |
| TC-TC-E11 | 係数の範囲 | `time_safety_factor` が 0、1.01、−0.5 | 🟡 |
| TC-TC-E12 | 開き角 ≤ 閉じ角 | 0.15 と 0.15(等しい)、0.1 と 0.6 | 🟡 |
| TC-TC-E13 | すき間が負 | `obstacle_m: -0.001` | 🟡 |
| TC-TC-E14 | `obstacle_geoms` の形 | 空の列、空文字、数値の要素、文字列そのもの `"floor"` | 🟡 |

### 2.3 境界値

| ID | テスト名 | 入力 | 期待 | 信頼性 |
|---|---|---|---|---|
| TC-TC-B01 | 係数ちょうど 1 | `time_safety_factor: 1.0` | 受け付ける | 🟡 |
| TC-TC-B02 | 保持 0 秒 | `close_hold_s: 0.0` | 受け付ける | 🟡 |
| TC-TC-B03 | すき間 0 | `obstacle_m: 0.0`、`self_m: 0.0` | 受け付ける | 🟡 |
| TC-TC-B04 | 整数 | `approach_height: 1`(int) | 受け付けて float の 1.0 にする | 🟡 |

---

## 3. `tests/policy/test_ik_policy.py`(実モデル `PICKPLACE_SCENE_PATH`)

fixture(module scope): `model`、`policy = IkPolicy(model)`、`lower, upper = limits_from_model(model)`、`safety_cfg = load_safety_config()`、`solved`(0.4 節の「検証」8 配置。`HOME_Q` から `act` した結果)。

共通ヘルパ:
- `run_safety(traj, cfg)`: `prev = traj.q[0]` から始め、k = 1..N−1 で `apply_safety(prev, traj.q[k], lower=..., upper=..., last_state_t=0.0, now=0.0, config=cfg, target_base=wp_target[SEG_TARGET[traj.labels[k]]])` を呼ぶ。`prev = result.q_command`。返り値はフラグが立った周期の数と、出力が入力と違った周期の数。
- `fk_tcp(q)`: 独立した `MjData` と `mj_kinematics` で、gripperframe + R · `TCP_OFFSET` を返す(`test_ik.py` の `fk_point` と同じ方式)。

### 3.1 正常系

| ID | テスト名 | Given | When | Then(期待値) | 信頼性 | 要件 |
|---|---|---|---|---|---|---|
| TC-IP-01 | Policy Protocol に適合 | `IkPolicy(model)` | `isinstance(policy, Policy)` | True | 🟡 | FR-11、2.2 |
| TC-IP-02 | 区間の順 | `solved` の 8 配置(parametrize) | `segments`、`labels` | `segments == SEGMENTS`。`labels[0] == "start"`。各区間に 1 サンプル以上 | 🔵 | TASK-0012 テストケース 1 |
| TC-IP-03 | 開始値 | 同上 | `q[0]` | `obs.q` と完全一致 | 🔵 | 2.3 |
| TC-IP-04 | 時刻 | 同上 | `times` | `times[k] ≈ k × 0.02`(atol 1e-12)、厳密に単調増加、`duration_s == times[-1]` | 🔵 | TASK-0012 テストケース 3 |
| TC-IP-05 | 経由点の関節角の関係 | 同上 | 区間の終点 | `lift` の終点 = `pre_grasp` の終点の腕 + `G_CLOSED`。`retreat` の終点 = `pre_place` の終点の腕 + `G_OPEN`。`close` / `close_hold` の腕 = `grasp` の終点。`open` / `open_hold` の腕 = `place` の終点。いずれも完全一致 | 🔵 | 2.3 |
| TC-IP-06 | gripper の値 | 同上 | 各区間の終点の `q[5]` | pre_grasp と grasp が 0.6、close〜place が 0.15、open〜retreat が 0.6。`close` / `open` の区間は 23 刻み(\|0.45\| / 0.0198 → 23、実測)、保持は 25 / 15 刻み | 🔵 | 2.3、調査 2.4 |
| TC-IP-07 | wrist_roll が一定 | 同上 | `q[:, 4]` | 全行が `clip(obs.q[4], ctrlrange)`(`HOME_Q` では 0.0) | 🟡 | 2.7 |
| TC-IP-08 | **安全フィルタを通しても何も起きない** | 同上 | `run_safety(traj, safety_cfg)` | 全周期で `clipped` / `rate_limited` / `rejected` / `stopped` がすべて False、`q_command == q[k]`(実測: 全配置で 0 件) | 🔵 | TASK-0012 テストケース 2、NFR-02、要件 3.1 |
| TC-IP-09 | 連続差と可動域(前回指令が基準) | 同上 | `np.abs(np.diff(q, axis=0))` | 全要素が `vmax × scale × dt` 以下(実測の最大比 0.990)。`q[1:]` は `lower`〜`upper` の中 | 🔵 | 2.7 |
| TC-IP-10 | 経由点の TCP が目標に合う | 同上 | `metadata["waypoints"]` の各 q を独立 FK にかける | TCP と目標の差が 2 mm 以下(実測の最大 0.97 mm)。目標は要件 2.5 の式どおり(`pre_grasp = P + (0, 0, 0.08)`、`place = (S.x, S.y, S.z + 0.030)` = z 0.035 など) | 🔵 | 要件 5 節 7 |
| TC-IP-11 | 経由点の目標は作業範囲の中 | 同上 | 6 つの TCP 目標 | すべて `is_in_workspace(..., safety_cfg.workspace)` | 🔵 | NFR-07(3) |
| TC-IP-12 | 共通ピッチの規則 | 同上 | `metadata` と `waypoints` | `pick_pitch_deg` が `pre_grasp` の採用候補と同じで、`grasp` も同じ候補で解いている。`place_pitch_deg == 10.0`(全配置)。各経由点の実際の傾きと候補の差は 5° 以内(実測で pre_place は 14.6° 対 10°) | 🔵 | 要件 2.5、TASK-0011 |
| TC-IP-13 | metadata のキー | `center` | `metadata` | キー `control_dt`、`vmax`、`speed_scale`、`time_safety_factor`、`tcp_offset`、`pick_pitch_deg`、`place_pitch_deg`、`waypoints`(名前の順は pre_grasp、grasp、lift、pre_place、place、retreat)、`min_obstacle_clearance_m`、`min_self_clearance_m`、`duration_s`。値は 0.02、(1.0,)×6、1.0、0.99、(0.004, 0, 0.03)。`pick_pitch_deg == 0.0`(実測) | 🟡 | 2.3 |
| TC-IP-14 | すき間の値 | `solved` の 8 配置 | metadata | `min_obstacle_clearance_m ≥ 0.001`(実測の最小 2.4 mm)、`min_self_clearance_m ≥ 0.002`(実測は全件 9.6 mm) | 🔵 | 2.8 |
| TC-IP-15 | 補間した TCP 経路 | 同上 | 全サンプルの FK による TCP | `pre_grasp` 区間の終点より後はすべて作業範囲の中。`pre_place` 区間の TCP z の最小が 0.09 以上(実測 0.0949、調査では 0.0942) | 🔵 | 要件 2.4 の 8、調査 2.3 |
| TC-IP-16 | 速度倍率 0.3 | `center`、`IkPolicy(model, safety=replace(cfg, speed_scale=0.3))` | `act` と、0.3 の設定での `run_safety` | 移動区間(保持を除き、`min_segment_s` で決まった区間も除く)で `n_0.3 ≥ (n_1.0 − 1) / 0.3`。移動区間の合計の比 ≥ 3.2(実測 3.28)。保持は 25 / 15 のまま。N = 811、16.2 s(記録のみ)。フラグは全周期で False(実測 0 件) | 🔵 | NFR-07(2) |
| TC-IP-17 | 20 配置の所要時間 | `sample_layout(seed)[0]`(赤)、seed 0〜19 | `act` | 全配置で解ける。平均が 4.5〜7.5 s(実測 5.39 s、最小 4.92、最大 6.28)。1 配置ずつは 4.0〜7.5 s | 🔵 | 要件 3.1、調査 2.4 |
| TC-IP-18 | 入力を変更しない | 書き込み可能な配列から作った obs / goal、`model` | `act` の前後 | `obs.q`、`goal` の配列が変わらない。`model.qpos0`、`actuator_ctrlrange`、`opt.timestep` が呼ぶ前のコピーと同じ | 🔵 | 3.2 |
| TC-IP-19 | 決定的 | `center` を 2 回 | `act` | `q`、`times`、`labels`、metadata のピッチが完全一致(実測で一致) | 🔵 | 3.2 |
| TC-IP-20 | 開始姿勢が pre_grasp と同じ | `obs.q` = `center` の `pre_grasp` 区間の終点 | `act` | `pre_grasp` 区間が 10 刻み(`min_segment_s`、実測)。時刻は単調増加のまま。p1 は同じ(0°) | 🟡 | 要件 4.2 |
| TC-IP-21 | 結果が変更できない | `center` | `traj.q[0, 0] = 1` | `ValueError` | 🔵 | 2.3 |
| TC-IP-22 | IK に渡す引数 | `center`、`solve_ik` を包んだスパイ | `act` | 4 回呼ばれる(pre_grasp、grasp、pre_place、place)。`tcp_offset` はすべて (0.004, 0, 0.030)。grasp は `pitch_candidates_deg == (p1,)` でシードが pre_grasp の解、place は `(p2,)`。lift と retreat では IK を呼ばない | 🔵 | 要件 2.5 |

### 3.2 異常系(すべて `TrajectoryError`。部分的な軌道は返さない)

| ID | テスト名 | Given | When | Then(期待値、実測) | 信頼性 | 要件 |
|---|---|---|---|---|---|---|
| TC-IP-E01 | 作業範囲外(x)は IK の前に拒否 | P = (0.45, 0, 0.015)、`solve_ik` のスパイ | `act` | `reason == "target_out_of_workspace"`、`waypoint == "pre_grasp"`、スパイは **0 回** | 🔵 | NFR-07(3)、2.4 の 3 |
| TC-IP-E02 | 作業範囲外(y、z)| P = (0.25, 0.27, 0.015) → `pre_grasp`。P = (0.25, 0, −0.01) → `grasp`(parametrize) | 同上 | 理由は同じ。waypoint は記載のとおり。IK は 0 回 | 🔵 | 同上 |
| TC-IP-E03 | 置く点が高すぎる | S = (0.20, 0.19, 0.15)(place の z が 0.18 > 0.17) | 同上 | `target_out_of_workspace`、`waypoint == "place"`、IK は 0 回 | 🔵 | 要件 4.2 |
| TC-IP-E04 | 運搬が低すぎる | `config = replace(既定, approach_height=0.05)`、`center`(lift の z = 0.065 < 0.08) | 同上 | `reason == "carry_too_low"`、`waypoint == "lift"`、`detail` に z 0.065、IK は 0 回 | 🔴(理由)/ 🔵(値) | 2.4 の 4 |
| TC-IP-E05 | IK の未収束を包む(置く側) | S = (0.20, 0.19, 0.14)(place の z 0.17 は範囲内だが解けない) | `act` | `reason == "not_converged"`、`waypoint == "place"`、`isinstance(err.__cause__, IkError)`、`err.__cause__.reason == "not_converged"`。IK は 4 回呼ばれる(実測 0.09 s) | 🔵 | FR-06、2.6 |
| TC-IP-E06 | 肩の近くの点(調査の失敗例) | P = `NEAR_SHOULDER` | `act` | `not_converged`、`waypoint == "pre_grasp"`、`__cause__` は `IkError`(調査 2.5 節の記録どおり。実測でも IK 1 回目で失敗) | 🔵 | 調査 2.5 |
| TC-IP-E07 | 届かない(out_of_reach)を包む | スパイで 1 回目の呼び出しに `IkError(reason="out_of_reach")` を投げさせる | `act` | `reason == "out_of_reach"`、`waypoint == "pre_grasp"`、`__cause__` は投げた例外そのもの(`is`)。※ 作業範囲の中の点は到達球の外にならないため、実モデルでは起こせない | 🟡 | FR-06 |
| TC-IP-E08 | 経由点の名前が正しく付く | スパイで 3 回目(pre_place)に `not_converged` を投げさせる | `act` | `waypoint == "pre_place"`、`detail` に目標点がある | 🟡 | 2.6 |
| TC-IP-E09 | 防御: 指令列の検査 | `monkeypatch.setattr(trajectory, "segment_steps", lambda *a, **k: 1)` | `act`(`center`) | `reason == "limit_violation"` | 🟡 | 2.4 の 7 |
| TC-IP-E10 | 自己干渉(肘を折り畳んだ解) | P = (0.17, 0.0, 0.015)(census で既定値のまま自己干渉になる点) | `act` | `reason == "self_collision"`、`waypoint == "grasp"`、`detail["min"] < 0`(実測 −5.1 mm) | 🔵 | 2.8、4 節 |
| TC-IP-E11 | 自己干渉: 調査の失敗例の再現 | `NEAR_SHOULDER`、`config` の `tcp_offset` を (0, 0, 0.030) にする(調査で失敗した値) | `act` | `reason in {"obstacle_clearance", "self_collision"}`、`waypoint == "grasp"`、`detail["min"] < 0`(下の注) | 🔵 | 調査 2.5 |
| TC-IP-E12 | 障害物のしきい値を上げる | `center`、`clearance.obstacle_m = 0.010`(最小 7.0 mm より大きい) | `act` | `reason == "obstacle_clearance"`、`waypoint` は {"place", "open", "open_hold"} のどれか(実測 "place")、`detail["min"]` ≈ 0.0070 | 🟡 | 2.8 |
| TC-IP-E13 | 自己干渉のしきい値を上げる | `center`、`clearance.self_m = 0.020`(常に 9.6 mm) | `act` | `reason == "self_collision"`(実測は waypoint "pre_grasp"、`detail["min"]` ≈ 0.0096。waypoint は記録のみ) | 🟡 | 2.8 |
| TC-IP-E14 | 補間した経路が作業範囲の外へ出る | `center`、`safety = replace(cfg, workspace=WorkspaceBox((0.12, −0.13, 0.0), (0.252, 0.26, 0.17)))`(目標の x は最大 0.25 で範囲内) | `act` | `reason == "path_out_of_workspace"`、`waypoint == "grasp"`(降下の途中で TCP x が 0.2526 まで張り出す。実測) | 🟡 | 2.4 の 8 |
| TC-IP-E15 | 失敗しても使い続けられる | TC-IP-E10 の直後に同じ policy で `center` | `act` | 正常に返し、`solved["center"]` と完全一致(状態を持ち越さない) | 🟡 | 3.2 |
| TC-IP-E16 | 入力の誤りは ValueError | `obs` の代わりに `None`、`goal` の代わりに dict | `act` | `ValueError`(TrajectoryError ではない) | 🟡 | 2.6 |
| TC-IP-E17 | 構築時の検査 | (a) `obstacle_geoms` に存在しない名前 "no_such_geom"、(b) `gripper_open = 2.0`(ctrlrange の上限 1.745 より上)、(c) site がないモデル(`SO101_SCENE_PATH` は site を持つため、箱の geom がないことで (a) と同じになる) | `IkPolicy(...)` | `ValueError`(名前を含む) | 🟡 | 2.4、2.9 |

**TC-IP-E11 の注(調査の失敗例の再現)**: `tcp_offset = (0, 0, 0.030)` では、`NEAR_SHOULDER` の把持姿勢が床に −1.6 mm、自己干渉が −10.3 mm になる(実測。q = (0.1771, −1.5682, 1.5033, 1.3098, 0, 0.6)、組は shoulder と gripper)。**どちらの検査を先に行うかで理由が変わる**。参照実装は障害物 → 自己干渉の順で、`obstacle_clearance`(grasp、−1.59 mm)になった。`obstacle_m` を負にして障害物の検査を外すと `self_collision`(grasp、−10.34 mm)。本書の既定は「障害物 → 自己干渉の順で判定」(🔴、9 節 O4)。テストでは `reason in {"obstacle_clearance", "self_collision"}` と `waypoint == "grasp"` を確かめ、`detail["min"] < 0` も確かめる。

### 3.3 境界値

| ID | テスト名 | Given | When | Then | 信頼性 | 要件 |
|---|---|---|---|---|---|---|
| TC-IP-B01 | 運搬の高さが下限ちょうど | `approach_height = 0.065`、`center`(lift の z = 0.080 = `min_carry_z`) | `act` | 例外なし(≥ は合格。実測 N = 260) | 🟡 | 2.4 の 4 |
| TC-IP-B02 | 作業範囲の角の上の目標 | `place` の z がちょうど 0.17(S.z = 0.14) | `act` | 作業範囲の検査は通り(境界を含む)、IK で `not_converged`(= TC-IP-E05)。`target_out_of_workspace` に **ならない** | 🔵 | TASK-0010(境界を含む) |
| TC-IP-B03 | 配置範囲の 4 隅 | 0.4 節の 4 隅 | `act` | すべて解ける(TC-IP-02〜15 に含む) | 🔵 | FR-12 |
| TC-IP-B04 | 肩に近い側で解ける最も近い点 | (0.18, 0.0, 0.015)(census: x ≥ 0.175 はすべて解ける) | `act` | 例外なし。自己干渉のすき間 ≥ 2 mm | 🔵 | 4 節 |

### 3.4 実行スモーク(任意、1 件)

| ID | テスト名 | Given | When | Then | 信頼性 | 要件 |
|---|---|---|---|---|---|---|
| TC-IP-S01 | SimRobot で最後まで動く | `SimRobot(PICKPLACE_SCENE_PATH)`、`obs = Observation.from_state(robot.get_state())`、`center` | `q[1:]` を 1 周期に 1 つずつ、`apply_safety`(前回指令が基準、`last_state_t = now = state.timestamp`、`target_base` = 区間の目標)を通して `send_command` する。最後に `q[-1]` を 25 周期送る | 全周期でフラグなし(実測 0 件)。軌道の終了時の関節誤差が 0.01 rad 以下(実測の最大 0.0044)、25 周期後は 0.002 rad 以下(実測 0.0004)。**ブロックの位置は確かめない**(物理設定 D1 は TASK-0013)。実測 0.4 s | 🟡 | 要件 6.1 |

---

## 4. census(配置範囲全体の到達性。記録のみ。テストにはしない)

### 4.1 方法

- 参照実装(要件どおり。tcp (0.004, 0, 0.030)、開き 0.6 / 閉じ 0.15、place 0.035、保持 0.5 / 0.3、min_segment 0.2、すき間 1 mm / 2 mm、ホーム `q = 0` から)で、掴む点を x 0.15〜0.35、y −0.10〜0.10 の **1 cm 格子(21 × 21 = 441 点)**、z 0.015 に置き、置く点を `PLACE_BOX` にした。
- 境界の付近は 2.5 mm × 5 mm の細かい格子(x 0.170〜0.195、y −0.045〜0.045)で確かめた。
- `sample_layout(seed)` の実際の分布として、seed 0〜299 の全 900 ブロックのうち、pan 軸 (0.0388, 0) から 0.15 m 未満の 119 個を実際に解いた(それより遠い 781 個は格子の結果から解ける)。
- 計算時間 60 s(441 点)。

### 4.2 結果

| 項目 | 値 |
|---|---|
| 解けない点(1 cm 格子) | **32 / 441 = 7.3%** |
| 理由の内訳 | `not_converged`(pre_grasp)18 点、`self_collision`(grasp)12 点、`self_collision`(pre_grasp)2 点。他の理由(障害物、経路、作業範囲)は 0 |
| 解けない領域 | x 0.15 で \|y\| ≤ 0.07、x 0.16 で \|y\| ≤ 0.05、x 0.17 で −0.03 ≤ y ≤ 0.02。細かい格子では x ≤ 0.1725 だけ(**x ≥ 0.175 はすべて解ける**) |
| 解けた 409 点 | 所要時間 4.80〜6.58 s(平均 5.51 s)。p1 は 0°: 157、10°: 215、20°: 37。p2 は全件 10°。障害物の最小すき間 2.29 mm、自己干渉の最小 9.64 mm |
| `sample_layout` の実分布(seed 0〜299) | 解けないブロック **58 / 900 = 6.4%**。1 つ以上解けないブロックを含む配置は **58 / 300 = 19.3%**。赤(添字 0)だけでは 15 / 300 = 5.0% |

y ごとの「この x 以上はすべて解ける」最小の x(1 cm 格子):

| y | −0.10〜−0.08 | −0.07, −0.06 | −0.05, −0.04 | −0.03〜0.02 | 0.03〜0.05 | 0.06, 0.07 | 0.08〜0.10 |
|---|---|---|---|---|---|---|---|
| 最小の x | 0.15 | 0.16 | 0.17 | 0.18 | 0.17 | 0.16 | 0.15 |

- 解けない点はすべて pan 軸から 0.135 m 未満の範囲にある。調査の 60 件(seed 0〜19)で解けなかった 1 件(`NEAR_SHOULDER`)はこの範囲の例。
- 自己干渉の 14 点は IK が肘を折り畳んだ解を返したもの。すき間の確認(要件 2.8)が無ければ、このまま実行されていた。

### 4.3 推奨(**未決事項。ユーザーが決める。本タスクではリポジトリを変更しない**)

| 案 | 内容 | 効果 | 影響 |
|---|---|---|---|
| **A(推奨)** | `SCENE_SPEC.work_area_x` の下限(=`sample_layout` の範囲)を 0.15 → **0.18** に上げる(0.175 に 5 mm の余裕) | 配置で解けない点が 0 になる | 配置範囲は 17 cm × 20 cm。`sample_layout` は中心 ± area/2 の正方形なので、中心と `area` の作り方を変える必要がある(TASK-0006 のテストと FR-12 の「20 cm 四方」の記述に影響) |
| B | 範囲は変えず、`sample_layout` で pan 軸から 0.14 m 未満の点を棄却する | 形は保ったまま解けない点を除ける | 棄却の条件が 1 つ増える |
| C | 変えない。解けない配置は FR-06 どおり「動かずに失敗」として成功率に含める | 変更なし | 評価(TASK-0026)の成功率が配置で最大約 19% 下がる(どれか 1 つ解けないブロックを含む配置。赤だけなら 5%) |

---

## 5. 開発言語・テストフレームワーク

- **言語**: Python 3.12(`pyproject.toml` の `requires-python`)。numpy の配列比較(`np.array_equal`、`np.testing.assert_allclose`)で期待値をそのまま書ける。🔵
- **テストフレームワーク**: pytest(`uv run pytest`)。`monkeypatch` で `solve_ik` と `segment_steps` を差し替え、`tmp_path` で YAML を書き、module scope の fixture で IK の結果を共有する。MuJoCo(`mujoco`)は実モデルの FK と `mj_geomDistance` に使う。🔵
- **実行**: `uv run pytest tests/policy/test_trajectory.py tests/policy/test_trajectory_config.py tests/policy/test_ik_policy.py`。🔵

---

## 6. 実装時の日本語コメントの書き方(Python)

```python
def test_trajectory_safety_passthrough(solved, ...):
    """TC-IP-08 安全フィルタを通しても何も起きない 🔵"""
    # 【テスト目的】: 生成した軌道が可動域・速度上限を守ることを、TASK-0010 の apply_safety で確かめる
    # 【テスト内容】: 前回の指令を基準にして全周期を通し、フラグと出力を見る
    # 【期待される動作】: 全周期でフラグが立たず、出力 = 入力
    # Given
    # 【テストデータ準備】: module scope で解いた代表配置の軌道を使う(IK を何度も解かない)
    # 【初期条件設定】: 基準は q[0](開始時の計測値)
    # When
    # 【実際の処理実行】: apply_safety を k = 1..N-1 で呼ぶ。target_base は区間の終点の TCP 目標
    # Then
    # 【結果検証】: フラグの数が 0、出力の不一致が 0
    assert n_flagged == 0  # 【確認内容】: クリップ・速度制限・拒否・停止が起きない 🔵
```

- fixture には `# 【テスト前準備】`(何を一度だけ用意するか)、monkeypatch には `# 【状態復元】`(pytest が自動で戻す)を書く。

---

## 7. 要件との対応

| 要件の節 | テスト |
|---|---|
| 2.2 Observation / Goal / Policy | TC-TJ-23, 24, E10, E11、TC-IP-01, E16 |
| 2.3 JointTrajectory | TC-TJ-08〜14, 17, E01〜E05、TC-IP-02〜07, 13, 21 |
| 2.4 act の処理手順 | 手順 3: TC-IP-E01〜E03、手順 4: TC-IP-E04, B01、手順 5: TC-IP-E05〜E08, 22、手順 7: TC-IP-E09、手順 8: TC-IP-15, E14、手順 9: TC-IP-14, E10〜E13 |
| 2.5 経由点とピッチ | TC-IP-10, 12, 22 |
| 2.6 TrajectoryError | TC-TJ-25, E06、TC-IP-E01〜E17 |
| 2.7 時間配分・補間・指令列 | TC-TJ-01〜16, 20, B01〜B08、TC-IP-08, 09, 16, 20 |
| 2.8 FK のすき間 | TC-TJ-21, 22, B09、TC-IP-14, E10〜E13 |
| 2.9 設定 | TC-TC-01〜07, E01〜E14, B01〜B04、TC-IP-E17 |
| 3.1 NFR-02 / NFR-07(3) / FR-06 / 所要時間 | TC-IP-08, 09, 11, E01, E05, 17 |
| 3.2 不変性・決定性 | TC-TJ-17〜19、TC-IP-18, 19, 21, E15 |
| 6.1 TASK-0013 への申し送り(実行の仕方) | TC-IP-S01 |
| TASK-0012.md テストケース 1 / 2 / 3 | TC-TJ-08 と TC-IP-02 / TC-IP-08 / TC-TJ-09 と TC-IP-04 |

---

## 8. 件数と信頼性

| ファイル | 正常系 | 異常系 | 境界値 | その他 | 計 |
|---|---|---|---|---|---|
| test_trajectory.py | 25 | 12 | 9 | | 46 |
| test_trajectory_config.py | 7 | 14 | 4 | | 25 |
| test_ik_policy.py | 22 | 17 | 4 | スモーク 1 | 44 |
| **合計** | **54** | **43** | **17** | **1** | **115** |

信頼性の分布(ID 単位。🔵/🟡 のように 2 つ付けたものは低い方で数えた): 🔵 57(50%)、🟡 55(48%)、🔴 3(3%: TC-TJ-E09、TC-TJ-B06、TC-IP-E04 の理由名)。このほか 0.3 節の API 名が 🔴。

**品質判定: ✅ 高品質**
- 正常系・異常系・境界値をそろえた。要件 2.4 の手順 3〜9 と、2.6 の全理由に失敗のテストがある。
- 期待値は具体的な数値で、参照実装で確かめた(4 隅を含む配置、刻み数、異常系の理由と経由点、scale 0.3、20 配置の所要時間、実行スモーク)。
- 技術(pytest、numpy、MuJoCo)は既存のテストと同じ。実行時間の予算は 10 s 以内。

---

## 9. 未決事項

| # | 内容 | 本書の既定 |
|---|---|---|
| O1 | **census の結果: 配置範囲の下限 x を上げるか**(4.3 節の A / B / C) | 決めない(ユーザーが判断)。推奨は A(0.15 → 0.18) |
| O2 | 0.3 節の純粋層の関数名・`JointWaypoint` の形 | Red で確定 🔴 |
| O3 | 実行時間の予算 6 s は、20 配置の確認(約 3.2 s)を残すと超える | 合計 < 10 s とした。6 s が必須なら TC-IP-17 を 10 配置にする(seed 0〜9 の赤の平均は 5.39 s 前後)か、すき間確認の高速化を Refactor で行う |
| O4 | 障害物と自己干渉が同時に起きたとき、どちらの理由にするか | 障害物 → 自己干渉の順で判定(TC-IP-E11 は両方を許す) 🔴 |
| O5 | 保持 0 秒のとき、その区間を `segments` に残すか | Red で決める(既定値 0.5 / 0.3 では起きない) |
| O6 | `interpolate_segment` の最後の行を `q_b` と完全一致させる | 本書で要件にした(lift = pre_grasp の完全一致のため) 🟡 |

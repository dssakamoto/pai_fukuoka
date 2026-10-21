# TASK-0010 テストケース定義: 安全フィルタ(可動域・速度・作業範囲)

**機能名**: safety(安全フィルタ)
**タスクID**: TASK-0010
**要件名**: so101-vlm-pickplace
**作成日**: 2026-10-20
**作成方法**: tsumiki:tdd-testcases(ユーザーへの質問なし。要件定義書の既定値で定義)
**入力文書**: `docs/implements/so101-vlm-pickplace/TASK-0010/safety-requirements.md`(以下「要件」)、`note.md`、`safety-survey.md`、`docs/tasks/so101-vlm-pickplace/TASK-0010.md`

**信頼性の記号**:
- 🔵 要件・設計・既存コード・実測にもとづく(推測なし)
- 🟡 要件・設計から妥当に推測した(境界の扱い、許容差など)
- 🔴 資料にない推測(要確認)

**対象テストファイル**:

| ファイル | 対象 |
|---|---|
| `tests/safety/test_limits.py` | `clip_to_limits`、`limit_step`、`is_in_workspace`、`check_state_fresh`、`limits_from_model`、`apply_safety`、`SafetyResult`、SimRobot との結合 |
| `tests/safety/test_config.py` | `WorkspaceBox`、`SafetyConfig`、`load_safety_config`、`configs/motion.yaml` |
| `tests/safety/conftest.py` | 共有 fixture(実シーンのモデル、既定の設定、タスク点の生成) |

テストの書き方は既存テストに合わせる(`tests/eval/test_eval_config.py`、`tests/robot/test_sim.py`): モジュール docstring に「テストケース定義: docs/implements/so101-vlm-pickplace/TASK-0010/safety-testcases.md」と信頼性記号の凡例を書く。各テストの docstring は `"""TC-xx-nn <日本語の要約> 🔵"""`。本文に `# Given` / `# When` / `# Then` のコメントを置く。

---

## 0. 共通の前提

### 0.1 定数(テストファイルの先頭に置く)

| 名前 | 値 | 意味 | 信頼性 |
|---|---|---|---|
| `VMAX` | `np.full(6, 1.0)` | 既定の最大角速度 [rad/s] | 🟡(要件 3.1。値そのものは D2 で 🔴) |
| `DT` | `0.02` | 制御周期 [s] | 🔵 |
| `M_STEP` | `VMAX * 1.0 * DT` = 0.02 | scale 1.0 の 1 周期の許容変化量 [rad] | 🔵(式) |
| `LOWER_SYN` | `[-1.0, -1.0, -1.0, -1.0, -2.0, 0.0]` | 単体テスト用の可動域の下限(関節ごとに違う値) | 🟡 |
| `UPPER_SYN` | `[1.0, 1.0, 1.0, 1.0, 2.0, 1.5]` | 同 上限 | 🟡 |
| `WS_LOWER` / `WS_UPPER` | `(0.12, -0.13, 0.00)` / `(0.38, 0.26, 0.17)` | 既定の作業範囲 | 🟡(要件 3.1、D1 で 🔴) |
| `CTRL_LOWER` | `(-1.91986, -1.74533, -1.69, -1.65806, -2.74385, -0.17453)` | 実モデルの ctrlrange 下限 | 🔵(実測) |
| `CTRL_UPPER` | `(1.91986, 1.74533, 1.69, 1.65806, 2.84121, 1.74533)` | 同 上限 | 🔵(実測) |
| `EXACT_ABS` | `1e-12` | 方向・比率の浮動小数の許容差 | 🟡 |

`SafetyConfig` は単体テストでは直接生成する(`DEFAULT_CFG = SafetyConfig(vmax=(1.0,)*6, control_dt=0.02, speed_scale=1.0, state_timeout=0.2, workspace=WorkspaceBox(lower=WS_LOWER, upper=WS_UPPER))`)。yaml には依存させない(test_config.py と独立に失敗原因を切り分けるため)🟡。

### 0.2 fixture の方針

| fixture | scope | 内容 | 理由 |
|---|---|---|---|
| `scene_model` | module | `mujoco.MjModel.from_xml_path(str(SO101_SCENE_PATH))` | 読み込みは 1 回だけ。モデルは読み取りのみ |
| `pickplace_model` | module | `PICKPLACE_SCENE_PATH` のモデル | 実際に使うシーンでも可動域が同じことの確認 |
| `task_points` | session(conftest) | 1.4 節の格子からタスク点 172 点を**テスト内で生成**した `(172, 3)` 配列 | scratchpad の `pts.npy` に依存しない |
| `sim_robot` | function | `SimRobot()` を作り `yield`、最後に `close()` | 結合テストは状態を進めるため毎回作り直す(生成は数 ms) |
| `tmp_path` | function(pytest 標準) | yaml を書く一時ディレクトリ | 設定ファイルの異常系 |

`tests/safety/__init__.py` は置かない(既存の `tests/robot/` などと同じ)。共有する関数は `conftest.py` の fixture にする(テストモジュール同士の import をしない)🟡。

### 0.3 実行時間の予算

- `tests/safety/` 全体で **3 秒未満**(目安)。単体テストは numpy のみで各 1 ms 未満。
- モデルの読み込みは module scope で 1 回(約 10 ms)。
- SimRobot の結合テストは 1 件 35 周期(350 回の `mj_step`)程度で、実測 0.05 s 前後。1 件あたり 1 秒未満。

### 0.4 タスク点の格子(`task_points`、172 点)🔵

safety-survey.md 1.1 節の点集合を、`SCENE_SPEC` から次のとおり生成する(throwaway 実行で survey の `pts.npy` と同じ集合になることを確認済み)。

```python
xs = np.round(np.linspace(*SCENE_SPEC.work_area_x, 9), 4)   # 0.15, 0.175, …, 0.35
ys = np.round(np.linspace(*SCENE_SPEC.work_area_y, 9), 4)   # -0.10, -0.075, …, 0.10
grasp_z = SCENE_SPEC.block_rest_z                            # 0.015
approach_z = grasp_z + 0.08                                  # 0.095
pts = [(x, y, z) for x in xs for y in ys for z in (grasp_z, approach_z)]   # 162 点
bx, by = SCENE_SPEC.box_center_xy; h = SCENE_SPEC.box_inner_half_size        # (0.20, 0.19), 0.04
pts += [(bx, by, 0.06), (bx, by, 0.14)]                                       # 解放と箱の上 2 点
pts += [(bx + sx*h, by + sy*h, z) for sx in (-1, 1) for sy in (-1, 1) for z in (0.06, 0.14)]  # 四隅 8 点
```

- 範囲: x 0.15〜0.35、y -0.10〜0.23、z 0.015〜0.14。
- `0.06`(縁 0.04 + 2 cm)と `0.14` は survey の値をそのまま定数として書く 🟡。

---

## 1. tests/safety/test_limits.py

### 1.1 `clip_to_limits`(TC-CL)

| ID | 分類 | Given | When | Then | 信頼性 |
|---|---|---|---|---|---|
| TC-CL-01 | 正常 | `q = [0.5, -0.5, 0.0, 0.9, -1.9, 1.0]`(全関節が LOWER_SYN〜UPPER_SYN の内側) | `clip_to_limits(q, LOWER_SYN, UPPER_SYN)` | 結果は `q` と完全一致(`np.array_equal`)。dtype float64 | 🔵 |
| TC-CL-02 | 正常(TASK-0010 ケース 1) | `q = [1.5, 0, 0, 0, 2.5, 2.0]`(関節 0, 4, 5 が上限超え) | 同上 | `[1.0, 0, 0, 0, 2.0, 1.5]`。上限の値ちょうど(完全一致) | 🔵 |
| TC-CL-03 | 正常 | `q = [-1.5, 0, 0, 0, -3.0, -0.1]`(下限未満) | 同上 | `[-1.0, 0, 0, 0, -2.0, 0.0]` | 🔵 |
| TC-CL-04 | 正常 | 関節ごとに内側・上限超え・下限未満を混在: `q = [2.0, -2.0, 0.3, 1.0, -2.0, 0.0]` | 同上 | `[1.0, -1.0, 0.3, 1.0, -2.0, 0.0]`。各関節が自分の範囲で独立に丸められる | 🔵 |
| TC-CL-05 | 境界 | `q = UPPER_SYN` ちょうど、`q = LOWER_SYN` ちょうど(parametrize) | 同上 | 入力と完全一致(境界は変えない) | 🟡 |
| TC-CL-06 | 境界 | 上限のすぐ外 `np.nextafter(UPPER_SYN, inf)` | 同上 | `UPPER_SYN` と完全一致 | 🟡 |
| TC-CL-07 | 正常(不変性) | `q`、`lower`、`upper` のコピーを保存 | TC-CL-02 の呼び出し | 3 つの入力は呼び出し後も保存したコピーと一致。戻り値は `q` と別オブジェクト(`not np.shares_memory`) | 🔵 |
| TC-CL-08 | 正常 | `q` を list で渡す、int の配列で渡す | 同上 | float64 の ndarray が返る | 🟡 |
| TC-CL-09 | 異常 | `q` に NaN / +inf / -inf(parametrize、関節 2 に入れる) | 同上 | `ValueError`。メッセージに引数名 `q` を含む | 🟡 |
| TC-CL-10 | 異常 | `q` の shape が (5,)、(7,)、(6, 1)、()(parametrize) | 同上 | `ValueError`(`q` を含む) | 🟡 |
| TC-CL-11 | 異常 | `lower` の shape (5,)、`upper` に NaN | 同上 | `ValueError`(`lower` / `upper` を含む) | 🟡 |
| TC-CL-12 | 異常 | `q = np.ones(6, dtype=bool)` | 同上 | `ValueError`(bool 配列は受け付けない) | 🟡 |
| TC-CL-13 | 異常 | `lower = upper` の関節がある(`lower[2] = upper[2] = 0.0`)、`lower[0] > upper[0]` | 同上 | `ValueError`(`lower_i < upper_i` でない) | 🟡 |

### 1.2 `limit_step`(TC-ST)

既定: `q_current = np.zeros(6)`、`vmax = VMAX`、`scale = 1.0`、`dt = 0.02`、`m = vmax * scale * dt`(テストでも同じ式で計算する)。

| ID | 分類 | Given | When | Then | 信頼性 |
|---|---|---|---|---|---|
| TC-ST-01 | 正常 | `q_target = [0.01, -0.005, 0.0, 0.015, 0.0, 0.02]`(全関節 `|Δ| ≤ 0.02`) | `limit_step(...)` | `q_target` と完全一致(`np.array_equal`) | 🔵 |
| TC-ST-02 | 正常(一様スケーリング) | `q_target = [1.0, -0.5, 0.25, 0.0, 2.0, -0.1]` | 同上 | 結果 `[0.01, -0.005, 0.0025, 0.0, 0.02, -0.001]`(abs 1e-12)。① 全関節 `|step_i| ≤ m_i`(許容差なし)② `max_i |step_i| / m_i == approx(1.0, abs=1e-12)`(少なくとも 1 関節=関節 4 が上限ちょうど)③ 方向余弦 `step·Δ / (‖step‖‖Δ‖) == approx(1.0, abs=1e-12)` ④ Δ ≠ 0 の関節で `step_i / Δ_i` がすべて同じ値 α = 0.01(abs 1e-12)⑤ Δ = 0 の関節 3 は動かない | 🔵(式)/ 🟡(一様) |
| TC-ST-03 | 正常(負の方向) | TC-ST-02 の符号を反転した `q_target` | 同上 | `[-0.01, 0.005, -0.0025, 0.0, -0.02, 0.001]`。方向余弦 1、上限に達する関節は関節 4 | 🟡 |
| TC-ST-04 | 正常(TASK-0010 ケース 2) | TC-ST-02 の `q_target`、`scale = 0.3` | 同上 | 最大の変化量 `max|step| = 0.006 = 1.0 × 0.3 × 0.02`(approx abs 1e-12)。全関節 `|step_i| ≤ vmax_i·0.3·dt`。`scale = 1.0` の結果と比べて各関節の変化量がちょうど 30%(`step_0.3 == approx(0.3 * step_1.0, abs=1e-12)`) | 🔵 |
| TC-ST-05 | 境界 | `q_target = [0.02, -0.02, 0, 0, 0, 0]`(`|Δ| = m` ちょうど。`q_current = 0` なので差は正確に 0.02) | 同上 | 制限されない: `q_target` と完全一致 | 🟡 |
| TC-ST-06 | 境界 | `q_target[0] = np.nextafter(0.02, 1)`(m をごくわずかに超える) | 同上 | `|step_0| ≤ 0.02`(許容差なし)。結果は `q_target` と一致しない | 🟡 |
| TC-ST-07 | 境界(0 移動) | `q_target = q_current = [0.3, -0.2, 0.1, 0.0, 0.5, 0.7]` | 同上 | 入力と完全一致。警告(0 除算)を出さない(`warnings` を error にして確認、`pytest.warns` は使わず `np.errstate(all="raise")` の中で呼ぶ) | 🟡 |
| TC-ST-08 | 正常(関節ごとの vmax) | `vmax = [1.0, 0.5, 2.0, 1.0, 1.0, 4.0]`(m = `[0.02, 0.01, 0.04, 0.02, 0.02, 0.08]`)、`q_target = np.full(6, 0.5)` | 同上 | 最も厳しい関節 1(m = 0.01)で α = 0.02 が決まり、結果は `np.full(6, 0.01)`(abs 1e-12)。全関節で上限以下、方向余弦 1 | 🟡 |
| TC-ST-09 | 正常(関節ごとの vmax、制限なし) | 同じ `vmax`、`q_target = [0.004, 0, 0.03, 0, 0, 0]`(関節 2 は 0.03 > 0.02 だが自分の m 0.04 以下) | 同上 | `q_target` と完全一致(関節ごとの上限で判定する) | 🟡 |
| TC-ST-10 | 境界(大きな目標) | `q_target = [100.0, -100.0, 0, 0, 0, 0]` | 同上 | `[0.02, -0.02, 0, 0, 0, 0]`(abs 1e-12)。有限 | 🟡 |
| TC-ST-11 | 正常(基準が 0 でない) | `q_current = [0.5, -0.3, 0.2, 0.1, -1.0, 0.8]`、`q_target = q_current + [1.0, 0, 0, 0, 0, -0.5]` | 同上 | 変化量 `step = result - q_current` が `[0.02, 0, 0, 0, 0, -0.01]`(abs 1e-12) | 🟡 |
| TC-ST-12 | 正常(不変性・決定性) | 入力のコピーを保存 | 同じ入力で 2 回呼ぶ | 入力は不変。2 回の結果は完全一致。戻り値は入力と別オブジェクト | 🔵 |
| TC-ST-13 | 異常(scale) | `scale` = 0.0、-0.1、1.01、NaN、inf、`True`(parametrize) | 同上 | `ValueError`(`scale` を含む) | 🟡 |
| TC-ST-14 | 境界(scale) | `scale = 1.0`(上限ちょうど)、`scale = 1e-6`(0 に近い正) | 同上 | 例外なし。1e-6 では変化量 `≤ 2e-8` | 🟡 |
| TC-ST-15 | 異常(dt) | `dt` = 0.0、-0.02、NaN、`True` | 同上 | `ValueError`(`dt` を含む) | 🟡 |
| TC-ST-16 | 異常(vmax) | `vmax` = スカラー 1.0、shape (5,)、要素 0.0、要素 -1.0、要素 NaN、bool 配列(parametrize) | 同上 | `ValueError`(`vmax` を含む) | 🟡 |
| TC-ST-17 | 異常(関節配列) | `q_current` / `q_target` に NaN・inf、shape (5,)、bool 配列 | 同上 | `ValueError`(引数名を含む) | 🟡 |

### 1.3 `is_in_workspace`(TC-WS)

`box = WorkspaceBox(lower=WS_LOWER, upper=WS_UPPER)`。境界の点は**設定値そのもの**(`box.lower[i]`、`box.upper[i]`)を代入して作り、足し算・引き算で作らない(浮動小数で境界がずれないため)。中心は `c = (0.25, 0.065, 0.085)`。

| ID | 分類 | Given | When | Then | 信頼性 |
|---|---|---|---|---|---|
| TC-WS-01 | 正常 | 中心 `c`、把持点 (0.25, 0.0, 0.015)、箱の上 (0.20, 0.19, 0.14) | `is_in_workspace(p, box)` | True。戻り値の型は `bool`(`np.bool_` でない) | 🔵 |
| TC-WS-02 | 境界(6 面) | `c` の軸 i だけを `box.lower[i]` または `box.upper[i]` に置き換えた 6 点(parametrize: 軸 × 面) | 同上 | すべて True(閉区間) | 🟡 |
| TC-WS-03 | 境界(角) | `p = box.lower`、`p = box.upper`(8 隅のうち 2 つ)、および 8 隅すべて(`itertools.product`) | 同上 | True | 🟡 |
| TC-WS-04 | 異常値(各軸で外) | TC-WS-02 の 6 点を外側へ 1 mm(下面は `lower[i] - 0.001`、上面は `upper[i] + 0.001`) | 同上 | すべて False | 🟡 |
| TC-WS-05 | 境界(すぐ外) | 6 面それぞれ `np.nextafter(lower[i], -inf)` / `np.nextafter(upper[i], inf)` | 同上 | False | 🟡 |
| TC-WS-06 | 正常(TASK-0010 ケース 3: 外側の代表点) | survey 2.5 節の外側の点(parametrize、ID 付き): 原点 (0, 0, 0)、土台の上 (0, 0, 0.10)、土台の前端 (0.10, 0, 0.05)、土台の後ろ (-0.10, 0, 0.05)、床の下 (0.25, 0, -0.001)、q = 0 の手先 (0.391, 0, 0.227)、高すぎる (0.25, 0, 0.18)、遠すぎる (0.40, 0, 0.05) | 同上 | すべて False | 🔵(実測) |
| TC-WS-07 | 正常(タスク点) | `task_points`(172 点) | 各点で判定 | 172 点すべて True。点数が 172 であることも確認 | 🔵(実測) |
| TC-WS-08 | 正常(survey の追加点) | gripperframe の把持目標 (0.25, 0, 0.0023)、経路の端 (0.3526, 0, 0.10)、(0.20, -0.1042, 0.05) | 同上 | True | 🔵(実測) |
| TC-WS-09 | 異常 | `p` に NaN / +inf / -inf(各軸、parametrize) | 同上 | `ValueError`(`p_base` を含む)。要件の変更点どおり False ではなく例外 | 🟡 |
| TC-WS-10 | 異常 | `p` の shape (2,)、(4,)、(3, 1)、bool 配列 | 同上 | `ValueError` | 🟡 |
| TC-WS-11 | 正常(入力の型) | `p` を tuple / list で渡す | 同上 | 判定できる(TC-WS-01 と同じ結果) | 🟡 |
| TC-WS-12 | 正常(不変性) | `p` の ndarray のコピーを保存 | 判定 | `p` は変わらない | 🔵 |

### 1.4 `check_state_fresh`(TC-FR)

| ID | 分類 | Given | When | Then | 信頼性 |
|---|---|---|---|---|---|
| TC-FR-01 | 正常 | `last = 10.0`、`now = 10.05`、`timeout = 0.2` | `check_state_fresh(last, now, timeout)` | True。型は `bool` | 🔵 |
| TC-FR-02 | 境界(ちょうど timeout) | 差が浮動小数で正確に表せる組: `(last, now, timeout) = (0.0, 0.2, 0.2)`、`(1.0, 1.25, 0.25)`、`(12345.0, 12345.125, 0.125)`(parametrize) | 同上 | True(ちょうどは新鮮) | 🟡 |
| TC-FR-03 | 境界(経過 0) | `now == last = 3.0` | 同上 | True | 🟡 |
| TC-FR-04 | 正常(TASK-0010 ケース 4) | `last = 0.0`、`now = 0.5`、`timeout = 0.2` | 同上 | False(停止) | 🔵 |
| TC-FR-05 | 境界(わずかに超える) | `last = 0.0`、`now = np.nextafter(0.2, 1.0)`、`timeout = 0.2` | 同上 | False | 🟡 |
| TC-FR-06 | 異常値(時計の逆行) | `last = 5.0`、`now = 4.999` | 同上 | False(停止側) | 🟡 |
| TC-FR-07 | 異常値(NaN / inf の時刻) | `last` または `now` が NaN、+inf、-inf(parametrize) | 同上 | False(例外にしない) | 🟡 |
| TC-FR-08 | 正常(単調時計の値) | `last = time.monotonic()` 相当の大きな値 `1.0e6`、`now = 1.0e6 + 0.125`、`timeout = 0.2` | 同上 | True。`now = 1.0e6 + 0.25` では False | 🟡 |
| TC-FR-09 | 異常(timeout) | `timeout` = 0.0、-0.1、NaN、inf、`True`(parametrize) | 同上 | `ValueError`(`timeout` を含む) | 🟡 |
| TC-FR-10 | 異常(bool の時刻) | `last = True` または `now = False` | 同上 | `ValueError`(実数スカラーに bool は不可。要件 2.2 共通規則) | 🟡 |

### 1.5 `limits_from_model`(TC-MD)

| ID | 分類 | Given | When | Then | 信頼性 |
|---|---|---|---|---|---|
| TC-MD-01 | 正常 | `scene_model`(`SO101_SCENE_PATH`) | `lower, upper = limits_from_model(scene_model)` | `lower == approx(CTRL_LOWER, abs=1e-5)`、`upper == approx(CTRL_UPPER, abs=1e-5)`。shape (6,)、float64 | 🔵(実測) |
| TC-MD-02 | 正常(名前で引く) | `scene_model` | 同上 | JOINT_NAMES の各名前で `mj_name2id(ACTUATOR)` した行の `actuator_ctrlrange` と完全一致。関節の `jnt_range` とも abs 1e-5 で一致(実測の差は最大 3.7e-6) | 🔵 |
| TC-MD-03 | 正常 | `pickplace_model`(`PICKPLACE_SCENE_PATH`) | 同上 | TC-MD-01 と同じ値(実シーンでも可動域は同じ) | 🔵(実測) |
| TC-MD-04 | 正常(コピー) | 戻り値の `lower[0] = 99.0` に書き換える(書き込み可の場合)、または書き込み不可 | もう一度 `limits_from_model` | 2 回目の値は元の値。`model.actuator_ctrlrange` も変わらない(`np.shares_memory(lower, model.actuator_ctrlrange)` が False) | 🔵 |
| TC-MD-05 | 正常(宣言順に依存しない) | `tests/robot/test_sim.py` と同じ方式の小さな MJCF(actuator の宣言順を JOINT_NAMES と変え、関節ごとに ctrlrange を `-(i+1)`〜`(i+1)` にする) | 同上 | JOINT_NAMES 順の `lower = -[1..6]`、`upper = [1..6]`(ただし宣言順ではなく名前順に対応) | 🟡 |
| TC-MD-06 | 異常 | 小さな MJCF から `wrist_roll` の actuator を除く | 同上 | `ValueError`(`wrist_roll` を含む) | 🟡 |

### 1.6 `apply_safety` と `SafetyResult`(TC-AS)

既定: `lower, upper = LOWER_SYN, UPPER_SYN`、`config = DEFAULT_CFG`(m = 0.02)、`last_state_t = 1.0`、`now = 1.0`(新鮮)、`target_base = None`。古い状態は `now = 1.5`(経過 0.5 > 0.2)。

#### 1.6.1 フラグと値

| ID | 分類 | Given | When | Then | 信頼性 |
|---|---|---|---|---|---|
| TC-AS-01 | 正常(素通し) | `q_current = 0`、`q_target = [0.01, 0, -0.01, 0, 0.02, 0.005]`(範囲内、`|Δ| ≤ m`) | `apply_safety(...)` | `q_command` が `q_target` と完全一致。4 フラグすべて False、`reason is None` | 🔵 |
| TC-AS-02 | 正常(速度制限のみ) | `q_current = 0`、`q_target = [0.5, 0, 0, 0, 0, 0]` | 同上 | `q_command == approx([0.02, 0, 0, 0, 0, 0], abs=1e-12)`、`rate_limited=True`、他 False、`reason is None` | 🔵 |
| TC-AS-03 | 正常(クリップ → 速度) | `q_current = [0.99, 0, 0, 0, 0, 0]`、`q_target = [1.5, 0, 0, 0, 0, 0]` | 同上 | 先にクリップ(1.0)してから速度制限: Δ = 0.01 ≤ m なので `q_command[0] == 1.0`(完全一致)、`clipped=True`、`rate_limited=False`。順序が逆(速度 → クリップ)だと Δ = 0.51 で速度制限が立ち 1.01 → 最終クリップで 1.0 となり `rate_limited=True` になるため、フラグで区別できる | 🟡 |
| TC-AS-04 | 正常(順序の区別) | `q_current = [0.95, 0, 0, 0, 0, 0]`、`q_target = [1.9, 0, 0, 0, 0, 0.5]` | 同上 | クリップ後の目標 `[1.0, 0, 0, 0, 0, 0.5]` に対して一様スケーリング: Δ = (0.05, …, 0.5)、α = 0.02/0.5 = 0.04 → `q_command == approx([0.952, 0, 0, 0, 0, 0.02], abs=1e-12)`。`clipped=True`、`rate_limited=True`。(クリップ前の Δ = (0.95, …, 0.5) で α を決めると α = 0.02/0.95 となり `q_command ≈ [0.97, 0, 0, 0, 0, 0.0105]` → 値で区別できる) | 🟡 |
| TC-AS-05 | 正常(最終クリップ) | `q_current = [1.05, 0, 0, 0, 0, 0]`(上限 1.0 の外)、`q_target = [1.0, 0, 0, 0, 0, 0]` | 同上 | `q_command[0] == 1.0`(範囲内)、`clipped=True`。最初のクリップでは目標が変わらない(`q_target` は範囲内)ので、最終クリップで立つことの確認 | 🟡 |
| TC-AS-06 | 正常(最終クリップ、離れた外側) | `q_current = [1.5, 0, 0, 0, 0, 0]`、`q_target = [1.5, 0, 0, 0, 0, 0]` | 同上 | `q_command[0] == 1.0`(変化量 0.5 は m を超えるが NFR-02 を優先)、`clipped=True`。`q_command` は全関節 `[lower, upper]` の内側 | 🟡 |
| TC-AS-07 | 正常(作業範囲の内側) | TC-AS-01 の入力 + `target_base = (0.25, 0.0, 0.015)` | 同上 | TC-AS-01 と同じ結果(`rejected=False`) | 🔵 |
| TC-AS-08 | 正常(TASK-0010 ケース 3: 拒否) | `q_current = [0.1, 0.2, -0.1, 0, 0, 0.5]`、`q_target` 任意の範囲内、`target_base = (0.40, 0, 0.05)` | 同上 | `rejected=True`、`reason == REASON_OUT_OF_WORKSPACE == "target_out_of_workspace"`、`q_command == q_current`(完全一致)、`clipped = rate_limited = stopped = False` | 🔵 |
| TC-AS-09 | 正常(拒否 > クリップ) | `q_target = [5.0, …]`(可動域外)かつ `target_base` が作業範囲外 | 同上 | `rejected=True`、`clipped=False`、`rate_limited=False`(要件 E13) | 🟡 |
| TC-AS-10 | 境界(作業範囲の面の上) | `target_base = WS_UPPER`(角) | 同上 | `rejected=False`(閉区間) | 🟡 |
| TC-AS-11 | 正常(TASK-0010 ケース 4: 停止) | `q_current = [0.1, 0.2, -0.1, 0, 0, 0.5]`、`q_target = [0.5, …]`、`now = 1.5` | 同上 | `stopped=True`、`reason == REASON_STALE_STATE == "stale_state"`、`q_command == q_current`、他フラグ False | 🔵 |
| TC-AS-12 | 正常(停止 > 拒否) | 古い状態 かつ `target_base` が作業範囲外 かつ `q_target` 可動域外 | 同上 | `stopped=True`、`rejected=False`、`clipped=False`、`rate_limited=False`、`reason == "stale_state"` | 🟡 |
| TC-AS-13 | 境界(ちょうど timeout) | `last_state_t = 0.0`、`now = 0.2`、`state_timeout = 0.2` | 同上 | `stopped=False`(新鮮) | 🟡 |
| TC-AS-14 | 正常(時刻の異常 → 停止) | `now` が NaN、`now < last_state_t` | 同上 | `stopped=True`(例外にしない) | 🟡 |
| TC-AS-15 | 正常(reason 定数) | — | `limits` モジュールを import | `REASON_STALE_STATE == "stale_state"`、`REASON_OUT_OF_WORKSPACE == "target_out_of_workspace"`。拒否・停止以外の結果の `reason` は常に None | 🔵 |
| TC-AS-16 | 正常(scale 0.3) | `dataclasses.replace(DEFAULT_CFG, speed_scale=0.3)`、TC-AS-02 の入力 | 同上 | `q_command[0] == approx(0.006, abs=1e-12)`、`rate_limited=True` | 🔵 |

#### 1.6.2 検証の順序と例外

| ID | 分類 | Given | When | Then | 信頼性 |
|---|---|---|---|---|---|
| TC-AS-20 | 正常(停止は値の検証より先) | 古い状態、`q_target` に NaN(または inf) | 同上 | 例外にならず `stopped=True`、`q_command == q_current`(要件 E10) | 🟡 |
| TC-AS-21 | 正常(停止は target_base の値より先) | 古い状態、`target_base = (nan, 0, 0)` | 同上 | `stopped=True` | 🟡 |
| TC-AS-22 | 異常(値) | 新鮮な状態、`q_target` に NaN / inf | 同上 | `ValueError`(`q_target` を含む)(要件 E11) | 🟡 |
| TC-AS-23 | 異常(値) | 新鮮な状態、`target_base` に NaN | 同上 | `ValueError`(`target_base` を含む) | 🟡 |
| TC-AS-24 | 異常(構造は停止より先) | **古い状態**でも、`q_target` の shape (5,)、`target_base` の shape (2,)、`q_current` に NaN、`q_current` の shape (7,)、`lower` の shape (5,)、`upper` に inf、`q_target` が bool 配列(parametrize) | 同上 | すべて `ValueError`(構造の検証は手順 1。停止で隠さない) | 🟡 |
| TC-AS-25 | 異常(config の型) | `config = {"vmax": ...}`(dict)、`config = None` | 同上 | `ValueError` | 🟡 |
| TC-AS-26 | 異常(時刻の型) | `last_state_t = True` | 同上 | `ValueError` | 🟡 |
| TC-AS-27 | 異常(引数の渡し方) | `lower` などを位置引数で渡す | 同上 | `TypeError`(キーワード専用) | 🟡 |

#### 1.6.3 結果型・不変性・決定性

| ID | 分類 | Given | When | Then | 信頼性 |
|---|---|---|---|---|---|
| TC-AS-30 | 正常(frozen) | TC-AS-02 の結果 `res` | `res.clipped = True`、`res.q_command = ...` | `dataclasses.FrozenInstanceError` | 🔵 |
| TC-AS-31 | 正常(読み取り専用) | 同上 | `res.q_command[0] = 9.0` | `ValueError`(numpy の read-only)。`res.q_command.flags.writeable is False`、dtype float64、shape (6,) | 🔵 |
| TC-AS-32 | 正常(保持はコピー) | TC-AS-11(停止)の結果 | `q_current` を呼び出し後に書き換える | `res.q_command` は変わらない(`not np.shares_memory`) | 🔵 |
| TC-AS-33 | 正常(入力不変) | 全引数(配列 4 個と `target_base`)のコピーを保存 | TC-AS-04、TC-AS-08、TC-AS-11 の各入力で呼ぶ | すべての入力配列が保存したコピーと一致 | 🔵 |
| TC-AS-34 | 正常(決定性) | 同じ入力 | 2 回呼ぶ | 2 つの結果が完全一致(`q_command` は `array_equal`、フラグと reason が同じ)。乱数・時計を使わない | 🟡 |
| TC-AS-35 | 正常(SafetyResult の直接生成) | `SafetyResult(q_command=[0]*6 の list, clipped=False, …)` | 生成 | `q_command` は float64 の読み取り専用 ndarray。元の list / ndarray と共有しない | 🟡 |

#### 1.6.4 制限内の軌道は変わらない(TASK-0012 の前提)

| ID | 分類 | Given | When | Then | 信頼性 |
|---|---|---|---|---|---|
| TC-AS-40 | 正常 | 関節空間の直線補間の軌道: `qa = [0.1, -0.3, 0.5, 0.2, -0.1, 1.0]`、`qb = [-0.767, -0.031, -0.133, 0.411, 0.2, 0.0]`、`N = ceil(max|qb − qa| / (0.99 · m)) = 51`、`q_k = qa + (qb − qa) · k / N`(k = 0…51)。可動域は**実モデル**の `limits_from_model(scene_model)`、`config = DEFAULT_CFG`、`target_base = None`、時刻は新鮮 | `q_ref = q_0` から始め、各 k で `res = apply_safety(q_ref, q_{k+1}, ...)`、`q_ref = res.q_command`(前回の指令を基準にする) | すべての k で 4 フラグが False、`reason is None`、`res.q_command` が `q_{k+1}` と**完全一致**(`np.array_equal`)。前提として 1 周期の変化量の最大は 0.019608(≤ 0.02)、全点が可動域の内側 | 🔵(要件 6.1)/ 🟡(数値) |
| TC-AS-41 | 正常(scale 0.3) | 同じ方法で `scale = 0.3`(m = 0.006、N = ceil(1.0 / 0.00594) = 169。1 周期の最大 0.005917、実測確認済み) | 同上 | TC-AS-40 と同じ(フラグすべて False、完全一致) | 🟡 |
| TC-AS-42 | 正常(作業範囲つき) | `task_points` から 3 点を経由点として `target_base` に与え、上の軌道の各周期で同じ点を渡す | 同上 | `rejected=False` のまま TC-AS-40 と同じ | 🟡 |

**実装上の注意(Green 向け)**: `α = 1` のときは `q_current + (q_target − q_current)` ではなく `q_target` のコピーを返すこと。前者は浮動小数の丸めで `q_target` と 1 ulp ずれることがあり、TC-AS-40 の完全一致が保証できない(上の 2 つの軌道では throwaway 確認で偶然ずれなかった)。🟡

### 1.7 SimRobot との結合(TC-IN)

`sim_robot`(既定シーン `SO101_SCENE_PATH`、初期 q = 0、`timestamp` = 0.0)、可動域は `limits_from_model(scene_model)`。時刻は sim の時計: 新鮮な周期では `last_state_t = now = state.timestamp`。各周期で `state = robot.get_state()` → `apply_safety(q_ref, q_target, ...)` → `robot.send_command(res.q_command)` → `q_ref = res.q_command`。

目標は可動域の外を含む大きな値 `Q_FAR = [3.0, 0.0, 2.5, 0.0, -3.5, 2.5]`(クリップ後 `[1.91986, 0, 1.69, 0, -2.74385, 1.74533]`)。shoulder_lift と wrist_flex は動かさない(重力の影響を避ける)。

| ID | 分類 | Given | When | Then | 信頼性 |
|---|---|---|---|---|---|
| TC-IN-01 | 正常(scale 1.0) | `config = DEFAULT_CFG`、`Q_FAR` | 25 周期(sim 時間 0.5 s)回す | ① 毎周期 `|q_command − q_ref_prev| ≤ m`(許容差なし)、全周期の `q_command` が可動域の内側 ② 1 周期目は `clipped=True` かつ `rate_limited=True` ③ 計測値の包絡: すべての周期 k で `|q_k − q_0| ≤ vmax · scale · (t_k − t_0) + 0.02`(関節ごと。実測の超過は最大 0.0095 rad)④ 平均速度 `|q_25 − q_0| / 0.5 ≤ vmax · scale · 1.05`(実測最大 1.009 rad/s、wrist_roll)⑤ 目標へ近づく: 動かした 4 関節(0, 2, 4, 5)で `|q_target_clip − q_25| < |q_target_clip − q_0|`、かつ最も遠い関節 wrist_roll の移動量 `≥ 0.9 × vmax · scale · 0.5 = 0.45`(実測 0.5046)⑥ 計測した q が全周期で `[lower − 0.01, upper + 0.01]` の内側 | 🟡(許容差は実測にもとづく) |
| TC-IN-02 | 正常(scale 0.3、NFR-07(2)) | `dataclasses.replace(DEFAULT_CFG, speed_scale=0.3)`、`Q_FAR` | 25 周期 | TC-IN-01 と同じ項目を scale 0.3 で確認。包絡の許容 0.02(実測超過 0.0017)、平均速度 ≤ 0.3 × 1.05 = 0.315(実測 0.303)、wrist_roll の移動量 ≥ 0.9 × 0.15 = 0.135(実測 0.1514)。さらに 25 周期後の移動量が TC-IN-01 の約 30%(`0.25 ≤ 比 ≤ 0.35`) | 🟡 |
| TC-IN-03 | 正常(停止で保持) | TC-IN-01 の 25 周期の後 | 次の 10 周期は `now = state.timestamp + 0.5`(古い状態)で `apply_safety` → `send_command` | ① 10 周期すべて `stopped=True`、`q_command` は直前の指令 `q_ref` と完全一致(毎周期同じ値)② 保持中の総移動量 `max|q − q_(停止直前)| ≤ 0.012`(追従遅れの解消分。実測 0.0062)③ 最後の 5 周期の移動量 `max|q_10 − q_5| ≤ 1e-3`(実測 9e-5)④ 10 周期後 `max|q − q_hold| ≤ 2e-3`(実測 5.4e-4) | 🟡 |
| TC-IN-04 | 正常(作業範囲外は動かない) | 初期状態から、`target_base = (0.40, 0, 0.05)` を毎周期与え `Q_FAR` を目標にする | 10 周期 | 毎周期 `rejected=True`、`q_command == q_0`(初期の q)。計測した q の変化 `≤ 2e-3`(重力による沈み込みの範囲。実測 5.4e-4) | 🟡 |

各テストは 35 周期以下(`mj_step` 350 回以下)。実測で 1 件 0.05 s 前後。

---

## 2. tests/safety/test_config.py

`VALID = {"safety": {"vmax": [1.0]*6, "control_dt": 0.02, "speed_scale": 1.0, "state_timeout": 0.2, "workspace": {"lower": [0.12, -0.13, 0.00], "upper": [0.38, 0.26, 0.17]}}}`。`_write(tmp_path, obj)`、`_with(path_keys, value)`、`_without(path_keys)` は `tests/eval/test_eval_config.py` と同じ形のヘルパー(`VALID` は deepcopy で不変に保つ)。

### 2.1 `SafetyConfig` / `WorkspaceBox` の直接生成(TC-SC)

| ID | 分類 | Given | When | Then | 信頼性 |
|---|---|---|---|---|---|
| TC-SC-01 | 正常 | 既定値 | `SafetyConfig(vmax=(1.0,)*6, control_dt=0.02, speed_scale=1.0, state_timeout=0.2, workspace=WorkspaceBox((0.12,-0.13,0.0),(0.38,0.26,0.17)))` | 生成できる。`vmax` は tuple、各値 float | 🔵 |
| TC-SC-02 | 正常(frozen) | TC-SC-01 | `cfg.speed_scale = 0.3`、`cfg.workspace.lower = ...` | `dataclasses.FrozenInstanceError` | 🔵 |
| TC-SC-03 | 正常(0.3 倍の作り方) | TC-SC-01 | `dataclasses.replace(cfg, speed_scale=0.3)` | 新しい設定の `speed_scale == 0.3`、元の `cfg.speed_scale == 1.0`(TASK-0010 完了条件「倍率を設定で変えられる」) | 🔵 |
| TC-SC-04 | 正常(list を渡す) | `vmax=[1.0]*6`、`lower=[...]` を list で | 生成 | tuple に変換して保持(外の list を後で変えても影響しない) | 🟡 |
| TC-SC-05 | 境界 | `speed_scale = 1.0`、`speed_scale = 1e-6`、`vmax` の要素 1e-9 | 生成 | 例外なし | 🟡 |
| TC-SC-06 | 異常(vmax) | 5 個、7 個、要素 0.0、要素 -1.0、要素 NaN / inf、要素 `True`、文字列 "1.0"、スカラー 1.0(parametrize) | 生成 | `ValueError`(`vmax` を含む) | 🟡 |
| TC-SC-07 | 異常(speed_scale) | 0.0、-0.3、1.01、NaN、inf、`True`、"0.3" | 生成 | `ValueError`(`speed_scale` を含む) | 🟡 |
| TC-SC-08 | 異常(control_dt) | 0.0、-0.02、NaN、inf、`True` | 生成 | `ValueError`(`control_dt` を含む) | 🟡 |
| TC-SC-09 | 異常(state_timeout) | 0.0、-0.2、NaN、inf、`False` | 生成 | `ValueError`(`state_timeout` を含む) | 🟡 |
| TC-SC-10 | 異常(workspace) | 各軸で `lower == upper`(parametrize: 軸 0, 1, 2)、`lower > upper`、要素 NaN、長さ 2 / 4、要素 bool | `WorkspaceBox(...)` の生成 | `ValueError`(`lower` / `upper` / `workspace` のいずれかを含む) | 🟡 |
| TC-SC-11 | 異常(型) | `workspace = {"lower": ..., "upper": ...}`(dict) | `SafetyConfig(...)` | `ValueError`(`workspace` を含む) | 🟡 |

### 2.2 `load_safety_config`(TC-LC)

| ID | 分類 | Given | When | Then | 信頼性 |
|---|---|---|---|---|---|
| TC-LC-01 | 正常(リポジトリの既定ファイル) | `configs/motion.yaml`(Green で作成) | `load_safety_config()` | `vmax == (1.0,)*6`、`control_dt == approx(0.02)`、`speed_scale == 1.0`、`state_timeout == approx(0.2)`、`workspace.lower == approx((0.12, -0.13, 0.00))`、`workspace.upper == approx((0.38, 0.26, 0.17))`。型は `SafetyConfig` / `WorkspaceBox` | 🟡(既定値は要件 3.1。D1〜D3 確定で 🔵) |
| TC-LC-02 | 正常(既定パス) | — | — | `DEFAULT_MOTION_CONFIG_PATH == REPO_ROOT / "configs" / "motion.yaml"`、`is_file()`、`load_safety_config() == load_safety_config(DEFAULT_MOTION_CONFIG_PATH)` | 🔵 |
| TC-LC-03 | 正常(str と Path) | `VALID` を tmp に書く | `load_safety_config(path)`、`load_safety_config(str(path))` | 2 つの結果が等しい | 🟡 |
| TC-LC-04 | 正常(他の最上位キーを無視) | `VALID` に `approach_height: 0.08`、`pre_grasp: {dz: 0.08}` を追加 | 読む | 例外なし。`VALID` だけのときと等しい | 🟡 |
| TC-LC-05 | 正常(scale の上書き) | `speed_scale: 0.3` の yaml | 読む | `speed_scale == 0.3` | 🔵 |
| TC-LC-06 | 正常(整数値) | `vmax: [1, 1, 1, 1, 1, 1]`、`state_timeout: 1` | 読む | float として保持 | 🟡 |
| TC-LC-07 | 異常(safety: の欠落キー) | `vmax`、`control_dt`、`speed_scale`、`state_timeout`、`workspace` をそれぞれ削除(parametrize) | 読む | `ValueError`。メッセージに欠けたキー名を含む | 🟡 |
| TC-LC-08 | 異常(safety: の未知キー) | `safety:` に `vmax_gripper: 2.0` を追加 | 読む | `ValueError`(`vmax_gripper` を含む) | 🟡 |
| TC-LC-09 | 異常(workspace: の欠落・未知) | `lower` / `upper` を削除、`center: [...]` を追加(note.md の旧形式) | 読む | `ValueError`(キー名を含む) | 🟡 |
| TC-LC-10 | 異常(safety: が無い) | `{"other": 1}` | 読む | `ValueError`(`safety` を含む) | 🟡 |
| TC-LC-11 | 異常(mapping でない) | 最上位が list `[1, 2]`、スカラー `"x"`、空ファイル(`None`)。`safety:` が list、`workspace:` がスカラー(parametrize) | 読む | `ValueError` | 🟡 |
| TC-LC-12 | 異常(値) | yaml 経由で `speed_scale: 0`、`vmax: [1.0]*5`、`workspace.lower[2] = 0.2`(upper 0.17 以上)、`control_dt: true` | 読む | `ValueError`(2.1 節の検証が yaml 経由でも働く) | 🟡 |
| TC-LC-13 | 異常(ファイルなし) | `tmp_path / "missing.yaml"` | 読む | `FileNotFoundError`(メッセージにパスを含む) | 🔵 |
| TC-LC-14 | 異常(safe_load) | `!!python/object/apply:os.system ["echo x"]` を含む yaml | 読む | `yaml.YAMLError`(`safe_load` は任意オブジェクトを作らない。コマンドは実行されない)。加えて `config.py` のソースに `yaml.safe_load` があり `yaml.load(` が無いこと(`inspect.getsource`)を確認 | 🔵 |

### 2.3 既定の作業範囲とシーン(TC-WC)

| ID | 分類 | Given | When | Then | 信頼性 |
|---|---|---|---|---|---|
| TC-WC-01 | 正常 | `load_safety_config()` の `workspace`、`task_points`(SCENE_SPEC から生成した 172 点) | 各点で `is_in_workspace` | 172 点すべて True | 🔵(実測) |
| TC-WC-02 | 正常(余裕) | 同上 | 各面までの最小距離 | x・y は 0.03 以上(abs 1e-9)、z の下側は 0.015 以上(床で止めたため) | 🔵(実測) |
| TC-WC-03 | 正常(scene_spec の作業範囲を含む) | `SCENE_SPEC.work_area_x/y` の 4 隅 × z ∈ {0.0, 0.17} | 判定 | すべて True | 🟡 |
| TC-WC-04 | 正常(箱を含む) | 箱の内寸の四隅 `(0.20 ± 0.04, 0.19 ± 0.04)` × z ∈ {`box_rim_z` 0.04, 0.14} | 判定 | すべて True | 🟡 |

---

## 3. 分類別の件数

| ファイル | 群 | 正常系 | 異常系 | 境界値 | 計 |
|---|---|---|---|---|---|
| test_limits.py | clip_to_limits(TC-CL) | 6 | 5 | 2 | 13 |
| | limit_step(TC-ST) | 8 | 4 | 5 | 17 |
| | is_in_workspace(TC-WS) | 6 | 3 | 3 | 12 |
| | check_state_fresh(TC-FR) | 3 | 4 | 3 | 10 |
| | limits_from_model(TC-MD) | 5 | 1 | 0 | 6 |
| | apply_safety(TC-AS) | 25 | 6 | 2 | 33 |
| | SimRobot 結合(TC-IN) | 4 | 0 | 0 | 4 |
| test_config.py | SafetyConfig(TC-SC) | 4 | 6 | 1 | 11 |
| | load_safety_config(TC-LC) | 6 | 8 | 0 | 14 |
| | 作業範囲とシーン(TC-WC) | 4 | 0 | 0 | 4 |
| **合計** | | **71** | **37** | **16** | **124** |

(parametrize で展開すると pytest の件数はおよそ 250〜300 件になる。)

### 信頼性の分布

| 記号 | 件数 | 割合 |
|---|---|---|
| 🔵 | 39 | 31% |
| 🟡 | 85 | 69% |
| 🔴 | 0 | 0% |

🟡 が多いのは、境界の扱い(閉区間、ちょうど timeout、`|Δ| = m`)と例外の範囲を要件定義書(tdd-requirements で既定として決めたもの)から取っているため。値そのものが資料に無い 🔴 の項目は、テストケースではなく既定値(作業範囲 D1、vmax D2、state_timeout D3)の側にあり、要件定義書 8 節で管理する。

---

## 4. 開発言語・フレームワーク

- **プログラミング言語**: Python 3.12(`pyproject.toml` の `requires-python`)🔵
  - numpy の配列比較(`np.array_equal`、`np.nextafter`、`np.shares_memory`)で、境界・完全一致・不変性を直接確かめられる。
- **テストフレームワーク**: pytest(`uv run pytest tests/safety -q`)、カバレッジは `uv run pytest --cov`。🔵
  - `pytest.mark.parametrize` で軸・面・不正値を展開、`pytest.approx` で浮動小数の比較、`tmp_path` で yaml、module scope の fixture でモデルを 1 回だけ読む。既存の `tests/eval/test_eval_config.py`、`tests/robot/test_sim.py` と同じ。
  - 依存: numpy、mujoco、pyyaml(すべて既存の依存)。追加のパッケージは不要。

---

## 5. テスト実装時の日本語コメント指針

既存テストの形に合わせ、Python のコメントで書く。

```python
def test_limit_step_scales_uniformly_and_keeps_direction():
    """TC-ST-02 大きな目標は一様に縮め、方向を保つ 🔵/🟡"""
    # 【テスト目的】: 速度制限が関節空間の方向を変えず、最も厳しい関節がちょうど上限になることを確認
    # Given
    # 【テストデータ準備】: 関節 4 の差が最大 (2.0) になる目標。m = 1.0 * 1.0 * 0.02 = 0.02
    q_current = np.zeros(6)
    q_target = np.array([1.0, -0.5, 0.25, 0.0, 2.0, -0.1])
    m = VMAX * 1.0 * DT
    # When
    # 【実際の処理実行】: 1 周期分だけ進める
    out = limit_step(q_current, q_target, VMAX, 1.0, DT)
    # Then
    step = out - q_current
    delta = q_target - q_current
    assert np.all(np.abs(step) <= m)  # 【確認内容】: 全関節が上限以下(許容差なし)🔵
    assert np.max(np.abs(step) / m) == pytest.approx(1.0, abs=EXACT_ABS)  # 【確認内容】: 1 関節は上限ちょうど 🟡
    cos = step @ delta / (np.linalg.norm(step) * np.linalg.norm(delta))
    assert cos == pytest.approx(1.0, abs=EXACT_ABS)  # 【確認内容】: 方向が変わらない 🟡
```

- fixture には `# 【テスト前準備】` / `# 【環境初期化】` を書く(`tests/robot/test_sim.py` と同じ)。
- 数値の根拠が実測(safety-survey.md、throwaway 確認)のものは、コメントに「実測 0.0062 rad」などと値を残す。

---

## 6. 要件定義との対応関係

| 要件(safety-requirements.md) | テストケース |
|---|---|
| 1 節 機能概要(4 機能、純粋関数) | 全体。TASK-0010 の 4 ケース = TC-CL-02、TC-ST-04、TC-WS-06 / TC-AS-08、TC-FR-04 / TC-AS-11 |
| 2.2 共通の入力規則(shape、NaN/inf、bool、不変、決定性) | TC-CL-07〜12、TC-ST-12〜17、TC-WS-09〜12、TC-FR-10、TC-AS-24〜26、TC-AS-33〜34 |
| 2.2 `clip_to_limits` | TC-CL-01〜13 |
| 2.2 `limit_step`(一様スケーリング、`|Δ| = m` は制限しない) | TC-ST-01〜17 |
| 2.2 `is_in_workspace`(閉区間、NaN は ValueError) | TC-WS-01〜12 |
| 2.2 `check_state_fresh`(ちょうど timeout は新鮮、逆行・NaN は停止) | TC-FR-01〜10 |
| 2.2 `limits_from_model` | TC-MD-01〜06 |
| 2.2 `apply_safety` の手順 1〜7 | 手順 1: TC-AS-24〜27、2: TC-AS-11〜14 / 20〜21、3: TC-AS-22〜23、4: TC-AS-08〜10、5〜6: TC-AS-02〜04 / 16、7: TC-AS-05〜06 |
| 2.3 `SafetyResult`(frozen、読み取り専用、優先順位、reason 定数) | TC-AS-09、12、15、30〜32、35 |
| 2.4 設定(検証、`safety:` だけを見る、未知・欠落キー、safe_load) | TC-SC-01〜11、TC-LC-01〜14 |
| 3 節 NFR-02 / NFR-07(2)(3)(4) | NFR-02: TC-AS-05〜06、TC-IN-01 ⑥。NFR-07(2): TC-ST-04、TC-AS-16、TC-IN-02、TC-SC-03、TC-LC-05。(3): TC-AS-08、TC-IN-04。(4): TC-AS-11、TC-IN-03 |
| 3.1 既定値と作業範囲の検証結果 | TC-LC-01、TC-WS-06〜08、TC-WC-01〜04 |
| 4.2 エッジケース E1〜E20 | E1: TC-CL-05 / E2: TC-CL-02〜03 / E3: TC-ST-04 / E4: TC-ST-05 / E5: TC-ST-08 / E6: TC-ST-07 / E7: TC-WS-02、04 / E8: TC-FR-02、05 / E9: TC-FR-06〜07 / E10: TC-AS-20 / E11: TC-AS-22 / E12: TC-CL-10、12、TC-FR-10 / E13: TC-AS-09 / E14: TC-AS-05〜06 / E15: TC-AS-31、33 / E16: TC-AS-34 / E17: TC-SC-06〜10、TC-LC-07〜12 / E18: TC-LC-04 / E19: TC-MD-01〜02 / E20: TC-WS-07、TC-WC-01 |
| 4.1 使用例 / 6.1 TASK-0012 の「制限を守る」の定義 | TC-AS-40〜42、TC-IN-01〜03 |

---

## 7. throwaway 実行で確認した数値

使い捨てスクリプト(リポジトリ外の scratchpad、`safety_survey/testcase_check.py`、`sim_envelope.py`、`sim_overshoot.py`)をプロジェクトルートで `uv run python` で実行した。参照実装(一様スケーリング + 最終クリップ)で次を確認した。

| 項目 | 結果 |
|---|---|
| SCENE_SPEC からの格子の再生成 | 172 点。survey の `pts.npy` と同じ集合。既定の作業範囲に 172 / 172 が内側 |
| 外側の代表点 8 点 | すべて外側 |
| 6 面の上(設定値を代入)/ 1 mm 外 | 内側 / 外側 |
| ctrlrange(scene.xml、scene_pickplace.xml) | 0.1 の CTRL_LOWER / CTRL_UPPER と一致。actuator id は JOINT_NAMES 順に 0〜5。`jnt_range` との差は最大 3.7e-6 |
| TC-ST-02 | 結果 (0.01, -0.005, 0.0025, 0, 0.02, -0.001)、方向余弦 1(誤差 2e-16)、関節 4 が 0.02 ちょうど |
| TC-ST-04 | 最大変化量 0.006、scale 1.0 の 30% |
| TC-ST-05 | `|Δ| = 0.02` ちょうどは制限されず、完全一致 |
| TC-ST-08 / 09 | 結果 0.01 × 6 / 制限なしで完全一致 |
| `1.0 * 0.3 * 0.02 == 0.006` | True |
| `0.2 − 0.0`、`1.25 − 1.0` | 0.2、0.25(ちょうど) |
| TC-AS-40 の軌道 | N = 51、1 周期の最大 0.019608、全周期で制限なし・完全一致 |
| TC-IN-01(scale 1.0、25 周期) | 包絡の超過 最大 0.0095 rad、1 周期の計測速度は最大 1.405 rad/s(wrist_roll。追従の振動。平均は 1.009)、移動量 wrist_roll 0.5046、可動域の内側。実行 0.05 s |
| TC-IN-02(scale 0.3) | 包絡の超過 0.0017、平均速度 0.303、移動量 0.1514 |
| TC-IN-03(停止 10 周期) | 総移動 0.0062、最後 5 周期 9e-5、保持指令との差 5.4e-4 |

**計測した速度を 1 周期ごとに vmax 以下と確認するテストは書かない**: 指令の変化量は毎周期 m 以下でも、sim の追従は 1 周期単位で最大 1.4 倍の速度になる(実測)。そのため計測値は「累積の包絡」と「平均速度」で確認する(TC-IN-01 ③④)🟡。

---

## 8. 未決事項・注意

| # | 事項 | 本書での扱い |
|---|---|---|
| T1 | 既定値(作業範囲 D1、vmax D2、state_timeout D3)はユーザー未確認 | TC-LC-01 と TC-WC は既定値に依存する。値が変わればこの 2 群の期待値だけを直す |
| T2 | `α = 1` で `q_target` のコピーを返すこと | TC-AS-40 / TC-ST-01 の完全一致の前提。Green の実装で守る(1.6.4 の注意) |
| T3 | 速度制限の判定が厳密な不等号(`|Δ_i| > m_i` で制限) | TC-ST-05、TC-ST-06 で固定 |
| T4 | SimRobot の結合テストの許容差(包絡 0.02 rad、平均速度 5%、停止 0.012 rad) | 実測値の約 2 倍の余裕。MuJoCo の版が変わって揺れる場合はここを見直す |
| T5 | `check_state_fresh` の bool 時刻を `ValueError` にする | 要件 2.2 の共通規則(実数スカラーに bool は不可)から推測 🟡 |
| T6 | `apply_safety` の `config` 型違反を `ValueError` にする(`TypeError` でない) | 要件 2.2 手順 1 の「config の型 … 違反は ValueError」による |

---

## 9. 品質判定

| 観点 | 評価 |
|---|---|
| テストケース分類 | 正常系 71・異常系 37・境界値 16(計 124)。6 関数・結果型・設定・ローダー・sim 結合を網羅 |
| 期待値定義 | すべて具体的な数値・完全一致・例外の型とメッセージのキーで定義 |
| 技術選択 | Python 3.12 + pytest + numpy + mujoco + pyyaml(既存の依存のみ) |
| 実装可能性 | 参照実装で主要な数値を確認済み。実行時間は全体で 3 秒未満の見込み |
| 信頼性 | 🔵 31% / 🟡 69% / 🔴 0%。🟡 は要件定義書で決めた境界・例外の扱い |

**判定**: ✅ 高品質

次のお勧めステップ: `/tsumiki:tdd-red so101-vlm-pickplace TASK-0010` でRedフェーズ(失敗テスト作成)を開始します。

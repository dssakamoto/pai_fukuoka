# TASK-0010 要件定義書: 安全フィルタ(可動域・速度・作業範囲)

**機能名**: safety(安全フィルタ)
**タスクID**: TASK-0010
**要件名**: so101-vlm-pickplace
**作成日**: 2026-10-20
**作成方法**: tsumiki:tdd-requirements(ユーザーへの質問なしでデフォルトを選び、仮定を記録した)
**関連文書**: `docs/implements/so101-vlm-pickplace/TASK-0010/note.md`、`docs/implements/so101-vlm-pickplace/TASK-0010/safety-survey.md`(数値の根拠)

**信頼性の記号**:
- 🔵 要件定義書・設計文書・既存コード・本タスクの実測にもとづく(推測なし)
- 🟡 要件定義書・設計文書から妥当に推測した
- 🔴 資料にない推測(要確認。8 節の未決事項)

**note.md からの変更点**(本書が優先する):

| 項目 | note.md | 本書 | 理由 |
|---|---|---|---|
| 作業範囲の表現 | `center` + `half_size` | `lower` + `upper` | 境界値が浮動小数の足し引きでずれない。設定値と境界テストの数値が一致する |
| 作業範囲の値 | x 0.15〜0.35 / y -0.10〜0.23 / z 0.00〜0.20 | x 0.12〜0.38 / y -0.13〜0.26 / z 0.00〜0.17 | タスク点 + 3 cm の余裕(safety-survey.md 2 節) |
| `vmax` | 0.5 rad/s | 1.0 rad/s | 0.5 では 1 回 10〜14 s(safety-survey.md 3.3 節) |
| 速度制限の方式 | 関節ごとのクリップ | 一様スケーリング(方向を保つ) | 4.3 節 |
| 速度制限の基準 | `q_current`(意味は未定) | 前回送った指令を推奨 | 追従遅れ(safety-survey.md 3.2 節) |
| `apply_safety` の順序 | 検証 → 停止 → 拒否 → クリップ → 速度 | 構造の検証 → 停止 → 値(NaN)の検証 → 拒否 → クリップ → 速度 → 最終クリップ | 指示どおり停止を最優先。最終クリップで NFR-02 を保証 |
| NaN の作業範囲判定 | False | `ValueError` | 配列の NaN は一律にバグとして扱う(4.4 節) |

---

## 1. 機能の概要

- 🔵 **何をする機能か**: 関節指令を送る直前に、(1) 可動域(`ctrlrange`)でクリップ、(2) 1 周期の変化量を速度上限で制限、(3) 作業範囲(baseframe 基準の直方体)外の目標を拒否、(4) 状態の更新が途絶えたら停止(保持)する。すべて純粋関数。
- 🔵 **解決する問題**: sim でも実機でも範囲外・急な指令を送らない(NFR-02)。実機の初回は速度 30% 以下、作業範囲外の目標を拒否、通信途絶で停止(NFR-07 (2)(3)(4))。`SimRobot.send_command` はクリップしないため、フィルタを呼び出し側で通す必要がある(`src/so101_vlm/robot/sim.py` の docstring)。
- 🟡 **想定する利用者**: TASK-0012 の軌道生成(テストで「フィルタが軌道を変えない」ことを確認)、TASK-0013 の実行スクリプト、TASK-0035 の実機の指令ループ。
- 🔵 **システム内の位置**: design.md 4.6 節「安全フィルタ(motion)」。Policy(IkPolicy)→ 安全フィルタ → Robot.send_command の間に入る。
- 🔵 **TASK-0010 で作らないもの**: 非常停止(トルク OFF、TASK-0035)、Robot をくるむラッパー(`SafeRobot` など、TASK-0013/0035 で検討)、FK による手先位置の監視、衝突判定。
- **参照した EARS 要件**: NFR-02、NFR-07(2)(3)(4)
- **参照した設計文書**: design.md 4.6 節、4.5 節、5 節

---

## 2. 入力・出力の仕様

### 2.1 配置 🟡

| ファイル | 内容 |
|---|---|
| `src/so101_vlm/safety/__init__.py` | 公開名の再エクスポート |
| `src/so101_vlm/safety/limits.py` | 純粋関数と `SafetyResult`(design.md / TASK-0010.md どおり 🔵) |
| `src/so101_vlm/safety/config.py` | `WorkspaceBox`、`SafetyConfig`、`load_safety_config`(yaml 依存を limits.py から分ける 🟡) |
| `configs/motion.yaml` | `safety:` セクション(Green で作成。内容は 5 節) |
| `tests/safety/test_limits.py`、`tests/safety/test_config.py` | テスト |

単位はすべて rad、rad/s、s、m。関節順は `JOINT_NAMES`(shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper)🔵。

### 2.2 関数の仕様

共通の入力規則(🟡、`validate_q_target` と `eval/validation.py` の方針に合わせる):
- 関節配列は shape (6,)、目標点は shape (3,)。違えば `ValueError`(引数名入り)。
- 配列の dtype が bool、または値に NaN / inf があれば `ValueError`。
- 実数のスカラー引数に bool を渡したら `ValueError`。
- 入力配列は変更しない。戻り値は新しい float64 配列。同じ入力なら同じ出力(乱数・時計・グローバル状態を使わない)。

#### `clip_to_limits(q, lower, upper) -> np.ndarray` 🔵

- 各関節を [lower_i, upper_i] に丸める(`np.clip` と同じ。境界ちょうどは変えない)。
- `lower_i < upper_i` でなければ `ValueError` 🟡。

#### `limit_step(q_current, q_target, vmax, scale, dt) -> np.ndarray` 🔵(式)/ 🟡(一様スケーリング)

- 1 周期の許容変化量 `m_i = vmax_i × scale × dt`(関節ごと)。
- `Δ = q_target − q_current`。`α = min(1, min_{i: Δ_i ≠ 0} m_i / |Δ_i|)`。
- 戻り値 `q_current + clip(α·Δ, −m, +m)`。`α = 1`(全関節が `|Δ_i| ≤ m_i`)なら `q_target` と同じ値を返す。
- 末尾の `clip` は浮動小数の丸めで `m_i` をわずかに超えないための保険で、方向はほぼ変えない。
- **一様スケーリングを選んだ理由**: 関節ごとのクリップでは、関節ごとに到着時刻がずれ、関節空間の経路が直線から曲がる。TASK-0012 は関節空間の直線補間を前提にしているため、方向を保つ方が計画した経路(safety-survey.md 2.3 節で直方体内を確認した経路)から外れない。各関節の上限は同じく守られる(一様スケーリングの結果は関節ごとのクリップの結果より常に遅いか同じ)。
- 引数: `vmax` は shape (6,)、各要素 > 0 🟡(スカラーは受け付けない。関節ごとに値を変えられる形にそろえる)。`scale` は (0, 1]、`dt` > 0。違反は `ValueError`。
- 「制限した」の判定(`rate_limited`)は、いずれかの関節で `|Δ_i| > m_i`(厳密な不等号)のとき。`|Δ_i| = m_i` ちょうどは制限しない。

#### `is_in_workspace(p_base, box: WorkspaceBox) -> bool` 🔵(目的)/ 🟡(境界)

- `box.lower ≤ p ≤ box.upper` が 3 軸すべてで成り立てば True(**境界を含む閉区間**)。
- `p_base` の shape が (3,) でない、bool、NaN / inf は `ValueError`。
- 判定の対象は「呼び出し側が IK に渡す目標点」(`solve_ik` の `target_pos`)。`tcp_offset` を使う場合は TCP 点、使わない場合は gripperframe の点。どちらも同じ直方体で判定する(safety-survey.md 2.4 節で両方とも内側に入ることを確認)。座標系は baseframe 基準(sim ではワールド座標と一致。実測 🔵)。

#### `check_state_fresh(last_update_t, now, timeout) -> bool` 🔵(目的)/ 🟡(細部)

- `0 ≤ now − last_update_t ≤ timeout` なら True(**ちょうど timeout は新鮮**)。
- `last_update_t` または `now` が NaN / inf、または `now < last_update_t`(時計の逆行)なら False(停止側に倒す。時刻は実行時の計測値なので例外にしない)。
- `timeout` が bool・非有限・0 以下なら `ValueError`(設定の誤り)。
- `last_update_t` と `now` は同じ時計で測る(sim はシミュレーション時刻 `RobotState.timestamp`、実機は単調時計)。

#### `limits_from_model(model: mujoco.MjModel) -> tuple[np.ndarray, np.ndarray]` 🔵

- アクチュエータを `JOINT_NAMES` の名前で引き、`actuator_ctrlrange[:, 0]` / `[:, 1]` を (lower, upper) として返す(`policy/ik.py` の `_Arm` と同じ方式)。新しい配列を返す。
- 名前が無ければ名前入りの `ValueError`。
- 現在のモデルの値: lower = (-1.91986, -1.74533, -1.69, -1.65806, -2.74385, -0.17453)、upper = (1.91986, 1.74533, 1.69, 1.65806, 2.84121, 1.74533)(`assets/so101/so101_new_calib.xml`)🔵。

#### `apply_safety(q_current, q_target, *, lower, upper, last_state_t, now, config, target_base=None) -> SafetyResult` 🟡

`q_current` は**速度制限と保持の基準**。前回送った指令を渡すことを推奨する(開始時は計測した関節角)。計測した関節角を渡すこともできるが、sim の追従遅れ(1.0 rad/s で最大 0.0096 rad、1 周期の許容 0.02 rad の約半分)のため、上限ちょうどの軌道が毎周期制限される(safety-survey.md 3.2 節)。

処理の順序(先に当てはまったもので終わる):

1. **構造の検証**: `q_current`、`lower`、`upper` の shape・dtype・有限性、`q_target` と `target_base` の shape・dtype、`config` の型。違反は `ValueError`。`q_current` は保持の指令になるため、ここで有限性まで確認する。
2. **状態途絶**: `check_state_fresh(last_state_t, now, config.state_timeout)` が False なら `stopped=True`、`q_command = q_current`(コピー)、`reason="stale_state"`。`q_target` の値は見ない(NaN でも停止を返す)。
3. **値の検証**: `q_target`、`target_base` に NaN / inf があれば `ValueError`。
4. **作業範囲**: `target_base` が与えられ、`is_in_workspace` が False なら `rejected=True`、`q_command = q_current`、`reason="target_out_of_workspace"`。
5. **可動域**: `q_clip = clip_to_limits(q_target, lower, upper)`。`q_clip ≠ q_target` なら `clipped=True`。
6. **速度**: `q_command = limit_step(q_current, q_clip, config.vmax, config.speed_scale, config.control_dt)`。制限したら `rate_limited=True`。
7. **最終クリップ**: `q_current` が可動域の外にあると、6 の結果も外になりうる。`q_command` を再度 `clip_to_limits` し、変わったら `clipped=True`。NFR-02(範囲外を送らない)を速度制限より優先する。この場合、変化量は `m` を超えうる(実機の較正誤差で関節角がわずかに範囲外に出る場合を想定。🟡)。

- 安全上の判定結果(停止・拒否・クリップ・速度制限)は**例外にせずフラグで返す**。`ValueError` は入力の型・shape・NaN/inf・bool の誤り(プログラムのバグ)だけ。
- フィルタは状態を持たない。停止・拒否の後に再開するかは呼び出し側が決める(TASK-0035)。

### 2.3 結果型 `SafetyResult`(frozen dataclass)🟡

| フィールド | 型 | 意味 |
|---|---|---|
| `q_command` | `np.ndarray` (6,) | 送る関節目標。`__post_init__` で float64 にコピーし読み取り専用にする(`IkSolution` と同じ方式) |
| `clipped` | `bool` | 可動域で丸めた(5 または 7) |
| `rate_limited` | `bool` | 速度上限で縮めた(6) |
| `rejected` | `bool` | 作業範囲外の目標を拒否した(4) |
| `stopped` | `bool` | 状態途絶で停止した(2) |
| `reason` | `str \| None` | `"stale_state"` / `"target_out_of_workspace"` / `None` |

- 優先順位は `stopped` > `rejected` > (`clipped`, `rate_limited`)。`stopped` か `rejected` のとき、他のフラグはすべて False。
- `reason` は `stopped` か `rejected` のときだけ値を持つ。理由の文字列は `limits.py` の定数(`REASON_STALE_STATE`、`REASON_OUT_OF_WORKSPACE`)にする(`ik_types.py` の `REASON_*` と同じ方式)。
- 「停止」= `q_command = q_current`(その場で保持)。位置制御のアクチュエータは保持し続ける。トルク OFF ではない(非常停止は TASK-0035)🔵(note.md 判断 4)。

### 2.4 設定 `WorkspaceBox` / `SafetyConfig`(frozen dataclass、`safety/config.py`)🟡

```python
@dataclass(frozen=True)
class WorkspaceBox:
    lower: tuple[float, float, float]  # baseframe 基準 [m]
    upper: tuple[float, float, float]  # 各軸 lower < upper

@dataclass(frozen=True)
class SafetyConfig:
    vmax: tuple[float, ...]   # 6 個、各 > 0 [rad/s]
    control_dt: float         # > 0 [s]
    speed_scale: float        # (0, 1]
    state_timeout: float      # > 0 [s]
    workspace: WorkspaceBox

def load_safety_config(path: str | Path = DEFAULT_MOTION_CONFIG_PATH) -> SafetyConfig
```

- `__post_init__` で検証し、違反は名前入りの `ValueError`: 値が bool・非有限・非数値、`vmax` が 6 個でない・0 以下、`control_dt` ≤ 0、`speed_scale` ∉ (0, 1]、`state_timeout` ≤ 0、`workspace` の各軸で `lower < upper` でない。
- 配列ではなく tuple で持つ(frozen を保つ)。関数に渡すときに `np.asarray` する。
- 実機の 0.3 倍は `dataclasses.replace(config, speed_scale=0.3)` で作れる(専用関数は作らない)。
- `load_safety_config`:
  - `DEFAULT_MOTION_CONFIG_PATH = REPO_ROOT / "configs" / "motion.yaml"`。`yaml.safe_load` で読む(`eval/eval_config.py` と同じ)。
  - ファイルが無い → `FileNotFoundError`(パス入り)。最上位が mapping でない、`safety:` が無い → `ValueError`。
  - `safety:` の中のキーは `vmax`、`control_dt`、`speed_scale`、`state_timeout`、`workspace` ちょうど。`workspace:` の中は `lower`、`upper` ちょうど。**欠落・未知のキーは `ValueError`**(キー名入り。`_require_mapping` と同じ方針)。
  - **`safety:` 以外の最上位キーは無視する**(TASK-0012 が同じファイルに接近高さなどを追加するため)。
  - 数値の検証は `eval/validation.py` の `require_number` を流用してよい。

---

## 3. 制約条件

- 🔵 **NFR-02**: 送る指令は常に可動域の内側(2.2 節の手順 5 と 7)。速度上限を設ける(手順 6)。
- 🔵 **NFR-07(2)**: 速度倍率 `speed_scale` を設定で変えられる。実機の初回は 0.3(TASK-0035 で `configs/real.yaml` から与える)。
- 🔵 **NFR-07(3)**: 作業範囲外の目標は拒否(手順 4)。
- 🔵 **NFR-07(4)**: 状態が途絶えたら停止(手順 2)。
- 🔵 **不変性・純粋関数**: 入力を変更しない、結果は frozen + 読み取り専用配列(`docs/spec/so101-vlm-pickplace/note.md`、`robot/base.py`)。
- 🔵 **ファイル規模**: 1 ファイル 200〜400 行目安・最大 800 行、関数 50 行未満。
- 🔵 **テスト**: `uv run pytest`、safety のカバレッジ 80% 以上。既存テスト(469 件)を壊さない。
- 🟡 **fail closed**: 判定できない時刻は停止側。配列の NaN は `ValueError`(送る前に止まる)。
- 🟡 **性能**: 1 回の `apply_safety` は numpy の数演算のみ(MuJoCo を呼ばない)。制御周期 0.02 s に比べ十分短い(目安 1 ms 未満。厳密な性能テストは不要)。

### 3.1 既定値と根拠

| 値 | 既定 | 信頼性 | 根拠 |
|---|---|---|---|
| 作業範囲 lower | (0.12, -0.13, 0.00) | 🟡 | タスク点の最小 (0.15, -0.10, 0.015) − 3 cm。z は床面で止める(gripperframe 目標は z 0.0023 まで下がる)。safety-survey.md 2 節 |
| 作業範囲 upper | (0.38, 0.26, 0.17) | 🟡 | タスク点の最大 (0.35, 0.23, 0.14) + 3 cm。y 0.23 は箱の内寸の端 |
| `vmax` | 全関節 1.0 rad/s | 🔴(実機)/ 🟡(sim) | sim の pick & place 1 回 5.2〜6.8 s(0.5 なら 10〜14 s)。sim は最大 5.3〜5.7 rad/s 出せ、1.0 rad/s の追従誤差は 0.0096 rad。STS3215 の公称無負荷速度(約 4.7 rad/s、プロジェクト外の情報で未確認)の約 2 割 |
| `speed_scale` | 1.0(sim)/ 0.3(実機の初回) | 🔵 | NFR-07(2) |
| `control_dt` | 0.02 s | 🔵 | `timestep` 0.002 × `DEFAULT_N_SUBSTEPS` 10 を実測。実機の周期は TASK-0034/0035 で確認 |
| `state_timeout` | 0.2 s(10 周期) | 🔴 | 資料に値なし。sim は毎周期 `get_state` するので途絶しない。実機の読み取り周期を見て TASK-0035 で調整 |

作業範囲の検証結果(safety-survey.md 2.5 節): タスク点 172 / 172 が内側。土台(base / shoulder の外接箱 x ≤ 0.110)、土台の後ろ、床の下、q = 0 の手先 (0.391, 0, 0.227)、z 0.18、x 0.40 は外側。

---

## 4. 想定される使用例

### 4.1 基本の使い方(sim、TASK-0013 想定)🟡

```python
config = load_safety_config()
lower, upper = limits_from_model(model)
q_ref = robot.get_state().q            # 開始時のみ計測値
for q_target, target_base in trajectory:
    state = robot.get_state()
    res = apply_safety(q_ref, q_target, lower=lower, upper=upper,
                       last_state_t=state.timestamp, now=state.timestamp,  # sim: 同じ時計
                       config=config, target_base=target_base)
    robot.send_command(res.q_command)
    if res.stopped or res.rejected:
        break                          # 再開の判断は呼び出し側
    q_ref = res.q_command              # 前回の指令を次の基準にする
```

### 4.2 エッジケース(テストで確認する境界)

| # | 入力 | 期待 | 信頼性 |
|---|---|---|---|
| E1 | 可動域の上限ちょうど / 下限ちょうど | 変わらない、`clipped=False` | 🟡 |
| E2 | 上限超え / 下限未満 | 境界の値、`clipped=True` | 🔵 |
| E3 | 大きく離れた目標、scale = 0.3 | 各関節の変化量 ≤ vmax·0.3·dt、変化の方向は Δ と平行 | 🔵 |
| E4 | `|Δ_i| = m_i` ちょうど | 目標に届く、`rate_limited=False` | 🟡 |
| E5 | 関節ごとに違う vmax | 最も厳しい関節で α が決まる | 🟡 |
| E6 | 目標 = 現在値(Δ = 0) | そのまま、`rate_limited=False`(0 除算しない) | 🟡 |
| E7 | 作業範囲の各面の上、各軸で 1 mm 外 | 面の上は True、外は False | 🟡 |
| E8 | 経過 = timeout ちょうど / timeout をわずかに超える | 新鮮 / 停止 | 🟡 |
| E9 | 未来の時刻、NaN の時刻 | 停止 | 🟡 |
| E10 | 停止 かつ q_target が NaN | 停止を返す(例外にしない) | 🟡 |
| E11 | 停止しておらず q_target が NaN / inf | `ValueError` | 🟡 |
| E12 | shape 違い、bool 配列、bool のスカラー | `ValueError` | 🟡 |
| E13 | 作業範囲外 かつ 可動域外の目標 | `rejected=True`、`clipped=False` | 🟡 |
| E14 | `q_current` が可動域外 | `q_command` は可動域内、`clipped=True` | 🟡 |
| E15 | 入力配列 | 呼び出し後も変わらない。`q_command` は書き込み不可 | 🔵 |
| E16 | 同じ入力で 2 回 | 同じ結果 | 🟡 |
| E17 | 設定: 欠落・未知キー(`safety:` と `workspace:` の中)、負の vmax、vmax 5 個、scale 0 / 1.01、lower ≥ upper、bool 値 | `ValueError`(キー名入り) | 🟡 |
| E18 | 設定: `safety:` 以外の最上位キーがある | 無視して読める | 🟡 |
| E19 | `limits_from_model` | JOINT_NAMES 順の ctrlrange と一致 | 🔵 |
| E20 | タスク点(把持・接近 × 9×9、箱の解放・上方)と既定の作業範囲 | すべて True | 🔵(実測) |

---

## 5. `configs/motion.yaml` の内容(提案。Green で作成する)

```yaml
# 安全フィルタ (TASK-0010)。単位: rad, rad/s, s, m。座標は baseframe 基準 (sim ではワールド座標と同じ)
# 数値の根拠: docs/implements/so101-vlm-pickplace/TASK-0010/safety-survey.md
safety:
  vmax: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]  # 関節ごとの最大角速度。STS3215 の実測値は未確認 (実機で調整)
  control_dt: 0.02                       # sim: timestep 0.002 x n_substeps 10
  speed_scale: 1.0                       # sim は 1.0。実機の初回は 0.3 (NFR-07, configs/real.yaml で上書き)
  state_timeout: 0.2                     # 状態途絶の判定 (制御周期 10 回分)。実機で調整
  workspace:                             # IK に渡す目標点の許容範囲 (境界を含む)
    lower: [0.12, -0.13, 0.00]           # ブロック範囲と箱の内寸 - 3 cm。z は床面
    upper: [0.38, 0.26, 0.17]            # 同 + 3 cm
```

---

## 6. 後続タスクへの影響

### 6.1 TASK-0012(軌道生成)🟡

「軌道が制限を満たす」= フィルタを通しても何も変わらない、の定義:

1. 軌道を `control_dt` ごとの指令列 q_0, q_1, …, q_N にしたとき、すべての k で `|q_{k+1,i} − q_{k,i}| ≤ vmax_i × speed_scale × control_dt`。q_0 は開始時の計測値。
2. すべての q_k が [lower, upper] の内側(境界を含む)。
3. デカルト座標の目標(pre_grasp、grasp、lift、pre_place、place)がすべて `is_in_workspace` で True。
4. テストは「q_ref = 前回の指令」で `apply_safety` を順に通し、すべての k で `clipped`・`rate_limited`・`rejected`・`stopped` が False、かつ `q_command == q_{k+1}` を確認する。
- 浮動小数の誤差で上限ちょうどを超えないよう、区間の時間は 1% 程度の余裕を持たせる(例: 速度 0.99 × vmax × scale で時間を決める)ことを推奨。
- 区間の時間は一様スケーリングと同じ考え方(区間内の最大 `|Δq_i| / (vmax_i × scale)`)で決めると、関節空間の直線を保てる。
- gripper の開閉も同じ vmax で制限される。開閉 1.0 rad で scale 1.0 なら 1.0 s、0.3 なら 3.3 s。
- 区間の途中の手先は直方体の内側に収まることを確認済み(最大 4 mm 膨らむ。safety-survey.md 2.3 節)。ホームなど関節空間で指定する姿勢(q = 0 の手先は直方体の外)は作業範囲の判定対象にしない。

### 6.2 TASK-0013(固定位置の pick & place)🟡

- 4.1 節の形で毎周期 `apply_safety` を通してから `send_command` する。sim では `last_state_t` と `now` に同じ `RobotState.timestamp` を使う(途絶は起きない)。
- 所要時間の目安: vmax 1.0、scale 1.0 で 1 回 5〜7 s(ホームからの移動を含む)。
- `Robot` をくるむラッパー(`SafeRobot`)を作るかはこのタスクで判断する。

### 6.3 TASK-0035(実機の安全機能)🟡

- `speed_scale` 0.3 を `configs/real.yaml` から与え、`dataclasses.replace` で上書きする(キー名は TASK-0035 で決める)。0.3 倍では 1 回 17〜23 s。
- `now` は単調時計(`time.monotonic()`)、`last_state_t` は最後に状態を読めた時刻(同じ時計)。`state_timeout` 0.2 s は実機の読み取り周期を測って調整する。
- 停止は保持(最後の指令を送り続ける、または送らない)。サーボ自身が目標を保持する。トルク OFF は非常停止として別に作る。
- 停止・拒否からの再開で、古い軌道を自動で続けない。
- 実機の較正誤差で関節角が可動域をわずかに外れる場合、最終クリップで指令は範囲内に戻る(2.2 節の手順 7)。

---

## 7. EARS 要件・設計文書との対応

- **参照した機能要件**: なし(安全は非機能要件)
- **参照した非機能要件**: NFR-02、NFR-07(2)(3)(4)。NFR-07(1) 非常停止は TASK-0035
- **参照したタスク**: TASK-0010(完了条件: 4 機能の境界テスト、速度倍率を設定で変更)、TASK-0012(テストケース 2「制限を守る」)、TASK-0013、TASK-0035
- **参照した設計文書**: design.md 1 節(ctrlrange、baseframe/gripperframe)、4.5 節(軌道・関節空間の直線補間)、4.6 節(安全フィルタ)、5 節(`motion.yaml`: 速度上限、接近高さ、作業範囲)
- **参照したコード**: `src/so101_vlm/robot/base.py`(`validate_q_target`、`RobotState`)、`src/so101_vlm/robot/sim.py`(`DEFAULT_N_SUBSTEPS`、クリップしない方針)、`src/so101_vlm/eval/scene_spec.py`、`src/so101_vlm/policy/ik.py`(`_Arm` の ctrlrange 取得、`tcp_offset`)、`src/so101_vlm/policy/ik_types.py`(frozen の結果型・`REASON_*`)、`src/so101_vlm/eval/eval_config.py` / `validation.py`(設定検証)
- **参照した調査**: `docs/implements/so101-vlm-pickplace/TASK-0011/reachability-survey.md`、`ik-verify-complete.md`、本タスクの `safety-survey.md`

---

## 8. 未決事項(ユーザー確認。既定値で進める)

| # | 事項 | 既定(推奨) | 信頼性 |
|---|---|---|---|
| D1 | 安全用の作業範囲を、配置用の `scene_spec.work_area_*`(y ≤ 0.10)とは別に持ち、箱の位置まで広げてよいか | 別に持つ。x 0.12〜0.38 / y -0.13〜0.26 / z 0.00〜0.17(シーンは変更しない) | 🔴 |
| D2 | `vmax` を全関節 1.0 rad/s にしてよいか。STS3215 の実測値・公称値があれば置き換える | 1.0 rad/s(sim で 1 回 5〜7 s)。実機初回は × 0.3 = 0.3 rad/s | 🔴 |
| D3 | `state_timeout` 0.2 s | 0.2 s。実機の読み取り周期を測って TASK-0035 で調整 | 🔴 |
| D4 | 実機の `speed_scale` を置く場所とキー名 | `configs/real.yaml`(TASK-0035 で決定)。`motion.yaml` は 1.0 | 🟡 |
| D5 | 速度制限の方式 | 一様スケーリング(方向を保つ)。関節ごとのクリップにしたい場合は TASK-0012 の補間と合わせて変更 | 🟡 |
| D6 | 床面 z = 0 の目標を許すか(手先が机に当たる問題はこの判定では防げない) | 許す(z ≥ 0.00、境界を含む)。机との接触は TASK-0012/0013 の sim で確認 | 🟡 |
| D7 | gripper も同じ vmax で制限してよいか(開閉 1 rad に scale 0.3 で 3.3 s) | 同じ値。遅すぎれば `vmax` の 6 番目だけ上げる | 🟡 |

---

## 9. 品質判定

| 観点 | 評価 |
|---|---|
| 要件の曖昧さ | 少ない。境界(閉区間・ちょうど timeout・`|Δ| = m`)、順序、フラグの優先順位、例外の範囲を明記 |
| 入出力定義 | 完全(関数 6 個、結果型、設定型、ローダー、yaml の形) |
| 制約条件 | 明確(NFR-02/07、不変性、規模、fail closed) |
| 実装可能性 | 確実(numpy のみ。既存の検証ヘルパーと同じ方式) |
| 信頼性の分布 | 🔵 中心 + 🟡 多め。🔴 は実機に依存する 3 値(作業範囲の採否 D1、`vmax` D2、`state_timeout` D3)のみ |

**判定**: ✅ 高品質(実機依存の数値 3 件はユーザー確認待ちだが、既定値で実装・テスト可能)

次のお勧めステップ: `/tsumiki:tdd-testcases so101-vlm-pickplace TASK-0010` でテストケースの洗い出しを行います。

# TASK-0010 開発コンテキストノート: 安全フィルタ(可動域・速度・作業範囲)

**要件名**: so101-vlm-pickplace
**タスク**: TASK-0010 安全フィルタ(可動域・速度・作業範囲)
**作成方法**: tsumiki:tdd-tasknote
**信頼性**: 🔵 要件・設計・既存コードに基づく / 🟡 妥当な推測 / 🔴 プロジェクト内に根拠がない仮定(要確認)

---

## 0. 目的と範囲

関節指令を「可動域でクリップ」「1 ステップの変化量を制限」「作業範囲外の目標を拒否」「状態途絶で停止」する。すべて純粋関数で、入力は変更しない。

**TASK-0010 で作るもの**: 純粋関数と設定読み込みだけ。SimRobot や実機ドライバには触れない。
**TASK-0010 で作らないもの**: 非常停止(TASK-0035)、`SafeRobot` ラッパー(TASK-0013 / TASK-0035 で検討)、FK による手先位置の監視(TASK-0012 のテストで必要になったら追加)。

---

## 1. 技術スタック

- Python 3.12(`requires-python = ">=3.12,<3.13"`)、パッケージ管理は uv、テストは pytest。参照元: `pyproject.toml`
- 設定読み込みは `pyyaml`(`yaml.safe_load`)。既存の `src/so101_vlm/eval/eval_config.py` と同じ方式。参照元: `pyproject.toml`、`src/so101_vlm/eval/eval_config.py`
- 可動域は MuJoCo モデルの `actuator_ctrlrange` から取る。関節順は `JOINT_NAMES`(6 関節)。参照元: `src/so101_vlm/robot/model_paths.py`、`src/so101_vlm/robot/base.py`
- 単位: 角度 rad、角速度 rad/s、時間 s、長さ m。参照元: `src/so101_vlm/robot/base.py`(RobotState の単位)

---

## 2. 開発ルール

- TDD(RED → GREEN → REFACTOR)。カバレッジ 80% 以上。参照元: `docs/spec/so101-vlm-pickplace/note.md`
- 不変性: データは `frozen` な dataclass、配列は書き換えず新しく返す。参照元: `docs/spec/so101-vlm-pickplace/note.md`、`src/so101_vlm/robot/base.py`(`RobotState`)
- 1 ファイル 200〜400 行目安、最大 800 行。関数は 50 行未満。参照元: `docs/spec/so101-vlm-pickplace/note.md`
- エラーは握りつぶさず、名前入りの `ValueError` にする。参照元: `src/so101_vlm/robot/base.py`(`validate_q_target`)
- 設定の検証は未知キー・欠落キー・型違反を `ValueError` にする。参照元: `src/so101_vlm/eval/eval_config.py`
- `AGENTS.md` と `docs/rule/` は存在しない。`CLAUDE.md` はテスト実行コマンドのみ(`uv run pytest`)。

---

## 3. 関連実装

| 対象 | 場所 | 使い方 |
|---|---|---|
| 関節検証・RobotState | `src/so101_vlm/robot/base.py` | `validate_q_target` を再利用(shape (6,)、NaN/inf 拒否、コピーを返す) |
| 関節順 | `src/so101_vlm/robot/model_paths.py` | `JOINT_NAMES`(shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper) |
| SimRobot | `src/so101_vlm/robot/sim.py` | `send_command` は**クリップしない**。安全フィルタを通すのは呼び出し側の責務(docstring にも明記) |
| 作業範囲 | `src/so101_vlm/eval/scene_spec.py` | `SCENE_SPEC.work_area_x = (0.15, 0.35)`、`work_area_y = (-0.10, 0.10)`、箱の中心 `box_center_xy = (0.20, 0.19)`、縁の高さ `box_rim_z = 0.04` |
| IK の可動域利用 | `src/so101_vlm/policy/ik.py` | `_Arm.__init__` が `model.actuator_ctrlrange[act_ids, 0/1]` を取得する方式。`limits_from_model` も同じ方式にそろえる |
| IK の結果型 | `src/so101_vlm/policy/ik_types.py` | 結果を frozen dataclass で返し、`__post_init__` で配列をコピーして読み取り専用にする方式。`SafetyResult` も同じ方式 |
| 設定読み込みの手本 | `src/so101_vlm/eval/eval_config.py`、`src/so101_vlm/eval/validation.py` | `_require_mapping`(未知/欠落キー検出)、`require_number`、`require_int` を流用できる |

参照元: `src/so101_vlm/robot/base.py`、`src/so101_vlm/robot/sim.py`、`src/so101_vlm/robot/model_paths.py`、`src/so101_vlm/eval/scene_spec.py`、`src/so101_vlm/policy/ik.py`、`src/so101_vlm/policy/ik_types.py`、`src/so101_vlm/eval/eval_config.py`、`src/so101_vlm/eval/validation.py`

---

## 4. 設計文書

### 4.1 要件との対応

| 要件 | 内容 | 対応 |
|---|---|---|
| NFR-02 | 関節角・速度に上限。実機では範囲外の指令を送らない | `clip_to_limits`、`limit_step` |
| NFR-07(2) | 初回の動作確認は速度上限を通常の 30% 以下 | `speed_scale`(設定値。実機初回は 0.3) |
| NFR-07(3) | 作業範囲の外の目標は拒否 | `is_in_workspace`、`apply_safety` の `rejected` |
| NFR-07(4) | 通信が途絶えたらその場で停止 | `check_state_fresh`、`apply_safety` の `stopped` |
| AC-V2-04 | 非常停止(キー 1 回でトルク OFF) | TASK-0035 の範囲。ここでは扱わない |

参照元: `docs/requirements.md`(NFR-02, NFR-07, AC-V2-04)、`docs/design.md` 4.6 節、`docs/tasks/so101-vlm-pickplace/TASK-0010.md`

### 4.2 関数の仕様

すべて `src/so101_vlm/safety/limits.py` に置く。

```python
def clip_to_limits(q: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> np.ndarray
    # 各関節を [lower, upper] に丸める。新しい配列を返す。入力は変更しない。

def limit_step(q_current, q_target, vmax, scale, dt) -> np.ndarray
    # 1 ステップの変化量を各関節 m = vmax * scale * dt 以下にする。
    # vmax は (6,) の配列(関節ごと)。scale は (0, 1]、dt > 0。
    # 戻り値 = q_current + clip(q_target - q_current, -m, +m)

def is_in_workspace(p_base, box: WorkspaceBox) -> bool
    # p_base(baseframe 基準の 3 次元点)が直方体の内側(境界を含む)なら True。
    # NaN / inf は False(fail closed)。shape が (3,) でなければ ValueError。

def check_state_fresh(last_update_t: float, now: float, timeout: float) -> bool
    # 0 <= now - last_update_t <= timeout なら True。
    # 未来の時刻・NaN は False(時計の異常は停止側に倒す)。timeout <= 0 は ValueError。

def limits_from_model(model: mujoco.MjModel) -> tuple[np.ndarray, np.ndarray]
    # JOINT_NAMES 順の (lower, upper) を actuator_ctrlrange から返す。

def apply_safety(q_current, q_target, *, lower, upper, last_state_t, now, config, target_base=None) -> SafetyResult
    # 合成関数。順序は次のとおり。
    # 1. 入力の shape と有限性を検証(違反は ValueError)
    # 2. 状態が古ければ停止: q_command = q_current(コピー)、stopped = True
    # 3. target_base があり作業範囲外なら保持: q_command = q_current、rejected = True
    # 4. q_target を clip_to_limits で可動域に収める(clipped)
    # 5. limit_step で変化量を制限(rate_limited)
```

### 4.3 結果型 `SafetyResult`(frozen dataclass)

```python
@dataclass(frozen=True)
class SafetyResult:
    q_command: np.ndarray   # 実際に送る関節目標 (6,)。__post_init__ でコピーし読み取り専用にする
    clipped: bool           # 可動域で丸めた
    rate_limited: bool      # 速度上限で制限した
    rejected: bool          # 作業範囲外の目標を拒否した
    stopped: bool           # 状態途絶により停止した
    reason: str | None      # None / "stale_state" / "target_out_of_workspace"
```

- 停止・拒否のときは `q_command` が `q_current` と同じになる(保持)。
- フラグ同士の優先順位: `stopped` > `rejected` > (`clipped`, `rate_limited`)。停止か拒否のときは `clipped` と `rate_limited` を False にする。
- `reason` は `stopped` または `rejected` のときだけ値を持つ。

### 4.4 設定データ(frozen dataclass)

`src/so101_vlm/safety/config.py` に置く。

```python
@dataclass(frozen=True)
class WorkspaceBox:
    center: tuple[float, float, float]     # baseframe 基準 [m]
    half_size: tuple[float, float, float]  # 各軸の半幅 [m] (> 0)

@dataclass(frozen=True)
class SafetyConfig:
    vmax: tuple[float, ...]        # 関節ごとの最大角速度 [rad/s] (6 個、各 > 0)
    control_dt: float              # 制御周期 [s] (> 0)
    speed_scale: float             # 速度倍率 (0, 1]
    state_timeout: float           # 状態途絶の判定時間 [s] (> 0)
    workspace: WorkspaceBox

def load_safety_config(path=DEFAULT_MOTION_CONFIG_PATH) -> SafetyConfig
```

- 配列ではなく tuple で持つ(frozen を保つため)。関数に渡すときに `np.asarray` する。
- 読み込みは `safety:` セクションだけを見る。`configs/motion.yaml` の他のキー(TASK-0012 の接近高さなど)は無視する。未知キーは `safety:` の中だけで検出して `ValueError` にする。

### 4.5 設定ファイル `configs/motion.yaml`

```yaml
# 安全フィルタ (TASK-0010)。単位: rad, rad/s, s, m
safety:
  vmax: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]   # 🔴 STS3215 の実測値はプロジェクト内に無い。仮置き
  control_dt: 0.02                        # 🟡 sim の send_command 1 回分(10 × 0.002 s)に合わせる
  speed_scale: 1.0                        # 🔵 sim は 1.0。実機の初回は 0.3(NFR-07)
  state_timeout: 0.2                      # 🔴 仮置き。制御周期の 10 回分
  workspace:
    center: [0.25, 0.065, 0.10]           # 🟡 baseframe 基準(baseframe はワールド原点と仮定)
    half_size: [0.10, 0.165, 0.10]        # x 0.15〜0.35 / y -0.10〜0.23 / z 0.00〜0.20
```

---

## 5. 設計判断(デフォルトで決めた事項)

### 判断 1: 設定の置き場所 → `configs/motion.yaml` の `safety:` セクション

- 理由: `docs/design.md` 5 節が「`motion.yaml`: 速度上限、接近高さ、作業範囲」と明記している。設計書に合わせる。
- 注意: TASK-0012 も `configs/motion.yaml` に接近高さなどを追加する。そのため読み込みは `safety:` だけを見る。
- 実機の 0.3 倍率: TASK-0035 は「`configs/real.yaml` で 0.3」と書いている。TASK-0010 は `speed_scale` の既定値(1.0)を持ち、実機用の値は TASK-0035 が `real.yaml` から渡す。どちらのキー名にするかは TASK-0035 で決める(🟡)。

### 判断 2: 配置

- 純粋関数と結果型は `src/so101_vlm/safety/limits.py`。設定の読み込みは `src/so101_vlm/safety/config.py` に分ける。
- 理由: limits.py は設計書どおり。設定読み込みを分けると、limits.py が yaml に依存せず、200〜400 行の目安にも収まる。

### 判断 3: 拒否は例外ではなくフラグ(`SafetyResult.rejected`)にする

- 理由: TASK-0012 の軌道を回す側は、経由点ごとに「拒否・停止」を見て安全に止める必要がある。例外にすると、1 点の拒否で処理全体が中断し、途中の停止処理を呼び出し側が書くことになる。
- ただし入力の不正(shape 違い、NaN/inf)は `ValueError` にする(`validate_q_target` と同じ方針)。これはプログラムのバグであり、実行時の判定結果ではないため。

### 判断 4: 「停止」の意味

- `q_command = q_current`(今の関節角をそのまま目標にする)。角速度は 0 とみなす。
- 位置制御のアクチュエータは、この目標を保持し続ける。`stopped` のあいだは、呼び出し側は毎周期この保持指令を送る。
- フィルタ自体は状態を持たない(純粋関数)。再開するかどうかは呼び出し側が決める。古い軌道を自動で再開しないこと(TASK-0035 で扱う)。
- 実機のトルク OFF(非常停止)とは別物。トルク OFF は TASK-0035 の責務。

### 判断 5: 手先位置(FK)の確認は TASK-0010 に入れない

- 作業範囲の判定は、**デカルト座標の目標**(IK に渡す前の `target_base`)に対して `is_in_workspace` で行う。設計書 4.6 の「作業範囲の外の目標を拒否」と一致する。
- 関節空間の制限(clip / rate)だけでは、手先が箱の外に出ることを防げない。そのため、実行される経由点の手先位置を確認したい場合は、FK が必要になる。
- FK は `policy/ik.py` の `_Arm.pose`(専用 `MjData` を使う方式)と同じやり方で書ける。ただし TASK-0010 の必須範囲には含めない。TASK-0012 のテスト「制限を守る」で必要になった時点で追加する(🟡)。

### 判断 6: 作業範囲は「ブロックの範囲」だけでなく「箱」も含める(要確認 🔴)

- `scene_spec.py` の作業範囲は x 0.15〜0.35、y -0.10〜0.10 である。しかし箱の中心は (0.20, 0.19) で、箱の縁は y 0.15〜0.23 にある。
- そのため、この作業範囲だけを使うと、箱の真上の `pre_place` と `place` が拒否される。
- 対応案: 作業範囲の箱を、ブロック範囲と箱の両方を覆う大きさにする(y -0.10〜0.23)。上の `configs/motion.yaml` の値がこれに当たる。
- 作業範囲は `scene_spec.py` の `work_area_*` と別に持つ(安全上の箱は、配置・評価の範囲とは目的が違うため)。ユーザーに確認すること。

### 判断 7: 作業範囲の z 下限は 0.00

- ブロックの静止中心は z 0.015(`block_rest_z`)。把持の目標は z 0.015 付近になる。下限を 0.02 などにすると、把持の目標が拒否されてしまう。
- 床(テーブル面)より下の目標は拒否する。手先が机に当たる問題は、この値では防げない(🔴 要確認)。

---

## 6. テスト関連情報

- テストフレームワーク: pytest(`pyproject.toml` の `[tool.pytest.ini_options]` は `testpaths = ["tests"]`)。参照元: `pyproject.toml`
- 実行コマンド: `uv run pytest -q`(現在 469 件 pass)、カバレッジは `uv run pytest --cov`。参照元: `CLAUDE.md`
- 既存テストの配置: `tests/robot/`、`tests/policy/`、`tests/eval/`、`tests/perception/`。`tests/safety/` はまだ無い。新規作成する。
- 既存テストの文書ヘッダ例: `tests/robot/test_sim.py`(「テストケース定義: docs/implements/.../testcases.md」と信頼性記号を書く形式)。参照元: `tests/robot/test_sim.py`
- モデルを使うテスト: `SO101_SCENE_PATH`(`src/so101_vlm/robot/model_paths.py`)を読むと実モデルの `ctrlrange` を確認できる。参照元: `src/so101_vlm/robot/model_paths.py`
- テストファイル(予定):
  - `tests/safety/test_limits.py`(TASK-0010 の 4 ケース + 境界ケース)
  - `tests/safety/test_config.py`(設定読み込みの検証)
- 既存の `tests/robot/` に `__init__.py` があるかは未確認。実装時に `ls tests/robot` で確認する(🟡)。

### テストケース案(境界を含む)

| # | テスト | 期待 |
|---|---|---|
| 1 | 可動域の内側の値 | 変わらない |
| 2 | 上限を超えた値 | 上限の値になる(TASK-0010 ケース 1) |
| 3 | 下限を下回った値 | 下限の値になる |
| 4 | 上限ちょうど | 変わらない(境界) |
| 5 | 大きく離れた目標、scale=0.3 | 変化量 ≤ vmax × 0.3 × dt(TASK-0010 ケース 2) |
| 6 | scale=1.0 と 0.3 を比べる | 0.3 のときの変化量は 1.0 の 30% 以下 |
| 7 | 関節ごとに違う vmax | 各関節が自分の上限で制限される |
| 8 | 近い目標 | 制限されず、目標にそのまま届く |
| 9 | 入力配列を変えたか | 変わっていない |
| 10 | 作業範囲の内側・境界上・外側(x/y/z 各軸) | 内側と境界は True、外側は False |
| 11 | 作業範囲の外の 3 次元目標 | `rejected=True`、`q_command == q_current`(TASK-0010 ケース 3) |
| 12 | 最後の更新から timeout 以上経過 | `stopped=True`、保持(TASK-0010 ケース 4) |
| 13 | ちょうど timeout | 新鮮(True) |
| 14 | 未来の時刻、NaN | False(停止側) |
| 15 | target に NaN / inf、shape 違い | `ValueError` |
| 16 | 設定: scale 0.3 の上書き | 読み込める |
| 17 | 設定: 欠落キー、未知キー、負の vmax、scale = 0 や 1 超 | `ValueError`(キー名入り) |
| 18 | `limits_from_model` | `actuator_ctrlrange` と JOINT_NAMES の順が一致 |

---

## 7. 注意事項

### 技術的制約

- **STS3215 の角速度が未確定(🔴)**: プロジェクト内の仕様書・モデルに実測値が無い。`vmax` は仮の 0.5 rad/s。実機の初回は、その 30%(`speed_scale` 0.3)で動かし、実測で調整する。
- **sim と実機の制御周期(🟡)**: `control_dt` の 0.02 s は sim の `send_command` 1 回分。実機の制御ループの周期は TASK-0034 / TASK-0035 で確認する。
- **sim は自動でクリップしない**: `SimRobot.send_command` は範囲外の値を受け付ける(docstring に明記)。安全フィルタを通すのは呼び出し側。TASK-0013 以降で `SafeRobot` のような Robot Protocol のラッパーを作ることを検討する(TASK-0010 の範囲外)。
- **時刻の基準をそろえる**: `last_state_t` と `now` は同じ時計で測る。SimRobot の `timestamp` は MuJoCo のシミュレーション時刻で、実時間とは違う。sim で使うときは両方シミュレーション時刻にする。
- **gripper の速度制限**: gripper も同じ vmax・scale で制限される。開閉に 1.9 rad かかるため、scale 0.3 では開閉に十数秒かかる。TASK-0012 の時間配分に影響する。

### セキュリティ・安全要件

- 停止は「保持」であり、トルク OFF ではない。非常停止は TASK-0035 で別に作る。
- フィルタは fail closed(判定できない場合は停止・拒否側に倒す): NaN の状態時刻・未来の時刻・範囲外の目標。
- 設定値の検証で、正でない `vmax`・`control_dt`・`state_timeout`、範囲外の `speed_scale` を拒否する。

### 参照元

- `docs/tasks/so101-vlm-pickplace/TASK-0010.md`、`TASK-0012.md`、`TASK-0013.md`、`TASK-0035.md`
- `docs/requirements.md`(NFR-02, NFR-07, AC-V2-04)
- `docs/design.md`(4.2, 4.5, 4.6 節、5 節の設定ファイル一覧)
- `docs/spec/so101-vlm-pickplace/note.md`
- `assets/so101/so101_new_calib.xml`(actuator の `ctrlrange`、`forcerange` ±3.35、`baseframe` / `gripperframe` の site)
- `assets/scene_pickplace.xml`(`box` の位置、ブロックの静止高さ)

---

## 8. 次のステップ(実装フェーズ)

1. `configs/motion.yaml` を作る(`safety:` セクションのみ)
2. `tests/safety/test_limits.py` と `tests/safety/test_config.py` を書く(RED)
3. `src/so101_vlm/safety/limits.py`、`src/so101_vlm/safety/config.py` を実装する(GREEN)
4. 整理する(REFACTOR)。ファイル末尾のコメント・型を確認する
5. 検証する(`uv run pytest -q` が 469 件 + 新規分すべて pass、safety のカバレッジ 80% 以上)

### 確認事項(実装前にユーザーへ)

- 🔴 作業範囲に箱の位置(y 0.15〜0.23)を含めてよいか(判断 6)
- 🔴 `vmax` の仮置き 0.5 rad/s を採用してよいか。STS3215 の実測値があれば教えてほしい
- 🟡 実機の `speed_scale` のキー名を `real.yaml` と `motion.yaml` のどちらに置くか(TASK-0035 と合わせる)

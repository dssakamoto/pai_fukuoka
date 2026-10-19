# TASK-0004 要件定義書: Robot 抽象と SimRobot

**機能名**: sim-robot(Robot Protocol / RobotState / SimRobot)
**タスクID**: TASK-0004
**要件名**: so101-vlm-pickplace
**作成日**: 2026-10-18
**出力ファイル**: `docs/implements/so101-vlm-pickplace/TASK-0004/sim-robot-requirements.md`

> このプロジェクトの要件は `docs/requirements.md`、設計は `docs/design.md` にある(tsumiki の標準配置ではない)。
> 信頼性: 🔵 要件・設計にほぼそのまま書かれている / 🟡 要件・設計から妥当に推測した / 🔴 要件・設計に無い推測(本ステップで決めた暫定値)

---

## 1. 機能の概要

- 🔵 **何をするか**: ロボットの操作(状態取得・関節位置指令・終了)を `Robot` Protocol で抽象化し、MuJoCo 上の実装 `SimRobot` を提供する。
- 🔵 **解決する問題**: 上位の制御(TASK-0013 の pick & place、TASK-0010 の安全フィルタなど)を、シミュレータと実機(`So101Robot`、TASK-0034)のどちらでも同じコードで動かせるようにする(FR-08)。
- 🟡 **想定ユーザー**: このリポジトリの開発者自身と、`Robot` を使う上位モジュール(Policy、評価ループ、dora ノード)。
- 🔵 **システム内での位置づけ**: `src/so101_vlm/robot/` 層。`base.py` に `Robot` Protocol と `RobotState`、`sim.py` に `SimRobot` を置く。関節順は `model_paths.JOINT_NAMES` が唯一の定義で、`base.py` はそれを再エクスポートする(既存テスト `test_joint_names_single_source` を壊さない)。
- **参照した要件**: FR-08、NFR-02
- **参照した設計**: `docs/design.md` 4.1 Robot 抽象、1 章(アクチュエータは位置制御、単位ラジアン)

---

## 2. 入力・出力の仕様

### 2.1 `Robot` Protocol(`src/so101_vlm/robot/base.py`) 🔵

```python
class Robot(Protocol):
    joint_names: tuple[str, ...]
    def get_state(self) -> RobotState: ...
    def send_command(self, q_target: np.ndarray) -> None: ...
    def close(self) -> None: ...
```

- 🔵 メンバーは `joint_names`、`get_state()`、`send_command(q_target)`、`close()` の 4 つ(design.md 4.1)。
- 🔵 単位はラジアン(角度)、ラジアン毎秒(速度)。
- 🟡 `joint_names` は `JOINT_NAMES`(shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper)と同じ順序のタプル。`q`・`dq`・`q_target` の要素順はこれに従う。
- 🟡 `typing.Protocol` として定義する。`runtime_checkable` を付け、テストで `isinstance(sim, Robot)` を確認できるようにする。

### 2.2 `RobotState`(`src/so101_vlm/robot/base.py`) 🟡

| フィールド | 型 | 形状 | 単位 | 説明 |
|---|---|---|---|---|
| `q` | `np.ndarray` (float64) | (6,) | rad | 関節角。`joint_names` の順 |
| `dq` | `np.ndarray` (float64) | (6,) | rad/s | 関節速度。`joint_names` の順 |
| `timestamp` | `float` | - | s | シミュレーション時刻(`MjData.time`)。実機では取得時刻 |

- 🟡 `@dataclass(frozen=True)` とする(タスクファイル「変更不可の dataclass」)。
- 🟡 `q`・`dq` は MuJoCo の配列のコピーで、`writeable=False` を設定した読み取り専用配列とする。書き込もうとすると numpy が `ValueError` を出す。
- 🔴 `timestamp` を持たせる(設計の記載は「関節角, 速度」のみ。記録 FR-09 で時刻が要るため追加)。

### 2.3 `SimRobot`(`src/so101_vlm/robot/sim.py`)

**コンストラクタ**

```python
SimRobot(scene_path: Path = SO101_SCENE_PATH, n_substeps: int = 10)
```

- 🟡 `scene_path`: 読み込む MuJoCo シーン。既定は `SO101_SCENE_PATH`。TASK-0005 のシーン(ブロック・箱入り)を後で渡せるようにするため引数にする。
- 🔴 `n_substeps`: `send_command` 1 回で進める `mj_step` の回数。既定 10(timestep 0.002 s なので 20 ms、50 Hz 制御相当)。1 未満の整数なら `ValueError`。
- 🟡 `JOINT_NAMES` の各名前を `mj_name2id` で引き、アクチュエータ番号・`jnt_qposadr`・`jnt_dofadr` を求めて保持する(位置で決め打ちしない)。名前が見つからなければ `ValueError`(どの名前が無いかをメッセージに含める)。
- 🟡 シーンファイルが無い・読めない場合は MuJoCo の例外をそのまま上げず、パスを含む分かりやすいエラー(`FileNotFoundError` または `ValueError`)にする。
- 🟡 初期姿勢はモデルの既定 qpos(ゼロ姿勢)。`timestamp` は 0.0 から始まる。

**`get_state() -> RobotState`**

- 🟡 現在の `qpos`・`qvel` から、`JOINT_NAMES` の順に取り出したコピーを返す。MjData の配列(やそのビュー)を外に出さない。
- 🟡 呼ぶだけではシミュレーションを進めない(副作用なし)。

**`send_command(q_target: np.ndarray) -> None`**

- 🟡 入力検証: `np.asarray(q_target, dtype=float)` に変換し、形状が `(6,)` でなければ `ValueError`。NaN・inf を含めば `ValueError`。検証に失敗したら `ctrl` も時刻も変えない。
- 🔵 `ctrl` に位置目標を書き込み(`JOINT_NAMES` 順 → アクチュエータ番号に対応付け)、`mj_step` を `n_substeps` 回進める(design.md 4.1)。
- 🟡 範囲のクリップはしない。可動域・速度の制限は安全フィルタ(TASK-0010)の責務。
- 🟡 `ctrlrange` 外の値: このモデルは全アクチュエータが `ctrllimited=true` なので、MuJoCo が力の計算時に内部でクランプする。`data.ctrl` に書いた値自体は変わらない(実測: ctrl=5.0 を入れると shoulder_pan は上限 1.91986 rad で止まり、`data.ctrl` は 5.0 のまま)。この挙動に依存する保証はせず、docstring に記載するのみとする。
- 🟡 呼び出し側の配列を変更しない(内部でコピーする)。

**`close() -> None`**

- 🟡 何度呼んでもよい(冪等)。2 回目以降は何もしない。
- 🟡 close 後に `get_state()`・`send_command()` を呼ぶと `RuntimeError`("closed" を含むメッセージ)。
- 🔴 `with` 文で使えるように `__enter__`/`__exit__` を付ける(exit で close)。Protocol には含めない。

### 2.4 データフロー 🟡

```
上位(Policy / 安全フィルタ) --q_target[rad](6,)--> SimRobot.send_command
    -> 検証 -> data.ctrl[actuator_ids] = q_target -> mj_step × n_substeps
上位 <--RobotState(q, dq, timestamp)-- SimRobot.get_state <- data.qpos/qvel のコピー
```

---

## 3. 制約条件

- 🔵 **単位**: ラジアン・ラジアン毎秒に統一(design.md 4.1)。
- 🔵 **関節順**: `JOINT_NAMES` が唯一の定義。sim と実機で同じ順序。
- 🟡 **不変性**: `RobotState` は frozen、配列は読み取り専用のコピー。MjData の配列を直接返さない。
- 🟡 **安全(NFR-02)との関係**: `SimRobot` は形状と有限値のみ検証する。範囲外指令の拒否・制限は TASK-0010。実機の `So101Robot` では必ず安全フィルタを通す前提。
- 🟡 **依存**: `mujoco`、`numpy` のみ。外部入力・秘密情報なし。
- 🟡 **ファイル規模**: 各ファイル 200 行程度以内、関数 50 行未満。
- 🟡 **テスト時間**: 実 MuJoCo を使う。1 テストで進めるステップ数は 1000 以下を目安にする。

---

## 4. 想定される使用例

### 4.1 基本パターン 🔵

```python
robot = SimRobot()
robot.send_command(np.array([0.3, 0.3, 0.5, -0.3, 0.5, 0.5]))
state = robot.get_state()   # q は目標に近づく
robot.close()
```

### 4.2 収束の基準(実測に基づく提案。testcases で確定する) 🟡

本ステップで `scene.xml` を使って実測した(重力あり、kp=998.22、kv=2.731、timestep 0.002 s)。

| 目標 [rad] (pan, lift, elbow, wflex, wroll, grip) | 100 step | 200 step | 300 step | 500 step 以降 |
|---|---|---|---|---|
| (0.3, 0.3, 0.5, -0.3, 0.5, 0.5) | 0.078 | 0.0007 | 0.0005 | 0.0005 |
| (0.0, 1.0, -1.0, 0.0, 0.0, 0.0) (shoulder_lift に重力負荷) | 0.216 | 0.025 | 0.0007 | 0.0008 |
| (0.5, -0.5, 0.8, 0.5, -1.0, 1.0) | - | - | - | 0.0004 |

(値は全関節の最大絶対誤差)

- 定常誤差は重力負荷のある shoulder_lift でも 1e-3 rad 未満。0.3 s 以内に整定する。
- **提案**: 許容誤差 `TOL_RAD = 0.01`、整定時間 1.0 s(500 step。`n_substeps=10` なら `send_command` 50 回)。余裕は約 3 倍以上。
- 注意: 全関節を可動域の端近く(例: 1.5, 1.5, 1.5, 1.5, 2.5, 1.5)にすると自己干渉で到達しない(誤差 1.2 rad 以上が残る)。収束テストの目標は可動域の中ほど(|q| ≤ 1.0 rad 程度)から選ぶ。
- TASK-0003 の verify-report にある shoulder_lift の「到達が小さい」は掃引(連続的に動く目標)での追従遅れであり、静止目標の定常誤差は小さい。

### 4.3 エッジケース・エラーケース

| ケース | 期待動作 | 信頼性 |
|---|---|---|
| 長さ 5 の配列 | `ValueError`、状態・時刻は変わらない | 🟡 |
| 形状 (1, 6) や (7,) | `ValueError` | 🟡 |
| NaN / inf を含む | `ValueError` | 🟡 |
| list で渡す | 受け付ける(`np.asarray` で変換) | 🔴 |
| ctrlrange 外の値 | 拒否しない。MuJoCo が内部でクランプし、関節は可動域内に留まる | 🟡 |
| 返った `q` に書き込む | numpy が `ValueError`(読み取り専用)。次の `get_state()` に影響しない | 🟡 |
| 呼び出し側の q_target を後で書き換える | ロボットの指令には影響しない | 🟡 |
| `send_command` 後に `timestamp` を見る | `n_substeps × timestep` だけ進む | 🟡 |
| `close()` を 2 回 | 例外なし | 🟡 |
| close 後に操作 | `RuntimeError` | 🟡 |
| `n_substeps=0` | `ValueError` | 🔴 |
| `SimRobot` が `Robot` を満たす | `isinstance(robot, Robot)` が True | 🟡 |
| `base.JOINT_NAMES is model_paths.JOINT_NAMES` | 引き続き True | 🔵 |

---

## 5. 要件・設計との対応関係

- **機能要件**: FR-08(Robot 抽象と SimRobot/So101Robot の切り替え)
- **非機能要件**: NFR-02(範囲外指令の制限。本タスクでは扱わず TASK-0010 へ)、NFR-05(単体テスト)
- **受け入れ基準(タスクファイル)**: Protocol と RobotState の定義、指令で関節が目標に近づく、単体テストが通る
- **設計文書**: `docs/design.md` 4.1(Robot 抽象)、1 章(位置制御アクチュエータ)
- **モデル**: `assets/so101/so101_new_calib.xml` の actuator(156〜162 行、position 型 6 本、関節名と同名、class sts3215: kp=998.22, kv=2.731)

---

## 6. 本ステップで置いた仮定(ユーザー確認なしで決めたもの)

1. 🔴 `RobotState.timestamp` を追加する(シミュレーション時刻)。
2. 🔴 `n_substeps` の既定値は 10(20 ms)。
3. 🔴 `SimRobot` に context manager(`with`)を付ける。
4. 🔴 list など配列以外の入力も `np.asarray` で受け付ける。
5. 🟡 `Robot` は `runtime_checkable` な Protocol。
6. 🟡 シーン読み込み失敗・名前が見つからない場合は、パスや名前を含むエラーにする。
7. 🟡 収束テストの許容誤差 0.01 rad、整定時間 1.0 s(500 step)、目標は |q| ≤ 1.0 rad 程度。testcases で確定する。
8. 🟡 範囲外指令はクリップも拒否もしない(TASK-0010 の責務)。

## 7. 信頼性レベルの分布

- 🔵 青: 9 項目
- 🟡 黄: 33 項目
- 🔴 赤: 7 項目

## 8. 品質判定

- 要件の曖昧さ: ほぼなし(収束の数値は実測で裏付け、testcases で確定)
- 入出力定義: 完全(型・形状・単位・エラーを定義)
- 制約条件: 明確(クリップしない、不変性、関節順)
- 実装可能性: 確実(既存モデルで実測済み)
- 信頼性: 🟡 が多い(設計 4.1 の記述が短いため)

**判定**: ⚠️ 要改善(内容は実装可能だが、🟡🔴 が多い。🔴 の仮定 1〜4 はレビューで確認するとよい)

次のお勧めステップ: `/tsumiki:tdd-testcases so101-vlm-pickplace TASK-0004` でテストケースの洗い出しを行います。

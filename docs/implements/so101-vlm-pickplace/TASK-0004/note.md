# TASK-0004 開発コンテキストノート: Robot 抽象と SimRobot

**要件名**: so101-vlm-pickplace
**タスクID**: TASK-0004(Robot 抽象と SimRobot、TDD、2h)
**作成日**: 2026-10-18
**作成方法**: tsumiki:tdd-tasknote(Explore サブエージェントは使わず、直接読み込みで収集)

> 注: このプロジェクトは tsumiki の標準配置(`docs/spec/{要件名}-requirements.md` など)を使わない。要件は `docs/requirements.md`、設計は `docs/design.md`、共通のコンテキストは `docs/spec/so101-vlm-pickplace/note.md` を参照する。
> 注: `AGENTS.md`、`docs/rule/` は存在しない。追加ルールは `docs/spec/so101-vlm-pickplace/note.md` の「開発ルール」と、ユーザーのグローバルルール(`~/.claude/rules/ecc/`)に従う。

---

## 1. 技術スタック

- 言語: Python 3.12(`requires-python = ">=3.12,<3.13"`)
- パッケージ管理: uv(`uv sync`、`uv run pytest`)
- シミュレータ: MuJoCo(`mujoco`、`numpy`)
- テスト: pytest、pytest-cov(カバレッジ 80% 以上)
- ビルド: hatchling(`src/so101_vlm` を wheel に含める)
- 参照元: `pyproject.toml`, `docs/spec/so101-vlm-pickplace/note.md`

### アーキテクチャ(設計どおり)

- `Robot` は Protocol(構造的部分型)。実装は `SimRobot`(TASK-0004)と、後で `So101Robot`(TASK-0034、LeRobot ドライバのラップ)。
- 単位はラジアンに統一する。LeRobot 側の単位変換は `So101Robot` 内に閉じ込める。
- 関節順は `JOINT_NAMES`(`src/so101_vlm/robot/model_paths.py` が唯一の定義)。`base.py` はこれを再エクスポートする。
- 参照元: `docs/design.md`(4.1 Robot 抽象)、`src/so101_vlm/robot/base.py`、`src/so101_vlm/robot/model_paths.py`

---

## 2. 開発ルール

- TDD: テストを先に書く(RED → GREEN → REFACTOR)。カバレッジ 80% 以上。
- 不変性: データは frozen な dataclass とする。配列は書き換えず、コピーして返す。
- 小さいファイル(200〜400 行目安、最大 800 行)、小さい関数(50 行未満)。
- エラーは握りつぶさず、ユーザーが対処できるメッセージにする。
- 秘密情報をコードに書かない(このタスクには該当なし)。
- コミットは conventional commits(今回のステップではコミットしない)。
- 実装は TASK-0004 の範囲に限る。安全フィルタ(可動域・速度の制限)は TASK-0010 の責務。
- 参照元: `docs/spec/so101-vlm-pickplace/note.md`, `docs/implements/so101-vlm-pickplace/TASK-0003/verify-report.md`(設計どおり実装しているか確認)

---

## 3. 関連実装

既存コード(TASK-0001〜0003 で作成済み):

- `src/so101_vlm/robot/model_paths.py`: `SO101_SCENE_PATH`、`JOINT_NAMES`(6 関節、順序は shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper)、`SITE_NAMES`(baseframe, gripperframe)
- `src/so101_vlm/robot/base.py`: 現状は `JOINT_NAMES` の再エクスポートのみ。docstring に「Robot Protocol は TASK-0004 で追加する」とある。**ここに `Robot` Protocol と `RobotState` を追加する。**
- `src/so101_vlm/robot/sweep.py`: `sweep_targets(ctrlrange, t, ...)`。純粋関数の書き方の参考(入力を変更せず新しい配列を返す、引数を検証して `ValueError`)。
- `scripts/view_so101.py`: MuJoCo でモデルを読み、`data.ctrl` に目標を入れて `mj_step` する既存の駆動ループ。`SimRobot` の参考になる(ただし TASK-0004 では scripts は変更しない)。
- `scripts/fetch_so101_model.py`: モデル取得スクリプト(変更しない)。

実装パターン(既存コードに合わせる):
- 純粋関数と、MuJoCo に触る部分を分ける。
- `_load()` のようなモデル読み込み関数は `SO101_SCENE_PATH` から `mujoco.MjModel.from_xml_path(str(...))` で読む。

参照元: `src/so101_vlm/robot/base.py`, `src/so101_vlm/robot/model_paths.py`, `src/so101_vlm/robot/sweep.py`, `scripts/view_so101.py`

---

## 4. 設計文書

### Robot Protocol(design.md 4.1 より)

```python
class Robot(Protocol):
    joint_names: tuple[str, ...]          # 6 関節の順序を固定
    def get_state(self) -> RobotState: ...   # 関節角[rad], 速度
    def send_command(self, q_target: np.ndarray) -> None: ...  # 位置目標[rad]
    def close(self) -> None: ...
```

- 値の単位は両実装でラジアンに統一する。
- `SimRobot`: MuJoCo の `ctrl` に位置目標を入れてステップを進める。

### 関連する要件(docs/requirements.md)

- FR-08: `Robot` 抽象インターフェース(関節指令の送信と状態取得)の背後に実装を隠し、`SimRobot` と `So101Robot` を設定で切り替えられること。
- NFR-02: 関節角・速度に上限を設け、実機では範囲外の指令を送らない(実機移行時の前提)。

### シーン(MuJoCo モデル)の確認結果

- ファイル: `assets/so101/scene.xml`(本体: `assets/so101/so101_new_calib.xml`)
- アクチュエータは position 型で 6 本。名前は関節名と同じ。`ctrlrange` は次のとおり(ラジアン)。
  - shoulder_pan: -1.91986 〜 1.91986
  - shoulder_lift: -1.74533 〜 1.74533
  - elbow_flex: -1.69 〜 1.69
  - wrist_flex: -1.65806 〜 1.65806
  - wrist_roll: -2.74385 〜 2.84121
  - gripper: -0.17453 〜 1.74533
- 関節の `range` は ctrlrange とほぼ一致する。
- 参照元: `assets/so101/so101_new_calib.xml`(156〜162 行目が actuator)、`tests/test_assets.py`

### 関連タスクとの接続

- 前提: TASK-0003(MuJoCo で SO-101 を表示・駆動、完了)
- 後続: TASK-0010(安全フィルタ)、TASK-0013(固定位置の pick & place)、TASK-0034(So101Robot)
- 参照元: `docs/tasks/so101-vlm-pickplace/TASK-0004.md`, `docs/tasks/so101-vlm-pickplace/overview.md`

---

## 5. テスト関連情報

### テストフレームワーク・設定

- pytest 設定: `pyproject.toml` の `[tool.pytest.ini_options]`(`testpaths = ["tests"]`)
- カバレッジ設定: `pyproject.toml` の `[tool.coverage.run]`(`source = ["so101_vlm"]`)
- 実行: `uv run pytest -q`(現状 18 passed)、`uv run pytest --cov`

### 既存テストの構成と命名

- `tests/test_assets.py`: シーンの読み込み、関節名、アクチュエータ名、サイト名の確認。`mujoco.MjModel.from_xml_path` を `scope="module"` の fixture で読む。
- `tests/test_package.py`: パッケージ読み込みの確認(内容は未読)。
- `tests/robot/test_sweep.py`: 純粋関数のテスト。モジュール定数 `CTRL`、`TIMES` を使う。関数名は `test_<内容>` 形式。
- 配置方針: `tests/robot/` 配下に、対象モジュールと同じ名前で置く。TASK-0004 のテストは `tests/robot/test_sim.py`。

### テストユーティリティ・モック設定

- 専用のモックは無い。実 MuJoCo モデルを使う(モックしない)。
- 既存テストは `pytest.fixture(scope="module")` でモデルを読み込んで共有している。
- `pytest.mark.parametrize` を使う(`tests/test_assets.py`)。

### E2E テスト

- 該当なし(このプロジェクトに Web 画面はない。UI 要件もない)。

### 注意: 既存テストが保証していること

- `tests/robot/test_sweep.py::test_joint_names_single_source` は `base.JOINT_NAMES is model_paths.JOINT_NAMES` を確認している。`base.py` に Protocol を追加しても、この再エクスポートは壊さないこと。

参照元: `pyproject.toml`, `tests/test_assets.py`, `tests/robot/test_sweep.py`

---

## 6. 注意事項

### TASK-0004 の完了条件(docs/tasks/so101-vlm-pickplace/TASK-0004.md)

- [ ] `Robot` Protocol と `RobotState` が定義されている
- [ ] `SimRobot` で指令を送ると、関節が目標に近づく
- [ ] 単体テストが通る

### 実装ファイル(タスクファイルの指定)

- `src/so101_vlm/robot/base.py`: `Robot` Protocol、`RobotState`(frozen dataclass)
- `src/so101_vlm/robot/sim.py`: `SimRobot`(新規)
- `tests/robot/test_sim.py`: 単体テスト(新規)

### テストケース(タスクファイルより)

1. 指令した角度に収束する(信頼性 🔵): 可動域内の目標を送り、一定時間ステップを進める。各関節が許容誤差内で目標に一致する。
2. 配列の形が違うとエラー(🟡): 長さ 5 の配列を送ると `ValueError`。
3. 状態はコピーで返る(🟡): `get_state()` で得た配列を書き換えても、次の `get_state()` に影響しない。
- 統合テスト(🟡): TASK-0005 のシーンを読み込んだ SimRobot で姿勢を変える。現時点では TASK-0005 が未実装のため、TASK-0004 では現在の `scene.xml` で代替する。

### 技術的制約・既知の事実

- **重力による追従遅れ**: `docs/implements/so101-vlm-pickplace/TASK-0003/verify-report.md` によると、shoulder_lift は重力負荷のため、指令に対する到達が小さい(最大 0.664 rad、指令の範囲 ±1.745 rad に対して遅れる)。収束テストは十分な整定時間を取り、許容誤差を実測で決めること。
- 指令は 1 関節だけでなく 6 関節すべてに入る。位置アクチュエータは `kp` で追従するため、整定までのステップ数を実測する。
- `SimRobot` は位置目標を MuJoCo の `ctrl` に入れてから `mj_step` を進める(design.md 4.1)。`data.ctrl` への入力順は `JOINT_NAMES` の順と一致させる。
- 関節の qpos/qvel の取り出しは、名前で引く(`mujoco.mj_name2id` と `model.jnt_qposadr`)のが安全。インデックスを直書きしない。
- 単位はラジアン。`get_state()` が返す関節角と速度もラジアン(速度は rad/s)。

### 未確定事項と暫定判断(テストの RED 段階で確定させる)

- **許容誤差の値**: 未定。実測して決める(例: 数百ステップ後に 0.02 rad 程度)。定数として名前を付ける。
- **整定時間(ステップ数)**: 未定。`timestep` と重力影響から実測する。
- **範囲外の目標の扱い**: 暫定では `SimRobot` は形状のみを検証し、範囲のクリップや拒否は行わない。範囲の制限は TASK-0010(安全フィルタ)の責務とする。NFR-02 との関係は TASK-0010 で扱う。
- **`get_state()` の速度**: `data.qvel` の関節速度を `JOINT_NAMES` の順で返す。
- **初期姿勢**: タスクでは「初期姿勢の SimRobot」とのみ記載。暫定はモデルの既定の qpos(ゼロ姿勢)とする。

### セキュリティ・パフォーマンス

- 外部入力はない(API キーなし、ネットワークなし)。
- テストは実 MuJoCo を使うため、ステップ数が多いと遅くなる。テストは必要最小限のステップ数にする。

### 参照元(まとめ)

- `docs/tasks/so101-vlm-pickplace/TASK-0004.md`
- `docs/requirements.md`(FR-08、NFR-02)
- `docs/design.md`(4.1)
- `docs/spec/so101-vlm-pickplace/note.md`
- `docs/implements/so101-vlm-pickplace/TASK-0003/verify-report.md`
- `src/so101_vlm/robot/base.py`, `src/so101_vlm/robot/model_paths.py`, `src/so101_vlm/robot/sweep.py`
- `scripts/view_so101.py`
- `assets/so101/so101_new_calib.xml`
- `tests/test_assets.py`, `tests/robot/test_sweep.py`
- `pyproject.toml`
- Refactor 完了: 2026-10-18

# TASK-0004 テストケース定義書: Robot 抽象と SimRobot

**機能名**: sim-robot(Robot Protocol / RobotState / SimRobot)
**タスクID**: TASK-0004
**要件名**: so101-vlm-pickplace
**作成日**: 2026-10-18
**出力ファイル**: `docs/implements/so101-vlm-pickplace/TASK-0004/sim-robot-testcases.md`
**テストファイル**: `tests/robot/test_sim.py`
**参照**: `docs/implements/so101-vlm-pickplace/TASK-0004/sim-robot-requirements.md`(以下「要件」)、`docs/implements/so101-vlm-pickplace/TASK-0004/note.md`、`docs/tasks/so101-vlm-pickplace/TASK-0004.md`

> 信頼性: 🔵 要件・設計・実測にほぼそのまま基づく / 🟡 要件から妥当に推測 / 🔴 要件に無い推測(本ステップで決めたもの)
> 本ステップではユーザーへの質問をせず、未確定の数値と方針は既定値を選んで「0. 確定事項」に記録した。

---

## 0. 本ステップで確定した数値と方針

### 0.1 収束の基準 🔵(要件 4.2 と本ステップの再測定)

| 定数名(テスト内) | 値 | 根拠 |
|---|---|---|
| `TOL_RAD` | `0.01` rad | 実測の定常誤差は最大 8.4e-4 rad。10 倍以上の余裕がある |
| `SETTLE_TIME_S` | `1.0` s | 実測では 0.3 s 以内に整定。約 3 倍の余裕 |
| `SIM_TIMESTEP_S` | `0.002` s | `assets/so101/scene.xml` の `opt.timestep`(実測) |
| `N_SUBSTEPS_DEFAULT` | `10` | 要件 2.3 の既定値 |
| `N_SETTLE_COMMANDS` | `50` | `SETTLE_TIME_S / (N_SUBSTEPS_DEFAULT * SIM_TIMESTEP_S)` で 500 step |

### 0.2 収束テストの目標ベクトル(すべて |q| ≤ 1.0 rad で、ctrlrange の内側) 🔵

関節順: (shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper)

| 名前 | 目標 [rad] | 選んだ理由 | 1.0 s 後の最大誤差(実測) |
|---|---|---|---|
| `TARGET_MIXED` | (0.3, 0.3, 0.5, -0.3, 0.5, 0.5) | 6 関節すべてを動かす代表的な姿勢。正負の両方向を含む(要件 4.1 の例) | 5.5e-4 |
| `TARGET_GRAVITY` | (0.0, 1.0, -1.0, 0.0, 0.0, 0.0) | 重力負荷が最も大きい shoulder_lift を上限 1.0 まで動かす。最も整定が遅い(200 step 時点で 0.025)。TASK-0003 の追従遅れの懸念を確認する | 8.4e-4 |
| `TARGET_OPPOSITE` | (-0.5, -0.5, 0.8, 0.5, -1.0, 1.0) | `TARGET_MIXED` と逆向きの符号。wrist_roll -1.0、gripper 1.0 で、|q| = 1.0 の境界を含む | 4.1e-4 |
| `TARGET_DISTINCT` | (0.1, 0.2, 0.3, 0.4, 0.5, 0.6) | 全要素が異なる。関節の取り違えがあれば必ず誤差 0.1 以上になる(関節順の確認に使う) | 5.4e-4 |

- 端に近い目標(例: 1.5, 1.5, 1.5, 1.5, 2.5, 1.5)は自己干渉で到達しない(要件 4.2)。このため収束テストでは使わない。
- 整定後の速度は全関節で |dq| < 1e-8 rad/s(実測)。テストでは `DQ_SETTLED = 0.05` rad/s 未満を確認する 🟡。

### 0.3 テストの速度対策(ファイル全体で約 2 s 以内) 🟡

- 実シーン(`SO101_SCENE_PATH`)の読み込みには 1 回 0.15〜0.25 s かかる(実測)。実シーンで `SimRobot` を作るテストは **7 回以内** にする。
- 指令の流れ・時刻・close・context manager・不変性など、物理の正しさに依らないテストは、`tmp_path_factory` で作る **小さな MJCF(以下「小モデル」)** を使う。読み込みは約 2 ms(実測)。
- 小モデルの仕様(関節順の確認を兼ねる):
  - 先頭に余分な slide 関節 `extra` を置き、qpos のアドレスをずらす。
  - 6 つの hinge 関節(名前は `JOINT_NAMES`)を **逆順** に宣言する。
  - position アクチュエータ(名前は関節名と同じ、kp=50、kv=1、ctrlrange -3〜3)を **(elbow_flex, wrist_flex, wrist_roll, gripper, shoulder_pan, shoulder_lift) の順** に宣言する。
  - 重力 0、timestep 0.002。
  - 実測: アクチュエータ番号 [4, 5, 0, 1, 2, 3]、qpos アドレス [6, 5, 4, 3, 2, 1]。`TARGET_DISTINCT` を 500 step で誤差 8e-12 に収束する。
- 実シーンを読み込む fixture:
  - `shared_robot`(`scope="module"`): 状態を進めない検証用。入力検証、読み取り専用、Protocol 適合のテストで共有する。
  - `fresh_robot`(関数スコープ): 収束、初期状態、範囲外指令のテストで使う。
- `SimRobot` の時刻は浮動小数の積み上げで誤差が出る(500 step で 1.0000000000000007)。時刻の比較は `pytest.approx(abs=1e-9)` を使う。

### 0.4 方針の確定(要件の 🔴 仮定をテストで固定する)

| 項目 | 確定内容 | 信頼性 |
|---|---|---|
| `RobotState` のフィールド | `q`、`dq`、`timestamp` | 🔴(要件 6-1) |
| `n_substeps` の既定値 | 10 | 🔴(要件 6-2) |
| context manager | `__enter__` は self を返す。`__exit__` は close して例外を握りつぶさない | 🔴(要件 6-3) |
| list 入力 | 受け付ける(int の list も可) | 🔴(要件 6-4) |
| シーンファイルが無い | `FileNotFoundError`、メッセージにパスを含む | 🟡 |
| シーンが壊れている(XML 不正) | `ValueError`、メッセージにパスを含む | 🟡 |
| 関節名・アクチュエータ名が無い | `ValueError`、メッセージに無い名前を含む | 🟡 |
| `n_substeps < 1` | `ValueError`(メッセージに `n_substeps` を含む) | 🔴 |
| close 後の操作 | `RuntimeError`、メッセージに `closed` を含む | 🟡 |
| ctrlrange 外の指令 | `SimRobot` は拒否もクリップもしない。MuJoCo の ctrllimited による内部クランプに任せる | 🟡 |

---

## 1. 正常系テストケース

### TC-N01: 指令した角度に収束する(3 つの目標で parametrize)

- **テスト名**: `test_converges_to_target[TARGET_MIXED|TARGET_GRAVITY|TARGET_OPPOSITE]`
  - **何をテストするか**: 可動域の中ほどの目標を送り続けると、全関節が目標に一致する。
  - **期待される動作**: `send_command(target)` を `N_SETTLE_COMMANDS`(50)回呼ぶと、`ctrl` に目標が入って 500 step 進む。
- **入力値**: 0.2 の 3 つの目標ベクトル。`fresh_robot`(実シーン、初期ゼロ姿勢)
  - **入力データの意味**: 代表的な姿勢、重力負荷が最大の姿勢、逆符号で |q|=1.0 を含む姿勢。
- **期待される結果**: `np.max(np.abs(state.q - target)) < TOL_RAD`(0.01)、かつ `np.max(np.abs(state.dq)) < DQ_SETTLED`(0.05)。
  - **期待結果の理由**: 実測の最大誤差は 8.4e-4、整定後の速度は 1e-8 未満。
- **テストの目的**: タスクの完了条件「指令を送ると関節が目標に近づく」を確認する。
  - **確認ポイント**: shoulder_lift の重力負荷でも収束すること。速度も 0 付近に落ちること。
- **対応要件**: 要件 2.3 send_command、4.2 収束の基準、タスクのテストケース 1
- 🔵

### TC-N02: 初期状態の形状・型・値

- **テスト名**: `test_initial_state_shape_dtype_and_zero_pose`
  - **何をテストするか**: 作った直後の `get_state()` が仕様どおりの `RobotState` を返す。
  - **期待される動作**: 既定の qpos(ゼロ姿勢)、速度 0、時刻 0 を返す。
- **入力値**: `fresh_robot`(実シーン)。指令は送らない。
  - **入力データの意味**: 最初の状態が決まっていることを確かめる。
- **期待される結果**:
  - `isinstance(state, RobotState)`
  - `state.q.shape == (6,)`、`state.dq.shape == (6,)`
  - `state.q.dtype == np.float64`、`state.dq.dtype == np.float64`
  - `np.allclose(state.q, 0.0)`、`np.allclose(state.dq, 0.0)`
  - `state.timestamp == 0.0`、`isinstance(state.timestamp, float)`
  - **期待結果の理由**: 要件 2.2 の表、2.3「初期姿勢はゼロ姿勢、timestamp は 0.0 から」。
- **テストの目的**: 出力の型と形状の契約を固定する。
  - **確認ポイント**: dtype が float64 であること(float32 や object にならない)。
- **対応要件**: 要件 2.2、2.3
- 🟡

### TC-N03: joint_names が JOINT_NAMES と一致する

- **テスト名**: `test_joint_names_match_model_paths`
  - **何をテストするか**: `robot.joint_names` が唯一の定義 `JOINT_NAMES` と同じ順序のタプルである。
  - **期待される動作**: タプルで、要素と順序が一致する。
- **入力値**: `shared_robot`
  - **入力データの意味**: 実シーンでの通常の構成。
- **期待される結果**: `isinstance(robot.joint_names, tuple)` かつ `robot.joint_names == JOINT_NAMES`。
  - **期待結果の理由**: 要件 2.1「joint_names は JOINT_NAMES と同じ順序のタプル」。
- **テストの目的**: 関節順の契約を確認する。
  - **確認ポイント**: list ではなく tuple であること(変更不可)。
- **対応要件**: 要件 2.1、3 章「関節順」
- 🔵

### TC-N04: 関節の順序が名前で対応付けられている(実シーン)

- **テスト名**: `test_joint_order_distinct_target_on_real_scene`
  - **何をテストするか**: `q_target[i]` が `joint_names[i]` の関節に届き、`state.q[i]` が同じ関節の角度である。
  - **期待される動作**: 要素がすべて異なる目標で収束し、要素ごとに一致する。
- **入力値**: `TARGET_DISTINCT` = (0.1, 0.2, 0.3, 0.4, 0.5, 0.6) を 50 回。`fresh_robot`
  - **入力データの意味**: 取り違えがあれば少なくとも 0.1 rad ずれる。
- **期待される結果**: 各 i で `abs(state.q[i] - TARGET_DISTINCT[i]) < TOL_RAD`。
  - **期待結果の理由**: 実測の最大誤差 5.4e-4。
- **テストの目的**: 指令と状態の要素順の一致を確認する。
  - **確認ポイント**: 実シーンはアクチュエータ番号と qpos アドレスがどちらも 0〜5 で一致するため、このテストだけでは直書きのインデックスを検出できない。TC-N05 で補う。
- **対応要件**: 要件 2.1、2.3(`mj_name2id` で引く)
- 🔵

### TC-N05: 宣言順が違うモデルでも名前で対応付ける(小モデル)

- **テスト名**: `test_joint_order_resolved_by_name_on_permuted_model`
  - **何をテストするか**: アクチュエータ・関節の宣言順と qpos アドレスが `JOINT_NAMES` とずれていても、名前で正しく対応付ける。
  - **期待される動作**: 小モデル(0.3 節)で `TARGET_DISTINCT` に収束する。
- **入力値**: `SimRobot(scene_path=小モデル)`、`TARGET_DISTINCT` を 50 回。
  - **入力データの意味**: アクチュエータ番号 [4,5,0,1,2,3]、qpos アドレス [6,5,4,3,2,1] で、インデックスの直書きは必ず失敗する。
- **期待される結果**: `np.max(np.abs(state.q - TARGET_DISTINCT)) < TOL_RAD`。
  - **期待結果の理由**: 実測で誤差 8e-12。
- **テストの目的**: 将来のシーン(TASK-0005 でブロックの free joint が先頭に入るなど)でも壊れないことを保証する。
  - **確認ポイント**: qpos(`jnt_qposadr`)と qvel(`jnt_dofadr`)の両方を名前で引くこと。free joint があると qpos と qvel のアドレスがずれるため。
- **対応要件**: 要件 2.3(名前で引く。位置で決め打ちしない)、note 6 章
- 🟡

### TC-N06: timestamp が n_substeps × timestep だけ進む

- **テスト名**: `test_timestamp_advances_by_substeps[n_substeps=10|3]`
  - **何をテストするか**: `send_command` 1 回で `n_substeps` 回だけ `mj_step` を進める。
  - **期待される動作**: k 回呼ぶと timestamp が `k * n_substeps * 0.002` になる。
- **入力値**: 小モデル、`n_substeps` = 既定(引数なし、10)と 3。ゼロ指令を k = 1, 2, 3 回。
  - **入力データの意味**: 既定値の確認と、引数が効いていることの確認。
- **期待される結果**: 既定: 0.02、0.04、0.06。`n_substeps=3`: 0.006、0.012、0.018(`pytest.approx(abs=1e-9)`)。
  - **期待結果の理由**: 要件 2.3「mj_step を n_substeps 回」、既定値 10。
- **テストの目的**: 制御周期(既定 20 ms)の契約を確認する。
  - **確認ポイント**: 既定値が 10 であること。浮動小数の誤差で完全一致を求めないこと。
- **対応要件**: 要件 2.2(timestamp)、2.3(n_substeps)、4.3「timestamp」
- 🟡(既定値 10 は 🔴)

### TC-N07: list 入力を受け付ける

- **テスト名**: `test_send_command_accepts_list[float_list|int_list]`
  - **何をテストするか**: `np.ndarray` 以外の数値列も受け付ける。
  - **期待される動作**: `np.asarray(..., dtype=float)` で変換され、エラーにならない。
- **入力値**: 小モデル。`[0.1, 0.2, 0.3, 0.4, 0.5, 0.6]` を 50 回、および `[0, 0, 0, 0, 0, 0]`(int)を 1 回。
  - **入力データの意味**: 上位コードや REPL から手で渡す場面。
- **期待される結果**: 例外が出ない。float の list は `TARGET_DISTINCT` に `TOL_RAD` 以内で収束する。int の list は timestamp が 0.02 進む。
  - **期待結果の理由**: 要件 2.3「`np.asarray(q_target, dtype=float)` に変換」、4.3「list で渡す」。
- **テストの目的**: 入力型の許容範囲を固定する。
  - **確認ポイント**: int を float に変換すること(int 配列のまま `ctrl` に入れて切り捨てないこと)。
- **対応要件**: 要件 2.3、4.3、6-4
- 🔴

### TC-N08: 動き出しの速度の符号と単位(rad/s)

- **テスト名**: `test_velocity_sign_follows_motion`
  - **何をテストするか**: `dq` が関節速度(rad/s)で、動く向きと符号が一致する。
  - **期待される動作**: 正の目標へ動き出した直後は `dq > 0`、負の目標なら `dq < 0`。
- **入力値**: `fresh_robot`(実シーン)、`TARGET_OPPOSITE` を 1 回(10 step)。
  - **入力データの意味**: 実測では 10 step 後の dq は (-1.32, -1.57, 1.97, 1.89, -1.88, 1.88)。すべて目標の符号と一致する。
- **期待される結果**: `np.all(np.sign(state.dq) == np.sign(TARGET_OPPOSITE))`。さらに `0.1 < abs(dq) < 10`(rad/s のオーダーで、deg/s ではないこと)。
  - **期待結果の理由**: 要件 2.2「dq: rad/s」。deg/s なら 50 倍以上の値になる。
- **テストの目的**: 速度の要素順・符号・単位を確認する。
  - **確認ポイント**: 実シーンの fresh 生成数を増やさないため、TC-N01 の `TARGET_OPPOSITE` のケースで 1 回目の指令の直後に確認してもよい(実装時に統合可)。
- **対応要件**: 要件 2.2、3 章「単位」
- 🟡

---

## 2. 異常系テストケース

> 入力検証の失敗では、`ctrl` も時刻も変えてはいけない(要件 2.3)。各異常系では、呼び出し前後の `timestamp` と `q` が変わらないことも確認する。

### TC-E01: 配列の長さ・形が違う

- **テスト名**: `test_send_command_rejects_wrong_shape[len5|len7]`
  - **エラーケースの概要**: 関節数と合わない配列。
  - **エラー処理の重要性**: 黙って一部の関節だけに入ると、実機では予期しない動きになる。
- **入力値**: `np.zeros(5)`、`np.zeros(7)`。`shared_robot`
  - **不正な理由**: 形状が `(6,)` ではない。
  - **実際の発生シナリオ**: グリッパを除いた 5 関節の配列を誤って渡す。
- **期待される結果**: `pytest.raises(ValueError, match="shape")`。メッセージに期待形状 `(6,)` と実際の形状を含む。呼び出し前後で `timestamp` と `q` が同じ。
  - **エラーメッセージの内容**: 例「q_target must have shape (6,), got (5,)」。
  - **システムの安全性**: 状態を変えない。
- **テストの目的**: タスクのテストケース 2 を確認する。
  - **品質保証の観点**: 誤った指令が物理に届かないこと。
- **対応要件**: 要件 2.3、4.3、タスクのテストケース 2
- 🔵(長さ 5 はタスクに明記。長さ 7 とメッセージ内容は 🟡)

### TC-E02: NaN・inf を含む

- **テスト名**: `test_send_command_rejects_non_finite[nan|posinf|neginf]`
  - **エラーケースの概要**: 有限でない値を含む指令。
  - **エラー処理の重要性**: NaN が `ctrl` に入ると MuJoCo のシミュレーションが発散し、以後の状態がすべて壊れる。
- **入力値**: `TARGET_MIXED` の 3 番目の要素を `np.nan`、`np.inf`、`-np.inf` に置き換えた配列。`shared_robot`
  - **不正な理由**: 位置目標として意味がない。
  - **実際の発生シナリオ**: 逆運動学や VLM 出力の変換で 0 除算が起きる。
- **期待される結果**: `pytest.raises(ValueError, match="finite")`。状態は変わらない。
  - **エラーメッセージの内容**: 有限値でないことを示す。
  - **システムの安全性**: `ctrl` を書き換えない。
- **テストの目的**: 数値の健全性の検証。
  - **品質保証の観点**: 上位のバグを早く表に出す。
- **対応要件**: 要件 2.3、4.3
- 🟡

### TC-E03: n_substeps が 1 未満

- **テスト名**: `test_rejects_invalid_n_substeps[0|-1]`
  - **エラーケースの概要**: ステップを進めない、または負の回数の設定。
  - **エラー処理の重要性**: 0 だと指令を送っても何も動かず、原因が分かりにくい。
- **入力値**: `SimRobot(scene_path=小モデル, n_substeps=0)`、`n_substeps=-1`
  - **不正な理由**: 要件 2.3「1 未満の整数なら ValueError」。
  - **実際の発生シナリオ**: 設定ファイルの誤り。
- **期待される結果**: `pytest.raises(ValueError, match="n_substeps")`。
  - **エラーメッセージの内容**: 引数名と値を含む。
  - **システムの安全性**: オブジェクトを作らない。
- **テストの目的**: コンストラクタ引数の検証。
  - **品質保証の観点**: 設定ミスを起動時に検出する。
- **対応要件**: 要件 2.3、4.3
- 🔴

### TC-E04: シーンファイルが無い

- **テスト名**: `test_missing_scene_file_raises_file_not_found`
  - **エラーケースの概要**: 存在しないパス。
  - **エラー処理の重要性**: `fetch_so101_model.py` を実行し忘れた場合に、対処方法が分かるようにする。
- **入力値**: `tmp_path / "no_such_scene.xml"`
  - **不正な理由**: ファイルが無い。
  - **実際の発生シナリオ**: 新しい環境でアセットを取得していない。
- **期待される結果**: `pytest.raises(FileNotFoundError)`。メッセージに `no_such_scene.xml` を含む。
  - **エラーメッセージの内容**: どのパスが無いかを示す。
  - **システムの安全性**: MuJoCo の汎用的な例外をそのまま出さない。
- **テストの目的**: 読み込み失敗時のエラーを確認する。
  - **品質保証の観点**: 利用者が自分で直せる。
- **対応要件**: 要件 2.3「シーンファイルが無い・読めない場合」、6-6
- 🟡

### TC-E05: シーンが壊れている(XML 不正)

- **テスト名**: `test_malformed_scene_raises_value_error`
  - **エラーケースの概要**: ファイルはあるが MJCF として読めない。
  - **エラー処理の重要性**: MuJoCo の例外型に上位が依存しないようにする。
- **入力値**: `tmp_path / "broken.xml"` に `"<mujoco><worldbody>"`(閉じタグなし)を書く。
  - **不正な理由**: XML の構文誤り。
  - **実際の発生シナリオ**: TASK-0005 でシーンを編集中に壊す。
- **期待される結果**: `pytest.raises(ValueError)`。メッセージに `broken.xml` を含む。
  - **エラーメッセージの内容**: どのファイルが読めないかを示す。
  - **システムの安全性**: 中途半端なオブジェクトを残さない。
- **テストの目的**: 読み込みエラーの型を固定する。
  - **品質保証の観点**: エラー処理を一箇所(`ValueError`)にまとめられる。
- **対応要件**: 要件 2.3、6-6
- 🟡

### TC-E06: 関節名・アクチュエータ名が無い

- **テスト名**: `test_missing_joint_name_raises_value_error[missing_joint|missing_actuator]`
  - **エラーケースの概要**: シーンに `JOINT_NAMES` のどれかが無い。
  - **エラー処理の重要性**: `mj_name2id` は -1 を返すだけで、そのまま使うと末尾の要素に誤って書き込む。
- **入力値**: 小モデルの変種を `tmp_path` に作る。(a) `gripper` 関節と、そのアクチュエータを除いたもの。(b) 関節は 6 つあるが、`gripper` のアクチュエータだけを除いたもの。
  - **不正な理由**: 要件 2.3「名前が見つからなければ ValueError」。
  - **実際の発生シナリオ**: 別バージョンのモデル(関節名の変更)を読み込む。
- **期待される結果**: `pytest.raises(ValueError, match="gripper")`。
  - **エラーメッセージの内容**: 無い名前を含む。
  - **システムの安全性**: -1 のインデックスで誤った要素に書き込まない。
- **テストの目的**: 名前解決の失敗を検出する。
  - **品質保証の観点**: モデル差し替え時の事故を防ぐ。
- **対応要件**: 要件 2.3、6-6
- 🟡

### TC-E07: close 後の操作は RuntimeError

- **テスト名**: `test_use_after_close_raises_runtime_error`
  - **エラーケースの概要**: 終了したロボットを使い続ける。
  - **エラー処理の重要性**: 実機(`So101Robot`)では切断後の指令が危険なので、同じ契約を sim で先に固める。
- **入力値**: 小モデルの `SimRobot` を `close()` してから `get_state()`、`send_command(np.zeros(6))` を呼ぶ。
  - **不正な理由**: 終了後の利用。
  - **実際の発生シナリオ**: 評価ループの後片付けの順序を誤る。
- **期待される結果**: どちらも `pytest.raises(RuntimeError, match="closed")`。
  - **エラーメッセージの内容**: 終了済みであることを示す。
  - **システムの安全性**: 解放済みのデータに触れない。
- **テストの目的**: ライフサイクルの契約を確認する。
  - **品質保証の観点**: 実機実装と同じ振る舞いを保証する土台になる。
- **対応要件**: 要件 2.3 close、4.3
- 🟡

---

## 3. 境界値テストケース

### TC-B01: close を 2 回呼んでもよい

- **テスト名**: `test_close_is_idempotent`
  - **境界値の意味**: 終了処理の 2 回目。
  - **境界値での動作保証**: `finally` と `with` の両方で close する書き方でも安全。
- **入力値**: 小モデルの `SimRobot` で `close()` を 2 回。
  - **境界値選択の根拠**: 要件 2.3「何度呼んでもよい」。
  - **実際の使用場面**: 例外処理と通常終了の両方で close する。
- **期待される結果**: 例外なし。2 回目の後も `get_state()` は `RuntimeError`(終了状態が保たれる)。
  - **境界での正確性**: 2 回目は何もしない。
  - **一貫した動作**: 1 回目の後と同じ状態。
- **テストの目的**: 冪等性の確認。
  - **堅牢性の確認**: 後片付けの順序に依存しない。
- **対応要件**: 要件 2.3、4.3
- 🟡

### TC-B02: ctrlrange 外の指令は SimRobot では拒否もクリップもしない

- **テスト名**: `test_out_of_ctrlrange_command_is_clamped_by_mujoco_not_rejected`
  - **境界値の意味**: 可動域を超える指令。範囲の制限は TASK-0010(安全フィルタ)の責務で、`SimRobot` は検証しない。
  - **境界値での動作保証**: 例外にならず、関節は MuJoCo の ctrllimited によって可動域内に留まる。
- **入力値**: `fresh_robot`(実シーン)に (5.0, 0.0, 0.0, 0.0, 0.0, -1.0) を 50 回。
  - **境界値選択の根拠**: shoulder_pan の上限 1.91986 を大きく超え、gripper の下限 -0.17453 を下回る。中間の関節は 0 なので自己干渉しない。
  - **実際の使用場面**: 安全フィルタを通さずに上位が誤った値を送る。
- **期待される結果**(実測: q[0] = 1.91986、q[5] = -0.174533):
  - 例外が出ない。
  - `state.q[0] == pytest.approx(1.91986, abs=TOL_RAD)`、`state.q[5] == pytest.approx(-0.17453, abs=TOL_RAD)`。
  - 全関節で `ctrlrange` の内側にある(上限・下限に `TOL_RAD` の余裕を含めて判定)。
  - **境界での正確性**: 可動域の端で止まる。
  - **一貫した動作**: 範囲内の指令と同じ経路で処理される(特別扱いしない)。
- **テストの目的**: 責務の分担(SimRobot は形と有限性だけを検証)を固定する。
  - **堅牢性の確認**: 範囲外でもシミュレーションが壊れない。
- **補足(任意のホワイトボックス確認)**: 「SimRobot がクリップしていない」ことは、内部の `MjData`(属性名は実装で決める。例 `robot._data`)の `ctrl[shoulder_pan]` が 5.0 のままであることで確かめられる。内部属性に依存するので必須にはしない。採用する場合は Red フェーズで属性名を固定する。
- **対応要件**: 要件 2.3「範囲のクリップはしない」「ctrlrange 外の値」、3 章 NFR-02 との関係、6-8
- 🟡

### TC-B03: 長さ 0、2 次元、0 次元の配列

- **テスト名**: `test_send_command_rejects_degenerate_shapes[empty|row_1x6|col_6x1|scalar]`
  - **境界値の意味**: 要素数が 0、または要素数は 6 でも次元が違う配列。
  - **境界値での動作保証**: 要素数が 6 でも形状が `(6,)` でなければ拒否する(暗黙の flatten をしない)。
- **入力値**: `np.array([])`、`np.zeros((1, 6))`、`np.zeros((6, 1))`、`np.float64(0.0)`。`shared_robot`
  - **境界値選択の根拠**: `(1, 6)` と `(6, 1)` は要素数 6 なので、`size` だけを見る誤った実装を検出できる。
  - **実際の使用場面**: バッチ次元付きの方策出力をそのまま渡す。
- **期待される結果**: すべて `pytest.raises(ValueError)`。状態は変わらない。
  - **境界での正確性**: 形状 `(6,)` だけを通す。
  - **一貫した動作**: TC-E01 と同じ例外の型。
- **テストの目的**: 形状検証の厳密さを確認する。
  - **堅牢性の確認**: 次元の誤りを黙って直さない。
- **対応要件**: 要件 2.3、4.3「形状 (1, 6) や (7,)」
- 🟡

### TC-B04: n_substeps = 1(最小の有効値)

- **テスト名**: `test_min_n_substeps_one_step_per_command`
  - **境界値の意味**: 有効な値の下限。
  - **境界値での動作保証**: 0 は拒否(TC-E03)、1 は受け付ける。
- **入力値**: `SimRobot(scene_path=小モデル, n_substeps=1)`、ゼロ指令を 1 回。
  - **境界値選択の根拠**: 境界の内側。
  - **実際の使用場面**: 1 step ごとに制御する細かい検証。
- **期待される結果**: `state.timestamp == pytest.approx(0.002, abs=1e-9)`。
  - **境界での正確性**: ちょうど 1 step 進む。
  - **一貫した動作**: TC-N06 の式 `k * n_substeps * timestep` と一致。
- **テストの目的**: 下限の確認。
  - **堅牢性の確認**: off-by-one(0 回や 2 回進む)が無い。
- **対応要件**: 要件 2.3
- 🟡

---

## 4. 不変性テストケース

### TC-I01: 返った配列は読み取り専用

- **テスト名**: `test_state_arrays_are_read_only[q|dq]`
  - **何をテストするか**: `state.q`、`state.dq` に書き込めない。
  - **期待される動作**: `flags.writeable` が False。代入すると numpy が `ValueError` を出す。
- **入力値**: `shared_robot.get_state()`。`state.q[0] = 1.0` を試す。
- **期待される結果**: `state.q.flags.writeable is False`。`pytest.raises(ValueError)`(numpy の "read-only")。そのあとの `get_state().q[0]` は元の値のまま。
  - **期待結果の理由**: 要件 2.2「writeable=False を設定した読み取り専用配列」。
- **テストの目的**: 不変性(グローバルルール)の確認。
  - **確認ポイント**: dq も同様であること。
- **対応要件**: 要件 2.2、3 章「不変性」、タスクのテストケース 3
- 🔵(タスクのテストケース 3 を、要件 2.2 の読み取り専用の決定に合わせて具体化)

### TC-I02: RobotState は frozen

- **テスト名**: `test_robot_state_is_frozen`
  - **何をテストするか**: フィールドを付け替えられない。
  - **期待される動作**: `state.q = np.zeros(6)`、`state.timestamp = 1.0` が `dataclasses.FrozenInstanceError` になる。
- **入力値**: `shared_robot.get_state()`
- **期待される結果**: `pytest.raises(dataclasses.FrozenInstanceError)`。
  - **期待結果の理由**: 要件 2.2「@dataclass(frozen=True)」。
- **テストの目的**: 状態オブジェクトの不変性の確認。
  - **確認ポイント**: `dataclasses.is_dataclass(state)` も True であること。
- **対応要件**: 要件 2.2
- 🟡

### TC-I03: 状態はスナップショット(後の指令で変わらない)

- **テスト名**: `test_previous_state_unchanged_after_command`
  - **何をテストするか**: 先に取った `RobotState` が、その後の `send_command` で書き換わらない(`MjData` のビューを返していない)。
  - **期待される動作**: 古い状態は取得したときの値のまま。
- **入力値**: 小モデル。`s0 = get_state()`、`TARGET_DISTINCT` を 10 回送り、`s1 = get_state()`。
- **期待される結果**: `np.allclose(s0.q, 0.0)`、`s0.timestamp == 0.0` のまま。`s1.q` は `s0.q` と異なる。`np.shares_memory(s0.q, s1.q)` は False。
  - **期待結果の理由**: 要件 2.3「MjData の配列(やそのビュー)を外に出さない」。
- **テストの目的**: タスクのテストケース 3(状態はコピーで返る)の本質の確認。
  - **確認ポイント**: 読み取り専用のビュー(`qpos[...]` に `writeable=False` を付けただけ)は TC-I01 を通っても、このテストで失敗する。
- **対応要件**: 要件 2.2、2.3、タスクのテストケース 3
- 🟡

### TC-I04: send_command は呼び出し側の配列を変更せず、後の変更の影響も受けない

- **テスト名**: `test_send_command_does_not_alias_input`
  - **何をテストするか**: (a) 渡した配列が書き換えられない。(b) 呼んだ後に呼び出し側が配列を書き換えても、シミュレーションに影響しない。
  - **期待される動作**: 内部でコピーする。
- **入力値**: 小モデルの `SimRobot` を 2 台(A、B)作る。`arr = TARGET_DISTINCT.copy()`。
  - A: `send_command(arr)` を 1 回、直後に `arr[:] = -2.0` に書き換え、さらに `send_command(TARGET_DISTINCT.copy())` を 9 回。
  - B: `send_command(TARGET_DISTINCT.copy())` を 10 回。
- **期待される結果**:
  - (a) 1 回目の呼び出し直後(書き換えの前)の `arr` が `TARGET_DISTINCT` と等しい。
  - (b) A と B の最終の `q`・`dq`・`timestamp` が完全に一致する(`np.array_equal`。MuJoCo は決定的)。
  - **期待結果の理由**: 要件 2.3「呼び出し側の配列を変更しない(内部でコピーする)」、4.3「呼び出し側の q_target を後で書き換える」。
- **テストの目的**: 入力側の不変性の確認。
  - **確認ポイント**: 小モデルを使うので、2 台作っても速い。
- **対応要件**: 要件 2.3、4.3
- 🟡

### TC-I05: get_state には副作用が無い

- **テスト名**: `test_get_state_has_no_side_effects`
  - **何をテストするか**: `get_state()` を何度呼んでもシミュレーションが進まない。
  - **期待される動作**: 連続した取得は同じ値で、別の配列を返す。
- **入力値**: 小モデル。`TARGET_DISTINCT` を 1 回送ってから(動いている状態にする)、`get_state()` を 3 回呼ぶ。
- **期待される結果**: 3 回とも `timestamp` が同じで、`q`・`dq` が `np.array_equal`。各回の `q` は別のメモリ(`np.shares_memory` が False)。
  - **期待結果の理由**: 要件 2.3「呼ぶだけではシミュレーションを進めない」。
- **テストの目的**: 読み取り操作の純粋性の確認。
  - **確認ポイント**: 動いている状態で確認する(静止状態では `mj_step` が走っても気づけない)。
- **対応要件**: 要件 2.3 get_state
- 🟡

---

## 5. Protocol 適合・context manager のテストケース

### TC-P01: SimRobot は Robot Protocol を満たす

- **テスト名**: `test_sim_robot_satisfies_robot_protocol`
  - **何をテストするか**: `runtime_checkable` な `Robot` に対して `isinstance` が True。
- **入力値**: `shared_robot`
- **期待される結果**: `isinstance(shared_robot, Robot) is True`。
  - **期待結果の理由**: 要件 2.1「runtime_checkable を付け、isinstance で確認できる」。
- **テストの目的**: FR-08(実装の差し替え)の土台の確認。
  - **確認ポイント**: `runtime_checkable` の `isinstance` はメンバーの有無しか見ない(シグネチャは見ない)。シグネチャは TC-N01 などの利用で間接的に確認する。
- **対応要件**: 要件 2.1、FR-08
- 🟡

### TC-P02: メンバーが欠けたクラスは Robot を満たさない

- **テスト名**: `test_object_missing_members_is_not_robot`
  - **何をテストするか**: Protocol が実際にメンバーを検査している(何でも True にならない)。
- **入力値**: `object()`、および `get_state` と `joint_names` だけを持ち `send_command`・`close` が無いテスト用クラスのインスタンス。
- **期待される結果**: どちらも `isinstance(x, Robot) is False`。
  - **期待結果の理由**: Protocol のメンバーは 4 つ(要件 2.1)。
- **テストの目的**: Protocol 定義の誤り(メンバー不足)を検出する。
  - **確認ポイント**: `Robot` が 4 つのメンバーすべてを宣言していること。
- **対応要件**: 要件 2.1
- 🟡

### TC-P03: JOINT_NAMES の再エクスポートを保つ(回帰)

- **テスト名**: 既存の `tests/robot/test_sweep.py::test_joint_names_single_source`(新規には書かない)
  - **何をテストするか**: `base.JOINT_NAMES is model_paths.JOINT_NAMES`。
- **期待される結果**: `base.py` に Protocol を追加した後も通る。
- **テストの目的**: 関節名の定義が一箇所であることを保つ。
- **対応要件**: 要件 1 章、4.3 最終行
- 🔵

### TC-C01: with 文で使え、抜けると close される

- **テスト名**: `test_context_manager_closes_on_exit`
  - **何をテストするか**: `__enter__` が self を返し、`__exit__` で close する。
- **入力値**: `with SimRobot(scene_path=小モデル) as robot:` の中で `send_command(np.zeros(6))` と `get_state()`。
- **期待される結果**: `with` の中では正常に動く。`as` で受けた値は `SimRobot` のインスタンス。`with` を抜けた後の `robot.get_state()` は `RuntimeError`。
  - **期待結果の理由**: 要件 2.3、6-3。
- **テストの目的**: 後片付けの忘れを防ぐ書き方を提供する。
  - **確認ポイント**: `__enter__` の戻り値が None ではないこと。
- **対応要件**: 要件 2.3 close、6-3
- 🔴

### TC-C02: with の中で例外が起きても close され、例外はそのまま伝わる

- **テスト名**: `test_context_manager_closes_and_propagates_exception`
  - **何をテストするか**: `__exit__` が例外を握りつぶさない。
- **入力値**: `with` の中で `send_command(np.zeros(5))`(TC-E01 と同じ不正入力)。
- **期待される結果**: 外側の `pytest.raises(ValueError)` で捕まる。その後 `robot.get_state()` は `RuntimeError`。
  - **期待結果の理由**: グローバルルール「エラーを握りつぶさない」、要件 6-3。
- **テストの目的**: 例外時の後片付けの確認。
  - **確認ポイント**: `__exit__` が True を返さないこと。
- **対応要件**: 要件 6-3、note 2 章(エラーを握りつぶさない)
- 🔴

---

## 6. 開発言語・フレームワーク

- **プログラミング言語**: Python 3.12
  - **言語選択の理由**: プロジェクトの既定(`pyproject.toml` の `requires-python = ">=3.12,<3.13"`)。
  - **テストに適した機能**: `typing.Protocol` + `runtime_checkable`、`dataclasses.FrozenInstanceError`、numpy の `flags.writeable`。
- **テストフレームワーク**: pytest(`pytest.mark.parametrize`、`pytest.approx`、`tmp_path`/`tmp_path_factory`、`scope="module"` fixture)、pytest-cov
  - **フレームワーク選択の理由**: 既存テスト(`tests/test_assets.py`、`tests/robot/test_sweep.py`)と同じ。
  - **テスト実行環境**: `uv run pytest tests/robot/test_sim.py -q`。実 MuJoCo を使い、モックしない。macOS で GUI 不要(viewer は使わない)。
- **テストファイル**: `tests/robot/test_sim.py`(新規)
- **import 対象**: `from so101_vlm.robot.base import Robot, RobotState, JOINT_NAMES`、`from so101_vlm.robot.sim import SimRobot`、`from so101_vlm.robot.model_paths import SO101_SCENE_PATH`
- **実行時間の目標**: ファイル全体で約 2 s 以内(実シーンの生成 7 回以内 × 約 0.2 s + 小モデル)。
- 🔵

### fixture の割り当て(実装の目安)

| fixture | スコープ | シーン | 使うテスト |
|---|---|---|---|
| `shared_robot` | module | 実シーン | TC-N03, TC-E01, TC-E02, TC-B03, TC-I01, TC-I02, TC-P01 |
| `fresh_robot` | function | 実シーン | TC-N01(×3), TC-N02, TC-N04, TC-N08, TC-B02 |
| `tiny_scene_path` | module(`tmp_path_factory`) | 小モデル | TC-N05, TC-N06, TC-N07, TC-E03, TC-E06(変種), TC-E07, TC-B01, TC-B04, TC-I03, TC-I04, TC-I05, TC-C01, TC-C02 |

- 実シーンの生成数: `shared_robot` 1 + `fresh_robot` 7 = 8 回。TC-N08 を TC-N01 の `TARGET_OPPOSITE` に統合すると 7 回(0.3 節の目標)。
- `shared_robot` を使うテストは状態を変えないはず。ただし入力検証が壊れていると状態が進む。各テストは「呼び出し前の timestamp と比べて変わらない」ことを確認するので、他のテストの結果には依存しない。
- `fresh_robot` と小モデルの `SimRobot` は、fixture の後処理(yield の後)で `close()` する。

---

## 7. テスト実装時の日本語コメント指針

各テストには次の日本語コメントを入れる(Python の例)。

```python
def test_converges_to_target(fresh_robot, target):
    # 【テスト目的】: 可動域の中ほどの目標を送り続けると、全関節が目標に一致することを確認する
    # 【テスト内容】: 目標を N_SETTLE_COMMANDS 回送り(1.0 s)、最終状態を目標と比べる
    # 【期待される動作】: 最大誤差 < TOL_RAD、速度 < DQ_SETTLED
    # 🔵 要件 4.2 の実測(最大誤差 8.4e-4 rad)に基づく

    # 【テストデータ準備】: 自己干渉しない |q| <= 1.0 rad の目標を使う
    # 【初期条件設定】: 初期姿勢(ゼロ姿勢、時刻 0)の SimRobot
    q_target = np.asarray(target, dtype=float)

    # 【実際の処理実行】: send_command を 50 回呼ぶ(10 step × 50 = 500 step = 1.0 s)
    for _ in range(N_SETTLE_COMMANDS):
        fresh_robot.send_command(q_target)
    state = fresh_robot.get_state()

    # 【結果検証】: 全関節の誤差と速度を確認する
    # 【検証項目】: 位置の最大誤差 🔵
    assert np.max(np.abs(state.q - q_target)) < TOL_RAD  # 【確認内容】: 全関節が 0.01 rad 以内
    # 【検証項目】: 整定していること 🟡
    assert np.max(np.abs(state.dq)) < DQ_SETTLED  # 【確認内容】: 速度がほぼ 0
```

fixture の例:

```python
@pytest.fixture
def fresh_robot():
    # 【テスト前準備】: 実シーンで初期姿勢の SimRobot を作る
    # 【環境初期化】: テストごとに新しい MjData にして、前のテストの状態を持ち込まない
    robot = SimRobot()
    yield robot
    # 【テスト後処理】: close して資源を解放する(close は冪等なので、テスト内で close 済みでもよい)
    robot.close()
```

---

## 8. 要件定義との対応関係

| 要件の項目 | テストケース |
|---|---|
| 2.1 Robot Protocol(4 メンバー、runtime_checkable) | TC-P01, TC-P02, TC-N03 |
| 2.1 関節順 = JOINT_NAMES | TC-N03, TC-N04, TC-N05, TC-P03 |
| 2.2 RobotState(q, dq, timestamp、frozen、読み取り専用) | TC-N02, TC-N08, TC-I01, TC-I02 |
| 2.3 コンストラクタ(scene_path、n_substeps、名前解決、読み込みエラー) | TC-N05, TC-N06, TC-E03, TC-E04, TC-E05, TC-E06, TC-B04 |
| 2.3 get_state(コピー、副作用なし) | TC-I03, TC-I05 |
| 2.3 send_command(検証、ctrl、n_substeps、クリップしない、入力を変えない) | TC-N01, TC-N06, TC-N07, TC-E01, TC-E02, TC-B02, TC-B03, TC-I04 |
| 2.3 close(冪等、close 後は RuntimeError、with 文) | TC-B01, TC-E07, TC-C01, TC-C02 |
| 3 章 単位(rad、rad/s) | TC-N01, TC-N08 |
| 3 章 NFR-02 との関係(範囲の制限は TASK-0010) | TC-B02 |
| 4.2 収束の基準 | TC-N01, TC-N04 |
| 4.3 エッジケースの表(全 13 行) | TC-E01, TC-B03, TC-E02, TC-N07, TC-B02, TC-I01, TC-I04, TC-N06, TC-B01, TC-E07, TC-E03, TC-P01, TC-P03 |
| タスクのテストケース 1〜3 | TC-N01 / TC-E01 / TC-I01, TC-I03 |
| タスクの統合テスト(TASK-0005 のシーン) | 対象外(TASK-0005 が未実装。TC-N05 で「先頭に余分な関節があるシーン」の名前解決だけ先に確認する) |

- **参照した機能概要**: 要件 1 章
- **参照した入力・出力仕様**: 要件 2.1〜2.4
- **参照した制約条件**: 要件 3 章
- **参照した使用例**: 要件 4.1〜4.3

---

## 9. テストケース数と信頼性の分布

| 分類 | ID | ケース数(parametrize 展開後の件数) |
|---|---|---|
| 正常系 | TC-N01〜N08 | 8(12) |
| 異常系 | TC-E01〜E07 | 7(12) |
| 境界値 | TC-B01〜B04 | 4(7) |
| 不変性 | TC-I01〜I05 | 5(6) |
| Protocol・context manager | TC-P01〜P03, TC-C01〜C02 | 5(4。TC-P03 は既存テストなので新規に書かない) |
| **合計** | | **29 ケース(新規に書くのは 28 ケース・41 件)** |

| 信頼性 | ケース数 | 該当 |
|---|---|---|
| 🔵 | 6 | N01, N03, N04, E01, I01, P03 |
| 🟡 | 19 | N02, N05, N06, N08, E02, E04〜E07, B01〜B04, I02〜I05, P01, P02 |
| 🔴 | 4 | N07, E03, C01, C02 |

(N06 は既定値 10 の部分だけ 🔴。6 章の言語・フレームワークは 🔵)

## 10. 品質判定

- テストケース分類: 正常系・異常系・境界値・不変性・Protocol を網羅 ✅
- 期待値定義: 数値(TOL_RAD、SETTLE_TIME_S、目標ベクトル、時刻)と例外の型・メッセージを明記 ✅
- 技術選択: Python 3.12 + pytest(既存と同じ)で確定 ✅
- 実装可能性: 収束・クランプ・小モデルの名前解決・読み込み時間をすべて実測で確認済み ✅
- 信頼性レベル: 🟡 が多い(設計 4.1 の記述が短いため)。ただし数値は実測で裏付けており、🔴 は要件で明示した仮定 4 点だけ

**判定**: ✅ 高品質(🟡 が多いのは要件段階から引き継いだもの。テストの期待値は具体的で、実装の判断に迷う点は無い)

## 11. 未決事項(Red フェーズ以降で確認)

1. 🔴 要件の仮定 1〜4(timestamp、n_substeps=10、context manager、list 入力)はレビューでの確認が望ましい。変える場合は TC-N02/N06/N07/C01/C02 を直す。
2. TC-B02 の任意のホワイトボックス確認(内部の `MjData` の属性名)を採用するかどうか。
3. `n_substeps` に int 以外(1.5、True)を渡したときの扱いは未定義。本書では対象外とする(必要なら `TypeError` を追加)。
4. `RobotState` を利用者が直接作ったとき(`RobotState(q=list, ...)`)に、`__post_init__` で配列化・読み取り専用化するかは未定義。本書では `SimRobot` が返す状態だけを検証する。
5. 実行時間が約 2 s を超える場合は、`fresh_robot` の数を減らす(TC-N08 を TC-N01 に統合)。または `sim.py` でパスごとに `MjModel` をキャッシュする(MjData は個別)。キャッシュは Refactor フェーズで検討する。
6. TASK-0005 のシーンでの統合テストは、TASK-0005 の完了後に追加する。

---

次のお勧めステップ: `/tsumiki:tdd-red so101-vlm-pickplace TASK-0004` でRedフェーズ(失敗テスト作成)を開始します。

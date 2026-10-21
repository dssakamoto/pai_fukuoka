# TASK-0011 開発コンテキストノート: IK(逆運動学)

**要件名**: so101-vlm-pickplace
**タスクID**: TASK-0011(IK(逆運動学)、TDD、2.5h)
**フェーズ**: Phase 2 - 知覚・制御・安全
**作成日**: 2026-10-20
**作成方法**: tsumiki:tdd-tasknote(Explore サブエージェントは使わず、直接読み込みで収集。一時的な探索スクリプトは実行したが、プロジェクトには保存していない)

> 注: このプロジェクトは tsumiki の標準配置(`docs/spec/{要件名}-requirements.md` など)を使わない。要件は `docs/requirements.md`、設計は `docs/design.md`、共通のコンテキストは `docs/spec/so101-vlm-pickplace/note.md` を参照する。
> 注: `AGENTS.md` と `docs/rule/` は存在しない。追加ルールは `docs/spec/so101-vlm-pickplace/note.md` の「開発ルール」、`CLAUDE.md`(テスト実行コマンド)、ユーザーのグローバルルール(`~/.claude/rules/ecc/`)に従う。
> 注: `docs/implements/so101-vlm-pickplace/TASK-0005/verify-report.md` は「姿勢制約付き IK の到達性は TASK-0009 で確認」と書いているが、IK 本体は TASK-0011 の範囲。本ノートでは TASK-0011 の Green 前に調査する前提にする。

---

## 0. 最初にやること(推奨: 到達性の探索調査)

Red に入る前に、**姿勢制約付き IK の到達性を調査する**(main risk)。結果は TASK-0012 の軌道設計、TASK-0009 の確認、シーンの配置調整(ブロック範囲・箱の位置・接近高さ)に使う。

- 対象: 作業範囲の格子点(x 0.15〜0.35、y -0.10〜0.10、間隔 5 cm 程度)、箱の中心 (0.20, 0.19)、各点の把持高さ z=0.015 と接近高さ z=0.095(真上 8 cm)。
- 各点について、接近方向の傾き候補(真下 0°、15°、30°、45°、60°)のどれで到達できるかを、DLS の複数シード試行で判定する。
- 出力: 格子ごとの「到達可否と最小傾き」の表。保存先は `docs/implements/so101-vlm-pickplace/TASK-0011/` 配下のレポート(例: `reachability-survey.md`)。プロジェクトに残すのは結果のみ。
- 判断基準: 把持高さで真下(0°)が到達できない点が多い場合は、(a) ピッチ候補を広げる、(b) 把持時の接近角を許容する、(c) 作業範囲を狭める、のいずれかを決めてから Red に進む。

### 参考: 予備調査の結果(ランダムサンプリング、粗い値)

> **訂正済み(2026-10-20)**: この表は接近方向を `gripperframe` の **−z 軸** として計算したもので、**誤り**。正しい接近軸は `gripperframe` の **+x 軸**(指の向き。`reachability-survey.md` 2.1 節)。下の表と「把持高さでは真下に近い姿勢がほとんど得られない」という解釈は、軸の取り違えから生じたもの。正しい軸での結果は `reachability-survey.md`(把持高さ z=0.015 では作業範囲の 78% が真下 0° で、全点が 15° 以下で届く)を参照する。表は経緯の記録として残す。

`PICKPLACE_SCENE_PATH` のモデルで、5 関節を可動域内にランダムサンプリング(30 万件)し、`gripperframe` の位置と傾きを記録した。各格子点の周囲 1 cm 以内にあったサンプルの最小傾き(真下からの角度)を示す。**サンプリングが粗いため、真の最小値ではなく上限の目安。** 列は x = 0.15 / 0.20 / 0.25 / 0.30 / 0.35。

| 高さ | y | 0.15 | 0.20 | 0.25 | 0.30 | 0.35 |
|---|---|---|---|---|---|---|
| z=0.015(把持) | -0.10 | 61 | 34 | 79 | 53 | 50 |
| | -0.05 | 65 | 40 | 33 | 67 | 31 |
| | 0.00 | 67 | 60 | 89 | 79 | 81 |
| | +0.05 | 66 | 52 | 54 | 93 | 96 |
| | +0.10 | 60 | 45 | 87 | 31 | 49 |
| z=0.095(接近) | -0.10 | 83 | 123 | 103 | 33 | 68 |
| | -0.05 | 61 | 61 | 32 | 68 | 44 |
| | 0.00 | なし | なし | 11 | 10 | 76 |
| | +0.05 | 62 | 69 | 18 | 33 | 42 |
| | +0.10 | 60 | 44 | 81 | 63 | なし |

- 解釈(暫定): **把持高さ z=0.015 では、真下に近い姿勢(0〜30°)が格子の大半で得られていない**。手先を真下に向けた把持は、作業範囲の多くで難しい可能性が高い。接近高さ z=0.095 では、x=0.25〜0.30 付近で 10° 台が得られる。
- 注意: 「真下」の定義(`gripperframe` の -z 軸)は未確定(下記 6 節)。**訂正済み(2026-10-20)**: 接近軸は +x 軸に確定。-z 軸の仮定は誤り。また、この表は DLS による判定ではない。Green 前の正式な調査で置き換える。
- 関連: 箱の中心 (0.20, 0.19) は z=0.015 で最小約 61°、z=0.095 で約 55°(サンプル数 3〜6 と少ない)。

---

## 1. 技術スタック

- 言語: Python 3.12(`requires-python = ">=3.12,<3.13"`)
- パッケージ管理: uv(`uv sync`、`uv run pytest`)
- シミュレータ・運動学: MuJoCo(`mujoco.MjData`、`mj_jacSite`、`mj_forward` / `mj_kinematics`)、numpy
- テスト: pytest、pytest-cov(カバレッジ 80% 以上)
- 参照元: `pyproject.toml`, `CLAUDE.md`

### アーキテクチャ(設計どおり)

- `IkPolicy` は design 4.2 の `Policy` の実装。ただし `IkPolicy` の `act()` は TASK-0012 の範囲。TASK-0011 は「目標位置と姿勢制約から関節角を解く純粋関数 + IkError」までとする。
- 解法は減衰最小二乗法(DLS)。ヤコビアンは `mj_jacSite`(design 4.5)。
- 解く対象は専用の `MjModel` / `MjData`。実行中のシミュレーションの状態は変更しない。
- 参照元: `docs/design.md`(4.2, 4.5)、`docs/tasks/so101-vlm-pickplace/TASK-0012.md`(IkPolicy の実装先)

---

## 2. 開発ルール

- TDD: テストを先に書く(RED → GREEN → REFACTOR)。カバレッジ 80% 以上。
- 不変性: 入力配列は変更しない。出力は新しい配列を返す。データ型は frozen dataclass。
- 関数は 50 行未満、ファイルは 200〜400 行目安(最大 800 行)。
- 数値の閾値・候補角は名前付き定数にする(マジックナンバーを避ける)。
- エラーは握りつぶさず、`IkError` のメッセージに目標と最良の残差を含める(ユーザーが対処できる内容)。
- 秘密情報: このタスクには該当なし。
- コミットは conventional commits(今回のステップではコミットしない)。
- 実装は TASK-0011 の範囲に限る。`IkPolicy` の軌道生成(TASK-0012)、安全フィルタ(TASK-0010)は含めない。
- 参照元: `docs/spec/so101-vlm-pickplace/note.md`、`docs/requirements.md`(NFR-05: 純粋ロジックに単体テスト)、`CLAUDE.md`、`~/.claude/rules/ecc/python/*.md`、`~/.claude/rules/ecc/common/*.md`

---

## 3. 関連実装

既存コード:

- `src/so101_vlm/robot/model_paths.py`: `JOINT_NAMES`(6 関節: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper)、`SITE_NAMES`(baseframe, gripperframe)、`PICKPLACE_SCENE_PATH`(IK はこのシーンを読む)、`SO101_SCENE_PATH`
- `src/so101_vlm/robot/base.py`: `JOINT_NAMES` / `N_JOINTS`(6) の再エクスポート、`validate_q_target(q)`(shape `(6,)`、NaN/inf を `ValueError`)、`RobotState`(読み取り専用コピー)
- `src/so101_vlm/robot/sim.py`: `SimRobot`。名前で関節・アクチュエータを引く。`_load_model`、`_name_id` の再利用候補(ただし `sim.py` は変更しない。必要なら共通化は REFACTOR で判断)
- `src/so101_vlm/robot/sweep.py`: 純粋関数の書き方の参考(入力を変更せず新しい配列を返す、引数検証で `ValueError`)
- `src/so101_vlm/eval/scene_spec.py`: `SCENE_SPEC`(work_area_x (0.15, 0.35)、work_area_y (-0.10, 0.10)、box_center_xy (0.20, 0.19)、block_rest_z 0.015、box_rim_z 0.04)
- `src/so101_vlm/policy/__init__.py`: 空。`ik.py` は未作成。
- 参照元: `src/so101_vlm/robot/model_paths.py`, `src/so101_vlm/robot/base.py`, `src/so101_vlm/robot/sim.py`, `src/so101_vlm/robot/sweep.py`, `src/so101_vlm/eval/scene_spec.py`, `src/so101_vlm/policy/__init__.py`

### モデル・シーンの事実(実測と XML)

- シーン: `assets/scene_pickplace.xml`(`so101/so101_new_calib.xml` を include)。ブロック 3 個は freejoint、箱は静的な 5 枚の geom。IK は関節だけを扱うため、ブロックや箱は運動学に影響しない。
- 関節順と可動域(rad):
  - shoulder_pan [-1.920, 1.920]、shoulder_lift [-1.745, 1.745]、elbow_flex [-1.69, 1.69]、wrist_flex [-1.658, 1.658]、wrist_roll [-2.744, 2.841]、gripper [-0.175, 1.745]
- アクチュエータの `ctrlrange` は関節の可動域と一致している(`assets/so101/so101_new_calib.xml` の actuator 節)。IK の上下限は `m.jnt_range`(または `m.actuator_ctrlrange`)のどちらでもよいが、一方に統一する。
- 基準 site: `baseframe` は世界原点 (0,0,0)。`gripperframe` は手先の基準点。
- 実測(一時スクリプト、プロジェクト外): 原点姿勢 q=0 で `gripperframe` は世界座標 (0.391, 0.000, 0.227)。その `-z` 軸は世界の下向き(z 成分 ≈ -0.999)。`+z` 軸は上向き。

---

## 4. 設計文書

### design.md 4.5 IkPolicy(抜粋)

- 5 自由度のため、目標は「`gripperframe` を目標位置に置き、手先を下向きに近づける」とする。
- 解法: `mj_jacSite` のヤコビアンを使う DLS を反復する。姿勢は「手先の向きの下向き成分」だけを重みを下げて拘束する。ピッチを複数候補(真下から斜めまで)で試し、解が得られた最初のものを採用する。
- 関節可動域(`ctrlrange`)でクリップし、収束しなければ `IkError`(FR-06)。
- 数値(接近高さ、速度、グリッパー開閉量)は `configs/motion.yaml`。

### design.md 1 / 4.1 / 4.2(抜粋)

- 関節は 6 つ。腕は 5 自由度 + グリッパー。IK は「位置 + 手先を下向きに近づける」制約付き(design 1)。
- 単位はラジアン。`Robot.send_command` は 6 要素の位置目標(design 4.1)。
- `Policy.act(obs, goal) -> JointTrajectory`(design 4.2)。TASK-0011 では直接使わない。

### requirements.md(FR-05, FR-06, NFR-05)

- FR-05: 3 次元位置から、接近・把持・持ち上げ・移動・解放の関節軌道を生成して実行する。
- FR-06: IK が解を得られない、または関節可動域を超えるとき、動作を実行せずエラーを返す。
- NFR-05: 座標変換・IK・成功判定などの純粋ロジックに単体テストを書く。

参照元: `docs/design.md`(1, 4.1, 4.2, 4.5, 9 章の注意)、`docs/requirements.md`(FR-05, FR-06, NFR-05)、`docs/tasks/so101-vlm-pickplace/TASK-0011.md`

---

## 5. テスト関連情報

- テストフレームワーク: pytest(`pyproject.toml` の `[tool.pytest.ini_options]` で `testpaths = ["tests"]`)。カバレッジ設定: `[tool.coverage.run] source = ["so101_vlm"]`。
- 実行: `uv run pytest -q`(現状 247 passed)。カバレッジ: `uv run pytest --cov`。
- 既存テスト構成:
  - `tests/test_assets.py`、`tests/test_package.py`、`tests/test_scene.py`
  - `tests/robot/test_sim.py`(`SimRobot` のテスト。docstring にテストケース定義へのリンクとして `docs/implements/so101-vlm-pickplace/TASK-0004/sim-robot-testcases.md` を記載する形式)、`tests/robot/test_sweep.py`
  - `tests/eval/test_layout.py`、`tests/eval/test_place_blocks.py`、`tests/eval/test_eval_config.py`
- 命名・書式: ファイル先頭に docstring(対象タスク、テストケース定義の参照先、信頼性の凡例 🔵🟡🔴)。定数は大文字の名前付き。テスト関数は `test_` 接頭辞。
- 新規テストファイル: `tests/policy/test_ik.py`(TASK-0011 の指定どおり)。`tests/policy/` ディレクトリは未作成。既存の `tests/robot/` に `__init__.py` があるかは Red 開始時に確認する。
- 新規実装ファイル: `src/so101_vlm/policy/ik.py`(`policy/__init__.py` は既存)。
- シーン・モデル関連テストの流儀: `tests/test_scene.py` が `scene_spec.py` と XML の値を照合している。IK のテストでも、シーン定数は `SCENE_SPEC` から取る。
- 参照元: `pyproject.toml`, `CLAUDE.md`, `tests/robot/test_sim.py`, `tests/robot/test_sweep.py`, `tests/test_scene.py`, `docs/implements/so101-vlm-pickplace/TASK-0005/verify-report.md`

---

## 6. 注意事項(設計上の判断と技術的制約)

### 6.1 自由度と変数

- 関数に渡す変数は **5 関節のみ**(shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll)。**gripper(index 5)は IK の変数から除外する。**
- 実測の位置ヤコビアン(`gripperframe`、3 つの姿勢のサンプル平均、単位 m/rad)の目安:
  - shoulder_lift: 約 0.25〜0.38
  - elbow_flex: 約 0.26〜0.28
  - shoulder_pan: 約 0.09〜0.24
  - wrist_flex: 約 0.16
  - **wrist_roll: 約 0.008**(手先が roll 軸から約 8 mm ずれているため。位置への寄与は小さい)
  - gripper: 0(位置に影響しない)
- したがって、**位置に効くのは実質 4 関節**(pan, lift, elbow, wrist_flex)。wrist_roll は位置をほぼ動かさず、主に手先まわりの回転に効く。
- 姿勢の拘束(接近方向の 2 自由度)+ 位置 3 自由度 = 5 行で、5 関節とちょうど釣り合う。ただし特異姿勢の近くでは DLS の減衰が必要。
- 戻り値: 6 要素(`Robot.send_command` と同じ形式)にする。gripper は **seed の値をそのままコピー**する(IK は gripper を動かさない)。
- 既定: 5 関節のみを返す案は採らない。TASK-0012 の軌道生成と `send_command` がどちらも 6 要素を前提にするため。

### 6.2 「真下」の定義(訂正済み 2026-10-20: 確定)

> **訂正済み(2026-10-20)**: 下の「接近方向 = `-z` 軸」の仮定は**誤り**。**接近方向(指の向き)は `gripperframe` の +x 軸**(手首から指先へ。wrist_roll の回転軸と平行)。q=0 では腕が水平に前へ伸びて site +x が世界 +x になり、site −z が世界の下向きに見えたのは偶然だった(`reachability-survey.md` 2.1 節)。ピッチは「site +x 軸と世界 (0, 0, −1) のなす角」、定数は `APPROACH_AXIS_LOCAL = (1, 0, 0)`(`ik-requirements.md` 2.2 節)。0 節・6.7 節の「把持高さでは真下に向けられない」という見立ては、この誤った軸から来たもの。以下の元の記述は経緯として残す。

- 原点姿勢で `gripperframe` の `-z` 軸が世界の下向きになっている(実測)。そこで **接近方向 = `gripperframe` の `-z` 軸** を仮定する。
- Red の前に、接近方向の定義を確定する(`gripperframe` の軸と向きを、ビューアか運動学の値で確認する)。確定値は定数として `ik.py` の先頭に置く。
- 傾き(ピッチ)の判定: 接近方向と世界の `(0, 0, -1)` のなす角。

### 6.3 ピッチ候補と緩い拘束

- 候補角(仮): 真下 0°、15°、30°、45°、60°。小さい順に試し、収束した最初の候補を採用する。
- 拘束は弱くする(design 4.5)。位置の残差を主、傾きの残差を重み付き(例 0.3、仮)として、同じ DLS の残差に入れる。
- 候補ごとに seed を変える必要はない。前の候補の結果を次の seed にしてもよいが、結果が seed に依存するため、テストは固定 seed で再現させる。

### 6.4 DLS の数値(仮)

- 減衰 λ: 1e-2 程度(仮。収束が遅い場合に調整)
- 反復上限: 200 回(仮)
- 位置の収束判定: 残差 1 mm 以下(テストの 2 mm しきい値より厳しくする)
- 傾きの収束判定: 候補角に対して ±2° 以内(仮)
- 1 反復の更新量の上限(例 0.1 rad、仮)。大きすぎるステップを抑える。
- 各反復で関節を `jnt_range` にクリップする(design 4.5)。クリップ後も収束しなければ、その候補は失敗とする。

### 6.5 シミュレーションを変えない

- IK は **専用の `MjModel` と `MjData`** を持つ(`mujoco.MjData(model)`)。`SimRobot` が持つ `MjData` の参照を受け取らない。
- `MjModel` は読み取り専用として扱う。読み込みは `PICKPLACE_SCENE_PATH` から行い、モジュール単位でキャッシュしてもよい(テストで確認する)。
- 入力の `seed_q` と `target` は変更しない(テストで確認する)。
- 検証: `SimRobot` を作り、IK を解く前後で `get_state()` が一致することを確認する(TASK-0011 の「シミュレーション本体を変えない」)。

### 6.6 エラーと失敗の扱い

- `IkError`(FR-06): 全候補で収束しない場合に送出する。メッセージに目標、最良の残差、試した候補角を含める。
- 可動域違反: クリップ後に収束しなければ `IkError`。解を返す場合は必ず可動域内にある。
- 入力検証: 目標は shape `(3,)`、有限値。seed は shape `(6,)`、有限値。不正なら `ValueError`(`validate_q_target` を流用できる)。
- 到達できない目標の例: 腕の長さより遠い点(例 (1.0, 0.0, 0.2))。

### 6.7 作業範囲と到達性(主なリスク)

- 作業範囲 x 0.15〜0.35 の点は、把持高さ z=0.015 で真下に近い姿勢がほとんど得られない(0 節の予備調査)。
- 対策の候補: (a) 把持時の接近角の許容を上げる(45〜60° 程度)、(b) 作業範囲の x を広げるか、ブロック配置(TASK-0006)を再考する、(c) 箱の位置を調整する(design 9 章)。
- 決定は、0 節の正式な調査の結果を見てから行う。TASK-0011 のテストでは、**「真下」の判定しきい値を仮置き**し、確定後に更新する。

### 6.8 ノート内の仮の値(テストケース)

- TASK-0011 のテストケース 1: 位置誤差 2 mm 以下(仮。solver の収束は 1 mm)。
- テストケース 4: 手先の傾きの許容角は仮(例 30°)。調査の結果で確定する。

---

## 7. テストケースの案(Red 用、TASK-0011 の 4 ケースを拡張)

| ID | 種類 | Given | When | Then |
|---|---|---|---|---|
| T1 | 正常 | 作業台上の到達可能な点(0 節の調査で到達性を確認済みの点) | `solve_ik` を呼ぶ | 位置誤差 ≤ 2 mm、戻り値 shape (6,) |
| T2 | 異常 | 腕の長さより遠い点 | `solve_ik` を呼ぶ | `IkError` が出る。メッセージに目標を含む |
| T3 | 境界 | 到達可能な点 | 解を得る | 全関節が `jnt_range` 内 |
| T4 | 姿勢 | 箱の真上 8 cm(接近点)または到達可能な点 | 解を得る | 接近方向と真下のなす角が許容内(仮) |
| T5 | 不変 | 入力の `seed_q`、`target` | 解を得る | 入力配列が変更されていない |
| T6 | 不変 | `SimRobot` を作る | IK を解く | 前後で `get_state()` の q, dq, timestamp が一致 |
| T7 | 規則 | 任意の seed | 解を得る | 戻り値の gripper(index 5)が seed と一致 |
| T8 | 再現 | 同じ target、seed | 2 回解く | 結果が一致(固定 seed) |
| T9 | 入力 | shape が (2,) の target、NaN を含む seed | 呼ぶ | `ValueError` |
| T10 | 異常 | 可動域の境界の外に解がある目標 | 呼ぶ | 可動域外の解を返さない(`IkError` または可動域内の解のみ) |

- 各テストは AAA 形式。テスト名は振る舞いを説明する(例: `returns_joints_within_ctrlrange_for_reachable_target`)。
- テストケースの詳細は、tsumiki の tdd-testcases で `docs/implements/so101-vlm-pickplace/TASK-0011/` に作る(後続ステップ)。

---

## 8. 注意事項の要約(実装者向け)

- 解く変数は 5 関節(gripper を除く)。戻り値は 6 要素で、gripper は seed をコピー。
- 専用の `MjModel` / `MjData` を使い、`SimRobot` の状態を触らない。
- ~~接近方向は `gripperframe` の `-z` 軸を仮定。Red 前に確定する。~~ **訂正済み(2026-10-20)**: 接近方向は `gripperframe` の +x 軸(6.2 節)。
- 位置に効くのは pan, lift, elbow, wrist_flex。wrist_roll は位置への寄与がほぼゼロ。
- 把持高さでの真下の到達性が、最大のリスク。Red の前に到達性調査を行う。
- `IkPolicy`(`act`)と軌道生成は TASK-0012。TASK-0011 では `solve_ik` と `IkError` まで。
- `CLAUDE.md` は変更しない。`docs/design.md` の変更は、必要なら別タスクで行う。

---

## 9. 参照元一覧(プロジェクトルートからの相対パス)

- `docs/tasks/so101-vlm-pickplace/TASK-0011.md`, `TASK-0012.md`, `TASK-0009.md`, `overview.md`
- `docs/requirements.md`(FR-05, FR-06, NFR-05)
- `docs/design.md`(1, 4.1, 4.2, 4.5, 9 章)
- `docs/spec/so101-vlm-pickplace/note.md`
- `docs/implements/so101-vlm-pickplace/TASK-0004/note.md`(ノートの形式の参考)
- `docs/implements/so101-vlm-pickplace/TASK-0005/verify-report.md`
- `src/so101_vlm/robot/model_paths.py`, `src/so101_vlm/robot/base.py`, `src/so101_vlm/robot/sim.py`, `src/so101_vlm/robot/sweep.py`
- `src/so101_vlm/eval/scene_spec.py`
- `src/so101_vlm/policy/__init__.py`
- `assets/scene_pickplace.xml`, `assets/so101/so101_new_calib.xml`
- `tests/robot/test_sim.py`, `tests/robot/test_sweep.py`, `tests/test_scene.py`
- `pyproject.toml`, `CLAUDE.md`

# TASK-0006 TDD 要件定義書: ブロック初期配置の乱数生成 (layout)

**作成日**: 2026-10-20
**要件名**: so101-vlm-pickplace
**タスクID**: TASK-0006
**機能名**: layout (ブロック初期配置の乱数生成)
**前ステップ**: docs/implements/so101-vlm-pickplace/TASK-0006/note.md
**実装ファイル (予定)**: `src/so101_vlm/eval/layout.py`、`src/so101_vlm/eval/eval_config.py`、`configs/eval.yaml`
**テストファイル (予定)**: `tests/eval/test_layout.py`、`tests/eval/test_eval_config.py`

信頼性レベル: 🔵 要件定義書・設計文書どおり / 🟡 文書からの妥当な推測 / 🔴 文書にない決定 (本書で決めた仮定)

---

## 1. 機能の概要

- 🔵 **何をするか**: 整数シードから、作業範囲内の正方形領域にブロック 3 個の初期位置 (x, y) を、中心間距離 5 cm 以上でランダムに生成する。生成した配置を MuJoCo のシーン (MjData) に適用する関数も用意する (FR-12, design 4.7)。
- 🔵 **解決する問題**: 評価 (FR-12) で、初期配置を変えた複数試行を自動実行し、かつシードで再現できるようにする (NFR-01)。
- 🟡 **想定ユーザー**: 評価ランナー (Evaluator, design 4.8) と、TASK-0013 以降の統合テスト。開発者本人がシードを指定して失敗試行を再現する。
- 🟡 **システム内の位置づけ**: `so101_vlm.eval` パッケージ内の純粋ロジック。寸法・名前の単一の情報源 `SCENE_SPEC` (src/so101_vlm/eval/scene_spec.py) を参照する。副作用は `place_blocks` に限定する。
- **参照した要件**: FR-12, NFR-01, NFR-05
- **参照した設計文書**: docs/design.md 4.7 (シーンとカメラ), 4.8 (Evaluator), 5 (設定ファイル)

---

## 2. 入力・出力の仕様

### 2.1 `sample_layout` (純粋関数)

```python
def sample_layout(
    seed: int,
    n: int = 3,
    area: float = 0.20,
    min_gap: float = 0.05,
    max_attempts: int = 10_000,
) -> tuple[tuple[float, float], ...]: ...
```

| 引数 | 型 | 制約 | 信頼性 |
|---|---|---|---|
| `seed` | `int` (`numpy.integer` も可) | 0 以上。`bool` は不可 | 🔵 (シード指定は FR-12/NFR-01) / 🔴 (型・負値の扱い) |
| `n` | `int` | 1 以上。`bool` は不可 | 🔵 (既定 3 は FR-12) / 🟡 (下限) |
| `area` | `float` (`int` も可) | 0 より大きく、有限、かつ作業範囲の幅 (x, y の小さい方。現在 0.20) 以下 | 🔵 (20 cm) / 🟡 (上限チェック) |
| `min_gap` | `float` (`int` も可) | 0 以上で有限。0 は「間隔の制約なし」 | 🔵 (5 cm) / 🟡 (0 を許す) |
| `max_attempts` | `int` | 1 以上。`bool` は不可 | 🟡 (既定 10,000 は推測) |

- 🔵 **出力**: 長さ `n` の tuple。各要素は Python の `float` 2 個からなる `(x, y)` [m, ワールド座標]。numpy 型・配列は返さない (不変性ルール)。
- 🟡 **領域の定義**: 中心 `(cx, cy)` は `SCENE_SPEC.work_area_x` / `work_area_y` の中点から計算する (現在の値で (0.25, 0.00))。領域は `cx ± area/2`, `cy ± area/2` の閉区間の正方形。境界値は `layout.py` に書かず、`SCENE_SPEC` のみから導く (単一の情報源)。
- 🟡 **領域は「ブロック中心」の範囲とする (余白なし)**: `SCENE_SPEC.work_area_*` は既存テスト (tests/test_scene.py::test_block_initial_position_in_work_area) でもブロック中心の範囲として扱われている。半寸 1.5 cm の余白を引くと 20 cm 四方が実質 17 cm になり FR-12 の値とずれるため、余白は入れない。箱の壁の外形 (y ≥ 0.145) と、中心の上限 y = 0.10 + 半寸 0.015 = 0.115 は重ならない。
- 🔵 **間隔の判定**: すべての組 (i, j) でブロック中心間のユークリッド距離 `>= min_gap`。tests/test_scene.py::test_blocks_are_at_least_5cm_apart と同じ基準。中心間 5 cm は辺の隙間 2 cm 以上に相当する。
- 🔵 **決定性**: 関数内で `rng = numpy.random.default_rng(seed)` を作り、それだけを使う。`np.random.seed` などのグローバル RNG は使わない、変更しない。同じ引数なら必ず同じ結果。
- 🟡 **異なるシード**: シードが違えば配置が違う (テストでは例えばシード 0 と 1 を比較し、0〜19 で全配置が互いに異なることを確認する)。
- 🟡 **棄却サンプリング**: 1 個ずつ `rng.uniform` で (x, y) を引き、採用済みの点すべてと `min_gap` 以上離れていれば採用する。`max_attempts` は **全ブロックを通した候補の引き数の合計** の上限とする (1 個目から全体をやり直す方式は採らない)。
- 🔴 **到達不能条件の扱い**: 事前の幾何判定は入れない。上限到達時に `ValueError` を送出する。ただし計算が軽く確実な必要条件として、`min_gap > area * sqrt(2)` かつ `n >= 2` (正方形の対角線より長い間隔は 2 点でも不可能) の場合は、引く前に `ValueError` にする。それ以上の判定 (円の詰め込み問題) は YAGNI で行わない。
- 🔵 **エラーメッセージ**: 上限到達時は `n`, `area`, `min_gap`, `max_attempts` の値を含める (TASK-0006 テストケース4 の確認対象は試行回数)。

### 2.2 試行ごとのシード

- 🟡 1 試行 = 1 シード → 1 配置 (design 4.8 は試行ごとにシードを記録する)。試行 i (0 始まり) のシードは `base_seed + i`。既定の `base_seed = 0`、`n_trials = 20` なので、評価の 20 試行はシード 0〜19 を使う。
- 🟡 `sample_layout` 自体は単一シードしか受け取らない。シード列の生成は Evaluator (TASK-0015 以降) の責務。本タスクでは `EvalConfig` にその値を持たせるだけ。

### 2.3 `place_blocks` (唯一の副作用関数)

```python
def place_blocks(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    layout: Sequence[tuple[float, float]],
    block_names: Sequence[str] = SCENE_SPEC.block_names,
) -> None: ...
```

- 🟡 各ブロックについて、body 名から `model.body(name).jntadr` で関節を引き (sim.py と同じく名前で解決し、添字を決め打ちしない)、その関節が freejoint (`mjJNT_FREE`) であることを確認する。関節名 `<body>_joint` の命名規則には依存しない。
- 🔵 qpos の 7 要素を `(x, y, SCENE_SPEC.block_rest_z, 1, 0, 0, 0)` に設定する (位置 + 単位クォータニオン)。
- 🟡 qvel の 6 要素 (`jnt_dofadr` から) を 0 にする。
- 🟡 書き込み後に `mujoco.mj_forward(model, data)` を 1 回呼ぶ。`mj_step` は呼ばない (時刻は進めない)。
- 🟡 対象ブロック以外の qpos / qvel (ロボットの関節など) は変更しない。
- 🟡 **先に全部検証してから書く**: 長さ不一致・名前不明・freejoint でない場合は、data を一切変更せずに `ValueError` を送出する (途中まで書き換えた状態を残さない)。
- 🟡 `len(layout) != len(block_names)` → `ValueError` (両方の長さをメッセージに入れる)。
- 🔵 戻り値は `None`。

### 2.4 設定ファイル `configs/eval.yaml` と読み込み関数

```yaml
# 評価設定 (TASK-0006 時点。判定しきい値は TASK-0015 で追加する)
n_trials: 20
base_seed: 0
placement:
  n_blocks: 3
  area: 0.20        # 配置領域 (正方形) の一辺 [m]。中心は SCENE_SPEC の作業範囲の中点
  min_gap: 0.05     # ブロック中心間の最小距離 [m]
  max_attempts: 10000
```

- 🔵 試行数・シード・配置範囲を `configs/eval.yaml` に置く (design 5)。
- 🟡 判定しきい値 (FR-07) は TASK-0015 の範囲。本タスクでは追加しない。
- 🟡 読み込みは `src/so101_vlm/eval/eval_config.py` に置く。

```python
@dataclass(frozen=True)
class PlacementConfig:
    n_blocks: int
    area: float
    min_gap: float
    max_attempts: int

@dataclass(frozen=True)
class EvalConfig:
    n_trials: int
    base_seed: int
    placement: PlacementConfig

def load_eval_config(path: Path = DEFAULT_EVAL_CONFIG_PATH) -> EvalConfig: ...
```

- 🔵 `yaml.safe_load` を使う (`yaml.load` は使わない)。
- 🟡 既定パスはリポジトリの `configs/eval.yaml` (パッケージ位置から解決する。sim.py の `SO101_SCENE_PATH` と同じ方式)。
- 🔴 **検証方針**:
  - ファイルがない → `FileNotFoundError` をそのまま送出 (握りつぶさない)
  - 最上位・`placement` が mapping でない → `ValueError`
  - 必須キーが欠けている → `ValueError` (欠けたキー名を含める)。暗黙の既定値で補わない
  - **未知のキー → `ValueError`** (タイプミスの検出。キー名を含める)
  - 型・値域は `sample_layout` と同じ規則 (`n_trials >= 1`, `base_seed >= 0`, bool 不可、area は正かつ作業範囲以内、など)。数値は YAML の int を float フィールドに受け入れる
- 🟡 `sample_layout` は設定を読まない (引数だけで決まる純粋関数)。呼び出し側が `cfg.placement` の値を引数に渡す。

---

## 3. 制約条件

- 🔵 **再現性 (NFR-01)**: 結果はシードと引数だけで決まる。グローバル状態 (グローバル RNG・環境変数・時刻) に依存しない、変更しない。
- 🔵 **品質 (NFR-05)**: 純粋ロジックに単体テストを書く。カバレッジ 80% 以上 (グローバルルール)。
- 🟡 **性能**: 既定値 (20 cm 四方、3 個、5 cm) で 1 回の呼び出しは数 ms 以内。シード 0〜99 の全テストが 1 秒未満。
- 🔵 **不変性**: `sample_layout` は入力を変更せず tuple を返す。MjData を変更するのは `place_blocks` だけ (明示的な副作用)。設定は frozen dataclass。
- 🔵 **単一の情報源**: 作業範囲の境界・ブロック名・静止高さは `SCENE_SPEC` のみ。`configs/eval.yaml` の `area` は一辺の長さであり、境界値ではない。
- 🟡 **例外の型**: 引数・設定の不正はすべて `ValueError` に統一する (seed 等の型不正も含む。呼び出し側が 1 種類の例外で扱える)。🔴
- 🟡 **ファイルサイズ**: `layout.py`、`eval_config.py` とも 200 行以内、関数 50 行未満。
- 🔵 **依存**: numpy, mujoco, pyyaml (pyproject.toml に既存)。追加依存なし。

---

## 4. 想定される使用例

### 4.1 基本パターン 🔵

```python
cfg = load_eval_config()
p = cfg.placement
for i in range(cfg.n_trials):
    seed = cfg.base_seed + i
    layout = sample_layout(seed, n=p.n_blocks, area=p.area,
                           min_gap=p.min_gap, max_attempts=p.max_attempts)
    place_blocks(model, data, layout)
    # ... 試行を実行し、seed を記録 (design 4.8)
```

### 4.2 正常系

- 🔵 シード 7 で 2 回呼ぶと同じ配置 (TASK-0006 ケース1)
- 🔵 シード 0〜99 のすべてで、全組の中心間距離 >= 0.05 (ケース2)
- 🔵 シード 0〜99 のすべてで、全点が領域 [0.15, 0.35] × [-0.10, 0.10] 内 (ケース3)
- 🟡 戻り値は tuple、要素は Python float の 2 要素 tuple、長さ n
- 🟡 異なるシードで異なる配置 (0〜19 の配置が互いに異なる)
- 🟡 `sample_layout` 呼び出しの前後で `np.random` のグローバル状態が変わらない
- 🟡 `area` を小さくすると、点が中心 (0.25, 0.00) ± area/2 に収まる
- 🟡 `n=1` は常に成功。`min_gap=0` も成功
- 🟡 `place_blocks` 後: 3 ブロックの qpos が (x, y, 0.015, 1, 0, 0, 0)、qvel が 0、ロボット関節の qpos/qvel は呼び出し前と同じ、`data.xpos` (body 位置) が layout と一致 (mj_forward が呼ばれた証拠)
- 🟡 統合: シーン `assets/scene_pickplace.xml` に配置を適用し 500 steps (1 秒) 進めると、各ブロックの速度ノルム < 1 cm/s、中心間距離 >= 3 cm (重なりなし)、xy の移動 < 5 mm
- 🟡 `load_eval_config()` がリポジトリの `configs/eval.yaml` を読み、`n_trials=20, base_seed=0, area=0.20, min_gap=0.05, n_blocks=3, max_attempts=10000` を返す
- 🟡 `configs/eval.yaml` の area が `SCENE_SPEC` の作業範囲の幅以下 (作業範囲が変わったら検出)

### 4.3 エラーケース

| 入力 | 期待 | 信頼性 |
|---|---|---|
| area=0.05, n=3, min_gap=0.05 | `ValueError` (上限到達。メッセージに試行回数) | 🟡 (TASK-0006 ケース4) |
| min_gap > area·√2, n≥2 | `ValueError` (事前判定) | 🔴 |
| n=0 / n=-1 / n=True | `ValueError` | 🟡 |
| area=0 / 負 / NaN / inf / 作業範囲の幅超え (0.21) | `ValueError` | 🟡 |
| min_gap 負 / NaN | `ValueError` | 🟡 |
| max_attempts=0 / True | `ValueError` | 🟡 |
| seed=-1 / True / 1.5 / "7" / None | `ValueError` | 🔴 |
| `place_blocks` で len(layout)=2, ブロック 3 | `ValueError`、data は不変 | 🟡 |
| `place_blocks` で存在しない body 名 | `ValueError` (名前を含む)、data は不変 | 🟡 |
| `place_blocks` で freejoint を持たない body (例: `box`) | `ValueError` | 🔴 |
| 設定に未知キー / 欠落キー / 型不正 / mapping でない | `ValueError` | 🔴 |
| 設定ファイルがない | `FileNotFoundError` | 🔴 |

### 4.4 エッジケース

- 🟡 `max_attempts=1, n=1` は成功 (1 回目の候補は必ず採用される)
- 🟡 `max_attempts=2, n=3` は `ValueError` (3 個に最低 3 回必要)
- 🟡 `numpy.int64(5)` のシードは int と同じ結果を返す

---

## 5. 要件・設計文書との対応関係

- **参照したユーザストーリー**: 評価ランナーが初期配置を変えた複数試行を再現可能に実行する
- **参照した機能要件**: FR-12 (20 cm × 20 cm、5 cm 以上、箱固定)
- **参照した非機能要件**: NFR-01 (シード固定)、NFR-05 (純粋ロジックの単体テスト)
- **参照した受け入れ基準**: TASK-0006 完了条件 (同じシードで同じ配置 / 範囲内・5 cm 以上 / 単体テスト)、テストケース1〜4、統合テスト1
- **参照した設計文書**:
  - design.md 4.7: シードから 20 cm 四方に生成、5 cm 以上
  - design.md 4.8: 試行ごとのシード記録
  - design.md 5: `configs/eval.yaml` (試行数, シード, 配置範囲, 判定しきい値)
- **参照した実装**: src/so101_vlm/eval/scene_spec.py (`SCENE_SPEC`)、assets/scene_pickplace.xml (freejoint ブロック)、src/so101_vlm/robot/sim.py (名前による添字解決)、tests/test_scene.py (範囲・間隔の判定基準)

---

## 6. 本書で決めた仮定 (🔴/🟡 の主なもの)

1. 領域はブロック中心の範囲で、半寸の余白は入れない (`SCENE_SPEC.work_area_*` の意味に合わせる) 🟡
2. `max_attempts` は全ブロック合計の候補数 🟡
3. 不可能条件は上限到達で検出。軽い必要条件 (`min_gap > area·√2`, n≥2) のみ事前判定 🔴
4. seed は 0 以上の int (numpy 整数可、bool 不可)。不正は型も含め `ValueError` 🔴
5. 試行 i のシードは `base_seed + i`、既定 `base_seed=0, n_trials=20` (シード 0〜19) 🟡
6. 設定は未知キー・欠落キーとも `ValueError`、既定値で補わない 🔴
7. 設定の読み込みは `eval_config.py` (frozen dataclass 2 つ + `load_eval_config`) 🟡
8. `place_blocks` は body 名から freejoint を解決し、全検証後に書き込む (失敗時 data 不変)。qvel 0、`mj_forward` のみ 🟡
9. 判定しきい値は `eval.yaml` に TASK-0015 で追加する 🟡

## 7. 信頼性レベルの分布 (概算)

| 🔵 | 🟡 | 🔴 |
|---|---|---|
| 約 16 項目 (35%) | 約 22 項目 (48%) | 約 8 項目 (17%) |

**品質判定**: ⚠️ 要改善 (ただし実装可能)。入出力・エラー条件はすべて確定し曖昧さはないが、🟡🔴 は文書にない詳細 (型の扱い・設定検証・余白なし) を本書で決めたもの。テストケース作成時にこの決定をそのまま期待値として使える。

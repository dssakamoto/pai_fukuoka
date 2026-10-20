# TASK-0006 テストケース定義書: ブロック初期配置の乱数生成 (layout)

**作成日**: 2026-10-20
**要件名**: so101-vlm-pickplace
**タスクID**: TASK-0006
**機能名**: layout
**前ステップ**: docs/implements/so101-vlm-pickplace/TASK-0006/layout-requirements.md (以下「要件書」)
**コンテキスト**: docs/implements/so101-vlm-pickplace/TASK-0006/note.md
**対象テストファイル**:

| ファイル | 対象 | 想定実行時間 |
|---|---|---|
| `tests/eval/test_layout.py` | `sample_layout` (純粋関数) | < 1.0 s |
| `tests/eval/test_eval_config.py` | `load_eval_config` / `EvalConfig` / `PlacementConfig` | < 0.5 s |
| `tests/eval/test_place_blocks.py` | `place_blocks` (単体: 小さな MJCF、統合: 実シーン) | < 2.0 s |
| **合計** | | **< 4 s (目標。実測の内訳は §5.3)** |

信頼性レベル: 🔵 要件・設計・TASK-0006 の記述どおり / 🟡 文書からの妥当な推測 (要件書で確定済み) / 🔴 文書にない決定 (要件書 §6 の仮定に依存)

---

## 0. 共通事項

### 0.1 定数・ヘルパ (各テストファイルの冒頭に置く)

```python
EPS = 1e-9                       # 浮動小数の境界比較の許容差 (範囲判定・面積の上限判定)
TRIAL_SEEDS = range(20)          # 評価の試行シード (base_seed=0, n_trials=20)
SWEEP_SEEDS = range(100)         # TASK-0006 ケース2/3 のシード 0〜99
SETTLE_STEPS = 500               # 1 s (timestep 2 ms)。tests/test_scene.py と同じ
REST_TOL = 0.002                 # 静止高さの許容 2 mm。tests/test_scene.py と同じ
SPEED_TOL = 0.01                 # 静止判定 1 cm/s
MOVE_TOL = 0.005                 # 静止中の xy 移動 5 mm
```

- 範囲の境界は `SCENE_SPEC.work_area_x` / `work_area_y` からのみ取る (テストにも 0.15 などを直書きしない)。🔵
- **注意 (実測で確認済み)**: `0.35 - 0.15 == 0.19999999999999998` (float)。`area <= 作業範囲の幅` を素直に比較すると、既定値 `area=0.20` が「幅超え」で弾かれる。実装は許容差付き比較 (例: `area > width + 1e-9` で `ValueError`) が必要。境界値テスト TC-L-B03 がこれを検出する。🔵 (実測)
- 中心 `(cx, cy)` はテスト側でも `SCENE_SPEC` の中点から計算する (`cx = sum(work_area_x)/2` → 0.25、`cy` → 0.0)。

### 0.2 ディレクトリ

- `tests/eval/` を新設する。既存の `tests/robot/` に `__init__.py` がないので、同じく置かない (テストファイル名はリポジトリ内で一意)。🟡

---

## 1. `tests/eval/test_layout.py` (sample_layout)

### フィクスチャ

| 名前 | スコープ | 内容 |
|---|---|---|
| `sweep_layouts` | module | `{seed: sample_layout(seed) for seed in SWEEP_SEEDS}`。TC-L-05/06 で共有し、100 回の生成を 1 回だけにする |

### 1.1 正常系

#### TC-L-01 同じシード・同じ引数なら同じ配置 🔵
- **対応**: TASK-0006 テストケース1、NFR-01、要件書 2.1「決定性」
- **パラメータ**: `seed ∈ (0, 7, 42, 12345)`
- **Given**: シード `seed` / **When**: `sample_layout(seed)` を 2 回呼ぶ / **Then**: 2 つの戻り値が `==` で等しい
- **確認ポイント**: 2 回目の呼び出しが 1 回目の状態 (内部キャッシュ・グローバル RNG) に依存しないこと

#### TC-L-02 numpy 整数のシードは int と同じ結果 🟡
- **対応**: 要件書 2.1 表 (`numpy.integer` も可)、4.4
- **Given/When**: `sample_layout(np.int64(5))` と `sample_layout(5)` / **Then**: 等しい

#### TC-L-03 異なるシードで配置が変わる 🟡
- **対応**: 要件書 2.1「異なるシード」
- **Given/When**: `sample_layout(0)` と `sample_layout(1)` / **Then**: `!=`
- **確認ポイント**: シードが実際に RNG に渡っていること (固定値を返す実装を弾く)

#### TC-L-04 グローバル乱数状態を読まない・変えない 🔵
- **対応**: NFR-01、要件書 2.1「決定性」・3「再現性」
- **Given**: `np.random.seed(123)`、`random.seed(123)` の後で `np_state = np.random.get_state()`、`py_state = random.getstate()` を保存
- **When**: `sample_layout(7)` を呼ぶ
- **Then**:
  - `np.random.get_state()` の各要素が `np_state` と等しい (配列は `np.array_equal`)、`random.getstate() == py_state`
  - 別途 `np.random.seed(999)` の後に呼んだ `sample_layout(7)` が、`np.random.seed(123)` の後に呼んだものと等しい (グローバル状態を読んでいない)

#### TC-L-05 シード 0〜99 で全点が作業範囲内 🔵
- **対応**: TASK-0006 テストケース3、FR-12 (20 cm 四方)
- **Given**: `sweep_layouts` / **When**: 各配置の全点を走査
- **Then**: すべての `(x, y)` で `work_area_x[0] - EPS <= x <= work_area_x[1] + EPS` かつ y も同様 (境界は `SCENE_SPEC` から)
- **確認ポイント**: 失敗時はシードと点をメッセージに出す (`assert ..., f"seed={seed} p={p}"`)

#### TC-L-06 シード 0〜99 で全組の中心間距離 >= 0.05 🔵
- **対応**: TASK-0006 テストケース2、FR-12、tests/test_scene.py::test_blocks_are_at_least_5cm_apart と同じ判定
- **Given**: `sweep_layouts` / **When**: `itertools.combinations(layout, 2)` の全組で `math.dist` を計算
- **Then**: すべて `>= 0.05` (許容差なし。実装と同じく `>=` で比較)

#### TC-L-07 戻り値の型: Python float 2 要素 tuple の tuple、長さ n 🟡
- **対応**: 要件書 2.1「出力」、不変性ルール
- **パラメータ**: `n ∈ (1, 3, 5)` (シード 3)
- **Then**: `type(result) is tuple`、`len(result) == n`、各要素 `type(p) is tuple` かつ `len(p) == 2`、各座標 `type(v) is float`
- **確認ポイント**: `np.float64` は `float` のサブクラスで `isinstance` を通ってしまうため、`type(v) is float` で判定する

#### TC-L-08 評価の 20 試行 (シード 0〜19) がすべて成功し、互いに異なる 🟡
- **対応**: 要件書 2.2 (試行 i のシード = base_seed + i、既定 0〜19)
- **Given**: `cfg = load_eval_config()`、`p = cfg.placement`
- **When**: `seed ∈ range(cfg.base_seed, cfg.base_seed + cfg.n_trials)` で `sample_layout(seed, n=p.n_blocks, area=p.area, min_gap=p.min_gap, max_attempts=p.max_attempts)`
- **Then**: 例外なく 20 個得られ、`len(set(layouts)) == 20` (tuple なのでハッシュ可能)、各配置の長さが 3

#### TC-L-09 area を小さくすると中心 ± area/2 に収まる 🟡
- **対応**: 要件書 2.1「領域の定義」、4.2
- **Given**: `area=0.10, min_gap=0.03`、シード 0〜19
- **Then**: 全点が `cx ± 0.05`、`cy ± 0.05` (EPS 付き) の内側。`cx, cy` は `SCENE_SPEC` の中点から計算

#### TC-L-10 既定の area が作業範囲の幅に収まる (単一の情報源の整合) 🟡
- **対応**: note 4.1-2、要件書 4.2 最終項
- **Then**: `inspect.signature(sample_layout).parameters["area"].default <= min(幅x, 幅y) + EPS`、かつ `load_eval_config().placement.area` も同様
- **確認ポイント**: 作業範囲 (SCENE_SPEC) が将来狭められたときに検出する

### 1.2 異常系

#### TC-L-E01 不可能な条件 (5 cm 四方に 3 個、間隔 5 cm) は有限時間で ValueError 🟡
- **対応**: TASK-0006 テストケース4、要件書 2.1「棄却サンプリング」
- **入力**: `sample_layout(0, n=3, area=0.05, min_gap=0.05)` (max_attempts は既定 10,000)
- **不正な理由**: 一辺 5 cm の正方形 (対角 7.07 cm) に、互いに 5 cm 以上離れた 3 点はほぼ置けない (2 点は置けるため事前判定には掛からず、上限到達の経路を通る)
- **Then**: `pytest.raises(ValueError)`。`time.perf_counter()` で計測した所要時間が **2.0 s 未満** (実測は数十 ms の想定。CI の揺らぎを見込んだ緩い上限)
- **確認ポイント**: 無限ループしないこと (上限が全ブロック合計の候補数であること)

#### TC-L-E02 上限到達のメッセージに n・area・min_gap・試行回数が入る 🟡
- **対応**: 要件書 2.1「エラーメッセージ」、note 5.1 (ケース4 の期待値)
- **入力**: TC-L-E01 と同じ
- **Then**: `str(excinfo.value)` が部分文字列 `"n=3"`、`"area=0.05"`、`"min_gap=0.05"`、`"10000"` をすべて含む
- **備考**: `key=value` 形式は本書で決めた形式 (🔴 寄り)。Red で実装と合わせて確定する

#### TC-L-E03 軽い事前判定: min_gap > area·√2 かつ n >= 2 はサンプリング前に ValueError 🔴
- **対応**: 要件書 2.1「到達不能条件の扱い」(本書で決めた仮定 3)
- **入力**: `sample_layout(0, n=2, area=0.05, min_gap=0.08, max_attempts=10**9)` (0.08 > 0.0707)
- **Then**: `ValueError`、所要時間 **0.5 s 未満** (10^9 回引いていたら終わらない = 事前判定が効いている証拠)。メッセージに `"min_gap"` を含む

#### TC-L-E04 seed の不正 → ValueError 🔴
- **対応**: 要件書 2.1 表 (0 以上の int、bool 不可)、3「例外の型」
- **パラメータ**: `seed ∈ (-1, True, False, 1.5, "7", None, float("nan"))`
- **Then**: `pytest.raises(ValueError)` (TypeError ではない)

#### TC-L-E05 n の不正 → ValueError 🟡
- **パラメータ**: `n ∈ (0, -1, True, 1.5, None)` (シード 0)
- **Then**: `ValueError`

#### TC-L-E06 area の不正 → ValueError 🟡
- **パラメータ**: `area ∈ (0, 0.0, -0.1, nan, inf, -inf, None, "0.2", True, 0.201, 0.21)`
- **Then**: `ValueError`
- **確認ポイント**: `0.201` / `0.21` は作業範囲の幅 (0.20) 超え (要件書 4.3)。`True` は bool 不可 (1 として扱わない)

#### TC-L-E07 min_gap の不正 → ValueError 🟡
- **パラメータ**: `min_gap ∈ (-0.01, nan, inf, None, "0.05", True)`
- **Then**: `ValueError` (inf は「有限」の条件で弾く。🔴 `True` の扱いは本書の決定)

#### TC-L-E08 max_attempts の不正 → ValueError 🟡
- **パラメータ**: `max_attempts ∈ (0, -1, True, 1.5, None)`
- **Then**: `ValueError`

### 1.3 境界値

#### TC-L-B01 n=1 は常に成功し、事前判定は n>=2 のときだけ 🟡
- **入力**: `sample_layout(0, n=1)`、`sample_layout(0, n=1, area=0.05, min_gap=1.0)`
- **Then**: どちらも長さ 1 で範囲内 (1 点には間隔の制約がない)

#### TC-L-B02 min_gap=0 は成功 🟡
- **パラメータ**: `min_gap ∈ (0, 0.0)` (int の 0 も可)、n=3、シード 0
- **Then**: 長さ 3、範囲内

#### TC-L-B03 area = 作業範囲の幅ちょうど (0.20) は成功 🔵
- **入力**: `sample_layout(0, area=0.20)` (明示指定) と既定値
- **境界の意味**: x 方向の幅は float で `0.19999999999999998`。許容差なしの比較だと誤って `ValueError` になる (§0.1)
- **Then**: 例外なし。外側の `0.201` は TC-L-E06 で `ValueError`

#### TC-L-B04 max_attempts=1, n=1 は成功 🟡
- **Then**: 1 回目の候補は必ず採用される (要件書 4.4)

#### TC-L-B05 max_attempts=2, n=3 は ValueError 🟡
- **Then**: 3 個に最低 3 回の引きが必要なので上限到達 (max_attempts が全体合計であることの確認。要件書 4.4)

#### TC-L-B06 max_attempts=3, n=3, min_gap=0 は成功 🟡
- **Then**: 間隔の制約がなければ 3 回ちょうどで足りる (上限の数え方の off-by-one を検出)

---

## 2. `tests/eval/test_eval_config.py` (load_eval_config)

### フィクスチャ・ヘルパ

| 名前 | 種類 | 内容 |
|---|---|---|
| `VALID` | モジュール定数 (dict) | `{"n_trials": 20, "base_seed": 0, "placement": {"n_blocks": 3, "area": 0.20, "min_gap": 0.05, "max_attempts": 10000}}` |
| `_write(tmp_path, obj)` | 関数 | `yaml.safe_dump(obj)` を `tmp_path / "eval.yaml"` に書いてパスを返す。文字列を渡した場合はそのまま書く (不正 YAML 用) |
| `_with(path_keys, value)` / `_without(path_keys)` | 関数 | `copy.deepcopy(VALID)` を変更した新しい dict を返す (VALID は書き換えない) |

- 実ファイル `configs/eval.yaml` を読むのは TC-C-01/02 のみ。他はすべて `tmp_path` 上のファイル。🟡

### 2.1 正常系

#### TC-C-01 リポジトリの configs/eval.yaml の値 🔵/🟡
- **対応**: design 5、要件書 2.4
- **When**: `load_eval_config()`
- **Then**: `n_trials == 20`、`base_seed == 0`、`placement.n_blocks == 3`、`placement.area == pytest.approx(0.20)`、`placement.min_gap == pytest.approx(0.05)`、`placement.max_attempts == 10000`、`isinstance(cfg, EvalConfig)`、`isinstance(cfg.placement, PlacementConfig)`

#### TC-C-02 既定パスはリポジトリの configs/eval.yaml 🟡
- **Then**: `DEFAULT_EVAL_CONFIG_PATH == REPO_ROOT / "configs" / "eval.yaml"` (`so101_vlm.robot.model_paths.REPO_ROOT`)、`.is_file()`、`load_eval_config() == load_eval_config(DEFAULT_EVAL_CONFIG_PATH)`

#### TC-C-03 設定は frozen 🔵
- **対応**: 不変性ルール、要件書 3
- **Then**: `cfg.n_trials = 1` と `cfg.placement.area = 0.1` がそれぞれ `dataclasses.FrozenInstanceError`

#### TC-C-04 tmp_path の別の値を正しく読む (値の直書きを弾く) 🟡
- **入力**: `n_trials: 5, base_seed: 100, placement: {n_blocks: 2, area: 0.1, min_gap: 0.03, max_attempts: 50}`
- **Then**: 各フィールドが入力どおり。`load_eval_config(path)` は `Path` と `str` の両方を受け付ける (🔴 str の受け入れ)

#### TC-C-05 型の正規化: YAML の int を float フィールドに受け入れる 🔴
- **入力**: `placement.min_gap: 0` (int)
- **Then**: `cfg.placement.min_gap == 0.0` かつ `type(...) is float`。`n_trials` / `base_seed` / `n_blocks` / `max_attempts` は `type(...) is int`

### 2.2 異常系

#### TC-C-E01 ファイルがない → FileNotFoundError 🔴
- **入力**: `tmp_path / "missing.yaml"`
- **Then**: `FileNotFoundError` (ValueError に包まない、握りつぶさない)

#### TC-C-E02 未知のキー → ValueError (キー名入り) 🔴
- **パラメータ**: 最上位に `n_trial: 20` を追加 / `placement` に `min_gapp: 0.05` を追加
- **Then**: `pytest.raises(ValueError, match="n_trial")` / `match="min_gapp"`

#### TC-C-E03 必須キーの欠落 → ValueError (キー名入り) 🔴
- **パラメータ**: `n_trials`、`base_seed`、`placement`、`placement.n_blocks`、`placement.area`、`placement.min_gap`、`placement.max_attempts` (7 件)
- **Then**: `ValueError`、メッセージに欠けたキー名 (末尾の名前) を含む。既定値で補わない

#### TC-C-E04 型・値域の不正 → ValueError 🔴/🟡
- **パラメータ** (キー: 値):
  `n_trials: 0`、`n_trials: "20"`、`n_trials: 1.5`、`n_trials: true`、`base_seed: -1`、`base_seed: true`、
  `placement.n_blocks: 0`、`placement.area: "0.2"`、`placement.area: 0.3` (作業範囲超え)、`placement.area: .nan`、
  `placement.min_gap: -0.1`、`placement.max_attempts: 0`
- **Then**: `ValueError` (sample_layout と同じ規則。メッセージにキー名を含む)
- **備考**: `.nan` は YAML 文字列 `"area: .nan"` を書いて作る (safe_dump は nan を `.nan` で出力する)

#### TC-C-E05 mapping でない → ValueError 🔴
- **パラメータ** (ファイル本文): `"- 1\n- 2\n"` (list)、`"42\n"` (scalar)、`""` (空 → `None`)、placement が list (`placement: [1, 2]`)、placement が null (`placement:`)
- **Then**: `ValueError` (`AttributeError` / `TypeError` が漏れない)

#### TC-C-E06 危険な YAML タグは解釈しない (safe_load) 🔵
- **対応**: 要件書 2.4 (`yaml.safe_load` を使う)
- **入力**: `"!!python/object/apply:os.getcwd []\n"`
- **Then**: `pytest.raises((yaml.YAMLError, ValueError))` (safe_load なら `ConstructorError`)。関数呼び出しが実行されない

### 2.3 境界値

#### TC-C-B01 下限値ちょうど 🟡
- **入力**: `n_trials: 1, base_seed: 0, placement: {n_blocks: 1, area: 0.20, min_gap: 0, max_attempts: 1}`
- **Then**: 例外なく読める (`area: 0.20` は幅 0.19999999999999998 との比較で許容差が要る。TC-L-B03 と同じ)

---

## 3. `tests/eval/test_place_blocks.py` (place_blocks)

### 3.1 フィクスチャ

| 名前 | スコープ | 内容 |
|---|---|---|
| `tiny_model` | module | `mujoco.MjModel.from_xml_string(TINY_XML)`。ファイル不要で読み込みは 1 ms 未満 |
| `tiny_data` | function | `MjData(tiny_model)`。ロボット代わりの hinge を `qpos=0.7, qvel=0.3`、各ブロックの qvel を 0.1 に設定し `mj_forward` 済み |
| `scene_model` | module | `MjModel.from_xml_path(str(PICKPLACE_SCENE_PATH))` (実測 約 0.25 s。ファイル全体で 1 回だけ) |
| `settled_runs` | module | `TRIAL_SEEDS` の各シードで `MjData(scene_model)` → `place_blocks` → `xy0` を記録 → `mj_step` を 500 回。`{seed: (layout, xy0, data)}` を返す。シミュレーションは 20 シード × 1 回だけ |

```python
TINY_XML = """
<mujoco>
  <worldbody>
    <geom name="floor" type="plane" size="1 1 0.05"/>
    <body name="arm" pos="0 0 0.3">            <!-- ロボット関節の代わり。qpos の先頭に来る -->
      <joint name="arm_hinge" type="hinge" axis="0 0 1"/>
      <geom type="capsule" size="0.01 0.05"/>
    </body>
    <body name="block_red" pos="0.20 -0.06 0.015"><freejoint name="block_red_joint"/>
      <geom type="box" size="0.015 0.015 0.015" mass="0.02"/></body>
    <body name="block_blue" pos="0.30 0.00 0.015"><freejoint name="block_blue_joint"/>
      <geom type="box" size="0.015 0.015 0.015" mass="0.02"/></body>
    <body name="block_green" pos="0.22 0.07 0.015"><freejoint name="block_green_joint"/>
      <geom type="box" size="0.015 0.015 0.015" mass="0.02"/></body>
    <body name="box" pos="0.20 0.19 0"><geom type="box" size="0.045 0.045 0.0025"/></body>
  </worldbody>
</mujoco>
"""
LAYOUT = ((0.16, -0.09), (0.34, 0.01), (0.25, 0.09))   # 3 点とも XML の初期位置と異なる値
```

- hinge を先頭に置くことで、ブロックの qpos 先頭は 1, 8, 15 になる。添字の決め打ち (0, 7, 14) を検出する。🟡
- ヘルパ `_free_adr(model, name)` は `model.body(name).jntadr[0]` → `jnt_qposadr` / `jnt_dofadr` を返す (テスト側も名前で引く。sim.py と同じ方針)。

### 3.2 正常系 (単体: tiny_model)

#### TC-P-01 各ブロックの qpos 7 要素 = (x, y, rest_z, 1, 0, 0, 0) 🔵
- **When**: `place_blocks(tiny_model, tiny_data, LAYOUT)`
- **Then**: `block_names[i]` の qpos 7 要素が `[*LAYOUT[i], SCENE_SPEC.block_rest_z, 1, 0, 0, 0]` と `pytest.approx` で一致 (順序の対応も確認)

#### TC-P-02 各ブロックの qvel 6 要素が 0 🟡
- **Given**: 事前に 0.1 を入れておく / **Then**: `np.all(qvel[dof:dof+6] == 0)`

#### TC-P-03 ブロック以外の状態は変わらない・時刻を進めない 🟡
- **Then**: hinge の qpos == 0.7、qvel == 0.3、`data.time` が呼び出し前と同じ (`mj_step` を呼んでいない)

#### TC-P-04 mj_forward が呼ばれ xpos / xquat が配置を反映 🟡
- **Then**: 各ブロック body の `data.xpos[:2] == approx(LAYOUT[i])`、`xpos[2] == approx(rest_z)`、`data.xquat == approx([1, 0, 0, 0])`
- **確認ポイント**: qpos だけ書いて mj_forward を忘れた実装では xpos が XML の初期位置のまま

#### TC-P-05 戻り値は None、layout は変更しない 🟡
- **入力**: `layout = [[0.16, -0.09], [0.34, 0.01], [0.25, 0.09]]` (list の list)
- **Then**: 戻り値 `is None`、`layout` が呼び出し前の deepcopy と等しい

#### TC-P-06 block_names を指定すると、その body だけを書き換える 🟡
- **入力**: `place_blocks(m, d, ((0.18, 0.0), (0.30, -0.05)), block_names=("block_green", "block_red"))`
- **Then**: green と red が指定位置、`block_blue` の qpos / qvel は呼び出し前と同じ

### 3.3 異常系 (単体: tiny_model) — いずれも「data は一切変わらない」を確認

共通の確認: 呼び出し前に `qpos0 = d.qpos.copy()`、`qvel0 = d.qvel.copy()`、`xpos0 = d.xpos.copy()`。例外後に `np.array_equal` で 3 つとも不変。

#### TC-P-E01 layout の長さがブロック数と違う → ValueError 🟡
- **パラメータ**: `LAYOUT[:2]` (2 個)、`LAYOUT + ((0.2, 0.0),)` (4 個)
- **Then**: `ValueError`、メッセージに両方の長さ (`"2"` と `"3"` / `"4"` と `"3"`) を含む。data 不変

#### TC-P-E02 存在しない body 名 → ValueError (名前入り)、途中まで書かない 🟡
- **入力**: `block_names=("block_red", "block_purple", "block_green")` と 3 点
- **Then**: `pytest.raises(ValueError, match="block_purple")`。先頭の `block_red` も書き換わっていない (先に全部検証してから書く)

#### TC-P-E03 関節を持たない body (box) → ValueError 🔴
- **入力**: `block_names=("block_red", "block_blue", "box")`
- **Then**: `ValueError, match="box"`、data 不変

#### TC-P-E04 freejoint でない関節の body (hinge の arm) → ValueError 🔴
- **入力**: `block_names=("arm", "block_blue", "block_green")`
- **Then**: `ValueError, match="arm"`、data 不変 (hinge の qpos=0.7 のまま)

### 3.4 統合テスト (実シーン: scene_model / settled_runs)

#### TC-I-01 実シーンでもロボット関節は変わらず、ブロックだけが配置される 🟡
- **Given**: `MjData(scene_model)`、`mj_forward` 後の `JOINT_NAMES` (model_paths) の qpos を保存
- **When**: `place_blocks(scene_model, d, sample_layout(0))`
- **Then**: ロボット 6 関節の qpos が不変。3 ブロックの `xpos[:2]` が layout と一致

#### TC-I-02 配置後 1 秒で静止する (速度 < 1 cm/s、高さ = rest_z) 🟡
- **対応**: TASK-0006 統合テスト1、note 5.1
- **パラメータ**: `seed ∈ TRIAL_SEEDS` (20 件。`settled_runs` を参照するだけでシミュレーションはしない)
- **Then**: 各ブロックの並進速度 `np.linalg.norm(qvel[dof:dof+3]) < SPEED_TOL`、`|z - rest_z| <= REST_TOL`

#### TC-I-03 重ならず、ほとんど動かない (中心間 >= 3 cm、移動 < 5 mm) 🟡
- **パラメータ**: `seed ∈ TRIAL_SEEDS`
- **Then**: 全組の xy 中心間距離 `>= 2 * SCENE_SPEC.block_half_size` (0.03)、各ブロックの `‖xy_after - xy0‖ < MOVE_TOL`
- **確認ポイント**: 初期の間隔 5 cm に対して 3 cm は「接触していない」の下限。移動 5 mm 未満で、初期の押し合い・跳ねがないことを確認する

#### TC-I-04 箱と交差せず、ブロックは床とだけ接触している 🟡
- **パラメータ**: `seed ∈ TRIAL_SEEDS`
- **Then**:
  - 幾何: 各ブロックの xy 外形 (中心 ± block_half_size) と箱の外形 (`box_center_xy ± (box_inner_half_size + box_wall_thickness)`) が重ならない (SCENE_SPEC の値で計算。y 方向で 0.115 < 0.145)
  - 接触: `data.contact[:data.ncon]` のうち、ブロックの geom を含む接触の相手はすべて `floor` (箱・ロボット・他のブロックとの接触なし)
- **備考**: 接触の相手を床に限るのは本書の判定 (ロボットの home 姿勢はグリッパ z ≈ 0.23 m で作業面から離れていることを実測で確認済み) 🟡

---

## 4. 件数のまとめ

| ファイル | 正常系 | 異常系 | 境界値 | 統合 | 計 (ケース) | パラメータ展開後の概数 |
|---|---|---|---|---|---|---|
| test_layout.py | 10 | 8 | 6 | 0 | 24 | 約 75 |
| test_eval_config.py | 5 | 6 | 1 | 0 | 12 | 約 40 |
| test_place_blocks.py | 6 | 4 | 0 | 4 | 14 | 約 72 |
| **合計** | **21** | **18** | **7** | **4** | **50** | **約 187** |

信頼性の分布 (50 ケース): 🔵 13 (26%) / 🟡 27 (54%) / 🔴 10 (20%)。🔴 はすべて要件書 §6 の仮定 (型の扱い・設定の検証方針・事前判定・freejoint 以外の拒否) に対応する。

---

## 5. 開発言語・フレームワーク・実行時間

### 5.1 言語・フレームワーク 🔵
- **言語**: Python 3.12 (pyproject.toml)。numpy / mujoco / pyyaml は既存依存
- **テスト**: pytest (`uv run pytest tests/eval`)、`pytest.mark.parametrize` と module スコープのフィクスチャ。カバレッジは `uv run pytest --cov` (目標 80% 以上、layout.py / eval_config.py は 95% 以上を狙う)
- 追加プラグイン (pytest-timeout など) は入れない。時間の上限は `time.perf_counter` で TC-L-E01/E03 だけ確認する

### 5.2 命名
- 既存に合わせ `test_<対象>_<振る舞い>` (例: `test_sample_layout_is_deterministic_for_same_seed`、`test_place_blocks_rejects_unknown_body_without_writing`)。各テストの docstring かコメントに TC-ID を書く

### 5.3 実行時間の予算 (合計 < 4 s)
| 項目 | 見積り | 根拠 |
|---|---|---|
| sweep_layouts (100 回) + 他の sample_layout 呼び出し | < 0.3 s | 既定条件ではほぼ 1 回で採用される |
| TC-L-E01 (10,000 回の候補で上限到達) | < 0.2 s (上限 2 s で検査) | Python ループ 1 万回 |
| test_eval_config.py (tmp_path 約 40 ファイル) | < 0.3 s | |
| scene_model の読み込み (1 回) | 約 0.25 s | 実測 0.247 s |
| settled_runs (20 シード × 500 steps) | 約 0.4 s | 実測 500 steps = 0.016 s |
| tiny_model 関連 | < 0.1 s | |
| **合計** | **約 1.5 s (予算 4 s)** | |

- 実シーンの読み込みは `scene_model` の 1 回だけ。ステップ実行は `settled_runs` の 20 回だけで、TC-I-02〜04 は結果を読むだけにする。

---

## 6. テスト実装時の日本語コメント指針

```python
def test_sample_layout_is_deterministic_for_same_seed(seed):
    # 【テスト目的】: TC-L-01 同じシードで同じ配置になること (NFR-01)
    # 【テスト内容】: sample_layout を同じ引数で 2 回呼んで比較する
    # 【期待される動作】: 2 つの tuple が等しい
    # 🔵 TASK-0006 テストケース1

    # 【テストデータ準備】: パラメータのシード (0, 7, 42, 12345)
    # 【実際の処理実行】: sample_layout(seed) を 2 回
    first = sample_layout(seed)
    second = sample_layout(seed)

    # 【結果検証】: 完全一致 (許容差なし)
    assert first == second  # 【確認内容】: 再現性
```

- Given / When / Then の区切り、各 assert の【確認内容】、信頼性 (🔵🟡🔴) をコメントで残す。フィクスチャには【テスト前準備】を書く。

---

## 7. 要件定義との対応関係

| 要件書の項目 | テストケース |
|---|---|
| 2.1 決定性・グローバル RNG | TC-L-01, 02, 04 |
| 2.1 異なるシード / 2.2 試行シード | TC-L-03, 08 |
| 2.1 領域 (SCENE_SPEC の中点 ± area/2) | TC-L-05, 09, 10, B03, E06 |
| 2.1 間隔 (中心間 >= min_gap) | TC-L-06, B01, B02 |
| 2.1 出力の型 | TC-L-07 |
| 2.1 棄却サンプリング・上限・事前判定・メッセージ | TC-L-E01, E02, E03, B04, B05, B06 |
| 2.1 引数の検証 | TC-L-E04〜E08 |
| 2.3 place_blocks | TC-P-01〜06, TC-P-E01〜E04, TC-I-01 |
| 2.4 設定ファイル | TC-C-01〜05, TC-C-E01〜E06, TC-C-B01 |
| TASK-0006 統合テスト1 (重ならず静止) | TC-I-02, 03, 04 |
| 3 制約 (再現性・不変性・単一の情報源・例外の型) | TC-L-04, 07, 10, E04, TC-C-03, TC-P-05 |

- **参照した機能概要**: 要件書 §1
- **参照した入力・出力仕様**: 要件書 §2.1〜2.4
- **参照した制約条件**: 要件書 §3
- **参照した使用例**: 要件書 §4.2〜4.4

---

## 8. 未決事項 (Red フェーズで確定する)

1. **上限到達メッセージの書式** (TC-L-E02): `n=3, area=0.05, min_gap=0.05, max_attempts=10000` の `key=value` 形式を提案。🔴
2. **area 上限の許容差**: `width + 1e-9` を提案。0.201 は弾き、0.20 は通す。🟡
3. **`load_eval_config` が str パスを受け付けるか** (TC-C-04): 受け付ける (`Path(path)` に変換) を提案。🔴
4. **危険な YAML タグの例外型** (TC-C-E06): `yaml.YAMLError` をそのまま出すか `ValueError` に包むか未定のため、テストは両方を許す。
5. **layout に NaN / 範囲外の座標を渡したときの place_blocks**: 要件にないためテストしない (YAGNI)。
6. **TC-I-04 の「床とだけ接触」**: ロボットの姿勢を変える後続タスクでは成り立たない可能性がある。本タスクの home 姿勢に限った判定。

## 9. 品質判定

- テストケース分類: 正常系・異常系・境界値・統合をすべて定義 ✅
- 期待値: すべて具体値 (数値・例外型・部分文字列) で定義 ✅
- 技術選択: Python 3.12 + pytest、フィクスチャとパラメータを指定済み ✅
- 実装可能性: 既存依存のみ。実シーンの読み込み・ステップ時間を実測済み (予算内) ✅
- 信頼性: 🔵 26% / 🟡 54% / 🔴 20%。🟡🔴 は要件書で確定済みの決定に対応し、期待値は曖昧でない

**判定: ✅ 高品質 (実装可能)**。ただし 🔴 の項目は要件書の仮定に依存するため、仮定を変える場合はテストも合わせて変更する。

次のお勧めステップ: `/tsumiki:tdd-red so101-vlm-pickplace TASK-0006` でRedフェーズ（失敗テスト作成）を開始します。

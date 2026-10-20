# TASK-0006 開発コンテキストノート: ブロック初期配置の乱数生成

**作成日**: 2026-10-20
**要件名**: so101-vlm-pickplace
**タスクID**: TASK-0006 (TDD, 推定 1.5h)
**タスク定義**: docs/tasks/so101-vlm-pickplace/TASK-0006.md
**信頼性**: 🟡 (TASK-0006 の信頼性レベルサマリーに準拠。棄却サンプリング・不可能条件・シーン適用が推測)

---

## 1. 技術スタック

- 言語・実行環境: Python 3.12 (`requires-python = ">=3.12,<3.13"`)。パッケージ管理は uv、テストは pytest
- 乱数: numpy。`numpy.random.default_rng(seed)` で関数内に Generator を作る (グローバル RNG は使わない)
- シミュレータ: MuJoCo (`mujoco`)。ブロックは freejoint の body (`block_red` / `block_blue` / `block_green`)
- 設定: pyyaml (依存に含まれる)。設定ファイルは `configs/eval.yaml` (現時点では未作成。`configs/` は空)
- アーキテクチャ: 純粋関数 (`sample_layout`) と、MjData への適用関数を分ける。配置の結果は frozen な値として返す
- 参照元:
  - pyproject.toml
  - docs/design.md (4.7 シーンとカメラ, 4.8 Evaluator, 5 設定ファイル)
  - docs/requirements.md (FR-12, NFR-01)

## 2. 開発ルール

- ユーザーのグローバルルール (~/.claude/rules/ecc) を適用する
  - TDD: テストを先に書く (RED → GREEN → REFACTOR)。カバレッジ 80% 以上
  - 不変性: 戻り値は tuple で返し、入力や配列を書き換えない。副作用のある適用関数 (MjData 更新) だけは例外として明記する
  - 小さいファイル (目安 200〜400 行、上限 800 行)、小さい関数 (50 行未満)
  - エラーは握りつぶさず、どの値が原因か分かるメッセージにする
  - 設定値はコードに直書きせず `configs/eval.yaml` に置く
  - コミットは conventional commits 形式
- プロジェクト CLAUDE.md: テストは `uv run pytest` で実行する (現状 81 passed)
- AGENTS.md と docs/rule/ は存在しないため、追加ルールは無し
- 参照元: CLAUDE.md, docs/spec/so101-vlm-pickplace/note.md

## 3. 関連実装

- 寸法・名前の単一の情報源: `src/so101_vlm/eval/scene_spec.py`
  - `SCENE_SPEC.block_names` = `("block_red", "block_blue", "block_green")`
  - `SCENE_SPEC.block_half_size` = 0.015 (一辺 3 cm)
  - `SCENE_SPEC.block_rest_z` = 0.015 (床上の静止時の中心高さ)
  - `SCENE_SPEC.work_area_x` = (0.15, 0.35), `SCENE_SPEC.work_area_y` = (-0.10, 0.10) (20 cm 四方が 2 方向にある)
  - `SCENE_SPEC.box_center_xy` = (0.20, 0.19), `box_rim_z` = 0.04
- シーン定義: `assets/scene_pickplace.xml`
  - ブロックの初期位置は現在固定 (red (0.20, -0.06)、blue (0.30, 0.00)、green (0.22, 0.07)、いずれも z = 0.015)
  - 箱 body `box` は (0.20, 0.19)、壁の外形は約 x ∈ [0.155, 0.245]、y ∈ [0.145, 0.235]
  - 作業範囲の y ≤ 0.10 と箱の y ≥ 0.145 は重ならない (箱との衝突を避けられる)
- 参照元: src/so101_vlm/eval/scene_spec.py, assets/scene_pickplace.xml
- 物理シミュレーションの類似例: `src/so101_vlm/robot/sim.py`
  - 関節は名前で引く (`_name_id` で mj_name2id を使い、添字を決め打ちしない)。新しい関数でも同じ方針にする
  - `SimRobot` の `_model` / `_data` は非公開。配置の適用関数は model と data を引数で受け取る
- 参照元: src/so101_vlm/robot/sim.py

## 4. 設計文書

- FR-12: ブロックの初期配置は、ロボット前方の作業台上の 20 cm × 20 cm (暫定) に、ブロック同士を 5 cm 以上離してランダムに置く。箱の位置は v1 では固定
- NFR-01: 乱数シードを固定できる (再現性)
- design.md 4.7: ブロックの初期配置は、シードから 20 cm 四方内にランダムに生成し、5 cm 以上離す
- design.md 4.8: 試行ごとにシードを記録する。1 試行 = シード・指示・結果を記録する単位
- design.md 5: `configs/eval.yaml` に「試行数, シード, 配置範囲, 判定しきい値」を置く
- 参照元: docs/requirements.md, docs/design.md, docs/tasks/so101-vlm-pickplace/TASK-0006.md

### 4.1 本タスクで確定する設計 (実装前の決定事項)

1. **関数シグネチャ**
   - `sample_layout(seed: int, n: int = 3, area: float = 0.20, min_gap: float = 0.05, max_attempts: int = 10_000) -> tuple[tuple[float, float], ...]`
   - 戻り値は Python の float 2 要素 tuple の tuple (numpy 配列は返さない)
   - `area` は正方形の一辺 [m]。範囲の中心は `SCENE_SPEC` の作業範囲の中心 (x=0.25, y=0.00) とする
2. **範囲の単一の情報源**
   - 範囲の境界は `SCENE_SPEC.work_area_x` / `work_area_y` のみで持つ。`layout.py` に境界を複製しない
   - `area` (0.20) が作業範囲の幅に収まることをテストで確認する (作業範囲が変わったら検出できる)
3. **間隔の判定**
   - 「5 cm 以上離す」は、ブロック中心間の距離 >= `min_gap` とする (ユークリッド距離)
   - ブロックの半寸 1.5 cm を考えると、中心間 5 cm は辺同士の隙間 2 cm 以上に相当し、重ならない
   - 既存の `tests/test_scene.py::test_blocks_are_at_least_5cm_apart` と同じ判定基準
4. **棄却サンプリング**
   - 1 個ずつ一様に引き、既存の点すべてと `min_gap` 以上離れた場合だけ採用する
   - 試行上限 `max_attempts` (既定 10,000) に達したら `ValueError` を送出する。メッセージに n・area・min_gap・試行回数を入れる
   - 1 個目の再試行で全体をやり直す方式は採らない (上限の意味が曖昧になるため)
5. **乱数**
   - `rng = np.random.default_rng(seed)` を関数内で作る。`np.random.seed` やグローバル RNG は使わない
   - 同じ seed と同じ引数なら同じ結果になる (テストケース 1)
6. **入力の検証**
   - `n < 1`、`area <= 0`、`min_gap < 0`、`area` が作業範囲より大きい、`max_attempts < 1` は `ValueError` にする
   - 不可能な条件 (例: 5 cm 四方に 3 個で間隔 5 cm) は、サンプリング中に上限到達で `ValueError` になる。事前の幾何判定は入れない (YAGNI)
7. **MjData への適用**
   - `place_blocks(model, data, layout) -> None` を別関数として `layout.py` に置く (統合テスト用)
   - 各ブロックの freejoint の qpos (x, y, z, qw, qx, qy, qz) を設定する。z は `SCENE_SPEC.block_rest_z`、姿勢は単位クォータニオン (1, 0, 0, 0)
   - 関節名 `block_red_joint` などから `mj_name2id` で `jnt_qposadr` を引く。見つからなければ名前入りの `ValueError`
   - 適用後は `mujoco.mj_forward` を呼ぶ。速度 (qvel) は 0 にする
   - 配置の個数 `len(layout)` と `SCENE_SPEC.block_names` の個数が違う場合は `ValueError`
8. **設定ファイル** (`configs/eval.yaml`)
   - 本タスクで作成する項目: `placement.area: 0.20`、`placement.min_gap: 0.05`、`placement.n_blocks: 3`、`placement.max_attempts: 10000`
   - `area` と `min_gap` を `layout.py` に直書きしない。既定値は設定ファイルから読む経路を用意する (読み込み関数はテストで確認)
   - 設定の読み込みは sample_layout の外に置く (sample_layout は引数だけで決まる純粋関数)

### 4.2 信頼性に関する注記 (TASK-0006 の 🟡 項目)

- 棄却サンプリングの上限回数 (10,000) は推測。実測で 99 シード × 3 個がほぼ 1 回で通るかを確認して調整する
- 不可能条件のテストは「5 cm 四方、3 個、間隔 5 cm」を使う (TASK-0006 テストケース4)
- 統合テスト (シーン適用 → 数秒の物理シミュレーション → 静止・非接触) は時間がかかるため、既存の `settled` フィクスチャと同じく module スコープで 1 回だけ実行する

## 5. テスト関連情報

- テストフレームワーク: pytest。設定は `pyproject.toml` の `[tool.pytest.ini_options]` (`testpaths = ["tests"]`)
- カバレッジ: `[tool.coverage.run] source = ["so101_vlm"]`。`uv run pytest --cov` で測定
- 既存テストの配置:
  - `tests/test_scene.py` (TASK-0005。MjModel を module スコープで読み込む `settled` フィクスチャ、`SETTLE_STEPS = 500`、`REST_TOL = 0.002` を使う)
  - `tests/robot/test_sim.py`, `tests/robot/test_sweep.py` (TASK-0004)
  - `tests/test_assets.py`, `tests/test_package.py`
- 本タスクで作るテストファイル: `tests/eval/test_layout.py`
  - `tests/eval/` は未作成。`tests/robot/` と同様に `__init__.py` が必要かを既存の tests 構成で確認する (現状 `tests/robot/` に `__init__.py` は見当たらない)
- 既存の命名パターン: `test_<対象>_<振る舞い>` (例: `test_block_rests_on_floor`、`test_blocks_are_at_least_5cm_apart`)
- フィクスチャ・ヘルパ: `_body_id(model, name)` が `tests/test_scene.py` にあり、同じ考え方で joint id を引く関数を使う
- E2E テスト: 対象外 (Web 画面なし。CLAUDE.md で起動・ビルドは該当なし)
- 参照元: pyproject.toml, tests/test_scene.py, tests/robot/test_sim.py, CLAUDE.md

### 5.1 テストケース (TASK-0006 の定義に準拠)

| ID | 内容 | 信頼性 | ファイル |
|----|------|--------|----------|
| 1 | シード 7 で 2 回呼ぶと同じ配置 | 🔵 | tests/eval/test_layout.py |
| 2 | シード 0〜99 で、すべての組が 5 cm 以上離れている | 🔵 | tests/eval/test_layout.py |
| 3 | シード 0〜99 で、すべてが 20 cm 四方 (SCENE_SPEC の作業範囲内) に収まる | 🔵 | tests/eval/test_layout.py |
| 4 | 5 cm 四方に 3 個、間隔 5 cm で `ValueError` | 🟡 | tests/eval/test_layout.py |
| 5 | 戻り値が tuple で、要素は Python float の 2 要素 tuple | 🟡 (追加) | tests/eval/test_layout.py |
| 6 | 不正な引数 (n=0、area<=0、min_gap<0) で `ValueError` | 🟡 (追加) | tests/eval/test_layout.py |
| 7 | 別シードで配置が変わる (乱数が効いている) | 🟡 (追加) | tests/eval/test_layout.py |
| 8 | `area` が作業範囲より大きい場合は `ValueError` | 🟡 (追加) | tests/eval/test_layout.py |
| 9 | `place_blocks` 後、3 ブロックの qpos 位置が layout と一致し、z = block_rest_z | 🟡 | tests/eval/test_layout.py |
| 10 | 統合: 配置を適用して 1 秒 (500 steps) 進めると、ブロックが重ならず静止する (速度が小さい) | 🟡 | tests/eval/test_layout.py |
| 11 | `configs/eval.yaml` の値 (area 0.20、min_gap 0.05) を読み込める | 🟡 (追加) | tests/eval/test_layout.py |

- 統合テスト (10) の判定: 中心間距離 >= min_gap を維持し、1 秒後の各ブロックの速度ノルムが 1 cm/s 未満。
  - 「ブロックが重ならない」は中心間距離 >= 2 × block_half_size (3 cm) で判定する (接触していないことの下限)
- テストケース 4 の期待値: `ValueError` のメッセージに試行回数が含まれること

## 6. 注意事項

- **範囲の二重定義を避ける**: 作業範囲の境界は `SCENE_SPEC` のみ。`configs/eval.yaml` の area は「正方形の一辺」であり、境界値ではない
- **箱との衝突**: 作業範囲 y ≤ 0.10 と、箱の壁の外形 y ≥ 0.145 は重ならない。配置が箱に入ることはない (テストで確認する)
- **ブロックの初期配置の y 範囲**: 中心の範囲は [-0.10, 0.10]。ブロックの半寸 1.5 cm を含めると外形は [-0.115, 0.115] まで出る。範囲は「中心」の意味で運用する
- **乱数**: グローバル RNG を使わない。シードは呼び出し側が決め、試行の記録 (design 4.8) に残す
- **不変性**: `sample_layout` は入力・外部状態を変更しない。`place_blocks` だけが MjData を変更する (明示的な副作用)
- **シミュレーション時間**: 統合テストは 500 steps (1 秒、timestep 2 ms) を module スコープで 1 回だけ回す
- **ファイルサイズ**: `layout.py` は 200 行以内を目安にする
- **工数**: 1.5 時間 (AI 支援前提の本人の作業時間)。TASK-0013 (固定位置 pick & place) の前提
- **未確認事項** (実装時に確認): freejoint の qpos アドレスの取り方 (`jnt_qposadr`)、`configs/` の読み込み方式 (pyyaml の `safe_load`)

---

## 参照元 (まとめ)

- docs/tasks/so101-vlm-pickplace/TASK-0006.md
- docs/tasks/so101-vlm-pickplace/overview.md
- docs/spec/so101-vlm-pickplace/note.md
- docs/requirements.md (FR-12, NFR-01)
- docs/design.md (4.7, 4.8, 5)
- CLAUDE.md
- src/so101_vlm/eval/scene_spec.py
- assets/scene_pickplace.xml
- tests/test_scene.py
- src/so101_vlm/robot/sim.py
- pyproject.toml
- docs/implements/so101-vlm-pickplace/TASK-0005/ (前タスクの記録)

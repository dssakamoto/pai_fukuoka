# Plan: scenario-ground-truth

> タスクは `docs/tasks/scenario-ground-truth/`(SGT-001〜014。既存の TASK 番号とは別系列)。

## Requirements Summary

箱の位置を「定数」でなく試行ごとの「状態(Scenario)」として扱い、真値(GroundTruth)と方策が見る情報(Observation)を型で分離する。箱を動かしても、成功判定・oracle・固定位置デモ・VLM 版が追従する。箱固定の既定では、既存の評価結果(oracle 85%・OpenAI 80%)は変わらない。

詳細: [requirements.md](requirements.md) | [user-stories.md](user-stories.md) | [acceptance-criteria.md](acceptance-criteria.md)

### スコープの決定(ユーザー確認済み)

| 項目 | 決定 |
|---|---|
| 箱の自由度 | xy のみ。yaw は対象外(bbox 方式では yaw が取れない。将来課題) |
| 再現性 | 箱固定の既定で、同じ seed なら同じ配置・同じ成功率(テストで保証) |
| 実機の Evaluator | Protocol と sim 実装まで。実機側は Phase 5 以降の別 plan |
| 依存の向きの検査 | pytest(ast)。import-linter は使わない |
| 到達可能領域(FR-009) | xy 格子で IK を解き、一度だけ計算して `configs/reachable.yaml` に保存 |
| 掃引(FR-105) | 3x3 格子 x oracle のみ、各点 10 試行(API 料金なし) |
| ブロックの棄却(G1) | **(b) 箱だけを領域で棄却する(2026-11-21 に (a) から変更)**。(a) は「既定領域がブロック領域を覆う」前提だったが、IK の格子計算(CI で実測)で、ロボットの付け根の近く(x 0.15〜0.165 付近)に掴めない領域があり、前提が崩れた。ブロックの配置は従来どおり(乱数の消費順序も不変)。領域は「箱を置く位置を計画できるか(pre_place / place が workspace 内で IK 可)」だけを表す |
| SCENE_SPEC(G3) | 互換の薄い別名で残し、最後(SGT-013)に `box_center_xy` と `work_area` を削除 |

### 暫定採用(ユーザー未回答。推奨案を仮に採用)

| 項目 | 暫定の決定 | 影響するタスク |
|---|---|---|
| G2 Observation の置き場 | 既存 `policy/base.Observation` に任意項目(rgb / depth / instruction)を追加 | SGT-008 |
| G4 workspace の自動生成 | 自動生成しない。`motion.yaml` に値を残し、整合テストで食い違いを検知 | SGT-009, SGT-013 |
| G5 `Evaluator.evaluate` の署名 | `evaluate(colour)` のみ(領域は構築時に渡す) | SGT-008 |
| G6 `UniformXY` の範囲 | 箱の外周(内寸 + 壁)が到達可能領域内に収まる点に限る | SGT-010, SGT-012 |

## Design Overview

```
SceneGeometry (形状=定数。XML から)          eval/geometry.py
PlaceRegion   (置き先の領域)                 eval/geometry.py
Scenario      (試行ごとの配置=状態)           eval/scenario.py     ← sample_scenario(seed, ScenarioSpec(box=Fixed|UniformXY))
GroundTruth   (評価・デバッグ専用の真値)      eval/ground_truth.py ← 真値の唯一の窓口
Observation   (方策が見る情報)               policy/base.py       ← RGB・深度・指示・関節角のみ
Evaluator     (Protocol) / SimEvaluator      eval/evaluator.py
ReachableRegion (到達可能領域)               eval/reachable.py    ← configs/reachable.yaml
```

### データフロー(1 試行)

1. `sample_scenario(seed, spec)` → `Scenario`(blocks, box_xy)
2. `SimRobot` 生成 → `apply_scenario`(ブロック + 箱の `body_pos`)。箱は IkPolicy 構築と撮影の**前**に動かす
3. 撮影 → `Observation(rgb, depth, instruction, q)`(真値なし)
4. 方策: `vlm.plan` → `estimate_goal(plan, depth, cal, instruction, geometry)`(xy は枠から、z は形状から)
5. oracle のみ `OracleVlm(gt.layout, colour, cal, gt.place_region)`
6. `IkPolicy` → `run_goal` で実行(障害物はモデルから自動収集)
7. `SimEvaluator(...).evaluate(colour)` → `JudgeResult`

### 設計上の落とし穴(調査で判明)

- **今の時点で依存の向きの検査は失敗する**: `policy/base.py`, `policy/trajectory_config.py`, `perception/calibration.py` などが `eval.validation` を import している。→ SGT-001 で中立モジュールへ移す。
- **乱数の順序**: `sample_layout` は候補ごとに x, y の順に引く。箱は**最後**に引く。`Fixed` なら乱数を消費しないので既存 seed が変わらない。
- **障害物の識別に geom group は使えない**: group 4 以上は既定の描画に写らず、箱が撮影画像から消える。「自由度を持たない body の geom」で判定する。
- **真値が 7 か所で独立に再構築されている**: oracle / fixed_pick / trial / runner / nodes(pipeline, world, evaluator)。→ SGT-007 で `GroundTruth` に集約。
- **箱を動かすと workspace(y ≤ 0.26)の外に出うる**: `motion.yaml` の値と到達可能領域の整合テストで検知(SGT-009)。

## Task Dependency Graph

```
SGT-001 validation 移設 ──► SGT-002 依存の向きテスト
SGT-003 SceneGeometry/PlaceRegion ─┬─► SGT-004 judge 領域化 ──────────────┐
                               ├─► SGT-005 Scenario/sample_scenario ─► SGT-006 apply_scenario ─┬─► SGT-007 GroundTruth 窓口 ─► SGT-008 Observation/Evaluator
                               └─► SGT-009 ReachableRegion ──┐          │                    └─► SGT-011 障害物自動収集
                                  SGT-005 ──────────────────► SGT-010 UniformXY 生成
SGT-004 ─┐
SGT-006 ─┴─► SGT-007
SGT-007, SGT-010, SGT-011 ─► SGT-012 箱位置の掃引 ─► SGT-013 SCENE_SPEC 整理・再現性の最終確認 ─► SGT-014(任意)OpenAI 再確認
```

### 着手順(トポロジカル順の一例)

SGT-001 → SGT-002 → SGT-003 → SGT-004 → SGT-005 → SGT-006 → SGT-007 → SGT-008 → SGT-009 → SGT-010 → SGT-011 → SGT-012 → SGT-013 → SGT-014

並行できる組: {SGT-001→SGT-002} と {SGT-003}、{SGT-004} と {SGT-005→SGT-006}、{SGT-009} と {SGT-007→SGT-008}。

### 要件とタスクの対応

| 要件 | タスク |
|---|---|
| FR-001, 403 | SGT-003, SGT-013 |
| FR-002, 010, 011 | SGT-005, SGT-006, SGT-010 |
| FR-003, 402 | SGT-007 |
| FR-004, 007 | SGT-008 |
| FR-005, 006, 102, 202 | SGT-003, SGT-004, SGT-007 |
| FR-008 | SGT-011 |
| FR-009 | SGT-009 |
| FR-101, 201 | SGT-005, SGT-007, SGT-013 |
| FR-103, 104 | SGT-010 |
| FR-105, 301 | SGT-012 |
| FR-106, 401 | SGT-001, SGT-002 |
| FR-302 | SGT-014(任意) |

### 完了の目安(AC)

- 全タスク後も `uv run pytest -m "not render and not api" --cov` が全件成功、カバレッジ 80% 以上(NFR-001, 002)
- `run_eval.py --provider oracle --trials 20 --seed 0` が 85.0%(17/20)のまま(SGT-013)
- 箱位置の掃引表が `docs/notes/box-sweep.md` に載っている(SGT-012)

## Cross-Plan Dependencies

- 既存タスク管理 `docs/tasks/so101-vlm-pickplace/`(TASK-0001〜0049、旧 tsumiki 形式)とは別系統。番号は重ならない。
- Phase 5(実機準備)の `So101Robot` / 実機 Evaluator は、本 plan の `Evaluator` Protocol と `Observation` を前提にする。
- `docs/notes/provider-comparison.md` は、SGT-013 で追記するが、数値は変えない。
- 工数の目安: SGT-001〜SGT-002 が各 low、SGT-003〜SGT-008・SGT-011〜SGT-013 が medium〜high。実装は `tsumiki:task-exec` で SGT-001 から順に進める。

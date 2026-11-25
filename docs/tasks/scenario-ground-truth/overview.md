# scenario-ground-truth タスク概要

**作成日**: 2026-11-15
**計画名**: scenario-ground-truth(箱位置の一般化。既存の `so101-vlm-pickplace` の TASK-0001〜0049 とは別系列)
**番号**: SGT-001 〜 SGT-014(連番は本計画の中だけ。既存の TASK 番号とは独立)
**推定工数**: 28時間(AI 支援前提の本人の作業時間)。日程は未定(期日は考慮しない)
**総タスク数**: 14件

## 目的

箱の位置を「定数」でなく試行ごとの状態(Scenario)にし、真値(GroundTruth)と方策が見る情報(Observation)を型で分離する。箱を動かしても判定・oracle・デモ・VLM 版が追従し、箱固定の既定では既存の評価結果(oracle 85%・OpenAI 80%)が変わらない。

## 関連文書

- **要件定義**: [📋 requirements.md](../../spec/scenario-ground-truth/requirements.md)
- **ユーザーストーリー**: [📋 user-stories.md](../../spec/scenario-ground-truth/user-stories.md)
- **受入基準**: [📋 acceptance-criteria.md](../../spec/scenario-ground-truth/acceptance-criteria.md)
- **設計(plan)**: [📐 plan.md](../../spec/scenario-ground-truth/plan.md)(暫定採用の G2・G4・G5・G6 を含む)
- **コンテキスト**: [📝 context.md](../../dev/context.md)
- **既存のタスク**: [📋 so101-vlm-pickplace/overview.md](../so101-vlm-pickplace/overview.md)

## タスク番号管理

**使用済みタスク番号**: SGT-001 ~ SGT-014
**次回開始番号**: SGT-015

## 全体進捗

- [ ] SGT-001 〜 SGT-014(6/14 完了、SGT-007 は CI 確認中。SGT-001〜003 は 2026-11-15、SGT-004〜006 は 2026-11-21 に完了)

## タスク一覧

- [x] [SGT-001: 検証関数を中立モジュールへ移設する](SGT-001.md) - 1h (TDD) 🔵 ✅ 完了 (2026-11-15。コミット 2b915d3)
- [x] [SGT-002: 依存の向きを検査する pytest を追加する](SGT-002.md) - 1h (TDD) 🔵 ✅ 完了 (2026-11-15。コミット 80bb94a)
- [x] [SGT-003: SceneGeometry と PlaceRegion を XML から読めるようにする](SGT-003.md) - 2h (TDD) 🔵 ✅ 完了 (2026-11-15。コミット 19f8bb6)
- [x] [SGT-004: 成功判定を PlaceRegion 基準にする](SGT-004.md) - 2h (TDD) 🔵 ✅ 完了 (2026-11-21。コミット a389d1a、CI 成功)
- [x] [SGT-005: Scenario と sample_scenario(Fixed)を追加する](SGT-005.md) - 2h (TDD) 🔵 ✅ 完了 (2026-11-21。コミット a389d1a、CI 成功)
- [x] [SGT-006: apply_scenario でシミュレーションに配置を反映する](SGT-006.md) - 2h (TDD) 🔵 ✅ 完了 (2026-11-21。コミット a389d1a、CI 成功)
- [x] [SGT-007: GroundTruth を真値の唯一の窓口にして oracle・デモ・試行を載せ替える](SGT-007.md) - 3h (TDD) 🔵 ✅ 実装済み (2026-11-21。CI で確認中。world に箱の xy を載せた点が設計との差分)
- [x] [SGT-008: Observation を拡張し、Evaluator Protocol と SimEvaluator を追加する](SGT-008.md) - 2h (TDD) 🟡 ✅ 実装済み (2026-11-21。CI で確認中)
- [ ] [SGT-009: 到達可能領域を IK 格子で計算して保存し、読み込めるようにする](SGT-009.md) - 3h (TDD) 🟡 ✅ 実装済み (2026-11-21。configs/reachable.yaml は GitHub Actions の一時ワークフローで、実際の IkPolicy の計画から生成。CI で確認中)
- [ ] [SGT-010: 箱位置の UniformXY 生成と到達可能性による棄却を実装する](SGT-010.md) - 3h (TDD) 🔵 ✅ 実装済み (2026-11-21。G1 は (b): 箱だけ領域で棄却。箱とブロックの間隔は block_gap 0.03。CI で確認中。下記「実測で分かったこと」参照)
- [x] [SGT-011: 障害物 geom をモデルから自動収集する](SGT-011.md) - 2h (TDD) 🔵 ✅ 実装済み (2026-11-22。CI で確認中。motion.yaml は明示の一覧のまま、`obstacle_geoms: auto` で自動に切り替えられる移行形)
- [ ] [SGT-012: 箱位置の掃引(3x3 格子 x oracle)スクリプトを追加する](SGT-012.md) - 2h (DIRECT) 🟡 ✅ 完了 (2026-11-25。掃引を GitHub Actions(osmesa)で実行: 88/90 (98%)、全点で IK 失敗を除く成功率 90% 以上。結果は docs/notes/box-sweep.md。窓は x 0.18〜0.20、y 0.13〜0.18 と狭い)
- [ ] [SGT-013: SCENE_SPEC の箱位置・作業範囲を削除し、再現性を最終確認する](SGT-013.md) - 2h (DIRECT) 🔵
- [ ] [SGT-014: 実 API(OpenAI)で箱位置を変えた再評価を確認する(任意)](SGT-014.md) - 1h (DIRECT) 🟡

### 依存関係

```
SGT-001 → SGT-002
SGT-003 → SGT-004
SGT-003 → SGT-005
SGT-005 → SGT-006
SGT-004 → SGT-007
SGT-006 → SGT-007
SGT-004 → SGT-008
SGT-007 → SGT-008
SGT-003 → SGT-009
SGT-005 → SGT-010
SGT-009 → SGT-010
SGT-006 → SGT-011
SGT-007 → SGT-012
SGT-010 → SGT-012
SGT-011 → SGT-012
SGT-007 → SGT-013
SGT-010 → SGT-013
SGT-012 → SGT-013
SGT-013 → SGT-014
```


## 実測で分かったこと(SGT-009 / 010、2026-11-21。GitHub Actions で IkPolicy の計画を格子ごとに実行)

- **箱を置ける範囲は狭い**: 箱の外周(半幅 0.045 m)が全部収まる中心は x 0.17〜0.22、y 0.11〜0.20 の 37 点(1 cm 格子)だけ。既定位置 (0.20, 0.19) の周り約 5〜6 cm の窓。SGT-012 の「3x3 格子」は、この窓の中に取る(領域全体 x 0.12〜0.32、y −0.12〜0.25 の外周が収まる範囲)。
- **理由**: 安全フィルタの workspace(y ≤ 0.26)、IK、箱の壁とのすき間(1 mm)、ブロックの作業範囲と箱の共存。workspace は広げていない(安全の設定)。広げれば窓は広がるが、実機の安全に関わるので別判断。
- **G1 を (a) から (b) に変更**: 付け根の近く(x 0.15〜0.165)にブロックを掴めない領域があり、領域がブロックの作業範囲を覆えない。領域は「箱を置く位置を計画できるか」だけを表し、ブロックは従来どおり。
- **箱とブロックの間隔**: ブロック間の min_gap 0.05 をそのまま使うと、既定の箱 (0.20, 0.19) 自体が違反する(実際の間隔 0.03)。`UniformXY.block_gap` = 0.03(既定の配置と同じ)。100 seed すべてで箱を置ける。
- **領域は基準のブロック (0.25, 0) 1 つで計算した近似**: 中央に、基準ブロックの足元として箱を置けない穴がある。ブロックの作業範囲の内側なので、どのみち実ブロックと共存しにくい(保守的)。

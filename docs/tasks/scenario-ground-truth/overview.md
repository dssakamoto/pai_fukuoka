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
- [ ] [SGT-009: 到達可能領域を IK 格子で計算して保存し、読み込めるようにする](SGT-009.md) - 3h (TDD) 🟡 🟡 一部完了 (2026-11-21。reachable.py・compute_reachable.py・テストは実装済み。configs/reachable.yaml は MuJoCo が動く環境での生成待ち)
- [ ] [SGT-010: 箱位置の UniformXY 生成と到達可能性による棄却を実装する](SGT-010.md) - 3h (TDD) 🔵 🟡 一部完了 (2026-11-21。生成ロジックは実装・合成領域でテスト済み。保存済み yaml が前提の 2 項目 (G1 (a) の 100 seed 保証、IK 計画失敗 0 件) は reachable.yaml の生成待ち)
- [ ] [SGT-011: 障害物 geom をモデルから自動収集する](SGT-011.md) - 2h (TDD) 🔵
- [ ] [SGT-012: 箱位置の掃引(3x3 格子 x oracle)スクリプトを追加する](SGT-012.md) - 2h (DIRECT) 🟡
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

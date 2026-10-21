# TASK-0013 TDD 完全性検証記録: 固定位置での pick & place

**検証日**: 2026-10-22 / **判定**: OK(完全実装済み、スコープ外失敗なし)

## 1. テスト状況
- `uv run pytest -q`: 1399 passed(43 s)。`uv run pytest --cov=so101_vlm -q`: 1399 passed、TOTAL 1722 行 / 未カバー 0 = 100%。
- スコープ内外ともに失敗なし。30 秒超は 43 s(全体 1399 件。本タスク由来の遅いテストはなし、推奨事項なし)。

## 2. テストケース対応(pick-testcases.md)
- TC ID 93 件 すべてテストの docstring に対応(93/93)。MANUAL-01 はテストではなく 20 seed 掃引で、本検証で実施。
- grep で直接ヒットしない ID は範囲・併記の docstring でカバー: TC-FL-02〜07、09〜11(`TC-FL-01〜11` の parametrize、tests/eval/test_failure.py)、TC-CLI-04(`TC-CLI-01 / 04` の headless parametrize、tests/test_run_fixed_pick_script.py)。意図的な欠落なし。

## 3. 完了条件
1. 真の位置からブロックを箱に入れる: 満たす。赤 seed 0 で箱中心からのずれ (-0.0023, -0.0043) m、z 0.0199、速さ 4e-11 m/s、箱内寸 ±0.04 m・縁 0.04 m の内側。掃引で計画可能 58/58 成功。
2. 動画で保存: 満たす。`/private/tmp/claude-501/-Users-takuya-project-20-pai-homework/b14af3ef-30ce-4f56-a63b-8ac03b5a9e70/scratchpad/pick_verify/red_seed0.mp4`(265,372 B、161 フレーム、25 fps、6.44 s、640x480)。

## 4. 独立スポットチェック
- MANUAL-01(seed 0, 20 seed x 3 色): 60 試行、計画失敗 2(青 seed 16 self_collision/grasp、緑 seed 5 not_converged/pre_grasp)、成功 58/58。
- 別範囲(seed 100〜119 x 3 色): 計画失敗 4/60(6.7%: 青 105、緑 110・112・115。全て x 0.150〜0.170 で self_collision)、計画可能 56/56 成功(100%)。
- 動画フェーズ別の赤ブロック位置(別走行のログ): pre_grasp z 0.015(x 0.277, y -0.046) → close_hold で x 0.269 → lift/pre_place で z 0.0875(最大 0.0886)、y -0.040 → place で (0.191, 0.181, 0.080) → open 0.0255 → 最終 (0.198, 0.186, 0.0199)。持ち上げ・運搬・箱への解放を確認。
- フレーム: /private/tmp/claude-501/-Users-takuya-project-20-pai-homework/b14af3ef-30ce-4f56-a63b-8ac03b5a9e70/scratchpad/pick_verify/frame_000pct.png / frame_030pct.png / frame_050pct.png / frame_070pct.png / frame_100pct.png、運搬中 PNG: /private/tmp/claude-501/-Users-takuya-project-20-pai-homework/b14af3ef-30ce-4f56-a63b-8ac03b5a9e70/scratchpad/pick_verify/out/fixed_pick_red_seed0_carry.png。
- 物理設定の必要性: シーンから `<option cone="elliptic" impratio="10"/>` を除いた一時コピーで赤 seed 0 を実行 → 失敗、分類 GRASP_SLIP(判定: outside_inner、lift_end_rise 0.0013 m < 0.03)。オプションは必須。リポジトリのシーンは未変更。
- 堅牢性: 同一 seed 2 回(赤 0、緑 3)で結果 JSON(時間系を除く)が一致。計画失敗(青 16、緑 5)は frame_sink 未呼び出しで execution/judge/trace が None。成功試行のフラグ合計 0(58/58)。箱内外の独立判定(箱中心との差・縁高さ・速さを検証側で再計算)は judge と 58/58 一致、不一致 0。他ブロックの最大移動 2.17 mm(しきい値 5 mm 未満)。
- 既存挙動: 全テスト green(test_scene.py、TASK-0006/0012 統合を含む)。`load_eval_config()` は judge (0.0, 0.01, 1.0)、failure (0.03, 0.02, 0.005, 20.0) で文書の既定値と一致。

## 5. 未確認の決定事項(既定値で実装済み)
- O1 配置領域の x 下限: 0.15 のまま。計画失敗率 seed 0〜19: 3.3%(2/60)、seed 100〜119: 6.7%(4/60)(要件の見積り 6.0%)。分母から外して報告。0.17 への引き上げは TASK-0006 の範囲で、ユーザー判断待ち。
- O2 TASK-0014: 物理設定 1 行で 100% のため追加調整なし(「調整なし」の記録のみ)。本検証でも調整不要を確認。
- O3 `SimRobot.mujoco_handles()` の追加(sim 専用、Protocol 外)。
- O4 judge の margin_m = 0.0(FR-07 どおり)。
- O5 other_move_m 5 mm: 成功は取り消さず other_block_moved を記録(本検証の最大 2.17 mm)。
- O6 判定時点: 軌道終了から settle_s(1 s)後。TASK-0015 で正式化。
- 引き継ぎ: judge は mj_forward を呼ばず xpos を使う(約 2 ms 古い可能性、静止後は無視可)。

## 6. 後工程
修正が必要な項目なし。次は TASK-0014(調整なしの記録)。

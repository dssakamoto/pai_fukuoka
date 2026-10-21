# IK(逆運動学) TDD開発完了記録

## 確認すべきドキュメント

- `docs/tasks/so101-vlm-pickplace/TASK-0011.md`
- `docs/implements/so101-vlm-pickplace/TASK-0011/ik-requirements.md`
- `docs/implements/so101-vlm-pickplace/TASK-0011/ik-testcases.md`
- `docs/implements/so101-vlm-pickplace/TASK-0011/reachability-survey.md`

## 最終結果 (2026-10-20)

- **判定**: OK(完全実装済み、スコープ外失敗なし)
- **実装率**: 100% (テストケース T1〜T37 と T6b の 38 項目すべてに対応。うち T26 は共通補助 `_assert_valid_solution`、T37 は「判定しない記録」でコメントのみ。テスト関数 36 個 = `test_ik.py` 35 + `test_ik_survey.py` 1、展開後 63 ケース想定に一致)
- **テスト**: `uv run pytest -q` = 311 passed (4.5 s)、`tests/policy` = 64 passed、失敗 0
- **カバレッジ**: `uv run pytest --cov -q` で TOTAL 100% (`ik.py` 160 行、`ik_types.py` 26 行、未カバー 0)
- **TODO更新**: TASK-0011.md と overview.md に完了マークを追加

## 完了条件の確認

| 完了条件 | 結果 | 証拠 |
|---|---|---|
| 到達可能な目標で収束する | 達成 | T1(把持点 pos_error ≤ 2 mm、候補 0°)、T2、T36(代表 8 点)。独立スポット確認で格子 164 点すべて解け、最大位置誤差 0.99 mm |
| 到達できない目標で `IkError` になる | 達成 | T12(球外: `out_of_reach`)、T13・T14(球内だが届かない: `not_converged`)、T15、T20 |
| 結果が可動域内にある | 達成 | T3、T22、T26。スポット確認で全解が `ctrlrange` 内 |

## 要件項目の網羅

- 署名 `solve_ik(model, target_pos, seed_q, *, pitch_candidates_deg, tcp_offset)`: T8・T10・T6 で確認
- 解く 4 関節、roll・gripper は seed のコピー(範囲外はクリップ): T5、T22。スポット確認で全解 `q[4]`・`q[5]` が seed と一致
- 候補 (0,10,20,30,45,60)、許容 ±5°、指定順に試行、`POS_TOL_M`=1 mm、λ=0.01: T25、T26、T9
- `IkError` の属性 (`target`, `reason`, `best_pos_error`, `best_pitch_deg`, `tried_pitches_deg`): T12〜T15、T20
- 専用 MjData、入力・モデル・実行中 sim を変更しない: T27〜T30(後述のスポット確認 (2) でも確認)
- `IkSolution` は frozen、`q` は読み取り専用: T4
- 決定性(乱数なし): T31、T32(後述 (3) でプロセス間でも確認)
- 性能: T33〜T35

## 独立スポット確認 (使い捨てスクリプト、リポジトリ外)

スクリプト: `/private/tmp/claude-501/-Users-takuya-project-20-pai-homework/b14af3ef-30ce-4f56-a63b-8ac03b5a9e70/scratchpad/ik_verify/`

### (1) 密な格子
x 0.15〜0.35 (2.5 cm 刻み 9)、y −0.10〜0.10 (9) を z=0.015 と z=0.095、さらに箱 (0.20, 0.19) の z=0.06 と 0.14。計 164 点。

| seed | 解けた | IkError | 最大 pos_error | 最大の傾き偏差(採用候補との差) | 独立 FK(新 MjData + mj_kinematics)の最大誤差 |
|---|---|---|---|---|---|
| q=0 | 164 | 0 | 0.989 mm | 4.99° | 0.989 mm (< 2 mm) |
| ランダム roll (範囲内) | 164 | 0 | 0.991 mm | 5.00°(許容 5° 以内) | 0.991 mm (< 2 mm) |

全解が `ctrlrange` 内。

### (2) 実行中シミュレーションの隔離
`SimRobot`(pickplace シーン)を 5 ステップ動かした後、qpos・qvel・ctrl・time・act・qacc・xpos・接触距離を記録し、`solve_ik` を 20 回呼んだ後で完全一致を確認(unchanged: True、time 0.1 s 不変、ブロック z は約 0.0149 m で不変)。

### (3) プロセス間決定性
`PYTHONHASHSEED=1` と `999` の別プロセスで、18 点の解(q・傾き・候補・誤差)と `IkError` 属性のハッシュが一致(sha256 66e69a89…)。

### (4) 所要時間 (164 点)
- 中央値 3.4 ms (q=0) / 3.3 ms (roll seed)
- p95 76.9 ms / 84.3 ms、最大 167 ms / 115 ms
- 候補 0° で解ける点は数 ms。10° 以上の候補になる点は前の候補の失敗分で遅い(要件 3.6 の想定内)。軌道生成の経由点 1 点あたり p95 でも 0.1 s 未満

### (5) TASK-0012 向けの依存確認 (9×9 格子 81 点)
- 把持 (z=0.015) の採用候補: 0°:61、10°:11、20°:9 (q=0)
- 接近 (z=0.095) の採用候補: 0°:29、10°:30、20°:13、30°:9
- **接近点と把持点で既定の探索結果の候補が異なる格子点: 52/81** (roll seed でも 52/81)
- ただし各点を `pitch_candidates_deg=(p,)` の単一候補で解く方式にすると、**81 点すべてで共通の p が存在**した。最小の共通 p の内訳は 0°:29、10°:30、20°:13、30°:9(= 接近点側の候補と一致)。箱 (0.20, 0.19) の解放点 (z=0.06) と上方点 (z=0.14) は p=20° で共通
- 結論: 軌道生成は「接近点の候補 = 共通ピッチ」とし、`(p,)` を固定して両方を解けばよい。`solve_ik` を既定の候補列で別々に呼ぶと 52/81 点でピッチが食い違う

## 重要な技術学習

### 実装パターン
- 4 関節 DLS + 下向き成分 1 行の弱拘束 (重み 0.1)、候補は真下に近い順、固定の追加シード 9 個、位置誤差の停滞打ち切り(25 反復)
- 到達球の事前判定は roll を振った最大距離を含む保守的な上界(届く点を棄却しない)
- 専用 `MjData` を毎回生成。`_Arm` のキャッシュは見送り (構築 0.6 ms)

### テスト設計
- 独立 FK(別 MjData + `mj_kinematics`)で IK 内部の計算とは別に位置・傾きを確認
- 収束しない呼び出しは module fixture で 1 回に共有し、テスト時間を抑制
- 境界で揺れる点 (T37) は判定せず記録のみ

### 品質保証
- 到達性の回帰 (T36) をシーン定数 (`SCENE_SPEC.box_center_xy`) から組み立てた代表点で保持

## 注意点 (TASK-0012 向け)

- 接近点と把持点は同じピッチで解く: 接近点の採用候補 p を求め、`pitch_candidates_deg=(p,)` で把持点を解く(上のとおり全格子点で成立)
- ウォームスタートは接近点の解 → 把持点。`wrist_roll`・`gripper` は seed で決まるため軌道生成側で指定する
- `gripperframe` は把持中心ではない。`tcp_offset` の値は暫定 (TASK-0012/0013 で実測、`configs/motion.yaml` へ)
- IK は衝突・床の高さを見ない(安全フィルタ TASK-0010、シミュレーション TASK-0013 の役割)
- 関節空間の線形補間では手先が直線に動くとは限らない
- 傾きの許容は ±5° のため、採用候補 0° でも実傾きは最大 5° 程度(格子で 4.99〜5.00°)

## 後工程での修正対象

なし(テスト失敗 0、実装不足 0、スコープ外失敗 0)。

## テスト実行速度

総実行時間 4.5 秒(30 秒未満)、2 秒以上のテストなし。最遅は `test_scene.py` の setup 0.86 s。

# TASK-0011 Red フェーズ記録: IK(逆運動学 `solve_ik`)

**作成日**: 2026-10-20 / **フェーズ**: Red(失敗するテストのみ。実装・スタブなし)

## 作成したテストファイル

- `tests/policy/test_ik.py`(503 行、テスト関数 35 個。T1〜T35 と T6b。T26 は補助関数 `_assert_valid_solution` として各テストから使用)
- `tests/policy/test_ik_survey.py`(53 行、テスト関数 1 個 = T36、parametrize 8 点。T37 は判定せずモジュール docstring に記録)
- `tests/policy/__init__.py` は作らない(`tests/robot`、`tests/eval` と同じ)

## テスト設計

- fixture(module スコープ): `scene_model`、`ctrlrange`(名前で引いた (6,2))、`grasp_solution`、`near_high_error`(例外と所要時間を T13/T35 で共有)。
- 独立 FK `fk_point`: 別の `MjData` + `mj_kinematics` で `gripperframe + R @ offset` と傾き(site +x と世界 -z の角)を計算。
- 時間予算: out_of_reach < 20 ms、候補 0° で解ける点は 3 回の最小値 < 50 ms、not_converged < 3 s。
- 信頼性: 🔵 T3, T12 / 🔴 T18, T22, T23, T24 / 他は 🟡(ik-testcases.md 11 節のとおり)。

## 失敗の内容(期待どおり)

`uv run pytest tests/policy -q` → 2 errors during collection。
`ModuleNotFoundError: No module named 'so101_vlm.policy.ik'`(両ファイルとも)。正しい理由での失敗。
既存テスト: `uv run pytest -q --ignore=tests/policy` → 247 passed。

## 未決事項の決定

| ID | 決定 |
|---|---|
| O1 | 停滞打ち切りは Green の責務。`test_not_converged_within_3s` の 3 秒予算を維持し、Green で実装する |
| O2 | 50 ms 判定は候補 0° で解ける点(`GRASP_CENTER`)のみ、3 回の最小値で判定 |
| O3 | `slow` マーカーは登録しない。Red では実測不可のため未マーク。Green 後に `test_ik_survey.py` が 5 秒を超えたら再検討 |
| O4 | `tcp_offset` のテストはテスト専用定数 `TCP_OFFSET` を使用 |

## Green で実装すべきこと

- `src/so101_vlm/policy/ik.py`(`__init__.py` は既存の `src/so101_vlm/policy` を確認)に `solve_ik`、`IkSolution`(frozen、`q` は読み取り専用 float64)、`IkError`(`RuntimeError` 子、属性 target/reason/best_pos_error/best_pitch_deg/tried_pitches_deg)。
- 定数: `PITCH_CANDIDATES_DEG`、`PITCH_TOL_DEG=5.0`、`POS_TOL_M=0.001`、`DAMPING`、`APPROACH_AXIS_LOCAL=(1,0,0)` ほか(要件 3.1)。テストが import するのは `APPROACH_AXIS_LOCAL, DAMPING, PITCH_CANDIDATES_DEG, PITCH_TOL_DEG, POS_TOL_M, IkError, IkSolution, solve_ik`。
- 方式: wrist_roll 固定の 4 関節 DLS、下向き成分 1 行拘束、ctrlrange クリップ、到達球の事前判定、固定追加シード、停滞打ち切り(O1)、専用 MjData、入力検証(ValueError)。

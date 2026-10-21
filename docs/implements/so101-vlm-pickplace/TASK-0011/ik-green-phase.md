# TASK-0011 Green フェーズ記録: IK (solve_ik)

**実装日**: 2026-10-20 / **要件名**: so101-vlm-pickplace / **機能名**: ik

## 実装ファイル
- `src/so101_vlm/policy/ik_types.py` (61 行): `IkSolution`(frozen、q は読み取り専用コピー)、`IkError`(target/reason/best_pos_error/best_pitch_deg/tried_pitches_deg)
- `src/so101_vlm/policy/ik.py` (242 行): `solve_ik` と定数、内部クラス `_Arm`、`_run_dls` など

## 実装方針 (参照ソルバの移植、ik-requirements.md 準拠)
- DLS (λ=0.01)、1 反復の更新量上限 0.2 rad、300 反復、傾きの重み 0.1、収束 1 mm かつ |傾き−候補| ≤ 5°。 🔵
- 解く関節は pan/lift/elbow/wrist_flex の 4 つ。wrist_roll・gripper は seed(ctrlrange にクリップ)をコピー。 🟡
- 接近軸は gripperframe +x。傾きは下向き成分 1 行のみ拘束。tcp_offset は局所座標(`mj_jac` を offset 点で評価)。 🟡
- 専用 `MjData`、毎反復 ctrlrange でクリップ、乱数なし、追加シードは固定 9 個(pan は目標方位角に置換)。 🟡
- 到達球の事前判定: 到達半径はモデルから実行時に計算(lift→elbow→wrist_flex→site の距離和 = 0.41147 m)+ |tcp_offset|。 🟡
- 停滞打ち切り: 位置誤差が 25 反復で 0.1 mm 以上改善しなければ次のシード/候補へ。 🟡
- 公開名: テストが `PITCH_CANDIDATES_DEG` を import するためこれを正とし、`DEFAULT_PITCH_CANDIDATES_DEG` を別名で提供。

## テスト結果
- `uv run pytest -q`: 311 passed (4.7 s)
- `tests/policy`: 64 passed (2.68 s)。`test_ik.py` 56 passed (1.70 s)、`test_ik_survey.py` 8 passed (1.03 s)
- not_converged 点 (0.10, 0, 0.095): 約 0.21 秒(予算 3 s)。out_of_reach は 20 ms 未満。
- カバレッジ: `ik.py` 100%、`ik_types.py` 100%(全体 100%)
- テストの変更: なし。

## 品質判定: ✅ 高品質
全テスト成功、ファイル 800 行以下、実装コードにモック・スタブなし。

## 課題・Refactor 候補
- `_reach_radius` の docstring が「roll 軸ずれ」に触れているが実際は加算していない(参照の 0.4115 と同等)。docstring 修正か保守側の加算を検討。
- `_Arm` は呼び出しごとに生成(モデル参照・ctrlrange 取得のみで軽量)。頻繁に呼ぶならキャッシュ化を検討。
- `_run_dls` / `solve_ik` の引数が多い。小さな dataclass(問題定義)にまとめる余地。
- 停滞判定は位置誤差のみ。傾きの収束が遅いケースで早期に切る可能性があるが、現テストでは問題なし。
- 外縁判定は誤差 0.4115 の丸めではなく計算値を使うため、境界ごく近傍で参照と 0.03 mm 差。

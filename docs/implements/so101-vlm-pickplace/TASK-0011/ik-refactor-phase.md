# TASK-0011 IK Refactor フェーズ記録

対象: `src/so101_vlm/policy/ik.py` (299 行), `src/so101_vlm/policy/ik_types.py` (61 行, 変更なし)

## 改善内容

1. 🔵 `_reach_radius` の docstring とコードの不一致を解消 (コード側を保守的に修正)
   - 実測 (qpos0): roll を振ると wrist_flex アンカー→サイト距離は 160.5 mm → 161.3 mm に伸びる。
     旧実装 (roll 無視) は到達半径が約 0.9 mm 小さく、roll が任意の seed では届く点を
     out_of_reach と誤判定しうる非保守側だった。
   - 新 `_max_wrist_to_site`: 円周上の最遠点 sqrt(h^2 + (d_w + d_s)^2) の解析値を使う (緩い上界ではなく厳密値)。
     半径 0.41147 → 0.41234 m。
   - 試行: 「サイトの軸ずれ x 2」の緩い上界は `test_just_outside_reach_raises_out_of_reach_quickly[+x]` が
     not_converged になり失敗したため不採用。厳密値では全 64 テスト green、ik-requirements.md 3 節
     (保守的な判定、届く点を外と判定しない) とも整合。
2. 🟡 `_Arm` キャッシュは見送り: 構築は約 0.6 ms (solve 1 回 約 4 ms の 15%)。MjModel は可変
   (ctrlrange 等を書き換え可能) で、id/weakref キーのキャッシュは古い可動域を返す危険があり、
   live-sim 不変・決定性の保証を弱めるため毎回構築する。理由は `solve_ik` 内コメントに記載。
3. 🔵 frozen dataclass `_Problem(arm, data, target, offset)` を追加し、`_dls_step` (6→3 引数)、
   `_run_dls` (6→3 引数) を簡素化。`data` は専用の作業バッファである旨を docstring に記載。
4. 🔵 停滞判定が位置誤差のみを見ることを定数コメントと `_run_dls` docstring に明記
   (閾値は既存の `STAGNATION_WINDOW` / `STAGNATION_MIN_GAIN_M`)。
5. 🔵 魔法数の添字を `IDX_PAN` / `IDX_LIFT` / `IDX_ROLL` に置換、未使用の `IDX_GRIPPER` を削除。
   private 関数・メソッドに型ヒントと日本語 docstring を追加。

## セキュリティ/性能レビュー

- 入力は `_validate_inputs` でコピー・検証済み。入力・モデル・実行中 sim は変更しない。グローバル状態・乱数・秘密情報なし。
- 計算量: 候補 6 x シード 10 x 最大 300 反復 (停滞で早期終了)。早期棄却は O(1)。性能問題なし。

## テスト結果

- `uv run pytest -q`: 311 passed (5.8 s)
- `uv run pytest --cov -q`: 311 passed、ik.py 100% / ik_types.py 100% / TOTAL 100%
- `uv run pytest tests/policy -q`: 64 passed (2.9 s、最遅 0.43 s、2 秒超なし)
- テストは無変更。

## 品質判定: 高品質

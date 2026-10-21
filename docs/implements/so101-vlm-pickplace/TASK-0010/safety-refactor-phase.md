# TASK-0010 安全フィルタ Refactor フェーズ記録

## 結果サマリ

- `uv run pytest -q`: 724 passed (6.6 s) / tests/safety: 255 passed (件数不変)
- カバレッジ: src/so101_vlm/safety 全ファイル 100%、eval/validation.py・eval/eval_config.py も 100%、TOTAL 100%
- 動作変更なし。テストの期待値は変更していない。

## 1. limit_step の浮動小数補正の検証 (🔵 実測)

`_within_budget` は `np.nextafter` で束縛関節の指令を q_current 側へ 1 ulp 戻す。ハックではなく妥当と判断し、現行のまま残した。

- 理由: `current + α·Δ` の丸めで、下流が `np.abs(cmd - prev)` で再計算した値が m を 1 ulp 超える (0.020000000000000004 > 0.02)。制限した場合の約半数で起きる。補正は 1 ulp なので方向のずれは 1e-15 未満。
- 代案 `α·(1-eps)` は方向を 1e-16 ずらし、丸めで再び超えうるため不採用。再計算した差を直接確かめる現方式は偽の合格が起きない。
- 乱数検証 (スクラッチパッドの throwaway スクリプト、seed 固定、q_current・q_target・vmax・scale・dt をばらつかせた 20,000 件):

| 項目 | 結果 |
|------|------|
| 速度制限が働いた件数 | 15,178 |
| 再計算 `abs(cmd - prev)` が m を超えた関節 | 0 件 (最大 d/m = 1.0) |
| 方向余弦 < 1 - 1e-9 | 0 件 (最小 0.9999999999999996) |
| 制限しないとき q_target と不一致 (alpha == 1) | 0 件 |
| 別試行 10 万件での補正回数 | 最大 1 回 (補正が必要だったのは約 49%) |

- `_MAX_ULP_ADJUST = 4` は余裕分。`_within_budget` の docstring に理由と検証結果を追記。

## 2. テスト定数の集約 (🔵)

`tests/safety/safety_constants.py` を新設し、VMAX・DT・LOWER_SYN・UPPER_SYN・WS_*・EXACT_ABS・BOX・DEFAULT_CFG・Z6・NAN/INF・`_arr` を集約。test_limits.py と test_apply_safety.py から重複定義と不要 import を削除。assert は変更なし。M_STEP・CTRL_* など片方だけが使うものは各ファイルに残した。

## 3. 'exact keys' 検証の共通化 (🟡)

`eval/validation.py` に公開関数 `require_exact_keys` を追加し、`eval/eval_config.py` の `_require_mapping` と `safety/config.py` の `_require_exact_keys` を置き換えた。safety は既に `eval.validation.require_number` に依存しており、新しい依存方向は増えない。メッセージは同一。

## 4. 二重検証の除去 (🔵)

`_validate_structure` が検証済みの `lo, up` を返し、`_filter_command` はそれを受け取る (内部で `_validated_limits` を呼ばない)。`_filter_command` は `apply_safety` からのみ呼ばれ、手順 1 で必ず検証済みのため冗長だった。異常入力のテスト (TC-AS / TC-IN) は全て通過。

## 5. 品質・セキュリティ確認

- 関数は全て 50 行未満、ファイルは最大 limits.py 245 行 (テストは最大 531 行)。
- 入力は `as_vector` で必ずコピー、`SafetyResult` は読み取り専用コピー。グローバル可変状態・乱数・時計なし。
- YAML は `yaml.safe_load` のみ。秘密情報なし。
- 型は mypy / ruff が環境に無く未実行 (型ヒントは既存どおり)。

## 意図的に残したもの

- TRY004 相当 (型違反に ValueError) と `dict()` 呼び出しは既存方針のまま。
- `limits.py` の `__all__` 再エクスポートは維持。
- test_limits.py (531 行) は 800 行未満のため分割しない。

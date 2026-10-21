# TASK-0010 Greenフェーズ記録: 安全フィルタ

**作成日**: 2026-10-21
**入力**: `safety-requirements.md` / `safety-testcases.md` / `safety-red-phase.md`

## 実装したファイル

| ファイル | 行数 | 内容 |
|---|---|---|
| `configs/motion.yaml` | 28 | `safety:` セクション(各値の日本語コメント。🔴 仮定: vmax は STS3215 の速度が資料にない、state_timeout は値の資料なし。実機の初回 speed_scale 0.3 は別の設定から与える) |
| `src/so101_vlm/safety/_validation.py` | 54 | 共通検証: `as_vector`(bool・shape・NaN/inf)、`require_positive`、`require_fraction`、`require_positive_vector`。`eval.validation.require_number` を流用 |
| `src/so101_vlm/safety/config.py` | 118 | `WorkspaceBox`、`SafetyConfig`(frozen、`__post_init__` で検証・tuple 化)、`DEFAULT_MOTION_CONFIG_PATH`、`load_safety_config`(`yaml.safe_load`、`safety:` と `workspace:` のキーは過不足なし、他の最上位キーは無視) |
| `src/so101_vlm/safety/workspace.py` | 47 | `is_in_workspace`(閉区間、Python bool)、`check_state_fresh`(NaN/inf/逆行は False、bool は ValueError) |
| `src/so101_vlm/safety/limits.py` | 231 | `clip_to_limits`、`limit_step`(一様スケーリング)、`limits_from_model`、`SafetyResult`、`REASON_*`、`apply_safety`。workspace.py の 2 関数を再エクスポート |
| `src/so101_vlm/safety/__init__.py` | 35 | 公開名の再エクスポート |

モック・スタブ・インメモリ代替は実装コードに無い。全ファイル 800 行未満、全関数 50 行未満。

## 実装方針と判断

- `limit_step`: 許容変化量 `m = vmax x scale x dt`。`|Δ_i| > m_i` の関節があれば、`α = min(m_i/|Δ_i|)` で Δ 全体を縮める。`|Δ_i| = m_i` ちょうどは制限しない。制限しなければ `q_target` のコピーをそのまま返す(α == 1 で完全一致)。
- **丸め補正(実装で追加した点)**: `q_current + α·Δ` の浮動小数の丸めで、テストが再計算した `|cmd - prev|` が `m` を 1 ulp 超えることがあった(0.020000000000000004 > 0.02、TC-IN-01)。`np.clip` の後に `_within_budget` で `np.nextafter` により q_current 側へ戻して、許容差なしで `m` 以下を保証した(最大 4 回、通常 1 回)。
- `apply_safety`: 構造検証 → 停止 → 値検証 → 拒否 → 可動域クリップ → 速度制限 → 最終クリップ。`_validate_structure` / `_validate_values` / `_hold` / `_filter_command` に分割。`rate_limited` は `limit_step` 内の「厳密な `|Δ| > m`」判定から返す。
- 検証は `eval/validation.py` の `require_number`(bool 除外・有限)を流用。キーの過不足検査は `eval_config._require_mapping` が private のため `config.py` に同等の `_require_exact_keys` を置いた(Refactor で共通化を検討)。

## 実行結果

```
uv run pytest tests/safety -q --durations=10   -> 255 passed in 1.32〜1.48s
uv run pytest -q                                -> 724 passed in 7.56s (既存 469 + safety 255)
uv run pytest --cov=so101_vlm.safety            -> safety 5 ファイル 100% (219 文)
```

最も遅いのは setup の `limits_from_model` 系 0.3 s、sim 結合テスト 0.1 s 程度。

## テストの変更

- 期待値・テスト本体の変更なし。
- 純粋な移動のみ: `tests/safety/test_limits.py`(938 行)から TC-AS / TC-IN を `tests/safety/test_apply_safety.py`(437 行)へ移した(test_limits.py は 535 行)。各ファイルの docstring・import・共通定数(VMAX, DT, LOWER_SYN など)を必要分だけ置き直した。移動前後で件数は同じ(255)。元ファイルの控えはスクラッチパッドに保存。

## 課題・改善点(Refactorで対応)

- 定数(VMAX, DT, LOWER_SYN, ...)と `_arr` が 2 テストファイルで重複。conftest.py 等へ集約を検討。
- `_require_exact_keys` と `eval_config._require_mapping` の重複。
- ruff の TRY004(型違反に TypeError)は既存コードと同じ ValueError 方針のため未対応。テストの C408(`dict()` 呼び出し)は既存のまま。
- `apply_safety` の `_filter_command` で `lower` / `upper` を検証済みでも再検証している(軽微)。
- 信頼性: 🔵 式・順序・既存方式 / 🟡 一様スケーリング・丸め補正・検証細部 / 🔴 vmax・state_timeout の数値(設定ファイル側)。

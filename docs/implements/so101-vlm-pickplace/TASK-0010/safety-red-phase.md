# TASK-0010 Redフェーズ記録: 安全フィルタ

**作成日**: 2026-10-21
**入力**: `safety-requirements.md` / `safety-testcases.md`(124 ケース)

## 作成したテストファイル

| ファイル | 内容 | 関数数 |
|---|---|---|
| `tests/safety/conftest.py` | `task_points`(session、SCENE_SPEC から 172 点を生成)、`scene_model` / `pickplace_model`(module) | fixture 3 |
| `tests/safety/test_limits.py` | TC-CL / ST / WS / FR / MD / AS / IN(95 関数、約 940 行) | 95 |
| `tests/safety/test_config.py` | TC-SC / LC / WC(29 関数) | 29 |

テストケース定義の 124 ID はすべて docstring の `TC-xx-nn` で網羅(機械的に照合済み)。parametrize 展開後の件数は実装後に確定する。
`tests/safety/__init__.py` は置いていない(既存テストと同じ)。

## 実行結果(Red)

```
uv run pytest tests/safety -q
ERROR tests/safety/test_config.py  -> ModuleNotFoundError: No module named 'so101_vlm.safety.config'
ERROR tests/safety/test_limits.py  -> ModuleNotFoundError: No module named 'so101_vlm.safety.config'
```

期待どおり、実装が無いため収集段階で失敗する(`so101_vlm.safety.limits` / `config` が未作成)。
既存テスト: `uv run pytest -q --ignore=tests/safety` -> 469 passed。

## Green で実装すること

- `src/so101_vlm/safety/config.py`: `WorkspaceBox`、`SafetyConfig`(`__post_init__` で検証、tuple 化、frozen)、`DEFAULT_MOTION_CONFIG_PATH`、`load_safety_config`(`yaml.safe_load`、欠落・未知キーは ValueError、`safety:` 以外の最上位キーは無視、`FileNotFoundError` はパス入り)。
- `src/so101_vlm/safety/limits.py`: `clip_to_limits`、`limit_step`(一様スケーリング、`|Δ| = m` は制限しない、α = 1 は `q_target` のコピー)、`is_in_workspace`(閉区間、`bool` を返す)、`check_state_fresh`、`limits_from_model`(名前で引く)、`apply_safety`(キーワード専用、手順 1〜7)、`SafetyResult`(frozen、q_command は読み取り専用 float64)、`REASON_STALE_STATE`、`REASON_OUT_OF_WORKSPACE`。
- `configs/motion.yaml`: `safety:` セクション(requirements 5 節)。TC-LC-01 / 02 と TC-WC が依存する。
- エラーメッセージには引数名(`q`、`q_target`、`vmax`、`scale`、`dt`、`timeout`、`p_base` など)を含める。`q` は単語境界で照合(`\bq\b`)。

## 信頼性

🔵 約 31% / 🟡 約 69% / 🔴 0%(テストケース定義と同じ)。

## テスト設計上の注記

- SimRobot 結合は包絡・平均速度・保持量の許容差で確認し、毎周期の計測速度 <= vmax は主張しない。
- TC-AS-33 の入力不変は parametrize 3 件で実施。

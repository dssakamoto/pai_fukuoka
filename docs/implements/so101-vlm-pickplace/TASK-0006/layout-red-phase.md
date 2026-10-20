# TASK-0006 Red フェーズ記録: ブロック初期配置の乱数生成 (layout)

**作成日**: 2026-10-20
**要件名**: so101-vlm-pickplace / **タスクID**: TASK-0006 / **機能名**: layout

## 1. 作成したテストファイル

| ファイル | テスト関数数 | 対象 |
|---|---|---|
| `tests/eval/test_layout.py` | 24 | `sample_layout` (TC-L-01〜10, E01〜E08, B01〜B06) |
| `tests/eval/test_eval_config.py` | 12 | `load_eval_config` / `EvalConfig` / `PlacementConfig` (TC-C-01〜05, E01〜E06, B01) |
| `tests/eval/test_place_blocks.py` | 14 | `place_blocks` (TC-P-01〜06, E01〜E04、統合 TC-I-01〜04) |

合計 50 関数 (テストケース定義と 1 対 1)。パラメータ展開後は約 190 件。`tests/eval/` に `__init__.py` は置かない (tests/robot に合わせる)。
フィクスチャ: module スコープの `scene_model` / `settled_runs` (シード 0〜19 を 1 回だけ 500 step)、`sweep_layouts` (シード 0〜99)、`tiny_model` (先頭に hinge を持つ小 MJCF)。
信頼性: 🔵 約 26% / 🟡 約 54% / 🔴 約 20% (定義書どおり)。

## 2. 未決事項の確定

| 項目 | 決定 |
|---|---|
| 上限到達メッセージ | `key=value` 形式。`n=3`, `area=0.05`, `min_gap=0.05`, `10000` を含める |
| area 上限の許容差 | `1e-9` (`area > width + 1e-9` で ValueError。0.20 は通り 0.201 は弾く) |
| `load_eval_config` の引数 | `str` と `Path` の両方を受け付ける |
| 危険な YAML タグ | `yaml.YAMLError` または `ValueError` のどちらでも可 |
| TC-I-04 の接触判定 | ロボットが home 姿勢のときに限り、ブロックの接触相手は床のみ |
| `place_blocks` の置き場所 | `src/so101_vlm/eval/layout.py` (要件書 2.3) |
| 設定の検証エラー | メッセージに該当キー名の末尾 (例 `area`) を含める (TC-C-E03/E04) |

## 3. 実行結果

```
uv run pytest tests/eval -q
ERROR tests/eval/test_eval_config.py / test_layout.py / test_place_blocks.py
ModuleNotFoundError: No module named 'so101_vlm.eval.layout' (eval_config も同様)
3 errors during collection
uv run pytest -q --ignore=tests/eval  ->  81 passed
```

失敗理由は想定どおり (未実装モジュールの collection エラー)。既存 81 件は成功。

## 4. Green フェーズで実装すべき内容

1. `src/so101_vlm/eval/layout.py`: `sample_layout(seed, n=3, area=0.20, min_gap=0.05, max_attempts=10_000)` と `place_blocks(model, data, layout, block_names=SCENE_SPEC.block_names)`。
   - `default_rng(seed)` のみ使用、棄却サンプリング (max_attempts は全体合計)、事前判定 `n>=2 かつ min_gap > area*sqrt(2)`、引数検証は全て ValueError (bool/nan/inf/None/str を弾く、seed は numpy.integer 可)。
   - 戻り値は float の tuple の tuple。領域は SCENE_SPEC の中点 ± area/2。
   - `place_blocks` は全検証後に書く (長さ、body 名、freejoint)。qpos=(x,y,rest_z,1,0,0,0)、qvel=0、`mj_forward` のみ。
2. `src/so101_vlm/eval/eval_config.py`: frozen dataclass `PlacementConfig` / `EvalConfig`、`DEFAULT_EVAL_CONFIG_PATH = REPO_ROOT / "configs" / "eval.yaml"`、`load_eval_config(path)` (safe_load、未知/欠落キー、型・値域検証)。
3. `configs/eval.yaml`: n_trials=20, base_seed=0, placement(n_blocks=3, area=0.20, min_gap=0.05, max_attempts=10000)。

# TASK-0009 Red フェーズ記録: sim 用キャリブレーションの自動生成(calibration)

**作成日**: 2026-10-21
**要件名**: so101-vlm-pickplace / **タスクID**: TASK-0009 / **機能名**: calibration
**参照**: `calibration-requirements.md`、`calibration-testcases.md`、`note.md`
**信頼性レベル**: 🔵 資料どおり / 🟡 妥当な推測 / 🔴 資料にない決定(各テストのドックストリング末尾に記載)

## 1. 作成したテストファイル

| ファイル | 内容 | テスト関数数(parametrize 展開前) | 行数 |
|---|---|---|---|
| `tests/perception/test_calibration.py` | C / T / D / F / Y / L / K / R / B と各異常系(描画なし) | 92 | 約 1080 |
| `tests/perception/test_calibration_scene.py` | I-01〜I-04(描画あり、コミット済み YAML 使用、しきい値 2 mm、既定配置 + seed 0〜9) | 4 | 150 |
| `tests/perception/test_generate_calibration.py` | G-01〜G-12、G-E01、G-E02(`main(argv)` 直接呼び出し + サブプロセス 1 組) | 14 | 184 |

- 合計 110 関数。parametrize 展開後は 300 件超になる見込み(L-E11 が 4 キー x 8 値など)。
- 信頼性の分布はテストケース定義書 7 章(🔵 28 / 🟡 54 / 🔴 34)と同じ。
- `test_calibration.py` は 500 行の目標を超えた(ケース数が多く、定義書の 400 行目安も超過)。3 ファイル構成の指示を優先した。Refactor で `test_calibration_io.py`(Y / L / B の読み書き)への分割を検討する。

## 2. 実行コマンドと期待される失敗

```bash
uv run pytest tests/perception -q --ignore=tests/perception/test_projection.py \
  --ignore=tests/perception/test_projection_scene.py --ignore=tests/perception/test_projection_extra.py
```

結果(Red): 3 ファイルが収集時に失敗(3 errors during collection)。

- `test_calibration.py` / `test_calibration_scene.py` / `test_generate_calibration.py`: `ModuleNotFoundError: No module named 'so101_vlm.perception.calibration'`。
- 実装後に次の理由でも失敗する(Green で解消): `scripts/generate_calibration.py` が存在しない(`script` フィクスチャ・G-12)、`configs/calibration.yaml` が存在しない(R-01〜R-03、L-02、I-01〜I-04、G-01、G-04)。
- 既存テスト: `uv run pytest -q --ignore=<新規 3 ファイル>` で 724 件すべて通過。

## 3. Green フェーズで実装すること

1. `src/so101_vlm/perception/calibration.py`: `camera_from_mujoco`、`t_base_cam_from_mujoco`、`Calibration`(frozen, eq=False, 読み取り専用コピー)、`calibration_from_mujoco`、`save_calibration`(SafeDumper サブクラスで行だけ flow、`sort_keys=False`、先頭コメント、numpy 型は Python 型へ変換)、`load_calibration`(厳格な検証)、`calibrations_close`(`atol=1e-9` キーワード専用)、`DEFAULT_CALIBRATION_PATH`。
2. `scripts/generate_calibration.py`: `main(argv) -> int`(`--out`、`--check`、`--force`。終了コード 0 / 1 / 2)。
3. `configs/calibration.yaml`: スクリプトで生成してコミット。
4. 既存の `projection.py` は変更しない(MuJoCo 非依存のまま)。

## 4. 決定と定義書からの逸脱

- **逸脱1(指示による)**: `tests/perception/test_projection_scene.py` は変更していない。定義書の「テスト専用ヘルパ `camera_and_T_base_cam` を `calibration_from_mujoco` に置き換える」は、Green 実装前に通過中のテストを壊すため **Refactor ステップへ延期**する。S-01 のドックストリングの書き換えも同時に行う。
- **逸脱2(軽微)**: Y-01 のゴールデンは、定義書の Given(`sim_calibration`)ではなく、要件 2.6 の実測リテラルから作った `Calibration` を使う(MuJoCo の版差に依存させないため)。実シーンの値との一致は T-03 / R-01 / R-02 / G-01 が担う。
- **逸脱3(軽微)**: `test_calibration.py` に L-02 の `load_calibration()` 既定パス読み込みを含めるため、`configs/calibration.yaml` がない間は L-02 も失敗する(定義書どおり)。
- **O-T2**: `--check` で既存が `source: real` のときは終了コード 1(G-10)。定義書どおり。2 にする場合は G-10 の期待を変える。
- **O-T3**: T-E05(別モデルの data → `ValueError`)は定義書どおり残した。tiny_model(ncam=4)と scene_data(cam_xpos 1 件)の組み合わせで、配列の大きさ照合を要求する。実装が難しければ Green で削除してよい(削除したら verify-complete に記録する)。
- **O-T4**: 統合テストのフィクスチャは `conftest.py` に移していない(各ファイルに定義)。共有は Refactor で判断する。
- **O-T5**: C-E06 は配列の書き換えで `fovy=0/180/-10/nan` を作る。MuJoCo がモデル配列を読み取り専用にした場合は削除する。
- G-05 の差の書式は緩い正規表現 `[0-9.]+e-0?[67]` で確認(差が `9.99e-07` や `1.0000000000287557e-06` でも通る)。
- 統合テストの `cal` は `source == "real"` のとき skip、ドリフトガード(R-)も同様。

## 5. 品質判定

- テスト実行: 収集段階で失敗することを確認(✅)。
- 期待値: 数値・例外型・終了コードは定義書どおり具体的(✅)。
- 実装方針: 要件 2 章と Green 項目が明確(✅)。
- 信頼性: 🔵 28 / 🟡 54 / 🔴 34(🔴は要件の決定事項の検証)。
- 判定: ✅ 高品質(行数超過のみ要改善として Refactor に申し送り)。

次のお勧めステップ: `/tsumiki:tdd-green so101-vlm-pickplace TASK-0009`

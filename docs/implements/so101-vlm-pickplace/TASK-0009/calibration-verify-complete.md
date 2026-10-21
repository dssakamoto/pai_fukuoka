# TASK-0009 TDD開発完了記録: sim 用キャリブレーションの自動生成(calibration)

**検証日**: 2026-10-21
**要件名**: so101-vlm-pickplace
**判定**: OK(完全実装済み・スコープ外失敗なし)

## 確認すべきドキュメント

- `docs/tasks/so101-vlm-pickplace/TASK-0009.md`
- `docs/implements/so101-vlm-pickplace/TASK-0009/calibration-requirements.md`
- `docs/implements/so101-vlm-pickplace/TASK-0009/calibration-testcases.md`

## 最終結果

- **テストケース対応**: 定義 116 件のうち 115 件がテストのドックストリング(または併記 ID)に対応。残る B-09 はテストではなく「統合テストの最悪値の余裕」の観察メモ(seed 0〜9 の最大 1.153 mm に対し 2 mm)で、意図的にテスト化していない。
  - C-E03、L-E05、L-E06、B-01、B-06、B-08 は「C-E02 / C-E03」「L-E03 / L-E05 / L-E06」「C-04 / B-01」などの併記ドックストリングで対応。
  - 定義書にない ID を使うテストはなし(grep で一度 K-00 が出たのは `TASK-0009` の部分文字列の誤検出)。
- **テスト**: `uv run pytest -q` 979 passed(14 秒前後)。calibration 関連(test_calibration*.py、test_generate_calibration.py)は 255 passed。
- **カバレッジ**: `uv run pytest --cov -q` 全体 932 行、未カバー 0、100%(`calibration.py` 131 行も 100%)。
- **遅いテスト**: 最長は `test_script_runs_as_subprocess_from_other_cwd` の 1.37 秒。2 秒超なし、総実行時間も 30 秒未満。
- **スコープ外の失敗**: なし。

## 完了条件

| 条件 | 判定 | 根拠 |
|---|---|---|
| calibration.yaml が生成される | 達成 | `configs/calibration.yaml`(source: sim)。`generate_calibration.py --check` が終了コード 0、`--out` で再生成した別ファイルとバイト一致 |
| 描画画像から真の位置を復元し誤差を記録 | 達成 | 下記(1)。50 配置 150 個で最大 1.235 mm(5 mm 以内、自動テストの 2 mm 以内も満たす) |

## 独立スポットチェック(使い捨てスクリプトは scratchpad の calib_verify 配下)

1. **真の位置の復元**(コミット済み YAML を `load_calibration` で読み、seed 0〜49、静定 300 ステップ、上面 4 隅の bbox、shrink=0.5、表面点の z から 0.015 を引いてブロック中心にする): 150 個で平均 0.495 mm、p95 1.042 mm、最大 1.235 mm(水平方向の最大も 1.235 mm)。5 mm に対し約 4 倍の余裕。
   - 注意: `baseframe` サイトはワールド原点・単位回転のため、この確認では「ベース座標 = ワールド座標」の違いは検出できない。
2. **YAML のバイト列**:
   - `--check` は終了コード 0。
   - `--out` で別パスに生成した結果は `cmp` でバイト一致。
   - 最後の桁だけの変更(差 2e-16)や 1e-14 の変更は許容差 1e-9 内のため `--check` が 0。これは仕様どおり(DEFAULT_ATOL)。
   - 許容差を超える変更は `--check` が 1: 平行移動 0.45 を 0.46 に変更(差 1e-2)、回転要素の 1e-7 の変更(差 1e-7)。
   - 回転を直交から外す変更(0.64 を 0.74 に変更)は `--check` が 2(読み込み時の検証で拒否)。
   - 一時ファイルを `source: real` にして `--force` なしで再生成すると終了コード 2 で、ファイルは未変更(SHA 一致)。`--force` を付けると上書きされ `source: sim` に戻る。
3. **往復の厳密性**: 読み込んで別パスへ保存した結果は元ファイルとバイト一致。`T_base_cam` は MuJoCo のモデル・データ(`cam_xmat` に OpenCV 規約の `diag(1,-1,-1)` を掛け、`baseframe` の逆変換を適用)から独立に再計算した値とビット完全一致(最大差 0.0)。`fy` も `(480/2)/tan(30°)` と差 0.0(fovy=60°)。
4. **ファイル内容**: 先頭 3 行が自動生成コメント(手編集禁止・再生成・確認コマンド)。キー順は version、metadata、camera、t_base_cam。`version: 1`、`metadata.source: sim`、camera_name `cam_top`、base_frame `baseframe`。camera は fx、fy、cx、cy、width、height。t_base_cam は 4x4 を 1 行ずつの flow 形式で保存。

## 未確定事項(要件定義書 8 章)と採用した既定

- **OPEN-1**(実機のレンズ歪み係数): YAML に入れない。sim は歪みなし。TASK-0036 で必要なら `version: 2` で追加する。`camera.k1` などの未知キーは読み込みで拒否する。
- **OPEN-2**(統合テストの配置数): 既定配置 3 個 + seed 0〜9 の 10 配置(約 3 秒)。50 配置(150 個)の結果は本書の記録のみ。
- **OPEN-3**(sim と real の置き場所): 同じ `configs/calibration.yaml` を差し替える(design.md 4.4)。別ファイルが必要になれば `--out` と `load_calibration(path)` で対応できる。real の上書きは `--force` が必要。
- **OPEN-4**(しきい値): 完了条件は 5 mm のまま、自動テストは 2 mm 以下(実測最大 1.235 mm)。要件定義書の本文は「テストのしきい値は 5 mm のまま」と読めるが、テストケース定義書で 2 mm に決定済み(呼び出し元の指示)。要件定義書側の記述が古いままなので、利用者に確認してもらうとよい。
- 🔴 テスト設計の未決(O-T1〜O-T5): `--check` の不一致メッセージの書式は緩い正規表現で判定。`--check` で既存が `source: real` のときは終了コード 1(O-T2)。その他は実装時に決定済み。

## 重要な技術学習

- 実装パターン: MuJoCo の `cam_xmat`(OpenGL 規約)に `diag(1,-1,-1)` を掛けて OpenCV 規約にする。数値は丸めず、YAML の往復をビット一致に保つ(`1e-3` が文字列になる YAML の罠を避けるため、小数点を含む表記で出力)。
- テスト設計: コミット済み YAML を使う統合テスト + ドリフトガード(`calibrations_close`、atol 1e-9)で、「コミットした値で位置が復元できる」ことと「シーン定義との乖離」を同時に検出する。
- 品質保証: `--check` の許容差は MuJoCo の版差による末尾の桁の揺れを吸収するためのもの。バイト一致を要求する R-02 / G-01 は版差で落ちうるので、落ちた場合は R-01 だけを残す(テストケース定義書 8.1)。

## 後工程での修正対象

なし(テスト失敗・実装不足・品質改善の必須項目はなし)。

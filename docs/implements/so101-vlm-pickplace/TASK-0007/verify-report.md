# TASK-0007 設定確認・動作テスト (verify-report)

- **タスク**: dora-rs の最小動作確認 (DIRECT)
- **実行日**: 2026-10-20 / **実行者**: Claude (direct-verify)
- **総合判定**: 合格 (軽微な修正 1 件)

## 1. dora run の再実行
- `dora run dataflow.yml` を Python subprocess (timeout 120s) で実行。**exit code 0**。
- viewer: `small_rgb 10件 / big_rgb 20件 / robot_state 10件`、`OK: 全配列が送信内容と一致 (shape/dtype/値)`。
- 遅延 median: small 0.51 ms, big 0.53 ms (max 0.65), robot_state 0.75 ms。NumPy->Arrow 0.07 ms。
- 実行後に dora の daemon / coordinator プロセスなし (`ps`)。
- ログ: scratchpad `dora_min/verify_run.log`

## 2. 検証が本物であることの確認 (故意の破損)
- リポジトリ外のコピーで、送信側が 480x640x3 画像の 1 画素 (`frame.flat[1000] ^= 1`) を変える版を実行。
- 結果: viewer が `NG: errors=['big_rgb#0 不一致', ... ]` を出力し **exit 1**。比較は 1 画素の破損も検出できる。
- 対照: 無改変コピーを同じ手順で実行 -> `OK`、exit 0。
- 補足: 最初の破損版実行は、dora が PATH 上の miniforge の python を選んでしまい `ModuleNotFoundError: dora` で失敗した (破損検出ではない) ため無効とし、`VIRTUAL_ENV` も設定して再実行した結果が上記。リポジトリ内 (`experiments/dora_min` で README の手順) では venv の python が選ばれ正常動作する。ただし、別ディレクトリのデータフローでは `VIRTUAL_ENV` の設定も要る点は TASK-0025 で注意。

## 3. ノードスクリプト
- camera_dummy: 送信 30 回で自分で `break`。robot_dummy: `STOP` / `INPUT_CLOSED` で `break`。viewer: 期待件数が揃うと終了、`STOP` でも終了、不一致時 exit 1。viewer は `INPUT_CLOSED` を明示処理しないが、入力が閉じればイベントループが終わるため問題ない。
- 絶対パス、API キー、秘密情報なし (grep で確認)。各ファイル 1 KB 前後。

## 4. 0001-dataflow.md の主張の検証
| 主張 | 結果 |
|---|---|
| dora-rs-cli / dora-rs 1.0.1、numpy 2.5.4、pyarrow 26.0.0、Python 3.12 | 確認 (`dora --version`=1.0.1、importlib.metadata) |
| 480x640x3 遅延 中央値 0.45〜0.56 ms | 今回 0.53 ms で範囲内 |
| NumPy->Arrow 中央値 0.07 ms | 確認 |
| 関節ベクトル往復 中央値 0.6 ms | 今回 0.75 ms (やや高い。「約 1 ms 未満」の結論は変わらない) |
| 最初の 1 件 57〜65 ms | 今回 small max 78 ms / robot_state max 69 ms (初回接続の遅れ。傾向は一致) |
| 全体 4.1 秒、user 0.98s + sys 0.36s、メモリ約 111 MB、初回約 9 秒 | 今回は未計測 (setup 時の値。検証不能として扱う) |
| `dora graph --mermaid` が動く | 今回は未再確認 (setup 時の記録を信頼) |
| 実行後にプロセスが残らない | 確認 |

## 5. リポジトリ衛生
- `pyproject.toml` / `uv.lock` は変更なし (`git status` に出ない)。
- `.venv`, `__pycache__` は `.gitignore` で無視される (`git check-ignore`)。
- **問題**: `dora run` が生成する `experiments/dora_min/out/` (実行ログ) が無視されていなかった。
- サイズ: `.venv` を除き、ソース・README は各 4 KB、`out/` 32 KB。`.venv` は 338 MB (無視対象)。

## 6. 完了条件
- [x] `dora run` で最後まで動く: exit 0 (再実行で確認)
- [x] NumPy 配列をノード間で受け渡せる: 3 種 40 件が shape/dtype/値で一致、破損検出も確認
- [x] 採用判断を記録: `docs/decisions/0001-dataflow.md` (採用、multiprocessing は予備)

## 修正
- `experiments/dora_min/.gitignore` を新規作成 (`out/`)。dora の実行ログがコミットされないようにした。

## 推奨・次のステップ
- TASK-0025 で pyproject に `dataflow` グループを追加 (案は 0001-dataflow.md)。
- 実機ドライバ・MuJoCo をノードに入れた場合、30 Hz 以上、1 ノード異常終了時の挙動は未確認。
- CLAUDE.md は変更していない (指示による)。実行手順は `experiments/dora_min/README.md` にある。

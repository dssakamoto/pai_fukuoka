# 0001: ノード間通信に dora-rs を採用する

- **日付**: 2026-10-20
- **状態**: 採用 (代替案 multiprocessing は予備)
- **関連**: TASK-0007, NFR-04, design.md 2・3・9 章

## 背景

design.md はノードを dora-rs の dataflow で結ぶ構成とし、「Mac で動かなければ multiprocessing + キュー」を代替案としていた。最小例で Mac (Apple Silicon) 上の動作を確かめた。

## 結果

`experiments/dora_min/` の最小データフローが `dora run` で最後まで動き、exit code 0 で自動終了した(2 回実行して同じ結果)。

| 項目 | 結果 |
|---|---|
| 環境 | macOS (Apple Silicon)、Python 3.12 |
| バージョン | dora-rs-cli 1.0.1、dora-rs 1.0.1、numpy 2.5.4、pyarrow 26.0.0 (講義第6回と同じ 1.0.1) |
| インストール | `uv pip install` で成功(arm64 の wheel あり)。ビルド不要 |
| 配列の受け渡し | 48x64x3 uint8 を 10 件、480x640x3 uint8 (921,600 B) を 20 件、float64 (6,) を 10 件。すべて shape・dtype・値が送信内容と一致 |
| 遅延 (送信直前 -> 受信直後、同一マシンの時刻) | 480x640x3: 中央値 0.45〜0.56 ms、最大 0.7〜2.2 ms。関節ベクトルは robot_dummy 経由の往復で中央値 0.6 ms。最初の 1 件だけ 57〜65 ms(接続の初期化) |
| NumPy -> Arrow 変換 (480x640x3) | 中央値 0.07 ms |
| 負荷 | 全体 4.1 秒、user 0.98 s + sys 0.36 s、最大メモリ(`dora run` 本体)約 111 MB。10 Hz 程度では CPU はほぼ空き |
| 配線図 | `dora graph dataflow.yml --mermaid` が動作 |
| 後始末 | 実行後に dora の daemon / coordinator プロセスは残っていない |

結論: 10 Hz のカメラ画像 (VGA) と 6 関節の指令を流すには、遅延・負荷とも十分。

## 判断

**dora-rs を採用する。** multiprocessing は予備として残すが、今は実装しない。

理由:
- Mac (arm64, Python 3.12) で追加ビルドなしに動き、NumPy 配列を欠落なく受け渡せた。
- 配線を `dataflow.yml` に集約でき、NFR-04 (配線は dataflow.yml、ドライバの差し替えは設定のみ) に合う。
- 遅延は 1 ms 未満で、VLM の応答 (秒単位) に比べて無視できる。
- 全ノードが終了すると `dora run` も終了するため、試行ごとの自動終了が書ける。

## TASK-0025 への注意点

- **ノードの形**: `Node()` のイベントループ + `send_output(名前, pa.array(...), metadata=...)`。受信は `event["value"].to_numpy()`。画像は 1 次元の Arrow 配列で流れるので、**shape は受信側で復元する**(`reshape`)。shape・dtype は固定値を共有するか、`metadata` で渡す(例: `{"height": 480, "width": 640}`)。2 次元以上の ndarray は `pa.array(frame.ravel())` のように平坦化して送る。
- **float64 のベクトル**は dtype を保ったまま届く。画像は uint8 のまま。
- **終了のさせ方**: 全ノードが `break` で抜ければ `dora run` が終わる。受信側は `INPUT_CLOSED` と `STOP` で抜ける。タイマー起点のノードは送信回数で自分で終了する。常駐させる場合は Ctrl-C で止める。
- **sim / real の切り替え**: yaml の `env:` で環境変数を渡せる(講義の `WINDOW` の例)。`camera` / `robot` ノードに `env: { CONFIG: configs/sim.yaml }` のように設定ファイルのパスを渡し、ノード内で読む方式にする。ノード側のコードは共通にする。API キーは env に直書きせず、シェルの環境変数から読む。
- **シーン / モデルの渡し方**: MuJoCo のモデルパスなどは `env:` で渡し、ノード起動時に 1 度だけ読み込む。モデルが大きい(VLM のローカル実行など)ノードは起動が遅い点に注意。
- **起動時間**: 初回は Python の import 込みで約 9 秒、2 回目以降は約 1 秒。全ノードの準備が揃ってからデータが流れ始める(`all nodes are ready`)。MuJoCo や LeRobot の import が重いと起動が長くなる。
- **Python の選択**: ノードは `PATH` 上の `python` (venv の bin を PATH 先頭に置いた場合は venv の python) で起動した。メイン環境で動かすときは `uv run dora run dataflow.yml` か、venv を activate して実行する。
- **共有コード**: ノードと同じディレクトリの `common.py` を import できた(ノードの作業ディレクトリがスクリプトの場所のため)。`src/` のライブラリを使う場合は、パッケージとして install する(`uv sync`)か PYTHONPATH を env で渡す。
- **ログ**: dora 1.0.1 は `WARN ... dora-rs/dora#2742 diagnostic` を出す(動作に影響なし)。ノードの print は `flush=True` を付けないと遅れる。
- **未確認**: 実機ドライバ(シリアル)・MuJoCo レンダラをノードに入れたときの動作、30 Hz 以上、多数のノードでの負荷、異常終了時のノードの扱い(1 ノードが落ちたときの他ノードの挙動)。TASK-0025 で確認する。

## pyproject への追加案(適用しない)

メインの `pyproject.toml` には、オプショナルな依存グループ `dataflow` として次を追加することを推奨する(TASK-0025 で適用)。

```toml
[project.optional-dependencies]
dataflow = ["dora-rs-cli==1.0.1", "dora-rs==1.0.1", "pyarrow"]
```

または `[dependency-groups]` の `dataflow` でもよい。バージョンは講義と同じ 1.0.1 に固定する。CI (GitHub Actions) の通常テストは dora なしで動くよう、ノードの純粋ロジックを分離しておく(design.md 2 章)。

## 代替案 (multiprocessing)

動作が不安定になった場合のみ、同じノード構成を `multiprocessing.Process` + `Queue` (大きい画像は `shared_memory`) で実装する。今回は採用しないため、プロトタイプは作っていない。

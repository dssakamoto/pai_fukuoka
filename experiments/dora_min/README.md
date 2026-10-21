# dora-rs 最小動作確認 (TASK-0007)

タイマー -> ダミー画像(NumPy -> Arrow) -> 受信側で NumPy に戻して検証、までを `dora run` で確かめる実験。
メインの uv 環境(`pyproject.toml` / `.venv`)とは **独立した仮想環境** を使う。

## 構成

| ファイル | 役割 |
|---|---|
| `dataflow.yml` | 配線。`camera_dummy` -> `viewer`、`camera_dummy` -> `robot_dummy` -> `viewer` |
| `camera_dummy.py` | 100ms ごとに 48x64x3 画像 10 枚、続けて 480x640x3 画像 20 枚と float64(6,) の関節ベクトル 10 件を送る |
| `robot_dummy.py` | 関節ベクトルを +1 して `robot_state` として返す |
| `viewer.py` | 受信配列を送信側と同じ式で再生成した配列と比較し、遅延を表示して終了(不一致なら exit 1) |
| `common.py` | パターン生成関数(送受信で共有) |

## 実行方法

```bash
cd experiments/dora_min
uv venv .venv --python 3.12
uv pip install --python .venv/bin/python "dora-rs-cli==1.0.1" "dora-rs==1.0.1" numpy pyarrow

# venv の python をノードが使えるよう、PATH 先頭に venv の bin を置く
export PATH=$PWD/.venv/bin:$PATH
dora graph dataflow.yml --mermaid   # 配線図(任意)
dora run dataflow.yml               # 自動で終了し exit code 0 になる
```

期待する出力の末尾: `viewer: OK: 全配列が送信内容と一致 (shape/dtype/値)`

## 注意

- 初回はノード起動(Python の import)に約 9 秒かかる。2 回目以降は約 1 秒。
- macOS には `timeout` コマンドがない。時間制限が必要なら Python の `subprocess.run(timeout=...)` を使う。
- `WARN ... dora-rs/dora#2742 diagnostic` というログが出るが、動作には影響しない(dora 1.0.1 の診断ログ)。

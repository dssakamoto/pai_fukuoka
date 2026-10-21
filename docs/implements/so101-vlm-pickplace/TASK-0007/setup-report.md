# TASK-0007 設定作業実行

## 作業概要

- **タスクID**: TASK-0007
- **作業内容**: dora-rs の Mac での最小動作確認と採用判断
- **実行日**: 2026-10-20
- **実行者**: Claude (direct-setup)

## 設計文書参照

- docs/tasks/so101-vlm-pickplace/TASK-0007.md、docs/design.md (2・3・9 章)、docs/requirements.md (NFR-04)
- 講義: 応用1 第6回 演習資料 (dora-rs 1.0.1)

## 実行した作業

### 1. 隔離環境の作成とインストール

メインの `pyproject.toml` / `uv.lock` / `.venv` には触れず、専用の仮想環境を使った。`.gitignore` の `.venv/` が `experiments/dora_min/.venv` にも一致するため、追記は不要。

```bash
uv venv experiments/dora_min/.venv --python 3.12
uv pip install --python experiments/dora_min/.venv/bin/python "dora-rs-cli==1.0.1" "dora-rs==1.0.1" numpy pyarrow
```

結果: dora-rs-cli 1.0.1、dora-rs 1.0.1、numpy 2.5.4、pyarrow 26.0.0 (arm64 wheel、問題なし)。

### 2. 最小データフローの作成

`experiments/dora_min/` に `dataflow.yml`、`camera_dummy.py`、`robot_dummy.py`、`viewer.py`、`common.py`、`README.md` を作成。

### 3. 実行

```bash
export PATH=$PWD/experiments/dora_min/.venv/bin:$PATH
dora graph dataflow.yml --mermaid
dora run dataflow.yml
```

- exit code 0、2 回実行。ログは scratchpad の `dora_min/run1.log`、`run2.log`。
- viewer が全配列の一致を確認 (shape / dtype / 値)。480x640x3 の遅延は中央値約 0.5 ms。
- 実行後に dora のプロセスは残っていない。

### 4. 判断の記録

`docs/decisions/0001-dataflow.md` に、結果・採用判断(採用)・TASK-0025 への注意点・pyproject への追加案(未適用)を記録。

## 作業結果

- [x] 依存関係のインストール完了 (隔離環境)
- [x] `dora run` が最後まで動作し exit code 0
- [x] NumPy 配列をノード間で受け渡し、一致を確認
- [x] 採用判断を記録
- [x] 代替案 (multiprocessing) は不要のため未実装

## 遭遇した問題と解決方法

- macOS に `timeout` がない: Python の `subprocess.run(timeout=)` で代用。
- 初回起動が約 9 秒かかる: Python の import の初回コスト。2 回目以降は約 1 秒。
- `dora-rs/dora#2742 diagnostic` の WARN ログ: 動作に影響なし。

## 次のステップ

- `/tsumiki:direct-verify` で確認
- TASK-0025 で `dataflow` 依存グループを pyproject に追加

# so101-vlm-pickplace

SO-101 ロボットアームを VLM（視覚言語モデル）の指示で動かす pick-and-place。まず MuJoCo のシミュレーションで動かし、実機へ差し替える。

## セットアップ

Python 3.12 と [uv](https://docs.astral.sh/uv/) を使う。

```bash
uv sync
```

## テスト

```bash
uv run pytest
uv run pytest --cov   # カバレッジ
```

## 環境変数

API キーは `.env` に書く（`.env` はコミットしない）。

```bash
cp .env.example .env
# .env を編集して ANTHROPIC_API_KEY / OPENAI_API_KEY / GEMINI_API_KEY を設定
```

Gemma はローカルの Ollama で動かすため、キーは不要。

## ドキュメント

- [要件定義書](docs/requirements.md)
- [設計文書](docs/design.md)
- [タスク概要](docs/tasks/so101-vlm-pickplace/overview.md)

## ライセンス・クレジット

SO-101 の MuJoCo モデル（MJCF とメッシュ）は [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) の `Simulation/SO101` を使う（Apache-2.0）。ライセンス全文と取得元コミットは [assets/so101/LICENSE](assets/so101/LICENSE)、[assets/so101/SOURCE.md](assets/so101/SOURCE.md) にある。ファイルは改変せず git に含めている（約 15 MB）。

モデルを（再）取得するには、固定コミットから次で取り直せる（`--force` で上書き）。

```bash
uv run python scripts/fetch_so101_model.py
```

## 動作確認(TASK-0003)

SO-101 の 6 関節を 1 つずつ順に正弦波で動かして確認する。

```bash
# 画面なし: 動画に書き出す(出力先パスを表示。media/ は git 管理外)
uv run python scripts/view_so101.py --headless --out media/so101_sweep.mp4 --seconds 12

# ビューア(macOS ではパッシブビューアに mjpython が必要)
uv run mjpython scripts/view_so101.py
```

関節順は `so101_vlm.robot.base.JOINT_NAMES`(定義は `model_paths.py`)。

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

## シーンの確認(TASK-0005)

ピック&プレース用シーン(`assets/scene_pickplace.xml`: ブロック 3 個、箱、固定カメラ `cam_top`)を撮影し、RGB と深度を保存する。

```bash
# media/scene_rgb.png, media/scene_depth.npy(m 単位), media/scene_depth.png(確認用)を出力
uv run python scripts/capture_scene.py
```

- 上流の `assets/so101/` は改変しない。`scene_pickplace.xml` は `so101_new_calib.xml` を include し、床・照明は上流 `scene.xml` と同内容を書き写している(`meshdir`/`strippath` の理由はファイル冒頭のコメント)。
- 寸法・名前の単一の情報源は `src/so101_vlm/eval/scene_spec.py`(箱の内寸・縁の高さ、作業エリア、カメラ名)。

## データフロー(TASK-0025)

camera / vlm_planner / perception / policy / motion / robot / evaluator / recorder を dora-rs のノードに分け、`dataflow.yml` で配線する。`dataflow.yml` はテンプレートで、実行スクリプトがインタプリタ・設定・試行の指定を埋めて `dora run` に渡す。

```bash
uv sync --group dataflow          # dora-rs 1.0.1 (通常のテストには不要)
# 1 試行(API キー不要の偽 VLM = oracle)。結果は media/dataflow/result.json
uv run python scripts/run_dataflow.py --seed 2 --colour red --instruction "put the red block into the box"
```

- 終了コード: 0 = 成功、1 = 試行は動いたが失敗(VLM・計画・把持など)、2 = 引数の誤り、3 = dora の失敗・時間切れ。
- sim と real の切り替えは `--config configs/sim.yaml` / `configs/real.yaml` だけ。real のドライバ(robot / camera)は TASK-0034 / 0036 で実装するまで、起動時に `NotImplementedError` で止まる。
- VLM は `configs/sim.yaml` の `vlm.provider`(`oracle` または `configs/vlm.yaml` のプロバイダ)で選ぶ。
- dora なしで同じ配線を確かめるには、`so101_vlm.nodes.local_runner.LocalRunner`(同じプロセス内で配線を再生。テストで使う)。
- ノードのロジックは dora に依存しない(`src/so101_vlm/nodes/`)。dora との接続は `dora_runtime.py` だけ。

## 複数試行の評価(TASK-0026)

シードと指示を変えた試行を繰り返し、試行ごとの記録(`trials.csv`)と成功率・失敗の分類を出す。

```bash
# API キー不要の偽 VLM(真の配置から枠を作る)で 20 試行(約 25 秒)
uv run python scripts/run_eval.py --provider oracle --trials 20 --seed 0
# 実際の VLM(configs/vlm.yaml のプロバイダ。キーは環境変数)
uv run python scripts/run_eval.py --provider gemini --trials 20 --seed 0 --min-success-rate 0.7
```

- 試行 i のシードは `--seed + i`、対象の色は `--colours`(既定は red,blue,green)を順に回す。指示は `--instruction-template` の `{colour}` を色名に置き換える。
- 出力は `results/<run-id>/trials.csv`(1 試行 1 行。シード・指示・プロバイダ・モデル・検出結果・成否・失敗の種類・時間・検出誤差)と `summary.json`。`results/` は git 管理外。同じ `--run-id` は上書きしない。
- 失敗の種類は 8 種(vlm_failure / perception_failure / planning_failure / grasp_slip / drop / collision_other / knocked_over / not_in_box)で、集計では VLM・IK・把持・配置の 4 分類にもまとめる。
- 終了コード: 0 = 完了、1 = `--min-success-rate` 未達、2 = 引数・設定・API キーの誤り。

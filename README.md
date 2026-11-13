# so101-vlm-pickplace

SO-101 ロボットアームを VLM（視覚言語モデル）の指示で動かす pick-and-place。まず MuJoCo のシミュレーションで動かし、実機へ差し替える。

## セットアップ

Python 3.12 と [uv](https://docs.astral.sh/uv/) を使う。

```bash
uv sync
```

## テスト

```bash
uv run pytest                          # すべて(MuJoCo の描画ができる環境で)
uv run pytest -m "not render and not api" --cov   # CI と同じ: 描画と実 API を除く + カバレッジ
```

- pytest の印: `render` = MuJoCo のオフスクリーン描画(OpenGL)が要る、`api` = 実際の VLM API(キーとネットワークが要る)。印の付いたテストは、GPU・キーのない環境では `-m "not render and not api"` で除く。印は `pyproject.toml` に登録してあり(`--strict-markers`)、綴りを間違えるとエラーになる。
- カバレッジは 80% 未満で失敗にする(AC-07、`pyproject.toml` の `fail_under`)。描画を除いた構成で約 91%。
- LeRobot(`--group record`)と dora(`--group dataflow`)は任意の依存で、入っていないと、それらのテストは自動でスキップされる。

### CI(GitHub Actions)

[`.github/workflows/ci.yml`](.github/workflows/ci.yml): `main` への push と pull request で、`uv sync --frozen` の後に上のコマンドを実行する(ubuntu、約 3 分)。アクションは完全なバージョンで固定してある。

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

## LeRobot 形式での記録(TASK-0027)

1 試行を 1 エピソードとして、画像・関節状態・行動(目標関節角)・言語指示を LeRobot のデータセット(v3.0)に保存する。v2 のデモ(実機)と同じ特徴量名・fps(30)なので、そのまま SmolVLA の微調整に使える。仕様と実測は [docs/notes/lerobot-format.md](docs/notes/lerobot-format.md)。

```bash
uv sync --group record     # lerobot 0.7.0(torch を含む。約 1.3 GB。通常のテストには不要)
uv run python scripts/run_eval.py --trials 2 --seed 2 --record-dir results/lerobot_demo
```

- `--record-dir` は新しいディレクトリのみ(LeRobot の制約)。動かなかった試行(VLM・知覚・計画の失敗)はエピソードにならず、`so101_trials.jsonl` に記録だけ残る。
- 記録つきの 1 試行は約 35 秒(描画が支配的)。評価の本番には付けず、記録したい試行にだけ使う。
- 設定は `configs/record.yaml`(fps、カメラ名、動画/画像、`repo_id`)。lerobot が無い環境では、記録のテストは自動でスキップされる。

## 実行動画(TASK-0028)

複数の試行をつないで、提出用の動画(mp4)を作る。試行ごとに 指示 → 検出(枠) → 実行 → 判定 のクリップになり、最後に成功率のカードが付く。

```bash
# API キー不要の偽 VLM で 10 試行、2 倍速(約 1 分 30 秒の動画。作るのに約 3 分)
uv run python scripts/render_video.py --provider oracle --trials 10 --speed 2 --out media/demo.mp4
```

- 画面の上に 指示と段階(指示 > 検出 > 実行 > 判定、今の段階を黄色)、検出枠(pick = マゼンタ、place = シアン)、右下に倍速(「2倍速」)、実行中は区間名、判定のときに成功(緑)/失敗(赤)の帯が出る。文字は動画を縮小しても読める大きさ(画像の高さの 1/20)。
- `--speed` は 30 の約数(1, 2, 3, 5, 6, 10, ...)。1 なら実時間で、倍速の表示は出ない。動画の長さを表示し、1〜3 分から外れるときは警告する(`--trials` と `--speed` で調整)。
- 動かなかった試行(VLM・知覚・計画の失敗)は、指示・検出・判定の静止画のクリップになる。
- 日本語フォント(Yu Gothic など)があれば日本語、無ければ英語の表示。`--font` または環境変数 `SO101_VIDEO_FONT` で指定できる。
- 描画が遅い環境では、1 試行あたり約 20 秒かかる。既存の `--out` は `--overwrite` なしでは上書きしない。

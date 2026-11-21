# so101-vlm-pickplace

SO-101 ロボットアームを、VLM(視覚言語モデル)への自然言語の指示で動かす pick-and-place。
カメラ画像と指示(例: "put the red block into the box")を VLM に渡し、返ってきた検出枠を 3 次元位置に直し、IK で軌道を作って、**MuJoCo のシミュレーション**で実行する。成功は、ブロックが箱に入ったかで自動判定する。実機への切り替えは、同じ構造で差し替えられるように設計してある(v2。未実装)。

```
指示 + 画像 -> VLM (Anthropic / OpenAI / Gemini) -> 検出枠 -> 3 次元位置 -> IK -> 実行 -> 成功判定
```

- 状況: **v1(シミュレーション + VLM + 自動評価 + 動画)は完成**。実機(v2)は未着手。
- 実測(OpenAI `gpt-6-luna`、20 試行): 成功率 80.0%、検出誤差 平均 9.0 mm。詳細は [プロバイダ別の比較表](docs/notes/provider-comparison.md)。

## 動作環境

| | |
|---|---|
| Python | 3.12(`.python-version`。[uv](https://docs.astral.sh/uv/) が用意する) |
| OS | Windows 11 / macOS / Linux。GPU は不要 |
| 描画 | MuJoCo のオフスクリーン描画(OpenGL)が使えること。使えない環境(ヘッドレスの Linux など)でも、描画を使わないテストは動く(CI) |
| ネットワーク・API キー | シミュレーションと評価は**不要**(偽の VLM `oracle` を使う)。実際の VLM を使うときだけ必要 |

動作を確認した環境: Windows 11(開発。全手順)、GitHub Actions の Ubuntu(描画を除くテスト)、macOS(開発の一部。dora-rs の確認など)。

## クイックスタート(新しい環境で再現する)

```bash
# 1. 取得と環境構築(MuJoCo と SO-101 のモデルは同梱。約 260 MB、数十秒〜数分)
git clone https://github.com/dssakamoto/pai_fukuoka
cd pai_fukuoka
uv sync

# 2. 単体テスト(約 2 分)。CI と同じ構成: 描画と実 API を除く + カバレッジ 80% 以上
uv run pytest -m "not render and not api" --cov

# 3. シーンを撮影する(ブロック 3 個、箱、固定カメラ)。media/scene_rgb.png ほかを出力
uv run python scripts/capture_scene.py

# 4. VLM なしで pick & place(真の位置から)。ブロックが箱に入る
uv run python scripts/run_fixed_pick.py --seed 0 --colour red --no-video

# 5. VLM を使う 1 試行(API キー不要の偽 VLM = oracle)。指示 -> 検出 -> 実行 -> 判定を表示
uv run python scripts/run_trial.py --provider oracle --seed 2 --colour red --instruction "put the red block into the box"

# 6. 20 試行の自動評価(約 25 秒)。成功率と失敗の分類、results/<run-id>/trials.csv
uv run python scripts/run_eval.py --provider oracle --trials 20 --seed 0

# 7. 実行動画(約 1 分 30 秒の動画。作るのに約 3 分)。media/demo.mp4
uv run python scripts/render_video.py --provider oracle --trials 10 --speed 2 --out media/demo.mp4
```

期待する結果(同じシードなら、環境が違っても同じになる):

| 手順 | 結果 |
|---|---|
| 2 | 約 1,780 件が通る、カバレッジ 約 92%(lerobot・dora が無い環境では、その分のテストがスキップされる) |
| 4 | `seed=0 red ... 成功` |
| 5 | `判定: 成功`、検出誤差 9.3 mm |
| 6 | 成功率 85.0%(17/20)、検出誤差 平均 7.9 mm(失敗: IK 2 件、把持 1 件) |

`uv run pytest`(印なし)は、描画を使うテストも含めて全部を実行する。**描画ができる環境で**実行する。

## 実際の VLM を使う

```bash
cp .env.example .env     # ANTHROPIC_API_KEY / OPENAI_API_KEY / GEMINI_API_KEY のうち、使うものを書く(.env は git 管理外)
```

```bash
# 1 試行。存在しない色を指示すると、ロボットは動かず、理由が表示される(AC-02、終了コード 1)
uv run python scripts/run_trial.py --provider openai --seed 2 --instruction "put the purple block into the box"
# 20 試行の評価
uv run python scripts/run_eval.py --provider openai --trials 20 --seed 0 --min-success-rate 0.7
# プロバイダ別に、同じ条件で評価して比較表を作る(キーのないプロバイダは「未評価」)
uv run python scripts/compare_providers.py --trials 20 --seed 0 --notes docs/notes/provider-comparison.md
# 実 API で、sim の画像から赤いブロックを検出できるか(キーがなければスキップ)
uv run pytest tests/vlm/test_live_apis.py -m api
```

```
指示: put the purple block into the box
VLM: openai / gpt-6-luna
検出: なし
実行: ロボットは動かしていません (動作の前に拒否)
判定: 失敗 (vlm_failure): VLM の応答が使えない (対象が見つからない・形式が不正・呼び出しの失敗)
理由: pick がありません: No purple block is visible in the image.
```

- スクリプトは `.env` を自動で読む(シェルで設定済みの環境変数が優先。値は表示しない)。キーが無い・空のときは、起動時に環境変数名つきのエラーで止まる。
- プロバイダは `configs/vlm.yaml` で選ぶ(`provider:`、モデル名、タイムアウト 30 秒、再試行 2 回、キャッシュ)。実装済み: Anthropic / OpenAI / Gemini / Gemma(Ollama、キー不要。`ollama pull gemma4:e4b` のあと `ollama serve`。起動していない・未取得のときは、比較表では「未評価」)。
- **料金**: 1 回の呼び出し(640x480 の画像 + 指示)で、入力 約 700 / 出力 約 100〜500 トークン。20 試行の評価で約 $0.003(`gpt-6-luna`)。VLM の応答はキャッシュされ、同じ画像・指示では API を呼び直さない。
- 実 VLM は、近い色で代用することがある(例: purple の指示で青いブロックを選ぶ)。プロンプトで色の厳密な一致を求めている。詳細は [docs/notes/vlm-providers.md](docs/notes/vlm-providers.md)。
- OpenAI のクレジット切れ(HTTP 429 `insufficient_quota`)は、レート制限と区別して、再試行しない。

## コマンド一覧

| スクリプト | 内容 |
|---|---|
| `scripts/run_trial.py` | 1 試行を指示つきで実行し、各段階と失敗の理由を表示する |
| `scripts/run_eval.py` | N 試行の自動評価。`trials.csv`(1 試行 1 行)、`summary.json`、成功率、失敗の分類。`--record-dir` で LeRobot 形式にも記録 |
| `scripts/compare_providers.py` | プロバイダ別の比較表(成功率・検出誤差 mm・トークン数) |
| `scripts/render_video.py` | 試行をつないだ実行動画(指示・検出枠・実行・判定・倍速の表示) |
| `scripts/run_dataflow.py` | dora-rs のデータフローで 1 試行(任意の依存。`uv sync --group dataflow`) |
| `scripts/run_fixed_pick.py` | VLM なしで、真の位置から pick & place(把持の調整・動作の確認) |
| `scripts/capture_scene.py` | シーンの RGB・深度の撮影 |
| `scripts/visualize_detection.py` | VLM の応答 JSON を画像に重ねる(座標の取り違えの目視確認) |
| `scripts/generate_calibration.py` | sim のカメラのキャリブレーション(`configs/calibration.yaml`)の生成・検査 |
| `scripts/view_so101.py` | SO-101 の 6 関節を動かして確認(`--headless` で動画。macOS のビューアは `mjpython`) |
| `scripts/fetch_so101_model.py` | SO-101 のモデルを固定コミットから取り直す |

終了コードの共通の決まり: `0` = 成功・完了、`1` = 試行は行ったが失敗(または基準未達)、`2` = 引数・設定・API キーの誤り。

## 設定ファイル

| ファイル | 内容 |
|---|---|
| `configs/vlm.yaml` | VLM のプロバイダ・モデル名・タイムアウト・再試行・キャッシュ |
| `configs/eval.yaml` | 評価の試行数・シード・ブロックの配置(20 cm 四方、5 cm 以上離す)・成功判定のしきい値 |
| `configs/motion.yaml` | 安全フィルタ(関節の可動域・速度・作業範囲)と、軌道の生成(接近の高さ、把持、運搬) |
| `configs/calibration.yaml` | カメラの内部・外部パラメータ(sim から自動生成) |
| `configs/sim.yaml` / `real.yaml` | データフローの設定。sim と real の切り替えはここだけ(real のドライバは v2) |
| `configs/record.yaml` | LeRobot 形式での記録(fps 30、カメラ名、動画/画像) |

## 任意の機能

通常の `uv sync` には含まれない依存は、必要なときだけ入れる。入っていなければ、関連のテストは自動でスキップされる。

| 機能 | 入れ方 | 使い方 |
|---|---|---|
| dora-rs のデータフロー | `uv sync --group dataflow`(dora-rs 1.0.1) | `uv run python scripts/run_dataflow.py --seed 2 --colour red`。ノード分割と配線は `dataflow.yml`(テンプレート)。ADR は [docs/decisions/0001-dataflow.md](docs/decisions/0001-dataflow.md) |
| LeRobot 形式での記録 | `uv sync --group record`(lerobot 0.7.0。torch を含み、約 1.3 GB) | `uv run python scripts/run_eval.py --trials 2 --seed 2 --record-dir results/lerobot_demo`。記録つきの 1 試行は約 35 秒。仕様は [docs/notes/lerobot-format.md](docs/notes/lerobot-format.md) |

## テストと CI

```bash
uv run pytest                                      # すべて(描画ができる環境で)
uv run pytest -m "not render and not api" --cov    # CI と同じ
```

- pytest の印: `render` = MuJoCo のオフスクリーン描画(OpenGL)が要る、`api` = 実際の VLM API(キーとネットワークが要る)。印は `pyproject.toml` に登録してあり(`--strict-markers`)、綴りを間違えるとエラーになる。
- カバレッジは 80% 未満で失敗にする(AC-07、`pyproject.toml` の `fail_under`)。描画を除いた構成で約 92%。
- 通常のテストは、本物の SDK を**偽の API サーバ**(ローカル)に向けて動かす。キーもネットワークも要らない。
- CI: [`.github/workflows/ci.yml`](.github/workflows/ci.yml)。`main` への push と pull request で、`uv sync --frozen` の後に上のコマンドを実行する(ubuntu、約 3 分)。

## トラブルシューティング

| 症状 | 原因と対処 |
|---|---|
| Windows で `import mujoco` が `DLL load failed` | Microsoft Visual C++ 再頒布可能パッケージ(x64)が無い。`winget install Microsoft.VCRedist.2015+.x64` |
| Windows で `.venv` の `python.exe` が「アプリケーション制御ポリシーによってブロックされました」 | 端末のセキュリティ設定による制限(このプロジェクトの問題ではない)。管理者に許可を依頼する |
| Linux のヘッドレスで、描画のテスト・スクリプトが落ちる | OpenGL が無い。`MUJOCO_GL=egl`(または `osmesa`)を設定するか、`-m "not render"` で除く |
| macOS で `scripts/view_so101.py` のビューアが動かない | `uv run mjpython scripts/view_so101.py`(パッシブビューアには `mjpython` が要る) |
| `API キーがありません: ... 環境変数 XXX を設定してください` | `.env` に値を書く(`cp .env.example .env`)。値が空だと「未設定」として扱う |
| OpenAI が HTTP 429 `You have no credits remaining` | クレジット切れ。課金設定を確認する(待っても直らないので、再試行しない) |
| 評価が遅い | 描画が約 0.1 秒/枚の環境がある(ソフトウェア描画)。記録・動画は描画の回数が多いので、本番の評価には付けない |
| `dora run` で `No module named 'dora'`(Windows) | `path: python` が venv の Python に解決されない。`scripts/run_dataflow.py` がインタプリタのパスを埋めるので、直接 `dora run` せず、このスクリプト経由で使う |

## プロジェクトの構成

```
src/so101_vlm/
  robot/       Robot 抽象と SimRobot(MuJoCo)。実機 So101Robot は v2
  perception/  画素+深度 -> ロボット座標(投影・キャリブレーション)
  policy/      IK(DLS)、把持・運搬の軌道生成、実行器(安全フィルタ経由)
  safety/      可動域・速度・作業範囲・状態途絶の安全フィルタ
  vlm/         VlmClient 抽象、Anthropic / OpenAI / Gemini / Ollama、出力の検証、キャッシュ
  eval/        1 試行(trial)、複数試行(runner)、成功判定(judge)、比較(compare)、動画
  nodes/       dora-rs のノード(ロジックは dora に依存しない)
  record/      LeRobot 形式での記録
assets/        シーン(scene_pickplace.xml)と SO-101 のモデル(assets/so101/、上流のまま)
configs/       設定  scripts/ コマンド  tests/ 単体・統合テスト  docs/ 要件・設計・タスク・ノート
```

寸法・名前の単一の情報源は `src/so101_vlm/eval/scene_spec.py`(箱の内寸・縁の高さ、作業エリア、カメラ名)。シーンは `scene_pickplace.xml` が `so101_new_calib.xml` を include して作る(上流の `assets/so101/` は改変しない)。

## ドキュメント

- [要件定義書](docs/requirements.md) / [設計文書](docs/design.md) / [タスク概要](docs/tasks/so101-vlm-pickplace/overview.md)
- [プロバイダ別の比較表](docs/notes/provider-comparison.md) / [VLM の仕様・実測のメモ](docs/notes/vlm-providers.md) / [把持の調整](docs/notes/grasp-tuning.md) / [LeRobot 形式](docs/notes/lerobot-format.md)
- [データフローの ADR](docs/decisions/0001-dataflow.md)

## 今後(v2 と提出物)

実機(SO-101)での pick & place と、SmolVLA の微調整(TASK-0032〜0045)は未着手。`Robot` 抽象・`Policy` 抽象・`configs/real.yaml`・LeRobot 形式の記録は、実機に差し替えられる形で用意してある。
未確認: Gemma(Ollama)の実機での検出(クライアントは実装・偽サーバでテスト済み。このPCでは動かさない方針)、Anthropic・Gemini の実 API での確認(キー待ち)。

## ライセンス・出典

- **このリポジトリのコード**: ライセンスは、まだ定めていない(課題の提出用)。再利用を許す場合は `LICENSE` を追加する。
- **SO-101 の MuJoCo モデル**(MJCF とメッシュ): [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) の `Simulation/SO101`(**Apache-2.0**)。ライセンス全文と取得元コミットは [assets/so101/LICENSE](assets/so101/LICENSE)、[assets/so101/SOURCE.md](assets/so101/SOURCE.md)。ファイルは改変せず git に含めている(約 15 MB)。取り直すには `uv run python scripts/fetch_so101_model.py`(`--force` で上書き)。
- **主な依存**: [MuJoCo](https://github.com/google-deepmind/mujoco)(Apache-2.0)。任意の依存: [LeRobot](https://github.com/huggingface/lerobot)(Apache-2.0)、[dora-rs](https://github.com/dora-rs/dora)(Apache-2.0)。VLM の SDK(`anthropic` / `openai` / `google-genai`)と、そのほかの依存のライセンスは、各パッケージに従う(版は `uv.lock`)。
- **VLM のサービス**: 各社の利用規約と料金に従う。送るのは、シミュレーションで描いた画像と、指示の文だけ。
- **動画の文字**: システムにあるフォントを使う(フォントは同梱しない)。

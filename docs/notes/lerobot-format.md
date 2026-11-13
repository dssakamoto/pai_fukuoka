# LeRobot データセット形式の確認 (TASK-0027)

確認日: 2026-11-08。公式ドキュメントとソース(lerobot v0.7.0)を読み、Windows 11 / Python 3.12 で実際に書き込み・読み戻しを試した。
🔵 = ドキュメント・ソースに明記 かつ 実際に確認 / 🟡 = ドキュメントから推測

## 版

- **lerobot 0.7.0 に固定**(`pyproject.toml` の依存グループ `record`。`uv sync --group record`)。Python 3.12 以上。🔵
- データセット形式は **v3.0**(`CODEBASE_VERSION = "v3.0"`)。v3 は 1 つの parquet / mp4 に複数エピソードを入れ、境界はメタデータで持つ(v2.1 は 1 エピソード 1 ファイル)。🔵
- `torch`(2.15)が必須依存。導入後の `.venv` は約 1.3 GB。`lerobot` は `numpy<2.3` を要求するため、ロックファイル全体で numpy が 2.5.4 から 2.2.6 に下がる(既存の全テストは通る)。🔵
- この環境では `torchcodec` が FFmpeg を読み込めず、LeRobot が自動で `pyav` に切り替えて動画を読む(警告が出るが、読み書きは動く)。🔵

## 特徴量(名前・型・形)

SO-101 の LeRobot 実装 (`so_follower`) に合わせる。v2 のデモ収集 (`lerobot-record`) と同じ名前になるので、そのまま SmolVLA の微調整に使える。

| キー | dtype | shape | names |
|---|---|---|---|
| `observation.state` | float32 | (6,) | `shoulder_pan.pos`, `shoulder_lift.pos`, `elbow_flex.pos`, `wrist_flex.pos`, `wrist_roll.pos`, `gripper.pos` |
| `action` | float32 | (6,) | 同上 |
| `observation.images.top` | `video`(`use_videos: false` なら `image`) | (480, 640, 3) | `height`, `width`, `channels` |

- 単位は rad(関節角)。両方とも `.pos` の目標/現在位置。🔵
- `index` / `episode_index` / `frame_index` / `timestamp` / `task_index` は LeRobot が自動で付ける。**`timestamp` と `frame_index` を自分で渡してはいけない。**🔵
- 特徴量名に `/` は使えない(カメラ名にも使えない)。🔵

## fps

- **30**(`configs/record.yaml`)。SO-101 の記録(`lerobot-record`)の標準値で、v2 の実機デモ(TASK-0040)も同じ値にする。🟡(「v2 と同じ値」はこの計画上の決め)
- sim の制御周期は 50 Hz(0.02 s)。そこから 30 Hz へ**間引いて**書く(5 周期に 3 回。補間はしない)。制御周期より速い fps は拒否する。

## API(使い方と落とし穴)

```python
ds = LeRobotDataset.create(repo_id, fps, features, root=root, robot_type="so101", use_videos=True, image_writer_threads=4)
ds.add_frame({"observation.state": ..., "action": ..., "observation.images.top": <uint8 HxWx3>, "task": "<指示>"})
ds.save_episode()
ds.finalize()          # 必須。しないと読めない
LeRobotDataset(repo_id, root=root)   # 読み戻し
```

- `add_frame` には **`task`(言語指示の文字列)が必須**。渡した dict から `task` を取り除く(破壊的)ので、フレームごとに新しい dict を作る。🔵
- `create` は **`root` が既存だと失敗**(`FileExistsError`)。`LeRobotWriter` は先に確認して、既存のディレクトリを触らずに断る。🔵
- 読み戻した `dataset[i]` は、画像が `(3, H, W)` の float(0〜1)、`task` は文字列で入る。`episode_index` / `frame_index` も入る。🔵
- `repo_id` は `<ユーザ>/<名前>`。ローカルだけで使うので `local/so101-vlm-pickplace`。Hub には送らない。書き込み・読み戻しともネットワークは不要(確認済み)。🔵
- 画像の書き出しは `image_writer_threads` で別スレッドにできる(既定 0 = 同期)。実機のリアルタイム記録では 4 程度。🟡

## このプロジェクトでの使い方

- `record/lerobot_writer.py`: `LeRobotWriter`(検証つきの書き込み)。不正な入力(画像の形・dtype、状態/行動の長さ・NaN、空の指示)は ValueError で、エピソードを壊さない。
- `record/sim_episode.py`: `record_trial` で、`run_trial` の 1 試行を 1 エピソードとして記録する。観測は「その周期の指令を送る前」、行動は「その周期の指令(目標関節角)」(LeRobot の流儀)。
- **動かなかった試行**(VLM・知覚・計画の失敗)は、フレームがないのでエピソードにしない。
- 試行の付随情報(指示、検出結果、推定位置、成否、失敗の種類)は `<root>/so101_trials.jsonl` に 1 試行 1 行(`episode_index` は動かなかった試行で `null`)。LeRobot の `meta/` には触らない。
- 評価コマンドから: `uv run python scripts/run_eval.py --trials 2 --record-dir results/lerobot_demo`。

## 実測(2026-11-08)

- 2 試行(seed 2 赤、seed 3 青、偽 VLM)→ 2 エピソード / 393 フレーム / 30 fps / mp4 1 本。読み戻して形・値を確認した。
- **記録つきの 1 試行は約 35 秒**(記録なしは約 1.2 秒)。描画 1 枚が約 0.12 s(この環境のソフトウェア描画。解像度を下げても変わらない)で、残すフレームの観測として約 200 回描くため。LeRobot 側の書き込みは約 5 s。評価(20 試行以上)では `--record-dir` を付けず、記録したい試行だけに使う。

## 未確認

- 実機のカメラ・関節値からの記録(TASK-0034 / 0038 で `So101Robot` と `lerobot-record` を使うときに確認)。
- `torchcodec` が FFmpeg を読めない件(この環境では pyav に自動で切り替わる)。TASK-0032 で LeRobot を実機用に入れるときに再確認する。

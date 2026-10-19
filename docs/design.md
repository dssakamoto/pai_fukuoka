# 技術設計文書: 言葉で片付けてくれる SO-101

要件は `requirements.md` を参照。ここでは v1 を中心に設計し、v2 の差し替え口を示す。
外部ライブラリ(LeRobot、dora-rs、各 VLM の SDK)の API は、実装時に公式ドキュメントで確認する。ここでは名前と責務のみを定める。

## 1. 確認済みの前提(SO-101 モデル)

公式 `TheRobotStudio/SO-ARM100` の `Simulation/SO101`(Apache-2.0)を確認した結果。

- 関節は 6 つ: `shoulder_pan`、`shoulder_lift`、`elbow_flex`、`wrist_flex`、`wrist_roll`、`gripper`。
- アクチュエータは 6 つとも位置制御(`position`)。`ctrlrange` が各関節の可動域に対応する。単位はラジアン。
- 手先の基準点は site `gripperframe`、根元は site `baseframe`。
- 腕は 5 自由度 + グリッパー。手先の姿勢を自由には決められないため、IK は「位置 + 手先を下向きに近づける」制約付きにする(4.4 節)。
- `scene.xml` は床と光源のみ。ブロック、箱、カメラは自前で追加する。
- 開発機は Apple Silicon の Mac、メモリ 16 GB。

## 2. 全体構成

```text
 指示(テキスト)
      │
      ▼
┌──────────┐ 画像+深度 ┌──────────────┐ 検出結果(px) ┌────────────┐
│ camera   │──────────▶│ vlm_planner  │─────────────▶│ perception │
│ (sim/real)│          │ VlmClient    │              │ px→3D(base)│
└──────────┘           └──────────────┘              └─────┬──────┘
      ▲                                                    │ 目標3D位置
      │ 画像・状態                                         ▼
┌──────────┐  関節指令   ┌──────────┐  関節軌道   ┌──────────────┐
│  robot   │◀────────────│  motion  │◀────────────│ policy       │
│ Sim/So101│  状態       │ 安全フィルタ│            │ IkPolicy/    │
└────┬─────┘────────────▶└──────────┘            │ (v2)SmolVLA  │
     │                                            └──────────────┘
     ▼
┌──────────┐   ┌──────────┐
│ recorder │   │ evaluator│  成功判定・成功率
│ (LeRobot)│   └──────────┘
└──────────┘
```

- ノードは dora-rs の dataflow で結ぶ。1 ノード = 1 プロセスの Python スクリプト。
- 純粋ロジック(座標変換、IK、成功判定、JSON 検証)はノードから分離したライブラリとして実装し、ノードは薄い入出力の殻にする。これで単体テストを書きやすくする。
- 画像・状態の流れは Arrow 配列(NumPy とゼロコピーで変換できる)。

## 3. ノードとデータフロー

| ノード | 入力 | 出力 | 責務 |
|---|---|---|---|
| `camera` | tick | `rgb`、`depth`、`intrinsics` | 画像と深度の取得。sim は MuJoCo のレンダラ、real は実機カメラ |
| `vlm_planner` | `rgb`、`instruction` | `detections` | `VlmClient` を呼び、共通スキーマの検出結果を返す |
| `perception` | `detections`、`depth`、`intrinsics` | `targets_3d` | ピクセル+深度 → ロボット座標系の3次元位置 |
| `policy` | `targets_3d`、`robot_state` | `joint_trajectory` | v1 は IK、v2 は SmolVLA |
| `motion` | `joint_trajectory` | `joint_command` | 安全フィルタを通して `Robot` に送る |
| `robot` | `joint_command` | `robot_state` | `SimRobot` / `So101Robot` |
| `recorder` | 各種 | LeRobot データセット | 実行の記録 |
| `evaluator` | `robot_state`、真値 | `result` | 成功判定(sim のみ真値を使う) |

`dataflow.yml` の概形(キー名は実装時に dora-rs の仕様で確認する)。

```yaml
nodes:
  - id: camera
    path: nodes/camera.py
    inputs: { tick: dora/timer/millis/100 }
    outputs: [rgb, depth, intrinsics]
  - id: vlm_planner
    path: nodes/vlm_planner.py
    inputs: { rgb: camera/rgb, instruction: cli/instruction }
    outputs: [detections]
  # perception, policy, motion, robot, recorder, evaluator も同様に配線
```

sim と real の切り替えは、`camera` と `robot` ノードが読む設定(`configs/sim.yaml` / `configs/real.yaml`)だけで行う。他のノードは変更しない。

## 4. 主要コンポーネント設計

### 4.1 Robot 抽象

```python
class Robot(Protocol):
    joint_names: tuple[str, ...]          # 6 関節の順序を固定
    def get_state(self) -> RobotState: ...   # 関節角[rad], 速度
    def send_command(self, q_target: np.ndarray) -> None: ...  # 位置目標[rad]
    def close(self) -> None: ...
```

- 値の単位は両実装でラジアンに統一する。
- `SimRobot`: MuJoCo の `ctrl` に位置目標を入れてステップを進める。
- `So101Robot`(v1 後半 / v2): LeRobot のドライバをラップする。LeRobot 側の値の単位(正規化値・度など)から、ラジアンへの変換をこのクラス内に閉じ込める。

### 4.2 Policy 抽象

```python
class Policy(Protocol):
    def act(self, obs: Observation, goal: Goal) -> JointTrajectory: ...
```

- `IkPolicy`(v1): 目標 3D 位置から、接近 → 把持 → 持ち上げ → 移動 → 解放の軌道を作る。
- `SmolVlaPolicy`(v2): 画像・状態・言語指示から行動を出す。同じ `Policy` として差し替える。
- 注意: SmolVLA は VLM の検出結果ではなく、画像と言語指示を直接入力する。そのため、v2 では `vlm_planner` と `perception` を経由しない経路をデータフローに用意する(`dataflow_v2.yml`)。

### 4.3 VlmClient 抽象

```python
@dataclass(frozen=True)
class Detection:
    label: str                 # 例 "red block", "box"
    bbox_px: tuple[int, int, int, int]  # x1, y1, x2, y2(画像ピクセル)
    confidence: float | None

@dataclass(frozen=True)
class PlanResult:
    pick: Detection
    place: Detection

class VlmClient(Protocol):
    def plan(self, image: np.ndarray, instruction: str) -> PlanResult: ...
```

- 実装: `AnthropicClient`、`OpenAIClient`、`GeminiClient`、`OllamaClient`(Gemma 用。OpenAI 互換のため `OpenAIClient` の `base_url` 違いで共通化してよい)。
- 共通プロンプト: 「画像と指示から、掴む対象と置き場所のバウンディングボックスを JSON だけで返す」。スキーマはプロバイダの構造化出力機能が使えれば使い、使えなければ JSON 検証で代替する。
- 座標変換: プロバイダごとの座標系(絶対ピクセル、0〜1000 の正規化、`[y, x, y, x]` の順など)は各実装内で `bbox_px` に変換する。座標形式はプロンプトで明示し、実装時に実測で確認する。
- エラー: タイムアウト(30 秒)、レート制限、通信エラーは最大 2 回、指数バックオフで再試行する。JSON 不正と対象なしは `PlanError` を送出し、`motion` は動かさない。
- キャッシュ: (画像のハッシュ、指示、プロバイダ、モデル)をキーに結果を保存し、再現実験で API を呼ばないようにする。
- 設定: `configs/vlm.yaml` にプロバイダ名とモデル名。キーは環境変数。

### 4.4 Perception(画像 → ロボット座標)

1. バウンディングボックスの中心ピクセル `(u, v)` を取る。外れ値を避けるため、ボックス内の深度の中央値を使う。
2. 内部パラメータ(`fx, fy, cx, cy`)で、カメラ座標系の点 `P_cam` を計算する。
3. 外部パラメータ(カメラ → `baseframe`)の同次変換行列 `T_base_cam` で、ロボット座標系に変換する。
4. 変換行列は `configs/calibration.yaml` に置く。sim ではシーン定義から自動計算し、real では計測した値を入れる。

このモジュールは純粋関数にして、合成データでテストする。

### 4.5 IkPolicy

- 5 自由度のため、目標は「`gripperframe` を目標位置に置き、手先を下向きに近づける」とする。
- 解法: ヤコビアンの減衰最小二乗法(DLS)を、MuJoCo の `mj_jacSite` を使って反復する。姿勢は「手先の向きの下向き成分」だけを重みを下げて拘束する。ピッチを複数候補(真下から斜めまで)で試し、解が得られた最初のものを採用する。
- 関節可動域(`ctrlrange`)でクリップし、収束しなければ `IkError`(FR-06)。
- 軌道: `pre_grasp`(目標の真上 8 cm)→ `grasp` → 閉じる → `lift` → `pre_place`(箱の真上)→ `place` → 開く。区間は関節空間で線形補間し、最大速度で時間を決める。
- 数値(接近高さ、速度、グリッパー開閉量)は `configs/motion.yaml` に置き、実測で調整する。

### 4.6 安全フィルタ(motion)

要件 NFR-02、NFR-07 に対応する。

- 関節角を `ctrlrange` でクリップする。
- 1 ステップあたりの変化量を速度上限(実機の初回は通常の 30% 以下)で制限する。
- 作業範囲(`baseframe` から見た直方体)の外の目標を拒否する。
- 状態の更新が一定時間止まったら、その場で停止する。
- 実機: キー入力でトルクを切る非常停止(別スレッド / 別ノード)。

### 4.7 シーンとカメラ(sim)

- `scene.xml` を include し、`assets/scene_pickplace.xml` で次を追加する。
  - 作業台(平面)
  - 色違いのブロック 3 個(赤・青・緑。一辺 3 cm 程度の立方体から始める。把持の様子を見て調整)
  - 箱(底と 4 枚の壁。内寸を成功判定に使う)
  - 固定カメラ(作業台を斜め上から見る。RGB と深度)
- ブロックの初期配置は、シードから 20 cm 四方内にランダムに生成し、5 cm 以上離す(FR-12)。
- 把持が安定しない場合は、摩擦係数、グリッパーの `forcerange`、ブロックの質量を調整する。

### 4.8 Evaluator

- 成功条件(FR-07): 対象ブロックの中心が箱の内寸の内側、かつ縁より低く、解放 1 秒後の速度が 1 cm/s 以下。
- 試行の記録: シード、指示、プロバイダ、モデル、検出結果、成功 / 失敗、失敗の種類(VLM 失敗 / IK 失敗 / 把持失敗 / 配置失敗)、所要時間、検出誤差(真値との距離[mm])。
- 出力: `results/<run_id>/trials.csv` と、集計(成功率、プロバイダ別の表)。

### 4.9 Recorder(LeRobot 形式)

- 1 試行 = 1 エピソード。フレームに、画像、関節状態、行動(目標関節角)、言語指示を入れる。
- v2 のデモ収集(リーダーアームによるテレオペ)と同じ形式で保存し、そのまま SmolVLA の微調整に使えるようにする。
- LeRobot のデータセット仕様(フィーチャー名、型、fps)は実装時に確認する。

## 5. 設定ファイル

```text
configs/
├── sim.yaml          # robot: sim, camera: sim, seed
├── real.yaml         # robot: so101, ポート名(udev の固定名), camera
├── vlm.yaml          # provider, model, timeout, retries, cache
├── motion.yaml       # 速度上限, 接近高さ, 作業範囲
├── calibration.yaml  # T_base_cam, 内部パラメータ
└── eval.yaml         # 試行数, シード, 配置範囲, 判定しきい値
```

## 6. ディレクトリ構成

```text
so101-vlm-pickplace/
├── README.md
├── pyproject.toml            # uv で管理
├── .env.example
├── dataflow.yml              # v1
├── dataflow_v2.yml           # v2(SmolVLA 経路)
├── configs/
├── assets/
│   ├── so101/                # 公式モデルの取得先(ライセンス表示を同梱)
│   └── scene_pickplace.xml
├── src/so101_vlm/
│   ├── robot/                # base.py, sim.py, so101.py
│   ├── vlm/                  # base.py, schema.py, anthropic.py, openai.py, gemini.py, ollama.py, cache.py
│   ├── perception/           # projection.py, calibration.py
│   ├── policy/               # base.py, ik.py, trajectory.py, (v2) smolvla.py
│   ├── safety/               # limits.py, estop.py
│   ├── eval/                 # judge.py, runner.py, report.py
│   ├── record/               # lerobot_writer.py
│   └── nodes/                # camera.py, vlm_planner.py, perception.py, policy.py, motion.py, robot.py, recorder.py, evaluator.py
├── scripts/                  # run_sim.py, run_eval.py, render_video.py
├── tests/
└── docs/
```

## 7. テスト方針

| 対象 | 種類 | 内容 |
|---|---|---|
| projection | 単体 | 既知の点を投影・逆投影して一致するか |
| ik | 単体 | 到達可能な目標で収束、可動域外で `IkError` |
| judge | 単体 | 箱の内外・縁・速度の境界ケース |
| vlm.schema | 単体 | 不正 JSON、座標系の変換、対象なし |
| vlm クライアント | 単体 | SDK をモックしてリトライ・エラー処理を確認 |
| vlm クライアント | 実 API(任意) | キーがあるプロバイダのみ。なければスキップ |
| safety | 単体 | クリップ、速度制限、範囲外拒否、通信途絶 |
| シミュレーション | 統合 | 固定位置のブロックを IK で箱に入れる(VLM なし) |
| エンドツーエンド | 統合 | VLM の結果をキャッシュした状態で、1 試行を通す |
| CI | 自動 | 単体テストとカバレッジ 80% の確認(実 API・描画は除く) |

## 8. 実機への差し替えと v2

- 実機では `camera` と `robot` ノードの実装と、`calibration.yaml` を差し替える。udev でポートとカメラを固定名にする。
- `perception` の外部パラメータは、実機の設置後に計測する(チェッカーボード、またはロボットの手先を既知の位置に動かして求める)。
- v2: `dataflow_v2.yml` で `policy` を `SmolVlaPolicy` に差し替える。デモはリーダーアームで収集し、`recorder` が LeRobot 形式で保存する。評価は同じ `evaluator` / `runner` を使い、実機では成功判定を手動入力にする(真値がないため)。

## 9. 設計上のリスクと対策

| リスク | 対策 |
|---|---|
| 5 自由度で下向き把持が届かない | ピッチの複数候補、ブロック配置範囲の調整、箱の位置の調整 |
| MuJoCo での把持が不安定 | 摩擦、質量、`forcerange` の調整。大きめのブロック |
| VLM の座標の向き・単位の誤解 | プロンプトで明示、プロバイダ別の実測テスト、可視化(検出枠を画像に描く) |
| dora-rs が Mac で動かない | 実装初期に最小例で確認。ダメなら `multiprocessing` + キューで同じノード構成を実装 |
| sim と real のずれ | calibration の分離、速度制限、段階的な確認 |

# TASK-0009 開発コンテキストノート: sim 用キャリブレーションの自動生成

**作成日**: 2026-10-21
**要件名**: so101-vlm-pickplace
**タスクID**: TASK-0009
**タスクタイプ**: TDD
**前提**: TASK-0008(投影・逆投影、完了 2026-10-20)
**後続**: TASK-0024(VLM → 座標変換 → IK の結合)、TASK-0036(実機キャリブレーション)

## 0. 目的と完了条件

シーン定義のカメラ `cam_top` の位置と画角から内部パラメータ `K` と `T_base_cam` を計算し、`configs/calibration.yaml` に書き出す。

- [ ] `configs/calibration.yaml` が生成される
- [ ] 描画画像からブロックの真の位置を復元でき、誤差を記録した(受け入れ基準は 5 mm 以下。実測値は記録する)

## 1. 技術スタック

- Python 3.12、uv、pytest。実行は `uv run pytest -q`(現在 724 passed)
- MuJoCo(`mujoco`)、numpy、pyyaml(`yaml.safe_load` / `yaml.safe_dump`)
- 参照元: pyproject.toml
- アーキテクチャ: `perception/projection.py` は MuJoCo 非依存の純粋関数。`calibration.py` は MuJoCo 依存を持つ層として分ける
  - 参照元: src/so101_vlm/perception/projection.py(冒頭の docstring「numpy と標準ライブラリだけを import する (MuJoCo 非依存)」)

## 2. 開発ルール

プロジェクトの追加ルールファイル(AGENTS.md、docs/rule/tdd/ 配下)は存在しない。CLAUDE.md にはテスト実行コマンドのみがある。以下はグローバルルールと既存コードの慣習を適用する。

- TDD(RED → GREEN → REFACTOR)、カバレッジ 80% 以上
- 不変性: データは frozen な dataclass。配列は書き換えず新しく返す
- ファイルは 200〜400 行目安、最大 800 行。関数は 50 行未満
- 不正入力は ValueError(キー名や値を含むメッセージ)。ファイル欠落は FileNotFoundError
- YAML は `yaml.safe_load` のみ。未知キー・欠落キーは拒否する(既存の慣習)
- 秘密情報は扱わない(対象外)
- コミットは conventional commits(今回はコミットしない)
- 参照元: CLAUDE.md、src/so101_vlm/eval/eval_config.py、src/so101_vlm/safety/config.py、src/so101_vlm/eval/validation.py(`require_exact_keys`, `require_int`)

## 3. 関連実装

### 3.1 既存の投影関数(TASK-0008)
- `Camera`(frozen dataclass: fx, fy, cx, cy, width, height)、`camera_matrix`、`validate_T_base_cam`、`mujoco_cam_to_opencv`(`T @ diag(1,-1,-1,1)`)、`pixel_to_camera`、`camera_to_base`、`project_base_to_pixel`、`bbox_to_base_point`
- 参照元: src/so101_vlm/perception/projection.py

### 3.2 テスト専用ヘルパ(本タスクで製品化する対象)
- `camera_and_T_base_cam(model, data, camera_name, width, height)` が `tests/perception/test_projection_scene.py` にある。K と T_base_cam を MuJoCo から計算する
- 本タスクで `src/so101_vlm/perception/calibration.py` の関数へ移す。テストは製品関数を import する形に直す(S-01 は製品関数の検証になる)
- 参照元: tests/perception/test_projection_scene.py(`camera_and_T_base_cam`、`settled`、`depth`、`segmentation`、`calib` フィクスチャ)

### 3.3 描画と深度
- `render_rgbd(model, data, camera_name, width, height)` が RGB と z 深度(float32, m)を返す
- 参照元: src/so101_vlm/nodes/camera.py

### 3.4 シーン定数
- `SCENE_SPEC`: `camera_name="cam_top"`、`image_width=640`、`image_height=480`、`block_half_size=0.015`、`block_names`
- 参照元: src/so101_vlm/eval/scene_spec.py
- `PICKPLACE_SCENE_PATH`、`REPO_ROOT`: src/so101_vlm/robot/model_paths.py

## 4. 設計文書・要件・前提事実

### 4.1 設計・要件
- design.md 4.4: 内部パラメータは画角と画像サイズから求める。`T_base_cam` はカメラと `baseframe` の姿勢から求める。結果は `configs/calibration.yaml` に置く。sim は自動計算、real は計測値で置き換える
- requirements.md FR-04: 深度とカメラパラメータからロボット座標系の 3 次元位置を算出する
- requirements.md の「実機とのずれ」対策: 座標変換を `calibration` として分離する
  - 参照元: docs/design.md(4.4 節、ディレクトリ構成の `calibration.yaml` と `perception/calibration.py`)、docs/requirements.md(FR-04、リスク表)

### 4.2 確定済みの事実(TASK-0008 の調査・検証より)
- 主点: `cx=(w-1)/2`, `cy=(h-1)/2`(画素中心規約)。640x480 では cx=319.5, cy=239.5
- 焦点距離: `fy = (h/2)/tan(fovy/2)`, `fx = fy`。fovy=60, 640x480 で fy ≈ 415.6922
- MuJoCo のカメラは -z 方向を向く。`mujoco_cam_to_opencv` で OpenCV 規約へ変換する
- `baseframe` は原点・単位回転。よって `T_base_cam == T_world_cam`
- `validate_T_base_cam` は直交性を 1e-6 で見る。YAML には**全精度の float** を保存する(丸めると検証で落ちる)
- 投影の精度: 床の逆投影は z 誤差 0.0076 mm。上面中心の 1x1 bbox 復元の最大誤差は 1.34 mm
- 参照元: docs/implements/so101-vlm-pickplace/TASK-0008/projection-survey.md(63〜72 行、110 行、148 行)、docs/implements/so101-vlm-pickplace/TASK-0008/projection-verify-complete.md

### 4.3 シーンの camera 定義(確認済み)
- `assets/scene_pickplace.xml` 67〜68 行: `<camera name="cam_top" pos="0.45 0.30 0.55" fovy="60" xyaxes="...">`
- 調査時の MuJoCo での値: `model.cam_mode == [0]`(mjCAMLIGHT_FIXED)、`model.cam_fovy == [60.]`、visual の offwidth/offheight は 640x480
- 参照元: assets/scene_pickplace.xml

## 5. テスト関連情報

### 5.1 テストフレームワーク・設定
- pytest。pyproject.toml の `[tool.pytest.ini_options]`(28 行〜)
- 実行: `uv run pytest -q`、カバレッジ: `uv run pytest --cov`
- 参照元: pyproject.toml、CLAUDE.md

### 5.2 既存テストの構成と命名
- `tests/perception/test_projection.py`、`tests/perception/test_projection_extra.py`、`tests/perception/test_projection_scene.py`
- 命名: `test_<対象>_<内容>`、S-01..S-09 のケース ID をドックストリングに書く
- 統合テストは MuJoCo シーンを使う(`settled` フィクスチャ: `mj_forward` 後に `SETTLE_STEPS=500` 回 `mj_step`)
- 設定系テストは `tests/eval/test_eval_config.py`(YAML 読み込み・未知キー・型違反)、`tests/safety/test_config.py`
- 参照元: tests/perception/、tests/eval/test_eval_config.py、tests/safety/test_config.py

### 5.3 単体テスト(タスク定義の 1 件 + 追加)
- タスク定義: fovy=90°、高さ 480 → fy=240(式 `(480/2)/tan(45°)=240` で確認済み)
- 追加候補: 主点 `(w-1)/2`、fovy=60 で 415.6922、YAML の往復、strict キー、型違反、ファイル欠落、固定でないカメラ(`cam_mode != mjCAMLIGHT_FIXED`)は ValueError、カメラ名不明は ValueError
- 参照元: docs/tasks/so101-vlm-pickplace/TASK-0009.md(テストケース1)

### 5.4 統合テストの設計(真の位置の復元)
- 描画: `render_rgbd` で深度、`mujoco.Renderer` の segmentation で geom の画素マスク(既存 `segmentation` フィクスチャと同じ方法)
- 真の位置: 静定後の `data.geom_xpos[block_<色>_geom]` の上面中心(`+[0,0,block_half_size]`)。ブロック中心は上面から半辺を引く(S-07)
- 推奨 bbox: 上面の 4 隅を投影した bbox(S-05 の `_top_face_bbox`)、`shrink=0.5`
- 受け入れ: 上面中心との誤差 <= 5 mm(TASK-0009 の基準)。実測は 1〜2 mm の見込み。誤差の数値を note に記録する
- 注意: 輪郭全体の bbox を使うと側面が混ざり、水平 5〜15 mm のずれになる(S-09 の特性記録)。TASK-0024 への警告として残す
- 参照元: tests/perception/test_projection_scene.py(S-04〜S-09)、docs/tasks/so101-vlm-pickplace/TASK-0009.md(統合テスト1)

### 5.5 テストデータ
- 既存の `test_projection_scene.py` が使う描画・フィクスチャをそのまま流用できる。本タスクでは fixture を `conftest.py` に移すかは実装時に判断する(重複を避けるため)

## 6. 実装方針(推奨。ユーザー確認なしで既定値を採用)

### 6.1 配置
- `src/so101_vlm/perception/calibration.py`(新規)
- `scripts/generate_calibration.py`(新規、sim 用の生成スクリプト)
- `configs/calibration.yaml`(生成物、リポジトリにコミットする)
- `tests/perception/test_calibration.py`(新規)

### 6.2 公開 API
- `camera_from_mujoco(model, camera_name, width, height) -> Camera`
  - `cid = mj_name2id(...)`。見つからなければ ValueError
  - `model.cam_mode[cid] != mujoco.mjtCamLight.mjCAMLIGHT_FIXED` なら ValueError(追従カメラは時刻で姿勢が変わるため)
  - `fy = (height/2)/tan(radians(model.cam_fovy[cid])/2)`、`fx = fy`、`cx=(width-1)/2`、`cy=(height-1)/2`
- `t_base_cam_from_mujoco(model, data, camera_name, base_site="baseframe") -> np.ndarray(4,4, float64)`
  - `T_world_cam_mj` は `cam_xpos` / `cam_xmat` から、`T_world_base` は `site_xpos` / `site_xmat` から作る
  - `mujoco_cam_to_opencv(inv(T_world_base) @ T_world_cam_mj)` を返す。`validate_T_base_cam` で検証
- `@dataclass(frozen=True) class Calibration`
  - フィールド: `camera: Camera`、`T_base_cam: np.ndarray`、`source: str`(`"sim"` / `"real"`)、`camera_name: str`、`base_frame: str`
  - 配列は書き込み不可(`flags.writeable=False`)にコピーして保持する。`__eq__` は配列の比較で壊れるため `eq=False` とし、比較は helper で行う
- `save_calibration(calibration, path)`: `yaml.safe_dump` で書く(float は Python の float のまま)
- `load_calibration(path) -> Calibration`
  - `yaml.safe_load`。`require_exact_keys` で未知キー・欠落キーを拒否。`validate_T_base_cam` で T を検証。型違反は ValueError(キー名入り)
  - ファイルがなければ FileNotFoundError
- 既定パス: `REPO_ROOT / "configs" / "calibration.yaml"`(`eval_config.py` と同じ方式)

### 6.3 MuJoCo 固有の注意
- `mj_forward` は必要。`from_xml_path` 直後は `cam_xpos` / `cam_xmat` / `site_xpos` が未計算。`mj_forward(model, data)` を呼ぶ
- 本カメラは固定なので、静定ステップは不要(qpos0 での `mj_forward` で十分)。ただし生成スクリプトは決定論的にするため、`mj_resetData` の後で `mj_forward` を呼ぶ
- `cam_fovy` は度単位(degrees)。ラジアンに変換して使う
- 描画サイズ(640x480)は `visual.global` の offwidth/offheight と一致している(確認済み)

### 6.4 YAML レイアウト(案)

```yaml
metadata:
  source: sim              # sim | real
  camera_name: cam_top
  base_frame: baseframe
camera:
  fx: 415.69...            # 全精度で保存する
  fy: 415.69...
  cx: 319.5
  cy: 239.5
  width: 640
  height: 480
t_base_cam:                # 4x4、行優先のネストしたリスト
  - [r00, r01, r02, tx]
  - [r10, r11, r12, ty]
  - [r20, r21, r22, tz]
  - [0.0, 0.0, 0.0, 1.0]
```

- トップレベルキーは `metadata` / `camera` / `t_base_cam` の 3 つ。未知キーは拒否する
- 期待値: `t_base_cam` の平行移動は `[0.45, 0.30, 0.55]`(テストで確認済み)、回転は OpenCV 規約(MuJoCo の -z を +z に反転させた結果)

### 6.5 生成スクリプト
- `scripts/generate_calibration.py`: `PICKPLACE_SCENE_PATH` を読み、`mj_forward` を呼び、`camera_from_mujoco` と `t_base_cam_from_mujoco` で `Calibration`(`source="sim"`)を作り、`save_calibration` で `configs/calibration.yaml` に書く
- 出力先は引数で変えられるようにする(既定は `configs/calibration.yaml`)

### 6.6 ドリフトガード
- `tests/perception/test_calibration.py` で「コミット済みの `configs/calibration.yaml` を読んだ値」と「その場で生成した値」を比較する
- 比較方法: 配列は `np.allclose(atol=1e-9)`(MuJoCo のプラットフォーム差で末尾の桁が揺れうるため、ビット一致は要求しない)。YAML の往復そのものはビット一致で検証する
- 既存の `scene` 系テストとは独立させる

### 6.7 既存テストの扱い
- `tests/perception/test_projection_scene.py` の `camera_and_T_base_cam` は製品関数へ置き換える。S-01〜S-09 の判定内容は変えない
- 2 mm のしきい値(`SCENE_TOL_M`)は既存のまま。TASK-0009 の受け入れは 5 mm

## 7. 実機への接続(TASK-0036)

- 実機では同じ `Calibration` 型・同じ YAML 形式で書く。`metadata.source` を `real` にする
- `camera` は実機の内部パラメータ(計測または較正の値)、`t_base_cam` は計測値(4x4)を入れる
- `load_calibration` は変更なしで両方を読める。TASK-0036 で必要になるのは「計測値から `Calibration` を作る手順」と「その保存」のみ
- sim 用ファイルと実機用ファイルは同じパス(`configs/calibration.yaml`)を差し替える方針(design.md 4.4、要件書のリスク表)
- 参照元: docs/tasks/so101-vlm-pickplace/TASK-0036.md、docs/design.md(4.4)

## 8. TASK-0024 への引き渡し

- TASK-0024 は `load_calibration(path)` を使い、`calibration.camera` と `calibration.t_base_cam` を `bbox_to_base_point` に渡す
- 注意事項: bbox は上面の範囲を使う。輪郭全体の bbox では 5〜15 mm ずれる(S-09)
- 参照元: docs/tasks/so101-vlm-pickplace/TASK-0024.md、tests/perception/test_projection_scene.py(S-09)

## 9. 注意事項

- 技術的制約: 本タスクは sim 用の生成のみ。実機の値は TASK-0036 で置き換える
- 精度: 受け入れ基準は 5 mm。実測は 1〜2 mm と見込むが、確定値は実装時に測って記録する(🔴 のまま。しきい値は資料にない)
- 座標規約の誤りは検出しにくい: `validate_T_base_cam` は `diag(1,-1,-1)` の掛け忘れを検出できない(det は +1 のまま)。統合テストと床の平面テスト(S-02)で確認する
- セキュリティ・パフォーマンス: 対象外。描画は 1 回だけ行う
- 🟡/🔴 の項目(タスク定義より): 内部パラメータの計算(式は確認済み)、画角から焦点距離のテスト値(式で確認済み)、真の位置の復元(受け入れ 5 mm は資料にない仮の値)

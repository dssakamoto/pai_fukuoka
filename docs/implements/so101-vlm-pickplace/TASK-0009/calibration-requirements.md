# TASK-0009 要件定義書: sim 用キャリブレーションの自動生成(calibration)

**作成日**: 2026-10-21
**要件名**: so101-vlm-pickplace
**タスクID**: TASK-0009(TDD)
**機能名**: calibration
**前提**: TASK-0008(projection、完了)
**後続**: TASK-0024(VLM → 座標変換 → IK の結合)、TASK-0036(実機キャリブレーション)

参照した資料: `docs/implements/so101-vlm-pickplace/TASK-0009/note.md`、`docs/tasks/so101-vlm-pickplace/TASK-0009.md` / `TASK-0008.md` / `TASK-0024.md` / `TASK-0036.md`、
`docs/requirements.md`(FR-04、リスク表)、`docs/design.md`(4.4、5)、
`docs/implements/so101-vlm-pickplace/TASK-0008/projection-requirements.md` / `projection-survey.md` / `projection-verify-complete.md`、
`src/so101_vlm/perception/projection.py`、`tests/perception/test_projection_scene.py`、`src/so101_vlm/eval/scene_spec.py` / `eval_config.py` / `validation.py` / `layout.py`、`src/so101_vlm/safety/config.py`、`assets/scene_pickplace.xml`。

ユーザーへの質問は行っていない。資料にない決定は 🔴 とし、8 章に一覧と未決事項をまとめた。
数値はすべて使い捨てスクリプト(リポジトリ外の scratchpad、`uv run python`)で実測した。実測は 9 章にまとめた。

**信頼性レベル**: 🔵 要件・設計にほぼそのまま書かれている / 🟡 要件・設計・実測から妥当に推測 / 🔴 資料にない決定

---

## 1. 機能の概要

- 🔵 **何をするか**: シーン定義(`assets/scene_pickplace.xml`)のカメラ `cam_top` の画角と姿勢、および `baseframe` の姿勢から、内部パラメータ(`fx, fy, cx, cy`)と外部パラメータ `T_base_cam`(4x4)を計算し、`configs/calibration.yaml` に書き出す。読み込み関数も提供する(design.md 4.4 の 4 項)。
- 🔵 **解決する問題**: 座標変換に使う値を「シーン定義から自動計算した値(sim)」と「計測した値(real)」で差し替えられるように、設定ファイルとして分離する(requirements.md リスク表「実機とのずれ」への対策、design.md 4.4)。
- 🟡 **想定される利用者**: 後続モジュール。TASK-0024 の `run_trial` が `load_calibration` を呼び、`bbox_to_base_point` に渡す。TASK-0036 が同じ型・同じ形式で実機の値を保存する。開発者は生成スクリプトで再生成する。
- 🟡 **システム内の位置づけ**: `perception/projection.py`(MuJoCo 非依存の純粋関数)はそのまま。MuJoCo に依存する計算と YAML の入出力は新規の `perception/calibration.py` に置く(design.md のディレクトリ構成に `perception/calibration.py` と `configs/calibration.yaml` がある)。
- 🔵 **TASK-0008 との分担**: TASK-0008 の統合テストにあるテスト専用ヘルパ `camera_and_T_base_cam` を製品関数に移す(projection-requirements.md 4 章)。
- **参照したEARS要件**: FR-04、requirements.md リスク表(実機とのずれ)
- **参照した設計文書**: design.md 4.4、5(`configs/calibration.yaml`)、6(ディレクトリ構成)

---

## 2. 入力・出力の仕様

### 2.1 配置するファイル 🟡

| ファイル | 内容 |
|---|---|
| `src/so101_vlm/perception/calibration.py`(新規) | `camera_from_mujoco`、`t_base_cam_from_mujoco`、`Calibration`、`calibration_from_mujoco`、`save_calibration`、`load_calibration`、`calibrations_close`、`DEFAULT_CALIBRATION_PATH` |
| `scripts/generate_calibration.py`(新規) | 生成スクリプト(`--out`、`--check`) |
| `configs/calibration.yaml`(新規、生成物をコミットする) | sim の値 |
| `tests/perception/test_calibration.py`(新規) | 単体テスト・YAML テスト・ドリフトガード |
| `tests/perception/test_calibration_scene.py`(新規) | 描画を使う統合テスト(真の位置の復元) |
| `tests/perception/test_projection_scene.py`(変更) | テスト専用ヘルパを製品関数の呼び出しに置き換える。S-01〜S-09 の判定は変えない |

- 1 ファイル 400 行以内を目安にする。`calibration.py` が大きくなる場合は YAML 入出力を `calibration_io.py` に分けてよい(実装時に判断)。🔴

### 2.2 `camera_from_mujoco(model, camera_name, width, height) -> Camera` 🟡

- 入力: `model: mujoco.MjModel`、`camera_name: str`、`width: int`、`height: int`(正の整数。bool 不可)。
- 画像サイズは**引数で受け取る**。モデルからは読まない(MuJoCo の `cam_resolution` は既定の `[1, 1]` で、描画サイズの情報ではないため。実測)。スクリプトとテストは `SCENE_SPEC.image_width` / `image_height`(640x480)を渡す。🟡
- 計算(TASK-0008 で確定済みの規約):
  - `fovy = model.cam_fovy[cid]`(**度単位**。実測 `[60.]`)
  - `fy = (height / 2) / tan(radians(fovy) / 2)`、`fx = fy`(正方画素)
  - `cx = (width - 1) / 2`、`cy = (height - 1) / 2`(画素中心規約)
- 640x480・fovy=60° で `fx = fy = 415.6921938165307`、`cx = 319.5`、`cy = 239.5`。🔵(survey (a)、実測)
- 戻り値: `Camera`(TASK-0008 の frozen dataclass)。フィールドは Python の `float` / `int`(numpy の型を残さない。YAML へそのまま書けるようにするため)。🔴
- エラー(すべて `ValueError`、メッセージにカメラ名を含める):
  - カメラ名がない(`mj_name2id` が -1。実測)
  - `model.cam_mode[cid] != mjCAMLIGHT_FIXED`(追従カメラは姿勢が時刻で変わるため)。実測で `cam_top` は `cam_mode == 0 == mjCAMLIGHT_FIXED`
  - fovy が (0, 180) の外
  - width / height が正の整数でない
- 🔴 `fovx` / `cam_intrinsic` を使うカメラ(`sensorsize` 指定)は扱わない。`model.cam_sensorsize[cid]` が 0 でないときは `ValueError`(実測で `cam_top` は `[0, 0]`)。

### 2.3 `t_base_cam_from_mujoco(model, data, camera_name, base_site="baseframe") -> np.ndarray` 🟡

- 入力: `model`、`data`(**`mj_forward` 済み**であること)、`camera_name`、`base_site`(site 名)。
- 計算: `T_world_cam_mj = [cam_xmat | cam_xpos]`、`T_world_base = [site_xmat | site_xpos]`、
  `T_base_cam = mujoco_cam_to_opencv(inv(T_world_base) @ T_world_cam_mj)`。🔵(design.md 4.4「カメラと baseframe の姿勢から」、survey 88 行)
- 戻り値: `validate_T_base_cam` を通した (4,4) float64 の**新しい書き込み可能な配列**。`data` は変更しない。🟡
- エラー(`ValueError`): カメラ名・site 名がない、固定カメラでない、`data` が未計算(`site_xmat` がすべて 0 のとき。実測では `from_xml_path` 直後は `cam_xmat` / `site_xmat` がすべて 0 で、そのまま使うと `validate_T_base_cam` が回転の直交性で落ちる)。🟡
- `mj_forward` を関数の中で呼ぶかについて: 呼ばない(`data` を変えないため)。呼び出し側(`calibration_from_mujoco` と生成スクリプト)が `mj_resetData` → `mj_forward` を行う。🔴
- 実測値(現在のシーン):
  - `baseframe` の位置は原点、回転は厳密に単位行列(`site_xmat == I` がビット一致)。
  - したがって `T_base_cam == T_world_cam_mj @ diag(1,-1,-1,1)` がビット一致で成り立つ。
  - 平行移動 `[0.45, 0.30, 0.55]`、回転の直交誤差 2.2e-16、det = 1.0000000000000002。
  - 500 ステップ静定後も、`mj_resetData` 後も、T はビット一致で変わらない(固定カメラ・固定 base のため)。
- 一般性(base site を移動・回転したとき)🟡: 9.2 節の小さな MJCF で確認した。式は一般に正しい。かけ算の順序を逆(`T_world_cam @ inv(T_world_base)`)にすると最大 0.71 ずれ、base を無視すると 225 mm ずれる。

### 2.4 `Calibration`(frozen dataclass)🟡

```python
@dataclass(frozen=True, eq=False)
class Calibration:
    camera: Camera
    T_base_cam: np.ndarray      # (4,4) float64、書き込み不可のコピーを保持
    source: str                 # "sim" | "real"
    camera_name: str
    base_frame: str
```

- `__post_init__` で検証する: `camera` が `Camera`、`T_base_cam` が `validate_T_base_cam` を通る、`source` が `{"sim", "real"}` のどれか、`camera_name` / `base_frame` が空でない `str`。違反は `ValueError`。🟡
- `T_base_cam` は検証後のコピーを `flags.writeable = False` にして保持する(`object.__setattr__` で設定。safety/config.py と同じ方式)。呼び出し側が渡した配列を後から書き換えても影響しない。🔴
- `eq=False`(ndarray を含むため `==` が配列を返し、dataclass の既定の `__eq__` は壊れる)。比較は `calibrations_close` を使う。🔴
- `source` は `Literal["sim", "real"]` の型注釈にする。🔵(design.md 4.4「sim では自動計算、real では計測値」)
- `camera_name` / `base_frame` は記録用(どのカメラ・どの基準座標の値か)。読み込み側はこれで分岐しない。🔴

### 2.5 `calibration_from_mujoco(model, data, camera_name, width, height, base_site="baseframe") -> Calibration` 🔴

- 2.2 と 2.3 をまとめ、`source="sim"` の `Calibration` を返す。`data` は `mj_forward` 済みであることを前提とする。
- 生成スクリプトとドリフトガードはこの関数を使う(同じ手順で作るため)。

### 2.6 YAML の形式 🟡(design.md 5「T_base_cam, 内部パラメータ」を具体化。キー名と構成は 🔴)

```yaml
# 自動生成ファイル (source: sim)。手で編集しないこと。
# 再生成: uv run python scripts/generate_calibration.py
# 確認:   uv run python scripts/generate_calibration.py --check
version: 1
metadata:
  source: sim
  camera_name: cam_top
  base_frame: baseframe
camera:
  fx: 415.6921938165307
  fy: 415.6921938165307
  cx: 319.5
  cy: 239.5
  width: 640
  height: 480
t_base_cam:
- [-0.6896566706214934, 0.6405487761198795, -0.3377439001352973, 0.45]
- [0.7241365041670508, 0.6100489807755903, -0.3216621898628844, 0.3]
- [-2.7755575615628914e-17, -0.46640916207337524, -0.8845691004856614, 0.55]
- [0.0, 0.0, 0.0, 1.0]
```

(上は実測の値。T は OpenCV 規約のカメラ座標 → base)

- トップレベルのキーは `version` / `metadata` / `camera` / `t_base_cam` の 4 つ。順序はこの順で固定(`sort_keys=False`)。🔴
- `version` は整数 `1`。読み込み時に 1 以外は `ValueError`。形式を変えたときの検出用。🔴
- `t_base_cam` は 4 行 x 4 列の行優先のネストしたリスト。各行は flow 形式(`[a, b, c, d]`)で 1 行に書く。`yaml.SafeDumper` のサブクラスで行だけ flow 形式にする(読み込みは `yaml.safe_load` のまま)。🔴
- 先頭にコメントで「自動生成であること」と「再生成・確認のコマンド」を書く。`yaml.safe_dump` はコメントを出せないため、文字列を前に付ける。`source: real` のときは「実機の計測値。scripts/generate_calibration.py で上書きしないこと」という文言にする。🔴
- 時刻・ホスト名などの非決定的な値は書かない(再生成で差分が出ないようにするため)。🔴

### 2.7 数値の正確さ(保存)🟡

- 浮動小数は**丸めずに**書く。`validate_T_base_cam` の直交判定(1e-6)を確実に通し、往復で値を変えないため(projection-requirements.md 2.3 の申し送り)。🔵
- 書く前に numpy の型を Python の型にする(`T.tolist()`、`float(x)`、`int(x)`)。**実測: `yaml.safe_dump` は `np.float64` / `np.float32` / `np.int64` を `RepresenterError` で拒否する**(np.float64 は float のサブクラスだが SafeRepresenter は型の完全一致で探すため)。`ndarray.tolist()` は Python の `float` を返す(実測)。🔵(実測)
- 往復の正確さ(実測、9.3 節): Python の `float` は `yaml.safe_dump` → `yaml.safe_load` で**ビット一致**で戻る(乱数のビット列 約 20 万個・一様乱数 10 万個・微小値・-0.0・非正規化数・最大値などの約 31 万個で不一致 0)。実際の行列(`-2.7755575615628914e-17` を含む)もビット一致。🔵(実測)
- PyYAML は指数表記に `.0` を補う(`1e-17` → `1.0e-17`)ため、書き出した値が文字列として読まれることはない。ただし**手で `1e-3` と書くと YAML 1.1 では文字列として読まれる**(実測)。読み込み側はこれを型違反の `ValueError` にする。🟡

### 2.8 `save_calibration(calibration, path) -> None` 🟡

- 2.6 の形式で書く。`path` は `str | Path`。親ディレクトリがなければ `FileNotFoundError`(自動作成しない)。🔴
- 書いた後に読み直して元と完全一致(`calibrations_close(atol=0)`)するかは、テストで確認する(関数の中では行わない)。🔴
- 書き込みは UTF-8。上書きする。

### 2.9 `load_calibration(path=DEFAULT_CALIBRATION_PATH) -> Calibration` 🟡

- `path` は `str | Path`。`DEFAULT_CALIBRATION_PATH = REPO_ROOT / "configs" / "calibration.yaml"`(eval_config.py と同じ方式)。🔵(design.md 4.4、5)
- `yaml.safe_load` のみ使う。🔵(既存の慣習)
- 厳格な検証(違反はすべて `ValueError`、キー名と値をメッセージに含める)🔴(既存の `require_exact_keys` / `require_int` / `require_number` を使う):
  - 空ファイル(`None`)・最上位が mapping でない
  - キーの過不足: トップレベル `{version, metadata, camera, t_base_cam}`、`metadata` は `{source, camera_name, base_frame}`、`camera` は `{fx, fy, cx, cy, width, height}`
  - `version` が整数 1 でない
  - `metadata.source` が `"sim"` / `"real"` 以外、`camera_name` / `base_frame` が空でない文字列でない
  - `camera` の `fx, fy, cx, cy` が有限の実数でない(bool・文字列 `"1e-3"`・`.inf`・`.nan` は拒否。整数は許して float にする)。`fx, fy > 0`
  - `width, height` が正の整数でない(bool・float は拒否)
  - `t_base_cam` が 4 個の「4 個の数」からなるリストでない(各要素は bool・文字列を拒否)、`validate_T_base_cam` を通らない
- ファイルがない: `FileNotFoundError`(`open` の例外をそのまま出す。既存と同じ)。🔵(既存の慣習)
- `width` / `height` と実際の画像サイズの一致はここでは見ない。`bbox_to_base_point` が深度画像の形状と照合する(TASK-0008)。🟡
- 🔴 PyYAML は重複キーを黙って後勝ちにする(実測)。本タスクでは検出しない(既知の制約として記録)。

### 2.10 `calibrations_close(a, b, *, atol=1e-9) -> bool` 🔴

- 文字列・整数(`source`、`camera_name`、`base_frame`、`width`、`height`)は完全一致、浮動小数(`fx, fy, cx, cy`、`T_base_cam` の全要素)は `abs(a - b) <= atol`(`np.allclose(rtol=0, atol=atol)`)。
- 生成スクリプトの `--check` とドリフトガードのテストは**同じこの関数を同じ `atol=1e-9` で**使う。
- `atol=1e-9` の理由: MuJoCo のプラットフォームや版の違いで末尾の桁が揺れても誤検出しないため。1e-9 の差は 0.6 m 先で 1e-9 m 未満で、精度に影響しない。YAML の往復そのものは `atol=0`(ビット一致)でテストする。🟡

### 2.11 生成スクリプト `scripts/generate_calibration.py` 🟡

- 手順: `MjModel.from_xml_path(PICKPLACE_SCENE_PATH)` → `MjData` → `mj_resetData` → `mj_forward` → `calibration_from_mujoco(model, data, SCENE_SPEC.camera_name, SCENE_SPEC.image_width, SCENE_SPEC.image_height)` → 書き出し。🟡
- CLI(`argparse`)🔴:
  - `--out PATH`(既定 `configs/calibration.yaml`。`DEFAULT_CALIBRATION_PATH`)
  - `--check`: 書き込まず、`--out` のファイルを `load_calibration` で読み、生成した値と `calibrations_close(atol=1e-9)` で比べる。一致なら終了コード 0、不一致は 1(差の最大値を標準エラーに出す)、ファイルなし・読み込みエラーは 2。
  - 既存ファイルの `metadata.source` が `real` のときは、上書きを拒否して終了コード 2(実機の値を sim で消さないため)。`--force` で上書きを許す。🔴
- ロジックは `main(argv: list[str] | None = None) -> int` に置き、テストから直接呼べるようにする(既存の `scripts/capture_scene.py` と同じく `uv run python scripts/...` で実行)。🔴
- 実行時間: 描画しないため 1 秒未満(モデル読み込みと `mj_forward` のみ)。🟡

### 2.12 データフロー 🔵

```
assets/scene_pickplace.xml ──(scripts/generate_calibration.py)──▶ configs/calibration.yaml   [sim]
       実機の計測 ──(TASK-0036: scripts/calibrate_real.py)──▶ configs/calibration.yaml   [real]
configs/calibration.yaml ──load_calibration──▶ Calibration(camera, T_base_cam)
   ──▶ bbox_to_base_point(bbox, depth, camera, T_base_cam) ──▶ base 座標の上面の点 ──(z − 0.015)──▶ IK(TASK-0024)
```

- **参照したEARS要件**: FR-04
- **参照した設計文書**: design.md 4.4、5、`src/so101_vlm/perception/projection.py`(`Camera`、`validate_T_base_cam`、`mujoco_cam_to_opencv`)

---

## 3. 制約条件

- 🔵 **規約**: カメラ座標系は OpenCV 規約(x 右、y 下、光軸 +z)。MuJoCo のカメラ(光軸 -z)は `mujoco_cam_to_opencv` で変換する。主点は `(w-1)/2`。深度は z 深度(TASK-0008 で確定)。
- 🔵 **層の分離**: `projection.py` は MuJoCo を import しないまま保つ。MuJoCo 依存は `calibration.py` に閉じる。
- 🟡 **不変性**: `Calibration` は frozen。配列は書き込み不可のコピー。関数は入力(`model`、`data`、渡された配列)を変更しない。
- 🟡 **エラー方針**: 不正な値・設定は `ValueError`、ファイル欠落は `FileNotFoundError`。`TypeError` は使わない(TASK-0008 と同じ)。
- 🟡 **決定性**: 同じシーンからは毎回ビット一致の値が出る(実測。静定の有無・`mj_resetData` に依存しない)。
- 🟡 **既存テストの維持**: 現在の 724 件はすべて通ったままにする。`test_projection_scene.py` の許容誤差(`SCENE_TOL_M = 2 mm` など)は変えない。
- 🟡 **テスト時間**: 統合テストの描画は 1 配置あたり約 0.28 秒(50 配置で 14 秒、実測)。テストに入れる配置数は 8 章 OPEN-2。
- 🔵 **セキュリティ**: 秘密情報は扱わない。YAML は `safe_load` のみ(任意オブジェクトを作らない)。
- **参照したEARS要件**: FR-04、NFR-05(純粋ロジックに単体テスト)
- **参照した設計文書**: design.md 4.4、TASK-0008 projection-requirements.md 3 章

---

## 4. 想定される使用例

### 4.1 基本パターン 🔵

```bash
uv run python scripts/generate_calibration.py          # configs/calibration.yaml を書く
uv run python scripts/generate_calibration.py --check  # コミット済みのファイルが最新か確認 (CI 向け)
```

```python
cal = load_calibration()                     # configs/calibration.yaml
p_top = bbox_to_base_point(bbox, depth, cal.camera, cal.T_base_cam, shrink=0.5)
p_center = p_top - [0.0, 0.0, SCENE_SPEC.block_half_size]
```

### 4.2 受け入れ(統合テスト1: 真の位置の復元)🟡(しきい値 5 mm はタスク定義の仮の値で 🔴)

- 手順: シーンを静定(500 ステップ)→ `render_rgbd` で深度 → `load_calibration`(YAML を往復した値)→ 各ブロックの**上面の 4 隅を投影した bbox**(静定後の `geom_xpos` と `geom_xmat` から計算)→ `bbox_to_base_point(..., shrink=0.5)` → 静定後の上面中心(`geom_xpos + R @ [0,0,0.015]`)との距離。
- 合格: 距離 ≤ 5 mm。
- **輪郭全体の bbox は使わない**(側面が混ざり水平に 6.9〜10.2 mm ずれる。TASK-0008 S-09 の特性記録)。🔵
- 実測(9.4 節): 既定配置 red 0.761 / blue 0.142 / green 0.615 mm。ランダム配置 50 個(seed 0〜49、eval.yaml の既定、計 150 個)で 平均 0.495 / 中央値 0.469 / p95 1.042 / 最大 1.235 mm(seed 34 の blue)。5 mm 超・2 mm 超ともに 0 件。
- 測った誤差は verify-complete の文書に記録する(タスクの完了条件「誤差を記録した」)。

### 4.3 生成とドリフトガード 🟡

- `test_calibration.py` で、コミット済みの `configs/calibration.yaml` を `load_calibration` で読み、その場で `calibration_from_mujoco` で作った値と `calibrations_close(atol=1e-9)` で比べる。シーンの XML や計算式を変えて再生成を忘れると落ちる。
- コミット済みファイルの `source` が `real` に差し替わった場合、このテストは `pytest.skip` する(実機の値を sim と比べても意味がないため)。🔴

### 4.4 エッジケース・エラーケース

| ケース | 期待 | 信頼性 |
|---|---|---|
| fovy=90°、高さ 480 | `fy = 240`(タスク定義のテストケース1) | 🟡 |
| fovy=60°、640x480 | `fx = fy = 415.6921938165307`(1e-9)、`cx=319.5`、`cy=239.5` | 🔵 |
| 幅と高さが異なる(例 320x240) | `fy = 120/tan(fovy/2)`、`cx = 159.5` | 🟡 |
| カメラ名がない | `ValueError` | 🟡 |
| 追従カメラ(`mode="track"` など、`cam_mode != 0`) | `ValueError` | 🟡 |
| base site 名がない | `ValueError` | 🟡 |
| `mj_forward` 前の `data` | `ValueError`(回転が 0 行列) | 🟡 |
| base site を移動・回転した MJCF | 解析値と 1e-12 以内で一致(9.2 節) | 🟡 |
| 保存 → 読み込み | 全フィールドがビット一致 | 🔵(実測) |
| numpy の型を含む `Calibration` を保存 | 正常に書ける(保存前に Python 型へ変換) | 🔵(実測) |
| 未知キー・欠落キー(各階層) | `ValueError`(キー名入り) | 🟡 |
| `version: 2` | `ValueError` | 🔴 |
| `source: test` | `ValueError` | 🔴 |
| `fx: 1e-3`(文字列として読まれる)、`fx: true`、`fx: .nan`、`fx: -1` | `ValueError` | 🟡 |
| `width: 640.0`、`width: 0` | `ValueError` | 🟡 |
| `t_base_cam` が 3x4、行の長さ違い、要素が文字列 | `ValueError` | 🟡 |
| `t_base_cam` の回転が直交でない(1e-6 を超える) | `ValueError` | 🔵 |
| 空ファイル、最上位がリスト | `ValueError` | 🟡 |
| ファイルなし | `FileNotFoundError` | 🔵 |
| `str` と `Path` の両方 | どちらも読める・書ける | 🔵 |
| `Calibration` の配列を書き換えようとする | `ValueError`(read-only) | 🔴 |
| 渡した配列を後から書き換える | `Calibration` の値は変わらない | 🔴 |
| `--check`(一致 / 不一致 / ファイルなし) | 終了コード 0 / 1 / 2 | 🔴 |
| 既存が `source: real` で生成を実行 | 終了コード 2、ファイルは変わらない | 🔴 |

- **参照したEARS要件**: FR-04
- **参照した設計文書**: design.md 4.4、TASK-0008 projection-testcases(S-01〜S-09)

---

## 5. EARS要件・設計文書との対応関係

- **参照したユーザストーリー**: 指示した色のブロックを箱に入れる(画像 → ロボット座標の変換部分)
- **参照した機能要件**: FR-04
- **参照した非機能要件**: NFR-05
- **参照したEdgeケース**: requirements.md リスク表「実機とのずれ」
- **参照した受け入れ基準**: TASK-0009 完了条件(calibration.yaml の生成、真の位置の復元と誤差の記録)、テストケース1(fovy=90°→fy=240)、統合テスト1(5 mm)
- **参照した設計文書**:
  - **アーキテクチャ**: design.md 4.4(Perception)、6(`perception/calibration.py`)
  - **設定**: design.md 5(`configs/calibration.yaml`)
  - **型定義**: `src/so101_vlm/perception/projection.py`(`Camera`、`validate_T_base_cam`、`mujoco_cam_to_opencv`、`bbox_to_base_point`)
  - **先行タスク**: TASK-0008 projection-requirements.md 4 章(分担)、projection-survey.md、projection-verify-complete.md 4 章
  - **データベース / API**: 該当なし

---

## 6. 後続タスクでの使い方

### 6.1 TASK-0024(VLM → 座標変換 → IK)🟡
- `load_calibration()`(既定パス)で読み、`cal.camera` と `cal.T_base_cam` を `bbox_to_base_point` に渡す。
- VLM の bbox が輪郭全体を囲む場合は水平 5〜15 mm の偏りが出る(S-09)。上面の範囲を使うか、偏りを許容するかは TASK-0024 で決める。本タスクの受け入れは上面 bbox で行う。
- 描画サイズは `cal.camera.width` / `height` を使う(深度画像の形状と食い違えば `bbox_to_base_point` が `ValueError`)。

### 6.2 TASK-0036(実機)🟡
- 同じ `Calibration` 型・同じ YAML 形式で、`source="real"` として `save_calibration` で書く。`load_calibration` は変更なしで読める。
- 同じパス `configs/calibration.yaml` を差し替える(design.md 4.4)。sim と real を両方残す必要が出たら `--out` / `load_calibration(path)` で別ファイルにできる(8 章 OPEN-3)。
- 実機カメラのレンズ歪み係数は本形式に含めない(8 章 OPEN-1)。

---

## 7. テスト方針(tdd-testcases への申し送り)

- 単体(描画なし、速い): 焦点距離の式(90°→240、60°→415.69…)、主点、エラー各種、`Calibration` の検証と不変性、YAML の往復(ビット一致)、厳格な読み込み、`calibrations_close`、CLI(`main(argv)` を `tmp_path` で呼ぶ)。
- MuJoCo(描画なし): 実シーンの値(S-01 相当、平行移動 `[0.45,0.30,0.55]`)、`mj_forward` 前の拒否、追従カメラの拒否(小さな MJCF)、base site を移動・回転した小さな MJCF での解析的な一致(9.2 節の方法)、ドリフトガード。
- 統合(描画あり): 4.2 節の受け入れ。輪郭全体の bbox は使わない。
- 既存の `test_projection_scene.py` は `calib` フィクスチャを `calibration_from_mujoco`(または `camera_from_mujoco` と `t_base_cam_from_mujoco`)に置き換えるだけにする。

---

## 8. 決定と仮定の一覧(ユーザー確認なしで決めたもの)と未決事項

### 8.1 決定・仮定
1. 🔴 API 名・引数は note.md 6.2 の案を採用し、`calibration_from_mujoco` と `calibrations_close` を追加した。
2. 🔴 `Calibration` は `eq=False`、配列は書き込み不可のコピー。`source` は `"sim" | "real"`。
3. 🔴 YAML のトップレベルは `version` / `metadata` / `camera` / `t_base_cam`。`version: 1`。キー順は固定。行列の行は flow 形式。
4. 🔴 先頭に生成元と再生成方法のコメントを書く。時刻などは書かない。
5. 🟡 浮動小数は丸めずに書き、保存前に Python の型へ変換する(実測で np 型は書けない)。
6. 🔴 `--check` とドリフトガードは同じ `calibrations_close(atol=1e-9)`。YAML の往復はビット一致で検証する。
7. 🔴 既存ファイルが `source: real` のとき、生成スクリプトは `--force` なしでは上書きしない。ドリフトガードは skip する。
8. 🔴 `t_base_cam_from_mujoco` は `mj_forward` を呼ばない。未計算の `data` は `ValueError`。
9. 🔴 `sensorsize` を使うカメラは `ValueError`(fovy のみ対応)。
10. 🔴 カメラが動く body に付いている場合(`cam_bodyid != 0`)も拒否はしない。`data` の現在の状態で計算する(生成スクリプトは `mj_resetData` 後の状態を使う)。現在のシーンは `cam_bodyid == 0`(ワールドに固定)。
11. 🔴 重複キーは検出しない(PyYAML の制約)。

### 8.2 未決事項(OPEN、既定値で進める)
- **OPEN-1**: 実機用のレンズ歪み係数(`dist`)を YAML に入れるか。既定: 入れない(sim は歪みなし。TASK-0036 で必要なら `version: 2` として追加する)。
- **OPEN-2**: 統合テストに入れるランダム配置の数。既定: 既定配置の 3 個 + seed 0〜9 の 10 配置(約 3 秒)。50 配置の結果は本書 9.4 節と verify-complete の文書に記録する。
- **OPEN-3**: sim と real を同じ `configs/calibration.yaml` で差し替えるか、別ファイルにするか。既定: design.md 4.4 に従い同じパスを差し替える。
- **OPEN-4**: 受け入れのしきい値 5 mm はタスク定義の仮の値。実測の最大は 1.235 mm。テストのしきい値は 5 mm のまま(TASK-0008 の 2 mm は S-04/S-05 に残る)。厳しくするかは利用者の判断。

---

## 9. 実測の記録(使い捨てスクリプト、リポジトリ外で実行)

環境: Python 3.12、mujoco 3.16.0、numpy 2.5.4、PyYAML 6.0.3。

### 9.1 現在のシーンの MuJoCo の値
- `ncam=1`、`cam_mode=[0]`(`mjCAMLIGHT_FIXED=0`)、`cam_fovy=[60.]`(度)、`cam_bodyid=[0]`、`cam_sensorsize=[0,0]`、`cam_resolution=[[1,1]]`、offwidth/offheight=640/480。
- `mj_forward` 前: `cam_xpos=0`、`cam_xmat=0`、`site_xmat=0`(すべて 0)。→ `mj_forward` は必須。
- `baseframe`: body 1 上、位置 0、回転は単位行列(ビット一致)。
- T_base_cam は `T_world_cam_mj @ diag(1,-1,-1,1)` とビット一致。直交誤差 2.2e-16。静定後・`mj_resetData` 後もビット一致。

### 9.2 base site を移動・回転したときの一般性
- 小さな MJCF: `robot_base` body(pos `0.10 -0.05 0.02`、euler `0 0 37`)の中に `baseframe` site(pos `0.03 0.02 0.01`、euler `5 -8 20`)、固定カメラ(pos `0.6 0.5 0.7`、fovy 50)、一辺 5 cm の箱、床。320x240。
- 解析的な確認(描画なし): 乱数の世界座標の点 1000 個について、MuJoCo のカメラ座標(`R_wc^T (p - t_wc)` を OpenCV 規約に反転)を `camera_to_base(·, T)` で変換した値と `inv(T_world_base) @ p` の差は最大 6.7e-16。
- 描画での確認: 箱の上面中心を投影した画素の描画深度から逆投影すると、base 座標で 0.150 mm 以内。床 2150 点を逆投影して世界座標に戻すと |z| 最大 0.0068 mm。
- 負の対照: base を無視すると 224.8 mm ずれる。かけ算の順序を逆にすると行列の要素が最大 0.706 ずれる。
- → テストは描画なしの解析的な方法で十分(速く、決定的)。

### 9.3 YAML 往復の正確さ
- Python `float` 約 31 万個(ランダムなビット列の有限値、一様乱数、正規乱数 1e-12 規模、`0.0`、`-0.0`、`5e-324`、最大値、`1e22`、`0.1`、`1/3`、実際の値)で `safe_dump` → `safe_load` の不一致 0(ビット比較)。
- 実際の T(`-2.7755575615628914e-17` を含む)と `fy=415.6921938165307` もビット一致。
- `np.float64` / `np.float32` / `np.int64` を `safe_dump` すると `RepresenterError`。`ndarray.tolist()` は Python の `float`。
- 読み込み: `1e-3` → 文字列、`1.0e-3` → float、`.inf` / `.nan` → float、`415` → int、`true` → bool、`0x10` → int(16)。重複キーは後勝ち。空ファイルは `None`。
- `sort_keys=True`(既定)だとキーがアルファベット順になる → `sort_keys=False` を指定する。

### 9.4 真の位置の復元(上面 bbox + shrink=0.5、YAML 往復後の値)
| 配置 | 誤差 |
|---|---|
| 既定 red(bbox 251,143,278,165) | 0.761 mm(z −0.004 mm) |
| 既定 blue(bbox 227,202,257,228) | 0.142 mm(z −0.005 mm) |
| 既定 green(bbox 300,197,330,222) | 0.615 mm(z −0.003 mm) |
| ランダム 50 配置 x 3 個(seed 0〜49) | 平均 0.495 / 中央値 0.469 / p95 1.042 / 最大 1.235 mm(seed 34 blue)。2 mm 超 0、5 mm 超 0 |
- 50 配置の実行時間は約 14 秒(描画と 500 ステップの静定を含む)。

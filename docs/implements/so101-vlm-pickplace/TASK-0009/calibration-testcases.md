# TASK-0009 テストケース定義書: sim 用キャリブレーションの自動生成(calibration)

**作成日**: 2026-10-21
**要件名**: so101-vlm-pickplace
**タスクID**: TASK-0009(TDD)
**機能名**: calibration
**要件定義書**: `docs/implements/so101-vlm-pickplace/TASK-0009/calibration-requirements.md`(以下「要件 x.y」)
**タスクノート**: `docs/implements/so101-vlm-pickplace/TASK-0009/note.md`

**信頼性レベル**: 🔵 要件・設計・実測にほぼそのまま書かれている / 🟡 要件・実測から妥当に推測 / 🔴 資料にない決定(テスト側で決めたもの)

ユーザーへの質問は行っていない。決定済み事項(呼び出し元の指示)は次のとおりで、本書では再検討しない。

- **OPEN-4(しきい値)**: タスクの完了条件は 5 mm のまま。**自動テストは 2 mm 以下を判定する**(実測最大 1.235 mm / 150 個。TASK-0008 の `SCENE_TOL_M` と同じ値)。
- **OPEN-2(配置数)**: 統合テストは既定配置 + seed 0〜9(約 3 秒)。50 配置の結果は文書(要件 9.4、verify-complete)だけに記録する。
- その他の API・YAML 形式は要件 2 章のとおり。

数値は使い捨てスクリプト(リポジトリ外の scratchpad、`uv run python`)で確認した。確認結果は 9 章にまとめた。

---

## 0. テストファイルとケース ID の体系

| ファイル | 内容 | 描画 | ケース ID |
|---|---|---|---|
| `tests/perception/test_calibration.py`(新規) | `camera_from_mujoco`(C)、`t_base_cam_from_mujoco`(T)、`Calibration`(D)、`calibration_from_mujoco`(F)、`save_calibration`(Y)、`load_calibration`(L)、`calibrations_close`(K)、ドリフトガード(R) | なし | C-, T-, D-, F-, Y-, L-, K-, R- |
| `tests/perception/test_calibration_scene.py`(新規) | コミット済み YAML を使った真の位置の復元・床の平面 | あり | I- |
| `tests/perception/test_generate_calibration.py`(新規)🔴 | `scripts/generate_calibration.py` の `main(argv)` と 1 回だけのサブプロセス実行 | なし | G- |
| `tests/perception/test_projection_scene.py`(変更) | テスト専用ヘルパ `camera_and_T_base_cam` を `calibration_from_mujoco` に置き換える。S-01〜S-09 の判定は変えない | あり | (既存 S-01〜S-09) |

- 🔴 CLI のテストを別ファイルにしたのは、`test_calibration.py` を 400 行以内に収めるため(要件 2.1 の目安)。要件 7 章は「CLI(`main(argv)` を `tmp_path` で呼ぶ)」を単体テストに含めており、置き場所だけを決めた。
- 命名は既存に合わせて `test_<対象>_<内容>`。ドックストリングの先頭にケース ID と信頼性マークを書く(例: `"""C-01: fovy=90°・高さ 480 なら fy=240。🟡"""`)。Given / When / Then はコメントで書く(`tests/eval/test_eval_config.py`、`tests/safety/test_config.py` と同じ)。

### 0.1 共通定数(テストモジュールの先頭)

```python
W, H = SCENE_SPEC.image_width, SCENE_SPEC.image_height   # 640, 480
FY_60_480 = 415.6921938165307            # (480/2)/tan(30°)。実測で式とビット一致
T_SCENE_TRANSLATION = (0.45, 0.30, 0.55) # cam_top の pos (XML 67 行)
DRIFT_ATOL = 1e-9                         # --check と同じ (要件 2.10)
ANALYTIC_TOL = 1e-12                      # 小さな MJCF の解析比較 (実測最大 4.6e-16)
SCENE_TOL_M = 2e-3                        # 統合テストのしきい値 (OPEN-4 の決定。完了条件は 5 mm)
FLOOR_TOL_M = 1e-4                        # 床の逆投影 |z| (TASK-0008 S-02 と同じ)
SETTLE_STEPS = 500                        # 1 s。test_projection_scene.py と同じ
SEEDS = range(10)                         # OPEN-2 の決定
```

### 0.2 フィクスチャ

| 名前 | スコープ | 内容 | 使うファイル |
|---|---|---|---|
| `scene_model` | module | `mujoco.MjModel.from_xml_path(str(PICKPLACE_SCENE_PATH))`。モデルは読み取りのみで共有する | test_calibration.py |
| `scene_data` | function | `MjData(scene_model)` → `mj_resetData` → `mj_forward`。テストが `data` を変えても他に影響しないよう毎回作る(1 回 < 1 ms) | test_calibration.py |
| `tiny_model` / `tiny_data` | module / function | 下の小さな MJCF(`from_xml_string`、ファイル不要)。`tiny_data` は `mj_forward` 済み | test_calibration.py |
| `tiny_xml_path` | function | 同じ XML を `tmp_path / "tiny.xml"` に書いたパス(ファイル経由の読み込みも同じ結果になることを 1 件で確認する) | test_calibration.py |
| `sim_calibration` | module | `calibration_from_mujoco(scene_model, data, "cam_top", 640, 480)`(`mj_resetData` → `mj_forward` 済みの data で 1 回) | test_calibration.py |
| `valid_doc` | function | 要件 2.6 の YAML と同じ内容の dict を `copy.deepcopy` で返す(異常系で 1 か所だけ変える) | test_calibration.py |
| `committed` | module | `load_calibration()`(既定パス)。`metadata.source == "real"` なら `pytest.skip` | test_calibration.py(R-)、test_calibration_scene.py |
| `script` | module | `importlib.util.spec_from_file_location("generate_calibration", REPO_ROOT / "scripts" / "generate_calibration.py")` で読み込んだモジュール(`scripts/` はパッケージではないため) | test_generate_calibration.py |

小さな MJCF(要件 9.2 の XML に、異常系用のカメラを足したもの。描画しないので `<visual>` は不要):

```xml
<mujoco>
  <worldbody>
    <geom name="floor" type="plane" size="2 2 0.1"/>
    <body name="robot_base" pos="0.10 -0.05 0.02" euler="0 0 37">
      <site name="baseframe" pos="0.03 0.02 0.01" euler="5 -8 20"/>
    </body>
    <body name="mover" pos="0 0 0.5">
      <camera name="cam_body_track" mode="track" pos="0 0 0.3"/>
    </body>
    <camera name="cam" pos="0.6 0.5 0.7" fovy="50" xyaxes="-0.7 0.7 0 -0.5 -0.5 0.7"/>
    <camera name="cam_trackcom" mode="trackcom" pos="0.6 0.5 0.7"/>
    <camera name="cam_sensor" pos="0.6 0.5 0.7" resolution="320 240" sensorsize="0.004 0.003" focal="0.003 0.003"/>
  </worldbody>
</mujoco>
```

実測(9 章): `cam` は `cam_mode=0`・fovy 50・sensorsize `[0,0]`。`cam_trackcom` は mode 2、`cam_body_track` は mode 1。`cam_sensor` は mode 0 だが sensorsize `[0.004, 0.003]`(fovy は 53.13 と自動計算される)。base site の世界位置は `[0.1119, -0.0160, 0.0300]`(原点から 117 mm)、回転は単位行列ではない。

---

## 1. 正常系テストケース

### 1.1 `camera_from_mujoco`(test_calibration.py)

| ID | テスト名 | Given | When | Then(期待値) | 信頼性 |
|---|---|---|---|---|---|
| C-01 | fovy=90°・高さ 480 なら fy=240(タスク定義のテストケース1) | 小さな MJCF の `cam` の fovy を `model.cam_fovy[cid] = 90.0` に書き換えたモデル(`tiny_model` を共有しないよう `from_xml_string` で作り直す) | `camera_from_mujoco(m, "cam", 640, 480)` | `fy == approx(240.0, abs=1e-9)`、`fx == fy`。※式の結果は `240.00000000000003` なので完全一致では比べない | 🟡 |
| C-02 | 実シーン 640x480・fovy=60° の値 | `scene_model` | `camera_from_mujoco(scene_model, "cam_top", 640, 480)` | `fx == fy == approx(415.6921938165307, abs=1e-9)`、`cx == 319.5`、`cy == 239.5`、`(width, height) == (640, 480)` | 🔵 |
| C-03 | 主点は画素中心規約 `(w-1)/2`(幅と高さが違う・奇数) | `scene_model` | 320x240 と 641x481 で呼ぶ | 320x240: `fy == approx(207.84609690826534, abs=1e-9)`、`cx == 159.5`、`cy == 119.5`。641x481: `cx == 320.0`、`cy == 240.0`、`fy == approx(240.5/tan(30°))` | 🟡 |
| C-04 | 画像サイズは引数から取り、モデルの値に依存しない | `scene_model`(`cam_resolution` は `[1,1]`、offwidth 640) | 1x1 で呼ぶ | `cx == cy == 0.0`、`fy == approx(0.8660254037844388, abs=1e-12)`(エラーにならない) | 🟡 |
| C-05 | 戻り値は `Camera` で、フィールドは Python の型 | `scene_model` | C-02 と同じ | `isinstance(c, Camera)`。`type(c.fx) is float`(np.float64 ではない)、`type(c.width) is int` の 6 フィールドすべて | 🔴(要件 2.2 の決定) |
| C-06 | fovy が 60 以外の固定カメラ(小さな MJCF) | `tiny_model` の `cam`(fovy 50) | 320x240 | `fy == approx(257.340830461147, abs=1e-9)`、`fx == fy` | 🟡 |
| C-07 | `numpy` の整数も幅・高さとして受け付ける | `scene_model` | `np.int64(640), np.int64(480)` | C-02 と同じ値。`type(c.width) is int` | 🟡(`_positive_int` の既存の挙動) |
| C-08 | モデルを変更しない | `scene_model` の `cam_fovy` のコピー | C-02 を呼ぶ | 呼んだ後の `cam_fovy` がコピーと等しい | 🟡 |

### 1.2 `t_base_cam_from_mujoco`(test_calibration.py)

| ID | テスト名 | Given | When | Then(期待値) | 信頼性 |
|---|---|---|---|---|---|
| T-01 | 実シーンの T(S-01 相当) | `scene_data`(reset → forward) | `t_base_cam_from_mujoco(scene_model, scene_data, "cam_top")` | `shape == (4,4)`、`dtype == float64`、`validate_T_base_cam` を通る、平行移動 `assert_allclose(T[:3,3], [0.45, 0.30, 0.55], atol=1e-12)`、最下行 `[0,0,0,1]` | 🔵 |
| T-02 | base が単位(原点・単位回転)のときビット一致 | T-01 と同じ。`T_world_cam_mj = [cam_xmat | cam_xpos]` をテスト内で組む | 同上 | `np.array_equal(T, T_world_cam_mj @ np.diag([1.,-1,-1,1]))` が True(**ビット一致**)。前提として `site_xpos == 0`、`site_xmat == I` もビット一致で確認 | 🔵(実測) |
| T-03 | 期待行列の具体値(回帰) | T-01 と同じ | 同上 | 要件 2.6 の 4 行と `atol=1e-12` で一致(例: `T[2,0] == approx(-2.7755575615628914e-17, abs=1e-12)`、`T[0,0] == approx(-0.6896566706214934)`) | 🔵(実測) |
| T-04 | 静定・reset に依存しない | `scene_data` を 500 ステップ静定したもの、`mj_resetData` → `mj_forward` したもの | それぞれ呼ぶ | 2 つの T と T-01 の T がビット一致(`np.array_equal`) | 🔵(実測) |
| T-05 | base site を移動・回転した MJCF で解析値と一致 | `tiny_data`。`rng = np.random.default_rng(0)` で世界座標の点 1000 個(`uniform([-0.5,-0.5,0], [1,1,0.8])`)。各点の OpenCV カメラ座標を `p_cam = (R_wc.T @ (p - t_wc)) * [1,-1,-1]`(`R_wc = cam_xmat`、`t_wc = cam_xpos`)で作る | `T = t_base_cam_from_mujoco(tiny_model, tiny_data, "cam")`、`camera_to_base(p_cam, T)` | `inv(T_world_base) @ p`(`site_xmat` / `site_xpos` から)との差の最大値 `< 1e-12`(実測 4.6e-16) | 🟡(要件 9.2 の方法) |
| T-06 | 負の対照: base を無視すると大きくずれる(T-05 の検出力) | T-05 と同じ点 | `T_ign = mujoco_cam_to_opencv(T_world_cam_mj)` で同じ変換 | 1000 点すべてで誤差 `> 0.05 m`(実測 最小 53.5 mm・最大 1197 mm。要件 9.2 の箱の上面 1 点では 224.8 mm)。※ドックストリングに「base を無視する実装は T-05 で落ちる」ことを書く | 🟡(実測) |
| T-07 | 負の対照: かけ算の順序を逆にした行列は T と違う | T-05 と同じ | `mujoco_cam_to_opencv(T_world_cam_mj @ inv(T_world_base))` | `np.abs(T_bad - T).max() > 0.5`(実測 0.706) | 🟡(実測) |
| T-08 | base_site の既定値は `"baseframe"` | `tiny_data` | 引数なしと `base_site="baseframe"` を明示 | 2 つがビット一致 | 🟡 |
| T-09 | 戻り値は新しい書き込み可能な配列で、`data` は変わらない | `tiny_data` の `cam_xmat` / `cam_xpos` / `site_xmat` / `site_xpos` のコピー | 呼んで、戻り値 `T[0,0] = 99.0` と書き換える | 書き換えが例外なくでき(`T.flags.writeable`)、`T` は `data` のどの配列ともメモリを共有しない(`not np.shares_memory`)、`data` の 4 配列はコピーとビット一致。2 回目の呼び出しの結果は 1 回目(書き換え前)と一致 | 🟡 |
| T-10 | XML ファイルから読んだモデルでも同じ | `tiny_xml_path`(tmp_path に書いた XML) | `MjModel.from_xml_path` → forward → 呼ぶ | `from_xml_string` 版とビット一致 | 🟡 |

### 1.3 `Calibration` と `calibration_from_mujoco`(test_calibration.py)

| ID | テスト名 | Given | When | Then(期待値) | 信頼性 |
|---|---|---|---|---|---|
| D-01 | 正しい値で作れる(sim / real の両方) | C-02 の `Camera`、T-01 の T | `Calibration(camera, T, "sim", "cam_top", "baseframe")` と `source="real"` | 例外なし。各フィールドが渡した値(T は `array_equal`) | 🔵 |
| D-02 | frozen | D-01 | `cal.source = "real"`、`cal.T_base_cam = np.eye(4)` | どちらも `dataclasses.FrozenInstanceError` | 🔵 |
| D-03 | 保持する T は書き込み不可 | D-01 | `cal.T_base_cam[0, 0] = 1.0` | `ValueError`(numpy の read-only 例外は `ValueError`)。`cal.T_base_cam.flags.writeable is False` | 🔴(要件 2.4) |
| D-04 | 渡した配列を後から書き換えても影響しない(コピー) | 書き込み可能な T を渡して作る | 渡した T の `[0,3] = 9.0` に書き換え | `cal.T_base_cam[0,3] == 0.45`。`not np.shares_memory(cal.T_base_cam, T)` | 🔴(要件 2.4) |
| D-05 | 整数の 4x4 / リストの T も float64 にする | `T.tolist()`(ネストしたリスト) | 作る | `cal.T_base_cam.dtype == float64`、値はビット一致 | 🟡 |
| D-06 | `eq=False`(`==` は同一性) | 同じ値で 2 つ作る | `a == b`、`a == a` | `a == b` は False、`a == a` は True(例外を出さない)。比較は `calibrations_close` を使う旨をドックストリングに書く | 🔴(要件 2.4) |
| F-01 | `calibration_from_mujoco` は C と T をまとめて `source="sim"` | `scene_data` | `calibration_from_mujoco(scene_model, scene_data, "cam_top", 640, 480)` | `camera` は C-02 と等しい(`Camera` は frozen dataclass なので `==`)、`T_base_cam` は T-01 とビット一致、`source == "sim"`、`camera_name == "cam_top"`、`base_frame == "baseframe"` | 🟡 |
| F-02 | `base_site` を渡すと `base_frame` に記録される | `tiny_data` | `calibration_from_mujoco(tiny_model, tiny_data, "cam", 320, 240, base_site="baseframe")` | `base_frame == "baseframe"`、`camera_name == "cam"`、T は T-05 と同じ | 🟡 |

### 1.4 `save_calibration`(test_calibration.py)

| ID | テスト名 | Given | When | Then(期待値) | 信頼性 |
|---|---|---|---|---|---|
| Y-01 | 書いた内容が要件 2.6 の形式と**バイト一致**(ゴールデン) | `sim_calibration` | `save_calibration(cal, tmp_path / "c.yaml")` | ファイルの文字列が、テストに書いた期待文字列(コメント 3 行 + 要件 2.6 の本文 16 行。9.2 節で dump 結果を確認済み)と完全一致 | 🔴(形式は要件 2.6 の決定、数値は実測 🔵) |
| Y-02 | 先頭のコメント(sim) | Y-01 | 1〜3 行目を読む | 1 行目 `# 自動生成ファイル (source: sim)。手で編集しないこと。`、`uv run python scripts/generate_calibration.py` と `--check` を含む行がある。`yaml.safe_load` の結果にコメントは現れない | 🔴(要件 2.6) |
| Y-03 | 先頭のコメント(real) | `source="real"` の `Calibration` | 保存 | コメントに「実機の計測値」と「scripts/generate_calibration.py で上書きしないこと」を含む。`自動生成ファイル` を含まない | 🔴(要件 2.6) |
| Y-04 | キーの順序 | Y-01 | 行頭にインデントのないキー行を順に取り出す。`metadata` / `camera` の子キーも同様 | トップ `[version, metadata, camera, t_base_cam]`、metadata `[source, camera_name, base_frame]`、camera `[fx, fy, cx, cy, width, height]` | 🔴(要件 2.6) |
| Y-05 | 行列の各行が 1 行の flow 形式 | Y-01 | `t_base_cam:` 以降の行 | ちょうど 4 行で、各行が正規表現 `^- \[[^\]]+\]$` に一致し、カンマ区切りで 4 要素 | 🔴(要件 2.6) |
| Y-06 | 往復でビット一致(実際の値) | `sim_calibration` | 保存 → `load_calibration` | `calibrations_close(a, b, atol=0)` が True。さらに T は `np.array_equal(a.T.view(np.uint64), b.T.view(np.uint64))`、`fx` などは `struct.pack("<d")` で比較 | 🔵(実測) |
| Y-07 | 往復でビット一致(乱数と特殊値) | `rng = default_rng(1)` で 20 組: 乱数の四元数から作った回転 + 平行移動(乱数ビット列の有限値のうち絶対値 < 1e3 のもの、`-0.0`、`5e-324`、`1.1125369292536007e-308`(非正規化数))、`fx`/`fy` は `uniform(1, 2000)`・`5e-324`・`1.7976931348623157e308`、`cx`/`cy` は `-0.0`・非正規化数・負の値。回転の非対角に `-0.0` を含む単位回転も 1 組 | 各組を保存 → 読み込み | 全要素がビット一致(`-0.0` の符号ビットも保たれる。`math.copysign(1, x) == -1`) | 🔵(実測: 約 20 万個の乱数ビット列で不一致 0) |
| Y-08 | numpy の型を含む `Calibration` も保存できる | `Camera(fx=np.float64(...), cx=np.float32(319.5), width=np.int64(640), ...)`、T は `np.float64` 配列 | 保存 | `RepresenterError` にならない。ファイルに `!!python` や `numpy` の文字列がない。読み戻すと値が一致(`np.float32(319.5)` は 319.5 で正確) | 🔵(実測: safe_dump は np 型を拒否する) |
| Y-09 | 非決定的な内容がない | 同じ `Calibration` | 2 つのパスに保存(間に 1 回別の値を保存しても) | 2 ファイルのバイト列が一致。内容に時刻らしき文字列(`\d{4}-\d{2}-\d{2}`)やホスト名(`socket.gethostname()`)を含まない | 🔴(要件 2.6) |
| Y-10 | 既存ファイルを上書きする | 別の値で保存済みのパス | 保存 | 中身が新しい値になる(Y-01 と一致) | 🟡(要件 2.8) |
| Y-11 | `str` と `Path` の両方を受け付ける | `sim_calibration` | `save_calibration(cal, str(p))` と `Path` | 2 ファイルがバイト一致 | 🔵 |
| Y-12 | UTF-8 で書く | Y-01 | `p.read_bytes().decode("utf-8")` | 例外なし。日本語のコメントが読める | 🟡(要件 2.8) |

### 1.5 `load_calibration`(test_calibration.py)

| ID | テスト名 | Given | When | Then(期待値) | 信頼性 |
|---|---|---|---|---|---|
| L-01 | 要件 2.6 の YAML を読める | `valid_doc` を `yaml.safe_dump` で tmp_path に書く | `load_calibration(p)` | `Calibration` を返し、`camera` が C-02 と等しく、`T_base_cam` は要件 2.6 の 4 行とビット一致、`source == "sim"` | 🔵 |
| L-02 | 既定パス | なし | `DEFAULT_CALIBRATION_PATH` | `== REPO_ROOT / "configs" / "calibration.yaml"`、`is_file()`、`load_calibration()` が例外なし | 🔵(要件 2.9) |
| L-03 | `str` と `Path` の両方 | L-01 のファイル | `load_calibration(str(p))` | L-01 と `calibrations_close(atol=0)` | 🔵 |
| L-04 | `source: real` も読める(TASK-0036 用) | `valid_doc` の source を `real` に | 読む | `source == "real"` | 🔵 |
| L-05 | 整数の内部パラメータは float にする | `fx: 415`、`cx: 320` | 読む | `type(cal.camera.fx) is float`、値 415.0 | 🟡(要件 2.9) |
| L-06 | コメント付き(save の出力)も読める | Y-01 のファイル | 読む | Y-06 と同じ | 🔵 |
| L-07 | 読み込んだ T は書き込み不可 | L-01 | `cal.T_base_cam[0,0] = 0` | `ValueError` | 🔴 |

### 1.6 `calibrations_close`(test_calibration.py)

| ID | テスト名 | Given | When | Then(期待値) | 信頼性 |
|---|---|---|---|---|---|
| K-01 | 同じ値は True(atol=0 でも) | `sim_calibration` と、その往復 | `calibrations_close(a, b)`、`atol=0` | どちらも True | 🔴(要件 2.10) |
| K-02 | 既定 `atol=1e-9` | なし | `inspect.signature(calibrations_close).parameters["atol"]` | `default == 1e-9`、キーワード専用 | 🔴 |
| K-03 | 浮動小数の差の境界(fx) | `fx` だけ `+5e-10` / `+2e-9` ずらす | 既定 atol | `+5e-10` は True、`+2e-9` は False | 🔴 |
| K-04 | 浮動小数の差の境界(T の要素) | `T[0,3]`(平行移動)だけ `+5e-10` / `+2e-9` | 既定 atol | True / False。※回転の要素は直交性を保つため平行移動で試す | 🔴 |
| K-05 | 文字列・整数は完全一致 | `source` だけ `real`、`camera_name` だけ別名、`base_frame` だけ別名、`width` だけ 641(`cx` は同じ値のまま) | `atol=1.0` のように大きくしても | すべて False | 🔴 |
| K-06 | `atol=0` は 1 ulp の差も False | `fx` を `np.nextafter(fx, inf)` に | `atol=0` | False | 🔴 |

### 1.7 ドリフトガード(test_calibration.py)

| ID | テスト名 | Given | When | Then(期待値) | 信頼性 |
|---|---|---|---|---|---|
| R-01 | コミット済みの YAML がシーンから作った値と一致 | `committed`(source が `real` なら skip)、`scene_model` から `mj_resetData` → `mj_forward` した data | `calibration_from_mujoco(..., SCENE_SPEC.camera_name, SCENE_SPEC.image_width, SCENE_SPEC.image_height)` | `calibrations_close(committed, fresh, atol=DRIFT_ATOL)` が True。失敗時のメッセージに「`uv run python scripts/generate_calibration.py` で再生成」を出す | 🟡(要件 4.3) |
| R-02 | コミット済みのファイルは save の出力とバイト一致(手編集の検出) | R-01 と同じ(skip 条件も同じ) | `fresh` を tmp_path に `save_calibration` | `configs/calibration.yaml` のバイト列と一致。※MuJoCo の版差で末尾の桁が変わると落ちうる。その場合は R-01 だけ残して R-02 を消す判断を verify-complete に記録する(🔴 の理由) | 🔴 |
| R-03 | コミット済みの値が TASK-0008 の前提と一致 | `committed` | 値を読む | `camera_name == "cam_top"`、`base_frame == "baseframe"`、`(width, height) == (640, 480)`、平行移動 `[0.45, 0.30, 0.55]`(atol 1e-12) | 🔵 |

- skip の扱い: `committed` フィクスチャの中で `if cal.source == "real": pytest.skip("configs/calibration.yaml は実機の計測値 (source: real) のため sim との比較を行わない")`。skip の分岐そのものは、`load_calibration` の L-04 と G-07(real のファイルを作れること)で間接的に確かめる。🔴

### 1.8 統合テスト(test_calibration_scene.py、描画あり)

フィクスチャ(すべて module スコープ、1 モジュールで 1 回作る):

- `cal`: `load_calibration()`(コミット済みの YAML。source が `real` なら `pytest.skip`。実機の値で sim を描画しても意味がないため)🔴
- `model`: `from_xml_path(PICKPLACE_SCENE_PATH)`
- `renderer_depth` は使わず、既存の `render_rgbd(model, data, SCENE_SPEC.camera_name, W, H)` を使う(画像サイズは `cal.camera.width/height`)

ヘルパ(テストファイル内。TASK-0008 の `_top_face_bbox` を、静定後の回転を考慮する形に一般化する。9 章の確認スクリプトと同じ):

```python
def _top_face(model, data, color, camera, T):
    """静定後の上面中心 (geom_xpos + R @ [0,0,h]) と、上面 4 隅を投影した半開区間の bbox。"""
    gid = mj_name2id(model, mjOBJ_GEOM, f"block_{color}_geom")
    c, R, h = data.geom_xpos[gid], data.geom_xmat[gid].reshape(3, 3), SCENE_SPEC.block_half_size
    corners = [project_base_to_pixel(c + R @ [sx*h, sy*h, h], T, camera)[:2] for sx in (-1, 1) for sy in (-1, 1)]
    x1, y1 = (round(v) for v in np.min(corners, 0)); x2, y2 = (round(v) + 1 for v in np.max(corners, 0))
    return c + R @ [0, 0, h], (int(x1), int(y1), int(x2), int(y2))
```

- 真の位置の計算には**静定後の** `geom_xpos` / `geom_xmat` を使う(XML の値は使わない)。base は世界原点・単位回転なので、世界座標 = base 座標(T-02 で確認済み)。🔵
- **輪郭全体の bbox(segmentation の外接矩形)は使わない**(側面が混ざり水平に 6.9〜10.2 mm ずれる。TASK-0008 S-09)。🔵

| ID | テスト名 | Given | When | Then(期待値) | 信頼性 |
|---|---|---|---|---|---|
| I-01 | 既定配置の 3 個を 2 mm 以内で復元(parametrize: red / blue / green) | `mj_resetData` → `mj_forward` → 500 ステップ → `mj_forward`。`depth = render_rgbd(...)[1]`(module フィクスチャで 1 回) | `bbox_to_base_point(bbox, depth, cal.camera, cal.T_base_cam, shrink=0.5)` | `norm(p - top) <= SCENE_TOL_M (2 mm)`。実測 red 0.761 / blue 0.142 / green 0.615 mm。bbox の回帰値 red `(251,143,278,165)` も `==` で確認(上面 bbox の作り方が変わったことの検出) | 🟡(しきい値は OPEN-4 の決定、実測 🔵) |
| I-02 | ランダム配置 seed 0〜9 の 30 個を 2 mm 以内で復元(parametrize: seed) | 各 seed で `mj_resetData` → `mj_forward` → `place_blocks(model, data, sample_layout(seed))` → 500 ステップ → `mj_forward` → 描画 | 各ブロックで I-01 と同じ | 3 個とも `<= 2 mm`。実測: 30 個の最大 1.153 mm(seed 9 green)、平均 0.546 mm。失敗メッセージに seed・色・誤差(mm)・bbox を出す | 🟡(配置数は OPEN-2 の決定) |
| I-03 | 床の逆投影が平ら(主点規約・符号の回帰) | I-01 の depth と segmentation(`mujoco.Renderer` の segmentation。既存 `segmentation` と同じ方法)、8 px 格子の床画素(> 1000 点) | `camera_to_base(pixel_to_camera(u, v, depth[v,u], cal.camera), cal.T_base_cam)` | `max |z| <= FLOOR_TOL_M (0.1 mm)`(TASK-0008 S-02 の実測 0.0076 mm) | 🔵 |
| I-04 | 上面中心 → ブロック中心の補正の例示(要件 4.1) | I-01 の red | `p_top - [0, 0, SCENE_SPEC.block_half_size]` | 静定後の `geom_xpos` から `<= 2 mm` | 🟡 |

- I-02 の seed ごとに `MjData` を作り直すか `mj_resetData` で戻すかは実装で決めてよい(どちらでも結果はビット一致。要件 9.1)。他のテストと `data` を共有しない。🟡
- 50 配置の結果(平均 0.495 / 中央値 0.469 / p95 1.042 / 最大 1.235 mm、2 mm 超 0 件)は自動テストに入れず、verify-complete の文書に記録する(OPEN-2)。🔵

### 1.9 生成スクリプト(test_generate_calibration.py)

前提: すべて `tmp_path` に書く。リポジトリの `configs/calibration.yaml` を書き換えるテストは作らない(`--out` を必ず渡す。G-04 は読むだけ)。`main(argv) -> int` を `script` フィクスチャ経由で直接呼ぶ。標準出力・標準エラーは `capsys` で読む。

| ID | テスト名 | Given | When | Then(期待値) | 信頼性 |
|---|---|---|---|---|---|
| G-01 | `--out` に書いた内容はコミット済みのファイルと一致 | 空の `tmp_path` | `main(["--out", str(tmp_path / "c.yaml")])` | 戻り値 0。ファイルができ、`load_calibration` が `committed` と `calibrations_close(atol=1e-9)`。さらにバイト一致(R-02 と同じ理由で 🔴) | 🟡 |
| G-02 | 生成したファイルは `source: sim` と sim のコメント | G-01 | 読む | `source == "sim"`、1 行目が Y-02 のコメント | 🔴 |
| G-03 | `--check` 一致なら 0、ファイルは変わらない | G-01 で書いたファイル(mtime と内容を控える) | `main(["--out", p, "--check"])` | 戻り値 0。ファイルのバイト列と `stat().st_mtime_ns` が変わらない | 🔴(要件 2.11) |
| G-04 | `--check` を既定パス(コミット済み)に対して実行 | なし(`--out` なし) | `main(["--check"])` | 戻り値 0(コミット済みが最新)。`configs/calibration.yaml` のバイト列は変わらない。※source が `real` なら skip | 🟡 |
| G-05 | `--check` 不一致なら 1、差の最大値を標準エラーに出す | G-01 のファイルの `fx` を `fx + 1e-6` に書き換えたもの(`save_calibration` で書く) | `main(["--out", p, "--check"])` | 戻り値 1。`capsys.readouterr().err` が空でなく、差 `1e-06` 付近の値(`"1e-06"` または `"1.0e-06"` を含む。数値の書式は実装で決めてよいので、正規表現 `1(\.0+)?e-0?6` で探す)。ファイルは変わらない | 🔴 |
| G-06 | `--check` でファイルがない・読めないなら 2 | (a) 存在しないパス、(b) `version: 2` のファイル、(c) 空ファイル | `--check` | いずれも戻り値 2。例外を外に出さない(`SystemExit` も出さない)。標準エラーにパスまたは理由を出す | 🔴 |
| G-07 | 既存が `source: real` なら上書きを拒否して 2 | `source="real"` の `Calibration`(値は sim と同じでよい)を `save_calibration` で書いたファイル | `main(["--out", p])` | 戻り値 2。ファイルのバイト列が変わらない。標準エラーに `--force` の案内 | 🔴(要件 2.11) |
| G-08 | `--force` なら real を上書きする | G-07 と同じファイル | `main(["--out", p, "--force"])` | 戻り値 0。`source == "sim"`、G-01 とバイト一致 | 🔴 |
| G-09 | 既存が sim なら `--force` なしで上書きする | 別の値の sim ファイル(fx をずらしたもの) | `main(["--out", p])` | 戻り値 0。G-01 とバイト一致 | 🔴 |
| G-10 | `--check` は real のファイルを不一致とする(source が違う) | G-07 のファイル | `main(["--out", p, "--check"])` | 戻り値 1(`calibrations_close` は source を完全一致で比べるため)。ファイルは変わらない | 🔴(要件 2.10 と 2.11 から導いた挙動。8 章 O-T2) |
| G-11 | 2 回の生成でバイト一致(決定性) | 2 つのパス | 2 回 `main(["--out", ...])` | 2 ファイルがバイト一致 | 🟡 |
| G-12 | サブプロセスで実行できる(スモーク、1 件だけ) | `tmp_path`、`cwd=tmp_path`(リポジトリ外から実行しても既定のパスや import が壊れないこと) | `subprocess.run([sys.executable, str(REPO_ROOT / "scripts" / "generate_calibration.py"), "--out", str(p)], capture_output=True, timeout=60, cwd=tmp_path)` → 続けて同じコマンドに `--check` | 終了コード 0 と 0。ファイルが G-01 とバイト一致 | 🟡 |

---

## 2. 異常系テストケース

すべて `pytest.raises(ValueError, match=...)` で、メッセージにカメラ名・キー名など手がかりが入っていることを `match` で確かめる(ファイル欠落だけ `FileNotFoundError`)。`TypeError` は使わない(要件 3 章)。

### 2.1 `camera_from_mujoco`

| ID | テスト名 | 入力 | 期待 | 信頼性 |
|---|---|---|---|---|
| C-E01 | カメラ名がない | `"no_such_cam"`(`mj_name2id` が -1) | `ValueError`、`match="no_such_cam"` | 🟡 |
| C-E02 | 追従カメラ(mode="trackcom") | `tiny_model` の `"cam_trackcom"`(cam_mode 2) | `ValueError`、`match="cam_trackcom"` | 🟡 |
| C-E03 | 動く body の追従カメラ(mode="track") | `"cam_body_track"`(cam_mode 1) | `ValueError` | 🟡 |
| C-E04 | sensorsize 指定のカメラ | `"cam_sensor"`(mode 0、sensorsize `[0.004, 0.003]`) | `ValueError`、`match="cam_sensor"`。※mode は固定なので、mode の検査だけでは通ってしまうことを確認する対照 | 🔴(要件 2.2 の決定) |
| C-E05 | width / height が正の整数でない(parametrize) | `0`、`-1`、`640.0`、`True`、`"640"`、`None` を width に。height も同様 | `ValueError` | 🟡 |
| C-E06 | fovy が範囲外(モデルを書き換え) | `cam_fovy[cid]` を `0.0`、`180.0`、`-10.0`、`nan` に(モデルは都度 `from_xml_string` で作る) | `ValueError` | 🟡(要件 2.2。MJCF ではこの値を作れないため配列を書き換える) |

### 2.2 `t_base_cam_from_mujoco`

| ID | テスト名 | 入力 | 期待 | 信頼性 |
|---|---|---|---|---|
| T-E01 | `mj_forward` 前の data | `MjData(scene_model)` を作っただけ(`site_xmat` / `cam_xmat` がすべて 0。前提を assert) | `ValueError`。`data` は変わらない(関数が `mj_forward` を呼ばない。呼んだ後の `site_xmat` がまだ 0) | 🟡(実測) |
| T-E02 | カメラ名がない | `"no_such_cam"` | `ValueError`、`match="no_such_cam"` | 🟡 |
| T-E03 | base site 名がない | `base_site="no_such_site"` | `ValueError`、`match="no_such_site"` | 🟡 |
| T-E04 | 追従カメラ | `tiny_data` の `"cam_trackcom"` | `ValueError` | 🟡 |
| T-E05 | 別モデルの data(大きさが違う) | `scene_model` と `tiny_data` の組み合わせ | `ValueError`(または、名前の解決で失敗する `ValueError`)。※MuJoCo 自体が別の例外を出す場合は 8 章 O-T3 | 🔴 |

### 2.3 `Calibration`

| ID | テスト名 | 入力 | 期待 | 信頼性 |
|---|---|---|---|---|
| D-E01 | camera が `Camera` でない | `camera={"fx": ...}`(dict)、`None` | `ValueError` | 🟡 |
| D-E02 | source が sim / real 以外 | `"test"`、`"SIM"`、`""`、`None` | `ValueError`、`match="source"` | 🔵(要件 2.4) |
| D-E03 | camera_name / base_frame が空・文字列でない | `""`、`1`、`None` | `ValueError` | 🟡 |
| D-E04 | T が不正 | 3x4、NaN 入り、最下行が `[0,0,0,2]`、回転が直交しない(`R[0,1] += 1e-3`)、鏡映(`diag(1,1,-1,1)`)、文字列の行列 | `ValueError`(`validate_T_base_cam` のメッセージ) | 🔵 |

### 2.4 `save_calibration`

| ID | テスト名 | 入力 | 期待 | 信頼性 |
|---|---|---|---|---|
| Y-E01 | 親ディレクトリがない | `tmp_path / "missing" / "c.yaml"` | `FileNotFoundError`。`tmp_path / "missing"` は作られない | 🔴(要件 2.8) |
| Y-E02 | `Calibration` 以外を渡す | dict | `ValueError` | 🔴 |

### 2.5 `load_calibration`(厳格な読み込み)

`valid_doc` の 1 か所だけを変えて `yaml.safe_dump` で書くか、YAML の文字列を直接書く(`_write(tmp_path, obj)` ヘルパは test_eval_config.py と同じ)。parametrize の id にケースの内容を書く。

| ID | テスト名 | 入力 | 期待 | 信頼性 |
|---|---|---|---|---|
| L-E01 | ファイルがない | `tmp_path / "none.yaml"` | `FileNotFoundError` | 🔵 |
| L-E02 | 空ファイル・最上位が mapping でない | `""`、`"[]"`、`"- 1\n"`、`"42"`、`"text"` | `ValueError` | 🟡 |
| L-E03 | トップレベルの未知キー | `extra: 1` を追加 | `ValueError`、`match="extra"` | 🟡 |
| L-E04 | トップレベルの欠落キー(parametrize 4 通り) | `version` / `metadata` / `camera` / `t_base_cam` をそれぞれ削除 | `ValueError`、`match=<キー名>` | 🟡 |
| L-E05 | metadata の未知・欠落キー | `metadata.note: x` を追加、`source` / `camera_name` / `base_frame` をそれぞれ削除 | `ValueError`、キー名入り | 🟡 |
| L-E06 | camera の未知・欠落キー | `camera.k1: 0.0` を追加(歪み係数。OPEN-1)、6 キーをそれぞれ削除 | `ValueError`、キー名入り | 🟡 |
| L-E07 | metadata / camera が mapping でない | `metadata: sim`、`camera: [1, 2]` | `ValueError` | 🟡 |
| L-E08 | version が 1 でない | `2`、`0`、`"1"`、`1.0`、`true`、`null` | `ValueError`、`match="version"` | 🔴 |
| L-E09 | source が不正 | `test`、`SIM`、`""`、`null`、`1` | `ValueError`、`match="source"` | 🔴 |
| L-E10 | camera_name / base_frame が不正 | `""`、`null`、`1`、`[cam_top]` | `ValueError` | 🟡 |
| L-E11 | fx / fy / cx / cy が有限の実数でない(parametrize: キー 4 x 値) | YAML 文字列で `1e-3`(YAML 1.1 では**文字列**として読まれる。前提として `yaml.safe_load("fx: 1e-3")` が `{"fx": "1e-3"}` であることを assert)、`"415.0"`(引用符付き)、`true`、`.inf`、`-.inf`、`.nan`、`null`、`[1]` | `ValueError`、キー名入り | 🟡(実測) |
| L-E12 | fx / fy が 0 以下 | `0`、`0.0`、`-1`、`-415.69` | `ValueError` | 🔵(`Camera` の既存の検証) |
| L-E13 | width / height が正の整数でない | `640.0`、`0`、`-480`、`true`、`"640"`、`null` | `ValueError`、キー名入り | 🟡 |
| L-E14 | t_base_cam の形が 4x4 でない | 3 行、5 行、ある行が 3 要素・5 要素、平らな 16 要素のリスト、`null`、mapping、スカラー | `ValueError`、`match="t_base_cam"` | 🟡 |
| L-E15 | t_base_cam の要素の型が不正 | ある要素を `"0.45"`(文字列)、`1e-3`(未引用で文字列になる)、`true`、`null`、`.nan`、`.inf` に | `ValueError` | 🟡 |
| L-E16 | t_base_cam の回転が直交でない | `R[0,1] += 1e-3`(直交誤差 ~1e-3 > 1e-6)、全回転を `2*R`、鏡映 `R[:,2] *= -1` | `ValueError`(`validate_T_base_cam` のメッセージ) | 🔵 |
| L-E17 | t_base_cam の最下行が不正 | `[0, 0, 0, 2]`、`[0, 0, 1e-6, 1]` | `ValueError` | 🔵 |
| L-E18 | 失敗時に部分的な結果を返さない・ファイルを変えない | L-E11 のファイル | 読む前後のバイト列 | 変わらない(読み込みは書き込まない) | 🟡 |

### 2.6 生成スクリプト

G-06、G-07、G-10 を参照(終了コード 2 / 2 / 1)。加えて:

| ID | テスト名 | 入力 | 期待 | 信頼性 |
|---|---|---|---|---|
| G-E01 | 未知のオプション | `main(["--bogus"])` | argparse の `SystemExit(2)`(`pytest.raises(SystemExit)` で `code == 2`) | 🔴 |
| G-E02 | `--out` の親ディレクトリがない(生成モード) | `tmp_path / "missing" / "c.yaml"` | 戻り値 2(`FileNotFoundError` を外に出さない)。ディレクトリは作られない | 🔴 |

---

## 3. 境界値テストケース

| ID | テスト名 | 境界 | 期待 | 信頼性 |
|---|---|---|---|---|
| B-01 | 画像サイズの最小 1x1 | width=height=1(正の整数の下限) | 正常(C-04)。0 は `ValueError`(C-E05) | 🟡 |
| B-02 | fovy の開区間 (0, 180) | `1e-6`・`179.999` は正常(`fy > 0`、有限)、`0`・`180` は `ValueError`(C-E06) | 🟡 |
| B-03 | `calibrations_close` の atol の境界 | 差 `5e-10`(内)/ `2e-9`(外)、`atol=0` で 1 ulp(K-03、K-04、K-06)。差がちょうど atol のときは True(`<=`)を `fx` で確認: `a.fx = 1.0`、`b.fx = 1.0 + 2**-20`、`atol = 2**-20`(2 進で正確に表せる値を使い、丸めの曖昧さを避ける) | 🔴 |
| B-04 | 回転の直交性の境界(load) | `R[0,1] += 1e-8`(直交誤差 ~1e-8 < 1e-6)は読める、`+= 1e-3` は `ValueError`(L-E16) | 🔵(`_ROTATION_TOL = 1e-6`) |
| B-05 | 最下行の許容(load) | `[0, 0, 0, 1 + 1e-12]` は読める(`_BOTTOM_ROW_TOL = 1e-9`)、`1e-6` のずれは `ValueError`(L-E17)。※読み込んだ値は**検証済みのまま保持**され、正規化しない(往復のビット一致を保つため) | 🟡 |
| B-06 | 浮動小数の極値の往復 | `-0.0`、`5e-324`(最小の非正規化数)、`1.1125369292536007e-308`、`1.7976931348623157e308`(fx に) | ビット一致(Y-07) | 🔵(実測) |
| B-07 | YAML の数値表記の境界 | `fx: 1.0e-3` は float として読める(内部パラメータとしては正の値なので有効)、`fx: 1e-3` は文字列で `ValueError`(L-E11) | 🟡(実測) |
| B-08 | 整数と浮動小数の境界 | `fx: 415`(int)は float にして受け付ける(L-05)、`width: 640.0` は拒否(L-E13) | 🟡 |
| B-09 | 統合テストの最悪値の余裕 | seed 0〜9 の最大 1.153 mm に対し 2 mm(余裕 0.85 mm、約 1.7 倍) | テストが偶然落ちる余地は小さい。MuJoCo の版更新で落ちたら値を記録して判断する | 🟡 |

---

## 4. 開発言語・フレームワーク

- **プログラミング言語**: Python 3.12 🔵
  - 既存の `src/so101_vlm` と同じ。numpy の配列比較(`array_equal`、`view(np.uint64)` によるビット比較)と `struct` で浮動小数の厳密な検証ができる。
- **テストフレームワーク**: pytest(`uv run pytest -q`)🔵
  - `tmp_path`、`capsys`、`pytest.mark.parametrize`、module スコープのフィクスチャを使う。既存のテストと同じ。
  - 追加の依存はない(`subprocess`・`importlib`・`struct`・`inspect` は標準ライブラリ)。
- **テスト実行環境**: macOS(darwin)・mujoco 3.16.0・numpy 2.5.4・PyYAML 6.0.3。描画は既存と同じオフスクリーン(`render_rgbd`)。🔵

### 4.1 実行時間の予算 🟡

| ファイル | 内容 | 見積もり |
|---|---|---|
| test_calibration.py | モデル読み込み(実シーン 1 回 + 小さな MJCF 数回)、`mj_forward`、YAML 入出力。描画なし | < 1.5 s |
| test_generate_calibration.py | `main()` 直接呼び出し 約 12 回(各 < 0.1 s)+ サブプロセス 2 回(各 約 0.5 s。Python 起動 + mujoco/numpy/yaml の import + モデル読み込み。実測 0.48 s) | < 2.5 s |
| test_calibration_scene.py | 既定配置 1 回(静定 + 描画 0.61 s、segmentation 1 回)+ 10 配置(実測 3.03 s) | < 4.5 s |
| **合計(3 ファイル)** | | **< 8 s**(目安。超える場合はサブプロセスを G-12 の 1 組だけに保つ、segmentation を既定配置だけにする) |

- サブプロセスは 1 回 約 0.5 s かかるため、CLI の挙動は `main(argv)` を直接呼んで確かめ、サブプロセスは G-12(生成と `--check` の 2 回)だけにする。
- 変更する `test_projection_scene.py` は描画の回数を変えない(実行時間はほぼ同じ)。
- 全体(現在 724 件)の実行時間の増加は 8 秒程度を見込む。

---

## 5. テストケース実装時の日本語コメント指針

既存のテスト(`tests/eval/test_eval_config.py`、`tests/perception/test_projection_scene.py`)に合わせ、次の形にする。

```python
def test_camera_from_mujoco_fovy90_height480_gives_fy240(tiny_xml):
    """C-01: fovy=90°・高さ 480 なら fy=240 (タスク定義のテストケース1)。🟡"""
    # 【テスト目的】: 画角 (度) から焦点距離 (画素) を求める式 fy=(h/2)/tan(fovy/2) を確かめる
    # Given: fovy を 90° にした固定カメラ (式の結果は 240.00000000000003 なので近似で比べる)
    model = mujoco.MjModel.from_xml_string(tiny_xml)
    model.cam_fovy[_cam_id(model, "cam")] = 90.0
    # When
    camera = camera_from_mujoco(model, "cam", 640, 480)
    # Then
    assert camera.fy == pytest.approx(240.0, abs=1e-9)  # 【確認内容】: 焦点距離
    assert camera.fx == camera.fy  # 【確認内容】: 正方画素
```

- ドックストリング: 先頭にケース ID、1 文の内容、信頼性マーク。負の対照(T-06、T-07)は「このテストが落ちたら何を見直すか」を書く(S-03 と同じ書き方)。
- Given / When / Then のコメントは必ず書く。数値の出どころ(「実測 0.761 mm」「XML 67 行」)をコメントに残す。
- 各 assert の行末に `# 【確認内容】: ...` を付ける(重要なものだけでよい)。
- モジュールのドックストリングに、テストケース定義書のパス(本書)と信頼性の凡例を書く。

---

## 6. 要件定義との対応関係

| 要件(calibration-requirements.md) | テストケース |
|---|---|
| 1 章 機能の概要(K と T の計算、YAML への書き出し、読み込み) | C-01〜C-08、T-01〜T-10、Y-01〜Y-12、L-01〜L-07 |
| 2.2 `camera_from_mujoco`(式、画像サイズは引数、Python 型、エラー 4 種、sensorsize) | C-01〜C-08、C-E01〜C-E06、B-01、B-02 |
| 2.3 `t_base_cam_from_mujoco`(式、新しい配列、data 不変、未計算の拒否、一般性) | T-01〜T-10、T-E01〜T-E05 |
| 2.4 `Calibration`(frozen、eq=False、read-only コピー、source) | D-01〜D-06、D-E01〜D-E04、L-07 |
| 2.5 `calibration_from_mujoco` | F-01、F-02 |
| 2.6 YAML の形式(キー・順序・flow の行・コメント・非決定的な値なし) | Y-01〜Y-05、Y-09 |
| 2.7 数値の正確さ(丸めない、np 型の変換、ビット一致、`1e-3` の罠) | Y-06〜Y-08、B-06、B-07、L-E11 |
| 2.8 `save_calibration`(親ディレクトリ、上書き、UTF-8) | Y-10〜Y-12、Y-E01、Y-E02 |
| 2.9 `load_calibration`(既定パス、厳格な検証、FileNotFoundError、str/Path) | L-01〜L-07、L-E01〜L-E18、B-04、B-05、B-08 |
| 2.10 `calibrations_close`(atol=1e-9、文字列・整数は完全一致) | K-01〜K-06、B-03 |
| 2.11 生成スクリプト(`--out`、`--check` 0/1/2、real の保護、`--force`、`main(argv)`) | G-01〜G-12、G-E01、G-E02 |
| 3 章 制約(OpenCV 規約、層の分離、不変性、エラー方針、決定性、既存テストの維持) | T-02、T-09、D-02〜D-04、C-08、Y-09、G-11、I-03、既存 S-01〜S-09 |
| 4.2 受け入れ(上面 bbox + shrink=0.5、輪郭全体の bbox は使わない) | I-01、I-02(しきい値 2 mm は OPEN-4 の決定)、I-04 |
| 4.3 ドリフトガード(atol 1e-9、real なら skip) | R-01〜R-03、G-01、G-04 |
| 4.4 エッジケース表 | 全行を C-/T-/L-/Y-/G- の各ケースで扱った(「fovy=90°」→C-01、「base site を移動・回転」→T-05 ほか) |
| 7 章 テスト方針(既存テストの置き換え) | `test_projection_scene.py` の `calib` フィクスチャを `calibration_from_mujoco` に置き換え、S-01 のドックストリングを「製品関数の検証」に直す。S-01〜S-09 の判定は変えない |

- **参照した機能概要**: 要件 1 章
- **参照した入力・出力仕様**: 要件 2.2〜2.11
- **参照した制約条件**: 要件 3 章
- **参照した使用例**: 要件 4.1〜4.4
- **参照したEARS要件**: FR-04、NFR-05
- **参照した設計文書**: design.md 4.4、5

---

## 7. ケース数と信頼性の分布

| 分類 | ケース(ID) | 件数 | 🔵 | 🟡 | 🔴 |
|---|---|---|---|---|---|
| 正常系: camera_from_mujoco | C-01〜C-08 | 8 | 1 | 6 | 1 |
| 正常系: t_base_cam_from_mujoco | T-01〜T-10 | 10 | 4 | 6 | 0 |
| 正常系: Calibration / from_mujoco | D-01〜D-06、F-01、F-02 | 8 | 2 | 3 | 3 |
| 正常系: save | Y-01〜Y-12 | 12 | 5 | 2 | 5 |
| 正常系: load | L-01〜L-07 | 7 | 5 | 1 | 1 |
| 正常系: calibrations_close | K-01〜K-06 | 6 | 0 | 0 | 6 |
| 正常系: ドリフトガード | R-01〜R-03 | 3 | 1 | 1 | 1 |
| 正常系: 統合(描画) | I-01〜I-04 | 4 | 2 | 2 | 0 |
| 正常系: 生成スクリプト | G-01〜G-12 | 12 | 0 | 4 | 8 |
| 異常系 | C-E01〜06、T-E01〜05、D-E01〜04、Y-E01〜02、L-E01〜18、G-E01〜02 | 37 | 7 | 23 | 7 |
| 境界値 | B-01〜B-09 | 9 | 2 | 6 | 1 |
| **合計** | | **116** | **28** | **54** | **34** |

- parametrize で展開すると実際のテスト関数の実行数は約 230 件になる見込み(L-E11 が 4 キー x 8 値で 32 件など)。
- 🔴 が多いのは、YAML の形式・CLI の終了コード・`calibrations_close` など、要件定義の段階で「資料にない決定」として決めた仕様をそのまま検証するため。要件定義書の 🔴 の判断を変えない限り、テストの期待値はぶれない。

---

## 8. テスト設計上の決定と未決事項

### 8.1 本書で決めたこと(🔴)
1. CLI のテストは `tests/perception/test_generate_calibration.py` に分ける(400 行の目安)。スクリプトは `importlib` で読み込み、`main(argv)` を呼ぶ。サブプロセスは G-12 の 1 組だけ。
2. 統合テストは**コミット済みの YAML**(`load_calibration()`)を使う。これでドリフトガード(R-01)と合わせ、「コミットした値で位置が復元できる」ことを確かめる。source が `real` のときは skip。
3. 統合テストのしきい値は 2 mm(OPEN-4 の決定)。完了条件の 5 mm は文書で扱う。
4. 真の位置は静定後の `geom_xpos + geom_xmat @ [0,0,h]`(ブロックの傾きも考慮)。TASK-0008 の `_top_face_bbox` は `+[0,0,h]` だったが、傾きがほぼ 0 なので結果は同じ。
5. R-02 / G-01 のバイト一致は、MuJoCo の版差で末尾の桁が揺れると落ちうる。落ちた場合は R-01(atol 1e-9)だけを残す。

### 8.2 未決事項(実装時に決めてよい)
- **O-T1**: `--check` の不一致メッセージの書式(G-05 は正規表現で緩く確かめる)。
- **O-T2**: `--check` で既存ファイルが `source: real` のとき、1(不一致)とするか、2(比べる意味がない)とするか。本書は要件 2.10 / 2.11 の文言どおり 1 とした(G-10)。2 にする場合は G-10 の期待を変える。
- **O-T3**: 別モデルの data を渡したとき(T-E05)、MuJoCo の配列の範囲外アクセスになるかは実装次第。`model.ncam` / `nsite` と data の配列長(`data.cam_xpos.shape[0]`)を照合して `ValueError` にする案。実装が難しければ T-E05 を削る。
- **O-T4**: 統合テストのフィクスチャ(静定・描画)を `tests/perception/conftest.py` に移して `test_projection_scene.py` と共有するか。共有する場合も、`test_projection_scene.py` の判定(S-01〜S-09)は変えない。
- **O-T5**: `camera_from_mujoco` の fovy 範囲検査(C-E06)は、MJCF からは作れない値を配列の書き換えで作る。MuJoCo がモデルの配列を読み取り専用にした場合は C-E06 を削る。

---

## 9. テストケースの数値の確認(使い捨てスクリプト)

スクリプト: scratchpad の `calib_survey/s5_testcase_checks.py`、`calib_survey/s6_dump_format.py`(リポジトリ外)。環境は要件 9 章と同じ。

| 項目 | 確認した値 | 使うケース |
|---|---|---|
| fovy=90°、h=480 | `240.00000000000003`(完全一致しない → `approx(abs=1e-9)`) | C-01 |
| fovy=60°、h=480 / h=240 / h=1 | `415.6921938165307` / `207.84609690826534` / `0.8660254037844388` | C-02、C-03、C-04 |
| fovy=50°、h=240 | `257.340830461147` | C-06 |
| 小さな MJCF のカメラ | `cam_mode`: cam 0、cam_trackcom 2、cam_body_track 1、cam_sensor 0。sensorsize: cam_sensor のみ `[0.004, 0.003]` | C-E02〜C-E04 |
| `mj_forward` 前 | `site_xmat`・`cam_xmat` がすべて 0 | T-E01 |
| カメラ名不明 | `mj_name2id` が -1 | C-E01 |
| 小さな MJCF の解析比較(1000 点、seed 0) | 最大誤差 4.6e-16 | T-05(< 1e-12) |
| base を無視したとき | 最小 53.5 mm・最大 1197 mm(要件 9.2 の箱の上面では 224.8 mm) | T-06(> 50 mm) |
| かけ算の順序を逆にしたとき | 要素の差の最大 0.706 | T-07(> 0.5) |
| 実シーン: base が単位のときのビット一致 | `array_equal(T, T_world_cam_mj @ diag(1,-1,-1,1))` が True。T の 4 行は要件 2.6 と一致 | T-02、T-03 |
| 既定配置の復元誤差 | red 0.761 / blue 0.142 / green 0.615 mm(静定 + 描画 0.61 s) | I-01 |
| seed 0〜9 の復元誤差 | 30 個の最大 1.153 mm(seed 9 green)、平均 0.546 mm、2 mm 超 0 件、10 配置で 3.03 s | I-02 |
| YAML の dump 形式 | `SafeDumper` のサブクラスで行だけ flow にすると、要件 2.6 の本文と完全に同じ文字列になる | Y-01、Y-05 |
| YAML 往復 | 乱数ビット列の有限値 199,908 個 + 特殊値でビット不一致 0。`-0.0` の符号も保たれる | Y-07、B-06 |
| YAML の読み込み | `fx: 1e-3` → 文字列 `'1e-3'`、`1.0e-3` → 0.001、`'415.0'` → 文字列、`640.0` → float | L-E11、L-E13、B-07 |
| サブプロセス 1 回 | Python 起動 + import + モデル読み込み + forward で 0.48 s | G-12、4.1 節 |

---

## 10. 品質判定

- **テストケース分類**: 正常系 70 件・異常系 37 件(parametrize で展開すると 100 件超)・境界値 9 件。各関数の正常・異常・境界がそろっている。✅
- **期待値定義**: すべてのケースに具体的な数値・例外の型・終了コードを書いた。浮動小数の許容値(ビット一致 / 1e-12 / 1e-9 / 0.1 mm / 2 mm)の使い分けも明記した。✅
- **技術選択**: Python 3.12 + pytest。既存のテストと同じ。✅
- **実装可能性**: 必要な数値と手順はすべて使い捨てスクリプトで実行して確かめた。実行時間は 3 ファイルで 8 秒以内の見込み。✅
- **信頼性レベル**: 🔵 28 / 🟡 54 / 🔴 34。🔴 は要件定義書で決めた仕様(YAML 形式・CLI・`calibrations_close`)の検証が中心で、期待値は要件の記述から一意に決まる。

**判定**: ✅ 高品質(期待値は一意で、数値は実測済み。🔴 は要件の決定事項の検証であり、テスト側の推測ではない)。

次のお勧めステップ: `/tsumiki:tdd-red so101-vlm-pickplace TASK-0009` で Red フェーズ(失敗テスト作成)を開始します。

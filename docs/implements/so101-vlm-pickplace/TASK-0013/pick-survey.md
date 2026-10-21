# TASK-0013 実測調査: 固定位置での pick & place(真の位置、VLM なし)

**作成日**: 2026-10-21
**目的**: TASK-0013 の要件(`pick-requirements.md`)の数値の根拠を、sim で実測して残す。
**方法**: リポジトリの実装(`IkPolicy`、`apply_safety`、`SimRobot`、`sample_layout` / `place_blocks`)をそのまま使う使い捨ての実行器を作り、シーン XML の一時コピー(物理設定だけ変更)で実行した。リポジトリのファイルは変更していない。
**MuJoCo**: 3.16.0

---

## 0. 結論(先に要点)

| 項目 | 結果 |
|---|---|
| (a) 物理設定の修正あり、20 seed × 赤 | **20/20 成功**(計画失敗 0) |
| (a) 物理設定の修正あり、20 seed × 3 色 = 60 件 | 計画失敗 2(3.3%)、**実行 58/58 成功(計画可能な配置の 100%)** |
| (a') 同、100 seed × 3 色 = 300 件 | 計画失敗 18(6.0%、全て x < 0.17)、**実行 282/282 成功(100%)** |
| (b) 物理設定の修正なし(既定: pyramidal / impratio 1)、60 件 | 計画失敗 2、**実行 0/58**(全件 grasp_slip。持ち上げ中に抜ける)。TASK-0012 の 0/29 → 29/29 を本実行器でも再現 |
| (c) 90% に届くための追加調整 | **不要**。`<option cone="elliptic" impratio="10"/>` だけで 100%。摩擦・solref・質量は変えない |
| (c) 感度 | 閉じ指令 `gripper_closed`: 0.10 / 0.15 / 0.20 で 58/58、0.25 で 51/58、0.30 で 8/58、0.0 で 27/58。impratio 5 でも 58/58。`close_hold_s` 0.2、`open_hold_s` 0.0 でも 58/58 |
| 安全フィルタの介入 | 全試行・全周期で 0 回(clipped / rate_limited / rejected / stopped) |
| (d) 既存テストへの影響 | 物理設定を変えたシーンで、シーンを使う既存テスト 1090 件が全て通る。静止時の z 誤差 0.108 mm(既定 0.072 mm)、ずれ 0、timestep 0.002 s は不変 |
| (f) 1 試行の時間(動画なし) | SimRobot の読み込み 0.28 s + 計画 平均 0.17 s(最大 0.70 s)+ 実行 平均 0.17 s。300 件で 118 s(1 件 0.39 s) |
| (f) 動画あり | 1 件 約 8.7 s(描画とエンコード 約 7.5 s、640×480・25 fps) |

---

## 1. 実行器(使い捨て)

場所: `/private/tmp/claude-501/-Users-takuya-project-20-pai-homework/b14af3ef-30ce-4f56-a63b-8ac03b5a9e70/scratchpad/pick_survey/`

| ファイル | 内容 |
|---|---|
| `common.py` | シーンの一時コピーの作成(`make_scene`)と 1 試行の実行・判定・失敗分類(`run_trial`) |
| `sweep.py` | seed × 色の掃引。結果を `res_*.json`、ログを `log_*.txt` に保存 |
| `run_tests_on_scene.py` | `PICKPLACE_SCENE_PATH` を一時シーンに差し替えて既存の pytest を走らせる |
| `settle.py` | 配置後 1 s の静止(z、ずれ、速度)と 1 step の計算時間の比較 |
| `frames.py` | 運搬中の画像と mp4 の書き出し(free camera と `cam_top`) |
| `jerr.py` | 最終の関節誤差、SimRobot / IkPolicy の初期化時間 |

### 1.1 シーンの一時コピー

`assets/scene_pickplace.xml` を読み、次だけを変えて `scenes/<name>.xml` に書いた。

- `include` と `meshdir` を絶対パスにする(一時コピーの場所から上流モデルを読むため。リポジトリ内で変えるときは不要)。
- `<include .../>` の直後(`<compiler>` の前)に `<option cone="elliptic" impratio="10"/>` を入れる(変種 `fix`)。変種 `base` は入れない(現在のシーンと同じ物理)。
- `assets/so101/so101_new_calib.xml` に `<option>` は無い(grep で確認)。重複の問題は起きない。

### 1.2 1 試行の手順

1. `SimRobot(scene_path=一時シーン)`。`place_blocks(model, data, sample_layout(seed))`(ブロックは静止高さ 0.015 に置かれる。事前の静置はしない)。
2. 対象ブロックの中心 `data.xpos`(真の位置)を `Goal.pick_target_base`、箱の底板上面の中心 `(0.20, 0.19, 0.005)` を `place_target_base` にする。
3. `IkPolicy(model).act(Observation.from_state(robot.get_state()), goal)`。`TrajectoryError` は計画失敗として記録し、実行しない。
4. `q[1:]` を 1 制御周期に 1 行ずつ送る。毎周期 `apply_safety(q_current=前回の指令, q_target=行, lower/upper=limits_from_model, last_state_t=now=state.timestamp, config=load_safety_config(), target_base=その行の区間の終点の TCP 目標)`。最初の基準は `q[0]`。`SafetyResult.q_command` を `send_command` する。
   - 区間 → 経由点: `pre_grasp`→pre_grasp、`grasp`/`close`/`close_hold`→grasp、`lift`→lift、`pre_place`→pre_place、`place`/`open`/`open_hold`→place、`retreat`→retreat(`metadata["waypoints"]` の `target`)。
5. 最後の指令を保持して 1.0 s(50 周期)待つ。
6. 判定: 中心 xy が箱の内寸の内側(|dx|, |dy| < 0.04 − margin、margin = 0)、中心 z < 縁 0.04、並進速度 ≤ 0.01 m/s。
7. 失敗の分類(判定が失敗のとき、上から順に最初に当てはまるもの):
   - `grasp_slip`: `lift` 区間の終わりでブロック中心 z < 静止高さ + 0.03、または `lift` / `pre_place` 区間でブロック中心と TCP の距離 > 0.02 m
   - `drop`: `place` 区間でブロック中心と TCP の距離 > 0.02 m(解放前に離れた)
   - `collision_other`: 他のブロックの xy の移動の最大 > 5 mm
   - `knocked_over`: 最終の傾き(どの面が上でもよい、最も近い面の法線と鉛直の角度)> 20°
   - `not_in_box`: それ以外

---

## 2. 結果

### 2.1 (a) 物理設定の修正あり(`fix`)

- 赤 × seed 0〜19: **20/20 成功**。計画失敗なし。
- 3 色 × seed 0〜19(60 件): 成功 58、計画失敗 2。
  - 青 seed 16: `self_collision`(grasp)、x = 0.169
  - 緑 seed 5: `not_converged`(pre_grasp)、x = 0.161
  - 計画可能な配置での成功率 **58/58 = 100%**。実行失敗は 0 件(分類の内訳なし)。
- 3 色 × seed 0〜99(300 件): 成功 282、計画失敗 18(6.0%)。
  - 計画失敗の内訳: `self_collision`(grasp)10、`not_converged`(pre_grasp)7、`self_collision`(pre_grasp)1。色別: 赤 5、青 9、緑 4。
  - 計画失敗の x は 0.151〜0.169 m。x < 0.17 の配置は 30 件あり、そのうち 18 件が失敗(60%)。x ≥ 0.17 の失敗は 0 件。成功した最小の x は 0.151。
  - 計画可能な配置での成功率 **282/282 = 100%**。
- 置いた位置: 箱中心からのずれは典型で dx ≈ −4 mm、dy ≈ −5 mm(系統的)、最大 15.2 mm(|dx|, |dy| の大きい方、300 件)。内寸の判定の限界(40 mm、ブロックが床に平らに乗る限界は 25 mm)に対して十分小さい。
- 最終の中心 z: 0.0199 m(底板上面 0.005 + 0.015。めり込み 0.1 mm)。傾き 0.0°。判定時の速度 ≤ 1e-5 m/s。
- 他のブロックの移動: 最大 4.2 mm(300 件中、成功試行での値)。5 mm を超えた試行は 0 件。60 件では緑 seed 9 の 2.2 mm(`close` 区間)が最大。
- 安全フィルタのフラグ: 全試行・全周期で 0。
- 最終の関節誤差(最後の指令と計測値の差、10 件): 腕 最大 0.0005 rad、グリッパー 0。
- 送った周期数(軌道の行 + 待ち 50): 292〜374(軌道 4.8〜6.5 s + 待ち 1 s)。

### 2.2 (b) 物理設定の修正なし(`base`、現在のシーンと同じ)

- 60 件: 計画失敗 2(同じ 2 件)、**実行 0/58**、全件 `grasp_slip`。
- ブロックは 25〜40 mm 持ち上がった後、`lift` の途中で抜けて床に落ちる(最終 z = 0.0149、箱の手前 6〜28 cm)。
- TASK-0012 の記録(既定 0/29、elliptic + impratio 10 で 29/29)と一致する。

### 2.3 (c) 調整の変種(いずれも 3 色 × seed 0〜19、計画可能 58 件)

| 変種 | 成功 | 備考 |
|---|---|---|
| `fix`(elliptic / impratio 10、既定の軌道設定) | **58/58** | 採用 |
| elliptic / impratio 5 | 58/58 | impratio は 5 でもよい |
| `gripper_closed` 0.0 | 27/58 | 失敗は全て grasp_slip |
| `gripper_closed` 0.10 | 58/58 | |
| `gripper_closed` 0.15(既定) | 58/58 | |
| `gripper_closed` 0.20 | 58/58 | |
| `gripper_closed` 0.25 | 51/58 | grasp_slip 7(TASK-0012 の赤 20 件では 20/20 だった) |
| `gripper_closed` 0.30 | 8/58 | grasp_slip 50 |
| `close_hold_s` 0.2(既定 0.5) | 58/58 | 短縮しても成功 |
| `open_hold_s` 0.0(既定 0.3) | 58/58 | |

- **結論**: 物理の `<option>` 1 行だけで 90% を超える(100%)。摩擦・`solref` / `solimp`・質量・`forcerange` の変更は不要。
- 閉じ指令の安全な幅は 0.10〜0.20 rad。既定 0.15 はその中央。0.25 以上と 0.0 で急に悪くなる(0.0 で悪くなる理由は未調査。TASK-0014 で記録するなら調べる)。
- 保持時間の短縮は可能だが、本タスクでは変えない(YAGNI。時間は NFR-06 の 3 分に対し十分短い)。

### 2.4 (d) 既存の期待への影響

- `run_tests_on_scene.py` で `so101_vlm.robot.model_paths.PICKPLACE_SCENE_PATH` を一時シーンに差し替え、`tests/test_scene.py`、`tests/eval/test_place_blocks.py`、`tests/perception/`、`tests/policy/`、`tests/safety/` を実行した。
  - `fix`: **1090 passed**。`base`: 1090 passed。
  - 差し替えが効いていることは、ブロック寸法を変えた試験用シーンで `test_block_geometry_matches_spec` が失敗することで確認した。
- 配置後の静置(20 seed × 3 ブロック、500 mj_step = 1 s):

| シーン | cone | impratio | timestep | 最大 \|z − 0.015\| | 最大のずれ | 最大の速度 | 1 mj_step |
|---|---|---|---|---|---|---|---|
| base | pyramidal | 1 | 0.002 | 0.072 mm | 0.0000 mm | 2.0e-10 m/s | 34.6 µs |
| fix | elliptic | 10 | 0.002 | 0.108 mm | 0.0000 mm | 8.1e-11 m/s | 29.2 µs |

- `tests/test_scene.py` の `REST_TOL = 0.002`(2 mm)と `test_place_blocks` の TC-I-02 / TC-I-03 に対し十分な余裕がある。
- `SimRobot` の時間の意味: `<option>` は timestep を変えない(0.002 s のまま)。`send_command` 1 回 = `n_substeps` 10 回の `mj_step` = 0.02 s は不変。1 step の計算時間はむしろ短い。

### 2.5 (e) 画像(目視確認用)

free camera(`lookat (0.22, 0.08, 0.05)`、`distance 0.62`、`azimuth 215`、`elevation −28`)と `cam_top`、640×480。

| seed(赤) | 画像 |
|---|---|
| 0 | `/private/tmp/claude-501/-Users-takuya-project-20-pai-homework/b14af3ef-30ce-4f56-a63b-8ac03b5a9e70/scratchpad/pick_survey/frames/red_seed0_lift_free.png` |
| 0 | `/private/tmp/claude-501/-Users-takuya-project-20-pai-homework/b14af3ef-30ce-4f56-a63b-8ac03b5a9e70/scratchpad/pick_survey/frames/red_seed0_pre_place_free.png` |
| 0 | `/private/tmp/claude-501/-Users-takuya-project-20-pai-homework/b14af3ef-30ce-4f56-a63b-8ac03b5a9e70/scratchpad/pick_survey/frames/red_seed0_open_free.png` |
| 0 | `/private/tmp/claude-501/-Users-takuya-project-20-pai-homework/b14af3ef-30ce-4f56-a63b-8ac03b5a9e70/scratchpad/pick_survey/frames/red_seed0_final_free.png` |
| 0 | `/private/tmp/claude-501/-Users-takuya-project-20-pai-homework/b14af3ef-30ce-4f56-a63b-8ac03b5a9e70/scratchpad/pick_survey/frames/red_seed0_pre_place_camtop.png` |
| 3 | `/private/tmp/claude-501/-Users-takuya-project-20-pai-homework/b14af3ef-30ce-4f56-a63b-8ac03b5a9e70/scratchpad/pick_survey/frames/red_seed3_lift_free.png` |
| 3 | `/private/tmp/claude-501/-Users-takuya-project-20-pai-homework/b14af3ef-30ce-4f56-a63b-8ac03b5a9e70/scratchpad/pick_survey/frames/red_seed3_pre_place_free.png` |
| 3 | `/private/tmp/claude-501/-Users-takuya-project-20-pai-homework/b14af3ef-30ce-4f56-a63b-8ac03b5a9e70/scratchpad/pick_survey/frames/red_seed3_open_free.png` |
| 3 | `/private/tmp/claude-501/-Users-takuya-project-20-pai-homework/b14af3ef-30ce-4f56-a63b-8ac03b5a9e70/scratchpad/pick_survey/frames/red_seed3_final_free.png` |
| 3 | `/private/tmp/claude-501/-Users-takuya-project-20-pai-homework/b14af3ef-30ce-4f56-a63b-8ac03b5a9e70/scratchpad/pick_survey/frames/red_seed3_pre_place_camtop.png` |
| 動画 | `.../pick_survey/frames/red_seed0.mp4`、`.../pick_survey/frames/red_seed3.mp4`(25 fps、制御周期 2 回に 1 フレーム) |

- 目視(本調査で 2 枚を確認): `pre_place` の画像で赤ブロックが顎に挟まれて運ばれている。`final` の画像で赤ブロックが箱の底に平らに置かれ、青・緑は元の位置のまま。free camera の構図で作業エリア・箱・腕が全て入る(腕が赤ブロックの一部を隠す場面がある)。

### 2.6 (f) 実行時間

| 項目 | 時間 |
|---|---|
| `SimRobot` の生成(シーンの読み込み) | 0.28 s |
| `IkPolicy` の生成 | 0.011 s |
| 計画(`act`) | 平均 0.17 s、最大 0.70 s(300 件) |
| 実行(軌道 + 待ち 1 s、動画なし) | 平均 0.17 s、最大 0.40 s |
| 1 試行の合計(動画なし) | 約 0.4 s(300 件で 118 s) |
| 1 試行の合計(動画あり、640×480、25 fps、libx264) | 約 8.7 s(描画とエンコード 約 7.5 s) |

- 統合テスト(3 seed、動画なし): 約 1.5 s。マーカー(`slow`)は不要。
- 20 seed × 3 色の掃引(動画なし): 約 20 s。100 seed × 3 色: 約 2 分。

---

## 3. 本調査の限界

- 失敗分類は実行失敗が 0 件のため、`fix` では検証できていない。`base`(全件 grasp_slip)と `gripper_closed` 0.30(grasp_slip 50 件)で `grasp_slip` が付くことだけを確認した。`drop` / `knocked_over` / `not_in_box` / `collision_other` の実例はない。
- 判定は解放から 1 s ちょうどではなく、軌道の終わり(解放 + `open_hold` 0.3 s + `retreat` 約 1 s)から 1 s 後に行った。FR-07 の「解放 1 秒後」より遅い時点なので、静止の条件は FR-07 より緩い側になる。補足の実測(`rel1s.py`、60 件中 計画可能 58 件): `open` 区間の開始から 50 周期(1 s)後のブロックの速さは最大 3.3e-5 m/s で、FR-07 の時点で測っても 1 cm/s を大きく下回る。
- sim の結果は固定顎の凸包の衝突形状と、この物理設定に依存する。実機の値は TASK-0035 で測り直す。

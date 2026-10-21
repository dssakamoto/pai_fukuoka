# TASK-0012 Red フェーズ記録: 把持・運搬の軌道生成(trajectory / IkPolicy)

**作成日**: 2026-10-21 / **要件名**: so101-vlm-pickplace / **機能名**: trajectory
**前提**: `trajectory-requirements.md`、`trajectory-testcases.md`(115 ケース)、`note.md`
**信頼性の凡例**: 🔵 要件・実測どおり / 🟡 妥当な推測 / 🔴 資料にない仮定

## 1. 作成したテスト

| ファイル | テスト関数 | 対応 ID |
|---|---|---|
| `tests/policy/test_trajectory.py` | 47 | TC-TJ-01〜25、E01〜E12、B01〜B09(46 ID。E12 は引数の誤りと NaN の 2 関数に分けた) |
| `tests/policy/test_trajectory_config.py` | 25 | TC-TC-01〜07、E01〜E14、B01〜B04 |
| `tests/policy/test_ik_policy.py` | 44 | TC-IP-01〜22、E01〜E17、B01〜B04、S01 |
| `tests/policy/trajectory_constants.py` | (補助モジュール) | 定数と合成の経由点。`conftest.py` は作らない(`tests/safety/conftest.py` と衝突させないため。`safety_constants.py` と同じ方式) |

- 合計 116 関数 = 115 ID(parametrize は ID 単位で 1 関数。実行されるテスト数はそれより多い)。意図的に欠けた ID はない。
- 書き方: docstring に `TC-xx-nn 名前 信頼性`、本文に `【確認内容】` コメント(日本語)。識別子は英語。
- 実行: `uv run pytest tests/policy/test_trajectory.py tests/policy/test_trajectory_config.py tests/policy/test_ik_policy.py`

## 2. Red で確定した API(Green が実装するもの)

### モジュール構成

| ファイル | 内容 |
|---|---|
| `src/so101_vlm/policy/base.py` | `Observation(q, timestamp)` + `Observation.from_state(RobotState)`、`Goal(pick_target_base, place_target_base, instruction="")`、`Policy`(`@runtime_checkable` Protocol、`act(obs, goal)`) |
| `src/so101_vlm/policy/trajectory.py` | 純粋層(下記) |
| `src/so101_vlm/policy/trajectory_config.py` | `TrajectoryConfig`、`ClearanceConfig`、`load_trajectory_config(path=DEFAULT_MOTION_CONFIG_PATH)` |
| `src/so101_vlm/policy/ik_policy.py` | `IkPolicy(model, *, config=None, safety=None)`。`from so101_vlm.policy.ik import solve_ik` をモジュールの名前空間に置く |
| `configs/motion.yaml` | 最上位キー `trajectory:` を追加(`safety:` は変えない) |

### `trajectory.py` の公開名(モジュール大域の関数。呼び出し時に名前を引くので monkeypatch できる)

- `segment_steps(q_a, q_b, *, vmax, speed_scale, control_dt, safety_factor, min_segment_s) -> int`
- `hold_steps(hold_s, control_dt) -> int`(`round(hold_s / control_dt)`)
- `interpolate_segment(q_a, q_b, n) -> ndarray (n, 6)`(k = 1..n、最後の行は `q_b` と完全一致)
- `JointWaypoint(name, q, hold_s=None)`(`hold_s` を与えた保持区間の `q` は直前の終点と同じ。違えば `ValueError`)
- `build_joint_trajectory(q_start, waypoints, *, vmax, speed_scale, control_dt, safety_factor, min_segment_s, metadata=None) -> JointTrajectory`
  - `segments` の先頭に `"start"` を自動で付ける。`metadata["duration_s"]` を自動で入れる。内部で大域名 `segment_steps` を引く(TC-IP-E09 で差し替える)。
- `check_command_sequence(q, labels, *, lower, upper, vmax, speed_scale, control_dt) -> None`(違反は `TrajectoryError("limit_violation", waypoint=その行のラベル)`。可動域は `q[1:]` だけ、速度は `q[0]→q[1]` を含む。≤ は合格)
- `find_clearance_violation(distances, labels, threshold) -> ClearanceViolation | None`(`.index`、`.label`、`.value`。最小のサンプルを返す。`< threshold` が違反)
- `JointTrajectory(times, q, labels, segments, metadata)`(frozen、配列は read-only、`metadata` は `MappingProxyType`、`__post_init__` の検証は `ValueError`)
- `TrajectoryError(reason, waypoint=None, detail=None)`(`RuntimeError` の派生、`detail` は read-only mapping、メッセージに経由点名)
- `REASON_*` の定数(値は要件 2.6 の文字列)
- `WaypointRecord`(`metadata["waypoints"]` の要素): 属性 `name`、`target`(3,)、`q`(6,)、`pos_error`、`pitch_deg`(実際の傾き)、`pitch_candidate_deg`(採用候補)

### 設定のフィールド(TC-TC-02)
`TrajectoryConfig`: `tcp_offset, approach_height, min_carry_z, place_height, gripper_open, gripper_closed, close_hold_s, open_hold_s, min_segment_s, time_safety_factor, clearance`。`ClearanceConfig`: `obstacle_m, self_m, obstacle_geoms`。frozen、list は float / str の tuple にそろえる。

### 例外の `detail` のキー(テストが読むもの)
- `carry_too_low`: `detail["z"]`
- `obstacle_clearance` / `self_collision`: `detail["min"]`(最小すき間 [m])
- IK の失敗を包んだもの: `detail["target"]`(目標点)。`__cause__` に元の `IkError`
- 理由の判定順(O4): 障害物 → 自己干渉

## 3. 期待される失敗(Red の実行結果)

`uv run pytest tests/policy -q -k "trajectory or ik_policy"`:

| ファイル | 失敗の理由 |
|---|---|
| `test_trajectory.py` | collection で `ModuleNotFoundError: No module named 'so101_vlm.policy.base'` |
| `test_trajectory_config.py` | collection で `ModuleNotFoundError: No module named 'so101_vlm.policy.trajectory_config'` |
| `test_ik_policy.py` | collection で `ModuleNotFoundError: No module named 'so101_vlm.policy.ik_policy'` |

3 ファイルとも文法は `py_compile` で確認済み。`trajectory_constants` の import は成功する。
既存テスト(上の 3 ファイルを除く): **979 passed**(11.9 s)。実装・スタブ・`configs/motion.yaml` は一切変えていない。

## 4. 実行時間の見込み

| ファイル | 予算 | 1 s を超えそうなテスト |
|---|---|---|
| `test_trajectory.py` | < 1 s | なし |
| `test_trajectory_config.py` | < 0.5 s | なし |
| `test_ik_policy.py` | < 8 s | fixture `solved`(8 配置、約 1.8 s)、TC-IP-17(20 配置、約 3.2 s)。TC-IP-16 は約 1 s(scale 0.3 の 1 回 + 安全フィルタ) |

## 5. 設計判断・テスト定義書との差異

1. TC-TJ-E12 を 2 関数に分けた(引数の誤り / Δ に NaN)。ID は 115 のまま。
2. conftest は作らず、fixture は `test_ik_policy.py` の中に置いた(`tests/safety/conftest.py` との衝突回避)。
3. 純粋層の関数名・`JointWaypoint` の形・`WaypointRecord` の属性名・`detail` のキーは本書 2 節で確定(O2)。
4. O5(保持 0 秒で区間を `segments` に残すか)は TC-TJ-B06 を `hold_steps(0.0, 0.02) == 0` だけにして Green に委ねた。
5. TC-IP-16 の「移動区間」: 保持を除く全区間の合計比 ≥ 3.2(実測 3.28 と同じ定義)、区間ごとの `(n1 - 1) / 0.3` の下限は `n1 > 10`(`min_segment_s` で決まらない区間)だけに適用。N = 811 などは記録のみでアサートしない。
6. TC-IP-22 のシード確認は腕の 5 関節(`[:5]`)だけ比較(IK の解は gripper を seed のまま返すため、経由点の q と gripper が違う)。
7. TC-TC-03 は既存の `load_safety_config` だけを使うので単独では今も通るが、ファイルの import 失敗で Red になる。TC-TC-01/02/04 は `trajectory:` を読むため Green まで失敗する。
8. 実モデル側の期待値(TC-IP-*)は参照実装の実測にもとづくが、テスト自体を参照実装に対して流してはいない。Green で数値が合わない場合は実測とテストのどちらが誤りかを確かめる。

## 6. Green への申し送り

- 上の API どおりに 4 ファイルを実装し、`configs/motion.yaml` に 2.9 節の `trajectory:`(根拠コメント付き)を追加する。
- `ik_policy.py` では `solve_ik` を `from so101_vlm.policy.ik import solve_ik` で import し、時間配分は `trajectory.build_joint_trajectory` に任せる(内部で `segment_steps` を大域名で引く)。
- 処理手順は要件 2.4(入力検証 → 目標 → 作業範囲 → carry_too_low → IK → 補間 → 指令列検査 → 経路の作業範囲 → すき間)。構築時に geom 名・gripper の ctrlrange を検査して `ValueError`。
- O1(配置範囲の下限 x)はユーザー判断のまま。本タスクでは変更しない。

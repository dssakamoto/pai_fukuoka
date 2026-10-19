# TASK-0004 Redフェーズ記録: sim-robot

- テストファイル: `tests/robot/test_sim.py`(テスト全文はこのファイルを参照)
- 対象: `src/so101_vlm/robot/sim.py` の `SimRobot`、`src/so101_vlm/robot/base.py` の `Robot`(runtime_checkable Protocol)と `RobotState`(frozen dataclass: q, dq, timestamp)
- 実行: `uv run pytest tests/robot/test_sim.py -q`

## 実行結果(期待どおりの失敗)

`ImportError: cannot import name 'Robot' from 'so101_vlm.robot.base'`(収集時エラー)。`SimRobot`(`so101_vlm.robot.sim`)も未実装。
既存 18 テスト(test_sim.py 以外)は成功。

## Greenフェーズで実装すること

- `base.py`: `RobotState`(frozen、q/dq は float64 の読み取り専用コピー、timestamp は float)、`Robot` Protocol(`joint_names`, `get_state`, `send_command`, `close`、runtime_checkable)
- `sim.py`: `SimRobot(scene_path=SO101_SCENE_PATH, n_substeps=10)`
  - 名前で関節・アクチュエータを解決(`jnt_qposadr` / `jnt_dofadr` / actuator id)。欠落は `ValueError`(名前を含む)
  - シーン無し `FileNotFoundError`、XML 不正 `ValueError`(パスを含む)、`n_substeps < 1` は `ValueError`
  - `send_command`: 形状 `(6,)` と有限性を検証(メッセージに shape / finite)、list 可、内部でコピー、n_substeps 回 `mj_step`、クリップしない
  - `get_state`: コピーを返す(副作用なし)。close 後は `RuntimeError("... closed")`、close は冪等、context manager 対応
- 信頼性: TC-B02 の内部属性確認は採用せず。TC-N08 は実シーン fresh を 7 回以内にするため別テストだが fresh_robot 使用数は N01x3,N02,N04,N08,B02 の 7 回

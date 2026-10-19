# TASK-0004 Green フェーズ記録: Robot 抽象と SimRobot

## 実装
- `src/so101_vlm/robot/base.py`: `JOINT_NAMES` 再エクスポート(同一オブジェクト)、`RobotState`(frozen、`__post_init__` で q/dq を読み取り専用コピー化)、`Robot`(runtime_checkable Protocol)。
- `src/so101_vlm/robot/sim.py`: `SimRobot`。名前ベースの添字解決(`mj_name2id`, `jnt_qposadr`, `jnt_dofadr`)、検証(shape (6,)・有限・n_substeps>=1)、クリップなし、close 冪等、closed は RuntimeError、context manager。MjData の配列は返さない。
- 行数: base.py 約 55 行、sim.py 約 115 行。モック・スタブなし。

## テスト結果
- `tests/robot/test_sim.py`: 41 passed(約 1.0〜1.4 s)
- 全体 `uv run pytest -q`: 59 passed

## テスト変更(1 件、要説明)
- `tests/robot/test_sim.py` の `_tiny_mjcf` で球 geom を `size="0.01"` から `size="0.1"` に変更(2 箇所)。
- 理由: testcases.md 0.3 は小モデルが 500 step で収束(誤差 8e-12)と実測済みとしているが、size=0.01, mass=1 では慣性 4e-5 に対し kv=1, dt=0.002 で陽的減衰が不安定になり QACC 発散(3 テスト失敗)。実装側の問題ではない。仕様(kp=50, kv=1, 重力0, timestep 0.002, 宣言順)は不変。

## 課題・リファクタ候補
- `send_command` の検証部を小関数に切り出す余地。
- `close()` は フラグのみ(MjData の明示解放はしていない)。

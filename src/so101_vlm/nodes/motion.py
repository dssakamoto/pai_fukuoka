"""motion ノード (TASK-0025): 軌道を安全フィルタ経由で 1 周期ずつ robot ノードへ送る。

🔵 design.md 3 章: motion = 安全フィルタを通して Robot に送る。既存の run_trajectory (Robot プロトコルだけを使う実行器) を、
通信越しの「代理ロボット」に対して走らせる。1 周期 = joint_command を送り、同じ seq の robot_state が返るのを待つ。
安全フィルタ・打ち切り・待ち (settle) の挙動は、プロセス内で実行する場合 (TASK-0013 / 0024) と同じ。
"""

import time
from types import SimpleNamespace

import numpy as np

from so101_vlm.eval.eval_config import load_eval_config
from so101_vlm.nodes.base import NodeBase, StopRequested
from so101_vlm.nodes.node_config import NodeConfig
from so101_vlm.nodes.state_codec import unpack_state
from so101_vlm.policy.executor import run_trajectory
from so101_vlm.policy.trajectory import JointTrajectory
from so101_vlm.robot.base import JOINT_NAMES, RobotState
from so101_vlm.robot.sim import SimRobot
from so101_vlm.safety.config import load_safety_config
from so101_vlm.safety.limits import limits_from_model


class _RemoteRobot:
    """【機能概要】: Robot プロトコルを満たす代理。send_command は送信して返事 (robot_state) を待つ 🔵"""

    joint_names = JOINT_NAMES

    def __init__(self, node: "MotionNode") -> None:
        self._node = node

    def get_state(self) -> RobotState:
        return self._node.latest_state()

    def send_command(self, q_target: np.ndarray) -> None:
        self._node.send_and_wait(q_target)

    def close(self) -> None:
        pass


def trajectory_from_json(raw: dict) -> JointTrajectory:
    """【機能概要】: trajectory_to_json の逆。経由点は executor が読む name / target だけを持つ 🔵"""
    waypoints = [SimpleNamespace(name=w["name"], target=np.array(w["target"])) for w in raw["waypoints"]]
    return JointTrajectory(
        times=np.array(raw["times"]), q=np.array(raw["q"]), labels=tuple(raw["labels"]),
        segments=tuple(raw["segments"]), metadata={"waypoints": waypoints} if waypoints else {},
    )


class MotionNode(NodeBase):
    """【機能概要】: joint_trajectory を受けて実行する。出力: joint_command (meta seq)、step ({"seq", "label"})、motion_done 🔵

    入力: joint_trajectory、robot_state (初期姿勢と、各指令への返事)、stop。
    stop が来たら、待っている途中でも静かに終わる (実機では、ここで指令を止める)。
    """

    def __init__(self, config: NodeConfig) -> None:
        super().__init__(config)
        self._model = SimRobot(config.scene_path).mujoco_handles()[0]
        self._state: RobotState | None = None
        self._state_seq = -1
        self._seq = 0

    # --- 代理ロボットが使う -------------------------------------------------
    def latest_state(self) -> RobotState:
        if self._state is None:
            raise RuntimeError("robot_state をまだ受け取っていません")
        return self._state

    def send_and_wait(self, q_target: np.ndarray) -> None:
        self._seq += 1
        seq = self._seq
        self.emit("joint_command", np.asarray(q_target, dtype=np.float64), {"seq": seq})
        self.wait_until(lambda: self._state_seq >= seq)

    # --- 入力 -----------------------------------------------------------------
    def handle(self, name: str, payload: object, meta: dict) -> None:
        if name == "robot_state":
            self._state = unpack_state(payload)
            self._state_seq = int(meta.get("seq", -1))
        elif name == "joint_trajectory":
            try:
                self._run(trajectory_from_json(payload))  # type: ignore[arg-type]
            except StopRequested:
                self.finished = True

    def _run(self, traj: JointTrajectory) -> None:
        self.wait_until(lambda: self._state is not None)
        lower, upper = limits_from_model(self._model)
        started = time.perf_counter()
        result = run_trajectory(
            _RemoteRobot(self), traj, load_safety_config(), lower=lower, upper=upper,
            settle_s=load_eval_config().judge.settle_s,
            on_step=lambda index, label: self.emit("step", {"seq": index, "label": label}, {}),
        )
        self.emit(
            "motion_done",
            {
                "n_steps": self._seq, "aborted": result.aborted, "abort_reason": result.abort_reason,
                "abort_label": result.abort_label, "flag_counts": dict(result.flag_counts),
                "final_joint_error": float(result.final_joint_error),
                "wall_time_s": time.perf_counter() - started,
            },
            {},
        )

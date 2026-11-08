"""sim の世界を持つノード: robot と camera (TASK-0025)。

🔵 design.md 3 章: sim と real の違いは、この 2 ノードが設定 (mode / robot.kind / camera.kind) で選ぶドライバだけ。
実機のドライバは TASK-0034 (So101Robot) / TASK-0036 (実機カメラ) で実装する。それまでは起動時に NotImplementedError で止める
(実機に誤って指令を送らない)。
sim の世界 (物理) は robot ノードだけが持つ。camera は robot が送る world (qpos / qvel) から同じ世界を作って描画する。
"""

import numpy as np

from so101_vlm.eval.eval_config import load_eval_config
from so101_vlm.eval.layout import place_blocks, sample_layout
from so101_vlm.nodes.base import NodeBase
from so101_vlm.nodes.node_config import NodeConfig
from so101_vlm.nodes.registry_types import TaskSpec
from so101_vlm.nodes.state_codec import apply_world, pack_state, pack_world
from so101_vlm.nodes.camera import render_rgbd
from so101_vlm.perception.calibration import camera_from_mujoco
from so101_vlm.robot.sim import SimRobot


def _require_sim(config: NodeConfig, what: str, kind: str) -> None:
    if config.mode != "sim" or kind != "sim":
        raise NotImplementedError(
            f"{what} は mode=sim / kind=sim だけ実装しています (mode={config.mode!r}, kind={kind!r})。"
            "real のドライバは TASK-0034 (So101Robot) / TASK-0036 (実機カメラ) で実装します"
        )


class RobotNode(NodeBase):
    """【機能概要】: sim のロボットと世界。task で初期配置を作り、joint_command ごとに物理を進める 🔵

    出力: robot_state ((13,) と meta seq)、world ((nq+nv,) と meta seq。初期は phase=init)。
    """

    def __init__(self, config: NodeConfig) -> None:
        super().__init__(config)
        _require_sim(config, "robot ノード", config.robot_kind)
        self._robot: SimRobot | None = None

    def _publish(self, seq: int, **meta: object) -> None:
        model, data = self._robot.mujoco_handles()  # type: ignore[union-attr]
        self.emit("robot_state", pack_state(self._robot.get_state()), {"seq": seq})  # type: ignore[union-attr]
        self.emit("world", pack_world(model, data), {"seq": seq, **meta})

    def handle(self, name: str, payload: object, meta: dict) -> None:
        if name == "task":
            task = TaskSpec(**payload)  # type: ignore[arg-type]
            place = load_eval_config().placement
            self._robot = SimRobot(self.config.scene_path)
            model, data = self._robot.mujoco_handles()
            place_blocks(model, data, sample_layout(task.seed, place.n_blocks, place.area, place.min_gap, place.max_attempts))
            self._publish(0, phase="init")
        elif name == "joint_command":
            if self._robot is None:
                raise RuntimeError("task より先に joint_command が届きました")
            self._robot.send_command(np.asarray(payload, dtype=np.float64))
            self._publish(int(meta["seq"]))


class CameraNode(NodeBase):
    """【機能概要】: sim のカメラ。最初の world (phase=init) から世界を再現して 1 回だけ描画する 🔵

    出力: rgb (H, W, 3 uint8)、depth (H, W float32 [m])、intrinsics ([fx, fy, cx, cy, width, height])。
    🟡 v1 は 1 試行 1 枚 (動作中の画像は流さない)。動画 (TASK-0028) は frame_sink で別に作る。
    """

    def __init__(self, config: NodeConfig) -> None:
        super().__init__(config)
        _require_sim(config, "camera ノード", config.camera_kind)
        self._sim = SimRobot(config.scene_path)
        self._done = False

    def handle(self, name: str, payload: object, meta: dict) -> None:
        if name != "world" or self._done or meta.get("phase") != "init":
            return
        cfg = self.config
        model, data = self._sim.mujoco_handles()
        apply_world(model, data, payload)
        rgb, depth = render_rgbd(model, data, cfg.camera_name, cfg.image_width, cfg.image_height)
        cam = camera_from_mujoco(model, cfg.camera_name, cfg.image_width, cfg.image_height)
        self._done = True
        self.emit("rgb", rgb, {})
        self.emit("depth", depth, {})
        self.emit("intrinsics", np.array([cam.fx, cam.fy, cam.cx, cam.cy, cam.width, cam.height], dtype=np.float64), {})

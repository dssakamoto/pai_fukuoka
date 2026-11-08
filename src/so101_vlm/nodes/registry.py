"""ノード id -> ノードの生成 (TASK-0025)。dataflow.yml の id と対応する。"""

from so101_vlm.nodes.base import NodeBase
from so101_vlm.nodes.evaluator import EvaluatorNode, RecorderNode, TaskNode
from so101_vlm.nodes.motion import MotionNode
from so101_vlm.nodes.node_config import NodeConfig
from so101_vlm.nodes.pipeline import PerceptionNode, PolicyNode, VlmPlannerNode
from so101_vlm.nodes.registry_types import TaskSpec
from so101_vlm.nodes.world import CameraNode, RobotNode

NODE_IDS = ("task", "robot", "camera", "vlm_planner", "perception", "policy", "motion", "evaluator", "recorder")


# 🔵 Intent: sim / real の切り替えは設定だけ。ドライバを選ぶのは robot / camera (world.py) で、ここは id で作るだけ。
def create_node(node_id: str, config: NodeConfig, task: TaskSpec | None = None) -> NodeBase:
    """【機能概要】: id のノードを作る。未知の id は ValueError。task ノードだけ TaskSpec が必要 🔵"""
    if node_id == "task":
        if task is None:
            raise ValueError("task ノードには TaskSpec が必要です")
        return TaskNode(config, task)
    factories = {
        "robot": RobotNode, "camera": CameraNode, "vlm_planner": VlmPlannerNode,
        "perception": PerceptionNode, "policy": PolicyNode, "motion": MotionNode,
        "evaluator": EvaluatorNode, "recorder": RecorderNode,
    }
    if node_id not in factories:
        raise ValueError(f"未知のノード id です: {node_id!r} (使えるもの: {NODE_IDS})")
    return factories[node_id](config)


__all__ = ["NODE_IDS", "TaskSpec", "create_node"]

"""各ノードの入出力のテスト (TASK-0025, Red)。dora なし。emit を記録する偽のランタイムに繋ぐ。"""

import dataclasses
import json

import numpy as np
import pytest

from so101_vlm.nodes.base import StopRequested
from so101_vlm.nodes.node_config import REAL_CONFIG_PATH, SIM_CONFIG_PATH, load_node_config
from so101_vlm.nodes.pipeline import PerceptionNode, VlmPlannerNode
from so101_vlm.nodes.registry import NODE_IDS, TaskSpec, create_node
from so101_vlm.vlm.schema import Detection, PlanError, PlanResult

CFG = load_node_config(SIM_CONFIG_PATH)
TASK = TaskSpec(seed=2, colour="red", instruction="put the red block into the box")
TASK_MSG = {"seed": 2, "colour": "red", "instruction": "go"}


class Bus:
    """emit を記録し、wait は条件が満たされているかだけを見る。"""

    def __init__(self):
        self.out = []

    def emit(self, name, payload, meta=None):
        self.out.append((name, payload, dict(meta or {})))

    def wait(self, predicate):
        if not predicate():
            raise AssertionError("単体テストでは、待つ条件が既に満たされている必要がある")

    def names(self):
        return [o[0] for o in self.out]

    def get(self, name):
        return [o for o in self.out if o[0] == name]


def _node(node_id, cfg=CFG, task=TASK):
    bus = Bus()
    node = create_node(node_id, cfg, task)
    node.bind(bus.emit, bus.wait)
    return node, bus


def test_registry_knows_every_node():
    assert set(NODE_IDS) == {
        "task", "robot", "camera", "vlm_planner", "perception", "policy", "motion", "evaluator", "recorder",
    }
    with pytest.raises(ValueError, match="mystery"):
        create_node("mystery", CFG, TASK)


def test_task_node_emits_once_and_finishes():
    node, bus = _node("task")
    node.start()
    assert bus.out == [("task", {"seed": 2, "colour": "red", "instruction": TASK.instruction}, {})]
    assert node.finished


@pytest.mark.parametrize("bad", [(-1, "red", "x"), (0, "purple", "x"), (0, "red", "")])
def test_task_spec_is_validated(bad):
    with pytest.raises(ValueError):
        TaskSpec(*bad)


def test_robot_node_initialises_the_scene_then_follows_commands():
    node, bus = _node("robot")
    node.on_input("task", TASK_MSG, {})
    state0, m0 = bus.get("robot_state")[0][1:]
    world0, wm0 = bus.get("world")[0][1:]
    assert state0.shape == (13,) and m0 == {"seq": 0}
    assert wm0 == {"seq": 0, "phase": "init"} and world0.ndim == 1 and world0.dtype == np.float64
    node.on_input("joint_command", state0[:6].copy(), {"seq": 1})
    state1, m1 = bus.get("robot_state")[1][1:]
    assert m1 == {"seq": 1} and state1[12] > state0[12]  # 時刻が進む
    assert bus.get("world")[1][2] == {"seq": 1}
    node.on_input("stop", {}, {})
    assert node.finished


def test_robot_node_rejects_commands_before_the_task():
    node, _ = _node("robot")
    with pytest.raises(RuntimeError, match="task"):
        node.on_input("joint_command", np.zeros(6), {"seq": 1})


def test_camera_renders_only_the_initial_world():
    robot, rbus = _node("robot")
    robot.on_input("task", TASK_MSG, {})
    world, wmeta = rbus.get("world")[0][1:]
    camera, cbus = _node("camera")
    camera.on_input("world", world, wmeta)
    assert cbus.names() == ["rgb", "depth", "intrinsics"]
    rgb, depth, intr = (o[1] for o in cbus.out)
    assert rgb.shape == (480, 640, 3) and rgb.dtype == np.uint8 and rgb.std() > 5
    assert depth.shape == (480, 640) and depth.dtype == np.float32
    assert intr.shape == (6,) and intr[4] == 640 and intr[5] == 480
    camera.on_input("world", world, {"seq": 1})
    assert len(cbus.out) == 3  # 2 回目以降は描かない


@pytest.mark.parametrize("node_id", ["robot", "camera"])
def test_real_mode_selects_drivers_that_are_not_implemented_yet(node_id):
    real = load_node_config(REAL_CONFIG_PATH)
    with pytest.raises(NotImplementedError, match="real"):
        create_node(node_id, real, TASK)


def test_other_nodes_do_not_depend_on_the_mode():
    real = load_node_config(REAL_CONFIG_PATH)
    for node_id in ("task", "perception", "evaluator", "recorder"):
        assert create_node(node_id, real, TASK) is not None


def test_missing_api_key_stops_the_vlm_planner_at_startup(monkeypatch):
    from so101_vlm.vlm import MissingApiKeyError

    monkeypatch.delenv("ANTHROPIC_API_KEY", raising=False)
    real = load_node_config(REAL_CONFIG_PATH)  # provider: anthropic
    with pytest.raises(MissingApiKeyError, match="ANTHROPIC_API_KEY"):
        create_node("vlm_planner", real, TASK)


# --- vlm_planner / perception -------------------------------------------------
class _Client:
    def __init__(self, error=None):
        self.error, self.calls = error, []

    def plan(self, image, instruction):
        self.calls.append((image.shape, instruction))
        if self.error:
            raise self.error
        return PlanResult(
            Detection("red block", (100, 100, 150, 150), 0.9), Detection("box", (300, 200, 400, 300), None)
        )


def _planner(client):
    bus = Bus()
    node = VlmPlannerNode(CFG, client_factory=lambda cfg, task: client)
    node.bind(bus.emit, bus.wait)
    return node, bus


RGB = np.zeros((480, 640, 3), np.uint8)


def test_vlm_planner_waits_for_both_inputs_then_emits_detections():
    client = _Client()
    node, bus = _planner(client)
    node.on_input("rgb", RGB, {})
    assert bus.out == []
    node.on_input("task", TASK_MSG, {})
    assert len(bus.out) == 1
    name, det, _ = bus.out[0]
    assert name == "detections" and det["pick"]["bbox_px"] == [100, 100, 150, 150]
    assert det["pick"]["confidence"] == 0.9 and det["place"]["confidence"] is None
    assert client.calls == [((480, 640, 3), "go")]


def test_vlm_planner_reports_plan_error_as_failure_and_nothing_else():
    node, bus = _planner(_Client(error=PlanError("no purple block")))
    node.on_input("task", TASK_MSG, {})
    node.on_input("rgb", RGB, {})
    assert bus.names() == ["failure_vlm"]
    assert bus.out[0][1] == {"kind": "vlm_failure", "error": "no purple block"}


def test_vlm_planner_does_not_swallow_unexpected_errors():
    node, _ = _planner(_Client(error=RuntimeError("401 unauthorized")))
    node.on_input("task", TASK_MSG, {})
    with pytest.raises(RuntimeError, match="401"):
        node.on_input("rgb", RGB, {})


def _detections():
    return {
        "pick": {"label": "red block", "bbox_px": [100, 100, 150, 150], "confidence": 0.9},
        "place": {"label": "box", "bbox_px": [300, 200, 400, 300], "confidence": None},
    }


def _perception(depth):
    bus = Bus()
    node = PerceptionNode(CFG)
    node.bind(bus.emit, bus.wait)
    node.on_input("task", TASK_MSG, {})
    node.on_input("intrinsics", np.array([500.0, 500.0, 319.5, 239.5, 640, 480]), {})
    node.on_input("depth", depth, {})
    node.on_input("detections", _detections(), {})
    return bus


def test_perception_emits_3d_targets():
    bus = _perception(np.full((480, 640), 0.5, np.float32))
    assert bus.names() == ["targets_3d"]
    targets = bus.out[0][1]
    assert len(targets["pick"]) == 3 and len(targets["place"]) == 3 and targets["instruction"] == "go"
    assert targets["place"][2] == pytest.approx(0.005)  # 箱の底板の上面


def test_perception_failure_when_depth_is_invalid():
    bus = _perception(np.zeros((480, 640), np.float32))  # 有効な深度がない
    assert bus.names() == ["failure_perception"]
    assert bus.out[0][1]["kind"] == "perception_failure" and "深度" in bus.out[0][1]["error"]


# --- policy ---------------------------------------------------------------------
def _initial_state():
    robot, rbus = _node("robot")
    robot.on_input("task", TASK_MSG, {})
    return rbus.get("robot_state")[0][1:]


def test_policy_refuses_a_target_outside_the_workspace_without_a_trajectory():
    state, smeta = _initial_state()
    policy, pbus = _node("policy")
    policy.on_input("robot_state", state, smeta)
    policy.on_input("targets_3d", {"pick": [0.9, 0.9, 0.015], "place": [0.2, 0.19, 0.005], "instruction": "x"}, {})
    assert pbus.names() == ["failure_policy"]
    failure = pbus.out[0][1]
    assert failure["kind"] == "planning_failure" and failure["reason"]


def test_policy_emits_a_serialisable_trajectory():
    state, smeta = _initial_state()
    policy, pbus = _node("policy")
    policy.on_input("targets_3d", {"pick": [0.2023, -0.0403, 0.015], "place": [0.2, 0.19, 0.005], "instruction": "x"}, {})
    assert pbus.out == []  # 状態がまだ
    policy.on_input("robot_state", state, smeta)
    assert pbus.names() == ["joint_trajectory"]
    traj = pbus.out[0][1]
    assert len(traj["q"]) == len(traj["labels"]) == len(traj["times"]) > 100
    assert len(traj["q"][0]) == 6 and {w["name"] for w in traj["waypoints"]} >= {"pre_grasp", "grasp"}


# --- motion / evaluator / recorder --------------------------------------------------
def test_motion_stops_when_asked_while_waiting():
    node, bus = _node("motion")

    def wait(predicate):
        raise StopRequested()

    node.bind(bus.emit, wait)
    node.on_input("stop", {}, {})
    assert node.finished


def test_evaluator_turns_a_failure_into_a_result_and_stops_everything():
    node, bus = _node("evaluator")
    node.on_input("task", TASK_MSG, {})
    node.on_input("failure_vlm", {"kind": "vlm_failure", "error": "no purple block"}, {})
    result = bus.get("result")[0][1]
    assert result["success"] is False and result["failure"] == "vlm_failure"
    assert result["error"] == "no purple block" and result["seed"] == 2 and result["colour"] == "red"
    assert bus.names() == ["result", "stop"] and node.finished


def test_recorder_writes_the_trial_record(tmp_path):
    cfg = dataclasses.replace(CFG, output_dir=tmp_path)
    node, _ = _node("recorder", cfg)
    node.on_input("targets_3d", {"pick": [0.1, 0.2, 0.015], "place": [0.2, 0.19, 0.005], "instruction": "go"}, {})
    node.on_input("result", {"seed": 2, "colour": "red", "success": True, "failure": None}, {})
    files = list(tmp_path.glob("trial_seed2_red.json"))
    assert len(files) == 1
    data = json.loads(files[0].read_text(encoding="utf-8"))
    assert data["result"]["success"] is True and data["targets_3d"]["pick"] == [0.1, 0.2, 0.015]
    assert node.finished

"""nodes.node_config とデータフロー定義のテスト (TASK-0025, Red)。"""

import copy
import json

import pytest
import yaml

from so101_vlm.nodes.dataflow import DATAFLOW_TEMPLATE_PATH, parse_graph, render_dataflow
from so101_vlm.nodes.node_config import (
    REAL_CONFIG_PATH,
    SIM_CONFIG_PATH,
    load_node_config,
)

EXPECTED_NODES = {
    "task", "robot", "camera", "vlm_planner", "perception", "policy", "motion", "evaluator", "recorder",
}


def test_sim_config_loads():
    cfg = load_node_config(SIM_CONFIG_PATH)
    assert cfg.mode == "sim" and cfg.robot_kind == "sim" and cfg.camera_kind == "sim"
    assert cfg.scene_path.is_file() and cfg.calibration_path.is_file()
    assert cfg.camera_name == "cam_top" and (cfg.image_width, cfg.image_height) == (640, 480)
    assert cfg.vlm_provider == "oracle" and cfg.timeout_s > 0


def test_real_config_loads_but_selects_real_drivers():
    cfg = load_node_config(REAL_CONFIG_PATH)
    assert cfg.mode == "real" and cfg.robot_kind == "so101" and cfg.camera_kind == "real"


def _raw():
    return yaml.safe_load(SIM_CONFIG_PATH.read_text(encoding="utf-8"))


def _write(tmp_path, raw):
    p = tmp_path / "c.yaml"
    p.write_text(yaml.safe_dump(raw), encoding="utf-8")
    return p


@pytest.mark.parametrize(
    "mutate",
    [
        lambda r: r.update(mode="hybrid"),
        lambda r: r["robot"].update(kind="mystery"),
        lambda r: r["camera"].update(kind="mystery"),
        lambda r: r["camera"].update(width=0),
        lambda r: r.update(timeout_s=-1),
        lambda r: r.update(surprise=1),
        lambda r: r.pop("scene"),
        lambda r: r.update(vlm={"provider": ""}),
    ],
)
def test_invalid_config_is_value_error(tmp_path, mutate):
    raw = copy.deepcopy(_raw())
    mutate(raw)
    with pytest.raises(ValueError):
        load_node_config(_write(tmp_path, raw))


# --- dataflow.yml ----------------------------------------------------------
def _render(**over):
    values = {
        "python": r"C:\Python 3.12\python.exe", "config": "configs/sim.yaml", "seed": 2,
        "colour": "red", "instruction": 'put the "red" block\ninto the box: now', "out_dir": "media/x",
    }
    values.update(over)
    return render_dataflow(DATAFLOW_TEMPLATE_PATH.read_text(encoding="utf-8"), **values)


def test_rendered_dataflow_is_valid_yaml_with_all_nodes_and_safe_values():
    doc = yaml.safe_load(_render())
    nodes = {n["id"]: n for n in doc["nodes"]}
    assert set(nodes) == EXPECTED_NODES
    for n in nodes.values():
        assert n["path"] == r"C:\Python 3.12\python.exe"  # OS 非依存にするため、インタプリタを明示
        assert n["args"].startswith("-m so101_vlm.nodes.run ") and n["args"].endswith(n["id"])
    env = nodes["task"]["env"]
    assert env["TASK_INSTRUCTION"] == 'put the "red" block\ninto the box: now'
    assert env["TASK_SEED"] == "2" and env["TASK_COLOUR"] == "red"
    assert nodes["camera"]["env"]["CONFIG"] == "configs/sim.yaml"


def test_only_camera_and_robot_read_the_mode_config_for_drivers():
    # sim/real の切り替えは CONFIG だけ。ドライバを持つのは camera と robot (設計書 3 章)
    doc = yaml.safe_load(_render(config="configs/real.yaml"))
    for n in doc["nodes"]:
        assert n["env"]["CONFIG"] == "configs/real.yaml"


def test_instruction_cannot_inject_yaml():
    doc = yaml.safe_load(_render(instruction="x\n  - id: evil\n    path: rm"))
    assert {n["id"] for n in doc["nodes"]} == EXPECTED_NODES


def test_graph_wiring():
    g = parse_graph(_render())
    assert set(g) == EXPECTED_NODES
    assert g["camera"].inputs["world"] == "robot/world"
    assert g["vlm_planner"].inputs["rgb"] == "camera/rgb"
    assert g["motion"].inputs["joint_trajectory"] == "policy/joint_trajectory"
    assert g["robot"].inputs["joint_command"] == "motion/joint_command"
    # 全ノードが evaluator の stop で終われる
    for node_id in EXPECTED_NODES - {"evaluator"}:
        assert g[node_id].inputs.get("stop") == "evaluator/stop", node_id
    # 出力の名前は、配線先の入力が参照するものと一致する
    for node in g.values():
        for src in node.inputs.values():
            src_node, out = src.split("/")
            assert out in g[src_node].outputs, src


def test_dataflow_values_are_json_quoted(tmp_path):
    text = _render(instruction="a")
    assert json.dumps(r"C:\Python 3.12\python.exe") in text

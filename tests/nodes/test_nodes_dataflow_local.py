"""データフロー全体 (ローカルランナー) のテスト (TASK-0025, Red)。

dora なしで、dataflow.yml と同じ配線を同じプロセス内で再生する。run_trial (TASK-0024) と同じ結果になることを確かめる。
"""

import numpy as np
import pytest

from so101_vlm.eval.failure import FailureKind
from so101_vlm.eval.layout import sample_layout
from so101_vlm.eval.oracle_vlm import OracleVlm
from so101_vlm.eval.trial import run_trial
from so101_vlm.nodes.local_runner import LocalRunner
from so101_vlm.nodes.node_config import SIM_CONFIG_PATH, load_node_config
from so101_vlm.nodes.pipeline import VlmPlannerNode
from so101_vlm.perception.calibration import load_calibration
from so101_vlm.vlm.schema import PlanError

CFG = load_node_config(SIM_CONFIG_PATH)
INSTRUCTION = "put the red block into the box"


def _trial(seed, colour="red"):
    return run_trial(seed, colour, INSTRUCTION, OracleVlm(sample_layout(seed), colour, load_calibration()))


@pytest.fixture(scope="module")
def seed2(tmp_path_factory):
    import dataclasses

    cfg = dataclasses.replace(CFG, output_dir=tmp_path_factory.mktemp("dataflow"))
    runner = LocalRunner(cfg, seed=2, colour="red", instruction=INSTRUCTION)
    return runner.run(), runner


@pytest.mark.render
def test_dataflow_gives_the_same_result_as_run_trial(seed2):
    result, _ = seed2
    expected = _trial(2)
    assert result["success"] is True and expected.success
    assert result["failure"] is None and result["error"] is None
    assert result["seed"] == 2 and result["colour"] == "red" and result["instruction"] == INSTRUCTION
    assert np.allclose(result["pick_estimate"], expected.pick_estimate, atol=1e-9)
    assert result["pick_error_mm"] == pytest.approx(expected.pick_error_mm, abs=1e-6)
    assert result["judge"]["reasons"] == []


@pytest.mark.render
def test_every_node_took_part_in_the_run(seed2):
    _, runner = seed2
    emitted = set(runner.log)
    for expected in [
        ("task", "task"), ("robot", "world"), ("camera", "rgb"), ("vlm_planner", "detections"),
        ("perception", "targets_3d"), ("policy", "joint_trajectory"), ("motion", "joint_command"),
        ("motion", "motion_done"), ("evaluator", "result"), ("evaluator", "stop"),
    ]:
        assert expected in emitted, expected


@pytest.mark.render
def test_recorder_wrote_the_trial(seed2):
    _, runner = seed2
    assert runner.recorded_paths and runner.recorded_paths[0].exists()


@pytest.mark.render
def test_vlm_failure_never_moves_the_robot():
    def failing(cfg, task):
        class C:
            def plan(self, image, instruction):
                raise PlanError("no purple block")

        return C()

    runner = LocalRunner(
        CFG, seed=2, colour="red", instruction=INSTRUCTION,
        overrides={"vlm_planner": lambda cfg, task: VlmPlannerNode(cfg, client_factory=failing)},
    )
    result = runner.run()
    assert result["success"] is False and result["failure"] == FailureKind.VLM_FAILURE.value
    assert "no purple block" in result["error"]
    assert ("motion", "joint_command") not in set(runner.log)
    assert ("policy", "joint_trajectory") not in set(runner.log)


@pytest.mark.render
def test_planning_failure_matches_run_trial_and_never_moves_the_robot():
    expected = _trial(1)  # seed 1 は obstacle_clearance で計画が拒否される
    assert expected.failure is FailureKind.PLANNING_FAILURE
    runner = LocalRunner(CFG, seed=1, colour="red", instruction=INSTRUCTION)
    result = runner.run()
    assert result["failure"] == "planning_failure" and result["success"] is False
    assert result["planning_error"] == expected.run.planning_error
    assert ("motion", "joint_command") not in set(runner.log)


@pytest.mark.render
def test_runner_without_a_result_is_an_error():
    runner = LocalRunner(
        CFG, seed=2, colour="red", instruction=INSTRUCTION,
        overrides={"evaluator": lambda cfg, task: _Silent()},
    )
    with pytest.raises(RuntimeError, match="result"):
        runner.run()


class _Silent:
    finished = False

    def bind(self, emit, wait):
        pass

    def start(self):
        pass

    def on_input(self, name, payload, meta):
        pass

"""run_trial の統合テスト (TASK-0024, Red)。

VLM は API を使わない偽物 (OracleVlm か、失敗を返す偽物)。シミュレーション (MuJoCo) で、
画像 -> VLM -> 知覚 -> IK -> 実行 -> 判定 を 1 試行として通す。
"""

import numpy as np
import pytest

import so101_vlm.eval.trial as trial_module
from so101_vlm.eval.failure import FailureKind
from so101_vlm.eval.fixed_pick import FixedPickOutcome
from so101_vlm.eval.layout import sample_layout
from so101_vlm.eval.oracle_vlm import OracleVlm
from so101_vlm.eval.trial import TrialOutcome, run_trial
from so101_vlm.perception.calibration import load_calibration
from so101_vlm.vlm.schema import Detection, PlanError, PlanResult

CAL = load_calibration()
INSTRUCTION = "put the red block into the box"


def _oracle(seed, colour):
    return OracleVlm(sample_layout(seed), colour, CAL)


class RecordingVlm:
    """渡された画像と指示を記録し、決まった結果を返す (または投げる)。"""

    def __init__(self, result=None, error=None):
        self.result, self.error, self.calls = result, error, []

    def plan(self, image, instruction):
        self.calls.append((image.copy(), instruction))
        if self.error is not None:
            raise self.error
        return self.result


@pytest.fixture
def no_motion(monkeypatch):
    """run_goal (計画・実行) が呼ばれたら失敗させる。呼ばれた回数を数える。"""
    calls = []

    def boom(*args, **kwargs):
        calls.append(1)
        raise AssertionError("run_goal は呼ばれてはいけない (動作前に拒否するはず)")

    monkeypatch.setattr(trial_module, "run_goal", boom)
    return calls


# --- 失敗時は動かない (FR-03) -----------------------------------------------
@pytest.mark.render
def test_vlm_failure_records_failure_and_does_not_move(no_motion):
    vlm = RecordingVlm(error=PlanError("no purple block"))
    seen = []
    out = run_trial(0, "red", INSTRUCTION, vlm, frame_sink=lambda *a: seen.append(a))
    assert isinstance(out, TrialOutcome)
    assert out.failure is FailureKind.VLM_FAILURE and not out.success
    assert "no purple block" in out.error
    assert out.plan is None and out.run is None and out.pick_error_mm is None
    assert no_motion == [] and seen == []


@pytest.mark.render
def test_perception_failure_records_failure_and_does_not_move(no_motion, monkeypatch):
    def broken(*a, **k):
        raise ValueError("bbox 中央領域に有効な深度がありません")

    monkeypatch.setattr(trial_module, "bbox_to_base_point", broken)
    out = run_trial(0, "red", INSTRUCTION, _oracle(0, "red"))
    assert out.failure is FailureKind.PERCEPTION_FAILURE and not out.success
    assert "深度" in out.error
    assert out.plan is not None and out.run is None
    assert no_motion == []


@pytest.mark.render
def test_target_far_outside_workspace_is_refused_before_motion():
    # 画像の隅 (作業台の外) を指す検出。IK / 作業範囲の検査で拒否され、実行されない
    far = PlanResult(Detection("red block", (0, 0, 20, 20), 1.0), Detection("box", (300, 250, 420, 370), 1.0))
    seen = []
    out = run_trial(0, "red", INSTRUCTION, RecordingVlm(far), frame_sink=lambda *a: seen.append(a))
    assert out.failure is not None and not out.success
    assert out.run is not None and out.run.execution is None  # 計画 (またはその前の検査) で拒否
    assert seen == []


# --- VLM への入力 ------------------------------------------------------------
@pytest.mark.render
def test_vlm_receives_the_camera_image_and_the_instruction():
    vlm = RecordingVlm(error=PlanError("stop here"))
    run_trial(0, "red", INSTRUCTION, vlm)
    image, instruction = vlm.calls[0]
    assert instruction == INSTRUCTION
    assert image.shape == (480, 640, 3) and image.dtype == np.uint8 and image.std() > 5


# --- 通し (統合) ---------------------------------------------------------------
@pytest.fixture(scope="module")
def red_seed0():
    return run_trial(0, "red", INSTRUCTION, _oracle(0, "red"))


@pytest.mark.render
def test_full_trial_succeeds_with_cached_style_detection(red_seed0):
    out = red_seed0
    assert out.success and out.failure is None and out.error is None
    assert isinstance(out.run, FixedPickOutcome) and out.run.judge.success
    assert out.seed == 0 and out.colour == "red" and out.instruction == INSTRUCTION
    assert out.plan.pick.label.startswith("red")


@pytest.mark.render
def test_estimates_are_close_to_the_truth(red_seed0):
    out = red_seed0
    assert out.pick_error_mm is not None and out.pick_error_mm < 15.0
    assert len(out.pick_estimate) == 3 and len(out.place_estimate) == 3
    # 置き場所の xy は、箱の中心から内寸の半分 (4 cm) より十分近い
    assert np.hypot(out.place_estimate[0] - 0.20, out.place_estimate[1] - 0.19) < 0.02


@pytest.mark.render
def test_trial_is_deterministic(red_seed0):
    again = run_trial(0, "red", INSTRUCTION, _oracle(0, "red"))
    assert again.pick_estimate == red_seed0.pick_estimate
    assert again.success == red_seed0.success


@pytest.mark.render
def test_frame_sink_is_called_during_execution():
    seen = []
    out = run_trial(2, "red", INSTRUCTION, _oracle(2, "red"), frame_sink=lambda m, d, i, label: seen.append(label))
    assert out.run.execution is not None and len(seen) > 0


# --- 引数 ------------------------------------------------------------------------
@pytest.mark.parametrize("seed,colour,instr", [(-1, "red", "x"), (0, "purple", "x"), (0, "red", ""), (0, "red", "  ")])
def test_bad_arguments_are_value_error_before_anything_runs(seed, colour, instr, no_motion):
    vlm = RecordingVlm(error=PlanError("must not be called"))
    with pytest.raises(ValueError):
        run_trial(seed, colour, instr, vlm)
    assert vlm.calls == []

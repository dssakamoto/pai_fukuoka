"""eval.trial_report のテスト (AC-02, Red)。1 試行の結果を、人が読む行にする。純粋ロジック (sim なし)。"""

from types import SimpleNamespace

import pytest

from so101_vlm.eval.failure import FAILURE_GROUP, FailureKind
from so101_vlm.eval.trial import TrialOutcome
from so101_vlm.eval.trial_report import FAILURE_EXPLANATION, format_trial_report
from so101_vlm.vlm.schema import Detection, PlanResult

PLAN = PlanResult(Detection("red block", (257, 150, 287, 184), 0.9), Detection("box", (339, 207, 434, 302), None))
INSTR = "put the purple block into the box"


def _outcome(**over):
    base = dict(
        seed=2, colour="red", instruction=INSTR, plan=None, pick_estimate=None, place_estimate=None,
        pick_error_mm=None, run=None, failure=None, error=None, vlm_time_s=0.5,
    )
    base.update(over)
    return TrialOutcome(**base)


def _text(outcome, provider="openai", model="gpt-6-luna"):
    return "\n".join(format_trial_report(outcome, provider=provider, model=model))


def test_nonexistent_colour_is_reported_as_not_moved_with_the_reason():
    # AC-02: 存在しない色 -> VLM が「対象なし」 -> 動作せず、理由が表示される
    out = _outcome(failure=FailureKind.VLM_FAILURE, error="pick がありません: no purple block in the image")
    text = _text(out)
    assert INSTR in text and "openai" in text and "gpt-6-luna" in text
    assert "動かしていません" in text and "no purple block in the image" in text
    assert "失敗" in text and "vlm_failure" in text and "対象が見つからない" in text
    assert "検出: なし" in text and "推定位置" not in text


def test_perception_failure_is_reported_as_not_moved():
    out = _outcome(plan=PLAN, failure=FailureKind.PERCEPTION_FAILURE, error="bbox 中央領域に有効な深度がありません")
    text = _text(out)
    assert "動かしていません" in text and "有効な深度がありません" in text and "(257, 150, 287, 184)" in text


def test_planning_failure_names_the_check_and_waypoint():
    run = SimpleNamespace(execution=None, planning_error="obstacle_clearance", planning_waypoint="place", judge=None)
    out = _outcome(
        plan=PLAN, pick_estimate=(0.257, 0.094, 0.013), place_estimate=(0.193, 0.189, 0.005), pick_error_mm=6.6,
        run=run, failure=FailureKind.PLANNING_FAILURE,
    )
    text = _text(out)
    assert "動かしていません" in text and "obstacle_clearance" in text and "place" in text
    assert "(0.257, 0.094, 0.013)" in text and "6.6 mm" in text


def test_success_shows_detection_estimate_and_result():
    run = SimpleNamespace(execution=object(), planning_error=None, planning_waypoint=None, success=True)
    out = _outcome(
        instruction="put the red block into the box", plan=PLAN, pick_estimate=(0.208, -0.034, 0.012),
        place_estimate=(0.193, 0.189, 0.005), pick_error_mm=9.3, run=run,
    )
    text = _text(out)
    assert "red block" in text and "(257, 150, 287, 184)" in text and "box" in text
    assert "ロボットを動かしました" in text and "判定: 成功" in text and "9.3 mm" in text
    assert "理由" not in text


def test_failure_after_execution_says_the_robot_moved():
    run = SimpleNamespace(execution=object(), planning_error=None, planning_waypoint=None, success=False)
    out = _outcome(plan=PLAN, pick_estimate=(0.2, 0.0, 0.015), pick_error_mm=7.0, run=run, failure=FailureKind.GRASP_SLIP)
    text = _text(out)
    assert "ロボットを動かしました" in text and "grasp_slip" in text and "持ち上げ" in text


def test_every_failure_kind_has_an_explanation():
    assert set(FAILURE_EXPLANATION) == set(FailureKind) == set(FAILURE_GROUP)
    assert all(isinstance(v, str) and v for v in FAILURE_EXPLANATION.values())


def test_reason_is_collapsed_to_one_line_and_never_empty_for_failures():
    text = _text(_outcome(failure=FailureKind.VLM_FAILURE, error="line1\nline2\n\n  line3"))
    assert "line1 line2 line3" in text
    text = _text(_outcome(failure=FailureKind.VLM_FAILURE, error=None))
    assert "理由:" in text  # 理由が無くても、失敗の種類の説明は出る


@pytest.mark.parametrize("bad", [None, "outcome", 3])
def test_bad_outcome_is_value_error(bad):
    with pytest.raises(ValueError):
        format_trial_report(bad, provider="p", model="m")


def _executed(kind, *, reasons=(), lift=0.02, offset=(0.0, 0.0), height=0.015, speed=0.0):
    from so101_vlm.eval.judge import JudgeReason

    judge = SimpleNamespace(reasons=tuple(JudgeReason(r) for r in reasons), offset_xy=offset, height=height, speed=speed)
    trace = SimpleNamespace(lift_end_rise=lift)
    run = SimpleNamespace(execution=object(), planning_error=None, planning_waypoint=None, success=False, judge=judge, trace=trace)
    return _outcome(plan=PLAN, pick_estimate=(0.2, 0.0, 0.015), pick_error_mm=7.0, run=run, failure=kind)


def test_failure_after_execution_always_has_a_concrete_reason():
    # 実 VLM の試行 (2026-11-15) で、「理由:」が空になっていた: 実行後の失敗にも、判定と追跡の数値から理由を出す
    text = _text(_executed(FailureKind.GRASP_SLIP, reasons=("outside_inner",), lift=0.002, offset=(0.081, -0.02)))
    reason = next(l for l in text.splitlines() if l.startswith("理由:"))
    assert len(reason) > len("理由:") + 5
    assert "outside_inner" in reason and "持ち上げ 2 mm" in reason and "81" in reason


def test_reason_for_each_executed_failure_mentions_the_judge_conditions():
    for kind in (FailureKind.NOT_IN_BOX, FailureKind.KNOCKED_OVER, FailureKind.DROP, FailureKind.COLLISION_OTHER):
        text = _text(_executed(kind, reasons=("above_rim", "moving"), height=0.045, speed=0.03))
        reason = next(l for l in text.splitlines() if l.startswith("理由:"))
        assert "above_rim" in reason and "moving" in reason and "45 mm" in reason and "3.0 cm/s" in reason

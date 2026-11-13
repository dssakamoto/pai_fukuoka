"""eval.runner のテスト (TASK-0026, Red)。集計・CSV・試行列は純粋ロジック。統合は偽 VLM (OracleVlm) で実際に回す。"""

import csv
import dataclasses
import json

import pytest

from so101_vlm.eval.failure import FAILURE_GROUP, FailureGroup, FailureKind
from so101_vlm.eval.runner import (
    CSV_COLUMNS,
    TrialRecord,
    format_summary,
    oracle_client_factory,
    record_from_outcome,
    run_trials,
    summarize,
    write_csv,
    write_summary,
)
from so101_vlm.eval.trial import TrialOutcome
from so101_vlm.vlm.schema import Detection, PlanError, PlanResult

DESIGN_4_8 = {
    "seed", "instruction", "provider", "model", "success", "failure", "pick_error_mm", "total_time_s",
    "pick_bbox", "place_bbox",  # 検出結果
}


def _rec(i=0, *, success=True, failure=None, error_mm=5.0, **over):
    base = dict(
        trial=i, seed=i, colour="red", instruction="put the red block into the box", provider="oracle",
        model="ground-truth", success=success, failure=failure,
        failure_group=None if failure is None else FAILURE_GROUP[FailureKind(failure)].value,
        error=None, planning_error=None, pick_error_mm=error_mm, pick_label="red block",
        pick_xyz="0.2 0.0 0.015", pick_bbox="1 2 3 4", place_bbox="5 6 7 8", vlm_time_s=0.01,
        plan_time_s=0.2, total_time_s=7.0, other_block_moved=False,
    )
    base.update(over)
    return TrialRecord(**base)


# --- 失敗のグループ (design.md 4.8) -----------------------------------------------
def test_every_failure_kind_belongs_to_a_group():
    assert set(FAILURE_GROUP) == set(FailureKind)
    assert FAILURE_GROUP[FailureKind.VLM_FAILURE] is FailureGroup.VLM
    assert FAILURE_GROUP[FailureKind.PERCEPTION_FAILURE] is FailureGroup.VLM
    assert FAILURE_GROUP[FailureKind.PLANNING_FAILURE] is FailureGroup.IK
    assert FAILURE_GROUP[FailureKind.GRASP_SLIP] is FailureGroup.GRASP
    assert FAILURE_GROUP[FailureKind.DROP] is FailureGroup.GRASP
    for kind in (FailureKind.COLLISION_OTHER, FailureKind.KNOCKED_OVER, FailureKind.NOT_IN_BOX):
        assert FAILURE_GROUP[kind] is FailureGroup.PLACE


# --- 集計 -----------------------------------------------------------------------------
def test_summary_of_14_successes_and_6_failures():
    records = [_rec(i) for i in range(14)]
    kinds = ["vlm_failure", "planning_failure", "planning_failure", "grasp_slip", "drop", "not_in_box"]
    records += [_rec(14 + i, success=False, failure=k, error_mm=None if k == "vlm_failure" else 8.0) for i, k in enumerate(kinds)]
    s = summarize(records)
    assert (s.n_trials, s.n_success) == (20, 14) and s.success_rate == pytest.approx(0.7)
    assert s.by_failure == {"vlm_failure": 1, "planning_failure": 2, "grasp_slip": 1, "drop": 1, "not_in_box": 1}
    assert s.by_group == {"vlm": 1, "ik": 2, "grasp": 2, "place": 1}
    assert s.mean_pick_error_mm == pytest.approx((14 * 5.0 + 5 * 8.0) / 19)  # 推定がない試行は平均に入れない
    assert s.mean_total_time_s == pytest.approx(7.0)


def test_summary_of_empty_run():
    s = summarize([])
    assert s.n_trials == 0 and s.success_rate is None and s.mean_pick_error_mm is None and s.by_failure == {}


def test_summary_without_any_estimate_has_no_mean_error():
    assert summarize([_rec(error_mm=None)]).mean_pick_error_mm is None


def test_format_summary_shows_rate_and_failure_table():
    text = format_summary(summarize([_rec(0), _rec(1, success=False, failure="planning_failure")]))
    assert "50.0%" in text and "1/2" in text and "planning_failure" in text and "ik" in text


def test_write_summary_json(tmp_path):
    path = write_summary(tmp_path / "s" / "summary.json", summarize([_rec(0), _rec(1, success=False, failure="drop")]))
    data = json.loads(path.read_text(encoding="utf-8"))
    assert data["n_trials"] == 2 and data["success_rate"] == 0.5 and data["by_failure"] == {"drop": 1}


# --- CSV ----------------------------------------------------------------------------
def test_csv_has_the_columns_of_design_4_8():
    assert DESIGN_4_8 <= set(CSV_COLUMNS) and len(set(CSV_COLUMNS)) == len(CSV_COLUMNS)
    assert set(CSV_COLUMNS) == {f.name for f in dataclasses.fields(TrialRecord)}


def test_csv_roundtrip_with_awkward_text(tmp_path):
    instr = 'put the "red", block\ninto the box 赤いブロックを箱へ'
    path = write_csv(tmp_path / "r" / "trials.csv", [_rec(0, instruction=instr), _rec(1, success=False, failure="drop", error_mm=None)])
    with open(path, encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))
    assert list(rows[0].keys()) == list(CSV_COLUMNS) and len(rows) == 2
    assert rows[0]["instruction"] == instr and rows[0]["success"] == "true" and rows[0]["failure"] == ""
    assert rows[1]["success"] == "false" and rows[1]["failure"] == "drop" and rows[1]["failure_group"] == "grasp"
    assert rows[1]["pick_error_mm"] == "" and rows[0]["pick_error_mm"] == "5.0"


def test_csv_with_no_records_still_has_the_header(tmp_path):
    path = write_csv(tmp_path / "t.csv", [])
    assert path.read_text(encoding="utf-8").strip().split(",") == list(CSV_COLUMNS)


# --- 記録の作り方 ------------------------------------------------------------------------
def test_record_from_a_vlm_failure_outcome():
    outcome = TrialOutcome(
        seed=3, colour="blue", instruction="go", plan=None, pick_estimate=None, place_estimate=None,
        pick_error_mm=None, run=None, failure=FailureKind.VLM_FAILURE, error="no purple block", vlm_time_s=0.5,
    )
    r = record_from_outcome(7, outcome, provider="gemini", model="m", total_time_s=1.25)
    assert (r.trial, r.seed, r.colour, r.provider, r.model) == (7, 3, "blue", "gemini", "m")
    assert r.success is False and r.failure == "vlm_failure" and r.failure_group == "vlm"
    assert r.error == "no purple block" and r.pick_error_mm is None and r.pick_bbox == "" and r.plan_time_s is None
    assert r.vlm_time_s == 0.5 and r.total_time_s == 1.25


def test_record_carries_the_detection_and_estimate():
    plan = PlanResult(Detection("red block", (10, 20, 30, 40), 0.9), Detection("box", (1, 2, 3, 4), None))
    outcome = TrialOutcome(
        seed=0, colour="red", instruction="go", plan=plan, pick_estimate=(0.1, 0.2, 0.015),
        place_estimate=(0.2, 0.19, 0.005), pick_error_mm=6.5, run=None, failure=None, error=None, vlm_time_s=0.0,
    )
    r = record_from_outcome(0, outcome, provider="p", model="m", total_time_s=2.0)
    assert r.pick_bbox == "10 20 30 40" and r.place_bbox == "1 2 3 4" and r.pick_label == "red block"
    assert r.pick_xyz == "0.1 0.2 0.015" and r.pick_error_mm == 6.5


# --- 試行列 (実際には回さない) -----------------------------------------------------------------
class _Client:
    provider, model = "fake", "fake-1"


def _fake_trial(log):
    def trial(seed, colour, instruction, vlm, **kwargs):
        log.append((seed, colour, instruction, vlm, kwargs))
        return TrialOutcome(
            seed=seed, colour=colour, instruction=instruction, plan=None, pick_estimate=None, place_estimate=None,
            pick_error_mm=None, run=None, failure=FailureKind.VLM_FAILURE, error="x", vlm_time_s=0.0,
        )

    return trial


def test_trials_use_consecutive_seeds_and_cycle_colours():
    log, seen = [], []
    records = run_trials(
        5, 10, lambda seed, colour: _Client(), trial_fn=_fake_trial(log), on_record=lambda i, n, r: seen.append((i, n)),
    )
    assert [(s, c) for s, c, *_ in log] == [(10, "red"), (11, "blue"), (12, "green"), (13, "red"), (14, "blue")]
    assert [r.trial for r in records] == [0, 1, 2, 3, 4] and [r.seed for r in records] == [10, 11, 12, 13, 14]
    assert seen == [(1, 5), (2, 5), (3, 5), (4, 5), (5, 5)]
    assert {(r.provider, r.model) for r in records} == {("fake", "fake-1")}
    assert all(r.total_time_s >= 0 for r in records)


def test_instruction_names_the_colour_and_template_cannot_break_out():
    log = []
    run_trials(1, 0, lambda s, c: _Client(), colours=("blue",), instruction_template="move {colour} {x} now", trial_fn=_fake_trial(log))
    assert log[0][2] == "move blue {x} now"  # str.format ではなく置換: 他の波括弧は触らない


def test_extra_keyword_arguments_are_passed_to_each_trial():
    log = []
    run_trials(2, 0, lambda s, c: _Client(), trial_fn=_fake_trial(log), calibration="CAL")
    assert all(entry[4] == {"calibration": "CAL"} for entry in log)


def test_unexpected_errors_stop_the_run_but_earlier_records_were_reported():
    calls, seen = [], []

    def trial(seed, colour, instruction, vlm, **kw):
        calls.append(seed)
        if seed == 2:
            raise RuntimeError("401 unauthorized")
        return _fake_trial([])(seed, colour, instruction, vlm)

    with pytest.raises(RuntimeError, match="401"):
        run_trials(5, 0, lambda s, c: _Client(), trial_fn=trial, on_record=lambda i, n, r: seen.append(i))
    assert calls == [0, 1, 2] and seen == [1, 2]


def test_plan_errors_become_failures_not_crashes():
    def trial(seed, colour, instruction, vlm, **kw):
        raise PlanError("should be handled inside run_trial, but the runner must not hide real bugs")

    with pytest.raises(PlanError):
        run_trials(1, 0, lambda s, c: _Client(), trial_fn=trial)


@pytest.mark.parametrize("kwargs", [{"n_trials": 0}, {"n_trials": -1}, {"base_seed": -1}, {"colours": ()}, {"colours": ("purple",)}, {"instruction_template": "no placeholder"}])
def test_bad_arguments_are_value_error(kwargs):
    args = {"n_trials": 1, "base_seed": 0, "colours": ("red",), "instruction_template": "go {colour}"}
    args.update(kwargs)
    with pytest.raises(ValueError):
        run_trials(args.pop("n_trials"), args.pop("base_seed"), lambda s, c: _Client(), trial_fn=_fake_trial([]), **args)


# --- 統合: 偽 VLM で実際に回す ------------------------------------------------------------------
@pytest.fixture(scope="module")
def real_run(tmp_path_factory):
    out = tmp_path_factory.mktemp("eval")
    seen = []
    records = run_trials(4, 0, oracle_client_factory(), on_record=lambda i, n, r: seen.append((i, n, r.success)))
    return records, seen, out


@pytest.mark.render
def test_real_run_produces_one_record_per_trial_and_a_rate(real_run):
    records, seen, _ = real_run
    assert len(records) == 4 and [r.seed for r in records] == [0, 1, 2, 3]
    assert [r.colour for r in records] == ["red", "blue", "green", "red"]
    assert [s[:2] for s in seen] == [(1, 4), (2, 4), (3, 4), (4, 4)]
    s = summarize(records)
    assert s.n_trials == 4 and s.success_rate == s.n_success / 4
    assert s.n_success >= 1  # 偽 VLM + 固定位置で成功率が出る (seed 2 緑は成功する)
    assert all(r.provider == "oracle" and r.pick_error_mm is not None for r in records)
    assert sum(s.by_failure.values()) == 4 - s.n_success


@pytest.mark.render
def test_real_run_csv_and_summary_files(real_run, tmp_path):
    records, _, _ = real_run
    csv_path = write_csv(tmp_path / "trials.csv", records)
    with open(csv_path, encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))
    assert [int(r["seed"]) for r in rows] == [0, 1, 2, 3]
    assert all(float(r["total_time_s"]) > 1 for r in rows)  # 実際にシミュレーションを回した時間

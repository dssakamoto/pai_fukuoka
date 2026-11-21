"""GroundTruth (真値の唯一の窓口) と、箱を動かした試行への追従のテスト (SGT-007)。"""

import mujoco
import numpy as np
import pytest

from so101_vlm.eval.fixed_pick import run_fixed_pick
from so101_vlm.eval.geometry import PlaceRegion, get_geometry
from so101_vlm.eval.ground_truth import GroundTruth
from so101_vlm.eval.layout import sample_layout
from so101_vlm.eval.oracle_vlm import OracleVlm
from so101_vlm.eval.scenario import Fixed, ScenarioSpec, apply_scenario, sample_scenario
from so101_vlm.eval.trial import estimate_goal, run_trial
from so101_vlm.perception.calibration import load_calibration
from so101_vlm.perception.projection import project_base_to_pixel
from so101_vlm.robot.model_paths import PICKPLACE_SCENE_PATH
from so101_vlm.vlm.schema import Detection, PlanResult

G = get_geometry()
CAL = load_calibration()
MOVED = ScenarioSpec(box=Fixed(xy=(0.25, 0.15)))
IMG = np.zeros((480, 640, 3), np.uint8)


def test_layout_and_region_come_from_the_scenario():
    s = sample_scenario(2, MOVED)
    gt = GroundTruth(s, G)
    assert gt.layout == s.blocks == sample_layout(2)
    assert isinstance(gt.place_region, PlaceRegion) and gt.place_region.center_xy == (0.25, 0.15)


def test_block_xyz_is_the_known_position_with_the_rest_height():
    gt = GroundTruth(sample_scenario(2), G)
    for colour, i in (("red", 0), ("blue", 1), ("green", 2)):
        x, y, z = gt.block_xyz(colour)
        assert (x, y) == gt.layout[i] and z == G.block_rest_z
    with pytest.raises(ValueError):
        gt.block_xyz("purple")


def test_from_sim_reads_the_actual_positions_and_the_moved_box():
    s = sample_scenario(2, MOVED)
    model = mujoco.MjModel.from_xml_path(str(PICKPLACE_SCENE_PATH))
    data = mujoco.MjData(model)
    apply_scenario(model, data, s)
    gt = GroundTruth.from_sim(s, G, model, data)
    assert gt.scenario.box_xy == pytest.approx((0.25, 0.15))
    for i, name in enumerate(G.block_names):
        assert np.allclose(gt.block_positions[i], data.xpos[model.body(name).id])
    assert gt.place_region.center_xy == pytest.approx((0.25, 0.15))


def _centre(det):
    x1, y1, x2, y2 = det.bbox_px
    return (x1 + x2) / 2, (y1 + y2) / 2


def test_oracle_place_box_follows_the_moved_region():
    s = sample_scenario(0, MOVED)
    gt = GroundTruth(s, G)
    moved = OracleVlm(gt.layout, "red", CAL, gt.place_region).plan(IMG, "x").place
    default = OracleVlm(gt.layout, "red", CAL).plan(IMG, "x").place
    # 箱の床の中心を画像に投影した点が、新しい枠の内側 (元の位置の枠ではない)
    u, v, _ = project_base_to_pixel(gt.place_region.goal_xyz, CAL.T_base_cam, CAL.camera)
    x1, y1, x2, y2 = moved.bbox_px
    assert x1 <= u <= x2 and y1 <= v <= y2
    assert moved.bbox_px != default.bbox_px


def test_oracle_default_region_is_unchanged():
    gt = GroundTruth(sample_scenario(0), G)
    assert OracleVlm(gt.layout, "red", CAL, gt.place_region).plan(IMG, "x") == OracleVlm(gt.layout, "red", CAL).plan(IMG, "x")


def test_oracle_rejects_a_bad_region():
    with pytest.raises(ValueError):
        OracleVlm(sample_layout(0), "red", CAL, place_region="box")


def test_estimate_goal_z_comes_from_the_geometry():
    plan = PlanResult(pick=Detection("red block", (300, 200, 330, 230), 1.0), place=Detection("box", (350, 200, 420, 270), 1.0))
    depth = np.full((480, 640), 0.5, np.float32)
    goal = estimate_goal(plan, depth, CAL, "x")
    assert goal.place_target_base[2] == G.box_floor_top_z
    import dataclasses

    other = dataclasses.replace(G, box_floor_top_z=G.box_floor_top_z + 0.01, block_half_size=G.block_half_size + 0.002)
    shifted = estimate_goal(plan, depth, CAL, "x", other)
    assert shifted.place_target_base[2] == pytest.approx(G.box_floor_top_z + 0.01)
    assert shifted.pick_target_base[2] == pytest.approx(goal.pick_target_base[2] - 0.002)


@pytest.mark.parametrize("seed", range(5))
def test_fixed_pick_default_is_unchanged_by_the_scenario_path(seed):
    """Fixed の既定: scenario を渡さなくても、既定 Scenario を渡しても同じ結果 (従来どおり) 🔵"""
    a = run_fixed_pick(seed, "red")
    b = run_fixed_pick(seed, "red", scenario=sample_scenario(seed))
    assert (a.success, a.failure, a.pick_target) == (b.success, b.failure, b.pick_target)
    assert a.judge.offset_xy == b.judge.offset_xy


def test_fixed_pick_goal_uses_the_moved_region():
    """目標の xy は、動かした箱の底の中心 (真のブロック位置は変わらない) 🔵"""
    from so101_vlm.eval.fixed_pick import _place_goal

    s = sample_scenario(0, MOVED)
    model = mujoco.MjModel.from_xml_path(str(PICKPLACE_SCENE_PATH))
    data = mujoco.MjData(model)
    apply_scenario(model, data, s)
    goal = _place_goal(data, model, G.block_names[0], GroundTruth(s, G).place_region)
    assert tuple(goal.place_target_base) == pytest.approx((0.25, 0.15, G.box_floor_top_z))
    assert tuple(goal.pick_target_base[:2]) == pytest.approx(s.blocks[0])


def test_fixed_pick_runs_on_a_moved_box_and_judges_against_it():
    """箱を動かしても 1 試行が最後まで通る (到達できずに計画失敗しても、例外にならず分類される) 🟡"""
    out = run_fixed_pick(0, "red", scenario=sample_scenario(0, MOVED))
    assert out.judge is not None or out.failure is not None


def test_run_trial_default_scenario_matches_the_explicit_default():
    seed = 2
    gt = GroundTruth(sample_scenario(seed), G)
    vlm = OracleVlm(gt.layout, "red", CAL, gt.place_region)
    a = run_trial(seed, "red", "put the red block into the box", vlm)
    b = run_trial(seed, "red", "put the red block into the box", vlm, scenario=sample_scenario(seed))
    assert a.pick_error_mm == b.pick_error_mm and a.success == b.success


def test_run_trial_rejects_a_bad_scenario_before_starting():
    with pytest.raises(ValueError):
        run_trial(0, "red", "x", object(), scenario="s")  # type: ignore[arg-type]

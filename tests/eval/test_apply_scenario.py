"""apply_scenario のテスト (SGT-006)。箱の body_pos を書き換えるので、テストごとに専用のモデルを作る。"""

import mujoco
import numpy as np
import pytest

from so101_vlm.eval.geometry import SceneGeometry
from so101_vlm.eval.layout import place_blocks, sample_layout
from so101_vlm.eval.scenario import Fixed, Scenario, ScenarioSpec, apply_scenario, sample_scenario
from so101_vlm.robot.model_paths import PICKPLACE_SCENE_PATH


def _fresh():
    model = mujoco.MjModel.from_xml_path(str(PICKPLACE_SCENE_PATH))
    return model, mujoco.MjData(model)


def _floor_xy(model, data):
    return tuple(float(v) for v in data.geom_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "box_floor")][:2])


def test_box_floor_follows_the_scenario():
    model, data = _fresh()
    apply_scenario(model, data, sample_scenario(0, ScenarioSpec(box=Fixed(xy=(0.25, 0.15)))))
    assert _floor_xy(model, data) == pytest.approx((0.25, 0.15), abs=1e-9)


def test_blocks_are_placed_like_place_blocks_even_when_the_box_moves():
    model, data = _fresh()
    scenario = sample_scenario(2, ScenarioSpec(box=Fixed(xy=(0.25, 0.15))))
    apply_scenario(model, data, scenario)
    ref_model, ref_data = _fresh()
    place_blocks(ref_model, ref_data, sample_layout(2))
    for name in SceneGeometry.from_model(model).block_names:
        i = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        assert np.allclose(data.xpos[i], ref_data.xpos[i])


def test_default_scenario_keeps_the_xml_box_position():
    model, data = _fresh()
    ref_model, ref_data = _fresh()
    mujoco.mj_forward(ref_model, ref_data)
    apply_scenario(model, data, sample_scenario(0))
    assert _floor_xy(model, data) == pytest.approx(_floor_xy(ref_model, ref_data), abs=1e-12)


def test_a_new_model_does_not_inherit_the_moved_box():
    model, data = _fresh()
    apply_scenario(model, data, sample_scenario(0, ScenarioSpec(box=Fixed(xy=(0.25, 0.15)))))
    fresh_model, fresh_data = _fresh()
    mujoco.mj_forward(fresh_model, fresh_data)
    assert _floor_xy(fresh_model, fresh_data) == pytest.approx((0.20, 0.19), abs=1e-9)


def test_invalid_scenario_raises_value_error_without_touching_the_data():
    model, data = _fresh()
    before = data.qpos.copy()
    with pytest.raises(ValueError):
        apply_scenario(model, data, "scenario")  # type: ignore[arg-type]
    assert np.array_equal(data.qpos, before)

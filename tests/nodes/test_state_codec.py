"""pack_world / apply_world のテスト (SGT-007)。箱の位置を world に載せて、camera / evaluator が箱を動かした試行を再現できること。"""

import mujoco
import numpy as np
import pytest

from so101_vlm.eval.scenario import Fixed, ScenarioSpec, apply_scenario, sample_scenario
from so101_vlm.nodes.state_codec import apply_world, pack_world
from so101_vlm.robot.model_paths import PICKPLACE_SCENE_PATH


def _fresh():
    model = mujoco.MjModel.from_xml_path(str(PICKPLACE_SCENE_PATH))
    return model, mujoco.MjData(model)


def _floor_xy(model, data):
    return tuple(float(v) for v in data.geom_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "box_floor")][:2])


def test_pack_world_appends_the_box_xy():
    model, data = _fresh()
    apply_scenario(model, data, sample_scenario(0, ScenarioSpec(box=Fixed(xy=(0.25, 0.15)))))
    world = pack_world(model, data)
    assert world.shape == (model.nq + model.nv + 2,) and world.dtype == np.float64
    assert tuple(world[-2:]) == pytest.approx((0.25, 0.15))


def test_apply_world_reproduces_a_moved_box_in_another_model():
    src, src_data = _fresh()
    apply_scenario(src, src_data, sample_scenario(1, ScenarioSpec(box=Fixed(xy=(0.25, 0.15)))))
    dst, dst_data = _fresh()  # 別のプロセス (camera / evaluator) のモデル: 箱は元の位置
    apply_world(dst, dst_data, pack_world(src, src_data))
    assert _floor_xy(dst, dst_data) == pytest.approx((0.25, 0.15), abs=1e-9)
    assert np.allclose(dst_data.qpos, src_data.qpos)


def test_apply_world_accepts_the_legacy_shape_without_moving_the_box():
    model, data = _fresh()
    legacy = np.concatenate([data.qpos, data.qvel])
    apply_world(model, data, legacy)
    assert _floor_xy(model, data) == pytest.approx((0.20, 0.19), abs=1e-9)


@pytest.mark.parametrize("extra", [-1, 1, 3])
def test_apply_world_rejects_other_shapes(extra):
    model, data = _fresh()
    with pytest.raises(ValueError):
        apply_world(model, data, np.zeros(model.nq + model.nv + extra))

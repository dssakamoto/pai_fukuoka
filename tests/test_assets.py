import mujoco
import pytest

from so101_vlm.robot.model_paths import JOINT_NAMES, SITE_NAMES, SO101_SCENE_PATH


@pytest.fixture(scope="module")
def model():
    return mujoco.MjModel.from_xml_path(str(SO101_SCENE_PATH))


def _names(model, obj, count):
    return tuple(mujoco.mj_id2name(model, obj, i) for i in range(count))


def test_scene_loads(model):
    assert model.nbody > 1


def test_joint_names_match_design(model):
    assert _names(model, mujoco.mjtObj.mjOBJ_JOINT, model.njnt) == JOINT_NAMES


def test_actuator_names_match_joints(model):
    assert _names(model, mujoco.mjtObj.mjOBJ_ACTUATOR, model.nu) == JOINT_NAMES


@pytest.mark.parametrize("site", SITE_NAMES)
def test_site_exists(model, site):
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site) >= 0


@pytest.mark.parametrize("name", JOINT_NAMES)
def test_ctrlrange_matches_joint_range(model, name):
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    assert model.actuator_ctrllimited[aid]
    assert model.actuator_ctrlrange[aid] == pytest.approx(model.jnt_range[jid], abs=1e-3)

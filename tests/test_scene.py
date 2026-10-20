"""ピック&プレースシーンのテスト (TASK-0005)。"""

import mujoco
import numpy as np
import pytest

from so101_vlm.eval.scene_spec import SCENE_SPEC as SPEC
from so101_vlm.nodes.camera import render_rgbd
from so101_vlm.robot.model_paths import PICKPLACE_SCENE_PATH
from so101_vlm.robot.sim import SimRobot

SETTLE_STEPS = 500  # 1 s (timestep 2 ms)
REST_TOL = 0.002  # 静止時の高さの許容誤差 2 mm


@pytest.fixture(scope="module")
def settled():
    model = mujoco.MjModel.from_xml_path(str(PICKPLACE_SCENE_PATH))
    data = mujoco.MjData(model)
    for _ in range(SETTLE_STEPS):
        mujoco.mj_step(model, data)
    return model, data


@pytest.fixture(scope="module")
def rgbd(settled):
    model, data = settled
    return render_rgbd(model, data, SPEC.camera_name, SPEC.image_width, SPEC.image_height)


def _body_id(model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)


@pytest.mark.parametrize("name", [*SPEC.block_names, SPEC.box_body_name])
def test_body_exists(settled, name):
    assert _body_id(settled[0], name) >= 0


def test_camera_exists(settled):
    assert mujoco.mj_name2id(settled[0], mujoco.mjtObj.mjOBJ_CAMERA, SPEC.camera_name) >= 0


def test_rgb_and_depth_shape_and_dtype(rgbd):
    rgb, depth = rgbd
    assert rgb.shape == (480, 640, 3) and rgb.dtype == np.uint8
    assert depth.shape == (480, 640) and depth.dtype == np.float32


def test_depth_is_finite_and_positive(rgbd):
    depth = rgbd[1]
    assert np.isfinite(depth).all() and (depth > 0).all()


def test_render_rgbd_unknown_camera_raises(settled):
    with pytest.raises(ValueError, match="no_such_cam"):
        render_rgbd(*settled, "no_such_cam")


@pytest.mark.parametrize("name", SPEC.block_names)
def test_block_rests_on_floor(settled, name):
    model, data = settled
    z = data.xpos[_body_id(model, name)][2]
    assert z == pytest.approx(SPEC.block_rest_z, abs=REST_TOL)


@pytest.mark.parametrize("name", SPEC.block_names)
def test_block_initial_position_in_work_area(settled, name):
    x, y = settled[1].xpos[_body_id(settled[0], name)][:2]
    assert SPEC.work_area_x[0] <= x <= SPEC.work_area_x[1]
    assert SPEC.work_area_y[0] <= y <= SPEC.work_area_y[1]


def test_blocks_are_at_least_5cm_apart(settled):
    model, data = settled
    pos = [data.xpos[_body_id(model, n)][:2] for n in SPEC.block_names]
    for i in range(3):
        for j in range(i + 1, 3):
            assert np.linalg.norm(pos[i] - pos[j]) >= 0.05


def _project(model, data, point):
    cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, SPEC.camera_name)
    c = data.cam_xmat[cid].reshape(3, 3).T @ (point - data.cam_xpos[cid])
    f = (SPEC.image_height / 2) / np.tan(np.radians(model.cam_fovy[cid] / 2))
    z = -c[2]
    return SPEC.image_width / 2 + f * c[0] / z, SPEC.image_height / 2 - f * c[1] / z, z


@pytest.mark.parametrize("name", SPEC.block_names)
def test_block_visible_and_depth_matches(settled, rgbd, name):
    model, data = settled
    top = data.xpos[_body_id(model, name)].copy()
    top[2] += SPEC.block_half_size
    u, v, z = _project(model, data, top)
    ui, vi = round(u), round(v)
    assert 0 <= ui < SPEC.image_width and 0 <= vi < SPEC.image_height
    assert rgbd[1][vi, ui] == pytest.approx(z, abs=0.005)


def test_box_visible_and_depth_matches(settled, rgbd):
    model, data = settled
    p = np.array([*SPEC.box_center_xy, SPEC.box_floor_top_z])
    u, v, z = _project(model, data, p)
    ui, vi = round(u), round(v)
    assert 0 <= ui < SPEC.image_width and 0 <= vi < SPEC.image_height
    assert rgbd[1][vi, ui] == pytest.approx(z, abs=0.005)


def test_box_geometry_matches_spec(settled):
    model, data = settled
    bid = _body_id(model, SPEC.box_body_name)
    assert data.xpos[bid][:2] == pytest.approx(SPEC.box_center_xy)
    geoms = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g): g
        for g in range(model.ngeom)
        if model.geom_bodyid[g] == bid
    }
    t = SPEC.box_wall_thickness
    wall = geoms["box_wall_px"]
    # 壁の内面 = 中心 + 内側半寸、上端 = 縁の高さ
    inner_face = data.geom_xpos[wall][0] - SPEC.box_center_xy[0] - model.geom_size[wall][0]
    assert inner_face == pytest.approx(SPEC.box_inner_half_size)
    assert data.geom_xpos[wall][2] + model.geom_size[wall][2] == pytest.approx(SPEC.box_rim_z)
    assert 2 * model.geom_size[wall][0] == pytest.approx(t)
    plate = geoms["box_floor"]
    assert data.geom_xpos[plate][2] + model.geom_size[plate][2] == pytest.approx(SPEC.box_floor_top_z)
    assert model.geom_size[plate][0] == pytest.approx(SPEC.box_inner_half_size + t)


def test_block_geometry_matches_spec(settled):
    model, _ = settled
    for n in SPEC.block_names:
        g = model.body_geomadr[_body_id(model, n)]
        assert model.geom_size[g] == pytest.approx([SPEC.block_half_size] * 3)


def test_simrobot_works_on_pickplace_scene_without_disturbing_blocks():
    with SimRobot(scene_path=PICKPLACE_SCENE_PATH) as robot:
        q0 = robot.get_state().q.copy()
        assert q0.shape == (6,)
        for _ in range(20):
            robot.send_command(q0)
        s = robot.get_state()
        assert s.q == pytest.approx(q0, abs=0.05)
        model, data = robot._model, robot._data
        for n in SPEC.block_names:
            assert data.xpos[_body_id(model, n)][2] == pytest.approx(SPEC.block_rest_z, abs=REST_TOL)

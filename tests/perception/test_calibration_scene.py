"""calibration の MuJoCo 統合テスト (TASK-0009, Red フェーズ。描画あり)。

テストケース定義: docs/implements/so101-vlm-pickplace/TASK-0009/calibration-testcases.md (I-01..I-04)
コミット済みの configs/calibration.yaml (load_calibration) を使い、描画深度から真の位置を復元できるか確かめる。
しきい値は 2 mm (OPEN-4 の決定。タスクの完了条件 5 mm は文書で扱う)。
輪郭全体の bbox は使わない (側面が混ざり水平に 6.9〜10.2 mm ずれる。TASK-0008 S-09)。
信頼性: 🔵 要件・設計・実測に基づく / 🟡 妥当な推測 / 🔴 要件にない決定
"""

import mujoco
import numpy as np
import pytest

from so101_vlm.eval.layout import place_blocks, sample_layout
from so101_vlm.eval.scene_spec import SCENE_SPEC
from so101_vlm.nodes.camera import render_rgbd
from so101_vlm.perception.calibration import load_calibration
from so101_vlm.perception.projection import (
    bbox_to_base_point,
    camera_to_base,
    pixel_to_camera,
    project_base_to_pixel,
)
from so101_vlm.robot.model_paths import PICKPLACE_SCENE_PATH

SETTLE_STEPS = 500  # 1 s (timestep 2 ms)。test_projection_scene.py と同じ
SCENE_TOL_M = 2e-3  # OPEN-4 の決定 (実測最大 1.235 mm / 150 個)
FLOOR_TOL_M = 1e-4  # TASK-0008 S-02 と同じ (実測 0.0076 mm)
SEEDS = range(10)  # OPEN-2 の決定
COLORS = ("red", "blue", "green")
GRID_STEP = 8
# 既定配置の上面 bbox の回帰値 (要件 9.4)。作り方が変わったことの検出用
DEFAULT_BBOX = {"red": (251, 143, 278, 165), "blue": (227, 202, 257, 228), "green": (300, 197, 330, 222)}


@pytest.fixture(scope="module")
def cal():
    c = load_calibration()
    if c.source == "real":
        pytest.skip("configs/calibration.yaml は実機の計測値 (source: real) のため sim の描画と比べない")
    return c


@pytest.fixture(scope="module")
def model():
    return mujoco.MjModel.from_xml_path(str(PICKPLACE_SCENE_PATH))


def _settle(model, data):
    mujoco.mj_forward(model, data)
    for _ in range(SETTLE_STEPS):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)


@pytest.fixture(scope="module")
def default_data(model):
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    _settle(model, data)
    return data


@pytest.fixture(scope="module")
def default_depth(model, default_data, cal):
    return render_rgbd(model, default_data, SCENE_SPEC.camera_name, cal.camera.width, cal.camera.height)[1]


@pytest.fixture(scope="module")
def segmentation(model, default_data, cal):
    """画素ごとの geom ID (H, W)。背景は -1。"""
    r = mujoco.Renderer(model, height=cal.camera.height, width=cal.camera.width)
    try:
        r.update_scene(default_data, camera=SCENE_SPEC.camera_name)
        r.enable_segmentation_rendering()
        return r.render()[..., 0].copy()
    finally:
        r.close()


def _gid(model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)


def _top_face(model, data, color, camera, T):
    """静定後の上面中心 (geom_xpos + R @ [0,0,h]) と、上面 4 隅を投影した半開区間の bbox。"""
    gid = _gid(model, f"block_{color}_geom")
    c, R, h = data.geom_xpos[gid], data.geom_xmat[gid].reshape(3, 3), SCENE_SPEC.block_half_size
    corners = [
        project_base_to_pixel(c + R @ [sx * h, sy * h, h], T, camera)[:2] for sx in (-1, 1) for sy in (-1, 1)
    ]
    x1, y1 = (round(v) for v in np.min(corners, 0))
    x2, y2 = (round(v) + 1 for v in np.max(corners, 0))
    return c + R @ [0, 0, h], (int(x1), int(y1), int(x2), int(y2))


@pytest.mark.parametrize("color", COLORS)
def test_default_layout_block_top_recovered_within_2mm(model, default_data, default_depth, cal, color):
    """I-01: 既定配置の 3 個を上面 bbox + shrink=0.5 で 2 mm 以内に復元する。🟡"""
    # 【テスト目的】: YAML 往復後の K と T_base_cam で、描画深度から真の上面中心を復元できる
    # Given: 静定後の既定配置と描画深度 (実測 red 0.761 / blue 0.142 / green 0.615 mm)
    top, bbox = _top_face(model, default_data, color, cal.camera, cal.T_base_cam)
    # When
    p = bbox_to_base_point(bbox, default_depth, cal.camera, cal.T_base_cam, shrink=0.5)
    # Then
    assert bbox == DEFAULT_BBOX[color]  # 【確認内容】: 上面 bbox の回帰値
    assert np.linalg.norm(p - top) <= SCENE_TOL_M  # 【確認内容】: 2 mm 以内


@pytest.mark.parametrize("seed", SEEDS)
def test_random_layout_block_tops_recovered_within_2mm(model, cal, seed):
    """I-02: ランダム配置 seed 0〜9 の 30 個を 2 mm 以内に復元する。🟡 (実測最大 1.153 mm, seed 9 green)"""
    # Given: seed ごとに MjData を作り直し、ブロックを配置して静定する
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    place_blocks(model, data, sample_layout(seed))
    _settle(model, data)
    depth = render_rgbd(model, data, SCENE_SPEC.camera_name, cal.camera.width, cal.camera.height)[1]
    for color in COLORS:
        top, bbox = _top_face(model, data, color, cal.camera, cal.T_base_cam)
        # When
        p = bbox_to_base_point(bbox, depth, cal.camera, cal.T_base_cam, shrink=0.5)
        # Then
        err_mm = 1e3 * float(np.linalg.norm(p - top))
        assert err_mm <= 1e3 * SCENE_TOL_M, f"seed={seed} color={color} err={err_mm:.3f} mm bbox={bbox}"


def test_floor_backprojection_is_flat_with_committed_calibration(model, default_depth, segmentation, cal):
    """I-03: 床の逆投影が平ら (|z| <= 0.1 mm。主点規約・符号の回帰)。🔵"""
    # Given: 8 px 間隔の格子のうち床の画素
    vv, uu = np.mgrid[0 : cal.camera.height : GRID_STEP, 0 : cal.camera.width : GRID_STEP]
    mask = segmentation[vv, uu] == _gid(model, "floor")
    u, v = uu[mask], vv[mask]
    assert len(u) > 1000  # 前提
    # When
    P = np.array(
        [camera_to_base(pixel_to_camera(a, b, c, cal.camera), cal.T_base_cam) for a, b, c in zip(u, v, default_depth[v, u])]
    )
    # Then
    assert np.abs(P[:, 2]).max() <= FLOOR_TOL_M  # 【確認内容】: 実測 0.0076 mm


def test_block_center_correction_from_top_point(model, default_data, default_depth, cal):
    """I-04: 上面の点から半辺を引くとブロック中心になる (要件 4.1 の例示)。🟡"""
    top, bbox = _top_face(model, default_data, "red", cal.camera, cal.T_base_cam)
    p_top = bbox_to_base_point(bbox, default_depth, cal.camera, cal.T_base_cam, shrink=0.5)
    p_center = p_top - [0.0, 0.0, SCENE_SPEC.block_half_size]
    center = default_data.geom_xpos[_gid(model, "block_red_geom")]
    assert np.linalg.norm(p_center - center) <= SCENE_TOL_M  # 【確認内容】: 2 mm 以内

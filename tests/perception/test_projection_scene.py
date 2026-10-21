"""projection の MuJoCo 統合テスト (TASK-0008, Red フェーズ)。

テストケース定義: docs/implements/so101-vlm-pickplace/TASK-0008/projection-testcases.md (S-01..S-09)
実際のシーン (assets/scene_pickplace.xml) を描画し、整数画素と描画深度で 3 次元位置を復元できるか確かめる。
K と T_base_cam は製品コード calibration_from_mujoco (TASK-0009) で MuJoCo から計算する。
信頼性: 🔵 要件・設計・実測に基づく / 🟡 妥当な推測 / 🔴 要件にない仮定
"""

import math

import mujoco
import numpy as np
import pytest

from so101_vlm.eval.scene_spec import SCENE_SPEC as SPEC
from so101_vlm.nodes.camera import render_rgbd
from so101_vlm.perception.calibration import calibration_from_mujoco
from so101_vlm.perception.projection import (
    Camera,
    bbox_to_base_point,
    camera_to_base,
    pixel_to_camera,
    project_base_to_pixel,
    validate_T_base_cam,
)
from so101_vlm.robot.model_paths import PICKPLACE_SCENE_PATH

SETTLE_STEPS = 500  # 1 s (timestep 2 ms)。tests/test_scene.py と同じ
W, H = SPEC.image_width, SPEC.image_height
SCENE_TOL_M = 2e-3  # 整数画素の量子化 (実測最大 1.34 mm)
FLOOR_TOL_M = 1e-4  # 主点規約の回帰 (規約A 0.0076 mm / 規約B 0.50 mm)
EUCLID_MIN_ERR_M = 0.1  # z 深度とユークリッド距離の判別 (実測 184〜345 mm)
SILHOUETTE_BIAS_RANGE_M = (0.005, 0.015)  # 特性記録用 (実測 6.9〜10.2 mm)
COLORS = ("red", "blue", "green")
GRID_STEP = 8


# ---------------------------------------------------------------- フィクスチャ・ヘルパ
@pytest.fixture(scope="module")
def settled():
    model = mujoco.MjModel.from_xml_path(str(PICKPLACE_SCENE_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    for _ in range(SETTLE_STEPS):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)
    return model, data


@pytest.fixture(scope="module")
def depth(settled):
    model, data = settled
    return render_rgbd(model, data, SPEC.camera_name, W, H)[1]


@pytest.fixture(scope="module")
def segmentation(settled):
    """画素ごとの geom ID (H, W)。背景は -1。"""
    model, data = settled
    renderer = mujoco.Renderer(model, height=H, width=W)
    try:
        renderer.update_scene(data, camera=SPEC.camera_name)
        renderer.enable_segmentation_rendering()
        return renderer.render()[..., 0].copy()
    finally:
        renderer.close()


@pytest.fixture(scope="module")
def calib(settled):
    """製品コード calibration_from_mujoco が作る (Camera, T_base_cam[OpenCV 規約])。"""
    cal = calibration_from_mujoco(*settled, SPEC.camera_name, W, H)
    return cal.camera, cal.T_base_cam


def _geom_id(model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)


def _top_center(settled, color):
    """静定後のブロック上面中心 (XML の値ではなく静定後の geom_xpos を使う)。"""
    model, data = settled
    return data.geom_xpos[_geom_id(model, f"block_{color}_geom")] + [0.0, 0.0, SPEC.block_half_size]


def _round_pixel(p_base, T, camera):
    u, v, _ = project_base_to_pixel(p_base, T, camera)
    return int(round(u)), int(round(v))


def _floor_grid(segmentation, settled):
    """8 px 間隔の格子のうち床の画素 (u, v) (int 配列)。"""
    fid = _geom_id(settled[0], "floor")
    vv, uu = np.mgrid[0:H:GRID_STEP, 0:W:GRID_STEP]
    mask = segmentation[vv, uu] == fid
    return uu[mask], vv[mask]


def _backproject(camera, T, u, v, z):
    return np.array([camera_to_base(pixel_to_camera(a, b, c, camera), T) for a, b, c in zip(u, v, z)])


# ---------------------------------------------------------------- S-01..S-03
def test_helper_camera_and_T_match_scene(settled, calib):
    """S-01: calibration_from_mujoco の K と T がシーンと一致する。🟡"""
    camera, T = calib
    fy = (H / 2) / math.tan(math.radians(60) / 2)
    for got, want in ((camera.fx, fy), (camera.fy, fy)):
        assert got == pytest.approx(want, abs=1e-9)
    assert (camera.cx, camera.cy, camera.width, camera.height) == (319.5, 239.5, 640, 480)
    validate_T_base_cam(T)
    np.testing.assert_allclose(T[:3, 3], [0.45, 0.30, 0.55], atol=1e-9)
    model, data = settled
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "baseframe")
    np.testing.assert_allclose(data.site_xpos[sid], 0.0, atol=1e-9)  # base = ワールド原点
    np.testing.assert_allclose(data.site_xmat[sid].reshape(3, 3), np.eye(3), atol=1e-9)


def test_floor_backprojection_is_flat(settled, depth, segmentation, calib):
    """S-02: 床の画素を逆投影すると |z| <= 0.1 mm (主点規約 (w-1)/2・z 深度・符号の回帰)。🟡"""
    camera, T = calib
    # Given: 8 px 間隔の格子のうち床の画素 (整数画素と描画深度)
    u, v = _floor_grid(segmentation, settled)
    assert len(u) > 1000
    # When
    P = _backproject(camera, T, u, v, depth[v, u])
    # Then: 実測 0.0076 mm
    assert np.abs(P[:, 2]).max() <= FLOOR_TOL_M


def test_floor_backprojection_with_wrong_principal_point_fails(settled, depth, segmentation, calib):
    """S-03: 負の対照。cx=w/2 規約なら S-02 の許容誤差を超える。🟡

    このテストが落ちたら S-02 の許容誤差が甘すぎる (規約A 0.0076 mm / 規約B 0.50 mm)。
    """
    camera, T = calib
    wrong = Camera(fx=camera.fx, fy=camera.fy, cx=320.0, cy=240.0, width=W, height=H)
    u, v = _floor_grid(segmentation, settled)
    P = _backproject(wrong, T, u, v, depth[v, u])
    assert np.abs(P[:, 2]).max() > FLOOR_TOL_M


# ---------------------------------------------------------------- S-04..S-07
@pytest.mark.parametrize("color", COLORS)
def test_block_top_center_from_single_pixel_bbox(settled, depth, calib, color):
    """S-04: 上面中心の画素を 1x1 の bbox にすると真値から 2 mm 以内。🟡"""
    camera, T = calib
    top = _top_center(settled, color)
    ui, vi = _round_pixel(top, T, camera)
    p = bbox_to_base_point((ui, vi, ui + 1, vi + 1), depth, camera, T)
    assert np.linalg.norm(p - top) <= SCENE_TOL_M


def _top_face_bbox(top, T, camera):
    """上面の 4 隅を投影した半開区間の bbox (x2, y2 は含まない端)。"""
    h = SPEC.block_half_size
    corners = np.array(
        [project_base_to_pixel(top + [sx * h, sy * h, 0.0], T, camera)[:2] for sx in (-1, 1) for sy in (-1, 1)]
    )
    x1, y1 = (round(c) for c in corners.min(axis=0))
    x2, y2 = (round(c) + 1 for c in corners.max(axis=0))
    return int(x1), int(y1), int(x2), int(y2)


@pytest.mark.parametrize("color", COLORS)
def test_block_top_face_bbox_within_2mm(settled, depth, calib, color):
    """S-05: bbox が上面を正しく囲めば、shrink=0.5 で上面中心から 2 mm 以内。🟡 (TASK-0024 の前提)"""
    camera, T = calib
    top = _top_center(settled, color)
    bbox = _top_face_bbox(top, T, camera)  # 実測 red (251,143,278,165)
    p = bbox_to_base_point(bbox, depth, camera, T, shrink=0.5)
    assert np.linalg.norm(p - top) <= SCENE_TOL_M


def test_box_floor_center(settled, depth, segmentation, calib):
    """S-06: 箱の底板の中心 (0.20, 0.19, 0.005) も 2 mm 以内で復元できる。🟡"""
    camera, T = calib
    model, data = settled
    gid = _geom_id(model, "box_floor")
    center = data.geom_xpos[gid] + [0.0, 0.0, 0.0025]
    ui, vi = _round_pixel(center, T, camera)
    assert segmentation[vi, ui] == gid  # 前提: その画素は底板
    p = bbox_to_base_point((ui, vi, ui + 1, vi + 1), depth, camera, T)
    assert np.linalg.norm(p - center) <= SCENE_TOL_M


def test_surface_to_block_center_correction_subtracts_half_size(settled, depth, calib):
    """S-07: 上面 → ブロック中心の補正は半辺を「引く」(足すと 30 mm ずれる)。呼び出し側の責務の例示。🟡"""
    camera, T = calib
    model, data = settled
    top = _top_center(settled, "red")
    ui, vi = _round_pixel(top, T, camera)
    p_surface = bbox_to_base_point((ui, vi, ui + 1, vi + 1), depth, camera, T)
    center_z = data.geom_xpos[_geom_id(model, "block_red_geom")][2]
    assert abs((p_surface[2] - SPEC.block_half_size) - center_z) <= SCENE_TOL_M
    assert abs((p_surface[2] + SPEC.block_half_size) - center_z) > 0.025


# ---------------------------------------------------------------- S-08
def test_depth_is_z_depth_not_euclidean_distance(settled, depth, segmentation, calib):
    """S-08: 描画深度は z 深度 (光軸方向) であり、ユークリッド距離ではない。🟡"""
    camera, T = calib
    R, t = T[:3, :3], T[:3, 3]
    # (a) 床の点で、描画深度と「光線と平面 z=0 の交点の解析的な z 深度」が 0.1 mm 以内で一致する
    u, v = _floor_grid(segmentation, settled)
    rays = np.stack([(u - camera.cx) / camera.fx, (v - camera.cy) / camera.fy, np.ones(len(u))], -1) @ R.T
    analytic = (0.0 - t[2]) / rays[:, 2]
    assert np.abs(depth[v, u] - analytic).max() <= 1e-4
    # (b) 隅の床画素: 深度をユークリッド距離と解釈すると 100 mm 以上ずれる (実測 184〜345 mm)
    fid = _geom_id(settled[0], "floor")
    for ui, vi in ((0, 0), (639, 0), (0, 479), (639, 479)):
        assert segmentation[vi, ui] == fid  # 前提: 床である
        z = float(depth[vi, ui])
        ray = np.array([(ui - camera.cx) / camera.fx, (vi - camera.cy) / camera.fy, 1.0])
        p_ok = R @ (z * ray) + t
        p_bad = R @ (z * ray / np.linalg.norm(ray)) + t
        assert np.linalg.norm(p_bad - p_ok) > EUCLID_MIN_ERR_M


# ---------------------------------------------------------------- S-09
@pytest.mark.parametrize("color", COLORS)
def test_characterize_silhouette_bbox_bias_for_task0024(settled, depth, segmentation, calib, color):
    """特性記録 (要件ではない): 見えている輪郭全体の bbox 中心は上面中心から水平に 5〜15 mm ずれる。🔴

    S-09。変換自体は正しく、「bbox の中心 = 上面の中心」という仮定の誤差 (側面が映るため)。
    実測 red 10.17 / blue 7.43 / green 6.92 mm、z 誤差は負 (-3.3 / -2.4 / -1.7 mm、側面の深度が混ざる)。
    TASK-0024 への警告として値を記録する。下限があるので偏りが消えたときも落ちる (記録を更新する合図)。
    上限は暴走の検出。S-05 (上面だけの bbox は 2 mm 以内) と対で読む。範囲外れ時は survey と
    TASK-0024 の記録を更新してから範囲を直す (O-6)。
    """
    camera, T = calib
    top = _top_center(settled, color)
    # Given: 該当 geom の全画素の外接 bbox (半開区間)
    ys, xs = np.where(segmentation == _geom_id(settled[0], f"block_{color}_geom"))
    bbox = (int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1)
    # When
    p = bbox_to_base_point(bbox, depth, camera, T, shrink=0.5)
    # Then
    e_xy = float(np.linalg.norm((p - top)[:2]))
    assert SILHOUETTE_BIAS_RANGE_M[0] <= e_xy <= SILHOUETTE_BIAS_RANGE_M[1]
    assert (p - top)[2] < 0

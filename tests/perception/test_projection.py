"""画像座標 → カメラ座標 → ロボット座標 (projection) の単体テスト (TASK-0008, Red フェーズ)。

テストケース定義: docs/implements/so101-vlm-pickplace/TASK-0008/projection-testcases.md (U-01..U-59)
対象: src/so101_vlm/perception/projection.py (MuJoCo 非依存の純粋関数)
信頼性: 🔵 要件・設計・実測に基づく / 🟡 妥当な推測 / 🔴 要件にない仮定
合成データ (既知の T と点、解析的な平面の深度画像) だけで検証する。モックは使わない。
"""

import ast
import copy
import dataclasses
import math
from pathlib import Path

import numpy as np
import pytest

from so101_vlm.perception.projection import (
    Camera,
    bbox_to_base_point,
    camera_matrix,
    camera_to_base,
    mujoco_cam_to_opencv,
    pixel_to_camera,
    project_base_to_pixel,
    representative_depth,
    validate_T_base_cam,
)

W, H = 640, 480
FOVY_DEG = 60.0
FY = (H / 2) / math.tan(math.radians(FOVY_DEG) / 2)  # 415.6921938165307
FX = FY
CX, CY = (W - 1) / 2, (H - 1) / 2  # 画素中心規約: 319.5, 239.5
ROUNDTRIP_TOL_M = 1e-3
EXACT_TOL_M = 1e-9
FLIP = np.diag([1.0, -1.0, -1.0, 1.0])  # MuJoCo <-> OpenCV
CAMERA = Camera(fx=FY, fy=FY, cx=CX, cy=CY, width=W, height=H)
PROJECTION_PATH = Path(__file__).resolve().parents[2] / "src/so101_vlm/perception/projection.py"


# ---------------------------------------------------------------- ヘルパ
def look_at(eye, target, up=(0.0, 0.0, 1.0)):
    """OpenCV 規約 (x 右, y 下, z 前方) のカメラ姿勢 T_base_cam を作る。"""
    eye = np.asarray(eye, float)
    z = np.asarray(target, float) - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, up)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, :3] = np.column_stack([x, y, z])
    T[:3, 3] = eye
    return T


T_OBLIQUE = look_at((0.45, 0.30, 0.55), (0.24, 0.10, 0.0))
T_DOWN = np.eye(4)
T_DOWN[:3, :3] = np.diag([1.0, -1.0, -1.0])
T_DOWN[:3, 3] = (0.25, 0.0, 0.60)
T_MIRROR = np.eye(4)  # 回転部の det = -1
T_MIRROR[:3, :3] = np.diag([1.0, 1.0, -1.0])


def _rays_world(T, u, v):
    """画素 (u, v) の光線 (z=1 に正規化したカメラ座標 → ワールド向き)。被テスト関数を使わない。"""
    cam = np.stack([(np.asarray(u) - CX) / FY, (np.asarray(v) - CY) / FY, np.ones(np.shape(u))], -1)
    return cam @ T[:3, :3].T


def plane_depth(T, d0):
    """平面 z = d0 (base) を写した z 深度画像 (H, W) を解析的に作る。"""
    uu, vv = np.meshgrid(np.arange(W, dtype=float), np.arange(H, dtype=float))
    r = _rays_world(T, uu, vv)
    return (d0 - T[2, 3]) / r[..., 2]


def ray_plane_point(u, v, T, d0):
    """画素 (u, v) の光線と平面 z = d0 の交点 (期待値の計算用)。"""
    r = _rays_world(T, np.float64(u), np.float64(v))
    return T[:3, 3] + (d0 - T[2, 3]) / r[2] * r


def _stepped_image(fill=9.0, inner=0.6):
    """外周 9.0 m、bbox (100,100,120,120) の中央領域 [105,115)^2 だけ 0.6 m の深度画像。"""
    D = np.full((H, W), fill)
    D[105:115, 105:115] = inner
    return D


def _set_center_pixels(D, indices, value):
    """中央領域 (10x10) の indices 番目 (0..99) の画素に value を入れる。"""
    rr, cc = np.divmod(np.asarray(indices), 10)
    D[105 + rr, 105 + cc] = value


def _sample_pixels(seed, n, T=T_OBLIQUE):
    rng = np.random.default_rng(seed)
    u = rng.uniform(0, W - 1, n)
    v = rng.uniform(0, H - 1, n)
    d = rng.uniform(0.3, 0.9, n)
    return u, v, d


# ================================================================ 1. 正常系
def test_camera_keeps_values_and_is_frozen():
    """U-01: Camera は値を保持し frozen である。🟡"""
    # Given/When: 各フィールドを指定して作る
    cam = Camera(fx=FY, fy=FY, cx=319.5, cy=239.5, width=640, height=480)
    # Then: 値が保持され、代入は FrozenInstanceError
    assert (cam.fx, cam.fy, cam.cx, cam.cy, cam.width, cam.height) == (FY, FY, 319.5, 239.5, 640, 480)
    with pytest.raises(dataclasses.FrozenInstanceError):
        cam.fx = 1.0


def test_camera_matrix_values():
    """U-02: camera_matrix は [[fx,0,cx],[0,fy,cy],[0,0,1]] の 3x3 float64。🔵"""
    # When
    K = camera_matrix(CAMERA)
    # Then
    assert K.shape == (3, 3) and K.dtype == np.float64
    np.testing.assert_allclose(K, [[FY, 0, CX], [0, FY, CY], [0, 0, 1]], atol=1e-12)
    assert K[1, 1] == pytest.approx(415.6922, abs=1e-4)  # 主点規約 (w-1)/2 とともに固定


def test_pixel_to_camera_principal_point_is_on_optical_axis():
    """U-03: 主点の画素は光軸上 (0, 0, depth)。🟡"""
    p = pixel_to_camera(CX, CY, 0.6, CAMERA)
    np.testing.assert_allclose(p, [0.0, 0.0, 0.6], atol=1e-12)


def test_pixel_to_camera_formula_with_float_pixels():
    """U-04: u=列→x、v=行→y の式。画素は丸めない。🔵"""
    np.testing.assert_allclose(pixel_to_camera(CX + FY, CY - FY / 2, 2.0, CAMERA), [2.0, -1.0, 2.0], atol=1e-12)
    expected = [(100.25 - CX) * 0.5 / FY, (50.75 - CY) * 0.5 / FY, 0.5]
    np.testing.assert_allclose(pixel_to_camera(100.25, 50.75, 0.5, CAMERA), expected, atol=1e-12)


def test_camera_to_base_homogeneous_transform():
    """U-05: camera_to_base は R @ p + t。🔵"""
    p = np.array([0.1, -0.2, 0.3])
    np.testing.assert_allclose(camera_to_base(p, np.eye(4)), p, atol=EXACT_TOL_M)
    T = np.eye(4)
    T[:3, 3] = (1.0, 2.0, 3.0)
    np.testing.assert_allclose(camera_to_base(p, T), p + [1.0, 2.0, 3.0], atol=EXACT_TOL_M)
    np.testing.assert_allclose(
        camera_to_base(np.array([0, 0, 0.6217716622683926]), T_OBLIQUE), [0.24, 0.10, 0.0], atol=EXACT_TOL_M
    )


def test_project_base_to_pixel_on_optical_axis():
    """U-06: 注視点は主点に投影され、戻り値は float 3 要素の tuple。🟡"""
    out = project_base_to_pixel(np.array([0.24, 0.10, 0.0]), T_OBLIQUE, CAMERA)
    assert isinstance(out, tuple) and len(out) == 3
    assert all(type(x) is float for x in out)
    assert out[0] == pytest.approx(319.5, abs=1e-9)
    assert out[1] == pytest.approx(239.5, abs=1e-9)
    assert out[2] == pytest.approx(0.6217716622683926, abs=1e-12)


def _roundtrip_pixels(camera, T, u, v, d):
    P = np.array([camera_to_base(pixel_to_camera(a, b, c, camera), T) for a, b, c in zip(u, v, d)])
    P2 = np.empty_like(P)
    uvd2 = np.empty((len(u), 3))
    for i, p in enumerate(P):
        uvd2[i] = project_base_to_pixel(p, T, camera)
        P2[i] = camera_to_base(pixel_to_camera(*uvd2[i], camera), T)
    return P, P2, uvd2


def test_roundtrip_pixel_base_pixel_within_1mm():
    """U-07: 乱数 1000 点 + 隅の往復誤差 (浮動小数の画素のまま) が 1 mm 以下。🔵"""
    # Given: 乱数 1000 点 + 隅・画像外縁 x 深度 {0.3, 0.9}
    u, v, d = _sample_pixels(0, 1000)
    corners = [(0, 0), (639, 0), (0, 479), (639, 479), (319.5, 239.5), (-0.5, -0.5), (639.5, 479.5)]
    cu, cv = zip(*corners * 2)
    cd = [0.3] * len(corners) + [0.9] * len(corners)
    u, v, d = np.r_[u, cu], np.r_[v, cv], np.r_[d, cd]
    # When
    P, P2, uvd2 = _roundtrip_pixels(CAMERA, T_OBLIQUE, u, v, d)
    # Then
    assert np.linalg.norm(P2 - P, axis=1).max() <= ROUNDTRIP_TOL_M
    assert np.abs(uvd2[:, 0] - u).max() <= 1e-6 and np.abs(uvd2[:, 1] - v).max() <= 1e-6
    assert np.abs(uvd2[:, 2] - d).max() <= 1e-9


def test_roundtrip_base_pixel_base_within_1mm():
    """U-08: base → 画素 → base の往復誤差が 1 mm 以下 (作業領域と箱を含む)。🔵"""
    rng = np.random.default_rng(1)
    pts = np.column_stack([rng.uniform(0.10, 0.40, 1000), rng.uniform(-0.15, 0.25, 1000), rng.uniform(0, 0.10, 1000)])
    for p in pts:
        u, v, z = project_base_to_pixel(p, T_OBLIQUE, CAMERA)
        p2 = camera_to_base(pixel_to_camera(u, v, z, CAMERA), T_OBLIQUE)
        assert np.linalg.norm(p2 - p) <= ROUNDTRIP_TOL_M


def test_roundtrip_non_square_pixels_and_offset_principal_point():
    """U-09: fx≠fy・主点が中心でないカメラでも往復できる (fx/fy, cx/cy の取り違え検出)。🟡"""
    cam = Camera(fx=500.0, fy=480.0, cx=300.3, cy=250.7, width=800, height=600)
    rng = np.random.default_rng(2)
    u, v, d = rng.uniform(0, 799, 200), rng.uniform(0, 599, 200), rng.uniform(0.3, 0.9, 200)
    P, P2, uvd2 = _roundtrip_pixels(cam, T_OBLIQUE, u, v, d)
    assert np.linalg.norm(P2 - P, axis=1).max() <= ROUNDTRIP_TOL_M
    assert np.abs(uvd2[:, 0] - u).max() <= 1e-6 and np.abs(uvd2[:, 1] - v).max() <= 1e-6


def test_mujoco_cam_to_opencv_identity():
    """U-10: 単位行列は diag(1,-1,-1,1) になる。🟡"""
    np.testing.assert_allclose(mujoco_cam_to_opencv(np.eye(4)), FLIP, atol=1e-12)


def test_mujoco_cam_to_opencv_is_involution_and_keeps_translation():
    """U-11: 2 回で元に戻り、平行移動は不変。🟡"""
    T_mj = T_OBLIQUE @ FLIP
    T_cv = mujoco_cam_to_opencv(T_mj)
    np.testing.assert_allclose(T_cv, T_OBLIQUE, atol=1e-12)
    np.testing.assert_allclose(mujoco_cam_to_opencv(T_cv), T_mj, atol=1e-12)
    assert np.array_equal(T_cv[:3, 3], T_mj[:3, 3])


def test_mujoco_cam_to_opencv_optical_axis_sign():
    """U-12: MuJoCo の前方 (0,0,-1) と OpenCV の前方 (0,0,1) が同じ世界点になる。🟡"""
    T_mj = T_OBLIQUE @ FLIP
    T_cv = mujoco_cam_to_opencv(T_mj)
    np.testing.assert_allclose(T_mj @ [0, 0, -1, 1], T_cv @ [0, 0, 1, 1], atol=1e-12)


@pytest.mark.parametrize("T", [T_OBLIQUE, np.eye(4, dtype=int), T_OBLIQUE.tolist()], ids=["ndarray", "int", "list"])
def test_validate_T_returns_float64_copy(T):
    """U-13: 正しい T は float64 の新しい 4x4 で返る (int・ネストしたリストも可)。🟡"""
    out = validate_T_base_cam(T)
    assert out.shape == (4, 4) and out.dtype == np.float64
    np.testing.assert_array_equal(out, np.asarray(T, dtype=float))
    if isinstance(T, np.ndarray):
        assert not np.shares_memory(out, T)


def test_representative_depth_reads_only_center_region():
    """U-14: 中央領域 (shrink=0.5) だけを読み、外周は混ざらない。🔵"""
    assert representative_depth(_stepped_image(), (100, 100, 120, 120)) == 0.6


def test_representative_depth_is_robust_to_outliers_below_half():
    """U-15: 外れ値が半分未満なら中央値は真値の 1 mm 以内。半分に達すると崩れる (特性)。🔵"""
    D = _stepped_image()
    _set_center_pixels(D, range(49), 5.0)
    assert abs(representative_depth(D, (100, 100, 120, 120)) - 0.6) <= 1e-3
    D50 = _stepped_image()
    _set_center_pixels(D50, range(50), 5.0)
    assert representative_depth(D50, (100, 100, 120, 120)) == pytest.approx(2.8)  # (0.6+5.0)/2


def test_representative_depth_ignores_nan_inf_zero_negative():
    """U-16: NaN・inf・0・負の深度は無視し、有効な 1 画素の値を返す。🔵"""
    D = _stepped_image()
    _set_center_pixels(D, range(0, 33), np.nan)
    _set_center_pixels(D, range(33, 66), 0.0)
    _set_center_pixels(D, range(66, 99), -1.0)
    assert representative_depth(D, (100, 100, 120, 120)) == 0.6
    _set_center_pixels(D, range(66, 83), np.inf)  # -1.0 の一部を +inf に
    assert representative_depth(D, (100, 100, 120, 120)) == 0.6


def test_representative_depth_shrink_one_uses_whole_bbox():
    """U-17: shrink=1.0 は bbox 全体 (0.6 が 100 画素、9.0 が 300 画素 → 中央値 9.0)。🟡"""
    assert representative_depth(_stepped_image(), (100, 100, 120, 120), shrink=1.0) == 9.0


def test_representative_depth_single_pixel_bbox_reads_row_v_col_u():
    """U-18: 1x1 の bbox (10,20,11,21) は D[行 v=20, 列 u=10] を読む。🔵"""
    D = np.full((H, W), 9.0)
    D[20, 10] = 0.5
    D[10, 20] = 0.7  # 行と列を取り違えたときの罠
    assert representative_depth(D, (10, 20, 11, 21)) == 0.5


def test_bbox_to_base_point_downward_camera_exact():
    """U-19: 真下を向くカメラ・平面 z=0.03 で厳密に一致する。中心画素は ((x1+x2-1)/2, (y1+y2-1)/2)。🔵"""
    D = plane_depth(T_DOWN, 0.03)
    assert np.allclose(D, 0.57)
    p = bbox_to_base_point((300, 220, 340, 260), D, CAMERA, T_DOWN)
    np.testing.assert_allclose(p, [0.25, 0.0, 0.03], atol=EXACT_TOL_M)
    p2 = bbox_to_base_point((100, 50, 121, 71), D, CAMERA, T_DOWN)  # 中心画素 (110, 60)
    np.testing.assert_allclose(p2, ray_plane_point(110, 60, T_DOWN, 0.03), atol=EXACT_TOL_M)
    np.testing.assert_allclose(p2, [-0.03726784, 0.24613164, 0.03], atol=1e-7)


def test_bbox_to_base_point_returns_surface_point_without_block_correction():
    """U-20: 戻り値は表面の点 (z=0.03)。ブロック中心 (0.015) への補正は呼び出し側の責務。🔵"""
    p = bbox_to_base_point((300, 220, 340, 260), plane_depth(T_DOWN, 0.03), CAMERA, T_DOWN)
    assert p[2] == pytest.approx(0.03, abs=EXACT_TOL_M)
    assert abs(p[2] - 0.015) > 0.01 and abs(p[2] - 0.045) > 0.01


@pytest.mark.parametrize(
    ("bbox", "center"),
    [((300, 220, 340, 260), (319.5, 239.5)), ((0, 0, 40, 40), (19.5, 19.5)), ((600, 440, 640, 480), (619.5, 459.5))],
)
def test_bbox_to_base_point_oblique_camera_within_1mm(bbox, center):
    """U-21: 斜めのカメラ・平面 z=0 でも中央値の深度で 1 mm 以内。🟡"""
    D = plane_depth(T_OBLIQUE, 0.0)
    p = bbox_to_base_point(bbox, D, CAMERA, T_OBLIQUE)
    assert np.linalg.norm(p - ray_plane_point(*center, T_OBLIQUE, 0.0)) <= ROUNDTRIP_TOL_M


def test_bbox_to_base_point_accepts_shrink():
    """U-22: shrink を渡せ、0.5 は深度 0.6、1.0 は深度 9.0 に対応する。🟡"""
    D, bbox = _stepped_image(), (100, 100, 120, 120)
    for shrink, depth in ((0.5, 0.6), (1.0, 9.0)):
        expected = camera_to_base(pixel_to_camera(109.5, 109.5, depth, CAMERA), T_DOWN)
        np.testing.assert_allclose(
            bbox_to_base_point(bbox, D, CAMERA, T_DOWN, shrink=shrink), expected, atol=EXACT_TOL_M
        )


# ================================================================ 1.2 不変性・決定性・依存
def _call_all(D, T, p, bbox):
    return {
        "camera_matrix": camera_matrix(CAMERA),
        "pixel_to_camera": pixel_to_camera(100.0, 50.0, 0.6, CAMERA),
        "camera_to_base": camera_to_base(p, T),
        "project": project_base_to_pixel(np.array([0.24, 0.10, 0.0]), T, CAMERA),
        "validate": validate_T_base_cam(T),
        "mj2cv": mujoco_cam_to_opencv(T),
        "rep_depth": representative_depth(D, bbox),
        "bbox_point": bbox_to_base_point(bbox, D, CAMERA, T),
    }


def _inputs():
    D = _stepped_image()
    D[0, 0] = np.nan  # NaN を含む深度画像
    return D, T_OBLIQUE.copy(), np.array([0.0, 0.0, 0.6]), [100, 100, 120, 120]


def test_inputs_are_not_modified():
    """U-23: 全公開関数が入力 (配列・リスト) と writeable フラグを変更しない。🔵"""
    D, T, p, bbox = _inputs()
    snap = copy.deepcopy((D, T, p, bbox))
    flags = [a.flags.writeable for a in (D, T, p)]
    _call_all(D, T, p, bbox)
    assert np.array_equal(D, snap[0], equal_nan=True) and np.array_equal(T, snap[1])
    assert np.array_equal(p, snap[2]) and bbox == snap[3]
    assert [a.flags.writeable for a in (D, T, p)] == flags


def test_read_only_inputs_are_accepted():
    """U-24: 読み取り専用の入力でも動く (内部でその場の書き換えをしない)。🟡"""
    D, T, p, bbox = _inputs()
    for a in (D, T, p):
        a.setflags(write=False)
    _call_all(D, T, p, bbox)  # 例外が出ないこと


def test_return_values_are_new_writable_float64_arrays():
    """U-25: 配列の戻り値は新しい書き込み可能な float64。representative_depth は float。🟡"""
    D, T, p, bbox = _inputs()
    out = _call_all(D, T, p, bbox)
    for key, src in (("camera_to_base", p), ("validate", T), ("mj2cv", T), ("bbox_point", D)):
        a = out[key]
        assert isinstance(a, np.ndarray) and a.dtype == np.float64 and a.flags.writeable
        assert not np.shares_memory(a, src)
    for key in ("camera_matrix", "pixel_to_camera"):
        assert out[key].dtype == np.float64 and out[key].flags.writeable
    assert type(out["rep_depth"]) is float
    K1 = camera_matrix(CAMERA)
    K1[0, 0] = -1.0
    assert camera_matrix(CAMERA)[0, 0] == FY and camera_matrix(CAMERA) is not camera_matrix(CAMERA)


def test_functions_are_deterministic():
    """U-26: 同じ入力で 2 回呼ぶとビット単位で一致する。🔵"""
    a, b = _call_all(*_inputs()), _call_all(*_inputs())
    for key in a:
        assert np.array_equal(a[key], b[key])


def test_float32_depth_image_is_accepted():
    """U-27: render_rgbd と同じ float32 の深度画像を受け付ける。🟡"""
    D32 = _stepped_image().astype(np.float32)
    d = representative_depth(D32, (100, 100, 120, 120))
    assert type(d) is float and d == float(np.float32(0.6))
    assert bbox_to_base_point((100, 100, 120, 120), D32, CAMERA, T_DOWN).dtype == np.float64


def test_projection_module_does_not_import_mujoco():
    """U-28: projection.py は numpy・標準ライブラリだけを import する (純粋関数)。🔵"""
    tree = ast.parse(PROJECTION_PATH.read_text(encoding="utf-8"))
    mods = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            mods |= {a.name.split(".")[0] for a in node.names}
        elif isinstance(node, ast.ImportFrom) and node.level == 0:
            mods.add((node.module or "").split(".")[0])
    allowed = {"numpy", "math", "dataclasses", "typing", "__future__", "collections", "numbers"}
    assert "mujoco" not in mods and mods <= allowed


# ================================================================ 2. 異常系 (すべて ValueError)
# TypeError は使わない (要件 2.7)。メッセージの文言はアサートしない (O-4)。
NAN, INF = float("nan"), float("inf")
BBOX = (100, 100, 120, 120)


def _cam(**kw):
    base = {"fx": FY, "fy": FY, "cx": CX, "cy": CY, "width": W, "height": H}
    return Camera(**{**base, **kw})


@pytest.mark.parametrize("field", ["fx", "fy"])
@pytest.mark.parametrize("value", [0, -1.0, NAN, INF])
def test_camera_rejects_invalid_focal_length(field, value):
    """U-29: 焦点距離は有限かつ正。🟡"""
    with pytest.raises(ValueError):
        _cam(**{field: value})


@pytest.mark.parametrize("field", ["width", "height"])
@pytest.mark.parametrize("value", [0, -640, 640.0, True, "640"])
def test_camera_rejects_invalid_image_size(field, value):
    """U-30: 画像サイズは正の整数 (bool・float・文字列は不可)。🔴"""
    with pytest.raises(ValueError):
        _cam(**{field: value})


@pytest.mark.parametrize("kw", [{"fx": True}, {"cx": NAN}, {"cy": INF}])
def test_camera_rejects_bool_and_non_finite_principal_point(kw):
    """U-31: bool の焦点距離・非有限の主点は不可。🔴"""
    with pytest.raises(ValueError):
        _cam(**kw)


@pytest.mark.parametrize("depth", [0.0, -0.1, NAN, INF, -INF])
def test_pixel_to_camera_rejects_invalid_depth(depth):
    """U-32: 深度が 0・負・NaN・inf ならエラー (完了条件「不正な深度でエラー」)。🔵"""
    with pytest.raises(ValueError):
        pixel_to_camera(100.0, 50.0, depth, CAMERA)


@pytest.mark.parametrize(("u", "v"), [(NAN, 1.0), (INF, 1.0), (1.0, NAN), (1.0, -INF), (True, 1.0)])
def test_pixel_to_camera_rejects_invalid_pixel(u, v):
    """U-33: 画素が NaN・inf・bool ならエラー。🟡(bool は 🔴)"""
    with pytest.raises(ValueError):
        pixel_to_camera(u, v, 0.6, CAMERA)


@pytest.mark.parametrize(
    "p",
    [np.zeros(2), np.zeros(4), np.zeros((3, 1)), np.array([NAN, 0, 0.6]), np.array([0, INF, 0.6])],
    ids=["len2", "len4", "col", "nan", "inf"],
)
def test_camera_to_base_rejects_invalid_point(p):
    """U-34: p は (3,) の有限ベクトルのみ (O-3)。🔴(形状)/ 🟡(有限)"""
    with pytest.raises(ValueError):
        camera_to_base(p, T_OBLIQUE)


def _t_with_nan():
    T = T_OBLIQUE.copy()
    T[0, 3] = np.nan
    return T


@pytest.mark.parametrize("T", [np.eye(4)[:3], _t_with_nan(), T_MIRROR], ids=["shape", "nan", "mirror"])
def test_camera_to_base_validates_T_fully(T):
    """U-35: camera_to_base も T を完全に検証する (O-1)。🔴"""
    with pytest.raises(ValueError):
        camera_to_base(np.array([0.0, 0.0, 0.6]), T)


def test_project_base_to_pixel_rejects_points_not_in_front_of_camera():
    """U-36: カメラの後ろ・同一平面 (カメラ座標 z<=0) の点はエラー。🟡

    同一平面の点は浮動小数の丸めで z が厳密に 0 にならないことがあるため、軸平行の T_DOWN で作る。
    """
    eye, z_axis = T_OBLIQUE[:3, 3], T_OBLIQUE[:3, 2]
    with pytest.raises(ValueError):
        project_base_to_pixel(eye - 0.1 * z_axis, T_OBLIQUE, CAMERA)  # 後ろ
    with pytest.raises(ValueError):
        project_base_to_pixel(T_DOWN[:3, 3] + [0.1, 0.0, 0.0], T_DOWN, CAMERA)  # 同一平面 (z=0)


def test_project_base_to_pixel_rejects_nan_point():
    """U-37: p_base に NaN があればエラー。🟡"""
    with pytest.raises(ValueError):
        project_base_to_pixel(np.array([NAN, 0.0, 0.0]), T_OBLIQUE, CAMERA)


@pytest.mark.parametrize(
    "T",
    [np.eye(3), np.zeros((4, 3)), np.zeros((3, 4)), np.zeros(16), np.zeros((4, 4, 1)), 1.0, np.float64(1.0)],
    ids=["3x3", "4x3", "3x4", "flat", "4x4x1", "scalar", "np_scalar"],
)
def test_validate_T_rejects_wrong_shape(T):
    """U-38: 形状は (4,4) のみ。🟡"""
    with pytest.raises(ValueError):
        validate_T_base_cam(T)


@pytest.mark.parametrize(
    "T",
    [_t_with_nan(), np.where(np.eye(4) == 1, np.inf, 0.0), np.full((4, 4), "a"), np.eye(4, dtype=complex)],
    ids=["nan", "inf", "string", "complex"],
)
def test_validate_T_rejects_non_finite_or_non_real(T):
    """U-39: 有限の実数以外 (NaN・inf・文字列・複素数) はエラー。🟡(複素数は 🔴)"""
    with pytest.raises(ValueError):
        validate_T_base_cam(T)


def test_validate_T_rejects_reflection():
    """U-40: det(R) = -1 (鏡映) はエラー。🟡"""
    with pytest.raises(ValueError):
        validate_T_base_cam(T_MIRROR)


@pytest.mark.parametrize("T", [T_MIRROR, np.eye(3)], ids=["mirror", "3x3"])
def test_mujoco_cam_to_opencv_validates_input(T):
    """U-41: 入力は validate_T_base_cam で検証され、呼んだ後も入力は変わらない。🟡"""
    before = T.copy()
    with pytest.raises(ValueError):
        mujoco_cam_to_opencv(T)
    assert np.array_equal(T, before)


def test_representative_depth_raises_when_no_valid_depth_and_does_not_fall_back():
    """U-42: 中央領域がすべて無効ならエラー。外周の有効な 9.0 へフォールバックしない。🔵"""
    D = _stepped_image()
    _set_center_pixels(D, range(0, 25), np.nan)
    _set_center_pixels(D, range(25, 50), 0.0)
    _set_center_pixels(D, range(50, 75), -1.0)
    _set_center_pixels(D, range(75, 100), np.inf)
    with pytest.raises(ValueError):
        representative_depth(D, BBOX)


@pytest.mark.parametrize(
    "bbox", [(10, 10, 10, 20), (10, 10, 20, 10), (20, 10, 5, 30), (10, 30, 20, 5)], ids=["x1=x2", "y1=y2", "x-rev", "y-rev"]
)
def test_representative_depth_rejects_empty_or_reversed_bbox(bbox):
    """U-43: 面積 0・反転の bbox はエラー。🔴"""
    with pytest.raises(ValueError):
        representative_depth(_stepped_image(), bbox)


@pytest.mark.parametrize("bbox", [(700, 10, 710, 20), (-20, -20, -5, -5), (640, 0, 650, 10), (-5, 0, 0, 10)])
def test_representative_depth_rejects_bbox_outside_image(bbox):
    """U-44: 完全に画像外・端に接するだけの bbox はクリップ後に面積 0 でエラー。🔴"""
    with pytest.raises(ValueError):
        representative_depth(_stepped_image(), bbox)


@pytest.mark.parametrize(
    "bbox",
    [(0, 0, 10), (0, 0, 10, 10, 10), (NAN, 0, 10, 10), (0, INF, 10, 10), (True, 0, 10, 10), ("0", 0, 10, 10), None],
    ids=["len3", "len5", "nan", "inf", "bool", "str", "None"],
)
def test_representative_depth_rejects_malformed_bbox(bbox):
    """U-45: bbox は長さ 4 の有限の実数 (bool 不可)。🔴"""
    with pytest.raises(ValueError):
        representative_depth(_stepped_image(), bbox)


@pytest.mark.parametrize("shrink", [0, -0.5, 1.0000001, 1.5, NAN, INF, True])
def test_representative_depth_rejects_invalid_shrink(shrink):
    """U-46: 0 < shrink <= 1 (True は bool として不可)。🔴"""
    with pytest.raises(ValueError):
        representative_depth(_stepped_image(), BBOX, shrink=shrink)


@pytest.mark.parametrize(
    "D",
    [np.ones(640), np.ones((480, 640, 1)), np.ones((0, 0)), np.ones((480, 640), bool),
     np.ones((480, 640), complex), np.full((480, 640), "a")],
    ids=["1d", "3d", "empty", "bool", "complex", "str"],
)
def test_representative_depth_rejects_invalid_depth_image(D):
    """U-47: 深度画像は 2 次元・実数・1x1 以上。🟡(dtype は 🔴)"""
    with pytest.raises(ValueError):
        representative_depth(D, BBOX)


@pytest.mark.parametrize("shape", [(480, 639), (640, 480)], ids=["narrow", "transposed"])
def test_bbox_to_base_point_rejects_wrong_depth_shape(shape):
    """U-48: 深度画像は (camera.height, camera.width) と一致する必要がある。🟡"""
    with pytest.raises(ValueError):
        bbox_to_base_point(BBOX, np.full(shape, 0.6), CAMERA, T_DOWN)


def test_bbox_to_base_point_rejects_reflection_T():
    """U-49: T の回転部が det=-1 ならエラー (validate_T_base_cam を通す)。🔵"""
    with pytest.raises(ValueError):
        bbox_to_base_point(BBOX, np.full((H, W), 0.6), CAMERA, T_MIRROR)


def test_bbox_to_base_point_propagates_no_valid_depth_error():
    """U-50: 中央領域の深度がすべて NaN なら representative_depth のエラーが伝わる。🔵"""
    D = _stepped_image()
    D[100:120, 100:120] = np.nan
    with pytest.raises(ValueError):
        bbox_to_base_point(BBOX, D, CAMERA, T_DOWN)


# ================================================================ 3. 境界値
def test_camera_accepts_boundary_values():
    """U-51: 極小の焦点距離・画像外の主点・numpy 整数のサイズを受け付ける。🔴"""
    _cam(fx=1e-9, fy=1e-9)
    _cam(cx=-100.0, cy=1000.0)
    cam = _cam(width=np.int64(640))
    assert cam.width == 640


def test_pixel_to_camera_boundaries():
    """U-52: 極小の深度・画像外の画素も値を返す (範囲判定は呼び出し側)。🟡(画像外は 🔴)"""
    assert pixel_to_camera(100.0, 50.0, 1e-9, CAMERA)[2] == pytest.approx(1e-9)
    for u in (-100.0, 1000.0):
        np.testing.assert_allclose(pixel_to_camera(u, 50.0, 0.6, CAMERA)[0], (u - CX) * 0.6 / FY, atol=1e-12)


def test_project_base_to_pixel_returns_value_outside_image():
    """U-53: カメラ前方だが画面外の点でも例外なく u > 640 を返す。🟡"""
    p = np.array([0.24, 0.10, 0.0]) + 2.0 * T_OBLIQUE[:3, 0]
    u, _, z = project_base_to_pixel(p, T_OBLIQUE, CAMERA)
    assert u > W and z > 0


def test_validate_T_bottom_row_tolerance():
    """U-54: 最下行の許容誤差 1e-9。🟡"""
    ok = T_OBLIQUE.copy()
    ok[3, 3] = 1 + 1e-10
    validate_T_base_cam(ok)
    for idx in ((3, 3), (3, 0)):
        bad = T_OBLIQUE.copy()
        bad[idx] += 1e-8
        with pytest.raises(ValueError):
            validate_T_base_cam(bad)


def test_validate_T_orthogonality_tolerance():
    """U-55: 直交性の許容誤差 1e-6 (スケール 1+1e-7 は可、1+1e-5・せん断は不可)。🟡"""
    ok = T_OBLIQUE.copy()
    ok[:3, :3] *= 1 + 1e-7
    validate_T_base_cam(ok)
    scaled = T_OBLIQUE.copy()
    scaled[:3, :3] *= 1 + 1e-5
    sheared = T_OBLIQUE.copy()
    sheared[0, 1] += 1e-3
    for bad in (scaled, sheared):
        with pytest.raises(ValueError):
            validate_T_base_cam(bad)


def test_validate_T_limitation_cannot_detect_missing_convention_flip():
    """U-56: 制限の記録。MuJoCo 規約のまま (diag(1,-1,-1,1) 未適用) の T は det=+1 なので通る。🟡

    規約の取り違えは T の検証では検出できない。統合テスト S-02 (床の逆投影) で検出する。
    """
    validate_T_base_cam(T_OBLIQUE @ FLIP)  # 例外が出ないこと


# bbox, shrink, 期待領域 (r0, r1, c0, c1)
REGION_CASES = [
    ((100, 100, 120, 120), 0.5, (105, 115, 105, 115)),
    ((0, 0, 4, 4), 0.5, (1, 3, 1, 3)),
    ((10, 20, 11, 21), 0.5, (20, 21, 10, 11)),
    ((100, 100, 121, 121), 1e-6, (110, 111, 110, 111)),
    ((100, 100, 120, 120), 1e-6, (109, 111, 109, 111)),
    ((100, 100, 120, 120), 1.0, (100, 120, 100, 120)),
    ((10.2, 20.7, 30.4, 40.1), 0.5, (25, 36, 15, 26)),
    ((-10, -10, 10, 10), 0.5, (2, 8, 2, 8)),
    ((630, 470, 650, 490), 0.5, (472, 478, 632, 638)),
]


@pytest.mark.parametrize(("bbox", "shrink", "region"), REGION_CASES)
def test_representative_depth_region_pixel_range(bbox, shrink, region):
    """U-57: 中央領域の画素範囲 (floor/ceil・最低 1 画素・クリップ)。🟡(浮動小数・クリップは 🔴)"""
    r0, r1, c0, c1 = region
    # (i) 領域 ⊆ 期待: 期待領域だけ NaN → 有効画素なしでエラー
    D = np.full((H, W), 0.5)
    D[r0:r1, c0:c1] = np.nan
    with pytest.raises(ValueError):
        representative_depth(D, bbox, shrink=shrink)
    # (ii) 領域 ⊇ 4 隅: 期待領域の隅 1 画素だけ 0.5、他は全部 NaN → 0.5
    for r, c in ((r0, c0), (r0, c1 - 1), (r1 - 1, c0), (r1 - 1, c1 - 1)):
        D = np.full((H, W), np.nan)
        D[r, c] = 0.5
        assert representative_depth(D, bbox, shrink=shrink) == 0.5


def test_bbox_to_base_point_uses_clipped_center():
    """U-58: はみ出した bbox は、クリップ後 (0,0,30,30) の中心 (14.5, 14.5) を使う。🔴"""
    D = plane_depth(T_OBLIQUE, 0.0)
    p = bbox_to_base_point((-10, -10, 30, 30), D, CAMERA, T_OBLIQUE)
    clipped = ray_plane_point(14.5, 14.5, T_OBLIQUE, 0.0)
    unclipped = ray_plane_point(9.5, 9.5, T_OBLIQUE, 0.0)
    assert np.linalg.norm(p - clipped) <= ROUNDTRIP_TOL_M
    assert np.linalg.norm(clipped - unclipped) > 3e-3  # 両者を区別できる
    assert np.linalg.norm(p - unclipped) > 3e-3


def test_bbox_to_base_point_accepts_whole_image_bbox():
    """U-59: 画像全体の bbox (0,0,640,480) を受け付け、中心 (319.5, 239.5) を使う。🟡"""
    p = bbox_to_base_point((0, 0, 640, 480), plane_depth(T_DOWN, 0.03), CAMERA, T_DOWN)
    np.testing.assert_allclose(p, [0.25, 0.0, 0.03], atol=EXACT_TOL_M)

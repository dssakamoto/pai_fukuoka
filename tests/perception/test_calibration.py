"""calibration (camera_from_mujoco / t_base_cam_from_mujoco / Calibration / calibrations_close) の単体テスト (TASK-0009, Red フェーズ)。

テストケース定義: docs/implements/so101-vlm-pickplace/TASK-0009/calibration-testcases.md
対象: src/so101_vlm/perception/calibration.py の MuJoCo 計算・データクラス・比較。YAML 入出力は test_calibration_io.py。
描画は行わない。統合テストは test_calibration_scene.py、CLI は test_generate_calibration.py。
共有フィクスチャは conftest.py、共有定数・ヘルパは calibration_support.py。
信頼性: 🔵 要件・設計・実測に基づく / 🟡 妥当な推測 / 🔴 要件にない決定
"""

import dataclasses
import inspect
import math

import mujoco
import numpy as np
import pytest
from so101_vlm.perception.calibration import (
    Calibration,
    calibration_from_mujoco,
    calibrations_close,
    camera_from_mujoco,
    load_calibration,
    save_calibration,
    t_base_cam_from_mujoco,
)
from so101_vlm.perception.projection import (
    Camera,
    camera_to_base,
    mujoco_cam_to_opencv,
    validate_T_base_cam,
)

from calibration_support import (
    FY_60_480,
    SIM_T_ROWS,
    TINY_XML,
    forwarded,
    golden_calibration,
)

ANALYTIC_TOL = 1e-12  # 小さな MJCF の解析比較 (実測最大 4.6e-16)


def _cid(model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)


def _transform(rot, pos):
    T = np.eye(4)
    T[:3, :3] = np.asarray(rot).reshape(3, 3)
    T[:3, 3] = pos
    return T


def _tiny_with_fovy(fovy):
    m = mujoco.MjModel.from_xml_string(TINY_XML)
    m.cam_fovy[_cid(m, "cam")] = fovy
    return m


# ================================================================ 1.1 camera_from_mujoco
def test_camera_from_mujoco_fovy90_height480_gives_fy240():
    """C-01: fovy=90°・高さ 480 なら fy=240 (タスク定義のテストケース1)。🟡"""
    # 【テスト目的】: 画角 (度) から焦点距離 (画素) を求める式 fy=(h/2)/tan(fovy/2)
    # Given: fovy を 90° にした固定カメラ (式の結果は 240.00000000000003 なので近似で比べる)
    model = _tiny_with_fovy(90.0)
    # When
    c = camera_from_mujoco(model, "cam", 640, 480)
    # Then
    assert c.fy == pytest.approx(240.0, abs=1e-9)  # 【確認内容】: 焦点距離
    assert c.fx == c.fy  # 【確認内容】: 正方画素


def test_camera_from_mujoco_scene_640x480_fovy60(scene_model):
    """C-02: 実シーン 640x480・fovy=60° の値。🔵"""
    c = camera_from_mujoco(scene_model, "cam_top", 640, 480)
    assert c.fx == pytest.approx(FY_60_480, abs=1e-9)  # 【確認内容】: fx
    assert c.fy == pytest.approx(FY_60_480, abs=1e-9)  # 【確認内容】: fy
    assert (c.cx, c.cy) == (319.5, 239.5)  # 【確認内容】: 画素中心規約の主点
    assert (c.width, c.height) == (640, 480)  # 【確認内容】: 画像サイズは引数どおり


def test_camera_from_mujoco_principal_point_uses_pixel_center_convention(scene_model):
    """C-03: 主点は (w-1)/2 (幅と高さが違う・奇数)。🟡"""
    a = camera_from_mujoco(scene_model, "cam_top", 320, 240)
    assert a.fy == pytest.approx(207.84609690826534, abs=1e-9)
    assert (a.cx, a.cy) == (159.5, 119.5)
    b = camera_from_mujoco(scene_model, "cam_top", 641, 481)
    assert (b.cx, b.cy) == (320.0, 240.0)
    assert b.fy == pytest.approx(240.5 / math.tan(math.radians(30)), abs=1e-9)


def test_camera_from_mujoco_takes_size_from_arguments_not_model(scene_model):
    """C-04 / B-01: 画像サイズは引数から取る (1x1 でもエラーにならない)。🟡"""
    c = camera_from_mujoco(scene_model, "cam_top", 1, 1)
    assert (c.cx, c.cy) == (0.0, 0.0)
    assert c.fy == pytest.approx(0.8660254037844388, abs=1e-12)


def test_camera_from_mujoco_returns_camera_with_python_types(scene_model):
    """C-05: 戻り値は Camera で、フィールドは Python の型 (YAML へそのまま書ける)。🔴"""
    c = camera_from_mujoco(scene_model, "cam_top", 640, 480)
    assert isinstance(c, Camera)
    for name in ("fx", "fy", "cx", "cy"):
        assert type(getattr(c, name)) is float, name  # 【確認内容】: np.float64 を残さない
    assert type(c.width) is int and type(c.height) is int


def test_camera_from_mujoco_other_fovy_fixed_camera(tiny_model):
    """C-06: fovy が 60 以外の固定カメラ (小さな MJCF, fovy 50)。🟡"""
    c = camera_from_mujoco(tiny_model, "cam", 320, 240)
    assert c.fy == pytest.approx(257.340830461147, abs=1e-9)
    assert c.fx == c.fy


def test_camera_from_mujoco_accepts_numpy_integers(scene_model):
    """C-07: numpy の整数も幅・高さとして受け付ける。🟡"""
    c = camera_from_mujoco(scene_model, "cam_top", np.int64(640), np.int64(480))
    assert c.fy == pytest.approx(FY_60_480, abs=1e-9)
    assert type(c.width) is int


def test_camera_from_mujoco_does_not_modify_model(scene_model):
    """C-08: モデルを変更しない。🟡"""
    before = scene_model.cam_fovy.copy()
    camera_from_mujoco(scene_model, "cam_top", 640, 480)
    assert np.array_equal(scene_model.cam_fovy, before)  # 【確認内容】: cam_fovy 不変


# ================================================================ 1.2 t_base_cam_from_mujoco
def test_t_base_cam_scene_matches_expected_translation(scene_model, scene_data):
    """T-01: 実シーンの T (S-01 相当)。🔵"""
    T = t_base_cam_from_mujoco(scene_model, scene_data, "cam_top")
    assert T.shape == (4, 4) and T.dtype == np.float64
    validate_T_base_cam(T)  # 【確認内容】: 剛体変換として妥当
    np.testing.assert_allclose(T[:3, 3], [0.45, 0.30, 0.55], atol=1e-12)
    np.testing.assert_array_equal(T[3], [0, 0, 0, 1])


def test_t_base_cam_is_bit_identical_when_base_is_identity(scene_model, scene_data):
    """T-02: base が原点・単位回転のとき T == T_world_cam_mj @ diag(1,-1,-1,1) (ビット一致)。🔵"""
    sid = mujoco.mj_name2id(scene_model, mujoco.mjtObj.mjOBJ_SITE, "baseframe")
    assert np.array_equal(scene_data.site_xpos[sid], np.zeros(3))  # 前提
    assert np.array_equal(scene_data.site_xmat[sid].reshape(3, 3), np.eye(3))  # 前提
    cid = _cid(scene_model, "cam_top")
    T_wc = _transform(scene_data.cam_xmat[cid], scene_data.cam_xpos[cid])
    T = t_base_cam_from_mujoco(scene_model, scene_data, "cam_top")
    assert np.array_equal(T, T_wc @ np.diag([1.0, -1, -1, 1]))  # 【確認内容】: ビット一致


def test_t_base_cam_matches_regression_values(scene_model, scene_data):
    """T-03: 期待行列の具体値 (回帰, 要件 2.6)。🔵"""
    T = t_base_cam_from_mujoco(scene_model, scene_data, "cam_top")
    np.testing.assert_allclose(T, SIM_T_ROWS, atol=1e-12)
    assert T[2, 0] == pytest.approx(-2.7755575615628914e-17, abs=1e-12)
    assert T[0, 0] == pytest.approx(-0.6896566706214934)


def test_t_base_cam_independent_of_settling_and_reset(scene_model, scene_data):
    """T-04: 静定・reset に依存しない (固定カメラ・固定 base)。🔵"""
    T0 = t_base_cam_from_mujoco(scene_model, scene_data, "cam_top")
    settled = forwarded(scene_model)
    for _ in range(500):
        mujoco.mj_step(scene_model, settled)
    mujoco.mj_forward(scene_model, settled)
    mujoco.mj_resetData(scene_model, settled)
    mujoco.mj_forward(scene_model, settled)
    assert np.array_equal(t_base_cam_from_mujoco(scene_model, settled, "cam_top"), T0)


def _tiny_points_and_expected(tiny_model, tiny_data, T):
    rng = np.random.default_rng(0)
    pts = rng.uniform([-0.5, -0.5, 0.0], [1.0, 1.0, 0.8], size=(1000, 3))
    cid = _cid(tiny_model, "cam")
    sid = mujoco.mj_name2id(tiny_model, mujoco.mjtObj.mjOBJ_SITE, "baseframe")
    R_wc, t_wc = tiny_data.cam_xmat[cid].reshape(3, 3), tiny_data.cam_xpos[cid]
    p_cam = ((pts - t_wc) @ R_wc) * [1.0, -1.0, -1.0]  # R_wc.T @ (p - t) を OpenCV 規約へ
    T_wb = _transform(tiny_data.site_xmat[sid], tiny_data.site_xpos[sid])
    expected = (np.linalg.inv(T_wb) @ np.c_[pts, np.ones(len(pts))].T).T[:, :3]
    return pts, p_cam, expected, _transform(R_wc, t_wc), T_wb


def test_t_base_cam_general_base_matches_analytic(tiny_model, tiny_data):
    """T-05: base site を移動・回転した MJCF でも inv(T_world_base) @ p と一致 (最大 < 1e-12)。🟡"""
    T = t_base_cam_from_mujoco(tiny_model, tiny_data, "cam")
    _, p_cam, expected, _, _ = _tiny_points_and_expected(tiny_model, tiny_data, T)
    got = np.array([camera_to_base(p, T) for p in p_cam])
    assert np.abs(got - expected).max() < ANALYTIC_TOL  # 【確認内容】: 実測 4.6e-16


def test_t_base_cam_negative_control_ignoring_base_is_far_off(tiny_model, tiny_data):
    """T-06: 負の対照。base を無視した T は 5 cm 以上ずれる (T-05 の検出力)。🟡

    このテストが落ちたら T-05 の点群が base の影響を拾えていない。base を無視する実装は T-05 で落ちる。
    """
    T = t_base_cam_from_mujoco(tiny_model, tiny_data, "cam")
    _, p_cam, expected, T_wc, _ = _tiny_points_and_expected(tiny_model, tiny_data, T)
    T_ign = mujoco_cam_to_opencv(T_wc)
    err = np.linalg.norm(np.array([camera_to_base(p, T_ign) for p in p_cam]) - expected, axis=1)
    assert err.min() > 0.05  # 【確認内容】: 実測 最小 53.5 mm


def test_t_base_cam_negative_control_reversed_product_differs(tiny_model, tiny_data):
    """T-07: 負の対照。かけ算の順序を逆にした行列は T と大きく違う。🟡"""
    T = t_base_cam_from_mujoco(tiny_model, tiny_data, "cam")
    _, _, _, T_wc, T_wb = _tiny_points_and_expected(tiny_model, tiny_data, T)
    T_bad = mujoco_cam_to_opencv(T_wc @ np.linalg.inv(T_wb))
    assert np.abs(T_bad - T).max() > 0.5  # 【確認内容】: 実測 0.706


def test_t_base_cam_default_base_site_is_baseframe(tiny_model, tiny_data):
    """T-08: base_site の既定値は "baseframe"。🟡"""
    a = t_base_cam_from_mujoco(tiny_model, tiny_data, "cam")
    b = t_base_cam_from_mujoco(tiny_model, tiny_data, "cam", base_site="baseframe")
    assert np.array_equal(a, b)


def test_t_base_cam_returns_fresh_writable_array_and_keeps_data(tiny_model, tiny_data):
    """T-09: 戻り値は新しい書き込み可能な配列で、data は変わらない。🟡"""
    names = ("cam_xmat", "cam_xpos", "site_xmat", "site_xpos")
    before = {n: getattr(tiny_data, n).copy() for n in names}
    T = t_base_cam_from_mujoco(tiny_model, tiny_data, "cam")
    first = T.copy()
    assert T.flags.writeable  # 【確認内容】: 書き込み可能
    T[0, 0] = 99.0
    for n in names:
        assert not np.shares_memory(T, getattr(tiny_data, n)), n  # 【確認内容】: メモリ非共有
        assert np.array_equal(getattr(tiny_data, n), before[n]), n  # 【確認内容】: data 不変
    assert np.array_equal(t_base_cam_from_mujoco(tiny_model, tiny_data, "cam"), first)


def test_t_base_cam_from_xml_file_matches_xml_string(tiny_model, tiny_data, tiny_xml_path):
    """T-10: XML ファイルから読んだモデルでも同じ結果。🟡"""
    m2 = mujoco.MjModel.from_xml_path(str(tiny_xml_path))
    T2 = t_base_cam_from_mujoco(m2, forwarded(m2), "cam")
    assert np.array_equal(T2, t_base_cam_from_mujoco(tiny_model, tiny_data, "cam"))


# ================================================================ 1.3 Calibration / calibration_from_mujoco
def test_calibration_constructs_for_sim_and_real():
    """D-01: 正しい値で作れる (sim / real)。🔵"""
    for source in ("sim", "real"):
        cal = golden_calibration(source)
        assert cal.source == source and cal.camera_name == "cam_top" and cal.base_frame == "baseframe"
        assert np.array_equal(cal.T_base_cam, np.array(SIM_T_ROWS))  # 【確認内容】: T は渡した値


def test_calibration_is_frozen():
    """D-02: frozen。🔵"""
    cal = golden_calibration()
    with pytest.raises(dataclasses.FrozenInstanceError):
        cal.source = "real"  # type: ignore[misc]
    with pytest.raises(dataclasses.FrozenInstanceError):
        cal.T_base_cam = np.eye(4)  # type: ignore[misc]


def test_calibration_holds_read_only_T():
    """D-03: 保持する T は書き込み不可。🔴"""
    cal = golden_calibration()
    assert cal.T_base_cam.flags.writeable is False
    with pytest.raises(ValueError):
        cal.T_base_cam[0, 0] = 1.0


def test_calibration_copies_input_array():
    """D-04: 渡した配列を後から書き換えても影響しない (コピー)。🔴"""
    T = np.array(SIM_T_ROWS)
    cam = Camera(fx=1.0, fy=1.0, cx=0.0, cy=0.0, width=2, height=2)
    cal = Calibration(cam, T, "sim", "cam_top", "baseframe")
    T[0, 3] = 9.0
    assert cal.T_base_cam[0, 3] == 0.45
    assert not np.shares_memory(cal.T_base_cam, T)


def test_calibration_converts_nested_list_to_float64():
    """D-05: ネストしたリストの T も float64 にする。🟡"""
    cam = Camera(fx=1.0, fy=1.0, cx=0.0, cy=0.0, width=2, height=2)
    cal = Calibration(cam, SIM_T_ROWS, "sim", "cam_top", "baseframe")  # type: ignore[arg-type]
    assert cal.T_base_cam.dtype == np.float64
    assert np.array_equal(cal.T_base_cam, np.array(SIM_T_ROWS))


def test_calibration_eq_is_identity_not_array_comparison():
    """D-06: eq=False (== は同一性)。比較は calibrations_close を使う。🔴"""
    a, b = golden_calibration(), golden_calibration()
    assert (a == b) is False  # 【確認内容】: 配列の == で例外/配列を返さない
    assert (a == a) is True


def test_calibration_from_mujoco_combines_camera_and_T(scene_model, scene_data):
    """F-01: C と T をまとめて source="sim" の Calibration にする。🟡"""
    cal = calibration_from_mujoco(scene_model, scene_data, "cam_top", 640, 480)
    assert cal.camera == camera_from_mujoco(scene_model, "cam_top", 640, 480)
    assert np.array_equal(cal.T_base_cam, t_base_cam_from_mujoco(scene_model, scene_data, "cam_top"))
    assert (cal.source, cal.camera_name, cal.base_frame) == ("sim", "cam_top", "baseframe")


def test_calibration_from_mujoco_records_base_site(tiny_model, tiny_data):
    """F-02: base_site が base_frame に記録される。🟡"""
    cal = calibration_from_mujoco(tiny_model, tiny_data, "cam", 320, 240, base_site="baseframe")
    assert (cal.base_frame, cal.camera_name) == ("baseframe", "cam")
    assert np.array_equal(cal.T_base_cam, t_base_cam_from_mujoco(tiny_model, tiny_data, "cam"))


# ================================================================ 1.6 calibrations_close
def _with_fx(cal, delta):
    return dataclasses.replace(cal, camera=dataclasses.replace(cal.camera, fx=cal.camera.fx + delta))


def _with_T03(cal, delta):
    T = cal.T_base_cam.copy()
    T[0, 3] += delta
    return dataclasses.replace(cal, T_base_cam=T)


def test_calibrations_close_same_values_true_even_with_atol0(sim_calibration, tmp_path):
    """K-01: 同じ値は True (atol=0 でも)。🔴"""
    p = tmp_path / "c.yaml"
    save_calibration(sim_calibration, p)
    back = load_calibration(p)
    assert calibrations_close(sim_calibration, back) and calibrations_close(sim_calibration, back, atol=0)


def test_calibrations_close_default_atol_is_keyword_only_1e9():
    """K-02: 既定 atol=1e-9 でキーワード専用。🔴"""
    prm = inspect.signature(calibrations_close).parameters["atol"]
    assert prm.default == 1e-9 and prm.kind is inspect.Parameter.KEYWORD_ONLY


def test_calibrations_close_float_boundary_fx():
    """K-03: fx の差の境界 (+5e-10 は True, +2e-9 は False)。🔴"""
    cal = golden_calibration()
    assert calibrations_close(cal, _with_fx(cal, 5e-10))
    assert not calibrations_close(cal, _with_fx(cal, 2e-9))


def test_calibrations_close_float_boundary_T_translation():
    """K-04: T の平行移動の差の境界。🔴"""
    cal = golden_calibration()
    assert calibrations_close(cal, _with_T03(cal, 5e-10))
    assert not calibrations_close(cal, _with_T03(cal, 2e-9))


def test_calibrations_close_strings_and_ints_must_match_exactly():
    """K-05: 文字列・整数は完全一致 (atol を大きくしても)。🔴"""
    cal = golden_calibration()
    others = [
        dataclasses.replace(cal, source="real"),
        dataclasses.replace(cal, camera_name="other_cam"),
        dataclasses.replace(cal, base_frame="other_base"),
        dataclasses.replace(cal, camera=dataclasses.replace(cal.camera, width=641)),
    ]
    for o in others:
        assert not calibrations_close(cal, o, atol=1.0)


def test_calibrations_close_atol0_detects_one_ulp():
    """K-06: atol=0 は 1 ulp の差も False。🔴"""
    cal = golden_calibration()
    assert not calibrations_close(cal, _with_fx(cal, np.nextafter(cal.camera.fx, np.inf) - cal.camera.fx), atol=0)


def test_calibrations_close_boundary_is_inclusive():
    """B-03: 差がちょうど atol のときは True (<=)。2 進で正確な値を使う。🔴"""
    cam = Camera(fx=1.0, fy=1.0, cx=0.0, cy=0.0, width=2, height=2)
    a = Calibration(cam, np.eye(4), "sim", "c", "b")
    b = Calibration(dataclasses.replace(cam, fx=1.0 + 2**-20), np.eye(4), "sim", "c", "b")
    assert calibrations_close(a, b, atol=2**-20)


# ================================================================ 2.1 camera_from_mujoco 異常系
def test_camera_from_mujoco_unknown_camera_raises(scene_model):
    """C-E01: カメラ名がない。🟡"""
    with pytest.raises(ValueError, match="no_such_cam"):
        camera_from_mujoco(scene_model, "no_such_cam", 640, 480)


@pytest.mark.parametrize("name", ["cam_trackcom", "cam_body_track"])
def test_camera_from_mujoco_rejects_tracking_cameras(tiny_model, name):
    """C-E02 / C-E03: 追従カメラ (trackcom / track) は拒否。🟡"""
    with pytest.raises(ValueError, match=name):
        camera_from_mujoco(tiny_model, name, 320, 240)


def test_camera_from_mujoco_rejects_sensorsize_camera(tiny_model):
    """C-E04: sensorsize 指定のカメラは拒否 (mode は固定なので mode の検査だけでは通ってしまう)。🔴"""
    cid = _cid(tiny_model, "cam_sensor")
    assert tiny_model.cam_mode[cid] == 0 and tiny_model.cam_sensorsize[cid].any()  # 前提
    with pytest.raises(ValueError, match="cam_sensor"):
        camera_from_mujoco(tiny_model, "cam_sensor", 320, 240)


@pytest.mark.parametrize("bad", [0, -1, 640.0, True, "640", None])
@pytest.mark.parametrize("which", ["width", "height"])
def test_camera_from_mujoco_rejects_bad_size(scene_model, which, bad):
    """C-E05: width / height が正の整数でない。🟡"""
    kw = {"width": 640, "height": 480, which: bad}
    with pytest.raises(ValueError):
        camera_from_mujoco(scene_model, "cam_top", kw["width"], kw["height"])


@pytest.mark.parametrize("fovy", [0.0, 180.0, -10.0, float("nan")])
def test_camera_from_mujoco_rejects_fovy_out_of_range(fovy):
    """C-E06: fovy が (0, 180) の外。🟡 (MJCF では作れないので配列を書き換える)"""
    with pytest.raises(ValueError):
        camera_from_mujoco(_tiny_with_fovy(fovy), "cam", 320, 240)


@pytest.mark.parametrize("fovy", [1e-6, 179.999])
def test_camera_from_mujoco_accepts_fovy_near_bounds(fovy):
    """B-02: fovy の開区間 (0, 180) の内側は正常。🟡"""
    c = camera_from_mujoco(_tiny_with_fovy(fovy), "cam", 320, 240)
    assert math.isfinite(c.fy) and c.fy > 0


# ================================================================ 2.2 t_base_cam_from_mujoco 異常系
def test_t_base_cam_rejects_unforwarded_data(scene_model):
    """T-E01: mj_forward 前の data は拒否し、data を変えない。🟡"""
    data = mujoco.MjData(scene_model)
    assert not data.site_xmat.any() and not data.cam_xmat.any()  # 前提
    with pytest.raises(ValueError):
        t_base_cam_from_mujoco(scene_model, data, "cam_top")
    assert not data.site_xmat.any()  # 【確認内容】: 関数が mj_forward を呼んでいない


def test_t_base_cam_unknown_camera_raises(scene_model, scene_data):
    """T-E02: カメラ名がない。🟡"""
    with pytest.raises(ValueError, match="no_such_cam"):
        t_base_cam_from_mujoco(scene_model, scene_data, "no_such_cam")


def test_t_base_cam_unknown_site_raises(scene_model, scene_data):
    """T-E03: base site 名がない。🟡"""
    with pytest.raises(ValueError, match="no_such_site"):
        t_base_cam_from_mujoco(scene_model, scene_data, "cam_top", base_site="no_such_site")


def test_t_base_cam_rejects_tracking_camera(tiny_model, tiny_data):
    """T-E04: 追従カメラ。🟡"""
    with pytest.raises(ValueError):
        t_base_cam_from_mujoco(tiny_model, tiny_data, "cam_trackcom")


def test_t_base_cam_rejects_data_from_other_model(tiny_model, scene_data):
    """T-E05: 別モデルの data (配列の大きさが違う) は ValueError。🔴

    O-T3: 実装が難しければ Green で削除してよい (記録する)。tiny は ncam=4、scene_data の cam_xpos は 1 件。
    """
    assert scene_data.cam_xpos.shape[0] != tiny_model.ncam  # 前提
    with pytest.raises(ValueError):
        t_base_cam_from_mujoco(tiny_model, scene_data, "cam")


# ================================================================ 2.3 Calibration 異常系
def _build(**over):
    kw = dict(
        camera=Camera(fx=1.0, fy=1.0, cx=0.0, cy=0.0, width=2, height=2),
        T_base_cam=np.array(SIM_T_ROWS),
        source="sim",
        camera_name="cam_top",
        base_frame="baseframe",
    )
    kw.update(over)
    return Calibration(**kw)


@pytest.mark.parametrize("bad", [{"fx": 1.0}, None])
def test_calibration_rejects_non_camera(bad):
    """D-E01: camera が Camera でない。🟡"""
    with pytest.raises(ValueError):
        _build(camera=bad)


@pytest.mark.parametrize("bad", ["test", "SIM", "", None])
def test_calibration_rejects_bad_source(bad):
    """D-E02: source が sim / real 以外。🔵"""
    with pytest.raises(ValueError, match="source"):
        _build(source=bad)


@pytest.mark.parametrize("field", ["camera_name", "base_frame"])
@pytest.mark.parametrize("bad", ["", 1, None])
def test_calibration_rejects_bad_names(field, bad):
    """D-E03: camera_name / base_frame が空・文字列でない。🟡"""
    with pytest.raises(ValueError):
        _build(**{field: bad})


def _bad_Ts():
    T = np.array(SIM_T_ROWS)
    out = {"3x4": T[:3], "nan": T.copy(), "bottom": T.copy(), "non_orthogonal": T.copy(), "mirror": T.copy()}
    out["nan"][0, 3] = np.nan
    out["bottom"][3] = [0, 0, 0, 2]
    out["non_orthogonal"][0, 1] += 1e-3
    out["mirror"] = np.diag([1.0, 1, -1, 1])
    out["strings"] = np.array([["a"] * 4] * 4)
    return out


@pytest.mark.parametrize("name", list(_bad_Ts()))
def test_calibration_rejects_bad_T(name):
    """D-E04: T が不正 (形・NaN・最下行・直交性・鏡映・文字列)。🔵"""
    with pytest.raises(ValueError):
        _build(T_base_cam=_bad_Ts()[name])


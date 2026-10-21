"""安全フィルタ純粋関数のテスト (clip / step / workspace / fresh / model) (TASK-0010, Red フェーズ)。

テストケース定義: docs/implements/so101-vlm-pickplace/TASK-0010/safety-testcases.md
信頼性: 🔵 要件・設計・実測に基づく / 🟡 妥当な推測 / 🔴 要件にない仮定
"""

import itertools

import mujoco
import numpy as np
import pytest

from safety_constants import (
    BOX,
    DT,
    EXACT_ABS,
    INF,
    LOWER_SYN,
    NAN,
    UPPER_SYN,
    VMAX,
    WS_LOWER,
    WS_UPPER,
    Z6,
    _arr,
)
from so101_vlm.robot.base import JOINT_NAMES
from so101_vlm.safety.limits import (
    check_state_fresh,
    clip_to_limits,
    is_in_workspace,
    limit_step,
    limits_from_model,
)

# --- 定数 (共通分は safety_constants.py) ---
M_STEP = VMAX * 1.0 * DT
CTRL_LOWER = np.array([-1.91986, -1.74533, -1.69, -1.65806, -2.74385, -0.17453])
CTRL_UPPER = np.array([1.91986, 1.74533, 1.69, 1.65806, 2.84121, 1.74533])
CENTER = np.array([0.25, 0.065, 0.085])


# =============================================================== TC-CL
def test_clip_inside_unchanged():
    """TC-CL-01 範囲内は完全一致 🔵"""
    q = _arr(0.5, -0.5, 0.0, 0.9, -1.9, 1.0)
    out = clip_to_limits(q, LOWER_SYN, UPPER_SYN)
    assert np.array_equal(out, q)
    assert out.dtype == np.float64


def test_clip_upper_exceeded():
    """TC-CL-02 上限超えは上限ちょうど 🔵"""
    out = clip_to_limits(_arr(1.5, 0, 0, 0, 2.5, 2.0), LOWER_SYN, UPPER_SYN)
    assert np.array_equal(out, _arr(1.0, 0, 0, 0, 2.0, 1.5))


def test_clip_lower_exceeded():
    """TC-CL-03 下限未満は下限ちょうど 🔵"""
    out = clip_to_limits(_arr(-1.5, 0, 0, 0, -3.0, -0.1), LOWER_SYN, UPPER_SYN)
    assert np.array_equal(out, _arr(-1.0, 0, 0, 0, -2.0, 0.0))


def test_clip_mixed_per_joint():
    """TC-CL-04 関節ごとに独立して丸める 🔵"""
    out = clip_to_limits(_arr(2.0, -2.0, 0.3, 1.0, -2.0, 0.0), LOWER_SYN, UPPER_SYN)
    assert np.array_equal(out, _arr(1.0, -1.0, 0.3, 1.0, -2.0, 0.0))


@pytest.mark.parametrize("bound", ["upper", "lower"])
def test_clip_boundary_unchanged(bound):
    """TC-CL-05 境界ちょうどは変えない 🟡"""
    q = (UPPER_SYN if bound == "upper" else LOWER_SYN).copy()
    assert np.array_equal(clip_to_limits(q, LOWER_SYN, UPPER_SYN), q)


def test_clip_just_outside_upper():
    """TC-CL-06 上限のすぐ外は上限と完全一致 🟡"""
    q = np.nextafter(UPPER_SYN, np.inf)
    assert np.array_equal(clip_to_limits(q, LOWER_SYN, UPPER_SYN), UPPER_SYN)


def test_clip_does_not_mutate_inputs():
    """TC-CL-07 入力不変・戻り値は別オブジェクト 🔵"""
    q, lo, up = _arr(1.5, 0, 0, 0, 2.5, 2.0), LOWER_SYN.copy(), UPPER_SYN.copy()
    saved = (q.copy(), lo.copy(), up.copy())
    out = clip_to_limits(q, lo, up)
    assert np.array_equal(q, saved[0]) and np.array_equal(lo, saved[1]) and np.array_equal(up, saved[2])
    assert not np.shares_memory(out, q)


@pytest.mark.parametrize("q", [[0.5, 0, 0, 0, 0, 1], np.array([0, 0, 0, 0, 0, 1])], ids=["list", "int"])
def test_clip_accepts_list_and_int(q):
    """TC-CL-08 list / int 配列でも float64 ndarray を返す 🟡"""
    out = clip_to_limits(q, LOWER_SYN, UPPER_SYN)
    assert isinstance(out, np.ndarray) and out.dtype == np.float64


@pytest.mark.parametrize("bad", [NAN, INF, -INF])
def test_clip_nonfinite_q_raises(bad):
    """TC-CL-09 q に NaN/inf → ValueError (q 入り) 🟡"""
    q = np.zeros(6)
    q[2] = bad
    with pytest.raises(ValueError, match=r"\bq\b"):
        clip_to_limits(q, LOWER_SYN, UPPER_SYN)


@pytest.mark.parametrize("shape", [(5,), (7,), (6, 1), ()])
def test_clip_bad_q_shape_raises(shape):
    """TC-CL-10 q の shape 違い → ValueError 🟡"""
    with pytest.raises(ValueError, match=r"\bq\b"):
        clip_to_limits(np.zeros(shape), LOWER_SYN, UPPER_SYN)


def test_clip_bad_lower_upper_raises():
    """TC-CL-11 lower の shape / upper の NaN → ValueError 🟡"""
    with pytest.raises(ValueError, match="lower"):
        clip_to_limits(Z6, np.zeros(5), UPPER_SYN)
    up = UPPER_SYN.copy()
    up[1] = NAN
    with pytest.raises(ValueError, match="upper"):
        clip_to_limits(Z6, LOWER_SYN, up)


def test_clip_bool_array_raises():
    """TC-CL-12 bool 配列は不可 🟡"""
    with pytest.raises(ValueError):
        clip_to_limits(np.ones(6, dtype=bool), LOWER_SYN, UPPER_SYN)


def test_clip_degenerate_range_raises():
    """TC-CL-13 lower == upper / lower > upper → ValueError 🟡"""
    lo, up = LOWER_SYN.copy(), UPPER_SYN.copy()
    lo[2] = up[2] = 0.0
    with pytest.raises(ValueError):
        clip_to_limits(Z6, lo, up)
    lo2 = LOWER_SYN.copy()
    lo2[0] = 2.0
    with pytest.raises(ValueError):
        clip_to_limits(Z6, lo2, UPPER_SYN)


# =============================================================== TC-ST
def _step(q_target, q_current=Z6, vmax=VMAX, scale=1.0, dt=DT):
    return limit_step(q_current, np.asarray(q_target, dtype=np.float64), vmax, scale, dt)


def test_step_within_limit_unchanged():
    """TC-ST-01 全関節 |Δ| <= m なら完全一致 🔵"""
    q = _arr(0.01, -0.005, 0.0, 0.015, 0.0, 0.02)
    assert np.array_equal(_step(q), q)


def test_step_uniform_scaling_keeps_direction():
    """TC-ST-02 一様スケーリングで方向を保つ 🔵/🟡"""
    q = _arr(1.0, -0.5, 0.25, 0.0, 2.0, -0.1)
    out = _step(q)
    step = out - Z6
    assert step == pytest.approx(_arr(0.01, -0.005, 0.0025, 0.0, 0.02, -0.001), abs=EXACT_ABS)
    assert np.all(np.abs(step) <= M_STEP)  # 【確認内容】: 許容差なしで上限以下
    assert np.max(np.abs(step) / M_STEP) == pytest.approx(1.0, abs=EXACT_ABS)
    cos = step @ q / (np.linalg.norm(step) * np.linalg.norm(q))
    assert cos == pytest.approx(1.0, abs=EXACT_ABS)
    nz = q != 0
    assert step[nz] / q[nz] == pytest.approx(np.full(nz.sum(), 0.01), abs=EXACT_ABS)
    assert step[3] == 0.0


def test_step_negative_direction():
    """TC-ST-03 負の方向 🟡"""
    q = -_arr(1.0, -0.5, 0.25, 0.0, 2.0, -0.1)
    step = _step(q)
    assert step == pytest.approx(_arr(-0.01, 0.005, -0.0025, 0.0, -0.02, 0.001), abs=EXACT_ABS)
    cos = step @ q / (np.linalg.norm(step) * np.linalg.norm(q))
    assert cos == pytest.approx(1.0, abs=EXACT_ABS)
    assert int(np.argmax(np.abs(step) / M_STEP)) == 4


def test_step_scale_03():
    """TC-ST-04 scale 0.3 で変化量がちょうど 30% 🔵"""
    q = _arr(1.0, -0.5, 0.25, 0.0, 2.0, -0.1)
    s1, s03 = _step(q, scale=1.0), _step(q, scale=0.3)
    assert np.max(np.abs(s03)) == pytest.approx(0.006, abs=EXACT_ABS)
    assert np.all(np.abs(s03) <= VMAX * 0.3 * DT)
    assert s03 == pytest.approx(0.3 * s1, abs=EXACT_ABS)


def test_step_exactly_m_not_limited():
    """TC-ST-05 |Δ| = m ちょうどは制限しない 🟡"""
    q = _arr(0.02, -0.02, 0, 0, 0, 0)
    assert np.array_equal(_step(q), q)


def test_step_just_over_m_is_limited():
    """TC-ST-06 m をわずかに超えると制限される 🟡"""
    q = _arr(float(np.nextafter(0.02, 1)), 0, 0, 0, 0, 0)
    out = _step(q)
    assert abs(out[0]) <= 0.02
    assert not np.array_equal(out, q)


def test_step_zero_move_no_warning():
    """TC-ST-07 Δ = 0 でも 0 除算しない 🟡"""
    q = _arr(0.3, -0.2, 0.1, 0.0, 0.5, 0.7)
    with np.errstate(all="raise"):
        out = limit_step(q, q.copy(), VMAX, 1.0, DT)
    assert np.array_equal(out, q)


def test_step_per_joint_vmax_tightest_decides():
    """TC-ST-08 最も厳しい関節で α が決まる 🟡"""
    vmax = _arr(1.0, 0.5, 2.0, 1.0, 1.0, 4.0)
    out = _step(np.full(6, 0.5), vmax=vmax)
    assert out == pytest.approx(np.full(6, 0.01), abs=EXACT_ABS)
    assert np.all(np.abs(out) <= vmax * DT)


def test_step_per_joint_vmax_unlimited():
    """TC-ST-09 関節ごとの上限以下なら制限なし 🟡"""
    vmax = _arr(1.0, 0.5, 2.0, 1.0, 1.0, 4.0)
    q = _arr(0.004, 0, 0.03, 0, 0, 0)
    assert np.array_equal(_step(q, vmax=vmax), q)


def test_step_large_target():
    """TC-ST-10 大きな目標でも有限で m に収まる 🟡"""
    out = _step(_arr(100.0, -100.0, 0, 0, 0, 0))
    assert out == pytest.approx(_arr(0.02, -0.02, 0, 0, 0, 0), abs=EXACT_ABS)
    assert np.all(np.isfinite(out))


def test_step_nonzero_base():
    """TC-ST-11 基準が 0 でない 🟡"""
    qc = _arr(0.5, -0.3, 0.2, 0.1, -1.0, 0.8)
    out = limit_step(qc, qc + _arr(1.0, 0, 0, 0, 0, -0.5), VMAX, 1.0, DT)
    assert out - qc == pytest.approx(_arr(0.02, 0, 0, 0, 0, -0.01), abs=EXACT_ABS)


def test_step_immutable_and_deterministic():
    """TC-ST-12 入力不変・決定性 🔵"""
    qc, qt = _arr(0.1, 0, 0, 0, 0, 0), _arr(1.0, 0.5, 0, 0, 0, 0)
    saved = (qc.copy(), qt.copy())
    a = limit_step(qc, qt, VMAX, 1.0, DT)
    b = limit_step(qc, qt, VMAX, 1.0, DT)
    assert np.array_equal(qc, saved[0]) and np.array_equal(qt, saved[1])
    assert np.array_equal(a, b)
    assert not np.shares_memory(a, qt) and not np.shares_memory(a, qc)


@pytest.mark.parametrize("scale", [0.0, -0.1, 1.01, NAN, INF, True])
def test_step_invalid_scale_raises(scale):
    """TC-ST-13 不正な scale → ValueError 🟡"""
    with pytest.raises(ValueError, match="scale"):
        _step(_arr(0.1, 0, 0, 0, 0, 0), scale=scale)


@pytest.mark.parametrize("scale", [1.0, 1e-6])
def test_step_boundary_scale(scale):
    """TC-ST-14 scale 境界 🟡"""
    out = _step(_arr(1.0, 0, 0, 0, 0, 0), scale=scale)
    if scale == 1e-6:
        assert np.max(np.abs(out)) <= 2e-8


@pytest.mark.parametrize("dt", [0.0, -0.02, NAN, True])
def test_step_invalid_dt_raises(dt):
    """TC-ST-15 不正な dt → ValueError 🟡"""
    with pytest.raises(ValueError, match="dt"):
        _step(_arr(0.1, 0, 0, 0, 0, 0), dt=dt)


@pytest.mark.parametrize(
    "vmax",
    [1.0, np.ones(5), _arr(0.0, 1, 1, 1, 1, 1), _arr(-1.0, 1, 1, 1, 1, 1), _arr(NAN, 1, 1, 1, 1, 1),
     np.ones(6, dtype=bool)],
    ids=["scalar", "shape5", "zero", "neg", "nan", "bool"],
)
def test_step_invalid_vmax_raises(vmax):
    """TC-ST-16 不正な vmax → ValueError 🟡"""
    with pytest.raises(ValueError, match="vmax"):
        _step(_arr(0.1, 0, 0, 0, 0, 0), vmax=vmax)


@pytest.mark.parametrize("name", ["q_current", "q_target"])
@pytest.mark.parametrize(
    "bad", [_arr(NAN, 0, 0, 0, 0, 0), _arr(INF, 0, 0, 0, 0, 0), np.zeros(5), np.ones(6, dtype=bool)],
    ids=["nan", "inf", "shape5", "bool"],
)
def test_step_invalid_joint_arrays_raise(name, bad):
    """TC-ST-17 関節配列の不正 → ValueError (引数名入り) 🟡"""
    kwargs = {"q_current": Z6, "q_target": _arr(0.1, 0, 0, 0, 0, 0)}
    kwargs[name] = bad
    with pytest.raises(ValueError, match=name):
        limit_step(kwargs["q_current"], kwargs["q_target"], VMAX, 1.0, DT)


# =============================================================== TC-WS
def _face_points(offset=0.0):
    pts = []
    for axis in range(3):
        lo, hi = CENTER.copy(), CENTER.copy()
        lo[axis] = WS_LOWER[axis] - offset
        hi[axis] = WS_UPPER[axis] + offset
        pts += [lo, hi]
    return pts


def test_workspace_inside():
    """TC-WS-01 内側は True (bool 型) 🔵"""
    for p in (CENTER, (0.25, 0.0, 0.015), (0.20, 0.19, 0.14)):
        res = is_in_workspace(p, BOX)
        assert res is True


@pytest.mark.parametrize("idx", range(6))
def test_workspace_faces_inclusive(idx):
    """TC-WS-02 6 面の上は True (閉区間) 🟡"""
    assert is_in_workspace(_face_points()[idx], BOX) is True


def test_workspace_corners_inclusive():
    """TC-WS-03 8 隅すべて True 🟡"""
    for c in itertools.product(*zip(WS_LOWER, WS_UPPER)):
        assert is_in_workspace(np.array(c), BOX) is True
    assert is_in_workspace(np.array(BOX.lower), BOX) and is_in_workspace(np.array(BOX.upper), BOX)


@pytest.mark.parametrize("idx", range(6))
def test_workspace_1mm_outside(idx):
    """TC-WS-04 6 面から 1 mm 外は False 🟡"""
    assert is_in_workspace(_face_points(0.001)[idx], BOX) is False


@pytest.mark.parametrize("idx", range(6))
def test_workspace_just_outside(idx):
    """TC-WS-05 nextafter で外は False 🟡"""
    axis, upper = divmod(idx, 2)[0], idx % 2 == 1
    p = CENTER.copy()
    p[axis] = np.nextafter(WS_UPPER[axis], np.inf) if upper else np.nextafter(WS_LOWER[axis], -np.inf)
    assert is_in_workspace(p, BOX) is False


@pytest.mark.parametrize(
    "p",
    [(0, 0, 0), (0, 0, 0.10), (0.10, 0, 0.05), (-0.10, 0, 0.05), (0.25, 0, -0.001),
     (0.391, 0, 0.227), (0.25, 0, 0.18), (0.40, 0, 0.05)],
    ids=["origin", "base_top", "base_front", "behind", "below_floor", "q0_hand", "too_high", "too_far"],
)
def test_workspace_outside_representatives(p):
    """TC-WS-06 外側の代表点 🔵"""
    assert is_in_workspace(p, BOX) is False


def test_workspace_task_points(task_points):
    """TC-WS-07 タスク点 172 点すべて True 🔵"""
    assert len(task_points) == 172
    assert all(is_in_workspace(p, BOX) for p in task_points)


@pytest.mark.parametrize("p", [(0.25, 0, 0.0023), (0.3526, 0, 0.10), (0.20, -0.1042, 0.05)])
def test_workspace_survey_points(p):
    """TC-WS-08 survey の追加点 🔵"""
    assert is_in_workspace(p, BOX) is True


@pytest.mark.parametrize("axis", range(3))
@pytest.mark.parametrize("bad", [NAN, INF, -INF])
def test_workspace_nonfinite_raises(axis, bad):
    """TC-WS-09 NaN/inf → ValueError (p_base 入り) 🟡"""
    p = CENTER.copy()
    p[axis] = bad
    with pytest.raises(ValueError, match="p_base"):
        is_in_workspace(p, BOX)


@pytest.mark.parametrize("p", [np.zeros(2), np.zeros(4), np.zeros((3, 1)), np.ones(3, dtype=bool)],
                         ids=["2", "4", "3x1", "bool"])
def test_workspace_bad_shape_raises(p):
    """TC-WS-10 shape / bool の不正 → ValueError 🟡"""
    with pytest.raises(ValueError):
        is_in_workspace(p, BOX)


def test_workspace_accepts_tuple_and_list():
    """TC-WS-11 tuple / list を受け付ける 🟡"""
    assert is_in_workspace(tuple(CENTER), BOX) is True
    assert is_in_workspace(list(CENTER), BOX) is True


def test_workspace_does_not_mutate():
    """TC-WS-12 入力不変 🔵"""
    p = CENTER.copy()
    is_in_workspace(p, BOX)
    assert np.array_equal(p, CENTER)


# =============================================================== TC-FR
def test_fresh_normal():
    """TC-FR-01 新鮮 → True (bool 型) 🔵"""
    assert check_state_fresh(10.0, 10.05, 0.2) is True


@pytest.mark.parametrize("last,now,timeout", [(0.0, 0.2, 0.2), (1.0, 1.25, 0.25), (12345.0, 12345.125, 0.125)])
def test_fresh_exactly_timeout(last, now, timeout):
    """TC-FR-02 ちょうど timeout は新鮮 🟡"""
    assert check_state_fresh(last, now, timeout) is True


def test_fresh_zero_elapsed():
    """TC-FR-03 経過 0 は新鮮 🟡"""
    assert check_state_fresh(3.0, 3.0, 0.2) is True


def test_fresh_stale():
    """TC-FR-04 timeout 超過 → False 🔵"""
    assert check_state_fresh(0.0, 0.5, 0.2) is False


def test_fresh_just_over():
    """TC-FR-05 わずかに超えると False 🟡"""
    assert check_state_fresh(0.0, float(np.nextafter(0.2, 1.0)), 0.2) is False


def test_fresh_clock_backwards():
    """TC-FR-06 時計の逆行 → False 🟡"""
    assert check_state_fresh(5.0, 4.999, 0.2) is False


@pytest.mark.parametrize("last,now", [(NAN, 1.0), (1.0, NAN), (INF, 1.0), (1.0, INF), (-INF, 1.0), (1.0, -INF)])
def test_fresh_nonfinite_time_is_stale(last, now):
    """TC-FR-07 NaN/inf の時刻 → False (例外にしない) 🟡"""
    assert check_state_fresh(last, now, 0.2) is False


def test_fresh_large_monotonic_values():
    """TC-FR-08 大きな単調時計の値 🟡"""
    assert check_state_fresh(1.0e6, 1.0e6 + 0.125, 0.2) is True
    assert check_state_fresh(1.0e6, 1.0e6 + 0.25, 0.2) is False


@pytest.mark.parametrize("timeout", [0.0, -0.1, NAN, INF, True])
def test_fresh_invalid_timeout_raises(timeout):
    """TC-FR-09 不正な timeout → ValueError 🟡"""
    with pytest.raises(ValueError, match="timeout"):
        check_state_fresh(0.0, 0.1, timeout)


@pytest.mark.parametrize("last,now", [(True, 1.0), (0.0, False)])
def test_fresh_bool_time_raises(last, now):
    """TC-FR-10 bool の時刻 → ValueError 🟡"""
    with pytest.raises(ValueError):
        check_state_fresh(last, now, 0.2)


# =============================================================== TC-MD
def test_limits_from_model_values(scene_model):
    """TC-MD-01 実モデルの ctrlrange 🔵"""
    lower, upper = limits_from_model(scene_model)
    assert lower.shape == (6,) and lower.dtype == np.float64
    assert lower == pytest.approx(CTRL_LOWER, abs=1e-5)
    assert upper == pytest.approx(CTRL_UPPER, abs=1e-5)


def test_limits_from_model_by_name(scene_model):
    """TC-MD-02 名前で引いた actuator / joint と一致 🔵"""
    lower, upper = limits_from_model(scene_model)
    for i, name in enumerate(JOINT_NAMES):
        a = mujoco.mj_name2id(scene_model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        j = mujoco.mj_name2id(scene_model, mujoco.mjtObj.mjOBJ_JOINT, name)
        assert lower[i] == scene_model.actuator_ctrlrange[a, 0]
        assert upper[i] == scene_model.actuator_ctrlrange[a, 1]
        assert lower[i] == pytest.approx(scene_model.jnt_range[j, 0], abs=1e-5)
        assert upper[i] == pytest.approx(scene_model.jnt_range[j, 1], abs=1e-5)


def test_limits_from_pickplace_scene(pickplace_model):
    """TC-MD-03 pick&place シーンでも同じ値 🔵"""
    lower, upper = limits_from_model(pickplace_model)
    assert lower == pytest.approx(CTRL_LOWER, abs=1e-5)
    assert upper == pytest.approx(CTRL_UPPER, abs=1e-5)


def test_limits_from_model_returns_copy(scene_model):
    """TC-MD-04 戻り値はモデルと共有しない 🔵"""
    lower, upper = limits_from_model(scene_model)
    assert not np.shares_memory(lower, scene_model.actuator_ctrlrange)
    assert not np.shares_memory(upper, scene_model.actuator_ctrlrange)
    before = scene_model.actuator_ctrlrange.copy()
    if lower.flags.writeable:
        lower[0] = 99.0
    again, _ = limits_from_model(scene_model)
    assert again[0] == pytest.approx(CTRL_LOWER[0], abs=1e-5)
    assert np.array_equal(scene_model.actuator_ctrlrange, before)


_TINY_ORDER = ("elbow_flex", "wrist_flex", "wrist_roll", "gripper", "shoulder_pan", "shoulder_lift")


def _tiny_mjcf(drop_actuator=None):
    bodies = "".join(
        f'<body name="b_{n}"><joint name="{n}" type="hinge" axis="0 0 1"/>'
        f'<geom type="sphere" size="0.1" mass="1"/></body>'
        for n in reversed(JOINT_NAMES)
    )
    acts = "".join(
        f'<position name="{n}" joint="{n}" kp="50" ctrllimited="true" '
        f'ctrlrange="-{JOINT_NAMES.index(n) + 1} {JOINT_NAMES.index(n) + 1}"/>'
        for n in _TINY_ORDER
        if n != drop_actuator
    )
    return (
        '<mujoco><option timestep="0.002" gravity="0 0 0"/>'
        f"<worldbody>{bodies}</worldbody><actuator>{acts}</actuator></mujoco>"
    )


def test_limits_from_model_name_order(tmp_path):
    """TC-MD-05 actuator 宣言順に依存せず JOINT_NAMES 順 🟡"""
    path = tmp_path / "tiny.xml"
    path.write_text(_tiny_mjcf(), encoding="utf-8")
    lower, upper = limits_from_model(mujoco.MjModel.from_xml_path(str(path)))
    assert np.array_equal(lower, -np.arange(1.0, 7.0))
    assert np.array_equal(upper, np.arange(1.0, 7.0))


def test_limits_from_model_missing_actuator(tmp_path):
    """TC-MD-06 actuator が無ければ名前入り ValueError 🟡"""
    path = tmp_path / "tiny.xml"
    path.write_text(_tiny_mjcf(drop_actuator="wrist_roll"), encoding="utf-8")
    with pytest.raises(ValueError, match="wrist_roll"):
        limits_from_model(mujoco.MjModel.from_xml_path(str(path)))

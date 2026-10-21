"""軌道の純粋層 (時間配分・補間・JointTrajectory・指令列の検査・Observation/Goal) のテスト (TASK-0012, Red)。

テストケース定義: docs/implements/so101-vlm-pickplace/TASK-0012/trajectory-testcases.md 1 節 (TC-TJ-*)
MuJoCo は使わない。対象モジュール (まだ存在しない):
  so101_vlm.policy.base        : Observation, Goal, Policy
  so101_vlm.policy.trajectory  : segment_steps, hold_steps, interpolate_segment, JointWaypoint,
                                 build_joint_trajectory, check_command_sequence,
                                 find_clearance_violation, JointTrajectory, TrajectoryError
信頼性: 🔵 要件・実測に基づく / 🟡 妥当な推測 / 🔴 要件にない仮定
"""

import dataclasses

import numpy as np
import pytest

from so101_vlm.policy.base import Goal, Observation
from so101_vlm.policy.trajectory import (
    JointTrajectory,
    JointWaypoint,
    TrajectoryError,
    build_joint_trajectory,
    check_command_sequence,
    find_clearance_violation,
    hold_steps,
    interpolate_segment,
    segment_steps,
)
from so101_vlm.robot.base import RobotState
from trajectory_constants import (
    CL,
    CLOSE_HOLD_S,
    DT,
    FACTOR,
    GR,
    LI,
    OP,
    OPEN_HOLD_S,
    PG,
    PICK_CENTER,
    PL,
    PLACE_BOX,
    PP,
    RT,
    SCALE,
    SEGMENTS,
    STEP_UNIT,
    STEPS_SCALE_03,
    STEPS_SCALE_1,
    VMAX,
    Z6,
    time_kwargs,
    unit_delta,
)

NAN, INF = float("nan"), float("inf")
EXACT_ABS = 1e-12
LOWER = np.array([-2.0, -2.0, -2.0, -2.0, -2.0, -0.2])
UPPER = np.array([2.0, 2.0, 2.0, 2.0, 2.0, 1.8])
SYNTHETIC_WAYPOINT_Q = {
    "pre_grasp": PG, "grasp": GR, "close": CL, "close_hold": CL, "lift": LI,
    "pre_place": PP, "place": PL, "open": OP, "open_hold": OP, "retreat": RT,
}  # fmt: skip


def _waypoints():
    """0.2 節の 10 区間分の経由点 (保持区間は hold_s 付き)。"""
    return [
        JointWaypoint("pre_grasp", PG),
        JointWaypoint("grasp", GR),
        JointWaypoint("close", CL),
        JointWaypoint("close_hold", CL, hold_s=CLOSE_HOLD_S),
        JointWaypoint("lift", LI),
        JointWaypoint("pre_place", PP),
        JointWaypoint("place", PL),
        JointWaypoint("open", OP),
        JointWaypoint("open_hold", OP, hold_s=OPEN_HOLD_S),
        JointWaypoint("retreat", RT),
    ]


def _build(scale=SCALE, q_start=None, waypoints=None):
    return build_joint_trajectory(
        Z6.copy() if q_start is None else q_start,
        _waypoints() if waypoints is None else waypoints,
        **time_kwargs(scale),
    )


def _rows(traj, name):
    return [k for k, label in enumerate(traj.labels) if label == name]


def _steps(delta, **over):
    kwargs = dict(vmax=VMAX, speed_scale=SCALE, control_dt=DT, safety_factor=FACTOR, min_segment_s=0.2)
    kwargs.update(over)
    return segment_steps(Z6, Z6 + delta, **kwargs)


@pytest.fixture(scope="module")
def synthetic():
    # 【テスト前準備】: 0.2 節の合成軌道 (scale 1.0) を 1 回だけ作って共有する
    return _build()


def _jt(times, q, labels, segments=("start", "a"), metadata=None):
    return JointTrajectory(
        times=times, q=q, labels=labels, segments=segments, metadata={} if metadata is None else metadata
    )


# =============================================================== 1.1 正常系
def test_segment_steps_large_move():
    """TC-TJ-01 刻み数: 大きな移動 🔵"""
    # Given: Δ = (1.0, 0, ...)  When: segment_steps  Then: ceil(1.0 / 0.0198) = 51
    assert _steps(unit_delta(1.0)) == 51  # 【確認内容】最大の関節の移動量で刻み数が決まる


def test_segment_steps_driven_by_largest_joint():
    """TC-TJ-02 刻み数: 最も大きく動く関節で決まる 🔵"""
    assert _steps(unit_delta(0.1, -0.5, 0.2)) == 26  # 【確認内容】0.5 / 0.0198 = 25.25 → 26


def test_segment_steps_per_joint_vmax():
    """TC-TJ-03 刻み数: 関節ごとの vmax 🔵"""
    vmax = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 0.5])
    n = _steps(unit_delta(0, 0, 0, 0, 0, 0.45), vmax=vmax)
    assert n == 46  # 【確認内容】0.45 / (0.5 x 0.02 x 0.99) = 45.45 → 46


def test_segment_steps_negative_direction_same():
    """TC-TJ-04 刻み数: 負の向きも同じ 🔵"""
    assert _steps(unit_delta(-0.45)) == 23  # 【確認内容】絶対値で数える (0.45 / 0.0198 = 22.7 → 23)


def test_interpolate_segment_endpoints_and_spacing():
    """TC-TJ-05 補間: 端点と等間隔 🔵(式)/ 🟡(完全一致)"""
    n = 61
    q = interpolate_segment(Z6, PG, n)
    assert q.shape == (n, 6)  # 【確認内容】k = 1..n の n 行
    assert np.array_equal(q[-1], PG)  # 【確認内容】最後の行は終点と完全一致 (1 ulp もずらさない)
    k = np.arange(1, n)[:, None]
    expected = Z6 + (k / n) * (PG - Z6)
    np.testing.assert_allclose(q[:-1], expected, atol=1e-12)  # 【確認内容】等間隔の線形補間


@pytest.mark.parametrize(
    "delta",
    [unit_delta(1.0), unit_delta(0.1, -0.5, 0.2), unit_delta(-0.45), unit_delta(0, 0, 0, 0, 0, 0.45)],
    ids=["tj01", "tj02", "tj04", "tj03_gripper"],
)
def test_interpolated_step_within_99_percent(delta):
    """TC-TJ-06 補間: 1 刻みの変化が上限の 99% 以下 🔵"""
    n = _steps(delta)
    q = interpolate_segment(Z6, Z6 + delta, n)
    steps = np.abs(np.diff(np.vstack([Z6, q]), axis=0))
    assert steps.max() <= 0.99 * 1.0 * 1.0 * 0.02 + 1e-12  # 【確認内容】係数 0.99 で上限の 99% 以下


def test_hold_steps():
    """TC-TJ-07 保持の刻み数 🔵"""
    assert hold_steps(0.5, 0.02) == 25  # 【確認内容】round(0.5 / 0.02)
    assert hold_steps(0.3, 0.02) == 15  # 【確認内容】浮動小数の割り算でも 15


def test_synthetic_trajectory_segments_and_lengths(synthetic):
    """TC-TJ-08 合成の軌道: 区間の順と長さ 🔵"""
    assert synthetic.segments == SEGMENTS  # 【確認内容】start を先頭にした 11 区間の順
    for name, expected in STEPS_SCALE_1.items():
        assert len(_rows(synthetic, name)) == expected, name  # 【確認内容】0.2 節の表どおりの刻み数
    assert synthetic.q.shape == (228, 6)  # 【確認内容】合計サンプル数 (start 含む)


def test_synthetic_trajectory_times(synthetic):
    """TC-TJ-09 時刻 🔵"""
    t = synthetic.times
    assert t[0] == 0.0  # 【確認内容】先頭は 0
    np.testing.assert_allclose(t, np.arange(228) * DT, atol=EXACT_ABS)  # 【確認内容】k x control_dt
    assert np.all(np.diff(t) > 0)  # 【確認内容】厳密に単調増加
    assert synthetic.metadata["duration_s"] == pytest.approx(4.54, abs=1e-9)  # 【確認内容】所要時間


def test_synthetic_trajectory_labels(synthetic):
    """TC-TJ-10 ラベル 🔵"""
    labels = synthetic.labels
    assert len(labels) == 228 and labels[0] == "start"  # 【確認内容】長さ N、先頭は start
    order = [SEGMENTS.index(label) for label in labels]
    assert all(b >= a for a, b in zip(order, order[1:]))  # 【確認内容】区間の順に並ぶ
    assert set(labels) == set(SEGMENTS)  # 【確認内容】各区間に 1 サンプル以上


def test_start_row_is_copy_of_q_start():
    """TC-TJ-11 開始値 🔵"""
    q_start = np.array([0.1, -0.2, 0.3, 0.0, 0.05, 0.4])
    traj = _build(q_start=q_start)
    assert np.array_equal(traj.q[0], q_start)  # 【確認内容】q[0] は開始値と完全一致
    before = traj.q[0].copy()
    q_start[0] = 9.9
    assert np.array_equal(traj.q[0], before)  # 【確認内容】コピーである (後で書き換えても変わらない)


def test_synthetic_command_step_within_limit(synthetic):
    """TC-TJ-12 指令列の速度 (前回指令が基準) 🔵"""
    steps = np.abs(np.diff(synthetic.q, axis=0))  # q[0] -> q[1] を含む
    assert np.all(steps <= VMAX * SCALE * DT)  # 【確認内容】全要素が 0.02 以下
    assert steps.max() <= STEP_UNIT + EXACT_ABS  # 【確認内容】最大は 0.0198 以下


def test_arm_stops_during_hold_and_gripper_segments(synthetic):
    """TC-TJ-13 保持・開閉の区間で腕が止まる 🔵"""
    q = synthetic.q
    for name, ref in (("close", GR), ("close_hold", GR), ("open", PL), ("open_hold", PL)):
        for k in _rows(synthetic, name):
            assert np.array_equal(q[k, :5], ref[:5]), (name, k)  # 【確認内容】腕の 5 関節は直前の経由点のまま
    for name in ("close_hold", "open_hold"):
        rows = q[_rows(synthetic, name)]
        assert np.all(rows == rows[0])  # 【確認内容】保持区間は全行が同じ値


def test_segment_end_rows_equal_waypoints(synthetic):
    """TC-TJ-14 経由点の終点 🔵"""
    for name, ref in SYNTHETIC_WAYPOINT_Q.items():
        last = _rows(synthetic, name)[-1]
        assert np.array_equal(synthetic.q[last], ref), name  # 【確認内容】各区間の最後の行が経由点と完全一致


def test_scale_03_step_counts():
    """TC-TJ-15 速度倍率 0.3 🔵"""
    traj = _build(scale=0.3)
    for name, expected in STEPS_SCALE_03.items():
        assert len(_rows(traj, name)) == expected, name  # 【確認内容】0.2 節の表どおり (保持は 25 / 15)
    assert traj.q.shape[0] == 650  # 【確認内容】合計サンプル数
    assert np.abs(np.diff(traj.q, axis=0)).max() <= 1.0 * 0.3 * 0.02 + EXACT_ABS  # 【確認内容】連続差


def test_scale_03_is_3_3_times_slower():
    """TC-TJ-16 速度倍率 0.3 で 3.3 倍以上 🔵"""
    n1 = _steps(unit_delta(1.0), speed_scale=1.0, min_segment_s=0.02)
    n03 = _steps(unit_delta(1.0), speed_scale=0.3, min_segment_s=0.02)
    assert (n1, n03) == (51, 169)  # 【確認内容】51 → 169
    assert n03 / n1 >= 3.3  # 【確認内容】3.3 倍以上


def test_joint_trajectory_is_immutable(synthetic):
    """TC-TJ-17 JointTrajectory は変更できない 🔵"""
    with pytest.raises(ValueError):
        synthetic.q[0, 0] = 1.0  # 【確認内容】q は読み取り専用
    with pytest.raises(ValueError):
        synthetic.times[0] = 1.0  # 【確認内容】times は読み取り専用
    with pytest.raises(dataclasses.FrozenInstanceError):
        synthetic.labels = ()  # type: ignore[misc]  # 【確認内容】frozen dataclass
    with pytest.raises(TypeError):
        synthetic.metadata["x"] = 1  # type: ignore[index]  # 【確認内容】metadata は MappingProxyType


def test_build_does_not_mutate_inputs():
    """TC-TJ-18 入力を変更しない 🔵"""
    q_start = np.zeros(6)
    waypoints = [JointWaypoint(w.name, np.array(w.q, copy=True), hold_s=w.hold_s) for w in _waypoints()]
    before_start = q_start.copy()
    before_q = [np.array(w.q, copy=True) for w in waypoints]
    build_joint_trajectory(q_start, waypoints, **time_kwargs())
    assert np.array_equal(q_start, before_start)  # 【確認内容】開始値が変わらない
    for w, before in zip(waypoints, before_q, strict=True):
        assert np.array_equal(w.q, before), w.name  # 【確認内容】経由点が変わらない


def test_build_is_deterministic():
    """TC-TJ-19 決定的 🔵"""
    a, b = _build(), _build()
    assert np.array_equal(a.times, b.times) and np.array_equal(a.q, b.q)  # 【確認内容】数値が完全一致
    assert a.labels == b.labels and a.segments == b.segments  # 【確認内容】ラベル・区間も一致


def test_check_command_sequence_accepts_valid(synthetic):
    """TC-TJ-20 指令列の検査: 正常 🔵"""
    check_command_sequence(
        synthetic.q, synthetic.labels, lower=LOWER, upper=UPPER, vmax=VMAX, speed_scale=SCALE, control_dt=DT
    )  # 【確認内容】例外が出ない


def test_clearance_no_violation():
    """TC-TJ-21 すき間: 違反なし 🟡"""
    result = find_clearance_violation(np.array([0.010, 0.005, 0.0022]), ("a", "b", "c"), 0.001)
    assert result is None  # 【確認内容】全距離がしきい値以上


def test_clearance_returns_minimum_sample():
    """TC-TJ-22 すき間: 最小のサンプルを返す 🟡"""
    d = np.array([0.010, -0.003, -0.010, 0.004])
    labels = ("pre_grasp", "grasp", "grasp", "close")
    v = find_clearance_violation(d, labels, 0.002)
    assert v is not None
    assert (v.index, v.label) == (2, "grasp")  # 【確認内容】最も小さいサンプルの位置と区間名
    assert v.value == pytest.approx(-0.010)  # 【確認内容】その距離


def test_observation_from_state():
    """TC-TJ-23 Observation の作成 🟡"""
    q = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    obs = Observation.from_state(RobotState(q=q, dq=np.zeros(6), timestamp=1.5))
    assert np.array_equal(obs.q, q) and obs.q is not q  # 【確認内容】値が等しいコピー
    assert not obs.q.flags.writeable  # 【確認内容】読み取り専用
    assert obs.timestamp == 1.5  # 【確認内容】時刻を引き継ぐ


def test_goal_construction():
    """TC-TJ-24 Goal の作成 🟡"""
    pick, place = list(PICK_CENTER), list(PLACE_BOX)
    goal = Goal(tuple(pick), tuple(place))
    for arr in (goal.pick_target_base, goal.place_target_base):
        assert arr.shape == (3,) and arr.dtype == np.float64  # 【確認内容】(3,) の float64
        assert not arr.flags.writeable  # 【確認内容】読み取り専用
    goal2 = Goal(pick, place)
    pick[0] = 9.0
    assert goal2.pick_target_base[0] == PICK_CENTER[0]  # 【確認内容】元の list を変えても影響しない
    assert goal.instruction == ""  # 【確認内容】instruction の既定は空文字


def test_trajectory_error_attributes():
    """TC-TJ-25 TrajectoryError の属性 🟡"""
    err = TrajectoryError("self_collision", waypoint="grasp", detail={"min": -0.01})
    assert isinstance(err, RuntimeError)  # 【確認内容】RuntimeError の派生 (IkError と同じ)
    assert (err.reason, err.waypoint) == ("self_collision", "grasp")
    assert err.detail["min"] == -0.01
    with pytest.raises(TypeError):
        err.detail["x"] = 1  # type: ignore[index]  # 【確認内容】detail は変更できない
    assert "grasp" in str(err)  # 【確認内容】メッセージに経由点の名前を含む


# =============================================================== 1.2 異常系
def test_joint_trajectory_rejects_length_mismatch():
    """TC-TJ-E01 JointTrajectory: 長さが合わない 🟡"""
    with pytest.raises(ValueError):
        _jt(np.arange(5) * DT, np.zeros((4, 6)), ("start",) * 5)


def test_joint_trajectory_rejects_bad_q_shape():
    """TC-TJ-E02 JointTrajectory: q の shape 🟡"""
    with pytest.raises(ValueError):
        _jt(np.arange(5) * DT, np.zeros((5, 5)), ("start",) * 5)


def test_joint_trajectory_rejects_single_sample():
    """TC-TJ-E03 JointTrajectory: N < 2 🟡"""
    with pytest.raises(ValueError):
        _jt(np.zeros(1), np.zeros((1, 6)), ("start",))


def test_joint_trajectory_rejects_non_increasing_times():
    """TC-TJ-E04 JointTrajectory: 時刻が増えない 🟡"""
    with pytest.raises(ValueError):
        _jt(np.array([0.0, 0.02, 0.02]), np.zeros((3, 6)), ("start",) * 3)


@pytest.mark.parametrize("bad", ["q_nan", "times_inf"])
def test_joint_trajectory_rejects_non_finite(bad):
    """TC-TJ-E05 JointTrajectory: 有限でない値 🟡"""
    times, q = np.arange(3) * DT, np.zeros((3, 6))
    if bad == "q_nan":
        q[1, 2] = NAN
    else:
        times[2] = INF
    with pytest.raises(ValueError):
        _jt(times, q, ("start",) * 3)


def _sequence(n=4):
    return np.zeros((n, 6)), tuple(f"s{k}" for k in range(n))


def _check(q, labels):
    check_command_sequence(q, labels, lower=LOWER, upper=UPPER, vmax=VMAX, speed_scale=SCALE, control_dt=DT)


def test_check_command_sequence_speed_violation():
    """TC-TJ-E06 指令列: 速度の超過 🟡"""
    q, labels = _sequence()
    q[1, 0] = 0.0201  # 上限 0.02 を 0.5% 超える
    with pytest.raises(TrajectoryError) as info:
        _check(q, labels)
    assert info.value.reason == "limit_violation"  # 【確認内容】理由
    assert info.value.waypoint == labels[1]  # 【確認内容】違反した行のラベル


def test_check_command_sequence_range_violation():
    """TC-TJ-E07 指令列: 可動域の外 🟡"""
    q, labels = _sequence()
    q[1:, 3] = UPPER[3]
    q[2, 3] = UPPER[3] + 1e-6
    with pytest.raises(TrajectoryError) as info:
        _check(q, labels)
    assert info.value.reason == "limit_violation"  # 【確認内容】可動域の外は limit_violation


def test_check_command_sequence_ignores_range_of_first_row():
    """TC-TJ-E08 指令列: q[0] は可動域を検査しない 🟡"""
    q, labels = _sequence()
    q[0, 0] = UPPER[0] + 0.01  # 開始時の計測値だけが範囲外
    q[1:, 0] = UPPER[0]
    _check(q, labels)  # 【確認内容】例外が出ない (検査は q[1:])


def test_hold_waypoint_must_repeat_previous_q():
    """TC-TJ-E09 保持区間の q が直前と違う 🔴"""
    bad = CL.copy()
    bad[0] += 0.01
    waypoints = [
        JointWaypoint("pre_grasp", PG),
        JointWaypoint("grasp", GR),
        JointWaypoint("close", CL),
        JointWaypoint("close_hold", bad, hold_s=CLOSE_HOLD_S),
    ]
    with pytest.raises(ValueError):
        _build(waypoints=waypoints)  # 【確認内容】保持の q は直前の終点と同じでなければならない


@pytest.mark.parametrize(
    "q",
    [np.zeros(5), np.array([0, 0, NAN, 0, 0, 0.0]), np.array([0, 0, INF, 0, 0, 0.0]), np.array([True] * 6)],
    ids=["shape5", "nan", "inf", "bool"],
)
def test_observation_rejects_invalid_q(q):
    """TC-TJ-E10 Observation の検証 🟡"""
    with pytest.raises(ValueError):
        Observation(q, 0.0)


@pytest.mark.parametrize(
    "kwargs",
    [
        dict(pick_target_base=(0.25, 0.0), place_target_base=PLACE_BOX),
        dict(pick_target_base=(NAN, 0.0, 0.015), place_target_base=PLACE_BOX),
        dict(pick_target_base=PICK_CENTER, place_target_base=(0.2, INF, 0.005)),
        dict(pick_target_base=[True, True, True], place_target_base=PLACE_BOX),
        dict(pick_target_base=PICK_CENTER, place_target_base=PLACE_BOX, instruction=1),
    ],
    ids=["len2", "nan", "inf", "bool", "instruction_int"],
)
def test_goal_rejects_invalid_values(kwargs):
    """TC-TJ-E11 Goal の検証 🟡"""
    with pytest.raises(ValueError):
        Goal(**kwargs)


@pytest.mark.parametrize(
    "over",
    [dict(speed_scale=0.0), dict(speed_scale=-0.5), dict(control_dt=0.0), dict(safety_factor=1.5)],
    ids=["scale0", "scale_neg", "dt0", "factor1.5"],
)
def test_segment_steps_rejects_bad_arguments(over):
    """TC-TJ-E12 刻み数の引数 🟡"""
    with pytest.raises(ValueError):
        _steps(unit_delta(0.5), **over)


def test_segment_steps_rejects_nan_delta():
    """TC-TJ-E12 (続き) Δ に NaN 🟡"""
    with pytest.raises(ValueError):
        _steps(unit_delta(NAN))


# =============================================================== 1.3 境界値
def test_steps_exact_multiple_does_not_round_up():
    """TC-TJ-B01 ちょうど割り切れる 🔵"""
    assert _steps(unit_delta(23 * STEP_UNIT), min_segment_s=0.02) == 23  # 【確認内容】24 にならない


def test_steps_slightly_over_rounds_up():
    """TC-TJ-B02 わずかに超える 🔵"""
    assert _steps(unit_delta(23 * STEP_UNIT + 1e-9), min_segment_s=0.02) == 24


def test_zero_motion_uses_min_segment():
    """TC-TJ-B03 動かない区間は最短時間になる 🔵"""
    assert _steps(unit_delta(0.0), min_segment_s=0.2) == 10  # 【確認内容】ceil(0.2 / 0.02) が 11 にならない


def test_min_segment_below_one_step():
    """TC-TJ-B04 最短時間が 1 刻み以下 🔵"""
    assert _steps(unit_delta(0.0), min_segment_s=0.02) == 1  # 【確認内容】max(1, ...)


def test_min_segment_longer_than_motion():
    """TC-TJ-B05 最短時間が移動時間より長い 🔵"""
    assert _steps(unit_delta(0.1), min_segment_s=0.2) == 10  # 【確認内容】移動は 6 刻み分


def test_hold_steps_zero_seconds():
    """TC-TJ-B06 保持 0 秒 🔴"""
    assert hold_steps(0.0, 0.02) == 0  # 【確認内容】hold_steps だけを確かめる (区間を残すかは Green で決める)


def test_safety_factor_one():
    """TC-TJ-B07 係数 1.0 🟡"""
    delta = unit_delta(23 * 0.02)
    n = _steps(delta, safety_factor=1.0, min_segment_s=0.02)
    assert n == 23  # 【確認内容】割り切れる
    q = interpolate_segment(Z6, Z6 + delta, n)
    assert np.abs(np.diff(np.vstack([Z6, q]), axis=0)).max() <= 0.02 + EXACT_ABS  # 【確認内容】上限ちょうどは合格


def test_command_sequence_step_exactly_at_limit():
    """TC-TJ-B08 速度の上限ちょうど 🟡"""
    q, labels = _sequence()
    q[1:, 0] = 0.02
    q[2:, 0] = 0.04
    q[3:, 0] = 0.06
    _check(q, labels)  # 【確認内容】差がちょうど 0.02 でも例外が出ない (≤ は合格)


def test_clearance_exactly_at_threshold():
    """TC-TJ-B09 すき間がしきい値ちょうど 🟡"""
    assert find_clearance_violation(np.array([0.001]), ("a",), 0.001) is None  # 【確認内容】≥ は合格

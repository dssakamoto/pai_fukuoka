"""apply_safety・SafetyResult・SimRobot 結合のテスト (TASK-0010, Red フェーズ)。

test_limits.py から TC-AS / TC-IN を移しただけ (テスト本体は変更なし)。
テストケース定義: docs/implements/so101-vlm-pickplace/TASK-0010/safety-testcases.md
信頼性: 🔵 要件・設計・実測に基づく / 🟡 妥当な推測 / 🔴 要件にない仮定
"""

import dataclasses
import math

import numpy as np
import pytest

from safety_constants import (
    DEFAULT_CFG,
    DT,
    EXACT_ABS,
    INF,
    LOWER_SYN,
    NAN,
    UPPER_SYN,
    VMAX,
    WS_UPPER,
    Z6,
    _arr,
)
from so101_vlm.robot.sim import SimRobot
from so101_vlm.safety.limits import (
    REASON_OUT_OF_WORKSPACE,
    REASON_STALE_STATE,
    SafetyResult,
    apply_safety,
    limits_from_model,
)


# =============================================================== TC-AS
def _apply(q_current=Z6, q_target=Z6, *, lower=LOWER_SYN, upper=UPPER_SYN, last_state_t=1.0, now=1.0,
           config=DEFAULT_CFG, target_base=None):
    return apply_safety(q_current, q_target, lower=lower, upper=upper, last_state_t=last_state_t,
                        now=now, config=config, target_base=target_base)


def _flags(res):
    return (res.clipped, res.rate_limited, res.rejected, res.stopped)


QC_H = _arr(0.1, 0.2, -0.1, 0, 0, 0.5)
QT_H = _arr(0.5, 0.3, 0.0, 0.2, 0.1, 0.6)


def test_apply_passthrough():
    """TC-AS-01 素通し 🔵"""
    q = _arr(0.01, 0, -0.01, 0, 0.02, 0.005)
    res = _apply(Z6, q)
    assert np.array_equal(res.q_command, q)
    assert _flags(res) == (False,) * 4 and res.reason is None


def test_apply_rate_limited_only():
    """TC-AS-02 速度制限のみ 🔵"""
    res = _apply(Z6, _arr(0.5, 0, 0, 0, 0, 0))
    assert res.q_command == pytest.approx(_arr(0.02, 0, 0, 0, 0, 0), abs=EXACT_ABS)
    assert _flags(res) == (False, True, False, False) and res.reason is None


def test_apply_clip_then_rate():
    """TC-AS-03 クリップが先、速度制限が後 🟡"""
    res = _apply(_arr(0.99, 0, 0, 0, 0, 0), _arr(1.5, 0, 0, 0, 0, 0))
    assert res.q_command[0] == 1.0
    assert res.clipped is True and res.rate_limited is False


def test_apply_order_visible_in_values():
    """TC-AS-04 クリップ後の目標で α を決める 🟡"""
    res = _apply(_arr(0.95, 0, 0, 0, 0, 0), _arr(1.9, 0, 0, 0, 0, 0.5))
    assert res.q_command == pytest.approx(_arr(0.952, 0, 0, 0, 0, 0.02), abs=EXACT_ABS)
    assert res.clipped is True and res.rate_limited is True


def test_apply_final_clip_when_current_outside():
    """TC-AS-05 q_current が範囲外 → 最終クリップ 🟡"""
    res = _apply(_arr(1.05, 0, 0, 0, 0, 0), _arr(1.0, 0, 0, 0, 0, 0))
    assert res.q_command[0] == 1.0 and res.clipped is True


def test_apply_final_clip_far_outside():
    """TC-AS-06 離れた外側でも q_command は範囲内 (NFR-02) 🟡"""
    q = _arr(1.5, 0, 0, 0, 0, 0)
    res = _apply(q, q)
    assert res.q_command[0] == 1.0 and res.clipped is True
    assert np.all(res.q_command >= LOWER_SYN) and np.all(res.q_command <= UPPER_SYN)


def test_apply_inside_workspace_passthrough():
    """TC-AS-07 作業範囲の内側は素通し 🔵"""
    q = _arr(0.01, 0, -0.01, 0, 0.02, 0.005)
    res = _apply(Z6, q, target_base=(0.25, 0.0, 0.015))
    assert np.array_equal(res.q_command, q) and res.rejected is False


def test_apply_rejects_outside_workspace():
    """TC-AS-08 作業範囲外は拒否し現在値を保持 🔵"""
    res = _apply(QC_H, QT_H, target_base=(0.40, 0, 0.05))
    assert res.rejected is True
    assert res.reason == REASON_OUT_OF_WORKSPACE == "target_out_of_workspace"
    assert np.array_equal(res.q_command, QC_H)
    assert (res.clipped, res.rate_limited, res.stopped) == (False, False, False)


def test_apply_reject_beats_clip():
    """TC-AS-09 拒否が可動域クリップより優先 🟡"""
    res = _apply(Z6, _arr(5.0, 0, 0, 0, 0, 0), target_base=(0.40, 0, 0.05))
    assert res.rejected is True and res.clipped is False and res.rate_limited is False


def test_apply_workspace_face_not_rejected():
    """TC-AS-10 作業範囲の角の上は拒否しない 🟡"""
    assert _apply(Z6, Z6, target_base=WS_UPPER).rejected is False


def test_apply_stale_state_stops():
    """TC-AS-11 状態途絶 → 停止 🔵"""
    res = _apply(QC_H, QT_H, now=1.5)
    assert res.stopped is True
    assert res.reason == REASON_STALE_STATE == "stale_state"
    assert np.array_equal(res.q_command, QC_H)
    assert (res.clipped, res.rate_limited, res.rejected) == (False, False, False)


def test_apply_stop_beats_reject():
    """TC-AS-12 停止が拒否・クリップより優先 🟡"""
    res = _apply(QC_H, _arr(5.0, 0, 0, 0, 0, 0), now=1.5, target_base=(0.40, 0, 0.05))
    assert res.stopped is True and res.rejected is False
    assert res.clipped is False and res.rate_limited is False and res.reason == "stale_state"


def test_apply_exact_timeout_is_fresh():
    """TC-AS-13 ちょうど timeout は新鮮 🟡"""
    assert _apply(Z6, Z6, last_state_t=0.0, now=0.2).stopped is False


@pytest.mark.parametrize("now", [NAN, 0.5])
def test_apply_bad_clock_stops(now):
    """TC-AS-14 NaN / 逆行する時刻 → 停止 🟡"""
    assert _apply(Z6, Z6, last_state_t=1.0, now=now).stopped is True


def test_reason_constants_and_none():
    """TC-AS-15 reason 定数と None 🔵"""
    assert REASON_STALE_STATE == "stale_state"
    assert REASON_OUT_OF_WORKSPACE == "target_out_of_workspace"
    assert _apply(Z6, _arr(0.5, 0, 0, 0, 0, 0)).reason is None


def test_apply_scale_03():
    """TC-AS-16 scale 0.3 🔵"""
    cfg = dataclasses.replace(DEFAULT_CFG, speed_scale=0.3)
    res = _apply(Z6, _arr(0.5, 0, 0, 0, 0, 0), config=cfg)
    assert res.q_command[0] == pytest.approx(0.006, abs=EXACT_ABS) and res.rate_limited is True


@pytest.mark.parametrize("bad", [NAN, INF])
def test_apply_stale_before_value_check(bad):
    """TC-AS-20 停止は値の検証より先 🟡"""
    res = _apply(QC_H, _arr(bad, 0, 0, 0, 0, 0), now=1.5)
    assert res.stopped is True and np.array_equal(res.q_command, QC_H)


def test_apply_stale_before_target_base_value_check():
    """TC-AS-21 停止は target_base の値より先 🟡"""
    assert _apply(QC_H, QT_H, now=1.5, target_base=(NAN, 0, 0)).stopped is True


@pytest.mark.parametrize("bad", [NAN, INF])
def test_apply_nonfinite_q_target_raises(bad):
    """TC-AS-22 新鮮な状態で q_target が NaN/inf → ValueError 🟡"""
    with pytest.raises(ValueError, match="q_target"):
        _apply(Z6, _arr(bad, 0, 0, 0, 0, 0))


def test_apply_nonfinite_target_base_raises():
    """TC-AS-23 target_base の NaN → ValueError 🟡"""
    with pytest.raises(ValueError, match="target_base"):
        _apply(Z6, Z6, target_base=(NAN, 0, 0))


@pytest.mark.parametrize(
    "kwargs",
    [
        {"q_target": np.zeros(5)},
        {"target_base": (0.1, 0.2)},
        {"q_current": _arr(NAN, 0, 0, 0, 0, 0)},
        {"q_current": np.zeros(7)},
        {"lower": np.zeros(5)},
        {"upper": _arr(INF, 1, 1, 1, 2, 2)},
        {"q_target": np.ones(6, dtype=bool)},
    ],
    ids=["target5", "base2", "current_nan", "current7", "lower5", "upper_inf", "target_bool"],
)
def test_apply_structure_checked_before_stop(kwargs):
    """TC-AS-24 構造の検証は停止より先 (古い状態でも ValueError) 🟡"""
    with pytest.raises(ValueError):
        _apply(**{"now": 1.5, **kwargs})


@pytest.mark.parametrize("config", [{"vmax": 1.0}, None])
def test_apply_bad_config_type_raises(config):
    """TC-AS-25 config の型違反 → ValueError 🟡"""
    with pytest.raises(ValueError):
        _apply(config=config)


def test_apply_bool_time_raises():
    """TC-AS-26 bool の時刻 → ValueError 🟡"""
    with pytest.raises(ValueError):
        _apply(last_state_t=True)


def test_apply_keyword_only():
    """TC-AS-27 lower などは位置引数不可 → TypeError 🟡"""
    with pytest.raises(TypeError):
        apply_safety(Z6, Z6, LOWER_SYN, UPPER_SYN, 1.0, 1.0, DEFAULT_CFG)


def test_result_is_frozen():
    """TC-AS-30 SafetyResult は frozen 🔵"""
    res = _apply(Z6, _arr(0.5, 0, 0, 0, 0, 0))
    with pytest.raises(dataclasses.FrozenInstanceError):
        res.clipped = True
    with pytest.raises(dataclasses.FrozenInstanceError):
        res.q_command = Z6


def test_result_q_command_readonly():
    """TC-AS-31 q_command は読み取り専用 float64 (6,) 🔵"""
    res = _apply(Z6, _arr(0.5, 0, 0, 0, 0, 0))
    with pytest.raises(ValueError):
        res.q_command[0] = 9.0
    assert res.q_command.flags.writeable is False
    assert res.q_command.dtype == np.float64 and res.q_command.shape == (6,)


def test_result_holds_copy():
    """TC-AS-32 保持はコピー 🔵"""
    qc = QC_H.copy()
    res = _apply(qc, QT_H, now=1.5)
    qc[0] = 99.0
    assert res.q_command[0] == QC_H[0]
    assert not np.shares_memory(res.q_command, qc)


@pytest.mark.parametrize(
    "case",
    [
        dict(q_current=_arr(0.95, 0, 0, 0, 0, 0), q_target=_arr(1.9, 0, 0, 0, 0, 0.5)),
        dict(q_current=QC_H, q_target=QT_H, target_base=(0.40, 0, 0.05)),
        dict(q_current=QC_H, q_target=QT_H, now=1.5),
    ],
    ids=["as04", "as08", "as11"],
)
def test_apply_does_not_mutate_inputs(case):
    """TC-AS-33 入力不変 🔵"""
    lo, up = LOWER_SYN.copy(), UPPER_SYN.copy()
    qc, qt = case["q_current"].copy(), case["q_target"].copy()
    tb = case.get("target_base")
    tb_arr = None if tb is None else np.array(tb)
    _apply(qc, qt, lower=lo, upper=up, now=case.get("now", 1.0), target_base=tb_arr)
    assert np.array_equal(qc, case["q_current"]) and np.array_equal(qt, case["q_target"])
    assert np.array_equal(lo, LOWER_SYN) and np.array_equal(up, UPPER_SYN)
    if tb_arr is not None:
        assert np.array_equal(tb_arr, np.array(tb))


def test_apply_deterministic():
    """TC-AS-34 決定性 🟡"""
    a = _apply(QC_H, _arr(1.9, 0, 0, 0, 0, 0.5))
    b = _apply(QC_H, _arr(1.9, 0, 0, 0, 0, 0.5))
    assert np.array_equal(a.q_command, b.q_command) and _flags(a) == _flags(b) and a.reason == b.reason


def test_result_direct_construction():
    """TC-AS-35 SafetyResult の直接生成 🟡"""
    src = [0.0] * 6
    res = SafetyResult(q_command=src, clipped=False, rate_limited=False, rejected=False,
                       stopped=False, reason=None)
    assert res.q_command.dtype == np.float64 and res.q_command.flags.writeable is False
    src[0] = 5.0
    assert res.q_command[0] == 0.0
    arr = np.zeros(6)
    res2 = SafetyResult(q_command=arr, clipped=False, rate_limited=False, rejected=False,
                        stopped=False, reason=None)
    assert not np.shares_memory(res2.q_command, arr)


# --- TC-AS-40..42 制限内の軌道は変わらない ---
def _trajectory(qa, qb, m):
    n = int(math.ceil(np.max(np.abs(qb - qa)) / (0.99 * m)))
    return [qa + (qb - qa) * k / n for k in range(n + 1)]


def _replay(traj, lower, upper, config, target_bases=None):
    q_ref = traj[0]
    for k in range(len(traj) - 1):
        tb = None if target_bases is None else target_bases[k % len(target_bases)]
        res = apply_safety(q_ref, traj[k + 1], lower=lower, upper=upper, last_state_t=1.0, now=1.0,
                           config=config, target_base=tb)
        assert _flags(res) == (False,) * 4 and res.reason is None
        assert np.array_equal(res.q_command, traj[k + 1])  # 【確認内容】: 完全一致
        q_ref = res.q_command


QA = _arr(0.1, -0.3, 0.5, 0.2, -0.1, 1.0)
QB = _arr(-0.767, -0.031, -0.133, 0.411, 0.2, 0.0)


def test_inside_limits_trajectory_unchanged(scene_model):
    """TC-AS-40 制限内の軌道はフィルタを通しても変わらない 🔵/🟡"""
    lower, upper = limits_from_model(scene_model)
    traj = _trajectory(QA, QB, 0.02)
    assert len(traj) - 1 == 51
    assert max(np.abs(np.diff(traj, axis=0)).max(axis=1)) <= 0.02
    assert all(np.all(q >= lower) and np.all(q <= upper) for q in traj)
    _replay(traj, lower, upper, DEFAULT_CFG)


def test_inside_limits_trajectory_unchanged_scale_03(scene_model):
    """TC-AS-41 scale 0.3 でも変わらない 🟡"""
    lower, upper = limits_from_model(scene_model)
    cfg = dataclasses.replace(DEFAULT_CFG, speed_scale=0.3)
    traj = _trajectory(QA, QB, 0.006)
    assert len(traj) - 1 == 169
    _replay(traj, lower, upper, cfg)


def test_inside_limits_trajectory_unchanged_with_workspace(scene_model, task_points):
    """TC-AS-42 作業範囲つきでも変わらない 🟡"""
    lower, upper = limits_from_model(scene_model)
    _replay(_trajectory(QA, QB, 0.02), lower, upper, DEFAULT_CFG,
            target_bases=[task_points[0], task_points[80], task_points[170]])


# =============================================================== TC-IN
Q_FAR = _arr(3.0, 0.0, 2.5, 0.0, -3.5, 2.5)
MOVED = [0, 2, 4, 5]


def _run_cycles(robot, cfg, lower, upper, n, q_target=Q_FAR, target_base=None, stale=False, q_ref=None):
    """1 周期ずつ get_state → apply_safety → send_command を回し (指令列, 計測 q, 時刻, 結果) を返す。"""
    state = robot.get_state()
    q_ref = state.q.copy() if q_ref is None else q_ref
    cmds, qs, ts, results = [], [state.q.copy()], [state.timestamp], []
    for _ in range(n):
        st = robot.get_state()
        now = st.timestamp + (0.5 if stale else 0.0)
        res = apply_safety(q_ref, q_target, lower=lower, upper=upper, last_state_t=st.timestamp,
                           now=now, config=cfg, target_base=target_base)
        robot.send_command(res.q_command)
        cmds.append((q_ref.copy(), res.q_command.copy()))
        q_ref = res.q_command
        results.append(res)
        st2 = robot.get_state()
        qs.append(st2.q.copy())
        ts.append(st2.timestamp)
    return cmds, np.array(qs), np.array(ts), results, q_ref


@pytest.fixture
def sim_robot():
    robot = SimRobot()
    yield robot
    robot.close()


def _check_envelope(scale, scene_model, sim_robot, tol_env):
    lower, upper = limits_from_model(scene_model)
    cfg = dataclasses.replace(DEFAULT_CFG, speed_scale=scale)
    m = VMAX * scale * DT
    cmds, qs, ts, results, q_ref = _run_cycles(sim_robot, cfg, lower, upper, 25)
    for prev, cmd in cmds:
        assert np.all(np.abs(cmd - prev) <= m)  # 【確認内容】: 毎周期 m 以下 (許容差なし)
        assert np.all(cmd >= lower) and np.all(cmd <= upper)
    assert results[0].clipped is True and results[0].rate_limited is True
    el = ts - ts[0]
    assert np.all(np.abs(qs - qs[0]) <= VMAX * scale * el[:, None] + 0.02)  # 包絡
    avg = np.abs(qs[-1] - qs[0]) / el[-1]
    assert np.all(avg <= VMAX * scale * 1.05)
    tgt = np.clip(Q_FAR, lower, upper)
    assert np.all(np.abs(tgt - qs[-1])[MOVED] < np.abs(tgt - qs[0])[MOVED])
    assert np.abs(qs[-1] - qs[0])[4] >= 0.9 * scale * 0.5
    assert np.all(qs >= lower - 0.01) and np.all(qs <= upper + 0.01)
    return qs, q_ref, results


def test_sim_envelope_scale_1(scene_model, sim_robot):
    """TC-IN-01 scale 1.0 の包絡・平均速度・可動域 🟡"""
    _check_envelope(1.0, scene_model, sim_robot, 0.02)


def test_sim_envelope_scale_03(scene_model, sim_robot):
    """TC-IN-02 scale 0.3 (NFR-07(2)) 🟡"""
    with SimRobot() as ref:
        qs1, _, _ = _check_envelope(1.0, scene_model, ref, 0.02)
    qs03, _, _ = _check_envelope(0.3, scene_model, sim_robot, 0.02)
    ratio = np.max(np.abs(qs03[-1] - qs03[0])) / np.max(np.abs(qs1[-1] - qs1[0]))
    assert 0.25 <= ratio <= 0.35


def test_sim_stop_holds_position(scene_model, sim_robot):
    """TC-IN-03 古い状態では停止して保持 (NFR-07(4)) 🟡"""
    lower, upper = limits_from_model(scene_model)
    _, qs, _, _, q_ref = _run_cycles(sim_robot, DEFAULT_CFG, lower, upper, 25)
    q_before = qs[-1]
    cmds, qh, _, results, _ = _run_cycles(sim_robot, DEFAULT_CFG, lower, upper, 10, stale=True, q_ref=q_ref)
    assert all(r.stopped for r in results)
    assert all(np.array_equal(cmd, q_ref) for _, cmd in cmds)
    assert np.max(np.abs(qh[1:] - q_before)) <= 0.012
    assert np.max(np.abs(qh[10] - qh[5])) <= 1e-3
    assert np.max(np.abs(qh[10] - q_ref)) <= 2e-3


def test_sim_rejected_target_does_not_move(scene_model, sim_robot):
    """TC-IN-04 作業範囲外は動かない (NFR-07(3)) 🟡"""
    lower, upper = limits_from_model(scene_model)
    q0 = sim_robot.get_state().q.copy()
    cmds, qs, _, results, _ = _run_cycles(sim_robot, DEFAULT_CFG, lower, upper, 10,
                                          target_base=(0.40, 0, 0.05))
    assert all(r.rejected for r in results)
    assert all(np.array_equal(cmd, q0) for _, cmd in cmds)
    assert np.max(np.abs(qs - q0)) <= 2e-3

"""IkPolicy (実モデル PICKPLACE_SCENE_PATH) のテスト (TASK-0012, Red)。

テストケース定義: docs/implements/so101-vlm-pickplace/TASK-0012/trajectory-testcases.md 3 節 (TC-IP-*)
対象モジュール (まだ存在しない): so101_vlm.policy.ik_policy (IkPolicy), so101_vlm.policy.base,
  so101_vlm.policy.trajectory, so101_vlm.policy.trajectory_config
IK を使うテストは module scope の fixture ``solved`` (代表 8 配置を 1 回だけ解いた軌道) を共有する。
実行時間の予算: このファイル < 8 s。1 s を超えそうなのは fixture ``solved`` (約 1.8 s) と
TC-IP-17 (20 配置、約 3.2 s) のみ。
信頼性: 🔵 要件・実測に基づく / 🟡 妥当な推測 / 🔴 要件にない仮定
"""

import dataclasses

import mujoco
import numpy as np
import pytest

import so101_vlm.policy.ik_policy as ik_policy_module
import so101_vlm.policy.trajectory as trajectory_module
from so101_vlm.eval.layout import sample_layout
from so101_vlm.policy.base import Goal, Observation, Policy
from so101_vlm.policy.ik import PITCH_CANDIDATES_DEG, solve_ik
from so101_vlm.policy.ik_policy import IkPolicy
from so101_vlm.policy.ik_types import IkError
from so101_vlm.policy.trajectory import TrajectoryError
from so101_vlm.policy.trajectory_config import load_trajectory_config
from so101_vlm.robot.model_paths import JOINT_NAMES, PICKPLACE_SCENE_PATH, SO101_SCENE_PATH
from so101_vlm.robot.sim import SimRobot
from so101_vlm.safety.config import WorkspaceBox, load_safety_config
from so101_vlm.safety.limits import apply_safety, limits_from_model
from so101_vlm.safety.workspace import is_in_workspace
from trajectory_constants import (
    BLOCK_Z,
    CORNERS,
    DT,
    FK_TOL_M,
    G_CLOSED,
    G_OPEN,
    HOME_Q,
    MIN_SEG_STEPS,
    NEAR_SHOULDER,
    PICK_CENTER,
    PITCH_TOL_DEG,
    PLACE_BOX,
    SEG_TARGET,
    SEGMENTS,
    TCP_OFFSET,
    WAYPOINT_NAMES,
)

HOLD_SEGMENTS = ("close_hold", "open_hold")
CENTER_XY = PICK_CENTER[:2]
REP_XY = {
    "center": CENTER_XY,
    **CORNERS,
    "seed0_red": sample_layout(0)[0],
    "seed0_blue": sample_layout(0)[1],
    "seed10_red": sample_layout(10)[0],
}
GRIPPER_AT_END = {
    "pre_grasp": G_OPEN, "grasp": G_OPEN, "close": G_CLOSED, "close_hold": G_CLOSED, "lift": G_CLOSED,
    "pre_place": G_CLOSED, "place": G_CLOSED, "open": G_OPEN, "open_hold": G_OPEN, "retreat": G_OPEN,
}  # fmt: skip
NAMES = tuple(REP_XY)


# --------------------------------------------------------------- ヘルパ
def make_goal(x, y, z=BLOCK_Z, place=PLACE_BOX):
    return Goal(pick_target_base=(x, y, z), place_target_base=place)


def home_obs():
    return Observation(HOME_Q.copy(), 0.0)


def rows(traj, name):
    return [k for k, label in enumerate(traj.labels) if label == name]


def end_q(traj, name):
    return traj.q[rows(traj, name)[-1]]


def count(traj, name):
    return len(rows(traj, name))


def records(traj):
    return {r.name: r for r in traj.metadata["waypoints"]}


def targets_of(traj):
    return {name: np.asarray(r.target) for name, r in records(traj).items()}


def fk_tcp(model, q):
    """独立 MjData + mj_kinematics で gripperframe + R @ TCP_OFFSET と傾き [度] を返す。"""
    data = mujoco.MjData(model)
    for name, value in zip(JOINT_NAMES, q, strict=True):
        data.qpos[model.joint(name).qposadr[0]] = value
    mujoco.mj_kinematics(model, data)
    site = model.site("gripperframe").id
    rot = data.site_xmat[site].reshape(3, 3)
    pos = data.site_xpos[site] + rot @ np.asarray(TCP_OFFSET)
    return pos, float(np.degrees(np.arccos(np.clip(-rot[2, 0], -1.0, 1.0))))


def tcp_path(model, q_rows):
    data = mujoco.MjData(model)
    site = model.site("gripperframe").id
    adr = [model.joint(n).qposadr[0] for n in JOINT_NAMES]
    out = []
    for q in q_rows:
        data.qpos[adr] = q
        mujoco.mj_kinematics(model, data)
        out.append(data.site_xpos[site] + data.site_xmat[site].reshape(3, 3) @ np.asarray(TCP_OFFSET))
    return np.array(out)


def run_safety(traj, cfg, lower, upper):
    """前回指令を基準に apply_safety を k = 1..N-1 で呼ぶ。(フラグが立った周期数, 出力が入力と違った周期数)。"""
    targets = targets_of(traj)
    prev, flagged, changed = traj.q[0], 0, 0
    for k in range(1, len(traj.q)):
        res = apply_safety(
            prev, traj.q[k], lower=lower, upper=upper, last_state_t=0.0, now=0.0, config=cfg,
            target_base=targets[SEG_TARGET[traj.labels[k]]],
        )  # fmt: skip
        flagged += int(res.clipped or res.rate_limited or res.rejected or res.stopped)
        changed += int(not np.array_equal(res.q_command, traj.q[k]))
        prev = res.q_command
    return flagged, changed


class SolveSpy:
    """solve_ik を包んで呼び出しを記録するスパイ。fail_on の呼び出し番号 (1 始まり) では例外を投げる。"""

    def __init__(self, fail_on=None, error=None):
        self.calls = []
        self.fail_on = fail_on
        self.error = error

    def __call__(self, model, target, seed, **kwargs):
        self.calls.append((np.asarray(target, dtype=float), np.asarray(seed, dtype=float), kwargs))
        if self.fail_on == len(self.calls):
            raise self.error
        return solve_ik(model, target, seed, **kwargs)


def ik_error(reason):
    return IkError(
        target=(0.0, 0.0, 0.0), reason=reason, best_pos_error=0.1, best_pitch_deg=None,
        tried_pitches_deg=(0.0,), message="テスト用の IK 失敗",
    )  # fmt: skip


def with_clearance(cfg, **changes):
    return dataclasses.replace(cfg, clearance=dataclasses.replace(cfg.clearance, **changes))


# --------------------------------------------------------------- fixture
@pytest.fixture(scope="module")
def model():
    # 【テスト前準備】: モデル読み込みは重いのでモジュールで 1 回だけ行う
    return mujoco.MjModel.from_xml_path(str(PICKPLACE_SCENE_PATH))


@pytest.fixture(scope="module")
def limits(model):
    return limits_from_model(model)


@pytest.fixture(scope="module")
def safety_cfg():
    return load_safety_config()


@pytest.fixture(scope="module")
def traj_cfg():
    return load_trajectory_config()


@pytest.fixture(scope="module")
def policy(model):
    return IkPolicy(model)


@pytest.fixture(scope="module")
def solved(policy):
    # 【テスト前準備】: 代表 8 配置を HOME_Q から 1 回だけ解いて共有する (IK を何度も解かない)
    return {name: policy.act(home_obs(), make_goal(*xy)) for name, xy in REP_XY.items()}


# =============================================================== 3.1 正常系
def test_policy_satisfies_protocol(policy):
    """TC-IP-01 Policy Protocol に適合 🟡"""
    assert isinstance(policy, Policy)  # 【確認内容】runtime_checkable な Policy として扱える


@pytest.mark.parametrize("name", NAMES)
def test_segment_order(solved, name):
    """TC-IP-02 区間の順 🔵"""
    traj = solved[name]
    assert traj.segments == SEGMENTS  # 【確認内容】start から retreat までの順
    assert traj.labels[0] == "start"
    assert set(traj.labels) == set(SEGMENTS)  # 【確認内容】各区間に 1 サンプル以上


@pytest.mark.parametrize("name", NAMES)
def test_first_row_is_observation(solved, name):
    """TC-IP-03 開始値 🔵"""
    assert np.array_equal(solved[name].q[0], HOME_Q)  # 【確認内容】q[0] は obs.q と完全一致


@pytest.mark.parametrize("name", NAMES)
def test_times(solved, name):
    """TC-IP-04 時刻 🔵"""
    t = solved[name].times
    np.testing.assert_allclose(t, np.arange(len(t)) * DT, atol=1e-12)  # 【確認内容】k x 0.02
    assert np.all(np.diff(t) > 0)  # 【確認内容】厳密に単調増加
    assert solved[name].metadata["duration_s"] == t[-1]


@pytest.mark.parametrize("name", NAMES)
def test_waypoint_joint_relations(solved, name):
    """TC-IP-05 経由点の関節角の関係 🔵"""
    traj = solved[name]
    arm = lambda seg: end_q(traj, seg)[:5]  # noqa: E731
    assert np.array_equal(arm("lift"), arm("pre_grasp"))  # 【確認内容】lift = pre_grasp の腕
    assert end_q(traj, "lift")[5] == G_CLOSED
    assert np.array_equal(arm("retreat"), arm("pre_place"))  # 【確認内容】retreat = pre_place の腕
    assert end_q(traj, "retreat")[5] == G_OPEN
    for seg in ("close", "close_hold"):
        assert np.array_equal(arm(seg), arm("grasp")), seg  # 【確認内容】グリッパーだけが動く
    for seg in ("open", "open_hold"):
        assert np.array_equal(arm(seg), arm("place")), seg


@pytest.mark.parametrize("name", NAMES)
def test_gripper_values_and_hold_lengths(solved, name):
    """TC-IP-06 gripper の値 🔵"""
    traj = solved[name]
    for seg, expected in GRIPPER_AT_END.items():
        assert end_q(traj, seg)[5] == expected, seg  # 【確認内容】区間の終点の gripper
    assert (count(traj, "close"), count(traj, "close_hold")) == (23, 25)
    assert (count(traj, "open"), count(traj, "open_hold")) == (23, 15)


@pytest.mark.parametrize("name", NAMES)
def test_wrist_roll_constant(solved, model, limits, name):
    """TC-IP-07 wrist_roll が一定 🟡"""
    lower, upper = limits
    expected = np.clip(HOME_Q[4], lower[4], upper[4])
    assert np.all(solved[name].q[:, 4] == expected)  # 【確認内容】全行が clip(obs.q[4])


@pytest.mark.parametrize("name", NAMES)
def test_safety_filter_is_transparent(solved, safety_cfg, limits, name):
    """TC-IP-08 安全フィルタを通しても何も起きない 🔵"""
    flagged, changed = run_safety(solved[name], safety_cfg, *limits)
    assert flagged == 0  # 【確認内容】clipped / rate_limited / rejected / stopped が全周期で False
    assert changed == 0  # 【確認内容】出力 = 入力


@pytest.mark.parametrize("name", NAMES)
def test_step_and_range_limits(solved, limits, name):
    """TC-IP-09 連続差と可動域 🔵"""
    q = solved[name].q
    assert np.all(np.abs(np.diff(q, axis=0)) <= 1.0 * 1.0 * DT)  # 【確認内容】前回指令基準の速度上限
    assert np.all(q[1:] >= limits[0]) and np.all(q[1:] <= limits[1])  # 【確認内容】可動域内


@pytest.mark.parametrize("name", NAMES)
def test_waypoint_tcp_matches_targets(solved, model, name):
    """TC-IP-10 経由点の TCP が目標に合う 🔵"""
    traj = solved[name]
    x, y = REP_XY[name]
    pick = np.array([x, y, BLOCK_Z])
    place = np.array(PLACE_BOX)
    pre_grasp = pick + (0.0, 0.0, 0.08)
    pre_place = np.array([place[0], place[1], pre_grasp[2]])
    expected = {
        "pre_grasp": pre_grasp, "grasp": pick, "lift": pre_grasp, "pre_place": pre_place,
        "place": np.array([place[0], place[1], place[2] + 0.030]), "retreat": pre_place,
    }  # fmt: skip
    recs = records(traj)
    assert tuple(recs) == WAYPOINT_NAMES
    for wp, target in expected.items():
        np.testing.assert_allclose(recs[wp].target, target, atol=1e-9, err_msg=wp)  # 【確認内容】要件 2.5 の式
        tcp, _ = fk_tcp(model, recs[wp].q)
        assert np.linalg.norm(tcp - target) <= FK_TOL_M, wp  # 【確認内容】独立 FK で 2 mm 以内


@pytest.mark.parametrize("name", NAMES)
def test_targets_inside_workspace(solved, safety_cfg, name):
    """TC-IP-11 経由点の目標は作業範囲の中 🔵"""
    for wp, target in targets_of(solved[name]).items():
        assert is_in_workspace(target, safety_cfg.workspace), wp


@pytest.mark.parametrize("name", NAMES)
def test_common_pitch_rule(solved, model, name):
    """TC-IP-12 共通ピッチの規則 🔵"""
    traj = solved[name]
    recs = records(traj)
    p1 = traj.metadata["pick_pitch_deg"]
    assert recs["pre_grasp"].pitch_candidate_deg == p1  # 【確認内容】metadata は pre_grasp の採用候補
    assert recs["grasp"].pitch_candidate_deg == p1  # 【確認内容】grasp も同じ候補で解く
    assert traj.metadata["place_pitch_deg"] == 10.0  # 【確認内容】置く側は全配置で 10°
    assert recs["pre_place"].pitch_candidate_deg == recs["place"].pitch_candidate_deg == 10.0
    for wp in ("pre_grasp", "grasp", "pre_place", "place"):
        _, tilt = fk_tcp(model, recs[wp].q)
        assert abs(tilt - recs[wp].pitch_candidate_deg) <= PITCH_TOL_DEG, wp  # 【確認内容】実際の傾きは候補の 5° 以内


def test_metadata_keys_and_values(solved):
    """TC-IP-13 metadata のキー 🟡"""
    md = solved["center"].metadata
    required = {
        "control_dt", "vmax", "speed_scale", "time_safety_factor", "tcp_offset", "pick_pitch_deg",
        "place_pitch_deg", "waypoints", "min_obstacle_clearance_m", "min_self_clearance_m", "duration_s",
    }  # fmt: skip
    assert required <= set(md)
    assert (md["control_dt"], md["speed_scale"], md["time_safety_factor"]) == (0.02, 1.0, 0.99)
    assert tuple(md["vmax"]) == (1.0,) * 6 and tuple(md["tcp_offset"]) == (0.004, 0.0, 0.03)
    assert tuple(r.name for r in md["waypoints"]) == WAYPOINT_NAMES
    assert md["pick_pitch_deg"] == 0.0  # 【確認内容】center の実測値


@pytest.mark.parametrize("name", NAMES)
def test_recorded_clearances(solved, name):
    """TC-IP-14 すき間の値 🔵"""
    md = solved[name].metadata
    assert md["min_obstacle_clearance_m"] >= 0.001  # 【確認内容】障害物とのすき間 (実測の最小 2.4 mm)
    assert md["min_self_clearance_m"] >= 0.002  # 【確認内容】自己干渉のすき間 (実測 9.6 mm)


@pytest.mark.parametrize("name", NAMES)
def test_interpolated_tcp_path(solved, model, safety_cfg, name):
    """TC-IP-15 補間した TCP 経路 🔵"""
    traj = solved[name]
    tcp = tcp_path(model, traj.q)
    first = traj.labels.index("grasp")
    for k in range(first, len(tcp)):
        assert is_in_workspace(tcp[k], safety_cfg.workspace), (k, traj.labels[k])  # 【確認内容】範囲内
    pre_place_z = tcp[rows(traj, "pre_place"), 2]
    assert pre_place_z.min() >= 0.09  # 【確認内容】運搬の高さ (実測 0.0949)


def test_speed_scale_03(model, solved, safety_cfg, limits):
    """TC-IP-16 速度倍率 0.3 🔵"""
    cfg03 = dataclasses.replace(safety_cfg, speed_scale=0.3)
    traj03 = IkPolicy(model, safety=cfg03).act(home_obs(), make_goal(*CENTER_XY))
    base = solved["center"]
    moving = [s for s in SEGMENTS[1:] if s not in HOLD_SEGMENTS]
    for seg in moving:
        if count(base, seg) > MIN_SEG_STEPS:  # min_segment_s で決まった区間は除く
            assert count(traj03, seg) >= (count(base, seg) - 1) / 0.3, seg  # 【確認内容】約 3.3 倍
    ratio = sum(count(traj03, s) for s in moving) / sum(count(base, s) for s in moving)
    assert ratio >= 3.2  # 【確認内容】移動区間の合計の比 (実測 3.28)
    assert (count(traj03, "close_hold"), count(traj03, "open_hold")) == (25, 15)  # 【確認内容】保持は倍率に依存しない
    assert run_safety(traj03, cfg03, *limits) == (0, 0)  # 【確認内容】0.3 の設定でもフラグなし
    # N = 811、16.2 s は記録のみ (IK の解のわずかな違いで変わりうる)


def test_duration_over_20_layouts(policy):
    """TC-IP-17 20 配置の所要時間 🔵 (約 3.2 s。このファイルで最も遅いテストの 1 つ)"""
    durations = []
    for seed in range(20):
        x, y = sample_layout(seed)[0]
        durations.append(policy.act(home_obs(), make_goal(x, y)).times[-1])  # 【確認内容】全配置で解ける
    assert 4.5 <= float(np.mean(durations)) <= 7.5  # 【確認内容】平均 (実測 5.39 s)
    assert all(4.0 <= d <= 7.5 for d in durations)  # 【確認内容】1 配置ずつの帯


def test_inputs_not_mutated(model):
    """TC-IP-18 入力を変更しない 🔵"""
    q = np.zeros(6)
    pick, place = np.array(PICK_CENTER), np.array(PLACE_BOX)
    obs, goal = Observation(q, 0.0), Goal(pick, place)
    snapshot = (model.qpos0.copy(), model.actuator_ctrlrange.copy(), float(model.opt.timestep))
    obs_before, goal_before = obs.q.copy(), (goal.pick_target_base.copy(), goal.place_target_base.copy())
    IkPolicy(model).act(obs, goal)
    assert np.array_equal(q, np.zeros(6)) and np.array_equal(pick, PICK_CENTER)  # 【確認内容】元の配列
    assert np.array_equal(obs.q, obs_before)
    assert np.array_equal(goal.pick_target_base, goal_before[0])
    assert np.array_equal(goal.place_target_base, goal_before[1])
    assert np.array_equal(model.qpos0, snapshot[0])  # 【確認内容】モデルが変わらない
    assert np.array_equal(model.actuator_ctrlrange, snapshot[1])
    assert model.opt.timestep == snapshot[2]


def test_deterministic(policy, solved):
    """TC-IP-19 決定的 🔵"""
    again = policy.act(home_obs(), make_goal(*CENTER_XY))
    first = solved["center"]
    assert np.array_equal(again.q, first.q) and np.array_equal(again.times, first.times)
    assert again.labels == first.labels
    assert again.metadata["pick_pitch_deg"] == first.metadata["pick_pitch_deg"]
    assert again.metadata["place_pitch_deg"] == first.metadata["place_pitch_deg"]


def test_start_already_at_pre_grasp(policy, solved):
    """TC-IP-20 開始姿勢が pre_grasp と同じ 🟡"""
    first = solved["center"]
    obs = Observation(end_q(first, "pre_grasp"), 0.0)
    traj = policy.act(obs, make_goal(*CENTER_XY))
    assert count(traj, "pre_grasp") == MIN_SEG_STEPS  # 【確認内容】min_segment_s で 10 刻み
    assert np.all(np.diff(traj.times) > 0)  # 【確認内容】時刻は単調増加のまま
    assert traj.metadata["pick_pitch_deg"] == first.metadata["pick_pitch_deg"]


def test_result_is_read_only(solved):
    """TC-IP-21 結果が変更できない 🔵"""
    with pytest.raises(ValueError):
        solved["center"].q[0, 0] = 1.0


def test_solve_ik_arguments(model, solved, monkeypatch):
    """TC-IP-22 IK に渡す引数 🔵"""
    spy = SolveSpy()
    monkeypatch.setattr(ik_policy_module, "solve_ik", spy)  # 【状態復元】pytest が自動で戻す
    traj = IkPolicy(model).act(home_obs(), make_goal(*CENTER_XY))
    assert len(spy.calls) == 4  # 【確認内容】pre_grasp、grasp、pre_place、place のみ (lift / retreat は呼ばない)
    for _, _, kwargs in spy.calls:
        assert tuple(kwargs["tcp_offset"]) == (0.004, 0.0, 0.030)
    p1 = traj.metadata["pick_pitch_deg"]
    p2 = traj.metadata["place_pitch_deg"]
    recs = records(traj)
    first = tuple(spy.calls[0][2].get("pitch_candidates_deg", PITCH_CANDIDATES_DEG))
    assert first == PITCH_CANDIDATES_DEG  # 【確認内容】pre_grasp は既定の候補
    assert tuple(spy.calls[1][2]["pitch_candidates_deg"]) == (p1,)  # 【確認内容】grasp は p1 だけ
    np.testing.assert_allclose(spy.calls[1][1][:5], recs["pre_grasp"].q[:5], atol=1e-12)  # シード = pre_grasp の解
    assert tuple(spy.calls[3][2]["pitch_candidates_deg"]) == (p2,)  # 【確認内容】place は p2 だけ
    np.testing.assert_allclose(spy.calls[3][1][:5], recs["pre_place"].q[:5], atol=1e-12)  # シード = pre_place の解


# =============================================================== 3.2 異常系
def _act_error(policy, goal, obs=None):
    with pytest.raises(TrajectoryError) as info:
        policy.act(home_obs() if obs is None else obs, goal)
    return info.value


def test_x_out_of_workspace_rejected_before_ik(model, monkeypatch):
    """TC-IP-E01 作業範囲外 (x) は IK の前に拒否 🔵"""
    spy = SolveSpy()
    monkeypatch.setattr(ik_policy_module, "solve_ik", spy)
    err = _act_error(IkPolicy(model), make_goal(0.45, 0.0))
    assert (err.reason, err.waypoint) == ("target_out_of_workspace", "pre_grasp")
    assert len(spy.calls) == 0  # 【確認内容】IK は 0 回


@pytest.mark.parametrize(
    "pick, waypoint",
    [((0.25, 0.27, 0.015), "pre_grasp"), ((0.25, 0.0, -0.01), "grasp")],
    ids=["y_high", "z_negative"],
)
def test_y_z_out_of_workspace_rejected_before_ik(model, monkeypatch, pick, waypoint):
    """TC-IP-E02 作業範囲外 (y、z) 🔵"""
    spy = SolveSpy()
    monkeypatch.setattr(ik_policy_module, "solve_ik", spy)
    err = _act_error(IkPolicy(model), make_goal(*pick))
    assert (err.reason, err.waypoint) == ("target_out_of_workspace", waypoint)
    assert len(spy.calls) == 0


def test_place_too_high_rejected_before_ik(model, monkeypatch):
    """TC-IP-E03 置く点が高すぎる 🔵"""
    spy = SolveSpy()
    monkeypatch.setattr(ik_policy_module, "solve_ik", spy)
    err = _act_error(IkPolicy(model), make_goal(*CENTER_XY, place=(0.20, 0.19, 0.15)))
    assert (err.reason, err.waypoint) == ("target_out_of_workspace", "place")  # 【確認内容】place の z 0.18 > 0.17
    assert len(spy.calls) == 0


def test_carry_too_low(model, traj_cfg, monkeypatch):
    """TC-IP-E04 運搬が低すぎる 🔴(理由名)/ 🔵(値)"""
    spy = SolveSpy()
    monkeypatch.setattr(ik_policy_module, "solve_ik", spy)
    cfg = dataclasses.replace(traj_cfg, approach_height=0.05)
    err = _act_error(IkPolicy(model, config=cfg), make_goal(*CENTER_XY))
    assert (err.reason, err.waypoint) == ("carry_too_low", "lift")
    assert err.detail["z"] == pytest.approx(0.065)  # 【確認内容】lift の z
    assert len(spy.calls) == 0


def test_place_not_converged_is_wrapped(model, monkeypatch):
    """TC-IP-E05 IK の未収束を包む (置く側) 🔵"""
    spy = SolveSpy()
    monkeypatch.setattr(ik_policy_module, "solve_ik", spy)
    err = _act_error(IkPolicy(model), make_goal(*CENTER_XY, place=(0.20, 0.19, 0.14)))
    assert (err.reason, err.waypoint) == ("not_converged", "place")
    assert isinstance(err.__cause__, IkError) and err.__cause__.reason == "not_converged"
    assert len(spy.calls) == 4  # 【確認内容】place の IK まで 4 回呼ぶ


def test_near_shoulder_not_converged(policy):
    """TC-IP-E06 肩の近くの点 (調査の失敗例) 🔵"""
    err = _act_error(policy, make_goal(*NEAR_SHOULDER[:2]))
    assert (err.reason, err.waypoint) == ("not_converged", "pre_grasp")
    assert isinstance(err.__cause__, IkError)


def test_out_of_reach_is_wrapped(model, monkeypatch):
    """TC-IP-E07 届かない (out_of_reach) を包む 🟡"""
    raised = ik_error("out_of_reach")
    monkeypatch.setattr(ik_policy_module, "solve_ik", SolveSpy(fail_on=1, error=raised))
    err = _act_error(IkPolicy(model), make_goal(*CENTER_XY))
    assert (err.reason, err.waypoint) == ("out_of_reach", "pre_grasp")
    assert err.__cause__ is raised  # 【確認内容】投げた例外そのもの


def test_failure_names_the_right_waypoint(model, monkeypatch):
    """TC-IP-E08 経由点の名前が正しく付く 🟡"""
    monkeypatch.setattr(ik_policy_module, "solve_ik", SolveSpy(fail_on=3, error=ik_error("not_converged")))
    err = _act_error(IkPolicy(model), make_goal(*CENTER_XY))
    assert err.waypoint == "pre_place"  # 【確認内容】3 回目の IK は pre_place
    assert "target" in err.detail  # 【確認内容】detail に目標点がある


def test_command_sequence_check_is_guarded(model, monkeypatch):
    """TC-IP-E09 防御: 指令列の検査 🟡"""
    monkeypatch.setattr(trajectory_module, "segment_steps", lambda *a, **k: 1)
    err = _act_error(IkPolicy(model), make_goal(*CENTER_XY))
    assert err.reason == "limit_violation"  # 【確認内容】時間配分の誤りを指令列の検査が止める


def test_folded_elbow_is_self_collision(policy):
    """TC-IP-E10 自己干渉 (肘を折り畳んだ解) 🔵"""
    err = _act_error(policy, make_goal(0.17, 0.0))
    assert (err.reason, err.waypoint) == ("self_collision", "grasp")
    assert err.detail["min"] < 0  # 【確認内容】実測 -5.1 mm


def test_survey_failure_reproduced(model, traj_cfg):
    """TC-IP-E11 自己干渉: 調査の失敗例の再現 🔵"""
    cfg = dataclasses.replace(traj_cfg, tcp_offset=(0.0, 0.0, 0.030))
    err = _act_error(IkPolicy(model, config=cfg), make_goal(*NEAR_SHOULDER[:2]))
    assert err.reason in {"obstacle_clearance", "self_collision"}  # 【確認内容】検査の順で理由が変わる (O4)
    assert err.waypoint == "grasp"
    assert err.detail["min"] < 0


def test_obstacle_threshold_raised(model, traj_cfg):
    """TC-IP-E12 障害物のしきい値を上げる 🟡"""
    cfg = with_clearance(traj_cfg, obstacle_m=0.010)
    err = _act_error(IkPolicy(model, config=cfg), make_goal(*CENTER_XY))
    assert err.reason == "obstacle_clearance"
    assert err.waypoint in {"place", "open", "open_hold"}  # 【確認内容】実測は place
    assert err.detail["min"] == pytest.approx(0.0070, abs=5e-4)


def test_self_threshold_raised(model, traj_cfg):
    """TC-IP-E13 自己干渉のしきい値を上げる 🟡"""
    cfg = with_clearance(traj_cfg, self_m=0.020)
    err = _act_error(IkPolicy(model, config=cfg), make_goal(*CENTER_XY))
    assert err.reason == "self_collision"  # 【確認内容】waypoint は記録のみ (実測 pre_grasp)
    assert err.detail["min"] == pytest.approx(0.0096, abs=1e-3)


def test_path_leaving_workspace(model, safety_cfg):
    """TC-IP-E14 補間した経路が作業範囲の外へ出る 🟡"""
    narrow = WorkspaceBox(lower=(0.12, -0.13, 0.0), upper=(0.252, 0.26, 0.17))
    cfg = dataclasses.replace(safety_cfg, workspace=narrow)
    err = _act_error(IkPolicy(model, safety=cfg), make_goal(*CENTER_XY))
    assert (err.reason, err.waypoint) == ("path_out_of_workspace", "grasp")  # 【確認内容】降下の途中で x が張り出す


def test_policy_reusable_after_failure(model, solved):
    """TC-IP-E15 失敗しても使い続けられる 🟡"""
    policy = IkPolicy(model)
    _act_error(policy, make_goal(0.17, 0.0))
    again = policy.act(home_obs(), make_goal(*CENTER_XY))
    assert np.array_equal(again.q, solved["center"].q)  # 【確認内容】状態を持ち越さない


@pytest.mark.parametrize("bad", ["obs", "goal"])
def test_invalid_inputs_raise_value_error(policy, bad):
    """TC-IP-E16 入力の誤りは ValueError 🟡"""
    obs, goal = home_obs(), make_goal(*CENTER_XY)
    if bad == "obs":
        obs = None
    else:
        goal = {"pick_target_base": PICK_CENTER}
    with pytest.raises(ValueError) as info:
        policy.act(obs, goal)
    assert not isinstance(info.value, TrajectoryError)  # 【確認内容】TrajectoryError ではない


def test_constructor_validation(model, traj_cfg):
    """TC-IP-E17 構築時の検査 🟡"""
    with pytest.raises(ValueError, match="no_such_geom"):  # (a) 存在しない geom 名
        IkPolicy(model, config=with_clearance(traj_cfg, obstacle_geoms=("no_such_geom",)))
    with pytest.raises(ValueError, match="gripper_open"):  # (b) ctrlrange の上限 1.745 より上
        IkPolicy(model, config=dataclasses.replace(traj_cfg, gripper_open=2.0))
    no_box = mujoco.MjModel.from_xml_path(str(SO101_SCENE_PATH))
    with pytest.raises(ValueError, match="box_floor"):  # (c) 箱の geom がないモデル
        IkPolicy(no_box)


# =============================================================== 3.3 境界値
def test_carry_height_exactly_at_minimum(model, traj_cfg):
    """TC-IP-B01 運搬の高さが下限ちょうど 🟡"""
    cfg = dataclasses.replace(traj_cfg, approach_height=0.065)
    traj = IkPolicy(model, config=cfg).act(home_obs(), make_goal(*CENTER_XY))  # lift の z = 0.080
    assert traj.segments == SEGMENTS  # 【確認内容】例外なし (≥ は合格)


def test_place_target_on_workspace_boundary(model, monkeypatch):
    """TC-IP-B02 作業範囲の角の上の目標 🔵"""
    spy = SolveSpy()
    monkeypatch.setattr(ik_policy_module, "solve_ik", spy)
    err = _act_error(IkPolicy(model), make_goal(*CENTER_XY, place=(0.20, 0.19, 0.14)))
    assert err.reason != "target_out_of_workspace"  # 【確認内容】境界を含むので作業範囲の検査は通る
    assert err.reason == "not_converged" and len(spy.calls) == 4  # 【確認内容】IK で失敗 (= TC-IP-E05)


@pytest.mark.parametrize("name", list(CORNERS))
def test_layout_corners_are_solvable(solved, name):
    """TC-IP-B03 配置範囲の 4 隅 🔵"""
    assert solved[name].segments == SEGMENTS  # 【確認内容】4 隅はすべて解ける


def test_closest_solvable_point_near_shoulder(policy):
    """TC-IP-B04 肩に近い側で解ける最も近い点 🔵"""
    traj = policy.act(home_obs(), make_goal(0.18, 0.0))
    assert traj.metadata["min_self_clearance_m"] >= 0.002  # 【確認内容】自己干渉のすき間 2 mm 以上


# =============================================================== 3.4 実行スモーク
def test_sim_robot_executes_trajectory(policy, safety_cfg, limits):
    """TC-IP-S01 SimRobot で最後まで動く 🟡 (ブロックの運搬は確かめない。物理設定は TASK-0013)"""
    with SimRobot(PICKPLACE_SCENE_PATH) as robot:
        state = robot.get_state()
        traj = policy.act(Observation.from_state(state), make_goal(*CENTER_XY))
        targets = targets_of(traj)
        prev, flagged = traj.q[0], 0

        def cycle(q_target, label):
            nonlocal prev, flagged
            now = robot.get_state().timestamp
            res = apply_safety(
                prev, q_target, lower=limits[0], upper=limits[1], last_state_t=now, now=now,
                config=safety_cfg, target_base=targets[SEG_TARGET[label]],
            )  # fmt: skip
            flagged += int(res.clipped or res.rate_limited or res.rejected or res.stopped)
            robot.send_command(res.q_command)
            prev = res.q_command

        for k in range(1, len(traj.q)):
            cycle(traj.q[k], traj.labels[k])
        end_error = float(np.max(np.abs(robot.get_state().q - traj.q[-1])))
        for _ in range(25):
            cycle(traj.q[-1], "retreat")
        settled_error = float(np.max(np.abs(robot.get_state().q - traj.q[-1])))
    assert flagged == 0  # 【確認内容】全周期でフラグなし (実測 0 件)
    assert end_error <= 0.01  # 【確認内容】軌道の終了時の関節誤差 (実測の最大 0.0044)
    assert settled_error <= 0.002  # 【確認内容】25 周期後 (実測 0.0004)

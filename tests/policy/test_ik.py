"""solve_ik (姿勢制約付き逆運動学) の単体テスト (TASK-0011, Red フェーズ)。

テストケース定義: docs/implements/so101-vlm-pickplace/TASK-0011/ik-testcases.md (T1..T35, T6b)
信頼性: 🔵 要件・設計・実測に基づく / 🟡 妥当な推測 / 🔴 要件にない仮定
"""

import copy
import dataclasses
import inspect
import random
import time

import mujoco
import numpy as np
import pytest

from so101_vlm.policy.ik import (
    APPROACH_AXIS_LOCAL,
    DAMPING,
    PITCH_CANDIDATES_DEG,
    PITCH_TOL_DEG,
    POS_TOL_M,
    IkError,
    IkSolution,
    solve_ik,
)
from so101_vlm.robot.model_paths import JOINT_NAMES, PICKPLACE_SCENE_PATH
from so101_vlm.robot.sim import SimRobot

# --- 定数 (testcases.md 0.1) ---
POS_TOL_TEST_M = 0.002
PITCH_TOL_TEST_DEG = 5.0
DEFAULT_CANDIDATES = (0.0, 10.0, 20.0, 30.0, 45.0, 60.0)
SEED_ZERO = np.zeros(6)
GRASP_CENTER = (0.20, 0.0, 0.015)
PREGRASP = (0.25, 0.05, 0.095)
FAR_TARGET = (1.0, 0.0, 0.2)
NEAR_HIGH_TARGET = (0.10, 0.0, 0.095)
BEHIND_TARGET = (-0.15, 0.0, 0.015)
TCP_OFFSET = (-0.0075, 0.0, 0.015)  # 把持中心の暫定オフセット (テスト専用定数, O4)
PAN_AXIS_POINT = np.array([0.0388, 0.0, 0.1166])
ROBUST_OUT_DIST_M = 0.452

OUT_OF_REACH_BUDGET_S = 0.020
TYPICAL_SOLVE_BUDGET_S = 0.050
NOT_CONVERGED_BUDGET_S = 3.0
N_TIMING_RUNS = 3


# --- 独立 FK (IK 実装とは別の MjData + mj_kinematics) ---
def fk_point(model, q, offset=(0.0, 0.0, 0.0)):
    """q から gripperframe + R @ offset の位置と、接近軸(+x)の傾き(度)を返す。"""
    data = mujoco.MjData(model)
    for name, value in zip(JOINT_NAMES, q, strict=True):
        data.qpos[model.joint(name).qposadr[0]] = value
    mujoco.mj_kinematics(model, data)
    site = model.site("gripperframe").id
    rot = data.site_xmat[site].reshape(3, 3)
    pos = data.site_xpos[site] + rot @ np.asarray(offset, dtype=float)
    pitch = float(np.degrees(np.arccos(np.clip(-rot[2, 0], -1.0, 1.0))))
    return pos, pitch


def _assert_valid_solution(sol, target, ctrlrange, candidates=DEFAULT_CANDIDATES):
    """T26: 収束判定の共通確認 (位置・許容帯・候補・ctrlrange)。"""
    assert isinstance(sol, IkSolution)
    assert sol.pos_error <= POS_TOL_TEST_M  # 【確認内容】位置誤差が許容内
    assert abs(sol.pitch_deg - sol.pitch_candidate_deg) <= PITCH_TOL_TEST_DEG  # 【確認内容】許容帯
    assert sol.pitch_candidate_deg in candidates  # 【確認内容】渡した候補のいずれか
    assert np.all(sol.q >= ctrlrange[:, 0]) and np.all(sol.q <= ctrlrange[:, 1])  # 【確認内容】可動域内


# --- fixture ---
@pytest.fixture(scope="module")
def scene_model():
    # 【テスト前準備】: モデル読み込みは重いのでモジュールで 1 回だけ行う
    return mujoco.MjModel.from_xml_path(str(PICKPLACE_SCENE_PATH))


@pytest.fixture(scope="module")
def ctrlrange(scene_model):
    # 【テスト前準備】: actuator_ctrlrange を JOINT_NAMES 順に名前で引く
    rows = [scene_model.actuator_ctrlrange[scene_model.actuator(n).id] for n in JOINT_NAMES]
    return np.array(rows, dtype=float)


@pytest.fixture(scope="module")
def grasp_solution(scene_model):
    # 【テスト前準備】: 呼び出し回数を減らすため把持点の解を共有する
    return solve_ik(scene_model, GRASP_CENTER, SEED_ZERO)


@pytest.fixture(scope="module")
def near_high_error(scene_model):
    # 【テスト前準備】: 収束しない呼び出しは重いので 1 回だけ実行し、例外と所要時間を共有する
    start = time.perf_counter()
    with pytest.raises(IkError) as info:
        solve_ik(scene_model, NEAR_HIGH_TARGET, SEED_ZERO)
    return info.value, time.perf_counter() - start


# =====================================================================
# 1. 正常系
# =====================================================================
def test_reachable_grasp_point_converges_within_2mm_near_vertical(grasp_solution):
    # 【テスト目的】: 到達可能な把持点で 2 mm 以内・候補 0° で収束する (T1)
    # 🟡 数値は参照ソルバで確認済み
    sol = grasp_solution
    assert sol.pos_error <= POS_TOL_TEST_M  # 【確認内容】完了条件「収束する」
    assert sol.q.shape == (6,)  # 【確認内容】send_command と同じ 6 要素
    assert sol.q.dtype == np.float64  # 【確認内容】float64
    assert sol.pitch_candidate_deg == 0.0  # 【確認内容】真下に近い候補から採用
    assert sol.pitch_deg <= 5.0  # 【確認内容】許容 5° の境界付近 (厳しい値は使わない)


def test_pregrasp_point_uses_tilted_candidate(scene_model, ctrlrange):
    # 【テスト目的】: 0° が不可の接近点で次の候補に進む (T2) 🟡
    sol = solve_ik(scene_model, PREGRASP, SEED_ZERO)
    assert sol.pos_error <= POS_TOL_TEST_M  # 【確認内容】位置誤差
    assert sol.pitch_candidate_deg in (10.0, 20.0)  # 【確認内容】0° の解を誤って返さない
    assert abs(sol.pitch_deg - sol.pitch_candidate_deg) <= PITCH_TOL_TEST_DEG  # 【確認内容】許容帯
    _assert_valid_solution(sol, PREGRASP, ctrlrange)


@pytest.mark.parametrize(
    "target",
    [GRASP_CENTER, (0.15, -0.10, 0.015), (0.20, 0.19, 0.06), (0.35, 0.10, 0.015)],
    ids=["center", "near_corner", "box_release", "far_corner"],
)
def test_solution_within_ctrlrange(scene_model, ctrlrange, target):
    # 【テスト目的】: 解の 6 関節 (gripper/wrist_roll を含む) が ctrlrange 内 (T3) 🔵
    sol = solve_ik(scene_model, target, SEED_ZERO)
    assert np.all(sol.q >= ctrlrange[:, 0])  # 【確認内容】下限以上 (等号を含む)
    assert np.all(sol.q <= ctrlrange[:, 1])  # 【確認内容】上限以下 (等号を含む)


def test_solution_is_frozen_and_q_readonly(grasp_solution):
    # 【テスト目的】: IkSolution の不変性 (T4) 🟡
    sol = grasp_solution
    with pytest.raises(dataclasses.FrozenInstanceError):
        sol.pitch_deg = 0.0  # type: ignore[misc]
    assert sol.q.flags.writeable is False  # 【確認内容】q は書き込み不可
    with pytest.raises(ValueError):
        sol.q[0] = 1.0  # 【確認内容】要素代入は ValueError
    assert isinstance(sol.pos_error, float)  # 【確認内容】Python float
    assert isinstance(sol.pitch_deg, float)
    assert isinstance(sol.pitch_candidate_deg, float)


def test_gripper_and_wrist_roll_copied_from_seed(scene_model):
    # 【テスト目的】: q[4], q[5] は seed のコピー (T5) 🟡
    seed = (0.1, 0.2, 0.3, 0.4, 0.7, 1.0)
    sol = solve_ik(scene_model, GRASP_CENTER, seed)
    assert sol.q[4] == 0.7  # 【確認内容】wrist_roll は解かずにコピー
    assert sol.q[5] == 1.0  # 【確認内容】gripper は動かさない
    assert sol.pos_error <= POS_TOL_TEST_M  # 【確認内容】残り 4 関節で収束


def test_tcp_offset_point_lands_on_target(scene_model):
    # 【テスト目的】: tcp_offset (局所座標) を加えた点が目標に乗る (T6) 🟡
    sol = solve_ik(scene_model, GRASP_CENTER, SEED_ZERO, tcp_offset=TCP_OFFSET)
    target = np.array(GRASP_CENTER)
    p_off, _ = fk_point(scene_model, sol.q, TCP_OFFSET)
    p_site, _ = fk_point(scene_model, sol.q)
    assert np.linalg.norm(p_off - target) <= POS_TOL_TEST_M  # 【確認内容】オフセット点が目標に一致
    assert np.linalg.norm(p_site - target) > 0.010  # 【確認内容】オフセットが実際に効いている
    assert sol.pos_error <= POS_TOL_TEST_M


def test_tcp_offset_in_local_frame_at_tilted_pose(scene_model):
    # 【テスト目的】: 傾いた姿勢でも tcp_offset が局所座標で効く (T6b) 🟡
    target = (0.35, 0.10, 0.015)
    sol = solve_ik(scene_model, target, SEED_ZERO, tcp_offset=TCP_OFFSET)
    p_off, _ = fk_point(scene_model, sol.q, TCP_OFFSET)
    assert np.linalg.norm(p_off - np.array(target)) <= POS_TOL_TEST_M  # 【確認内容】世界座標加算では約 19 mm ずれる
    assert sol.pitch_candidate_deg >= 10.0  # 【確認内容】傾いた候補で解けている


@pytest.mark.parametrize("which", ["grasp", "pregrasp"])
def test_independent_fk_matches_target_and_reported_values(scene_model, grasp_solution, which):
    # 【テスト目的】: pos_error / pitch_deg が q から再現できる (T7) 🟡
    target = np.array(GRASP_CENTER if which == "grasp" else PREGRASP)
    sol = grasp_solution if which == "grasp" else solve_ik(scene_model, PREGRASP, SEED_ZERO)
    p_fk, pitch_fk = fk_point(scene_model, sol.q)
    err = np.linalg.norm(p_fk - target)
    assert err <= POS_TOL_TEST_M  # 【確認内容】独立 FK でも目標に届く
    assert abs(err - sol.pos_error) <= 1e-6  # 【確認内容】報告誤差が自己申告でない
    assert abs(pitch_fk - sol.pitch_deg) <= 1e-4  # 【確認内容】接近軸を +x として扱う
    assert abs(pitch_fk - sol.pitch_candidate_deg) <= PITCH_TOL_TEST_DEG


def test_single_candidate_restricts_pitch_band(scene_model):
    # 【テスト目的】: 候補 1 個ならその帯の解だけを返す (T8) 🟡
    sol = solve_ik(scene_model, (0.25, 0.0, 0.14), SEED_ZERO, pitch_candidates_deg=(20,))
    assert sol.pitch_candidate_deg == 20.0  # 【確認内容】指定候補
    assert 15.0 <= sol.pitch_deg <= 25.0  # 【確認内容】20° の帯
    assert sol.pos_error <= POS_TOL_TEST_M


def test_candidates_tried_in_given_order(scene_model):
    # 【テスト目的】: 候補は並べ替えず渡した順に試す (T9) 🟡
    sol = solve_ik(scene_model, GRASP_CENTER, SEED_ZERO, pitch_candidates_deg=(30, 0))
    assert sol.pitch_candidate_deg == 30.0  # 【確認内容】先頭の 30° を採用
    assert 25.0 <= sol.pitch_deg <= 35.0


def test_accepts_array_like_inputs(scene_model, grasp_solution):
    # 【テスト目的】: list/tuple 入力でも ndarray と同じ結果 (T10) 🟡
    sol = solve_ik(
        scene_model, [0.20, 0.0, 0.015], (0,) * 6,
        tcp_offset=[0, 0, 0], pitch_candidates_deg=[0, 10, 20, 30, 45, 60],
    )
    assert np.array_equal(sol.q, grasp_solution.q)  # 【確認内容】q 一致
    assert sol.pitch_deg == grasp_solution.pitch_deg
    assert sol.pitch_candidate_deg == grasp_solution.pitch_candidate_deg
    assert sol.pos_error == grasp_solution.pos_error


def test_warm_start_from_pregrasp_solution(scene_model):
    # 【テスト目的】: 接近点の解 (読み取り専用) を seed に把持点を解ける (T11) 🟡
    q_a = solve_ik(scene_model, PREGRASP, SEED_ZERO).q
    sol = solve_ik(scene_model, (0.25, 0.05, 0.015), q_a)
    assert sol.pos_error <= POS_TOL_TEST_M  # 【確認内容】ウォームスタートで収束
    assert sol.q[4] == q_a[4]  # 【確認内容】roll を引き継ぐ
    assert sol.q[5] == q_a[5]  # 【確認内容】gripper を引き継ぐ


# =====================================================================
# 2. 異常系
# =====================================================================
def test_far_target_raises_out_of_reach(scene_model):
    # 【テスト目的】: 到達球の外は反復せず out_of_reach (T12) 🔵 FR-06
    with pytest.raises(IkError) as info:
        solve_ik(scene_model, FAR_TARGET, SEED_ZERO)
    exc = info.value
    assert isinstance(exc, RuntimeError)  # 【確認内容】RuntimeError の子
    assert exc.reason == "out_of_reach"  # 【確認内容】理由
    assert exc.target == (1.0, 0.0, 0.2)  # 【確認内容】目標を保持
    assert isinstance(exc.target, tuple) and all(isinstance(v, float) for v in exc.target)
    assert exc.tried_pitches_deg == ()  # 【確認内容】反復していない
    assert exc.best_pitch_deg is None
    assert exc.best_pos_error > 0  # 【確認内容】推定の距離超過分
    assert "1.0" in str(exc) and "0.2" in str(exc)  # 【確認内容】メッセージに目標


def test_unreachable_target_inside_sphere_raises_not_converged(near_high_error):
    # 【テスト目的】: 球内だが届かない点は not_converged (T13) 🟡
    exc, _ = near_high_error
    assert exc.reason == "not_converged"  # 【確認内容】理由
    assert exc.best_pos_error > 0.001  # 【確認内容】1 mm 超の誤差が残る
    assert exc.tried_pitches_deg == DEFAULT_CANDIDATES  # 【確認内容】全候補を試した
    assert isinstance(exc.best_pitch_deg, float)
    assert "0.1" in str(exc) and "mm" in str(exc)  # 【確認内容】目標と mm 表記の誤差


def test_target_behind_base_raises_not_converged(scene_model):
    # 【テスト目的】: pan の可動域外の点で可動域外の解を返さない (T14) 🟡
    with pytest.raises(IkError) as info:
        solve_ik(scene_model, BEHIND_TARGET, SEED_ZERO)
    assert info.value.reason == "not_converged"  # 【確認内容】球内なので反復して失敗
    assert info.value.best_pos_error > 0.05  # 【確認内容】大きく届かない


def test_vertical_only_candidate_fails_for_high_point(scene_model):
    # 【テスト目的】: 真下のみだと高い点は not_converged (T15) 🟡
    with pytest.raises(IkError) as info:
        solve_ik(scene_model, (0.25, 0.0, 0.14), SEED_ZERO, pitch_candidates_deg=(0,))
    exc = info.value
    assert exc.reason == "not_converged"  # 【確認内容】可動域外の解を返さない
    assert exc.tried_pitches_deg == (0.0,)  # 【確認内容】試した候補
    assert isinstance(exc.best_pitch_deg, float)


@pytest.mark.parametrize(
    "bad_target",
    [[0.2, 0.0], [0.2, 0.0, 0.0, 0.0], [[0.2], [0.0], [0.015]],
     [float("nan"), 0.0, 0.015], [0.2, float("inf"), 0.015]],
    ids=["shape2", "shape4", "shape3x1", "nan", "inf"],
)
def test_invalid_target_raises_value_error(scene_model, bad_target):
    # 【テスト目的】: 不正な target_pos は ValueError で IkError ではない (T16) 🟡
    with pytest.raises(ValueError, match="target") as info:
        solve_ik(scene_model, bad_target, SEED_ZERO)
    assert not isinstance(info.value, IkError)  # 【確認内容】入力の誤りと IK 失敗を区別


@pytest.mark.parametrize(
    "bad_seed",
    [np.zeros(5), np.zeros(7), [0, 0, 0, 0, float("nan"), 0], [0, 0, 0, 0, 0, float("-inf")]],
    ids=["len5", "len7", "nan", "-inf"],
)
def test_invalid_seed_raises_value_error(scene_model, bad_seed):
    # 【テスト目的】: 不正な seed_q は ValueError (T17) 🟡
    with pytest.raises(ValueError):
        solve_ik(scene_model, GRASP_CENTER, bad_seed)


@pytest.mark.parametrize(
    "bad_candidates",
    [(), (-5,), (95,), (float("nan"),), (0, 120)],
    ids=["empty", "negative", "over90", "nan", "mixed"],
)
def test_invalid_pitch_candidates_raise_value_error(scene_model, bad_candidates):
    # 【テスト目的】: 空または [0, 90] 外の候補は ValueError (T18) 🔴
    with pytest.raises(ValueError):
        solve_ik(scene_model, GRASP_CENTER, SEED_ZERO, pitch_candidates_deg=bad_candidates)


@pytest.mark.parametrize(
    "bad_offset",
    [[0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [float("nan"), 0, 0], [0, 0, float("inf")]],
    ids=["shape2", "shape4", "nan", "inf"],
)
def test_invalid_tcp_offset_raises_value_error(scene_model, bad_offset):
    # 【テスト目的】: 不正な tcp_offset は ValueError (T19) 🟡
    with pytest.raises(ValueError):
        solve_ik(scene_model, GRASP_CENTER, SEED_ZERO, tcp_offset=bad_offset)


# =====================================================================
# 3. 境界値
# =====================================================================
def _just_outside_points():
    dirs = [np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0.6, 0.3, -0.2])]
    return [PAN_AXIS_POINT + ROBUST_OUT_DIST_M * d / np.linalg.norm(d) for d in dirs]


@pytest.mark.parametrize("target", _just_outside_points(), ids=["+x", "+y", "diag"])
def test_just_outside_reach_raises_out_of_reach_quickly(scene_model, target):
    # 【テスト目的】: どの姿勢でも届かない点を即座に out_of_reach (T20) 🟡
    start = time.perf_counter()
    with pytest.raises(IkError) as info:
        solve_ik(scene_model, target, SEED_ZERO)
    elapsed = time.perf_counter() - start
    assert info.value.reason == "out_of_reach"  # 【確認内容】方向によらず同じ扱い
    assert elapsed < OUT_OF_REACH_BUDGET_S  # 【確認内容】反復しない (< 20 ms)


def test_reachable_point_near_outer_limit_is_solved(scene_model, ctrlrange):
    # 【テスト目的】: 外縁の届く点を out_of_reach と誤判定しない (T21) 🟡
    target = (0.42, 0.0, 0.015)
    sol = solve_ik(scene_model, target, SEED_ZERO)
    assert sol.pitch_candidate_deg in (45.0, 60.0)  # 【確認内容】予備候補の経路
    _assert_valid_solution(sol, target, ctrlrange)


@pytest.mark.parametrize("side", ["upper", "lower"])
def test_seed_outside_ctrlrange_is_clipped(scene_model, ctrlrange, side):
    # 【テスト目的】: ctrlrange 外の seed はクリップして使う (T22) 🔴
    if side == "upper":
        seed, expect = (0, 0, 0, 0, 3.0, 2.0), ctrlrange[4:6, 1]
    else:
        seed, expect = (0, 0, 0, 0, -3.0, -0.5), ctrlrange[4:6, 0]
    sol = solve_ik(scene_model, GRASP_CENTER, seed)
    assert np.allclose(sol.q[4:6], expect, atol=1e-12)  # 【確認内容】範囲端にクリップ
    assert sol.pos_error <= POS_TOL_TEST_M


def test_candidate_bounds_0_and_90_are_accepted(scene_model):
    # 【テスト目的】: 候補の閉区間の両端は ValueError にしない (T23) 🔴
    sol = solve_ik(scene_model, GRASP_CENTER, SEED_ZERO, pitch_candidates_deg=(0,))
    assert isinstance(sol, IkSolution)  # 【確認内容】0 は解を返す
    try:
        solve_ik(scene_model, GRASP_CENTER, SEED_ZERO, pitch_candidates_deg=(90,))
    except IkError:
        pass  # 【確認内容】90 は解でも IkError でもよい (ValueError でなければよい)


def test_target_below_floor_is_solved_kinematically(scene_model):
    # 【テスト目的】: IK は床を見ない (T24) 🔴
    sol = solve_ik(scene_model, (0.20, 0.0, -0.05), SEED_ZERO)
    assert sol.pos_error <= POS_TOL_TEST_M  # 【確認内容】運動学的に届けば解を返す


def test_module_constants():
    # 【テスト目的】: 名前付き定数の値 (T25) 🟡
    assert tuple(float(v) for v in PITCH_CANDIDATES_DEG) == (0.0, 10.0, 20.0, 30.0, 45.0, 60.0)
    assert PITCH_TOL_DEG == 5.0  # 【確認内容】許容帯
    assert POS_TOL_M == 0.001
    assert POS_TOL_M < POS_TOL_TEST_M  # 【確認内容】テストしきい値に余裕
    assert DAMPING > 0  # 【確認内容】減衰 0 は特異姿勢で不可
    assert tuple(float(v) for v in APPROACH_AXIS_LOCAL) == (1.0, 0.0, 0.0)  # 【確認内容】接近軸は +x
    default = inspect.signature(solve_ik).parameters["pitch_candidates_deg"].default
    assert default == PITCH_CANDIDATES_DEG  # 【確認内容】既定の候補


# =====================================================================
# 4. 副作用・不変性
# =====================================================================
def test_inputs_not_mutated(scene_model):
    # 【テスト目的】: 入力を変更せず、読み取り専用入力も受け付ける (T27) 🟡
    target = np.array([0.25, 0.05, 0.095])
    seed = np.array([0.1, 0.2, 0.3, 0.4, 0.7, 1.0])
    offset = np.array([-0.0075, 0.0, 0.015])
    cands = [0, 10, 20]
    before = (target.copy(), seed.copy(), offset.copy(), list(cands))
    solve_ik(scene_model, target, seed, tcp_offset=offset, pitch_candidates_deg=cands)
    assert np.array_equal(target, before[0])  # 【確認内容】target 不変
    assert np.array_equal(seed, before[1])
    assert np.array_equal(offset, before[2])
    assert cands == before[3]
    for arr in (target, seed, offset):
        arr.flags.writeable = False
    solve_ik(scene_model, target, seed, tcp_offset=offset, pitch_candidates_deg=cands)  # 【確認内容】書き込み不可でも動く


def test_live_mjdata_untouched(scene_model):
    # 【テスト目的】: 同じモデルの既存 MjData を使い回さない (T28) 🟡
    data = mujoco.MjData(scene_model)
    for name, v in zip(JOINT_NAMES, (0.3, -0.2, 0.4, 0.5, 0.6, 0.2), strict=True):
        data.qpos[scene_model.joint(name).qposadr[0]] = v
    data.ctrl[:] = 0.1
    data.time = 1.234
    mujoco.mj_forward(scene_model, data)
    snap = {k: getattr(data, k).copy() for k in ("qpos", "qvel", "ctrl", "site_xpos")}
    solve_ik(scene_model, PREGRASP, SEED_ZERO)
    for k, v in snap.items():
        assert np.array_equal(getattr(data, k), v), k  # 【確認内容】既存 MjData が不変
    assert data.time == 1.234


def test_sim_robot_state_unchanged():
    # 【テスト目的】: SimRobot の状態が IK の前後で変わらない (T29) 🟡
    robot = SimRobot(PICKPLACE_SCENE_PATH)
    try:
        for _ in range(5):
            robot.send_command(np.array([0.2, -0.3, 0.4, 0.5, 0.1, 0.3]))
        before = robot.get_state()
        model = mujoco.MjModel.from_xml_path(str(PICKPLACE_SCENE_PATH))
        solve_ik(model, GRASP_CENTER, before.q)
        after = robot.get_state()
        assert np.array_equal(after.q, before.q)  # 【確認内容】q 不変
        assert np.array_equal(after.dq, before.dq)  # 【確認内容】dq 不変
        assert after.timestamp == before.timestamp
    finally:
        robot.close()


def test_model_not_mutated(scene_model):
    # 【テスト目的】: モデルのフィールドを書き換えない (T30) 🟡
    fields = ("qpos0", "jnt_range", "actuator_ctrlrange", "body_pos", "body_quat",
              "site_pos", "site_quat")
    before = {f: getattr(scene_model, f).copy() for f in fields}
    timestep = scene_model.opt.timestep
    solve_ik(scene_model, GRASP_CENTER, SEED_ZERO)
    solve_ik(scene_model, GRASP_CENTER, SEED_ZERO, tcp_offset=TCP_OFFSET)
    for f, v in before.items():
        assert np.array_equal(getattr(scene_model, f), v), f  # 【確認内容】不変
    assert scene_model.opt.timestep == timestep


# =====================================================================
# 5. 決定性
# =====================================================================
def test_deterministic_same_inputs(scene_model):
    # 【テスト目的】: 同じ入力で完全に同じ結果 (T31) 🟡
    s1 = solve_ik(scene_model, PREGRASP, SEED_ZERO)
    s2 = solve_ik(scene_model, PREGRASP, SEED_ZERO)
    assert np.array_equal(s1.q, s2.q)  # 【確認内容】許容なしの一致
    assert s1.pitch_deg == s2.pitch_deg
    assert s1.pitch_candidate_deg == s2.pitch_candidate_deg
    assert s1.pos_error == s2.pos_error


def test_independent_of_global_rng(scene_model):
    # 【テスト目的】: グローバル乱数に依存せず消費もしない (T32) 🟡
    np.random.seed(0)
    s1 = solve_ik(scene_model, GRASP_CENTER, SEED_ZERO)
    np.random.seed(12345)
    random.seed(7)
    np_state = np.random.get_state()
    py_state = random.getstate()
    s2 = solve_ik(scene_model, GRASP_CENTER, SEED_ZERO)
    assert np.array_equal(s1.q, s2.q)  # 【確認内容】乱数状態によらず一致
    assert np.array_equal(np.random.get_state()[1], np_state[1])  # 【確認内容】numpy 乱数を消費しない
    assert random.getstate() == py_state  # 【確認内容】random を消費しない


# =====================================================================
# 6. 性能 (しきい値は実測の 10 倍以上の余裕)
# =====================================================================
def test_out_of_reach_is_fast(scene_model):
    # 【テスト目的】: out_of_reach は即座 (T33) 🟡
    start = time.perf_counter()
    with pytest.raises(IkError):
        solve_ik(scene_model, FAR_TARGET, SEED_ZERO)
    assert time.perf_counter() - start < OUT_OF_REACH_BUDGET_S  # 【確認内容】< 20 ms


def test_typical_solve_under_50ms(scene_model):
    # 【テスト目的】: 候補 0° で解ける点は 50 ms 未満 (T34, O2: 0° の点のみ) 🟡
    times = []
    for _ in range(N_TIMING_RUNS):
        start = time.perf_counter()
        solve_ik(scene_model, GRASP_CENTER, SEED_ZERO)
        times.append(time.perf_counter() - start)
    assert min(times) < TYPICAL_SOLVE_BUDGET_S  # 【確認内容】初回遅延・GC を除いた最小値


def test_not_converged_within_3s(near_high_error):
    # 【テスト目的】: 収束しない点でも 3 秒以内に IkError (T35, O1: 停滞打ち切りは Green) 🟡
    _, elapsed = near_high_error
    assert elapsed < NOT_CONVERGED_BUDGET_S  # 【確認内容】時間予算

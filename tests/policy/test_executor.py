"""run_trajectory / ExecutionResult のテスト (TASK-0013, Red フェーズ)。

テストケース定義: docs/implements/so101-vlm-pickplace/TASK-0013/pick-testcases.md 3 節 (TC-EX-*, TC-EXI-*)
対象モジュール (まだ存在しない): so101_vlm.policy.executor
  (run_trajectory, ExecutionResult, SEGMENT_WAYPOINT, START_TOLERANCE_RAD)、SimRobot.mujoco_handles
前半: フェイク Robot (MuJoCo なし、< 0.2 s)。後半: SimRobot の統合 (計画 1 回 + 実行、約 1 s)。
実行時間の予算: このファイル < 1.5 s。
信頼性: 🔵 要件・実測に基づく / 🟡 妥当な推測 / 🔴 要件にない仮定
"""

import dataclasses
import math

import mujoco
import numpy as np
import pytest

import so101_vlm.policy.executor as executor_module
from so101_vlm.eval.layout import place_blocks, sample_layout
from so101_vlm.eval.scene_spec import SCENE_SPEC
from so101_vlm.policy import ik_policy
from so101_vlm.policy.base import Goal, Observation
from so101_vlm.policy.executor import (
    SEGMENT_WAYPOINT,
    START_TOLERANCE_RAD,
    ExecutionResult,
    run_trajectory,
)
from so101_vlm.policy.ik_policy import IkPolicy
from so101_vlm.policy.trajectory import JointTrajectory, WaypointRecord
from so101_vlm.robot.base import JOINT_NAMES, Robot, RobotState
from so101_vlm.robot.model_paths import PICKPLACE_SCENE_PATH, SO101_SCENE_PATH
from so101_vlm.robot.sim import SimRobot
from so101_vlm.safety.config import SafetyConfig, WorkspaceBox, load_safety_config
from so101_vlm.safety.limits import apply_safety, limits_from_model

DT = 0.02
SETTLE_STEPS_DEFAULT = 50  # ceil(1.0 / 0.02)
JOINT_ERR_TOL = 0.01
UNIT_LOWER, UNIT_UPPER = -np.ones(6), np.ones(6)
SAFETY = SafetyConfig(
    vmax=(1.0,) * 6,
    control_dt=DT,
    speed_scale=1.0,
    state_timeout=0.2,
    workspace=WorkspaceBox(lower=(0.12, -0.13, 0.00), upper=(0.38, 0.26, 0.17)),
)
INSIDE = (0.25, 0.0, 0.05)  # 作業範囲 x 0.12〜0.38、y −0.13〜0.26、z 0〜0.17 の内側
OUTSIDE = (0.5, 0.0, 0.05)
# 要件 3.2 の対応表 🔵
EXPECTED_SEGMENT_WAYPOINT = {
    "pre_grasp": "pre_grasp", "grasp": "grasp", "close": "grasp", "close_hold": "grasp",
    "lift": "lift", "pre_place": "pre_place", "place": "place", "open": "place",
    "open_hold": "place", "retreat": "retreat",
}  # fmt: skip
ALL_SEGMENTS = tuple(name for name, _, _ in ik_policy.SEGMENT_TABLE)


# --------------------------------------------------------------- フェイク Robot (0.6 節)
class FakeRobot:
    """【機能概要】: MuJoCo なしの Robot。最後に送った指令をそのまま状態にする。🟡

    nan_time_when_sent: 送信済みの指令数がこの値のときだけ get_state の時刻が NaN になる
    (「N 回目の指令を送る前の読み取り」を、実装の get_state 呼び出し回数に依らず指定する)。
    """

    def __init__(self, q0=None, dt=DT, nan_time_when_sent=()):
        self.joint_names = JOINT_NAMES
        self._q0 = np.zeros(6) if q0 is None else np.asarray(q0, dtype=np.float64)
        self._dt = dt
        self._nan_when = tuple(nan_time_when_sent)
        self.sent: list[np.ndarray] = []
        self.closed = False

    def _time(self):
        return len(self.sent) * self._dt

    def get_state(self):
        q = self.sent[-1] if self.sent else self._q0
        t = float("nan") if len(self.sent) in self._nan_when else self._time()
        return RobotState(q=q, dq=np.zeros(6), timestamp=t)

    def send_command(self, q_target):
        self.sent.append(np.array(q_target, copy=True))

    def close(self):
        self.closed = True


def make_traj(rows, labels, metadata=None):
    """【機能概要】: 0.02 s 刻みの JointTrajectory を作る (segments は labels の出現順)。"""
    rows = np.asarray(rows, dtype=np.float64)
    segments = tuple(dict.fromkeys(labels))
    return JointTrajectory(
        times=np.arange(len(rows)) * DT, q=rows, labels=tuple(labels), segments=segments,
        metadata=metadata or {},
    )  # fmt: skip


def pan_rows(values):
    """shoulder_pan だけ values、他は 0 の行列。"""
    rows = np.zeros((len(values), 6))
    rows[:, 0] = values
    return rows


def waypoint_metadata(targets=None, **overrides):
    """【機能概要】: metadata["waypoints"] を作る。targets は経由点名 -> 目標、overrides で上書き。"""
    targets = {name: INSIDE for name in ik_policy.WAYPOINT_ORDER} if targets is None else targets
    targets = {**targets, **overrides}
    return {
        "waypoints": tuple(
            WaypointRecord(name, targets[name], np.zeros(6), 0.0, 0.0, 0.0)
            for name in ik_policy.WAYPOINT_ORDER
        )
    }


def run(robot, traj, *, settle_s=1.0, **kwargs):
    return run_trajectory(
        robot, traj, SAFETY, lower=UNIT_LOWER, upper=UNIT_UPPER, settle_s=settle_s, **kwargs
    )


def basic_traj():
    """TC-EX-01 の軌道: 行 k = 1..5 は shoulder_pan だけ 0.01 k。"""
    return make_traj(pan_rows([0.0, 0.01, 0.02, 0.03, 0.04, 0.05]), ("start",) + ("pre_grasp",) * 5)


def walking_traj(labels):
    """各行が前の行 + 0.01 rad (全関節) の軌道 (速度制限にかからない)。"""
    rows = np.arange(len(labels))[:, None] * 0.01 * np.ones(6)
    return rows, labels


def comparable(result):
    """【機能概要】: wall_time_s と states 以外のフィールドを比較用に取り出す。"""
    out = {}
    for f in dataclasses.fields(result):
        if f.name in ("wall_time_s", "states"):
            continue
        value = getattr(result, f.name)
        if isinstance(value, np.ndarray):
            value = value.tolist()
        elif isinstance(value, RobotState):
            value = (value.q.tolist(), value.dq.tolist(), value.timestamp)
        elif hasattr(value, "items"):
            value = dict(value)
        out[f.name] = value
    return out


# =============================================================== 正常系 (フェイク Robot)
def test_run_sends_rows_after_first_then_holds_for_settle():
    """TC-EX-01 基本: 1 行 1 周期で q[1:] を送り、待ちを足す 🔵"""
    # 【テストデータ準備】: q0 = 0、5 行 (1 周期の上限 0.02 rad 未満の 0.01 刻み)
    robot, traj = FakeRobot(), basic_traj()
    # 【実際の処理実行】
    result = run(robot, traj)
    # 【結果検証】
    assert isinstance(result, ExecutionResult)
    assert len(robot.sent) == 5 + SETTLE_STEPS_DEFAULT  # 【確認内容】: 軌道 5 周期 + 待ち 50 周期
    for k in range(5):
        assert np.array_equal(robot.sent[k], traj.q[k + 1])  # 【確認内容】: q[0] は送らず q[1:] をそのまま
    for k in range(5, 55):
        assert np.array_equal(robot.sent[k], traj.q[5])  # 【確認内容】: 待ちは最後の指令を保持
    assert result.n_commands == 5 and result.settle_steps == SETTLE_STEPS_DEFAULT
    assert result.duration_s == pytest.approx(55 * DT)  # 【確認内容】: 制御周期で数えた時間
    assert dict(result.flag_counts) == {"clipped": 0, "rate_limited": 0, "rejected": 0, "stopped": 0}
    assert result.has_safety_flags is False and result.aborted is False
    assert result.abort_reason is None and result.abort_label is None
    assert result.workspace_checked is False  # 【確認内容】: waypoints なしの軌道
    assert np.array_equal(result.final_command, traj.q[5])
    assert result.final_joint_error == 0.0  # 【確認内容】: フェイクは指令どおりに動く
    assert result.final_state.timestamp == pytest.approx(1.1)
    assert result.states is None


def test_sent_commands_equal_apply_safety_output():
    """TC-EX-02 送った指令は apply_safety の出力と一致する (q_current は前回の指令) 🔵"""
    # 【テストデータ準備】: 速度制限にかかる行 (0.05 → 0.12 → 0.20) を含める
    traj = make_traj(pan_rows([0.0, 0.05, 0.12, 0.2, 0.2]), ("start",) + ("pre_grasp",) * 4)
    robot = FakeRobot()
    result = run(robot, traj, settle_s=0.0)
    # 【期待値】: テスト内で apply_safety を順に呼ぶ (prev は前回の出力、最初は q[0])
    prev, expected = traj.q[0], []
    for row in traj.q[1:]:
        res = apply_safety(
            prev, row, lower=UNIT_LOWER, upper=UNIT_UPPER, last_state_t=0.0, now=0.0, config=SAFETY
        )
        expected.append(res.q_command)
        prev = res.q_command
    assert len(robot.sent) == len(expected) == result.n_commands
    for sent, want in zip(robot.sent, expected, strict=True):
        assert np.array_equal(sent, want)  # 【確認内容】: NFR-02 実測の状態ではなく前回の指令が基準


def test_rate_limited_rows_are_counted_and_execution_continues():
    """TC-EX-03 速度上限を超える行は rate_limited を数えて続ける 🔵"""
    robot = FakeRobot()
    traj = make_traj(pan_rows([0.0, 0.05, 0.05, 0.05]), ("start",) + ("pre_grasp",) * 3)
    result = run(robot, traj, settle_s=0.0)
    assert [c[0] for c in robot.sent] == pytest.approx([0.02, 0.04, 0.05], abs=1e-12)  # 【確認内容】: 0.02 ずつ近づく
    assert result.flag_counts["rate_limited"] == 2  # 【確認内容】: 超えた 2 周期だけ数える
    assert result.flag_counts["clipped"] == result.flag_counts["rejected"] == result.flag_counts["stopped"] == 0
    assert result.has_safety_flags is True and result.aborted is False
    assert result.n_commands == 3
    assert result.final_command[0] == pytest.approx(0.05)


def test_clipped_rows_are_counted_and_execution_continues():
    """TC-EX-04 可動域外の行は clipped を数えて続ける 🔵"""
    robot = FakeRobot(q0=np.full(6, 0.99))
    rows = np.vstack([np.full(6, 0.99), np.full(6, 1.01), np.full(6, 1.01)])
    result = run(robot, make_traj(rows, ("start", "pre_grasp", "pre_grasp")), settle_s=0.0)
    assert all(np.array_equal(c, np.ones(6)) for c in robot.sent)  # 【確認内容】: 上限 1.0 に丸める
    assert result.flag_counts["clipped"] == 2 and result.flag_counts["rate_limited"] == 0
    assert result.aborted is False


def test_rejected_target_aborts_and_holds():
    """TC-EX-05 作業範囲外の目標で rejected → 打ち切り 🟡"""
    # 【テストデータ準備】: lift の目標だけ作業範囲の外
    labels = ("start", "pre_grasp", "pre_grasp", "grasp", "lift", "lift", "lift", "pre_place", "pre_place")
    rows, labels = walking_traj(labels)
    traj = make_traj(rows, labels, waypoint_metadata(lift=OUTSIDE))
    robot, steps = FakeRobot(), []
    result = run(robot, traj, on_step=lambda i, label: steps.append(label))
    assert result.aborted is True
    assert result.abort_reason == "target_out_of_workspace"  # 【確認内容】: 理由
    assert result.abort_label == "lift"  # 【確認内容】: 打ち切った区間
    assert result.flag_counts["rejected"] == 1  # 【確認内容】: 待ちの周期は target_base=None で増えない (T3)
    assert result.n_commands == 4  # 【確認内容】: 打ち切った周期の保持指令を含める (T2)
    assert np.array_equal(robot.sent[3], robot.sent[2])  # 【確認内容】: 打ち切りの周期は直前の指令を保持
    assert len(robot.sent) == 4 + SETTLE_STEPS_DEFAULT  # 【確認内容】: 残りの行は送らず待ちへ
    assert all(np.array_equal(c, robot.sent[2]) for c in robot.sent[3:])
    assert result.workspace_checked is True
    assert "pre_place" not in steps  # 【確認内容】: 打ち切り後の行は送らない


def test_stale_state_aborts_with_stopped():
    """TC-EX-06 状態の時刻が途絶えたら stopped → 打ち切り 🟡"""
    # 【テストデータ準備】: 2 つ指令を送った後の状態の時刻だけ NaN (3 行目の周期)
    robot = FakeRobot(nan_time_when_sent=(2,))
    result = run(robot, basic_traj())
    assert result.aborted is True and result.abort_reason == "stale_state"
    assert result.abort_label == "pre_grasp"
    assert result.flag_counts["stopped"] == 1
    assert result.n_commands == 3  # 【確認内容】: 打ち切った周期の保持指令を含める (T2)
    assert np.array_equal(robot.sent[2], robot.sent[1])  # 【確認内容】: 保持
    assert len(robot.sent) == 3 + SETTLE_STEPS_DEFAULT  # 【確認内容】: その後は待ちだけ


def test_record_true_keeps_state_every_cycle():
    """TC-EX-07 record=True で毎周期の状態を残す 🟡"""
    robot = FakeRobot()
    result = run(robot, basic_traj(), record=True)
    assert type(result.states) is tuple and len(result.states) == 55  # 【確認内容】: 軌道 + 待ちの全周期
    assert all(isinstance(s, RobotState) for s in result.states)
    assert np.array_equal(result.states[-1].q, result.final_state.q)
    for k in (0, 4, 30, 54):
        assert np.array_equal(result.states[k].q, robot.sent[k])  # 【確認内容】: 送信後の状態
    assert run(FakeRobot(), basic_traj()).states is None  # 【確認内容】: 既定は残さない


def test_on_step_call_sequence():
    """TC-EX-08 on_step の呼ばれ方 (1 始まりの通し番号、待ちは "settle") 🔴"""
    robot, calls, sent_at_call = FakeRobot(), [], []

    def on_step(i, label):
        calls.append((i, label))
        sent_at_call.append(len(robot.sent))

    run(robot, basic_traj(), on_step=on_step)
    expected = [(k, "pre_grasp") for k in range(1, 6)] + [(k, "settle") for k in range(6, 56)]
    assert calls == expected  # 【確認内容】: 周期の番号と区間名 (T4)
    assert sent_at_call == [i for i, _ in expected]  # 【確認内容】: 送信の後に呼ぶ


def test_target_base_follows_segment_to_waypoint_table(monkeypatch):
    """TC-EX-09 区間ごとの target_base (close は grasp、open は place、待ちは None) 🔵"""
    # 【テストデータ準備】: 6 つの経由点の目標を互いに違う値にする (全て作業範囲の内側)
    targets = {
        name: (0.20 + 0.02 * k, -0.05 + 0.03 * k, 0.04 + 0.01 * k)
        for k, name in enumerate(ik_policy.WAYPOINT_ORDER)
    }
    labels = ("start", *ALL_SEGMENTS)
    rows = np.arange(len(labels))[:, None] * 0.001 * np.ones(6)
    traj = make_traj(rows, labels, waypoint_metadata(targets))
    seen, real = [], apply_safety

    def spy(*args, **kwargs):
        seen.append(kwargs.get("target_base"))
        return real(*args, **kwargs)

    monkeypatch.setattr(executor_module, "apply_safety", spy)
    result = run(FakeRobot(), traj)
    assert len(seen) == len(ALL_SEGMENTS) + SETTLE_STEPS_DEFAULT
    for label, got in zip(ALL_SEGMENTS, seen, strict=False):
        want = targets[EXPECTED_SEGMENT_WAYPOINT[label]]
        assert np.array_equal(np.asarray(got), np.asarray(want)), label  # 【確認内容】: 区間の終点の目標
    assert all(t is None for t in seen[len(ALL_SEGMENTS) :])  # 【確認内容】: 待ちの周期は None (T3)
    assert result.workspace_checked is True


def test_segment_waypoint_table_matches_segment_table():
    """TC-EX-10 区間の対応表が SEGMENT_TABLE と一致する 🔵"""
    assert set(SEGMENT_WAYPOINT) == set(ALL_SEGMENTS)  # 【確認内容】: 10 区間を過不足なく
    assert set(SEGMENT_WAYPOINT.values()) <= set(ik_policy.WAYPOINT_ORDER)  # 【確認内容】: 経由点名
    assert dict(SEGMENT_WAYPOINT) == EXPECTED_SEGMENT_WAYPOINT  # 【確認内容】: 要件 3.2 の表


def test_fake_robot_satisfies_protocol_and_executor_does_not_close():
    """TC-EX-11 フェイク Robot は Robot を満たし、executor は close を呼ばない 🟡"""
    robot = FakeRobot()
    assert isinstance(robot, Robot)  # 【確認内容】: Protocol の 4 メンバー
    run(robot, basic_traj())
    assert robot.closed is False  # 【確認内容】: 寿命は呼び出し側が持つ


# =============================================================== 正常系 (性質)
def test_inputs_unchanged_and_result_immutable():
    """TC-EX-12 入力を変更しない / 結果は変更できない 🔵"""
    traj, lower, upper = basic_traj(), UNIT_LOWER.copy(), UNIT_UPPER.copy()
    q_before = traj.q.copy()
    result = run_trajectory(FakeRobot(), traj, SAFETY, lower=lower, upper=upper)
    assert np.array_equal(lower, UNIT_LOWER) and np.array_equal(upper, UNIT_UPPER)  # 【確認内容】: 可動域
    assert np.array_equal(traj.q, q_before) and not traj.q.flags.writeable  # 【確認内容】: 軌道
    with pytest.raises(dataclasses.FrozenInstanceError):
        result.n_commands = 1
    with pytest.raises(TypeError):
        result.flag_counts["clipped"] = 9
    with pytest.raises(ValueError):
        result.final_command[0] = 9.0  # 【確認内容】: 読み取り専用の配列


def test_run_is_deterministic():
    """TC-EX-13 決定性 (wall_time_s 以外は全て一致) 🔵"""
    r1, r2 = FakeRobot(), FakeRobot()
    a, b = run(r1, basic_traj()), run(r2, basic_traj())
    assert all(np.array_equal(x, y) for x, y in zip(r1.sent, r2.sent, strict=True))
    assert comparable(a) == comparable(b)


def test_wall_time_is_recorded_only():
    """TC-EX-14 wall_time_s は記録のみ (duration_s は周期数で決まる) 🟡"""
    result = run(FakeRobot(), basic_traj())
    assert math.isfinite(result.wall_time_s) and result.wall_time_s >= 0.0
    assert result.duration_s == (result.n_commands + result.settle_steps) * DT  # 【確認内容】: 実時間と無関係


# =============================================================== 異常系
@pytest.mark.parametrize("offset", [0.05, -0.05])
def test_start_pose_mismatch_raises(offset):
    """TC-EX-E01 開始姿勢の不一致 → ValueError (何も送らない) 🔵"""
    rows = np.zeros((2, 6))
    rows[:, 2] = offset  # elbow_flex
    robot = FakeRobot()  # 現在の姿勢は 0
    with pytest.raises(ValueError, match="開始"):
        run(robot, make_traj(rows, ("start", "pre_grasp")))
    assert robot.sent == []


def test_unknown_segment_label_raises_before_sending():
    """TC-EX-E02 未知の区間名 → ValueError (送る前に全ラベルを確かめる) 🟡"""
    robot = FakeRobot()
    with pytest.raises(ValueError, match="dance"):
        run(robot, make_traj(np.zeros((3, 6)), ("start", "pre_grasp", "dance")))
    assert robot.sent == []


@pytest.mark.parametrize("settle_s", [-0.01, float("nan"), float("inf"), True])
def test_invalid_settle_s_raises(settle_s):
    """TC-EX-E03 settle_s の不正 → ValueError (何も送らない) 🟡"""
    robot = FakeRobot()
    with pytest.raises(ValueError):
        run(robot, basic_traj(), settle_s=settle_s)
    assert robot.sent == []


def test_limits_shape_mismatch_raises_before_sending():
    """TC-EX-E04 可動域の shape 違い → ValueError (最初の送信前) 🟡"""
    robot = FakeRobot()
    with pytest.raises(ValueError):
        run_trajectory(robot, basic_traj(), SAFETY, lower=-np.ones(5), upper=UNIT_UPPER)
    assert robot.sent == []


def test_on_step_exception_propagates():
    """TC-EX-E05 on_step の例外はそのまま伝わる (その後は送らない) 🔴"""
    robot, count = FakeRobot(), []

    def on_step(i, label):
        count.append(i)
        if len(count) == 2:
            raise RuntimeError("boom")

    with pytest.raises(RuntimeError, match="boom"):
        run(robot, basic_traj(), on_step=on_step)
    assert len(robot.sent) == 2


def test_non_trajectory_raises():
    """TC-EX-E06 軌道の型違い → TypeError か ValueError 🟡"""
    with pytest.raises((TypeError, ValueError)):
        run(FakeRobot(), np.zeros((3, 6)))


# =============================================================== 境界値
def test_start_pose_tolerance_boundary():
    """TC-EX-B01 開始姿勢の差: 0.0499 rad は実行でき、0.05 rad 以上は ValueError (T1) 🔵"""
    assert START_TOLERANCE_RAD == 0.05
    ok = np.zeros((2, 6))
    ok[:, 2] = 0.0499
    result = run(FakeRobot(), make_traj(ok, ("start", "pre_grasp")))
    assert result.n_commands == 1  # 【確認内容】: 許容内は実行できる
    bad = np.zeros((2, 6))
    bad[:, 2] = START_TOLERANCE_RAD
    with pytest.raises(ValueError, match="開始"):
        run(FakeRobot(), make_traj(bad, ("start", "pre_grasp")))  # 【確認内容】: ちょうど 0.05 は拒否


@pytest.mark.parametrize(
    ("settle_s", "steps"), [(0.0, 0), (0.01, 1), (0.02, 1), (0.03, 2), (0.5, 25), (1.0, 50)]
)
def test_settle_steps_use_ceil(settle_s, steps):
    """TC-EX-B02 待ちの周期数は ceil (0.01 は round だと 0 になる) 🔵"""
    robot = FakeRobot()
    result = run(robot, basic_traj(), settle_s=settle_s)
    assert result.settle_steps == steps
    assert len(robot.sent) == result.n_commands + steps
    assert result.duration_s == pytest.approx((result.n_commands + steps) * DT)


def test_shortest_trajectory_two_rows():
    """TC-EX-B03 最短の軌道 (2 行) 🟡"""
    robot = FakeRobot()
    traj = make_traj(pan_rows([0.0, 0.01]), ("start", "pre_grasp"))
    result = run(robot, traj)
    assert result.n_commands == 1
    assert len(robot.sent) == 1 + result.settle_steps


def test_trajectory_without_waypoints_skips_workspace_check():
    """TC-EX-B04 waypoints なしの軌道は作業範囲を見ない 🟡"""
    result = run(FakeRobot(), make_traj(pan_rows([0.0, 0.01]), ("start", "lift"), metadata={}))
    assert result.workspace_checked is False
    assert result.flag_counts["rejected"] == 0 and result.aborted is False


# =============================================================== 統合 (SimRobot)
@pytest.fixture(scope="module")
def seed0_run():
    """【テスト前準備】: 実シーンで赤 seed 0 の軌道を作り、SimRobot で 1 回だけ実行する (約 0.8 s)。"""
    with SimRobot(PICKPLACE_SCENE_PATH) as robot:
        model, data = robot.mujoco_handles()
        place_blocks(model, data, sample_layout(0))
        pick = data.xpos[model.body("block_red").id].copy()
        box_x, box_y = SCENE_SPEC.box_center_xy
        goal = Goal(pick, (box_x, box_y, SCENE_SPEC.box_floor_top_z))
        lower, upper = limits_from_model(model)
        safety = load_safety_config()
        traj = IkPolicy(model, safety=safety).act(Observation.from_state(robot.get_state()), goal)
        result = run_trajectory(robot, traj, safety, lower=lower, upper=upper)
    return traj, result


def test_ik_policy_trajectory_runs_without_safety_flags(seed0_run):
    """TC-EXI-01 IkPolicy の軌道を実行してフラグ 0 🔵"""
    traj, result = seed0_run
    assert result.n_commands == len(traj.q) - 1  # 【確認内容】: 実測 273
    assert result.settle_steps == SETTLE_STEPS_DEFAULT
    assert result.has_safety_flags is False and result.aborted is False
    assert result.workspace_checked is True  # 【確認内容】: metadata の waypoints から target_base を引いた
    assert result.final_joint_error < JOINT_ERR_TOL  # 【確認内容】: 実測 0.00044 rad
    assert result.duration_s == pytest.approx(6.46, abs=0.02)


def _rate_limited_run():
    """pan 0 -> 0.3 を 11 行 (+ 待ち 50) で実行する (SimRobot の既定シーン)。"""
    lower, upper = limits_from_model(mujoco.MjModel.from_xml_path(str(SO101_SCENE_PATH)))
    traj = make_traj(pan_rows([0.0] + [0.3] * 11), ("start",) + ("pre_grasp",) * 11)
    with SimRobot() as robot:
        return run_trajectory(robot, traj, SAFETY, lower=lower, upper=upper)


def test_hand_written_fast_trajectory_is_rate_limited_on_sim():
    """TC-EXI-02 手書きの速度超過の軌道 🔵"""
    result = _rate_limited_run()
    assert result.flag_counts["rate_limited"] == 11  # 【確認内容】: 全 11 周期で制限
    assert result.final_command[0] == pytest.approx(0.22, abs=1e-9)  # 【確認内容】: 0.02 x 11
    assert result.final_joint_error < JOINT_ERR_TOL  # 【確認内容】: 実測 0.00054 rad
    assert result.final_state.timestamp == pytest.approx(1.22, abs=1e-9)  # 【確認内容】: (11 + 50) x 0.02


def test_executor_uses_simrobot_only_through_robot_api(monkeypatch):
    """TC-EXI-03 実機と同じ API で SimRobot を使える (mujoco_handles を呼ばない) 🔵"""

    def fail(self):
        raise AssertionError("run_trajectory が mujoco_handles を呼んだ")

    monkeypatch.setattr(SimRobot, "mujoco_handles", fail)
    assert _rate_limited_run().aborted is False

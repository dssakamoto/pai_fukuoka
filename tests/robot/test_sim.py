"""SimRobot / Robot Protocol / RobotState のテスト (TASK-0004, Red フェーズ)。

テストケース定義: docs/implements/so101-vlm-pickplace/TASK-0004/sim-robot-testcases.md
信頼性: 🔵 要件・設計・実測に基づく / 🟡 妥当な推測 / 🔴 要件にない仮定
"""

import dataclasses
from pathlib import Path

import mujoco  # noqa: F401  (小モデル生成の前提確認用)
import numpy as np
import pytest

from so101_vlm.robot.base import JOINT_NAMES, Robot, RobotState
from so101_vlm.robot.sim import SimRobot

# --- 定数 (testcases.md 0.1 / 0.2) ---
TOL_RAD = 0.01
DQ_SETTLED = 0.05
SIM_TIMESTEP_S = 0.002
N_SUBSTEPS_DEFAULT = 10
N_SETTLE_COMMANDS = 50
TIME_ABS = 1e-9

TARGET_MIXED = np.array([0.3, 0.3, 0.5, -0.3, 0.5, 0.5])
TARGET_GRAVITY = np.array([0.0, 1.0, -1.0, 0.0, 0.0, 0.0])
TARGET_OPPOSITE = np.array([-0.5, -0.5, 0.8, 0.5, -1.0, 1.0])
TARGET_DISTINCT = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])

# 小モデルのアクチュエータ宣言順 (JOINT_NAMES とは異なる順にする)
_TINY_ACTUATOR_ORDER = (
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
    "shoulder_pan",
    "shoulder_lift",
)


# --- 小さな MJCF ---
def _tiny_mjcf(*, drop_joint: str | None = None, drop_actuator: str | None = None) -> str:
    """余分な slide 関節 + 逆順の hinge 関節 + 並べ替えた actuator を持つ MJCF を返す。"""
    bodies = ['<body name="b_extra"><joint name="extra" type="slide" axis="1 0 0"/>'
              '<geom type="sphere" size="0.1" mass="1"/></body>']
    for name in reversed(JOINT_NAMES):  # 関節を逆順に宣言する
        if name == drop_joint:
            continue
        bodies.append(
            f'<body name="b_{name}"><joint name="{name}" type="hinge" axis="0 0 1"/>'
            f'<geom type="sphere" size="0.1" mass="1"/></body>'
        )
    acts = [
        f'<position name="{n}" joint="{n}" kp="50" kv="1" ctrllimited="true" ctrlrange="-3 3"/>'
        for n in _TINY_ACTUATOR_ORDER
        if n not in (drop_joint, drop_actuator)
    ]
    return (
        '<mujoco><option timestep="0.002" gravity="0 0 0"/>'
        f'<worldbody>{"".join(bodies)}</worldbody>'
        f'<actuator>{"".join(acts)}</actuator></mujoco>'
    )


def _write(path: Path, text: str) -> Path:
    path.write_text(text, encoding="utf-8")
    return path


# --- fixture ---
@pytest.fixture(scope="module")
def tiny_scene_path(tmp_path_factory) -> Path:
    # 【テスト前準備】: 宣言順をずらした小モデルを一度だけ作る (読み込み約 2 ms)
    return _write(tmp_path_factory.mktemp("tiny") / "tiny.xml", _tiny_mjcf())


@pytest.fixture
def tiny_robot(tiny_scene_path):
    robot = SimRobot(scene_path=tiny_scene_path)
    yield robot
    robot.close()  # close は冪等


@pytest.fixture(scope="module")
def shared_robot():
    # 【環境初期化】: 状態を進めない検証用に実シーンを 1 回だけ読み込む
    robot = SimRobot()
    yield robot
    robot.close()


@pytest.fixture
def fresh_robot():
    # 【環境初期化】: テストごとに新しい MjData (ゼロ姿勢、時刻 0)
    robot = SimRobot()
    yield robot
    robot.close()


def _settle(robot, target, n=N_SETTLE_COMMANDS):
    for _ in range(n):
        robot.send_command(target)
    return robot.get_state()


# =========================== 正常系 ===========================
@pytest.mark.parametrize(
    "target",
    [TARGET_MIXED, TARGET_GRAVITY, TARGET_OPPOSITE],
    ids=["TARGET_MIXED", "TARGET_GRAVITY", "TARGET_OPPOSITE"],
)
def test_converges_to_target(fresh_robot, target):
    """TC-N01 🔵 Given ゼロ姿勢 / When 目標を 50 回送る / Then 全関節が目標に収束し速度が落ちる。"""
    state = _settle(fresh_robot, target)
    assert np.max(np.abs(state.q - target)) < TOL_RAD  # 位置誤差
    assert np.max(np.abs(state.dq)) < DQ_SETTLED  # 整定


def test_initial_state_shape_dtype_and_zero_pose(fresh_robot):
    """TC-N02 🟡 Given 作成直後 / When get_state / Then (6,) float64 のゼロ姿勢、時刻 0.0。"""
    state = fresh_robot.get_state()
    assert isinstance(state, RobotState)
    assert state.q.shape == (6,) and state.dq.shape == (6,)
    assert state.q.dtype == np.float64 and state.dq.dtype == np.float64
    assert np.allclose(state.q, 0.0) and np.allclose(state.dq, 0.0)
    assert state.timestamp == 0.0 and isinstance(state.timestamp, float)


def test_joint_names_match_model_paths(shared_robot):
    """TC-N03 🔵 Given 実シーン / When joint_names / Then JOINT_NAMES と同順のタプル。"""
    assert isinstance(shared_robot.joint_names, tuple)
    assert shared_robot.joint_names == JOINT_NAMES


def test_joint_order_distinct_target_on_real_scene(fresh_robot):
    """TC-N04 🔵 Given 全要素が異なる目標 / When 50 回送る / Then 要素ごとに一致 (取り違えなし)。"""
    state = _settle(fresh_robot, TARGET_DISTINCT)
    for i in range(6):
        assert abs(state.q[i] - TARGET_DISTINCT[i]) < TOL_RAD


def test_joint_order_resolved_by_name_on_permuted_model(tiny_robot):
    """TC-N05 🟡 Given 宣言順・qpos アドレスがずれた小モデル / When 送る / Then 名前で対応付け収束。"""
    state = _settle(tiny_robot, TARGET_DISTINCT)
    assert np.max(np.abs(state.q - TARGET_DISTINCT)) < TOL_RAD


@pytest.mark.parametrize("n_substeps", [None, 3], ids=["default10", "n3"])
def test_timestamp_advances_by_substeps(tiny_scene_path, n_substeps):
    """TC-N06 🟡(既定値10は🔴) Given n_substeps / When k 回送る / Then timestamp = k*n*0.002。"""
    kwargs = {} if n_substeps is None else {"n_substeps": n_substeps}
    n = N_SUBSTEPS_DEFAULT if n_substeps is None else n_substeps
    robot = SimRobot(scene_path=tiny_scene_path, **kwargs)
    try:
        for k in (1, 2, 3):
            robot.send_command(np.zeros(6))
            expected = k * n * SIM_TIMESTEP_S
            assert robot.get_state().timestamp == pytest.approx(expected, abs=TIME_ABS)
    finally:
        robot.close()


def test_send_command_accepts_float_list(tiny_robot):
    """TC-N07a 🔴 Given float の list / When 50 回送る / Then 例外なく収束。"""
    state = _settle(tiny_robot, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    assert np.max(np.abs(state.q - TARGET_DISTINCT)) < TOL_RAD


def test_send_command_accepts_int_list(tiny_robot):
    """TC-N07b 🔴 Given int の list / When 1 回送る / Then 例外なく timestamp が 0.02 進む。"""
    tiny_robot.send_command([0, 0, 0, 0, 0, 0])
    assert tiny_robot.get_state().timestamp == pytest.approx(0.02, abs=TIME_ABS)


def test_velocity_sign_follows_motion(fresh_robot):
    """TC-N08 🟡 Given ゼロ姿勢 / When TARGET_OPPOSITE を 1 回 / Then dq の符号が目標と一致、rad/s のオーダー。"""
    fresh_robot.send_command(TARGET_OPPOSITE)
    dq = fresh_robot.get_state().dq
    assert np.all(np.sign(dq) == np.sign(TARGET_OPPOSITE))
    assert np.all((np.abs(dq) > 0.1) & (np.abs(dq) < 10))  # deg/s ではない


# =========================== 異常系 ===========================
def _assert_unchanged(robot, before, call):
    """call が ValueError を出し、時刻と q が変わらないことを確認する。"""
    with pytest.raises(ValueError):
        call()
    after = robot.get_state()
    assert after.timestamp == before.timestamp
    assert np.array_equal(after.q, before.q)


@pytest.mark.parametrize("length", [5, 7], ids=["len5", "len7"])
def test_send_command_rejects_wrong_shape(shared_robot, length):
    """TC-E01 🔵 Given 長さ違いの配列 / When send_command / Then ValueError(shape) で状態不変。"""
    before = shared_robot.get_state()
    with pytest.raises(ValueError, match="shape") as exc:
        shared_robot.send_command(np.zeros(length))
    assert "(6,)" in str(exc.value)  # 期待形状をメッセージに含む
    after = shared_robot.get_state()
    assert after.timestamp == before.timestamp
    assert np.array_equal(after.q, before.q)


@pytest.mark.parametrize("bad", [np.nan, np.inf, -np.inf], ids=["nan", "posinf", "neginf"])
def test_send_command_rejects_non_finite(shared_robot, bad):
    """TC-E02 🟡 Given NaN/inf を含む / When send_command / Then ValueError(finite) で状態不変。"""
    target = TARGET_MIXED.copy()
    target[2] = bad
    before = shared_robot.get_state()
    with pytest.raises(ValueError, match="finite"):
        shared_robot.send_command(target)
    after = shared_robot.get_state()
    assert after.timestamp == before.timestamp
    assert np.array_equal(after.q, before.q)


@pytest.mark.parametrize("n_substeps", [0, -1])
def test_rejects_invalid_n_substeps(tiny_scene_path, n_substeps):
    """TC-E03 🔴 Given n_substeps < 1 / When 生成 / Then ValueError(n_substeps)。"""
    with pytest.raises(ValueError, match="n_substeps"):
        SimRobot(scene_path=tiny_scene_path, n_substeps=n_substeps)


def test_missing_scene_file_raises_file_not_found(tmp_path):
    """TC-E04 🟡 Given 存在しないパス / When 生成 / Then FileNotFoundError(パスを含む)。"""
    with pytest.raises(FileNotFoundError, match="no_such_scene.xml"):
        SimRobot(scene_path=tmp_path / "no_such_scene.xml")


def test_malformed_scene_raises_value_error(tmp_path):
    """TC-E05 🟡 Given XML 不正のファイル / When 生成 / Then ValueError(パスを含む)。"""
    broken = _write(tmp_path / "broken.xml", "<mujoco><worldbody>")
    with pytest.raises(ValueError, match="broken.xml"):
        SimRobot(scene_path=broken)


@pytest.mark.parametrize(
    "variant",
    [{"drop_joint": "gripper"}, {"drop_actuator": "gripper"}],
    ids=["missing_joint", "missing_actuator"],
)
def test_missing_joint_name_raises_value_error(tmp_path, variant):
    """TC-E06 🟡 Given 関節/アクチュエータが欠けたモデル / When 生成 / Then ValueError(gripper)。"""
    path = _write(tmp_path / "variant.xml", _tiny_mjcf(**variant))
    with pytest.raises(ValueError, match="gripper"):
        SimRobot(scene_path=path)


def test_use_after_close_raises_runtime_error(tiny_scene_path):
    """TC-E07 🟡 Given close 済み / When get_state, send_command / Then RuntimeError(closed)。"""
    robot = SimRobot(scene_path=tiny_scene_path)
    robot.close()
    with pytest.raises(RuntimeError, match="closed"):
        robot.get_state()
    with pytest.raises(RuntimeError, match="closed"):
        robot.send_command(np.zeros(6))


# =========================== 境界値 ===========================
def test_close_is_idempotent(tiny_scene_path):
    """TC-B01 🟡 Given close 済み / When もう一度 close / Then 例外なし、終了状態のまま。"""
    robot = SimRobot(scene_path=tiny_scene_path)
    robot.close()
    robot.close()
    with pytest.raises(RuntimeError, match="closed"):
        robot.get_state()


def test_out_of_ctrlrange_command_is_clamped_by_mujoco_not_rejected(fresh_robot):
    """TC-B02 🟡 Given 範囲外の指令 / When 50 回送る / Then 例外なし、MuJoCo により可動域端で止まる。"""
    target = np.array([5.0, 0.0, 0.0, 0.0, 0.0, -1.0])
    state = _settle(fresh_robot, target)  # 例外が出ないこと
    assert state.q[0] == pytest.approx(1.91986, abs=TOL_RAD)  # shoulder_pan 上限
    assert state.q[5] == pytest.approx(-0.17453, abs=TOL_RAD)  # gripper 下限
    assert np.all(np.isfinite(state.q))


@pytest.mark.parametrize(
    "bad",
    [np.array([]), np.zeros((1, 6)), np.zeros((6, 1)), np.float64(0.0)],
    ids=["empty", "row_1x6", "col_6x1", "scalar"],
)
def test_send_command_rejects_degenerate_shapes(shared_robot, bad):
    """TC-B03 🟡 Given 形状が (6,) でない配列 / When send_command / Then ValueError で状態不変。"""
    before = shared_robot.get_state()
    _assert_unchanged(shared_robot, before, lambda: shared_robot.send_command(bad))


def test_min_n_substeps_one_step_per_command(tiny_scene_path):
    """TC-B04 🟡 Given n_substeps=1 / When 1 回送る / Then timestamp が 0.002 (ちょうど 1 step)。"""
    robot = SimRobot(scene_path=tiny_scene_path, n_substeps=1)
    try:
        robot.send_command(np.zeros(6))
        assert robot.get_state().timestamp == pytest.approx(SIM_TIMESTEP_S, abs=TIME_ABS)
    finally:
        robot.close()


# =========================== 不変性 ===========================
@pytest.mark.parametrize("field", ["q", "dq"])
def test_state_arrays_are_read_only(shared_robot, field):
    """TC-I01 🔵 Given 取得した状態 / When 配列に代入 / Then ValueError、再取得しても値は不変。"""
    state = shared_robot.get_state()
    arr = getattr(state, field)
    assert arr.flags.writeable is False
    original = arr[0]
    with pytest.raises(ValueError):
        arr[0] = 1.0
    assert getattr(shared_robot.get_state(), field)[0] == original


def test_robot_state_is_frozen(shared_robot):
    """TC-I02 🟡 Given RobotState / When フィールド代入 / Then FrozenInstanceError。"""
    state = shared_robot.get_state()
    assert dataclasses.is_dataclass(state)
    with pytest.raises(dataclasses.FrozenInstanceError):
        state.q = np.zeros(6)
    with pytest.raises(dataclasses.FrozenInstanceError):
        state.timestamp = 1.0


def test_previous_state_unchanged_after_command(tiny_robot):
    """TC-I03 🟡 Given 取得済みの状態 s0 / When 指令を進める / Then s0 は不変でメモリも共有しない。"""
    s0 = tiny_robot.get_state()
    s1 = _settle(tiny_robot, TARGET_DISTINCT, n=10)
    assert np.allclose(s0.q, 0.0) and s0.timestamp == 0.0
    assert not np.allclose(s1.q, s0.q)
    assert not np.shares_memory(s0.q, s1.q)


def test_send_command_does_not_alias_input(tiny_scene_path):
    """TC-I04 🟡 Given 呼び出し側の配列 / When 送信後に書き換える / Then 入力は不変で結果に影響しない。"""
    a = SimRobot(scene_path=tiny_scene_path)
    b = SimRobot(scene_path=tiny_scene_path)
    try:
        arr = TARGET_DISTINCT.copy()
        a.send_command(arr)
        assert np.array_equal(arr, TARGET_DISTINCT)  # (a) 入力を変更しない
        arr[:] = -2.0  # (b) 呼び出し後の書き換え
        for _ in range(9):
            a.send_command(TARGET_DISTINCT.copy())
        for _ in range(10):
            b.send_command(TARGET_DISTINCT.copy())
        sa, sb = a.get_state(), b.get_state()
        assert np.array_equal(sa.q, sb.q) and np.array_equal(sa.dq, sb.dq)
        assert sa.timestamp == sb.timestamp
    finally:
        a.close()
        b.close()


def test_get_state_has_no_side_effects(tiny_robot):
    """TC-I05 🟡 Given 動いている状態 / When get_state を 3 回 / Then 同じ値で別メモリ、時刻は進まない。"""
    tiny_robot.send_command(TARGET_DISTINCT)
    states = [tiny_robot.get_state() for _ in range(3)]
    for s in states[1:]:
        assert s.timestamp == states[0].timestamp
        assert np.array_equal(s.q, states[0].q) and np.array_equal(s.dq, states[0].dq)
        assert not np.shares_memory(s.q, states[0].q)


# ================= Protocol / context manager =================
def test_sim_robot_satisfies_robot_protocol(shared_robot):
    """TC-P01 🟡 Given SimRobot / When isinstance / Then Robot (runtime_checkable) を満たす。"""
    assert isinstance(shared_robot, Robot) is True


def test_object_missing_members_is_not_robot():
    """TC-P02 🟡 Given メンバー不足のオブジェクト / When isinstance / Then Robot ではない。"""

    class Partial:
        joint_names = JOINT_NAMES

        def get_state(self):
            return None

    assert isinstance(object(), Robot) is False
    assert isinstance(Partial(), Robot) is False


def test_context_manager_closes_on_exit(tiny_scene_path):
    """TC-C01 🔴 Given with 文 / When 抜ける / Then 中では動作し、抜けると close 済み。"""
    with SimRobot(scene_path=tiny_scene_path) as robot:
        assert isinstance(robot, SimRobot)
        robot.send_command(np.zeros(6))
        robot.get_state()
    with pytest.raises(RuntimeError, match="closed"):
        robot.get_state()


def test_context_manager_closes_and_propagates_exception(tiny_scene_path):
    """TC-C02 🔴 Given with 内で例外 / When 抜ける / Then 例外は伝わり close される。"""
    with pytest.raises(ValueError):
        with SimRobot(scene_path=tiny_scene_path) as robot:
            robot.send_command(np.zeros(5))
    with pytest.raises(RuntimeError, match="closed"):
        robot.get_state()

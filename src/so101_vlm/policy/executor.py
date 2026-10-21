"""軌道の実行器 run_trajectory (TASK-0013)。

🟡 JointTrajectory を 1 制御周期ごとに apply_safety (安全フィルタ) に通して Robot へ送る。
MuJoCo には依存しない (Robot Protocol だけを使うので、実機にも同じ API で使える)。
入力 (trajectory、lower、upper) は変更しない。計画の情報 (TrajectoryError など) は持たない。
"""

import math
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from types import MappingProxyType

import numpy as np

from so101_vlm.policy.segments import SEGMENT_WAYPOINT
from so101_vlm.policy.trajectory import JointTrajectory
from so101_vlm.robot.base import Robot, RobotState
from so101_vlm.safety.config import SafetyConfig
from so101_vlm.safety.limits import SafetyResult, apply_safety

# 【定数定義】: 開始姿勢の許容 [rad]。ロボットの現在角と軌道の先頭の差がこれ以上なら拒否する 🔴
START_TOLERANCE_RAD = 0.05
# 【定数定義】: 待ちの周期の区間名 (on_step に渡す) 🔴
SETTLE_LABEL = "settle"
# 【定数定義】: 安全フィルタのフラグ名 (数え上げ・報告で共有) 🔵
FLAG_NAMES = ("clipped", "rate_limited", "rejected", "stopped")

StepCallback = Callable[[int, str], None]


@dataclass(frozen=True)
class ExecutionResult:
    """【機能概要】: 実行の結果。変更不可。🟡

    n_commands: 送った軌道の行数 (打ち切った周期の保持指令を含む)、settle_steps: 待ちの周期数、
    flag_counts: 安全フィルタのフラグが立った周期数 (clipped / rate_limited / rejected / stopped)、
    aborted / abort_reason / abort_label: 打ち切りの有無と理由・区間、
    workspace_checked: 経由点の target_base を渡したか、final_command: 最後に送った指令、
    final_state: 待ちの後の状態、final_joint_error: max |final_state.q - final_command| [rad]、
    duration_s: 制御周期で数えた時間、wall_time_s: 実時間 (記録のみ)、
    states: 毎周期の送信後の状態 (record=True のときだけ)。
    """

    n_commands: int
    settle_steps: int
    flag_counts: Mapping[str, int]
    aborted: bool
    abort_reason: str | None
    abort_label: str | None
    workspace_checked: bool
    final_command: np.ndarray
    final_state: RobotState
    final_joint_error: float
    duration_s: float
    wall_time_s: float
    states: tuple[RobotState, ...] | None

    @property
    def has_safety_flags(self) -> bool:
        """【機能概要】: 安全フィルタのフラグが 1 周期でも立ったか 🟡"""
        return any(count > 0 for count in self.flag_counts.values())


def _validate_inputs(trajectory: object, settle_s: object) -> None:
    """【機能概要】: 何も送る前に軌道の型・区間名・待ち時間を検査する 🟡"""
    if not isinstance(trajectory, JointTrajectory):
        raise TypeError(f"trajectory は JointTrajectory で指定してください: {trajectory!r}")
    if isinstance(settle_s, bool) or not isinstance(settle_s, (int, float)):
        raise ValueError(f"settle_s は数値で指定してください: settle_s={settle_s!r}")
    if not math.isfinite(settle_s) or settle_s < 0:
        raise ValueError(f"settle_s は 0 以上の有限値で指定してください: settle_s={settle_s!r}")
    unknown = sorted({lab for lab in trajectory.labels[1:] if lab not in SEGMENT_WAYPOINT})
    if unknown:
        raise ValueError(f"未知の区間名: {', '.join(unknown)}")


def _check_start_pose(robot: Robot, trajectory: JointTrajectory) -> None:
    """【機能概要】: ロボットの現在角と軌道の先頭の差が許容以上なら ValueError 🔴"""
    gap = float(np.max(np.abs(robot.get_state().q - trajectory.q[0])))
    if gap >= START_TOLERANCE_RAD:
        raise ValueError(
            f"開始姿勢が軌道の先頭と違います: 差 {gap:.4f} rad >= {START_TOLERANCE_RAD} rad"
        )


def _waypoint_targets(trajectory: JointTrajectory) -> dict[str, np.ndarray] | None:
    """【機能概要】: metadata["waypoints"] から 経由点名 -> TCP 目標 を作る。無ければ None 🟡"""
    records = trajectory.metadata.get("waypoints")
    if records is None:
        return None
    return {record.name: record.target for record in records}


@dataclass(frozen=True)
class _Progress:
    """【機能概要】: 送信ループの進み具合 (指令数・待ちの周期数・打ち切り (理由, 区間名))。🟡"""

    n_commands: int
    settle_steps: int
    abort: tuple[str | None, str] | None


class _Stepper:
    """【機能概要】: 1 周期分 (状態の読み取り -> 安全フィルタ -> 送信 -> 記録) を行う。🟡

    直前の指令・周期の番号・フラグの数え上げを持つ (実行中だけの内部状態)。
    """

    def __init__(
        self,
        robot: Robot,
        safety_config: SafetyConfig,
        lower: np.ndarray,
        upper: np.ndarray,
        start: np.ndarray,
        record: bool,
        on_step: StepCallback | None,
    ) -> None:
        self.prev = np.array(start, dtype=np.float64)
        self.counts = dict.fromkeys(FLAG_NAMES, 0)
        self.states: list[RobotState] = []
        self._robot = robot
        self._config = safety_config
        self._lower, self._upper = lower, upper
        self.recording = record
        self._on_step = on_step
        self._step = 0

    def cycle(self, row: np.ndarray, label: str, target_base: np.ndarray | None) -> SafetyResult:
        """【機能概要】: 1 周期ぶん安全フィルタを通して送信する。結果を返す 🟡"""
        state = self._robot.get_state()
        result = apply_safety(
            self.prev, row, lower=self._lower, upper=self._upper,
            last_state_t=state.timestamp, now=state.timestamp,
            config=self._config, target_base=target_base,
        )  # fmt: skip
        for name in FLAG_NAMES:
            self.counts[name] += int(bool(getattr(result, name)))
        self._robot.send_command(result.q_command)
        self.prev = np.array(result.q_command)
        self._step += 1
        if self.recording:
            self.states.append(self._robot.get_state())
        if self._on_step is not None:
            self._on_step(self._step, label)
        return result


def run_trajectory(
    robot: Robot,
    trajectory: JointTrajectory,
    safety_config: SafetyConfig,
    *,
    lower: np.ndarray,
    upper: np.ndarray,
    settle_s: float = 1.0,
    record: bool = False,
    on_step: StepCallback | None = None,
) -> ExecutionResult:
    """【機能概要】: 軌道を安全フィルタ経由で 1 周期ずつ送り、最後の指令で settle_s だけ待つ 🔵

    【実装方針】: rejected / stopped の周期で送信を打ち切り (その周期の保持指令は送る)、待ちへ進む。
    clipped / rate_limited は数えて続ける。on_step(周期の番号 (1 始まり), 区間名) は送信の後に呼ぶ
    (待ちの区間名は "settle")。例外は呼び出し側へそのまま伝える。
    開始姿勢が軌道の先頭と START_TOLERANCE_RAD 以上違う場合、未知の区間名、不正な settle_s は ValueError。
    """
    started = time.perf_counter()
    _validate_inputs(trajectory, settle_s)
    _check_start_pose(robot, trajectory)
    targets = _waypoint_targets(trajectory)
    stepper = _Stepper(robot, safety_config, lower, upper, trajectory.q[0], record, on_step)
    n_commands, abort = 0, None
    for row, label in zip(trajectory.q[1:], trajectory.labels[1:], strict=True):
        target = None if targets is None else targets[SEGMENT_WAYPOINT[label]]
        result = stepper.cycle(row, label, target)
        n_commands += 1
        if result.rejected or result.stopped:
            abort = (result.reason, label)
            break
    settle_steps = math.ceil(settle_s / safety_config.control_dt)
    for _ in range(settle_steps):
        stepper.cycle(stepper.prev, SETTLE_LABEL, None)
    progress = _Progress(n_commands, settle_steps, abort)
    return _build_result(robot, stepper, progress, targets is not None, started, safety_config.control_dt)


def _build_result(
    robot: Robot,
    stepper: _Stepper,
    progress: _Progress,
    workspace_checked: bool,
    started: float,
    control_dt: float,
) -> ExecutionResult:
    """【機能概要】: 実行後の状態を読み、ExecutionResult にまとめる 🟡"""
    final_state = robot.get_state()
    final_command = stepper.prev.copy()
    final_command.flags.writeable = False  # 【不変化】: 結果の指令は書き換え不可
    abort = progress.abort
    return ExecutionResult(
        n_commands=progress.n_commands,
        settle_steps=progress.settle_steps,
        flag_counts=MappingProxyType(dict(stepper.counts)),
        aborted=abort is not None,
        abort_reason=None if abort is None else abort[0],
        abort_label=None if abort is None else abort[1],
        workspace_checked=workspace_checked,
        final_command=final_command,
        final_state=final_state,
        final_joint_error=float(np.max(np.abs(final_state.q - final_command))),
        duration_s=(progress.n_commands + progress.settle_steps) * control_dt,
        wall_time_s=time.perf_counter() - started,
        states=tuple(stepper.states) if stepper.recording else None,
    )

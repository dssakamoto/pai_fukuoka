"""IkPolicy: IK と関節空間の線形補間による pick & place 軌道の生成 (TASK-0012)。

🔵 design.md 4.5、trajectory-requirements.md 2.4〜2.8 節。
処理: 入力検証 -> TCP 目標 -> 作業範囲 -> carry_too_low -> IK -> 補間 -> 指令列の検査
-> 経路の作業範囲 -> すき間。どこで失敗しても TrajectoryError (部分的な軌道は返さない)。
solve_ik / build_joint_trajectory は大域名から引く (テストで差し替えられる)。
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass

import mujoco
import numpy as np

from so101_vlm.policy.base import Goal, Observation
from so101_vlm.policy.clearance import ClearanceChecker
from so101_vlm.policy.ik import PITCH_CANDIDATES_DEG, solve_ik
from so101_vlm.policy.ik_types import IkError, IkSolution
from so101_vlm.policy.trajectory import (
    REASON_CARRY_TOO_LOW,
    REASON_OBSTACLE_CLEARANCE,
    REASON_PATH_OUT_OF_WORKSPACE,
    REASON_SELF_COLLISION,
    REASON_TARGET_OUT_OF_WORKSPACE,
    JointTrajectory,
    JointWaypoint,
    TrajectoryError,
    WaypointRecord,
    build_joint_trajectory,
    check_command_sequence,
    find_clearance_violation,
)
from so101_vlm.policy.trajectory_config import TrajectoryConfig, load_trajectory_config
from so101_vlm.safety.config import SafetyConfig, load_safety_config
from so101_vlm.safety.limits import limits_from_model
from so101_vlm.safety.workspace import is_in_workspace

# 【定数定義】: 作業範囲を IK の前に確認する経由点 (この順に確認する) 🔵
WAYPOINT_ORDER = ("pre_grasp", "grasp", "lift", "pre_place", "place", "retreat")
# 【区間表】: (区間名, q_wp のキー, 保持時間の設定名)。保持時間が None の区間は補間する 🔵
SEGMENT_TABLE: tuple[tuple[str, str, str | None], ...] = (
    ("pre_grasp", "pre_grasp", None),
    ("grasp", "grasp", None),
    ("close", "close", None),
    ("close_hold", "close", "close_hold_s"),
    ("lift", "lift", None),
    ("pre_place", "pre_place", None),
    ("place", "place", None),
    ("open", "open", None),
    ("open_hold", "open", "open_hold_s"),
    ("retreat", "retreat", None),
)
ROLL_INDEX = 4
GRIPPER_INDEX = 5
N_ARM_JOINTS = 5  # 【腕の関節数】: gripper を除く


def _build_targets(goal: Goal, cfg: TrajectoryConfig) -> dict[str, np.ndarray]:
    """【機能概要】: 6 つの経由点の TCP 目標を作る (要件 2.5 節) 🔵"""
    pick, place = goal.pick_target_base, goal.place_target_base
    pre_grasp = pick + np.array([0.0, 0.0, cfg.approach_height])
    pre_place = np.array([place[0], place[1], pre_grasp[2]])
    above_place = np.array([place[0], place[1], place[2] + cfg.place_height])
    return {
        "pre_grasp": pre_grasp, "grasp": pick.copy(), "lift": pre_grasp.copy(),
        "pre_place": pre_place, "place": above_place, "retreat": pre_place.copy(),
    }  # fmt: skip


@dataclass(frozen=True)
class _Solutions:
    """【機能概要】: 4 回の IK の結果。🟡"""

    pre_grasp: IkSolution
    grasp: IkSolution
    pre_place: IkSolution
    place: IkSolution


class IkPolicy:
    """【機能概要】: Policy の v1 実装。掴む点と置く点から関節軌道を返す。🔵

    状態を持たない (act ごとに専用の MjData を使う)。モデル・入力を変更しない。
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        *,
        config: TrajectoryConfig | None = None,
        safety: SafetyConfig | None = None,
    ) -> None:
        self._model = model
        self._config = load_trajectory_config() if config is None else config
        self._safety = load_safety_config() if safety is None else safety
        self._lower, self._upper = limits_from_model(model)
        self._check_gripper_range()
        self._checker = ClearanceChecker(model, self._config.clearance.obstacle_geoms)

    def _check_gripper_range(self) -> None:
        """【機能概要】: gripper の開き / 閉じ指令が ctrlrange 内か確かめる (範囲外は ValueError) 🟡"""
        lo, up = self._lower[GRIPPER_INDEX], self._upper[GRIPPER_INDEX]
        for name in ("gripper_open", "gripper_closed"):
            value = getattr(self._config, name)
            if not lo <= value <= up:
                raise ValueError(f"{name} が ctrlrange [{lo}, {up}] の外です: {name}={value}")

    # ------------------------------------------------------------- 公開 API
    def act(self, obs: Observation, goal: Goal) -> JointTrajectory:
        """【機能概要】: 観測と目標から JointTrajectory を返す。失敗は TrajectoryError 🔵"""
        if not isinstance(obs, Observation) or not isinstance(goal, Goal):
            raise ValueError("obs は Observation、goal は Goal で指定してください")
        targets = _build_targets(goal, self._config)
        self._check_targets(targets)
        sols = self._solve_all(obs, targets)
        q_wp = self._waypoint_joints(obs, sols)
        traj = self._interpolate(obs, q_wp)
        # 【FK は 1 回】: 経路の作業範囲確認とすき間確認で同じサンプルの FK を共有する
        tcp, obstacle, own = self._checker.sample(traj.q, self._config.tcp_offset)
        self._check_path(traj, tcp)
        return self._finish(traj, targets, sols, q_wp, obstacle, own)

    # ------------------------------------------------------------- 目標の検査
    def _check_targets(self, targets: dict[str, np.ndarray]) -> None:
        """【機能概要】: 作業範囲 (IK の前) と運搬の高さを確かめる 🔵"""
        for name in WAYPOINT_ORDER:
            if not is_in_workspace(targets[name], self._safety.workspace):
                raise TrajectoryError(
                    REASON_TARGET_OUT_OF_WORKSPACE, name, {"target": targets[name]}
                )
        z = float(targets["lift"][2])
        if z < self._config.min_carry_z:
            raise TrajectoryError(REASON_CARRY_TOO_LOW, "lift", {"z": z})

    # ------------------------------------------------------------- IK
    def _ik(
        self, name: str, target: np.ndarray, seed: np.ndarray, candidates: Sequence[float]
    ) -> IkSolution:
        """【機能概要】: solve_ik を呼び、IkError を TrajectoryError に包む (__cause__ に元の例外) 🔵"""
        try:
            return solve_ik(
                self._model, target, seed,
                pitch_candidates_deg=candidates, tcp_offset=self._config.tcp_offset,
            )  # fmt: skip
        except IkError as err:
            raise TrajectoryError(err.reason, name, {"target": tuple(target)}) from err

    def _solve_all(self, obs: Observation, targets: dict[str, np.ndarray]) -> _Solutions:
        """【機能概要】: pre_grasp -> grasp -> pre_place -> place の順に、共通ピッチで IK を解く 🔵"""
        seed = np.array(obs.q)
        seed[GRIPPER_INDEX] = self._config.gripper_open
        pre_grasp = self._ik("pre_grasp", targets["pre_grasp"], seed, PITCH_CANDIDATES_DEG)
        p1 = (pre_grasp.pitch_candidate_deg,)
        grasp = self._ik("grasp", targets["grasp"], pre_grasp.q, p1)
        pre_place = self._ik("pre_place", targets["pre_place"], pre_grasp.q, PITCH_CANDIDATES_DEG)
        p2 = (pre_place.pitch_candidate_deg,)
        place = self._ik("place", targets["place"], pre_place.q, p2)
        return _Solutions(pre_grasp, grasp, pre_place, place)

    # ------------------------------------------------------------- 軌道
    def _waypoint_joints(self, obs: Observation, sols: _Solutions) -> dict[str, np.ndarray]:
        """【機能概要】: 経由点ごとの関節角 (roll は開始値、gripper は区間の値) を作る 🔵"""
        roll = float(np.clip(obs.q[ROLL_INDEX], self._lower[ROLL_INDEX], self._upper[ROLL_INDEX]))
        opened, closed = self._config.gripper_open, self._config.gripper_closed

        def joints(sol: IkSolution, gripper: float) -> np.ndarray:
            q = np.array(sol.q)
            q[ROLL_INDEX], q[GRIPPER_INDEX] = roll, gripper
            return q

        return {
            "pre_grasp": joints(sols.pre_grasp, opened), "grasp": joints(sols.grasp, opened),
            "close": joints(sols.grasp, closed), "lift": joints(sols.pre_grasp, closed),
            "pre_place": joints(sols.pre_place, closed), "place": joints(sols.place, closed),
            "open": joints(sols.place, opened), "retreat": joints(sols.pre_place, opened),
        }  # fmt: skip

    def _interpolate(self, obs: Observation, q_wp: dict[str, np.ndarray]) -> JointTrajectory:
        """【機能概要】: 経由点を補間して軌道にし、指令列の定義を検査する 🔵"""
        cfg = self._config
        waypoints = [
            JointWaypoint(name, q_wp[key], None if hold is None else getattr(cfg, hold))
            for name, key, hold in SEGMENT_TABLE
        ]  # 【表駆動】: 区間の並びは SEGMENT_TABLE だけで決まる
        traj = build_joint_trajectory(
            obs.q, waypoints,
            vmax=self._safety.vmax, speed_scale=self._safety.speed_scale,
            control_dt=self._safety.control_dt, safety_factor=cfg.time_safety_factor,
            min_segment_s=cfg.min_segment_s,
        )  # fmt: skip
        check_command_sequence(
            traj.q, traj.labels, lower=self._lower, upper=self._upper,
            vmax=self._safety.vmax, speed_scale=self._safety.speed_scale,
            control_dt=self._safety.control_dt,
        )  # fmt: skip
        return traj

    def _check_path(self, traj: JointTrajectory, tcp: np.ndarray) -> None:
        """【機能概要】: pre_grasp の終点より後の TCP 経路 (tcp は全サンプル分) が作業範囲内か確かめる 🟡"""
        first = traj.labels.index("grasp")
        for offset, point in enumerate(tcp[first:]):
            if not is_in_workspace(point, self._safety.workspace):
                raise TrajectoryError(
                    REASON_PATH_OUT_OF_WORKSPACE, traj.labels[first + offset], {"tcp": tuple(point)}
                )

    # ------------------------------------------------------------- 仕上げ
    def _clearances(
        self, traj: JointTrajectory, obstacle: np.ndarray, own: np.ndarray
    ) -> tuple[float, float]:
        """【機能概要】: 全サンプルのすき間を確かめ、(障害物, 自己) の最小値を返す (違反は例外) 🟡"""
        clr = self._config.clearance
        for reason, dist, limit in (
            (REASON_OBSTACLE_CLEARANCE, obstacle, clr.obstacle_m),
            (REASON_SELF_COLLISION, own, clr.self_m),
        ):  # 【判定順】: 障害物 -> 自己干渉 (O4)
            bad = find_clearance_violation(dist, traj.labels, limit)
            if bad is not None:
                raise TrajectoryError(
                    reason, bad.label, {"min": bad.value, "index": bad.index, "threshold": limit}
                )
        return float(obstacle.min()), float(own.min())

    def _records(
        self, targets: Mapping[str, np.ndarray], sols: _Solutions, q_wp: Mapping[str, np.ndarray]
    ) -> tuple[WaypointRecord, ...]:
        """【機能概要】: metadata["waypoints"] の記録を WAYPOINT_ORDER の順に作る 🟡"""
        source = {
            "pre_grasp": sols.pre_grasp, "grasp": sols.grasp, "lift": sols.pre_grasp,
            "pre_place": sols.pre_place, "place": sols.place, "retreat": sols.pre_place,
        }  # fmt: skip
        return tuple(
            WaypointRecord(
                name, targets[name], q_wp[name], source[name].pos_error,
                source[name].pitch_deg, source[name].pitch_candidate_deg,
            )
            for name in WAYPOINT_ORDER
        )  # fmt: skip

    def _finish(
        self,
        traj: JointTrajectory,
        targets: Mapping[str, np.ndarray],
        sols: _Solutions,
        q_wp: Mapping[str, np.ndarray],
        obstacle: np.ndarray,
        own: np.ndarray,
    ) -> JointTrajectory:
        """【機能概要】: すき間を確認し、metadata を付けた最終の JointTrajectory を返す 🟡"""
        min_obstacle, min_self = self._clearances(traj, obstacle, own)
        metadata = {
            "control_dt": self._safety.control_dt, "vmax": tuple(self._safety.vmax),
            "speed_scale": self._safety.speed_scale,
            "time_safety_factor": self._config.time_safety_factor,
            "tcp_offset": self._config.tcp_offset,
            "pick_pitch_deg": sols.pre_grasp.pitch_candidate_deg,
            "place_pitch_deg": sols.pre_place.pitch_candidate_deg,
            "waypoints": self._records(targets, sols, q_wp),
            "min_obstacle_clearance_m": min_obstacle, "min_self_clearance_m": min_self,
            "duration_s": float(traj.times[-1]),
        }  # fmt: skip
        return JointTrajectory(traj.times, traj.q, traj.labels, traj.segments, metadata)

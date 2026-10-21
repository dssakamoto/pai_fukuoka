"""Policy 抽象と入力型 (Observation / Goal) (TASK-0012)。

🔵 design.md 4.2 の `Policy.act(obs, goal) -> JointTrajectory` に基づく。
MuJoCo には依存しない (型のみ)。画像などの観測は v2 (SmolVLA) で追加する (YAGNI)。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Protocol, runtime_checkable

import numpy as np

from so101_vlm.eval.validation import require_number
from so101_vlm.robot.base import N_JOINTS, RobotState
from so101_vlm.safety._validation import as_vector

if TYPE_CHECKING:
    from so101_vlm.policy.trajectory import JointTrajectory

_N_AXES = 3


def _frozen_vector(name: str, value: object, size: int) -> np.ndarray:
    """【機能概要】: 検証済みの float64 コピーを読み取り専用にして返す (bool・NaN・shape 違いは ValueError) 🟡"""
    vec = as_vector(name, value, size)
    vec.flags.writeable = False
    return vec


@dataclass(frozen=True)
class Observation:
    """【機能概要】: 方策への観測。q は (6,) の関節角 [rad]、timestamp は状態の時刻 [s]。🟡

    生成時にコピーして読み取り専用にする (呼び出し側の配列と共有しない)。
    """

    q: np.ndarray
    timestamp: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "q", _frozen_vector("q", self.q, N_JOINTS))
        object.__setattr__(self, "timestamp", require_number("timestamp", self.timestamp))

    @classmethod
    def from_state(cls, state: RobotState) -> Observation:
        """【機能概要】: RobotState から Observation を作る (q と timestamp を引き継ぐ) 🟡"""
        return cls(q=state.q, timestamp=state.timestamp)


@dataclass(frozen=True)
class Goal:
    """【機能概要】: 方策への目標。🟡

    pick_target_base: 掴むブロックの中心 [m] (baseframe)、
    place_target_base: 置く面の点 [m] (箱の底板上面の中心、baseframe)、
    instruction: 自然言語の指示 (IkPolicy は使わない。記録・v2 用)。
    """

    pick_target_base: np.ndarray
    place_target_base: np.ndarray
    instruction: str = ""

    def __post_init__(self) -> None:
        if not isinstance(self.instruction, str):
            raise ValueError(f"instruction は str で指定してください: {self.instruction!r}")
        for name in ("pick_target_base", "place_target_base"):
            object.__setattr__(self, name, _frozen_vector(name, getattr(self, name), _N_AXES))


@runtime_checkable
class Policy(Protocol):
    """【機能概要】: 観測と目標から関節軌道を返す方策の抽象 (v1: IkPolicy、v2: SmolVlaPolicy)。🔵"""

    def act(self, obs: Observation, goal: Goal) -> JointTrajectory: ...

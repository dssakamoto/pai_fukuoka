"""Policy 抽象と入力型 (Observation / Goal) (TASK-0012)。

🔵 design.md 4.2 の `Policy.act(obs, goal) -> JointTrajectory` に基づく。
MuJoCo には依存しない (型のみ)。画像などの観測は v2 (SmolVLA) で追加する (YAGNI)。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Protocol, runtime_checkable

import numpy as np

from so101_vlm.robot.base import N_JOINTS, RobotState
from so101_vlm.safety._validation import as_vector
from so101_vlm.validation import require_number

if TYPE_CHECKING:
    from so101_vlm.policy.trajectory import JointTrajectory

_N_AXES = 3


def _frozen_vector(name: str, value: object, size: int) -> np.ndarray:
    """【機能概要】: 検証済みの float64 コピーを読み取り専用にして返す (bool・NaN・shape 違いは ValueError) 🟡"""
    vec = as_vector(name, value, size)
    vec.flags.writeable = False
    return vec


def _frozen_image(name: str, value: object, *, ndim: int, dtype_kind: str) -> np.ndarray | None:
    """【機能概要】: 画像 (None 可) を検証し、読み取り専用のコピーにして返す。型・次元の違いは ValueError 🟡

    dtype_kind: "u" (uint8 の RGB) か "f" (float の深度)。深度は欠測 (0 や inf) があり得るので有限は要求しない。
    """
    if value is None:
        return None
    kind = "uint8" if dtype_kind == "u" else "float"
    if not isinstance(value, np.ndarray) or value.dtype.kind != dtype_kind or value.ndim != ndim:
        raise ValueError(f"{name} は {ndim} 次元の {kind} 配列で指定してください")
    if ndim == 3 and value.shape[2] != 3:
        raise ValueError(f"{name} は (H, W, 3) で指定してください: shape={value.shape}")
    if value.size == 0:
        raise ValueError(f"{name} が空です: shape={value.shape}")
    copy = np.array(value, copy=True)
    copy.flags.writeable = False
    return copy


@dataclass(frozen=True)
class Observation:
    """【機能概要】: 方策への観測。q は (6,) の関節角 [rad]、timestamp は状態の時刻 [s]。🟡

    rgb ((H, W, 3) uint8)・depth ((H, W) float [m])・instruction は任意 (SGT-008)。真値 (GroundTruth / Scenario) は持たない
    (方策は、評価用の真値を見てはいけない。FR-004 / FR-402)。
    生成時にコピーして読み取り専用にする (呼び出し側の配列と共有しない)。
    """

    q: np.ndarray
    timestamp: float
    rgb: np.ndarray | None = None
    depth: np.ndarray | None = None
    instruction: str = ""

    def __post_init__(self) -> None:
        object.__setattr__(self, "q", _frozen_vector("q", self.q, N_JOINTS))
        object.__setattr__(self, "timestamp", require_number("timestamp", self.timestamp))
        object.__setattr__(self, "rgb", _frozen_image("rgb", self.rgb, ndim=3, dtype_kind="u"))
        object.__setattr__(self, "depth", _frozen_image("depth", self.depth, ndim=2, dtype_kind="f"))
        if not isinstance(self.instruction, str):
            raise ValueError(f"instruction は str で指定してください: {self.instruction!r}")

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

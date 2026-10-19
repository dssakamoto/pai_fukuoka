"""Robot 抽象 (Protocol) と RobotState。

関節順は model_paths.JOINT_NAMES を唯一の定義とし、ここから再エクスポートする
(sim と実機で同じ順序を使う。design.md 4.1)。単位は rad / rad/s。
"""

from dataclasses import dataclass
from typing import Protocol, runtime_checkable

import numpy as np

from so101_vlm.robot.model_paths import JOINT_NAMES

__all__ = ["JOINT_NAMES", "N_JOINTS", "Robot", "RobotState", "validate_q_target"]

# 【関節数】: 関節順の唯一の定義 JOINT_NAMES から導出する 🔵
N_JOINTS = len(JOINT_NAMES)


def _readonly_copy(values: np.ndarray) -> np.ndarray:
    """【機能概要】: 配列を float64 のコピーにして読み取り専用にする。🟡"""
    copied = np.array(values, dtype=np.float64, copy=True)
    copied.flags.writeable = False
    return copied


def validate_q_target(q_target: np.ndarray) -> np.ndarray:
    """【ヘルパー関数】: 関節目標の検証と float64 コピーを返す(入力は変更しない)。🟡

    【再利用性】: SimRobot / 実機 Robot(TASK-0034)の send_command で共通利用する。
    shape が (N_JOINTS,) でない、または NaN/inf を含む場合は ValueError。
    """
    target = np.array(q_target, dtype=np.float64, copy=True)
    if target.shape != (N_JOINTS,):
        raise ValueError(f"q_target must have shape ({N_JOINTS},), got {target.shape}")
    if not np.all(np.isfinite(target)):
        raise ValueError("q_target must be finite (no NaN/inf)")
    return target


@dataclass(frozen=True)
class RobotState:
    """【機能概要】: ロボットのある時点の状態。変更不可。🟡

    q: 関節角 [rad] (6,)、dq: 関節速度 [rad/s] (6,)、timestamp: 時刻 [s]。
    q / dq は生成時にコピーされ、読み取り専用になる(呼び出し側の配列と共有しない)。
    """

    q: np.ndarray
    dq: np.ndarray
    timestamp: float

    def __post_init__(self) -> None:
        # 【不変性】: frozen でも配列の中身は書き換えられるため、コピーして書き込み禁止にする
        object.__setattr__(self, "q", _readonly_copy(self.q))
        object.__setattr__(self, "dq", _readonly_copy(self.dq))
        object.__setattr__(self, "timestamp", float(self.timestamp))


@runtime_checkable
class Robot(Protocol):
    """【機能概要】: sim / 実機共通のロボット操作インタフェース (design.md 4.1)。🔵"""

    joint_names: tuple[str, ...]

    def get_state(self) -> RobotState: ...

    def send_command(self, q_target: np.ndarray) -> None: ...

    def close(self) -> None: ...

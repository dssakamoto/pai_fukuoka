"""ロボットの状態・シミュレーションの世界の、配列への詰め替え (TASK-0025)。

🔵 float64 のベクトルは dtype を保ったまま届く (ADR 0001)。robot_state = [q(6), dq(6), t] の 13 要素、
world = [qpos(nq), qvel(nv)] (sim のみ。camera の描画と evaluator の判定が、ロボットと同じ世界を再現するのに使う)。
"""

import mujoco
import numpy as np

from so101_vlm.robot.base import N_JOINTS, RobotState

STATE_SIZE = 2 * N_JOINTS + 1


def pack_state(state: RobotState) -> np.ndarray:
    """【機能概要】: RobotState を (13,) の float64 にする 🔵"""
    return np.concatenate([state.q, state.dq, [state.timestamp]]).astype(np.float64)


def unpack_state(arr: object) -> RobotState:
    """【機能概要】: pack_state の逆。形が違えば ValueError 🔵"""
    a = np.asarray(arr, dtype=np.float64)
    if a.shape != (STATE_SIZE,):
        raise ValueError(f"robot_state は ({STATE_SIZE},) で指定してください: shape={a.shape}")
    return RobotState(q=a[:N_JOINTS], dq=a[N_JOINTS : 2 * N_JOINTS], timestamp=float(a[-1]))


def pack_world(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    """【機能概要】: qpos と qvel をつないだ (nq + nv,) の float64 にする 🔵"""
    return np.concatenate([data.qpos, data.qvel]).astype(np.float64)


def apply_world(model: mujoco.MjModel, data: mujoco.MjData, world: object) -> None:
    """【機能概要】: world を data に書き込み、mj_forward で位置・姿勢を再計算する。形が違えば ValueError 🔵"""
    w = np.asarray(world, dtype=np.float64)
    if w.shape != (model.nq + model.nv,):
        raise ValueError(f"world は ({model.nq + model.nv},) で指定してください: shape={w.shape}")
    data.qpos[:] = w[: model.nq]
    data.qvel[:] = w[model.nq :]
    mujoco.mj_forward(model, data)

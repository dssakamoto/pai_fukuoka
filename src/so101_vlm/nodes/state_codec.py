"""ロボットの状態・シミュレーションの世界の、配列への詰め替え (TASK-0025)。

🔵 float64 のベクトルは dtype を保ったまま届く (ADR 0001)。robot_state = [q(6), dq(6), t] の 13 要素、
world = [qpos(nq), qvel(nv)] + 箱の (x, y) (sim のみ。camera の描画と evaluator の判定が、ロボットと同じ世界を再現するのに使う)。
"""

import mujoco
import numpy as np

from so101_vlm.eval.geometry import BOX_BODY_NAME
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


def _box_id(model: mujoco.MjModel) -> int:
    return int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, BOX_BODY_NAME))


def pack_world(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    """【機能概要】: qpos と qvel をつないだ float64 にする。箱の body があれば、末尾に箱の (x, y) を付ける (SGT-007) 🔵

    箱は自由関節を持たず qpos に入らないので、箱を動かした試行 (apply_scenario) を camera / evaluator が再現できるよう、
    箱の位置を別に運ぶ。形は (nq + nv,) か、箱があれば (nq + nv + 2,)。
    """
    world = np.concatenate([data.qpos, data.qvel]).astype(np.float64)
    box = _box_id(model)
    if box < 0:
        return world
    return np.concatenate([world, model.body_pos[box][:2]]).astype(np.float64)


def apply_world(model: mujoco.MjModel, data: mujoco.MjData, world: object) -> None:
    """【機能概要】: world を data に書き込み、mj_forward で位置・姿勢を再計算する。形が違えば ValueError 🔵

    末尾に箱の (x, y) があれば (nq + nv + 2)、model の箱の body_pos にも反映する。旧形式 (nq + nv) は箱を動かさない。
    """
    w = np.asarray(world, dtype=np.float64)
    base = model.nq + model.nv
    box = _box_id(model)
    allowed = {base} if box < 0 else {base, base + 2}
    if w.ndim != 1 or w.shape[0] not in allowed:
        expected = " か ".join(f"({n},)" for n in sorted(allowed))
        raise ValueError(f"world は {expected} で指定してください: shape={w.shape}")
    data.qpos[:] = w[: model.nq]
    data.qvel[:] = w[model.nq : base]
    if w.shape[0] == base + 2:
        model.body_pos[box][:2] = w[base:]
    mujoco.mj_forward(model, data)

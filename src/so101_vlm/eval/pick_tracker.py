"""pick & place 1 試行の追跡 PickTracker (TASK-0013)。

🟡 sim の真値 (MjData) を毎制御周期に読み、failure.PickTrace の値を更新する。
読み取り専用 (mj_step を呼ばず、MjData を書き換えない)。sim 専用で、実機では使えない。
"""

import math

import mujoco
import numpy as np

from so101_vlm.eval.failure import PickTrace
from so101_vlm.eval.scene_spec import SCENE_SPEC
from so101_vlm.policy.segments import SEGMENT_LIFT, SEGMENT_PLACE, SEGMENT_PRE_PLACE

# 【定数定義】: 区間名ごとの追跡の対象。区間名の定義元は policy.segments (executor と共有) 🔵
LIFT_LABEL = SEGMENT_LIFT
CARRY_LABELS = frozenset({SEGMENT_LIFT, SEGMENT_PRE_PLACE})
PLACE_LABEL = SEGMENT_PLACE
_N_AXES = 3
_TCP_SITE = "gripperframe"


def tcp_position(model: mujoco.MjModel, data: mujoco.MjData, offset: tuple[float, ...]) -> np.ndarray:
    """【機能概要】: TCP の位置 = gripperframe の位置 + 回転 x tcp_offset (TASK-0012 と同じ定義) 🔵"""
    site = model.site(_TCP_SITE).id
    return data.site_xpos[site] + data.site_xmat[site].reshape(3, 3) @ np.asarray(offset)


def tilt_degrees(data: mujoco.MjData, body_id: int) -> float:
    """【機能概要】: ブロックの傾き [deg] (最も鉛直に近い軸と鉛直の角度。立方体なので最も近い面の法線) 🔵"""
    rotation = data.xmat[body_id].reshape(3, 3)
    cosine = min(1.0, float(np.max(np.abs(rotation[2, :]))))
    return math.degrees(math.acos(cosine))


def block_linear_velocity(model: mujoco.MjModel, data: mujoco.MjData, body_id: int) -> np.ndarray:
    """【機能概要】: freejoint を持つブロックの並進速度 [m/s] (3,) のコピーを返す 🔵"""
    dof = int(model.jnt_dofadr[model.body_jntadr[body_id]])
    return np.array(data.qvel[dof : dof + _N_AXES])


class PickTracker:
    """【機能概要】: 対象ブロックと他のブロックを追跡し、PickTrace を作る。🟡

    update(label) を送信の後に毎周期呼ぶ。区間名ごとに lift の終わりの高さ、運搬中・配置中の
    ブロックと TCP の距離の最大、他のブロックの xy 移動の最大を更新する。
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        target: str,
        tcp_offset: tuple[float, ...],
    ) -> None:
        self._model, self._data, self._offset = model, data, tcp_offset
        self._target = int(model.body(target).id)
        self._others = {
            name: int(model.body(name).id) for name in SCENE_SPEC.block_names if name != target
        }
        self._others_start = {n: data.xpos[i][:2].copy() for n, i in self._others.items()}
        self._lift_end_rise = 0.0
        self._sep_carry = 0.0
        self._sep_place = 0.0
        self._other_move = 0.0

    def update(self, label: str) -> None:
        """【機能概要】: 現在の MjData から追跡値を更新する 🟡"""
        block = self._data.xpos[self._target]
        if label == LIFT_LABEL:
            self._lift_end_rise = float(block[2]) - SCENE_SPEC.block_rest_z
        separation = float(
            np.linalg.norm(block - tcp_position(self._model, self._data, self._offset))
        )
        if label in CARRY_LABELS:
            self._sep_carry = max(self._sep_carry, separation)
        elif label == PLACE_LABEL:
            self._sep_place = max(self._sep_place, separation)
        for name, body in self._others.items():
            move = float(np.linalg.norm(self._data.xpos[body][:2] - self._others_start[name]))
            self._other_move = max(self._other_move, move)

    def finish(self) -> PickTrace:
        """【機能概要】: 最終の傾きを読み、PickTrace にまとめる 🟡"""
        return PickTrace(
            lift_end_rise=self._lift_end_rise,
            max_separation_carry=self._sep_carry,
            max_separation_place=self._sep_place,
            max_other_move=self._other_move,
            final_tilt_deg=tilt_degrees(self._data, self._target),
        )

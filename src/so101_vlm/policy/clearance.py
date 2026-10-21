"""FK によるすき間の計算 (ロボットと障害物、ロボットの自己干渉) (TASK-0012)。

🟡 trajectory-requirements.md 2.8 節。全サンプルで mj_geomDistance を計算する。
専用の MjData を使い、モデルは変更しない。
"""

from __future__ import annotations

from collections.abc import Sequence

import mujoco
import numpy as np

from so101_vlm.robot.base import JOINT_NAMES

BASE_BODY_NAME = "base"
# 【距離の上限】: これより遠い組は distmax を返す (しきい値 mm 単位に対し十分大きい) 🟡
GEOM_DISTANCE_MAX_M = 0.1
_FROMTO_SIZE = 6


def _robot_bodies(model: mujoco.MjModel) -> set[int]:
    """【機能概要】: base body とその子孫の body id の集合 🟡"""
    base = model.body(BASE_BODY_NAME).id
    bodies = set()
    for body in range(model.nbody):
        walker = body
        while walker != 0 and walker != base:
            walker = int(model.body_parentid[walker])
        if walker == base:
            bodies.add(body)
    return bodies


def _is_parent_child(model: mujoco.MjModel, a: int, b: int) -> bool:
    """【機能概要】: body a と b が同じ、または親子関係なら True 🟡"""
    return a == b or model.body_parentid[a] == b or model.body_parentid[b] == a


class ClearanceChecker:
    """【機能概要】: 関節角の列から、サンプルごとの TCP 位置と最小すき間 (障害物・自己) を計算する 🟡"""

    def __init__(self, model: mujoco.MjModel, obstacle_geoms: Sequence[str]) -> None:
        geom = mujoco.mjtObj.mjOBJ_GEOM
        missing = [n for n in obstacle_geoms if mujoco.mj_name2id(model, geom, n) < 0]
        if missing:
            raise ValueError(f"障害物の geom がモデルにありません: {', '.join(missing)}")
        bodies = _robot_bodies(model)
        self._model = model
        self._obstacles = [model.geom(n).id for n in obstacle_geoms]
        self._robot = [
            g for g in range(model.ngeom)
            if model.geom_bodyid[g] in bodies and (model.geom_contype[g] or model.geom_conaffinity[g])
        ]  # fmt: skip
        self._pairs = [
            (a, b)
            for i, a in enumerate(self._robot)
            for b in self._robot[i + 1 :]
            if not _is_parent_child(model, int(model.geom_bodyid[a]), int(model.geom_bodyid[b]))
        ]
        self._qpos_adr = [int(model.joint(n).qposadr[0]) for n in JOINT_NAMES]

    def sample(
        self, q_rows: np.ndarray, tcp_offset: Sequence[float]
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """【機能概要】: サンプルごとに FK を 1 回だけ計算し、(TCP 位置, 障害物との最小距離, 自己干渉の最小距離) を返す 🟡

        【改善内容】: 経路の作業範囲確認とすき間確認で FK を二重に計算していたのを 1 回にまとめた。
        TCP は gripperframe + R·tcp_offset (N, 3) [m]、距離は (N,) [m]。
        """
        model, data = self._model, mujoco.MjData(self._model)
        site = model.site("gripperframe").id
        offset = np.asarray(tcp_offset, dtype=np.float64)
        fromto = np.zeros(_FROMTO_SIZE)

        def dist(a: int, b: int) -> float:
            return mujoco.mj_geomDistance(model, data, a, b, GEOM_DISTANCE_MAX_M, fromto)

        n = len(q_rows)
        tcp, obstacle, own = np.empty((n, 3)), np.empty(n), np.empty(n)
        for k, q in enumerate(q_rows):
            self._forward(data, q)
            tcp[k] = data.site_xpos[site] + data.site_xmat[site].reshape(3, 3) @ offset
            obstacle[k] = min(dist(g, o) for g in self._robot for o in self._obstacles)
            own[k] = min(dist(a, b) for a, b in self._pairs)
        return tcp, obstacle, own

    def _forward(self, data: mujoco.MjData, q: np.ndarray) -> None:
        """【機能概要】: 関節角を入れて FK を計算する (qpos のみ変更) 🔵"""
        data.qpos[self._qpos_adr] = q
        mujoco.mj_kinematics(self._model, data)
        mujoco.mj_comPos(self._model, data)

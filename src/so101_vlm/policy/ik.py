"""姿勢制約付き逆運動学 solve_ik (TASK-0011)。

【機能概要】: gripperframe を目標位置に置く 4 関節 (pan, lift, elbow, wrist_flex) を、
mj_jac によるヤコビアンの減衰最小二乗法 (DLS) で求める。接近軸 (gripperframe の +x) の
「下向き成分」だけを弱く拘束し、ピッチ候補を真下に近い順に試す。
【設計方針】: 専用 MjData を使い、入力・モデル・実行中のシミュレーションを変更しない。
乱数は使わず、追加シードは固定値。到達球の外は反復せず即座に out_of_reach。
🔵 信頼性: ik-requirements.md / reachability-survey.md に基づく。
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import mujoco
import numpy as np
from numpy.typing import ArrayLike

from so101_vlm.policy.ik_types import (
    REASON_NOT_CONVERGED,
    REASON_OUT_OF_REACH,
    IkError,
    IkSolution,
)
from so101_vlm.robot.base import N_JOINTS, validate_q_target
from so101_vlm.robot.model_paths import JOINT_NAMES

__all__ = [
    "APPROACH_AXIS_LOCAL",
    "DAMPING",
    "DEFAULT_PITCH_CANDIDATES_DEG",
    "PITCH_CANDIDATES_DEG",
    "PITCH_TOL_DEG",
    "POS_TOL_M",
    "IkError",
    "IkSolution",
    "solve_ik",
]

# 【定数定義】: 数値は reachability-survey.md の実測に基づく 🟡
PITCH_CANDIDATES_DEG = (0.0, 10.0, 20.0, 30.0, 45.0, 60.0)  # 真下に近い順
DEFAULT_PITCH_CANDIDATES_DEG = PITCH_CANDIDATES_DEG
PITCH_TOL_DEG = 5.0  # 候補との許容差 [度]
POS_TOL_M = 0.001  # 収束判定の位置誤差 [m]
DAMPING = 0.01  # DLS の減衰 λ
MAX_STEP_RAD = 0.2  # 1 反復の更新量ノルム上限 [rad]
MAX_ITERS = 300  # 1 シード・1 候補あたりの反復数
ORIENTATION_WEIGHT = 0.1  # 傾き残差の重み
APPROACH_AXIS_LOCAL = (1.0, 0.0, 0.0)  # gripperframe 局所の接近軸 (指の向き)
N_SOLVED = 4  # 解く関節数 (pan, lift, elbow, wrist_flex)
IDX_PAN, IDX_LIFT, IDX_ROLL = 0, 1, 4  # JOINT_NAMES 内の添字
MIN_PITCH_DEG, MAX_PITCH_DEG = 0.0, 90.0
# 【停滞判定】: 監視するのは「位置誤差」だけ (傾き誤差は見ない)。位置誤差が
# STAGNATION_MIN_GAIN_M 以上減らない状態が STAGNATION_WINDOW 反復続けたら、そのシードを諦める。
# 傾きは収束判定 (POS_TOL_M かつ PITCH_TOL_DEG) で別途見るため、位置が収束していれば停滞しない。
STAGNATION_WINDOW = 25  # この反復数で改善が無ければ打ち切る
STAGNATION_MIN_GAIN_M = 1e-4  # 「改善」とみなす最小の位置誤差の減少 (0.1 mm)
SITE_NAME = "gripperframe"
# 【固定シード】: wrist_flex 上限付近 (1.55〜1.65) を含む。pan は目標の方位角に置換する
EXTRA_SEEDS = (
    (0.0, 0.0, 0.0, 0.0),
    (0.0, -0.5, 0.5, 1.2),
    (0.0, 0.5, 0.5, 1.0),
    (0.0, 1.0, -0.5, 1.2),
    (0.0, -1.0, 1.5, 0.5),
    (0.0, 0.3, 1.2, -0.5),
    (0.0, 0.3, 0.0, 1.60),
    (0.0, -0.3, 0.8, 1.62),
    (0.0, 0.6, -0.2, 1.65),
)


class _Arm:
    """【機能概要】: モデルから関節・サイトの ID と可動域を引き、順運動学とヤコビアンを提供する。🟡"""

    def __init__(self, model: mujoco.MjModel) -> None:
        self.model = model
        self.site_id = model.site(SITE_NAME).id
        self.body_id = int(model.site_bodyid[self.site_id])
        joint_ids = [model.joint(n).id for n in JOINT_NAMES]
        act_ids = [model.actuator(n).id for n in JOINT_NAMES]
        self.qpos_adr = np.array([model.jnt_qposadr[j] for j in joint_ids])
        self.dof_adr = np.array([model.jnt_dofadr[j] for j in joint_ids])
        self.lo = model.actuator_ctrlrange[act_ids, 0].copy()
        self.hi = model.actuator_ctrlrange[act_ids, 1].copy()
        data = mujoco.MjData(model)
        mujoco.mj_kinematics(model, data)
        self.lift_anchor = data.xanchor[joint_ids[IDX_LIFT]].copy()
        self.pan_xy = data.xanchor[joint_ids[IDX_PAN]][:2].copy()
        self.radius = _reach_radius(data, joint_ids, self.site_id)

    def pose(
        self, data: mujoco.MjData, q: np.ndarray, offset: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """q での (目標点の位置, gripperframe の回転行列) を返す。"""
        data.qpos[:] = self.model.qpos0
        data.qpos[self.qpos_adr] = q
        mujoco.mj_kinematics(self.model, data)
        mujoco.mj_comPos(self.model, data)
        rot = data.site_xmat[self.site_id].reshape(3, 3).copy()
        return data.site_xpos[self.site_id] + rot @ offset, rot

    def jacobians(
        self, data: mujoco.MjData, point: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """point の位置ヤコビアンと回転ヤコビアン (解く 4 関節分) を返す。"""
        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))
        mujoco.mj_jac(self.model, data, jacp, jacr, point, self.body_id)
        cols = self.dof_adr[:N_SOLVED]
        return jacp[:, cols], jacr[:, cols]


def _max_wrist_to_site(data: mujoco.MjData, joint_ids: list[int], site_id: int) -> float:
    """【機能概要】: roll を振ったときの wrist_flex アンカー→サイト距離の最大値を返す。🔵

    サイトは roll 軸まわりの円周上を動く。軸方向の差 h、アンカーの軸からの距離 d_w、
    サイトの軸からの距離 d_s とすると、最大距離は円周の反対側で sqrt(h^2 + (d_w + d_s)^2)。
    """
    flex = data.xanchor[joint_ids[IDX_ROLL - 1]]
    roll_anchor = data.xanchor[joint_ids[IDX_ROLL]]
    axis = data.xaxis[joint_ids[IDX_ROLL]]
    site_rel = data.site_xpos[site_id] - roll_anchor
    flex_rel = flex - roll_anchor
    h = float((site_rel - flex_rel) @ axis)
    d_s = float(np.linalg.norm(site_rel - (site_rel @ axis) * axis))
    d_w = float(np.linalg.norm(flex_rel - (flex_rel @ axis) * axis))
    return float(np.hypot(h, d_w + d_s))


def _reach_radius(data: mujoco.MjData, joint_ids: list[int], site_id: int) -> float:
    """【機能概要】: lift 軸から gripperframe までの最大距離の上界 (保守側) を求める。🔵
    【改善内容】: 旧 docstring は roll 軸のずれの加算を謳っていたが実装に無かったため、実装を合わせた。
    qpos0 での lift→elbow、elbow→wrist_flex アンカー間距離 + roll を振ったときの
    wrist_flex アンカー→サイト距離の最大値 (_max_wrist_to_site) の合計。
    roll は seed 任意なので、roll を考慮しないと届く点を out_of_reach と誤判定しうる
    (実測: 手首→サイト距離は roll で 160.5 mm → 161.3 mm に伸びる)。
    早期棄却は保守的であるべきで、この上界は届く点を棄却しない。
    """
    lift, elbow, flex = (data.xanchor[j] for j in joint_ids[IDX_LIFT:N_SOLVED])
    links = float(np.linalg.norm(elbow - lift) + np.linalg.norm(flex - elbow))
    return links + _max_wrist_to_site(data, joint_ids, site_id)


def _validate_inputs(
    target_pos: ArrayLike,
    seed_q: ArrayLike,
    tcp_offset: ArrayLike,
    pitch_candidates_deg: Sequence[float],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """【入力値検証】: shape・有限性・候補の範囲を検証し float64 コピーを返す。🟡"""
    target = np.array(target_pos, dtype=np.float64, copy=True)
    if target.shape != (3,) or not np.all(np.isfinite(target)):
        raise ValueError(f"target_pos must be finite with shape (3,), got {target.shape}")
    offset = np.array(tcp_offset, dtype=np.float64, copy=True)
    if offset.shape != (3,) or not np.all(np.isfinite(offset)):
        raise ValueError(f"tcp_offset must be finite with shape (3,), got {offset.shape}")
    seed = validate_q_target(np.asarray(seed_q))
    cands = np.array(pitch_candidates_deg, dtype=np.float64, copy=True)
    if cands.ndim != 1 or cands.size == 0 or not np.all(np.isfinite(cands)):
        raise ValueError("pitch_candidates_deg must be a non-empty finite 1-D sequence")
    if np.any(cands < MIN_PITCH_DEG) or np.any(cands > MAX_PITCH_DEG):
        raise ValueError(f"pitch_candidates_deg must be within [{MIN_PITCH_DEG}, {MAX_PITCH_DEG}]")
    return target, seed, offset, cands


def _build_starts(arm: _Arm, seed: np.ndarray, target: np.ndarray) -> list[np.ndarray]:
    """【機能概要】: seed と固定の追加シード (pan は目標の方位角) を順に並べる。🟡"""
    dx, dy = target[0] - arm.pan_xy[0], target[1] - arm.pan_xy[1]
    pan = float(np.clip(np.arctan2(dy, dx), arm.lo[IDX_PAN], arm.hi[IDX_PAN]))
    starts = [seed.copy()]
    for extra in EXTRA_SEEDS:
        start = seed.copy()
        start[:N_SOLVED] = extra
        start[IDX_PAN] = pan
        starts.append(start)
    return starts


@dataclass(frozen=True)
class _Problem:
    """【機能概要】: 1 回の solve_ik で共通の問題定義 (腕・作業用 MjData・目標・TCP オフセット)。🔵

    data は反復ごとに上書きされる作業用バッファ (専用に生成した MjData)。
    それ以外のフィールドは変更されない。引数の数を減らすためにまとめた。
    """

    arm: _Arm
    data: mujoco.MjData
    target: np.ndarray
    offset: np.ndarray


def _dls_step(
    prob: _Problem, q: np.ndarray, cos_target: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """【機能概要】: 1 反復分の (位置, 接近軸, 新しい q) を返す。🟡"""
    arm = prob.arm
    point, rot = arm.pose(prob.data, q, prob.offset)
    axis = rot[:, 0]
    jacp, jacr = arm.jacobians(prob.data, point)
    jac_tilt = (-np.cross(axis, jacr.T).T)[2:3]
    jac = np.vstack([jacp, ORIENTATION_WEIGHT * jac_tilt])
    err = np.concatenate([prob.target - point, [ORIENTATION_WEIGHT * (cos_target - axis[2])]])
    dq = jac.T @ np.linalg.solve(jac @ jac.T + DAMPING**2 * np.eye(N_SOLVED), err)
    norm = np.linalg.norm(dq)
    if norm > MAX_STEP_RAD:
        dq *= MAX_STEP_RAD / norm
    q_new = q.copy()
    q_new[:N_SOLVED] += dq
    return point, axis, np.clip(q_new, arm.lo, arm.hi)


def _run_dls(
    prob: _Problem, start: np.ndarray, pitch_deg: float
) -> tuple[bool, np.ndarray, float, float]:
    """【機能概要】: 1 つのシード・候補で DLS を回す。位置誤差が停滞したら打ち切る。🟡

    停滞判定は位置誤差のみを見る (定数 STAGNATION_* のコメント参照)。
    戻り値: (収束したか, q, 最良の位置誤差, そのときの傾き [度])
    """
    q = start.copy()
    cos_target = -np.cos(np.deg2rad(pitch_deg))
    best_err, best_pitch, last_gain_iter = np.inf, 0.0, 0
    for it in range(MAX_ITERS):
        point, axis, q_next = _dls_step(prob, q, cos_target)
        err = float(np.linalg.norm(prob.target - point))
        pitch = float(np.degrees(np.arccos(np.clip(-axis[2], -1.0, 1.0))))
        if err <= POS_TOL_M and abs(pitch - pitch_deg) <= PITCH_TOL_DEG:
            return True, q, err, pitch
        if err < best_err - STAGNATION_MIN_GAIN_M:
            last_gain_iter = it
        if err < best_err:
            best_err, best_pitch = err, pitch
        if it - last_gain_iter >= STAGNATION_WINDOW:
            break
        q = q_next
    return False, q, best_err, best_pitch


def _out_of_reach_error(target: np.ndarray, distance: float, radius: float) -> IkError:
    """【機能概要】: 到達球の外を示す IkError を作る。🔵"""
    t = tuple(float(v) for v in target)
    msg = (
        f"目標 {t} は到達範囲外です (肩からの距離 {distance:.4f} m > 到達半径 {radius:.4f} m)"
    )
    return IkError(t, REASON_OUT_OF_REACH, distance - radius, None, (), msg)


def _not_converged_error(
    target: np.ndarray, best_err: float, best_pitch: float | None, tried: tuple[float, ...]
) -> IkError:
    """【機能概要】: 未収束を示す IkError を作る。🔵"""
    t = tuple(float(v) for v in target)
    msg = (
        f"目標 {t} で IK が収束しませんでした (最良の位置誤差 {best_err * 1000:.1f} mm, "
        f"傾き {best_pitch:.1f} 度, 試した候補 {tried})"
    )
    return IkError(t, REASON_NOT_CONVERGED, best_err, best_pitch, tried, msg)


def solve_ik(
    model: mujoco.MjModel,
    target_pos: ArrayLike,
    seed_q: ArrayLike,
    *,
    pitch_candidates_deg: Sequence[float] = PITCH_CANDIDATES_DEG,
    tcp_offset: ArrayLike = (0.0, 0.0, 0.0),
) -> IkSolution:
    """【機能概要】: 目標位置に gripperframe (+ tcp_offset) を置く関節角を求める。🔵
    【実装方針】: 候補ピッチ(指定順)× シード(seed → 固定追加シード)で DLS を試し、
    最初に収束した解を返す。wrist_roll と gripper は seed(ctrlrange にクリップ)のまま。
    【テスト対応】: tests/policy/test_ik.py, test_ik_survey.py 全般。
    @raises ValueError: 入力の shape/値が不正 @raises IkError: 到達不可または未収束
    """
    target, seed, offset, cands = _validate_inputs(
        target_pos, seed_q, tcp_offset, pitch_candidates_deg
    )
    # 【_Arm のキャッシュは見送り】: 構築は約 0.6 ms (1 回の solve の 15% 程度)。MjModel は可変で
    # (ctrlrange 等を書き換えられる)、id/weakref キーのキャッシュは古い値を返す危険があるため、毎回作る。
    arm = _Arm(model)
    seed = np.clip(seed, arm.lo, arm.hi)
    distance = float(np.linalg.norm(target - arm.lift_anchor))
    radius = arm.radius + float(np.linalg.norm(offset))
    if distance > radius:
        raise _out_of_reach_error(target, distance, radius)
    prob = _Problem(arm, mujoco.MjData(model), target, offset)
    starts = _build_starts(arm, seed, target)
    best_err, best_pitch = np.inf, None
    for cand in cands:
        for start in starts:
            ok, q, err, pitch = _run_dls(prob, start, float(cand))
            if ok:
                return IkSolution(q, pitch, float(cand), err)
            if err < best_err:
                best_err, best_pitch = err, pitch
    tried = tuple(float(c) for c in cands)
    raise _not_converged_error(target, best_err, best_pitch, tried)

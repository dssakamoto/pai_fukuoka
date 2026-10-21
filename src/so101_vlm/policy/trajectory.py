"""軌道の純粋層: 時間配分・補間・JointTrajectory・指令列の検査・すき間の判定 (TASK-0012)。

MuJoCo に依存しない純粋関数と型だけを置く。入力を変更せず、乱数・時計を使わない。
🔵 時間配分は trajectory-requirements.md 2.7 節、型は 2.3 / 2.6 節に基づく。
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from types import MappingProxyType

import numpy as np

from so101_vlm.robot.base import N_JOINTS
from so101_vlm.safety._validation import (
    as_vector,
    require_fraction,
    require_positive,
    require_positive_vector,
)
from so101_vlm.safety.limits import REASON_OUT_OF_WORKSPACE

# 【理由文字列】: TrajectoryError.reason の値 (要件 2.6)。IK の失敗は IkError.reason をそのまま使う 🟡
REASON_TARGET_OUT_OF_WORKSPACE = REASON_OUT_OF_WORKSPACE
REASON_CARRY_TOO_LOW = "carry_too_low"
REASON_LIMIT_VIOLATION = "limit_violation"
REASON_PATH_OUT_OF_WORKSPACE = "path_out_of_workspace"
REASON_OBSTACLE_CLEARANCE = "obstacle_clearance"
REASON_SELF_COLLISION = "self_collision"

START_LABEL = "start"  # 【先頭の区間名】: q[0] (開始値) 1 サンプルだけ 🔵


def _readonly(value: object, ndim: int) -> np.ndarray:
    """【機能概要】: float64 のコピーを読み取り専用にして返す。有限でない値は ValueError 🟡"""
    arr = np.array(value, dtype=np.float64, copy=True)
    if arr.ndim != ndim or not np.all(np.isfinite(arr)):
        raise ValueError(f"有限の数値の {ndim} 次元配列が必要です: shape={arr.shape}")
    arr.flags.writeable = False
    return arr


# ----------------------------------------------------------------- 例外・記録の型
class TrajectoryError(RuntimeError):
    """【機能概要】: 軌道を作れないときの例外 (FR-06)。部分的な軌道は返さない。🟡

    reason: 理由 (REASON_* または IkError.reason)、waypoint: 失敗した経由点・区間名、
    detail: 目標点・最小すき間などの読み取り専用 mapping。
    """

    def __init__(
        self, reason: str, waypoint: str | None = None, detail: Mapping[str, object] | None = None
    ) -> None:
        self.reason = reason
        self.waypoint = waypoint
        self.detail: Mapping[str, object] = MappingProxyType(dict(detail or {}))
        where = f" (経由点: {waypoint})" if waypoint is not None else ""
        extra = f" {dict(self.detail)}" if self.detail else ""
        super().__init__(f"軌道を生成できません: {reason}{where}{extra}")


@dataclass(frozen=True)
class WaypointRecord:
    """【機能概要】: metadata["waypoints"] の要素。経由点の TCP 目標と IK の結果。🟡"""

    name: str
    target: np.ndarray
    q: np.ndarray
    pos_error: float
    pitch_deg: float
    pitch_candidate_deg: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "target", _readonly(self.target, 1))
        object.__setattr__(self, "q", _readonly(self.q, 1))


@dataclass(frozen=True)
class ClearanceViolation:
    """【機能概要】: すき間が最も小さいサンプルの位置・区間名・距離 [m]。🟡"""

    index: int
    label: str
    value: float


@dataclass(frozen=True)
class JointWaypoint:
    """【機能概要】: 関節空間の経由点。hold_s を与えた区間は q を保持する (q は直前の終点と同じ)。🟡"""

    name: str
    q: np.ndarray
    hold_s: float | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "q", _readonly(as_vector("q", self.q, N_JOINTS), 1))


@dataclass(frozen=True, eq=False)
class JointTrajectory:
    """【機能概要】: 制御周期ごとの関節目標の列。変更不可。🟡

    times (N,)、q (N, 6)、labels (長さ N)、segments (区間名の順)、metadata (読み取り専用)。
    q[0] は開始時の関節角、q[1:] が送る指令。
    """

    times: np.ndarray
    q: np.ndarray
    labels: tuple[str, ...]
    segments: tuple[str, ...]
    metadata: Mapping[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        times, q = _readonly(self.times, 1), _readonly(self.q, 2)
        labels = tuple(self.labels)
        if q.shape[1] != N_JOINTS or not (len(times) == len(q) == len(labels)):
            raise ValueError(
                f"times / q / labels の長さと q の列数 ({N_JOINTS}) を合わせてください: "
                f"times={times.shape}, q={q.shape}, labels={len(labels)}"
            )
        if len(times) < 2:
            raise ValueError("サンプルは 2 個以上必要です")
        if not np.all(np.diff(times) > 0):
            raise ValueError("times は厳密に単調増加が必要です")
        object.__setattr__(self, "times", times)
        object.__setattr__(self, "q", q)
        object.__setattr__(self, "labels", labels)
        object.__setattr__(self, "segments", tuple(self.segments))
        object.__setattr__(self, "metadata", MappingProxyType(dict(self.metadata)))


# ----------------------------------------------------------------- 時間配分・補間
def segment_steps(
    q_a: object,
    q_b: object,
    *,
    vmax: object,
    speed_scale: float,
    control_dt: float,
    safety_factor: float,
    min_segment_s: float,
) -> int:
    """【機能概要】: 区間 a -> b の刻み数 N を返す 🔵

    N = max(1, ceil(max_j |Δ_j| / (vmax_j x scale x dt x factor)), ceil(min_segment_s / dt))
    """
    delta = np.abs(as_vector("q_b", q_b, N_JOINTS) - as_vector("q_a", q_a, N_JOINTS))
    v = require_positive_vector("vmax", vmax, N_JOINTS)
    step = v * require_fraction("speed_scale", speed_scale)
    step = step * require_positive("control_dt", control_dt) * require_fraction(
        "safety_factor", safety_factor
    )
    by_speed = math.ceil(float(np.max(delta / step)))
    by_min = math.ceil(require_positive("min_segment_s", min_segment_s) / control_dt)
    return max(1, by_speed, by_min)


def hold_steps(hold_s: float, control_dt: float) -> int:
    """【機能概要】: 保持区間のサンプル数 round(hold_s / control_dt) を返す 🔵"""
    if hold_s < 0 or not math.isfinite(hold_s):
        raise ValueError(f"hold_s は 0 以上の有限値で指定してください: {hold_s!r}")
    return int(round(hold_s / require_positive("control_dt", control_dt)))


def interpolate_segment(q_a: object, q_b: object, n: int) -> np.ndarray:
    """【機能概要】: q_a -> q_b の線形補間 k = 1..n の (n, 6)。最後の行は q_b と完全一致 🔵"""
    a, b = as_vector("q_a", q_a, N_JOINTS), as_vector("q_b", q_b, N_JOINTS)
    if isinstance(n, bool) or not isinstance(n, (int, np.integer)) or n < 1:
        raise ValueError(f"n は 1 以上の整数で指定してください: {n!r}")
    k = np.arange(1, int(n) + 1, dtype=np.float64)[:, None]
    rows = a + (k / n) * (b - a)
    rows[-1] = b  # 【端点一致】: 再計算せず終点そのものを使う (丸め誤差を残さない)
    return rows


def _append_hold(waypoint: JointWaypoint, prev: np.ndarray, dt: float) -> np.ndarray:
    """【機能概要】: 保持区間の行 (prev の繰り返し) を返す。q が prev と違えば ValueError 🔴"""
    if not np.array_equal(waypoint.q, prev):
        raise ValueError(f"保持区間 {waypoint.name} の q は直前の終点と同じにしてください")
    return np.tile(prev, (hold_steps(float(waypoint.hold_s), dt), 1))


def build_joint_trajectory(
    q_start: object,
    waypoints: Sequence[JointWaypoint],
    *,
    vmax: object,
    speed_scale: float,
    control_dt: float,
    safety_factor: float,
    min_segment_s: float,
    metadata: Mapping[str, object] | None = None,
) -> JointTrajectory:
    """【機能概要】: 開始値と経由点から、制御周期ごとの JointTrajectory を作る 🔵

    segment_steps はこのモジュールの大域名から引く (テストで差し替えられる)。
    """
    prev = as_vector("q_start", q_start, N_JOINTS)
    rows, labels, segments = [prev.copy()[None, :]], [START_LABEL], [START_LABEL]
    for wp in waypoints:
        if wp.hold_s is not None:
            block = _append_hold(wp, prev, control_dt)
        else:
            n = segment_steps(
                prev, wp.q, vmax=vmax, speed_scale=speed_scale, control_dt=control_dt,
                safety_factor=safety_factor, min_segment_s=min_segment_s,
            )  # fmt: skip
            block = interpolate_segment(prev, wp.q, n)
            prev = np.array(wp.q)
        if len(block) == 0:  # 保持 0 秒の区間は残さない (O5)
            continue
        rows.append(block)
        labels += [wp.name] * len(block)
        segments.append(wp.name)
    q = np.vstack(rows)
    times = np.arange(len(q), dtype=np.float64) * control_dt
    meta = {**(metadata or {}), "duration_s": float(times[-1])}
    return JointTrajectory(times, q, tuple(labels), tuple(segments), meta)


# ----------------------------------------------------------------- 検査
def check_command_sequence(
    q: object,
    labels: Sequence[str],
    *,
    lower: object,
    upper: object,
    vmax: object,
    speed_scale: float,
    control_dt: float,
) -> None:
    """【機能概要】: 指令列が速度上限 (q[0]->q[1] を含む) と可動域 (q[1:]) を守るか確かめる 🔵

    違反は TrajectoryError("limit_violation", waypoint=違反した行のラベル)。≤ は合格。
    """
    arr = np.asarray(q, dtype=np.float64)
    budget = (
        require_positive_vector("vmax", vmax, N_JOINTS)
        * require_fraction("speed_scale", speed_scale)
        * require_positive("control_dt", control_dt)
    )
    over_speed = np.abs(np.diff(arr, axis=0)) > budget
    out_range = (arr[1:] < np.asarray(lower)) | (arr[1:] > np.asarray(upper))
    for kind, flags in (("speed", over_speed), ("range", out_range)):
        bad = np.flatnonzero(flags.any(axis=1))
        if bad.size:
            row = int(bad[0]) + 1
            raise TrajectoryError(
                REASON_LIMIT_VIOLATION, labels[row], {"kind": kind, "index": row}
            )


def find_clearance_violation(
    distances: object, labels: Sequence[str], threshold: float
) -> ClearanceViolation | None:
    """【機能概要】: 距離の列から、最小のサンプルがしきい値未満ならその位置・区間名・値を返す 🟡

    ちょうどしきい値は合格 (None)。
    """
    d = np.asarray(distances, dtype=np.float64)
    index = int(np.argmin(d))
    if d[index] < threshold:
        return ClearanceViolation(index=index, label=labels[index], value=float(d[index]))
    return None

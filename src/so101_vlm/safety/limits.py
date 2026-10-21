"""安全フィルタ: 可動域・速度・作業範囲・状態途絶 (TASK-0010)。

すべて純粋関数 (入力を変更しない、乱数・時計・グローバル状態を使わない)。
単位は rad、rad/s、s、m。関節順は JOINT_NAMES。
作業範囲と状態の新鮮さの判定は workspace.py にあり、ここから再エクスポートする。
"""

from dataclasses import dataclass

import mujoco
import numpy as np

from so101_vlm.robot.base import JOINT_NAMES, N_JOINTS
from so101_vlm.safety._validation import (
    as_vector,
    require_fraction,
    require_positive,
    require_positive_vector,
)
from so101_vlm.safety.config import SafetyConfig
from so101_vlm.safety.workspace import check_state_fresh, is_in_workspace

# 【理由文字列】: SafetyResult.reason の値 (ik_types.py の REASON_* と同じ方式) 🔵
REASON_STALE_STATE = "stale_state"
REASON_OUT_OF_WORKSPACE = "target_out_of_workspace"

_N_AXES = 3
# 【定数定義】: 丸め補正の最大回数 (通常 1 回で収まる) 🟡
_MAX_ULP_ADJUST = 4


# ----------------------------------------------------------------- 可動域
def _validated_limits(lower: object, upper: object) -> tuple[np.ndarray, np.ndarray]:
    """【機能概要】: lower / upper を検証し、各関節で lower < upper を要求する 🟡"""
    lo = as_vector("lower", lower, N_JOINTS)
    up = as_vector("upper", upper, N_JOINTS)
    if not np.all(lo < up):
        raise ValueError("lower は upper より小さい値で指定してください (全関節)")
    return lo, up


def clip_to_limits(q: object, lower: object, upper: object) -> np.ndarray:
    """【機能概要】: 各関節を [lower, upper] に丸めた新しい float64 配列を返す 🔵

    【実装方針】: 境界ちょうどは変えない (np.clip)。
    【テスト対応】: TC-CL-01〜13
    """
    q_arr = as_vector("q", q, N_JOINTS)
    lo, up = _validated_limits(lower, upper)
    return np.clip(q_arr, lo, up)


# ----------------------------------------------------------------- 速度
def _limit_step_checked(
    q_current: object, q_target: object, vmax: object, scale: object, dt: object
) -> tuple[np.ndarray, bool]:
    """【機能概要】: limit_step の本体。(結果, 制限したか) を返す 🔵

    【実装方針】: 1 周期の許容変化量 m = vmax x scale x dt。|Δ_i| > m_i の関節があれば、
    比 m_i/|Δ_i| の最小値 α で Δ 全体を縮める (方向を保つ)。制限しなければ q_target のコピー。
    """
    current = as_vector("q_current", q_current, N_JOINTS)
    target = as_vector("q_target", q_target, N_JOINTS)
    v = require_positive_vector("vmax", vmax, N_JOINTS)
    budget = v * require_fraction("scale", scale) * require_positive("dt", dt)
    delta = target - current
    magnitude = np.abs(delta)
    over = magnitude > budget  # 【境界】: ちょうど m は制限しない
    if not np.any(over):
        return target, False
    alpha = float(np.min(budget[over] / magnitude[over]))
    # 【丸め対策】: 浮動小数の誤差で m をわずかに超えないよう、クリップして 1 ulp ずつ戻す
    return _within_budget(current, current + np.clip(alpha * delta, -budget, budget), budget), True


def _within_budget(current: np.ndarray, command: np.ndarray, budget: np.ndarray) -> np.ndarray:
    """【機能概要】: 再計算した |command - current| が budget 以下になるよう、必要な関節だけ補正する 🟡

    【改善内容】: 補正の理由と検証結果を明記した (動作は変更なし)。
    【設計方針】: 束縛関節 (比 m/|Δ| が最小の関節) では current + α·Δ の浮動小数の丸めにより、
        下流が np.abs(cmd - prev) で再計算した値が m を 1 ulp 超えることがある
        (例: 0.020000000000000004 > 0.02)。実測では制限した場合の約半数で起きる。
        α に (1 - eps) を掛ける方法は方向を 1e-16 ずらすうえ、丸めで再び超えうるため採らない。
        np.nextafter で command を current 側へ 1 ulp だけ戻す方法は、補正量が最小
        (方向余弦 >= 1 - 1e-15) で、再計算した差を直接確かめるため偽の合格が起きない。
    【検証】: 乱数 2 万件 (rad/s・scale・dt をばらつかせた) で、再計算した差の超過 0 件、
        方向余弦の最小 0.9999999999999996、必要だった補正は 1 回 (10 万件の別試行でも最大 1 回)。
        _MAX_ULP_ADJUST = 4 は余裕であり、通常は 1 回で収まる。
    """
    result = command.copy()
    for _ in range(_MAX_ULP_ADJUST):
        over = np.abs(result - current) > budget
        if not np.any(over):
            break
        result[over] = np.nextafter(result[over], current[over])
    return result


def limit_step(
    q_current: object, q_target: object, vmax: object, scale: object, dt: object
) -> np.ndarray:
    """【機能概要】: 1 周期の変化量を速度上限以下に制限した指令を返す (一様スケーリング) 🔵/🟡

    【テスト対応】: TC-ST-01〜17
    """
    return _limit_step_checked(q_current, q_target, vmax, scale, dt)[0]


# ----------------------------------------------------------------- モデル
def limits_from_model(model: mujoco.MjModel) -> tuple[np.ndarray, np.ndarray]:
    """【機能概要】: JOINT_NAMES 順の actuator_ctrlrange (lower, upper) をコピーして返す 🔵

    【実装方針】: policy/ik.py の _Arm と同じく名前で引く。無ければ名前入りの ValueError。
    """
    ids = []
    for name in JOINT_NAMES:
        act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if act_id < 0:
            raise ValueError(f"アクチュエータが見つかりません: {name}")
        ids.append(act_id)
    ctrlrange = model.actuator_ctrlrange
    return (
        np.array(ctrlrange[ids, 0], dtype=np.float64),
        np.array(ctrlrange[ids, 1], dtype=np.float64),
    )


# ----------------------------------------------------------------- 結果型
@dataclass(frozen=True)
class SafetyResult:
    """【機能概要】: apply_safety の結果。変更不可。🟡

    q_command: 送る関節目標 (6,)。生成時に float64 のコピーにして読み取り専用にする。
    優先順位は stopped > rejected > (clipped, rate_limited)。reason は stopped / rejected のときだけ値を持つ。
    """

    q_command: np.ndarray
    clipped: bool
    rate_limited: bool
    rejected: bool
    stopped: bool
    reason: str | None

    def __post_init__(self) -> None:
        q = np.array(self.q_command, dtype=np.float64, copy=True)
        q.flags.writeable = False
        object.__setattr__(self, "q_command", q)


def _hold(q_current: np.ndarray, *, stopped: bool, reason: str) -> SafetyResult:
    """【機能概要】: その場で保持する結果 (停止または拒否) を作る 🟡"""
    return SafetyResult(
        q_command=q_current,
        clipped=False,
        rate_limited=False,
        rejected=not stopped,
        stopped=stopped,
        reason=reason,
    )


def _filter_command(
    current: np.ndarray, target: np.ndarray, lo: np.ndarray, up: np.ndarray, config: SafetyConfig
) -> SafetyResult:
    """【機能概要】: 可動域クリップ → 速度制限 → 最終クリップを順に当てる (手順 5〜7) 🟡

    【改善内容】: lo / up は _validate_structure で検証済みの配列を受け取る (二重検証の除去)。
    """
    clipped_target = np.clip(target, lo, up)
    clipped = not np.array_equal(clipped_target, target)
    command, rate_limited = _limit_step_checked(
        current, clipped_target, config.vmax, config.speed_scale, config.control_dt
    )
    # 【最終クリップ】: q_current が可動域外だと速度制限後も外になりうる。NFR-02 を優先する
    final = np.clip(command, lo, up)
    clipped = clipped or not np.array_equal(final, command)
    return SafetyResult(final, clipped, rate_limited, False, False, None)


def _validate_structure(
    q_current: object, q_target: object, lower: object, upper: object,
    config: object, target_base: object,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray | None]:
    """【機能概要】: 手順 1。構造 (shape・dtype・型) を検証する。q_target / target_base の値は見ない 🟡

    【戻り値】: (current, target, lo, up, base)。lo / up は検証済みの可動域。
    """
    if not isinstance(config, SafetyConfig):
        raise ValueError(f"config は SafetyConfig で指定してください: {config!r}")
    current = as_vector("q_current", q_current, N_JOINTS)
    lo, up = _validated_limits(lower, upper)
    target = as_vector("q_target", q_target, N_JOINTS, finite=False)
    base = None if target_base is None else as_vector(
        "target_base", target_base, _N_AXES, finite=False
    )
    return current, target, lo, up, base


def _validate_values(target: np.ndarray, base: np.ndarray | None) -> None:
    """【機能概要】: 手順 3。q_target / target_base の NaN/inf を ValueError にする 🟡"""
    if not np.all(np.isfinite(target)):
        raise ValueError("q_target は有限の数値で指定してください (NaN/inf 不可)")
    if base is not None and not np.all(np.isfinite(base)):
        raise ValueError("target_base は有限の数値で指定してください (NaN/inf 不可)")


def apply_safety(
    q_current: object,
    q_target: object,
    *,
    lower: object,
    upper: object,
    last_state_t: object,
    now: object,
    config: SafetyConfig,
    target_base: object = None,
) -> SafetyResult:
    """【機能概要】: 送信直前の安全フィルタ。構造検証 → 停止 → 値検証 → 拒否 → 可動域 → 速度 → 最終クリップ 🟡

    【実装方針】: q_current は速度制限と保持の基準 (前回送った指令を推奨)。安全上の判定は
    例外にせずフラグで返し、ValueError は入力の誤り (型・shape・NaN・bool) だけに使う。
    【テスト対応】: TC-AS-01〜42
    """
    current, target, lo, up, base = _validate_structure(
        q_current, q_target, lower, upper, config, target_base
    )
    if not check_state_fresh(last_state_t, now, config.state_timeout):
        return _hold(current, stopped=True, reason=REASON_STALE_STATE)
    _validate_values(target, base)
    if base is not None and not is_in_workspace(base, config.workspace):
        return _hold(current, stopped=False, reason=REASON_OUT_OF_WORKSPACE)
    return _filter_command(current, target, lo, up, config)


__all__ = [
    "REASON_OUT_OF_WORKSPACE",
    "REASON_STALE_STATE",
    "SafetyResult",
    "apply_safety",
    "check_state_fresh",
    "clip_to_limits",
    "is_in_workspace",
    "limit_step",
    "limits_from_model",
]

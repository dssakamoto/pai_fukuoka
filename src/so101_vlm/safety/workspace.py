"""作業範囲の判定と状態の新鮮さの判定 (TASK-0010)。純粋関数。"""

import math
import numbers

import numpy as np

from so101_vlm.safety._validation import as_vector, require_positive
from so101_vlm.safety.config import WorkspaceBox

_N_AXES = 3


def is_in_workspace(p_base: object, box: WorkspaceBox) -> bool:
    """【機能概要】: baseframe 基準の点が作業範囲 (閉区間) の中にあれば True 🟡

    【実装方針】: 境界上は内側。NaN/inf・shape 違い・bool は ValueError (バグとして扱う)。
    【テスト対応】: TC-WS-01〜12
    """
    p = as_vector("p_base", p_base, _N_AXES)
    inside = np.all(p >= np.asarray(box.lower)) and np.all(p <= np.asarray(box.upper))
    return bool(inside)


def _time_or_none(name: str, value: object) -> float | None:
    """【機能概要】: 時刻を float にする。bool・非数値は ValueError、非有限は None (停止側) 🟡"""
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise ValueError(f"{name} は数値で指定してください: {name}={value!r}")
    number = float(value)
    return number if math.isfinite(number) else None


def check_state_fresh(last_update_t: object, now: object, timeout: object) -> bool:
    """【機能概要】: 状態の更新が timeout 以内なら True (ちょうど timeout は新鮮) 🟡

    【実装方針】: NaN/inf の時刻と時計の逆行は False (停止側に倒す)。timeout の誤りは ValueError。
    【テスト対応】: TC-FR-01〜10
    """
    limit = require_positive("timeout", timeout)
    last = _time_or_none("last_update_t", last_update_t)
    current = _time_or_none("now", now)
    if last is None or current is None:
        return False
    return 0.0 <= current - last <= limit


__all__ = ["check_state_fresh", "is_in_workspace"]

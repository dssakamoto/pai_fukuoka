"""設定値・引数の検証ヘルパ (TASK-0006)。

layout.py・eval_config.py・safety/config.py が同じ規則を使うため共有する。違反は ValueError (名前入り)。
🟡 要件定義書 2.1 / 2.4 の検証規則に基づく。
"""

import math
import numbers
from collections.abc import Mapping

import numpy as np

from so101_vlm.eval.scene_spec import SCENE_SPEC

# 【定数定義】: area と作業範囲の幅の比較許容差 (0.35-0.15 == 0.19999999999999998 対策) 🔵
WIDTH_TOLERANCE = 1e-9


def work_area_width() -> float:
    """【機能概要】: 作業範囲の幅 (x, y の小さい方) を SCENE_SPEC から求める 🔵"""
    x_lo, x_hi = SCENE_SPEC.work_area_x
    y_lo, y_hi = SCENE_SPEC.work_area_y
    return min(x_hi - x_lo, y_hi - y_lo)


def require_int(name: str, value: object, minimum: int) -> int:
    """【機能概要】: bool を除く整数 (numpy 整数可) で minimum 以上を要求し int で返す 🟡"""
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)):
        raise ValueError(f"{name} は整数で指定してください: {name}={value!r}")
    if value < minimum:
        raise ValueError(f"{name} は {minimum} 以上で指定してください: {name}={value!r}")
    return int(value)


def require_number(name: str, value: object) -> float:
    """【機能概要】: bool を除く有限の実数を float で返す 🟡"""
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise ValueError(f"{name} は数値で指定してください: {name}={value!r}")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} は有限の数値で指定してください: {name}={value!r}")
    return result


def require_area(name: str, value: object) -> float:
    """【機能概要】: 0 より大きく作業範囲の幅以下の area を要求する 🟡"""
    area = require_number(name, value)
    limit = work_area_width()
    if area <= 0 or area > limit + WIDTH_TOLERANCE:
        raise ValueError(f"{name} は 0 より大きく {limit:.3f} 以下で指定してください: {name}={value!r}")
    return area


def require_min_gap(name: str, value: object) -> float:
    """【機能概要】: 0 以上の有限の間隔を要求する 🟡"""
    gap = require_number(name, value)
    if gap < 0:
        raise ValueError(f"{name} は 0 以上で指定してください: {name}={value!r}")
    return gap


def require_exact_keys(name: str, value: object, allowed: frozenset[str]) -> Mapping:
    """【機能概要】: mapping であり、キーが allowed と過不足なく一致することを要求する 🟡

    【再利用性】: eval_config (placement) と safety.config (safety / workspace) の YAML 検証で共有。
    未知キーは「未知のキー」、欠落は「必須キーがありません」の ValueError (キー名入り)。
    """
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} は mapping で指定してください: {name}={value!r}")
    unknown = sorted(str(k) for k in value if k not in allowed)
    if unknown:
        raise ValueError(f"未知のキー: {', '.join(unknown)} ({name})")
    missing = sorted(allowed - set(value))
    if missing:
        raise ValueError(f"必須キーがありません: {', '.join(missing)} ({name})")
    return value

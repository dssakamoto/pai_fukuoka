"""安全フィルタ共通の入力検証 (TASK-0010)。

🟡 robot/base.py の validate_q_target と eval/validation.py の方針に合わせる。
違反は ValueError (引数名入り)。bool は数値として受け付けない。
"""

import numpy as np

from so101_vlm.eval.validation import require_number


def as_vector(
    name: str, value: object, size: int, *, finite: bool = True
) -> np.ndarray:
    """【機能概要】: shape (size,) の float64 コピーを返す。bool 配列・shape 違いは ValueError 🟡

    【実装方針】: finite=False のときは NaN/inf を許す (apply_safety の停止優先順のため)。
    """
    try:
        raw = np.asarray(value)
        if raw.dtype == np.bool_:
            raise ValueError(f"{name} に bool は使えません")
        arr = np.array(raw, dtype=np.float64, copy=True)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} は数値の配列で指定してください: {exc}") from exc
    if arr.shape != (size,):
        raise ValueError(f"{name} は shape ({size},) で指定してください: shape={arr.shape}")
    if finite and not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} は有限の数値で指定してください (NaN/inf 不可)")
    return arr


def require_positive(name: str, value: object) -> float:
    """【機能概要】: 0 より大きい有限の実数を float で返す 🟡"""
    number = require_number(name, value)
    if number <= 0:
        raise ValueError(f"{name} は 0 より大きい値で指定してください: {name}={value!r}")
    return number


def require_fraction(name: str, value: object) -> float:
    """【機能概要】: 0 より大きく 1 以下の実数を float で返す (速度倍率用) 🟡"""
    number = require_positive(name, value)
    if number > 1.0:
        raise ValueError(f"{name} は 1 以下で指定してください: {name}={value!r}")
    return number


def require_positive_vector(name: str, value: object, size: int) -> np.ndarray:
    """【機能概要】: 全要素が正の shape (size,) の float64 配列を返す 🟡"""
    arr = as_vector(name, value, size)
    if not np.all(arr > 0):
        raise ValueError(f"{name} の全要素は 0 より大きい値で指定してください")
    return arr


def to_float_tuple(name: str, value: object, size: int) -> tuple[float, ...]:
    """【機能概要】: 長さ size の数列を float の tuple にする (bool・非有限は ValueError) 🟡"""
    if isinstance(value, (str, bytes)) or not hasattr(value, "__iter__"):
        raise ValueError(f"{name} は長さ {size} の数列で指定してください: {name}={value!r}")
    items = tuple(value)  # type: ignore[call-overload]
    if len(items) != size:
        raise ValueError(f"{name} は長さ {size} で指定してください: 長さ={len(items)}")
    return tuple(require_number(f"{name}[{i}]", v) for i, v in enumerate(items))

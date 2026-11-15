"""設定値・引数の検証ヘルパ (TASK-0006)。

layout.py・eval_config.py が使う作業範囲依存の検証 (work_area_width / require_area) をここに残す。
SCENE_SPEC に依存しない require_* は中立側の so101_vlm.validation に移設し (SGT-001)、
互換のためここから再エクスポートする。違反は ValueError (名前入り)。
🟡 要件定義書 2.1 / 2.4 の検証規則に基づく。
"""

from so101_vlm.eval.scene_spec import SCENE_SPEC
from so101_vlm.validation import require_exact_keys, require_int, require_min_gap, require_number  # noqa: F401

__all__ = [
    "WIDTH_TOLERANCE",
    "require_area",
    "require_exact_keys",
    "require_int",
    "require_min_gap",
    "require_number",
    "work_area_width",
]

# 【定数定義】: area と作業範囲の幅の比較許容差 (0.35-0.15 == 0.19999999999999998 対策) 🔵
WIDTH_TOLERANCE = 1e-9


def work_area_width() -> float:
    """【機能概要】: 作業範囲の幅 (x, y の小さい方) を SCENE_SPEC から求める 🔵"""
    x_lo, x_hi = SCENE_SPEC.work_area_x
    y_lo, y_hi = SCENE_SPEC.work_area_y
    return min(x_hi - x_lo, y_hi - y_lo)


def require_area(name: str, value: object) -> float:
    """【機能概要】: 0 より大きく作業範囲の幅以下の area を要求する 🟡"""
    area = require_number(name, value)
    limit = work_area_width()
    if area <= 0 or area > limit + WIDTH_TOLERANCE:
        raise ValueError(f"{name} は 0 より大きく {limit:.3f} 以下で指定してください: {name}={value!r}")
    return area

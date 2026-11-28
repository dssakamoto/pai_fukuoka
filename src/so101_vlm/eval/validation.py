"""設定値・引数の検証ヘルパ (TASK-0006)。

layout.py・eval_config.py が使う、ブロックを置く領域 (BLOCK_AREA_X / BLOCK_AREA_Y) の定数と、それに依存する検証 (block_area_width / require_area)。
SCENE_SPEC に依存しない require_* は中立側の so101_vlm.validation に移設し (SGT-001)、
互換のためここから再エクスポートする。違反は ValueError (名前入り)。
🟡 要件定義書 2.1 / 2.4 の検証規則に基づく。
"""

from so101_vlm.validation import require_exact_keys, require_int, require_min_gap, require_number  # noqa: F401

__all__ = [
    "BLOCK_AREA_X",
    "BLOCK_AREA_Y",
    "WIDTH_TOLERANCE",
    "require_area",
    "require_exact_keys",
    "require_int",
    "require_min_gap",
    "require_number",
    "block_area_width",
]

# 【定数定義】: area と作業範囲の幅の比較許容差 (0.35-0.15 == 0.19999999999999998 対策) 🔵
WIDTH_TOLERANCE = 1e-9


# 【定数定義】: ブロックを置く領域 (20 cm 四方、ワールド座標 [m])。XML に無い評価の条件 (SGT-013 で SCENE_SPEC から移した)。
# 箱の位置はここに含まない (箱は Scenario と到達可能領域で決まる)。ブロックは領域で棄却しない (SGT-010 の G1 (b))。🟡
BLOCK_AREA_X = (0.15, 0.35)
BLOCK_AREA_Y = (-0.10, 0.10)


def block_area_width() -> float:
    """【機能概要】: ブロックを置く領域の幅 (x, y の小さい方) 🔵"""
    x_lo, x_hi = BLOCK_AREA_X
    y_lo, y_hi = BLOCK_AREA_Y
    return min(x_hi - x_lo, y_hi - y_lo)


def require_area(name: str, value: object) -> float:
    """【機能概要】: 0 より大きくブロックを置く領域の幅以下の area を要求する 🟡"""
    area = require_number(name, value)
    limit = block_area_width()
    if area <= 0 or area > limit + WIDTH_TOLERANCE:
        raise ValueError(f"{name} は 0 より大きく {limit:.3f} 以下で指定してください: {name}={value!r}")
    return area

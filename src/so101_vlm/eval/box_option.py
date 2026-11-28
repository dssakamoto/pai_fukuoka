"""スクリプトの --box-xy (箱の位置の指定) の検証と、実 API を呼ぶ実行の確認 (SGT-014)。

【機能概要】
- `parse_box_xy(values)`: `--box-xy X Y` を `Fixed(xy=(X, Y))` に変換する。箱の外周が到達可能領域に収まらない位置は、動作前に理由つきで
  ValueError (スクリプトは終了コード 2)。
- `confirm_api_run(...)`: 実 API (有料) を呼ぶ実行の前に、想定トークンと料金の目安を表示して確認を取る。oracle は API を呼ばないので確認しない。
  非対話の実行は、明示の --yes が無ければ拒否する (黙って課金しない)。

純粋に近い関数で、実際の入出力は引数で差し替えられる (テストで API を使わずに確かめる)。
"""

from __future__ import annotations

import sys
from collections.abc import Callable, Sequence

from so101_vlm.eval.geometry import SceneGeometry, get_geometry
from so101_vlm.eval.reachable import ReachableRegion, load_reachable_region
from so101_vlm.eval.scenario import Fixed, box_outline_fits
from so101_vlm.validation import require_int, require_number

# 🟡 1 試行あたりの目安 (README の実測: 640x480 の画像 + 指示で、入力 約 700 / 出力 約 100〜500 トークン)
TOKENS_IN_PER_TRIAL = 700
TOKENS_OUT_PER_TRIAL = (100, 500)
# 🟡 gpt-6-luna の実測 (README): 20 試行で約 $0.003。他のプロバイダは料金表で確認する
USD_PER_20_TRIALS_GPT_6_LUNA = 0.003


def parse_box_xy(
    values: Sequence[object] | None, region: ReachableRegion | None = None, geometry: SceneGeometry | None = None
) -> Fixed | None:
    """【機能概要】: --box-xy の 2 つの値を Fixed にする。None なら None (箱は既定の位置) 🔵

    2 つの有限の数でない、または箱の外周が到達可能領域に収まらない (計画できない位置) は ValueError (理由と目安の範囲つき)。
    """
    if values is None:
        return None
    if len(values) != 2:
        raise ValueError(f"--box-xy は X Y の 2 つの数で指定してください: {list(values)!r}")
    x = require_number("--box-xy の X", values[0])
    y = require_number("--box-xy の Y", values[1])
    region = load_reachable_region() if region is None else region
    geometry = get_geometry() if geometry is None else geometry
    if not box_outline_fits(region, (x, y), geometry):
        (xl, xh), (yl, yh) = region.bbox()
        dx, dy = geometry.box_default_xy
        raise ValueError(
            f"箱 ({x:.3f}, {y:.3f}) は、箱の外周が到達可能領域に収まらず、軌道を計画できません。"
            f"領域の範囲は x {xl:.3f}〜{xh:.3f}, y {yl:.3f}〜{yh:.3f} (外周の半幅ぶん内側)。既定の箱の位置は ({dx:.2f}, {dy:.2f})。"
            "箱を置ける目安は docs/notes/box-sweep.md"
        )
    return Fixed(xy=(x, y))


def cost_estimate(provider: str, n_trials: int) -> str:
    """【機能概要】: 想定トークンと料金の目安の文章 (複数行) 🟡"""
    n = require_int("n_trials", n_trials, 1)
    lines = [
        f"実 API ({provider}) を {n} 回呼びます (有料)。",
        f"  想定トークン: 入力 約 {TOKENS_IN_PER_TRIAL * n:,} / 出力 約 {TOKENS_OUT_PER_TRIAL[0] * n:,}〜{TOKENS_OUT_PER_TRIAL[1] * n:,}",
    ]
    if provider == "openai":
        lines.append(f"  料金の目安: 約 ${USD_PER_20_TRIALS_GPT_6_LUNA * n / 20:.4f} (gpt-6-luna の実測。20 試行で約 $0.003)")
    else:
        lines.append("  料金の目安: プロバイダの料金表を確認してください (この値は gpt-6-luna の実測のみ)")
    lines.append("  VLM の応答はキャッシュされ、同じ画像・指示では API を呼び直しません (configs/vlm.yaml)")
    return "\n".join(lines)


def confirm_api_run(
    provider: str,
    n_trials: int,
    *,
    yes: bool = False,
    input_fn: Callable[[str], str] = input,
    is_interactive: Callable[[], bool] = lambda: sys.stdin.isatty(),
    out: Callable[[str], None] = print,
) -> bool:
    """【機能概要】: 実 API を呼ぶ実行の確認。続けてよければ True、取りやめなら False 🔵

    oracle (API を呼ばない) は表示も確認もせず True。yes なら目安を表示して True。
    yes でなく非対話 (端末でない) なら ValueError (黙って課金しない)。対話なら y / yes だけを肯定とする。
    """
    if provider == "oracle":
        return True
    out(cost_estimate(provider, n_trials))
    if yes:
        return True
    if not is_interactive():
        raise ValueError("実 API を呼ぶ実行は確認が必要です。対話端末で実行するか、内容を確認のうえ --yes を付けてください")
    return input_fn("続けますか? [y/N] ").strip().lower() in {"y", "yes"}

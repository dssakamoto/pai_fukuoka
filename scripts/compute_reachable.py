"""到達可能領域を IK の格子で計算して configs/reachable.yaml に保存する (SGT-009)。一度だけ実行する。

  uv run python scripts/compute_reachable.py                  # 既定の格子 (x 0.05〜0.50, y -0.35〜0.35, 刻み 1 cm)
  uv run python scripts/compute_reachable.py --step 0.005     # 細かく
  uv run python scripts/compute_reachable.py --out /tmp/r.yaml   # 別の場所へ (configs を汚さない)

xy 格子の各点を「箱を置く位置として、軌道を計画できるか」で判定する。IkPolicy と同じ条件 (安全フィルタの workspace に入る、
solve_ik、tcp_offset、既定のピッチ候補、seed は 0 姿勢) で、箱の上の 2 つの経由点が全部解ける点を到達可能 (=1) とする:
  pre_place (運搬の高さ。pre_grasp と同じ z = ブロックの静止高さ + approach_height) /
  place (箱の底の上面 + place_height)。 (retreat は pre_place と同じ目標)
ブロックを掴む位置 (grasp / pre_grasp) は含めない: ロボットの付け根の近くには掴めない領域があり (実測)、
それはブロックの配置の問題で、箱を置く位置の問題ではない (SGT-010: G1 は (b)。箱だけを領域で棄却する)。
workspace を条件に入れるので、領域は必ず workspace の内側に収まる (workspace は安全の設定なので、このスクリプトは広げない)。
結果は 0/1 の文字列行で保存する (差分が読める)。実行後に、領域の範囲と、configs/motion.yaml の workspace に反映すべき値を表示する
(motion.yaml は自動では書き換えない。値を見て手で更新する。tests/eval/test_reachable.py が食い違いを検知する)。
終了コード: 0 = 保存した、2 = 引数の誤り・到達できる点が無い。
"""

import argparse
import sys
from pathlib import Path

import mujoco
import numpy as np
import yaml

from so101_vlm.eval.geometry import get_geometry
from so101_vlm.eval.reachable import REACHABLE_PATH, ReachableRegion
from so101_vlm.policy.ik import solve_ik
from so101_vlm.policy.ik_types import IkError
from so101_vlm.policy.trajectory_config import load_trajectory_config
from so101_vlm.robot.model_paths import PICKPLACE_SCENE_PATH
from so101_vlm.safety.config import load_safety_config
from so101_vlm.safety.workspace import is_in_workspace

HEADER = (
    "# 到達可能領域 (SGT-009)。scripts/compute_reachable.py が生成する。手で編集しない。\n"
    "# 行 = x、列 = y の 0/1。格子点 (x0 + i*step, y0 + j*step) に箱を置く軌道を計画できる (workspace の内側で pre_place / place の IK が解ける) =1。\n"
)


def _target_heights() -> dict[str, float]:
    cfg = load_trajectory_config()
    g = get_geometry()
    pre_grasp_z = g.block_rest_z + cfg.approach_height  # IkPolicy: pre_place は pre_grasp と同じ高さ
    return {
        "pre_place": max(cfg.min_carry_z, pre_grasp_z),
        "place": g.box_floor_top_z + cfg.place_height,
    }


def reachable_at(model: mujoco.MjModel, x: float, y: float, heights: dict[str, float], tcp_offset) -> bool:
    """【機能概要】: (x, y) で全ての高さの目標が解けるか。最初に失敗した時点で打ち切る 🟡"""
    seed = np.zeros(6)
    workspace = load_safety_config().workspace
    for z in heights.values():
        if not is_in_workspace(np.array((x, y, z)), workspace):
            return False  # 安全フィルタが拒否する位置 (IK の前に TrajectoryError target_out_of_workspace)
        try:
            solve_ik(model, (x, y, z), seed, tcp_offset=tcp_offset)
        except IkError:
            return False
    return True


def compute(x_range: tuple[float, float], y_range: tuple[float, float], step: float) -> ReachableRegion:
    model = mujoco.MjModel.from_xml_path(str(PICKPLACE_SCENE_PATH))
    heights = _target_heights()
    tcp_offset = load_trajectory_config().tcp_offset
    nx = int(round((x_range[1] - x_range[0]) / step)) + 1
    ny = int(round((y_range[1] - y_range[0]) / step)) + 1
    rows = []
    for i in range(nx):
        x = x_range[0] + i * step
        rows.append(tuple(reachable_at(model, x, y_range[0] + j * step, heights, tcp_offset) for j in range(ny)))
        print(f"x={x:.3f}: {''.join('1' if v else '0' for v in rows[-1])}", flush=True)
    return ReachableRegion(x_range[0], y_range[0], step, tuple(rows))


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--x-range", type=float, nargs=2, default=(0.05, 0.50), metavar=("LO", "HI"))
    p.add_argument("--y-range", type=float, nargs=2, default=(-0.35, 0.35), metavar=("LO", "HI"))
    p.add_argument("--step", type=float, default=0.01)
    p.add_argument("--out", type=Path, default=REACHABLE_PATH)
    args = p.parse_args(argv)
    if args.step <= 0 or args.x_range[0] >= args.x_range[1] or args.y_range[0] >= args.y_range[1]:
        print("引数が不正です: step は正、範囲は LO < HI で指定してください", file=sys.stderr)
        return 2
    try:
        region = compute(tuple(args.x_range), tuple(args.y_range), args.step)
    except ValueError as exc:  # 到達できる点が 1 つも無い (ReachableRegion の検証)
        print(f"到達できる点がありません: {exc}", file=sys.stderr)
        return 2
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(HEADER + yaml.safe_dump(region.to_yaml_dict(), sort_keys=False, allow_unicode=True), encoding="utf-8")
    (xl, xh), (yl, yh) = region.bbox()
    print(f"\n保存しました: {args.out}")
    print(f"到達可能な範囲: x {xl:.3f}〜{xh:.3f}, y {yl:.3f}〜{yh:.3f}")
    print(
        "configs/motion.yaml の workspace は、この範囲を含むようにしてください (手で更新): "
        f"lower の x, y は {xl:.3f}, {yl:.3f} 以下、upper の x, y は {xh:.3f}, {yh:.3f} 以上"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())

"""到達可能領域を、実際の IkPolicy で格子ごとに計画して configs/reachable.yaml に保存する (SGT-009)。一度だけ実行する。

  uv run python scripts/compute_reachable.py                  # 既定の格子 (x 0.05〜0.50, y -0.35〜0.35, 刻み 1 cm)
  uv run python scripts/compute_reachable.py --step 0.005     # 細かく
  uv run python scripts/compute_reachable.py --out /tmp/r.yaml   # 別の場所へ (configs を汚さない)

xy 格子の各点を「その位置に箱を置いて、1 試行の軌道を計画できるか」で判定する。箱の body_pos を格子点に動かし、
固定の基準のブロック (0.25, 0.0) を掴んで箱に入れる軌道を、実際の IkPolicy (安全フィルタの workspace・IK・障害物とのすき間・
経路の作業範囲) で計画する。計画できたら到達可能 (=1)。
- 掴む側 (pre_grasp / grasp / lift) は基準のブロックで固定する。最初に、箱を既定の位置 (基準のブロックから離れている) に置いて
  計画できることを確かめ、できないならスクリプトを止める (基準が悪い)。格子の途中で、箱が基準のブロックに近すぎて計画できない点は、
  単に到達不可 (=0) とする。ブロックの配置との関係は領域に含めない (SGT-010: G1 は (b)。実測で、付け根の近くには掴めない領域がある)。
- workspace・すき間も計画の条件に入るので、領域は必ず workspace の内側に収まる (workspace は安全の設定なので、このスクリプトは広げない)。
- 計画の判定は、実際の試行と同じコード (IkPolicy.act) なので、領域と実際の計画が食い違わない。

実行後に、領域の範囲を表示する。tests/eval/test_reachable.py が、領域と configs/motion.yaml の workspace の食い違いを検知する。
終了コード: 0 = 保存した、2 = 引数の誤り・到達できる点が無い・基準のブロックが計画できない。
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml

from so101_vlm.eval.geometry import BOX_BODY_NAME, get_geometry
from so101_vlm.eval.reachable import REACHABLE_PATH, ReachableRegion
from so101_vlm.policy.base import Goal, Observation
from so101_vlm.policy.ik_policy import IkPolicy
from so101_vlm.policy.trajectory import TrajectoryError
from so101_vlm.policy.trajectory_config import load_trajectory_config
from so101_vlm.robot.model_paths import PICKPLACE_SCENE_PATH
from so101_vlm.robot.sim import SimRobot
from so101_vlm.safety.config import load_safety_config

HEADER = (
    "# 到達可能領域 (SGT-009)。scripts/compute_reachable.py が生成する。手で編集しない。\n"
    "# 行 = x、列 = y の 0/1。格子点 (x0 + i*step, y0 + j*step) に箱を置く軌道を、実際の IkPolicy で計画できる =1。\n"
)
REFERENCE_PICK_XY = (0.25, 0.0)  # 掴む側を固定する基準のブロック (作業範囲の中央)


class ReferencePickError(RuntimeError):
    """基準のブロックを、箱が既定の位置のときでも計画できない (基準を見直す)。"""


def plannable(policy_factory, obs: Observation, pick: np.ndarray, x: float, y: float, floor_top_z: float) -> bool:
    """【機能概要】: 箱の中心を (x, y) にして、軌道を計画できるか。計画できなければ False (理由は問わない) 🟡"""
    goal = Goal(pick_target_base=pick, place_target_base=np.array((x, y, floor_top_z)))
    try:
        policy_factory().act(obs, goal)
    except TrajectoryError:
        return False
    return True


def compute(x_range: tuple[float, float], y_range: tuple[float, float], step: float) -> ReachableRegion:
    g = get_geometry()
    traj_cfg, safety_cfg = load_trajectory_config(), load_safety_config()
    pick = np.array((*REFERENCE_PICK_XY, g.block_rest_z))
    nx = int(round((x_range[1] - x_range[0]) / step)) + 1
    ny = int(round((y_range[1] - y_range[0]) / step)) + 1
    rows = []
    with SimRobot(PICKPLACE_SCENE_PATH) as robot:
        model, _ = robot.mujoco_handles()
        obs = Observation.from_state(robot.get_state())
        box = model.body(BOX_BODY_NAME).id
        model.body_pos[box][:2] = g.box_default_xy  # 基準の確認: 箱が既定の位置で、基準のブロックを計画できること
        if not plannable(lambda: IkPolicy(model, config=traj_cfg, safety=safety_cfg), obs, pick, *g.box_default_xy, g.box_floor_top_z):
            raise ReferencePickError(f"基準のブロック {REFERENCE_PICK_XY} が、箱が既定の位置 {g.box_default_xy} でも計画できません")
        for i in range(nx):
            x = x_range[0] + i * step
            row = []
            for j in range(ny):
                y = y_range[0] + j * step
                model.body_pos[box][:2] = (x, y)  # 箱を動かす (IkPolicy は model のすき間を見る。試行間で漏れないよう、このプロセス限り)
                row.append(
                    plannable(
                        lambda: IkPolicy(model, config=traj_cfg, safety=safety_cfg), obs, pick, x, y, g.box_floor_top_z
                    )
                )
            rows.append(tuple(row))
            print(f"x={x:.3f}: {''.join('1' if v else '0' for v in row)}", flush=True)
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
    except ReferencePickError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    except ValueError as exc:  # 到達できる点が 1 つも無い (ReachableRegion の検証)
        print(f"到達できる点がありません: {exc}", file=sys.stderr)
        return 2
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(HEADER + yaml.safe_dump(region.to_yaml_dict(), sort_keys=False, allow_unicode=True), encoding="utf-8")
    (xl, xh), (yl, yh) = region.bbox()
    print(f"\n保存しました: {args.out}")
    print(f"到達可能な範囲: x {xl:.3f}〜{xh:.3f}, y {yl:.3f}〜{yh:.3f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

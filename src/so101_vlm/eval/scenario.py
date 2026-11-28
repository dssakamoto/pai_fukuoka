"""試行ごとの配置 (Scenario) と、その生成・適用 (SGT-005, SGT-006)。

【機能概要】ブロックと箱の位置を「定数」でなく試行ごとの状態として表す。
- `sample_scenario(seed, spec)`: シードから Scenario を作る。`box=Fixed` の既定では、ブロック配置が既存の
  `sample_layout` と同じ (乱数を同じ順序で消費する。箱は最後に引き、Fixed は乱数を消費しない = 既存の seed が変わらない)。
- `apply_scenario(model, data, scenario)`: シミュレーションに配置を反映する (ブロック + 箱の body_pos)。

純粋関数 (sample_scenario) は入力を変更せず、乱数は seed からのみ作る。Scenario は tuple のみを持つ (ndarray を持たない)。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import mujoco
import numpy as np

from so101_vlm.eval.eval_config import PlacementConfig
from so101_vlm.eval.geometry import BOX_BODY_NAME, PlaceRegion, SceneGeometry, get_geometry
from so101_vlm.eval.layout import _check_feasible, _sample_points, place_blocks
from so101_vlm.eval.reachable import ReachableRegion, load_reachable_region
from so101_vlm.eval.validation import require_area, require_int, require_min_gap
from so101_vlm.validation import require_number

OUTLINE_EPS_M = 1e-5  # 外周の判定の余裕 [m] (格子の中間点での判定を外側に固定する。座標の丸め 5e-7 より大きい)


@dataclass(frozen=True)
class Fixed:
    """【機能概要】: 箱を固定位置に置く。xy が None なら XML の既定位置 🔵"""

    xy: tuple[float, float] | None = None

    def __post_init__(self) -> None:
        if self.xy is not None:
            if not isinstance(self.xy, (tuple, list)) or len(self.xy) != 2:
                raise ValueError(f"xy は 2 要素で指定してください: xy={self.xy!r}")
            object.__setattr__(self, "xy", (require_number("xy[0]", self.xy[0]), require_number("xy[1]", self.xy[1])))


@dataclass(frozen=True)
class UniformXY:
    """【機能概要】: 箱の位置を、到達可能領域の中から一様に引く (SGT-010) 🔵

    region: 到達可能領域 (ReachableRegion)。None なら sample_scenario の reachable、それも None なら保存済みの
    configs/reachable.yaml を使う。margin: 箱の外周に足す余裕 [m] (0 以上)。
    block_gap: 箱の外周とブロックの縁の最小間隔 [m] (0 以上)。既定 0.03 は、既定の箱 (0.20, 0.19) と、作業範囲の端 (y 0.10)
    のブロックの縁との間隔 (0.19 − 0.045 − 0.115) と同じ。ブロック間の min_gap (0.05) をそのまま使うと、既定の配置自体が違反する。
    """

    region: ReachableRegion | None = None
    margin: float = 0.0
    block_gap: float = 0.03

    def __post_init__(self) -> None:
        if self.region is not None and not isinstance(self.region, ReachableRegion):
            raise ValueError(f"region は ReachableRegion で指定してください: {self.region!r}")
        margin = require_number("margin", self.margin)
        if margin < 0:
            raise ValueError(f"margin は 0 以上で指定してください: margin={self.margin!r}")
        object.__setattr__(self, "margin", margin)
        gap = require_number("block_gap", self.block_gap)
        if gap < 0:
            raise ValueError(f"block_gap は 0 以上で指定してください: block_gap={self.block_gap!r}")
        object.__setattr__(self, "block_gap", gap)


BoxSpec = Fixed | UniformXY


@dataclass(frozen=True)
class ScenarioSpec:
    """【機能概要】: Scenario の生成条件。既定は configs/eval.yaml の placement と一致する 🔵"""

    n_blocks: int = 3
    area: float = 0.20
    min_gap: float = 0.05
    max_attempts: int = 10_000
    box: BoxSpec = field(default_factory=Fixed)


@dataclass(frozen=True)
class Scenario:
    """【機能概要】: 1 試行の配置。ブロック中心 (x, y) と箱の中心 (x, y)。ワールド座標 [m] 🔵"""

    seed: int
    blocks: tuple[tuple[float, float], ...]
    box_xy: tuple[float, float]

    def place_region(self, geometry: SceneGeometry | None = None) -> PlaceRegion:
        """【機能概要】: この試行の置き先の領域 (箱の中心 + 形状) 🔵"""
        g = get_geometry() if geometry is None else geometry
        return g.place_region(self.box_xy)


def sample_scenario(
    seed: int,
    spec: ScenarioSpec = ScenarioSpec(),
    *,
    geometry: SceneGeometry | None = None,
    reachable: ReachableRegion | None = None,
) -> Scenario:
    """【機能概要】: シードから Scenario を作る 🔵

    不正な seed・spec は、乱数を作る前に ValueError。ブロック → 箱の順に、同じ rng から引く。
    UniformXY は、箱の外周が到達可能領域に収まり、ブロックと block_gap 以上離れる位置を棄却サンプリングで引く
    (max_attempts を超えたら ValueError。FR-104)。ブロックの列は Fixed と同じ (乱数の消費順序を変えない。G1 (b))。
    """
    seed = require_int("seed", seed, 0)
    if not isinstance(spec, ScenarioSpec):
        raise ValueError(f"spec は ScenarioSpec で指定してください: {spec!r}")
    n = require_int("n_blocks", spec.n_blocks, 1)
    area = require_area("area", spec.area)
    min_gap = require_min_gap("min_gap", spec.min_gap)
    max_attempts = require_int("max_attempts", spec.max_attempts, 1)
    _check_feasible(n, area, min_gap)
    if not isinstance(spec.box, (Fixed, UniformXY)):
        raise ValueError(f"box は Fixed か UniformXY で指定してください: {spec.box!r}")
    g = get_geometry() if geometry is None else geometry

    rng = np.random.default_rng(seed)
    blocks = _sample_points(rng, n, area, min_gap, max_attempts)
    if isinstance(spec.box, Fixed):  # 乱数を消費しない (既存の seed と同じ配置になる)
        box_xy = g.box_default_xy if spec.box.xy is None else spec.box.xy
    else:
        box_xy = _sample_box(rng, spec.box, blocks, g, max_attempts, reachable)
    return Scenario(seed=seed, blocks=blocks, box_xy=(float(box_xy[0]), float(box_xy[1])))


def _outline_points(cx: float, cy: float, half: float, step: float) -> list[tuple[float, float]]:
    """【機能概要】: 中心 (cx, cy)・半幅 half の正方形の外周を、step 以下の間隔で走査する点 + 中心 🟡"""
    n = max(1, math.ceil(2 * half / step))
    points = [(cx, cy)]
    for k in range(n + 1):
        t = -half + 2 * half * k / n
        points += [(cx + t, cy - half), (cx + t, cy + half), (cx - half, cy + t), (cx + half, cy + t)]
    return points


def box_outline_fits(
    region: ReachableRegion, center_xy: tuple[float, float], geometry: SceneGeometry, margin: float = 0.0
) -> bool:
    """【機能概要】: 箱 (中心 center_xy) の外周 (内寸 + 壁 + margin) が、全部到達可能領域の内側か 🟡 (UniformXY と掃引が共有)

    外周に OUTLINE_EPS_M を足す: 外周 (半幅 0.045) は 1 cm 格子の中間点に当たり、丸めの向きが浮動小数点の誤差 (座標を丸めただけ) で
    割れる。境界を常に外側の格子点で判定する (保守的)。
    """
    outer = geometry.box_inner_half_size + geometry.box_wall_thickness + margin + OUTLINE_EPS_M
    return all(region.contains_xy(px, py) for px, py in _outline_points(center_xy[0], center_xy[1], outer, region.step))


def box_clear_of_blocks(
    blocks: tuple[tuple[float, float], ...],
    center_xy: tuple[float, float],
    geometry: SceneGeometry,
    block_gap: float,
    margin: float = 0.0,
) -> bool:
    """【機能概要】: 箱の外周とブロックの縁の間隔が、どのブロックとも block_gap 以上か (正方形どうしの最大座標差で測る = 保守的) 🟡"""
    outer = geometry.box_inner_half_size + geometry.box_wall_thickness + margin
    clearance = outer + geometry.block_half_size + block_gap
    return all(max(abs(center_xy[0] - bx), abs(center_xy[1] - by)) >= clearance for bx, by in blocks)


def _sample_box(
    rng: np.random.Generator,
    box: UniformXY,
    blocks: tuple[tuple[float, float], ...],
    g: SceneGeometry,
    max_attempts: int,
    reachable: ReachableRegion | None,
) -> tuple[float, float]:
    """【機能概要】: 箱の中心を、ブロックの後に同じ rng から引く (候補ごとに x, y の順) 🔵

    棄却条件 (🟡): (a) 箱の外周 (内寸 + 壁 + margin) が到達可能領域の内側、(b) 箱の外周とブロックの縁の間隔が block_gap 以上
    (正方形どうしの最大座標差で測る = 保守的)。
    ブロックは領域で棄却しない (G1 は (b): 実測で、ロボットの付け根の近くにはブロックを掴めない領域があり、領域は「箱を置く位置」の
    ものだから)。ブロックの列は Fixed と同じまま (乱数の消費順序を変えない)。
    """
    region = box.region or reachable or load_reachable_region()
    outer = g.box_inner_half_size + g.box_wall_thickness + box.margin
    (xl, xh), (yl, yh) = region.bbox()
    x_lo, x_hi, y_lo, y_hi = xl + outer, xh - outer, yl + outer, yh - outer
    if x_lo > x_hi or y_lo > y_hi:
        raise ValueError(
            f"到達可能領域 (x {xl:.3f}〜{xh:.3f}, y {yl:.3f}〜{yh:.3f}) が、箱の外周 (半幅 {outer:.3f}) を置くには狭すぎます"
        )
    for _ in range(max_attempts):
        cx, cy = float(rng.uniform(x_lo, x_hi)), float(rng.uniform(y_lo, y_hi))
        if not box_clear_of_blocks(blocks, (cx, cy), g, box.block_gap, box.margin):
            continue
        if box_outline_fits(region, (cx, cy), g, box.margin):
            return (cx, cy)
    raise ValueError(
        f"箱の位置が見つかりません: 試行 {max_attempts} 回で、到達可能領域に収まり、ブロックと {box.block_gap} m 以上離れる位置が出ませんでした"
    )


def scenario_for(seed: int, placement: PlacementConfig, box: BoxSpec | None = None) -> Scenario:
    """【機能概要】: 評価設定 (configs/eval.yaml の placement) の配置条件で Scenario を作る。box を省くと Fixed (従来どおり) 🔵

    run_trial / run_fixed_pick / runner / nodes が共有する (配置条件の読み替えを 1 か所にする)。
    """
    spec = ScenarioSpec(
        n_blocks=placement.n_blocks, area=placement.area, min_gap=placement.min_gap,
        max_attempts=placement.max_attempts, box=Fixed() if box is None else box,
    )  # fmt: skip
    return sample_scenario(seed, spec)


def apply_scenario(model: mujoco.MjModel, data: mujoco.MjData, scenario: Scenario) -> None:
    """【機能概要】: Scenario をモデルに書き込む (ブロックの位置 + 箱の body_pos の xy) して mj_forward する 🟡

    箱は IkPolicy の構築と撮影の前に動かす (FR-010, FR-102)。model.body_pos を書き換えるので、
    試行ごとに SimRobot を作り直す (共有のモデルを使い回すと、箱の位置が次の試行に漏れる)。
    全検証 (箱の body があるか) を終えてから書き込む。
    """
    if not isinstance(scenario, Scenario):
        raise ValueError(f"scenario は Scenario で指定してください: {scenario!r}")
    box_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, BOX_BODY_NAME)
    if box_id < 0:
        raise ValueError(f"モデルに body {BOX_BODY_NAME!r} がありません")
    g = SceneGeometry.from_model(model)
    place_blocks(model, data, scenario.blocks, g.block_names)  # 内部で mj_forward (箱の移動前)
    model.body_pos[box_id][0] = scenario.box_xy[0]
    model.body_pos[box_id][1] = scenario.box_xy[1]
    mujoco.mj_forward(model, data)

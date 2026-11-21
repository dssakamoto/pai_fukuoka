"""試行ごとの配置 (Scenario) と、その生成・適用 (SGT-005, SGT-006)。

【機能概要】ブロックと箱の位置を「定数」でなく試行ごとの状態として表す。
- `sample_scenario(seed, spec)`: シードから Scenario を作る。`box=Fixed` の既定では、ブロック配置が既存の
  `sample_layout` と同じ (乱数を同じ順序で消費する。箱は最後に引き、Fixed は乱数を消費しない = 既存の seed が変わらない)。
- `apply_scenario(model, data, scenario)`: シミュレーションに配置を反映する (ブロック + 箱の body_pos)。

純粋関数 (sample_scenario) は入力を変更せず、乱数は seed からのみ作る。Scenario は tuple のみを持つ (ndarray を持たない)。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import mujoco
import numpy as np

from so101_vlm.eval.geometry import BOX_BODY_NAME, PlaceRegion, SceneGeometry, get_geometry
from so101_vlm.eval.layout import _check_feasible, _sample_points, place_blocks
from so101_vlm.eval.validation import require_area, require_int, require_min_gap
from so101_vlm.validation import require_number

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
    """【機能概要】: 箱の位置を、到達可能領域の中から一様に引く。生成は SGT-010 で実装する 🟡

    region: 到達可能領域 (ReachableRegion)、margin: 箱の外周に足す余裕 [m]。
    """

    region: Any = None  # ReachableRegion (SGT-009 で追加)。型は後で付ける
    margin: float = 0.0


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
    reachable: Any = None,
) -> Scenario:
    """【機能概要】: シードから Scenario を作る 🔵

    不正な seed・spec は、乱数を作る前に ValueError。ブロック → 箱の順に、同じ rng から引く。
    UniformXY (箱の一様生成) は SGT-010 で実装するまで NotImplementedError。
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
        raise NotImplementedError("UniformXY の生成は SGT-010 で実装します")
    return Scenario(seed=seed, blocks=blocks, box_xy=(float(box_xy[0]), float(box_xy[1])))


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

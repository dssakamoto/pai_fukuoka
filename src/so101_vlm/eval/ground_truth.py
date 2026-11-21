"""真値の唯一の窓口 GroundTruth (SGT-007)。

【機能概要】評価・デバッグ専用。試行の Scenario (配置の状態) と SceneGeometry (形状) から、真のブロック位置と置き先の領域を返す。
oracle・固定位置デモ・試行・評価ノードは、真値をここ経由でだけ得る (これまで 7 か所で独立に再構築していた)。

【依存の向き】policy / perception / vlm は本モジュールを import してはならない (方策は真値を見ない。SGT-002 のテストが検知する)。
"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco

from so101_vlm.eval.geometry import BOX_BODY_NAME, PlaceRegion, SceneGeometry
from so101_vlm.eval.scenario import Scenario

COLOURS = ("red", "blue", "green")  # 🔵 block_names の順 = 赤・青・緑


@dataclass(frozen=True)
class GroundTruth:
    """【機能概要】: 1 試行の真値。block_positions があれば (from_sim)、実際の位置、なければ配置時の既知の位置 🟡"""

    scenario: Scenario
    geometry: SceneGeometry
    block_positions: tuple[tuple[float, float, float], ...] | None = None

    @property
    def layout(self) -> tuple[tuple[float, float], ...]:
        """【機能概要】: ブロック中心の (x, y)。block_positions があればそちらの xy 🟡"""
        if self.block_positions is not None:
            return tuple((p[0], p[1]) for p in self.block_positions)
        return self.scenario.blocks

    @property
    def place_region(self) -> PlaceRegion:
        """【機能概要】: 置き先の領域 (箱の中心 + 形状) 🟡"""
        return self.scenario.place_region(self.geometry)

    def block_xyz(self, colour: str) -> tuple[float, float, float]:
        """【機能概要】: colour のブロック中心の位置。配置時は z = 静止の高さ 🟡"""
        if colour not in COLOURS:
            raise ValueError(f"colour は {', '.join(COLOURS)} のどれかで指定してください: {colour!r}")
        i = COLOURS.index(colour)
        if self.block_positions is not None:
            return self.block_positions[i]
        x, y = self.scenario.blocks[i]
        return (x, y, self.geometry.block_rest_z)

    @classmethod
    def from_sim(
        cls, scenario: Scenario, geometry: SceneGeometry, model: mujoco.MjModel, data: mujoco.MjData
    ) -> GroundTruth:
        """【機能概要】: 実行後のシミュレーションから、ブロックと箱の実際の位置を読む 🟡

        箱は body_pos (apply_scenario が書いた値) を読んで、scenario の box_xy を実際の値に置き換える。
        """
        positions = tuple(
            tuple(float(v) for v in data.xpos[model.body(name).id]) for name in geometry.block_names
        )
        box = model.body(BOX_BODY_NAME).id
        box_xy = (float(model.body_pos[box][0]), float(model.body_pos[box][1]))
        actual = Scenario(seed=scenario.seed, blocks=tuple((p[0], p[1]) for p in positions), box_xy=box_xy)
        return cls(scenario=actual, geometry=geometry, block_positions=positions)  # type: ignore[arg-type]

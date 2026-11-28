"""評価の抽象 Evaluator と、sim 実装 SimEvaluator (SGT-008)。

【機能概要】成功判定を方策から分離し、差し替えられるようにする (FR-007, FR-402)。
- `Evaluator.evaluate(colour) -> JudgeResult`: 実機版は画像・人の入力で判定する前提で、colour のみを受ける
  (置き先の領域などは構築時に渡す。G5)。実機の Evaluator は別 plan (Phase 5 以降)。
- `SimEvaluator`: シミュレーションの真のブロック位置・速度と、置き先の領域 (PlaceRegion) で判定する。箱を動かした試行では
  その領域を渡す (SGT-004 の judge_in_region)。
"""

from __future__ import annotations

from typing import Protocol

import mujoco

from so101_vlm.eval.eval_config import JudgeConfig
from so101_vlm.eval.geometry import PlaceRegion, SceneGeometry
from so101_vlm.eval.ground_truth import COLOURS
from so101_vlm.eval.judge import JudgeResult, judge_in_region
from so101_vlm.eval.pick_tracker import block_linear_velocity


class Evaluator(Protocol):
    """【機能概要】: colour のブロックが置き先で成功しているかを判定する 🟡"""

    def evaluate(self, colour: str) -> JudgeResult: ...


class SimEvaluator:
    """【機能概要】: sim の真値で判定する Evaluator 🟡

    model / data は参照を持つ (実行が進むと、その時点の状態で判定する。読み取りのみ)。
    不正な引数は、判定の前 (生成時) に ValueError。
    """

    def __init__(
        self, model: mujoco.MjModel, data: mujoco.MjData, judge_config: JudgeConfig, region: PlaceRegion
    ) -> None:
        if not isinstance(judge_config, JudgeConfig):
            raise ValueError(f"judge_config は JudgeConfig で指定してください: {judge_config!r}")
        if not isinstance(region, PlaceRegion):
            raise ValueError(f"region は PlaceRegion で指定してください: {region!r}")
        self._model = model
        self._data = data
        self._config = judge_config
        self._region = region
        self._block_names = SceneGeometry.from_model(model).block_names

    @property
    def region(self) -> PlaceRegion:
        return self._region

    def evaluate(self, colour: str) -> JudgeResult:
        """【機能概要】: colour のブロックの中心・速度を読んで judge_in_region に渡す 🟡"""
        if not isinstance(colour, str) or colour not in COLOURS:
            raise ValueError(f"colour は {', '.join(COLOURS)} のどれかで指定してください: {colour!r}")
        body_id = int(self._model.body(self._block_names[COLOURS.index(colour)]).id)
        velocity = block_linear_velocity(self._model, self._data, body_id)
        return judge_in_region(self._data.xpos[body_id], velocity, self._config, self._region)

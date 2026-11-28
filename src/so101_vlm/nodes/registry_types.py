"""試行の指定 TaskSpec (TASK-0025)。registry と各ノードが共有する (循環 import を避けるため別ファイル)。"""

from dataclasses import dataclass

from so101_vlm.eval.fixed_pick import COLOUR_TO_BODY
from so101_vlm.validation import require_int, require_number


@dataclass(frozen=True)
class TaskSpec:
    """【機能概要】: 1 試行の指定。seed (配置)、box_xy (箱の位置。省略可)、colour (判定する対象の真値)、instruction (VLM への指示)。検証つき 🔵"""

    seed: int
    colour: str
    instruction: str
    box_xy: tuple[float, float] | None = None  # 箱の位置 (None なら XML の既定)。JSON を経由すると list で届くので tuple にそろえる

    def __post_init__(self) -> None:
        require_int("seed", self.seed, 0)
        if self.box_xy is not None:
            if not isinstance(self.box_xy, (tuple, list)) or len(self.box_xy) != 2:
                raise ValueError(f"box_xy は 2 要素で指定してください: {self.box_xy!r}")
            object.__setattr__(self, "box_xy", (require_number("box_xy[0]", self.box_xy[0]), require_number("box_xy[1]", self.box_xy[1])))
        if not isinstance(self.colour, str) or self.colour not in COLOUR_TO_BODY:
            raise ValueError(f"colour は {', '.join(COLOUR_TO_BODY)} のどれかで指定してください: {self.colour!r}")
        if not isinstance(self.instruction, str) or not self.instruction.strip():
            raise ValueError("instruction は空でない文字列で指定してください")

    def scenario(self):
        """【機能概要】: この試行の配置 (Scenario)。box_xy が None なら既定の箱 (従来どおり) 🔵 (SGT-007)"""
        from so101_vlm.eval.eval_config import load_eval_config
        from so101_vlm.eval.scenario import Fixed, scenario_for

        return scenario_for(self.seed, load_eval_config().placement, None if self.box_xy is None else Fixed(self.box_xy))

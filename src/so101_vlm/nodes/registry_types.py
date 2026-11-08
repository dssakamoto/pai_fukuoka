"""試行の指定 TaskSpec (TASK-0025)。registry と各ノードが共有する (循環 import を避けるため別ファイル)。"""

from dataclasses import dataclass

from so101_vlm.eval.fixed_pick import COLOUR_TO_BODY
from so101_vlm.eval.validation import require_int


@dataclass(frozen=True)
class TaskSpec:
    """【機能概要】: 1 試行の指定。seed (配置)、colour (判定する対象の真値)、instruction (VLM への指示)。検証つき 🔵"""

    seed: int
    colour: str
    instruction: str

    def __post_init__(self) -> None:
        require_int("seed", self.seed, 0)
        if not isinstance(self.colour, str) or self.colour not in COLOUR_TO_BODY:
            raise ValueError(f"colour は {', '.join(COLOUR_TO_BODY)} のどれかで指定してください: {self.colour!r}")
        if not isinstance(self.instruction, str) or not self.instruction.strip():
            raise ValueError("instruction は空でない文字列で指定してください")

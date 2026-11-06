"""真の配置から検出結果を作る偽 VLM OracleVlm (TASK-0024)。

🟡 API キーなしで 1 試行を通す (統合テスト、TASK-0025 のデータフロー、TASK-0026 の集計の動作確認) ための VlmClient 互換。
ブロックと箱の 3 次元の角を、キャリブレーションで画像に投影して、ちょうど囲む枠を返す。画像と指示の文章は使わない。
実際の VLM の枠はもっとゆるい (大きい・ずれる) ので、検出誤差の評価 (AC-05) には使えない。
"""

from collections.abc import Sequence
from itertools import product

import numpy as np

from so101_vlm.eval.scene_spec import SCENE_SPEC, SceneSpec
from so101_vlm.perception.calibration import Calibration
from so101_vlm.perception.projection import project_base_to_pixel
from so101_vlm.vlm.schema import Detection, PlanResult, box_to_pixels

_COLOURS = ("red", "blue", "green")


def _corners(cx: float, cy: float, half: float, z_lo: float, z_hi: float) -> list[tuple[float, float, float]]:
    return [(cx + sx * half, cy + sy * half, z) for sx, sy, z in product((-1, 1), (-1, 1), (z_lo, z_hi))]


class OracleVlm:
    """【機能概要】: layout (ブロック中心 xy の列) と色から、pick / place の枠を作る 🟡"""

    def __init__(
        self,
        layout: Sequence[tuple[float, float]],
        colour: str,
        calibration: Calibration,
        spec: SceneSpec = SCENE_SPEC,
    ) -> None:
        if not isinstance(colour, str) or colour not in _COLOURS:
            raise ValueError(f"colour は {_COLOURS} のどれかで指定してください: {colour!r}")
        if len(layout) != len(spec.block_names):
            raise ValueError(f"layout は {len(spec.block_names)} 個の (x, y) で指定してください: {len(layout)}")
        self.provider = "oracle"
        self.model = "ground-truth"
        self._colour = colour
        self._cal = calibration
        self._spec = spec
        x, y = layout[_COLOURS.index(colour)]
        half = spec.block_half_size
        self._pick_corners = _corners(x, y, half, spec.block_rest_z - half, spec.block_rest_z + half)
        bx, by = spec.box_center_xy
        outer = spec.box_inner_half_size + spec.box_wall_thickness
        self._place_corners = _corners(bx, by, outer, 0.0, spec.box_rim_z)

    def _bbox(self, corners: list[tuple[float, float, float]]) -> tuple[int, int, int, int]:
        cam = self._cal.camera
        uv = np.array([project_base_to_pixel(c, self._cal.T_base_cam, cam)[:2] for c in corners])
        lo, hi = uv.min(axis=0), uv.max(axis=0)
        box = [np.floor(lo[0]), np.floor(lo[1]), np.ceil(hi[0]), np.ceil(hi[1])]
        return box_to_pixels(box, "xyxy_px", cam.width, cam.height)

    # 🟡 Intent: 本物の VlmClient と同じシグネチャ。画像・指示は使わず、真の配置だけで決まる。
    def plan(self, image: np.ndarray, instruction: str) -> PlanResult:
        """【機能概要】: 真の位置を囲む枠を PlanResult で返す (image・instruction は使わない) 🟡"""
        return PlanResult(
            pick=Detection(f"{self._colour} block", self._bbox(self._pick_corners), 1.0),
            place=Detection("box", self._bbox(self._place_corners), 1.0),
        )

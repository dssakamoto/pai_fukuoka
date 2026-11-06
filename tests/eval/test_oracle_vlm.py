"""OracleVlm のテスト (TASK-0024, Red)。真の配置から作る検出結果 (API なしで 1 試行を通すための偽 VLM)。"""

import numpy as np
import pytest

from so101_vlm.eval.layout import sample_layout
from so101_vlm.eval.oracle_vlm import OracleVlm
from so101_vlm.eval.scene_spec import SCENE_SPEC
from so101_vlm.perception.calibration import load_calibration
from so101_vlm.perception.projection import project_base_to_pixel
from so101_vlm.vlm.schema import PlanResult

IMG = np.zeros((480, 640, 3), np.uint8)
CAL = load_calibration()
LAYOUT = sample_layout(0)


def _pixel(point):
    u, v, _ = project_base_to_pixel(point, CAL.T_base_cam, CAL.camera)
    return u, v


def test_returns_plan_with_boxes_inside_image():
    r = OracleVlm(LAYOUT, "red", CAL).plan(IMG, "red block into the box")
    assert isinstance(r, PlanResult)
    for det in (r.pick, r.place):
        x1, y1, x2, y2 = det.bbox_px
        assert 0 <= x1 < x2 <= 639 and 0 <= y1 < y2 <= 479
        assert det.confidence == 1.0


@pytest.mark.parametrize("colour,idx", [("red", 0), ("blue", 1), ("green", 2)])
def test_pick_box_contains_the_projected_block_centre(colour, idx):
    x, y = LAYOUT[idx]
    u, v = _pixel((x, y, SCENE_SPEC.block_rest_z))
    x1, y1, x2, y2 = OracleVlm(LAYOUT, colour, CAL).plan(IMG, "go").pick.bbox_px
    assert x1 <= u <= x2 and y1 <= v <= y2
    assert colour in OracleVlm(LAYOUT, colour, CAL).plan(IMG, "go").pick.label


def test_box_is_larger_than_the_block_and_place_contains_box_centre():
    r = OracleVlm(LAYOUT, "red", CAL).plan(IMG, "go")
    pw = r.pick.bbox_px[2] - r.pick.bbox_px[0]
    bw = r.place.bbox_px[2] - r.place.bbox_px[0]
    assert bw > pw
    u, v = _pixel((*SCENE_SPEC.box_center_xy, SCENE_SPEC.box_floor_top_z))
    x1, y1, x2, y2 = r.place.bbox_px
    assert x1 <= u <= x2 and y1 <= v <= y2
    assert r.place.label == "box"


def test_ignores_the_instruction_text():
    a = OracleVlm(LAYOUT, "red", CAL).plan(IMG, "a")
    b = OracleVlm(LAYOUT, "red", CAL).plan(IMG, "something else")
    assert a == b


@pytest.mark.parametrize("colour", ["purple", "", None])
def test_bad_colour_is_value_error(colour):
    with pytest.raises(ValueError):
        OracleVlm(LAYOUT, colour, CAL)


def test_layout_size_must_match_blocks():
    with pytest.raises(ValueError):
        OracleVlm(LAYOUT[:2], "red", CAL)

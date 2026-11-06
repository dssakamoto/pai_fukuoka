"""vlm.schema のテスト (TASK-0017, Red)。純粋ロジックのみ。"""

import dataclasses
import json

import pytest

from so101_vlm.vlm.schema import (
    COORD_FORMATS,
    Detection,
    PlanError,
    PlanResult,
    box_to_pixels,
    parse_plan,
)

W, H = 640, 480


def _plan(pick_box=(100, 100, 200, 200), place_box=(300, 300, 400, 400), **extra):
    return json.dumps(
        {
            "pick": {"label": "red block", "box": list(pick_box), **extra},
            "place": {"label": "box", "box": list(place_box)},
        }
    )


# --- 変換 ---------------------------------------------------------------
def test_normalized_yxyx_converts_to_pixels():
    assert box_to_pixels([437, 628, 525, 684], "yxyx_norm1000", W, H) == (402, 210, 438, 252)


def test_xyxy_px_is_passed_through():
    assert box_to_pixels([10, 20, 110, 220], "xyxy_px", W, H) == (10, 20, 110, 220)


def test_out_of_range_is_clipped_into_image():
    x1, y1, x2, y2 = box_to_pixels([100, 100, 700, 500], "xyxy_px", W, H)
    assert (x2, y2) == (W - 1, H - 1)
    assert 0 <= x1 < x2 <= W - 1 and 0 <= y1 < y2 <= H - 1


def test_negative_coordinates_are_clipped_to_zero():
    assert box_to_pixels([-30, -5, 50, 60], "xyxy_px", W, H)[:2] == (0, 0)


def test_reversed_corners_are_reordered():
    assert box_to_pixels([200, 200, 100, 100], "xyxy_px", W, H) == (100, 100, 200, 200)


@pytest.mark.parametrize("box", [[10, 10, 10, 50], [10, 10, 50, 10], [700, 10, 800, 50]])
def test_empty_box_after_clip_is_plan_error(box):
    with pytest.raises(PlanError):
        box_to_pixels(box, "xyxy_px", W, H)


@pytest.mark.parametrize(
    "box", [[1, 2, 3], [1, 2, 3, 4, 5], "abc", None, [1, 2, 3, "x"], [1, 2, 3, float("nan")], [1, 2, 3, True]]
)
def test_malformed_box_is_plan_error(box):
    with pytest.raises(PlanError):
        box_to_pixels(box, "xyxy_px", W, H)


def test_unknown_coord_format_is_value_error():
    with pytest.raises(ValueError):
        box_to_pixels([1, 2, 3, 4], "nope", W, H)
    assert set(COORD_FORMATS) == {"xyxy_px", "yxyx_norm1000"}


@pytest.mark.parametrize("w,h", [(0, 480), (640, 0), (-1, 5)])
def test_bad_image_size_is_value_error(w, h):
    with pytest.raises(ValueError):
        box_to_pixels([1, 2, 3, 4], "xyxy_px", w, h)


# --- parse_plan -----------------------------------------------------------
def test_parse_plan_returns_frozen_result():
    r = parse_plan(_plan(confidence=0.9), "xyxy_px", W, H)
    assert isinstance(r, PlanResult) and isinstance(r.pick, Detection)
    assert r.pick.label == "red block" and r.pick.bbox_px == (100, 100, 200, 200)
    assert r.pick.confidence == 0.9 and r.place.confidence is None
    with pytest.raises(dataclasses.FrozenInstanceError):
        r.pick.label = "x"  # type: ignore[misc]


def test_parse_plan_converts_by_format():
    r = parse_plan(_plan((437, 628, 525, 684)), "yxyx_norm1000", W, H)
    assert r.pick.bbox_px == (402, 210, 438, 252)


def test_parse_plan_strips_code_fence():
    text = "```json\n" + _plan() + "\n```"
    assert parse_plan(text, "xyxy_px", W, H).pick.bbox_px == (100, 100, 200, 200)


@pytest.mark.parametrize("text", ['{"pick": {"label": "a", "box": [1,2,3,4]', "", "not json", "[1,2]", "null"])
def test_invalid_json_is_plan_error(text):
    with pytest.raises(PlanError):
        parse_plan(text, "xyxy_px", W, H)


@pytest.mark.parametrize(
    "payload",
    [
        {"place": {"label": "box", "box": [1, 2, 30, 40]}},
        {"pick": None, "place": {"label": "box", "box": [1, 2, 30, 40]}},
        {"pick": {"label": "a", "box": [1, 2, 30, 40]}},
        {"pick": {"label": "a"}, "place": {"label": "b", "box": [1, 2, 30, 40]}},
        {"pick": {"box": [1, 2, 30, 40]}, "place": {"label": "b", "box": [1, 2, 30, 40]}},
        {"pick": {"label": "", "box": [1, 2, 30, 40]}, "place": {"label": "b", "box": [1, 2, 30, 40]}},
    ],
)
def test_missing_target_is_plan_error(payload):
    with pytest.raises(PlanError):
        parse_plan(json.dumps(payload), "xyxy_px", W, H)


def test_not_found_reason_is_carried_in_error():
    with pytest.raises(PlanError, match="no purple block"):
        parse_plan(json.dumps({"pick": None, "error": "no purple block"}), "xyxy_px", W, H)


@pytest.mark.parametrize("conf", [-0.1, 1.5, "high", True, float("nan")])
def test_bad_confidence_is_plan_error(conf):
    with pytest.raises(PlanError):
        parse_plan(_plan(confidence=conf), "xyxy_px", W, H)

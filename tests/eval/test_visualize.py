"""eval.visualize のテスト (TASK-0023, Red)。純粋関数 (MuJoCo なし)。"""

import numpy as np
import pytest

from so101_vlm.eval.visualize import PICK_COLOR, PLACE_COLOR, draw_plan, save_overlay
from so101_vlm.vlm.schema import Detection, PlanResult

H, W = 480, 640


def _plan(pick=(100, 200, 160, 260), place=(300, 250, 420, 370), pick_conf=0.9):
    return PlanResult(
        Detection("red block", pick, pick_conf),
        Detection("box", place, None),
    )


def _img(v=40):
    return np.full((H, W, 3), v, dtype=np.uint8)


def test_original_is_untouched_and_new_array_returned():
    img = _img()
    before = img.copy()
    out = draw_plan(img, _plan())
    assert out is not img and not np.shares_memory(out, img)
    assert np.array_equal(img, before)
    assert out.shape == img.shape and out.dtype == np.uint8
    assert not np.array_equal(out, img)


def test_box_edges_have_the_role_colour():
    x1, y1, x2, y2 = 100, 200, 160, 260
    out = draw_plan(_img(), _plan())
    pick = np.array(PICK_COLOR)
    # 辺の中央 (ラベルと重ならない下辺・左右の辺)
    assert np.array_equal(out[y2, (x1 + x2) // 2], pick)
    assert np.array_equal(out[(y1 + y2) // 2, x1], pick)
    assert np.array_equal(out[(y1 + y2) // 2, x2], pick)
    place = np.array(PLACE_COLOR)
    assert np.array_equal(out[370, 360], place)


def test_inside_of_box_is_not_filled():
    out = draw_plan(_img(), _plan())
    assert np.array_equal(out[230, 130], np.array([40, 40, 40]))


def test_far_away_pixels_are_unchanged():
    out = draw_plan(_img(), _plan())
    assert np.array_equal(out[0:50, 0:50], _img()[0:50, 0:50])
    assert np.array_equal(out[450:, 500:], _img()[450:, 500:])


def test_pick_and_place_colours_differ():
    assert PICK_COLOR != PLACE_COLOR


def test_label_is_drawn_next_to_the_box():
    plain = draw_plan(_img(), _plan(), labels=False)
    labelled = draw_plan(_img(), _plan(), labels=True)
    diff = np.any(plain != labelled, axis=2)
    assert diff.any()
    ys, xs = np.nonzero(diff)
    # ラベルは pick の枠 (x 100〜160) のすぐ上/下に出る
    assert xs.min() >= 100 - 2 and ys.min() >= 200 - 40


def test_label_fits_inside_image_even_at_the_top_edge():
    plan = _plan(pick=(10, 0, 70, 40), place=(300, 250, 420, 370))
    out = draw_plan(_img(), plan)
    assert out.shape == (H, W, 3)


def test_box_at_the_far_corner_is_drawn_without_error():
    plan = _plan(pick=(600, 440, W - 1, H - 1))
    out = draw_plan(_img(), plan)
    assert np.array_equal(out[H - 1, 620], np.array(PICK_COLOR))


def test_out_of_range_box_is_clipped_not_crash():
    plan = _plan(pick=(-20, -20, 900, 700))
    out = draw_plan(_img(), plan)
    assert out.shape == (H, W, 3)


def test_small_image_still_works():
    img = np.zeros((48, 64, 3), np.uint8)
    out = draw_plan(img, _plan(pick=(5, 10, 20, 30), place=(30, 10, 60, 40)))
    assert out.shape == img.shape and out.any()


@pytest.mark.parametrize(
    "bad", [np.zeros((4, 4)), np.zeros((4, 4, 3), np.float32), np.zeros((4, 4, 4), np.uint8), "x", None]
)
def test_bad_image_is_value_error(bad):
    with pytest.raises(ValueError):
        draw_plan(bad, _plan())


def test_bad_plan_is_value_error():
    with pytest.raises(ValueError):
        draw_plan(_img(), "plan")


def test_save_overlay_writes_a_readable_png(tmp_path):
    import imageio.v3 as iio

    img = _img()
    path = save_overlay(tmp_path / "sub" / "det.png", img, _plan())
    assert path.exists()
    back = iio.imread(path)
    assert back.shape == img.shape
    assert np.array_equal(back, draw_plan(img, _plan()))
    assert np.array_equal(img, _img())

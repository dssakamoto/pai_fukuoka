"""eval.video のテスト (TASK-0028, Red)。フレームの合成・クリップの構成・ラベルは純粋ロジック (MuJoCo なし)。"""

import numpy as np
import pytest

from so101_vlm.eval.video import (
    FrameSpec,
    Overlay,
    clip_duration_s,
    compose_frame,
    compose_summary,
    find_cjk_font,
    make_style,
    plan_clip,
    result_label,
    speed_label,
    stage_label,
    validate_speed,
)
from so101_vlm.eval.visualize import PICK_COLOR, PLACE_COLOR
from so101_vlm.vlm.schema import Detection, PlanResult

H, W = 480, 640
EN = make_style(lang="en")
PLAN = PlanResult(Detection("red block", (100, 250, 160, 310), 0.9), Detection("box", (400, 250, 520, 370), None))


def _gray(v=80, h=H, w=W):
    return np.full((h, w, 3), v, dtype=np.uint8)


def _overlay(**over):
    base = dict(instruction="put the red block into the box", stage="execute", phase="grasp", plan=PLAN,
                result_text=None, result_ok=None, speed=1, header_right=None)
    base.update(over)
    return Overlay(**base)


# --- 倍速・ラベル (AC-06: 倍速の場合は動画内に明記) -------------------------------------------------
def test_speed_label_says_2x_in_japanese():
    assert speed_label(2, "ja") == "2倍速"
    assert speed_label(2, "en") == "2x speed"
    assert speed_label(1, "ja") is None  # 実時間なら表示しない


@pytest.mark.parametrize("speed", [0, -1, 7, 4.5, "2", True, 31])
def test_speed_must_divide_the_fps(speed):
    with pytest.raises(ValueError):
        validate_speed(speed, 30)  # 7 や 4 は 30 fps を割り切れず、間引きの間隔が揃わない


@pytest.mark.parametrize("speed", [1, 2, 3, 5, 6, 10, 15, 30])
def test_valid_speeds(speed):
    assert validate_speed(speed, 30) == speed


def test_stage_and_result_labels():
    assert [stage_label(s, "ja") for s in ("task", "detect", "execute", "result")] == ["指示", "検出", "実行", "判定"]
    assert [stage_label(s, "en") for s in ("task", "detect", "execute", "result")] == ["Task", "Detect", "Execute", "Result"]
    assert result_label(True, None, "ja") == "成功" and result_label(True, None, "en") == "SUCCESS"
    assert result_label(False, "planning_failure", "ja") == "失敗 (planning_failure)"
    assert result_label(False, None, "en") == "FAILED"
    with pytest.raises(ValueError):
        stage_label("nope", "ja")


def test_style_falls_back_to_english_without_a_font(monkeypatch):
    monkeypatch.setattr("so101_vlm.eval.video.find_cjk_font", lambda: None)
    assert make_style().lang == "en"
    with pytest.raises(ValueError, match="フォント"):
        make_style(lang="ja")


def test_style_uses_japanese_when_a_font_exists():
    if find_cjk_font() is None:
        pytest.skip("この環境には日本語フォントがない")
    assert make_style().lang == "ja"


def test_bad_style_arguments():
    with pytest.raises(ValueError):
        make_style(lang="fr")
    with pytest.raises(ValueError):
        make_style(font_path="no/such/font.ttf", lang="en")


# --- クリップの構成 (どのフレームを、どの順で) -----------------------------------------------------------
def test_clip_follows_task_detect_execute_result_in_order():
    labels = ["pre_grasp"] * 3 + ["grasp"] * 2 + ["settle"]
    specs = plan_clip(labels, fps=30, hold_task_s=1.0, hold_detect_s=2.0, hold_result_s=1.5)
    stages = [s.stage for s in specs]
    assert stages == ["task"] * 30 + ["detect"] * 60 + ["execute"] * 6 + ["result"] * 45
    assert [s.phase for s in specs if s.stage == "execute"] == labels
    assert [s.source for s in specs if s.stage == "execute"] == list(range(6))
    assert all(s.source is None for s in specs if s.stage in ("task", "detect"))
    assert all(s.source == 5 for s in specs if s.stage == "result")  # 結果は最後の実行フレームのまま見せる


def test_clip_without_motion_has_no_execute_stage():
    specs = plan_clip([], fps=30, hold_task_s=1.0, hold_detect_s=1.0, hold_result_s=1.0)
    assert [s.stage for s in specs] == ["task"] * 30 + ["detect"] * 30 + ["result"] * 30
    assert all(s.source is None for s in specs) and isinstance(specs[0], FrameSpec)


@pytest.mark.parametrize("kw", [{"fps": 0}, {"hold_task_s": -1}, {"hold_result_s": float("nan")}])
def test_bad_clip_arguments(kw):
    with pytest.raises(ValueError):
        plan_clip(["a"], **kw)


def test_duration():
    assert clip_duration_s(90, 30) == 3.0
    with pytest.raises(ValueError):
        clip_duration_s(10, 0)


# --- フレームの合成 -----------------------------------------------------------------------------------
def test_compose_returns_a_new_frame_and_leaves_the_input_alone():
    img = _gray()
    before = img.copy()
    out = compose_frame(img, _overlay(), EN)
    assert out is not img and out.shape == img.shape and out.dtype == np.uint8
    assert np.array_equal(img, before) and not np.array_equal(out, img)


def test_instruction_is_drawn_in_the_header():
    out = compose_frame(_gray(), _overlay(), EN)
    header = out[: H // 10]
    assert np.any(header != 80)
    assert np.array_equal(out[H // 2 : H // 2 + 20, : 40], _gray()[H // 2 : H // 2 + 20, : 40])  # 中央左の領域は触らない


def test_boxes_appear_from_the_detect_stage_on():
    task = compose_frame(_gray(), _overlay(stage="task"), EN)
    detect = compose_frame(_gray(), _overlay(stage="detect"), EN)
    assert np.array_equal(task[310, 130], np.array([80, 80, 80]))  # 検出の前には枠がない
    assert np.array_equal(detect[310, 130], np.array(PICK_COLOR))
    assert np.array_equal(detect[370, 460], np.array(PLACE_COLOR))


def test_no_plan_draws_no_boxes():
    out = compose_frame(_gray(), _overlay(stage="detect", plan=None), EN)
    assert np.array_equal(out[310, 130], np.array([80, 80, 80]))


def test_speed_badge_only_in_the_bottom_right_and_only_when_sped_up():
    one = compose_frame(_gray(), _overlay(speed=1), EN)
    two = compose_frame(_gray(), _overlay(speed=2), EN)
    ys, xs = np.nonzero(np.any(one != two, axis=2))
    assert len(ys) > 0 and ys.min() >= int(0.8 * H) and xs.min() >= W // 2
    assert np.array_equal(compose_frame(_gray(), _overlay(speed=1), EN), one)


def test_result_banner_is_green_for_success_and_red_for_failure():
    ok = compose_frame(_gray(), _overlay(stage="result", result_text="SUCCESS", result_ok=True), EN)
    ng = compose_frame(_gray(), _overlay(stage="result", result_text="FAILED", result_ok=False), EN)
    region = (slice(int(0.62 * H), int(0.78 * H)), slice(int(0.25 * W), int(0.75 * W)))
    g, r = ok[region].reshape(-1, 3).mean(axis=0), ng[region].reshape(-1, 3).mean(axis=0)
    assert g[1] > g[0] + 20 and g[1] > g[2] + 20  # 緑が優勢
    assert r[0] > r[1] + 20 and r[0] > r[2] + 20  # 赤が優勢
    assert not np.array_equal(ok, compose_frame(_gray(), _overlay(stage="result"), EN))


def test_stage_bar_highlights_the_current_stage():
    a = compose_frame(_gray(), _overlay(stage="task", plan=None), EN)
    b = compose_frame(_gray(), _overlay(stage="result", plan=None), EN)
    assert not np.array_equal(a[: H // 6], b[: H // 6])


def test_phase_text_changes_the_bottom_left():
    a = compose_frame(_gray(), _overlay(phase="grasp"), EN)
    b = compose_frame(_gray(), _overlay(phase="pre_place"), EN)
    assert not np.array_equal(a, b)


def test_japanese_text_renders_when_a_font_exists():
    if find_cjk_font() is None:
        pytest.skip("この環境には日本語フォントがない")
    ja = make_style()
    out = compose_frame(_gray(), _overlay(speed=2, instruction="赤いブロックを箱に入れて"), ja)
    one = compose_frame(_gray(), _overlay(speed=1, instruction="赤いブロックを箱に入れて"), ja)
    assert np.any(out != one)  # 「2倍速」の文字が入る


def test_long_instruction_and_tiny_frames_do_not_crash():
    compose_frame(_gray(), _overlay(instruction="x" * 500), EN)
    compose_frame(_gray(v=40, h=48, w=64), _overlay(plan=None, speed=2, result_text="SUCCESS", result_ok=True, stage="result"), EN)
    compose_frame(_gray(), _overlay(instruction="", header_right="trial 3/10  seed 2"), EN)


@pytest.mark.parametrize("bad", [np.zeros((4, 4)), np.zeros((4, 4, 3), np.float32), "x", None])
def test_bad_image_is_value_error(bad):
    with pytest.raises(ValueError):
        compose_frame(bad, _overlay(), EN)


def test_summary_card():
    card = compose_summary((H, W), ["Success rate: 70% (14/20)", "planning_failure: 3"], EN)
    assert card.shape == (H, W, 3) and card.dtype == np.uint8 and card.std() > 1
    assert np.array_equal(card, compose_summary((H, W), ["Success rate: 70% (14/20)", "planning_failure: 3"], EN))
    with pytest.raises(ValueError):
        compose_summary((0, W), ["x"], EN)

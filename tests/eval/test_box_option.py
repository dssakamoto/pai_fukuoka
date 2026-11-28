"""--box-xy の検証と、実 API を呼ぶ実行の確認のテスト (SGT-014)。API は使わない。"""

import pytest

from so101_vlm.eval.box_option import confirm_api_run, cost_estimate, parse_box_xy
from so101_vlm.eval.geometry import get_geometry
from so101_vlm.eval.reachable import ReachableRegion
from so101_vlm.eval.scenario import Fixed, ScenarioSpec, sample_scenario

G = get_geometry()


def _grid(x_range, y_range, step=0.01):
    nx, ny = round((x_range[1] - x_range[0]) / step) + 1, round((y_range[1] - y_range[0]) / step) + 1
    return ReachableRegion(x_range[0], y_range[0], step, tuple(tuple(True for _ in range(ny)) for _ in range(nx)))


REGION = _grid((0.10, 0.45), (-0.15, 0.30))


# =============================================================== parse_box_xy
def test_none_means_the_default_box():
    assert parse_box_xy(None) is None


def test_values_become_a_fixed_box_and_a_scenario_follows_it():
    box = parse_box_xy([0.25, 0.15], REGION, G)
    assert box == Fixed(xy=(0.25, 0.15))
    assert sample_scenario(3, ScenarioSpec(box=box)).box_xy == (0.25, 0.15)  # Fixed(xy) の ScenarioSpec に変換される


def test_a_position_outside_the_region_is_rejected_with_the_reason_and_the_range():
    with pytest.raises(ValueError) as info:
        parse_box_xy([0.60, 0.15], REGION, G)
    message = str(info.value)
    assert "0.600" in message and "収まらず" in message and "x 0.100〜0.450" in message and "box-sweep.md" in message


def test_a_position_whose_outline_sticks_out_is_rejected():
    inside_but_edge = (0.11, 0.0)  # 中心は領域内だが、箱の外周 (半幅 0.045) が領域の外にはみ出す
    assert REGION.contains_xy(*inside_but_edge)
    with pytest.raises(ValueError, match="収まらず"):
        parse_box_xy(list(inside_but_edge), REGION, G)


@pytest.mark.parametrize("bad", [[0.2], [0.2, 0.1, 0.0], [float("nan"), 0.1], [0.2, float("inf")], [True, 0.1], ["0.2", 0.1]])
def test_malformed_values_raise_value_error(bad):
    with pytest.raises(ValueError):
        parse_box_xy(bad, REGION, G)


# =============================================================== 料金の目安
def test_cost_estimate_shows_tokens_and_a_price_for_openai():
    text = cost_estimate("openai", 3)
    assert "3 回" in text and "2,100" in text and "$0.0005" in text and "キャッシュ" in text


def test_cost_estimate_does_not_invent_a_price_for_other_providers():
    text = cost_estimate("gemini", 3)
    assert "料金表" in text and "$" not in text


@pytest.mark.parametrize("n", [0, -1, True, 1.5])
def test_cost_estimate_rejects_a_bad_count(n):
    with pytest.raises(ValueError):
        cost_estimate("openai", n)


# =============================================================== 確認
def _confirm(provider="openai", n=3, **kw):
    shown = []
    ok = confirm_api_run(provider, n, out=shown.append, **kw)
    return ok, shown


def test_oracle_never_asks_or_prints():
    ok, shown = _confirm("oracle", input_fn=lambda _: pytest.fail("尋ねてはいけない"), is_interactive=lambda: False)
    assert ok is True and shown == []


def test_yes_prints_the_estimate_and_goes_on_without_asking():
    ok, shown = _confirm(yes=True, input_fn=lambda _: pytest.fail("尋ねてはいけない"))
    assert ok is True and "有料" in shown[0]


@pytest.mark.parametrize("answer, expected", [("y", True), ("YES", True), (" yes ", True), ("", False), ("n", False), ("はい", False), ("yess", False)])
def test_interactive_answers(answer, expected):
    ok, shown = _confirm(input_fn=lambda _: answer, is_interactive=lambda: True)
    assert ok is expected and "有料" in shown[0]  # 尋ねる前に目安を表示する


def test_non_interactive_without_yes_is_refused_not_charged():
    with pytest.raises(ValueError, match="--yes"):
        _confirm(is_interactive=lambda: False, input_fn=lambda _: pytest.fail("尋ねてはいけない"))

"""judge_in_box / JudgeResult / JudgeReason のテスト (TASK-0013, Red フェーズ)。

テストケース定義: docs/implements/so101-vlm-pickplace/TASK-0013/pick-testcases.md 4 節 (TC-JG-*)
対象モジュール (まだ存在しない): so101_vlm.eval.judge, so101_vlm.eval.eval_config.JudgeConfig
TASK-0013 の最小版。TASK-0015 が同じファイルを拡張する (TC-JG-01〜04 = TASK-0015 のケース 1〜4)。
実行時間の予算: < 0.1 s (純粋関数のみ、MuJoCo なし)。
境界値は 10 進の値では正確に作れない (0.20 + 0.04 = 0.24000000000000002) ため、
2 進で正確な値の SPEC_EXACT を使う。
信頼性: 🔵 要件・実測に基づく / 🟡 妥当な推測 / 🔴 要件にない仮定
"""

import dataclasses

import numpy as np
import pytest

from so101_vlm.eval.eval_config import JudgeConfig
from so101_vlm.eval.geometry import get_geometry
from so101_vlm.eval.judge import JudgeReason, JudgeResult, judge_in_box, judge_in_region
from so101_vlm.eval.scene_spec import SCENE_SPEC

CFG = JudgeConfig(margin_m=0.0, speed_max_mps=0.01, settle_s=1.0)
# 【2 進で正確な箱】: 中心 (0.25, 0.125)、内寸の半幅 0.0625、縁 0.0625 (全て 2 のべき乗の和)
SPEC_EXACT = dataclasses.replace(
    SCENE_SPEC, box_center_xy=(0.25, 0.125), box_inner_half_size=0.0625, box_rim_z=0.0625
)
STILL = (0.0, 0.0, 0.0)
NAN, INF = float("nan"), float("inf")


def judge(centre, velocity=STILL, config=CFG, spec=SCENE_SPEC):
    return judge_in_box(centre, velocity, config, spec)


# =============================================================== 正常系
def test_success_when_inside_box_and_still():
    """TC-JG-01 箱の中で静止 → 成功 🔵 (TASK-0015 ケース 1。実測の seed 0 の値)"""
    # 【テストデータ準備】: 箱中心 (0.20, 0.19) から (-2, -4) mm ずれた静止位置
    result = judge((0.198, 0.186, 0.0199))
    # 【結果検証】
    assert result.success is True  # 【確認内容】: 成功
    assert result.reasons == ()  # 【確認内容】: 失敗の理由なし
    assert result.offset_xy == pytest.approx((-0.002, -0.004))  # 【確認内容】: 箱中心からのずれ
    assert result.height == pytest.approx(0.0199)  # 【確認内容】: 中心の z
    assert result.speed == 0.0  # 【確認内容】: 速さ


@pytest.mark.parametrize(
    "centre",
    [(0.25, 0.19, 0.02), (0.15, 0.19, 0.02), (0.20, 0.24, 0.02), (0.20, 0.13, 0.02)],
    ids=["+x", "-x", "+y", "-y"],
)
def test_outside_inner_when_beyond_any_side(centre):
    """TC-JG-02 内寸の外 → OUTSIDE_INNER 🔵 (ケース 2。各辺の外 4 通り)"""
    result = judge(centre)
    assert result.success is False  # 【確認内容】: 失敗
    assert result.reasons == (JudgeReason.OUTSIDE_INNER,)  # 【確認内容】: 理由は内寸の外だけ


def test_above_rim_when_resting_on_the_rim():
    """TC-JG-03 縁以上 → ABOVE_RIM 🔵 (ケース 3。縁に乗ったブロックの中心 = 0.04 + 0.015)"""
    result = judge((0.20, 0.19, 0.055))
    assert result.success is False  # 【確認内容】: 失敗
    assert result.reasons == (JudgeReason.ABOVE_RIM,)  # 【確認内容】: 理由は縁以上だけ


def test_moving_when_speed_exceeds_threshold():
    """TC-JG-04 動いている → MOVING 🔵 (ケース 4)"""
    result = judge((0.20, 0.19, 0.0199), (0.0, 0.0, -0.05))
    assert result.success is False  # 【確認内容】: 失敗
    assert result.reasons == (JudgeReason.MOVING,)  # 【確認内容】: 理由は速さだけ
    assert result.speed == pytest.approx(0.05)  # 【確認内容】: 並進の速さ [m/s]


def test_multiple_reasons_are_reported_in_order():
    """TC-JG-05 複数の理由は (1)(2)(3) の順に全部 🟡"""
    result = judge((0.30, 0.19, 0.10), (0.1, 0.0, 0.0))
    assert result.reasons == (
        JudgeReason.OUTSIDE_INNER,
        JudgeReason.ABOVE_RIM,
        JudgeReason.MOVING,
    )  # 【確認内容】: 失敗した条件を全部、順序は (1)(2)(3)
    assert [r.value for r in JudgeReason] == ["outside_inner", "above_rim", "moving"]  # 【確認内容】: 値
    assert isinstance(JudgeReason.MOVING, str)  # 【確認内容】: str の Enum


def test_judge_is_pure_and_result_is_immutable():
    """TC-JG-06 純粋・不変 🔵"""
    centre = np.array([0.198, 0.186, 0.0199])
    velocity = np.array([0.001, 0.0, 0.0])
    before = (centre.copy(), velocity.copy())
    first = judge(centre, velocity)
    second = judge(centre, velocity)
    assert np.array_equal(centre, before[0]) and np.array_equal(velocity, before[1])  # 【確認内容】: 入力不変
    assert first == second  # 【確認内容】: 同じ入力で同じ結果
    assert isinstance(first, JudgeResult) and isinstance(first.reasons, tuple)  # 【確認内容】: 型
    with pytest.raises(dataclasses.FrozenInstanceError):
        first.success = False  # 【確認内容】: frozen


# =============================================================== 異常系
@pytest.mark.parametrize(
    ("centre", "velocity"),
    [
        ((NAN, 0.19, 0.02), STILL),
        ((0.2, 0.19), STILL),
        ((INF, 0.0, 0.0), STILL),
        (True, STILL),
        ((0.2, 0.19, 0.02), (0.0, 0.0, NAN)),
        ((0.2, 0.19, 0.02), (0.0, 0.0)),
    ],
    ids=["centre-nan", "centre-2d", "centre-inf", "centre-bool", "vel-nan", "vel-2d"],
)
def test_invalid_vectors_raise_value_error(centre, velocity):
    """TC-JG-E01 NaN / inf / shape 違い / bool → ValueError 🟡"""
    with pytest.raises(ValueError):
        judge(centre, velocity)


@pytest.mark.parametrize("config", [None, {"margin_m": 0.0}], ids=["none", "dict"])
def test_invalid_config_raises_value_error(config):
    """TC-JG-E02 config の型違い → ValueError (TypeError にしない) 🔴"""
    with pytest.raises(ValueError):
        judge((0.2, 0.19, 0.02), STILL, config)


# =============================================================== 境界値 (SPEC_EXACT)
@pytest.mark.parametrize("axis", [0, 1], ids=["x", "y"])
def test_exactly_inner_half_width_fails(axis):
    """TC-JG-B01 ちょうど内寸の半幅は失敗 (厳密な不等号) 🟡"""
    cx, cy = SPEC_EXACT.box_center_xy
    on_edge = [cx, cy, 0.03125]
    on_edge[axis] += SPEC_EXACT.box_inner_half_size  # 【境界】: 0.3125 / 0.1875 は 2 進で正確
    assert judge(on_edge, spec=SPEC_EXACT).reasons == (JudgeReason.OUTSIDE_INNER,)  # 【確認内容】: ちょうどは失敗
    inside = list(on_edge)
    inside[axis] = float(np.nextafter(on_edge[axis], 0.0))
    assert judge(inside, spec=SPEC_EXACT).success is True  # 【確認内容】: 1 ulp 内側は成功


def test_exactly_rim_height_fails():
    """TC-JG-B02 ちょうど縁は失敗 🟡"""
    cx, cy = SPEC_EXACT.box_center_xy
    assert judge((cx, cy, 0.0625), spec=SPEC_EXACT).reasons == (JudgeReason.ABOVE_RIM,)  # 【確認内容】: ちょうど
    below = float(np.nextafter(0.0625, 0.0))
    assert judge((cx, cy, below), spec=SPEC_EXACT).success is True  # 【確認内容】: 1 ulp 下は成功


def test_exactly_speed_max_succeeds():
    """TC-JG-B03 ちょうど speed_max は成功 (「以下」) 🟡"""
    inside = (0.20, 0.19, 0.02)
    assert judge(inside, (0.01, 0.0, 0.0)).success is True  # 【確認内容】: 0.01 ちょうどは成功
    over = float(np.nextafter(0.01, 1.0))
    assert judge(inside, (over, 0.0, 0.0)).reasons == (JudgeReason.MOVING,)  # 【確認内容】: 1 ulp 超は失敗


def test_margin_shrinks_the_inner_region():
    """TC-JG-B04 margin の効果 🟡"""
    centre = (0.237, 0.19, 0.02)  # dx ≈ 0.037
    assert judge(centre).success is True  # 【確認内容】: margin 0 なら内側
    tight = dataclasses.replace(CFG, margin_m=0.005)
    assert judge(centre, config=tight).reasons == (JudgeReason.OUTSIDE_INNER,)  # 【確認内容】: 0.035 未満が必要


# =============================================================== judge_in_region (SGT-004)
def _region(dx=0.0, dy=0.0):
    c = get_geometry().box_default_xy
    return get_geometry().place_region((c[0] + dx, c[1] + dy))


def test_region_follows_a_moved_box():
    """SGT-004 箱を +0.05 ずらした領域: ずらした中心なら成功、元の位置だと OUTSIDE_INNER 🔵"""
    moved = _region(dx=0.05)
    cx, cy = moved.center_xy
    assert judge_in_region((cx, cy, 0.02), STILL, CFG, moved).success is True
    old = get_geometry().box_default_xy
    r = judge_in_region((old[0], old[1], 0.02), STILL, CFG, moved)
    assert r.reasons == (JudgeReason.OUTSIDE_INNER,) and r.offset_xy[0] == pytest.approx(-0.05)


def test_region_boundary_with_margin_is_exact_then_one_mm_out_fails():
    """SGT-004 内寸の境界 (margin 込み) の 1 mm 内側は成功、1 mm 外は OUTSIDE_INNER 🔵"""
    region = _region()
    cfg = dataclasses.replace(CFG, margin_m=0.005)
    limit = region.inner_half_size - cfg.margin_m
    cx, cy = region.center_xy
    assert judge_in_region((cx + limit - 0.001, cy, 0.02), STILL, cfg, region).success is True
    assert judge_in_region((cx + limit + 0.001, cy, 0.02), STILL, cfg, region).reasons == (JudgeReason.OUTSIDE_INNER,)


def test_region_height_and_speed():
    """SGT-004 縁未満は成功、縁以上は ABOVE_RIM、速さ超過は MOVING 🔵"""
    region = _region()
    cx, cy = region.center_xy
    assert judge_in_region((cx, cy, region.rim_z - 0.001), STILL, CFG, region).success is True
    assert judge_in_region((cx, cy, region.rim_z), STILL, CFG, region).reasons == (JudgeReason.ABOVE_RIM,)
    assert judge_in_region((cx, cy, 0.02), (0.05, 0.0, 0.0), CFG, region).reasons == (JudgeReason.MOVING,)


def test_region_does_not_mutate_inputs():
    """SGT-004 入力の配列を変更しない 🔵"""
    region = _region()
    centre = np.array([*region.center_xy, 0.02])
    vel = np.zeros(3)
    c0, v0 = centre.copy(), vel.copy()
    judge_in_region(centre, vel, CFG, region)
    assert np.array_equal(centre, c0) and np.array_equal(vel, v0)


def test_region_invalid_region_raises_value_error():
    with pytest.raises(ValueError):
        judge_in_region((0.2, 0.19, 0.02), STILL, CFG, object())


def test_judge_in_box_equals_judge_in_region_for_the_default_region():
    """SGT-004 互換の別名 judge_in_box が、既定の領域の judge_in_region と同じ結果 🔵"""
    region = get_geometry().place_region()
    for centre, vel in [((0.20, 0.19, 0.02), STILL), ((0.30, 0.19, 0.02), STILL), ((0.2, 0.19, 0.2), (0.1, 0, 0))]:
        assert judge_in_box(centre, vel, CFG) == judge_in_region(centre, vel, CFG, region)

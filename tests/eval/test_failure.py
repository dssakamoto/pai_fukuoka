"""FailureKind / PickTrace / classify_failure のテスト (TASK-0013, Red フェーズ)。

テストケース定義: docs/implements/so101-vlm-pickplace/TASK-0013/pick-testcases.md 5 節 (TC-FL-*)
対象モジュール (まだ存在しない): so101_vlm.eval.failure, so101_vlm.eval.eval_config.FailureConfig
PLANNING_FAILURE は classify_failure からは返らない (run_fixed_pick が付ける。test_fixed_pick.py で確認)。
実行時間の予算: < 0.1 s (純粋関数のみ)。
信頼性: 🔵 要件・実測に基づく / 🟡 妥当な推測 / 🔴 要件にない仮定
"""

import dataclasses

import numpy as np
import pytest

from so101_vlm.eval.eval_config import FailureConfig
from so101_vlm.eval.failure import FailureKind, PickTrace, classify_failure
from so101_vlm.eval.judge import JudgeReason, JudgeResult

CFG = FailureConfig(lift_min_m=0.03, carry_separation_m=0.02, other_move_m=0.005, tilt_max_deg=20.0)
# 【実測の赤 seed 0】
NOMINAL = PickTrace(
    lift_end_rise=0.0736,
    max_separation_carry=0.0158,
    max_separation_place=0.0160,
    max_other_move=2e-10,
    final_tilt_deg=0.0,
)
FAILED_JUDGE = JudgeResult(False, (JudgeReason.OUTSIDE_INNER,), (0.1, 0.0), 0.015, 0.0)
OK_JUDGE = JudgeResult(True, (), (-0.002, -0.004), 0.0199, 0.0)
K = FailureKind


def trace(**changes):
    return dataclasses.replace(NOMINAL, **changes)


# =============================================================== 正常系 (1 種類 1 件 + 優先順位)
@pytest.mark.parametrize(
    ("tr", "jd", "expected"),
    [
        pytest.param(NOMINAL, OK_JUDGE, None, id="FL-01-success"),
        pytest.param(trace(max_other_move=0.01, final_tilt_deg=45.0), OK_JUDGE, None, id="FL-02-success-ignores-trace"),
        pytest.param(trace(lift_end_rise=0.0), FAILED_JUDGE, K.GRASP_SLIP, id="FL-03-no-lift"),
        pytest.param(trace(max_separation_carry=0.05), FAILED_JUDGE, K.GRASP_SLIP, id="FL-04-slip-carry"),
        pytest.param(trace(max_separation_place=0.05), FAILED_JUDGE, K.DROP, id="FL-05-drop"),
        pytest.param(trace(max_other_move=0.006), FAILED_JUDGE, K.COLLISION_OTHER, id="FL-06-collision"),
        pytest.param(trace(final_tilt_deg=45.0), FAILED_JUDGE, K.KNOCKED_OVER, id="FL-07-tilt"),
        pytest.param(NOMINAL, FAILED_JUDGE, K.NOT_IN_BOX, id="FL-08-not-in-box"),
        pytest.param(
            trace(lift_end_rise=0.0, max_separation_place=0.05, max_other_move=0.01, final_tilt_deg=45.0),
            FAILED_JUDGE, K.GRASP_SLIP, id="FL-09-priority-slip",
        ),
        pytest.param(
            trace(max_separation_place=0.05, max_other_move=0.01, final_tilt_deg=45.0),
            FAILED_JUDGE, K.DROP, id="FL-10-priority-drop",
        ),
        pytest.param(
            trace(max_other_move=0.01, final_tilt_deg=45.0), FAILED_JUDGE, K.COLLISION_OTHER,
            id="FL-11-priority-collision",
        ),
    ],
)
def test_classify_failure(tr, jd, expected):
    """TC-FL-01〜11 失敗の分類と優先順位 (slip > drop > collision > tilt > not_in_box) 🔵/🟡"""
    assert classify_failure(tr, jd, CFG) is expected  # 【確認内容】: 成功は None、失敗は上から最初の該当


def test_failure_kind_values():
    """TC-FL-08 (列挙) FailureKind の値と型 🔵"""
    assert {k.value for k in FailureKind} == {
        "planning_failure", "grasp_slip", "drop", "collision_other", "knocked_over", "not_in_box",
    }  # 【確認内容】: 6 種類
    assert isinstance(FailureKind.DROP, str)  # 【確認内容】: str の Enum


# =============================================================== 異常系
@pytest.mark.parametrize(
    "bad",
    [
        {"lift_end_rise": float("nan")},
        {"max_separation_carry": -0.01},
        {"final_tilt_deg": float("inf")},
    ],
    ids=["nan", "negative-distance", "inf-tilt"],
)
def test_pick_trace_rejects_invalid_values(bad):
    """TC-FL-E01 トレースの値の不正 → 生成時に ValueError 🔴"""
    with pytest.raises(ValueError):
        trace(**bad)


def test_pick_trace_allows_negative_lift_rise():
    """TC-FL-E01 (補足) lift_end_rise は負を許す (床にめり込む・落ちる場合がある) 🔴"""
    assert trace(lift_end_rise=-0.01).lift_end_rise == pytest.approx(-0.01)  # 【確認内容】: 負でも生成できる


# =============================================================== 境界値
def test_values_exactly_at_thresholds_do_not_apply():
    """TC-FL-B01 しきい値ちょうどは当てはまらない → NOT_IN_BOX 🟡"""
    at_threshold = trace(
        lift_end_rise=0.03,  # 厳密な <
        max_separation_carry=0.02,
        max_separation_place=0.02,
        max_other_move=0.005,
        final_tilt_deg=20.0,
    )
    assert classify_failure(at_threshold, FAILED_JUDGE, CFG) is K.NOT_IN_BOX  # 【確認内容】: どれにも当たらない


@pytest.mark.parametrize(
    ("field", "threshold", "direction", "expected"),
    [
        ("lift_end_rise", 0.03, 0.0, K.GRASP_SLIP),
        ("max_separation_carry", 0.02, 1.0, K.GRASP_SLIP),
        ("max_separation_place", 0.02, 1.0, K.DROP),
        ("max_other_move", 0.005, 1.0, K.COLLISION_OTHER),
        ("final_tilt_deg", 20.0, 90.0, K.KNOCKED_OVER),
    ],
)
def test_one_ulp_beyond_each_threshold_applies(field, threshold, direction, expected):
    """TC-FL-B02 しきい値の 1 ulp 外側はそれぞれの種類になる 🟡"""
    value = float(np.nextafter(threshold, direction if direction != 0.0 else 0.0))
    assert classify_failure(trace(**{field: value}), FAILED_JUDGE, CFG) is expected  # 【確認内容】: 外側で該当

"""失敗の分類 FailureKind / PickTrace / classify_failure (TASK-0013)。

🟡 純粋関数 (TASK-0026 の集計でも同じ型を使う)。design.md 4.8 の「IK 失敗 / 把持失敗 / 配置失敗」との対応:
IK 失敗 = planning_failure、把持失敗 = grasp_slip / drop、配置失敗 = collision_other / knocked_over / not_in_box。
🔴 しきい値 (configs/eval.yaml の failure:) は初期値。
"""

import math
from dataclasses import dataclass, fields
from enum import Enum

from so101_vlm.eval.eval_config import FailureConfig
from so101_vlm.eval.judge import JudgeResult


class FailureKind(str, Enum):
    """【機能概要】: 失敗の種類 (6 種)。🟡"""

    PLANNING_FAILURE = "planning_failure"  # TrajectoryError (動作前に拒否)
    GRASP_SLIP = "grasp_slip"  # 持ち上がらない / 持ち上げ・運搬中に抜けた
    DROP = "drop"  # place 区間 (解放前) に離れた
    COLLISION_OTHER = "collision_other"  # 他のブロックを動かした
    KNOCKED_OVER = "knocked_over"  # 最終の傾きが大きい
    NOT_IN_BOX = "not_in_box"  # 上のどれでもなく判定が失敗


@dataclass(frozen=True)
class PickTrace:
    """【機能概要】: 1 試行の追跡の要約。🟡

    lift_end_rise: lift 区間の最後の周期でのブロック中心 z − 静止高さ [m] (負も許す)、
    max_separation_carry: lift / pre_place 区間のブロック中心と TCP の距離の最大 [m]、
    max_separation_place: place 区間 (open の前) の同じ距離の最大 [m]、
    max_other_move: 他のブロックの xy 移動の最大 (全周期) [m]、
    final_tilt_deg: 最終の傾き [deg]。有限であること。lift_end_rise 以外は 0 以上。
    """

    lift_end_rise: float
    max_separation_carry: float
    max_separation_place: float
    max_other_move: float
    final_tilt_deg: float

    def __post_init__(self) -> None:
        for field in fields(self):
            value = float(getattr(self, field.name))
            if not math.isfinite(value):
                raise ValueError(f"{field.name} は有限の数値で指定してください: {value!r}")
            if field.name != "lift_end_rise" and value < 0:
                raise ValueError(f"{field.name} は 0 以上で指定してください: {value!r}")


def classify_failure(
    trace: PickTrace, judge: JudgeResult, config: FailureConfig
) -> FailureKind | None:
    """【機能概要】: 判定が失敗のとき、上から順に最初に当てはまる失敗の種類を返す。成功は None 🟡

    優先順位: grasp_slip > drop > collision_other > knocked_over > not_in_box。
    しきい値ちょうどは当てはまらない (lift_min_m は未満、他は超えたとき)。
    """
    if judge.success:
        return None
    if (
        trace.lift_end_rise < config.lift_min_m
        or trace.max_separation_carry > config.carry_separation_m
    ):
        return FailureKind.GRASP_SLIP
    if trace.max_separation_place > config.carry_separation_m:
        return FailureKind.DROP
    if trace.max_other_move > config.other_move_m:
        return FailureKind.COLLISION_OTHER
    if trace.final_tilt_deg > config.tilt_max_deg:
        return FailureKind.KNOCKED_OVER
    return FailureKind.NOT_IN_BOX

"""成功判定 judge_in_box (TASK-0013 の最小版。TASK-0015 で正式化する)。

🔵 design.md 6 節の配置、FR-07 の 3 条件に基づく。純粋関数 (入力を変更しない、乱数・時計を使わない)。
(1) ブロック中心が箱の内寸の内側、(2) 中心が縁の高さ未満、(3) 並進の速さがしきい値以下。
座標はワールド (= baseframe、sim)。単位は m、m/s。
"""

from dataclasses import dataclass
from enum import Enum

import numpy as np

from so101_vlm.eval.eval_config import JudgeConfig
from so101_vlm.eval.scene_spec import SCENE_SPEC, SceneSpec
from so101_vlm.safety._validation import as_vector

_N_AXES = 3


class JudgeReason(str, Enum):
    """【機能概要】: 判定が失敗した条件。🟡"""

    OUTSIDE_INNER = "outside_inner"  # (1) 中心が内寸の外 (margin 込み)
    ABOVE_RIM = "above_rim"  # (2) 中心が縁以上
    MOVING = "moving"  # (3) 速さがしきい値を超える


@dataclass(frozen=True)
class JudgeResult:
    """【機能概要】: 判定結果。🟡

    reasons: 失敗した条件を (1)(2)(3) の順に全部。成功なら空。
    offset_xy: 箱中心からのずれ [m]、height: 中心の z [m]、speed: 並進の速さ [m/s]。
    """

    success: bool
    reasons: tuple[JudgeReason, ...]
    offset_xy: tuple[float, float]
    height: float
    speed: float


def judge_in_box(
    centre: object,
    linear_velocity: object,
    config: JudgeConfig,
    spec: SceneSpec = SCENE_SPEC,
) -> JudgeResult:
    """【機能概要】: ブロックが箱の中で静止しているかを判定する 🔵

    【実装方針】: (1)(2) はちょうどで失敗 (厳密な不等号)、(3) はちょうどで成功 (FR-07 の「以下」)。
    NaN / inf / shape 違い / bool / config の型違いは ValueError。ブロックの傾きは使わない。
    """
    if not isinstance(config, JudgeConfig):
        raise ValueError(f"config は JudgeConfig で指定してください: {config!r}")
    pos = as_vector("centre", centre, _N_AXES)
    vel = as_vector("linear_velocity", linear_velocity, _N_AXES)
    dx = float(pos[0] - spec.box_center_xy[0])
    dy = float(pos[1] - spec.box_center_xy[1])
    limit = spec.box_inner_half_size - config.margin_m
    speed = float(np.linalg.norm(vel))
    reasons = []
    if not (abs(dx) < limit and abs(dy) < limit):
        reasons.append(JudgeReason.OUTSIDE_INNER)
    if not pos[2] < spec.box_rim_z:
        reasons.append(JudgeReason.ABOVE_RIM)
    if not speed <= config.speed_max_mps:
        reasons.append(JudgeReason.MOVING)
    return JudgeResult(not reasons, tuple(reasons), (dx, dy), float(pos[2]), speed)

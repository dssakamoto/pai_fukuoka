"""1 試行を通す run_trial (TASK-0024): 画像 -> VLM -> 知覚 (枠 -> 3 次元位置) -> IK -> 実行 -> 判定。

🔵 design.md 4.3〜4.5。FR-03 / FR-06: PlanError (VLM)、深度なし (知覚)、IK・作業範囲の拒否 (計画) では指令を送らず、
失敗の種類を記録する。VLM は VlmClient (plan(image, instruction)) なら何でもよい (本物でも OracleVlm でも)。
sim 専用の組み立て層。物理の部分 (計画・実行・判定) は fixed_pick.run_goal を共有する。
"""

import time
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from so101_vlm.eval.eval_config import EvalConfig
from so101_vlm.eval.failure import FailureKind
from so101_vlm.eval.fixed_pick import (
    COLOUR_TO_BODY,
    FixedPickOutcome,
    FrameSink,
    load_run_configs,
    run_goal,
)
from so101_vlm.eval.layout import place_blocks, sample_layout
from so101_vlm.eval.scene_spec import SCENE_SPEC
from so101_vlm.eval.validation import require_int
from so101_vlm.nodes.camera import render_rgbd
from so101_vlm.perception.calibration import Calibration, load_calibration
from so101_vlm.perception.projection import bbox_to_base_point
from so101_vlm.policy.base import Goal
from so101_vlm.policy.trajectory_config import TrajectoryConfig
from so101_vlm.robot.model_paths import PICKPLACE_SCENE_PATH
from so101_vlm.robot.sim import SimRobot
from so101_vlm.safety.config import SafetyConfig
from so101_vlm.vlm.base import VlmClient
from so101_vlm.vlm.schema import PlanError, PlanResult

_MM = 1000.0


@dataclass(frozen=True)
class TrialOutcome:
    """【機能概要】: 1 試行の結果。各段の出力を持つ。変更不可。🟡

    plan: VLM の出力 (VLM 失敗なら None)、pick_estimate / place_estimate: 知覚が出した目標 (ワールド = baseframe、
    ブロック中心 / 箱の底板上面の中心)、pick_error_mm: pick_estimate と真のブロック中心の距離 [mm] (AC-05)、
    run: 計画・実行・判定の結果 (動作前に拒否したら None。計画で拒否したときは planning_failure が入る)、
    failure: 成功なら None、error: VLM・知覚の失敗の理由、vlm_time_s: VLM 呼び出しの実時間。
    """

    seed: int
    colour: str
    instruction: str
    plan: PlanResult | None
    pick_estimate: tuple[float, float, float] | None
    place_estimate: tuple[float, float, float] | None
    pick_error_mm: float | None
    run: FixedPickOutcome | None
    failure: FailureKind | None
    error: str | None
    vlm_time_s: float

    @property
    def success(self) -> bool:
        """【機能概要】: 成功と判定されたか (動作前の拒否は False) 🔵"""
        return self.run is not None and self.run.success


def _failed(seed, colour, instruction, kind, error, vlm_time_s, plan=None) -> TrialOutcome:
    return TrialOutcome(
        seed=seed, colour=colour, instruction=instruction, plan=plan, pick_estimate=None,
        place_estimate=None, pick_error_mm=None, run=None, failure=kind, error=error,
        vlm_time_s=vlm_time_s,
    )  # fmt: skip


# 🟡 Intent: 深度は上面を見ているので、ブロック中心は表面の点から半辺ぶん下 (projection.bbox_to_base_point の注記)。
#    箱の位置は v1 では固定なので、置く高さは底板の上面 (SCENE_SPEC) を使い、xy だけ検出から求める。
def _estimate_goal(plan: PlanResult, depth: np.ndarray, cal: Calibration, instruction: str) -> Goal:
    pick = bbox_to_base_point(plan.pick.bbox_px, depth, cal.camera, cal.T_base_cam)
    pick[2] -= SCENE_SPEC.block_half_size
    place = bbox_to_base_point(plan.place.bbox_px, depth, cal.camera, cal.T_base_cam)
    place[2] = SCENE_SPEC.box_floor_top_z
    return Goal(pick_target_base=pick, place_target_base=place, instruction=instruction)


def run_trial(
    seed: int,
    colour: str,
    instruction: str,
    vlm: VlmClient,
    *,
    calibration: Calibration | None = None,
    scene_path: Path = PICKPLACE_SCENE_PATH,
    eval_config: EvalConfig | None = None,
    safety_config: SafetyConfig | None = None,
    trajectory_config: TrajectoryConfig | None = None,
    frame_sink: FrameSink | None = None,
) -> TrialOutcome:
    """【機能概要】: seed の配置を撮影し、VLM の検出から colour のブロックを箱に入れる 1 試行を実行する 🔵

    【実装方針】: 配置 -> 撮影 (RGB + 深度) -> vlm.plan -> 知覚 -> run_goal。colour は判定する対象 (真値) で、
    VLM には instruction だけを渡す。PlanError は vlm_failure、知覚の ValueError は perception_failure として
    記録し、動かさない。それ以外の例外 (認証エラーなど) は握りつぶさずそのまま上に出す。
    不正な seed / colour / instruction は、何かを始める前に ValueError。
    """
    seed = require_int("seed", seed, 0)
    if not isinstance(colour, str) or colour not in COLOUR_TO_BODY:
        raise ValueError(f"colour は {', '.join(COLOUR_TO_BODY)} のどれかで指定してください: {colour!r}")
    if not isinstance(instruction, str) or not instruction.strip():
        raise ValueError("instruction は空でない文字列で指定してください")
    configs = load_run_configs(eval_config, safety_config, trajectory_config)
    cal = load_calibration() if calibration is None else calibration
    place = configs.eval.placement
    with SimRobot(scene_path) as robot:
        model, data = robot.mujoco_handles()
        place_blocks(model, data, sample_layout(seed, place.n_blocks, place.area, place.min_gap, place.max_attempts))
        rgb, depth = render_rgbd(model, data, SCENE_SPEC.camera_name, cal.camera.width, cal.camera.height)
        truth = np.array(data.xpos[model.body(COLOUR_TO_BODY[colour]).id])
        started = time.perf_counter()
        try:
            plan = vlm.plan(rgb, instruction)
        except PlanError as exc:
            return _failed(seed, colour, instruction, FailureKind.VLM_FAILURE, str(exc), time.perf_counter() - started)
        vlm_time_s = time.perf_counter() - started
        try:
            goal = _estimate_goal(plan, depth, cal, instruction)
        except ValueError as exc:
            return _failed(seed, colour, instruction, FailureKind.PERCEPTION_FAILURE, str(exc), vlm_time_s, plan)
        run = run_goal(robot, goal, seed=seed, colour=colour, configs=configs, frame_sink=frame_sink)
    return TrialOutcome(
        seed=seed, colour=colour, instruction=instruction, plan=plan,
        pick_estimate=tuple(float(v) for v in goal.pick_target_base),
        place_estimate=tuple(float(v) for v in goal.place_target_base),
        pick_error_mm=float(np.linalg.norm(goal.pick_target_base - truth) * _MM),
        run=run, failure=run.failure, error=None, vlm_time_s=vlm_time_s,
    )  # fmt: skip

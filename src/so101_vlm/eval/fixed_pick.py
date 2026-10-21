"""固定位置の pick & place 1 試行 run_fixed_pick (TASK-0013)。VLM・知覚は使わない。

🔵 seed -> 配置 -> 真の位置 -> IkPolicy -> run_trajectory (毎周期 apply_safety) -> 判定 -> 失敗の分類。
計画 (TrajectoryError) と実行は分けて持つ。計画に失敗したら実行しない (FR-06)。
sim 専用の組み立て層 (SimRobot.mujoco_handles で真値を読む)。実行器・判定は MuJoCo に依存しない。
"""

import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType

import mujoco
import numpy as np

from so101_vlm.eval.eval_config import EvalConfig, load_eval_config
from so101_vlm.eval.failure import FailureKind, PickTrace, classify_failure
from so101_vlm.eval.judge import JudgeResult, judge_in_box
from so101_vlm.eval.layout import place_blocks, sample_layout
from so101_vlm.eval.pick_tracker import PickTracker, block_linear_velocity
from so101_vlm.eval.scene_spec import SCENE_SPEC
from so101_vlm.eval.validation import require_int
from so101_vlm.policy.base import Goal, Observation
from so101_vlm.policy.executor import ExecutionResult, run_trajectory
from so101_vlm.policy.ik_policy import IkPolicy
from so101_vlm.policy.trajectory import JointTrajectory, TrajectoryError
from so101_vlm.policy.trajectory_config import TrajectoryConfig, load_trajectory_config
from so101_vlm.robot.model_paths import PICKPLACE_SCENE_PATH
from so101_vlm.robot.sim import SimRobot
from so101_vlm.safety.config import SafetyConfig, load_safety_config
from so101_vlm.safety.limits import limits_from_model

# 【定数定義】: 色 -> ブロックの body 名 (SCENE_SPEC.block_names の順 = 赤・青・緑) 🔵
COLOUR_TO_BODY: Mapping[str, str] = MappingProxyType(
    dict(zip(("red", "blue", "green"), SCENE_SPEC.block_names, strict=True))
)

FrameSink = Callable[[mujoco.MjModel, mujoco.MjData, int, str], None]


@dataclass(frozen=True)
class FixedPickOutcome:
    """【機能概要】: 1 試行の結果。計画と実行を分けて持つ。変更不可。🟡

    pick_target: 真の位置 (ブロック中心、ワールド = baseframe)、planning_error / planning_waypoint:
    TrajectoryError の理由・経由点 (計画失敗のとき)、plan_time_s: 計画の実時間、
    execution / judge / trace: 計画失敗なら None、failure: 成功なら None、
    other_block_moved: 他のブロックが other_move_m を超えて動いたか (成功は取り消さない)。
    """

    seed: int
    colour: str
    pick_target: tuple[float, float, float]
    planning_error: str | None
    planning_waypoint: str | None
    plan_time_s: float
    execution: ExecutionResult | None
    judge: JudgeResult | None
    trace: PickTrace | None
    failure: FailureKind | None
    other_block_moved: bool

    @property
    def success(self) -> bool:
        """【機能概要】: 判定が成功か (計画失敗は False) 🟡"""
        return self.judge is not None and self.judge.success


def _validate_args(seed: object, colour: object) -> tuple[int, str]:
    """【機能概要】: seed (0 以上の整数) と colour (red / blue / green) を検証する。SimRobot より前に呼ぶ 🔵"""
    checked_seed = require_int("seed", seed, 0)
    if not isinstance(colour, str) or colour not in COLOUR_TO_BODY:
        raise ValueError(f"colour は {', '.join(COLOUR_TO_BODY)} のどれかで指定してください: {colour!r}")
    return checked_seed, colour


def _place_goal(data: mujoco.MjData, model: mujoco.MjModel, body: str) -> Goal:
    """【機能概要】: 真の位置 (data.xpos) と箱の底板の中心から Goal を作る 🔵 (TASK-0012 の D6)"""
    pick = np.array(data.xpos[model.body(body).id])
    place = (*SCENE_SPEC.box_center_xy, SCENE_SPEC.box_floor_top_z)
    return Goal(pick_target_base=pick, place_target_base=np.array(place))


@dataclass(frozen=True)
class _Configs:
    """【機能概要】: 1 試行で使う 3 つの設定をまとめる (引数の数を減らすための内部用)。🟡"""

    eval: EvalConfig
    safety: SafetyConfig
    trajectory: TrajectoryConfig


def _plan(
    robot: SimRobot, model: mujoco.MjModel, goal: Goal, configs: _Configs
) -> tuple[JointTrajectory | None, TrajectoryError | None, float]:
    """【機能概要】: IkPolicy で軌道を作る。TrajectoryError は捕まえて返す (実行しない) 🔵"""
    started = time.perf_counter()
    policy = IkPolicy(model, config=configs.trajectory, safety=configs.safety)
    try:
        trajectory = policy.act(Observation.from_state(robot.get_state()), goal)
    except TrajectoryError as error:
        return None, error, time.perf_counter() - started
    return trajectory, None, time.perf_counter() - started


def _execute(
    robot: SimRobot, trajectory: JointTrajectory, body: str, configs: _Configs,
    frame_sink: FrameSink | None,
) -> tuple[ExecutionResult, PickTrace]:
    """【機能概要】: 軌道を実行し、毎周期の追跡と frame_sink 呼び出しを行う 🔵"""
    model, data = robot.mujoco_handles()
    tracker = PickTracker(model, data, body, configs.trajectory.tcp_offset)

    def on_step(index: int, label: str) -> None:
        tracker.update(label)
        if frame_sink is not None:
            frame_sink(model, data, index, label)

    lower, upper = limits_from_model(model)
    result = run_trajectory(
        robot, trajectory, configs.safety, lower=lower, upper=upper,
        settle_s=configs.eval.judge.settle_s, on_step=on_step,
    )  # fmt: skip
    return result, tracker.finish()


def _judge_block(
    model: mujoco.MjModel, data: mujoco.MjData, body: str, eval_config: EvalConfig
) -> JudgeResult:
    """【機能概要】: 実行後のブロックを判定する 🔵

    【注意 (TASK-0015 へ引き継ぎ)】: data.xpos は mj_step が積分の前に計算した値なので、最後の mj_step の
    1 サブステップ分 (約 2 ms) だけ古い可能性がある。qvel は積分後の値。静止後は差が無視できるため
    TASK-0013 では mj_forward を呼ばずそのまま使う (読み取り専用を保つ)。厳密にするなら TASK-0015 で検討する。
    """
    body_id = int(model.body(body).id)
    velocity = block_linear_velocity(model, data, body_id)
    return judge_in_box(data.xpos[body_id], velocity, eval_config.judge)


def _outcome_planning_failed(
    seed: int, colour: str, goal: Goal, error: TrajectoryError, plan_time_s: float
) -> FixedPickOutcome:
    """【機能概要】: 計画失敗の結果 (実行・判定・追跡は None、failure は planning_failure) 🔵"""
    return FixedPickOutcome(
        seed=seed, colour=colour, pick_target=tuple(float(v) for v in goal.pick_target_base),
        planning_error=error.reason, planning_waypoint=error.waypoint, plan_time_s=plan_time_s,
        execution=None, judge=None, trace=None, failure=FailureKind.PLANNING_FAILURE,
        other_block_moved=False,
    )  # fmt: skip


def run_fixed_pick(
    seed: int,
    colour: str = "red",
    *,
    scene_path: Path = PICKPLACE_SCENE_PATH,
    eval_config: EvalConfig | None = None,
    safety_config: SafetyConfig | None = None,
    trajectory_config: TrajectoryConfig | None = None,
    frame_sink: FrameSink | None = None,
) -> FixedPickOutcome:
    """【機能概要】: seed の配置で colour のブロックを、真の位置から箱に入れる 1 試行を実行する 🔵

    【実装方針】: 手順は配置 -> 真の位置 -> 計画 -> 実行 -> 判定 -> 分類。計画失敗は実行しない
    (frame_sink も呼ばない)。frame_sink(model, data, 周期の番号, 区間名) は毎周期 (待ちを含む)、
    追跡の更新の後に呼ぶ。不正な colour / seed は SimRobot を作る前に ValueError。
    """
    seed, colour = _validate_args(seed, colour)
    configs = _Configs(
        eval=load_eval_config() if eval_config is None else eval_config,
        safety=load_safety_config() if safety_config is None else safety_config,
        trajectory=load_trajectory_config() if trajectory_config is None else trajectory_config,
    )
    body = COLOUR_TO_BODY[colour]
    place = configs.eval.placement
    with SimRobot(scene_path) as robot:
        model, data = robot.mujoco_handles()
        layout = sample_layout(seed, place.n_blocks, place.area, place.min_gap, place.max_attempts)
        place_blocks(model, data, layout)
        goal = _place_goal(data, model, body)
        trajectory, error, plan_time_s = _plan(robot, model, goal, configs)
        if error is not None:
            return _outcome_planning_failed(seed, colour, goal, error, plan_time_s)
        execution, trace = _execute(robot, trajectory, body, configs, frame_sink)
        judge = _judge_block(model, data, body, configs.eval)
    return FixedPickOutcome(
        seed=seed, colour=colour, pick_target=tuple(float(v) for v in goal.pick_target_base),
        planning_error=None, planning_waypoint=None, plan_time_s=plan_time_s,
        execution=execution, judge=judge, trace=trace,
        failure=classify_failure(trace, judge, configs.eval.failure),
        other_block_moved=trace.max_other_move > configs.eval.failure.other_move_m,
    )  # fmt: skip

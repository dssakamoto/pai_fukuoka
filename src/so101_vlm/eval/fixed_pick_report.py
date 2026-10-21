"""固定位置 pick & place の結果の整形 (TASK-0013 Refactor)。

🟡 FixedPickOutcome -> JSON 用 dict / 1 試行 1 行 / 集計行。純粋関数 (入力を変更しない)。
scripts/run_fixed_pick.py と、TASK-0026 の集計から共有する。
"""

from collections.abc import Sequence

from so101_vlm.eval.fixed_pick import FixedPickOutcome
from so101_vlm.policy.executor import FLAG_NAMES


def _planning_flag_counts(outcome: FixedPickOutcome) -> dict[str, int]:
    """【機能概要】: 安全フィルタのフラグの回数。計画失敗 (未実行) は全て 0 🟡"""
    if outcome.execution is None:
        return dict.fromkeys(FLAG_NAMES, 0)
    return {name: int(outcome.execution.flag_counts[name]) for name in FLAG_NAMES}


def outcome_to_dict(outcome: FixedPickOutcome) -> dict:
    """【機能概要】: 結果を JSON にできる dict にする (必須キー: seed, colour, success, failure, planning_error, flag_counts)🟡"""
    execution, judge = outcome.execution, outcome.judge
    return {
        "seed": outcome.seed,
        "colour": outcome.colour,
        "pick_target": list(outcome.pick_target),
        "success": outcome.success,
        "failure": None if outcome.failure is None else outcome.failure.value,
        "planning_error": outcome.planning_error,
        "planning_waypoint": outcome.planning_waypoint,
        "flag_counts": _planning_flag_counts(outcome),
        "aborted": None if execution is None else execution.aborted,
        "other_block_moved": outcome.other_block_moved,
        "plan_time_s": outcome.plan_time_s,
        "duration_s": None if execution is None else execution.duration_s,
        "wall_time_s": None if execution is None else execution.wall_time_s,
        "final_joint_error": None if execution is None else execution.final_joint_error,
        "judge_reasons": None if judge is None else [r.value for r in judge.reasons],
        "offset_xy": None if judge is None else list(judge.offset_xy),
        "height": None if judge is None else judge.height,
        "speed": None if judge is None else judge.speed,
    }


def format_trial_line(outcome: FixedPickOutcome) -> str:
    """【機能概要】: 1 試行 1 行の標準出力 (seed、色、x, y、成否、分類、フラグ数、計画・実行時間)🟡"""
    x, y, _ = outcome.pick_target
    head = f"seed={outcome.seed} {outcome.colour} x={x:.3f} y={y:.3f}"
    if outcome.execution is None:
        return (
            f"{head} 計画失敗: {outcome.planning_error} (経由点 {outcome.planning_waypoint}) "
            f"計画 {outcome.plan_time_s:.2f}s"
        )
    flags = sum(_planning_flag_counts(outcome).values())
    result = "成功" if outcome.success else f"失敗: {outcome.failure.value}"
    return (
        f"{head} {result} フラグ {flags} 回 計画 {outcome.plan_time_s:.2f}s "
        f"実行 {outcome.execution.duration_s:.1f}s"
    )


def format_summary(outcomes: Sequence[FixedPickOutcome]) -> str:
    """【機能概要】: 集計行 (試行数、計画失敗数、計画可能な配置での成功数と率)🟡"""
    planned = [o for o in outcomes if o.execution is not None]
    succeeded = sum(o.success for o in planned)
    rate = f"{succeeded / len(planned):.1%}" if planned else "-"
    return (
        f"集計: 試行 {len(outcomes)} 件, 計画失敗 {len(outcomes) - len(planned)}, "
        f"計画可能な配置での成功 {succeeded}/{len(planned)} ({rate})"
    )

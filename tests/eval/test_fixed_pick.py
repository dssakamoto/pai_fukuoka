"""run_fixed_pick / FixedPickOutcome の統合テスト (TASK-0013, Red フェーズ)。

テストケース定義: docs/implements/so101-vlm-pickplace/TASK-0013/pick-testcases.md 7 節 (TC-FP-*)
対象モジュール (まだ存在しない): so101_vlm.eval.fixed_pick (run_fixed_pick, FixedPickOutcome)
実シーン PICKPLACE_SCENE_PATH (物理設定あり) で、真の位置から軌道を作り、安全フィルタ経由で
SimRobot に送り、判定と失敗の分類まで行う。VLM・知覚は使わない。
実行時間の予算: < 4 s (赤 seed 0〜2 を module scope で 1 回、計画失敗 2 件、決定性の再実行、設定の注入)。
信頼性: 🔵 要件・実測に基づく / 🟡 妥当な推測 / 🔴 要件にない仮定
"""

import dataclasses

import mujoco
import numpy as np
import pytest

import so101_vlm.eval.fixed_pick as fixed_pick_module
from so101_vlm.eval.eval_config import JudgeConfig, load_eval_config
from so101_vlm.eval.failure import FailureKind
from so101_vlm.eval.fixed_pick import FixedPickOutcome, run_fixed_pick
from so101_vlm.eval.judge import JudgeReason
from so101_vlm.eval.layout import sample_layout

PLANNABLE_RED_SEEDS = (0, 1, 2)
PLAN_FAIL_CASES = [(16, "blue", "self_collision", "grasp"), (5, "green", "not_converged", "pre_grasp")]
JOINT_ERR_TOL = 0.01
DT = 0.02
SETTLE_STEPS = 50
OFFSET_MAX_M = 0.016  # 実測の最大 15.2 mm
BLOCK_REST_Z = 0.015


class FrameRecorder:
    """frame_sink の呼び出し (周期の番号と区間名) を記録する。"""

    def __init__(self):
        self.calls: list[tuple[int, str]] = []
        self.types_ok = True

    def __call__(self, model, data, i, label):
        self.types_ok &= isinstance(model, mujoco.MjModel) and isinstance(data, mujoco.MjData)
        self.calls.append((i, label))


@pytest.fixture(scope="module")
def red_runs():
    """【テスト前準備】: 赤 seed 0〜2 を 1 回ずつ実行して共有する (約 1.5 s)。"""
    runs = {}
    for seed in PLANNABLE_RED_SEEDS:
        recorder = FrameRecorder()
        runs[seed] = (run_fixed_pick(seed, "red", frame_sink=recorder), recorder)
    return runs


# =============================================================== 正常系
@pytest.mark.parametrize("seed", PLANNABLE_RED_SEEDS)
def test_red_seeds_succeed_without_safety_flags(red_runs, seed):
    """TC-FP-01 赤 seed 0〜2 で成功、フラグ 0、関節誤差 < 0.01 🔵"""
    out = red_runs[seed][0]
    assert isinstance(out, FixedPickOutcome)
    assert out.planning_error is None and out.planning_waypoint is None  # 【確認内容】: 計画できた
    assert out.success is True and out.failure is None  # 【確認内容】: 箱に入った
    assert out.judge.reasons == ()
    assert out.execution.has_safety_flags is False and out.execution.aborted is False
    assert out.execution.final_joint_error < JOINT_ERR_TOL  # 【確認内容】: 実測 0.0005 rad
    assert out.other_block_moved is False
    assert out.colour == "red" and out.seed == seed


@pytest.mark.parametrize("seed", PLANNABLE_RED_SEEDS)
def test_execution_numbers(red_runs, seed):
    """TC-FP-02 実行の数値 🔵"""
    out = red_runs[seed][0]
    ex = out.execution
    assert ex.settle_steps == SETTLE_STEPS  # 【確認内容】: ceil(1.0 / 0.02)
    assert ex.duration_s == pytest.approx((ex.n_commands + SETTLE_STEPS) * DT)
    assert 240 <= ex.n_commands <= 330  # 【確認内容】: 実測 273 / 258 / 296 (TASK-0012 は 250〜325)
    assert out.plan_time_s > 0.0


@pytest.mark.parametrize("seed", PLANNABLE_RED_SEEDS)
def test_trace_summary(red_runs, seed):
    """TC-FP-03 追跡の要約 (PickTrace) 🔵"""
    tr = red_runs[seed][0].trace
    assert tr.lift_end_rise > 0.03  # 【確認内容】: 実測 0.074〜0.076
    assert tr.max_separation_carry < 0.02  # 【確認内容】: 実測 ≤ 0.0160
    assert tr.max_separation_place < 0.02  # 【確認内容】: 実測 ≤ 0.0162
    assert tr.max_other_move < 0.005  # 【確認内容】: 実測 2e-10
    assert tr.final_tilt_deg < 1.0


@pytest.mark.parametrize("seed", PLANNABLE_RED_SEEDS)
def test_judge_values_and_true_position(red_runs, seed):
    """TC-FP-04 判定の値と真の位置 (配置の経路は既存のまま) 🔵"""
    out = red_runs[seed][0]
    red_xy = sample_layout(seed, 3, 0.20, 0.05, 10000)[0]
    assert out.pick_target[:2] == pytest.approx(red_xy, abs=1e-9)  # 【確認内容】: 真の位置 (x, y)
    assert out.pick_target[2] == pytest.approx(BLOCK_REST_Z, abs=0.002)
    assert max(abs(v) for v in out.judge.offset_xy) < OFFSET_MAX_M  # 【確認内容】: 箱中心からのずれ
    assert out.judge.height == pytest.approx(0.0199, abs=0.001)
    assert out.judge.speed <= 0.01


@pytest.mark.parametrize(("seed", "colour", "reason", "waypoint"), PLAN_FAIL_CASES)
def test_planning_failure_is_rejected_without_execution(monkeypatch, seed, colour, reason, waypoint):
    """TC-FP-05 計画失敗は実行しない (FR-06 の正常な拒否) 🔵"""

    def must_not_run(*args, **kwargs):
        raise AssertionError("計画失敗なのに run_trajectory が呼ばれた")

    monkeypatch.setattr(fixed_pick_module, "run_trajectory", must_not_run)
    recorder = FrameRecorder()
    out = run_fixed_pick(seed, colour, frame_sink=recorder)
    assert out.planning_error == reason and out.planning_waypoint == waypoint  # 【確認内容】: 実測の理由
    assert out.execution is None and out.judge is None and out.trace is None
    assert out.failure == FailureKind.PLANNING_FAILURE and out.success is False
    assert out.other_block_moved is False
    assert out.plan_time_s > 0.0
    assert out.pick_target[0] < 0.17  # 【確認内容】: 実測 0.1688 / 0.1608 (肩に近い配置)
    assert recorder.calls == []  # 【確認内容】: 動かさないので frame_sink も呼ばない


def test_planning_and_execution_results_are_separate(red_runs):
    """TC-FP-06 計画と実行の結果を分けて持つ 🟡"""
    out = red_runs[0][0]
    assert None not in (out.execution, out.judge, out.trace)  # 【確認内容】: 実行側は全て値あり
    assert out.planning_error is None and out.planning_waypoint is None
    assert not hasattr(out.execution, "planning_error")  # 【確認内容】: ExecutionResult は計画を知らない
    assert not hasattr(out.execution, "plan_time_s")


def test_frame_sink_is_called_every_cycle(red_runs):
    """TC-FP-07 frame_sink は毎周期呼ばれる 🟡"""
    out, recorder = red_runs[0]
    total = out.execution.n_commands + out.execution.settle_steps  # 実測 323
    assert [i for i, _ in recorder.calls] == list(range(1, total + 1))  # 【確認内容】: 1 始まりの通し番号
    assert all(label == "settle" for _, label in recorder.calls[-SETTLE_STEPS:])  # 【確認内容】: 最後の待ち
    assert recorder.types_ok  # 【確認内容】: model / data は MuJoCo のオブジェクト


def test_same_seed_gives_same_result(red_runs):
    """TC-FP-08 決定性 (plan_time_s と wall_time_s は比べない) 🔵"""
    first = red_runs[0][0]
    second = run_fixed_pick(0, "red")
    assert first.judge == second.judge and first.trace == second.trace
    assert first.execution.n_commands == second.execution.n_commands
    assert np.array_equal(first.execution.final_command, second.execution.final_command)
    assert first.execution.final_joint_error == second.execution.final_joint_error  # 【確認内容】: ビット単位


def test_injected_judge_config_changes_classification():
    """TC-FP-09 設定の注入で判定が変わる (分類の経路) 🟡"""
    # 【テストデータ準備】: margin 0.039 → 内側の限界 1 mm に対し、実測のずれ (-2.3, -4.3) mm
    strict = dataclasses.replace(
        load_eval_config(), judge=JudgeConfig(margin_m=0.039, speed_max_mps=0.01, settle_s=1.0)
    )
    out = run_fixed_pick(0, "red", eval_config=strict)
    assert out.success is False
    assert out.judge.reasons == (JudgeReason.OUTSIDE_INNER,)
    assert out.failure == FailureKind.NOT_IN_BOX  # 【確認内容】: トレースは正常値なので not_in_box
    assert out.execution.has_safety_flags is False


def test_outcome_is_immutable(red_runs):
    """TC-FP-10 結果は変更できない 🔵"""
    out = red_runs[0][0]
    with pytest.raises(dataclasses.FrozenInstanceError):
        out.seed = 1
    with pytest.raises(dataclasses.FrozenInstanceError):
        out.trace.lift_end_rise = 0.0
    with pytest.raises(dataclasses.FrozenInstanceError):
        out.execution.n_commands = 0
    assert out.execution.final_command.flags.writeable is False
    assert isinstance(out.pick_target, tuple)


# =============================================================== 異常系
@pytest.mark.parametrize("colour", ["purple", "RED", ""])
def test_invalid_colour_raises_before_building_robot(monkeypatch, colour):
    """TC-FP-E01 色の不正 → ValueError (SimRobot を作る前に失敗) 🔵"""

    def must_not_build(*args, **kwargs):
        raise AssertionError("SimRobot を作ってしまった")

    monkeypatch.setattr(fixed_pick_module, "SimRobot", must_not_build)
    with pytest.raises(ValueError):
        run_fixed_pick(0, colour)


@pytest.mark.parametrize("seed", [-1, True, 1.5, "0"])
def test_invalid_seed_raises(seed):
    """TC-FP-E02 seed の不正 → ValueError (require_int) 🔵"""
    with pytest.raises(ValueError):
        run_fixed_pick(seed, "red")

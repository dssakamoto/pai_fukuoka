"""動画の組み立て (VideoWriter / FrameCollector / clip_trial) のテスト (TASK-0028, Red)。実際に sim を回して mp4 を書く。"""

import imageio.v3 as iio
import numpy as np
import pytest

from so101_vlm.eval.failure import FailureKind
from so101_vlm.eval.layout import sample_layout
from so101_vlm.eval.oracle_vlm import OracleVlm
from so101_vlm.eval.video import clip_duration_s, make_style
from so101_vlm.eval.video_clip import VideoWriter, clip_trial, summary_lines
from so101_vlm.eval.runner import summarize
from so101_vlm.perception.calibration import load_calibration
from so101_vlm.vlm.schema import PlanError

INSTRUCTION = "put the red block into the box"
STYLE = make_style(lang="en")
HOLDS = {"hold_task_s": 0.2, "hold_detect_s": 0.2, "hold_result_s": 0.2}


def _oracle(seed, colour="red"):
    return OracleVlm(sample_layout(seed), colour, load_calibration())


@pytest.fixture(scope="module")
def clip(tmp_path_factory):
    path = tmp_path_factory.mktemp("video") / "trial.mp4"
    with VideoWriter(path, fps=30) as video:
        outcome = clip_trial(video, 2, "red", INSTRUCTION, _oracle(2), speed=6, style=STYLE, trial_label="trial 1/1", **HOLDS)
        n_frames = video.n_frames
    return path, outcome, n_frames


@pytest.mark.render
def test_one_trial_becomes_a_playable_mp4(clip):
    path, outcome, n_frames = clip
    assert outcome.success and path.exists() and path.stat().st_size > 1000
    frames = iio.imread(path)  # (N, H, W, 3)
    assert frames.shape[0] == n_frames and frames.shape[1:] == (480, 640, 3)


@pytest.mark.render
def test_frame_count_follows_speed_and_holds(clip):
    _, outcome, n_frames = clip
    n_src = outcome.run.execution.n_commands + outcome.run.execution.settle_steps
    held = 3 * round(0.2 * 30)
    # 6 倍速: 30 / 6 = 5 fps で間引く (50 Hz -> 5 Hz: 10 周期に 1 回)
    assert abs(n_frames - held - n_src / 10) <= 2


@pytest.mark.render
def test_video_shows_the_boxes_and_the_result_banner(clip):
    path, _, _ = clip
    frames = iio.imread(path)
    # 結果の帯 (緑) は最後のフレームの下寄りに出る (mp4 は劣化するので、緑が優勢かだけを見る)
    region = frames[-1][int(0.62 * 480) : int(0.78 * 480), int(0.25 * 640) : int(0.75 * 640)].reshape(-1, 3).mean(axis=0)
    assert region[1] > region[0] + 15 and region[1] > region[2] + 15
    # 検出の段階 (先頭の少し後) には、検出枠の色 (マゼンタ) の画素がある
    detect = frames[int(0.2 * 30) + 2].astype(int)
    magenta = (detect[..., 0] > 200) & (detect[..., 1] < 80) & (detect[..., 2] > 200)
    assert magenta.sum() > 20


@pytest.mark.render
def test_vlm_failure_still_makes_a_short_clip_with_a_failed_result(tmp_path):
    class Failing:
        def plan(self, image, instruction):
            raise PlanError("no purple block")

    path = tmp_path / "f.mp4"
    with VideoWriter(path) as video:
        outcome = clip_trial(video, 2, "red", INSTRUCTION, Failing(), speed=2, style=STYLE, **HOLDS)
        assert outcome.failure is FailureKind.VLM_FAILURE
        assert video.n_frames == 3 * round(0.2 * 30)  # 動かないので、指示・検出・判定の静止画だけ
    frames = iio.imread(path)
    region = frames[-1][int(0.62 * 480) : int(0.78 * 480), int(0.25 * 640) : int(0.75 * 640)].reshape(-1, 3).mean(axis=0)
    assert region[0] > region[1] + 15  # 赤い帯


@pytest.mark.render
def test_unexpected_errors_propagate_and_the_writer_stays_usable(tmp_path):
    class Exploding:
        def plan(self, image, instruction):
            raise RuntimeError("401 unauthorized")

    with VideoWriter(tmp_path / "e.mp4") as video:
        with pytest.raises(RuntimeError, match="401"):
            clip_trial(video, 2, "red", INSTRUCTION, Exploding(), speed=2, style=STYLE, **HOLDS)
        assert video.n_frames == 0  # 途中の試行のフレームは書かない


def test_bad_speed_is_refused_before_running(tmp_path):
    with VideoWriter(tmp_path / "b.mp4") as video:
        with pytest.raises(ValueError):
            clip_trial(video, 2, "red", INSTRUCTION, _oracle(2), speed=7, style=STYLE)
        assert video.n_frames == 0


def test_writer_rules(tmp_path):
    video = VideoWriter(tmp_path / "w.mp4", fps=30)
    with pytest.raises(RuntimeError):
        video.append(np.zeros((48, 64, 3), np.uint8))  # with の外
    with video:
        video.append(np.zeros((48, 64, 3), np.uint8))
        with pytest.raises(ValueError):
            video.append(np.zeros((48, 32, 3), np.uint8))  # サイズが変わる
        with pytest.raises(ValueError):
            video.append(np.zeros((48, 64), np.uint8))
        assert video.n_frames == 1 and video.duration_s == clip_duration_s(1, 30)


def test_summary_lines():
    from so101_vlm.eval.runner import TrialRecord

    def rec(i, ok, failure=None):
        return TrialRecord(
            trial=i, seed=i, colour="red", instruction="x", provider="oracle", model="m", success=ok, failure=failure,
            failure_group=None if ok else "ik", error=None, planning_error=None, pick_error_mm=5.0, pick_label="",
            pick_xyz="", pick_bbox="", place_bbox="", vlm_time_s=0.0, plan_time_s=0.0, total_time_s=1.0,
            other_block_moved=False,
        )

    lines = summary_lines(summarize([rec(0, True), rec(1, True), rec(2, False, "planning_failure")]), "en")
    assert lines[0] == "Success rate: 66.7% (2/3)" and any("planning_failure" in x for x in lines)
    assert summary_lines(summarize([rec(0, True)]), "ja")[0] == "成功率: 100.0% (1/1)"

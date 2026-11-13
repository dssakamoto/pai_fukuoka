"""SimEpisodeRecorder / record_trial のテスト (TASK-0027, Red)。偽 VLM で 1 試行を実際に回して記録する。"""

import dataclasses
import json

import numpy as np
import pytest

pytest.importorskip("lerobot", reason="lerobot が入っていない (uv sync --group record)")

from so101_vlm.eval.failure import FailureKind  # noqa: E402
from so101_vlm.eval.layout import sample_layout  # noqa: E402
from so101_vlm.eval.oracle_vlm import OracleVlm  # noqa: E402
from so101_vlm.perception.calibration import load_calibration  # noqa: E402
from so101_vlm.record.lerobot_writer import LeRobotWriter, load_lerobot, load_record_config  # noqa: E402
from so101_vlm.record.sim_episode import keep_frame, record_trial  # noqa: E402
from so101_vlm.vlm.schema import PlanError  # noqa: E402

INSTRUCTION = "put the red block into the box"
DT = 0.02


def _light_writer(root):
    """描画が遅い (この環境では約 0.12 s/枚、解像度によらない) ので、画像の中身を見ないテストは 10 fps で回す。"""
    cfg = dataclasses.replace(load_record_config(), fps=10)
    return LeRobotWriter(root, config=cfg, use_videos=False, image_shape=(120, 160, 3))


def _oracle(seed, colour="red"):
    return OracleVlm(sample_layout(seed), colour, load_calibration())


def _reader(root):
    return load_lerobot()(load_record_config().repo_id, root=root)


@pytest.fixture(scope="module")
def recorded(tmp_path_factory):
    root = tmp_path_factory.mktemp("rec") / "ds"
    with LeRobotWriter(root, use_videos=False) as w:
        outcome = record_trial(w, 2, "red", INSTRUCTION, _oracle(2))
    return root, outcome


@pytest.mark.render
def test_one_trial_becomes_one_episode(recorded):
    root, outcome = recorded
    assert outcome.success
    ds = _reader(root)
    assert ds.num_episodes == 1 and ds.fps == 30
    n_src = outcome.run.execution.n_commands + outcome.run.execution.settle_steps
    expected = sum(keep_frame(k, DT, 30) for k in range(1, n_src + 1))
    assert len(ds) == expected and 150 < len(ds) < 400
    assert ds[0]["task"] == INSTRUCTION


@pytest.mark.render
def test_frame_contents_are_sensible(recorded):
    root, _ = recorded
    ds = _reader(root)
    first, last = ds[0], ds[len(ds) - 1]
    s0, a0 = first["observation.state"].numpy(), first["action"].numpy()
    assert np.max(np.abs(s0 - a0)) < 0.2  # 開始直後: 行動 (次の目標) は現在の姿勢の近く
    assert np.max(np.abs(last["action"].numpy() - last["observation.state"].numpy())) < 0.1  # 終わりは静止
    img = first["observation.images.top"].numpy()
    assert img.shape == (3, 480, 640) and 0.0 <= img.min() and img.max() <= 1.0 and img.std() > 0.05
    states = np.stack([ds[i]["observation.state"].numpy() for i in range(0, len(ds), 10)])
    assert np.ptp(states, axis=0).max() > 0.5  # 腕が実際に動いている


@pytest.mark.render
def test_action_leads_the_state(recorded):
    # 位置制御なので、行動 (目標) は次の状態に向かう: 行動と次フレームの状態の差は、同じフレームの状態との差より小さい
    root, _ = recorded
    ds = _reader(root)
    moving = [i for i in range(len(ds) - 1) if np.abs(ds[i]["action"].numpy() - ds[i]["observation.state"].numpy()).max() > 0.02]
    assert moving
    now = np.mean([np.abs(ds[i]["action"].numpy() - ds[i]["observation.state"].numpy()).mean() for i in moving[:40]])
    nxt = np.mean([np.abs(ds[i]["action"].numpy() - ds[i + 1]["observation.state"].numpy()).mean() for i in moving[:40]])
    assert nxt < now


@pytest.mark.render
def test_sidecar_describes_the_trial(recorded):
    root, outcome = recorded
    (line,) = [json.loads(x) for x in (root / "so101_trials.jsonl").read_text(encoding="utf-8").splitlines()]
    assert line["episode_index"] == 0 and line["seed"] == 2 and line["colour"] == "red"
    assert line["success"] is True and line["failure"] is None and line["instruction"] == INSTRUCTION
    assert line["pick_error_mm"] == pytest.approx(outcome.pick_error_mm)
    assert line["plan"]["pick"]["label"] == "red block" and len(line["plan"]["pick"]["bbox_px"]) == 4


@pytest.mark.render
def test_failure_before_motion_records_no_episode_but_logs_the_trial(tmp_path):
    class Failing:
        def plan(self, image, instruction):
            raise PlanError("no purple block")

    root = tmp_path / "ds"
    with _light_writer(root) as w:
        outcome = record_trial(w, 2, "red", INSTRUCTION, Failing())
        assert outcome.failure is FailureKind.VLM_FAILURE
        # 動かなかった試行の後でも、次の試行を普通に記録できる
        assert record_trial(w, 2, "red", INSTRUCTION, _oracle(2)).success
    assert _reader(root).num_episodes == 1
    lines = [json.loads(x) for x in (root / "so101_trials.jsonl").read_text(encoding="utf-8").splitlines()]
    assert [(x["failure"], x["episode_index"]) for x in lines] == [("vlm_failure", None), (None, 0)]


@pytest.mark.render
def test_unexpected_errors_discard_the_partial_episode(tmp_path):
    class Exploding:
        def plan(self, image, instruction):
            raise RuntimeError("401 unauthorized")

    root = tmp_path / "ds"
    with _light_writer(root) as w:
        with pytest.raises(RuntimeError, match="401"):
            record_trial(w, 2, "red", INSTRUCTION, Exploding())
        assert record_trial(w, 2, "red", INSTRUCTION, _oracle(2)).success
    assert _reader(root).num_episodes == 1

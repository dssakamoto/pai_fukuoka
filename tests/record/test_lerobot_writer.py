"""LeRobotWriter のテスト (TASK-0027, Red)。保存して LeRobot で読み戻す。lerobot が無ければスキップ (uv sync --group record)。"""

import json

import numpy as np
import pytest

pytest.importorskip("lerobot", reason="lerobot が入っていない (uv sync --group record)")

from so101_vlm.record.lerobot_writer import LeRobotWriter, load_lerobot, load_record_config  # noqa: E402

H, W = 24, 32
TASK = "put the red block into the box"


def _open(root, **kw):
    return LeRobotWriter(root, image_shape=(H, W, 3), use_videos=kw.pop("use_videos", False), **kw)


def _frames(n, ep=0, rng_seed=0):
    rng = np.random.default_rng(rng_seed)
    return [
        (
            rng.integers(0, 256, (H, W, 3), dtype=np.uint8),
            np.full(6, i + 10 * ep, np.float32),
            np.full(6, i + 0.5, np.float32),
        )
        for i in range(n)
    ]


def _read(root):
    LeRobotDataset = load_lerobot()
    return LeRobotDataset(load_record_config().repo_id, root=root)


def test_save_and_read_back_frames_and_values(tmp_path):
    root = tmp_path / "ds"
    frames = _frames(6)
    with _open(root) as w:
        for image, state, action in frames:
            w.add_frame(image, state, action, TASK)
        assert w.save_episode({"seed": 3}) == 0
    ds = _read(root)
    assert len(ds) == 6 and ds.num_episodes == 1 and ds.fps == 30
    for i, (image, state, action) in enumerate(frames):
        item = ds[i]
        assert item["task"] == TASK and int(item["episode_index"]) == 0 and int(item["frame_index"]) == i
        assert np.allclose(item["observation.state"].numpy(), state) and np.allclose(item["action"].numpy(), action)
        chw = item["observation.images.top"].numpy()  # (3, H, W) の 0〜1
        assert chw.shape == (3, H, W) and np.allclose(chw, image.transpose(2, 0, 1) / 255.0, atol=1e-6)


def test_features_and_robot_type_are_recorded(tmp_path):
    with _open(tmp_path / "ds") as w:
        for image, state, action in _frames(2):
            w.add_frame(image, state, action, TASK)
        w.save_episode()
    ds = _read(tmp_path / "ds")
    assert ds.features["observation.state"]["names"][0] == "shoulder_pan.pos"
    assert tuple(ds.features["observation.images.top"]["shape"]) == (H, W, 3)
    assert ds.meta.robot_type == "so101"


def test_two_episodes_get_consecutive_indices(tmp_path):
    root = tmp_path / "ds"
    with _open(root) as w:
        for ep in range(2):
            for image, state, action in _frames(4, ep):
                w.add_frame(image, state, action, TASK)
            assert w.save_episode() == ep
        assert w.episodes_saved == 2
    ds = _read(root)
    assert len(ds) == 8 and ds.num_episodes == 2
    assert [int(ds[i]["episode_index"]) for i in range(8)] == [0, 0, 0, 0, 1, 1, 1, 1]


def test_video_mode_roundtrip(tmp_path):
    root = tmp_path / "ds"
    with _open(root, use_videos=True) as w:
        for image, state, action in _frames(4):
            w.add_frame(image, state, action, TASK)
        w.save_episode()
    ds = _read(root)
    assert len(ds) == 4 and ds[2]["observation.images.top"].shape == (3, H, W)
    assert np.allclose(ds[2]["observation.state"].numpy(), 2.0)
    assert any(root.rglob("*.mp4"))


def test_sidecar_log_has_one_line_per_trial_including_skipped(tmp_path):
    root = tmp_path / "ds"
    with _open(root) as w:
        for image, state, action in _frames(3):
            w.add_frame(image, state, action, TASK)
        w.save_episode({"seed": 1, "success": True})
        w.log_skipped({"seed": 2, "success": False, "failure": "vlm_failure"})
    lines = [json.loads(x) for x in (root / "so101_trials.jsonl").read_text(encoding="utf-8").splitlines()]
    assert lines == [
        {"seed": 1, "success": True, "episode_index": 0},
        {"seed": 2, "success": False, "failure": "vlm_failure", "episode_index": None},
    ]


def test_existing_root_is_refused_and_left_alone(tmp_path):
    root = tmp_path / "ds"
    root.mkdir()
    (root / "keep.txt").write_text("x", encoding="utf-8")
    with pytest.raises(FileExistsError, match="既に"):
        _open(root)
    assert (root / "keep.txt").read_text(encoding="utf-8") == "x"


def test_discard_drops_the_pending_episode(tmp_path):
    root = tmp_path / "ds"
    with _open(root) as w:
        for image, state, action in _frames(3):
            w.add_frame(image, state, action, TASK)
        w.discard_episode()
        for image, state, action in _frames(2, 1):
            w.add_frame(image, state, action, TASK)
        assert w.save_episode() == 0
    assert len(_read(root)) == 2


def test_finalize_is_idempotent_and_blocks_further_writes(tmp_path):
    w = _open(tmp_path / "ds")
    image, state, action = _frames(1)[0]
    w.add_frame(image, state, action, TASK)
    w.save_episode()
    w.finalize()
    w.finalize()
    with pytest.raises(RuntimeError, match="finalize"):
        w.add_frame(image, state, action, TASK)


def test_save_without_frames_is_an_error(tmp_path):
    with _open(tmp_path / "ds") as w, pytest.raises(ValueError, match="フレーム"):
        w.save_episode()


@pytest.mark.parametrize(
    "mutate",
    [
        lambda i, s, a, t: (i[:-1], s, a, t),  # 画像の形
        lambda i, s, a, t: (i.astype(np.float32), s, a, t),  # 画像の dtype
        lambda i, s, a, t: (i, s[:5], a, t),  # 状態の長さ
        lambda i, s, a, t: (i, s, np.full(6, np.nan, np.float32), t),  # 行動が NaN
        lambda i, s, a, t: (i, np.full(6, np.inf), a, t),
        lambda i, s, a, t: (i, s, a, ""),  # 指示が空
        lambda i, s, a, t: (i, s, a, None),
    ],
)
def test_invalid_frames_are_value_error_and_do_not_corrupt_the_episode(tmp_path, mutate):
    with _open(tmp_path / "ds") as w:
        good = (*_frames(1)[0], TASK)
        with pytest.raises(ValueError):
            w.add_frame(*mutate(*good))
        w.add_frame(*good)  # 直前の失敗が尾を引かない
        assert w.save_episode() == 0
    assert len(_read(tmp_path / "ds")) == 1

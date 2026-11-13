"""record の純粋ロジックのテスト (TASK-0027, Red)。LeRobot が無くても動く部分: 設定・特徴量・間引き・import の案内。"""

import copy
import sys

import pytest
import yaml

from so101_vlm.record.lerobot_writer import (
    JOINT_FEATURE_NAMES,
    build_features,
    load_lerobot,
    load_record_config,
)
from so101_vlm.record.record_config import DEFAULT_RECORD_CONFIG_PATH
from so101_vlm.record.sim_episode import keep_frame


def test_joint_feature_names_follow_the_so101_convention():
    assert JOINT_FEATURE_NAMES == (
        "shoulder_pan.pos", "shoulder_lift.pos", "elbow_flex.pos", "wrist_flex.pos", "wrist_roll.pos", "gripper.pos",
    )


def test_committed_config_loads():
    cfg = load_record_config()
    assert cfg.fps == 30 and cfg.robot_type == "so101" and cfg.camera_key == "top"
    assert cfg.use_videos is True and cfg.repo_id == "local/so101-vlm-pickplace"
    assert cfg.image_key == "observation.images.top"


def _raw():
    return yaml.safe_load(DEFAULT_RECORD_CONFIG_PATH.read_text(encoding="utf-8"))


def _write(tmp_path, raw):
    p = tmp_path / "record.yaml"
    p.write_text(yaml.safe_dump(raw), encoding="utf-8")
    return p


@pytest.mark.parametrize(
    "mutate",
    [
        lambda r: r.update(fps=0),
        lambda r: r.update(fps=30.5),
        lambda r: r.update(robot_type=""),
        lambda r: r.update(camera="a/b"),  # LeRobot の特徴量名に "/" は使えない
        lambda r: r.update(use_videos="yes"),
        lambda r: r.update(repo_id="no-slash"),
        lambda r: r.update(image_writer_threads=-1),
        lambda r: r.update(surprise=1),
        lambda r: r.pop("fps"),
    ],
)
def test_invalid_config_is_value_error(tmp_path, mutate):
    raw = copy.deepcopy(_raw())
    mutate(raw)
    with pytest.raises(ValueError):
        load_record_config(_write(tmp_path, raw))


def test_features_for_video_and_image_modes():
    cfg = load_record_config()
    f = build_features(cfg, (480, 640, 3), use_videos=True)
    assert set(f) == {"observation.state", "action", "observation.images.top"}
    for key in ("observation.state", "action"):
        assert f[key] == {"dtype": "float32", "shape": (6,), "names": list(JOINT_FEATURE_NAMES)}
    assert f["observation.images.top"] == {"dtype": "video", "shape": (480, 640, 3), "names": ["height", "width", "channels"]}
    assert build_features(cfg, (480, 640, 3), use_videos=False)["observation.images.top"]["dtype"] == "image"


@pytest.mark.parametrize("shape", [(480, 640), (480, 640, 4), (0, 640, 3), (480, 640, 3, 1), "x"])
def test_bad_image_shape_is_value_error(shape):
    with pytest.raises(ValueError):
        build_features(load_record_config(), shape, use_videos=True)


# --- 間引き: 制御周期 (50 Hz) -> 記録の fps (30) ------------------------------------------------
def _kept(n, dt=0.02, fps=30):
    return [k for k in range(1, n + 1) if keep_frame(k, dt, fps)]


def test_downsampling_keeps_about_fps_per_second_evenly():
    kept = _kept(500)  # 10 秒
    assert len(kept) == 300
    gaps = {b - a for a, b in zip(kept, kept[1:])}
    assert gaps <= {1, 2}  # 50 Hz -> 30 Hz なので、間隔は 1 か 2 周期


def test_same_rate_keeps_every_frame_and_float_noise_does_not_drop_any():
    assert _kept(100, 0.02, 50) == list(range(1, 101))
    assert len(_kept(300, 0.01, 100)) == 300


def test_cannot_upsample():
    with pytest.raises(ValueError):
        keep_frame(1, 0.02, 60)


@pytest.mark.parametrize("args", [(0, 0.02, 30), (1, 0.0, 30), (1, 0.02, 0), (1.5, 0.02, 30)])
def test_bad_downsampling_arguments_are_value_error(args):
    with pytest.raises(ValueError):
        keep_frame(*args)


def test_missing_lerobot_gives_an_actionable_message(monkeypatch):
    monkeypatch.setitem(sys.modules, "lerobot.datasets.lerobot_dataset", None)
    with pytest.raises(ImportError, match="uv sync --group record"):
        load_lerobot()

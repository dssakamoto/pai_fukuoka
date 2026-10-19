import numpy as np
import pytest

from so101_vlm.robot.base import JOINT_NAMES
from so101_vlm.robot.model_paths import JOINT_NAMES as SOURCE_NAMES
from so101_vlm.robot.sweep import sweep_targets

CTRL = np.array(
    [[-1.9, 1.9], [-1.7, 1.7], [-1.7, 1.6], [-1.6, 1.7], [-2.7, 2.8], [-0.2, 1.7]]
)
TIMES = np.linspace(0.0, 30.0, 301)


def test_joint_names_single_source():
    assert JOINT_NAMES is SOURCE_NAMES and len(JOINT_NAMES) == 6


def test_targets_within_ctrlrange():
    for t in TIMES:
        out = sweep_targets(CTRL, t)
        assert np.all(out >= CTRL[:, 0]) and np.all(out <= CTRL[:, 1])


def test_only_one_joint_deviates_at_a_time():
    center = CTRL.mean(axis=1)
    for t in TIMES:
        dev = np.flatnonzero(~np.isclose(sweep_targets(CTRL, t), center))
        assert len(dev) <= 1


def test_each_joint_is_driven_in_sequence():
    center = CTRL.mean(axis=1)
    for j in range(6):
        t = j * 2.0 + 0.5  # 周期 2 秒の 1/4 地点
        out = sweep_targets(CTRL, t)
        assert not np.isclose(out[j], center[j])


def test_inputs_not_mutated_and_shape():
    before = CTRL.copy()
    out = sweep_targets(CTRL, 0.5)
    assert np.array_equal(CTRL, before)
    assert out.shape == (6,) and out is not CTRL


def test_invalid_args():
    with pytest.raises(ValueError):
        sweep_targets(CTRL, 0.0, period=0)
    with pytest.raises(ValueError):
        sweep_targets(np.zeros(6), 0.0)

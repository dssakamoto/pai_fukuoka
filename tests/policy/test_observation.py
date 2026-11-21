"""Observation の任意項目 (rgb / depth / instruction) のテスト (SGT-008)。"""

import dataclasses

import numpy as np
import pytest

from so101_vlm.policy.base import Goal, Observation
from so101_vlm.robot.base import N_JOINTS

Q = np.zeros(N_JOINTS)


def test_q_and_timestamp_only_still_works():
    o = Observation(q=Q, timestamp=0.5)
    assert o.rgb is None and o.depth is None and o.instruction == ""


def test_rgb_and_depth_are_copied_and_readonly():
    rgb = np.zeros((4, 5, 3), np.uint8)
    depth = np.ones((4, 5), np.float32)
    o = Observation(q=Q, timestamp=0.0, rgb=rgb, depth=depth, instruction="put it in")
    rgb[0, 0, 0] = 9  # 元の配列を変えても、Observation には影響しない
    assert o.rgb[0, 0, 0] == 0 and o.rgb is not rgb
    with pytest.raises(ValueError):
        o.rgb[0, 0, 0] = 1
    with pytest.raises(ValueError):
        o.depth[0, 0] = 1.0
    assert o.instruction == "put it in"


@pytest.mark.parametrize(
    "kwargs",
    [
        {"rgb": np.zeros((4, 5), np.uint8)},  # 次元違い
        {"rgb": np.zeros((4, 5, 4), np.uint8)},  # チャンネル違い
        {"rgb": np.zeros((4, 5, 3), np.float32)},  # dtype 違い
        {"rgb": [[0]]},  # 配列でない
        {"rgb": np.zeros((0, 5, 3), np.uint8)},  # 空
        {"depth": np.zeros((4, 5, 1), np.float32)},
        {"depth": np.zeros((4, 5), np.uint8)},
        {"instruction": 3},
    ],
)
def test_invalid_optional_fields_raise_value_error(kwargs):
    with pytest.raises(ValueError):
        Observation(q=Q, timestamp=0.0, **kwargs)


def test_depth_allows_missing_values():
    depth = np.array([[0.0, np.inf], [0.5, 0.4]], np.float32)  # 欠測 (0 / inf) は有限でなくてよい
    assert Observation(q=Q, timestamp=0.0, depth=depth).depth.shape == (2, 2)


def test_observation_has_no_ground_truth_fields():
    names = {f.name for f in dataclasses.fields(Observation)}
    assert names == {"q", "timestamp", "rgb", "depth", "instruction"}


def test_goal_is_unchanged():
    g = Goal(pick_target_base=np.zeros(3), place_target_base=np.ones(3))
    assert g.instruction == ""

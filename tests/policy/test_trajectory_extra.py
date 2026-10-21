"""TASK-0012 Refactor: 防御的な分岐 (入力検証・0 秒保持) の追加テスト。

既存テストの期待値は変えず、Green 時点で未カバーだった分岐だけを確かめる。
信頼性: 🟡 要件 2.6 / 2.7 節の検証方針からの妥当な推測 (O5 の 0 秒保持は現行挙動の固定)
"""

import dataclasses

import numpy as np
import pytest

from so101_vlm.policy.trajectory import (
    JointWaypoint,
    build_joint_trajectory,
    hold_steps,
    interpolate_segment,
)
from so101_vlm.policy.trajectory_config import ClearanceConfig, TrajectoryConfig

DT = 0.02
Q0 = np.zeros(6)
Q1 = np.full(6, 0.1)
VMAX = [1.0] * 6


@pytest.mark.parametrize("bad", [-0.1, float("nan"), float("inf")])
def test_hold_steps_rejects_invalid_hold_s(bad):
    with pytest.raises(ValueError, match="hold_s"):
        hold_steps(bad, DT)


@pytest.mark.parametrize("bad", [0, -1, 1.5, True, "3"])
def test_interpolate_segment_rejects_invalid_n(bad):
    with pytest.raises(ValueError, match="n は"):
        interpolate_segment(Q0, Q1, bad)


def _build(waypoints):
    return build_joint_trajectory(
        Q0, waypoints, vmax=VMAX, speed_scale=1.0, control_dt=DT,
        safety_factor=1.0, min_segment_s=DT,
    )  # fmt: skip


def test_zero_hold_segment_is_not_kept_in_segments():
    """O5: 0 秒の保持区間は行もラベルも segments も残さない。"""
    traj = _build([JointWaypoint("move", Q1), JointWaypoint("pause", Q1, hold_s=0.0)])
    assert "pause" not in traj.segments
    assert "pause" not in traj.labels
    assert traj.segments == ("start", "move")


def test_trajectory_config_rejects_non_clearance_object(traj_cfg_kwargs):
    with pytest.raises(ValueError, match="ClearanceConfig"):
        TrajectoryConfig(**{**traj_cfg_kwargs, "clearance": {"obstacle_m": 0.01}})


@pytest.fixture
def traj_cfg_kwargs():
    clr = ClearanceConfig(obstacle_m=0.002, self_m=0.001, obstacle_geoms=("floor",))
    return dict(
        tcp_offset=(0.004, 0.0, 0.03), approach_height=0.08, min_carry_z=0.08,
        place_height=0.03, gripper_open=0.6, gripper_closed=0.15, close_hold_s=0.5,
        open_hold_s=0.5, min_segment_s=0.1, time_safety_factor=0.9, clearance=clr,
    )  # fmt: skip


def test_trajectory_config_accepts_valid_kwargs(traj_cfg_kwargs):
    cfg = TrajectoryConfig(**traj_cfg_kwargs)
    assert dataclasses.replace(cfg, close_hold_s=0).close_hold_s == 0.0

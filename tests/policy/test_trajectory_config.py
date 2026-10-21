"""TrajectoryConfig / ClearanceConfig / load_trajectory_config のテスト (TASK-0012, Red)。

テストケース定義: docs/implements/so101-vlm-pickplace/TASK-0012/trajectory-testcases.md 2 節 (TC-TC-*)
書き方は tests/safety/test_config.py と同じ (VALID の dict を _with / _without で変えて tmp_path に書く)。
対象モジュール (まだ存在しない): so101_vlm.policy.trajectory_config
信頼性: 🔵 要件・実測に基づく / 🟡 妥当な推測 / 🔴 要件にない仮定
"""

import copy
import dataclasses

import pytest
import yaml

from so101_vlm.policy.trajectory_config import (
    ClearanceConfig,
    TrajectoryConfig,
    load_trajectory_config,
)
from so101_vlm.safety.config import DEFAULT_MOTION_CONFIG_PATH, load_safety_config

NAN, INF = float("nan"), float("inf")
FLOOR_GEOMS = ["floor", "box_floor", "box_wall_px", "box_wall_nx", "box_wall_py", "box_wall_ny"]

VALID = {
    "safety": {
        "vmax": [1.0] * 6,
        "control_dt": 0.02,
        "speed_scale": 1.0,
        "state_timeout": 0.2,
        "workspace": {"lower": [0.12, -0.13, 0.00], "upper": [0.38, 0.26, 0.17]},
    },
    "trajectory": {
        "tcp_offset": [0.004, 0.0, 0.030],
        "approach_height": 0.08,
        "min_carry_z": 0.08,
        "place_height": 0.030,
        "gripper_open": 0.6,
        "gripper_closed": 0.15,
        "close_hold_s": 0.5,
        "open_hold_s": 0.3,
        "min_segment_s": 0.2,
        "time_safety_factor": 0.99,
        "clearance": {"obstacle_m": 0.001, "self_m": 0.002, "obstacle_geoms": FLOOR_GEOMS},
    },
}


def _write(tmp_path, obj):
    """obj が str ならそのまま、それ以外は safe_dump して motion.yaml に書く。"""
    path = tmp_path / "motion.yaml"
    path.write_text(obj if isinstance(obj, str) else yaml.safe_dump(obj), encoding="utf-8")
    return path


def _with(*changes):
    """VALID の深いコピーに (path_keys, value) を順に適用した dict を返す (VALID は不変)。"""
    obj = copy.deepcopy(VALID)
    for path_keys, value in changes:
        target = obj
        for key in path_keys[:-1]:
            target = target[key]
        target[path_keys[-1]] = value
    return obj


def _without(path_keys):
    obj = copy.deepcopy(VALID)
    target = obj
    for key in path_keys[:-1]:
        target = target[key]
    del target[path_keys[-1]]
    return obj


def _load(tmp_path, obj):
    return load_trajectory_config(_write(tmp_path, obj))


def _t(key):
    return ("trajectory", key)


def _c(key):
    return ("trajectory", "clearance", key)


# =============================================================== 2.1 正常系
def test_default_file_loads():
    """TC-TC-01 既定のファイルを読める 🔵"""
    cfg = load_trajectory_config()
    assert isinstance(cfg, TrajectoryConfig)  # 【確認内容】configs/motion.yaml の trajectory: を読める


def test_default_values_match_survey():
    """TC-TC-02 既定値 = 調査の推奨値 🔵"""
    cfg = load_trajectory_config()
    assert cfg.tcp_offset == (0.004, 0.0, 0.030)
    assert (cfg.approach_height, cfg.min_carry_z, cfg.place_height) == (0.08, 0.08, 0.030)
    assert (cfg.gripper_open, cfg.gripper_closed) == (0.6, 0.15)
    assert (cfg.close_hold_s, cfg.open_hold_s) == (0.5, 0.3)
    assert (cfg.min_segment_s, cfg.time_safety_factor) == (0.2, 0.99)
    assert (cfg.clearance.obstacle_m, cfg.clearance.self_m) == (0.001, 0.002)
    assert cfg.clearance.obstacle_geoms == tuple(FLOOR_GEOMS)  # 【確認内容】調査 3 節の値そのまま


def test_safety_section_unchanged():
    """TC-TC-03 safety: は変わらず読める 🔵"""
    s = load_safety_config()
    assert s.vmax == (1.0,) * 6 and s.control_dt == 0.02 and s.speed_scale == 1.0
    assert s.state_timeout == 0.2
    assert s.workspace.lower == (0.12, -0.13, 0.0) and s.workspace.upper == (0.38, 0.26, 0.17)


def test_trajectory_is_top_level_key_not_inside_safety():
    """TC-TC-04 trajectory: は safety: の中にない 🔵"""
    raw = yaml.safe_load(DEFAULT_MOTION_CONFIG_PATH.read_text(encoding="utf-8"))
    assert set(raw) == {"safety", "trajectory"}  # 【確認内容】最上位のキー
    assert set(raw["safety"]) == {"vmax", "control_dt", "speed_scale", "state_timeout", "workspace"}


def test_config_is_frozen_with_tuples():
    """TC-TC-05 frozen と tuple 🟡"""
    cfg = load_trajectory_config()
    with pytest.raises(dataclasses.FrozenInstanceError):
        cfg.approach_height = 0.1  # type: ignore[misc]
    assert isinstance(cfg.tcp_offset, tuple) and isinstance(cfg.clearance.obstacle_geoms, tuple)
    assert isinstance(cfg.clearance, ClearanceConfig)


def test_constructor_normalizes_lists_to_float_tuples():
    """TC-TC-06 list で渡しても tuple にそろう 🟡"""
    cfg = TrajectoryConfig(
        tcp_offset=[0.004, 0, 0.03],
        approach_height=0.08,
        min_carry_z=0.08,
        place_height=0.03,
        gripper_open=0.6,
        gripper_closed=0.15,
        close_hold_s=0.5,
        open_hold_s=0.3,
        min_segment_s=0.2,
        time_safety_factor=0.99,
        clearance=ClearanceConfig(obstacle_m=0.001, self_m=0.002, obstacle_geoms=["floor"]),
    )
    assert cfg.tcp_offset == (0.004, 0.0, 0.03)
    assert all(isinstance(v, float) for v in cfg.tcp_offset)  # 【確認内容】float の tuple
    assert cfg.clearance.obstacle_geoms == ("floor",)


def test_replace_creates_new_config():
    """TC-TC-07 dataclasses.replace で変えられる 🟡"""
    cfg = load_trajectory_config()
    changed = dataclasses.replace(cfg, approach_height=0.05)
    assert changed.approach_height == 0.05 and cfg.approach_height == 0.08  # 【確認内容】元は変わらない


# =============================================================== 2.2 異常系
def test_missing_file_raises_file_not_found(tmp_path):
    """TC-TC-E01 ファイルがない 🔵"""
    with pytest.raises(FileNotFoundError):
        load_trajectory_config(tmp_path / "nope.yaml")


def test_missing_trajectory_section(tmp_path):
    """TC-TC-E02 trajectory: がない 🟡"""
    with pytest.raises(ValueError, match="trajectory"):
        _load(tmp_path, {"safety": VALID["safety"]})


def test_top_level_not_mapping(tmp_path):
    """TC-TC-E03 最上位が mapping でない 🟡"""
    with pytest.raises(ValueError):
        _load(tmp_path, "- 1\n- 2")


def test_unknown_key(tmp_path):
    """TC-TC-E04 未知のキー 🟡"""
    with pytest.raises(ValueError, match="carry_height"):
        _load(tmp_path, _with((_t("carry_height"), 0.1)))


@pytest.mark.parametrize("path", [_t("gripper_open"), _c("self_m")], ids=["gripper_open", "self_m"])
def test_missing_key(tmp_path, path):
    """TC-TC-E05 キーの欠落 🟡"""
    with pytest.raises(ValueError, match=path[-1]):
        _load(tmp_path, _without(path))


def test_unknown_clearance_key(tmp_path):
    """TC-TC-E06 clearance: の未知のキー 🟡"""
    with pytest.raises(ValueError, match="block_m"):
        _load(tmp_path, _with((_c("block_m"), 0.001)))


@pytest.mark.parametrize("key", ["approach_height", "gripper_closed", "close_hold_s"])
@pytest.mark.parametrize("bad", [True, "0.1", NAN, INF], ids=["bool", "str", "nan", "inf"])
def test_non_numeric_or_non_finite_value(tmp_path, key, bad):
    """TC-TC-E07 bool / 文字列 / NaN / inf 🟡"""
    with pytest.raises(ValueError, match=key):
        _load(tmp_path, _with((_t(key), bad)))


@pytest.mark.parametrize("bad", [[0.004, 0.0], [0.004, 0.0, 0.03, 0.0], 0.03], ids=["len2", "len4", "scalar"])
def test_tcp_offset_length(tmp_path, bad):
    """TC-TC-E08 tcp_offset の長さ 🟡"""
    with pytest.raises(ValueError, match="tcp_offset"):
        _load(tmp_path, _with((_t("tcp_offset"), bad)))


@pytest.mark.parametrize("key", ["approach_height", "min_carry_z", "place_height", "min_segment_s"])
@pytest.mark.parametrize("bad", [0.0, -0.01])
def test_positive_values_reject_zero_and_negative(tmp_path, key, bad):
    """TC-TC-E09 正であるべき値が 0 以下 🟡"""
    with pytest.raises(ValueError, match=key):
        _load(tmp_path, _with((_t(key), bad)))


def test_negative_hold_time(tmp_path):
    """TC-TC-E10 保持時間が負 🟡"""
    with pytest.raises(ValueError, match="close_hold_s"):
        _load(tmp_path, _with((_t("close_hold_s"), -0.1)))


@pytest.mark.parametrize("bad", [0.0, 1.01, -0.5])
def test_time_safety_factor_range(tmp_path, bad):
    """TC-TC-E11 係数の範囲 🟡"""
    with pytest.raises(ValueError, match="time_safety_factor"):
        _load(tmp_path, _with((_t("time_safety_factor"), bad)))


@pytest.mark.parametrize("opened, closed", [(0.15, 0.15), (0.1, 0.6)], ids=["equal", "reversed"])
def test_gripper_open_must_exceed_closed(tmp_path, opened, closed):
    """TC-TC-E12 開き角 ≤ 閉じ角 🟡"""
    with pytest.raises(ValueError, match="gripper"):
        _load(tmp_path, _with((_t("gripper_open"), opened), (_t("gripper_closed"), closed)))


def test_negative_clearance(tmp_path):
    """TC-TC-E13 すき間が負 🟡"""
    with pytest.raises(ValueError, match="obstacle_m"):
        _load(tmp_path, _with((_c("obstacle_m"), -0.001)))


@pytest.mark.parametrize("bad", [[], [""], [1], "floor"], ids=["empty", "empty_str", "number", "bare_str"])
def test_obstacle_geoms_shape(tmp_path, bad):
    """TC-TC-E14 obstacle_geoms の形 🟡"""
    with pytest.raises(ValueError, match="obstacle_geoms"):
        _load(tmp_path, _with((_c("obstacle_geoms"), bad)))


# =============================================================== 2.3 境界値
def test_factor_exactly_one(tmp_path):
    """TC-TC-B01 係数ちょうど 1 🟡"""
    assert _load(tmp_path, _with((_t("time_safety_factor"), 1.0))).time_safety_factor == 1.0


def test_hold_zero_seconds(tmp_path):
    """TC-TC-B02 保持 0 秒 🟡"""
    assert _load(tmp_path, _with((_t("close_hold_s"), 0.0))).close_hold_s == 0.0


def test_clearance_zero(tmp_path):
    """TC-TC-B03 すき間 0 🟡"""
    cfg = _load(tmp_path, _with((_c("obstacle_m"), 0.0), (_c("self_m"), 0.0)))
    assert (cfg.clearance.obstacle_m, cfg.clearance.self_m) == (0.0, 0.0)


def test_integer_becomes_float(tmp_path):
    """TC-TC-B04 整数 🟡"""
    cfg = _load(tmp_path, _with((_t("approach_height"), 1)))
    assert cfg.approach_height == 1.0 and isinstance(cfg.approach_height, float)

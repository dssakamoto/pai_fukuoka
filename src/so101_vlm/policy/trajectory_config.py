"""軌道生成の設定 (configs/motion.yaml の trajectory: セクション) (TASK-0012)。

🔵 yaml.safe_load を使う。🟡 trajectory: と clearance: のキーは過不足なく一致させる
(safety/config.py と同じ方式)。trajectory: 以外の最上位キーは無視する。
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

import yaml

from so101_vlm.eval.validation import require_exact_keys, require_number
from so101_vlm.safety._validation import require_fraction, require_positive, to_float_tuple
from so101_vlm.safety.config import DEFAULT_MOTION_CONFIG_PATH

_N_AXES = 3
_TRAJECTORY_KEYS = frozenset(
    {
        "tcp_offset", "approach_height", "min_carry_z", "place_height", "gripper_open",
        "gripper_closed", "close_hold_s", "open_hold_s", "min_segment_s", "time_safety_factor",
        "clearance",
    }
)  # fmt: skip
_CLEARANCE_KEYS = frozenset({"obstacle_m", "self_m", "obstacle_geoms"})


def _non_negative(name: str, value: object) -> float:
    """【機能概要】: 0 以上の有限の実数を float で返す 🟡"""
    number = require_number(name, value)
    if number < 0:
        raise ValueError(f"{name} は 0 以上で指定してください: {name}={value!r}")
    return number


def _name_tuple(name: str, value: object) -> tuple[str, ...]:
    """【機能概要】: 空でない文字列の列を tuple にする (裸の str・空・非文字列は ValueError) 🟡"""
    if isinstance(value, (str, bytes)) or not hasattr(value, "__iter__"):
        raise ValueError(f"{name} は文字列のリストで指定してください: {name}={value!r}")
    items = tuple(value)  # type: ignore[call-overload]
    if not items or any(not isinstance(v, str) or not v for v in items):
        raise ValueError(f"{name} は空でない文字列の列で指定してください: {name}={value!r}")
    return items


@dataclass(frozen=True)
class ClearanceConfig:
    """【機能概要】: FK のすき間確認の設定。変更不可。🟡

    obstacle_m: ロボットと障害物のすき間の下限 [m]、self_m: 自己干渉のすき間の下限 [m]、
    obstacle_geoms: 障害物とみなす geom 名 (床・箱の底と壁)。
    """

    obstacle_m: float
    self_m: float
    obstacle_geoms: tuple[str, ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "obstacle_m", _non_negative("clearance.obstacle_m", self.obstacle_m))
        object.__setattr__(self, "self_m", _non_negative("clearance.self_m", self.self_m))
        object.__setattr__(
            self, "obstacle_geoms", _name_tuple("clearance.obstacle_geoms", self.obstacle_geoms)
        )


@dataclass(frozen=True)
class TrajectoryConfig:
    """【機能概要】: 軌道生成の設定。変更不可 (dataclasses.replace で変える)。🟡

    長さは m、角度は rad、時間は s。各値の根拠は configs/motion.yaml のコメントを参照。
    gripper の値が ctrlrange 内かはモデルが要るので IkPolicy の構築時に確認する。
    """

    tcp_offset: tuple[float, float, float]
    approach_height: float
    min_carry_z: float
    place_height: float
    gripper_open: float
    gripper_closed: float
    close_hold_s: float
    open_hold_s: float
    min_segment_s: float
    time_safety_factor: float
    clearance: ClearanceConfig

    def __post_init__(self) -> None:
        # 【正規化】: list を float の tuple に、整数を float にそろえる
        set_ = lambda k, v: object.__setattr__(self, k, v)  # noqa: E731
        set_("tcp_offset", to_float_tuple("tcp_offset", self.tcp_offset, _N_AXES))
        for key in ("approach_height", "min_carry_z", "place_height", "min_segment_s"):
            set_(key, require_positive(key, getattr(self, key)))
        for key in ("close_hold_s", "open_hold_s"):
            set_(key, _non_negative(key, getattr(self, key)))
        set_("time_safety_factor", require_fraction("time_safety_factor", self.time_safety_factor))
        set_("gripper_open", require_number("gripper_open", self.gripper_open))
        set_("gripper_closed", require_number("gripper_closed", self.gripper_closed))
        if self.gripper_open <= self.gripper_closed:
            raise ValueError(
                "gripper_open は gripper_closed より大きい値で指定してください: "
                f"gripper_open={self.gripper_open}, gripper_closed={self.gripper_closed}"
            )
        if not isinstance(self.clearance, ClearanceConfig):
            raise ValueError(f"clearance は ClearanceConfig で指定してください: {self.clearance!r}")


def load_trajectory_config(path: str | Path = DEFAULT_MOTION_CONFIG_PATH) -> TrajectoryConfig:
    """【機能概要】: motion.yaml の trajectory: を読み、検証済みの TrajectoryConfig を返す 🟡

    ファイルなしは FileNotFoundError、キーの過不足・値の違反は ValueError。
    """
    with open(path, encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    if not isinstance(raw, Mapping):
        raise ValueError(f"最上位は mapping で指定してください (trajectory: が必要です): {path}")
    if "trajectory" not in raw:
        raise ValueError(f"必須キーがありません: trajectory ({path})")
    data = require_exact_keys("trajectory", raw["trajectory"], _TRAJECTORY_KEYS)
    clr = require_exact_keys("clearance", data["clearance"], _CLEARANCE_KEYS)
    fields = {k: data[k] for k in _TRAJECTORY_KEYS if k != "clearance"}
    return TrajectoryConfig(
        **fields,
        clearance=ClearanceConfig(
            obstacle_m=clr["obstacle_m"], self_m=clr["self_m"], obstacle_geoms=clr["obstacle_geoms"]
        ),
    )

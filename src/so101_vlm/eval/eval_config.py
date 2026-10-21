"""評価設定 (configs/eval.yaml) の読み込み (TASK-0006)。

🔵 yaml.safe_load を使う。🔴 未知キー・欠落キー・型違反は ValueError (キー名入り)。
"""

from dataclasses import dataclass
from pathlib import Path

import yaml

from so101_vlm.eval.scene_spec import SCENE_SPEC
from so101_vlm.eval.validation import (
    require_area,
    require_exact_keys,
    require_int,
    require_min_gap,
    require_number,
)
from so101_vlm.robot.model_paths import REPO_ROOT

# 【定数定義】: 既定の設定ファイルのパス (パッケージ位置から解決) 🟡
DEFAULT_EVAL_CONFIG_PATH = REPO_ROOT / "configs" / "eval.yaml"

_TOP_KEYS = frozenset({"n_trials", "base_seed", "placement", "judge", "failure"})
_PLACEMENT_KEYS = frozenset({"n_blocks", "area", "min_gap", "max_attempts"})
_JUDGE_KEYS = frozenset({"margin_m", "speed_max_mps", "settle_s"})
_FAILURE_KEYS = frozenset({"lift_min_m", "carry_separation_m", "other_move_m", "tilt_max_deg"})
# 【定数定義】: 傾きのしきい値の上限 [deg]。90 度は「横倒し」で、しきい値として意味を持たない 🟡
TILT_LIMIT_DEG = 90.0


@dataclass(frozen=True)
class PlacementConfig:
    """【機能概要】: ブロック配置の設定 🔵"""

    n_blocks: int
    area: float
    min_gap: float
    max_attempts: int


def _positive(name: str, value: object) -> float:
    """【機能概要】: 0 より大きい有限の実数を float で返す (bool 不可) 🟡"""
    number = require_number(name, value)
    if number <= 0:
        raise ValueError(f"{name} は 0 より大きく指定してください: {name}={value!r}")
    return number


@dataclass(frozen=True)
class JudgeConfig:
    """【機能概要】: 成功判定 (FR-07) の設定。TASK-0013 の最小版 🟡

    margin_m: 内寸からの内側の余裕 [m] (0 以上、箱の内寸の半分未満)、
    speed_max_mps: 静止とみなす並進速度 [m/s] (> 0)、
    settle_s: 軌道の終わりから判定までの待ち [s] (0 以上)。
    生成時に検証し、int は float にそろえる。
    """

    margin_m: float
    speed_max_mps: float
    settle_s: float

    def __post_init__(self) -> None:
        margin = require_number("margin_m", self.margin_m)
        if not 0 <= margin < SCENE_SPEC.box_inner_half_size:
            raise ValueError(
                f"margin_m は 0 以上 {SCENE_SPEC.box_inner_half_size} 未満で指定してください: margin_m={self.margin_m!r}"
            )
        settle = require_number("settle_s", self.settle_s)
        if settle < 0:
            raise ValueError(f"settle_s は 0 以上で指定してください: settle_s={self.settle_s!r}")
        object.__setattr__(self, "margin_m", margin)
        object.__setattr__(self, "speed_max_mps", _positive("speed_max_mps", self.speed_max_mps))
        object.__setattr__(self, "settle_s", settle)


@dataclass(frozen=True)
class FailureConfig:
    """【機能概要】: 失敗の分類のしきい値 (TASK-0013 の初期値) 🔴

    lift_min_m / carry_separation_m / other_move_m は [m]、tilt_max_deg は (0, 90) の [deg]。
    """

    lift_min_m: float
    carry_separation_m: float
    other_move_m: float
    tilt_max_deg: float

    def __post_init__(self) -> None:
        for name in ("lift_min_m", "carry_separation_m", "other_move_m", "tilt_max_deg"):
            object.__setattr__(self, name, _positive(name, getattr(self, name)))
        if self.tilt_max_deg >= TILT_LIMIT_DEG:
            raise ValueError(
                f"tilt_max_deg は {TILT_LIMIT_DEG} 未満で指定してください: tilt_max_deg={self.tilt_max_deg!r}"
            )


@dataclass(frozen=True)
class EvalConfig:
    """【機能概要】: 評価全体の設定 🔵"""

    n_trials: int
    base_seed: int
    placement: PlacementConfig
    judge: JudgeConfig
    failure: FailureConfig


def _parse_placement(raw: object) -> PlacementConfig:
    """【機能概要】: placement セクションを検証して PlacementConfig にする 🔴"""
    data = require_exact_keys("placement", raw, _PLACEMENT_KEYS)
    return PlacementConfig(
        n_blocks=require_int("n_blocks", data["n_blocks"], 1),
        area=require_area("area", data["area"]),
        min_gap=require_min_gap("min_gap", data["min_gap"]),
        max_attempts=require_int("max_attempts", data["max_attempts"], 1),
    )


def _parse_judge(raw: object) -> JudgeConfig:
    """【機能概要】: judge セクションを検証して JudgeConfig にする 🟡"""
    return JudgeConfig(**require_exact_keys("judge", raw, _JUDGE_KEYS))


def _parse_failure(raw: object) -> FailureConfig:
    """【機能概要】: failure セクションを検証して FailureConfig にする 🔴"""
    return FailureConfig(**require_exact_keys("failure", raw, _FAILURE_KEYS))


def load_eval_config(path: str | Path = DEFAULT_EVAL_CONFIG_PATH) -> EvalConfig:
    """【機能概要】: YAML を読み、検証済みの EvalConfig を返す。ファイルなしは FileNotFoundError 🔴"""
    with open(path, encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    data = require_exact_keys("(top level)", raw, _TOP_KEYS)
    return EvalConfig(
        n_trials=require_int("n_trials", data["n_trials"], 1),
        base_seed=require_int("base_seed", data["base_seed"], 0),
        placement=_parse_placement(data["placement"]),
        judge=_parse_judge(data["judge"]),
        failure=_parse_failure(data["failure"]),
    )

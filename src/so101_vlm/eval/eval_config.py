"""評価設定 (configs/eval.yaml) の読み込み (TASK-0006)。

🔵 yaml.safe_load を使う。🔴 未知キー・欠落キー・型違反は ValueError (キー名入り)。
"""

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

import yaml

from so101_vlm.eval.validation import require_area, require_int, require_min_gap
from so101_vlm.robot.model_paths import REPO_ROOT

# 【定数定義】: 既定の設定ファイルのパス (パッケージ位置から解決) 🟡
DEFAULT_EVAL_CONFIG_PATH = REPO_ROOT / "configs" / "eval.yaml"

_TOP_KEYS = frozenset({"n_trials", "base_seed", "placement"})
_PLACEMENT_KEYS = frozenset({"n_blocks", "area", "min_gap", "max_attempts"})


@dataclass(frozen=True)
class PlacementConfig:
    """【機能概要】: ブロック配置の設定 🔵"""

    n_blocks: int
    area: float
    min_gap: float
    max_attempts: int


@dataclass(frozen=True)
class EvalConfig:
    """【機能概要】: 評価全体の設定 🔵"""

    n_trials: int
    base_seed: int
    placement: PlacementConfig


def _require_mapping(name: str, value: object, allowed: frozenset[str]) -> Mapping:
    """【機能概要】: mapping であり、キーが allowed と過不足なく一致することを要求する 🔴"""
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} は mapping で指定してください: {name}={value!r}")
    unknown = sorted(str(k) for k in value if k not in allowed)
    if unknown:
        raise ValueError(f"未知のキー: {', '.join(unknown)} ({name})")
    missing = sorted(allowed - set(value))
    if missing:
        raise ValueError(f"必須キーがありません: {', '.join(missing)} ({name})")
    return value


def _parse_placement(raw: object) -> PlacementConfig:
    """【機能概要】: placement セクションを検証して PlacementConfig にする 🔴"""
    data = _require_mapping("placement", raw, _PLACEMENT_KEYS)
    return PlacementConfig(
        n_blocks=require_int("n_blocks", data["n_blocks"], 1),
        area=require_area("area", data["area"]),
        min_gap=require_min_gap("min_gap", data["min_gap"]),
        max_attempts=require_int("max_attempts", data["max_attempts"], 1),
    )


def load_eval_config(path: str | Path = DEFAULT_EVAL_CONFIG_PATH) -> EvalConfig:
    """【機能概要】: YAML を読み、検証済みの EvalConfig を返す。ファイルなしは FileNotFoundError 🔴"""
    with open(path, encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    data = _require_mapping("(top level)", raw, _TOP_KEYS)
    return EvalConfig(
        n_trials=require_int("n_trials", data["n_trials"], 1),
        base_seed=require_int("base_seed", data["base_seed"], 0),
        placement=_parse_placement(data["placement"]),
    )

"""安全フィルタの設定 (configs/motion.yaml の safety: セクション) (TASK-0010)。

🔵 yaml.safe_load を使う。🟡 safety: と workspace: の中のキーは過不足なく一致させる。
safety: 以外の最上位キーは無視する (TASK-0012 が同じファイルに追加するため)。
"""

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

import yaml

from so101_vlm.eval.validation import require_exact_keys
from so101_vlm.robot.base import N_JOINTS
from so101_vlm.robot.model_paths import REPO_ROOT
from so101_vlm.safety._validation import require_fraction, require_positive, to_float_tuple

# 【定数定義】: 既定の設定ファイルのパス (パッケージ位置から解決) 🔵
DEFAULT_MOTION_CONFIG_PATH = REPO_ROOT / "configs" / "motion.yaml"

_N_AXES = 3
_SAFETY_KEYS = frozenset({"vmax", "control_dt", "speed_scale", "state_timeout", "workspace"})
_WORKSPACE_KEYS = frozenset({"lower", "upper"})


@dataclass(frozen=True)
class WorkspaceBox:
    """【機能概要】: baseframe 基準の作業範囲 (直方体、境界を含む)。単位 m。🟡

    list を渡しても tuple (float) に正規化して保持する。各軸で lower < upper が必要。
    """

    lower: tuple[float, float, float]
    upper: tuple[float, float, float]

    def __post_init__(self) -> None:
        lower = to_float_tuple("workspace.lower", self.lower, _N_AXES)
        upper = to_float_tuple("workspace.upper", self.upper, _N_AXES)
        if any(lo >= up for lo, up in zip(lower, upper)):
            raise ValueError(
                f"workspace は各軸で lower < upper が必要です: lower={lower}, upper={upper}"
            )
        object.__setattr__(self, "lower", lower)
        object.__setattr__(self, "upper", upper)


@dataclass(frozen=True)
class SafetyConfig:
    """【機能概要】: 安全フィルタの設定。変更不可。🟡

    vmax: 関節ごとの最大角速度 [rad/s] (6 個、各 > 0)、control_dt: 制御周期 [s] (> 0)、
    speed_scale: 速度倍率 (0, 1]、state_timeout: 状態途絶の判定時間 [s] (> 0)、
    workspace: 作業範囲。実機の 0.3 倍は dataclasses.replace(config, speed_scale=0.3)。
    """

    vmax: tuple[float, ...]
    control_dt: float
    speed_scale: float
    state_timeout: float
    workspace: WorkspaceBox

    def __post_init__(self) -> None:
        vmax = to_float_tuple("vmax", self.vmax, N_JOINTS)
        if any(v <= 0 for v in vmax):
            raise ValueError(f"vmax の全要素は 0 より大きい値で指定してください: vmax={vmax}")
        if not isinstance(self.workspace, WorkspaceBox):
            raise ValueError(f"workspace は WorkspaceBox で指定してください: {self.workspace!r}")
        object.__setattr__(self, "vmax", vmax)
        object.__setattr__(self, "control_dt", require_positive("control_dt", self.control_dt))
        object.__setattr__(self, "speed_scale", require_fraction("speed_scale", self.speed_scale))
        object.__setattr__(
            self, "state_timeout", require_positive("state_timeout", self.state_timeout)
        )


def load_safety_config(path: str | Path = DEFAULT_MOTION_CONFIG_PATH) -> SafetyConfig:
    """【機能概要】: motion.yaml の safety: を読み、検証済みの SafetyConfig を返す 🟡

    ファイルなしは FileNotFoundError、キーの過不足・値の違反は ValueError。
    """
    with open(path, encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    if not isinstance(raw, Mapping):
        raise ValueError(f"最上位は mapping で指定してください (safety: が必要です): {path}")
    if "safety" not in raw:
        raise ValueError(f"必須キーがありません: safety ({path})")
    data = require_exact_keys("safety", raw["safety"], _SAFETY_KEYS)
    ws = require_exact_keys("workspace", data["workspace"], _WORKSPACE_KEYS)
    return SafetyConfig(
        vmax=data["vmax"],
        control_dt=data["control_dt"],
        speed_scale=data["speed_scale"],
        state_timeout=data["state_timeout"],
        workspace=WorkspaceBox(lower=ws["lower"], upper=ws["upper"]),
    )

"""ノードの設定 (configs/sim.yaml / configs/real.yaml) (TASK-0025)。

🔵 design.md 3 章: sim と real の切り替えは、camera と robot ノードが読むこの設定だけで行う。
他のノードのコードは変えない (どのノードも同じ設定ファイルを読む。ドライバを選ぶのは camera / robot だけ)。
違反は ValueError (キー名入り)。未知のキーは拒否する。
"""

from dataclasses import dataclass
from pathlib import Path

import yaml

from so101_vlm.eval.validation import require_exact_keys, require_int, require_number
from so101_vlm.robot.model_paths import REPO_ROOT

SIM_CONFIG_PATH = REPO_ROOT / "configs" / "sim.yaml"
REAL_CONFIG_PATH = REPO_ROOT / "configs" / "real.yaml"

MODES = ("sim", "real")
ROBOT_KINDS = ("sim", "so101")
CAMERA_KINDS = ("sim", "real")

_TOP_KEYS = frozenset(
    {"mode", "scene", "calibration", "robot", "camera", "vlm", "output_dir", "timeout_s"}
)
_CAMERA_KEYS = frozenset({"kind", "name", "width", "height"})


@dataclass(frozen=True)
class NodeConfig:
    """【機能概要】: 全ノード共通の設定。パスは絶対パスにしてある。🔵"""

    mode: str
    scene_path: Path
    calibration_path: Path
    robot_kind: str
    robot_options: dict
    camera_kind: str
    camera_name: str
    image_width: int
    image_height: int
    vlm_provider: str
    output_dir: Path
    timeout_s: float


def _choice(name: str, value: object, allowed: tuple[str, ...]) -> str:
    if value not in allowed:
        raise ValueError(f"{name} は {allowed} のどれか: {value!r}")
    return value  # type: ignore[return-value]


def _text(name: str, value: object) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} は空でない文字列で指定してください: {value!r}")
    return value


def _path(name: str, value: object) -> Path:
    p = Path(_text(name, value))
    return p if p.is_absolute() else REPO_ROOT / p


# 🔵 Intent: eval_config / vlm.config と同じ流儀 (yaml.safe_load + 厳密なキー検証)。
def load_node_config(path: str | Path = SIM_CONFIG_PATH) -> NodeConfig:
    """【機能概要】: YAML を読み、検証済みの NodeConfig を返す。ファイルなしは FileNotFoundError 🔵"""
    with open(path, encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    data = require_exact_keys("(top level)", raw, _TOP_KEYS)
    robot = data["robot"]
    if not isinstance(robot, dict) or "kind" not in robot:
        raise ValueError(f"robot は kind を持つ mapping で指定してください: {robot!r}")
    camera = require_exact_keys("camera", data["camera"], _CAMERA_KEYS)
    vlm = data["vlm"]
    provider = vlm.get("provider") if isinstance(vlm, dict) else None
    if not isinstance(provider, str) or not provider.strip():
        raise ValueError(f"vlm.provider は空でない文字列で指定してください: {vlm!r}")
    timeout_s = require_number("timeout_s", data["timeout_s"])
    if timeout_s <= 0:
        raise ValueError(f"timeout_s は 0 より大きくしてください: {timeout_s}")
    return NodeConfig(
        mode=_choice("mode", data["mode"], MODES),
        scene_path=_path("scene", data["scene"]),
        calibration_path=_path("calibration", data["calibration"]),
        robot_kind=_choice("robot.kind", robot["kind"], ROBOT_KINDS),
        robot_options={k: v for k, v in robot.items() if k != "kind"},
        camera_kind=_choice("camera.kind", camera["kind"], CAMERA_KINDS),
        camera_name=_text("camera.name", camera["name"]),
        image_width=require_int("camera.width", camera["width"], 1),
        image_height=require_int("camera.height", camera["height"], 1),
        vlm_provider=provider,
        output_dir=_path("output_dir", data["output_dir"]),
        timeout_s=timeout_s,
    )

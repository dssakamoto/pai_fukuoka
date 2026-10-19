"""SO-101 モデルのパスと関節名。"""

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
SO101_ASSET_DIR = REPO_ROOT / "assets" / "so101"
SO101_SCENE_PATH = SO101_ASSET_DIR / "scene.xml"

JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
SITE_NAMES = ("baseframe", "gripperframe")

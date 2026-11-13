"""記録の設定 (configs/record.yaml) の読み込みと検証 (TASK-0027)。

🟡 fps は資料にない値なので、SO-101 の記録の標準 (30) にした。違反は ValueError (キー名入り)、未知のキーは拒否する。
"""

from dataclasses import dataclass
from pathlib import Path

import yaml

from so101_vlm.eval.validation import require_exact_keys, require_int
from so101_vlm.robot.model_paths import REPO_ROOT

DEFAULT_RECORD_CONFIG_PATH = REPO_ROOT / "configs" / "record.yaml"
_KEYS = frozenset({"fps", "robot_type", "camera", "use_videos", "repo_id", "image_writer_threads"})


@dataclass(frozen=True)
class RecordConfig:
    """【機能概要】: 記録の設定。image_key は LeRobot の特徴量名 (observation.images.<camera>) 🔵"""

    fps: int
    robot_type: str
    camera_key: str
    use_videos: bool
    repo_id: str
    image_writer_threads: int

    @property
    def image_key(self) -> str:
        return f"observation.images.{self.camera_key}"


def _text(name: str, value: object) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} は空でない文字列で指定してください: {value!r}")
    return value


def load_record_config(path: str | Path = DEFAULT_RECORD_CONFIG_PATH) -> RecordConfig:
    """【機能概要】: YAML を読み、検証済みの RecordConfig を返す。ファイルなしは FileNotFoundError 🔵"""
    with open(path, encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    data = require_exact_keys("(top level)", raw, _KEYS)
    camera = _text("camera", data["camera"])
    if "/" in camera or "." in camera:
        raise ValueError(f"camera に '/' と '.' は使えません (LeRobot の特徴量名になる): {camera!r}")
    if not isinstance(data["use_videos"], bool):
        raise ValueError(f"use_videos は true/false で指定してください: {data['use_videos']!r}")
    repo_id = _text("repo_id", data["repo_id"])
    if repo_id.count("/") != 1 or not all(repo_id.split("/")):
        raise ValueError(f"repo_id は <ユーザ>/<名前> の形で指定してください: {repo_id!r}")
    return RecordConfig(
        fps=require_int("fps", data["fps"], 1),
        robot_type=_text("robot_type", data["robot_type"]),
        camera_key=camera,
        use_videos=data["use_videos"],
        repo_id=repo_id,
        image_writer_threads=require_int("image_writer_threads", data["image_writer_threads"], 0),
    )

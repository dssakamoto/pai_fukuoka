"""LeRobot のデータセット (v3.0) への書き込み (TASK-0027)。

🔵 design.md 4.9: 1 試行 = 1 エピソード。フレームは 画像・関節状態・行動 (目標関節角)・言語指示。
特徴量の名前は SO-101 の LeRobot 実装と同じ (observation.state / action は "<関節>.pos" の float32 (6,)、
カメラは observation.images.<名前>)。v2 のデモ収集 (lerobot-record) と同じ形式なので、そのまま SmolVLA の微調整に使える。
仕様の確認結果は docs/notes/lerobot-format.md。LeRobot (torch を含む) は重いので、必要なときだけ import する
(uv sync --group record)。LeRobot の版は pyproject.toml で固定 (0.7.0)。
"""

import json
from pathlib import Path
from types import TracebackType

import numpy as np

from so101_vlm.record.record_config import RecordConfig, load_record_config
from so101_vlm.robot.base import N_JOINTS
from so101_vlm.robot.model_paths import JOINT_NAMES

# 🔵 SO-101 の LeRobot 実装 (so_follower) の名前: <モーター名>.pos
JOINT_FEATURE_NAMES = tuple(f"{name}.pos" for name in JOINT_NAMES)
SIDECAR_NAME = "so101_trials.jsonl"
_IMAGE_NAMES = ["height", "width", "channels"]

__all__ = ["JOINT_FEATURE_NAMES", "LeRobotWriter", "build_features", "load_lerobot", "load_record_config"]


def load_lerobot():
    """【機能概要】: LeRobotDataset クラスを返す。入っていなければ、入れ方つきの ImportError 🔵"""
    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except ImportError as exc:
        raise ImportError(
            "lerobot が入っていません。`uv sync --group record` で入れてください (torch を含むため重い)"
        ) from exc
    return LeRobotDataset


def build_features(config: RecordConfig, image_shape: object, *, use_videos: bool) -> dict:
    """【機能概要】: LeRobotDataset.create に渡す特徴量の定義を作る。画像の形が (H, W, 3) でなければ ValueError 🔵"""
    ok = (
        isinstance(image_shape, tuple) and len(image_shape) == 3 and image_shape[2] == 3
        and all(isinstance(n, int) and not isinstance(n, bool) and n > 0 for n in image_shape)
    )
    if not ok:
        raise ValueError(f"image_shape は (H, W, 3) で指定してください: {image_shape!r}")
    joint = {"dtype": "float32", "shape": (N_JOINTS,), "names": list(JOINT_FEATURE_NAMES)}
    return {
        "observation.state": dict(joint),
        "action": dict(joint),
        config.image_key: {"dtype": "video" if use_videos else "image", "shape": image_shape, "names": list(_IMAGE_NAMES)},
    }


def _vector(name: str, value: object) -> np.ndarray:
    try:
        arr = np.asarray(value, dtype=np.float32)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} は数値の配列で指定してください: {exc}") from exc
    if arr.shape != (N_JOINTS,) or not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} は有限の数値 {N_JOINTS} 個で指定してください: shape={arr.shape}")
    return arr


class LeRobotWriter:
    """【機能概要】: LeRobotDataset への書き込み。入力を検証し、試行ごとの付随情報 (sidecar) を残す 🔵

    使い方: with LeRobotWriter(root) as w: w.add_frame(...) を繰り返し -> w.save_episode(info)。with を抜けると finalize
    (LeRobot v3 は finalize しないと読めない)。root は新しいディレクトリでなければならない (LeRobot の制約。既存は拒否)。
    sidecar: <root>/so101_trials.jsonl に 1 試行 1 行 (episode_index、動かず記録しなかった試行は null)。
    LeRobot の meta/ には触らない。
    """

    def __init__(
        self,
        root: str | Path,
        *,
        config: RecordConfig | None = None,
        image_shape: tuple[int, int, int] = (480, 640, 3),
        use_videos: bool | None = None,
    ) -> None:
        self._config = load_record_config() if config is None else config
        self._root = Path(root)
        if self._root.exists():
            raise FileExistsError(
                f"{self._root} は既にあります。LeRobot は既存のディレクトリには作れません。別のパスを指定してください"
            )
        self._image_shape = image_shape
        videos = self._config.use_videos if use_videos is None else use_videos
        features = build_features(self._config, image_shape, use_videos=videos)
        self._dataset = load_lerobot().create(
            self._config.repo_id, self._config.fps, features, root=self._root,
            robot_type=self._config.robot_type, use_videos=videos,
            image_writer_threads=self._config.image_writer_threads,
        )  # fmt: skip
        self._buffered = 0
        self._saved = 0
        self._finalized = False

    @property
    def episodes_saved(self) -> int:
        """【機能概要】: 保存したエピソードの数 🔵"""
        return self._saved

    @property
    def fps(self) -> int:
        return self._config.fps

    @property
    def image_shape(self) -> tuple[int, int, int]:
        return self._image_shape

    def _check_open(self) -> None:
        if self._finalized:
            raise RuntimeError("finalize 済みです。もう書き込めません")

    def add_frame(self, image: np.ndarray, state: object, action: object, task: str) -> None:
        """【機能概要】: 1 フレームを現在のエピソードに追加する。不正な入力は ValueError (エピソードは壊れない) 🔵

        image: uint8 (H, W, 3)、state / action: 有限の数値 6 個 [rad] (関節順)、task: 言語指示 (空でない文字列)。
        """
        self._check_open()
        if not isinstance(image, np.ndarray) or image.dtype != np.uint8 or image.shape != self._image_shape:
            raise ValueError(
                f"image は uint8 の {self._image_shape} で指定してください: "
                f"{getattr(image, 'dtype', None)} {getattr(image, 'shape', None)}"
            )
        frame_state, frame_action = _vector("state", state), _vector("action", action)
        if not isinstance(task, str) or not task.strip():
            raise ValueError("task (言語指示) は空でない文字列で指定してください")
        self._dataset.add_frame(
            {"observation.state": frame_state, "action": frame_action, self._config.image_key: image, "task": task}
        )
        self._buffered += 1

    def _log(self, info: dict, episode_index: int | None) -> None:
        line = {**info, "episode_index": episode_index}
        with open(self._root / SIDECAR_NAME, "a", encoding="utf-8", newline="") as f:
            f.write(json.dumps(line, ensure_ascii=False, allow_nan=False) + "\n")

    def save_episode(self, info: dict | None = None) -> int:
        """【機能概要】: 溜めたフレームを 1 エピソードとして保存し、エピソードの番号を返す。0 フレームは ValueError 🔵"""
        self._check_open()
        if self._buffered == 0:
            raise ValueError("フレームが 1 枚もありません。エピソードは保存できません")
        index = self._saved
        self._dataset.save_episode()
        self._saved += 1
        self._buffered = 0
        self._log(info or {}, index)
        return index

    def discard_episode(self) -> None:
        """【機能概要】: 溜めたフレームを捨てる (何も溜めていなければ何もしない) 🔵"""
        self._check_open()
        if self._buffered:
            self._dataset.clear_episode_buffer()
            self._buffered = 0

    def log_skipped(self, info: dict) -> None:
        """【機能概要】: 動かず、記録するフレームがなかった試行を sidecar にだけ残す (episode_index は null) 🔵"""
        self._check_open()
        self._log(info, None)

    def finalize(self) -> None:
        """【機能概要】: 書き込みを閉じる (parquet を閉じる)。何度呼んでもよい。閉じた後は読める 🔵"""
        if self._finalized:
            return
        if self._buffered:
            self._dataset.clear_episode_buffer()
            self._buffered = 0
        self._dataset.finalize()
        self._finalized = True

    def __enter__(self) -> "LeRobotWriter":
        return self

    def __exit__(self, exc_type: type | None, exc: BaseException | None, tb: TracebackType | None) -> None:
        self.finalize()


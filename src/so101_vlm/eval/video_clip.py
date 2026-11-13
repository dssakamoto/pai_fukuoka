"""sim の試行を実行動画 (mp4) にする (TASK-0028)。フレームの合成と構成は eval/video.py (純粋ロジック)。

🔵 FR-10 / AC-06: 試行ごとに 指示 -> 検出 (枠) -> 実行 -> 判定 のクリップを作り、つないで 1 本の mp4 にする。
実行中は run_trial の frame_sink で、倍速に合わせて間引きながら描画する (30 fps 固定の動画に N 倍速なら、制御周期の
N * 50 / 30 周期ごとに 1 枚)。重ね表示は試行が終わってから (検出結果と成否が分かってから) 合成する。
"""

from pathlib import Path
from types import TracebackType

import mujoco
import numpy as np

from so101_vlm.eval.runner import Summary
from so101_vlm.eval.scene_spec import SCENE_SPEC
from so101_vlm.eval.trial import TrialOutcome, run_trial
from so101_vlm.eval.video import (
    Overlay,
    VideoStyle,
    clip_duration_s,
    compose_frame,
    make_style,
    plan_clip,
    result_label,
    validate_speed,
)
from so101_vlm.record.sim_episode import keep_frame
from so101_vlm.safety.config import load_safety_config

DEFAULT_FPS = 30


class VideoWriter:
    """【機能概要】: mp4 を 1 フレームずつ書く。最初のフレームで開く (フレームが 1 枚もなければファイルを作らない) 🔵

    with で使う。フレームは uint8 の (H, W, 3) で、最初のフレームと同じ大きさ (違えば ValueError)。with の外は RuntimeError。
    """

    def __init__(self, path: str | Path, fps: int = DEFAULT_FPS) -> None:
        clip_duration_s(0, fps)  # fps の検査
        self.path = Path(path)
        self.fps = fps
        self.n_frames = 0
        self._writer = None
        self._size: tuple[int, int] | None = None
        self._open = False

    @property
    def duration_s(self) -> float:
        """【機能概要】: 今までに書いたフレームの長さ [s] 🔵"""
        return clip_duration_s(self.n_frames, self.fps)

    def __enter__(self) -> "VideoWriter":
        self._open = True
        return self

    def append(self, frame: np.ndarray) -> None:
        """【機能概要】: 1 フレーム書く 🔵"""
        if not self._open:
            raise RuntimeError("VideoWriter は with の中で使ってください")
        if not isinstance(frame, np.ndarray) or frame.dtype != np.uint8 or frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError("frame は uint8 の (H, W, 3) で指定してください")
        if self._size is not None and frame.shape[:2] != self._size:
            raise ValueError(f"フレームの大きさが途中で変わりました: {frame.shape[:2]} (最初は {self._size})")
        if self._writer is None:
            import imageio.v2 as imageio

            self.path.parent.mkdir(parents=True, exist_ok=True)
            self._writer = imageio.get_writer(str(self.path), fps=self.fps, codec="libx264", macro_block_size=1)
            self._size = frame.shape[:2]
        self._writer.append_data(frame)
        self.n_frames += 1

    def __exit__(self, exc_type: type | None, exc: BaseException | None, tb: TracebackType | None) -> None:
        self._open = False
        if self._writer is not None:
            self._writer.close()
            self._writer = None


class FrameCollector:
    """【機能概要】: run_trial の start_sink (begin) / frame_sink (on_step) に渡して、動画に使うフレームを集める 🔵

    initial: 配置直後 (動く前) の画像、frames / labels: 残した周期の画像と区間名。残す周期は
    keep_frame(k, 制御周期, fps / speed)。周期の番号が連続でなければ RuntimeError。close で Renderer を閉じる。
    """

    def __init__(self, *, fps: int, speed: int, src_dt: float | None = None) -> None:
        self._sample_fps = fps // validate_speed(speed, fps)
        self._src_dt = load_safety_config().control_dt if src_dt is None else src_dt
        keep_frame(1, self._src_dt, self._sample_fps)  # 組み合わせを使う前に検査する
        self._renderer: mujoco.Renderer | None = None
        self._last_index = 0
        self.initial: np.ndarray | None = None
        self.frames: list[np.ndarray] = []
        self.labels: list[str] = []

    def _render(self, data: mujoco.MjData) -> np.ndarray:
        self._renderer.update_scene(data, camera=SCENE_SPEC.camera_name)  # type: ignore[union-attr]
        return self._renderer.render().copy()  # type: ignore[union-attr]

    def begin(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        """【機能概要】: 試行の開始 (配置と撮影の直後)。Renderer を作り、最初の画像を撮る 🔵"""
        self._renderer = mujoco.Renderer(model, height=SCENE_SPEC.image_height, width=SCENE_SPEC.image_width)
        self.initial = self._render(data)
        self._last_index = 0

    def on_step(self, model: mujoco.MjModel, data: mujoco.MjData, index: int, label: str) -> None:
        """【機能概要】: 周期 index (1 始まり) の送信の後。残す周期なら描画して溜める 🔵"""
        if self._renderer is None:
            raise RuntimeError("begin() より先に on_step が呼ばれました")
        if index != self._last_index + 1:
            raise RuntimeError(f"周期の番号が連続していません: {self._last_index} の次に {index}")
        self._last_index = index
        if keep_frame(index, self._src_dt, self._sample_fps):
            self.frames.append(self._render(data))
            self.labels.append(label)

    def close(self) -> None:
        """【機能概要】: Renderer を閉じる。何度呼んでもよい 🔵"""
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None


def clip_trial(
    video: VideoWriter,
    seed: int,
    colour: str,
    instruction: str,
    vlm: object,
    *,
    speed: int = 2,
    style: VideoStyle | None = None,
    trial_label: str | None = None,
    hold_task_s: float = 1.5,
    hold_detect_s: float = 2.0,
    hold_result_s: float = 2.5,
    **trial_kwargs: object,
) -> TrialOutcome:
    """【機能概要】: run_trial を実行し、その試行のクリップ (指示 -> 検出 -> 実行 -> 判定) を video に追記する 🔵

    run_trials の trial_fn に functools.partial(clip_trial, video, speed=...) で渡せる。動かなかった試行も、
    指示・検出・判定の静止画のクリップになる。想定外の例外はそのまま出し、その試行のフレームは書かない。
    不正な speed は、何も実行する前に ValueError。frame_sink / start_sink はここで使うので trial_kwargs には渡せない。
    """
    validate_speed(speed, video.fps)
    style = make_style() if style is None else style
    collector = FrameCollector(fps=video.fps, speed=speed)
    try:
        outcome = run_trial(
            seed, colour, instruction, vlm, frame_sink=collector.on_step, start_sink=collector.begin, **trial_kwargs
        )  # type: ignore[arg-type]
    finally:
        collector.close()
    if collector.initial is None:
        raise RuntimeError("試行の最初の画像が撮れていません")
    text = result_label(outcome.success, None if outcome.failure is None else outcome.failure.value, style.lang)
    for spec in plan_clip(
        collector.labels, fps=video.fps, hold_task_s=hold_task_s, hold_detect_s=hold_detect_s, hold_result_s=hold_result_s
    ):
        base = collector.initial if spec.source is None else collector.frames[spec.source]
        at_result = spec.stage == "result"
        overlay = Overlay(
            instruction=instruction, stage=spec.stage, phase=spec.phase, plan=outcome.plan,
            result_text=text if at_result else None, result_ok=outcome.success if at_result else None,
            speed=speed, header_right=trial_label,
        )  # fmt: skip
        video.append(compose_frame(base, overlay, style))
    return outcome


def summary_lines(summary: Summary, lang: str) -> list[str]:
    """【機能概要】: 最後の集計カードに出す行 (成功率、失敗の種類別の件数、検出誤差の平均) 🔵"""
    ja = lang == "ja"
    rate = "-" if summary.success_rate is None else f"{100 * summary.success_rate:.1f}%"
    lines = [f"{'成功率' if ja else 'Success rate'}: {rate} ({summary.n_success}/{summary.n_trials})"]
    lines += [f"{kind}: {count}" for kind, count in sorted(summary.by_failure.items(), key=lambda kv: (-kv[1], kv[0]))]
    if summary.mean_pick_error_mm is not None:
        lines.append(f"{'検出誤差 (平均)' if ja else 'Mean detection error'}: {summary.mean_pick_error_mm:.1f} mm")
    return lines


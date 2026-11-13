"""sim の 1 試行を LeRobot の 1 エピソードとして記録する (TASK-0027)。

🔵 FR-09: 実行ごとに 画像・関節状態・行動・指示・結果 を記録する。run_trial の start_sink / frame_sink に繋ぎ、
制御周期 (50 Hz) から記録の fps (30) へ間引きながら 1 周期ずつ LeRobot に書く (画像を溜め込まない)。
LeRobot の流儀に合わせて、フレーム t の観測 (画像・状態) は「行動 t を送る前」のもの、行動 t は t 番目の指令 (目標関節角)。
動かなかった試行 (VLM・知覚・計画の失敗) はフレームがないのでエピソードにせず、sidecar にだけ残す。
"""

import math

import mujoco
import numpy as np

from so101_vlm.eval.scene_spec import SCENE_SPEC
from so101_vlm.eval.trial import TrialOutcome, run_trial
from so101_vlm.record.lerobot_writer import LeRobotWriter
from so101_vlm.robot.model_paths import JOINT_NAMES
from so101_vlm.safety.config import load_safety_config

_EPS = 1e-9  # 2 進の誤差で、本来残すフレームを落とさないための余裕


# 🔵 Intent: 時刻 k*dt を fps の格子に切り捨てて、格子の番号が進んだ周期だけ残す (50 Hz -> 30 Hz なら 5 周期に 3 回)。
def keep_frame(k: int, src_dt: float, fps: int) -> bool:
    """【機能概要】: k 番目 (1 始まり) の周期を記録に残すか。fps が制御周期より速い (補間が要る) なら ValueError 🔵"""
    if isinstance(k, bool) or not isinstance(k, int) or k < 1:
        raise ValueError(f"k は 1 以上の整数で指定してください: {k!r}")
    if not isinstance(src_dt, int | float) or isinstance(src_dt, bool) or not math.isfinite(src_dt) or src_dt <= 0:
        raise ValueError(f"src_dt は正の有限値で指定してください: {src_dt!r}")
    if isinstance(fps, bool) or not isinstance(fps, int) or fps < 1:
        raise ValueError(f"fps は 1 以上の整数で指定してください: {fps!r}")
    ratio = fps * src_dt
    if ratio > 1 + _EPS:
        raise ValueError(f"fps={fps} は制御周期 ({1 / src_dt:.1f} Hz) より速く、間引きではなく補間が必要です")
    return math.floor(k * ratio + _EPS) > math.floor((k - 1) * ratio + _EPS)


def outcome_to_info(outcome: TrialOutcome) -> dict:
    """【機能概要】: 試行の結果を、sidecar に書く JSON 向けの dict にする (指示・検出結果・推定・成否・失敗の種類) 🔵"""

    def det(d):
        return {"label": d.label, "bbox_px": list(d.bbox_px), "confidence": d.confidence}

    plan = outcome.plan
    return {
        "seed": outcome.seed, "colour": outcome.colour, "instruction": outcome.instruction,
        "success": outcome.success, "failure": None if outcome.failure is None else outcome.failure.value,
        "error": outcome.error, "pick_error_mm": outcome.pick_error_mm,
        "pick_estimate": None if outcome.pick_estimate is None else list(outcome.pick_estimate),
        "place_estimate": None if outcome.place_estimate is None else list(outcome.place_estimate),
        "plan": None if plan is None else {"pick": det(plan.pick), "place": det(plan.place)},
        "vlm_time_s": outcome.vlm_time_s,
    }  # fmt: skip


class SimEpisodeRecorder:
    """【機能概要】: run_trial の start_sink (begin) と frame_sink (on_step) に渡して、1 エピソードを書く 🔵

    begin で最初の観測を取り、on_step (周期 k の送信の後) で「前の観測 + k 番目の行動」を 1 フレームとして書く。
    画像は残すフレームの観測としてだけ描く (専用の Renderer を 1 つ使い回す)。使い終わったら close。
    on_step の周期の番号が連続でなければ RuntimeError (記録のずれを黙って通さない)。
    """

    def __init__(self, writer: LeRobotWriter, *, task: str, src_dt: float | None = None) -> None:
        self._writer = writer
        self._task = task
        self._src_dt = load_safety_config().control_dt if src_dt is None else src_dt
        keep_frame(1, self._src_dt, writer.fps)  # fps と制御周期の組を、使う前に検査する
        self._renderer: mujoco.Renderer | None = None
        self._qpos_idx: np.ndarray | None = None
        self._ctrl_idx: np.ndarray | None = None
        self._prev_state: np.ndarray | None = None
        self._prev_image: np.ndarray | None = None
        self._last_index = 0
        self.n_frames = 0

    def _keep(self, k: int) -> bool:
        return keep_frame(k, self._src_dt, self._writer.fps)

    def _state(self, data: mujoco.MjData) -> np.ndarray:
        return np.array(data.qpos[self._qpos_idx], dtype=np.float32)

    def _render(self, data: mujoco.MjData) -> np.ndarray:
        self._renderer.update_scene(data, camera=SCENE_SPEC.camera_name)  # type: ignore[union-attr]
        return self._renderer.render().copy()  # type: ignore[union-attr]

    def begin(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        """【機能概要】: 試行の開始。関節の添字を引き、最初の観測 (状態、必要なら画像) を取る 🔵"""
        h, w, _ = self._writer.image_shape
        self._renderer = mujoco.Renderer(model, height=h, width=w)
        self._qpos_idx = np.array([model.joint(n).qposadr[0] for n in JOINT_NAMES])
        self._ctrl_idx = np.array([model.actuator(n).id for n in JOINT_NAMES])
        self._prev_state = self._state(data)
        self._prev_image = self._render(data) if self._keep(1) else None
        self._last_index = 0
        self.n_frames = 0

    def on_step(self, model: mujoco.MjModel, data: mujoco.MjData, index: int, label: str) -> None:
        """【機能概要】: 周期 index (1 始まり) の送信の後。残す周期なら 1 フレーム書き、次の観測を取る 🔵"""
        if self._renderer is None:
            raise RuntimeError("begin() より先に on_step が呼ばれました")
        if index != self._last_index + 1:
            raise RuntimeError(f"周期の番号が連続していません: {self._last_index} の次に {index}")
        self._last_index = index
        if self._keep(index):
            action = np.array(data.ctrl[self._ctrl_idx], dtype=np.float32)
            self._writer.add_frame(self._prev_image, self._prev_state, action, self._task)  # type: ignore[arg-type]
            self.n_frames += 1
        self._prev_state = self._state(data)
        self._prev_image = self._render(data) if self._keep(index + 1) else None

    def close(self) -> None:
        """【機能概要】: Renderer を閉じる。何度呼んでもよい 🔵"""
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None


def record_trial(
    writer: LeRobotWriter, seed: int, colour: str, instruction: str, vlm: object, **trial_kwargs: object
) -> TrialOutcome:
    """【機能概要】: run_trial を実行し、動いた試行を 1 エピソードとして書く。run_trials の trial_fn に functools.partial で渡せる 🔵

    動かなかった試行はエピソードにせず sidecar にだけ残す。想定外の例外では、途中のフレームを捨てて例外をそのまま出す
    (次の試行は普通に記録できる)。frame_sink / start_sink は、ここで使うので trial_kwargs には渡せない。
    """
    recorder = SimEpisodeRecorder(writer, task=instruction)
    try:
        outcome = run_trial(
            seed, colour, instruction, vlm, frame_sink=recorder.on_step, start_sink=recorder.begin, **trial_kwargs
        )  # type: ignore[arg-type]
    except BaseException:
        recorder.close()
        writer.discard_episode()
        raise
    recorder.close()
    info = outcome_to_info(outcome)
    if recorder.n_frames:
        writer.save_episode(info)
    else:
        writer.discard_episode()
        writer.log_skipped(info)
    return outcome

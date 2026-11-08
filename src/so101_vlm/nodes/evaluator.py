"""task / evaluator / recorder ノード (TASK-0025)。

🔵 evaluator: sim の真値 (world) で成功判定し、失敗を分類する。判定・分類は run_trial / run_goal と同じライブラリ
(judge_in_box / PickTracker / classify_failure) を使う。result を出したら stop を全ノードに送って終わる。
入力は到着順が保証されないので、必要なものが揃うまで溜めて、揃ったら進める (dora でも、ローカルランナーでも同じ)。
recorder: 試行の記録 (JSON)。🟡 LeRobot 形式での記録は TASK-0027 で、この位置に差し替える。
"""

import json
from pathlib import Path

import numpy as np

from so101_vlm.eval.eval_config import load_eval_config
from so101_vlm.eval.failure import classify_failure
from so101_vlm.eval.fixed_pick import COLOUR_TO_BODY
from so101_vlm.eval.judge import judge_in_box
from so101_vlm.eval.pick_tracker import PickTracker, block_linear_velocity
from so101_vlm.nodes.base import NodeBase
from so101_vlm.nodes.node_config import NodeConfig
from so101_vlm.nodes.registry_types import TaskSpec
from so101_vlm.nodes.state_codec import apply_world
from so101_vlm.policy.trajectory_config import load_trajectory_config
from so101_vlm.robot.sim import SimRobot

_MM = 1000.0


class TaskNode(NodeBase):
    """【機能概要】: 起点。試行の指定 (seed / colour / instruction) を 1 回だけ task で出して終わる 🔵"""

    def __init__(self, config: NodeConfig, task: TaskSpec) -> None:
        super().__init__(config)
        self._task = task

    def start(self) -> None:
        t = self._task
        self.emit("task", {"seed": t.seed, "colour": t.colour, "instruction": t.instruction}, {})
        self.finished = True

    def handle(self, name: str, payload: object, meta: dict) -> None:
        pass


class EvaluatorNode(NodeBase):
    """【機能概要】: 結果 (result) を作る。失敗の入力 (failure_*) か、実行の完了 (motion_done) と全周期の world が揃ったとき 🔵

    入力: task、world (seq 0 = 初期、1.. = 各周期)、step ({"seq", "label"})、motion_done、targets_3d、failure_vlm /
    failure_perception / failure_policy。出力: result、stop。
    result のキーは TrialOutcome (eval/trial.py) と揃える: success / failure / error / pick_estimate / place_estimate /
    pick_error_mm / planning_error / planning_waypoint / judge / motion / other_block_moved / trace。
    """

    def __init__(self, config: NodeConfig) -> None:
        super().__init__(config)
        self._eval = load_eval_config()
        self._traj_cfg = load_trajectory_config()
        self._sim = SimRobot(config.scene_path)
        self._task: TaskSpec | None = None
        self._init_world: np.ndarray | None = None
        self._worlds: dict[int, np.ndarray] = {}
        self._labels: dict[int, str] = {}
        self._targets: dict | None = None
        self._motion_done: dict | None = None
        self._failure: dict | None = None
        self._tracker: PickTracker | None = None
        self._truth: np.ndarray | None = None
        self._processed = 0

    # --- 入力 -----------------------------------------------------------------
    def handle(self, name: str, payload: object, meta: dict) -> None:
        if self.finished:
            return
        if name == "task":
            self._task = TaskSpec(**payload)  # type: ignore[arg-type]
        elif name == "world":
            if meta.get("phase") == "init":
                self._init_world = np.asarray(payload)
            else:
                self._worlds[int(meta["seq"])] = np.asarray(payload)
        elif name == "step":
            self._labels[int(payload["seq"])] = payload["label"]  # type: ignore[index]
        elif name == "motion_done":
            self._motion_done = payload  # type: ignore[assignment]
        elif name == "targets_3d":
            self._targets = payload  # type: ignore[assignment]
        elif name.startswith("failure"):
            self._failure = payload  # type: ignore[assignment]
        self._progress()

    # --- 進める ------------------------------------------------------------------
    def _setup_tracking(self) -> bool:
        if self._tracker is not None:
            return True
        if self._task is None or self._init_world is None:
            return False
        model, data = self._sim.mujoco_handles()
        apply_world(model, data, self._init_world)
        body = COLOUR_TO_BODY[self._task.colour]
        self._truth = np.array(data.xpos[model.body(body).id])
        self._tracker = PickTracker(model, data, body, self._traj_cfg.tcp_offset)
        return True

    def _progress(self) -> None:
        if self._task is None:
            return
        if self._failure is not None:
            self._setup_tracking()  # 真値が分かれば pick_error_mm に使う
            self._finish(self._failure_result())
            return
        if self._motion_done is None or not self._setup_tracking():
            return
        model, data = self._sim.mujoco_handles()
        while (self._processed + 1) in self._worlds and (self._processed + 1) in self._labels:
            seq = self._processed + 1
            apply_world(model, data, self._worlds.pop(seq))
            self._tracker.update(self._labels.pop(seq))  # type: ignore[union-attr]
            self._processed = seq
        if self._processed >= int(self._motion_done["n_steps"]):
            self._finish(self._executed_result())

    # --- 結果 ---------------------------------------------------------------------
    def _base_result(self) -> dict:
        t = self._task
        pick = None if self._targets is None else list(self._targets["pick"])
        error = None
        if pick is not None and self._truth is not None:
            error = float(np.linalg.norm(np.array(pick) - self._truth) * _MM)
        return {
            "seed": t.seed, "colour": t.colour, "instruction": t.instruction,  # type: ignore[union-attr]
            "success": False, "failure": None, "error": None,
            "pick_estimate": pick,
            "place_estimate": None if self._targets is None else list(self._targets["place"]),
            "pick_error_mm": error, "planning_error": None, "planning_waypoint": None,
            "judge": None, "motion": None, "other_block_moved": False, "trace": None,
        }

    def _failure_result(self) -> dict:
        f = self._failure
        out = self._base_result()
        out["failure"] = f["kind"]  # type: ignore[index]
        out["error"] = f.get("error")  # type: ignore[union-attr]
        if f["kind"] == "planning_failure":  # type: ignore[index]
            out["planning_error"], out["planning_waypoint"] = f.get("reason"), f.get("waypoint")  # type: ignore[union-attr]
        return out

    def _executed_result(self) -> dict:
        model, data = self._sim.mujoco_handles()
        body_id = int(model.body(COLOUR_TO_BODY[self._task.colour]).id)  # type: ignore[union-attr]
        judge = judge_in_box(data.xpos[body_id], block_linear_velocity(model, data, body_id), self._eval.judge)
        trace = self._tracker.finish()  # type: ignore[union-attr]
        kind = classify_failure(trace, judge, self._eval.failure)
        out = self._base_result()
        out.update(
            success=judge.success, failure=None if kind is None else kind.value,
            judge={
                "reasons": [r.value for r in judge.reasons], "offset_xy": list(judge.offset_xy),
                "height": judge.height, "speed": judge.speed,
            },
            motion=self._motion_done,
            other_block_moved=bool(trace.max_other_move > self._eval.failure.other_move_m),
            trace={k: float(v) for k, v in vars(trace).items()},
        )
        return out

    def _finish(self, result: dict) -> None:
        self.finished = True
        self.emit("result", result, {})
        self.emit("stop", {}, {})


class RecorderNode(NodeBase):
    """【機能概要】: 試行の記録を JSON で書く。result が来たら、検出結果・3 次元目標と一緒に保存して終わる 🟡

    書くもの: <output_dir>/trial_seed<N>_<colour>.json (全部入り)、<output_dir>/result.json (result だけ)。
    """

    def __init__(self, config: NodeConfig) -> None:
        super().__init__(config)
        self._detections: dict | None = None
        self._targets: dict | None = None
        self.written: list[Path] = []

    def handle(self, name: str, payload: object, meta: dict) -> None:
        if name == "detections":
            self._detections = payload  # type: ignore[assignment]
        elif name == "targets_3d":
            self._targets = payload  # type: ignore[assignment]
        elif name == "result":
            out_dir = self.config.output_dir
            out_dir.mkdir(parents=True, exist_ok=True)
            trial = out_dir / f"trial_seed{payload['seed']}_{payload['colour']}.json"  # type: ignore[index]
            record = {"result": payload, "detections": self._detections, "targets_3d": self._targets}
            trial.write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8")
            (out_dir / "result.json").write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
            self.written.append(trial)
            self.finished = True


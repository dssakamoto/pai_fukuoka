"""知覚から計画までのノード: vlm_planner / perception / policy (TASK-0025)。

🔵 design.md 3 章。ノードは入出力だけを扱い、処理は既存のライブラリ (VlmClient、bbox_to_base_point、IkPolicy) を呼ぶ。
失敗 (PlanError / 知覚の ValueError / TrajectoryError) は failure_* 出力で evaluator に知らせ、後段には何も送らない
(FR-03 / FR-06: ロボットは動かない)。それ以外の例外 (認証エラーなど) は握りつぶさない。
"""

import dataclasses
import time
from collections.abc import Callable

import numpy as np

from so101_vlm.eval.eval_config import load_eval_config
from so101_vlm.eval.layout import sample_layout
from so101_vlm.eval.oracle_vlm import OracleVlm
from so101_vlm.eval.trial import estimate_goal
from so101_vlm.nodes.base import NodeBase
from so101_vlm.nodes.node_config import NodeConfig
from so101_vlm.nodes.registry_types import TaskSpec
from so101_vlm.nodes.state_codec import unpack_state
from so101_vlm.perception.calibration import load_calibration
from so101_vlm.perception.projection import Camera
from so101_vlm.policy.base import Goal, Observation
from so101_vlm.policy.ik_policy import IkPolicy
from so101_vlm.policy.trajectory import JointTrajectory, TrajectoryError
from so101_vlm.robot.sim import SimRobot
from so101_vlm.vlm import create_client
from so101_vlm.vlm.config import load_vlm_config
from so101_vlm.vlm.schema import Detection, PlanError, PlanResult

ClientFactory = Callable[[NodeConfig, TaskSpec], object]


def _det_to_json(det: Detection) -> dict:
    return {"label": det.label, "bbox_px": list(det.bbox_px), "confidence": det.confidence}


def _det_from_json(raw: dict) -> Detection:
    return Detection(raw["label"], tuple(int(v) for v in raw["bbox_px"]), raw["confidence"])


def default_client(config: NodeConfig, task: TaskSpec) -> object:
    """【機能概要】: 設定のプロバイダの VlmClient。oracle は真の配置から作る偽 VLM (キー不要)、他は create_client 🔵"""
    if config.vlm_provider == "oracle":
        place = load_eval_config().placement
        layout = sample_layout(task.seed, place.n_blocks, place.area, place.min_gap, place.max_attempts)
        return OracleVlm(layout, task.colour, load_calibration(config.calibration_path))
    return create_client(load_vlm_config(), config.vlm_provider)


class VlmPlannerNode(NodeBase):
    """【機能概要】: task (指示) と rgb が揃ったら VlmClient を呼び、検出結果を detections で出す 🔵

    入力: task, rgb。出力: detections、または failure_vlm ({"kind": "vlm_failure", "error"})。
    キーのないプロバイダは、起動時 (この初期化) に MissingApiKeyError で止まる (NFR-03)。
    """

    def __init__(self, config: NodeConfig, client_factory: ClientFactory | None = None) -> None:
        super().__init__(config)
        self._factory = client_factory or default_client
        self._task: TaskSpec | None = None
        self._rgb: np.ndarray | None = None
        self._done = False
        if client_factory is None and config.vlm_provider != "oracle":
            default_client(config, TaskSpec(0, "red", "startup check"))  # キー未設定を早く知る

    def handle(self, name: str, payload: object, meta: dict) -> None:
        if name == "task":
            self._task = TaskSpec(**payload)  # type: ignore[arg-type]
        elif name == "rgb":
            self._rgb = np.asarray(payload)
        if self._task is None or self._rgb is None or self._done:
            return
        self._done = True
        client = self._factory(self.config, self._task)
        started = time.perf_counter()
        try:
            plan: PlanResult = client.plan(self._rgb, self._task.instruction)  # type: ignore[attr-defined]
        except PlanError as exc:
            self.emit("failure_vlm", {"kind": "vlm_failure", "error": str(exc)}, {})
            return
        self.emit(
            "detections",
            {
                "pick": _det_to_json(plan.pick), "place": _det_to_json(plan.place),
                "provider": getattr(client, "provider", self.config.vlm_provider),
                "model": getattr(client, "model", ""), "vlm_time_s": time.perf_counter() - started,
            },
            {},
        )


class PerceptionNode(NodeBase):
    """【機能概要】: 検出枠 + 深度 + 内部パラメータ から、ロボット座標系の目標を求める 🔵

    入力: task (指示を引き継ぐ)、detections、depth、intrinsics。出力: targets_3d ({"pick", "place", "instruction"})、
    または failure_perception。T_base_cam は設定のキャリブレーションファイルから読む。
    """

    def __init__(self, config: NodeConfig) -> None:
        super().__init__(config)
        self._calibration = load_calibration(config.calibration_path)
        self._have: dict[str, object] = {}
        self._done = False

    def handle(self, name: str, payload: object, meta: dict) -> None:
        self._have[name] = payload
        if self._done or not {"task", "detections", "depth", "intrinsics"} <= self._have.keys():
            return
        self._done = True
        task = TaskSpec(**self._have["task"])  # type: ignore[arg-type]
        det = self._have["detections"]
        plan = PlanResult(_det_from_json(det["pick"]), _det_from_json(det["place"]))  # type: ignore[index]
        fx, fy, cx, cy, w, h = (float(v) for v in self._have["intrinsics"])  # type: ignore[attr-defined]
        calibration = dataclasses.replace(self._calibration, camera=Camera(fx, fy, cx, cy, int(w), int(h)))
        try:
            goal = estimate_goal(plan, np.asarray(self._have["depth"]), calibration, task.instruction)
        except ValueError as exc:
            self.emit("failure_perception", {"kind": "perception_failure", "error": str(exc)}, {})
            return
        self.emit(
            "targets_3d",
            {
                "pick": [float(v) for v in goal.pick_target_base],
                "place": [float(v) for v in goal.place_target_base],
                "instruction": task.instruction,
            },
            {},
        )


def trajectory_to_json(traj: JointTrajectory) -> dict:
    """【機能概要】: 実行に必要な部分 (時刻・関節角・区間名・経由点の TCP 目標) を JSON にできる dict にする 🔵"""
    records = traj.metadata.get("waypoints") or []
    return {
        "times": traj.times.tolist(), "q": traj.q.tolist(), "labels": list(traj.labels),
        "segments": list(traj.segments),
        "waypoints": [{"name": r.name, "target": [float(v) for v in r.target]} for r in records],  # type: ignore[attr-defined]
    }


class PolicyNode(NodeBase):
    """【機能概要】: 目標と初期の状態から IkPolicy で軌道を作る。作れなければ動かさない (FR-06) 🔵

    入力: targets_3d、robot_state (最初の 1 件 = 初期姿勢を使う)。出力: joint_trajectory、または failure_policy
    ({"kind": "planning_failure", "reason", "waypoint", "plan_time_s"})。IK のモデルは設定のシーンから読む。
    """

    def __init__(self, config: NodeConfig) -> None:
        super().__init__(config)
        self._model = SimRobot(config.scene_path).mujoco_handles()[0]
        self._policy = IkPolicy(self._model)
        self._targets: dict | None = None
        self._state: np.ndarray | None = None
        self._done = False

    def handle(self, name: str, payload: object, meta: dict) -> None:
        if name == "targets_3d":
            self._targets = payload  # type: ignore[assignment]
        elif name == "robot_state" and self._state is None:
            self._state = np.asarray(payload)
        if self._done or self._targets is None or self._state is None:
            return
        self._done = True
        goal = Goal(
            pick_target_base=np.array(self._targets["pick"]),
            place_target_base=np.array(self._targets["place"]),
            instruction=self._targets["instruction"],
        )
        started = time.perf_counter()
        try:
            traj = self._policy.act(Observation.from_state(unpack_state(self._state)), goal)
        except TrajectoryError as exc:
            self.emit(
                "failure_policy",
                {"kind": "planning_failure", "reason": exc.reason, "waypoint": exc.waypoint,
                 "plan_time_s": time.perf_counter() - started},
                {},
            )
            return
        out = trajectory_to_json(traj)
        out["plan_time_s"] = time.perf_counter() - started
        self.emit("joint_trajectory", out, {})

"""MuJoCo 上の Robot 実装 SimRobot (TASK-0004)。

関節・アクチュエータは名前で引いて対応付ける(位置の決め打ちはしない)。
範囲のクリップはしない(安全フィルタ TASK-0010 の責務)。ctrlrange 外の値は
MuJoCo が力の計算時に内部でクランプするが、その挙動に依存する保証はしない。
"""

from pathlib import Path

import mujoco
import numpy as np

from so101_vlm.robot.base import JOINT_NAMES, RobotState, validate_q_target
from so101_vlm.robot.model_paths import SO101_SCENE_PATH

# 【既定サブステップ数】: send_command 1 回あたりの mj_step 回数 🟡
DEFAULT_N_SUBSTEPS = 10


def _load_model(scene_path: Path) -> mujoco.MjModel:
    """【機能概要】: シーンを読み込む。失敗時はパスを含むエラーにする。🟡"""
    path = Path(scene_path)
    if not path.is_file():
        raise FileNotFoundError(f"scene file not found: {path}")
    try:
        return mujoco.MjModel.from_xml_path(str(path))
    except ValueError as exc:
        raise ValueError(f"failed to load scene {path}: {exc}") from exc


def _name_id(model: mujoco.MjModel, obj: mujoco.mjtObj, name: str) -> int:
    """【機能概要】: 名前から id を引く。無ければ名前入りの ValueError。🟡"""
    idx = mujoco.mj_name2id(model, obj, name)
    if idx < 0:
        kind = mujoco.mjtObj(obj).name.removeprefix("mjOBJ_").lower()
        raise ValueError(f"{kind} '{name}' not found in model")
    return idx


class SimRobot:
    """【機能概要】: MuJoCo シミュレーション上の Robot。🔵

    send_command 1 回で n_substeps 回 mj_step を進める。MjData は get_state では外に出さない
    (sim 専用の組み立て層だけが mujoco_handles で受け取る)。
    """

    def __init__(self, scene_path: Path = SO101_SCENE_PATH, n_substeps: int = DEFAULT_N_SUBSTEPS) -> None:
        if n_substeps < 1:
            raise ValueError(f"n_substeps must be >= 1, got {n_substeps}")
        self._model = _load_model(scene_path)
        self._data = mujoco.MjData(self._model)
        self._n_substeps = int(n_substeps)
        self._resolve_indices()
        self._closed = False

    def _resolve_indices(self) -> None:
        # 【名前解決】: JOINT_NAMES 順に qpos / dof / actuator の添字を求める
        m = self._model
        joint_ids = [_name_id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in JOINT_NAMES]
        self._qpos_idx = np.array([m.jnt_qposadr[j] for j in joint_ids])
        self._dof_idx = np.array([m.jnt_dofadr[j] for j in joint_ids])
        self._act_idx = np.array(
            [_name_id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in JOINT_NAMES]
        )

    @property
    def joint_names(self) -> tuple[str, ...]:
        return JOINT_NAMES

    def _ensure_open(self) -> None:
        if self._closed:
            raise RuntimeError("SimRobot is closed")

    def get_state(self) -> RobotState:
        """【機能概要】: 現在の関節角・速度・時刻のコピーを返す(副作用なし)。🟡"""
        self._ensure_open()
        d = self._data
        # 【コピー】: 防御コピーは RobotState 側の 1 回のみ(ここの fancy indexing は添字の gather)
        return RobotState(
            q=d.qpos[self._qpos_idx], dq=d.qvel[self._dof_idx], timestamp=float(d.time)
        )

    def send_command(self, q_target: np.ndarray) -> None:
        """【機能概要】: 関節位置目標 [rad] (6,) を設定し n_substeps 進める。🔵"""
        self._ensure_open()
        target = validate_q_target(q_target)  # 入力は変更されない
        self._data.ctrl[self._act_idx] = target
        for _ in range(self._n_substeps):
            mujoco.mj_step(self._model, self._data)

    def mujoco_handles(self) -> tuple[mujoco.MjModel, mujoco.MjData]:
        """【機能概要】: (MjModel, MjData) を返す。sim 専用。🔴

        シーンの準備(ブロックの配置)、真値の読み取り、描画に使う。呼び出し側は mj_step を呼ばない
        (時間は send_command だけで進める)。Robot Protocol には入れない(実機に MuJoCo はない)。
        閉じた後は RuntimeError。
        """
        self._ensure_open()
        return self._model, self._data

    def close(self) -> None:
        """【機能概要】: 終了する。何度呼んでもよい。🟡"""
        self._closed = True

    def __enter__(self) -> "SimRobot":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

"""safety テスト共有 fixture (TASK-0010, Red フェーズ)。

テストケース定義: docs/implements/so101-vlm-pickplace/TASK-0010/safety-testcases.md
信頼性: 🔵 要件・設計・実測に基づく / 🟡 妥当な推測 / 🔴 要件にない仮定
"""

import mujoco
import numpy as np
import pytest

from so101_vlm.eval.validation import BLOCK_AREA_X, BLOCK_AREA_Y
from so101_vlm.eval.geometry import get_geometry
from so101_vlm.eval.scene_spec import SCENE_SPEC
from so101_vlm.robot.model_paths import PICKPLACE_SCENE_PATH, SO101_SCENE_PATH


@pytest.fixture(scope="session")
def task_points() -> np.ndarray:
    """【タスク点】: SCENE_SPEC から 172 点を生成する (testcases 0.4)。🔵"""
    xs = np.round(np.linspace(*BLOCK_AREA_X, 9), 4)
    ys = np.round(np.linspace(*BLOCK_AREA_Y, 9), 4)
    grasp_z = SCENE_SPEC.block_rest_z
    approach_z = grasp_z + 0.08
    pts = [(x, y, z) for x in xs for y in ys for z in (grasp_z, approach_z)]
    bx, by = get_geometry().box_default_xy
    h = SCENE_SPEC.box_inner_half_size
    pts += [(bx, by, 0.06), (bx, by, 0.14)]
    pts += [(bx + sx * h, by + sy * h, z) for sx in (-1, 1) for sy in (-1, 1) for z in (0.06, 0.14)]
    return np.array(pts, dtype=np.float64)


@pytest.fixture(scope="module")
def scene_model() -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(str(SO101_SCENE_PATH))


@pytest.fixture(scope="module")
def pickplace_model() -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(str(PICKPLACE_SCENE_PATH))

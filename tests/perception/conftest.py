"""calibration テスト共有のフィクスチャ (test_calibration.py / test_calibration_io.py)。"""

import copy

import mujoco
import pytest
from so101_vlm.perception.calibration import calibration_from_mujoco, load_calibration
from so101_vlm.robot.model_paths import PICKPLACE_SCENE_PATH

from calibration_support import FY_60_480, SIM_T_ROWS, TINY_XML, H, W, forwarded


@pytest.fixture(scope="module")
def scene_model():
    return mujoco.MjModel.from_xml_path(str(PICKPLACE_SCENE_PATH))


@pytest.fixture
def scene_data(scene_model):
    return forwarded(scene_model)


@pytest.fixture(scope="module")
def tiny_model():
    return mujoco.MjModel.from_xml_string(TINY_XML)


@pytest.fixture
def tiny_data(tiny_model):
    return forwarded(tiny_model)


@pytest.fixture
def tiny_xml_path(tmp_path):
    p = tmp_path / "tiny.xml"
    p.write_text(TINY_XML, encoding="utf-8")
    return p


@pytest.fixture(scope="module")
def sim_calibration(scene_model):
    return calibration_from_mujoco(scene_model, forwarded(scene_model), "cam_top", W, H)


@pytest.fixture(scope="module")
def committed():
    cal = load_calibration()
    if cal.source == "real":
        pytest.skip("configs/calibration.yaml は実機の計測値 (source: real) のため sim との比較を行わない")
    return cal


@pytest.fixture
def valid_doc():
    return copy.deepcopy(
        {
            "version": 1,
            "metadata": {"source": "sim", "camera_name": "cam_top", "base_frame": "baseframe"},
            "camera": {"fx": FY_60_480, "fy": FY_60_480, "cx": 319.5, "cy": 239.5, "width": 640, "height": 480},
            "t_base_cam": copy.deepcopy(SIM_T_ROWS),
        }
    )

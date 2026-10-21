"""calibration テスト (test_calibration*.py) が共有する定数とヘルパ (TASK-0009)。

pytest のフィクスチャは conftest.py に置き、こちらには import して使う定数・関数だけを置く。
信頼性: 🔵 要件・設計・実測に基づく / 🟡 妥当な推測 / 🔴 要件にない決定
"""

import mujoco
import numpy as np
from so101_vlm.eval.scene_spec import SCENE_SPEC
from so101_vlm.perception.calibration import Calibration
from so101_vlm.perception.projection import Camera

W, H = SCENE_SPEC.image_width, SCENE_SPEC.image_height  # 640, 480
FY_60_480 = 415.6921938165307  # (480/2)/tan(30°)。実測で式とビット一致
SIM_T_ROWS = [  # 要件 2.6 の実測値 (cam_top → baseframe, OpenCV 規約)
    [-0.6896566706214934, 0.6405487761198795, -0.3377439001352973, 0.45],
    [0.7241365041670508, 0.6100489807755903, -0.3216621898628844, 0.3],
    [-2.7755575615628914e-17, -0.46640916207337524, -0.8845691004856614, 0.55],
    [0.0, 0.0, 0.0, 1.0],
]
TINY_XML = """
<mujoco>
  <worldbody>
    <geom name="floor" type="plane" size="2 2 0.1"/>
    <body name="robot_base" pos="0.10 -0.05 0.02" euler="0 0 37">
      <site name="baseframe" pos="0.03 0.02 0.01" euler="5 -8 20"/>
    </body>
    <body name="mover" pos="0 0 0.5">
      <camera name="cam_body_track" mode="track" pos="0 0 0.3"/>
    </body>
    <camera name="cam" pos="0.6 0.5 0.7" fovy="50" xyaxes="-0.7 0.7 0 -0.5 -0.5 0.7"/>
    <camera name="cam_trackcom" mode="trackcom" pos="0.6 0.5 0.7"/>
    <camera name="cam_sensor" pos="0.6 0.5 0.7" resolution="320 240" sensorsize="0.004 0.003" focal="0.003 0.003"/>
  </worldbody>
</mujoco>
"""


def forwarded(model):
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    return data


def golden_calibration(source="sim"):
    cam = Camera(fx=FY_60_480, fy=FY_60_480, cx=319.5, cy=239.5, width=640, height=480)
    return Calibration(cam, np.array(SIM_T_ROWS), source, "cam_top", "baseframe")

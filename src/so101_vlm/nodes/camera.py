"""固定カメラの RGB と深度の取得 (TASK-0005)。"""

import mujoco
import numpy as np

DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480


def render_rgbd(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    camera_name: str,
    width: int = DEFAULT_WIDTH,
    height: int = DEFAULT_HEIGHT,
) -> tuple[np.ndarray, np.ndarray]:
    """【機能概要】: 指定カメラで RGB (uint8, HxWx3) と深度 (float32, HxW, m) を新しい配列で返す。🟡

    深度はカメラの光軸方向の距離(z 深度)。何も写らない画素(遠方)は far 平面の値になる。
    """
    if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name) < 0:
        raise ValueError(f"camera '{camera_name}' not found in model")
    renderer = mujoco.Renderer(model, height=height, width=width)
    try:
        renderer.update_scene(data, camera=camera_name)
        rgb = renderer.render().copy()
        renderer.enable_depth_rendering()
        depth = renderer.render().astype(np.float32, copy=True)
        renderer.disable_depth_rendering()
    finally:
        renderer.close()
    return rgb, depth

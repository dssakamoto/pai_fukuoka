"""ピック&プレースシーンの寸法・名前の単一の情報源 (TASK-0005)。

assets/scene_pickplace.xml の値はここと一致させる(tests/test_scene.py が照合する)。
成功判定 (TASK-0015)・初期配置 (TASK-0006)・画像→3 次元変換 (TASK-0008) が使う。単位は m。
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class SceneSpec:
    """【機能概要】: シーンの定数。🟡(寸法は把持の様子を見て調整する前提)"""

    block_names: tuple[str, ...] = ("block_red", "block_blue", "block_green")
    block_half_size: float = 0.015  # 一辺 3 cm
    block_rest_z: float = 0.015  # 床の上で静止したときのブロック中心の高さ
    box_body_name: str = "box"
    box_center_xy: tuple[float, float] = (0.20, 0.19)  # ワールド座標
    box_inner_half_size: float = 0.04  # 内寸 8 x 8 cm の半分
    box_wall_thickness: float = 0.005
    box_floor_top_z: float = 0.005  # 底板の上面
    box_rim_z: float = 0.04  # 壁の上端(縁の高さ)
    work_area_x: tuple[float, float] = (0.15, 0.35)  # 20 cm 四方
    work_area_y: tuple[float, float] = (-0.10, 0.10)
    camera_name: str = "cam_top"
    image_width: int = 640
    image_height: int = 480


SCENE_SPEC = SceneSpec()

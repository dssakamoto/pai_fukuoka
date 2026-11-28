"""ピック&プレースシーンの寸法・名前の情報源 (TASK-0005)。

寸法は assets/scene_pickplace.xml から導く(SGT-003: eval/geometry.py が XML を読む)。
tests/test_scene.py が XML の値をリテラルで固定して照合する。
成功判定 (TASK-0015)・初期配置 (TASK-0006)・画像→3 次元変換 (TASK-0008) が使う。単位は m。
"""

from dataclasses import dataclass

from so101_vlm.eval.geometry import BOX_BODY_NAME, get_geometry

_G = get_geometry()


@dataclass(frozen=True)
class SceneSpec:
    """【機能概要】: シーンの定数。XML から導く。箱の位置は持たない (試行ごとの Scenario。SGT-013) 🟡"""

    block_names: tuple[str, ...] = _G.block_names
    block_half_size: float = _G.block_half_size
    block_rest_z: float = _G.block_rest_z  # 床の上で静止したときのブロック中心の高さ
    box_body_name: str = BOX_BODY_NAME
    box_inner_half_size: float = _G.box_inner_half_size
    box_wall_thickness: float = _G.box_wall_thickness
    box_floor_top_z: float = _G.box_floor_top_z  # 底板の上面
    box_rim_z: float = _G.box_rim_z  # 壁の上端(縁の高さ)
    camera_name: str = _G.camera_name
    image_width: int = _G.image_width
    image_height: int = _G.image_height


SCENE_SPEC = SceneSpec()

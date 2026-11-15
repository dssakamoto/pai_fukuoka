"""シーンの形状を XML の名前付き要素から読む (SGT-003)。

【機能概要】assets/scene_pickplace.xml の body / geom / camera から、ブロックと箱の寸法を導く。
XML の数値を Python に再掲しない(FR-403)。scene_spec.py はここから値を取る。
置き先は領域 (PlaceRegion) として表し、目標 (IK) と成功判定が同じ形状を共有する (FR-005 / FR-006)。

【依存の向き】so101_vlm.eval.scene_spec は import しない(scene_spec が本モジュールを import するため循環する)。
policy / perception から本モジュールを import してはならない(eval 内部専用)。

【前提】🟡 回転 (quat 等)・<default> の class は解決しない(あれば ValueError)。
pos が無い body / geom は MuJoCo の既定どおり 0 0 0 として扱う。
"""

from __future__ import annotations

import functools
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import mujoco

from so101_vlm.robot.model_paths import PICKPLACE_SCENE_PATH
from so101_vlm.validation import require_number

CAMERA_NAME = "cam_top"  # 🔵 XML 外の Python 定数(解像度は XML に無い)
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
BOX_BODY_NAME = "box"  # 🔵 XML の名前
BOX_FLOOR_GEOM = "box_floor"
BOX_WALL_GEOM = "box_wall_px"
BLOCK_BODY_PREFIX = "block_"  # 🟡 worldbody 直下の block_* で、自由関節を持つ body がブロック
FLOAT_DIGITS = 9  # 🟡 導出値の丸め(浮動小数点ノイズ除去)

_UNSUPPORTED_ATTRS = ("quat", "euler", "axisangle", "xyaxes", "zaxis", "class", "childclass")


def _r(value: float) -> float:
    return round(value, FLOAT_DIGITS)


@dataclass(frozen=True)
class PlaceRegion:
    """【機能概要】: 置き先の領域。目標 (IK) と成功判定で共有する 🔵"""

    center_xy: tuple[float, float]
    inner_half_size: float
    rim_z: float
    floor_top_z: float

    def __post_init__(self) -> None:
        if not isinstance(self.center_xy, (tuple, list)) or len(self.center_xy) != 2:
            raise ValueError(f"center_xy は 2 要素で指定してください: center_xy={self.center_xy!r}")
        for i, v in enumerate(self.center_xy):
            require_number(f"center_xy[{i}]", v)
        require_number("inner_half_size", self.inner_half_size)
        require_number("rim_z", self.rim_z)
        require_number("floor_top_z", self.floor_top_z)
        if self.inner_half_size <= 0:
            raise ValueError(f"inner_half_size は正で指定してください: inner_half_size={self.inner_half_size!r}")
        if self.rim_z <= self.floor_top_z:
            raise ValueError(
                f"rim_z は floor_top_z より大きくしてください: rim_z={self.rim_z!r}, floor_top_z={self.floor_top_z!r}"
            )

    @property
    def goal_xyz(self) -> tuple[float, float, float]:
        """箱の底の中心(置く目標位置)"""
        return (self.center_xy[0], self.center_xy[1], self.floor_top_z)


@dataclass(frozen=True)
class SceneGeometry:
    """【機能概要】: XML から導いたシーンの形状 🔵"""

    block_names: tuple[str, ...]
    block_half_size: float
    block_rest_z: float
    box_inner_half_size: float
    box_wall_thickness: float
    box_floor_top_z: float
    box_rim_z: float
    box_default_xy: tuple[float, float]
    camera_name: str
    image_width: int
    image_height: int

    @classmethod
    def from_xml(cls, path: Path = PICKPLACE_SCENE_PATH) -> SceneGeometry:
        """【機能概要】: XML を ElementTree で読んで導く(include・メッシュは読まない)"""
        return _from_xml(Path(path))

    @classmethod
    def from_model(cls, model: mujoco.MjModel) -> SceneGeometry:
        """【機能概要】: コンパイル済みモデルから、名前で body / geom を引いて導く"""
        return _from_model(model)

    def place_region(self, center_xy: tuple[float, float] | None = None) -> PlaceRegion:
        """【機能概要】: 置き先の領域。中心を省くと箱の既定の中心"""
        center = self.box_default_xy if center_xy is None else center_xy
        return PlaceRegion(
            center_xy=tuple(center),  # type: ignore[arg-type]
            inner_half_size=self.box_inner_half_size,
            rim_z=self.box_rim_z,
            floor_top_z=self.box_floor_top_z,
        )


@functools.lru_cache(maxsize=1)
def get_geometry() -> SceneGeometry:
    """【機能概要】: 既定の XML から読んだ形状(キャッシュ)"""
    return SceneGeometry.from_xml()


# --- XML 直読み ---------------------------------------------------------------
def _vec3(path: Path, elem: ET.Element, attr: str, *, required: bool) -> tuple[float, float, float]:
    label = f"<{elem.tag} name={elem.get('name')!r}> の {attr}"
    raw = elem.get(attr)
    if raw is None:
        if required:
            raise ValueError(f"{path}: {label} がありません")
        return (0.0, 0.0, 0.0)
    try:
        values = [float(t) for t in raw.split()]
    except ValueError:
        raise ValueError(f"{path}: {label} が数値ではありません: {raw!r}") from None
    if len(values) != 3 or not all(math.isfinite(v) for v in values):
        raise ValueError(f"{path}: {label} は有限の数値 3 個で指定してください: {raw!r}")
    return (values[0], values[1], values[2])


def _reject_unsupported(path: Path, elem: ET.Element) -> None:
    for attr in _UNSUPPORTED_ATTRS:
        if elem.get(attr) is not None:
            raise ValueError(f"{path}: <{elem.tag} name={elem.get('name')!r}> の {attr} 属性は未対応です")


def _find_named(path: Path, root: ET.Element, tag: str, name: str, parent: ET.Element | None = None) -> ET.Element:
    scope = root if parent is None else parent
    for elem in scope.iter(tag):
        if elem.get("name") == name:
            return elem
    raise ValueError(f"{path}: <{tag} name={name!r}> が見つかりません")


def _box_geom(path: Path, body: ET.Element, name: str) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    geom = _find_named(path, body, "geom", name)
    _reject_unsupported(path, geom)
    if geom.get("type") != "box":
        raise ValueError(f"{path}: <geom name={name!r}> は type=\"box\" である必要があります: {geom.get('type')!r}")
    return _vec3(path, geom, "pos", required=False), _vec3(path, geom, "size", required=True)


def _is_free(body: ET.Element) -> bool:
    return any(
        c.tag == "freejoint" or (c.tag == "joint" and c.get("type") == "free") for c in body
    )


def _from_xml(path: Path) -> SceneGeometry:
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError as e:
        raise ValueError(f"{path}: XML の構文エラー: {e}") from e

    box = _find_named(path, root, "body", BOX_BODY_NAME)
    _reject_unsupported(path, box)
    bx, by, bz = _vec3(path, box, "pos", required=False)
    floor_pos, floor_size = _box_geom(path, box, BOX_FLOOR_GEOM)
    wall_pos, wall_size = _box_geom(path, box, BOX_WALL_GEOM)

    if not any(c.get("name") == CAMERA_NAME for c in root.iter("camera")):
        raise ValueError(f"{path}: <camera name={CAMERA_NAME!r}> が見つかりません")

    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError(f"{path}: <worldbody> が見つかりません")
    names: list[str] = []
    sizes: list[tuple[float, float, float]] = []
    for body in worldbody.findall("body"):
        name = body.get("name") or ""
        if not name.startswith(BLOCK_BODY_PREFIX) or not _is_free(body):
            continue
        _reject_unsupported(path, body)
        geoms = body.findall("geom")
        if not geoms:
            raise ValueError(f"{path}: ブロック <body name={name!r}> に geom がありません")
        geom = geoms[0]
        gname = geom.get("name")
        _reject_unsupported(path, geom)
        if geom.get("type") != "box":
            raise ValueError(f"{path}: <geom name={gname!r}> は type=\"box\" である必要があります")
        if _vec3(path, geom, "pos", required=False) != (0.0, 0.0, 0.0):
            raise ValueError(f"{path}: <geom name={gname!r}> の pos は 0 0 0 である必要があります(静止高さの前提)")
        size = _vec3(path, geom, "size", required=True)
        if not (size[0] == size[1] == size[2]):
            raise ValueError(f"{path}: ブロック <geom name={gname!r}> が立方体ではありません: size={size!r}")
        names.append(name)
        sizes.append(size)
    if not names:
        raise ValueError(f"{path}: 自由関節を持つ {BLOCK_BODY_PREFIX}* の body が見つかりません")
    if len(set(sizes)) != 1:
        raise ValueError(f"{path}: ブロックの size がそろっていません: {dict(zip(names, sizes))!r}")
    half = sizes[0][0]

    return SceneGeometry(
        block_names=tuple(names),
        block_half_size=_r(half),
        block_rest_z=_r(half),
        box_inner_half_size=_r(wall_pos[0] - wall_size[0]),
        box_wall_thickness=_r(2 * wall_size[0]),
        box_floor_top_z=_r(bz + floor_pos[2] + floor_size[2]),
        box_rim_z=_r(bz + wall_pos[2] + wall_size[2]),
        box_default_xy=(_r(bx), _r(by)),
        camera_name=CAMERA_NAME,
        image_width=IMAGE_WIDTH,
        image_height=IMAGE_HEIGHT,
    )


# --- コンパイル済みモデル --------------------------------------------------------
def _from_model(model: mujoco.MjModel) -> SceneGeometry:
    def obj_id(kind: mujoco.mjtObj, name: str) -> int:
        i = mujoco.mj_name2id(model, kind, name)
        if i < 0:
            raise ValueError(f"モデルに {kind.name} {name!r} がありません")
        return i

    box_id = obj_id(mujoco.mjtObj.mjOBJ_BODY, BOX_BODY_NAME)
    floor_id = obj_id(mujoco.mjtObj.mjOBJ_GEOM, BOX_FLOOR_GEOM)
    wall_id = obj_id(mujoco.mjtObj.mjOBJ_GEOM, BOX_WALL_GEOM)
    obj_id(mujoco.mjtObj.mjOBJ_CAMERA, CAMERA_NAME)

    bx, by, bz = (float(v) for v in model.body_pos[box_id])
    floor_z = float(model.geom_pos[floor_id][2])
    floor_h = float(model.geom_size[floor_id][2])
    wall_x = float(model.geom_pos[wall_id][0])
    wall_z = float(model.geom_pos[wall_id][2])
    wall_sx = float(model.geom_size[wall_id][0])
    wall_hz = float(model.geom_size[wall_id][2])

    names: list[str] = []
    halves: list[float] = []
    for b in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or ""
        if not name.startswith(BLOCK_BODY_PREFIX) or model.body_jntnum[b] < 1:
            continue
        if model.jnt_type[model.body_jntadr[b]] != mujoco.mjtJoint.mjJNT_FREE:
            continue
        if model.body_geomnum[b] < 1:
            raise ValueError(f"モデルのブロック body {name!r} に geom がありません")
        size = [float(v) for v in model.geom_size[model.body_geomadr[b]]]
        if not (size[0] == size[1] == size[2]):
            raise ValueError(f"モデルのブロック body {name!r} が立方体ではありません: size={size!r}")
        names.append(name)
        halves.append(size[0])
    if not names:
        raise ValueError(f"モデルに自由関節を持つ {BLOCK_BODY_PREFIX}* の body がありません")
    if len(set(halves)) != 1:
        raise ValueError(f"モデルのブロックの size がそろっていません: {dict(zip(names, halves))!r}")

    return SceneGeometry(
        block_names=tuple(names),
        block_half_size=_r(halves[0]),
        block_rest_z=_r(halves[0]),
        box_inner_half_size=_r(wall_x - wall_sx),
        box_wall_thickness=_r(2 * wall_sx),
        box_floor_top_z=_r(bz + floor_z + floor_h),
        box_rim_z=_r(bz + wall_z + wall_hz),
        box_default_xy=(_r(bx), _r(by)),
        camera_name=CAMERA_NAME,
        image_width=IMAGE_WIDTH,
        image_height=IMAGE_HEIGHT,
    )

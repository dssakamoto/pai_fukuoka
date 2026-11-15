"""SceneGeometry / PlaceRegion のテスト (SGT-003)。

数値のリテラルは tests/test_scene.py に一本化している。ここでは「XML から導く」ことを検証する。
"""

import ast
import dataclasses
import math
import subprocess
import sys
from pathlib import Path

import mujoco
import pytest

from so101_vlm.eval import geometry as geometry_module
from so101_vlm.eval.geometry import PlaceRegion, SceneGeometry, get_geometry
from so101_vlm.eval.scene_spec import SCENE_SPEC, SceneSpec
from so101_vlm.robot.model_paths import PICKPLACE_SCENE_PATH

SCENE_SPEC_PATH = Path(geometry_module.__file__).with_name("scene_spec.py")

BOX_LINE = '<body name="box" pos="0.20 0.19 0">'
WALL_LINE = '<geom name="box_wall_px" type="box" pos="0.0425 0 0.02" size="0.0025 0.045 0.02"'
FLOOR_LINE = '<geom name="box_floor" type="box" pos="0 0 0.0025" size="0.045 0.045 0.0025"'
BLOCK_SIZE = 'size="0.015 0.015 0.015"'


@pytest.fixture(scope="module")
def xml_text():
    return PICKPLACE_SCENE_PATH.read_text(encoding="utf-8")


def _write(tmp_path, text):
    path = tmp_path / "scene.xml"
    path.write_text(text, encoding="utf-8")
    return path


def _mutated(tmp_path, xml_text, old, new, count=1):
    assert xml_text.count(old) >= 1, old
    return _write(tmp_path, xml_text.replace(old, new, count))


# --- C2: from_xml と from_model が一致 ---------------------------------------
def test_from_xml_equals_from_model():
    """TC-GEO-01 XML 直読みとコンパイル済みモデルからの値が全フィールドで一致する 🔵"""
    gx = SceneGeometry.from_xml()
    gm = SceneGeometry.from_model(mujoco.MjModel.from_xml_path(str(PICKPLACE_SCENE_PATH)))
    for f in dataclasses.fields(SceneGeometry):
        a, b = getattr(gx, f.name), getattr(gm, f.name)
        if isinstance(a, (float, tuple)) and a != () and not isinstance(a[0] if isinstance(a, tuple) else a, str):
            assert a == pytest.approx(b, abs=1e-9), f.name
        else:
            assert a == b, f.name


def test_from_model_without_camera_raises():
    """TC-GEO-02 cam_top が無いモデルは ValueError 🔵"""
    spec = mujoco.MjSpec.from_file(str(PICKPLACE_SCENE_PATH))
    cam = next(c for c in spec.cameras if c.name == "cam_top")
    spec.delete(cam)
    with pytest.raises(ValueError, match="cam_top"):
        SceneGeometry.from_model(spec.compile())


# --- C3: XML からの導出 ------------------------------------------------------
def test_box_body_pos_drives_center_and_heights(tmp_path, xml_text):
    """TC-GEO-03 box の pos を変えると中心と高さが式どおりに変わる 🔵"""
    path = _mutated(tmp_path, xml_text, BOX_LINE, '<body name="box" pos="0.31 -0.12 0.01">')
    g = SceneGeometry.from_xml(path)
    assert g.box_default_xy == (0.31, -0.12)
    assert g.box_floor_top_z == pytest.approx(0.01 + 0.0025 + 0.0025)
    assert g.box_rim_z == pytest.approx(0.01 + 0.02 + 0.02)


def test_wall_pos_and_size_drive_inner_size_and_thickness(tmp_path, xml_text):
    """TC-GEO-04 壁の pos / size を変えると内寸・壁厚が式どおりに変わる 🔵"""
    new = '<geom name="box_wall_px" type="box" pos="0.0625 0 0.03" size="0.005 0.045 0.03"'
    g = SceneGeometry.from_xml(_mutated(tmp_path, xml_text, WALL_LINE, new))
    assert g.box_inner_half_size == pytest.approx(0.0625 - 0.005)
    assert g.box_wall_thickness == pytest.approx(0.01)
    assert g.box_rim_z == pytest.approx(0.03 + 0.03)


def test_block_size_drives_half_size_and_rest_z(tmp_path, xml_text):
    """TC-GEO-05 ブロックの size を変えると half_size / rest_z が変わる 🔵"""
    path = _mutated(tmp_path, xml_text, BLOCK_SIZE, 'size="0.02 0.02 0.02"', count=3)
    g = SceneGeometry.from_xml(path)
    assert g.block_half_size == 0.02
    assert g.block_rest_z == 0.02


def test_derived_values_are_rounded(tmp_path, xml_text):
    """TC-GEO-06 導出値は丸められ、浮動小数点ノイズが残らない 🟡"""
    new = '<geom name="box_wall_px" type="box" pos="0.0425 0 0.02" size="0.0025000000001 0.045 0.02"'
    g = SceneGeometry.from_xml(_mutated(tmp_path, xml_text, WALL_LINE, new))
    assert g.box_wall_thickness == 0.005


# --- C4: エラー --------------------------------------------------------------
def _expect_error(path, *needles):
    with pytest.raises(ValueError) as exc:
        SceneGeometry.from_xml(path)
    msg = str(exc.value)
    assert str(path) in msg
    for n in needles:
        assert n in msg, (n, msg)


@pytest.mark.parametrize("name", ["box_wall_px", "box_floor"])
def test_missing_geom_raises(tmp_path, xml_text, name):
    """TC-GEO-07 必要な geom が無いと ValueError 🔵"""
    _expect_error(_write(tmp_path, xml_text.replace(f'name="{name}"', 'name="renamed"')), name)


def test_missing_box_body_raises(tmp_path, xml_text):
    """TC-GEO-08 box body が無いと ValueError 🔵"""
    _expect_error(_write(tmp_path, xml_text.replace('<body name="box"', '<body name="crate"')), "box")


def test_missing_camera_raises(tmp_path, xml_text):
    """TC-GEO-09 カメラが無いと ValueError 🔵"""
    _expect_error(_write(tmp_path, xml_text.replace('<camera name="cam_top"', '<camera name="cam_x"')), "cam_top")


def test_non_box_geom_raises(tmp_path, xml_text):
    """TC-GEO-10 geom が box 型でないと ValueError 🔵"""
    path = _mutated(tmp_path, xml_text, WALL_LINE, WALL_LINE.replace('type="box"', 'type="sphere"'))
    _expect_error(path, "box_wall_px", "box")


def test_pos_with_two_numbers_raises(tmp_path, xml_text):
    """TC-GEO-11 pos が 2 個だと ValueError 🔵"""
    path = _mutated(tmp_path, xml_text, WALL_LINE, WALL_LINE.replace('pos="0.0425 0 0.02"', 'pos="0.0425 0"'))
    _expect_error(path, "box_wall_px", "pos")


def test_size_with_nan_raises(tmp_path, xml_text):
    """TC-GEO-12 size に nan があると ValueError 🔵"""
    path = _mutated(tmp_path, xml_text, WALL_LINE, WALL_LINE.replace('size="0.0025 0.045 0.02"', 'size="nan 0.045 0.02"'))
    _expect_error(path, "box_wall_px", "size")


def test_missing_size_raises(tmp_path, xml_text):
    """TC-GEO-13 size が無いと ValueError 🔵"""
    path = _mutated(tmp_path, xml_text, WALL_LINE, WALL_LINE.replace(' size="0.0025 0.045 0.02"', ""))
    _expect_error(path, "box_wall_px", "size")


@pytest.mark.parametrize("attr", ['quat="1 0 0 0"', 'euler="0 0 1"', 'class="x"'])
def test_rotation_or_class_on_body_raises(tmp_path, xml_text, attr):
    """TC-GEO-14 body に回転・class があると ValueError 🟡"""
    path = _mutated(tmp_path, xml_text, BOX_LINE, BOX_LINE.replace('<body name="box"', f'<body name="box" {attr}'))
    _expect_error(path, "box")


def test_geom_with_quat_raises(tmp_path, xml_text):
    """TC-GEO-15 geom に quat があると ValueError 🟡"""
    path = _mutated(tmp_path, xml_text, FLOOR_LINE, FLOOR_LINE + ' quat="1 0 0 0"')
    _expect_error(path, "box_floor")


def test_no_blocks_raises(tmp_path, xml_text):
    """TC-GEO-16 ブロックが 0 個だと ValueError 🔵"""
    path = _write(tmp_path, xml_text.replace('<body name="block_', '<body name="item_'))
    _expect_error(path, "block_")


def test_non_cubic_block_raises(tmp_path, xml_text):
    """TC-GEO-17 ブロックが直方体だと ValueError 🔵"""
    path = _mutated(tmp_path, xml_text, BLOCK_SIZE, 'size="0.015 0.02 0.015"')
    _expect_error(path, "block_red_geom")


def test_blocks_with_different_sizes_raise(tmp_path, xml_text):
    """TC-GEO-18 ブロック間でサイズが違うと ValueError 🔵"""
    path = _mutated(tmp_path, xml_text, BLOCK_SIZE, 'size="0.02 0.02 0.02"')
    _expect_error(path, "block")


def test_block_geom_with_local_pos_raises(tmp_path, xml_text):
    """TC-GEO-19 ブロック geom の局所 pos が 0 でないと ValueError 🟡"""
    old = '<geom name="block_red_geom" type="box"'
    path = _mutated(tmp_path, xml_text, old, old + ' pos="0 0 0.01"')
    _expect_error(path, "block_red_geom", "pos")


def test_block_geom_without_pos_is_zero(xml_text):
    """TC-GEO-20 実 XML のブロック geom は pos 無し(0 0 0 扱い)で読める 🔵"""
    assert 'name="block_red_geom" pos' not in xml_text
    assert SceneGeometry.from_xml().block_names[0] == "block_red"


def test_syntax_error_raises_value_error(tmp_path):
    """TC-GEO-21 XML 構文エラーは ValueError(パス入り) 🔵"""
    _expect_error(_write(tmp_path, "<mujoco><worldbody></mujoco>"))


def test_missing_file_raises_file_not_found(tmp_path):
    """TC-GEO-22 存在しないパスは FileNotFoundError 🔵"""
    with pytest.raises(FileNotFoundError):
        SceneGeometry.from_xml(tmp_path / "nothing.xml")


# --- C5: PlaceRegion ---------------------------------------------------------
def _region(**kw):
    base = dict(center_xy=(0.2, 0.19), inner_half_size=0.04, rim_z=0.04, floor_top_z=0.005)
    base.update(kw)
    return PlaceRegion(**base)


def test_goal_xyz_is_center_and_floor_top():
    """TC-GEO-23 goal_xyz は (cx, cy, floor_top_z) 🔵"""
    assert _region().goal_xyz == (0.2, 0.19, 0.005)


def test_place_region_default_and_custom_center():
    """TC-GEO-24 place_region は既定で箱中心、指定で中心だけ変わる 🔵"""
    g = get_geometry()
    r0 = g.place_region()
    assert r0.center_xy == g.box_default_xy
    assert (r0.inner_half_size, r0.rim_z, r0.floor_top_z) == (g.box_inner_half_size, g.box_rim_z, g.box_floor_top_z)
    r1 = g.place_region((0.25, 0.15))
    assert r1.center_xy == (0.25, 0.15)
    assert (r1.inner_half_size, r1.rim_z, r1.floor_top_z) == (r0.inner_half_size, r0.rim_z, r0.floor_top_z)


def test_place_region_is_frozen_hashable_and_comparable():
    """TC-GEO-25 PlaceRegion は frozen・等値・ハッシュ可能 🔵"""
    r = _region()
    with pytest.raises(dataclasses.FrozenInstanceError):
        r.rim_z = 1.0  # type: ignore[misc]
    assert r == _region()
    assert hash(r) == hash(_region())
    assert r != _region(center_xy=(0.0, 0.0))


@pytest.mark.parametrize(
    "kw",
    [
        {"inner_half_size": 0.0},
        {"inner_half_size": -0.01},
        {"rim_z": 0.005},
        {"rim_z": 0.001},
        {"inner_half_size": math.nan},
        {"rim_z": math.inf},
        {"floor_top_z": math.nan},
        {"center_xy": (math.nan, 0.0)},
        {"center_xy": (0.1,)},
        {"center_xy": (0.1, 0.2, 0.3)},
    ],
)
def test_place_region_invalid_values_raise(kw):
    """TC-GEO-26 不正値は ValueError 🔵"""
    with pytest.raises(ValueError):
        _region(**kw)


def test_place_region_invalid_center_raises():
    """TC-GEO-27 place_region の中心が不正なら ValueError 🔵"""
    with pytest.raises(ValueError):
        get_geometry().place_region((0.1,))  # type: ignore[arg-type]


# --- C6: get_geometry --------------------------------------------------------
def test_get_geometry_is_cached_and_equals_from_xml():
    """TC-GEO-28 get_geometry は同じオブジェクトを返し from_xml と等しい 🔵"""
    assert get_geometry() is get_geometry()
    assert get_geometry() == SceneGeometry.from_xml()


# --- C7: scene_spec の別名化 -------------------------------------------------
DERIVED = (
    "block_names", "block_half_size", "block_rest_z", "box_center_xy", "box_inner_half_size",
    "box_wall_thickness", "box_floor_top_z", "box_rim_z", "camera_name", "image_width", "image_height",
)
FIELD_ORDER = (
    "block_names", "block_half_size", "block_rest_z", "box_body_name", "box_center_xy",
    "box_inner_half_size", "box_wall_thickness", "box_floor_top_z", "box_rim_z", "work_area_x",
    "work_area_y", "camera_name", "image_width", "image_height",
)


def test_scene_spec_matches_geometry():
    """TC-GEO-29 SCENE_SPEC の各値は get_geometry() と等しい 🔵"""
    g = get_geometry()
    for name in DERIVED:
        gname = "box_default_xy" if name == "box_center_xy" else name
        assert getattr(SCENE_SPEC, name) == getattr(g, gname), name


def test_scene_spec_field_names_unchanged():
    """TC-GEO-30 SceneSpec のフィールド名と順序は変わらない 🔵"""
    assert tuple(f.name for f in dataclasses.fields(SceneSpec)) == FIELD_ORDER


def _is_literal(node):
    if isinstance(node, ast.Constant):
        return True
    if isinstance(node, ast.UnaryOp):
        return _is_literal(node.operand)
    return isinstance(node, ast.Tuple) and all(_is_literal(e) for e in node.elts)


def test_scene_spec_derived_defaults_are_not_literals():
    """TC-GEO-31 導出対象フィールドの既定値が数値・文字列リテラルでない 🔵"""
    tree = ast.parse(SCENE_SPEC_PATH.read_text(encoding="utf-8"))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "SceneSpec")
    defaults = {n.target.id: n.value for n in cls.body if isinstance(n, ast.AnnAssign) and n.value is not None}
    for name in DERIVED:
        assert not _is_literal(defaults[name]), name


# --- C8: 循環防止 ------------------------------------------------------------
def test_geometry_import_does_not_load_scene_spec():
    """TC-GEO-32 geometry を import しても scene_spec は読み込まれない 🔵"""
    code = (
        "import sys, so101_vlm.eval.geometry;"
        "sys.exit(1 if 'so101_vlm.eval.scene_spec' in sys.modules else 0)"
    )
    assert subprocess.run([sys.executable, "-I", "-c", code]).returncode == 0

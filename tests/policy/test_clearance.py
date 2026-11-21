"""障害物 geom の自動収集 collect_obstacle_geoms のテスト (SGT-011)。"""

import dataclasses

import mujoco
import pytest
import yaml

from so101_vlm.policy.clearance import _robot_bodies, collect_obstacle_geoms
from so101_vlm.policy.ik_policy import IkPolicy
from so101_vlm.policy.trajectory_config import ClearanceConfig, load_trajectory_config
from so101_vlm.robot.model_paths import PICKPLACE_SCENE_PATH, REPO_ROOT

CURRENT = ("floor", "box_floor", "box_wall_px", "box_wall_nx", "box_wall_py", "box_wall_ny")

# ロボット (base と子孫)・床・箱・ブロックだけの小さなシーン。ロボットの geom は group 3 (視覚用) も持つ
TINY = """
<mujoco>
  <worldbody>
    <geom name="floor" type="plane" size="1 1 0.05"/>
    <body name="base">
      <geom name="base_geom" type="box" size="0.02 0.02 0.02"/>
      <geom name="base_visual" type="box" size="0.02 0.02 0.02" group="3" contype="0" conaffinity="0"/>
      <body name="arm" pos="0 0 0.1">
        <joint name="hinge" type="hinge" axis="0 0 1"/>
        <geom name="arm_geom" type="capsule" size="0.01 0.05"/>
      </body>
    </body>
    <body name="box" pos="0.2 0.19 0">
      <geom name="box_floor" type="box" size="0.045 0.045 0.0025"/>
      <geom name="box_wall_px" type="box" size="0.005 0.045 0.025" pos="0.04 0 0.025"/>
      %EXTRA%
    </body>
    <body name="block_red" pos="0.25 0 0.015"><freejoint name="block_red_joint"/>
      <geom name="block_red_geom" type="box" size="0.015 0.015 0.015"/></body>
  </worldbody>
</mujoco>
"""


def _tiny(extra: str = "") -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_string(TINY.replace("%EXTRA%", extra))


@pytest.fixture(scope="module")
def scene():
    return mujoco.MjModel.from_xml_path(str(PICKPLACE_SCENE_PATH))


# =============================================================== 実際のシーン
def test_collected_set_equals_the_hand_written_list(scene):
    """現行の手書きの 6 件と一致する (順序は問わない) 🔵"""
    assert set(collect_obstacle_geoms(scene)) == set(CURRENT) and len(collect_obstacle_geoms(scene)) == 6
    assert set(CURRENT) == set(load_trajectory_config().clearance.obstacle_geoms)  # motion.yaml の値とも同じ


def test_robot_geoms_and_blocks_are_not_obstacles(scene):
    got = set(collect_obstacle_geoms(scene))
    robot = _robot_bodies(scene)
    robot_geoms = {
        mujoco.mj_id2name(scene, mujoco.mjtObj.mjOBJ_GEOM, g)
        for g in range(scene.ngeom) if int(scene.geom_bodyid[g]) in robot
    }  # fmt: skip
    assert not (got & {n for n in robot_geoms if n})
    for g in range(scene.ngeom):  # 自由度を持つ body (ブロック) の geom
        if scene.body_dofnum[int(scene.geom_bodyid[g])] > 0:
            assert mujoco.mj_id2name(scene, mujoco.mjtObj.mjOBJ_GEOM, g) not in got


def test_moving_the_box_does_not_change_the_names(scene):
    model = mujoco.MjModel.from_xml_path(str(PICKPLACE_SCENE_PATH))  # 共有のモデルを汚さない
    before = collect_obstacle_geoms(model)
    box = model.body("box").id
    model.body_pos[box][:2] = (0.25, 0.15)
    assert collect_obstacle_geoms(model) == before


# =============================================================== 合成モデル
def test_tiny_scene_collects_floor_and_box_only():
    assert set(collect_obstacle_geoms(_tiny())) == {"floor", "box_floor", "box_wall_px"}


def test_an_added_box_geom_is_included():
    extra = '<geom name="box_wall_extra" type="box" size="0.005 0.045 0.025" pos="-0.04 0 0.025"/>'
    assert "box_wall_extra" in collect_obstacle_geoms(_tiny(extra))


def test_visual_only_geoms_are_excluded():
    extra = '<geom name="box_label" type="box" size="0.01 0.01 0.01" contype="0" conaffinity="0"/>'
    assert "box_label" not in collect_obstacle_geoms(_tiny(extra))


def test_an_unnamed_static_collision_geom_raises_value_error():
    with pytest.raises(ValueError, match="名前"):
        collect_obstacle_geoms(_tiny('<geom type="box" size="0.01 0.01 0.01"/>'))


# =============================================================== 設定と IkPolicy
def _cfg_with(path, value):
    raw = yaml.safe_load((REPO_ROOT / "configs" / "motion.yaml").read_text(encoding="utf-8"))
    raw["trajectory"]["clearance"]["obstacle_geoms"] = value
    path.write_text(yaml.safe_dump(raw, allow_unicode=True), encoding="utf-8")
    return path


def test_auto_in_yaml_means_collect_from_the_model(tmp_path):
    cfg = load_trajectory_config(_cfg_with(tmp_path / "m.yaml", "auto"))
    assert cfg.clearance.obstacle_geoms is None


def test_explicit_list_in_yaml_is_used_as_is(tmp_path):
    cfg = load_trajectory_config(_cfg_with(tmp_path / "m.yaml", ["floor"]))
    assert cfg.clearance.obstacle_geoms == ("floor",)


@pytest.mark.parametrize("bad", ["floor", "AUTO", ""])
def test_a_bare_string_other_than_auto_is_still_rejected(tmp_path, bad):
    with pytest.raises(ValueError, match="obstacle_geoms"):
        load_trajectory_config(_cfg_with(tmp_path / "m.yaml", bad))


def test_ik_policy_with_auto_uses_the_same_obstacles_as_the_explicit_list(scene):
    base = load_trajectory_config()
    explicit = IkPolicy(scene, config=base)
    auto = IkPolicy(scene, config=dataclasses.replace(base, clearance=dataclasses.replace(base.clearance, obstacle_geoms=None)))
    assert sorted(auto._checker._obstacles) == sorted(explicit._checker._obstacles)


def test_clearance_config_accepts_none_but_not_an_empty_list():
    assert ClearanceConfig(obstacle_m=0.001, self_m=0.002, obstacle_geoms=None).obstacle_geoms is None
    with pytest.raises(ValueError, match="obstacle_geoms"):
        ClearanceConfig(obstacle_m=0.001, self_m=0.002, obstacle_geoms=[])

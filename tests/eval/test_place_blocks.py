"""place_blocks (MjData へのブロック配置適用) のテスト (TASK-0006, Red フェーズ)。

テストケース定義: docs/implements/so101-vlm-pickplace/TASK-0006/layout-testcases.md
信頼性: 🔵 要件・設計・実測に基づく / 🟡 妥当な推測 / 🔴 要件にない仮定
"""

import copy
import itertools
import math

import mujoco
import numpy as np
import pytest

from so101_vlm.eval.layout import place_blocks, sample_layout
from so101_vlm.eval.scene_spec import SCENE_SPEC
from so101_vlm.robot.model_paths import JOINT_NAMES, PICKPLACE_SCENE_PATH

TRIAL_SEEDS = range(20)
SETTLE_STEPS = 500  # 1 s (timestep 2 ms)
REST_TOL = 0.002
SPEED_TOL = 0.01
MOVE_TOL = 0.005

TINY_XML = """
<mujoco>
  <worldbody>
    <geom name="floor" type="plane" size="1 1 0.05"/>
    <body name="arm" pos="0 0 0.3">
      <joint name="arm_hinge" type="hinge" axis="0 0 1"/>
      <geom type="capsule" size="0.01 0.05"/>
    </body>
    <body name="block_red" pos="0.20 -0.06 0.015"><freejoint name="block_red_joint"/>
      <geom type="box" size="0.015 0.015 0.015" mass="0.02"/></body>
    <body name="block_blue" pos="0.30 0.00 0.015"><freejoint name="block_blue_joint"/>
      <geom type="box" size="0.015 0.015 0.015" mass="0.02"/></body>
    <body name="block_green" pos="0.22 0.07 0.015"><freejoint name="block_green_joint"/>
      <geom type="box" size="0.015 0.015 0.015" mass="0.02"/></body>
    <body name="box" pos="0.20 0.19 0"><geom type="box" size="0.045 0.045 0.0025"/></body>
  </worldbody>
</mujoco>
"""
LAYOUT = ((0.16, -0.09), (0.34, 0.01), (0.25, 0.09))
BLOCKS = SCENE_SPEC.block_names


# --- ヘルパ ---
def _qpos_adr(model, body):
    return int(model.jnt_qposadr[model.body(body).jntadr[0]])


def _dof_adr(model, body):
    return int(model.jnt_dofadr[model.body(body).jntadr[0]])


def _snapshot(data):
    return data.qpos.copy(), data.qvel.copy(), data.xpos.copy()


def _assert_unchanged(data, snap):
    qpos0, qvel0, xpos0 = snap
    assert np.array_equal(data.qpos, qpos0)  # 【確認内容】: qpos 不変
    assert np.array_equal(data.qvel, qvel0)  # 【確認内容】: qvel 不変
    assert np.array_equal(data.xpos, xpos0)  # 【確認内容】: xpos 不変


# --- フィクスチャ ---
@pytest.fixture(scope="module")
def tiny_model():
    return mujoco.MjModel.from_xml_string(TINY_XML)


@pytest.fixture
def tiny_data(tiny_model):
    # 【テスト前準備】: hinge は qpos=0.7/qvel=0.3、ブロックの qvel は 0.1 にして forward 済みにする
    data = mujoco.MjData(tiny_model)
    data.qpos[0] = 0.7
    data.qvel[0] = 0.3
    for name in BLOCKS:
        dof = _dof_adr(tiny_model, name)
        data.qvel[dof : dof + 6] = 0.1
    mujoco.mj_forward(tiny_model, data)
    return data


@pytest.fixture(scope="module")
def scene_model():
    return mujoco.MjModel.from_xml_path(str(PICKPLACE_SCENE_PATH))


@pytest.fixture(scope="module")
def settled_runs(scene_model):
    # 【テスト前準備】: 20 シード分、配置 → 500 step を 1 回だけ実行して結果を共有する
    runs = {}
    for seed in TRIAL_SEEDS:
        data = mujoco.MjData(scene_model)
        layout = sample_layout(seed)
        place_blocks(scene_model, data, layout)
        xy0 = {n: data.xpos[scene_model.body(n).id][:2].copy() for n in BLOCKS}
        for _ in range(SETTLE_STEPS):
            mujoco.mj_step(scene_model, data)
        runs[seed] = (layout, xy0, data)
    return runs


# =============================================================== 正常系 (単体)
def test_place_blocks_sets_qpos_of_each_block(tiny_model, tiny_data):
    """TC-P-01 各ブロックの qpos 7 要素 = (x, y, rest_z, 1, 0, 0, 0) 🔵"""
    place_blocks(tiny_model, tiny_data, LAYOUT)
    for name, (x, y) in zip(BLOCKS, LAYOUT, strict=True):
        adr = _qpos_adr(tiny_model, name)
        expected = [x, y, SCENE_SPEC.block_rest_z, 1, 0, 0, 0]
        assert tiny_data.qpos[adr : adr + 7] == pytest.approx(expected), name  # 【確認内容】: 位置+姿勢


def test_place_blocks_zeroes_block_velocity(tiny_model, tiny_data):
    """TC-P-02 各ブロックの qvel 6 要素が 0 🟡"""
    place_blocks(tiny_model, tiny_data, LAYOUT)
    for name in BLOCKS:
        dof = _dof_adr(tiny_model, name)
        assert np.all(tiny_data.qvel[dof : dof + 6] == 0), name  # 【確認内容】: 速度 0


def test_place_blocks_keeps_non_block_state_and_time(tiny_model, tiny_data):
    """TC-P-03 ブロック以外の状態は変わらず、時刻も進めない 🟡"""
    time0 = tiny_data.time
    place_blocks(tiny_model, tiny_data, LAYOUT)
    assert tiny_data.qpos[0] == pytest.approx(0.7)  # 【確認内容】: hinge qpos
    assert tiny_data.qvel[0] == pytest.approx(0.3)  # 【確認内容】: hinge qvel
    assert tiny_data.time == time0  # 【確認内容】: mj_step していない


def test_place_blocks_calls_forward_so_xpos_reflects_layout(tiny_model, tiny_data):
    """TC-P-04 mj_forward が呼ばれ xpos / xquat が配置を反映 🟡"""
    place_blocks(tiny_model, tiny_data, LAYOUT)
    for name, (x, y) in zip(BLOCKS, LAYOUT, strict=True):
        bid = tiny_model.body(name).id
        assert tiny_data.xpos[bid][:2] == pytest.approx([x, y]), name
        assert tiny_data.xpos[bid][2] == pytest.approx(SCENE_SPEC.block_rest_z)
        assert tiny_data.xquat[bid] == pytest.approx([1, 0, 0, 0])


def test_place_blocks_returns_none_and_keeps_layout(tiny_model, tiny_data):
    """TC-P-05 戻り値は None、layout は変更しない 🟡"""
    layout = [[0.16, -0.09], [0.34, 0.01], [0.25, 0.09]]
    before = copy.deepcopy(layout)
    result = place_blocks(tiny_model, tiny_data, layout)
    assert result is None  # 【確認内容】: 戻り値なし
    assert layout == before  # 【確認内容】: 入力不変


def test_place_blocks_with_block_names_only_touches_named(tiny_model, tiny_data):
    """TC-P-06 block_names 指定時はその body だけを書き換える 🟡"""
    blue_adr = _qpos_adr(tiny_model, "block_blue")
    blue_dof = _dof_adr(tiny_model, "block_blue")
    qpos_blue0 = tiny_data.qpos[blue_adr : blue_adr + 7].copy()
    qvel_blue0 = tiny_data.qvel[blue_dof : blue_dof + 6].copy()
    place_blocks(
        tiny_model,
        tiny_data,
        ((0.18, 0.0), (0.30, -0.05)),
        block_names=("block_green", "block_red"),
    )
    g = _qpos_adr(tiny_model, "block_green")
    r = _qpos_adr(tiny_model, "block_red")
    assert tiny_data.qpos[g : g + 2] == pytest.approx([0.18, 0.0])  # 【確認内容】: green
    assert tiny_data.qpos[r : r + 2] == pytest.approx([0.30, -0.05])  # 【確認内容】: red
    assert np.array_equal(tiny_data.qpos[blue_adr : blue_adr + 7], qpos_blue0)  # blue 不変
    assert np.array_equal(tiny_data.qvel[blue_dof : blue_dof + 6], qvel_blue0)


# =============================================================== 異常系 (単体)
@pytest.mark.parametrize("layout", [LAYOUT[:2], (*LAYOUT, (0.2, 0.0))], ids=["short", "long"])
def test_place_blocks_rejects_length_mismatch(tiny_model, tiny_data, layout):
    """TC-P-E01 layout の長さがブロック数と違う → ValueError (両方の長さ入り) 🟡"""
    snap = _snapshot(tiny_data)
    with pytest.raises(ValueError) as excinfo:
        place_blocks(tiny_model, tiny_data, layout)
    msg = str(excinfo.value)
    assert str(len(layout)) in msg and "3" in msg  # 【確認内容】: 長さがメッセージにある
    _assert_unchanged(tiny_data, snap)


def test_place_blocks_rejects_unknown_body_without_writing(tiny_model, tiny_data):
    """TC-P-E02 存在しない body 名 → ValueError (名前入り)、途中まで書かない 🟡"""
    snap = _snapshot(tiny_data)
    with pytest.raises(ValueError, match="block_purple"):
        place_blocks(
            tiny_model, tiny_data, LAYOUT, block_names=("block_red", "block_purple", "block_green")
        )
    _assert_unchanged(tiny_data, snap)  # 【確認内容】: 先頭の block_red も未変更


def test_place_blocks_rejects_body_without_joint(tiny_model, tiny_data):
    """TC-P-E03 関節を持たない body (box) → ValueError 🔴"""
    snap = _snapshot(tiny_data)
    with pytest.raises(ValueError, match="box"):
        place_blocks(tiny_model, tiny_data, LAYOUT, block_names=("block_red", "block_blue", "box"))
    _assert_unchanged(tiny_data, snap)


def test_place_blocks_rejects_non_free_joint(tiny_model, tiny_data):
    """TC-P-E04 freejoint でない関節の body (hinge の arm) → ValueError 🔴"""
    snap = _snapshot(tiny_data)
    with pytest.raises(ValueError, match="arm"):
        place_blocks(
            tiny_model, tiny_data, LAYOUT, block_names=("arm", "block_blue", "block_green")
        )
    _assert_unchanged(tiny_data, snap)


# =============================================================== 統合 (実シーン)
def test_place_blocks_on_real_scene_keeps_robot_joints(scene_model):
    """TC-I-01 実シーンでもロボット関節は変わらず、ブロックだけが配置される 🟡"""
    data = mujoco.MjData(scene_model)
    mujoco.mj_forward(scene_model, data)
    robot_adr = [int(scene_model.joint(n).qposadr[0]) for n in JOINT_NAMES]
    robot_before = data.qpos[robot_adr].copy()
    layout = sample_layout(0)
    place_blocks(scene_model, data, layout)
    assert np.array_equal(data.qpos[robot_adr], robot_before)  # 【確認内容】: ロボット不変
    for name, (x, y) in zip(BLOCKS, layout, strict=True):
        pos = data.xpos[scene_model.body(name).id]
        assert pos[:2] == pytest.approx([x, y])  # 【確認内容】: ブロック位置


@pytest.mark.parametrize("seed", TRIAL_SEEDS)
def test_placed_blocks_settle_within_one_second(scene_model, settled_runs, seed):
    """TC-I-02 配置後 1 秒で静止する (速度 < 1 cm/s、高さ = rest_z) 🟡"""
    _, _, data = settled_runs[seed]
    for name in BLOCKS:
        dof = _dof_adr(scene_model, name)
        assert np.linalg.norm(data.qvel[dof : dof + 3]) < SPEED_TOL, f"seed={seed} {name}"
        z = data.xpos[scene_model.body(name).id][2]
        assert abs(z - SCENE_SPEC.block_rest_z) <= REST_TOL, f"seed={seed} {name} z={z}"


@pytest.mark.parametrize("seed", TRIAL_SEEDS)
def test_placed_blocks_do_not_overlap_or_drift(scene_model, settled_runs, seed):
    """TC-I-03 重ならず (中心間 >= 3 cm)、ほとんど動かない (< 5 mm) 🟡"""
    _, xy0, data = settled_runs[seed]
    xy = {n: data.xpos[scene_model.body(n).id][:2] for n in BLOCKS}
    for a, b in itertools.combinations(BLOCKS, 2):
        assert math.dist(xy[a], xy[b]) >= 2 * SCENE_SPEC.block_half_size, f"seed={seed} {a} {b}"
    for n in BLOCKS:
        assert np.linalg.norm(xy[n] - xy0[n]) < MOVE_TOL, f"seed={seed} {n}"


@pytest.mark.parametrize("seed", TRIAL_SEEDS)
def test_placed_blocks_clear_of_box_and_touch_only_floor(scene_model, settled_runs, seed):
    """TC-I-04 箱と交差せず、ブロックは床とだけ接触 (home 姿勢に限る) 🟡"""
    layout, _, data = settled_runs[seed]
    h = SCENE_SPEC.block_half_size
    bx, by = SCENE_SPEC.box_center_xy
    outer = SCENE_SPEC.box_inner_half_size + SCENE_SPEC.box_wall_thickness
    for x, y in layout:  # 幾何: xy 外形が箱の外形と重ならない
        overlap_x = (x - h) < (bx + outer) and (x + h) > (bx - outer)
        overlap_y = (y - h) < (by + outer) and (y + h) > (by - outer)
        assert not (overlap_x and overlap_y), f"seed={seed} ({x}, {y})"
    block_geoms = {scene_model.body(n).geomadr[0] for n in BLOCKS}
    floor = mujoco.mj_name2id(scene_model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    for c in data.contact[: data.ncon]:
        g1, g2 = int(c.geom1), int(c.geom2)
        if g1 in block_geoms or g2 in block_geoms:
            other = g2 if g1 in block_geoms else g1
            assert other == floor, f"seed={seed} geom {g1}-{g2}"  # 【確認内容】: 相手は床のみ

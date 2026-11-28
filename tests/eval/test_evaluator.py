"""Evaluator Protocol と SimEvaluator のテスト (SGT-008)。"""

import mujoco
import pytest

from so101_vlm.eval.eval_config import load_eval_config
from so101_vlm.eval.evaluator import Evaluator, SimEvaluator
from so101_vlm.eval.geometry import get_geometry
from so101_vlm.eval.judge import JudgeReason, JudgeResult
from so101_vlm.eval.layout import _resolve_free_joint
from so101_vlm.robot.model_paths import PICKPLACE_SCENE_PATH

G = get_geometry()
JUDGE = load_eval_config().judge


def _sim():
    model = mujoco.MjModel.from_xml_path(str(PICKPLACE_SCENE_PATH))
    return model, mujoco.MjData(model)


def _put(model, data, body, xyz):
    qadr, dadr = _resolve_free_joint(model, body)
    data.qpos[qadr : qadr + 3] = xyz
    data.qvel[dadr : dadr + 6] = 0.0
    mujoco.mj_forward(model, data)


def _in_box_z():
    return G.box_floor_top_z + G.block_half_size


def test_success_inside_the_box_and_outside_inner_when_outside():
    model, data = _sim()
    region = G.place_region()
    ev = SimEvaluator(model, data, JUDGE, region)
    cx, cy = region.center_xy
    _put(model, data, G.block_names[0], (cx, cy, _in_box_z()))
    assert ev.evaluate("red").success is True
    _put(model, data, G.block_names[0], (cx + 0.15, cy, G.block_rest_z))
    r = ev.evaluate("red")
    assert r.success is False and JudgeReason.OUTSIDE_INNER in r.reasons


def test_colours_map_to_their_own_blocks():
    model, data = _sim()
    region = G.place_region()
    ev = SimEvaluator(model, data, JUDGE, region)
    cx, cy = region.center_xy
    _put(model, data, G.block_names[1], (cx, cy, _in_box_z()))  # blue だけ入れる
    assert ev.evaluate("blue").success is True
    assert ev.evaluate("red").success is False


def test_follows_a_moved_region():
    model, data = _sim()
    ev = SimEvaluator(model, data, JUDGE, G.place_region((0.25, 0.15)))
    _put(model, data, G.block_names[0], (0.25, 0.15, _in_box_z()))
    assert ev.evaluate("red").success is True
    assert SimEvaluator(model, data, JUDGE, G.place_region()).evaluate("red").success is False  # 元の箱の領域では外


def test_invalid_arguments_raise_value_error():
    model, data = _sim()
    with pytest.raises(ValueError):
        SimEvaluator(model, data, "judge", G.place_region())  # type: ignore[arg-type]
    with pytest.raises(ValueError):
        SimEvaluator(model, data, JUDGE, "region")  # type: ignore[arg-type]
    with pytest.raises(ValueError):
        SimEvaluator(model, data, JUDGE, G.place_region()).evaluate("purple")


def test_any_stub_satisfies_the_protocol():
    """型検査相当: Evaluator を受ける関数に、スタブも sim 実装も同じ呼び方で渡せる 🟡"""

    class Stub:
        def evaluate(self, colour: str) -> JudgeResult:
            return JudgeResult(True, (), (0.0, 0.0), 0.02, 0.0)

    def use(ev: Evaluator) -> bool:
        return ev.evaluate("red").success

    assert use(Stub()) is True
    assert use(SimEvaluator(*_sim(), JUDGE, G.place_region())) in (True, False)

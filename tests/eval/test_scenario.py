"""Scenario と sample_scenario のテスト (SGT-005)。"""

import dataclasses

import numpy as np
import pytest

from so101_vlm.eval.geometry import PlaceRegion, get_geometry
from so101_vlm.eval.layout import sample_layout
from so101_vlm.eval.reachable import REACHABLE_PATH, ReachableRegion
from so101_vlm.eval.scenario import Fixed, Scenario, ScenarioSpec, UniformXY, sample_scenario


@pytest.mark.parametrize("seed", range(100))
def test_blocks_equal_sample_layout_for_the_default_spec(seed):
    """SGT-005 乱数の消費順序が変わっていない (既存の seed と同じ配置) 🔵"""
    assert sample_scenario(seed).blocks == sample_layout(seed)


def test_fixed_default_is_the_xml_box_position():
    g = get_geometry()
    assert sample_scenario(0).box_xy == g.box_default_xy == (0.20, 0.19)


def test_fixed_with_xy_is_used_as_is_and_does_not_change_the_blocks():
    s = sample_scenario(3, ScenarioSpec(box=Fixed(xy=(0.25, 0.15))))
    assert s.box_xy == (0.25, 0.15)
    assert s.blocks == sample_scenario(3).blocks  # Fixed は乱数を消費しない


def test_same_seed_gives_equal_scenarios_and_they_are_frozen():
    a, b = sample_scenario(7), sample_scenario(7)
    assert a == b and hash(a) == hash(b)
    with pytest.raises(dataclasses.FrozenInstanceError):
        a.seed = 1  # type: ignore[misc]
    assert all(isinstance(p, tuple) for p in a.blocks) and isinstance(a.box_xy, tuple)  # ndarray を持たない


@pytest.mark.parametrize("seed", [-1, True, 1.5, "0", None])
def test_invalid_seed_raises_value_error(seed):
    with pytest.raises(ValueError):
        sample_scenario(seed)


def test_invalid_spec_raises_value_error():
    with pytest.raises(ValueError):
        sample_scenario(0, ScenarioSpec(n_blocks=0))
    with pytest.raises(ValueError):
        sample_scenario(0, "spec")  # type: ignore[arg-type]
    with pytest.raises(ValueError):
        Fixed(xy=(0.1,))  # type: ignore[arg-type]
    with pytest.raises(ValueError):
        Fixed(xy=(float("nan"), 0.0))


def test_place_region_uses_the_box_centre_and_geometry():
    g = get_geometry()
    r = Scenario(seed=0, blocks=((0.2, 0.0),), box_xy=(0.25, 0.15)).place_region(g)
    assert isinstance(r, PlaceRegion) and r.center_xy == (0.25, 0.15)
    assert r.inner_half_size == g.box_inner_half_size and r.rim_z == g.box_rim_z and r.floor_top_z == g.box_floor_top_z


def test_sampling_does_not_use_the_global_rng():
    np.random.seed(123)
    a = sample_scenario(4)
    np.random.seed(999)
    assert sample_scenario(4) == a


# =============================================================== UniformXY (SGT-010)
G = get_geometry()
OUTER = G.box_inner_half_size + G.box_wall_thickness  # 箱の外周の半幅
STEP = 0.01


def _grid(x_range, y_range, keep=lambda x, y, i, j: True) -> ReachableRegion:
    nx = round((x_range[1] - x_range[0]) / STEP) + 1
    ny = round((y_range[1] - y_range[0]) / STEP) + 1
    mask = tuple(
        tuple(bool(keep(x_range[0] + i * STEP, y_range[0] + j * STEP, i, j)) for j in range(ny)) for i in range(nx)
    )
    return ReachableRegion(x_range[0], y_range[0], STEP, mask)


FULL = _grid((0.05, 0.50), (-0.35, 0.35))  # ブロックの作業範囲 (x 0.15〜0.35, y −0.10〜0.10) を覆う広い領域
UNIFORM = ScenarioSpec(box=UniformXY())


def _outline_inside(region, cx, cy):
    n = 40
    pts = [(cx + OUTER * t / n, cy + sy * OUTER) for t in range(-n, n + 1) for sy in (-1, 1)]
    pts += [(cx + sx * OUTER, cy + OUTER * t / n) for t in range(-n, n + 1) for sx in (-1, 1)]
    return all(region.contains_xy(x, y) for x, y in pts)


@pytest.mark.parametrize("seed", range(100))
def test_uniform_blocks_equal_fixed_blocks(seed):
    """G1 (a) / 乱数は箱を最後に引く: UniformXY のブロック配置は Fixed と完全に同じ 🔵"""
    assert sample_scenario(seed, UNIFORM, reachable=FULL).blocks == sample_scenario(seed).blocks


@pytest.mark.parametrize("seed", range(30))
def test_uniform_box_outline_is_inside_the_region_and_clear_of_the_blocks(seed):
    region = _grid((0.10, 0.45), (-0.15, 0.30))
    s = sample_scenario(seed, UNIFORM, reachable=region)
    cx, cy = s.box_xy
    assert _outline_inside(region, cx, cy)
    spec = ScenarioSpec()
    for bx, by in s.blocks:
        assert max(abs(cx - bx), abs(cy - by)) >= OUTER + G.block_half_size + spec.min_gap


def test_uniform_is_deterministic_and_varies_with_the_seed():
    a = sample_scenario(5, UNIFORM, reachable=FULL)
    assert a == sample_scenario(5, UNIFORM, reachable=FULL)
    assert len({sample_scenario(k, UNIFORM, reachable=FULL).box_xy for k in range(10)}) == 10


def test_explicit_region_wins_over_the_keyword():
    unusable = _grid((0.20, 0.25), (-0.10, 0.10))  # ブロックが外れて使えない領域
    spec = ScenarioSpec(box=UniformXY(region=FULL))
    assert sample_scenario(1, spec, reachable=unusable) == sample_scenario(1, UNIFORM, reachable=FULL)


def test_margin_keeps_the_box_further_from_the_edge():
    region = _grid((0.10, 0.45), (-0.15, 0.30))
    (xl, xh), (yl, yh) = region.bbox()
    for seed in range(20):
        cx, cy = sample_scenario(seed, ScenarioSpec(box=UniformXY(margin=0.03)), reachable=region).box_xy
        assert xl + OUTER + 0.03 <= cx <= xh - OUTER - 0.03 and yl + OUTER + 0.03 <= cy <= yh - OUTER - 0.03


def test_region_too_small_for_the_box_raises_value_error():
    spec = ScenarioSpec(n_blocks=1, area=0.05, box=UniformXY())  # ブロックは中央の 5 cm 角 (x 0.225〜0.275, y −0.025〜0.025)
    narrow = _grid((0.20, 0.28), (-0.05, 0.05))  # ブロックは入るが、幅 0.08 m は箱の外周の幅 (2 x 0.045) より狭い
    with pytest.raises(ValueError, match="狭すぎ"):
        sample_scenario(0, spec, reachable=narrow)


def test_attempts_exhausted_raises_value_error_with_the_count():
    # ブロックの作業範囲は全部 True、その外は市松 (箱の外周が必ず False の点を踏む)
    region = _grid(
        (0.05, 0.50), (-0.35, 0.35),
        keep=lambda x, y, i, j: (0.14 <= x <= 0.36 and -0.11 <= y <= 0.11) or (i + j) % 2 == 0,
    )  # fmt: skip
    with pytest.raises(ValueError, match="200 回"):
        sample_scenario(0, ScenarioSpec(max_attempts=200, box=UniformXY()), reachable=region)


def test_blocks_outside_the_region_raise_value_error():
    region = _grid((0.05, 0.50), (-0.35, 0.35), keep=lambda x, y, i, j: x < 0.30)  # ブロックの作業範囲の半分が外
    with pytest.raises(ValueError, match="ブロック"):
        sample_scenario(0, UNIFORM, reachable=region)


@pytest.mark.parametrize("kwargs", [{"margin": -0.01}, {"margin": float("nan")}, {"margin": True}, {"region": "r"}])
def test_invalid_uniform_xy_raises_value_error(kwargs):
    with pytest.raises(ValueError):
        UniformXY(**kwargs)


def test_fixed_ignores_the_reachable_argument():
    assert sample_scenario(3, reachable=FULL) == sample_scenario(3)


# --- 保存済みの configs/reachable.yaml (SGT-009 で生成してコミットされていれば走る) ---------------------------------
needs_saved = pytest.mark.skipif(
    not REACHABLE_PATH.exists(),
    reason="configs/reachable.yaml がありません (scripts/compute_reachable.py で生成してコミットしてください)",
)


@needs_saved
@pytest.mark.parametrize("seed", range(100))
def test_saved_region_covers_the_blocks_so_uniform_equals_fixed(seed):
    """G1 (a) の保証: 既定の領域がブロック領域を覆う (違反する seed が出たら G1 を (b) に落とす) 🔵"""
    assert sample_scenario(seed, UNIFORM).blocks == sample_scenario(seed).blocks


@needs_saved
def test_uniform_scenarios_do_not_fail_in_ik_planning():
    """箱を乱数で動かした 20 件が、IK の計画失敗にならない (到達可能領域が実際の IK と整合している) 🟡"""
    from so101_vlm.eval.failure import FailureKind
    from so101_vlm.eval.fixed_pick import run_fixed_pick

    for seed in range(20):
        out = run_fixed_pick(seed, "red", scenario=sample_scenario(seed, UNIFORM))
        assert out.failure != FailureKind.PLANNING_FAILURE, f"seed {seed}: {out.planning_error} ({out.planning_waypoint})"

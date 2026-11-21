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
    for bx, by in s.blocks:
        assert max(abs(cx - bx), abs(cy - by)) >= OUTER + G.block_half_size + UniformXY().block_gap


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
    # 市松模様の領域: 箱の外周のどこかが必ず False の点を踏む (領域に外周が収まる位置が無い)
    region = _grid((0.05, 0.50), (-0.35, 0.35), keep=lambda x, y, i, j: (i + j) % 2 == 0)
    with pytest.raises(ValueError, match="200 回"):
        sample_scenario(0, ScenarioSpec(max_attempts=200, box=UniformXY()), reachable=region)


def test_blocks_are_not_rejected_by_the_region():
    """G1 (b): 領域は箱だけを棄却する。ブロックが領域の外にあっても、ブロックの列は Fixed と同じまま 🔵"""
    region = _grid((0.05, 0.50), (-0.35, 0.35), keep=lambda x, y, i, j: x >= 0.17)  # 付け根の近く (x < 0.17) は領域の外
    s = sample_scenario(0, UNIFORM, reachable=region)
    assert s.blocks == sample_scenario(0).blocks
    assert region.contains_xy(*s.box_xy)


def test_default_block_gap_is_what_the_default_box_already_has():
    """既定の箱 (0.20, 0.19) と、作業範囲の端 (y 0.10) のブロックの縁との間隔 = block_gap の既定。それより厳しくすると既定の配置が違反する 🔵"""
    default_gap = G.box_default_xy[1] - OUTER - (0.10 + G.block_half_size)
    assert UniformXY().block_gap == pytest.approx(default_gap)


def test_block_gap_widens_the_clearance():
    region = _grid((0.10, 0.45), (-0.15, 0.30))
    for seed in range(10):
        s = sample_scenario(seed, ScenarioSpec(box=UniformXY(block_gap=0.06)), reachable=region)
        cx, cy = s.box_xy
        assert all(max(abs(cx - bx), abs(cy - by)) >= OUTER + G.block_half_size + 0.06 for bx, by in s.blocks)


@pytest.mark.parametrize("kwargs", [{"block_gap": -0.01}, {"block_gap": float("nan")}, {"margin": -0.01}, {"margin": float("nan")}, {"margin": True}, {"region": "r"}])
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
def test_saved_region_uniform_keeps_the_fixed_blocks_and_places_the_box_inside(seed):
    """保存済みの領域でも、UniformXY のブロックは Fixed と同じ。箱の外周は領域の内側 🔵"""
    from so101_vlm.eval.reachable import load_reachable_region

    s = sample_scenario(seed, UNIFORM)
    assert s.blocks == sample_scenario(seed).blocks
    assert _outline_inside(load_reachable_region(), *s.box_xy)


_BOX_SIDE_WAYPOINTS = ("pre_place", "place", "retreat")


@needs_saved
def test_uniform_scenarios_have_no_box_side_planning_failure():
    """箱を乱数で動かした 20 件で、箱の側の経由点 (pre_place / place / retreat) の計画が失敗しない 🟡

    掴む側 (ブロックが付け根に近い等) の計画失敗は、箱の位置と無関係なので数えない (従来から起こりうる)。
    """
    from so101_vlm.eval.failure import FailureKind
    from so101_vlm.eval.fixed_pick import run_fixed_pick

    for seed in range(20):
        out = run_fixed_pick(seed, "red", scenario=sample_scenario(seed, UNIFORM))
        if out.failure == FailureKind.PLANNING_FAILURE:
            assert out.planning_waypoint not in _BOX_SIDE_WAYPOINTS, f"seed {seed}: {out.planning_error} ({out.planning_waypoint})"

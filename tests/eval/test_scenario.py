"""Scenario と sample_scenario のテスト (SGT-005)。"""

import dataclasses

import numpy as np
import pytest

from so101_vlm.eval.geometry import PlaceRegion, get_geometry
from so101_vlm.eval.layout import sample_layout
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


def test_uniform_xy_is_not_implemented_yet():
    with pytest.raises(NotImplementedError):
        sample_scenario(0, ScenarioSpec(box=UniformXY()))


def test_sampling_does_not_use_the_global_rng():
    np.random.seed(123)
    a = sample_scenario(4)
    np.random.seed(999)
    assert sample_scenario(4) == a

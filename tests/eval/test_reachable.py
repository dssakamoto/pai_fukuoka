"""ReachableRegion と読み込みのテスト (SGT-009)。

合成の mask で振る舞いを確かめる (IK は回さない)。保存済みの configs/reachable.yaml に対するテストは、
ファイルがあるときだけ走る (scripts/compute_reachable.py で生成してコミットする。無ければ skip 理由に手順を出す)。
"""

import dataclasses

import pytest
import yaml

from so101_vlm.eval.reachable import REACHABLE_PATH, ReachableRegion, load_reachable_region, region_from_dict
from so101_vlm.robot.model_paths import REPO_ROOT

# x: 0.10, 0.20, 0.30 / y: -0.10, 0.00, 0.10 (行 = x、列 = y)
MASK = (
    (False, True, False),
    (True, True, True),
    (False, True, True),
)
REGION = ReachableRegion(0.10, -0.10, 0.10, MASK)


# =============================================================== contains_xy / bbox
def test_contains_xy_on_grid_points():
    assert REGION.contains_xy(0.20, -0.10) is True
    assert REGION.contains_xy(0.10, -0.10) is False
    assert REGION.contains_xy(0.30, 0.10) is True


def test_contains_xy_uses_the_nearest_grid_point():
    assert REGION.contains_xy(0.2049, 0.0) is True  # 0.20 の格子点に丸まる
    assert REGION.contains_xy(0.14, -0.10) is False  # 0.10 の格子点 (False) に丸まる
    assert REGION.contains_xy(0.16, -0.10) is True  # 0.20 の格子点 (True) に丸まる


def test_contains_xy_outside_the_grid_is_false():
    assert REGION.contains_xy(0.0, 0.0) is False
    assert REGION.contains_xy(0.9, 0.0) is False
    assert REGION.contains_xy(0.2, -0.5) is False
    assert REGION.contains_xy(0.2, 0.5) is False


@pytest.mark.parametrize("x,y", [(float("nan"), 0.0), (0.0, float("inf")), (True, 0.0), ("0.2", 0.0)])
def test_contains_xy_rejects_non_numbers(x, y):
    with pytest.raises(ValueError):
        REGION.contains_xy(x, y)


def test_bbox_matches_the_true_range():
    (xl, xh), (yl, yh) = REGION.bbox()
    assert (xl, xh) == pytest.approx((0.10, 0.30)) and (yl, yh) == pytest.approx((-0.10, 0.10))
    inner = ReachableRegion(0.0, 0.0, 0.5, ((False, False, False), (False, True, False), (False, False, False)))
    assert inner.bbox() == ((0.5, 0.5), (0.5, 0.5))  # True が 1 点だけ


def test_region_is_frozen_and_comparable():
    assert REGION == ReachableRegion(0.10, -0.10, 0.10, MASK) and hash(REGION) == hash(ReachableRegion(0.10, -0.10, 0.10, MASK))
    with pytest.raises(dataclasses.FrozenInstanceError):
        REGION.step = 1.0  # type: ignore[misc]


# =============================================================== 不正な入力
@pytest.mark.parametrize(
    "kwargs",
    [
        {"step": 0.0}, {"step": -0.1}, {"step": float("nan")}, {"x0": float("inf")},
        {"mask": ()}, {"mask": ((),)},
        {"mask": ((True, False), (True,))},  # 行の長さが不揃い
        {"mask": ((False, False), (False, False))},  # True が無い
        {"mask": ((1, 0), (0, 1))},  # bool でない
        {"mask": [[True]]},  # tuple でない
    ],
)
def test_invalid_region_raises_value_error(kwargs):
    base = dict(x0=0.0, y0=0.0, step=0.1, mask=((True, False), (False, True)))
    with pytest.raises(ValueError):
        ReachableRegion(**{**base, **kwargs})


# =============================================================== yaml の読み書き
def _write(tmp_path, raw):
    path = tmp_path / "reachable.yaml"
    path.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    return path


def test_roundtrip_through_yaml(tmp_path):
    path = _write(tmp_path, REGION.to_yaml_dict())
    assert load_reachable_region(path) == REGION
    assert REGION.to_yaml_dict()["rows"] == ["010", "111", "011"]


@pytest.mark.parametrize(
    "raw",
    [
        {"x0": 0, "y0": 0, "step": 0.1, "rows": ["010", "11"]},  # 行長が不揃い
        {"x0": 0, "y0": 0, "step": 0.1, "rows": ["012", "111"]},  # 0/1 以外
        {"x0": 0, "y0": 0, "step": 0.1, "rows": ["000", "000"]},  # True が無い
        {"x0": 0, "y0": 0, "step": 0.1, "rows": []},
        {"x0": 0, "y0": 0, "step": 0.1, "rows": [10, 11]},  # 引用符なしの数字 (文字列でない)
        {"x0": 0, "y0": 0, "step": 0.1},  # rows が無い
        {"x0": 0, "y0": 0, "rows": ["1"]},  # step が無い
        {"x0": "a", "y0": 0, "step": 0.1, "rows": ["1"]},
        {"x0": 0, "y0": 0, "step": 0, "rows": ["1"]},
        {"x0": True, "y0": 0, "step": 0.1, "rows": ["1"]},
        ["not", "a", "dict"],
    ],
)
def test_invalid_yaml_raises_value_error(tmp_path, raw):
    with pytest.raises(ValueError):
        load_reachable_region(_write(tmp_path, raw))


def test_unquoted_digit_rows_are_rejected_not_misread(tmp_path):
    path = tmp_path / "r.yaml"
    path.write_text("x0: 0\ny0: 0\nstep: 0.1\nrows:\n  - 010\n  - 111\n", encoding="utf-8")  # YAML では数値になる
    with pytest.raises(ValueError):
        load_reachable_region(path)


def test_missing_and_broken_files_raise_value_error_with_the_path(tmp_path):
    with pytest.raises(ValueError, match="compute_reachable"):
        load_reachable_region(tmp_path / "none.yaml")
    broken = tmp_path / "broken.yaml"
    broken.write_text("x0: [unclosed", encoding="utf-8")
    with pytest.raises(ValueError, match="broken.yaml"):
        load_reachable_region(broken)


def test_region_from_dict_names_the_source():
    with pytest.raises(ValueError, match="my-source"):
        region_from_dict({}, "my-source")


# =============================================================== 保存済みの configs/reachable.yaml
needs_saved = pytest.mark.skipif(
    not REACHABLE_PATH.exists(),
    reason="configs/reachable.yaml がありません。MuJoCo が動く環境で `uv run python scripts/compute_reachable.py` を実行してコミットしてください",
)


@pytest.fixture(scope="module")
def saved():
    return load_reachable_region()


@needs_saved
def test_saved_region_contains_the_current_box_position(saved):
    from so101_vlm.eval.geometry import get_geometry

    assert saved.contains_xy(*get_geometry().box_default_xy) is True  # (0.20, 0.19)


@needs_saved
def test_saved_region_excludes_points_too_near_and_too_far(saved):
    assert saved.contains_xy(0.0, 0.0) is False  # ロボットの付け根の真上 (近すぎる)
    assert saved.contains_xy(0.9, 0.0) is False  # 遠すぎる


@needs_saved
def test_saved_bbox_is_inside_the_motion_workspace(saved):
    """G4: workspace は自動生成しない。到達可能領域が workspace からはみ出すと、安全フィルタが拒否する位置に箱が置かれうる → 検知する 🟡"""
    ws = yaml.safe_load((REPO_ROOT / "configs" / "motion.yaml").read_text(encoding="utf-8"))["safety"]["workspace"]
    (xl, xh), (yl, yh) = saved.bbox()
    eps = 1e-9
    assert ws["lower"][0] - eps <= xl and xh <= ws["upper"][0] + eps, (
        f"到達可能な x {xl:.3f}〜{xh:.3f} が workspace x {ws['lower'][0]}〜{ws['upper'][0]} からはみ出しています。"
        "configs/motion.yaml の workspace を更新してください"
    )
    assert ws["lower"][1] - eps <= yl and yh <= ws["upper"][1] + eps, (
        f"到達可能な y {yl:.3f}〜{yh:.3f} が workspace y {ws['lower'][1]}〜{ws['upper'][1]} からはみ出しています。"
        "configs/motion.yaml の workspace を更新してください"
    )

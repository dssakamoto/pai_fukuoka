"""SCENE_SPEC から箱の位置と作業範囲を削除した後始末の検査 (SGT-013)。

箱の位置は試行ごとの Scenario (と XML の既定位置 SceneGeometry.box_default_xy) が決める。ブロックを置く領域は
eval/validation.py の BLOCK_AREA_X / BLOCK_AREA_Y。src/ にそれらの古い名前が戻ってこないことを保証する。
"""

import dataclasses
import re

import pytest

from so101_vlm.eval.scene_spec import SCENE_SPEC, SceneSpec
from so101_vlm.robot.model_paths import REPO_ROOT

FORBIDDEN = ("box_center_xy", "work_area_x", "work_area_y", "work_area_width", "judge_in_box")


def _sources():
    return sorted((REPO_ROOT / "src").rglob("*.py")) + sorted((REPO_ROOT / "scripts").glob("*.py"))


@pytest.mark.parametrize("token", FORBIDDEN)
def test_removed_names_do_not_appear_in_src_or_scripts(token):
    pattern = re.compile(rf"\b{token}\b")
    hits = [
        f"{path.relative_to(REPO_ROOT)}:{n}"
        for path in _sources()
        for n, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1)
        if pattern.search(line) and not line.lstrip().startswith(("#", '"""'))
    ]  # fmt: skip
    # judge.py の docstring が、削除した経緯として judge_in_box の名前を 1 度だけ挙げるのは許す
    hits = [h for h in hits if not (token == "judge_in_box" and h.startswith("src/so101_vlm/eval/judge.py:1"))]
    assert not hits, f"{token} の参照が残っています: {hits}"


def test_scene_spec_has_no_box_position_or_work_area():
    names = {f.name for f in dataclasses.fields(SceneSpec)}
    assert not names & {"box_center_xy", "work_area_x", "work_area_y"}
    assert not hasattr(SCENE_SPEC, "box_center_xy")


def test_block_area_constants_are_the_old_work_area():
    """移した値は従来と同じ (ブロックの配置が変わらない) 🔵"""
    from so101_vlm.eval.validation import BLOCK_AREA_X, BLOCK_AREA_Y, block_area_width

    assert BLOCK_AREA_X == (0.15, 0.35) and BLOCK_AREA_Y == (-0.10, 0.10)
    assert block_area_width() == pytest.approx(0.20)

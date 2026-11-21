"""scripts/sweep_box.py のテスト (SGT-012)。実際の描画を使う実行だけ render 印 (CI では除かれる)。"""

import csv
import importlib.util
from types import SimpleNamespace

import pytest

from so101_vlm.eval.failure import FailureKind
from so101_vlm.eval.geometry import get_geometry
from so101_vlm.eval.reachable import REACHABLE_PATH, ReachableRegion
from so101_vlm.eval.scenario import box_outline_fits
from so101_vlm.eval.trial import TrialOutcome
from so101_vlm.robot.model_paths import REPO_ROOT

SCRIPT_PATH = REPO_ROOT / "scripts" / "sweep_box.py"
G = get_geometry()


@pytest.fixture(scope="module")
def script():
    spec = importlib.util.spec_from_file_location("sweep_box_script", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _grid(x_range, y_range, step=0.01):
    nx, ny = round((x_range[1] - x_range[0]) / step) + 1, round((y_range[1] - y_range[0]) / step) + 1
    return ReachableRegion(x_range[0], y_range[0], step, tuple(tuple(True for _ in range(ny)) for _ in range(nx)))


REGION = _grid((0.10, 0.45), (-0.15, 0.30))


class FakeClient:
    provider, model = "oracle", "ground-truth"


def _outcome(seed, colour, instruction, *, ok=True, failure=FailureKind.NOT_IN_BOX):
    run = SimpleNamespace(success=True, planning_error=None, plan_time_s=0.1, other_block_moved=False) if ok else None
    return TrialOutcome(
        seed=seed, colour=colour, instruction=instruction, plan=None, pick_estimate=None, place_estimate=None,
        pick_error_mm=None, run=run, failure=None if ok else failure, error=None, vlm_time_s=0.0,
    )  # fmt: skip


def _trial_fn(log=None, fail=lambda seed: None):
    def trial(seed, colour, instruction, vlm, **kw):
        if log is not None:
            log.append((seed, colour, kw.get("scenario")))
        kind = fail(seed)
        return _outcome(seed, colour, instruction, ok=kind is None, failure=kind)

    return trial


# =============================================================== sweep_points
def test_sweep_points_are_nine_and_all_fit(script):
    pts = script.sweep_points(REGION, G)
    assert len(pts) == 9 == len(set(pts))
    assert all(box_outline_fits(REGION, p, G) for p in pts)


def test_sweep_points_are_deterministic(script):
    assert script.sweep_points(REGION, G) == script.sweep_points(REGION, G)


def test_sweep_points_cover_the_window_corners(script):
    pts = script.sweep_points(REGION, G)
    outer = G.box_inner_half_size + G.box_wall_thickness
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    assert min(xs) == pytest.approx(0.10 + outer, abs=0.011) and max(xs) == pytest.approx(0.45 - outer, abs=0.011)
    assert min(ys) == pytest.approx(-0.15 + outer, abs=0.011) and max(ys) == pytest.approx(0.30 - outer, abs=0.011)


def test_sweep_points_window_too_narrow_raises_value_error(script):
    with pytest.raises(ValueError, match="狭く"):
        script.sweep_points(_grid((0.20, 0.30), (0.10, 0.30)), G)  # x 方向に適格な格子点が 1 つしかない (3 点に分けられない)
    with pytest.raises(ValueError, match="収まる位置"):
        script.sweep_points(_grid((0.20, 0.25), (0.10, 0.30)), G)  # 箱の外周が入らない


needs_saved = pytest.mark.skipif(not REACHABLE_PATH.exists(), reason="configs/reachable.yaml がありません")


@needs_saved
def test_sweep_points_with_the_saved_region(script):
    from so101_vlm.eval.reachable import load_reachable_region

    region = load_reachable_region()
    pts = script.sweep_points(region, G)
    assert len(set(pts)) == 9 and all(box_outline_fits(region, p, G) for p in pts)


# =============================================================== run_point / 集計
def test_run_point_skips_seeds_whose_blocks_are_too_close_to_the_box(script):
    log = []
    near_blocks = (0.20, 0.10)  # ブロックの作業範囲の中。近すぎる seed が出る
    res = script.run_point(
        near_blocks, 3, 0, geometry=G, client_for=lambda s, c: FakeClient(), trial_fn=_trial_fn(log),
    )  # fmt: skip
    assert len(res.records) == 3 and res.skipped >= 0
    from so101_vlm.eval.scenario import box_clear_of_blocks

    for seed, _colour, scenario in log:
        assert box_clear_of_blocks(scenario.blocks, near_blocks, G, 0.03)
        assert scenario.box_xy == near_blocks  # 箱はその位置に固定


def test_run_point_uses_consecutive_valid_seeds_and_cycles_colours(script):
    log = []
    far = (0.20, 0.25)  # ブロックから遠い: 除かれる seed なし
    res = script.run_point(far, 4, 5, geometry=G, client_for=lambda s, c: FakeClient(), trial_fn=_trial_fn(log))
    assert res.skipped == 0 and [s for s, _c, _sc in log] == [5, 6, 7, 8]
    assert [c for _s, c, _sc in log] == ["red", "blue", "green", "red"]


def test_run_point_raises_when_no_seed_is_clear_enough(script):
    inside_blocks = (0.25, 0.0)  # 箱がブロックの作業範囲の真ん中: どの seed も近すぎる
    with pytest.raises(ValueError, match="見つかりません"):
        script.run_point(inside_blocks, 5, 0, geometry=G, client_for=lambda s, c: FakeClient(), trial_fn=_trial_fn())


def test_row_separates_ik_failures_from_the_rest(script):
    far = (0.20, 0.25)
    fail = lambda seed: FailureKind.PLANNING_FAILURE if seed == 0 else (FailureKind.NOT_IN_BOX if seed == 1 else None)  # noqa: E731
    res = script.run_point(far, 4, 0, geometry=G, client_for=lambda s, c: FakeClient(), trial_fn=_trial_fn(fail=fail))
    row = res.row()
    assert (row["trials"], row["successes"], row["ik_failures"], row["place_failures"]) == (4, 2, 1, 1)
    assert row["success_rate"] == "0.500" and row["success_rate_excl_ik"] == "0.667"  # 2 / (4 - 1)


def test_write_outputs_has_the_expected_columns_and_rows(script, tmp_path):
    far = (0.20, 0.25)
    results = [
        script.run_point(far, 2, 0, geometry=G, client_for=lambda s, c: FakeClient(), trial_fn=_trial_fn()),
        script.run_point((0.22, 0.25), 2, 0, geometry=G, client_for=lambda s, c: FakeClient(), trial_fn=_trial_fn()),
    ]
    script.write_outputs(results, tmp_path, 2, 0)
    with open(tmp_path / "box_sweep.csv", encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))
    assert tuple(rows[0]) == script.CSV_COLUMNS and len(rows) == 2
    assert [r["box_x"] for r in rows] == ["0.2", "0.22"] and all(r["successes"] == "2" for r in rows)
    md = (tmp_path / "box_sweep.md").read_text(encoding="utf-8")
    assert "| 箱 x |" in md and "2 / 2" in md and "全体: 4 / 4" in md


# =============================================================== 引数
def test_help_exits_zero(script, capsys):
    with pytest.raises(SystemExit) as exc:
        script.main(["--help"])
    assert exc.value.code == 0 and "--trials" in capsys.readouterr().out


@pytest.mark.parametrize("args", [["--trials", "0"], ["--trials", "-1"], ["--seed", "-1"], ["--limit-points", "0"]])
def test_invalid_arguments_exit_with_code_2(script, args, capsys):
    assert script.main(args) == 2
    assert "不正" in capsys.readouterr().err


@needs_saved
@pytest.mark.render
def test_small_real_sweep_writes_csv(script, tmp_path, capsys):
    """2 点 x 1 試行の実行 (描画が要る) 🟡"""
    assert script.main(["--trials", "1", "--limit-points", "2", "--out", str(tmp_path)]) == 0
    with open(tmp_path / "box_sweep.csv", encoding="utf-8", newline="") as f:
        assert len(list(csv.DictReader(f))) == 2

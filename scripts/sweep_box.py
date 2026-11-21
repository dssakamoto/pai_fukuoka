"""箱の位置を 3x3 の格子で動かし、各点で oracle の成功率を集計する (SGT-012)。API 料金は発生しない。

  uv run python scripts/sweep_box.py --trials 10 --seed 0 --out results/box_sweep
  uv run python scripts/sweep_box.py --trials 1 --limit-points 2 --out /tmp/sweep   # 動作確認 (2 点 x 1 試行)

格子点: 到達可能領域 (configs/reachable.yaml) のうち、箱の外周が全部収まる中心 (適格な点) を、x を 3 等分した 3 列と、各列の適格な
y の範囲を 3 等分した 3 行の 3x3 とする (適格な点は長方形とは限らない。設計 G6)。箱を置ける範囲は狭い (既定位置の周り約 5〜6 cm) ので、その窓の中を掃引する。
各点で `--trials` 試行 (seed は --seed から順に、色は red / blue / green を順に)。ブロックは箱に近すぎる seed (箱の外周と
ブロックの縁の間隔が block_gap 未満) を除く: ブロックは箱と無関係に配置されるので、重なった配置は物理的に無意味。除いた seed 数は表に出す。
出力: <out>/box_sweep.csv (箱 xy・試行数・成功数・失敗の分類)、<out>/box_sweep.md (Markdown の表)。
表の「IK 失敗を除く成功率」= 成功 / (試行 - IK の計画失敗)。箱の位置による影響と、ブロックが付け根に近いことによる計画失敗を分けて見る。
描画 (OpenGL) が要る (render)。終了コード: 0 = 完了、2 = 引数・設定の誤り。
"""

import argparse
import csv
import sys
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from pathlib import Path

from so101_vlm.eval.eval_config import load_eval_config
from so101_vlm.eval.geometry import SceneGeometry, get_geometry
from so101_vlm.eval.reachable import ReachableRegion, load_reachable_region
from so101_vlm.eval.runner import (
    DEFAULT_COLOURS,
    DEFAULT_INSTRUCTION,
    TrialRecord,
    oracle_client_factory,
    record_from_outcome,
    summarize,
)
from so101_vlm.eval.scenario import Fixed, UniformXY, box_clear_of_blocks, box_outline_fits, scenario_for
from so101_vlm.eval.trial import run_trial
from so101_vlm.perception.calibration import load_calibration
from so101_vlm.robot.model_paths import REPO_ROOT
from so101_vlm.validation import require_int

DEFAULT_OUT_DIR = REPO_ROOT / "results" / "box_sweep"
SCAN_FACTOR = 20  # 1 点で、除外を含めて何倍の seed まで探すか (これを超えたら ValueError)
CSV_COLUMNS = (
    "box_x", "box_y", "trials", "successes", "success_rate", "ik_failures", "success_rate_excl_ik",
    "vlm_failures", "grasp_failures", "place_failures", "skipped_seeds",
)  # fmt: skip


def sweep_points(region: ReachableRegion, geometry: SceneGeometry, n: int = 3) -> tuple[tuple[float, float], ...]:
    """【機能概要】: 箱の外周が領域に収まる中心 (格子点) から、n x n の点列を返す (x の列ごとに y を n 等分。決定的) 🟡

    点が重なる (窓が n 点より狭い) なら ValueError。
    """
    n = require_int("n", n, 2)
    (xl, xh), (yl, yh) = region.bbox()
    eligible = [
        (xl + i * region.step, yl + j * region.step)
        for i in range(round((xh - xl) / region.step) + 1)
        for j in range(round((yh - yl) / region.step) + 1)
        if box_outline_fits(region, (xl + i * region.step, yl + j * region.step), geometry)
    ]  # fmt: skip
    if not eligible:
        raise ValueError("箱の外周が収まる位置が、到達可能領域にありません")
    # 適格な点は長方形とは限らない (角が欠ける) ので、x を n 等分して最寄りの列を選び、その列の適格な y の範囲を n 等分する
    xs = sorted({x for x, _ in eligible if sum(1 for px, _ in eligible if px == x) >= n})  # n 点以上取れる列だけ
    if len(xs) < n:
        raise ValueError(f"適格な位置の範囲が狭く、{n}x{n} の異なる点を取れません (y に n 点以上取れる x の列が {len(xs)} 本)")
    columns = [min(xs, key=lambda v: (abs(v - (xs[0] + (xs[-1] - xs[0]) * k / (n - 1))), v)) for k in range(n)]
    points: list[tuple[float, float]] = []
    for x in columns:
        ys = sorted(p[1] for p in eligible if p[0] == x)
        for k in range(n):
            target = ys[0] + (ys[-1] - ys[0]) * k / (n - 1)
            points.append((x, min(ys, key=lambda v: (abs(v - target), v))))
    if len(set(points)) != len(points):
        raise ValueError(f"適格な位置の範囲が狭く、{n}x{n} の異なる点を取れません: x {xs[0]:.3f}〜{xs[-1]:.3f}")
    return tuple((round(x, 6), round(y, 6)) for x, y in points)


@dataclass(frozen=True)
class PointResult:
    """【機能概要】: 1 つの箱位置の集計。records は採用した試行、skipped は箱に近すぎて除いた seed の数 🟡"""

    box_xy: tuple[float, float]
    records: tuple[TrialRecord, ...]
    skipped: int

    def row(self) -> dict[str, object]:
        s = summarize(self.records)
        n_ik = s.by_group.get("ik", 0)
        non_ik = s.n_trials - n_ik
        return {
            "box_x": self.box_xy[0], "box_y": self.box_xy[1], "trials": s.n_trials, "successes": s.n_success,
            "success_rate": "" if s.success_rate is None else f"{s.success_rate:.3f}",
            "ik_failures": n_ik, "success_rate_excl_ik": "" if non_ik <= 0 else f"{s.n_success / non_ik:.3f}",
            "vlm_failures": s.by_group.get("vlm", 0), "grasp_failures": s.by_group.get("grasp", 0),
            "place_failures": s.by_group.get("place", 0), "skipped_seeds": self.skipped,
        }  # fmt: skip


def run_point(
    box_xy: tuple[float, float],
    trials: int,
    base_seed: int,
    *,
    geometry: SceneGeometry,
    client_for: Callable[[int, str], object],
    trial_fn: Callable[..., object] = run_trial,
    block_gap: float | None = None,
    on_record: Callable[[tuple[float, float], int, int, TrialRecord], None] | None = None,
) -> PointResult:
    """【機能概要】: 箱を box_xy に固定して、trials 試行を行う。ブロックが箱に近すぎる seed は除く 🟡"""
    placement = load_eval_config().placement
    gap = UniformXY().block_gap if block_gap is None else block_gap
    records: list[TrialRecord] = []
    skipped, seed = 0, base_seed
    while len(records) < trials:
        if seed - base_seed >= SCAN_FACTOR * trials:
            raise ValueError(f"箱 {box_xy} に近すぎない seed が {trials} 個見つかりません (seed {base_seed}〜{seed - 1} を調べました)")
        scenario = scenario_for(seed, placement, Fixed(box_xy))
        if not box_clear_of_blocks(scenario.blocks, box_xy, geometry, gap):
            skipped += 1
            seed += 1
            continue
        k = len(records)
        colour = DEFAULT_COLOURS[k % len(DEFAULT_COLOURS)]
        client = client_for(seed, colour)
        started = time.perf_counter()
        outcome = trial_fn(seed, colour, DEFAULT_INSTRUCTION.replace("{colour}", colour), client, scenario=scenario)
        record = record_from_outcome(
            k, outcome, provider=str(getattr(client, "provider", "unknown")),
            model=str(getattr(client, "model", "unknown")), total_time_s=time.perf_counter() - started,
        )  # fmt: skip
        records.append(record)
        if on_record is not None:
            on_record(box_xy, k + 1, trials, record)
        seed += 1
    return PointResult(box_xy, tuple(records), skipped)


def format_markdown(results: Sequence[PointResult], trials: int, base_seed: int) -> str:
    """【機能概要】: 箱位置ごとの成功数と、IK 失敗以外の失敗を、Markdown の表にする 🟡"""
    lines = [
        f"箱位置の掃引 (oracle、各点 {trials} 試行、seed {base_seed} から)",
        "",
        "| 箱 x | 箱 y | 成功 / 試行 | 成功率 | IK 失敗 | IK 失敗を除く成功率 | 掴み失敗 | 置き失敗 | 除いた seed |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for r in results:
        row = r.row()
        rate = f"{float(row['success_rate']) * 100:.0f}%" if row["success_rate"] != "" else "-"
        excl = f"{float(row['success_rate_excl_ik']) * 100:.0f}%" if row["success_rate_excl_ik"] != "" else "-"
        lines.append(
            f"| {row['box_x']:.3f} | {row['box_y']:.3f} | {row['successes']} / {row['trials']} | {rate} | {row['ik_failures']} "
            f"| {excl} | {row['grasp_failures']} | {row['place_failures']} | {row['skipped_seeds']} |"
        )
    total = sum(len(r.records) for r in results)
    ok = sum(1 for r in results for rec in r.records if rec.success)
    lines += ["", f"全体: {ok} / {total} ({ok / total * 100:.0f}%)" if total else "全体: 試行なし"]
    return "\n".join(lines) + "\n"


def write_outputs(results: Sequence[PointResult], out: Path, trials: int, base_seed: int) -> None:
    """【機能概要】: box_sweep.csv と box_sweep.md を書く (UTF-8) 🟡"""
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "box_sweep.csv", "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS, lineterminator="\n")
        writer.writeheader()
        writer.writerows(r.row() for r in results)
    (out / "box_sweep.md").write_text(format_markdown(results, trials, base_seed), encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--trials", type=int, default=10, help="各点の試行数 (既定 10)")
    p.add_argument("--seed", type=int, default=0, help="最初の seed (既定 0)")
    p.add_argument("--out", type=Path, default=DEFAULT_OUT_DIR)
    p.add_argument("--limit-points", type=int, default=None, help="動作確認用: 先頭の N 点だけ")
    args = p.parse_args(argv)
    try:
        trials = require_int("--trials", args.trials, 1)
        seed = require_int("--seed", args.seed, 0)
        limit = None if args.limit_points is None else require_int("--limit-points", args.limit_points, 1)
        geometry = get_geometry()
        points = sweep_points(load_reachable_region(), geometry)
    except ValueError as exc:
        print(f"引数・設定が不正です: {exc}", file=sys.stderr)
        return 2
    if limit is not None:
        points = points[:limit]
    results: list[PointResult] = []
    try:
        for xy in points:
            print(f"箱 ({xy[0]:.3f}, {xy[1]:.3f})", flush=True)
            results.append(
                run_point(
                    xy, trials, seed, geometry=geometry,
                    client_for=oracle_client_factory(load_calibration(), Fixed(xy)),
                    on_record=lambda b, d, n, r: print(f"  [{d}/{n}] {'成功' if r.success else '失敗 ' + str(r.failure)}", flush=True),
                )
            )  # fmt: skip
    except ValueError as exc:
        print(f"掃引できません: {exc}", file=sys.stderr)
        return 2
    write_outputs(results, args.out, trials, seed)
    print()
    print(format_markdown(results, trials, seed))
    print(f"保存しました: {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

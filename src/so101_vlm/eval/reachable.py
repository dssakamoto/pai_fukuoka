"""到達可能領域 ReachableRegion と、保存済みの yaml の読み込み (SGT-009)。

【機能概要】ワールド xy の格子上で「IK が解ける点」の mask を持つ。計算は scripts/compute_reachable.py で一度だけ行い、
configs/reachable.yaml に保存する。実行時とテストは保存済みの結果を読むだけ (IK を回さない。FR-009 / NFR-301)。
箱の位置の生成 (UniformXY、SGT-010) が、到達できない位置を棄却するのに使う。

【形式】yaml: x0 / y0 [m] (格子の原点)、step [m]、rows (行 = x、列 = y の「0」「1」の文字列)。差分が目で読める。
格子点 (x0 + i * step, y0 + j * step) が代表点で、contains_xy は最も近い格子点の値を返す (格子の外は False)。

【依存の向き】policy / perception / safety を import しない (IK は計算スクリプトだけが使う)。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import yaml

from so101_vlm.robot.model_paths import REPO_ROOT
from so101_vlm.validation import require_number

REACHABLE_PATH = REPO_ROOT / "configs" / "reachable.yaml"


@dataclass(frozen=True)
class ReachableRegion:
    """【機能概要】: 格子上の到達可能 mask。行 = x、列 = y。tuple のみ (frozen・比較可能) 🟡

    x0 / y0: 格子点 (0, 0) の位置 [m]、step: 格子の間隔 [m] (> 0)。
    mask は空でなく、行の長さがそろい、True が 1 つ以上あること。
    """

    x0: float
    y0: float
    step: float
    mask: tuple[tuple[bool, ...], ...]

    def __post_init__(self) -> None:
        for name in ("x0", "y0", "step"):
            require_number(name, getattr(self, name))
        if self.step <= 0:
            raise ValueError(f"step は正で指定してください: step={self.step!r}")
        if not isinstance(self.mask, tuple) or not self.mask or not all(isinstance(r, tuple) for r in self.mask):
            raise ValueError("mask は空でない tuple の tuple で指定してください")
        width = len(self.mask[0])
        if width == 0 or any(len(r) != width for r in self.mask):
            raise ValueError("mask の行の長さがそろっていません (または空の行があります)")
        if any(type(v) is not bool for r in self.mask for v in r):
            raise ValueError("mask は bool だけで指定してください")
        if not any(v for r in self.mask for v in r):
            raise ValueError("mask に到達可能な点 (True) がありません")

    def contains_xy(self, x: float, y: float) -> bool:
        """【機能概要】: (x, y) に最も近い格子点が到達可能なら True。格子の外は False 🟡"""
        x = require_number("x", x)
        y = require_number("y", y)
        i = round((x - self.x0) / self.step)
        j = round((y - self.y0) / self.step)
        if not (0 <= i < len(self.mask) and 0 <= j < len(self.mask[0])):
            return False
        return self.mask[i][j]

    def bbox(self) -> tuple[tuple[float, float], tuple[float, float]]:
        """【機能概要】: 到達可能な格子点の範囲 ((x の最小, 最大), (y の最小, 最大)) [m] 🟡"""
        xs = [i for i, row in enumerate(self.mask) if any(row)]
        ys = [j for j in range(len(self.mask[0])) if any(row[j] for row in self.mask)]
        return (
            (self.x0 + min(xs) * self.step, self.x0 + max(xs) * self.step),
            (self.y0 + min(ys) * self.step, self.y0 + max(ys) * self.step),
        )

    def to_yaml_dict(self) -> dict:
        """【機能概要】: yaml に書く辞書 (rows は 0/1 の文字列。load_reachable_region と対) 🟡"""
        return {
            "x0": float(self.x0), "y0": float(self.y0), "step": float(self.step),
            "rows": ["".join("1" if v else "0" for v in row) for row in self.mask],
        }  # fmt: skip


def region_from_dict(raw: object, source: str = "reachable") -> ReachableRegion:
    """【機能概要】: yaml の辞書から ReachableRegion を作る。形式の違いは ValueError (どの項目かを含む) 🟡"""
    if not isinstance(raw, dict):
        raise ValueError(f"{source}: 辞書で書いてください")
    missing = [k for k in ("x0", "y0", "step", "rows") if k not in raw]
    if missing:
        raise ValueError(f"{source}: 項目がありません: {missing}")
    rows = raw["rows"]
    if not isinstance(rows, list) or not rows or not all(isinstance(r, str) for r in rows):
        raise ValueError(f"{source}: rows は 0/1 の文字列の空でないリストで指定してください")
    for n, row in enumerate(rows):
        bad = set(row) - {"0", "1"}
        if bad:
            raise ValueError(f"{source}: rows[{n}] に 0/1 以外の文字があります: {sorted(bad)}")
    if len({len(r) for r in rows}) != 1:
        raise ValueError(f"{source}: rows の行の長さがそろっていません: {[len(r) for r in rows]}")
    for key in ("x0", "y0", "step"):
        value = raw[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"{source}: {key} は有限の数値で指定してください: {value!r}")
    mask = tuple(tuple(c == "1" for c in row) for row in rows)
    return ReachableRegion(float(raw["x0"]), float(raw["y0"]), float(raw["step"]), mask)


def load_reachable_region(path: Path = REACHABLE_PATH) -> ReachableRegion:
    """【機能概要】: 保存済みの yaml を読む。無い・壊れている・形式が違うは ValueError (パスを含む) 🟡"""
    path = Path(path)
    try:
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise ValueError(
            f"{path} がありません。`uv run python scripts/compute_reachable.py` で作ってください"
        ) from None
    except yaml.YAMLError as exc:
        raise ValueError(f"{path}: yaml として読めません: {exc}") from exc
    return region_from_dict(raw, str(path))

"""VLM の出力スキーマと検証 (TASK-0017)。

🔵 design.md 4.3: Detection / PlanResult は frozen。座標は各社の形式から画像ピクセルに直す。
🔵 FR-03: JSON 不正・対象なしは PlanError (ロボットは動かさない)。
座標形式の名前は configs/vlm.yaml の coord_format と一致させる。

VLM の JSON 形式 (共通プロンプトで要求する):
    {"pick": {"label": str, "box": [4 つの数], "confidence": 0〜1 (省略可)},
     "place": {"label": str, "box": [...]},
     "error": str (省略可。対象が見つからない理由)}
"""

import json
import math
import re
from dataclasses import dataclass

# 🔵 configs/vlm.yaml の coord_format と同じ名前
COORD_FORMATS = ("xyxy_px", "yxyx_norm1000")
_NORM = 1000.0
_FENCE = re.compile(r"^```[a-zA-Z]*\s*\n?(.*?)\n?```\s*$", re.DOTALL)


class PlanError(Exception):
    """【機能概要】: VLM の返事が使えない (JSON 不正・対象なし・座標不正)。ロボットは動かさない 🔵"""


@dataclass(frozen=True)
class Detection:
    """【機能概要】: 1 つの検出。bbox_px = (x1, y1, x2, y2) 画像ピクセル、x1<x2, y1<y2 🔵"""

    label: str
    bbox_px: tuple[int, int, int, int]
    confidence: float | None


@dataclass(frozen=True)
class PlanResult:
    """【機能概要】: 掴む対象と置き場所 🔵"""

    pick: Detection
    place: Detection


def _is_number(v: object) -> bool:
    return isinstance(v, int | float) and not isinstance(v, bool) and math.isfinite(v)


# 🟡 Intent: はみ出しは画像内に収め、潰れた箱 (面積 0) は検出失敗として拒否する。
def box_to_pixels(
    box: object, coord_format: str, width: int, height: int
) -> tuple[int, int, int, int]:
    """【機能概要】: 4 つの数を画像ピクセルの (x1, y1, x2, y2) に直す 🔵

    角の順序が逆なら入れ替え、[0, width-1] x [0, height-1] にクリップする。
    クリップ後に幅か高さが 0 なら PlanError。未知の形式・画像サイズ不正は ValueError。
    """
    if coord_format not in COORD_FORMATS:
        raise ValueError(f"coord_format は {COORD_FORMATS} のどれか: {coord_format!r}")
    if not (isinstance(width, int) and isinstance(height, int) and width > 0 and height > 0):
        raise ValueError(f"画像サイズは正の整数で指定してください: {width!r}x{height!r}")
    if not isinstance(box, list | tuple) or len(box) != 4 or not all(_is_number(v) for v in box):
        raise PlanError(f"box は 4 つの有限の数で指定してください: {box!r}")
    a, b, c, d = (float(v) for v in box)
    if coord_format == "yxyx_norm1000":
        x1, y1, x2, y2 = b / _NORM * width, a / _NORM * height, d / _NORM * width, c / _NORM * height
    else:
        x1, y1, x2, y2 = a, b, c, d
    x1, x2 = sorted((x1, x2))
    y1, y2 = sorted((y1, y2))
    px = (
        min(max(round(x1), 0), width - 1),
        min(max(round(y1), 0), height - 1),
        min(max(round(x2), 0), width - 1),
        min(max(round(y2), 0), height - 1),
    )
    if px[0] >= px[2] or px[1] >= px[3]:
        raise PlanError(f"箱が画像の外か、潰れています: {box!r} -> {px}")
    return px


def _parse_detection(
    name: str, raw: object, coord_format: str, width: int, height: int, reason: str | None
) -> Detection:
    if not isinstance(raw, dict):
        suffix = f": {reason}" if reason else ""
        raise PlanError(f"{name} がありません{suffix}")
    label = raw.get("label")
    if not isinstance(label, str) or not label.strip():
        raise PlanError(f"{name}.label は空でない文字列で指定してください: {label!r}")
    if "box" not in raw:
        raise PlanError(f"{name}.box がありません")
    bbox = box_to_pixels(raw["box"], coord_format, width, height)
    conf = raw.get("confidence")
    if conf is not None and not (_is_number(conf) and 0.0 <= conf <= 1.0):
        raise PlanError(f"{name}.confidence は 0〜1 の数で指定してください: {conf!r}")
    return Detection(label.strip(), bbox, None if conf is None else float(conf))


# 🔵 Intent: コードフェンス付きで返すモデルがあるため剥がす。JSON 以外は PlanError (FR-03)。
def parse_plan(text: str, coord_format: str, width: int, height: int) -> PlanResult:
    """【機能概要】: VLM の返事 (文字列) を検証して PlanResult にする 🔵

    不正な JSON・pick/place の欠落・座標不正・confidence 範囲外は PlanError。
    pick が null で "error" があれば、その理由を PlanError のメッセージに入れる。
    """
    if not isinstance(text, str):
        raise PlanError(f"返事は文字列で指定してください: {type(text).__name__}")
    body = text.strip()
    m = _FENCE.match(body)
    if m:
        body = m.group(1).strip()
    try:
        data = json.loads(body)
    except json.JSONDecodeError as exc:
        raise PlanError(f"JSON として読めません: {exc}") from exc
    if not isinstance(data, dict):
        raise PlanError(f"JSON のトップは object で指定してください: {type(data).__name__}")
    reason = data.get("error")
    reason = reason if isinstance(reason, str) and reason else None
    pick = _parse_detection("pick", data.get("pick"), coord_format, width, height, reason)
    place = _parse_detection("place", data.get("place"), coord_format, width, height, reason)
    return PlanResult(pick, place)

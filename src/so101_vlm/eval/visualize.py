"""検出結果の可視化 (TASK-0023)。

🔵 tasks.md T-23: 画像に VLM の検出枠とラベルを描いて、座標が正しいかを目で確かめる。動画 (TASK-0028) でも使う。
純粋関数 (入力の画像は変更しない、新しい配列を返す)。枠は bbox の内側に描く (辺のピクセルが bbox 座標そのもの)。
ラベルは PIL の既定フォントで描くため、ASCII 以外の文字は読めない (豆腐になる)。座標の確認には影響しない。
"""

from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from so101_vlm.vlm.schema import Detection, PlanResult

# 🟡 シーンのブロック (赤・青・緑)・床・箱と混ざらない色。pick はマゼンタ、place はシアン (ロボットアームは黄色)。
PICK_COLOR = (255, 0, 255)
PLACE_COLOR = (0, 230, 255)
_TEXT_COLOR = (0, 0, 0)


def _check_image(image: object) -> np.ndarray:
    if not isinstance(image, np.ndarray) or image.dtype != np.uint8:
        raise ValueError("image は uint8 の numpy 配列で指定してください")
    if image.ndim != 3 or image.shape[2] != 3 or image.shape[0] < 1 or image.shape[1] < 1:
        raise ValueError(f"image は (H, W, 3) で指定してください: shape={getattr(image, 'shape', None)}")
    return image


def _draw_rect(canvas: np.ndarray, box: tuple[int, int, int, int], color, thickness: int) -> tuple:
    """bbox を画像内にクリップし、内側に太さ thickness の枠を描く。クリップ後の bbox を返す。"""
    h, w = canvas.shape[:2]
    x1, y1, x2, y2 = (
        min(max(box[0], 0), w - 1),
        min(max(box[1], 0), h - 1),
        min(max(box[2], 0), w - 1),
        min(max(box[3], 0), h - 1),
    )
    t = max(1, thickness)
    canvas[y1 : min(y1 + t, y2 + 1), x1 : x2 + 1] = color
    canvas[max(y2 - t + 1, y1) : y2 + 1, x1 : x2 + 1] = color
    canvas[y1 : y2 + 1, x1 : min(x1 + t, x2 + 1)] = color
    canvas[y1 : y2 + 1, max(x2 - t + 1, x1) : x2 + 1] = color
    return x1, y1, x2, y2


def _label_text(role: str, det: Detection) -> str:
    conf = "" if det.confidence is None else f" {det.confidence:.2f}"
    return f"{role}: {det.label}{conf}"


def _draw_label(pil: Image.Image, font, text: str, box: tuple, color) -> None:
    """枠の上に、無ければ下、それも無ければ枠の内側に、色つきの帯とラベルを描く。"""
    draw = ImageDraw.Draw(pil)
    left, top, right, bottom = draw.textbbox((0, 0), text, font=font)
    pad = 2
    lw, lh = right - left + 2 * pad, bottom - top + 2 * pad
    w, h = pil.size
    x1, y1, _, y2 = box
    x = min(max(x1, 0), max(w - lw, 0))
    if y1 - lh >= 0:
        y = y1 - lh
    elif y2 + 1 + lh <= h:
        y = y2 + 1
    else:
        y = y1
    draw.rectangle([x, y, x + lw - 1, y + lh - 1], fill=color)
    draw.text((x + pad - left, y + pad - top), text, fill=_TEXT_COLOR, font=font)


# 🟡 Intent: 小さい画像でも読めるよう、枠の太さと文字の大きさは画像の高さに比例させる。
def draw_plan(image: np.ndarray, plan: PlanResult, *, labels: bool = True) -> np.ndarray:
    """【機能概要】: pick / place の枠 (とラベル) を描いた新しい画像を返す。元の画像は変えない 🔵

    不正な image / plan は ValueError。画像の外にはみ出す bbox は画像内にクリップして描く。
    """
    _check_image(image)
    if not isinstance(plan, PlanResult):
        raise ValueError(f"plan は PlanResult で指定してください: {type(plan).__name__}")
    h = image.shape[0]
    canvas = image.copy()
    thickness = max(1, round(h / 240))
    items = (("pick", plan.pick, PICK_COLOR), ("place", plan.place, PLACE_COLOR))
    boxes = [_draw_rect(canvas, det.bbox_px, color, thickness) for _, det, color in items]
    if not labels:
        return canvas
    pil = Image.fromarray(canvas)
    font = ImageFont.load_default(size=max(12, h // 32))
    for (role, det, color), box in zip(items, boxes, strict=True):
        _draw_label(pil, font, _label_text(role, det), box, color)
    return np.asarray(pil).copy()


def save_overlay(
    path: str | Path, image: np.ndarray, plan: PlanResult, *, labels: bool = True
) -> Path:
    """【機能概要】: draw_plan の結果を PNG で保存し、パスを返す。親ディレクトリは作る 🔵"""
    import imageio.v3 as iio

    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    iio.imwrite(out, draw_plan(image, plan, labels=labels))
    return out

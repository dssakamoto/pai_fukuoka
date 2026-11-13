"""実行動画のフレーム合成とクリップの構成 (TASK-0028)。MuJoCo にも mp4 にも依存しない純粋ロジック。

🔵 AC-06: 動画に 指示・VLM の検出結果・実行・成功判定 が映り、倍速のときは動画内に明記する。流れは 指示 -> 検出 -> 実行 -> 判定。
文字は動画を縮小しても読める大きさ (画像の高さの 1/20) にして、帯の上に描く。日本語は日本語フォントがあれば使い、
無ければ英語の表示に切り替える (Pillow の既定フォントは日本語を描けない)。フォントは環境変数 SO101_VIDEO_FONT でも指定できる。
"""

import math
import os
from collections.abc import Sequence
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from so101_vlm.eval.visualize import draw_plan
from so101_vlm.vlm.schema import PlanResult

STAGES = ("task", "detect", "execute", "result")
FONT_ENV = "SO101_VIDEO_FONT"
# 🟡 日本語を描けるフォントの探索候補 (Windows / macOS / Linux)。最初に見つかったものを使う
CJK_FONT_CANDIDATES = (
    "C:/Windows/Fonts/YuGothM.ttc", "C:/Windows/Fonts/meiryo.ttc", "C:/Windows/Fonts/msgothic.ttc",
    "/System/Library/Fonts/ヒラギノ角ゴシック W4.ttc", "/System/Library/Fonts/Hiragino Sans GB.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/ipafont-gothic/ipag.ttf", "/usr/share/fonts/truetype/takao-gothic/TakaoGothic.ttf",
)  # fmt: skip
_LABELS = {
    "ja": {"task": "指示", "detect": "検出", "execute": "実行", "result": "判定", "ok": "成功", "ng": "失敗"},
    "en": {"task": "Task", "detect": "Detect", "execute": "Execute", "result": "Result", "ok": "SUCCESS", "ng": "FAILED"},
}
_HEADER_BG = (0, 0, 0, 170)
_BAR_BG = (0, 0, 0, 150)
_TEXT = (255, 255, 255)
_ACTIVE = (255, 220, 0)
_DIM = (170, 170, 170)
_OK = (30, 160, 60)
_NG = (200, 50, 50)
_BADGE = (255, 140, 0)
_BANNER_ALPHA = 205  # 帯は半透明: 箱にブロックが入った様子が、帯の下から見える


# --------------------------------------------------------------------------- スタイル・ラベル
def find_cjk_font() -> Path | None:
    """【機能概要】: 日本語を描けるフォントのパス。環境変数 SO101_VIDEO_FONT を優先し、無ければ候補から探す。無ければ None 🟡"""
    for raw in (os.environ.get(FONT_ENV), *CJK_FONT_CANDIDATES):
        if raw and Path(raw).is_file():
            return Path(raw)
    return None


@dataclass(frozen=True)
class VideoStyle:
    """【機能概要】: 文字の描き方。lang は表示言語 (ja / en)、font_path が None なら Pillow の既定フォント 🔵"""

    font_path: Path | None
    lang: str


def make_style(font_path: str | Path | None = None, lang: str | None = None) -> VideoStyle:
    """【機能概要】: 表示のスタイルを決める。lang 省略時は、日本語フォントがあれば ja、無ければ en 🔵

    lang="ja" なのに日本語フォントが無い・font_path が存在しない・lang が ja/en 以外は ValueError。
    """
    if lang not in (None, "ja", "en"):
        raise ValueError(f"lang は ja か en で指定してください: {lang!r}")
    if font_path is not None and not Path(font_path).is_file():
        raise ValueError(f"フォントが見つかりません: {font_path}")
    path = Path(font_path) if font_path is not None else None
    if lang == "en":
        return VideoStyle(path, "en")
    path = path or find_cjk_font()
    if path is None:
        if lang == "ja":
            raise ValueError(f"日本語フォントが見つかりません。--font か環境変数 {FONT_ENV} で指定してください")
        return VideoStyle(None, "en")
    return VideoStyle(path, "ja")


def validate_speed(speed: object, fps: int) -> int:
    """【機能概要】: 倍速 (1 以上の整数で fps の約数) を検査する。間引きの間隔を揃えるため 🔵"""
    if isinstance(speed, bool) or not isinstance(speed, int) or speed < 1 or fps % speed != 0:
        raise ValueError(f"speed は fps={fps} の約数 (1, 2, 3, 5, 6, 10, ...) で指定してください: {speed!r}")
    return speed


def speed_label(speed: int, lang: str) -> str | None:
    """【機能概要】: 倍速の表示文字 (2 倍速 -> "2倍速" / "2x speed")。実時間 (1) は None 🔵"""
    if speed <= 1:
        return None
    return f"{speed}倍速" if lang == "ja" else f"{speed}x speed"


def stage_label(stage: str, lang: str) -> str:
    """【機能概要】: 段階 (task / detect / execute / result) の表示名。未知は ValueError 🔵"""
    if stage not in STAGES:
        raise ValueError(f"未知の段階です: {stage!r} (使えるもの: {STAGES})")
    return _LABELS[lang][stage]


def result_label(success: bool, failure: str | None, lang: str) -> str:
    """【機能概要】: 判定の表示 (成功 / 失敗 (種類)) 🔵"""
    base = _LABELS[lang]["ok" if success else "ng"]
    return base if success or not failure else f"{base} ({failure})"


# --------------------------------------------------------------------------- クリップの構成
@dataclass(frozen=True)
class FrameSpec:
    """【機能概要】: 出力動画の 1 フレーム。source は実行フレームの番号 (None = 試行の最初の画像)、phase は実行中の区間名 🔵"""

    source: int | None
    stage: str
    phase: str | None


def _hold_frames(name: str, seconds: object, fps: int) -> int:
    if isinstance(seconds, bool) or not isinstance(seconds, int | float) or not math.isfinite(seconds) or seconds < 0:
        raise ValueError(f"{name} は 0 以上の有限の数で指定してください: {seconds!r}")
    return round(seconds * fps)


def plan_clip(
    step_labels: Sequence[str],
    *,
    fps: int = 30,
    hold_task_s: float = 1.5,
    hold_detect_s: float = 2.0,
    hold_result_s: float = 2.5,
) -> list[FrameSpec]:
    """【機能概要】: 1 試行ぶんの動画の構成を返す: 指示 (静止) -> 検出 (静止) -> 実行 (1 周期ごと) -> 判定 (静止) 🔵

    動かなかった試行 (step_labels が空) は 実行 の段階がなく、判定は最初の画像のまま。判定の間は最後の実行フレームを見せる。
    """
    if isinstance(fps, bool) or not isinstance(fps, int) or fps < 1:
        raise ValueError(f"fps は 1 以上の整数で指定してください: {fps!r}")
    n_task = _hold_frames("hold_task_s", hold_task_s, fps)
    n_detect = _hold_frames("hold_detect_s", hold_detect_s, fps)
    n_result = _hold_frames("hold_result_s", hold_result_s, fps)
    specs = [FrameSpec(None, "task", None)] * n_task + [FrameSpec(None, "detect", None)] * n_detect
    specs += [FrameSpec(i, "execute", label) for i, label in enumerate(step_labels)]
    last = len(step_labels) - 1 if step_labels else None
    return specs + [FrameSpec(last, "result", None)] * n_result


def clip_duration_s(n_frames: int, fps: int) -> float:
    """【機能概要】: フレーム数と fps から動画の長さ [s]。fps が 1 未満なら ValueError 🔵"""
    if isinstance(fps, bool) or not isinstance(fps, int) or fps < 1:
        raise ValueError(f"fps は 1 以上の整数で指定してください: {fps!r}")
    return n_frames / fps


# --------------------------------------------------------------------------- フレームの合成
@dataclass(frozen=True)
class Overlay:
    """【機能概要】: 1 フレームに重ねる情報。plan は検出の段階以降に枠を描く、result_text があれば判定の帯を出す 🔵"""

    instruction: str
    stage: str
    phase: str | None
    plan: PlanResult | None
    result_text: str | None
    result_ok: bool | None
    speed: int
    header_right: str | None


@lru_cache(maxsize=32)
def _font(path: Path | None, size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    return ImageFont.truetype(str(path), size) if path is not None else ImageFont.load_default(size=size)


def _fit(draw: ImageDraw.ImageDraw, text: str, font: ImageFont.ImageFont, max_w: float) -> str:
    """幅 max_w に収まるように、末尾を "..." で切る。"""
    if draw.textlength(text, font=font) <= max_w:
        return text
    while text and draw.textlength(text + "...", font=font) > max_w:
        text = text[:-1]
    return text + "..." if text else ""


def _check_image(image: object) -> np.ndarray:
    if not isinstance(image, np.ndarray) or image.dtype != np.uint8:
        raise ValueError("image は uint8 の numpy 配列で指定してください")
    if image.ndim != 3 or image.shape[2] != 3 or image.shape[0] < 1 or image.shape[1] < 1:
        raise ValueError(f"image は (H, W, 3) で指定してください: shape={getattr(image, 'shape', None)}")
    return image


def _draw_header(d: ImageDraw.ImageDraw, w: int, fs: int, font: ImageFont.ImageFont, ov: Overlay, style: VideoStyle) -> None:
    line = round(fs * 1.3)
    d.rectangle([0, 0, w, 2 * line + 6], fill=_HEADER_BG)
    right = ov.header_right or ""
    right_w = d.textlength(right, font=font) + 12 if right else 0
    prefix = "指示: " if style.lang == "ja" else "Task: "
    d.text((6, 3), _fit(d, prefix + ov.instruction, font, w - 12 - right_w), fill=_TEXT, font=font)
    if right:
        d.text((w - right_w + 6, 3), right, fill=_DIM, font=font)
    x = 6
    for i, stage in enumerate(STAGES):
        if i:
            d.text((x, line + 3), ">", fill=_DIM, font=font)
            x += d.textlength("> ", font=font)
        name = stage_label(stage, style.lang)
        d.text((x, line + 3), name, fill=_ACTIVE if stage == ov.stage else _DIM, font=font)
        x += d.textlength(name + " ", font=font)


def _draw_bar(d: ImageDraw.ImageDraw, w: int, h: int, fs: int, font: ImageFont.ImageFont, ov: Overlay, style: VideoStyle) -> None:
    bar = round(fs * 1.5)
    d.rectangle([0, h - bar, w, h], fill=_BAR_BG)
    if ov.phase:
        d.text((6, h - bar + 3), _fit(d, f"{stage_label('execute', style.lang)}: {ov.phase}", font, w / 2 - 12), fill=_TEXT, font=font)
    label = speed_label(ov.speed, style.lang)
    if label:
        tw = d.textlength(label, font=font)
        d.rounded_rectangle([w - tw - 20, h - bar + 2, w - 4, h - 2], radius=6, fill=_BADGE)
        d.text((w - tw - 12, h - bar + 3), label, fill=(0, 0, 0), font=font)


def _draw_banner(d: ImageDraw.ImageDraw, w: int, h: int, fs: int, style: VideoStyle, ov: Overlay) -> None:
    bw, bh = round(w * 0.6), round(fs * 3.4)
    x0, y0 = (w - bw) // 2, round(h * 0.70) - bh // 2
    d.rectangle([x0, y0, x0 + bw, y0 + bh], fill=(*(_OK if ov.result_ok else _NG), _BANNER_ALPHA))
    big = _font(style.font_path, round(fs * 1.6))
    text = _fit(d, ov.result_text or "", big, bw - 16)
    tw = d.textlength(text, font=big)
    d.text((x0 + (bw - tw) / 2, y0 + (bh - fs * 1.6) / 2 - 2), text, fill=_TEXT, font=big)


# 🟡 Intent: 枠 (draw_plan) -> 半透明の帯と文字、の順。帯は半透明にして、ロボットや箱が帯の下でも見えるようにする。
def compose_frame(image: np.ndarray, overlay: Overlay, style: VideoStyle) -> np.ndarray:
    """【機能概要】: 画像に 検出枠・指示・段階・実行中の区間・倍速・判定を重ねた新しい画像を返す。元の画像は変えない 🔵

    不正な image は ValueError。小さな画像でも、長い指示でも、はみ出さず落ちない (文字は切る)。
    """
    _check_image(image)
    stage_label(overlay.stage, style.lang)  # 未知の段階を弾く
    frame = draw_plan(image, overlay.plan, labels=True) if overlay.plan is not None and overlay.stage != "task" else image.copy()
    h, w = frame.shape[:2]
    fs = max(11, h // 20)
    font = _font(style.font_path, fs)
    layer = Image.new("RGBA", (w, h), (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)
    _draw_header(d, w, fs, font, overlay, style)
    _draw_bar(d, w, h, fs, font, overlay, style)
    if overlay.result_text:
        _draw_banner(d, w, h, fs, style, overlay)
    out = Image.alpha_composite(Image.fromarray(frame).convert("RGBA"), layer)
    return np.asarray(out.convert("RGB")).copy()


def compose_summary(size: tuple[int, int], lines: Sequence[str], style: VideoStyle) -> np.ndarray:
    """【機能概要】: 最後に出す集計のカード (暗い背景に、行を中央揃えで) を作る。size は (高さ, 幅) 🔵"""
    h, w = size
    if not (isinstance(h, int) and isinstance(w, int) and h > 0 and w > 0):
        raise ValueError(f"size は正の整数の (高さ, 幅) で指定してください: {size!r}")
    img = Image.new("RGB", (w, h), (24, 28, 36))
    d = ImageDraw.Draw(img)
    fs = max(11, h // 16)
    font = _font(style.font_path, fs)
    line_h = round(fs * 1.5)
    y = (h - line_h * len(lines)) // 2
    for i, text in enumerate(lines):
        text = _fit(d, text, font, w - 20)
        d.text(((w - d.textlength(text, font=font)) / 2, y + i * line_h), text, fill=_ACTIVE if i == 0 else _TEXT, font=font)
    return np.asarray(img).copy()

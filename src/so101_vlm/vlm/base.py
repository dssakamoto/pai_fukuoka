"""VlmClient の共通部分 (TASK-0018): 共通プロンプト、再試行、キャッシュ。

🔵 design.md 4.3 / FR-02a / FR-03a。プロバイダ固有の部分は「トランスポート」 (png, prompt, timeout_s) -> 応答テキスト
として差し込む (TASK-0019〜0022)。トランスポートは、タイムアウト・レート制限・通信エラーを VlmTransientError に
包んで投げる。それ以外の例外 (認証エラーなど) は再試行せず、そのまま上に出す。
"""

import time
from collections.abc import Callable
from typing import Protocol

import imageio.v3 as iio
import numpy as np

from so101_vlm.vlm.cache import VlmCache, make_key
from so101_vlm.vlm.schema import PlanError, PlanResult, parse_plan

# 🟡 プロンプトを変えたらキャッシュを無効にするための版数
PROMPT_VERSION = "1"
# 🟡 再試行の待ち [s]: 1, 2, 4, ... (指数バックオフ、設計書 4.3)
BACKOFF_BASE_S = 1.0

Transport = Callable[[bytes, str, float], str]


class VlmTransientError(Exception):
    """【機能概要】: 再試行してよい失敗 (タイムアウト、レート制限、通信エラー) 🔵"""


class VlmCallError(PlanError):
    """【機能概要】: 再試行しても VLM を呼べなかった。PlanError なので、ロボットは動かさない (FR-03a) 🔵"""


class VlmClient(Protocol):
    """【機能概要】: 画像と指示から、掴む対象と置き場所を返す (design.md 4.3) 🔵"""

    def plan(self, image: np.ndarray, instruction: str) -> PlanResult: ...


_FORMAT_TEXT = {
    "xyxy_px": (
        "Each box is [x1, y1, x2, y2] in absolute pixel coordinates of the image I send "
        "(origin at the top-left, x to the right, y downward). The image is {w} pixels wide and "
        "{h} pixels tall."
    ),
    "yxyx_norm1000": (
        "Each box is [ymin, xmin, ymax, xmax] (y first), normalized to the range 0-1000 relative "
        "to the image height and width (origin at the top-left)."
    ),
}


# 🔵 Intent: 設計書 4.3 の共通プロンプト。座標形式をプロバイダごとに明示する (誤解のリスク対策)。
def build_prompt(instruction: str, coord_format: str, width: int, height: int) -> str:
    """【機能概要】: 画像と指示から、掴む対象と置き場所の枠を JSON だけで返させるプロンプト 🔵"""
    fmt = _FORMAT_TEXT[coord_format].format(w=width, h=height)
    return (
        "You control a robot arm that does pick-and-place. Look at the image and follow the "
        "instruction. Find the object to pick up and the place to put it.\n"
        f"Instruction: {instruction}\n\n"
        f"{fmt}\n\n"
        "Reply with JSON only (no explanation, no code fence) in this form:\n"
        '{"pick": {"label": "<short name>", "box": [n, n, n, n], "confidence": <0 to 1>}, '
        '"place": {"label": "<short name>", "box": [n, n, n, n], "confidence": <0 to 1>}}\n'
        "If the object to pick does not exist in the image, reply with "
        '{"pick": null, "place": null, "error": "<why>"} instead.'
    )


def _encode_png(image: object) -> tuple[bytes, int, int]:
    if not isinstance(image, np.ndarray) or image.dtype != np.uint8:
        raise ValueError("image は uint8 の numpy 配列で指定してください")
    if image.ndim != 3 or image.shape[2] != 3 or image.shape[0] < 1 or image.shape[1] < 1:
        raise ValueError(f"image は (H, W, 3) で指定してください: shape={image.shape}")
    return iio.imwrite("<bytes>", image, extension=".png"), image.shape[1], image.shape[0]


class BaseVlmClient:
    """【機能概要】: プロバイダに依らない plan() の流れ。キャッシュ → 呼び出し (再試行) → 検証 🔵

    検証に失敗した応答 (PlanError) は再試行も保存もしない。成功した応答のテキストだけを保存する。
    """

    def __init__(
        self,
        *,
        provider: str,
        model: str,
        coord_format: str,
        transport: Transport,
        cache: VlmCache,
        timeout_s: float,
        max_retries: int,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self.provider = provider
        self.model = model
        self._coord_format = coord_format
        self._transport = transport
        self._cache = cache
        self._timeout_s = timeout_s
        self._max_retries = max_retries
        self._sleep = sleep

    def _call_with_retry(self, png: bytes, prompt: str) -> str:
        attempts = self._max_retries + 1
        last: Exception | None = None
        for i in range(attempts):
            try:
                return self._transport(png, prompt, self._timeout_s)
            except VlmTransientError as exc:
                last = exc
                if i < attempts - 1:
                    self._sleep(BACKOFF_BASE_S * 2**i)
        raise VlmCallError(f"VLM の呼び出しが {attempts} 回失敗しました: {last}") from last

    def plan(self, image: np.ndarray, instruction: str) -> PlanResult:
        """【機能概要】: 画像と指示から PlanResult を返す。不正な入力は ValueError、使えない応答は PlanError 🔵"""
        if not isinstance(instruction, str) or not instruction.strip():
            raise ValueError("instruction は空でない文字列で指定してください")
        png, width, height = _encode_png(image)
        tag = f"{PROMPT_VERSION}|{self._coord_format}|{width}x{height}"
        key = make_key(image, instruction, self.provider, self.model, tag)
        cached = self._cache.get(key)
        if cached is not None:
            try:
                return parse_plan(cached, self._coord_format, width, height)
            except PlanError:
                pass  # 壊れたキャッシュは無かったことにして呼び直す
        prompt = build_prompt(instruction, self._coord_format, width, height)
        text = self._call_with_retry(png, prompt)
        result = parse_plan(text, self._coord_format, width, height)
        self._cache.put(key, text)
        return result

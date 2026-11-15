"""Gemini のトランスポート (TASK-0021)。vlm/base.py の BaseVlmClient に差し込む。

🔵 公式 SDK (google-genai) の models.generate_content。画像は inline の PNG (Part.from_bytes)。座標は
[ymin, xmin, ymax, xmax] を 0〜1000 に正規化した値 (coord_format: yxyx_norm1000。ピクセルへの変換は schema.box_to_pixels が
元の画像の大きさで行う)。構造化出力は response_mime_type + response_json_schema。
実測 (2026-11-15、偽サーバ): この SDK はタイムアウト・接続エラーを google.genai.errors ではなく httpx2 の生の例外
(ReadTimeout / ConnectTimeout) で出す。429 は ClientError (4xx 側)、5xx は ServerError。これらを一時的な失敗に直す。
SDK の自動再試行は切る (HttpRetryOptions(attempts=1))。再試行は BaseVlmClient が管理する。タイムアウトは HttpOptions.timeout [ms]。
"""

import importlib

from so101_vlm.vlm.base import Transport, VlmTransientError
from so101_vlm.vlm.config import ProviderConfig
from so101_vlm.vlm.schema import PlanError
from so101_vlm.vlm.transport_common import DEFAULT_MAX_TOKENS, PLAN_SCHEMA

_BLOCKING_FINISH = ("SAFETY", "RECITATION", "BLOCKLIST", "PROHIBITED_CONTENT", "SPII", "IMAGE_SAFETY", "IMAGE_PROHIBITED_CONTENT")


def _transport_error_types() -> tuple[type[BaseException], ...]:
    """SDK が裏で使う HTTP ライブラリ (httpx2 / httpx) の通信エラーの型。どちらが使われても拾う。"""
    found = []
    for name in ("httpx2", "httpx"):
        try:
            found.append(importlib.import_module(name).TransportError)
        except (ImportError, AttributeError):
            continue
    return tuple(found)


def _name(value: object) -> str:
    return str(getattr(value, "name", value))


def make_transport(pc: ProviderConfig, api_key: str | None) -> Transport:
    """【機能概要】: (png, prompt, timeout_s) -> 応答のテキスト、の関数を返す。options: base_url / max_output_tokens 🔵"""
    from google import genai
    from google.genai import errors, types

    client = genai.Client(
        api_key=api_key,
        http_options=types.HttpOptions(base_url=pc.options.get("base_url"), retry_options=types.HttpRetryOptions(attempts=1)),
    )
    max_output_tokens = int(pc.options.get("max_output_tokens", DEFAULT_MAX_TOKENS))
    transport_errors = _transport_error_types()

    def call(png: bytes, prompt: str, timeout_s: float) -> str:
        config = types.GenerateContentConfig(
            response_mime_type="application/json", response_json_schema=PLAN_SCHEMA,
            max_output_tokens=max_output_tokens, http_options=types.HttpOptions(timeout=max(1, round(timeout_s * 1000))),
        )  # fmt: skip
        contents = [types.Part.from_bytes(data=png, mime_type="image/png"), prompt]
        try:
            response = client.models.generate_content(model=pc.model, contents=contents, config=config)
        except transport_errors as exc:
            raise VlmTransientError(f"{type(exc).__name__}: {exc}") from exc
        except errors.ServerError as exc:
            raise VlmTransientError(f"ServerError {exc.code}: {exc}") from exc
        except errors.ClientError as exc:
            if exc.code in (408, 429):
                raise VlmTransientError(f"ClientError {exc.code}: {exc}") from exc
            raise
        feedback = getattr(response, "prompt_feedback", None)
        if feedback is not None and getattr(feedback, "block_reason", None):
            raise PlanError(f"プロンプトがブロックされました: {_name(feedback.block_reason)}")
        candidates = response.candidates or []
        if not candidates:
            raise PlanError("Gemini の応答に候補がありません")
        finish = _name(candidates[0].finish_reason) if candidates[0].finish_reason is not None else "STOP"
        if finish == "MAX_TOKENS":
            raise PlanError(f"応答が max_output_tokens ({max_output_tokens}) で途中で切れました")
        if finish in _BLOCKING_FINISH:
            raise PlanError(f"応答がブロックされました: {finish}")
        return response.text or ""

    return call

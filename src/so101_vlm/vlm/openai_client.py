"""OpenAI のトランスポート (TASK-0020)。vlm/base.py の BaseVlmClient に差し込む。

🔵 公式 SDK (openai) の Responses API。画像は data URL (base64 PNG) の input_image。座標は絶対ピクセル
[x1, y1, x2, y2] (coord_format: xyxy_px)。公式ドキュメントに bbox の規約はなく、座標が要る用途には detail="original"
(画像を縮小せずに送る) を勧めているので、既定を original にする (options.image_detail で変える。original 非対応のモデルは high)。
🔴 返る座標が本当に送った画像の絶対ピクセルかは、実測が要る (tests/vlm/test_live_apis.py と scripts/visualize_detection.py)。
構造化出力は text.format (json_schema, strict)。SDK の自動再試行は切る (再試行は BaseVlmClient が管理する)。
base_url を設定で変えられる (オプション。OpenAI 互換のサーバ向け)。
"""

from so101_vlm.vlm.base import Transport, VlmTransientError
from so101_vlm.vlm.config import ProviderConfig
from so101_vlm.vlm.schema import PlanError
from so101_vlm.vlm.transport_common import add_usage, new_usage, DEFAULT_MAX_TOKENS, PLAN_SCHEMA, is_transient_status, png_to_b64


def make_transport(pc: ProviderConfig, api_key: str | None) -> Transport:
    """【機能概要】: (png, prompt, timeout_s) -> 応答のテキスト、の関数を返す。options: base_url / image_detail / max_output_tokens 🔵"""
    import openai

    client = openai.OpenAI(api_key=api_key, base_url=pc.options.get("base_url"), max_retries=0)
    detail = pc.options.get("image_detail", "original")
    max_output_tokens = int(pc.options.get("max_output_tokens", DEFAULT_MAX_TOKENS))
    text_format = {"format": {"type": "json_schema", "name": "pick_place", "schema": PLAN_SCHEMA, "strict": True}}

    usage = new_usage()

    def call(png: bytes, prompt: str, timeout_s: float) -> str:
        content = [
            {"type": "input_text", "text": prompt},
            {"type": "input_image", "image_url": f"data:image/png;base64,{png_to_b64(png)}", "detail": detail},
        ]
        try:
            response = client.responses.create(
                model=pc.model, input=[{"role": "user", "content": content}], text=text_format,
                max_output_tokens=max_output_tokens, timeout=timeout_s,
            )  # fmt: skip
        except openai.RateLimitError as exc:
            # 🔵 実 API の観測: クレジット切れも HTTP 429 (code: insufficient_quota) で返る。待っても直らないので再試行しない
            if "insufficient_quota" in (getattr(exc, "code", None), getattr(exc, "type", None)):
                raise
            raise VlmTransientError(f"{type(exc).__name__}: {exc}") from exc
        except (openai.APITimeoutError, openai.APIConnectionError) as exc:
            raise VlmTransientError(f"{type(exc).__name__}: {exc}") from exc
        except openai.APIStatusError as exc:
            if is_transient_status(exc.status_code):
                raise VlmTransientError(f"{type(exc).__name__}: {exc}") from exc
            raise
        add_usage(usage, getattr(response.usage, 'input_tokens', None), getattr(response.usage, 'output_tokens', None))  # 課金は、応答が使えなくても発生する
        for item in response.output or []:
            if getattr(item, "type", None) == "message":
                for part in item.content:
                    if getattr(part, "type", None) == "refusal":
                        raise PlanError(f"OpenAI が応答を拒否しました: {getattr(part, 'refusal', '')}")
        if getattr(response, "status", "completed") == "incomplete":
            raise PlanError(f"応答が途中で終わりました: {getattr(response, 'incomplete_details', None)}")
        return response.output_text or ""

    call.usage = usage  # type: ignore[attr-defined]  # BaseVlmClient.usage が読む
    return call

"""Anthropic (Claude) のトランスポート (TASK-0019)。vlm/base.py の BaseVlmClient に差し込む。

🔵 公式 SDK (anthropic) の Messages API。画像 (base64 PNG) を先に、指示を後に送る (公式の推奨)。座標は絶対ピクセル
[x1, y1, x2, y2] (coord_format: xyxy_px)。送る画像は 640x480 で、Claude の縮小条件 (長辺 1568 px 超) に当たらないので、
返る座標は元の画像の座標のまま。構造化出力は output_config.format (json_schema)。thinking / temperature は指定しない
(Haiku 5.5 は adaptive thinking が既定で、sampling 引数は 400 になる)。
SDK の自動再試行は切る (max_retries=0)。再試行は BaseVlmClient が一括して管理する (回数がずれて増えないように)。
"""

from so101_vlm.vlm.base import Transport, VlmTransientError
from so101_vlm.vlm.config import ProviderConfig
from so101_vlm.vlm.schema import PlanError
from so101_vlm.vlm.transport_common import DEFAULT_MAX_TOKENS, PLAN_SCHEMA, is_transient_status, png_to_b64


def make_transport(pc: ProviderConfig, api_key: str | None) -> Transport:
    """【機能概要】: (png, prompt, timeout_s) -> 応答のテキスト、の関数を返す。options: base_url / max_tokens / effort 🔵"""
    import anthropic

    client = anthropic.Anthropic(api_key=api_key, base_url=pc.options.get("base_url"), max_retries=0)
    max_tokens = int(pc.options.get("max_tokens", DEFAULT_MAX_TOKENS))
    output_config: dict = {"format": {"type": "json_schema", "schema": PLAN_SCHEMA}}
    if pc.options.get("effort"):
        output_config["effort"] = pc.options["effort"]

    # 🔵 Intent: タイムアウト・接続・レート制限・5xx は再試行できる失敗。認証・不正なリクエストは再試行せず、SDK の例外のまま出す。
    def call(png: bytes, prompt: str, timeout_s: float) -> str:
        content = [
            {"type": "image", "source": {"type": "base64", "media_type": "image/png", "data": png_to_b64(png)}},
            {"type": "text", "text": prompt},
        ]
        try:
            response = client.messages.create(
                model=pc.model, max_tokens=max_tokens, messages=[{"role": "user", "content": content}],
                output_config=output_config, timeout=timeout_s,
            )  # fmt: skip
        except (anthropic.APITimeoutError, anthropic.APIConnectionError, anthropic.RateLimitError) as exc:
            raise VlmTransientError(f"{type(exc).__name__}: {exc}") from exc
        except anthropic.APIStatusError as exc:
            if is_transient_status(exc.status_code):
                raise VlmTransientError(f"{type(exc).__name__}: {exc}") from exc
            raise
        if response.stop_reason == "refusal":
            raise PlanError(f"Claude が応答を拒否しました: {getattr(response.stop_details, 'category', None)}")
        if response.stop_reason == "max_tokens":
            raise PlanError(f"応答が max_tokens ({max_tokens}) で途中で切れました")
        return "".join(block.text for block in response.content if block.type == "text")

    return call

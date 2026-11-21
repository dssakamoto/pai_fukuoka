"""Ollama (Gemma) のトランスポート (TASK-0022)。vlm/base.py の BaseVlmClient に差し込む。

🔵 Ollama のネイティブ API (POST {base_url}/api/chat)。画像は messages[].images に base64 (PNG)、構造化出力は format に
JSON スキーマ、stream は false。座標は Gemma 4 の [ymin, xmin, ymax, xmax] を 0〜1000 に正規化した値 (coord_format: yxyx_norm1000)。
OpenAI 互換の response_format は json_schema の形が文書化されていない (docs/notes/vlm-providers.md) ので、ネイティブ API を使う。
追加の依存は要らない (標準ライブラリの urllib)。キーは不要。ローカルのサーバなので、プロキシ設定は使わない。
options: base_url (既定 http://localhost:11434) / num_ctx (KV キャッシュ節約のため小さく) / max_output_tokens (num_predict)。
モデルが未取得 (404) は待っても直らないので再試行しない。起動していない (接続拒否) は一時的な失敗として扱い、ヒントを添える。
"""

import base64
import json
import socket
import urllib.error
import urllib.request

from so101_vlm.vlm.base import Transport, VlmTransientError
from so101_vlm.vlm.config import ProviderConfig
from so101_vlm.vlm.schema import PlanError
from so101_vlm.vlm.transport_common import DEFAULT_MAX_TOKENS, PLAN_SCHEMA, add_usage, is_transient_status, new_usage

DEFAULT_BASE_URL = "http://localhost:11434"


def make_transport(pc: ProviderConfig, api_key: str | None) -> Transport:
    """【機能概要】: (png, prompt, timeout_s) -> 応答のテキスト、の関数を返す。api_key は使わない 🔵"""
    base_url = str(pc.options.get("base_url") or DEFAULT_BASE_URL).rstrip("/")
    num_ctx = pc.options.get("num_ctx")
    num_predict = int(pc.options.get("max_output_tokens", DEFAULT_MAX_TOKENS))
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))  # localhost への通信をプロキシに回さない
    usage = new_usage()

    def call(png: bytes, prompt: str, timeout_s: float) -> str:
        options: dict[str, object] = {"temperature": 0, "num_predict": num_predict}
        if num_ctx is not None:
            options["num_ctx"] = int(num_ctx)
        payload = {
            "model": pc.model, "stream": False, "format": PLAN_SCHEMA, "options": options,
            "messages": [{"role": "user", "content": prompt, "images": [base64.b64encode(png).decode("ascii")]}],
        }  # fmt: skip
        request = urllib.request.Request(
            f"{base_url}/api/chat", data=json.dumps(payload).encode(), headers={"content-type": "application/json"}, method="POST"
        )
        try:
            with opener.open(request, timeout=timeout_s) as response:
                data = json.loads(response.read())
        except urllib.error.HTTPError as exc:
            detail = _error_text(exc)
            if is_transient_status(exc.code):
                raise VlmTransientError(f"Ollama が HTTP {exc.code} を返しました: {detail}") from exc
            hint = f" `ollama pull {pc.model}` で取得してください。" if exc.code == 404 else ""
            raise PlanError(f"Ollama が HTTP {exc.code} を返しました: {detail}.{hint}") from exc
        except (TimeoutError, socket.timeout) as exc:
            raise VlmTransientError(f"Ollama の応答が {timeout_s} 秒以内に返りませんでした") from exc
        except urllib.error.URLError as exc:
            if isinstance(exc.reason, (TimeoutError, socket.timeout)):
                raise VlmTransientError(f"Ollama の応答が {timeout_s} 秒以内に返りませんでした") from exc
            raise VlmTransientError(f"ollama に接続できません ({base_url})。`ollama serve` で起動しているか確認してください: {exc.reason}") from exc
        except (ConnectionError, OSError) as exc:
            raise VlmTransientError(f"ollama との通信に失敗しました ({base_url}): {exc}") from exc
        except json.JSONDecodeError as exc:
            raise VlmTransientError(f"Ollama の応答が JSON ではありません: {exc}") from exc

        add_usage(usage, data.get("prompt_eval_count"), data.get("eval_count"))
        if data.get("done_reason") == "length":
            raise PlanError("応答が途中で終わりました (done_reason=length)。max_output_tokens を増やしてください")
        message = data.get("message") or {}
        return message.get("content") or ""

    call.usage = usage  # type: ignore[attr-defined]  # BaseVlmClient.usage が読む
    return call


def _error_text(exc: urllib.error.HTTPError) -> str:
    try:
        body = json.loads(exc.read())
        return str(body.get("error", body)) if isinstance(body, dict) else str(body)
    except Exception:  # noqa: BLE001  本文が読めなくても、ステータスは伝える
        return exc.reason if isinstance(exc.reason, str) else ""


def unavailable_reason(pc: ProviderConfig, timeout_s: float = 2.0) -> str | None:
    """【機能概要】: Ollama が使えない理由を返す (使えるなら None)。起動していない、またはモデルが未取得 (/api/tags で確認) 🔵"""
    base_url = str(pc.options.get("base_url") or DEFAULT_BASE_URL).rstrip("/")
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        with opener.open(f"{base_url}/api/tags", timeout=timeout_s) as response:
            names = {m.get("name") for m in json.loads(response.read()).get("models", [])}
    except Exception as exc:  # noqa: BLE001  通信できない = 使えない (理由に原因を残す)
        return f"ollama に接続できません ({base_url})。`ollama serve` で起動してください: {exc}"
    if pc.model not in names:
        return f"モデル {pc.model} が未取得です。`ollama pull {pc.model}` で取得してください"
    return None

"""VLM クライアントのテスト用の偽 API サーバ (TASK-0019〜0021)。tests/vlm/conftest.py が fixture を公開する。

SDK をモックせず、本物の SDK を、ローカルの HTTP サーバ (各社の応答の形を返す) に向けて動かす。
リクエストの形・ヘッダ・画像の添付・エラーの型・再試行の回数まで、SDK を通して確かめられる。
"""

import dataclasses
import json
import threading
import time
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from so101_vlm.vlm.config import ProviderConfig, VlmConfig, load_vlm_config


@dataclass
class Reply:
    """偽サーバの 1 回分の返事。status と body (JSON)、delay [s] だけ待ってから返す。"""

    status: int = 200
    body: dict | None = None
    delay: float = 0.0


@dataclass
class Recorded:
    path: str
    headers: dict
    body: dict


@dataclass
class FakeApi:
    url: str
    requests: list[Recorded] = field(default_factory=list)
    _queue: list[Reply] = field(default_factory=list)
    _lock: threading.Lock = field(default_factory=threading.Lock)

    def respond(self, *replies: Reply) -> None:
        """返事の列を設定する。尽きたら、最後の返事を繰り返す。"""
        with self._lock:
            self._queue = list(replies)
            self.requests.clear()

    def _next(self) -> Reply:
        with self._lock:
            return self._queue.pop(0) if len(self._queue) > 1 else self._queue[0]


@pytest.fixture
def fake_api():
    api = FakeApi(url="")

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_POST(self):
            raw = self.rfile.read(int(self.headers.get("content-length", 0)))
            with api._lock:
                api.requests.append(Recorded(self.path, {k.lower(): v for k, v in self.headers.items()}, json.loads(raw or b"{}")))
            reply = api._next()
            if reply.delay:
                time.sleep(reply.delay)
            body = json.dumps(reply.body if reply.body is not None else {"error": {"message": "fake error", "code": reply.status}}).encode()
            try:
                self.send_response(reply.status)
                self.send_header("content-type", "application/json")
                self.send_header("content-length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            except (ConnectionError, OSError):
                pass  # クライアントがタイムアウトで切った

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    api.url = f"http://127.0.0.1:{server.server_address[1]}"
    api.respond(Reply())
    yield api
    server.shutdown()
    server.server_close()


def config_for(api: FakeApi, *, timeout_s: float = 5.0, max_retries: int = 2) -> VlmConfig:
    """configs/vlm.yaml のプロバイダ設定の base_url を偽サーバに向けた VlmConfig。"""
    cfg = load_vlm_config()
    providers = {
        name: dataclasses.replace(pc, options={**pc.options, "base_url": api.url}) for name, pc in cfg.providers.items()
    }  # type: dict[str, ProviderConfig]
    return dataclasses.replace(cfg, providers=providers, timeout_s=timeout_s, max_retries=max_retries)

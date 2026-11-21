"""OllamaClient (Gemma) のテスト (TASK-0022, Red)。

Ollama のネイティブ API (POST /api/chat) の形を返す偽サーバに向けて動かす (fake_api.py)。キー・ネットワーク・Ollama 本体は要らない。
実 Ollama での検出は tests/vlm/test_live_apis.py (起動していなければスキップ)。
"""

import base64
import json

import numpy as np
import pytest

from so101_vlm.vlm import create_client
from so101_vlm.vlm.base import VlmCallError
from so101_vlm.vlm.config import load_vlm_config
from so101_vlm.vlm.schema import PlanError

from fake_api import Reply, config_for  # noqa: E402

IMG = np.zeros((480, 640, 3), np.uint8)
INSTRUCTION = "put the red block into the box"
NORM_PLAN = {  # Gemma 4: [ymin, xmin, ymax, xmax] を 0〜1000 に正規化 (640x480 で pick = (402, 210, 438, 252))
    "pick": {"label": "red block", "box": [437, 628, 525, 684], "confidence": 0.9},
    "place": {"label": "box", "box": [625, 469, 833, 625], "confidence": None},
    "error": None,
}


def ollama_ok(text, *, done_reason="stop"):
    return {
        "model": "gemma4:e4b", "created_at": "2026-11-21T00:00:00Z", "done": True, "done_reason": done_reason,
        "message": {"role": "assistant", "content": text}, "prompt_eval_count": 12, "eval_count": 34,
    }  # fmt: skip


def _client(api, tmp_path, *, timeout_s=5.0, max_retries=2, sleeps=None):
    return create_client(
        config_for(api, timeout_s=timeout_s, max_retries=max_retries), "ollama", env={}, cache_dir=tmp_path / "cache",
        sleep=(sleeps.append if sleeps is not None else (lambda s: None)),
    )  # fmt: skip


def _ok(plan=None):
    return Reply(200, ollama_ok(json.dumps(plan or NORM_PLAN)))


def test_plan_returns_pixel_boxes(fake_api, tmp_path):
    fake_api.respond(_ok())
    r = _client(fake_api, tmp_path).plan(IMG, INSTRUCTION)
    assert r.pick.label == "red block" and r.pick.bbox_px == (402, 210, 438, 252) and r.pick.confidence == 0.9
    assert r.place.label == "box" and len(fake_api.requests) == 1


def test_no_api_key_is_required(fake_api, tmp_path):
    _client(fake_api, tmp_path)  # env={} でも作れる (キー不要)
    assert load_vlm_config().providers["ollama"].api_key_env is None


def test_request_shape(fake_api, tmp_path):
    fake_api.respond(_ok())
    _client(fake_api, tmp_path).plan(IMG, INSTRUCTION)
    req = fake_api.requests[0]
    assert req.path == "/api/chat" and "authorization" not in req.headers
    body = req.body
    cfg = config_for(fake_api).providers["ollama"]
    assert body["model"] == cfg.model and body["stream"] is False
    msg = body["messages"][0]
    assert msg["role"] == "user" and INSTRUCTION in msg["content"]
    assert len(msg["images"]) == 1 and base64.b64decode(msg["images"][0])[:8] == b"\x89PNG\r\n\x1a\n"
    assert body["format"]["type"] == "object" and "pick" in body["format"]["properties"]  # 構造化出力 (JSON スキーマ)
    assert body["options"]["num_ctx"] == cfg.options["num_ctx"] and body["options"]["temperature"] == 0


def test_the_response_is_cached(fake_api, tmp_path):
    fake_api.respond(_ok())
    c = _client(fake_api, tmp_path)
    c.plan(IMG, INSTRUCTION)
    c.plan(IMG, INSTRUCTION)
    assert len(fake_api.requests) == 1


def test_token_usage_is_recorded(fake_api, tmp_path):
    fake_api.respond(_ok())
    c = _client(fake_api, tmp_path)
    c.plan(IMG, INSTRUCTION)
    assert c.usage["calls"] == 1 and c.usage["input_tokens"] == 12 and c.usage["output_tokens"] == 34


def test_not_found_target_is_reported_without_boxes(fake_api, tmp_path):
    fake_api.respond(_ok({"pick": None, "place": None, "error": "no purple block in the image"}))
    with pytest.raises(PlanError, match="purple"):
        _client(fake_api, tmp_path).plan(IMG, "put the purple block into the box")


def test_server_errors_are_retried(fake_api, tmp_path):
    sleeps = []
    fake_api.respond(Reply(503), Reply(503), _ok())
    r = _client(fake_api, tmp_path, sleeps=sleeps).plan(IMG, INSTRUCTION)
    assert r.pick is not None and len(fake_api.requests) == 3 and len(sleeps) == 2


def test_model_not_pulled_is_not_retried(fake_api, tmp_path):
    fake_api.respond(Reply(404, {"error": "model 'gemma4:e4b' not found, try pulling it first"}))
    with pytest.raises(PlanError, match="not found"):
        _client(fake_api, tmp_path).plan(IMG, INSTRUCTION)
    assert len(fake_api.requests) == 1  # 待っても直らない


def test_timeout_is_retried_then_fails(fake_api, tmp_path):
    fake_api.respond(Reply(200, ollama_ok("{}"), delay=1.5))
    with pytest.raises(VlmCallError):
        _client(fake_api, tmp_path, timeout_s=0.3, max_retries=1).plan(IMG, INSTRUCTION)
    assert len(fake_api.requests) == 2


def test_connection_refused_is_reported_with_a_hint(tmp_path):
    import dataclasses
    import socket

    with socket.socket() as s:  # 空いているポートを取って閉じる = 誰も待ち受けていないポート (OS により即拒否される)
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    cfg = load_vlm_config()
    pc = dataclasses.replace(cfg.providers["ollama"], options={**cfg.providers["ollama"].options, "base_url": f"http://127.0.0.1:{port}"})
    cfg = dataclasses.replace(cfg, providers={**cfg.providers, "ollama": pc}, max_retries=0, timeout_s=15)
    c = create_client(cfg, "ollama", env={}, cache_dir=tmp_path / "cache", sleep=lambda s: None)
    with pytest.raises(VlmCallError, match="ollama"):
        c.plan(IMG, INSTRUCTION)


def test_truncated_output_is_a_plan_error(fake_api, tmp_path):
    fake_api.respond(Reply(200, ollama_ok('{"pick": {"label"', done_reason="length")))
    with pytest.raises(PlanError, match="途中"):
        _client(fake_api, tmp_path).plan(IMG, INSTRUCTION)


# --- 評価の前の可用性チェック (起動していない・モデル未取得は「未評価」にする。AC-04) ---------------------------------
def _pc(api):
    return config_for(api).providers["ollama"]


def test_unavailable_reason_is_none_when_the_model_is_pulled(tmp_path):
    import http.server
    import threading

    from so101_vlm.vlm.ollama_client import unavailable_reason

    def serve(models):
        class H(http.server.BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_GET(self):
                body = json.dumps({"models": [{"name": m} for m in models]}).encode()
                self.send_response(200)
                self.send_header("content-length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        srv = http.server.ThreadingHTTPServer(("127.0.0.1", 0), H)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        return srv

    import dataclasses

    base = load_vlm_config().providers["ollama"]
    for models, expect in ((["gemma4:e4b"], None), (["llama3:8b"], "ollama pull")):
        srv = serve(models)
        try:
            pc = dataclasses.replace(base, options={**base.options, "base_url": f"http://127.0.0.1:{srv.server_address[1]}"})
            reason = unavailable_reason(pc)
            assert reason is None if expect is None else expect in reason
        finally:
            srv.shutdown()
            srv.server_close()


def test_unavailable_reason_when_the_server_is_down():
    import dataclasses
    import socket

    from so101_vlm.vlm.ollama_client import unavailable_reason

    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    base = load_vlm_config().providers["ollama"]
    pc = dataclasses.replace(base, options={**base.options, "base_url": f"http://127.0.0.1:{port}"})
    assert "ollama serve" in unavailable_reason(pc, timeout_s=15)

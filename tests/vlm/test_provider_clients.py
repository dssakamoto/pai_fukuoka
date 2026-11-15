"""Anthropic / OpenAI / Gemini のクライアントのテスト (TASK-0019〜0021, Red)。

本物の SDK を、各社の応答の形を返す偽サーバに向けて動かす (conftest.py)。API キー・ネットワークは要らない。
"""

import base64
import dataclasses

import numpy as np
import pytest

from so101_vlm.vlm import MissingApiKeyError, create_client
from so101_vlm.vlm.base import VlmCallError
from so101_vlm.vlm.schema import PlanError, PlanResult

from fake_api import Reply, config_for  # noqa: E402  (tests/vlm/fake_api.py)

IMG = np.zeros((480, 640, 3), np.uint8)
INSTRUCTION = "put the red block into the box"
PX_PLAN = {
    "pick": {"label": "red block", "box": [402, 210, 438, 252], "confidence": 0.9},
    "place": {"label": "box", "box": [300, 300, 400, 400], "confidence": None},
    "error": None,
}
NORM_PLAN = {  # Gemini: [ymin, xmin, ymax, xmax] を 0〜1000 に正規化 (640x480 で pick = (402, 210, 438, 252))
    "pick": {"label": "red block", "box": [437, 628, 525, 684], "confidence": 0.9},
    "place": {"label": "box", "box": [625, 469, 833, 625], "confidence": None},
    "error": None,
}
NOT_FOUND = {"pick": None, "place": None, "error": "no purple block in the image"}


def _text(plan: dict) -> str:
    import json

    return json.dumps(plan)


# --- 各社の応答の形 -----------------------------------------------------------------------------
def anthropic_ok(text, stop="end_turn"):
    return {
        "id": "msg_1", "type": "message", "role": "assistant", "model": "claude-haiku-5-5",
        "content": [{"type": "text", "text": text}], "stop_reason": stop, "stop_sequence": None,
        "usage": {"input_tokens": 10, "output_tokens": 10},
    }  # fmt: skip


def openai_ok(text, *, status="completed", part=None):
    return {
        "id": "resp_1", "object": "response", "created_at": 0, "status": status, "model": "gpt-6-luna",
        "output": [{
            "type": "message", "id": "msg_1", "status": "completed", "role": "assistant",
            "content": [part or {"type": "output_text", "text": text, "annotations": []}],
        }],
        "parallel_tool_calls": True, "tool_choice": "auto", "tools": [],
    }  # fmt: skip


def gemini_ok(text, finish="STOP"):
    return {
        "candidates": [{"content": {"role": "model", "parts": [{"text": text}]}, "finishReason": finish, "index": 0}],
        "usageMetadata": {"promptTokenCount": 1, "candidatesTokenCount": 1, "totalTokenCount": 2},
    }


PROVIDERS = {
    "anthropic": dict(key="ANTHROPIC_API_KEY", ok=anthropic_ok, plan=PX_PLAN),
    "openai": dict(key="OPENAI_API_KEY", ok=openai_ok, plan=PX_PLAN),
    "gemini": dict(key="GEMINI_API_KEY", ok=gemini_ok, plan=NORM_PLAN),
}
NAMES = list(PROVIDERS)


def _client(api, name, tmp_path, *, timeout_s=5.0, max_retries=2, sleeps=None):
    env = {PROVIDERS[name]["key"]: "test-key-123"}
    return create_client(
        config_for(api, timeout_s=timeout_s, max_retries=max_retries), name, env=env, cache_dir=tmp_path / "cache",
        sleep=(sleeps.append if sleeps is not None else (lambda s: None)),
    )


def _reply_ok(name, plan=None):
    p = PROVIDERS[name]
    return Reply(200, p["ok"](_text(plan or p["plan"])))


# --- 正常系: 画像と指示を送り、ピクセル座標の PlanResult を返す ---------------------------------------------------------
@pytest.mark.parametrize("name", NAMES)
def test_plan_returns_pixel_boxes(fake_api, tmp_path, name):
    fake_api.respond(_reply_ok(name))
    r = _client(fake_api, name, tmp_path).plan(IMG, INSTRUCTION)
    assert isinstance(r, PlanResult)
    assert r.pick.label == "red block" and r.pick.bbox_px == (402, 210, 438, 252) and r.pick.confidence == 0.9
    assert r.place.label == "box" and r.place.confidence is None
    assert len(fake_api.requests) == 1


@pytest.mark.parametrize("name", NAMES)
def test_client_reports_provider_and_model_from_the_config(fake_api, tmp_path, name):
    c = _client(fake_api, name, tmp_path)
    assert c.provider == name and c.model == config_for(fake_api).providers[name].model


@pytest.mark.parametrize("name", NAMES)
def test_the_response_is_cached(fake_api, tmp_path, name):
    fake_api.respond(_reply_ok(name))
    c = _client(fake_api, name, tmp_path)
    c.plan(IMG, INSTRUCTION)
    c.plan(IMG, INSTRUCTION)
    assert len(fake_api.requests) == 1


def _png_from_b64(data: str) -> bytes:
    # Gemini の SDK は URL セーフな base64 (パディングなし) で送る (proto の JSON ではどちらも有効)。両方を読めるようにする
    raw = base64.b64decode(data.replace("-", "+").replace("_", "/") + "=" * (-len(data) % 4))
    assert raw[:8] == b"\x89PNG\r\n\x1a\n"
    return raw


def test_anthropic_request_shape(fake_api, tmp_path):
    fake_api.respond(_reply_ok("anthropic"))
    _client(fake_api, "anthropic", tmp_path).plan(IMG, INSTRUCTION)
    req = fake_api.requests[0]
    # anthropic 2.x の SDK は、キーを authorization: Bearer で送る (実測)。旧来の x-api-key でも通るようにしておく
    assert req.path.endswith("/v1/messages") and "test-key-123" in (req.headers.get("x-api-key") or req.headers["authorization"])
    body = req.body
    assert body["model"] == config_for(fake_api).providers["anthropic"].model and body["max_tokens"] >= 1024
    content = body["messages"][0]["content"]
    assert content[0]["type"] == "image" and content[0]["source"]["type"] == "base64"  # 画像が先 (推奨)
    assert content[0]["source"]["media_type"] == "image/png" and _png_from_b64(content[0]["source"]["data"])
    assert content[1]["type"] == "text" and INSTRUCTION in content[1]["text"] and "pixel" in content[1]["text"]
    assert body["output_config"]["format"]["type"] == "json_schema"
    assert not {"temperature", "top_p", "top_k", "budget_tokens"} & set(body) and "thinking" not in body


def test_openai_request_shape(fake_api, tmp_path):
    fake_api.respond(_reply_ok("openai"))
    _client(fake_api, "openai", tmp_path).plan(IMG, INSTRUCTION)
    req = fake_api.requests[0]
    assert req.path.endswith("/responses") and req.headers["authorization"] == "Bearer test-key-123"
    body = req.body
    parts = body["input"][0]["content"]
    kinds = {p["type"] for p in parts}
    assert kinds == {"input_text", "input_image"}
    image = next(p for p in parts if p["type"] == "input_image")
    assert image["image_url"].startswith("data:image/png;base64,") and _png_from_b64(image["image_url"].split(",", 1)[1])
    assert image["detail"] == "original"  # 座標が要るので、画像を縮小させない (公式ドキュメント)
    assert INSTRUCTION in next(p for p in parts if p["type"] == "input_text")["text"]
    fmt = body["text"]["format"]
    assert fmt["type"] == "json_schema" and fmt["strict"] is True and fmt["schema"]["additionalProperties"] is False


def test_gemini_request_shape(fake_api, tmp_path):
    fake_api.respond(_reply_ok("gemini"))
    _client(fake_api, "gemini", tmp_path).plan(IMG, INSTRUCTION)
    req = fake_api.requests[0]
    assert ":generateContent" in req.path and req.headers["x-goog-api-key"] == "test-key-123"
    assert config_for(fake_api).providers["gemini"].model in req.path
    parts = req.body["contents"][0]["parts"]
    inline = next(p for p in parts if "inlineData" in p)["inlineData"]
    assert (inline.get("mimeType") or inline["mime_type"]) == "image/png" and _png_from_b64(inline["data"])  # SDK は mime_type で送る (実測)
    assert INSTRUCTION in next(p for p in parts if "text" in p)["text"] and "ymin" in next(p for p in parts if "text" in p)["text"]
    gen = req.body["generationConfig"]
    assert gen["responseMimeType"] == "application/json" and "responseJsonSchema" in gen


@pytest.mark.parametrize("name", NAMES)
def test_the_schema_matches_what_parse_plan_expects(fake_api, tmp_path, name):
    fake_api.respond(_reply_ok(name))
    _client(fake_api, name, tmp_path).plan(IMG, INSTRUCTION)
    body = fake_api.requests[0].body
    schema = (
        body["output_config"]["format"]["schema"] if name == "anthropic"
        else body["text"]["format"]["schema"] if name == "openai"
        else body["generationConfig"]["responseJsonSchema"]
    )  # fmt: skip
    assert set(schema["properties"]) == {"pick", "place", "error"} and set(schema["required"]) == {"pick", "place", "error"}


# --- 再試行・タイムアウト (SDK 側の再試行は切り、BaseVlmClient が一括して管理する) ---------------------------------
@pytest.mark.parametrize("name", NAMES)
def test_rate_limit_is_retried_then_succeeds(fake_api, tmp_path, name):
    fake_api.respond(Reply(429), _reply_ok(name))
    sleeps = []
    r = _client(fake_api, name, tmp_path, sleeps=sleeps).plan(IMG, INSTRUCTION)
    assert r.pick.label == "red block" and len(fake_api.requests) == 2 and sleeps == [1.0]


@pytest.mark.parametrize("name", NAMES)
def test_server_errors_exhaust_exactly_max_retries_plus_one_requests(fake_api, tmp_path, name):
    fake_api.respond(Reply(503))
    sleeps = []
    with pytest.raises(VlmCallError, match="3") as ei:
        _client(fake_api, name, tmp_path, sleeps=sleeps).plan(IMG, INSTRUCTION)
    assert len(fake_api.requests) == 3 and sleeps == [1.0, 2.0]  # SDK の隠れた再試行で、回数が増えない
    assert isinstance(ei.value, PlanError)


@pytest.mark.parametrize("name", NAMES)
def test_timeout_is_transient(fake_api, tmp_path, name):
    fake_api.respond(Reply(200, PROVIDERS[name]["ok"](_text(PROVIDERS[name]["plan"])), delay=1.5))
    with pytest.raises(VlmCallError):
        _client(fake_api, name, tmp_path, timeout_s=0.3, max_retries=1).plan(IMG, INSTRUCTION)
    assert len(fake_api.requests) == 2


@pytest.mark.parametrize("name", NAMES)
@pytest.mark.parametrize("status", [400, 401, 403, 404])
def test_client_errors_are_not_retried_and_keep_their_type(fake_api, tmp_path, name, status):
    fake_api.respond(Reply(status))
    with pytest.raises(Exception) as ei:  # noqa: PT011  SDK ごとの型 (BadRequestError / ClientError など)
        _client(fake_api, name, tmp_path).plan(IMG, INSTRUCTION)
    assert not isinstance(ei.value, PlanError) and len(fake_api.requests) == 1
    assert "test-key-123" not in str(ei.value)  # 例外のメッセージに API キーを出さない


def test_openai_insufficient_quota_is_not_retried(fake_api, tmp_path):
    # 実 API の観測 (2026-11-15): クレジット切れは HTTP 429 で返る。レート制限と違い、待っても直らないので再試行しない
    import openai

    quota = {"error": {"message": "You exceeded your current quota, please check your plan and billing details.", "type": "insufficient_quota", "code": "insufficient_quota"}}
    fake_api.respond(Reply(429, quota))
    sleeps = []
    with pytest.raises(openai.RateLimitError, match="quota"):
        _client(fake_api, "openai", tmp_path, sleeps=sleeps).plan(IMG, INSTRUCTION)
    assert len(fake_api.requests) == 1 and sleeps == []
    # 通常のレート制限 (code が insufficient_quota でない 429) は、これまでどおり再試行する
    fake_api.respond(Reply(429, {"error": {"message": "Rate limit reached", "type": "requests", "code": "rate_limit_exceeded"}}), _reply_ok("openai"))
    assert _client(fake_api, "openai", tmp_path / "b").plan(IMG, INSTRUCTION).pick.label == "red block"
    assert len(fake_api.requests) == 2


@pytest.mark.parametrize("name", NAMES)
def test_connection_refused_is_transient(tmp_path, name):
    class Dead:
        url = "http://127.0.0.1:1"
        requests: list = []

    with pytest.raises(VlmCallError):
        _client(Dead, name, tmp_path, timeout_s=1.0, max_retries=1).plan(IMG, INSTRUCTION)


# --- 使えない応答は PlanError (動かさない) ----------------------------------------------------------------------
@pytest.mark.parametrize("name", NAMES)
def test_not_found_reason_reaches_the_error(fake_api, tmp_path, name):
    fake_api.respond(Reply(200, PROVIDERS[name]["ok"](_text(NOT_FOUND))))
    with pytest.raises(PlanError, match="no purple block"):
        _client(fake_api, name, tmp_path).plan(IMG, INSTRUCTION)
    assert len(fake_api.requests) == 1  # 対象なしは再試行しない


@pytest.mark.parametrize("name", NAMES)
def test_non_json_text_is_a_plan_error(fake_api, tmp_path, name):
    fake_api.respond(Reply(200, PROVIDERS[name]["ok"]("I cannot see any block.")))
    with pytest.raises(PlanError):
        _client(fake_api, name, tmp_path).plan(IMG, INSTRUCTION)


def test_anthropic_refusal_and_truncation_are_plan_errors(fake_api, tmp_path):
    fake_api.respond(Reply(200, anthropic_ok("", stop="refusal")))
    with pytest.raises(PlanError, match="拒否"):
        _client(fake_api, "anthropic", tmp_path).plan(IMG, INSTRUCTION)
    fake_api.respond(Reply(200, anthropic_ok('{"pick": {"label"', stop="max_tokens")))
    with pytest.raises(PlanError, match="途中"):
        _client(fake_api, "anthropic", tmp_path / "b").plan(IMG, INSTRUCTION)


def test_openai_refusal_and_incomplete_are_plan_errors(fake_api, tmp_path):
    fake_api.respond(Reply(200, openai_ok("", part={"type": "refusal", "refusal": "I can't help with that"})))
    with pytest.raises(PlanError, match="拒否"):
        _client(fake_api, "openai", tmp_path).plan(IMG, INSTRUCTION)
    body = openai_ok("")
    body.update(status="incomplete", incomplete_details={"reason": "max_output_tokens"})
    fake_api.respond(Reply(200, body))
    with pytest.raises(PlanError, match="途中"):
        _client(fake_api, "openai", tmp_path / "b").plan(IMG, INSTRUCTION)


def test_gemini_blocked_and_empty_responses_are_plan_errors(fake_api, tmp_path):
    fake_api.respond(Reply(200, {"promptFeedback": {"blockReason": "SAFETY"}, "usageMetadata": {}}))
    with pytest.raises(PlanError, match="SAFETY"):
        _client(fake_api, "gemini", tmp_path).plan(IMG, INSTRUCTION)
    fake_api.respond(Reply(200, gemini_ok(_text(PX_PLAN), finish="MAX_TOKENS")))
    with pytest.raises(PlanError, match="途中"):
        _client(fake_api, "gemini", tmp_path / "b").plan(IMG, INSTRUCTION)
    fake_api.respond(Reply(200, {"candidates": [], "usageMetadata": {}}))
    with pytest.raises(PlanError):
        _client(fake_api, "gemini", tmp_path / "c").plan(IMG, INSTRUCTION)


# --- 起動時のキー検査 -------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("name", NAMES)
def test_missing_key_stops_at_startup_without_calling_the_api(fake_api, tmp_path, name):
    with pytest.raises(MissingApiKeyError, match=PROVIDERS[name]["key"]):
        create_client(config_for(fake_api), name, env={}, cache_dir=tmp_path)
    assert fake_api.requests == []


def test_all_three_providers_are_registered_by_default():
    from so101_vlm import vlm

    assert {"anthropic", "openai", "gemini"} <= set(vlm._FACTORIES)


def test_config_options_are_not_required_for_the_real_endpoint():
    # base_url を設定しない通常の構成でも、クライアントを作れる (実際には呼ばない)
    from so101_vlm.vlm.config import load_vlm_config

    cfg = load_vlm_config()
    for name in NAMES:
        c = create_client(cfg, name, env={PROVIDERS[name]["key"]: "k"})
        assert c.provider == name
    assert dataclasses.is_dataclass(cfg)

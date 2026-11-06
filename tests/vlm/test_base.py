"""vlm.base / vlm.__init__ (create_client) のテスト (TASK-0018, Red)。SDK は使わずトランスポートを偽物にする。"""

import json

import numpy as np
import pytest

from so101_vlm import vlm
from so101_vlm.vlm import MissingApiKeyError, create_client, register_provider
from so101_vlm.vlm.base import BaseVlmClient, VlmCallError, VlmTransientError, build_prompt
from so101_vlm.vlm.cache import VlmCache
from so101_vlm.vlm.config import load_vlm_config
from so101_vlm.vlm.schema import PlanError, PlanResult

IMG = np.zeros((480, 640, 3), np.uint8)
GOOD = json.dumps(
    {
        "pick": {"label": "red block", "box": [100, 100, 200, 200]},
        "place": {"label": "box", "box": [300, 300, 400, 400]},
    }
)


class FakeTransport:
    def __init__(self, *outcomes):
        self.outcomes = list(outcomes)
        self.calls = []

    def __call__(self, png, prompt, timeout_s):
        self.calls.append((png, prompt, timeout_s))
        o = self.outcomes[min(len(self.calls) - 1, len(self.outcomes) - 1)]
        if isinstance(o, Exception):
            raise o
        return o


@pytest.fixture(autouse=True)
def _restore_registry():
    saved = dict(vlm._FACTORIES)
    yield
    vlm._FACTORIES.clear()
    vlm._FACTORIES.update(saved)


def _client(transport, tmp_path, *, retries=2, cache=True, fmt="xyxy_px", sleeps=None):
    return BaseVlmClient(
        provider="fake",
        model="m",
        coord_format=fmt,
        transport=transport,
        cache=VlmCache(tmp_path / "cache", enabled=cache),
        timeout_s=30,
        max_retries=retries,
        sleep=(sleeps.append if sleeps is not None else (lambda s: None)),
    )


def test_success_returns_plan_and_passes_timeout(tmp_path):
    t = FakeTransport(GOOD)
    r = _client(t, tmp_path).plan(IMG, "red block into box")
    assert isinstance(r, PlanResult) and r.pick.bbox_px == (100, 100, 200, 200)
    png, prompt, timeout = t.calls[0]
    assert png[:8] == b"\x89PNG\r\n\x1a\n" and timeout == 30
    assert "red block into box" in prompt


def test_retry_then_success_calls_twice_with_backoff(tmp_path):
    t = FakeTransport(VlmTransientError("timeout"), GOOD)
    sleeps = []
    r = _client(t, tmp_path, sleeps=sleeps).plan(IMG, "x")
    assert r.pick.label == "red block" and len(t.calls) == 2
    assert sleeps == [1.0]


def test_retries_are_capped_and_fail_with_plan_error(tmp_path):
    t = FakeTransport(VlmTransientError("rate limit"))
    sleeps = []
    with pytest.raises(VlmCallError, match="3") as ei:
        _client(t, tmp_path, sleeps=sleeps).plan(IMG, "x")
    assert len(t.calls) == 3 and sleeps == [1.0, 2.0]
    assert isinstance(ei.value, PlanError)  # motion は動かさない (FR-03a)


def test_zero_retries_calls_once(tmp_path):
    t = FakeTransport(VlmTransientError("x"))
    with pytest.raises(VlmCallError):
        _client(t, tmp_path, retries=0).plan(IMG, "x")
    assert len(t.calls) == 1


def test_non_transient_error_is_not_retried(tmp_path):
    t = FakeTransport(RuntimeError("401 unauthorized"))
    with pytest.raises(RuntimeError):
        _client(t, tmp_path).plan(IMG, "x")
    assert len(t.calls) == 1


def test_bad_response_is_plan_error_not_retried_not_cached(tmp_path):
    t = FakeTransport("not json", GOOD)
    c = _client(t, tmp_path)
    with pytest.raises(PlanError):
        c.plan(IMG, "x")
    assert len(t.calls) == 1
    assert c.plan(IMG, "x").pick.label == "red block"  # 失敗はキャッシュされず、再実行できる
    assert len(t.calls) == 2


def test_cache_hit_skips_transport(tmp_path):
    t = FakeTransport(GOOD)
    c = _client(t, tmp_path)
    a = c.plan(IMG, "x")
    b = c.plan(IMG, "x")
    assert a == b and len(t.calls) == 1


def test_cache_persists_across_clients(tmp_path):
    _client(FakeTransport(GOOD), tmp_path).plan(IMG, "x")
    t2 = FakeTransport(RuntimeError("must not be called"))
    assert _client(t2, tmp_path).plan(IMG, "x").pick.label == "red block"
    assert t2.calls == []


def test_cache_distinguishes_instruction_and_image(tmp_path):
    t = FakeTransport(GOOD)
    c = _client(t, tmp_path)
    c.plan(IMG, "a")
    c.plan(IMG, "b")
    c.plan(np.ones((480, 640, 3), np.uint8), "a")
    assert len(t.calls) == 3


def test_cache_can_be_disabled(tmp_path):
    t = FakeTransport(GOOD)
    c = _client(t, tmp_path, cache=False)
    c.plan(IMG, "x")
    c.plan(IMG, "x")
    assert len(t.calls) == 2


def test_coord_format_is_applied(tmp_path):
    raw = json.dumps(
        {
            "pick": {"label": "a", "box": [437, 628, 525, 684]},
            "place": {"label": "b", "box": [437, 628, 525, 684]},
        }
    )
    r = _client(FakeTransport(raw), tmp_path, fmt="yxyx_norm1000").plan(IMG, "x")
    assert r.pick.bbox_px == (402, 210, 438, 252)


@pytest.mark.parametrize("bad", [np.zeros((4, 4)), np.zeros((4, 4, 3), np.float32), "img", None])
def test_bad_image_is_value_error(tmp_path, bad):
    with pytest.raises(ValueError):
        _client(FakeTransport(GOOD), tmp_path).plan(bad, "x")


@pytest.mark.parametrize("instr", ["", "   ", None])
def test_bad_instruction_is_value_error(tmp_path, instr):
    with pytest.raises(ValueError):
        _client(FakeTransport(GOOD), tmp_path).plan(IMG, instr)


def test_prompt_states_format_size_and_json_only():
    p = build_prompt("put the red block in the box", "yxyx_norm1000", 640, 480)
    assert "put the red block in the box" in p and "JSON" in p
    assert "ymin" in p and "1000" in p
    q = build_prompt("x", "xyxy_px", 640, 480)
    assert "640" in q and "480" in q and "pixel" in q
    assert "pick" in q and "place" in q and "error" in q


# --- create_client ---------------------------------------------------------
def _cfg():
    return load_vlm_config()


def test_missing_key_names_the_variable(tmp_path):
    register_provider("anthropic", lambda pc, key: FakeTransport(GOOD))
    with pytest.raises(MissingApiKeyError, match="ANTHROPIC_API_KEY"):
        create_client(_cfg(), "anthropic", env={}, cache_dir=tmp_path)


def test_blank_key_counts_as_missing(tmp_path):
    register_provider("openai", lambda pc, key: FakeTransport(GOOD))
    with pytest.raises(MissingApiKeyError, match="OPENAI_API_KEY"):
        create_client(_cfg(), "openai", env={"OPENAI_API_KEY": "  "}, cache_dir=tmp_path)


def test_missing_key_error_is_value_error_and_hides_other_secrets(tmp_path):
    register_provider("gemini", lambda pc, key: FakeTransport(GOOD))
    with pytest.raises(ValueError) as ei:
        create_client(_cfg(), "gemini", env={"OPENAI_API_KEY": "sk-secret"}, cache_dir=tmp_path)
    assert "sk-secret" not in str(ei.value)


def test_create_client_builds_working_client_and_passes_key(tmp_path):
    seen = {}

    def factory(pc, key):
        seen["key"], seen["model"] = key, pc.model
        return FakeTransport(GOOD)

    register_provider("anthropic", factory)
    c = create_client(_cfg(), "anthropic", env={"ANTHROPIC_API_KEY": "k-1"}, cache_dir=tmp_path)
    assert c.plan(IMG, "x").pick.label == "red block"
    assert seen == {"key": "k-1", "model": _cfg().providers["anthropic"].model}


def test_keyless_provider_needs_no_env(tmp_path):
    register_provider("ollama", lambda pc, key: FakeTransport(GOOD))
    c = create_client(_cfg(), "ollama", env={}, cache_dir=tmp_path)
    assert c.plan(IMG, "x").place.label == "box"


def test_default_provider_from_config(tmp_path):
    register_provider("anthropic", lambda pc, key: FakeTransport(GOOD))
    c = create_client(_cfg(), env={"ANTHROPIC_API_KEY": "k"}, cache_dir=tmp_path)
    assert c.provider == "anthropic"


def test_unknown_provider_is_value_error(tmp_path):
    with pytest.raises(ValueError, match="nope"):
        create_client(_cfg(), "nope", env={}, cache_dir=tmp_path)


def test_unimplemented_provider_is_clear_error(tmp_path):
    vlm._FACTORIES.pop("gemini", None)
    with pytest.raises(NotImplementedError, match="gemini"):
        create_client(_cfg(), "gemini", env={"GEMINI_API_KEY": "k"}, cache_dir=tmp_path)


def test_corrupt_cache_entry_falls_back_to_a_call(tmp_path):
    t = FakeTransport(GOOD)
    c = _client(t, tmp_path)
    c.plan(IMG, "x")
    for f in (tmp_path / "cache").glob("*.txt"):
        f.write_text("garbage", encoding="utf-8")
    assert c.plan(IMG, "x").pick.label == "red block"
    assert len(t.calls) == 2

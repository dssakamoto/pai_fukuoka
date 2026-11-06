"""vlm.config のテスト (TASK-0018, Red)。"""

import copy

import pytest
import yaml

from so101_vlm.vlm.config import DEFAULT_VLM_CONFIG_PATH, load_vlm_config


def test_committed_config_loads():
    cfg = load_vlm_config()
    assert cfg.provider in cfg.providers
    assert set(cfg.providers) == {"anthropic", "openai", "gemini", "ollama"}
    assert cfg.timeout_s == 30 and cfg.max_retries == 2
    assert cfg.providers["ollama"].api_key_env is None
    assert cfg.providers["anthropic"].api_key_env == "ANTHROPIC_API_KEY"
    assert cfg.providers["gemini"].coord_format == "yxyx_norm1000"
    assert cfg.providers["ollama"].options["base_url"].startswith("http")


def _raw():
    return yaml.safe_load(DEFAULT_VLM_CONFIG_PATH.read_text(encoding="utf-8"))


def _write(tmp_path, raw):
    p = tmp_path / "vlm.yaml"
    p.write_text(yaml.safe_dump(raw), encoding="utf-8")
    return p


def test_unknown_top_key_is_value_error(tmp_path):
    raw = _raw()
    raw["surprise"] = 1
    with pytest.raises(ValueError, match="surprise"):
        load_vlm_config(_write(tmp_path, raw))


def test_default_provider_must_exist(tmp_path):
    raw = _raw()
    raw["provider"] = "nope"
    with pytest.raises(ValueError, match="nope"):
        load_vlm_config(_write(tmp_path, raw))


@pytest.mark.parametrize(
    "mutate",
    [
        lambda r: r.update(timeout_s=0),
        lambda r: r.update(timeout_s="x"),
        lambda r: r.update(max_retries=-1),
        lambda r: r["cache"].update(enabled="yes"),
        lambda r: r["providers"]["openai"].update(coord_format="weird"),
        lambda r: r["providers"]["openai"].update(model=""),
        lambda r: r["providers"]["openai"].update(api_key_env=5),
        lambda r: r["providers"]["openai"].pop("model"),
    ],
)
def test_invalid_values_are_value_error(tmp_path, mutate):
    raw = copy.deepcopy(_raw())
    mutate(raw)
    with pytest.raises(ValueError):
        load_vlm_config(_write(tmp_path, raw))

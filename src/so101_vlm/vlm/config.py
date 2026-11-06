"""VLM 設定 (configs/vlm.yaml) の読み込みと検証 (TASK-0018)。

🔵 design.md 4.3: プロバイダ名・モデル名・タイムアウト・再試行・キャッシュ。API キーはここに置かない (環境変数名だけ)。
違反は ValueError (キー名入り)。未知のキーは拒否する (typo を黙って無視しない)。
"""

from dataclasses import dataclass, field
from pathlib import Path

import yaml

from so101_vlm.eval.validation import require_exact_keys, require_int, require_number
from so101_vlm.robot.model_paths import REPO_ROOT
from so101_vlm.vlm.schema import COORD_FORMATS

DEFAULT_VLM_CONFIG_PATH = REPO_ROOT / "configs" / "vlm.yaml"

_TOP_KEYS = frozenset({"provider", "timeout_s", "max_retries", "cache", "providers"})
_CACHE_KEYS = frozenset({"enabled", "dir"})
_PROVIDER_REQUIRED = ("model", "api_key_env", "coord_format")


@dataclass(frozen=True)
class ProviderConfig:
    """【機能概要】: 1 プロバイダの設定。options は base_url など、プロバイダ固有の項目 🔵"""

    name: str
    model: str
    api_key_env: str | None
    coord_format: str
    options: dict = field(default_factory=dict)


@dataclass(frozen=True)
class VlmConfig:
    """【機能概要】: VLM 全体の設定 🔵"""

    provider: str
    timeout_s: float
    max_retries: int
    cache_enabled: bool
    cache_dir: Path
    providers: dict[str, ProviderConfig]


def _require_str(name: str, value: object) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} は空でない文字列で指定してください: {value!r}")
    return value


def _parse_provider(name: str, raw: object) -> ProviderConfig:
    if not isinstance(raw, dict):
        raise ValueError(f"providers.{name} は mapping で指定してください: {raw!r}")
    missing = [k for k in _PROVIDER_REQUIRED if k not in raw]
    if missing:
        raise ValueError(f"providers.{name} に必須キーがありません: {missing}")
    env = raw["api_key_env"]
    if env is not None:
        env = _require_str(f"providers.{name}.api_key_env", env)
    fmt = raw["coord_format"]
    if fmt not in COORD_FORMATS:
        raise ValueError(f"providers.{name}.coord_format は {COORD_FORMATS} のどれか: {fmt!r}")
    options = {k: v for k, v in raw.items() if k not in _PROVIDER_REQUIRED}
    return ProviderConfig(
        name, _require_str(f"providers.{name}.model", raw["model"]), env, fmt, options
    )


# 🔵 Intent: eval_config と同じ流儀 (yaml.safe_load + 厳密なキー検証)。
def load_vlm_config(path: str | Path = DEFAULT_VLM_CONFIG_PATH) -> VlmConfig:
    """【機能概要】: YAML を読み、検証済みの VlmConfig を返す。ファイルなしは FileNotFoundError 🔵"""
    with open(path, encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    data = require_exact_keys("(top level)", raw, _TOP_KEYS)
    cache = require_exact_keys("cache", data["cache"], _CACHE_KEYS)
    if not isinstance(cache["enabled"], bool):
        raise ValueError(f"cache.enabled は true/false で指定してください: {cache['enabled']!r}")
    raw_providers = data["providers"]
    if not isinstance(raw_providers, dict) or not raw_providers:
        raise ValueError("providers は 1 つ以上の mapping で指定してください")
    providers = {str(n): _parse_provider(str(n), p) for n, p in raw_providers.items()}
    default = data["provider"]
    if default not in providers:
        raise ValueError(f"provider={default!r} が providers にありません: {sorted(providers)}")
    timeout_s = require_number("timeout_s", data["timeout_s"])
    if timeout_s <= 0:
        raise ValueError(f"timeout_s は 0 より大きくしてください: {timeout_s}")
    cache_dir = Path(_require_str("cache.dir", cache["dir"]))
    if not cache_dir.is_absolute():
        cache_dir = REPO_ROOT / cache_dir
    return VlmConfig(
        provider=default,
        timeout_s=timeout_s,
        max_retries=require_int("max_retries", data["max_retries"], 0),
        cache_enabled=cache["enabled"],
        cache_dir=cache_dir,
        providers=providers,
    )

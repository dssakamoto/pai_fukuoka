"""VLM クライアント (TASK-0016〜0022)。create_client で設定のプロバイダから生成する。

プロバイダ実装 (TASK-0019〜0022) は register_provider でファクトリを登録する。
ファクトリは (ProviderConfig, api_key | None) -> トランスポート を返す (base.py 参照)。
"""

import os
from collections.abc import Callable, Mapping
from pathlib import Path

from so101_vlm.vlm.base import BaseVlmClient, Transport
from so101_vlm.vlm.cache import VlmCache
from so101_vlm.vlm.config import ProviderConfig, VlmConfig

Factory = Callable[[ProviderConfig, str | None], Transport]
_FACTORIES: dict[str, Factory] = {}


class MissingApiKeyError(ValueError):
    """【機能概要】: API キーの環境変数が未設定 (NFR-03)。変数名を含み、値は含まない 🔵"""


def register_provider(name: str, factory: Factory) -> None:
    """【機能概要】: プロバイダ名にトランスポートのファクトリを結びつける 🔵"""
    _FACTORIES[name] = factory


# 🔵 Intent: キーがないときは起動時 (クライアント生成時) に、変数名つきで失敗させる。
def create_client(
    config: VlmConfig,
    provider: str | None = None,
    *,
    env: Mapping[str, str] | None = None,
    cache_dir: str | Path | None = None,
    sleep: Callable[[float], None] | None = None,
) -> BaseVlmClient:
    """【機能概要】: 設定のプロバイダ (または引数で指定) の VlmClient を作る 🔵

    未知のプロバイダは ValueError、キー未設定は MissingApiKeyError、
    実装が未登録なら NotImplementedError。
    """
    name = provider or config.provider
    if name not in config.providers:
        raise ValueError(f"未知のプロバイダ {name!r} です。設定にあるもの: {sorted(config.providers)}")
    pc = config.providers[name]
    env = os.environ if env is None else env
    key = None
    if pc.api_key_env is not None:
        key = env.get(pc.api_key_env, "").strip()
        if not key:
            raise MissingApiKeyError(
                f"{name} の API キーがありません。環境変数 {pc.api_key_env} を設定してください"
                "(.env に書く。README 参照)"
            )
    factory = _FACTORIES.get(name)
    if factory is None:
        raise NotImplementedError(f"プロバイダ {name!r} の実装はまだ登録されていません")
    kwargs = {} if sleep is None else {"sleep": sleep}
    return BaseVlmClient(
        provider=name,
        model=pc.model,
        coord_format=pc.coord_format,
        transport=factory(pc, key),
        cache=VlmCache(cache_dir if cache_dir is not None else config.cache_dir, config.cache_enabled),
        timeout_s=config.timeout_s,
        max_retries=config.max_retries,
        **kwargs,
    )

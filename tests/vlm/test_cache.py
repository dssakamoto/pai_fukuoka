"""vlm.cache のテスト (TASK-0018, Red)。"""

import numpy as np

from so101_vlm.vlm.cache import VlmCache, make_key


def _img(v=0):
    return np.full((4, 5, 3), v, dtype=np.uint8)


def test_key_depends_on_every_component():
    base = make_key(_img(), "pick red", "anthropic", "m1", "p1")
    assert base == make_key(_img(), "pick red", "anthropic", "m1", "p1")
    assert base != make_key(_img(1), "pick red", "anthropic", "m1", "p1")
    assert base != make_key(_img(), "pick blue", "anthropic", "m1", "p1")
    assert base != make_key(_img(), "pick red", "openai", "m1", "p1")
    assert base != make_key(_img(), "pick red", "anthropic", "m2", "p1")
    assert base != make_key(_img(), "pick red", "anthropic", "m1", "p2")


def test_key_depends_on_image_shape_not_just_bytes():
    a = np.zeros((2, 6, 3), np.uint8)
    b = np.zeros((6, 2, 3), np.uint8)
    assert make_key(a, "i", "p", "m", "x") != make_key(b, "i", "p", "m", "x")


def test_put_get_roundtrip_and_persists(tmp_path):
    c = VlmCache(tmp_path / "c")
    assert c.get("k1") is None
    c.put("k1", '{"a": "日本語"}')
    assert c.get("k1") == '{"a": "日本語"}'
    assert VlmCache(tmp_path / "c").get("k1") == '{"a": "日本語"}'


def test_disabled_cache_never_stores(tmp_path):
    c = VlmCache(tmp_path / "c", enabled=False)
    c.put("k", "v")
    assert c.get("k") is None
    assert not (tmp_path / "c").exists()


def test_unsafe_key_is_a_miss_and_never_writes_outside(tmp_path):
    root = tmp_path / "cache"
    c = VlmCache(root)
    assert c.get("../evil") is None
    c.put("../evil", "v")
    assert not (tmp_path / "evil.txt").exists()
    assert not list(tmp_path.glob("evil*"))

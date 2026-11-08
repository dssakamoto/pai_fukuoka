"""nodes.messages のテスト (TASK-0025, Red)。dora に依存しない符号化。"""

import numpy as np
import pytest

from so101_vlm.nodes.messages import from_wire, to_wire


def _roundtrip(payload, meta=None):
    values, wire_meta = to_wire(payload, meta)
    return from_wire(values, wire_meta)


@pytest.mark.parametrize(
    "arr",
    [
        np.arange(480 * 640 * 3, dtype=np.uint8).reshape(480, 640, 3),
        np.linspace(0, 1, 12, dtype=np.float32).reshape(3, 4),
        np.arange(6, dtype=np.float64) * 0.1,
        np.zeros((0,), dtype=np.float64),
    ],
)
def test_array_roundtrip_keeps_shape_dtype_and_values(arr):
    back, meta = _roundtrip(arr, {"seq": 3})
    assert isinstance(back, np.ndarray) and back.shape == arr.shape and back.dtype == arr.dtype
    assert np.array_equal(back, arr) and meta == {"seq": 3}


def test_wire_array_is_flat_for_arrow():
    values, wire_meta = to_wire(np.zeros((2, 3, 4), np.uint8))
    assert values.ndim == 1 and values.shape == (24,)
    assert all(isinstance(v, str | int | float | bool) for v in wire_meta.values())


def test_dict_roundtrip_with_unicode_and_nested_lists():
    payload = {"label": "赤いブロック", "bbox": [1, 2, 3, 4], "x": {"y": [0.5, None, True]}}
    back, meta = _roundtrip(payload)
    assert back == payload and meta == {}


def test_dict_wire_is_a_single_json_string():
    values, _ = to_wire({"a": 1})
    assert isinstance(values, list) and len(values) == 1 and isinstance(values[0], str)


def test_numpy_scalars_inside_dict_are_converted():
    back, _ = _roundtrip({"a": np.float64(1.5), "b": np.int64(2), "c": np.array([1, 2])})
    assert back == {"a": 1.5, "b": 2, "c": [1, 2]}


@pytest.mark.parametrize("bad", [{"a": float("nan")}, {"a": {1, 2}}, "text", 3, None])
def test_unsupported_payload_is_value_error(bad):
    with pytest.raises(ValueError):
        to_wire(bad)


def test_unsupported_dtype_is_value_error():
    with pytest.raises(ValueError):
        to_wire(np.array(["a", "b"]))


def test_meta_must_be_json_serialisable():
    with pytest.raises(ValueError):
        to_wire({"a": 1}, {"x": object()})


def test_from_wire_rejects_unknown_kind():
    with pytest.raises(ValueError):
        from_wire([], {"_kind": "mystery"})

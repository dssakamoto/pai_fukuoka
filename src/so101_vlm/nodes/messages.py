"""ノード間メッセージの符号化 (TASK-0025)。dora / pyarrow に依存しない。

🔵 ADR 0001: dora の出力は 1 次元の Arrow 配列 + メタデータ。配列は平坦化して shape / dtype をメタデータで渡し、
受信側で復元する。構造のあるデータ (検出結果・軌道・結果) は JSON 文字列 1 個にする。
ローカルランナーも同じ to_wire / from_wire を通すので、dora でしか見つからない符号化の不具合を CI で拾える。

ペイロード: numpy 配列 (uint8 / float32 / float64 / int64) または JSON にできる dict。
ユーザのメタデータ meta は JSON にできる dict (seq など)。
"""

import json

import numpy as np

_KIND = "_kind"
_USER = "_meta"
_SHAPE = "_shape"
_DTYPE = "_dtype"
_ALLOWED_DTYPES = ("uint8", "float32", "float64", "int64")

Payload = np.ndarray | dict
WireMeta = dict[str, str | int | float | bool]


def _jsonable(obj: object) -> object:
    """numpy のスカラ・配列を Python の値にする (json.dumps の default)。それ以外は拒否する。"""
    if isinstance(obj, np.generic):
        return obj.item()
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    raise TypeError(f"JSON にできない値です: {type(obj).__name__}")


def _dumps(value: object, what: str) -> str:
    try:
        return json.dumps(value, ensure_ascii=False, allow_nan=False, default=_jsonable)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{what} を JSON にできません: {exc}") from exc


# 🔵 Intent: 配列は平坦化 (ADR 0001)、dict は JSON 1 個。NaN は JSON に入れない (相手が読めなくなる)。
def to_wire(
    payload: object, meta: dict | None = None
) -> tuple[np.ndarray | list[str], WireMeta]:
    """【機能概要】: ペイロードとメタデータを、1 次元の値とフラットなメタデータにする 🔵

    配列 -> (1 次元配列, {_kind: array, _shape, _dtype, _meta})、dict -> ([JSON 文字列], {_kind: json, _meta})。
    未対応の型・dtype・JSON にできないものは ValueError。
    """
    wire: WireMeta = {_USER: _dumps(meta or {}, "meta")}
    if isinstance(payload, np.ndarray):
        if payload.dtype.name not in _ALLOWED_DTYPES:
            raise ValueError(f"未対応の dtype です: {payload.dtype} (対応: {_ALLOWED_DTYPES})")
        wire.update({_KIND: "array", _SHAPE: ",".join(str(n) for n in payload.shape), _DTYPE: payload.dtype.name})
        return np.ascontiguousarray(payload).ravel(), wire
    if isinstance(payload, dict):
        wire[_KIND] = "json"
        return [_dumps(payload, "payload")], wire
    raise ValueError(f"payload は numpy 配列か dict で指定してください: {type(payload).__name__}")


def from_wire(values: object, wire: dict) -> tuple[Payload, dict]:
    """【機能概要】: to_wire の逆。(ペイロード, ユーザのメタデータ) を返す 🔵"""
    kind = wire.get(_KIND)
    meta = json.loads(wire.get(_USER, "{}"))
    if kind == "array":
        shape_text = str(wire[_SHAPE])
        shape = tuple(int(n) for n in shape_text.split(",")) if shape_text else ()
        flat = np.asarray(values, dtype=np.dtype(str(wire[_DTYPE])))
        return flat.reshape(shape).copy(), meta
    if kind == "json":
        return json.loads(values[0]), meta
    raise ValueError(f"未知のメッセージ種別です: {kind!r}")

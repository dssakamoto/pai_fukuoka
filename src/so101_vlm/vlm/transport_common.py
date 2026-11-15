"""プロバイダ共通の部品: 応答の JSON スキーマ、画像の符号化、一時的な失敗の判定 (TASK-0019〜0021)。

🔵 3 社とも構造化出力に同じスキーマを使う (vlm/schema.py の parse_plan が読む形)。スキーマは各社の strict の制約
(全プロパティが required、additionalProperties: false、minimum/maximum などは使わない) の範囲で書く。
値の範囲 (座標の大きさ、confidence の 0〜1) は、スキーマではなく parse_plan が検査する。
"""

import base64

_DETECTION = {
    "type": "object",
    "properties": {
        "label": {"type": "string"},
        "box": {"type": "array", "items": {"type": "number"}},
        "confidence": {"type": ["number", "null"]},
    },
    "required": ["label", "box", "confidence"],
    "additionalProperties": False,
}

# pick / place は、対象が見つからないとき null (理由は error)。base.py の共通プロンプトと同じ形。
PLAN_SCHEMA = {
    "type": "object",
    "properties": {
        "pick": {"anyOf": [_DETECTION, {"type": "null"}]},
        "place": {"anyOf": [_DETECTION, {"type": "null"}]},
        "error": {"type": ["string", "null"]},
    },
    "required": ["pick", "place", "error"],
    "additionalProperties": False,
}

# 🟡 構造化出力が truncation しないよう、思考のぶんも見込んだ余裕 (出力の JSON 自体は数十トークン)
DEFAULT_MAX_TOKENS = 4096


def png_to_b64(png: bytes) -> str:
    """【機能概要】: PNG のバイト列を base64 の文字列にする 🔵"""
    return base64.standard_b64encode(png).decode("ascii")


def is_transient_status(code: int | None) -> bool:
    """【機能概要】: 再試行してよい HTTP ステータスか (408 タイムアウト、409 競合、429 レート制限、5xx) 🔵"""
    return code is not None and (code >= 500 or code in (408, 409, 429))

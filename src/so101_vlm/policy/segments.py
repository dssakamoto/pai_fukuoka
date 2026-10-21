"""区間名の単一の定義元 (TASK-0013 Refactor)。

🔵 ik_policy.SEGMENT_TABLE の区間名と、executor (経由点の引き当て) / eval.pick_tracker (追跡の対象区間)
が使う名前を 1 か所にまとめる。標準ライブラリだけに依存するので、どのモジュールからも循環せず読める。
"""

from collections.abc import Mapping
from types import MappingProxyType

# 【定数定義】: 追跡・判定が参照する区間名 🔵
SEGMENT_LIFT = "lift"
SEGMENT_PRE_PLACE = "pre_place"
SEGMENT_PLACE = "place"

# 【定数定義】: 区間名 -> 経由点 (target_base を引くための対応)。ik_policy.SEGMENT_TABLE と一致させる 🔵
SEGMENT_WAYPOINT: Mapping[str, str] = MappingProxyType(
    {
        "pre_grasp": "pre_grasp",
        "grasp": "grasp",
        "close": "grasp",
        "close_hold": "grasp",
        SEGMENT_LIFT: "lift",
        SEGMENT_PRE_PLACE: "pre_place",
        SEGMENT_PLACE: "place",
        "open": "place",
        "open_hold": "place",
        "retreat": "retreat",
    }
)

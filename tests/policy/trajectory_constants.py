"""trajectory / IkPolicy テスト共有の定数とヘルパ (TASK-0012, Red フェーズ)。

tests/safety/safety_constants.py と同じ方式 (conftest を使わず、名前の違う補助モジュールにする)。
値の根拠: docs/implements/so101-vlm-pickplace/TASK-0012/trajectory-testcases.md 0.1, 0.2, 0.4
信頼性: 🔵 要件・実測どおり / 🟡 妥当な推測 / 🔴 資料にない仮定
"""

import numpy as np

# 【時間配分の定数】: configs/motion.yaml の safety: と要件 2.7 🔵
VMAX = np.ones(6)
DT = 0.02
SCALE = 1.0
FACTOR = 0.99
MIN_SEG_S = 0.2
STEP_UNIT = 1.0 * 1.0 * 0.02 * 0.99  # 掛ける順もこのとおり (= 0.0198)
MIN_SEG_STEPS = 10  # ceil(0.2 / 0.02)

TCP_OFFSET = (0.004, 0.0, 0.030)
G_OPEN = 0.6
G_CLOSED = 0.15
PICK_CENTER = (0.25, 0.0, 0.015)
PLACE_BOX = (0.20, 0.19, 0.005)  # 箱の底板上面の中心
HOME_Q = np.zeros(6)

SEGMENTS = (
    "start", "pre_grasp", "grasp", "close", "close_hold", "lift",
    "pre_place", "place", "open", "open_hold", "retreat",
)  # fmt: skip
WAYPOINT_NAMES = ("pre_grasp", "grasp", "lift", "pre_place", "place", "retreat")
# 区間 -> TCP 目標の経由点名 (apply_safety の target_base に使う) 🟡
SEG_TARGET = {
    "pre_grasp": "pre_grasp",
    "grasp": "grasp",
    "close": "grasp",
    "close_hold": "grasp",
    "lift": "lift",
    "pre_place": "pre_place",
    "place": "place",
    "open": "place",
    "open_hold": "place",
    "retreat": "retreat",
}

FK_TOL_M = 0.002
PITCH_TOL_DEG = 5.0
NEAR_SHOULDER = (0.16078614047633127, -0.023326223842896357, 0.015)
BLOCK_Z = 0.015

# 【合成の経由点】: 0.2 節 (MuJoCo 不要の純粋層テスト用) 🟡
Z6 = np.zeros(6)
PG = np.array([0.3, -0.5, 0.4, 1.2, 0.0, 0.6])
GR = np.array([0.3, -0.3, 0.6, 1.0, 0.0, 0.6])
CL = GR.copy()
CL[5] = 0.15
LI = PG.copy()
LI[5] = 0.15
PP = np.array([1.0, -0.4, 0.5, 1.2, 0.0, 0.15])
PL = np.array([1.0, -0.2, 0.7, 1.0, 0.0, 0.15])
OP = PL.copy()
OP[5] = 0.6
RT = PP.copy()
RT[5] = 0.6
CLOSE_HOLD_S = 0.5
OPEN_HOLD_S = 0.3

# 0.2 節の刻み数の表 (scale 1.0 / 0.3)
STEPS_SCALE_1 = {
    "pre_grasp": 61, "grasp": 11, "close": 23, "close_hold": 25, "lift": 11,
    "pre_place": 36, "place": 11, "open": 23, "open_hold": 15, "retreat": 11,
}  # fmt: skip
STEPS_SCALE_03 = {
    "pre_grasp": 203, "grasp": 34, "close": 76, "close_hold": 25, "lift": 34,
    "pre_place": 118, "place": 34, "open": 76, "open_hold": 15, "retreat": 34,
}  # fmt: skip

# 0.4 節の代表配置 (検証する 8 配置)。値は (x, y) = ブロック中心の水平位置
CORNERS = {
    "corner_xlo_ylo": (0.15, -0.10),
    "corner_xlo_yhi": (0.15, 0.10),
    "corner_xhi_ylo": (0.35, -0.10),
    "corner_xhi_yhi": (0.35, 0.10),
}


def time_kwargs(speed_scale: float = SCALE, min_segment_s: float = MIN_SEG_S) -> dict:
    """segment_steps / build_joint_trajectory に共通のキーワード引数 (metadata を除く)。"""
    return dict(
        vmax=VMAX,
        speed_scale=speed_scale,
        control_dt=DT,
        safety_factor=FACTOR,
        min_segment_s=min_segment_s,
    )


def unit_delta(first: float, *rest: float) -> np.ndarray:
    """先頭の関節から順に値を並べ、残りを 0 で埋めた 6 要素の差分を返す。"""
    values = [first, *rest]
    return np.array(values + [0.0] * (6 - len(values)), dtype=np.float64)

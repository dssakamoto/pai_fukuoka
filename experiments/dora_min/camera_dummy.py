"""タイマーごとにダミー画像・関節コマンドを送るノード(カメラ / 指令役)。"""
import time

import pyarrow as pa
from dora import Node

from common import BIG_SHAPE, N_BIG, N_JOINT, N_SMALL, SMALL_SHAPE, make_frame, make_joint

node = Node()
tick = 0
total = N_SMALL + N_BIG
for event in node:
    if event["type"] != "INPUT":
        break
    if tick < N_SMALL:
        frame = make_frame(SMALL_SHAPE, tick)
        node.send_output("small_rgb", pa.array(frame.ravel()), metadata={"counter": tick, "t_ns": time.time_ns()})
    else:
        c = tick - N_SMALL
        frame = make_frame(BIG_SHAPE, c)
        t0 = time.perf_counter()
        arr = pa.array(frame.ravel())  # NumPy -> Arrow (コピーあり)
        t_conv = time.perf_counter() - t0
        node.send_output("big_rgb", arr, metadata={"counter": c, "t_ns": time.time_ns(), "conv_s": t_conv})
    if tick < N_JOINT:
        node.send_output("joint_cmd", pa.array(make_joint(tick)), metadata={"counter": tick, "t_ns": time.time_ns()})
    tick += 1
    if tick >= total:
        break

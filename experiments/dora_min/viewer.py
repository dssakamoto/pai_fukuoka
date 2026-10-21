"""画像と robot_state を受信し、送信側と同じ式で再生成した配列と一致するか検証する。
全て揃ったら要約を表示して終了する。不一致なら exit code 1。"""
import statistics
import sys
import time

import numpy as np
from dora import Node

from common import BIG_SHAPE, N_BIG, N_JOINT, N_SMALL, SMALL_SHAPE, make_frame, make_joint

node = Node()
lat = {"small_rgb": [], "big_rgb": [], "robot_state": []}
errors = []
count = {k: 0 for k in lat}
conv_s = []
t_first_big = t_last_big = None

for event in node:
    if event["type"] == "INPUT":
        now = time.time_ns()
        eid, md = event["id"], event["metadata"]
        arr = event["value"].to_numpy()
        c = md["counter"]
        if eid == "small_rgb":
            ok = arr.dtype == np.uint8 and np.array_equal(arr.reshape(SMALL_SHAPE), make_frame(SMALL_SHAPE, c))
        elif eid == "big_rgb":
            ok = arr.dtype == np.uint8 and np.array_equal(arr.reshape(BIG_SHAPE), make_frame(BIG_SHAPE, c))
            conv_s.append(md["conv_s"])
            t_first_big = t_first_big or time.perf_counter()
            t_last_big = time.perf_counter()
        else:
            ok = arr.dtype == np.float64 and arr.shape == (6,) and np.array_equal(arr, make_joint(c) + 1.0)
        if not ok:
            errors.append(f"{eid}#{c} 不一致")
        count[eid] += 1
        lat[eid].append((now - md["t_ns"]) / 1e6)
    elif event["type"] == "STOP":
        break
    if count == {"small_rgb": N_SMALL, "big_rgb": N_BIG, "robot_state": N_JOINT}:
        break


def summary(name):
    v = lat[name]
    return f"{name}: 受信 {len(v)} 件, 遅延 median={statistics.median(v):.2f}ms max={max(v):.2f}ms" if v else f"{name}: 受信 0 件"


for k in lat:
    print(summary(k), flush=True)
if conv_s:
    print(f"NumPy->Arrow 変換(480x640x3) median={statistics.median(conv_s)*1e3:.2f}ms", flush=True)
exp = {"small_rgb": N_SMALL, "big_rgb": N_BIG, "robot_state": N_JOINT}
if errors or count != exp:
    print(f"NG: errors={errors} count={count}", flush=True)
    sys.exit(1)
print("OK: 全配列が送信内容と一致 (shape/dtype/値)", flush=True)

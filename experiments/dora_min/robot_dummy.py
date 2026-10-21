"""joint_cmd を受け取り、+1 した値を robot_state として返すノード(robot 役)。"""
from dora import Node
import pyarrow as pa

node = Node()
for event in node:
    if event["type"] == "INPUT":
        cmd = event["value"].to_numpy()
        node.send_output("robot_state", pa.array(cmd + 1.0), metadata=event["metadata"])
    elif event["type"] in ("STOP", "INPUT_CLOSED"):
        break

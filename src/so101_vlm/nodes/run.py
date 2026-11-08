"""dora が各ノードを起動するときの入口: python -m so101_vlm.nodes.run <node_id> (TASK-0025)。"""

import sys

from so101_vlm.nodes.dora_runtime import main

if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("使い方: python -m so101_vlm.nodes.run <node_id>")
    main(sys.argv[1])

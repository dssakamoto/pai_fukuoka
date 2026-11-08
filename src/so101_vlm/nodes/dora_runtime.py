"""dora-rs とノードをつなぐ薄い層 (TASK-0025)。dora と pyarrow が必要 (uv sync --group dataflow)。

🔵 ADR 0001: Node() のイベントループ + send_output(名前, pa.array(...), metadata=...)。受信は event["value"] を numpy / 文字列に戻す。
ノード (NodeBase) は dora を知らない。ここが emit (= send_output) と wait (= 返事が来るまで他の入力を処理して待つ) を差し込む。
環境変数: CONFIG (configs/sim.yaml など)、OUT_DIR (出力先)、task ノードだけ TASK_SEED / TASK_COLOUR / TASK_INSTRUCTION。
"""

import dataclasses
import os
from collections.abc import Callable
from pathlib import Path

from so101_vlm.nodes.base import NodeBase, StopRequested
from so101_vlm.nodes.messages import from_wire, to_wire
from so101_vlm.nodes.node_config import SIM_CONFIG_PATH, NodeConfig, load_node_config
from so101_vlm.nodes.registry import TaskSpec, create_node
from so101_vlm.robot.model_paths import REPO_ROOT


def config_from_env(env: dict[str, str] | None = None) -> NodeConfig:
    """【機能概要】: CONFIG (設定ファイル) と OUT_DIR (出力先) の環境変数から NodeConfig を作る 🔵"""
    env = dict(os.environ) if env is None else env
    raw = env.get("CONFIG")
    path = SIM_CONFIG_PATH if not raw else (Path(raw) if Path(raw).is_absolute() else REPO_ROOT / raw)
    config = load_node_config(path)
    out_dir = env.get("OUT_DIR")
    return dataclasses.replace(config, output_dir=Path(out_dir)) if out_dir else config


def task_from_env(env: dict[str, str] | None = None) -> TaskSpec:
    """【機能概要】: TASK_SEED / TASK_COLOUR / TASK_INSTRUCTION から TaskSpec を作る。不正なら ValueError 🔵"""
    env = dict(os.environ) if env is None else env
    try:
        seed = int(env["TASK_SEED"])
        return TaskSpec(seed, env["TASK_COLOUR"], env["TASK_INSTRUCTION"])
    except KeyError as exc:
        raise ValueError(f"環境変数 {exc.args[0]} がありません") from exc


def _decode(event: dict) -> tuple[object, dict]:
    import pyarrow as pa  # noqa: F401  (型の確認用。dora が pyarrow を使う)

    value = event["value"]
    metadata = dict(event["metadata"])
    kind = metadata.get("_kind")
    values = value.to_numpy(zero_copy_only=False) if kind == "array" else value.to_pylist()
    return from_wire(values, metadata)


def run_node(node: NodeBase) -> None:
    """【機能概要】: dora の Node にノードをつなぎ、STOP か finished まで入力を処理する 🔵"""
    import pyarrow as pa
    from dora import Node as DoraNode

    dora_node = DoraNode()

    def emit(name: str, payload: object, meta: dict | None = None) -> None:
        values, wire = to_wire(payload, meta)
        dora_node.send_output(name, pa.array(values), metadata=wire)

    def dispatch(event: dict | None) -> bool:
        """イベント 1 件を処理する。続けてよければ True。"""
        if event is None or event["type"] == "STOP":
            return False
        if event["type"] == "INPUT":
            payload, meta = _decode(event)
            node.on_input(event["id"], payload, meta)
        return not node.finished

    def wait(predicate: Callable[[], bool]) -> None:
        while not predicate():
            if not dispatch(dora_node.next()):
                raise StopRequested()

    node.bind(emit, wait)
    node.start()
    while not node.finished:
        if not dispatch(dora_node.next()):
            break


def main(node_id: str) -> None:
    """【機能概要】: node_id のノードを環境変数の設定で作り、dora で動かす 🔵"""
    config = config_from_env()
    task = task_from_env() if node_id == "task" else None
    run_node(create_node(node_id, config, task))


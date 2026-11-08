"""dataflow.yml の配線を、同じプロセス内で再生するランナー (TASK-0025)。

🔵 dora なしで (CI・デバッグで) データフロー全体を確かめる。配線は dataflow.yml から読み、メッセージは dora と同じ
to_wire / from_wire を通す。配信は深さ優先 (emit した瞬間に、配線先の on_input を呼ぶ)。
そのため、入力の到着順は dora と違う。ノードは「揃うまで溜める」書き方なので、どちらでも同じ結果になる。
終了したノード (finished) には配信しない (dora ではプロセスがなくなるのと同じ)。
"""

from collections.abc import Callable
from pathlib import Path

from so101_vlm.nodes.base import NodeBase
from so101_vlm.nodes.dataflow import DATAFLOW_TEMPLATE_PATH, parse_graph, render_dataflow
from so101_vlm.nodes.evaluator import RecorderNode
from so101_vlm.nodes.messages import from_wire, to_wire
from so101_vlm.nodes.node_config import NodeConfig
from so101_vlm.nodes.registry import create_node
from so101_vlm.nodes.registry_types import TaskSpec

Override = Callable[[NodeConfig, TaskSpec], object]


class LocalRunner:
    """【機能概要】: 1 試行ぶんのデータフローをプロセス内で実行する 🔵

    log: (ノード id, 出力名) の発行順の記録。result: evaluator の result。recorded_paths: recorder が書いたファイル。
    overrides: ノード id -> (config, task) -> ノード。テストで一部のノードを差し替える。
    """

    def __init__(
        self,
        config: NodeConfig,
        seed: int,
        colour: str,
        instruction: str,
        *,
        overrides: dict[str, Override] | None = None,
    ) -> None:
        self.config = config
        self.task = TaskSpec(seed, colour, instruction)
        graph = parse_graph(
            render_dataflow(
                DATAFLOW_TEMPLATE_PATH.read_text(encoding="utf-8"), python="python", config="(local)",
                seed=seed, colour=colour, instruction=instruction, out_dir=str(config.output_dir),
            )
        )
        self._routes: dict[tuple[str, str], list[tuple[str, str]]] = {}
        for node_id, node in graph.items():
            for input_name, source in node.inputs.items():
                src_node, output = source.split("/")
                self._routes.setdefault((src_node, output), []).append((node_id, input_name))
        overrides = overrides or {}
        self.nodes: dict[str, object] = {
            node_id: overrides[node_id](config, self.task) if node_id in overrides else create_node(node_id, config, self.task)
            for node_id in graph
        }
        self.log: list[tuple[str, str]] = []
        self.result: dict | None = None
        for node_id, node in self.nodes.items():
            node.bind(self._make_emit(node_id), self._wait)  # type: ignore[attr-defined]

    @property
    def recorded_paths(self) -> list[Path]:
        recorder = self.nodes.get("recorder")
        return recorder.written if isinstance(recorder, RecorderNode) else []

    def _wait(self, predicate: Callable[[], bool]) -> None:
        if not predicate():
            raise RuntimeError("待っている返事が届いていません (ローカルランナーでは、返事は送信の中で届くはずです)")

    def _make_emit(self, node_id: str) -> Callable[[str, object, dict | None], None]:
        def emit(name: str, payload: object, meta: dict | None = None) -> None:
            self.log.append((node_id, name))
            if node_id == "evaluator" and name == "result":
                self.result = payload  # type: ignore[assignment]
            values, wire = to_wire(payload, meta)  # dora と同じ符号化を通す
            for dst, input_name in self._routes.get((node_id, name), []):
                node: NodeBase = self.nodes[dst]  # type: ignore[assignment]
                if node.finished:
                    continue
                decoded, decoded_meta = from_wire(values, wire)  # 配信先ごとに別のコピー
                node.on_input(input_name, decoded, decoded_meta)

        return emit

    def run(self) -> dict:
        """【機能概要】: 全ノードを始め、evaluator の result を返す。result が出なければ RuntimeError 🔵"""
        for node in self.nodes.values():
            node.start()  # type: ignore[attr-defined]
        if self.result is None:
            raise RuntimeError("データフローが result を出さずに止まりました")
        return self.result

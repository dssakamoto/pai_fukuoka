"""dataflow.yml (テンプレート) の描画と配線の読み取り (TASK-0025)。

🔵 dataflow.yml は {{PYTHON}} などのプレースホルダを持つテンプレート。値は JSON の文字列リテラルにして埋める
(JSON の文字列は YAML の二重引用符の文字列として読めるので、改行・引用符・コロンを含む指示でも YAML を壊せない)。
"""

import re
import shutil
import sys
from dataclasses import dataclass
from pathlib import Path

import yaml

from so101_vlm.robot.model_paths import REPO_ROOT

DATAFLOW_TEMPLATE_PATH = REPO_ROOT / "dataflow.yml"
_PLACEHOLDER = re.compile(r"\{\{([A-Z_]+)\}\}")


@dataclass(frozen=True)
class GraphNode:
    """【機能概要】: データフローの 1 ノード。inputs は 入力名 -> "ノード/出力名" 🔵"""

    inputs: dict[str, str]
    outputs: list[str]


def _quote(value: object) -> str:
    import json

    return json.dumps(str(value), ensure_ascii=False)


# 🔵 Intent: 値は全て JSON 文字列にして YAML に埋める (YAML 注入の防止)。未知のプレースホルダは拒否する。
def render_dataflow(
    template: str, *, python: str, config: str, seed: int, colour: str, instruction: str, out_dir: str
) -> str:
    """【機能概要】: テンプレートのプレースホルダを埋めた dataflow.yml のテキストを返す 🔵"""
    values = {
        "PYTHON": python, "CONFIG": config, "SEED": seed, "COLOUR": colour,
        "INSTRUCTION": instruction, "OUT_DIR": out_dir,
    }

    def fill(match: re.Match) -> str:
        key = match.group(1)
        if key not in values:
            raise ValueError(f"未知のプレースホルダです: {{{{{key}}}}}")
        return _quote(values[key])

    return _PLACEHOLDER.sub(fill, template)


def parse_graph(rendered: str) -> dict[str, GraphNode]:
    """【機能概要】: 描画済みの dataflow.yml から ノード id -> GraphNode を読む 🔵"""
    doc = yaml.safe_load(rendered)
    return {
        n["id"]: GraphNode(dict(n.get("inputs") or {}), list(n.get("outputs") or []))
        for n in doc["nodes"]
    }


def find_dora() -> str | None:
    """【機能概要】: dora 実行ファイルのパス。今の Python と同じ環境 (venv) を優先し、無ければ PATH。無ければ None 🟡"""
    exe = "dora.exe" if sys.platform == "win32" else "dora"
    beside = Path(sys.executable).parent / exe
    if beside.is_file():
        return str(beside)
    return shutil.which("dora")

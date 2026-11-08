"""dora run での実行テスト (TASK-0025)。dora-rs-cli / dora-rs が入っていないときはスキップする。

scripts/run_dataflow.py が dataflow.yml を描画し (インタプリタのパスを埋める)、dora run で 1 試行を実行する。
結果はローカルランナー・run_trial と同じになるはず。
"""

import importlib.util
import json
import subprocess
import sys

import pytest

from so101_vlm.robot.model_paths import REPO_ROOT
from so101_vlm.nodes.dataflow import find_dora

pytestmark = pytest.mark.skipif(
    find_dora() is None or importlib.util.find_spec("dora") is None,
    reason="dora-rs が入っていない (uv sync --group dataflow)",
)


def test_dora_run_gives_the_same_result(tmp_path):
    proc = subprocess.run(
        [sys.executable, "-X", "utf8", "-I", str(REPO_ROOT / "scripts" / "run_dataflow.py"),
         "--seed", "2", "--colour", "red", "--instruction", "put the red block into the box",
         "--out-dir", str(tmp_path), "--timeout", "240"],
        capture_output=True, text=True, encoding="utf-8", timeout=300, cwd=REPO_ROOT,
    )
    assert proc.returncode == 0, proc.stdout[-2000:] + proc.stderr[-2000:]
    result = json.loads((tmp_path / "result.json").read_text(encoding="utf-8"))
    assert result["success"] is True and result["failure"] is None and result["seed"] == 2

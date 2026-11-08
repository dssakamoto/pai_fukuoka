"""dora-rs のデータフローで 1 試行を実行する (TASK-0025)。

  uv sync --group dataflow
  uv run python scripts/run_dataflow.py --seed 0 --colour red --instruction "put the red block into the box"

dataflow.yml (テンプレート) に、この環境のインタプリタ・設定・試行の指定を埋めて <out-dir>/dataflow.rendered.yml に書き、
`dora run` で実行する。結果は <out-dir>/result.json (成功判定・失敗の種類・推定位置など) と trial_seed*_*.json。
sim と real の切り替えは --config (configs/sim.yaml / configs/real.yaml) だけ。
終了コード: 0 = 成功、1 = 試行は動いたが失敗 (VLM・計画・把持など)、2 = 引数の誤り、3 = dora の実行失敗・時間切れ。
"""

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

from so101_vlm.nodes.dataflow import DATAFLOW_TEMPLATE_PATH, find_dora, render_dataflow
from so101_vlm.nodes.node_config import SIM_CONFIG_PATH, load_node_config
from so101_vlm.nodes.registry_types import TaskSpec
from so101_vlm.robot.model_paths import REPO_ROOT


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", type=Path, default=SIM_CONFIG_PATH)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--colour", default="red")
    parser.add_argument("--instruction", default="put the red block into the box")
    parser.add_argument("--out-dir", type=Path, default=None, help="既定は設定の output_dir")
    parser.add_argument("--timeout", type=float, default=None, help="dora run を打ち切る秒数 (既定は設定の timeout_s)")
    args = parser.parse_args(argv)

    try:
        config = load_node_config(args.config)
        task = TaskSpec(args.seed, args.colour, args.instruction)
    except (ValueError, FileNotFoundError) as exc:
        print(f"引数・設定の誤り: {exc}", file=sys.stderr)
        return 2
    dora = find_dora()
    if dora is None:
        print("dora が見つかりません。`uv sync --group dataflow` を実行してください。", file=sys.stderr)
        return 3
    out_dir = (args.out_dir or config.output_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    result_path = out_dir / "result.json"
    result_path.unlink(missing_ok=True)  # 前回の結果を成功と取り違えない

    rendered = render_dataflow(
        DATAFLOW_TEMPLATE_PATH.read_text(encoding="utf-8"), python=sys.executable, config=str(args.config.resolve()),
        seed=task.seed, colour=task.colour, instruction=task.instruction, out_dir=str(out_dir),
    )
    flow_path = out_dir / "dataflow.rendered.yml"
    flow_path.write_text(rendered, encoding="utf-8")

    env = {**os.environ, "PYTHONUTF8": "1", "PYTHONIOENCODING": "utf-8"}
    timeout = args.timeout or config.timeout_s
    try:
        proc = subprocess.run(
            [dora, "run", str(flow_path)], cwd=REPO_ROOT, env=env, text=True, encoding="utf-8",
            errors="replace", capture_output=True, timeout=timeout,
        )
    except subprocess.TimeoutExpired:
        print(f"dora run が {timeout:.0f} 秒で終わりませんでした", file=sys.stderr)
        return 3
    (out_dir / "dora.log").write_text(proc.stdout + "\n" + proc.stderr, encoding="utf-8")
    if not result_path.exists():
        print(f"result.json ができませんでした (dora の終了コード {proc.returncode})。ログ: {out_dir / 'dora.log'}", file=sys.stderr)
        print((proc.stderr or proc.stdout)[-1500:], file=sys.stderr)
        return 3
    result = json.loads(result_path.read_text(encoding="utf-8"))
    state = "成功" if result["success"] else f"失敗 ({result['failure']})"
    print(f"seed={result['seed']} {result['colour']}: {state}  pick 誤差 {result['pick_error_mm']} mm  -> {result_path}")
    return 0 if result["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

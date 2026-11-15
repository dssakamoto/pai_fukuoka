"""プロバイダ別に、同じ条件で評価して比較表を作る (TASK-0029)。

  # 既定: anthropic / openai / gemini を 20 試行ずつ + 参考として oracle (偽 VLM)
  uv run python scripts/compare_providers.py --trials 20 --seed 0
  # 結果を docs/notes にも保存する (レポートに載せる表)
  uv run python scripts/compare_providers.py --notes docs/notes/provider-comparison.md

全プロバイダで、シード (--seed + i)・色の並び・指示が同じ。API キーのないプロバイダ・クライアントが未実装のプロバイダは
「未評価」(成功率 0% ではなく、評価していない) として表に残す。評価の途中で想定外の例外が出たプロバイダは「中断」にして、
他のプロバイダの評価は続ける。出力: results/<run-id>/comparison.md と comparison.json、プロバイダごとの trials.csv。
Gemma (ollama) は、この PC のメモリで止まる懸念があるため、既定では評価しない。--providers に ollama を明示したときだけ評価する。
VLM の呼び出しはキャッシュ (configs/vlm.yaml) されるので、同じ画像・指示で API を呼び直さない。
終了コード: 0 = 完了、1 = --require-target で、基準 (既定 70%) を満たしたプロバイダがない、2 = 引数・設定の誤り。
"""

import argparse
import sys
from datetime import datetime
from pathlib import Path

from so101_vlm.eval.compare import (
    DEFAULT_TARGET,
    ProviderResult,
    compare_providers,
    default_client_factory_for,
    format_markdown,
    write_report,
)
from so101_vlm.eval.runner import DEFAULT_COLOURS, DEFAULT_INSTRUCTION, TrialRecord
from so101_vlm.env import load_dotenv
from so101_vlm.robot.model_paths import REPO_ROOT
from so101_vlm.vlm.config import load_vlm_config

DEFAULT_PROVIDERS = "anthropic,openai,gemini"
DEFAULT_OUT_DIR = REPO_ROOT / "results"


def _progress(provider: str, done: int, total: int, r: TrialRecord) -> None:
    state = "成功" if r.success else f"失敗 ({r.failure})"
    print(f"[{provider} {done}/{total}] seed={r.seed} {r.colour}: {state}", flush=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--providers", default=DEFAULT_PROVIDERS, help="評価するプロバイダ (カンマ区切り。configs/vlm.yaml の名前)")
    parser.add_argument("--no-oracle", action="store_true", help="参考の行 (oracle = 真の配置から作る偽 VLM) をつけない")
    parser.add_argument("--trials", type=int, default=20, help="プロバイダごとの試行数")
    parser.add_argument("--seed", type=int, default=0, help="最初のシード。試行 i は seed + i")
    parser.add_argument("--colours", default=",".join(DEFAULT_COLOURS))
    parser.add_argument("--instruction-template", default=DEFAULT_INSTRUCTION, help="{colour} を色名に置き換える")
    parser.add_argument("--target", type=float, default=DEFAULT_TARGET, help="成功率の基準 (AC-01。既定 0.7)")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    parser.add_argument("--run-id", default=None)
    parser.add_argument("--notes", type=Path, default=None, help="比較表 (Markdown) をこのパスにも書く (既存は上書き)")
    parser.add_argument("--require-target", action="store_true", help="基準を満たしたプロバイダがなければ終了コード 1")
    args = parser.parse_args(argv)
    load_dotenv()  # API キーを .env から (既存の環境変数は上書きしない。値は表示しない)

    names = [p for p in args.providers.split(",") if p]
    config = load_vlm_config()
    unknown = [n for n in names if n not in config.providers and n != "oracle"]
    if unknown or not 0 < args.target <= 1:
        why = (
            f"未知のプロバイダ {unknown} (使えるもの: {sorted(config.providers)})"
            if unknown
            else f"--target は 0 より大きく 1 以下: {args.target}"
        )
        print(f"引数の誤り: {why}", file=sys.stderr)
        return 2
    if not args.no_oracle and "oracle" not in names:
        names = ["oracle", *names]
    run_id = args.run_id or f"{datetime.now():%Y%m%d-%H%M%S}-compare"
    if "/" in run_id or "\\" in run_id or run_id in {"", ".", ".."}:
        print(f"--run-id にパス区切りは使えません: {run_id!r}", file=sys.stderr)
        return 2
    out = args.out_dir / run_id
    if (out / "comparison.md").exists():
        print(f"{out / 'comparison.md'} が既にあります。別の --run-id を指定してください", file=sys.stderr)
        return 2

    plan = {
        "n_trials": args.trials,
        "base_seed": args.seed,
        "colours": tuple(c for c in args.colours.split(",") if c),
        "instruction_template": args.instruction_template,
    }
    try:
        results: list[ProviderResult] = compare_providers(
            names,
            client_factory_for=default_client_factory_for(),
            on_record=_progress,
            model_hints={n: p.model for n, p in config.providers.items()},
            **plan,
        )
    except (ValueError, FileNotFoundError) as exc:
        print(f"引数・設定の誤り: {exc}", file=sys.stderr)
        return 2

    paths = write_report(out, results, target=args.target, **plan)
    text = format_markdown(results, target=args.target, **plan)
    if args.notes is not None:
        args.notes.parent.mkdir(parents=True, exist_ok=True)
        args.notes.write_text(text, encoding="utf-8")
    print()
    print(text)
    print(f"記録: {paths['markdown']}" + (f"  /  {args.notes}" if args.notes else ""))
    met = [r for r in results if r.meets_target(args.target) and not r.reference]
    if args.require_target and not met:
        print(f"成功率 {args.target:.0%} 以上を満たしたプロバイダがありません", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""複数試行を自動実行し、trials.csv と成功率・失敗の分類を出す (TASK-0026)。

  # API キー不要の偽 VLM (真の配置から枠を作る) で 20 試行
  uv run python scripts/run_eval.py --provider oracle --trials 20 --seed 0
  # 実際の VLM (configs/vlm.yaml のプロバイダ。API キーは環境変数)
  uv run python scripts/run_eval.py --provider gemini --trials 20 --seed 0

試行 i のシードは --seed + i、対象の色は --colours を順に回す。出力: results/<run-id>/trials.csv と summary.json
(results/ は git 管理外)。run-id の既定は 日時-プロバイダ。1 試行ごとに CSV に追記するので、途中で止まっても記録は残る。
終了コード: 0 = 完了、1 = --min-success-rate を下回った、2 = 引数・設定・プロバイダの誤り (API キー未設定を含む)。
"""

import argparse
import sys
from datetime import datetime
from pathlib import Path

from so101_vlm.eval.runner import (
    DEFAULT_COLOURS,
    DEFAULT_INSTRUCTION,
    TrialCsv,
    TrialRecord,
    format_summary,
    oracle_client_factory,
    run_trials,
    summarize,
    write_summary,
)
from so101_vlm.perception.calibration import load_calibration
from so101_vlm.robot.model_paths import REPO_ROOT
from so101_vlm.vlm import MissingApiKeyError, create_client
from so101_vlm.vlm.config import load_vlm_config

DEFAULT_OUT_DIR = REPO_ROOT / "results"


def _client_factory(provider: str):
    """oracle は試行ごとに作る。それ以外は VlmClient を 1 つ作って使い回す (キャッシュ・再試行の設定を共有)。"""
    if provider == "oracle":
        return oracle_client_factory(load_calibration())
    client = create_client(load_vlm_config(), provider)
    return lambda seed, colour: client


def _discard_if_empty(out: Path, records: list[TrialRecord]) -> None:
    """1 試行も記録できずに止まったとき、空の trials.csv とディレクトリを残さない。"""
    if records:
        return
    (out / "trials.csv").unlink(missing_ok=True)
    try:
        out.rmdir()
    except OSError:
        pass


def _progress(done: int, total: int, r: TrialRecord) -> None:
    state = "成功" if r.success else f"失敗 ({r.failure})"
    err = "" if r.pick_error_mm is None else f"  誤差 {r.pick_error_mm:.1f} mm"
    print(f"[{done}/{total}] seed={r.seed} {r.colour}: {state}{err}  {r.total_time_s:.1f} s", flush=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--provider", default="oracle", help="oracle (偽 VLM) または configs/vlm.yaml のプロバイダ")
    parser.add_argument("--trials", type=int, default=20)
    parser.add_argument("--seed", type=int, default=0, help="最初のシード。試行 i は seed + i")
    parser.add_argument("--colours", default=",".join(DEFAULT_COLOURS), help="対象の色を順に回す (カンマ区切り)")
    parser.add_argument("--instruction-template", default=DEFAULT_INSTRUCTION, help="{colour} を色名に置き換える")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    parser.add_argument("--run-id", default=None)
    parser.add_argument("--min-success-rate", type=float, default=None, help="下回ったら終了コード 1 (例: 0.7)")
    args = parser.parse_args(argv)

    run_id = args.run_id or f"{datetime.now():%Y%m%d-%H%M%S}-{args.provider}"
    if "/" in run_id or "\\" in run_id or run_id in {"", ".", ".."}:
        print(f"--run-id にパス区切りは使えません: {run_id!r}", file=sys.stderr)
        return 2
    out = args.out_dir / run_id
    if (out / "trials.csv").exists():
        print(f"{out / 'trials.csv'} が既にあります。別の --run-id を指定してください (前回の記録を上書きしません)", file=sys.stderr)
        return 2
    records: list[TrialRecord] = []
    try:
        factory = _client_factory(args.provider)
        with TrialCsv(out / "trials.csv") as csv_out:

            def on_record(done: int, total: int, r: TrialRecord) -> None:
                csv_out.append(r)
                records.append(r)
                _progress(done, total, r)

            run_trials(
                args.trials, args.seed, factory, colours=tuple(c for c in args.colours.split(",") if c),
                instruction_template=args.instruction_template, on_record=on_record,
            )
    except (MissingApiKeyError, ValueError, NotImplementedError, FileNotFoundError) as exc:
        _discard_if_empty(out, records)
        label = "API キーがありません" if isinstance(exc, MissingApiKeyError) else "引数・設定の誤り"
        print(f"{label}: {exc}", file=sys.stderr)
        return 2

    summary = summarize(records)
    write_summary(out / "summary.json", summary)
    print()
    print(format_summary(summary))
    print(f"\n記録: {out / 'trials.csv'}")
    if args.min_success_rate is not None and (summary.success_rate or 0.0) < args.min_success_rate:
        print(f"成功率が {args.min_success_rate:.0%} を下回りました", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

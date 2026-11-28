"""複数試行を自動実行し、trials.csv と成功率・失敗の分類を出す (TASK-0026)。

  # API キー不要の偽 VLM (真の配置から枠を作る) で 20 試行
  uv run python scripts/run_eval.py --provider oracle --trials 20 --seed 0
  # 実際の VLM (configs/vlm.yaml のプロバイダ。API キーは環境変数)
  uv run python scripts/run_eval.py --provider gemini --trials 20 --seed 0

試行 i のシードは --seed + i、対象の色は --colours を順に回す。出力: results/<run-id>/trials.csv と summary.json
(results/ は git 管理外)。run-id の既定は 日時-プロバイダ。1 試行ごとに CSV に追記するので、途中で止まっても記録は残る。
--record-dir を付けると、動いた試行を LeRobot のデータセット (v3.0) にも記録する (uv sync --group record が必要)。
--box-xy X Y で、全試行の箱を指定の位置に動かす (範囲外は動作前に理由つきで拒否、終了コード 2)。実 API を呼ぶ実行は、想定トークンと
料金の目安を表示して確認する (--yes で確認を省く)。--record-dir とは併用できない。
終了コード: 0 = 完了、1 = --min-success-rate を下回った、2 = 引数・設定・プロバイダの誤り (API キー未設定を含む)。
"""

import argparse
import contextlib
import functools
import sys
from datetime import datetime
from pathlib import Path

from so101_vlm.eval.box_option import confirm_api_run, parse_box_xy
from so101_vlm.eval.eval_config import load_eval_config
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
from so101_vlm.eval.scenario import scenario_for
from so101_vlm.eval.trial import run_trial
from so101_vlm.perception.calibration import load_calibration
from so101_vlm.record.lerobot_writer import LeRobotWriter
from so101_vlm.record.sim_episode import record_trial
from so101_vlm.env import load_dotenv
from so101_vlm.robot.model_paths import REPO_ROOT
from so101_vlm.vlm import MissingApiKeyError, create_client
from so101_vlm.vlm.config import load_vlm_config

DEFAULT_OUT_DIR = REPO_ROOT / "results"


def _client_factory(provider: str, box=None):
    """oracle は試行ごとに作る (box は箱の位置の指定)。それ以外は VlmClient を 1 つ作って使い回す (キャッシュ・再試行の設定を共有)。"""
    if provider == "oracle":
        return oracle_client_factory(load_calibration(), box)
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
    parser.add_argument(
        "--record-dir", type=Path, default=None,
        help="動いた試行を LeRobot のデータセットとして記録する (新しいディレクトリ。要 uv sync --group record。1 試行 約 35 秒増える)",
    )
    parser.add_argument("--box-xy", type=float, nargs=2, metavar=("X", "Y"), default=None, help="箱の位置 [m] (ワールド座標)。省略時は XML の位置")
    parser.add_argument("--yes", action="store_true", help="実 API を呼ぶ実行の確認を省く (料金の目安は表示する)")
    parser.add_argument("--min-success-rate", type=float, default=None, help="下回ったら終了コード 1 (例: 0.7)")
    args = parser.parse_args(argv)
    load_dotenv()  # API キーを .env から (既存の環境変数は上書きしない。値は表示しない)

    run_id = args.run_id or f"{datetime.now():%Y%m%d-%H%M%S}-{args.provider}"
    if "/" in run_id or "\\" in run_id or run_id in {"", ".", ".."}:
        print(f"--run-id にパス区切りは使えません: {run_id!r}", file=sys.stderr)
        return 2
    out = args.out_dir / run_id
    if (out / "trials.csv").exists():
        print(f"{out / 'trials.csv'} が既にあります。別の --run-id を指定してください (前回の記録を上書きしません)", file=sys.stderr)
        return 2
    if args.record_dir is not None and args.record_dir.exists():
        print(f"--record-dir {args.record_dir} は既にあります。LeRobot は既存のディレクトリには作れません", file=sys.stderr)
        return 2
    records: list[TrialRecord] = []
    try:
        box = parse_box_xy(args.box_xy)  # 動作の前に検証する (範囲外は理由つきで拒否)
        if box is not None and args.record_dir is not None:
            raise ValueError("--box-xy と --record-dir は併用できません (記録の経路は箱の位置の指定に未対応)")
        if box is not None and not confirm_api_run(args.provider, args.trials, yes=args.yes):
            print("中止しました (API は呼んでいません)", file=sys.stderr)
            return 2
        factory = _client_factory(args.provider, box)
        with contextlib.ExitStack() as stack:
            csv_out = stack.enter_context(TrialCsv(out / "trials.csv"))
            trial_fn = run_trial
            if box is not None:  # 試行ごとの seed で、同じ箱の位置の Scenario を作る
                placement = load_eval_config().placement

                def trial_fn(seed, colour, instruction, vlm, **kw):  # noqa: F811
                    return run_trial(seed, colour, instruction, vlm, scenario=scenario_for(seed, placement, box), **kw)

            if args.record_dir is not None:
                writer = stack.enter_context(LeRobotWriter(args.record_dir))
                trial_fn = functools.partial(record_trial, writer)

            def on_record(done: int, total: int, r: TrialRecord) -> None:
                csv_out.append(r)
                records.append(r)
                _progress(done, total, r)

            run_trials(
                args.trials, args.seed, factory, colours=tuple(c for c in args.colours.split(",") if c),
                instruction_template=args.instruction_template, trial_fn=trial_fn, on_record=on_record,
            )
    except (MissingApiKeyError, ValueError, NotImplementedError, FileNotFoundError, ImportError) as exc:
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

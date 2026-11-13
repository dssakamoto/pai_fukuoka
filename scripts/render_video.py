"""複数の試行をつないで、実行動画 (mp4) を作る (TASK-0028)。

  # 偽 VLM (API キー不要) で 10 試行、2 倍速
  uv run python scripts/render_video.py --provider oracle --trials 10 --speed 2 --out media/demo.mp4
  # 実際の VLM で
  uv run python scripts/render_video.py --provider gemini --trials 10 --speed 2

試行ごとに 指示 -> 検出 (枠) -> 実行 -> 判定 のクリップを作り、最後に成功率のカードをつける。
倍速のときは、動画の右下に「N倍速」と出す (AC-06)。--speed は 30 の約数 (1, 2, 3, 5, 6, 10, ...)。
動画の長さを表示し、1〜3 分 (提出動画の条件) から外れるときは警告する (--trials や --speed で調整する)。
日本語フォントが無い環境では英語の表示になる。--font か環境変数 SO101_VIDEO_FONT で指定できる。
終了コード: 0 = 完了、2 = 引数・設定・プロバイダの誤り (API キー未設定を含む)。
"""

import argparse
import functools
import itertools
import sys
from pathlib import Path

from so101_vlm.eval.runner import (
    DEFAULT_COLOURS,
    DEFAULT_INSTRUCTION,
    TrialRecord,
    oracle_client_factory,
    run_trials,
    summarize,
)
from so101_vlm.eval.scene_spec import SCENE_SPEC
from so101_vlm.eval.video import compose_summary, make_style
from so101_vlm.eval.video_clip import DEFAULT_FPS, VideoWriter, clip_trial, summary_lines
from so101_vlm.perception.calibration import load_calibration
from so101_vlm.robot.model_paths import REPO_ROOT
from so101_vlm.vlm import MissingApiKeyError, create_client
from so101_vlm.vlm.config import load_vlm_config

DEFAULT_OUT = REPO_ROOT / "media" / "demo.mp4"
SUMMARY_HOLD_S = 3.0
MIN_SECONDS, MAX_SECONDS = 60.0, 180.0  # 提出動画の条件 (要件定義書: 1〜3 分)


def _client_factory(provider: str):
    if provider == "oracle":
        return oracle_client_factory(load_calibration())
    client = create_client(load_vlm_config(), provider)
    return lambda seed, colour: client


def _progress(done: int, total: int, r: TrialRecord) -> None:
    state = "成功" if r.success else f"失敗 ({r.failure})"
    print(f"[{done}/{total}] seed={r.seed} {r.colour}: {state}", flush=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--provider", default="oracle", help="oracle (偽 VLM) または configs/vlm.yaml のプロバイダ")
    parser.add_argument("--trials", type=int, default=10)
    parser.add_argument("--seed", type=int, default=0, help="最初のシード。試行 i は seed + i")
    parser.add_argument("--colours", default=",".join(DEFAULT_COLOURS))
    parser.add_argument("--instruction-template", default=DEFAULT_INSTRUCTION, help="{colour} を色名に置き換える")
    parser.add_argument("--speed", type=int, default=2, help="倍速 (30 の約数)。1 なら実時間")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--font", type=Path, default=None, help="日本語フォントのパス (省略時は自動)")
    parser.add_argument("--lang", choices=["ja", "en"], default=None, help="表示言語 (省略時は、日本語フォントがあれば ja)")
    parser.add_argument("--no-summary", action="store_true", help="最後の成功率のカードをつけない")
    parser.add_argument("--overwrite", action="store_true", help="既存の --out を上書きする")
    args = parser.parse_args(argv)

    if args.out.exists() and not args.overwrite:
        print(f"{args.out} が既にあります。別の --out か --overwrite を指定してください", file=sys.stderr)
        return 2
    records: list[TrialRecord] = []
    counter = itertools.count(1)
    try:
        style = make_style(args.font, args.lang)
        factory = _client_factory(args.provider)
        with VideoWriter(args.out, DEFAULT_FPS) as video:

            def trial_fn(seed, colour, instruction, vlm, **kw):
                return clip_trial(
                    video, seed, colour, instruction, vlm, speed=args.speed, style=style,
                    trial_label=f"{next(counter)}/{args.trials}  seed {seed}", **kw,
                )  # fmt: skip

            def on_record(done: int, total: int, r: TrialRecord) -> None:
                records.append(r)
                _progress(done, total, r)

            run_trials(
                args.trials, args.seed, factory, colours=tuple(c for c in args.colours.split(",") if c),
                instruction_template=args.instruction_template, trial_fn=trial_fn, on_record=on_record,
            )
            if not args.no_summary:
                card = compose_summary(
                    (SCENE_SPEC.image_height, SCENE_SPEC.image_width), summary_lines(summarize(records), style.lang), style
                )
                for _ in range(round(SUMMARY_HOLD_S * DEFAULT_FPS)):
                    video.append(card)
            seconds = video.duration_s
    except (MissingApiKeyError, ValueError, NotImplementedError, FileNotFoundError) as exc:
        if not records:
            args.out.unlink(missing_ok=True)
        label = "API キーがありません" if isinstance(exc, MissingApiKeyError) else "引数・設定の誤り"
        print(f"{label}: {exc}", file=sys.stderr)
        return 2

    speed_note = "実時間" if args.speed == 1 else f"{args.speed} 倍速"
    print(f"\n動画: {args.out}  ({seconds:.0f} 秒 = {int(seconds // 60)} 分 {seconds % 60:.0f} 秒、{speed_note})")
    if not MIN_SECONDS <= seconds <= MAX_SECONDS:
        print(f"注意: 提出動画の条件 (1〜3 分) から外れています。--trials か --speed で調整してください", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

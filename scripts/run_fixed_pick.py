"""固定位置の pick & place (VLM なし) を実行し、判定・動画・結果 JSON を保存する (TASK-0013)。

ブロックの真の位置から IkPolicy で軌道を作り、安全フィルタ経由で sim に送り、箱に入ったかを判定する。

  1 試行(動画つき):
    uv run python scripts/run_fixed_pick.py --seed 0 --colour red --headless --video media/fixed_pick_red_seed0.mp4
  20 seed x 3 色の掃引(動画なし):
    uv run python scripts/run_fixed_pick.py --colour all --n-seeds 20 --seed 0 --no-video

標準出力に 1 試行 1 行と集計を出し、<out-dir>/fixed_pick_summary.json に全試行の結果を書く。
終了コード: 計画可能な試行が全て成功なら 0、実行の失敗が 1 件でもあれば 1、引数の誤りは 2。
計画失敗(IK が解けない配置)は FR-06 どおり動作前の拒否なので、それだけでは 1 にしない。
"""

import argparse
import json
from collections.abc import Sequence
from pathlib import Path

import mujoco
import numpy as np

from so101_vlm.eval.fixed_pick import COLOUR_TO_BODY, FixedPickOutcome, run_fixed_pick
from so101_vlm.eval.fixed_pick_report import format_summary, format_trial_line, outcome_to_dict

# 【定数定義】: 出力先・ファイル名 🟡
DEFAULT_OUT_DIR = Path("media/fixed_pick")
SUMMARY_NAME = "fixed_pick_summary.json"
ALL_COLOURS = "all"
# 【定数定義】: 動画 (scripts/view_so101.py と同じ方式)。制御 50 Hz の 2 周期に 1 フレーム = 25 fps 🟡
FPS = 25
FRAME_STRIDE = 2
WIDTH, HEIGHT = 640, 480
# 【定数定義】: 録画用の free camera。作業エリア・箱・腕が全て入る構図 (pick-survey.md 2.5 節) 🟡
CAMERA_LOOKAT = (0.22, 0.08, 0.05)
CAMERA_DISTANCE = 0.62
CAMERA_AZIMUTH = 215.0
CAMERA_ELEVATION = -28.0
# 【定数定義】: 運搬中の PNG を撮る区間 (その区間の中点のフレーム) 🟡
CARRY_LABEL = "pre_place"


def default_video_path(out_dir: Path, colour: str, seed: int) -> Path:
    """【機能概要】: 動画の既定の保存先 <out-dir>/fixed_pick_<colour>_seed<seed>.mp4 🟡"""
    return Path(out_dir) / f"fixed_pick_{colour}_seed{seed}.mp4"


def default_carry_png_path(out_dir: Path, colour: str, seed: int) -> Path:
    """【機能概要】: 運搬中の PNG の保存先 <out-dir>/fixed_pick_<colour>_seed<seed>_carry.png 🟡"""
    return Path(out_dir) / f"fixed_pick_{colour}_seed{seed}_carry.png"


def _free_camera() -> mujoco.MjvCamera:
    """【機能概要】: 録画用の free camera を作る (XML と知覚用の cam_top は変えない) 🟡"""
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = CAMERA_LOOKAT
    camera.distance = CAMERA_DISTANCE
    camera.azimuth = CAMERA_AZIMUTH
    camera.elevation = CAMERA_ELEVATION
    return camera


def _encode_png(frame: np.ndarray) -> bytes:
    """【機能概要】: フレームを PNG のバイト列にする (imageio.imwrite でファイルに書くのと同じ符号化)🟡"""
    import imageio.v2 as imageio

    return bytes(imageio.imwrite("<bytes>", frame, format="PNG"))


class VideoRecorder:
    """【機能概要】: frame_sink として使い、オフスクリーン描画で mp4 と運搬中の PNG を保存する 🟡

    最初の呼び出しでレンダラと書き出しを作る(計画失敗で呼ばれなければ何も書かない)。
    close() で mp4 を閉じ、pre_place 区間の中点のフレームを PNG にする。
    """

    def __init__(self, video_path: Path, carry_png_path: Path) -> None:
        self._video_path, self._carry_png_path = video_path, carry_png_path
        self._renderer: mujoco.Renderer | None = None
        self._writer = None
        self._camera = _free_camera()
        # 【メモリ削減】: 運搬中のフレームは PNG のバイト列で持つ (生の配列 約 0.9 MB/枚 -> 数十 KB/枚)
        self._carry_pngs: list[bytes] = []

    def __call__(self, model: mujoco.MjModel, data: mujoco.MjData, index: int, label: str) -> None:
        if index % FRAME_STRIDE:
            return
        if self._renderer is None:
            self._open(model)
        self._renderer.update_scene(data, camera=self._camera)
        frame = self._renderer.render()
        self._writer.append_data(frame)
        if label == CARRY_LABEL:
            self._carry_pngs.append(_encode_png(frame))

    def _open(self, model: mujoco.MjModel) -> None:
        import imageio.v2 as imageio

        self._video_path.parent.mkdir(parents=True, exist_ok=True)
        self._renderer = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)
        self._writer = imageio.get_writer(
            str(self._video_path), fps=FPS, codec="libx264", macro_block_size=1
        )

    def close(self) -> None:
        """【機能概要】: 書き出しを閉じ、運搬中の PNG を保存する(frame が無ければ何もしない)🟡"""
        if self._renderer is None:
            return
        self._writer.close()
        self._renderer.close()
        if self._carry_pngs:
            self._carry_png_path.parent.mkdir(parents=True, exist_ok=True)
            self._carry_png_path.write_bytes(self._carry_pngs[len(self._carry_pngs) // 2])


def _non_negative_int(text: str) -> int:
    """【機能概要】: argparse 用。0 以上の整数 🟡"""
    value = int(text)
    if value < 0:
        raise argparse.ArgumentTypeError(f"0 以上の整数を指定してください: {text}")
    return value


def _positive_int(text: str) -> int:
    """【機能概要】: argparse 用。1 以上の整数 🟡"""
    value = int(text)
    if value < 1:
        raise argparse.ArgumentTypeError(f"1 以上の整数を指定してください: {text}")
    return value


def build_parser() -> argparse.ArgumentParser:
    """【機能概要】: 引数の定義 (7.1 節)。--headless は受け付けるだけ(常にオフスクリーン)🔴"""
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("--seed", type=_non_negative_int, default=0, help="最初の seed (既定 0)")
    parser.add_argument("--n-seeds", type=_positive_int, default=1, help="seed N..N+K-1 を順に実行")
    parser.add_argument(
        "--colour", choices=(*COLOUR_TO_BODY, ALL_COLOURS), default="red", help="対象 (既定 red)"
    )
    parser.add_argument("--video", type=Path, default=None, help="動画の保存先 (1 試行のときだけ)")
    parser.add_argument("--no-video", action="store_true", help="動画を書かない")
    parser.add_argument("--headless", action="store_true", help="受け付けるだけ (常にオフスクリーン)")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR, help="出力先 (既定 media/fixed_pick)")
    return parser


def parse_args(argv: Sequence[str] | None) -> tuple[argparse.Namespace, tuple[str, ...]]:
    """【機能概要】: 引数を解釈し、組み合わせの誤りは終了コード 2 (parser.error) にする 🟡"""
    parser = build_parser()
    args = parser.parse_args(argv)
    colours = tuple(COLOUR_TO_BODY) if args.colour == ALL_COLOURS else (args.colour,)
    if args.video is not None and args.no_video:
        parser.error("--video と --no-video は同時に指定できません")
    if args.video is not None and args.n_seeds * len(colours) > 1:
        parser.error("--video は 1 試行のときだけ指定できます (--n-seeds 1 かつ 色を 1 つ)")
    return args, colours


def _run_one(args: argparse.Namespace, colour: str, seed: int, with_video: bool) -> FixedPickOutcome:
    """【機能概要】: 1 試行を実行する。動画ありなら VideoRecorder を frame_sink に渡す 🟡"""
    if not with_video:
        return run_fixed_pick(seed, colour)
    video = args.video or default_video_path(args.out_dir, colour, seed)
    recorder = VideoRecorder(video, default_carry_png_path(args.out_dir, colour, seed))
    try:
        return run_fixed_pick(seed, colour, frame_sink=recorder)
    finally:
        recorder.close()


def main(argv: Sequence[str] | None = None) -> int:
    """【機能概要】: CLI の入口。終了コードを返す (引数の誤りは SystemExit(2))🟡"""
    args, colours = parse_args(argv)
    n_trials = args.n_seeds * len(colours)
    # 【動画の既定】: 1 試行のときだけ既定で書く。掃引 (複数試行) は書かない 🟡
    with_video = not args.no_video and n_trials == 1
    outcomes = []
    for seed in range(args.seed, args.seed + args.n_seeds):
        for colour in colours:
            outcome = _run_one(args, colour, seed, with_video)
            print(format_trial_line(outcome), flush=True)
            outcomes.append(outcome)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    summary_path = args.out_dir / SUMMARY_NAME
    summary_path.write_text(
        json.dumps([outcome_to_dict(o) for o in outcomes], ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print(format_summary(outcomes))
    print(f"結果を書き出しました: {summary_path}")
    failed = any(o.execution is not None and not o.success for o in outcomes)
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())

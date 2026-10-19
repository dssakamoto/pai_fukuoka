"""SO-101 を MuJoCo で表示し、6 関節を順に動かして確認するスクリプト。

  動画に書き出す(画面なし):
    uv run python scripts/view_so101.py --headless --out media/so101_sweep.mp4 --seconds 12
  ビューア(macOS は mjpython が必要):
    uv run mjpython scripts/view_so101.py
"""

import argparse
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

from so101_vlm.robot.model_paths import SO101_SCENE_PATH
from so101_vlm.robot.sweep import sweep_targets

FPS = 30
WIDTH, HEIGHT = 640, 480
MJPYTHON_COMMAND = "uv run mjpython scripts/view_so101.py"


def _load():
    model = mujoco.MjModel.from_xml_path(str(SO101_SCENE_PATH))
    return model, mujoco.MjData(model)


def _advance(model, data, t):
    data.ctrl[:] = sweep_targets(model.actuator_ctrlrange, t)
    mujoco.mj_step(model, data)


def run_headless(out: Path, seconds: float) -> Path:
    import imageio.v2 as imageio

    model, data = _load()
    out.parent.mkdir(parents=True, exist_ok=True)
    steps_per_frame = max(1, round(1.0 / (FPS * model.opt.timestep)))
    n_frames = int(seconds * FPS)
    with mujoco.Renderer(model, height=HEIGHT, width=WIDTH) as renderer, imageio.get_writer(
        str(out), fps=FPS, codec="libx264", macro_block_size=1
    ) as writer:
        for _ in range(n_frames):
            for _ in range(steps_per_frame):
                _advance(model, data, data.time)
            renderer.update_scene(data)
            writer.append_data(renderer.render())
    return out


def run_interactive() -> None:
    import mujoco.viewer

    if sys.platform == "darwin" and getattr(mujoco.viewer, "_MJPYTHON", None) is None:
        print(
            "macOS ではパッシブビューアに mjpython が必要です。次のコマンドで起動してください:\n"
            f"  {MJPYTHON_COMMAND}\n"
            "画面なしで確認する場合: uv run python scripts/view_so101.py --headless",
            file=sys.stderr,
        )
        sys.exit(1)
    model, data = _load()
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            start = time.time()
            _advance(model, data, data.time)
            viewer.sync()
            time.sleep(max(0.0, model.opt.timestep - (time.time() - start)))


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument("--headless", action="store_true", help="動画に書き出す")
    p.add_argument("--out", type=Path, default=Path("media/so101_sweep.mp4"))
    p.add_argument("--seconds", type=float, default=12.0)
    args = p.parse_args()
    if args.seconds <= 0:
        p.error("--seconds は正の値を指定してください")
    if args.headless:
        path = run_headless(args.out, args.seconds)
        print(f"動画を書き出しました: {path.resolve()}")
    else:
        run_interactive()


if __name__ == "__main__":
    main()

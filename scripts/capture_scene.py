"""ピック&プレースシーンを撮影し、RGB と深度を保存する。

  uv run python scripts/capture_scene.py [--out-dir media]
"""

import argparse
from pathlib import Path

import mujoco
import numpy as np

from so101_vlm.eval.scene_spec import SCENE_SPEC
from so101_vlm.nodes.camera import render_rgbd
from so101_vlm.robot.model_paths import PICKPLACE_SCENE_PATH

SETTLE_SECONDS = 1.0


def capture(out_dir: Path) -> list[Path]:
    import imageio.v2 as imageio

    model = mujoco.MjModel.from_xml_path(str(PICKPLACE_SCENE_PATH))
    data = mujoco.MjData(model)
    for _ in range(round(SETTLE_SECONDS / model.opt.timestep)):
        mujoco.mj_step(model, data)
    rgb, depth = render_rgbd(
        model, data, SCENE_SPEC.camera_name, SCENE_SPEC.image_width, SCENE_SPEC.image_height
    )
    out_dir.mkdir(parents=True, exist_ok=True)
    paths = [out_dir / "scene_rgb.png", out_dir / "scene_depth.npy", out_dir / "scene_depth.png"]
    imageio.imwrite(paths[0], rgb)
    np.save(paths[1], depth)
    lo, hi = float(depth.min()), float(depth.max())
    norm = (depth - lo) / max(hi - lo, 1e-9)
    imageio.imwrite(paths[2], (255 * (1.0 - norm)).astype(np.uint8))  # 近いほど明るい
    return paths


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path, default=Path("media"))
    for p in capture(parser.parse_args().out_dir):
        print(p)


if __name__ == "__main__":
    main()

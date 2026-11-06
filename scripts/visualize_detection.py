"""VLM の応答 (JSON) を画像に重ねて保存する目視確認ツール (TASK-0023)。

座標の向きや単位の取り違え (design.md のリスク) を、目で確かめるために使う。

  uv run python scripts/visualize_detection.py --image media/scene_rgb.png \
      --response response.json --coord-format yxyx_norm1000 --out media/detection.png

--response は VLM が返した JSON のテキストファイル。--coord-format は configs/vlm.yaml の coord_format と同じ名前。
"""

import argparse
from pathlib import Path

import imageio.v3 as iio

from so101_vlm.eval.visualize import save_overlay
from so101_vlm.vlm.schema import COORD_FORMATS, PlanError, parse_plan


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--image", type=Path, required=True)
    parser.add_argument("--response", type=Path, required=True)
    parser.add_argument("--coord-format", choices=COORD_FORMATS, required=True)
    parser.add_argument("--out", type=Path, default=Path("media/detection.png"))
    args = parser.parse_args(argv)

    image = iio.imread(args.image)[..., :3]
    text = args.response.read_text(encoding="utf-8")
    try:
        plan = parse_plan(text, args.coord_format, image.shape[1], image.shape[0])
    except PlanError as exc:
        print(f"応答を読めません: {exc}")
        return 1
    print(f"pick : {plan.pick}")
    print(f"place: {plan.place}")
    print(save_overlay(args.out, image, plan))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

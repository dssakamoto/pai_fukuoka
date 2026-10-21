"""シーン定義から sim 用キャリブレーション (configs/calibration.yaml) を生成・確認する。

  uv run python scripts/generate_calibration.py [--out PATH] [--check] [--force]

終了コード: 0 成功/一致、1 --check で不一致、2 読み込み不可・real の保護・親ディレクトリなし。
"""

import argparse
import sys
from pathlib import Path

import mujoco

from so101_vlm.eval.scene_spec import SCENE_SPEC
from so101_vlm.perception.calibration import (
    DEFAULT_ATOL,
    DEFAULT_CALIBRATION_PATH,
    Calibration,
    calibration_from_mujoco,
    calibration_max_abs_diff,
    load_calibration,
    save_calibration,
)
from so101_vlm.robot.model_paths import PICKPLACE_SCENE_PATH

EXIT_OK = 0
EXIT_MISMATCH = 1
EXIT_ERROR = 2


def build_calibration() -> Calibration:
    """【機能概要】: シーンを読み、mj_resetData -> mj_forward の後で sim の Calibration を作る 🟡"""
    model = mujoco.MjModel.from_xml_path(str(PICKPLACE_SCENE_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    return calibration_from_mujoco(
        model, data, SCENE_SPEC.camera_name, SCENE_SPEC.image_width, SCENE_SPEC.image_height
    )


def _check(out: Path, fresh: Calibration) -> int:
    """【機能概要】: out のファイルを fresh と比べる (書き込まない) 🔴"""
    try:
        existing = load_calibration(out)
    except (OSError, ValueError) as exc:
        print(f"読み込めません: {out}: {exc}", file=sys.stderr)
        return EXIT_ERROR
    diff = calibration_max_abs_diff(existing, fresh)
    if diff <= DEFAULT_ATOL:
        print(f"一致しています: {out}")
        return EXIT_OK
    print(
        f"不一致です (source={existing.source}, 差の最大値={diff:.3e}): {out}\n"
        "再生成: uv run python scripts/generate_calibration.py",
        file=sys.stderr,
    )
    return EXIT_MISMATCH


def _is_real_file(out: Path) -> bool:
    """【機能概要】: out が読めて source: real のファイルか 🔴"""
    try:
        return load_calibration(out).source == "real"
    except (OSError, ValueError):
        return False


def _generate(out: Path, fresh: Calibration, force: bool) -> int:
    """【機能概要】: out へ書く。既存が real なら --force なしでは拒否する 🔴"""
    if not force and _is_real_file(out):
        print(
            f"実機の計測値 (source: real) を上書きしません: {out}\n"
            "上書きするには --force を付けてください。",
            file=sys.stderr,
        )
        return EXIT_ERROR
    try:
        save_calibration(fresh, out)
    except OSError as exc:
        print(f"書き込めません: {out}: {exc}", file=sys.stderr)
        return EXIT_ERROR
    print(f"生成しました: {out}")
    return EXIT_OK


def _build_parser() -> argparse.ArgumentParser:
    """【ヘルパー関数】: コマンドライン引数の定義 (main から分離して短く保つ) 🔴"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--out",
        type=Path,
        default=DEFAULT_CALIBRATION_PATH,
        help="出力先 (既定: configs/calibration.yaml)",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="書き込まず、既存ファイルがシーンと一致するか確認する",
    )
    parser.add_argument(
        "--force", action="store_true", help="source: real のファイルも上書きする"
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """【機能概要】: 引数を解釈し、--check なら比較、それ以外は生成する。終了コードを返す 🔴"""
    args = _build_parser().parse_args(argv)
    fresh = build_calibration()
    if args.check:
        return _check(args.out, fresh)
    return _generate(args.out, fresh, args.force)


if __name__ == "__main__":
    sys.exit(main())

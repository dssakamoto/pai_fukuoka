"""SO-101 の MuJoCo モデルを SO-ARM100 リポジトリから取得して assets/so101/ に置く。

取得元コミットは固定（再取得しても同一内容）。標準ライブラリのみで動く。

    uv run python scripts/fetch_so101_model.py [--force]
"""

from __future__ import annotations

import argparse
import re
import sys
import urllib.error
import urllib.request
from pathlib import Path

REPO = "TheRobotStudio/SO-ARM100"
COMMIT_SHA = "a758567c3978dfeefe282ede0500085a48fe8f78"
MODEL_DIR = "Simulation/SO101"
ROOT_MODEL = "so101_new_calib.xml"
SCENE = "scene.xml"
TIMEOUT_S = 60

REPO_ROOT = Path(__file__).resolve().parents[1]
DEST = REPO_ROOT / "assets" / "so101"


def _url(repo_path: str) -> str:
    return f"https://raw.githubusercontent.com/{REPO}/{COMMIT_SHA}/{repo_path}"


def _download(repo_path: str) -> bytes:
    url = _url(repo_path)
    try:
        with urllib.request.urlopen(url, timeout=TIMEOUT_S) as res:
            return res.read()
    except urllib.error.HTTPError as e:
        raise SystemExit(f"HTTPエラー {e.code}: {url}") from e
    except urllib.error.URLError as e:
        raise SystemExit(f"接続に失敗: {url} ({e.reason})") from e


def _fetch(repo_path: str, dest: Path, force: bool) -> bytes:
    """dest に保存（既存ならスキップ）し、内容を返す。"""
    if dest.exists() and not force:
        print(f"skip   {dest.relative_to(REPO_ROOT)}")
        return dest.read_bytes()
    data = _download(repo_path)
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_bytes(data)
    print(f"fetch  {dest.relative_to(REPO_ROOT)} ({len(data)} bytes)")
    return data


def _referenced_files(model_xml: str) -> list[str]:
    """MJCF が参照するメッシュ(assets/ 下)と include を列挙する。"""
    meshes = [f"assets/{m}" for m in re.findall(r'<mesh[^>]*\bfile="([^"]+)"', model_xml)]
    includes = re.findall(r'<include[^>]*\bfile="([^"]+)"', model_xml)
    return meshes + includes


def fetch_model(force: bool = False) -> None:
    files = [ROOT_MODEL, SCENE]
    model_xml = _fetch(f"{MODEL_DIR}/{ROOT_MODEL}", DEST / ROOT_MODEL, force).decode()
    scene_xml = _fetch(f"{MODEL_DIR}/{SCENE}", DEST / SCENE, force).decode()
    for xml in (model_xml, scene_xml):
        for ref in _referenced_files(xml):
            if ref not in files:
                files.append(ref)
    for rel in files[2:]:
        _fetch(f"{MODEL_DIR}/{rel}", DEST / rel, force)
    _fetch("LICENSE", DEST / "LICENSE", force)


def validate() -> None:
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(DEST / SCENE))
    print(f"OK: scene.xml を読み込めた (njnt={model.njnt}, nu={model.nu})")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--force", action="store_true", help="既存ファイルも再取得する")
    args = parser.parse_args()
    fetch_model(force=args.force)
    validate()
    return 0


if __name__ == "__main__":
    sys.exit(main())

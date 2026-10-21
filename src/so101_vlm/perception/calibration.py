"""sim 用キャリブレーションの自動生成・保存・読み込み (TASK-0009)。

【機能概要】: MuJoCo シーンのカメラ画角と姿勢から、内部パラメータ Camera と外部パラメータ T_base_cam を
計算し、configs/calibration.yaml へ書き出す。後続 (TASK-0024 / TASK-0036) が同じ形式で読み書きする。
【層の分離】: MuJoCo に依存する計算と YAML の入出力はここに置く。projection.py は MuJoCo 非依存のまま。
【規約】: カメラ座標系は OpenCV 規約 (T_base_cam は OpenCV カメラ座標 -> base)。主点は画素中心規約。
信頼性: 🔵 要件・設計・実測に基づく / 🟡 妥当な推測 / 🔴 要件にない決定 (calibration-requirements.md 参照)
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import mujoco
import numpy as np
import yaml

from so101_vlm.eval.validation import require_exact_keys, require_int, require_number
from so101_vlm.perception.projection import Camera, mujoco_cam_to_opencv, validate_T_base_cam
from so101_vlm.robot.model_paths import REPO_ROOT

# 【定数定義】: 既定の保存先 (パッケージ位置から解決) 🔵
DEFAULT_CALIBRATION_PATH = REPO_ROOT / "configs" / "calibration.yaml"
DEFAULT_ATOL = 1e-9  # 【許容差】: --check とドリフトガード共通 (MuJoCo の版差による末尾の桁の揺れを吸収) 🟡
DEFAULT_BASE_SITE = "baseframe"
FORMAT_VERSION = 1
SOURCES = ("sim", "real")
_TOP_KEYS = frozenset({"version", "metadata", "camera", "t_base_cam"})
_META_KEYS = frozenset({"source", "camera_name", "base_frame"})
_CAMERA_KEYS = frozenset({"fx", "fy", "cx", "cy", "width", "height"})
_MATRIX_SIZE = 4
_NO_WRAP = 10**9  # 【整形】: flow 形式の行を折り返さないための幅
_SIM_HEADER = (
    "# 自動生成ファイル (source: sim)。手で編集しないこと。\n"
    "# 再生成: uv run python scripts/generate_calibration.py\n"
    "# 確認:   uv run python scripts/generate_calibration.py --check\n"
)
_REAL_HEADER = (
    "# 実機の計測値 (source: real)。scripts/generate_calibration.py で上書きしないこと。\n"
    "# 内容を確認する場合: uv run python scripts/generate_calibration.py --check (不一致が正常)\n"
    "# 上書きが必要な場合のみ --force を付ける。\n"
)


# ---------------------------------------------------------------- MuJoCo からの計算
def _fixed_camera_id(model: mujoco.MjModel, camera_name: str) -> int:
    """【機能概要】: 固定カメラの id を返す。名前なし・追従カメラ・sensorsize 指定は ValueError 🟡"""
    cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
    if cid < 0:
        raise ValueError(f"カメラが見つかりません: {camera_name!r}")
    if int(model.cam_mode[cid]) != int(mujoco.mjtCamLight.mjCAMLIGHT_FIXED):
        raise ValueError(f"固定カメラのみ対応します (追従カメラは姿勢が変わる): {camera_name!r}")
    if np.any(model.cam_sensorsize[cid]):
        raise ValueError(f"sensorsize 指定のカメラは未対応です (fovy のみ対応): {camera_name!r}")
    return int(cid)


def camera_from_mujoco(model: mujoco.MjModel, camera_name: str, width: int, height: int) -> Camera:
    """【機能概要】: カメラの画角 fovy (度) から Camera を作る。fx=fy=(h/2)/tan(fovy/2) 🔵

    【実装方針】: 画像サイズは引数で受ける (MuJoCo の cam_resolution は描画サイズではない)。
    """
    width = require_int("width", width, 1)
    height = require_int("height", height, 1)
    cid = _fixed_camera_id(model, camera_name)
    fovy = float(model.cam_fovy[cid])
    if not 0.0 < fovy < 180.0:
        raise ValueError(f"fovy は (0, 180) 度で指定してください: {camera_name!r} fovy={fovy}")
    focal = (height / 2.0) / math.tan(math.radians(fovy) / 2.0)
    return Camera(fx=focal, fy=focal, cx=(width - 1) / 2.0, cy=(height - 1) / 2.0, width=width, height=height)


def _pose(xmat: np.ndarray, xpos: np.ndarray) -> np.ndarray:
    """【ヘルパー関数】: MuJoCo の (xmat, xpos) から 4x4 同次変換を新しく作る 🔵"""
    T = np.eye(4)
    T[:3, :3] = np.asarray(xmat, dtype=np.float64).reshape(3, 3)
    T[:3, 3] = xpos
    return T


def t_base_cam_from_mujoco(
    model: mujoco.MjModel, data: mujoco.MjData, camera_name: str, base_site: str = DEFAULT_BASE_SITE
) -> np.ndarray:
    """【機能概要】: T_base_cam = mujoco_cam_to_opencv(inv(T_world_base) @ T_world_cam) を返す 🔵

    【前提】: data は mj_forward 済み (本関数は呼ばない。data を変更しないため)。
    """
    cid = _fixed_camera_id(model, camera_name)
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, base_site)
    if sid < 0:
        raise ValueError(f"site が見つかりません: {base_site!r}")
    if data.cam_xpos.shape[0] != model.ncam or data.site_xpos.shape[0] != model.nsite:
        raise ValueError("data が model と対応していません (別のモデルの data)")
    if not data.cam_xmat[cid].any() or not data.site_xmat[sid].any():
        raise ValueError("data が未計算です。mujoco.mj_forward を先に呼んでください")
    T_world_cam = _pose(data.cam_xmat[cid], data.cam_xpos[cid])
    T_world_base = _pose(data.site_xmat[sid], data.site_xpos[sid])
    return mujoco_cam_to_opencv(np.linalg.inv(T_world_base) @ T_world_cam)


# ---------------------------------------------------------------- Calibration
def _require_name(name: str, value: object) -> str:
    """【ヘルパー関数】: 空でない文字列を要求する 🟡"""
    if not isinstance(value, str) or not value:
        raise ValueError(f"{name} は空でない文字列で指定してください: {name}={value!r}")
    return value


@dataclass(frozen=True, eq=False)
class Calibration:
    """【機能概要】: 内部パラメータと T_base_cam の組。変更不可 (配列は書き込み不可のコピー)。🟡

    eq=False: ndarray を含むため == は同一性。値の比較は calibrations_close を使う。
    """

    camera: Camera
    T_base_cam: np.ndarray
    source: Literal["sim", "real"]
    camera_name: str
    base_frame: str

    def __post_init__(self) -> None:
        if not isinstance(self.camera, Camera):
            raise ValueError(f"camera は Camera で指定してください: {self.camera!r}")
        if not isinstance(self.source, str) or self.source not in SOURCES:
            raise ValueError(f"source は {SOURCES} のどれかで指定してください: source={self.source!r}")
        _require_name("camera_name", self.camera_name)
        _require_name("base_frame", self.base_frame)
        T = validate_T_base_cam(self.T_base_cam)  # 新しい float64 配列
        T.flags.writeable = False
        object.__setattr__(self, "T_base_cam", T)


def calibration_from_mujoco(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    camera_name: str,
    width: int,
    height: int,
    base_site: str = DEFAULT_BASE_SITE,
) -> Calibration:
    """【機能概要】: シーンから source="sim" の Calibration を作る (data は mj_forward 済み) 🔴"""
    return Calibration(
        camera=camera_from_mujoco(model, camera_name, width, height),
        T_base_cam=t_base_cam_from_mujoco(model, data, camera_name, base_site),
        source="sim",
        camera_name=camera_name,
        base_frame=base_site,
    )


def _floats(cal: Calibration) -> np.ndarray:
    """【ヘルパー関数】: 浮動小数の項目を 1 本のベクトルにする 🔴"""
    cam = cal.camera
    return np.concatenate([[cam.fx, cam.fy, cam.cx, cam.cy], cal.T_base_cam.ravel()])


def _discrete(cal: Calibration) -> tuple:
    """【ヘルパー関数】: 完全一致で比べる項目 (文字列・整数) 🔴"""
    return (cal.source, cal.camera_name, cal.base_frame, cal.camera.width, cal.camera.height)


def calibration_max_abs_diff(a: Calibration, b: Calibration) -> float:
    """【機能概要】: 浮動小数項目の差の最大値。文字列・整数が違えば inf を返す 🔴"""
    if _discrete(a) != _discrete(b):
        return math.inf
    return float(np.max(np.abs(_floats(a) - _floats(b))))


def calibrations_close(a: Calibration, b: Calibration, *, atol: float = DEFAULT_ATOL) -> bool:
    """【機能概要】: 文字列・整数は完全一致、浮動小数は差が atol 以下なら True (境界を含む) 🔴"""
    return calibration_max_abs_diff(a, b) <= atol


# ---------------------------------------------------------------- YAML 入出力
class _FlowRow(list):
    """【ヘルパークラス】: YAML で flow 形式 (1 行) に書く行列の行 🔴"""


class _CalibrationDumper(yaml.SafeDumper):
    """【機能概要】: 行列の行だけ flow 形式にする SafeDumper。読み込みは yaml.safe_load のまま 🔴"""

    def ignore_aliases(self, data: object) -> bool:
        return True


_CalibrationDumper.add_representer(
    _FlowRow, lambda dumper, data: dumper.represent_sequence("tag:yaml.org,2002:seq", data, flow_style=True)
)


def _to_document(cal: Calibration) -> dict:
    """【機能概要】: 保存用の dict を作る。numpy の型は Python の型にする (safe_dump が拒否するため) 🔵"""
    cam = cal.camera
    return {
        "version": FORMAT_VERSION,
        "metadata": {"source": cal.source, "camera_name": cal.camera_name, "base_frame": cal.base_frame},
        "camera": {
            "fx": float(cam.fx),
            "fy": float(cam.fy),
            "cx": float(cam.cx),
            "cy": float(cam.cy),
            "width": int(cam.width),
            "height": int(cam.height),
        },
        "t_base_cam": [_FlowRow(row) for row in cal.T_base_cam.tolist()],
    }


def save_calibration(calibration: Calibration, path: str | Path) -> None:
    """【機能概要】: Calibration を YAML (先頭に生成元コメント) で書く。親ディレクトリがなければ FileNotFoundError 🟡"""
    if not isinstance(calibration, Calibration):
        raise ValueError(f"Calibration を指定してください: {type(calibration).__name__}")
    header = _SIM_HEADER if calibration.source == "sim" else _REAL_HEADER
    body = yaml.dump(
        _to_document(calibration),
        Dumper=_CalibrationDumper,
        sort_keys=False,
        default_flow_style=False,
        allow_unicode=True,
        width=_NO_WRAP,
    )
    Path(path).write_text(header + body, encoding="utf-8")


def _parse_matrix(value: object) -> list[list[float]]:
    """【機能概要】: 4x4 の数のリストを検証して返す (bool・文字列・非有限は ValueError) 🟡"""
    if not isinstance(value, list) or len(value) != _MATRIX_SIZE:
        raise ValueError(f"t_base_cam は 4 行のリストで指定してください: {value!r}")
    rows = []
    for i, row in enumerate(value):
        if not isinstance(row, list) or len(row) != _MATRIX_SIZE:
            raise ValueError(f"t_base_cam の各行は 4 個の数で指定してください: t_base_cam[{i}]={row!r}")
        rows.append([require_number(f"t_base_cam[{i}][{j}]", v) for j, v in enumerate(row)])
    return rows


def _parse_camera(section: object) -> Camera:
    """【機能概要】: camera セクションを検証して Camera にする 🟡"""
    cam = require_exact_keys("camera", section, _CAMERA_KEYS)
    return Camera(
        fx=require_number("camera.fx", cam["fx"]),
        fy=require_number("camera.fy", cam["fy"]),
        cx=require_number("camera.cx", cam["cx"]),
        cy=require_number("camera.cy", cam["cy"]),
        width=require_int("camera.width", cam["width"], 1),
        height=require_int("camera.height", cam["height"], 1),
    )


def _parse_document(doc: object) -> Calibration:
    """【機能概要】: safe_load の結果を厳格に検証して Calibration にする 🔴"""
    top = require_exact_keys("calibration", doc, _TOP_KEYS)
    if require_int("version", top["version"], 1) != FORMAT_VERSION:
        raise ValueError(f"version は {FORMAT_VERSION} のみ対応します: version={top['version']!r}")
    meta = require_exact_keys("metadata", top["metadata"], _META_KEYS)
    return Calibration(
        camera=_parse_camera(top["camera"]),
        T_base_cam=np.array(_parse_matrix(top["t_base_cam"]), dtype=np.float64),
        source=meta["source"],
        camera_name=_require_name("metadata.camera_name", meta["camera_name"]),
        base_frame=_require_name("metadata.base_frame", meta["base_frame"]),
    )


def load_calibration(path: str | Path = DEFAULT_CALIBRATION_PATH) -> Calibration:
    """【機能概要】: YAML を yaml.safe_load で読み、厳格に検証して Calibration を返す 🔵

    ファイルがなければ FileNotFoundError、内容の違反は ValueError。
    """
    with open(path, encoding="utf-8") as f:
        try:
            doc = yaml.safe_load(f)
        except yaml.YAMLError as exc:
            raise ValueError(f"YAML として読めません: {path}: {exc}") from exc
    return _parse_document(doc)

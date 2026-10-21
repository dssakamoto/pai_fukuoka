"""画像座標 -> カメラ座標 -> ロボット座標 (base) の変換 (TASK-0008)。

【機能概要】: VLM が返す bbox と深度画像から、baseframe 座標系の 3 次元位置を求める純粋関数群。
【規約】:
  - 画像: u = 列 (右が正)、v = 行 (下が正)。整数の画素座標は画素中心を指す。
    主点は cx=(w-1)/2, cy=(h-1)/2 (画素中心規約)。
  - カメラ座標系: OpenCV 規約 (x 右, y 下, 光軸 +z)。本モジュールの入出力はすべてこの規約。
    MuJoCo のカメラ (x 右, y 上, 光軸 -z) は mujoco_cam_to_opencv で変換する。
  - 深度: z 深度 (光軸方向の距離)。ユークリッド距離ではない。
  - bbox: (x1, y1, x2, y2)。x2, y2 を含まない半開区間 [x1,x2) x [y1,y2)。
  - 表面の点: 深度は物体の上面を見ているので、bbox_to_base_point が返すのは表面の点。
    ブロック中心へは呼び出し側が上面から半辺 (0.015 m) を引いて補正する (base の z は上向き)。
  - 不正入力はすべて ValueError (TypeError は使わない)。入力は変更せず、戻り値は新しい float64 配列。
信頼性: 🔵 要件・設計に基づく / 🟡 妥当な推測 / 🔴 要件にない決定 (projection-requirements.md 参照)
numpy と標準ライブラリだけを import する (MuJoCo 非依存)。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# 【定数】: validate_T_base_cam の許容誤差 🟡
_BOTTOM_ROW_TOL = 1e-9
_ROTATION_TOL = 1e-6
# 【定数】: bbox 中央領域の既定縮小率 (0 < shrink <= 1)。調整可能 🟡
DEFAULT_SHRINK = 0.5
# 【定数】: MuJoCo (y 上, -z 前方) <-> OpenCV (y 下, +z 前方) の軸反転 🟡
_FLIP = np.diag([1.0, -1.0, -1.0, 1.0])


def _is_bool(x: object) -> bool:
    """【ヘルパー関数】: bool / numpy bool か。数値として受理しないために判定する。 🟡"""
    return isinstance(x, (bool, np.bool_))


def _finite_float(x: object, name: str) -> float:
    """【機能概要】: bool・文字列・非数値・非有限を弾いて float にする。 🟡"""
    if _is_bool(x) or isinstance(x, (str, bytes)):
        raise ValueError(f"{name} は有限の実数である必要があります: {x!r}")
    try:
        value = float(x)  # type: ignore[arg-type]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} は有限の実数である必要があります: {x!r}") from exc
    if not math.isfinite(value):
        raise ValueError(f"{name} は有限である必要があります: {x!r}")
    return value


def _positive_int(x: object, name: str) -> int:
    """【ヘルパー関数】: bool を除く正の整数だけを int にする。 🟡"""
    if _is_bool(x) or not isinstance(x, (int, np.integer)) or x <= 0:
        raise ValueError(f"{name} は正の整数である必要があります: {x!r}")
    return int(x)


@dataclass(frozen=True)
class Camera:
    """【機能概要】: ピンホールカメラの内部パラメータ (画素単位)。 🟡

    cx, cy は画素中心規約 ((w-1)/2, (h-1)/2)。画像外の値も許す。
    """

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    def __post_init__(self) -> None:
        for name in ("fx", "fy"):
            if _finite_float(getattr(self, name), name) <= 0:
                raise ValueError(f"{name} は正である必要があります")
        for name in ("cx", "cy"):
            _finite_float(getattr(self, name), name)
        for name in ("width", "height"):
            _positive_int(getattr(self, name), name)


def camera_matrix(camera: Camera) -> np.ndarray:
    """【機能概要】: 内部パラメータ行列 K (3x3, float64) を新しく作る。 🔵"""
    return np.array(
        [[camera.fx, 0.0, camera.cx], [0.0, camera.fy, camera.cy], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def _real_array(a: object, name: str) -> np.ndarray:
    """【機能概要】: 整数・浮動小数の配列だけを float64 の新しい配列へ変換する (bool・複素・文字列は不可)。 🟡"""
    try:
        arr = np.asarray(a)
    except ValueError as exc:
        raise ValueError(f"{name} を数値配列に変換できません") from exc
    if arr.dtype.kind not in "iuf":
        raise ValueError(f"{name} は実数の配列である必要があります (dtype={arr.dtype})")
    return arr.astype(np.float64, copy=True)


def _vec3(p: object, name: str) -> np.ndarray:
    """【ヘルパー関数】: 有限な (3,) 実数ベクトルを float64 の新しい配列にする。 🟡"""
    arr = _real_array(p, name)
    if arr.shape != (3,) or not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} は有限の (3,) ベクトルである必要があります")
    return arr


def validate_T_base_cam(T: object) -> np.ndarray:
    """【機能概要】: 4x4 同次変換 (剛体) を検証し、float64 の新しい配列を返す。 🟡

    条件: 形状 (4,4)・有限・最下行 [0,0,0,1] (1e-9)・R 直交 (1e-6)・det(R)=+1 (1e-6)。
    注意: diag(1,-1,-1,1) の掛け忘れ (det は +1 のまま) はここでは検出できない。
    """
    arr = _real_array(T, "T_base_cam")
    if arr.shape != (4, 4):
        raise ValueError(f"T_base_cam の形状は (4, 4) である必要があります: {arr.shape}")
    if not np.all(np.isfinite(arr)):
        raise ValueError("T_base_cam に NaN / inf が含まれています")
    if np.max(np.abs(arr[3] - [0.0, 0.0, 0.0, 1.0])) > _BOTTOM_ROW_TOL:
        raise ValueError("T_base_cam の最下行が [0, 0, 0, 1] ではありません")
    rot = arr[:3, :3]
    if np.max(np.abs(rot.T @ rot - np.eye(3))) > _ROTATION_TOL:
        raise ValueError("T_base_cam の回転部が直交していません")
    if abs(np.linalg.det(rot) - 1.0) > _ROTATION_TOL:
        raise ValueError("T_base_cam の回転部の det が +1 ではありません (鏡映)")
    return arr


def mujoco_cam_to_opencv(T_base_cam_mj: object) -> np.ndarray:
    """【機能概要】: MuJoCo 規約のカメラ姿勢を OpenCV 規約へ変換する (T @ diag(1,-1,-1,1))。 🟡"""
    return validate_T_base_cam(T_base_cam_mj) @ _FLIP


def pixel_to_camera(u: float, v: float, depth: float, camera: Camera) -> np.ndarray:
    """【機能概要】: 画素 (u, v) と z 深度 [m] からカメラ座標 (OpenCV 規約) の点を求める。 🔵

    画素は浮動小数のまま使い丸めない。範囲外の画素も許す。
    """
    uf, vf = _finite_float(u, "u"), _finite_float(v, "v")
    z = _finite_float(depth, "depth")
    if z <= 0:
        raise ValueError(f"depth は正である必要があります: {depth!r}")
    return np.array(
        [(uf - camera.cx) * z / camera.fx, (vf - camera.cy) * z / camera.fy, z],
        dtype=np.float64,
    )


def camera_to_base(p_cam: object, T_base_cam: object) -> np.ndarray:
    """【機能概要】: カメラ座標の点を base 座標へ変換する (R @ p + t)。 🔵"""
    p = _vec3(p_cam, "p_cam")
    T = validate_T_base_cam(T_base_cam)
    return T[:3, :3] @ p + T[:3, 3]


def project_base_to_pixel(
    p_base: object, T_base_cam: object, camera: Camera
) -> tuple[float, float, float]:
    """【機能概要】: base 座標の点を画素 (u, v) と z 深度へ投影する。画像外でも値を返す。 🟡

    カメラ座標で z <= 0 (カメラの後ろ・同一平面) の点は ValueError。
    """
    p = _vec3(p_base, "p_base")
    T = validate_T_base_cam(T_base_cam)
    x, y, z = T[:3, :3].T @ (p - T[:3, 3])
    if z <= 0:
        raise ValueError(f"点がカメラの前方にありません (カメラ座標 z={z})")
    return (
        float(camera.fx * x / z + camera.cx),
        float(camera.fy * y / z + camera.cy),
        float(z),
    )


def _clipped_bbox(bbox: object, width: int, height: int) -> tuple[float, float, float, float]:
    """【機能概要】: bbox を検証し画像範囲 [0,w]x[0,h] へクリップする。面積 0 は ValueError。 🔴"""
    try:
        items = list(bbox)  # type: ignore[call-overload]
    except TypeError as exc:
        raise ValueError(f"bbox は長さ 4 の列である必要があります: {bbox!r}") from exc
    if len(items) != 4:
        raise ValueError(f"bbox は長さ 4 である必要があります: {len(items)}")
    x1, y1, x2, y2 = (_finite_float(c, "bbox") for c in items)
    if not (x1 < x2 and y1 < y2):
        raise ValueError(f"bbox が空または反転しています: {bbox!r}")
    cx1, cy1 = max(x1, 0.0), max(y1, 0.0)
    cx2, cy2 = min(x2, float(width)), min(y2, float(height))
    if not (cx1 < cx2 and cy1 < cy2):
        raise ValueError(f"bbox が画像外です: {bbox!r}")
    return cx1, cy1, cx2, cy2


def _center_range(lo: float, hi: float, shrink: float) -> tuple[int, int]:
    """【機能概要】: 区間 [lo,hi] の中央 shrink 倍を [floor, ceil) の整数範囲にし、最低 1 画素を保証する。 🟡"""
    mid, half = (lo + hi) / 2, (hi - lo) * shrink / 2
    start = max(math.floor(mid - half), math.floor(lo))
    stop = min(math.ceil(mid + half), math.ceil(hi))
    if stop <= start:
        stop = start + 1
    return start, stop


def _check_shrink(shrink: object) -> float:
    """【ヘルパー関数】: shrink が有限かつ (0, 1] であることを検証する。 🟡"""
    value = _finite_float(shrink, "shrink")
    if not 0.0 < value <= 1.0:
        raise ValueError(f"shrink は (0, 1] である必要があります: {shrink!r}")
    return value


def _depth_with_clipped_bbox(
    depth_image: object, bbox: object, shrink: float
) -> tuple[float, tuple[float, float, float, float]]:
    """【ヘルパー関数】: 代表深度とクリップ済み bbox を一度の検証でまとめて返す。 🔵

    【改善内容】: representative_depth と bbox_to_base_point が _clipped_bbox を二重に呼んでいた重複を解消。
    【検証順序】: shrink -> 深度画像 -> bbox (従来と同じ)。
    """
    shrink_v = _check_shrink(shrink)
    arr = np.asarray(depth_image)
    if arr.ndim != 2 or arr.size == 0 or arr.dtype.kind not in "iuf":
        raise ValueError("深度画像は 1x1 以上の 2 次元の実数配列である必要があります")
    height, width = arr.shape
    clipped = _clipped_bbox(bbox, width, height)
    x1, y1, x2, y2 = clipped
    c0, c1 = _center_range(x1, x2, shrink_v)
    r0, r1 = _center_range(y1, y2, shrink_v)
    region = arr[r0:r1, c0:c1].astype(np.float64)
    valid = region[np.isfinite(region) & (region > 0)]
    if valid.size == 0:
        raise ValueError("bbox 中央領域に有効な深度がありません")
    return float(np.median(valid)), clipped


def representative_depth(
    depth_image: object, bbox: object, *, shrink: float = DEFAULT_SHRINK
) -> float:
    """【機能概要】: bbox 中央領域の有効深度 (有限かつ > 0) の中央値 [m] を返す。 🔵

    有効画素が 0 個なら ValueError (外側へのフォールバックはしない)。
    """
    return _depth_with_clipped_bbox(depth_image, bbox, shrink)[0]


def bbox_to_base_point(
    bbox: object,
    depth_image: object,
    camera: Camera,
    T_base_cam: object,
    *,
    shrink: float = DEFAULT_SHRINK,
) -> np.ndarray:
    """【機能概要】: bbox と深度画像から base 座標の「表面の点」を求める。 🔵

    中心画素はクリップ後 bbox の ((x1+x2-1)/2, (y1+y2-1)/2)。深度は上面を見ているため、
    ブロック中心への補正 (z から 0.015 を引く) は呼び出し側で行う。
    """
    T = validate_T_base_cam(T_base_cam)
    if np.shape(depth_image) != (camera.height, camera.width):
        raise ValueError(
            f"深度画像の形状は {(camera.height, camera.width)} である必要があります: "
            f"{np.shape(depth_image)}"
        )
    z, (x1, y1, x2, y2) = _depth_with_clipped_bbox(depth_image, bbox, shrink)
    u_c, v_c = (x1 + x2 - 1) / 2, (y1 + y2 - 1) / 2
    return camera_to_base(pixel_to_camera(u_c, v_c, z, camera), T)

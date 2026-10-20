"""ブロック初期配置の乱数生成と MjData への適用 (TASK-0006)。

寸法・名前は SCENE_SPEC のみから導く (単一の情報源)。
"""

import math
from collections.abc import Sequence

import mujoco
import numpy as np

from so101_vlm.eval.scene_spec import SCENE_SPEC
from so101_vlm.eval.validation import require_area, require_int, require_min_gap

# 【定数定義】: freejoint の qpos / qvel の要素数 🔵
FREE_QPOS_SIZE = 7
FREE_QVEL_SIZE = 6
# 【定数定義】: 単位クォータニオン (姿勢なし) 🔵
IDENTITY_QUAT = (1.0, 0.0, 0.0, 0.0)


def _check_feasible(n: int, area: float, min_gap: float) -> None:
    """【機能概要】: 対角線より長い間隔は 2 点でも不可能なので事前に弾く 🔴"""
    if n >= 2 and min_gap > area * math.sqrt(2):
        raise ValueError(
            f"min_gap が領域の対角線より長く配置できません: n={n}, area={area}, min_gap={min_gap}"
        )


def _sampling_bounds(area: float) -> tuple[tuple[float, float], tuple[float, float]]:
    """【ヘルパー関数】: 作業範囲の中心を中心とする一辺 area の正方形の (x 範囲, y 範囲) を返す
    【単一責任】: 領域計算のみ (乱数・検証は扱わない純粋関数)。範囲は SCENE_SPEC からのみ導く 🔵
    """
    cx = sum(SCENE_SPEC.work_area_x) / 2
    cy = sum(SCENE_SPEC.work_area_y) / 2
    half = area / 2
    return (cx - half, cx + half), (cy - half, cy + half)


def sample_layout(
    seed: int,
    n: int = 3,
    area: float = 0.20,
    min_gap: float = 0.05,
    max_attempts: int = 10_000,
) -> tuple[tuple[float, float], ...]:
    """【機能概要】: シードから n 個のブロック中心 (x, y) を棄却サンプリングで生成する
    【実装方針】: default_rng(seed) のみ使用 (グローバル RNG 不使用)。引き数の合計が上限
    【テスト対応】: tests/eval/test_layout.py
    🔵 要件定義書 2.1 に基づく
    """
    seed = require_int("seed", seed, 0)
    n = require_int("n", n, 1)
    area = require_area("area", area)
    min_gap = require_min_gap("min_gap", min_gap)
    max_attempts = require_int("max_attempts", max_attempts, 1)
    _check_feasible(n, area, min_gap)

    (x_lo, x_hi), (y_lo, y_hi) = _sampling_bounds(area)
    rng = np.random.default_rng(seed)
    points: list[tuple[float, float]] = []
    for _ in range(max_attempts):
        candidate = (float(rng.uniform(x_lo, x_hi)), float(rng.uniform(y_lo, y_hi)))
        if all(math.dist(candidate, p) >= min_gap for p in points):
            points.append(candidate)
            if len(points) == n:
                return tuple(points)
    raise ValueError(
        f"配置が上限に達しました: n={n}, area={area}, min_gap={min_gap}, max_attempts={max_attempts}"
    )


def _resolve_free_joint(model: mujoco.MjModel, name: str) -> tuple[int, int]:
    """【機能概要】: body 名から freejoint の (qpos 開始, dof 開始) を返す。不正は ValueError 🟡
    【仕様】: body は関節をちょうど 1 つ (freejoint) 持つことを要求する (要件定義書 2.3)。
    関節が 0 個 (固定 body) や複数個の body は、qpos の位置が一意に定まらず
    誤った領域へ書き込む恐れがあるため、あえて拒否する。
    """
    try:
        body = model.body(name)
    except KeyError as exc:
        raise ValueError(f"body が見つかりません: {name}") from exc
    if body.jntnum[0] != 1:
        raise ValueError(f"body が freejoint を 1 つ持っていません: {name}")
    jnt = int(body.jntadr[0])
    if model.jnt_type[jnt] != mujoco.mjtJoint.mjJNT_FREE:
        raise ValueError(f"body の関節が freejoint ではありません: {name}")
    return int(model.jnt_qposadr[jnt]), int(model.jnt_dofadr[jnt])


def place_blocks(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    layout: Sequence[tuple[float, float]],
    block_names: Sequence[str] = SCENE_SPEC.block_names,
) -> None:
    """【機能概要】: layout を各ブロックの freejoint に書き込み mj_forward する
    【実装方針】: 全検証を終えてから書き込む (失敗時 data 不変)。他の qpos/qvel は触らない
    【テスト対応】: tests/eval/test_place_blocks.py
    🟡 要件定義書 2.3 に基づく
    """
    if len(layout) != len(block_names):
        raise ValueError(f"layout の長さ({len(layout)})がブロック数({len(block_names)})と違います")
    addresses = [_resolve_free_joint(model, name) for name in block_names]
    for (qadr, dadr), (x, y) in zip(addresses, layout, strict=True):
        data.qpos[qadr : qadr + FREE_QPOS_SIZE] = (x, y, SCENE_SPEC.block_rest_z, *IDENTITY_QUAT)
        data.qvel[dadr : dadr + FREE_QVEL_SIZE] = 0.0
    mujoco.mj_forward(model, data)

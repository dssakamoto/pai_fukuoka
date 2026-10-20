"""sample_layout (ブロック初期配置の乱数生成) のテスト (TASK-0006, Red フェーズ)。

テストケース定義: docs/implements/so101-vlm-pickplace/TASK-0006/layout-testcases.md
信頼性: 🔵 要件・設計・実測に基づく / 🟡 妥当な推測 / 🔴 要件にない仮定
"""

import inspect
import itertools
import math
import random
import time

import numpy as np
import pytest

from so101_vlm.eval.eval_config import load_eval_config
from so101_vlm.eval.layout import sample_layout
from so101_vlm.eval.scene_spec import SCENE_SPEC

# --- 定数 (testcases.md 0.1) ---
EPS = 1e-9
SWEEP_SEEDS = range(100)
TRIAL_SEEDS = range(20)

_X_LO, _X_HI = SCENE_SPEC.work_area_x
_Y_LO, _Y_HI = SCENE_SPEC.work_area_y
CX = sum(SCENE_SPEC.work_area_x) / 2
CY = sum(SCENE_SPEC.work_area_y) / 2
WIDTH_MIN = min(_X_HI - _X_LO, _Y_HI - _Y_LO)


@pytest.fixture(scope="module")
def sweep_layouts():
    # 【テスト前準備】: シード 0〜99 の配置を 1 回だけ生成し TC-L-05/06 で共有する
    return {seed: sample_layout(seed) for seed in SWEEP_SEEDS}


# =============================================================== 正常系
@pytest.mark.parametrize("seed", [0, 7, 42, 12345])
def test_sample_layout_is_deterministic_for_same_seed(seed):
    """TC-L-01 同じシード・同じ引数なら同じ配置 🔵"""
    # Given: シード / When: 2 回呼ぶ
    first = sample_layout(seed)
    second = sample_layout(seed)
    # Then: 完全一致
    assert first == second  # 【確認内容】: 再現性 (NFR-01)


def test_sample_layout_accepts_numpy_integer_seed():
    """TC-L-02 numpy 整数のシードは int と同じ結果 🟡"""
    assert sample_layout(np.int64(5)) == sample_layout(5)  # 【確認内容】: numpy.integer 受け入れ


def test_sample_layout_differs_for_different_seeds():
    """TC-L-03 異なるシードで配置が変わる 🟡"""
    assert sample_layout(0) != sample_layout(1)  # 【確認内容】: シードが RNG に渡っている


def test_sample_layout_does_not_touch_global_random_state():
    """TC-L-04 グローバル乱数状態を読まない・変えない 🔵"""
    # Given: グローバル状態を固定して保存
    np.random.seed(123)
    random.seed(123)
    np_state = np.random.get_state()
    py_state = random.getstate()
    # When
    after_123 = sample_layout(7)
    # Then: 状態が変わっていない
    new_state = np.random.get_state()
    assert new_state[0] == np_state[0]  # 【確認内容】: アルゴリズム名
    assert np.array_equal(new_state[1], np_state[1])  # 【確認内容】: 状態配列
    assert new_state[2:] == np_state[2:]  # 【確認内容】: pos 等
    assert random.getstate() == py_state  # 【確認内容】: random モジュール
    # 別のグローバルシードでも同じ結果 (グローバル状態を読んでいない)
    np.random.seed(999)
    assert sample_layout(7) == after_123  # 【確認内容】: グローバル状態に依存しない


def test_sample_layout_all_points_inside_work_area(sweep_layouts):
    """TC-L-05 シード 0〜99 で全点が作業範囲内 🔵"""
    for seed, layout in sweep_layouts.items():
        for p in layout:
            x, y = p
            assert _X_LO - EPS <= x <= _X_HI + EPS, f"seed={seed} p={p}"
            assert _Y_LO - EPS <= y <= _Y_HI + EPS, f"seed={seed} p={p}"


def test_sample_layout_all_pairs_at_least_5cm_apart(sweep_layouts):
    """TC-L-06 シード 0〜99 で全組の中心間距離 >= 0.05 🔵"""
    for seed, layout in sweep_layouts.items():
        for a, b in itertools.combinations(layout, 2):
            assert math.dist(a, b) >= 0.05, f"seed={seed} a={a} b={b}"


@pytest.mark.parametrize("n", [1, 3, 5])
def test_sample_layout_returns_tuple_of_float_pairs(n):
    """TC-L-07 戻り値は float 2 要素 tuple の tuple、長さ n 🟡"""
    result = sample_layout(3, n=n)
    assert type(result) is tuple  # 【確認内容】: 外側は tuple
    assert len(result) == n  # 【確認内容】: 長さ n
    for p in result:
        assert type(p) is tuple and len(p) == 2  # 【確認内容】: 内側も tuple
        for v in p:
            assert type(v) is float  # 【確認内容】: np.float64 を返さない


def test_sample_layout_twenty_trials_succeed_and_differ():
    """TC-L-08 評価の 20 試行がすべて成功し互いに異なる 🟡"""
    cfg = load_eval_config()  # configs/eval.yaml が未作成の間は失敗する
    p = cfg.placement
    layouts = [
        sample_layout(
            seed, n=p.n_blocks, area=p.area, min_gap=p.min_gap, max_attempts=p.max_attempts
        )
        for seed in range(cfg.base_seed, cfg.base_seed + cfg.n_trials)
    ]
    assert len(layouts) == 20  # 【確認内容】: 20 試行
    assert len(set(layouts)) == 20  # 【確認内容】: 互いに異なる
    assert all(len(layout) == 3 for layout in layouts)  # 【確認内容】: 各 3 ブロック


def test_sample_layout_small_area_stays_around_center():
    """TC-L-09 area を小さくすると中心 ± area/2 に収まる 🟡"""
    half = 0.10 / 2
    for seed in TRIAL_SEEDS:
        for x, y in sample_layout(seed, area=0.10, min_gap=0.03):
            assert CX - half - EPS <= x <= CX + half + EPS, f"seed={seed} x={x}"
            assert CY - half - EPS <= y <= CY + half + EPS, f"seed={seed} y={y}"


def test_default_area_fits_inside_work_area_width():
    """TC-L-10 既定の area が作業範囲の幅に収まる 🟡"""
    default = inspect.signature(sample_layout).parameters["area"].default
    assert default <= WIDTH_MIN + EPS  # 【確認内容】: 関数既定値
    assert load_eval_config().placement.area <= WIDTH_MIN + EPS  # 【確認内容】: 設定値


# =============================================================== 異常系
def test_sample_layout_impossible_condition_raises_in_finite_time():
    """TC-L-E01 5 cm 四方に 3 個 (間隔 5 cm) は有限時間で ValueError 🟡"""
    start = time.perf_counter()
    with pytest.raises(ValueError):
        sample_layout(0, n=3, area=0.05, min_gap=0.05)
    assert time.perf_counter() - start < 2.0  # 【確認内容】: 無限ループしない


def test_sample_layout_exhaustion_message_has_parameters():
    """TC-L-E02 上限到達のメッセージに n・area・min_gap・試行回数が入る 🟡"""
    with pytest.raises(ValueError) as excinfo:
        sample_layout(0, n=3, area=0.05, min_gap=0.05)
    msg = str(excinfo.value)
    for part in ("n=3", "area=0.05", "min_gap=0.05", "10000"):
        assert part in msg  # 【確認内容】: key=value 形式の値が含まれる 🔴


def test_sample_layout_precheck_rejects_gap_longer_than_diagonal():
    """TC-L-E03 min_gap > area*sqrt(2) かつ n>=2 は引く前に ValueError 🔴"""
    start = time.perf_counter()
    with pytest.raises(ValueError, match="min_gap"):
        sample_layout(0, n=2, area=0.05, min_gap=0.08, max_attempts=10**9)
    assert time.perf_counter() - start < 0.5  # 【確認内容】: 事前判定が効いている


@pytest.mark.parametrize("seed", [-1, True, False, 1.5, "7", None, float("nan")])
def test_sample_layout_rejects_invalid_seed(seed):
    """TC-L-E04 seed の不正 → ValueError 🔴"""
    with pytest.raises(ValueError):
        sample_layout(seed)


@pytest.mark.parametrize("n", [0, -1, True, 1.5, None])
def test_sample_layout_rejects_invalid_n(n):
    """TC-L-E05 n の不正 → ValueError 🟡"""
    with pytest.raises(ValueError):
        sample_layout(0, n=n)


@pytest.mark.parametrize(
    "area",
    [0, 0.0, -0.1, float("nan"), float("inf"), float("-inf"), None, "0.2", True, 0.201, 0.21],
)
def test_sample_layout_rejects_invalid_area(area):
    """TC-L-E06 area の不正 (0.201/0.21 は作業範囲の幅超え) → ValueError 🟡"""
    with pytest.raises(ValueError):
        sample_layout(0, area=area)


@pytest.mark.parametrize("min_gap", [-0.01, float("nan"), float("inf"), None, "0.05", True])
def test_sample_layout_rejects_invalid_min_gap(min_gap):
    """TC-L-E07 min_gap の不正 → ValueError 🟡"""
    with pytest.raises(ValueError):
        sample_layout(0, min_gap=min_gap)


@pytest.mark.parametrize("max_attempts", [0, -1, True, 1.5, None])
def test_sample_layout_rejects_invalid_max_attempts(max_attempts):
    """TC-L-E08 max_attempts の不正 → ValueError 🟡"""
    with pytest.raises(ValueError):
        sample_layout(0, max_attempts=max_attempts)


# =============================================================== 境界値
def test_sample_layout_single_block_ignores_gap():
    """TC-L-B01 n=1 は常に成功し、事前判定は n>=2 のときだけ 🟡"""
    assert len(sample_layout(0, n=1)) == 1
    assert len(sample_layout(0, n=1, area=0.05, min_gap=1.0)) == 1  # 【確認内容】: 事前判定なし


@pytest.mark.parametrize("min_gap", [0, 0.0])
def test_sample_layout_zero_min_gap_succeeds(min_gap):
    """TC-L-B02 min_gap=0 は成功 🟡"""
    layout = sample_layout(0, n=3, min_gap=min_gap)
    assert len(layout) == 3  # 【確認内容】: 長さ 3


def test_sample_layout_area_equal_to_work_area_width_succeeds():
    """TC-L-B03 area = 作業範囲の幅ちょうど (0.20) は成功 🔵"""
    # float 誤差 (0.35-0.15 == 0.19999999999999998) で誤って弾かないこと
    assert len(sample_layout(0, area=0.20)) == 3  # 【確認内容】: 明示指定
    assert len(sample_layout(0)) == 3  # 【確認内容】: 既定値


def test_sample_layout_max_attempts_one_with_one_block_succeeds():
    """TC-L-B04 max_attempts=1, n=1 は成功 🟡"""
    assert len(sample_layout(0, n=1, max_attempts=1)) == 1


def test_sample_layout_max_attempts_is_total_across_blocks():
    """TC-L-B05 max_attempts=2, n=3 は ValueError (合計の上限) 🟡"""
    with pytest.raises(ValueError):
        sample_layout(0, n=3, max_attempts=2)


def test_sample_layout_max_attempts_exactly_enough_succeeds():
    """TC-L-B06 max_attempts=3, n=3, min_gap=0 は成功 (off-by-one 検出) 🟡"""
    assert len(sample_layout(0, n=3, min_gap=0, max_attempts=3)) == 3

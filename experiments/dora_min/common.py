"""camera_dummy と viewer で共有する、検証用パターン生成関数。"""
import numpy as np

SMALL_SHAPE = (48, 64, 3)
BIG_SHAPE = (480, 640, 3)
N_SMALL = 10   # 小画像の送信回数
N_BIG = 20     # 大画像の送信回数
N_JOINT = 10   # 関節ベクトルの送信回数


def make_frame(shape, counter):
    """カウンタ値から決定的に作る画像。受信側で同じ式で再生成して比較する。"""
    n = int(np.prod(shape))
    return ((np.arange(n, dtype=np.int64) + counter * 7) % 256).astype(np.uint8).reshape(shape)


def make_joint(counter):
    """SO-101 の 6 関節(5 + グリッパ)を想定した float64 ベクトル。"""
    return np.arange(6, dtype=np.float64) * 0.1 + counter

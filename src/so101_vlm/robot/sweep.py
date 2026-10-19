"""確認用の関節スイープ指令(純粋関数)。

関節を 1 つずつ順に、可動域の中心まわりで正弦波駆動する。他の関節は中心に置く。
"""

import numpy as np


def sweep_targets(
    ctrlrange: np.ndarray,
    t: float,
    period: float = 2.0,
    amplitude_fraction: float = 0.5,
) -> np.ndarray:
    """時刻 t の目標位置(n,)を返す。入力は変更しない。

    各関節は period 秒ずつ順番に駆動される(1 周期で 1 回振れる)。
    """
    if period <= 0:
        raise ValueError("period must be positive")
    cr = np.asarray(ctrlrange, dtype=float)
    if cr.ndim != 2 or cr.shape[1] != 2:
        raise ValueError("ctrlrange must have shape (n, 2)")
    frac = float(np.clip(amplitude_fraction, 0.0, 1.0))
    center = (cr[:, 0] + cr[:, 1]) / 2.0
    half = (cr[:, 1] - cr[:, 0]) / 2.0
    n = cr.shape[0]
    active = int(t // period) % n
    phase = 2.0 * np.pi * ((t % period) / period)
    offset = np.zeros(n)
    offset[active] = frac * half[active] * np.sin(phase)
    return np.clip(center + offset, cr[:, 0], cr[:, 1])

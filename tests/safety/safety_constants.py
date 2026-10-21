"""safety テスト共有の定数とヘルパ (TASK-0010, Refactor フェーズ)。

test_limits.py と test_apply_safety.py で重複していた定義を集約しただけ (値は変更なし)。
値の根拠: docs/implements/so101-vlm-pickplace/TASK-0010/safety-testcases.md 0.1
"""

import numpy as np

from so101_vlm.safety.config import SafetyConfig, WorkspaceBox

# 【合成の可動域・速度】: 実モデルに依存しない値 (testcases 0.1)
VMAX = np.full(6, 1.0)
DT = 0.02
LOWER_SYN = np.array([-1.0, -1.0, -1.0, -1.0, -2.0, 0.0])
UPPER_SYN = np.array([1.0, 1.0, 1.0, 1.0, 2.0, 1.5])
# 【作業範囲】: configs/motion.yaml の workspace と同じ値
WS_LOWER = (0.12, -0.13, 0.00)
WS_UPPER = (0.38, 0.26, 0.17)
EXACT_ABS = 1e-12
BOX = WorkspaceBox(lower=WS_LOWER, upper=WS_UPPER)
DEFAULT_CFG = SafetyConfig(
    vmax=(1.0,) * 6, control_dt=0.02, speed_scale=1.0, state_timeout=0.2, workspace=BOX
)
Z6 = np.zeros(6)
NAN, INF = float("nan"), float("inf")


def _arr(*vals):
    """【ヘルパー】: 可変長の数値から float64 配列を作る"""
    return np.array(vals, dtype=np.float64)

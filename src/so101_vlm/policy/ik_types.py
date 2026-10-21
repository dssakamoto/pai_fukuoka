"""IK の結果型と例外 (TASK-0011)。

【機能概要】: solve_ik の戻り値 IkSolution と、失敗時の IkError を定義する。
🔵 信頼性: ik-requirements.md 2.1 節に基づく。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

REASON_OUT_OF_REACH = "out_of_reach"
REASON_NOT_CONVERGED = "not_converged"


@dataclass(frozen=True)
class IkSolution:
    """【機能概要】: IK の解。変更不可。🔵

    q: 関節角 [rad] (6,)、JOINT_NAMES 順。生成時にコピーされ読み取り専用になる。
    pitch_deg: 解での実際の傾き [度]。pitch_candidate_deg: 採用した候補 [度]。
    pos_error: 解での位置誤差 [m]。
    """

    q: np.ndarray
    pitch_deg: float
    pitch_candidate_deg: float
    pos_error: float

    def __post_init__(self) -> None:
        # 【不変化】: 呼び出し側の配列と共有せず、書き込み不可にする
        q = np.array(self.q, dtype=np.float64, copy=True)
        q.flags.writeable = False
        object.__setattr__(self, "q", q)
        object.__setattr__(self, "pitch_deg", float(self.pitch_deg))
        object.__setattr__(self, "pitch_candidate_deg", float(self.pitch_candidate_deg))
        object.__setattr__(self, "pos_error", float(self.pos_error))


class IkError(RuntimeError):
    """【機能概要】: IK が解を返せないときの例外 (FR-06)。🔵

    reason は "out_of_reach"(到達球の外)か "not_converged"(収束せず)。
    """

    def __init__(
        self,
        target: tuple[float, float, float],
        reason: str,
        best_pos_error: float,
        best_pitch_deg: float | None,
        tried_pitches_deg: tuple[float, ...],
        message: str,
    ) -> None:
        super().__init__(message)
        self.target = target
        self.reason = reason
        self.best_pos_error = best_pos_error
        self.best_pitch_deg = best_pitch_deg
        self.tried_pitches_deg = tried_pitches_deg

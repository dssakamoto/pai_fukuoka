"""到達性の回帰テスト (TASK-0011, Red フェーズ)。

テストケース定義: docs/implements/so101-vlm-pickplace/TASK-0011/ik-testcases.md (T36, T37)
信頼性: 🟡 到達性調査 (reachability-survey.md) と参照ソルバの実測に基づく

T37 (判定しない記録): (0.25, 0.0, 0.095) や把持高さ x = 0.25〜0.30 の点は、
候補が 0° か 10° のどちらでも仕様どおり (参照ソルバでは (0.25, 0, 0.095) は
候補 10°、傾き 14.0°)。解けること自体は堅いが、候補の値は追加シードの選び方で
揺れるため、ここでは判定しない。
"""

import mujoco
import numpy as np
import pytest

from so101_vlm.eval.scene_spec import SCENE_SPEC
from so101_vlm.policy.ik import solve_ik
from so101_vlm.robot.model_paths import JOINT_NAMES, PICKPLACE_SCENE_PATH

POS_TOL_TEST_M = 0.002
MAX_SURVEY_PITCH_CANDIDATE_DEG = 30.0
BOX_X, BOX_Y = SCENE_SPEC.box_center_xy

SURVEY_POINTS = {
    "pregrasp_center_high": (0.25, 0.00, 0.14),
    "box_release": (BOX_X, BOX_Y, 0.06),
    "box_preplace": (BOX_X, BOX_Y, 0.14),
    "far_corner_grasp": (0.35, -0.10, 0.015),
    "far_corner_approach": (0.35, 0.10, 0.095),
    "far_center_approach": (0.35, 0.00, 0.095),
    "near_corner_grasp": (0.15, -0.10, 0.015),
    "near_corner_approach": (0.15, 0.10, 0.095),
}


@pytest.fixture(scope="module")
def scene_model():
    # 【テスト前準備】: モデル読み込みは重いのでモジュールで 1 回だけ行う
    return mujoco.MjModel.from_xml_path(str(PICKPLACE_SCENE_PATH))


@pytest.mark.parametrize("target", list(SURVEY_POINTS.values()), ids=list(SURVEY_POINTS))
def test_survey_points_solvable_with_default_candidates(scene_model, target):
    # 【テスト目的】: 作業範囲・箱の代表点が既定の候補 (<= 30°) で解ける (T36) 🟡
    # Given: seed は 0 姿勢、候補は既定
    # When: 解く (IkError が出れば失敗)
    sol = solve_ik(scene_model, target, np.zeros(6))
    ctrl = np.array(
        [scene_model.actuator_ctrlrange[scene_model.actuator(n).id] for n in JOINT_NAMES]
    )
    assert sol.pos_error <= POS_TOL_TEST_M  # 【確認内容】位置誤差が許容内
    assert sol.pitch_candidate_deg <= MAX_SURVEY_PITCH_CANDIDATE_DEG  # 【確認内容】調査の結論 (30° 以下)
    assert np.all(sol.q >= ctrl[:, 0]) and np.all(sol.q <= ctrl[:, 1])  # 【確認内容】可動域内

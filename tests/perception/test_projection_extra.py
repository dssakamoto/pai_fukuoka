"""projection.py の防御的分岐を補う追加テスト (TASK-0008, Refactor フェーズ)。

既存テストは変更せず、未到達だった分岐 (非数値オブジェクト・不整形配列・極小 shrink) だけを検証する。
"""

import numpy as np
import pytest

from so101_vlm.perception.projection import (
    Camera,
    pixel_to_camera,
    representative_depth,
    validate_T_base_cam,
)

CAMERA = Camera(fx=100.0, fy=100.0, cx=4.5, cy=4.5, width=10, height=10)


@pytest.mark.parametrize("bad", [None, [1.0], object(), 1 + 2j])
def test_非数値オブジェクトのu座標はValueErrorになる(bad):
    with pytest.raises(ValueError):
        pixel_to_camera(bad, 1.0, 1.0, CAMERA)


def test_不整形配列のT_base_camはValueErrorになる():
    with pytest.raises(ValueError, match="数値配列"):
        validate_T_base_cam([[1.0, 0.0], [0.0]])


def test_極小shrinkでも最低1画素を使って深度を返す():
    depth = np.full((10, 10), 0.5)
    assert representative_depth(depth, (1, 1, 3, 3), shrink=1e-300) == 0.5

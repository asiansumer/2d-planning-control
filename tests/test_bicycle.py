import math

import pytest

from bicycle import step


def test_straight():
    # 车在 (0, 0)，朝右（0），车速 2，方向盘回正，走一步（dt 默认 0.1）
    assert step(0, 0, 0, 2, 0) == pytest.approx((0.2, 0, 0))


def test_turn_left():
    # 车在 (0, 0)，朝右（0），车速 2.5，方向盘左转 45°（tan = 1），走一步（L 默认 2.5）
    new_x, new_y, new_theta = step(0, 0, 0, 2.5, math.pi / 4)
    assert new_theta == pytest.approx(0.1)

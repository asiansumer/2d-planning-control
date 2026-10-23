import math

import pytest

from bicycle import step


def test_straight():
    # 朝右、车速 2、转角 0，走一步（dt = 0.1）：前进 0.2 米
    assert step(0, 0, 0, 2, 0) == pytest.approx((0.2, 0, 0))


def test_turn_left():
    # 车速 2.5、左转 45°（tan = 1）、轴距 2.5：朝向变化 2.5 × 0.1 / 2.5 × 1 = 0.1
    new_x, new_y, new_theta = step(0, 0, 0, 2.5, math.pi / 4)
    assert new_theta == pytest.approx(0.1)

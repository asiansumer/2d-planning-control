import math

import pytest

from geometry import move_forward, is_collision

def test_move_up():
    # 车在 (1, 1)，朝上（π/2），走 2 米，应该到 (1, 3)
    assert move_forward(1, 1, math.pi / 2, 2) == pytest.approx((1, 3))


def test_move_left():
    # 车在 (0, 0)，朝左（π），走 2 米，应该到 (-2, 0)
    assert move_forward(0, 0, math.pi, 2) == pytest.approx((-2, 0))


def test_is_collision():
    obstacles = [(5, 5, 1)]
    assert is_collision(5, 5.5, obstacles) == True  # 在圆里面
    assert is_collision(8, 8, obstacles) == False   # 离得很远
    assert is_collision(6, 5, obstacles) == True     # 正好在边上

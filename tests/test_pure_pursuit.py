import math

import pytest

from pure_pursuit import find_target, nearest_index, steer_to_point


def test_straight_ahead():
    # 预瞄点在正前方：不用打方向
    assert steer_to_point(0, 0, 0, 5, 0) == pytest.approx(0)


def test_left_30():
    # 预瞄点在左前方 30°、距离 5 米：tan(delta) = 2 × 2.5 × sin 30° / 5 = 0.5
    tx = 5 * math.cos(math.pi / 6)
    ty = 5 * math.sin(math.pi / 6)
    assert steer_to_point(0, 0, 0, tx, ty) == pytest.approx(math.atan(0.5))


def test_find_target():
    path = [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (5, 0), (6, 0)]
    assert nearest_index(1.2, 0.5, path) == 1
    assert find_target(1.2, 0.5, path, 3) == (5, 0)

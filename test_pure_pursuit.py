import math

import pytest

from pure_pursuit import steer_to_point, nearest_index, find_target


def test_straight_ahead():
    # 点就在正前方 5 米：方向盘不用打
    assert steer_to_point(0, 0, 0, 5, 0) == pytest.approx(0)


def test_left_30():
    # 点在左前方 30°、距离 5 米，也就是刚才那道小题：tan(delta) = 0.5
    tx = 5 * math.cos(math.pi / 6)
    ty = 5 * math.sin(math.pi / 6)
    assert steer_to_point(0, 0, 0, tx, ty) == pytest.approx(math.atan(0.5))


def test_find_target():
    path = [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (5, 0), (6, 0)]
    assert nearest_index(1.2, 0.5, path) == 1
    assert find_target(1.2, 0.5, path, 3) == (5, 0)

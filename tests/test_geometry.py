import math

import pytest

from geometry import angle_to_point, inflate, is_collision, move_forward, segment_collision


def test_move_up():
    # 朝上（π/2）走 2 米
    assert move_forward(1, 1, math.pi / 2, 2) == pytest.approx((1, 3))


def test_move_left():
    # 朝左（π）走 2 米
    assert move_forward(0, 0, math.pi, 2) == pytest.approx((-2, 0))


def test_is_collision():
    obstacles = [(5, 5, 1)]
    # 圆内、很远、正好在边上
    assert is_collision(5, 5.5, obstacles)
    assert not is_collision(8, 8, obstacles)
    assert is_collision(6, 5, obstacles)


def test_angle_to_point():
    # 车朝右，点在右上方：偏左 45°
    assert angle_to_point(0, 0, 0, 3, 3) == pytest.approx(math.pi / 4)
    # 车朝上，同一个点：偏右 45°
    assert angle_to_point(0, 0, math.pi / 2, 3, 3) == pytest.approx(-math.pi / 4)
    # 车不在原点：点在车的正左方
    assert angle_to_point(2, 0, 0, 2, 5) == pytest.approx(math.pi / 2)
    # 朝向 2π 和 0 是同一个方向
    assert angle_to_point(0, 0, 2 * math.pi, 3, 3) == pytest.approx(math.pi / 4)


def test_segment_collision():
    obstacles = [(5, 0, 1)]
    # 两个端点都不撞，但线段从圆中间穿过
    assert segment_collision((0, 0), (10, 0), obstacles)
    # 线段从圆上方 3 米经过
    assert not segment_collision((0, 3), (10, 3), obstacles)


def test_inflate():
    assert inflate([(5, 5, 1)], 0.5) == [(5, 5, 1.5)]

import pytest

from path_utils import densify, distance_to_path, path_length, point_to_segment, shortcut


def test_path_length():
    # (0,0) → (3,0) → (3,4)：3 + 4 = 7
    assert path_length([(0, 0), (3, 0), (3, 4)]) == pytest.approx(7)


def test_densify():
    # 10 米的直线，每 0.5 米一个点：一共 21 个点，总长不变
    dense = densify([(0, 0), (10, 0)], ds=0.5)
    assert len(dense) == 21
    assert dense[1] == pytest.approx((0.5, 0))
    assert path_length(dense) == pytest.approx(10)


def test_shortcut_no_obstacle():
    # 没有障碍物：弯弯绕绕的路线直接拉成起点到终点一条线
    path = [(0, 0), (1, 1), (2, -1), (3, 1), (4, 0)]
    assert shortcut(path, []) == [(0, 0), (4, 0)]


def test_shortcut_keeps_detour():
    # 中间有障碍物：不能直接连，必须保留绕路的那个点
    path = [(0, 0), (5, 3), (10, 0)]
    obstacles = [(5, 0, 1)]
    assert shortcut(path, obstacles) == path


def test_point_to_segment():
    # 点在线段正上方 2 米
    assert point_to_segment((5, 2), (0, 0), (10, 0)) == pytest.approx(2)
    # 点在线段延长线外：距离算到最近的端点 (10, 0)
    assert point_to_segment((13, 4), (0, 0), (10, 0)) == pytest.approx(5)


def test_distance_to_path():
    # 车在两个路点中间、正好压在路上：误差应该是 0，而不是到最近路点的距离
    path = [(0, 0), (1, 0), (2, 0)]
    assert distance_to_path(0.5, 0, path) == pytest.approx(0)

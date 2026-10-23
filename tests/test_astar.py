from astar import astar, heuristic, neighbors, reconstruct_path


def test_neighbors():
    blocked = {(3, 3), (3, 4), (3, 5)}
    # 靠墙：右边 (3, 4) 是墙，只剩 3 个邻居
    assert sorted(neighbors((2, 4), blocked, 10, 10)) == sorted([(1, 4), (2, 5), (2, 3)])
    # 地图角落：只剩 2 个邻居
    assert sorted(neighbors((0, 0), blocked, 10, 10)) == sorted([(1, 0), (0, 1)])


def test_reconstruct_path():
    came_from = {(1, 0): (0, 0), (2, 0): (1, 0), (3, 0): (2, 0), (0, 1): (0, 0)}
    assert reconstruct_path(came_from, (0, 0), (3, 0)) == [(0, 0), (1, 0), (2, 0), (3, 0)]


def test_heuristic():
    # (0, 1) 到 (4, 0)：横向差 4，纵向差 1
    assert heuristic((0, 1), (4, 0)) == 5


def test_astar_straight():
    # 没有墙：直接往右走
    path = astar((0, 0), (3, 0), set(), 10, 10)
    assert path == [(0, 0), (1, 0), (2, 0), (3, 0)]


def test_astar_wall():
    # x = 3 处一堵竖墙，从 y = 2 到 6，必须绕过去
    blocked = {(3, 2), (3, 3), (3, 4), (3, 5), (3, 6)}
    path = astar((0, 4), (6, 4), blocked, 10, 10)
    assert path is not None
    assert path[0] == (0, 4)
    assert path[-1] == (6, 4)
    assert len(path) == 13
    for cell in path:
        assert cell not in blocked

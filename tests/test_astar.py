from astar import neighbors

from astar import reconstruct_path, heuristic, astar
def test_neighbors():
    blocked = {(3, 3), (3, 4), (3, 5)}
    # 墙边：(3, 4) 是墙，剩 3 个
    assert sorted(neighbors((2, 4), blocked, 10, 10)) == sorted([(1, 4), (2, 5), (2, 3)])
    # 角落：只剩 2 个
    assert sorted(neighbors((0, 0), blocked, 10, 10)) == sorted([(1, 0), (0, 1)])


def test_reconstruct_path():
    came_from = {(1, 0): (0, 0), (2, 0): (1, 0), (3, 0): (2, 0), (0, 1): (0, 0)}
    assert reconstruct_path(came_from, (0, 0), (3, 0)) == [(0, 0), (1, 0), (2, 0), (3, 0)]


def test_heuristic():
    # 小题里算过：(0, 1) 到 (4, 0)，横着差 4、竖着差 1
    assert heuristic((0, 1), (4, 0)) == 5


def test_astar_straight():
    # 没有墙，从 (0, 0) 到 (3, 0)：应该直接往右走
    path = astar((0, 0), (3, 0), set(), 10, 10)
    assert path == [(0, 0), (1, 0), (2, 0), (3, 0)]


def test_astar_wall():
    # 一堵竖墙挡在 x = 3，从 y = 2 到 y = 6
    blocked = {(3, 2), (3, 3), (3, 4), (3, 5), (3, 6)}
    path = astar((0, 4), (6, 4), blocked, 10, 10)
    if path is None:
        raise ValueError("找不到路：起点和终点之间被墙完全挡住了")
    assert path[0] == (0, 4)          # 从起点出发
    assert path[-1] == (6, 4)         # 到达终点
    assert len(path) == 13          # 路线一共几个格子？
    for cell in path:
        assert cell not in blocked                   # 路线上每个格子都不能是墙

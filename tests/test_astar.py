from astar import neighbors

from astar import reconstruct_path, heuristic
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

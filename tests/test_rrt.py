import math
import random

import pytest

from rrt import rrt, rrt_star, steer


def test_steer():
    # 小题：从 (2, 0) 朝 (2, 4) 伸 1 米，到 (2, 1)
    assert steer((2, 0), (2, 4), 1) == pytest.approx((2, 1))
    # 随机点只有 0.5 米远，比 1 米近：直接到它
    assert steer((0, 0), (0.5, 0), 1) == pytest.approx((0.5, 0))


def path_length(path):
    length = 0.0
    for a, b in zip(path, path[1:]):
        length += math.hypot(b[0] - a[0], b[1] - a[1])
    return length


def test_rrt_star_finds_path():
    random.seed(0)
    path, nodes, parent = rrt_star((1, 1), (9, 9), [(5, 5, 2)], 10, 10)
    assert path is not None
    assert path[0] == (1, 1)
    assert path[-1] == (9, 9)


def test_rrt_star_shorter_than_rrt():
    # 同一个 seed，RRT* 的路线应该比 RRT 短
    random.seed(0)
    path1, nodes1, parent1 = rrt((1, 1), (9, 9), [(5, 5, 2)], 10, 10)
    random.seed(0)
    path2, nodes2, parent2 = rrt_star((1, 1), (9, 9), [(5, 5, 2)], 10, 10)
    assert path1 is not None and path2 is not None
    assert path_length(path2) < path_length(path1)

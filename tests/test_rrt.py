import random

import pytest

from path_utils import path_length
from rrt import rrt, rrt_star, steer


def test_steer():
    # 从 (2, 0) 朝 (2, 4) 走 1 米
    assert steer((2, 0), (2, 4), 1) == pytest.approx((2, 1))
    # 目标只有 0.5 米远，比步长近：直接到目标
    assert steer((0, 0), (0.5, 0), 1) == pytest.approx((0.5, 0))


def test_rrt_star_finds_path():
    random.seed(0)
    path, nodes, parent = rrt_star((1, 1), (9, 9), [(5, 5, 2)], 10, 10)
    assert path is not None
    assert path[0] == (1, 1)
    assert path[-1] == (9, 9)


def test_rrt_star_shorter_than_rrt():
    # 同一个随机种子，RRT* 的路线应该比 RRT 短
    random.seed(0)
    path1, nodes1, parent1 = rrt((1, 1), (9, 9), [(5, 5, 2)], 10, 10)
    random.seed(0)
    path2, nodes2, parent2 = rrt_star((1, 1), (9, 9), [(5, 5, 2)], 10, 10)
    assert path1 is not None and path2 is not None
    assert path_length(path2) < path_length(path1)

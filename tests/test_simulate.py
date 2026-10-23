from path_utils import densify
from simulate import simulate


def test_straight_road():
    # 20 米直路、没有障碍物：应该到达、不撞、几乎没有横向误差
    path = densify([(0, 0), (20, 0)])
    log, result = simulate(path, [])
    assert result["reached"]
    assert not result["collided"]
    assert result["max_err"] < 0.05


def test_hits_obstacle_on_road():
    # 障碍物压在路上：仿真只负责照着路线开，绕路是规划的事，所以一定会撞
    path = densify([(0, 0), (20, 0)])
    log, result = simulate(path, [(10, 0, 1)])
    assert result["collided"]

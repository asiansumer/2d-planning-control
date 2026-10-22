from path_utils import densify
from simulate import simulate


def test_straight_road():
    # 一条 20 米的直路，没有障碍物：车应该开到终点、不撞、几乎没有横向误差
    path = densify([(0, 0), (20, 0)])
    log, result = simulate(path, [])
    assert result["reached"] == True
    assert result["collided"] == False
    assert result["max_err"] < 0.05


def test_hits_obstacle_on_road():
    # 障碍物就压在路上：车照着路线开一定会撞（仿真只负责开车，绕路是规划的事）
    path = densify([(0, 0), (20, 0)])
    log, result = simulate(path, [(10, 0, 1)])
    assert result["collided"] == True

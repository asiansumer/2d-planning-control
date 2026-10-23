"""Pure Pursuit 路径跟踪：在路线上找一个预瞄点，算出开过去需要的转角。"""
import math

from geometry import angle_to_point


def steer_to_point(x, y, theta, tx, ty, L=2.5):
    """车在 (x, y)、朝 theta，返回开向预瞄点 (tx, ty) 所需的前轮转角 delta（左正右负）。

    公式：delta = atan(2 · L · sin(alpha) / Ld)，
    alpha 是预瞄点偏离车头的角度，Ld 是车到预瞄点的距离。
    """
    Ld = math.hypot(tx - x, ty - y)
    alpha = angle_to_point(x, y, theta, tx, ty)
    delta = math.atan(2 * L * math.sin(alpha) / Ld)
    return delta


def nearest_index(x, y, path):
    """返回 path 上离 (x, y) 最近的点的下标。"""
    best_i = 0
    best_d = math.hypot(path[0][0] - x, path[0][1] - y)
    for i, (px, py) in enumerate(path):
        d = math.hypot(px - x, py - y)
        if d < best_d:
            best_i = i
            best_d = d
    return best_i


def find_target(x, y, path, Ld):
    """从最近点开始往前找，返回第一个离车不小于 Ld 的点。

    快到终点、前面已经没有足够远的点时，返回终点。
    """
    start = nearest_index(x, y, path)
    for px, py in path[start:]:
        d = math.hypot(px - x, py - y)
        if d >= Ld:
            return px, py
    return path[-1]

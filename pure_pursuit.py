import math

from geometry import angle_to_point


def steer_to_point(x, y, theta, tx, ty, L=2.5):
    """Pure Pursuit：车在 (x, y)、朝 theta，要开向预瞄点 (tx, ty)，返回方向盘转角 delta（左正右负）。"""
    Ld = math.hypot(tx-x, ty-y)          # 车到预瞄点的距离，用 math.hypot
    alpha = angle_to_point(x, y, theta, tx, ty)       # 预瞄点偏离车头的角度，用你写的 angle_to_point
    delta = math.atan(2 * L * math.sin(alpha) / Ld)       # 公式：atan(2 × L × sin(alpha) ÷ Ld)
    return delta


def nearest_index(x, y, path):
    """返回路线 path 上离车 (x, y) 最近的点的下标。"""
    best_i = 0
    best_d = math.hypot(path[0][0] - x, path[0][1] - y)
    for i, (px, py) in enumerate(path):
        d = math.hypot(px - x, py - y)    # 这个点离车多远
        if d < best_d:
            best_i = i
            best_d = d
    return best_i


def find_target(x, y, path, Ld):
    """从最近点往前找，返回第一个离车 >= Ld 的点；都不够远就返回终点。"""
    start = nearest_index(x, y, path)
    for px, py in path[start:]:
        d = math.hypot(px - x, py - y)
        if d >= Ld:
            return px, py
    return path[-1]                    # 快到终点时，前面没有够远的点，就瞄准终点

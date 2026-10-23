"""运动学自行车模型。"""
import math

from geometry import move_forward


def step(x, y, theta, v, delta, dt=0.1, L=2.5):
    """自行车模型前进一步。

    状态 (x, y, theta)：位置和车头朝向（弧度）。
    输入 (v, delta)：车速（m/s）和前轮转角（弧度，左正右负）。
    dt 是步长（秒），L 是轴距（米）。
    返回下一步的状态 (new_x, new_y, new_theta)。
    """
    new_x, new_y = move_forward(x, y, theta, v * dt)
    new_theta = theta + v * dt / L * math.tan(delta)
    return new_x, new_y, new_theta

import math

from geometry import move_forward


def step(x, y, theta, v, delta, dt=0.1, L=2.5):
    """自行车模型走一步：给当前状态 (x, y, theta) 和输入 (v, delta)，返回下一步的 (new_x, new_y, new_theta)。

    dt：每一步的时间（秒）；L：轴距，前后轮之间的距离（米）。
    """
    new_x, new_y = move_forward(x, y, theta, v*dt)
    new_theta = theta + v  * dt / L * math.tan(delta)
    return new_x, new_y, new_theta

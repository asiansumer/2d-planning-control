import math


def move_forward(x, y, theta, d):
    """车在 (x, y)，朝角度 theta（弧度），往前走 d 米，返回新位置 (new_x, new_y)。"""
    new_x = x + math.cos(theta) * d
    new_y = y + math.sin(theta) * d
    return new_x, new_y


def is_collision(x, y, obstacles):
    """点 (x, y) 撞到任意一个障碍物就返回 True，都没撞到返回 False。"""
    for cx, cy, r in obstacles:
        dist = math.hypot(x-cx, y-cy)
        if dist <= r:
            return True
    return False

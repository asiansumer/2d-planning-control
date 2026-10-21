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


def angle_to_point(x, y, theta, tx, ty):
    """车在 (x, y)、朝 theta，目标点 (tx, ty) 偏离车头多少弧度（左正右负）。"""
    target_angle = math.atan2(ty-y, tx-x)          # 目标点方向
    alpha = target_angle - theta             # 减去车头朝向
    return wrap_angle(alpha)


def wrap_angle(a):
    """把角度规整到 -π 到 π 之间（同一个方向，取最近的转法）。"""
    while a > math.pi:
        a = a - 2*math.pi
    while a < -math.pi:
        a = a + 2*math.pi
    return a

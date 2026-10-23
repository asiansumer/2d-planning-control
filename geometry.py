"""几何工具：移动、角度、碰撞检测、障碍物膨胀。

障碍物统一用圆表示：(cx, cy, r)，即圆心坐标和半径。
"""
import math


def move_forward(x, y, theta, d):
    """从 (x, y) 沿朝向 theta（弧度）前进 d 米，返回新位置 (new_x, new_y)。"""
    new_x = x + math.cos(theta) * d
    new_y = y + math.sin(theta) * d
    return new_x, new_y


def is_collision(x, y, obstacles):
    """点 (x, y) 落在任意一个障碍物里（含边界）就返回 True。"""
    for cx, cy, r in obstacles:
        dist = math.hypot(x - cx, y - cy)
        if dist <= r:
            return True
    return False


def angle_to_point(x, y, theta, tx, ty):
    """目标点 (tx, ty) 相对车头朝向 theta 偏了多少弧度，左正右负，范围 [-π, π]。"""
    target_angle = math.atan2(ty - y, tx - x)
    alpha = target_angle - theta
    return wrap_angle(alpha)


def wrap_angle(a):
    """把角度规整到 [-π, π]，表示同一个方向，取转得最少的那种写法。"""
    while a > math.pi:
        a = a - 2 * math.pi
    while a < -math.pi:
        a = a + 2 * math.pi
    return a


def segment_collision(p1, p2, obstacles, step=0.1):
    """线段 p1 → p2 是否碰到障碍物。

    沿线段每隔 step 米取一个点检查，只要有一个点撞到就返回 True。
    """
    dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
    n = max(1, math.ceil(dist / step))
    for k in range(n + 1):
        t = k / n
        x = p1[0] + t * (p2[0] - p1[0])
        y = p1[1] + t * (p2[1] - p1[1])
        if is_collision(x, y, obstacles):
            return True
    return False


def inflate(obstacles, margin):
    """障碍物膨胀：每个圆的半径加大 margin 米，返回新的障碍物列表。"""
    return [(cx, cy, r + margin) for cx, cy, r in obstacles]

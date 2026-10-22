import random

from geometry import inflate, is_collision
from path_utils import densify, shortcut
from rrt import rrt_star


def narrow():
    """窄道：中间一堵墙，只在 y=7 到 13 之间留一个口子。"""
    wall = [(15, y, 1.0) for y in range(0, 7)] + [(15, y, 1.0) for y in range(14, 21)]
    return {
        "name": "narrow",
        "width": 30, "height": 20,
        "start": (2, 3), "goal": (28, 17),
        "obstacles": wall + [(7, 8, 2.0), (22, 12, 2.0)],
    }


def u_turn():
    """U 弯：中间一道横墙，车要先往右开，绕过墙头再掉头往左开。"""
    wall = [(x, 10, 1.0) for x in range(0, 21)]
    return {
        "name": "u_turn",
        "width": 30, "height": 20,
        "start": (3, 4), "goal": (3, 16),
        "obstacles": wall,
    }


def random_obstacles(seed=3, n=12):
    """随机障碍：随机撒 n 个圆，避开起点和终点附近。"""
    rng = random.Random(seed)
    start, goal = (2, 2), (28, 18)
    obstacles = []
    while len(obstacles) < n:
        ob = (rng.uniform(4, 26), rng.uniform(2, 18), rng.uniform(1.0, 2.0))
        if not is_collision(start[0], start[1], inflate([ob], 3)) and not is_collision(goal[0], goal[1], inflate([ob], 3)):
            obstacles.append(ob)
    return {
        "name": "random",
        "width": 30, "height": 20,
        "start": start, "goal": goal,
        "obstacles": obstacles,
    }


def plan(scn, margin=1.5, seed=0, max_iter=3000):
    """规划：障碍物膨胀 margin 米 → RRT* 找路 → 拉直 → 加密。返回 (加密后的路线, RRT* 原始路线)。"""
    big = inflate(scn["obstacles"], margin)
    random.seed(seed)
    raw, nodes, parent = rrt_star(scn["start"], scn["goal"], big, scn["width"], scn["height"], max_iter=max_iter)
    if raw is None:
        return None, None
    smooth = shortcut(raw, big)
    return densify(smooth), raw

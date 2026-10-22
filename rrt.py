import math
import random

from astar import reconstruct_path
from geometry import is_collision, move_forward
from pure_pursuit import nearest_index


def steer(from_pt, to_pt, step):
    """从 from_pt 朝 to_pt 的方向伸 step 米，返回新点。如果 to_pt 比 step 还近，就直接返回 to_pt。"""
    fx, fy = from_pt
    tx, ty = to_pt
    dist = math.hypot(tx-fx, ty-fy)
    if dist <= step:
        return to_pt
    angle = math.atan2(ty-fy, tx-fx)
    return move_forward(fx, fy, angle, step)


def rrt(start, goal, obstacles, width, height, step=1.0, goal_tol=1.0, max_iter=2000):
    """RRT：从 start 长一棵树找到 goal 附近。返回 (路线, 所有节点, 父节点字典)；找不到时路线是 None。"""
    nodes = [start]
    parent: dict = {start: None}

    for i in range(max_iter):
        # 第 1 块：随机撒点，找最近节点
        if random.random() < 0.1:
            rand_pt = goal
        else:
            rand_pt = (random.uniform(0, width), random.uniform(0, height))
        j = nearest_index(rand_pt[0], rand_pt[1], nodes)
        near = nodes[j]

        # 第 2 块：伸一段、查碰撞、挂到树上、判终点
        new_pt = steer(near, rand_pt, step)
        if is_collision(new_pt[0], new_pt[1], obstacles):
            continue
        nodes.append(new_pt)
        parent[new_pt] = near
        if math.hypot(goal[0] - new_pt[0], goal[1] - new_pt[1]) <= goal_tol:
            if new_pt != goal:
                parent[goal] = new_pt
            return reconstruct_path(parent, start, goal), nodes, parent

    return None, nodes, parent    # 撒完 max_iter 次还没到：没找到

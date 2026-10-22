import math
import random

from astar import reconstruct_path
from geometry import move_forward, segment_collision
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
        if segment_collision(near, new_pt, obstacles):
            continue
        nodes.append(new_pt)
        parent[new_pt] = near
        if math.hypot(goal[0] - new_pt[0], goal[1] - new_pt[1]) <= goal_tol and not segment_collision(new_pt, goal, obstacles):
            if new_pt != goal:
                parent[goal] = new_pt
            return reconstruct_path(parent, start, goal), nodes, parent

    return None, nodes, parent    # 撒完 max_iter 次还没到：没找到



def rrt_star(start, goal, obstacles, width, height, step=1.0, goal_tol=1.0, radius=2.0, max_iter=2000):
    """RRT*：和 RRT 一样长树，但新点会挑最省的父节点，还会帮邻居重新连线。"""
    nodes = [start]
    parent: dict = {start: None}
    cost = {start: 0.0}

    for i in range(max_iter):
        # 撒点、找最近、伸一段、查碰撞：和 rrt 一样
        if random.random() < 0.1:
            rand_pt = goal
        else:
            rand_pt = (random.uniform(0, width), random.uniform(0, height))
        j = nearest_index(rand_pt[0], rand_pt[1], nodes)
        near = nodes[j]
        new_pt = steer(near, rand_pt, step)
        if segment_collision(near, new_pt, obstacles):
            continue
        # 已经在树上的点不再加
        if new_pt in parent:
            continue

        # 第 1 块：选父节点
        best_parent = near
        best_cost = cost[near] + math.hypot(new_pt[0] - near[0], new_pt[1] - near[1])
        for node in nodes:
            d = math.hypot(new_pt[0] - node[0], new_pt[1] - node[1])
            if d <= radius and cost[node] + d < best_cost and not segment_collision(node, new_pt, obstacles):
                best_parent = node
                best_cost = cost[node] + d
        nodes.append(new_pt)
        parent[new_pt] = best_parent
        cost[new_pt] = best_cost
        # 第 2 块：重新连线
        for node in nodes:
            d = math.hypot(node[0] - new_pt[0], node[1] - new_pt[1])
            if d <= radius and cost[new_pt] + d < cost[node] and not segment_collision(new_pt, node, obstacles):
                parent[node] = new_pt
                cost[node] = cost[new_pt]+d
    # 第 3 块：在终点附近挑最省的点
    best_end = None
    best_total = float("inf")
    for node in nodes:
        d = math.hypot(goal[0] - node[0], goal[1] - node[1])
        if d <= goal_tol and cost[node] + d < best_total and not segment_collision(node, goal, obstacles):
            best_end = node
            best_total = cost[node] + d
    if best_end is None:
        return None, nodes, parent
    if best_end != goal:
        parent[goal] = best_end
    return reconstruct_path(parent, start, goal), nodes, parent

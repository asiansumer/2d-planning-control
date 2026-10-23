"""RRT 和 RRT* 路径规划（连续空间，圆形障碍物）。

两个函数都返回 (path, nodes, parent)：
path 是从起点到终点的点列表，找不到时为 None；
nodes 是树上所有节点；parent[node] 是该节点的父节点，起点的父节点是 None。
"""
import math
import random

from astar import reconstruct_path
from geometry import move_forward, segment_collision
from pure_pursuit import nearest_index


def steer(from_pt, to_pt, step):
    """从 from_pt 朝 to_pt 走 step 米，返回新点；如果 to_pt 比 step 还近，直接返回 to_pt。"""
    fx, fy = from_pt
    tx, ty = to_pt
    dist = math.hypot(tx - fx, ty - fy)
    if dist <= step:
        return to_pt
    angle = math.atan2(ty - fy, tx - fx)
    return move_forward(fx, fy, angle, step)


def sample(goal, width, height, goal_bias=0.1):
    """随机采样一个点：有 goal_bias 的概率直接取终点，其余在地图内均匀采样。"""
    if random.random() < goal_bias:
        return goal
    return (random.uniform(0, width), random.uniform(0, height))


def rrt(start, goal, obstacles, width, height, step=1.0, goal_tol=1.0, max_iter=2000):
    """RRT：从 start 长一棵树，第一次到达 goal 附近就停下。"""
    nodes = [start]
    parent: dict = {start: None}

    for _ in range(max_iter):
        rand_pt = sample(goal, width, height)
        near = nodes[nearest_index(rand_pt[0], rand_pt[1], nodes)]
        new_pt = steer(near, rand_pt, step)
        if segment_collision(near, new_pt, obstacles):
            continue
        nodes.append(new_pt)
        parent[new_pt] = near

        close_to_goal = math.hypot(goal[0] - new_pt[0], goal[1] - new_pt[1]) <= goal_tol
        if close_to_goal and not segment_collision(new_pt, goal, obstacles):
            # 新点本身就是终点时不能再挂一次，否则会形成自环
            if new_pt != goal:
                parent[goal] = new_pt
            return reconstruct_path(parent, start, goal), nodes, parent

    return None, nodes, parent


def rrt_star(start, goal, obstacles, width, height, step=1.0, goal_tol=1.0, radius=2.0, max_iter=2000):
    """RRT*：在 RRT 的基础上，为每个新点选代价最小的父节点，并对附近节点重新连线。

    cost[node] 是从起点沿树走到 node 的路程。跑满 max_iter 次后，
    在终点附近 goal_tol 内挑总路程最短的节点连到终点。
    """
    nodes = [start]
    parent: dict = {start: None}
    cost = {start: 0.0}

    for _ in range(max_iter):
        rand_pt = sample(goal, width, height)
        near = nodes[nearest_index(rand_pt[0], rand_pt[1], nodes)]
        new_pt = steer(near, rand_pt, step)
        if segment_collision(near, new_pt, obstacles):
            continue
        # 同一个点不能加两次，否则 parent 会被覆盖，甚至成环
        if new_pt in parent:
            continue

        # 选父节点：半径 radius 内，让新点总路程最短、且连线不撞的节点
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

        # 重新连线：附近的节点如果改从新点走更近，就把父节点换成新点
        for node in nodes:
            d = math.hypot(node[0] - new_pt[0], node[1] - new_pt[1])
            if d <= radius and cost[new_pt] + d < cost[node] and not segment_collision(new_pt, node, obstacles):
                parent[node] = new_pt
                cost[node] = cost[new_pt] + d

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

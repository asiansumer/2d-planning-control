import math
import random

import matplotlib.pyplot as plt
from matplotlib.patches import Circle

from rrt import rrt, rrt_star

width, height = 20, 20
obstacles = [(6, 6, 2), (12, 12, 3), (15, 5, 2), (6, 15, 2)]
start = (1, 1)
goal = (18, 18)


def path_length(path):
    """把路线上相邻两点的距离加起来。"""
    length = 0.0
    for a, b in zip(path, path[1:]):
        length += math.hypot(b[0] - a[0], b[1] - a[1])
    return length


def draw(ax, title, path, nodes, parent):
    """在一个子图 ax 上画障碍物、树、路线。"""
    for cx, cy, r in obstacles:
        ax.add_patch(Circle((cx, cy), r, color="gray"))
    for node in nodes:
        p = parent[node]
        if p is not None:
            ax.plot([p[0], node[0]], [p[1], node[1]], color="lightgreen", linewidth=0.5)
    ax.plot([pt[0] for pt in path], [pt[1] for pt in path], "b-", linewidth=2)
    ax.plot(start[0], start[1], "go", markersize=10)
    ax.plot(goal[0], goal[1], "r*", markersize=14)
    ax.set_title(title)
    ax.set_xlim(0, width)
    ax.set_ylim(0, height)
    ax.set_aspect("equal")


seed = 0

random.seed(seed)
path1, nodes1, parent1 = rrt(start, goal, obstacles, width, height)
random.seed(seed)
path2, nodes2, parent2 = rrt_star(start, goal, obstacles, width, height)
if path1 is None or path2 is None:
    raise ValueError("没找到路，试试调大 max_iter")

len1 = path_length(path1)
len2 = path_length(path2)
print("RRT  路线长度:", round(len1, 2), "节点数:", len(nodes1))
print("RRT* 路线长度:", round(len2, 2), "节点数:", len(nodes2))

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
draw(ax1, "RRT  length " + str(round(len1, 1)), path1, nodes1, parent1)
draw(ax2, "RRT* length " + str(round(len2, 1)), path2, nodes2, parent2)
plt.show()

import math
import random

import matplotlib.pyplot as plt
from matplotlib.patches import Circle

from rrt import rrt

random.seed(2)

width, height = 20, 20
obstacles = [(6, 6, 2), (12, 12, 3), (15, 5, 2), (6, 15, 2)]
start = (1, 1)
goal = (18, 18)

path, nodes, parent = rrt(start, goal, obstacles, width, height,)
if path is None:
    raise ValueError("没找到路，试试调大 max_iter")

length = 0.0
for a, b in zip(path, path[1:]):
    length += math.hypot(b[0] - a[0], b[1] - a[1])
print("树上节点数:", len(nodes))
print("路线长度:", length)

fig, ax = plt.subplots()
for cx, cy, r in obstacles:
    ax.add_patch(Circle((cx, cy), r, color="gray"))

# 画树：每个节点和它的父节点连一条细线
for node in nodes:
    p = parent[node]
    if p is not None:
        ax.plot([p[0], node[0]], [p[1], node[1]], color="lightgreen", linewidth=0.8)

path_xs = [pt[0] for pt in path]
path_ys = [pt[1] for pt in path]
ax.plot(path_xs, path_ys, "b-", linewidth=2, label="RRT path")
ax.plot(start[0], start[1], "go", markersize=10, label="start")
ax.plot(goal[0], goal[1], "r*", markersize=14, label="goal")
ax.set_xlim(0, width)
ax.set_ylim(0, height)
ax.set_aspect("equal")
ax.legend()
plt.show()

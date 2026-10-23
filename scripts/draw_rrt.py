"""第 6 关：用 RRT 在 4 个圆形障碍物之间找路，画出整棵树和路线。

换 random.seed 的数字可以看到每次长出来的树和路线都不一样。
运行：python -m scripts.draw_rrt
"""
import random

import matplotlib.pyplot as plt
from matplotlib.patches import Circle

from path_utils import path_length
from rrt import rrt

random.seed(2)

width, height = 20, 20
obstacles = [(6, 6, 2), (12, 12, 3), (15, 5, 2), (6, 15, 2)]
start = (1, 1)
goal = (18, 18)

path, nodes, parent = rrt(start, goal, obstacles, width, height)
if path is None:
    raise ValueError("没找到路，试试调大 max_iter")
print("树上节点数:", len(nodes))
print("路线长度:", path_length(path))

fig, ax = plt.subplots()
for cx, cy, r in obstacles:
    ax.add_patch(Circle((cx, cy), r, color="gray"))
# 画树：每个节点和它的父节点之间连一条细线
for node in nodes:
    p = parent[node]
    if p is not None:
        ax.plot([p[0], node[0]], [p[1], node[1]], color="lightgreen", linewidth=0.8)
ax.plot([pt[0] for pt in path], [pt[1] for pt in path], "b-", linewidth=2, label="RRT path")
ax.plot(start[0], start[1], "go", markersize=10, label="start")
ax.plot(goal[0], goal[1], "r*", markersize=14, label="goal")
ax.set_xlim(0, width)
ax.set_ylim(0, height)
ax.set_aspect("equal")
ax.legend()
plt.show()

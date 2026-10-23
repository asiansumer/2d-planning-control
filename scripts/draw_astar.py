"""第 5 关：在 20×20 的格子地图上用 A* 绕过一堵竖墙。

运行：python -m scripts.draw_astar
"""
import matplotlib.pyplot as plt

from astar import astar

width, height = 20, 20
# x = 10 处一堵竖墙，从 y = 3 到 16
blocked = {(10, y) for y in range(3, 17)}
start = (2, 10)
goal = (17, 10)

path = astar(start, goal, blocked, width, height)
if path is None:
    raise ValueError("找不到路：起点和终点之间被墙完全挡住了")
print("路线格子数:", len(path))

fig, ax = plt.subplots()
ax.scatter([c[0] for c in blocked], [c[1] for c in blocked], marker="s", color="black", s=80, label="wall")
ax.plot([c[0] for c in path], [c[1] for c in path], "b.-", label="A* path")
ax.plot(start[0], start[1], "go", markersize=10, label="start")
ax.plot(goal[0], goal[1], "r*", markersize=14, label="goal")
ax.set_xlim(-1, width)
ax.set_ylim(-1, height)
ax.set_aspect("equal")
ax.grid(True)
ax.legend()
plt.show()

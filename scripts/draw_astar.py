import matplotlib.pyplot as plt

from astar import astar

width, height = 20, 20
blocked = set()
for y in range(3, 17):
    blocked.add((10, y))              # 在 x = 10 砌一堵竖墙，从 y = 3 到 16

start = (2, 10)
goal = (17, 10)
path = astar(start, goal, blocked, width, height)
if path is None:
        raise ValueError("找不到路：起点和终点之间被墙完全挡住了")
print("路线格子数:", len(path))

wall_xs = [c[0] for c in blocked]     # 墙的 x、y 分开，方便画
wall_ys = [c[1] for c in blocked]
path_xs = [c[0] for c in path]
path_ys = [c[1] for c in path]

fig, ax = plt.subplots()
ax.scatter(wall_xs, wall_ys, marker="s", color="black", s=80, label="wall")   # 墙画成黑方块
ax.plot(path_xs, path_ys, "b.-", label="A* path")                            # 路线
ax.plot(start[0], start[1], "go", markersize=10, label="start")
ax.plot(goal[0], goal[1], "r*", markersize=14, label="goal")
ax.set_xlim(-1, width)
ax.set_ylim(-1, height)
ax.set_aspect("equal")
ax.grid(True)
ax.legend()
plt.show()

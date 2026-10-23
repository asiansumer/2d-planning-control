"""第 4 关：用 Pure Pursuit 跟一条 S 弯路线，统计横向误差。

车速固定，可以改 v 和 Ld 做实验：车速越高，Ld 要越大。
运行：python -m scripts.follow_path
"""
import math

import matplotlib.pyplot as plt

from bicycle import step
from path_utils import distance_to_path
from pure_pursuit import find_target, steer_to_point

# S 弯：x 从 0 到 60 米，每 0.2 米一个点，y = 3 · sin(x / 5)
path = []
for i in range(301):
    px = i * 0.2
    path.append((px, 3 * math.sin(px / 5)))

# 车从路线下方 1 米出发，故意偏一点
x, y, theta = 0.0, -1.0, 0.0
v = 5.0
Ld = 4.0

xs = [x]
ys = [y]
errs = []
# 120 步 = 12 秒，约 60 米
for i in range(120):
    tx, ty = find_target(x, y, path, Ld)
    delta = steer_to_point(x, y, theta, tx, ty, L=2.5)
    x, y, theta = step(x, y, theta, v, delta, dt=0.1, L=2.5)
    xs.append(x)
    ys.append(y)
    errs.append(distance_to_path(x, y, path))

# 前 20 步是从偏离位置回到路线上的过程，不算进误差
print("平均误差:", sum(errs[20:]) / len(errs[20:]))
print("最大误差:", max(errs[20:]))

fig, ax = plt.subplots()
ax.plot([p[0] for p in path], [p[1] for p in path], "r--", label="path")
ax.plot(xs, ys, "b-", label="car")
ax.set_aspect("equal")
ax.legend()
ax.grid(True)
plt.show()

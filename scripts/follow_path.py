import math

import matplotlib.pyplot as plt

from bicycle import step

from pure_pursuit import find_target, nearest_index, steer_to_point

# 生成一条 S 弯路线：x 从 0 到 60 米，每 0.2 米一个点，y = 3 × sin(x / 5)
path = []
for i in range(301):
    px = i * 0.2
    path.append((px, 3 * math.sin(px / 5)))

x, y, theta = 0.0, -1.0, 0.0   # 车从路线下方 1 米出发，故意偏一点
v = 15.0                        # 车速固定 5 米/秒（车速控制留到第 7 关再接上）
Ld = 4.0                       # 预瞄距离

xs = [x]
errs = []
ys = [y]
for i in range(40):                       # 120 步 = 12 秒，约 60 米
    tx, ty = find_target(x, y, path, Ld)
    delta = steer_to_point(x, y, theta, tx, ty, L=2.5)
    x, y, theta = step(x, y, theta, v, delta, dt=0.1, L=2.5)
    xs.append(x)
    ys.append(y)
    j = nearest_index(x, y, path)
    px, py = path[j]
    errs.append(math.hypot(px-x, py-y))

path_xs = [p[0] for p in path]
path_ys = [p[1] for p in path]

fig, ax = plt.subplots()
ax.plot(path_xs, path_ys, "r--", label="path")
ax.plot(xs, ys, "b-", label="car")
ax.set_aspect("equal")
ax.legend()
ax.grid(True)
print("平均误差:", sum(errs[20:]) / len(errs[20:]))
print("最大误差:", max(errs[20:]))
plt.show()

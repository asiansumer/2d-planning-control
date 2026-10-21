import math

import matplotlib.pyplot as plt

from bicycle import step

x, y, theta = 0, 0, 0     # 起点：原点，朝右
v = 2.5                   # 车速
delta = math.pi / 4       # 方向盘左转 45°

xs = [x]                  # 记下每一步的 x，先放进起点
ys = [y]                  # 记下每一步的 y

for i in range(63):
    x, y, theta = step(x, y, theta, v, delta)   # 走一步，用新状态覆盖旧状态
    xs.append(x)       # 把新的 x 记下来
    ys.append(y)       # 把新的 y 记下来

fig, ax = plt.subplots()
ax.plot(xs, ys, "b-")     # 把所有点连成线
ax.set_aspect("equal")
ax.grid(True)
plt.show()

"""第 2 关：用自行车模型连续走 63 步，画出轨迹（转角固定时是一个圆）。

运行：python -m scripts.draw_trajectory
"""
import math

import matplotlib.pyplot as plt

from bicycle import step

# 从原点出发，车头朝右
x, y, theta = 0, 0, 0
v = 2.5
# 前轮左转 45°
delta = math.pi / 4

xs = [x]
ys = [y]
for i in range(63):
    x, y, theta = step(x, y, theta, v, delta)
    xs.append(x)
    ys.append(y)

fig, ax = plt.subplots()
ax.plot(xs, ys, "b-")
ax.set_aspect("equal")
ax.grid(True)
plt.show()

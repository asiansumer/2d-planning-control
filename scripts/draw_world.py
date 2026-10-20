import math

import matplotlib.pyplot as plt
from matplotlib.patches import Circle

from geometry import move_forward

# 车的状态：位置 (x, y) 和朝向 theta（弧度）
x, y, theta = 2, 3, math.pi / 4

# 车头前方 1 米的点，用来画朝向线
tip_x, tip_y = move_forward(x, y, theta, 1)

fig, ax = plt.subplots()
# 障碍物：每个是 (圆心x, 圆心y, 半径)
obstacles = [(5, 5, 1), (7, 2, 0.5), (3, 8, 1.5)]

for cx, cy, r in obstacles:
    circle = Circle((cx, cy), r, color="gray")   # 建一个灰色的圆
    ax.add_patch(circle)                             # 把圆贴到画布上
ax.plot(x, y, "bo")                        # 车的位置：蓝色圆点
ax.plot([x, tip_x], [y, tip_y], "r-")      # 车头朝向：红色线
ax.set_xlim(0, 10)                         # x 轴显示 0 到 10
ax.set_ylim(0, 10)                         # y 轴显示 0 到 10
ax.set_aspect("equal")                     # x、y 一格一样长，不然 45° 看起来不像 45°
ax.grid(True)                              # 画网格
plt.show()                                 # 弹出窗口显示

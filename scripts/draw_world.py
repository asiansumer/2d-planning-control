"""第 1 关：画出世界，几个圆形障碍物和一辆车（位置 + 车头朝向）。

运行：python -m scripts.draw_world
"""
import math

import matplotlib.pyplot as plt
from matplotlib.patches import Circle

from geometry import move_forward

# 车的位置 (x, y) 和朝向 theta（弧度）
x, y, theta = 2, 3, math.pi / 4
# 每个障碍物是 (圆心 x, 圆心 y, 半径)
obstacles = [(5, 5, 1), (7, 2, 0.5), (3, 8, 1.5)]

# 车头前方 1 米的点，用来画朝向线
tip_x, tip_y = move_forward(x, y, theta, 1)

fig, ax = plt.subplots()
for cx, cy, r in obstacles:
    ax.add_patch(Circle((cx, cy), r, color="gray"))
ax.plot(x, y, "bo")
ax.plot([x, tip_x], [y, tip_y], "r-")
ax.set_xlim(0, 10)
ax.set_ylim(0, 10)
# x、y 方向等比例，否则 45° 看起来不像 45°
ax.set_aspect("equal")
ax.grid(True)
plt.show()

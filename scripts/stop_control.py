"""第 3 关：用 PD 控制让车停在 20 米处的停止线。

把 kd 改成 0 可以看到没有 D 项时车会在停止线前后来回晃。
运行：python -m scripts.stop_control
"""
import matplotlib.pyplot as plt

from control import pd_control

dt = 0.1
target_x = 20.0
x = 0.0
v = 0.0
kp = 1
kd = 2

ts = [0.0]
xs = [x]
# 150 步 = 15 秒；每步先用 PD 算加速度，再更新车速，最后更新位置
for i in range(150):
    a = pd_control(target_x, x, v, kp, kd)
    v = v + a * dt
    x = x + v * dt
    ts.append((i + 1) * dt)
    xs.append(x)

fig, ax = plt.subplots()
ax.plot(ts, xs, "b-", label="x")
ax.axhline(target_x, color="r", linestyle="--", label="stop line")
ax.set_xlabel("time (s)")
ax.set_ylabel("position (m)")
ax.legend()
ax.grid(True)
plt.show()

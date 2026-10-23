"""第 3 关：用 PI 控制把车速从 0 提到 10 m/s。

车速模型：v ← v + (a − drag · v) · dt，阻力和车速成正比。
运行：python -m scripts.speed_control
"""
import matplotlib.pyplot as plt

from control import pi_control

dt = 0.1
target_v = 10
v = 0.0
kp = 2
ki = 1
integral = 0.0
drag = 0.5

ts = [0.0]
vs = [v]
# 100 步 = 10 秒
for i in range(100):
    a, integral = pi_control(target_v, v, integral, kp, ki, dt)
    v = v + (a - drag * v) * dt
    ts.append((i + 1) * dt)
    vs.append(v)

print("最终车速:", vs[-1])

fig, ax = plt.subplots()
ax.plot(ts, vs, "b-", label="v")
ax.axhline(target_v, color="r", linestyle="--", label="target")
ax.set_xlabel("time (s)")
ax.set_ylabel("speed (m/s)")
ax.legend()
ax.grid(True)
plt.show()

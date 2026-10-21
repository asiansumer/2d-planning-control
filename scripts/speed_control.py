import matplotlib.pyplot as plt

from control import pi_control

dt = 0.1          # 每一步 0.1 秒
target_v = 10     # 目标车速
v = 0.0             # 一开始车是停着的
kp = 2
ki = 1
integral = 0.0    # 账本，从 0 开始记
drag = 0.5        # 阻力系数：阻力 = drag × 车速

ts = [0.0]          # 记下每一步的时间
vs = [v]          # 记下每一步的车速

for i in range(100):                 # 100 步 = 10 秒
    a, integral = pi_control(target_v, v, integral, kp, ki, dt)
    v = v + (a - drag * v) * dt      # 实际加速度 = 油门给的 a 减去阻力
    ts.append((i + 1) * dt)          # 第 i+1 步的时间
    vs.append(v)

fig, ax = plt.subplots()
ax.plot(ts, vs, "b-", label="v")
ax.axhline(target_v, color="r", linestyle="--", label="target")   # 画一条目标车速的水平虚线
ax.set_xlabel("time (s)")
ax.set_ylabel("speed (m/s)")
ax.legend()
ax.grid(True)
print("最终车速:", vs[-1])
plt.show()

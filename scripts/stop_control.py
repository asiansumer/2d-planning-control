import matplotlib.pyplot as plt

from control import pd_control

dt = 0.1
target_x = 20.0   # 停止线在 20 米
x = 0.0           # 车从 0 米出发
v = 0.0           # 一开始停着
kp = 1
kd = 2            # 先不加 D

ts = [0.0]
xs = [x]

for i in range(150):                 # 150 步 = 15 秒
    a = pd_control(target_x, x, v, kp, kd)  # 1. 用 PD 控制算加速度
    v = v + a * dt                         # 2. 更新车速：v = v + a × dt
    x = x + v * dt                         # 3. 更新位置：x = x + v × dt
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

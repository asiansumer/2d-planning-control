import math
import os

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Circle

from path_utils import path_length
from scenarios import narrow, plan, random_obstacles, u_turn
from simulate import simulate

MARGIN = 1.5
CAR_RADIUS = 0.9
OUT_DIR = "results"


def draw_map(ax, scn, path, raw):
    """画地图：障碍物（灰）、膨胀范围（虚线圈）、RRT* 原始路线（浅色）、拉直后的路线（红虚线）。"""
    for cx, cy, r in scn["obstacles"]:
        ax.add_patch(Circle((cx, cy), r, color="gray"))
        ax.add_patch(Circle((cx, cy), r + MARGIN, fill=False, color="gray", linestyle=":", linewidth=0.6))
    ax.plot([p[0] for p in raw], [p[1] for p in raw], color="orange", linewidth=1, alpha=0.6, label="RRT* raw")
    ax.plot([p[0] for p in path], [p[1] for p in path], "r--", linewidth=1.2, label="planned path")
    ax.plot(scn["start"][0], scn["start"][1], "go", markersize=9)
    ax.plot(scn["goal"][0], scn["goal"][1], "r*", markersize=13)
    ax.set_xlim(0, scn["width"])
    ax.set_ylim(0, scn["height"])
    ax.set_aspect("equal")


def save_png(scn, path, raw, log, result):
    """存静态图：地图 + 车实际开过的轨迹。"""
    fig, ax = plt.subplots(figsize=(9, 6))
    draw_map(ax, scn, path, raw)
    ax.plot(log["x"], log["y"], "b-", linewidth=2, label="car")
    title = scn["name"] + "  time " + str(result["time"]) + " s  mean err " + str(round(result["mean_err"], 2)) + " m"
    ax.set_title(title)
    ax.legend(loc="lower right")
    fig.savefig(os.path.join(OUT_DIR, scn["name"] + ".png"), dpi=120)
    plt.close(fig)


def save_gif(scn, path, raw, log):
    """存动图：每一帧画出车的位置（圆 = 车身，短线 = 车头朝向）和走过的轨迹。"""
    fig, ax = plt.subplots(figsize=(9, 6))
    draw_map(ax, scn, path, raw)
    trail, = ax.plot([], [], "b-", linewidth=2)
    body = Circle((log["x"][0], log["y"][0]), CAR_RADIUS, color="tab:blue", alpha=0.5)
    ax.add_patch(body)
    heading, = ax.plot([], [], "k-", linewidth=2)
    # 每 2 步画一帧，动图不会太大
    frames = list(range(0, len(log["x"]), 2))

    def update(k):
        x, y, theta = log["x"][k], log["y"][k], log["theta"][k]
        trail.set_data(log["x"][:k + 1], log["y"][:k + 1])
        body.center = (x, y)
        heading.set_data([x, x + 1.5 * math.cos(theta)], [y, y + 1.5 * math.sin(theta)])
        ax.set_title(scn["name"] + "  t = " + str(round(log["t"][k], 1)) + " s  v = " + str(round(log["v"][k], 1)) + " m/s")
        return trail, body, heading

    anim = FuncAnimation(fig, update, frames=frames, blit=False)
    anim.save(os.path.join(OUT_DIR, scn["name"] + ".gif"), writer=PillowWriter(fps=20))
    plt.close(fig)


os.makedirs(OUT_DIR, exist_ok=True)
print("场景      路线长(m)  用时(s)  平均误差(m)  最大误差(m)  到达  碰撞")
for scn in [narrow(), u_turn(), random_obstacles()]:
    path, raw = plan(scn, margin=MARGIN)
    if path is None:
        print(scn["name"], "没找到路")
        continue
    log, result = simulate(path, scn["obstacles"], car_radius=CAR_RADIUS)
    print(scn["name"].ljust(9),
          str(round(path_length(path), 1)).rjust(8),
          str(result["time"]).rjust(8),
          str(round(result["mean_err"], 2)).rjust(11),
          str(round(result["max_err"], 2)).rjust(11),
          "  是" if result["reached"] else "  否",
          "  是" if result["collided"] else "  否")
    save_png(scn, path, raw, log, result)
    save_gif(scn, path, raw, log)
print("图和动图已存到", OUT_DIR, "文件夹")

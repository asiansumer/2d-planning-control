"""生成 README 里用的图和动图，全部存到 results/ 文件夹。

运行（在项目根目录）：
    python -m scripts.make_figures            生成全部
    python -m scripts.make_figures hero pid   只生成名字里列出的几张
"""
import math
import os
import random
import sys

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.collections import LineCollection
from matplotlib.patches import Circle

from astar import astar
from bicycle import step
from control import p_control, pd_control, pi_control
from geometry import inflate, is_collision
from path_utils import densify, distance_to_path, path_length, shortcut
from pure_pursuit import find_target, steer_to_point
from rrt import rrt, rrt_star
from scenarios import narrow, plan, random_obstacles, u_turn
from simulate import simulate

OUT = "results"
MARGIN = 1.5
CAR_R = 0.9

C = {
    "bg": "#f7f7f5",
    "obs": "#3d4451",
    "margin": "#9aa3ad",
    "tree": "#a8d5ba",
    "raw": "#f4a261",
    "path": "#e63946",
    "car": "#1d6fd1",
    "car2": "#7b2cbf",
    "target": "#ff9f1c",
    "start": "#2a9d8f",
    "hit": "#d62828",
    "grid": "#d9d9d9",
}

plt.rcParams.update({
    "font.size": 11,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "axes.titleweight": "bold",
    "axes.titlesize": 12,
    "legend.frameon": False,
})


def save(fig, name):
    fig.savefig(os.path.join(OUT, name), dpi=130, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("  saved", name)


def draw_world(ax, scn, show_margin=True):
    """画地图底板：障碍物、膨胀范围、起点终点。"""
    ax.set_facecolor(C["bg"])
    for cx, cy, r in scn["obstacles"]:
        if show_margin:
            ax.add_patch(Circle((cx, cy), r + MARGIN, color=C["margin"], alpha=0.12, lw=0))
        ax.add_patch(Circle((cx, cy), r, color=C["obs"], lw=0))
    ax.plot(*scn["start"], "o", color=C["start"], ms=10, zorder=5)
    ax.plot(*scn["goal"], "*", color=C["path"], ms=16, zorder=5)
    ax.set_xlim(0, scn["width"])
    ax.set_ylim(0, scn["height"])
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)


def tree_segments(nodes, parent, k):
    """前 k 个节点和它们父节点之间的连线，给 LineCollection 用。"""
    segs = []
    for node in nodes[:k]:
        p = parent.get(node)
        if p is not None:
            segs.append([p, node])
    return segs


def car_artists(ax, color):
    """车：一个圆（车身）+ 一条短线（车头朝向）+ 预瞄线和预瞄点。"""
    body = Circle((0, 0), CAR_R, color=color, alpha=0.85, zorder=6)
    ax.add_patch(body)
    nose, = ax.plot([], [], "-", color="white", lw=2.2, zorder=7)
    look, = ax.plot([], [], "--", color=C["target"], lw=1.2, zorder=5)
    tgt, = ax.plot([], [], "o", color=C["target"], ms=5, zorder=6)
    trail, = ax.plot([], [], "-", color=color, lw=2.2, zorder=4)
    return body, nose, look, tgt, trail


def update_car(arts, log, k, color):
    body, nose, look, tgt, trail = arts
    x, y, th = log["x"][k], log["y"][k], log["theta"][k]
    body.center = (x, y)
    body.set_color(C["hit"] if any(log["hit"][:k + 1]) else color)
    nose.set_data([x, x + CAR_R * math.cos(th)], [y, y + CAR_R * math.sin(th)])
    look.set_data([x, log["tx"][k]], [y, log["ty"][k]])
    tgt.set_data([log["tx"][k]], [log["ty"][k]])
    trail.set_data(log["x"][:k + 1], log["y"][:k + 1])


# ---------------------------------------------------------------- 1. 主图动画

def city():
    """主图用的地图：几排“楼”加一些随机小障碍。"""
    obs = []
    for x0, y0, n, dx, dy in [(8, 4, 4, 0, 1.6), (8, 15, 5, 0, 1.6), (17, 9, 6, 0, 1.6),
                              (26, 2, 4, 0, 1.6), (26, 14, 5, 0, 1.6), (33, 8, 4, 1.6, 0)]:
        for i in range(n):
            obs.append((x0 + i * dx, y0 + i * dy, 1.2))
    rng = random.Random(7)
    while len(obs) < 34:
        ob = (rng.uniform(3, 38), rng.uniform(2, 23), rng.uniform(0.5, 0.9))
        if math.hypot(ob[0] - 2, ob[1] - 2) > 4 and math.hypot(ob[0] - 38, ob[1] - 22) > 4:
            obs.append(ob)
    return {"name": "city", "width": 40, "height": 25, "start": (2, 2), "goal": (38, 22), "obstacles": obs}


def fig_hero():
    scn = city()
    big = inflate(scn["obstacles"], MARGIN)
    random.seed(2)
    raw, nodes, parent = rrt_star(scn["start"], scn["goal"], big, scn["width"], scn["height"], max_iter=4000)
    smooth = densify(shortcut(raw, big))
    log, res = simulate(smooth, scn["obstacles"], car_radius=CAR_R)
    print("  hero:", res)

    fig, ax = plt.subplots(figsize=(10, 6.6))
    fig.subplots_adjust(left=0.01, right=0.99, bottom=0.01, top=0.93)
    draw_world(ax, scn)
    lc = LineCollection([], colors=C["tree"], linewidths=0.6, zorder=1)
    ax.add_collection(lc)
    raw_line, = ax.plot([], [], "-", color=C["raw"], lw=1.4, zorder=2)
    path_line, = ax.plot([], [], "--", color=C["path"], lw=1.6, zorder=3)
    arts = car_artists(ax, C["car"])
    title = ax.set_title("")

    n_tree = 30
    n_path = 12
    drive = list(range(0, len(log["x"]), 2))
    total = n_tree + n_path + len(drive) + 15

    def update(f):
        if f < n_tree:
            k = int(len(nodes) * (f + 1) / n_tree)
            lc.set_segments(tree_segments(nodes, parent, k))
            title.set_text("1. RRT* grows a tree   (nodes: " + str(k) + ")")
        elif f < n_tree + n_path:
            raw_line.set_data([p[0] for p in raw], [p[1] for p in raw])
            if f >= n_tree + n_path // 2:
                path_line.set_data([p[0] for p in smooth], [p[1] for p in smooth])
                title.set_text("2. Shortcut the path   (" + str(round(path_length(raw), 1)) + " m -> " + str(round(path_length(smooth), 1)) + " m)")
            else:
                title.set_text("2. Best path found on the tree")
        else:
            lc.set_alpha(0.35)
            k = drive[min(f - n_tree - n_path, len(drive) - 1)]
            update_car(arts, log, k, C["car"])
            title.set_text("3. Pure Pursuit + PI drive it   t = " + format(log["t"][k], ".1f") + " s   v = " + format(log["v"][k], ".1f") + " m/s")
        return []

    anim = FuncAnimation(fig, update, frames=total)
    anim.save(os.path.join(OUT, "hero.gif"), writer=PillowWriter(fps=15), dpi=72)
    plt.close(fig)
    print("  saved hero.gif")


# ---------------------------------------------------------------- 2. U 弯：开慢 vs 开快

def fig_uturn_speed():
    scn = u_turn()
    path, raw = plan(scn, margin=MARGIN)
    speeds = [4, 8]
    logs = []
    for v in speeds:
        log, res = simulate(path, scn["obstacles"], v_cruise=v, car_radius=CAR_R)
        print("  u_turn v =", v, res)
        logs.append((log, res))

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.4))
    fig.subplots_adjust(left=0.01, right=0.99, bottom=0.02, top=0.9, wspace=0.04)
    arts = []
    for ax, v, (log, res) in zip(axes, speeds, logs):
        draw_world(ax, scn)
        ax.plot([p[0] for p in path], [p[1] for p in path], "--", color=C["path"], lw=1.4)
        arts.append(car_artists(ax, C["car"]))
    n = max(len(log["x"]) for log, _ in logs)
    frames = list(range(0, n, 2)) + [n - 1] * 15

    def update(k):
        for ax, v, a, (log, res) in zip(axes, speeds, arts, logs):
            kk = min(k, len(log["x"]) - 1)
            update_car(a, log, kk, C["car"])
            hit = any(log["hit"][:kk + 1])
            label = "v = " + str(v) + " m/s   max err " + format(max(log["err"][:kk + 1]), ".2f") + " m"
            ax.set_title(label + ("   COLLISION" if hit else ""), color=C["hit"] if hit else "black")
        return []

    anim = FuncAnimation(fig, update, frames=frames)
    anim.save(os.path.join(OUT, "uturn_speed.gif"), writer=PillowWriter(fps=20), dpi=80)
    plt.close(fig)
    print("  saved uturn_speed.gif")


# ---------------------------------------------------------------- 3. 预瞄距离

S_PATH = [(i * 0.2, 3 * math.sin(i * 0.2 / 5)) for i in range(501)]


def pursue(v, Ld, T=8.0, dt=0.1):
    """固定车速、固定预瞄距离，从路线下方 1 米出发跟 S 弯。返回 (xs, ys, 平均误差)。"""
    x, y, th = 0.0, -1.0, 0.0
    xs, ys, errs = [x], [y], []
    md = math.radians(35)
    for i in range(int(T / dt)):
        tx, ty = find_target(x, y, S_PATH, Ld)
        d = max(-md, min(md, steer_to_point(x, y, th, tx, ty)))
        x, y, th = step(x, y, th, v, d, dt)
        xs.append(x)
        ys.append(y)
        errs.append(distance_to_path(x, y, S_PATH))
        if x > 95:
            break
    return xs, ys, sum(errs) / len(errs)


def fig_lookahead():
    speeds = [4, 6, 8, 10, 12, 14, 16]
    lds = [0.5, 1, 1.5, 2, 3, 4, 6, 8, 10]
    grid = np.array([[pursue(v, L)[2] for L in lds] for v in speeds])

    fig, ax = plt.subplots(figsize=(7.5, 4.2))
    im = ax.imshow(np.minimum(grid, 2.0), cmap="RdYlGn_r", aspect="auto", origin="lower", vmin=0, vmax=2.0)
    ax.set_xticks(range(len(lds)))
    ax.set_xticklabels([str(L) for L in lds])
    ax.set_yticks(range(len(speeds)))
    ax.set_yticklabels([str(v) for v in speeds])
    for i in range(len(speeds)):
        for j in range(len(lds)):
            ax.text(j, i, format(grid[i, j], ".2f"), ha="center", va="center", fontsize=8,
                    color="white" if grid[i, j] > 1.2 else "black")
    ax.set_xlabel("look-ahead distance Ld (m)")
    ax.set_ylabel("speed v (m/s)")
    ax.set_title("Mean lateral error on an S-curve (start 1 m off the path)")
    fig.colorbar(im, ax=ax, label="mean error (m), clipped at 2")
    save(fig, "lookahead_heatmap.png")

    v = 12
    cases = [(1.0, "#d62828", "Ld = 1 m: oscillates"), (3.0, C["car"], "Ld = 3 m: good"), (10.0, C["car2"], "Ld = 10 m: cuts corners")]
    runs = [pursue(v, L) for L, _, _ in cases]
    fig, ax = plt.subplots(figsize=(10, 2.7))
    fig.subplots_adjust(left=0.05, right=0.98, top=0.66, bottom=0.12)
    ax.set_facecolor(C["bg"])
    ax.plot([p[0] for p in S_PATH], [p[1] for p in S_PATH], "--", color="gray", lw=1.2, label="path")
    lines = []
    dots = []
    for (L, col, lab), (xs, ys, e) in zip(cases, runs):
        ln, = ax.plot([], [], "-", color=col, lw=2, label=lab + "  (mean err " + format(e, ".2f") + " m)")
        dt, = ax.plot([], [], "o", color=col, ms=7)
        lines.append(ln)
        dots.append(dt)
    ax.set_xlim(0, 96)
    ax.set_ylim(-6, 6)
    ax.set_aspect("equal")
    ax.legend(loc="lower center", ncol=2, fontsize=9, bbox_to_anchor=(0.5, 1.02), frameon=False)
    fig.suptitle("Pure Pursuit at v = 12 m/s", y=0.97, fontsize=12)
    n = max(len(r[0]) for r in runs)

    def update(k):
        for ln, dt, (xs, ys, e) in zip(lines, dots, runs):
            kk = min(k, len(xs) - 1)
            ln.set_data(xs[:kk + 1], ys[:kk + 1])
            dt.set_data([xs[kk]], [ys[kk]])
        return []

    anim = FuncAnimation(fig, update, frames=list(range(n)) + [n - 1] * 12)
    anim.save(os.path.join(OUT, "lookahead.gif"), writer=PillowWriter(fps=15), dpi=80)
    plt.close(fig)
    print("  saved lookahead.gif")


# ---------------------------------------------------------------- 4. 安全余量取舍

def fig_margin():
    margins = [0.3, 0.6, 0.9, 1.2, 1.5, 1.8, 2.1]
    seeds = list(range(12))
    hit_rate, mean_err, found = [], [], []
    for m in margins:
        hits, errs, ok = 0, [], 0
        for s in seeds:
            scn = random_obstacles(seed=s)
            path, raw = plan(scn, margin=m, seed=s, max_iter=2000)
            if path is None:
                continue
            ok += 1
            log, res = simulate(path, scn["obstacles"], car_radius=CAR_R)
            hits += res["collided"]
            errs.append(res["mean_err"])
        found.append(ok / len(seeds) * 100)
        hit_rate.append(hits / max(ok, 1) * 100)
        mean_err.append(sum(errs) / max(len(errs), 1))
        print("  margin", m, "found", ok, "hits", hits, "err", round(mean_err[-1], 3))

    fig, ax1 = plt.subplots(figsize=(7.5, 4.2))
    ax1.bar(margins, hit_rate, width=0.22, color=C["hit"], alpha=0.8, label="collision rate")
    ax1.set_xlabel("planning margin around obstacles (m)")
    ax1.set_ylabel("collision rate / path found (%)")
    ax1.set_ylim(0, 105)
    ax1.plot(margins, found, "s--", color="gray", lw=1.5, label="path found")
    ax1.axvline(CAR_R, color="gray", ls=":", lw=1.2)
    ax1.text(CAR_R + 0.04, 84, "car half-width 0.9 m", fontsize=9, color="gray")
    ax2 = ax1.twinx()
    ax2.spines["right"].set_visible(True)
    ax2.plot(margins, mean_err, "o-", color=C["car"], lw=2, label="mean tracking error")
    ax2.set_ylabel("mean tracking error (m)", color=C["car"])
    ax2.set_ylim(0, max(mean_err) * 1.4)
    ax1.set_title("Tracking accurately is not the same as being safe (12 random maps)")
    h1, l1 = ax1.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax1.legend(h1 + h2, l1 + l2, loc="center right", fontsize=9)
    save(fig, "margin_tradeoff.png")


# ---------------------------------------------------------------- 5. RRT* 收敛

def fig_convergence():
    scn = narrow()
    big = inflate(scn["obstacles"], MARGIN)
    iters = [300, 500, 1000, 2000, 4000]
    seeds = list(range(8))
    star = np.full((len(seeds), len(iters)), np.nan)
    plain = []
    for i, s in enumerate(seeds):
        random.seed(s)
        p, _, _ = rrt(scn["start"], scn["goal"], big, scn["width"], scn["height"], max_iter=4000)
        if p is not None:
            plain.append(path_length(p))
        for j, n in enumerate(iters):
            random.seed(s)
            p, _, _ = rrt_star(scn["start"], scn["goal"], big, scn["width"], scn["height"], max_iter=n)
            if p is not None:
                star[i, j] = path_length(p)
    print("  rrt mean", np.mean(plain), "rrt* means", np.nanmean(star, axis=0))
    fig, ax = plt.subplots(figsize=(7, 4))
    m = np.nanmean(star, axis=0)
    sd = np.nanstd(star, axis=0)
    ax.fill_between(iters, m - sd, m + sd, color=C["car"], alpha=0.15)
    ax.plot(iters, m, "o-", color=C["car"], lw=2, label="RRT* (mean ± std, 8 seeds)")
    ax.axhline(np.mean(plain), color=C["raw"], lw=2, ls="--", label="RRT (stops at first path)")
    ax.set_xscale("log")
    ax.set_xticks(iters)
    ax.set_xticklabels([str(n) for n in iters])
    ax.minorticks_off()
    ax.set_xlabel("RRT* iterations")
    ax.set_ylabel("path length (m)")
    ax.set_title("More iterations, shorter paths (narrow-passage map)")
    ax.set_ylim(29, 38.5)
    ax.legend(loc="lower left")
    save(fig, "rrt_star_convergence.png")

    random.seed(0)
    p1, n1, par1 = rrt(scn["start"], scn["goal"], big, scn["width"], scn["height"])
    random.seed(0)
    p2, n2, par2 = rrt_star(scn["start"], scn["goal"], big, scn["width"], scn["height"], max_iter=3000)
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.4))
    for ax, name, p, nodes, parent in [(axes[0], "RRT", p1, n1, par1), (axes[1], "RRT*", p2, n2, par2)]:
        draw_world(ax, scn)
        ax.add_collection(LineCollection(tree_segments(nodes, parent, len(nodes)), colors=C["tree"], linewidths=0.6))
        ax.plot([q[0] for q in p], [q[1] for q in p], "-", color=C["car"], lw=2.2)
        ax.set_title(name + ":  " + str(len(nodes)) + " nodes,  path " + format(path_length(p), ".1f") + " m")
    save(fig, "rrt_vs_rrt_star.png")


# ---------------------------------------------------------------- 6. PID

def fig_pid():
    dt, drag = 0.1, 0.5
    fig, axes = plt.subplots(1, 3, figsize=(13, 3.6))

    ax = axes[0]
    for kp, col in [(0.5, C["raw"]), (2.0, C["car"])]:
        v, vs = 0.0, [0.0]
        for i in range(100):
            a = p_control(10, v, kp)
            v = v + (a - drag * v) * dt
            vs.append(v)
        ax.plot(np.arange(101) * dt, vs, color=col, lw=2, label="kp = " + str(kp) + "  -> " + format(vs[-1], ".1f"))
    ax.axhline(10, color="gray", ls="--", lw=1)
    ax.set_title("P only: steady-state error")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("speed (m/s)")
    ax.legend(fontsize=9)

    ax = axes[1]
    for ki, col in [(1.0, C["car"]), (5.0, C["hit"])]:
        v, integ, vs = 0.0, 0.0, [0.0]
        for i in range(100):
            a, integ = pi_control(10, v, integ, 2.0, ki, dt)
            v = v + (a - drag * v) * dt
            vs.append(v)
        ax.plot(np.arange(101) * dt, vs, color=col, lw=2, label="ki = " + str(ki) + "  peak " + format(max(vs), ".1f"))
    ax.axhline(10, color="gray", ls="--", lw=1)
    ax.set_title("PI: error removed, large ki overshoots")
    ax.set_xlabel("time (s)")
    ax.legend(fontsize=9)

    ax = axes[2]
    for kd, col in [(0.0, C["hit"]), (2.0, C["car"])]:
        x, v, xs = 0.0, 0.0, [0.0]
        for i in range(200):
            a = pd_control(20, x, v, 1.0, kd)
            v = v + a * dt
            x = x + v * dt
            xs.append(x)
        ax.plot(np.arange(201) * dt, xs, color=col, lw=2, label="kd = " + str(kd))
    ax.axhline(20, color="gray", ls="--", lw=1)
    ax.set_title("PD stop line: D adds damping")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("position (m)")
    ax.set_ylim(-2, 50)
    ax.legend(fontsize=9, loc="upper right", ncol=2)
    fig.tight_layout()
    save(fig, "pid.png")


# ---------------------------------------------------------------- 7. A* vs RRT*

def fig_astar():
    scn = narrow()
    big = inflate(scn["obstacles"], MARGIN)
    W, H = scn["width"], scn["height"]
    blocked = set()
    for gx in range(W):
        for gy in range(H):
            if is_collision(gx + 0.5, gy + 0.5, big):
                blocked.add((gx, gy))
    s = (int(scn["start"][0]), int(scn["start"][1]))
    g = (int(scn["goal"][0]), int(scn["goal"][1]))
    cells = astar(s, g, blocked, W, H)
    a_path = [(c[0] + 0.5, c[1] + 0.5) for c in cells]
    smooth, raw = plan(scn, margin=MARGIN)

    fig, ax = plt.subplots(figsize=(9, 6))
    draw_world(ax, scn)
    for (gx, gy) in blocked:
        ax.add_patch(plt.Rectangle((gx, gy), 1, 1, color=C["margin"], alpha=0.25, lw=0))
    ax.plot([p[0] for p in a_path], [p[1] for p in a_path], "-", color=C["raw"], lw=2.4,
            label="A* on a 1 m grid, 4 neighbours: " + format(path_length(a_path), ".1f") + " m")
    ax.plot([p[0] for p in smooth], [p[1] for p in smooth], "-", color=C["car"], lw=2.4,
            label="RRT* + shortcut: " + format(path_length(smooth), ".1f") + " m")
    ax.legend(loc="lower right", fontsize=9, frameon=True)
    ax.set_title("A* vs RRT* on the same map (grey cells = blocked after inflation)")
    save(fig, "astar_vs_rrt_star.png")


FIGS = {
    "hero": fig_hero,
    "uturn": fig_uturn_speed,
    "lookahead": fig_lookahead,
    "margin": fig_margin,
    "convergence": fig_convergence,
    "pid": fig_pid,
    "astar": fig_astar,
}

if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    names = sys.argv[1:] or list(FIGS)
    for name in names:
        print(name)
        FIGS[name]()

"""闭环仿真：Pure Pursuit 控制转向 + PI 控制车速 + 自行车模型。"""
import math

from bicycle import step
from control import pi_control
from geometry import inflate, is_collision
from path_utils import distance_to_path
from pure_pursuit import find_target, nearest_index, steer_to_point


def simulate(path, obstacles, v_cruise=5.0, dt=0.1, L=2.5, max_time=60.0,
             car_radius=0.9, k_ld=0.8, ld_min=2.0, max_delta=math.radians(35),
             kp=2.0, ki=1.0, drag=0.5, goal_tol=0.5):
    """让车沿 path 开到终点，返回 (log, result)。

    log：每一步的记录，键为 t, x, y, theta, v, err（横向误差）, tx, ty（预瞄点）, hit（是否碰撞）。
    result：reached（是否到达）, time（用时）, mean_err / max_err（平均 / 最大横向误差）, collided（是否碰撞过）。
    车身视为半径 car_radius 的圆；预瞄距离 Ld = max(ld_min, k_ld · v)；转角限制在 ±max_delta。
    """
    # 车放在路线起点，车头朝向路线的第一段
    x, y = path[0]
    theta = math.atan2(path[1][1] - y, path[1][0] - x)
    v = 0.0
    integral = 0.0
    goal = path[-1]
    # 车身是半径 car_radius 的圆：车中心进入膨胀后的障碍物，等价于车身碰到原障碍物
    body_obstacles = inflate(obstacles, car_radius)
    # idx：车当前对应路线上的哪个点，只往前推进，不回头
    idx = 0

    log = {"t": [0.0], "x": [x], "y": [y], "theta": [theta], "v": [v], "err": [0.0],
           "tx": [x], "ty": [y], "hit": [False]}
    t = 0.0
    reached = False
    collided = False

    while t < max_time:
        dist_goal = math.hypot(goal[0] - x, goal[1] - y)
        if dist_goal < goal_tol:
            reached = True
            break

        # 1. 车速：平时按巡航速度，离终点不到 v_cruise 米时按距离减速
        target_v = min(v_cruise, 1.0 * dist_goal)
        a, integral = pi_control(target_v, v, integral, kp, ki, dt)
        v = v + (a - drag * v) * dt

        # 2. 转向：只在 idx 之后的 50 个点里找最近点，防止 U 弯时跳到对面那段路上
        window = path[idx:idx + 50]
        idx = idx + nearest_index(x, y, window)
        Ld = max(ld_min, k_ld * v)
        tx, ty = find_target(x, y, path[idx:], Ld)
        if math.hypot(tx - x, ty - y) < 1e-6:
            delta = 0.0
        else:
            delta = steer_to_point(x, y, theta, tx, ty, L)
        # 前轮转角有物理上限
        delta = max(-max_delta, min(max_delta, delta))

        # 3. 自行车模型走一步
        x, y, theta = step(x, y, theta, v, delta, dt, L)
        t = t + dt

        # 4. 记录这一步
        err = distance_to_path(x, y, path)
        hit = is_collision(x, y, body_obstacles)
        if hit:
            collided = True
        log["t"].append(t)
        log["x"].append(x)
        log["y"].append(y)
        log["theta"].append(theta)
        log["v"].append(v)
        log["err"].append(err)
        log["tx"].append(tx)
        log["ty"].append(ty)
        log["hit"].append(hit)

    errs = log["err"][1:]
    result = {
        "reached": reached,
        "time": round(t, 1),
        "mean_err": sum(errs) / len(errs),
        "max_err": max(errs),
        "collided": collided,
    }
    return log, result

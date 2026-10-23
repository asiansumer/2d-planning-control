"""P / PI / PD 控制器，输出都是加速度 a。"""


def p_control(target_v, v, kp):
    """P 控制：a = kp × 速度误差。"""
    error = target_v - v
    a = kp * error
    return a


def pi_control(target_v, v, integral, kp, ki, dt):
    """PI 控制：在 P 的基础上加上误差的累积（积分项）。

    integral 是到上一步为止的误差累积，每步由调用方传回来。
    返回 (a, new_integral)。
    """
    error = target_v - v
    new_integral = integral + error * dt
    a = kp * error + ki * new_integral
    return a, new_integral


def pd_control(target_x, x, v, kp, kd):
    """PD 控制：让车停在 target_x。

    P 项看离目标还有多远，D 项看正以多快的速度靠近（这里位置的导数就是车速 v）。
    """
    error = target_x - x
    a = kp * error - kd * v
    return a

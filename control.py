def p_control(target_v, v, kp):
    """P 控制：根据目标车速和当前车速，算出这一步的加速度 a。"""
    error = target_v - v
    a = kp * error
    return a


def pi_control(target_v, v, integral, kp, ki, dt):
    """PI 控制：返回这一步的加速度 a 和更新后的账本 new_integral。"""
    error = target_v - v
    new_integral = integral + error * dt          # 账本 = 旧账本 + 误差 × dt
    a = kp * error + ki * new_integral            # P 的部分 + I 的部分
    return a, new_integral


def pd_control(target_x, x, v, kp, kd):
    """PD 控制：让车停在 target_x。P 看离多远，D 看靠近得多快。返回加速度 a。"""
    error = target_x - x
    a = kp * error - kd * v
    return a

import pytest

from control import p_control, pi_control, pd_control


def test_p_control():
    # 目标 10，当前 4，Kp = 0.5：误差 6，加速度应该是 3
    assert p_control(10, 4, 0.5) == pytest.approx(3)
    assert p_control(10, 12, 0.5) == pytest.approx(-1)


def test_pi_control():
    # 目标 10，当前 8，账本 0，kp=2，ki=1，dt=0.1：误差 2，账本 0.2，a = 2×2 + 1×0.2 = 4.2
    a, new_integral = pi_control(10, 8, 0, 2, 1, 0.1)
    assert new_integral == pytest.approx(0.2)
    assert a == pytest.approx(4.2)


def test_pd_control():
    # 停止线 20，车在 15，车速 5，kp=1，kd=2：P 部分 5，D 部分 -10，a = -5（提前刹车）
    assert pd_control(20, 15, 5, 1, 2) == pytest.approx(-5)

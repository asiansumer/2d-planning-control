import pytest

from control import p_control, pd_control, pi_control


def test_p_control():
    # 目标 10、当前 4、kp = 0.5：误差 6，a = 3
    assert p_control(10, 4, 0.5) == pytest.approx(3)
    # 超速时 a 为负，即减速
    assert p_control(10, 12, 0.5) == pytest.approx(-1)


def test_pi_control():
    # 误差 2，积分 0 + 2 × 0.1 = 0.2，a = 2 × 2 + 1 × 0.2 = 4.2
    a, new_integral = pi_control(10, 8, 0, 2, 1, 0.1)
    assert new_integral == pytest.approx(0.2)
    assert a == pytest.approx(4.2)


def test_pd_control():
    # 离停止线 5 米、车速 5：P 项 5，D 项 -10，a = -5，提前刹车
    assert pd_control(20, 15, 5, 1, 2) == pytest.approx(-5)

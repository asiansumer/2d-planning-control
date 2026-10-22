import pytest

from rrt import steer


def test_steer():
    # 小题：从 (2, 0) 朝 (2, 4) 伸 1 米，到 (2, 1)
    assert steer((2, 0), (2, 4), 1) == pytest.approx((2, 1))
    # 随机点只有 0.5 米远，比 1 米近：直接到它
    assert steer((0, 0), (0.5, 0), 1) == pytest.approx((0.5, 0))

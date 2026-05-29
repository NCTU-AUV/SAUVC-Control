import pytest

from xy_control.bottom_camera_pid_bridge_node import _clamp_symmetric


def test_clamp_symmetric_limits_positive_and_negative_values():
    assert _clamp_symmetric(12.0, 10.0) == pytest.approx(10.0)
    assert _clamp_symmetric(-12.0, 10.0) == pytest.approx(-10.0)
    assert _clamp_symmetric(3.0, 10.0) == pytest.approx(3.0)


def test_clamp_symmetric_disables_limit_when_non_positive():
    assert _clamp_symmetric(12.0, 0.0) == pytest.approx(12.0)
    assert _clamp_symmetric(-12.0, -1.0) == pytest.approx(-12.0)

import math

import pytest

from atlas.control.pid import PID


def test_proportional_only_output():
    pid = PID(kp=2.0)
    assert pid.update(3.0, 0.1) == pytest.approx(6.0)


def test_derivative_is_zero_on_first_sample():
    pid = PID(kp=0.0, kd=1.0)
    assert pid.update(1.0, 0.1) == pytest.approx(0.0)


def test_derivative_reacts_to_error_change():
    pid = PID(kp=0.0, kd=1.0)
    pid.update(1.0, 0.1)
    assert pid.update(2.0, 0.1) == pytest.approx(10.0)


def test_derivative_is_zero_for_constant_error():
    pid = PID(kp=0.0, kd=1.0)
    pid.update(1.0, 0.1)
    assert pid.update(1.0, 0.1) == pytest.approx(0.0)


def test_integral_accumulates():
    pid = PID(kp=0.0, ki=2.0)
    assert pid.update(1.0, 0.5) == pytest.approx(1.0)
    assert pid.integral == pytest.approx(0.5)
    assert pid.update(1.0, 0.5) == pytest.approx(2.0)
    assert pid.integral == pytest.approx(1.0)


def test_integral_is_clamped():
    pid = PID(kp=0.0, ki=1.0, integral_limit=0.2)
    for _ in range(10):
        pid.update(1.0, 1.0)
    assert pid.integral == pytest.approx(0.2)


def test_output_is_clamped_both_directions():
    pid = PID(kp=100.0, output_limit=2.0)
    assert pid.update(1.0, 0.1) == pytest.approx(2.0)
    assert pid.update(-1.0, 0.1) == pytest.approx(-2.0)


def test_anti_windup_freezes_integration_when_saturated():
    pid = PID(kp=0.0, ki=1.0, output_limit=1.0)
    assert pid.update(20.0, 0.1) == pytest.approx(1.0)
    assert pid.integral == pytest.approx(0.0)
    # reversing the error should reverse the output immediately, not after
    # unwinding a large stored integral
    assert pid.update(-20.0, 0.1) == pytest.approx(-1.0)
    assert pid.integral == pytest.approx(0.0)


def test_reset_clears_integral_and_derivative_history():
    pid = PID(kp=1.0, ki=1.0, kd=1.0)
    pid.update(1.0, 0.1)
    pid.update(2.0, 0.1)
    assert pid.integral != 0

    pid.reset()
    assert pid.integral == 0
    fresh = PID(kp=1.0, ki=1.0, kd=1.0)
    assert pid.update(1.0, 0.1) == pytest.approx(fresh.update(1.0, 0.1))


def test_pid_eliminates_steady_state_error_with_a_disturbance():
    pid = PID(kp=1.0, ki=1.0, output_limit=10.0)
    dt, setpoint, y = 0.01, 1.0, 0.0
    for _ in range(3000):
        u = pid.update(setpoint - y, dt)
        y += (u - 0.5) * dt  # constant disturbance requires u = 0.5

    assert abs(setpoint - y) < 0.01


def test_pid_beats_p_only_on_the_same_plant():
    def final_error(controller):
        dt, setpoint, y = 0.01, 1.0, 0.0
        for _ in range(3000):
            u = controller.update(setpoint - y, dt)
            y += (u - 0.5) * dt
        return abs(setpoint - y)

    p_only = final_error(PID(kp=1.0))
    pid = final_error(PID(kp=1.0, ki=1.0))
    assert pid < p_only / 10


def test_non_positive_dt_is_rejected():
    with pytest.raises(ValueError, match="dt"):
        PID(kp=1.0).update(1.0, 0.0)


def test_non_finite_error_is_rejected():
    with pytest.raises(ValueError, match="finite"):
        PID(kp=1.0).update(math.nan, 0.1)


def test_invalid_limits_are_rejected():
    with pytest.raises(ValueError, match="output_limit"):
        PID(kp=1.0, output_limit=0)
    with pytest.raises(ValueError, match="integral_limit"):
        PID(kp=1.0, integral_limit=-1)

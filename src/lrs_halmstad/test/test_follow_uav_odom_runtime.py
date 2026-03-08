import math

from lrs_halmstad.follow_uav_odom import compute_rate_limited_axis_value


def test_rate_limited_axis_steps_upward_without_teleporting():
    stepped = compute_rate_limited_axis_value(
        7.0,
        9.0,
        tick_hz=20.0,
        max_speed_per_s=5.0,
        gain=2.0,
    )
    assert math.isclose(stepped, 7.2, abs_tol=1e-6)


def test_rate_limited_axis_steps_downward_without_teleporting():
    stepped = compute_rate_limited_axis_value(
        9.0,
        7.0,
        tick_hz=20.0,
        max_speed_per_s=5.0,
        gain=2.0,
    )
    assert math.isclose(stepped, 8.8, abs_tol=1e-6)


def test_rate_limited_axis_respects_custom_gain_and_speed_cap():
    stepped = compute_rate_limited_axis_value(
        7.0,
        9.0,
        tick_hz=20.0,
        max_speed_per_s=8.0,
        gain=4.0,
    )
    assert math.isclose(stepped, 7.4, abs_tol=1e-6)


def test_rate_limited_axis_snaps_to_target_once_close_enough():
    stepped = compute_rate_limited_axis_value(
        8.995,
        9.0,
        tick_hz=20.0,
        max_speed_per_s=5.0,
        gain=2.0,
    )
    assert math.isclose(stepped, 9.0, abs_tol=1e-9)


def test_rate_limited_axis_reaches_target_after_repeated_ticks():
    current = 7.0
    for _ in range(200):
        current = compute_rate_limited_axis_value(
            current,
            9.0,
            tick_hz=20.0,
            max_speed_per_s=5.0,
            gain=2.0,
        )
    assert math.isclose(current, 9.0, abs_tol=1e-9)


def test_rate_limited_axis_holds_position_when_speed_cap_is_zero():
    stepped = compute_rate_limited_axis_value(
        7.0,
        9.0,
        tick_hz=20.0,
        max_speed_per_s=0.0,
        gain=2.0,
    )
    assert math.isclose(stepped, 7.0, abs_tol=1e-9)

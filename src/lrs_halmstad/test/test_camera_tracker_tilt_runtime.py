import math

from lrs_halmstad.camera_tracker import apply_deadband_command, should_prefer_command_pose
from lrs_halmstad.follow_math import compute_camera_tilt_deg, compute_leader_look_target


def test_should_prefer_fresh_command_pose_when_newer_than_actual():
    assert should_prefer_command_pose(
        have_cmd=True,
        cmd_stamp_ns=2_000_000_000,
        have_actual=True,
        actual_stamp_ns=1_500_000_000,
        now_ns=2_100_000_000,
        pose_timeout_s=1.0,
    )


def test_should_not_prefer_stale_command_pose_over_fresh_actual():
    assert not should_prefer_command_pose(
        have_cmd=True,
        cmd_stamp_ns=1_000_000_000,
        have_actual=True,
        actual_stamp_ns=2_000_000_000,
        now_ns=2_100_000_000,
        pose_timeout_s=0.5,
    )


def test_deadband_holds_previous_value_for_small_tilt_changes():
    assert math.isclose(
        apply_deadband_command(10.3, 10.0, 0.5),
        10.0,
        abs_tol=1e-6,
    )


def test_deadband_allows_large_tilt_changes():
    assert math.isclose(
        apply_deadband_command(10.8, 10.0, 0.5),
        10.8,
        abs_tol=1e-6,
    )


def test_camera_tilt_gets_more_downward_when_altitude_increases():
    base_tilt = compute_camera_tilt_deg(
        0.0, 0.0, 7.0, 0.0,
        7.0, 0.0, 0.0,
        0.0, 0.0, 0.27, 0.0,
    )
    higher_tilt = compute_camera_tilt_deg(
        0.0, 0.0, 9.0, 0.0,
        7.0, 0.0, 0.0,
        0.0, 0.0, 0.27, 0.0,
    )
    assert higher_tilt < base_tilt


def test_camera_tilt_gets_less_downward_when_follow_distance_increases():
    near_tilt = compute_camera_tilt_deg(
        0.0, 0.0, 7.0, 0.0,
        5.0, 0.0, 0.0,
        0.0, 0.0, 0.27, 0.0,
    )
    far_tilt = compute_camera_tilt_deg(
        0.0, 0.0, 7.0, 0.0,
        9.0, 0.0, 0.0,
        0.0, 0.0, 0.27, 0.0,
    )
    assert far_tilt > near_tilt


def test_uniform_distance_scaling_preserves_tilt_angle():
    tilt_7_7 = compute_camera_tilt_deg(
        0.0, 0.0, 7.0, 0.0,
        7.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0,
    )
    tilt_10_10 = compute_camera_tilt_deg(
        0.0, 0.0, 10.0, 0.0,
        10.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0,
    )
    assert math.isclose(tilt_7_7, tilt_10_10, abs_tol=1e-6)


def test_leader_look_target_offsets_rotate_with_leader_yaw():
    target_x, target_y, target_z = compute_leader_look_target(
        leader_x=1.0,
        leader_y=2.0,
        leader_yaw=math.pi / 2.0,
        leader_z=0.0,
        leader_look_target_x_m=1.0,
        leader_look_target_y_m=0.5,
        leader_look_target_z_m=0.4,
    )
    assert math.isclose(target_x, 0.5, abs_tol=1e-6)
    assert math.isclose(target_y, 3.0, abs_tol=1e-6)
    assert math.isclose(target_z, 0.4, abs_tol=1e-6)


def test_leader_look_target_z_changes_tilt_target_height():
    target_x, target_y, target_z = compute_leader_look_target(
        leader_x=7.0,
        leader_y=0.0,
        leader_yaw=0.0,
        leader_z=0.0,
        leader_look_target_x_m=0.0,
        leader_look_target_y_m=0.0,
        leader_look_target_z_m=0.5,
    )
    tilt = compute_camera_tilt_deg(
        0.0, 0.0, 7.0, 0.0,
        target_x, target_y, target_z,
        0.0, 0.0, 0.27, 0.0,
    )
    ground_tilt = compute_camera_tilt_deg(
        0.0, 0.0, 7.0, 0.0,
        7.0, 0.0, 0.0,
        0.0, 0.0, 0.27, 0.0,
    )
    assert tilt > ground_tilt

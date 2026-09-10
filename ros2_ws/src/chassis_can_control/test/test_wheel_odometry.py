import math

from chassis_can_control.wheel_odometry import WheelOdometry


def test_first_sample_only_sets_baseline():
    odometry = WheelOdometry()
    sample = odometry.update(100, -50, 0.1, 1000.0, 0.4)
    assert sample is not None
    assert sample.x_m == 0.0
    assert sample.y_m == 0.0
    assert sample.yaw_rad == 0.0


def test_equal_counts_move_straight():
    odometry = WheelOdometry()
    odometry.update(0, 0, 0.1, 1000.0, 0.4)
    sample = odometry.update(1000, 1000, 0.1, 1000.0, 0.4)
    assert sample is not None
    assert math.isclose(sample.x_m, math.pi * 0.1, rel_tol=1e-6)
    assert math.isclose(sample.y_m, 0.0, abs_tol=1e-9)
    assert math.isclose(sample.yaw_rad, 0.0, abs_tol=1e-9)


def test_int32_wrap_is_small_delta():
    odometry = WheelOdometry()
    assert odometry.wrapped_count_delta(-2147483648, 2147483647) == 1
    assert odometry.wrapped_count_delta(2147483647, -2147483648) == -1


def test_imu_yaw_rate_is_blended_with_wheel_yaw():
    odometry = WheelOdometry()
    odometry.update(0, 0, 0.1, 1000.0, 0.4)
    sample = odometry.update(
        0,
        0,
        0.1,
        1000.0,
        0.4,
        imu_yaw_rate_radps=1.0,
        dt_s=0.1,
        imu_weight=0.8,
    )
    assert sample is not None
    assert math.isclose(sample.yaw_rad, 0.08, abs_tol=1e-9)


def test_missing_imu_uses_original_wheel_only_result():
    odometry = WheelOdometry()
    odometry.update(0, 0, 0.1, 1000.0, 0.4)
    sample = odometry.update(100, 100, 0.1, 1000.0, 0.4)
    assert sample is not None
    assert math.isclose(sample.yaw_rad, 0.0, abs_tol=1e-9)

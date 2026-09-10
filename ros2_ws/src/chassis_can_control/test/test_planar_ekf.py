"""独立数学与数据流测试，不需要 ROS、NumPy 或实物。"""

import math
import random

from chassis_can_control.planar_ekf import EkfConfig, EkfOdometry, PlanarEkf


def test_motion_jacobian_matches_numerical_derivative():
    state = [0.3, -0.2, 0.7, 0.4, -0.3]
    _, jacobian = PlanarEkf.motion(state, 0.07)
    for column in range(5):
        plus, minus = state[:], state[:]
        plus[column] += 1e-6
        minus[column] -= 1e-6
        a, _ = PlanarEkf.motion(plus, 0.07)
        b, _ = PlanarEkf.motion(minus, 0.07)
        for row in range(5):
            numerical = (a[row] - b[row]) / 2e-6
            assert math.isclose(numerical, jacobian[row][column], abs_tol=1e-8)


def test_scalar_kalman_update_and_noise_weight():
    ekf = PlanarEkf(EkfConfig(gyro_stddev=0.5))
    assert ekf.observe_gyro(1.0, 0.0)
    # P_omega=1, R=0.25，所以 K=0.8；Joseph 后 P=0.2。
    assert math.isclose(ekf.state[4], 0.8, abs_tol=1e-12)
    assert math.isclose(ekf.covariance[4][4], 0.2, abs_tol=1e-12)
    less_trusted = PlanarEkf(EkfConfig(gyro_stddev=2.0))
    less_trusted.observe_gyro(1.0, 0.0)
    assert less_trusted.state[4] < ekf.state[4]


def test_constant_turn_prediction_tracks_analytic_arc():
    ekf = PlanarEkf()
    ekf.state = [0.0, 0.0, 0.0, 0.3, 0.2]
    ekf.predict_to(0.0)
    for i in range(1, 501):
        ekf.predict_to(i * 0.02)
    assert abs(ekf.state[0] - 1.5 * math.sin(2.0)) < 1e-5
    assert abs(ekf.state[1] - 1.5 * (1.0 - math.cos(2.0))) < 1e-5
    assert abs(ekf.state[2] - 2.0) < 1e-10


def test_invalid_and_outlier_observations_cannot_poison_state():
    ekf = PlanarEkf()
    for i in range(20):
        ekf.observe_wheels(0.0, 0.0, i * 0.05)
        ekf.observe_gyro(0.0, i * 0.05)
    before = ekf.state[:]
    covariance = [row[:] for row in ekf.covariance]
    assert not ekf.observe_gyro(float('nan'), 0.95)
    assert not ekf.observe_gyro(9.0, 0.95)  # 有限且在硬阈值内，但 NIS 过大。
    assert not ekf.observe_wheels(float('inf'), 0.0, 0.95)
    assert not ekf.observe_wheels(0.0, 0.0, 0.8)
    assert ekf.state == before
    assert ekf.covariance == covariance


def test_covariance_is_symmetric_positive_definite_under_async_updates():
    random_source = random.Random(31)
    ekf = PlanarEkf()
    for i in range(600):
        timestamp = i * 0.01
        if i % 2 == 0:
            ekf.observe_gyro(0.2 + random_source.gauss(0, 0.03), timestamp)
        if i % 5 == 0:
            ekf.observe_wheels(0.3 + random_source.gauss(0, 0.02),
                               0.2 + random_source.gauss(0, 0.12), timestamp)
        p = ekf.covariance
        lower = [[0.0] * 5 for _ in range(5)]
        # Cholesky 分解是比仅检查对角线更强的正定性检查。
        for row in range(5):
            for column in range(row + 1):
                assert abs(p[row][column] - p[column][row]) < 1e-10
                value = p[row][column] - sum(
                    lower[row][k] * lower[column][k] for k in range(column))
                if row == column:
                    assert value > 0.0
                    lower[row][column] = math.sqrt(value)
                else:
                    lower[row][column] = value / lower[column][column]
    assert abs(ekf.state[3] - 0.3) < 0.05
    assert abs(ekf.state[4] - 0.2) < 0.08


def test_raw_observation_is_independent_of_imu_and_gyro_not_reused():
    a, b = EkfOdometry(), EkfOdometry()
    for item in (a, b):
        item.update(0, 0, 0.065, 60000, 0.162, 0.0)
    assert a.add_imu(0.4, 0.02)
    covariance = [row[:] for row in a.filter.covariance]
    assert not a.add_imu(0.4, 0.02)
    assert a.filter.covariance == covariance
    a.update(300, 300, 0.065, 60000, 0.162, 0.05)
    b.update(300, 300, 0.065, 60000, 0.162, 0.05)
    assert a.raw.sample() == b.raw.sample()
    assert a.sample().yaw_rad != b.sample().yaw_rad
    accepted_time = a.last_accepted_gyro_time
    a.update(600, 600, 0.065, 60000, 0.162, 0.1)
    assert a.last_accepted_gyro_time == accepted_time


def test_wheel_only_operation_and_imu_recovery():
    odom = EkfOdometry()
    for i in range(101):
        sample = odom.update(i * 300, i * 300, 0.065, 60000, 0.162, i * 0.05)
    expected = 30000 * math.pi * 0.065 / 60000
    assert abs(sample.x_m - expected) < 0.003
    assert abs(sample.yaw_rad) < 1e-12
    assert odom.add_imu(0.1, 5.02)
    assert odom.last_accepted_gyro_time == 5.02


def test_gap_rebase_preserves_pose_and_does_not_extrapolate_unknown_motion():
    odom = EkfOdometry()
    for i in range(10):
        odom.update(i * 300, i * 300, 0.065, 60000, 0.162, i * 0.05)
    before = odom.sample()
    raw_before = odom.raw.sample()
    covariance_before = odom.filter.covariance[0][0]
    assert not odom.add_imu(0.5, 20.0)
    after = odom.update(90000, 100000, 0.065, 60000, 0.162, 20.0)
    assert before == after
    assert raw_before == odom.raw.sample()
    assert odom.filter.state[3:] == [0.0, 0.0]
    assert odom.filter.covariance[0][0] > covariance_before
    assert odom.filter.gap_count == 1


def test_int32_wrap_jump_and_reset():
    odom = EkfOdometry()
    odom.update(2147483640, 2147483640, 0.065, 60000, 0.162, 0.0)
    odom.update(-2147483640, -2147483640, 0.065, 60000, 0.162, 0.05)
    assert math.isclose(odom.raw_speed, 16 * math.pi * 0.065 / 60000 / 0.05)
    before = odom.raw.sample()
    assert odom.update(0, 0, 0.065, 60000, 0.162, 0.1) is None
    assert odom.raw.sample() == before
    odom.update(300, 300, 0.065, 60000, 0.162, 0.15)
    assert odom.wheel_accepted
    odom.reset()
    assert odom.filter.state == [0.0] * 5
    assert odom.previous is None
    assert not odom.add_imu(0.2, 1.0)


def test_ros_covariance_mapping_and_unknown_axes():
    odom = EkfOdometry()
    odom.update(0, 0, 0.065, 60000, 0.162, 0.0)
    odom.update(300, 350, 0.065, 60000, 0.162, 0.05)
    pose, twist = odom.ros_covariances()
    for i, ros_i in enumerate((0, 1, 5)):
        for j, ros_j in enumerate((0, 1, 5)):
            assert pose[ros_i * 6 + ros_j] == odom.filter.covariance[i][j]
    assert pose[14] == 1e6
    assert twist[0] == odom.filter.covariance[3][3]
    assert twist[35] == odom.filter.covariance[4][4]


def test_invalid_configuration_and_timestamps():
    for invalid in (0.0, -1.0, float('nan'), float('inf'), 1e300):
        try:
            EkfConfig(gyro_stddev=invalid)
        except ValueError:
            pass
        else:
            raise AssertionError('invalid configuration accepted')
    odom = EkfOdometry()
    assert odom.update(0, 0, float('nan'), 60000, 0.162, 0.0) is None
    odom.update(0, 0, 0.065, 60000, 0.162, 0.0)
    assert odom.update(300, 300, 0.065, 60000, 0.162, 0.0) is None
    assert odom.previous == (0, 0, 0.0)

"""五状态平面 EKF；只用 Python 标准库，便于离线验证和逐行学习。

状态顺序始终是 [x, y, yaw, v, omega]，单位为 m、m、rad、m/s、rad/s。
编码器观测 v、omega，IMU 只观测 omega。编码器不是预测输入，不重复融合位置。
本版不估计 gyro bias，不使用加速度或虚构的绝对航向。
"""

from dataclasses import dataclass
import math

from .wheel_odometry import OdometrySample, WheelOdometry


def identity(size):
    return [[float(i == j) for j in range(size)] for i in range(size)]


def transpose(matrix):
    return [list(column) for column in zip(*matrix)]


def multiply(left, right):
    columns = transpose(right)
    return [[sum(a * b for a, b in zip(row, column)) for column in columns]
            for row in left]


def add(left, right):
    return [[a + b for a, b in zip(row_a, row_b)]
            for row_a, row_b in zip(left, right)]


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class EkfConfig:
    # 过程噪声是连续白噪声的幅度，平方后为谱密度。
    # acceleration_noise 单位 m/s^(3/2)，angular_acceleration_noise 为 rad/s^(3/2)。
    acceleration_noise: float = 0.8
    angular_acceleration_noise: float = 2.0
    wheel_speed_stddev: float = 0.04
    wheel_yaw_rate_stddev: float = 0.15
    gyro_stddev: float = 0.03
    innovation_gate: float = 5.0
    max_gap_s: float = 0.5
    max_wheel_speed_mps: float = 2.0
    max_yaw_rate_radps: float = 10.0

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or not 1e-6 <= value <= 100.0:
                raise ValueError(f"EKF {name} must be finite and in [1e-6, 100]")


class PlanarEkf:
    """运动模型非线性，用解析 Jacobian 传播 P；使用 Joseph 形式更新 P。"""

    def __init__(self, config=None):
        self.config = config or EkfConfig()
        self.reset()

    def reset(self):
        self.state = [0.0] * 5
        self.covariance = [[0.0] * 5 for _ in range(5)]
        for i, variance in enumerate([1e-6, 1e-6, 1e-6, 1.0, 1.0]):
            self.covariance[i][i] = variance
        self.time_s = None
        self.rejected_wheel = 0
        self.rejected_imu = 0
        self.gap_count = 0
        self.last_nis = 0.0

    @staticmethod
    def motion(state, dt):
        """中点运动模型及其对状态的导数 F，单独暴露以便数值验证。"""
        x, y, yaw, speed, rate = state
        middle = yaw + 0.5 * rate * dt
        cosine, sine = math.cos(middle), math.sin(middle)
        predicted = [x + speed * dt * cosine, y + speed * dt * sine,
                     wrap_angle(yaw + rate * dt), speed, rate]
        jacobian = identity(5)
        jacobian[0][2] = -speed * dt * sine
        jacobian[0][3] = dt * cosine
        jacobian[0][4] = -0.5 * speed * dt * dt * sine
        jacobian[1][2] = speed * dt * cosine
        jacobian[1][3] = dt * sine
        jacobian[1][4] = 0.5 * speed * dt * dt * cosine
        jacobian[2][4] = dt
        return predicted, jacobian

    def suspend(self, timestamp):
        """数据长时间中断：保留位姿，停止沿最后速度无限外推，增大不确定性。

        中断期间运动未知，不能补出真实轨迹；重建速度且清除旧速度相关性。
        """
        self.state[3:] = [0.0, 0.0]
        for i in (3, 4):
            for j in range(5):
                self.covariance[i][j] = self.covariance[j][i] = 0.0
            self.covariance[i][i] = 1.0
        for i in range(3):
            self.covariance[i][i] += 1.0
        self.time_s = timestamp
        self.gap_count += 1

    def predict_to(self, timestamp):
        if not math.isfinite(timestamp):
            return False
        if self.time_s is None:
            self.time_s = timestamp
            return True
        dt = timestamp - self.time_s
        if dt < 0.0:
            return False
        if dt > self.config.max_gap_s:
            self.suspend(timestamp)
            return True
        # 小步预测减小快速转弯时的离散化误差；Q 随真实 dt 缩放。
        while dt > 1e-12:
            step = min(dt, 0.02)
            self._predict_step(step)
            dt -= step
        self.time_s = timestamp
        return True

    def _predict_step(self, dt):
        predicted, jacobian = self.motion(self.state, dt)
        angle = self.state[2] + 0.5 * self.state[4] * dt
        direction = [math.cos(angle), math.sin(angle)]
        noise = [[0.0] * 5 for _ in range(5)]
        qv = self.config.acceleration_noise ** 2
        qw = self.config.angular_acceleration_noise ** 2
        # 积分白加速度模型：位置方差 q*dt^3/3，位置速度协方差 q*dt^2/2。
        # 每个短步以当前朝向近似平移噪声方向。
        for i in range(2):
            for j in range(2):
                noise[i][j] = direction[i] * direction[j] * qv * dt ** 3 / 3.0
            noise[i][3] = noise[3][i] = direction[i] * qv * dt ** 2 / 2.0
        noise[3][3] = qv * dt
        noise[2][2] = qw * dt ** 3 / 3.0
        noise[2][4] = noise[4][2] = qw * dt ** 2 / 2.0
        noise[4][4] = qw * dt
        self.covariance = add(
            multiply(multiply(jacobian, self.covariance), transpose(jacobian)), noise)
        self.state = predicted

    def _correct(self, indices, values, variances):
        """观测直接选择状态分量。只需逆 1x1 或 2x2，不依赖 NumPy。

        当前将 wheel v、wheel omega 视作独立观测误差的近似。
        编码器若左右误差不对称，二者可能相关，需由实车数据改为完整 R。
        """
        residual = [value - self.state[i] for i, value in zip(indices, values)]
        size = len(indices)
        innovation = [[self.covariance[i][j] for j in indices] for i in indices]
        for k in range(size):
            innovation[k][k] += variances[k]
        if size == 1:
            inverse = [[1.0 / innovation[0][0]]]
        else:
            a, b = innovation[0]
            c, d = innovation[1]
            determinant = a * d - b * c
            if determinant <= 0.0 or not math.isfinite(determinant):
                return False
            inverse = [[d / determinant, -b / determinant],
                       [-c / determinant, a / determinant]]
        nis = sum(residual[i] * inverse[i][j] * residual[j]
                  for i in range(size) for j in range(size))
        self.last_nis = nis
        if not math.isfinite(nis) or nis > self.config.innovation_gate ** 2:
            return False
        pht = [[row[i] for i in indices] for row in self.covariance]
        gain = multiply(pht, inverse)
        for i in range(5):
            self.state[i] += sum(gain[i][j] * residual[j] for j in range(size))
        self.state[2] = wrap_angle(self.state[2])
        residual_matrix = identity(5)
        for i in range(5):
            for column, state_index in enumerate(indices):
                residual_matrix[i][state_index] -= gain[i][column]
        krkt = [[sum(gain[i][k] * variances[k] * gain[j][k] for k in range(size))
                 for j in range(5)] for i in range(5)]
        self.covariance = add(
            multiply(multiply(residual_matrix, self.covariance), transpose(residual_matrix)),
            krkt)
        # 消除浮点舍入造成的轻微不对称。
        for i in range(5):
            for j in range(i):
                value = 0.5 * (self.covariance[i][j] + self.covariance[j][i])
                self.covariance[i][j] = self.covariance[j][i] = value
        return True

    def observe_wheels(self, speed, rate, timestamp):
        if (not all(math.isfinite(x) for x in (speed, rate, timestamp))
                or abs(speed) > self.config.max_wheel_speed_mps
                or abs(rate) > self.config.max_yaw_rate_radps):
            self.rejected_wheel += 1
            return False
        if not self.predict_to(timestamp):
            self.rejected_wheel += 1
            return False
        accepted = self._correct([3, 4], [speed, rate],
                                 [self.config.wheel_speed_stddev ** 2,
                                  self.config.wheel_yaw_rate_stddev ** 2])
        if not accepted:
            self.rejected_wheel += 1
        return accepted

    def observe_gyro(self, rate, timestamp):
        if (not math.isfinite(rate) or abs(rate) > self.config.max_yaw_rate_radps
                or not self.predict_to(timestamp)):
            self.rejected_imu += 1
            return False
        accepted = self._correct([4], [rate], [self.config.gyro_stddev ** 2])
        if not accepted:
            self.rejected_imu += 1
        return accepted


class EkfOdometry:
    """将累计计数转换成纯编码器观测，再驱动异步 EKF。

    timestamp 为同一单调时钟的秒数。节点使用接收时间；不是 MCU 采样时间。
    每条 gyro 只使用一次，不在每次编码器到达时重复使用最近 gyro。
    """

    def __init__(self, config=None):
        self.filter = PlanarEkf(config)
        self.reset()

    def reset(self):
        self.filter.reset()
        self.raw = WheelOdometry()
        self.previous = None
        self.last_wheel_time = None
        self.last_gyro_time = None
        self.last_accepted_gyro_time = None
        self.raw_speed = 0.0
        self.raw_rate = 0.0
        self.raw_velocity_valid = False
        self.wheel_accepted = False

    def add_imu(self, rate, timestamp):
        if (not math.isfinite(timestamp) or self.last_wheel_time is None
                or timestamp < self.last_wheel_time
                or timestamp - self.last_wheel_time > self.filter.config.max_gap_s
                or (self.last_gyro_time is not None and timestamp <= self.last_gyro_time)):
            return False
        self.last_gyro_time = timestamp
        accepted = self.filter.observe_gyro(rate, timestamp)
        if accepted:
            self.last_accepted_gyro_time = timestamp
        return accepted

    def update(self, left, right, diameter, counts_per_rev, track, timestamp):
        if (not all(math.isfinite(x) for x in (diameter, counts_per_rev, track, timestamp))
                or min(diameter, counts_per_rev, track) <= 0.0):
            return None
        if self.filter.time_s is not None and timestamp < self.filter.time_s:
            return None
        self.wheel_accepted = False
        self.raw_velocity_valid = False
        current = (left, right, timestamp)
        if self.previous is None:
            self.previous = current
            self.last_wheel_time = timestamp
            self.raw.set_count_baseline(left, right)
            self.filter.predict_to(timestamp)
            return self.sample()
        old_left, old_right, old_time = self.previous
        dt = timestamp - old_time
        if dt <= 0.0:
            return None
        self.previous = current
        if dt > self.filter.config.max_gap_s:
            self.raw.set_count_baseline(left, right)
            self.filter.suspend(timestamp)
            self.last_wheel_time = timestamp
            self.last_accepted_gyro_time = None
            self.raw_speed = self.raw_rate = 0.0
            return self.sample()
        scale = math.pi * diameter / counts_per_rev
        left_speed = WheelOdometry.wrapped_count_delta(left, old_left) * scale / dt
        right_speed = WheelOdometry.wrapped_count_delta(right, old_right) * scale / dt
        speed = 0.5 * (left_speed + right_speed)
        rate = (right_speed - left_speed) / track
        # 巨大跳变（包括未等到 heartbeat 的 MCU 复位）只重建计数基准。
        if (max(abs(left_speed), abs(right_speed)) > self.filter.config.max_wheel_speed_mps
                or abs(rate) > self.filter.config.max_yaw_rate_radps):
            self.raw.set_count_baseline(left, right)
            self.filter.rejected_wheel += 1
            self.filter.suspend(timestamp)
            self.last_accepted_gyro_time = None
            return None
        self.raw.update(left, right, diameter, counts_per_rev, track)
        self.raw_speed, self.raw_rate = speed, rate
        self.raw_velocity_valid = True
        self.wheel_accepted = self.filter.observe_wheels(speed, rate, timestamp)
        if self.wheel_accepted:
            self.last_wheel_time = timestamp
        elif (self.last_wheel_time is not None
              and timestamp - self.last_wheel_time > self.filter.config.max_gap_s):
            self.filter.suspend(timestamp)
            self.last_accepted_gyro_time = None
        return self.sample()

    def sample(self):
        x, y, yaw, _, _ = self.filter.state
        raw = self.raw.sample()
        return OdometrySample(x, y, yaw, raw.left_wheel_angle_rad, raw.right_wheel_angle_rad)

    def ros_covariances(self):
        """ROS 顺序 [x,y,z,roll,pitch,yaw]；未估计的轴显式标为高不确定性。"""
        pose = [0.0] * 36
        twist = [0.0] * 36
        for i in range(6):
            pose[i * 6 + i] = twist[i * 6 + i] = 1e6
        for i, ros_i in enumerate((0, 1, 5)):
            for j, ros_j in enumerate((0, 1, 5)):
                pose[ros_i * 6 + ros_j] = self.filter.covariance[i][j]
        for i, ros_i in ((3, 0), (4, 5)):
            for j, ros_j in ((3, 0), (4, 5)):
                twist[ros_i * 6 + ros_j] = self.filter.covariance[i][j]
        return pose, twist

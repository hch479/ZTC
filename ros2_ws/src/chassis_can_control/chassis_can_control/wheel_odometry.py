"""只负责双轮编码器里程计，不依赖 ROS，便于阅读和单元测试。"""

from dataclasses import dataclass
import math
from typing import Optional


@dataclass
class OdometrySample:
    """一次编码器更新后的车体位姿和车轮角度。"""

    x_m: float
    y_m: float
    yaw_rad: float
    left_wheel_angle_rad: float
    right_wheel_angle_rad: float


class WheelOdometry:
    """根据左右累计编码器计数，计算二维差速里程计。"""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.x_m = 0.0
        self.y_m = 0.0
        self.yaw_rad = 0.0
        self.left_wheel_angle_rad = 0.0
        self.right_wheel_angle_rad = 0.0
        self._previous_left_count: Optional[int] = None
        self._previous_right_count: Optional[int] = None

    @staticmethod
    def wrapped_count_delta(current: int, previous: int) -> int:
        """计算 int32 累计计数差，同时正确处理 0x7FFFFFFF 附近的环绕。"""
        return ((current - previous + (1 << 31)) % (1 << 32)) - (1 << 31)

    def set_count_baseline(self, left_count: int, right_count: int) -> None:
        """中断/计数跳变后只重建基准，保留已知位姿和轮角。"""
        self._previous_left_count = left_count
        self._previous_right_count = right_count

    def update(
        self,
        left_count: int,
        right_count: int,
        wheel_diameter_m: float,
        counts_per_wheel_rev: float,
        wheel_track_m: float,
        imu_yaw_rate_radps: Optional[float] = None,
        dt_s: Optional[float] = None,
        imu_weight: float = 0.0,
    ) -> Optional[OdometrySample]:
        """
        输入一对累计计数；几何参数非法时返回 None。

        imu_weight=0 表示完全使用车轮转角；imu_weight=1 表示本周期转角
        完全使用陀螺仪角速度积分。位置的前进距离始终来自编码器。
        """
        if (
            wheel_diameter_m <= 0.0
            or counts_per_wheel_rev <= 0.0
            or wheel_track_m <= 0.0
        ):
            return None

        # 第一对计数只建立基准，不假设 STM32 一定从零启动。
        if self._previous_left_count is None or self._previous_right_count is None:
            self._previous_left_count = left_count
            self._previous_right_count = right_count
            return self.sample()

        left_delta = self.wrapped_count_delta(left_count, self._previous_left_count)
        right_delta = self.wrapped_count_delta(right_count, self._previous_right_count)
        self._previous_left_count = left_count
        self._previous_right_count = right_count

        meters_per_count = math.pi * wheel_diameter_m / counts_per_wheel_rev
        left_distance = left_delta * meters_per_count
        right_distance = right_delta * meters_per_count
        center_distance = 0.5 * (left_distance + right_distance)
        wheel_yaw_delta = (right_distance - left_distance) / wheel_track_m
        yaw_delta = wheel_yaw_delta

        # 只有数据完整且数值有效时才融合，否则自然退回纯编码器结果。
        if (
            imu_yaw_rate_radps is not None
            and dt_s is not None
            and math.isfinite(imu_yaw_rate_radps)
            and math.isfinite(dt_s)
            and dt_s > 0.0
        ):
            safe_weight = max(0.0, min(1.0, imu_weight))
            imu_yaw_delta = imu_yaw_rate_radps * dt_s
            yaw_delta = (
                (1.0 - safe_weight) * wheel_yaw_delta
                + safe_weight * imu_yaw_delta
            )

        # 中点法比直接使用周期起点角度的误差小，表达仍然很直观。
        middle_yaw = self.yaw_rad + 0.5 * yaw_delta
        self.x_m += center_distance * math.cos(middle_yaw)
        self.y_m += center_distance * math.sin(middle_yaw)
        self.yaw_rad = math.atan2(
            math.sin(self.yaw_rad + yaw_delta),
            math.cos(self.yaw_rad + yaw_delta),
        )
        self.left_wheel_angle_rad += left_delta * 2.0 * math.pi / counts_per_wheel_rev
        self.right_wheel_angle_rad += right_delta * 2.0 * math.pi / counts_per_wheel_rev
        return self.sample()

    def sample(self) -> OdometrySample:
        return OdometrySample(
            x_m=self.x_m,
            y_m=self.y_m,
            yaw_rad=self.yaw_rad,
            left_wheel_angle_rad=self.left_wheel_angle_rad,
            right_wheel_angle_rad=self.right_wheel_angle_rad,
        )

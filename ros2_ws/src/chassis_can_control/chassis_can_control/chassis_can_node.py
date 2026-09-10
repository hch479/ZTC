"""RDK X5 上运行的 ROS 2 节点：/cmd_vel <-> SocketCAN <-> STM32。"""

from collections import deque
from dataclasses import dataclass
import math
import time
from typing import Deque, Dict, Optional, Tuple

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import SetBool, Trigger
from tf2_ros import TransformBroadcaster

from . import can_protocol as protocol
from .can_protocol import CanFrame, EncoderData, MotorOutput, StatusData, WheelSpeed
from .socketcan_transport import SocketCanTransport
from .serial_transport import SerialTransport
from .wheel_odometry import OdometrySample, WheelOdometry


@dataclass
class PendingParameter:
    """已经发出、正在等待 STM32 应答的一个参数。"""

    parameter_id: int
    value: float
    sequence: int
    attempts: int
    sent_time: float


class ChassisCanNode(Node):
    """把 ROS 速度命令和底盘遥测连接起来。"""
   
    def __init__(self) -> None:
        super().__init__("chassis_can_node")

        # 通讯、话题与里程计参数。
        self.declare_parameter("can_interface", "can0")
        self.declare_parameter("transport", "can")
        self.declare_parameter(
            "serial_port",
            "/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00",
        )
        self.declare_parameter("serial_baud_rate", 115200)
        self.declare_parameter("command_rate_hz", 50.0)
        self.declare_parameter("ros_command_timeout_s", 0.20)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("left_joint_name", "left_wheel_joint")
        self.declare_parameter("right_joint_name", "right_wheel_joint")
        self.declare_parameter("push_parameters_on_start", False)

        # MPU6050 与 ICM20948 在本工程中都配置为加速度 +-2 g、陀螺仪 +-500 dps。
        self.declare_parameter("imu.frame_id", "base_link")
        self.declare_parameter("imu.accel_lsb_per_g", 16384.0)
        self.declare_parameter("imu.gyro_lsb_per_dps", 65.5)
        # source: 0=x, 1=y, 2=z；sign 用来适配传感器在底盘上的安装方向。
        self.declare_parameter("imu.x_source", 0)
        self.declare_parameter("imu.y_source", 1)
        self.declare_parameter("imu.z_source", 2)
        self.declare_parameter("imu.x_sign", 1.0)
        self.declare_parameter("imu.y_sign", 1.0)
        self.declare_parameter("imu.z_sign", 1.0)
        self.declare_parameter("imu.angular_velocity_stddev", 0.02)
        self.declare_parameter("imu.linear_acceleration_stddev", 0.15)
        self.declare_parameter("imu.fusion_enabled", True)
        self.declare_parameter("imu.fusion_weight", 0.85)
        self.declare_parameter("imu.timeout_s", 0.20)

        # 这些参数名称与 can_protocol.PARAMETER_IDS 完全一致。
        defaults = {
            "controller.left_kp": 3000.0,
            "controller.left_ki": 1500.0,
            "controller.left_kff": 9000.0,
            "controller.right_kp": 3000.0,
            "controller.right_ki": 1500.0,
            "controller.right_kff": 9000.0,
            "controller.left_dead_forward": 700.0,
            "controller.left_dead_reverse": 700.0,
            "controller.right_dead_forward": 700.0,
            "controller.right_dead_reverse": 700.0,
            "controller.filter_alpha": 0.35,
            "controller.straight_sync_kp": 0.40,
            "controller.straight_sync_max_mps": 0.08,
            "geometry.wheel_track_m": 0.162,
            "geometry.wheel_diameter_m": 0.065,
            "geometry.counts_per_wheel_rev": 60000.0,
            "geometry.chassis_type": 1.0,
            "geometry.wheelbase_m": 0.144,
            "geometry.maximum_steering_angle_rad": 0.52,
            "geometry.servo_center_pwm": 1500.0,
            "geometry.servo_pwm_per_rad": 636.56,
            "limits.acceleration_mps2": 0.50,
            "limits.deceleration_mps2": 0.80,
            "limits.maximum_linear_speed_mps": 0.60,
            "safety.low_battery_mv": 9800.0,
            "safety.command_timeout_ms": 200.0,
            "safety.stall_target_speed_mps": 0.15,
            "safety.stall_measured_speed_mps": 0.02,
            "safety.stall_pwm_ratio": 0.80,
            "safety.stall_time_ms": 600.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        transport = str(self.get_parameter("transport").value).lower()
        if transport == "serial":
            self._can = SerialTransport(
                str(self.get_parameter("serial_port").value),
                int(self.get_parameter("serial_baud_rate").value),
            )
        elif transport == "can":
            self._can = SocketCanTransport(
                str(self.get_parameter("can_interface").value)
            )
        else:
            raise ValueError("transport must be 'can' or 'serial'")
        self._transport_name = transport
        self._last_can_open_attempt = 0.0
        self._last_error_log_time = 0.0
        self._open_can_if_needed()

        self._sequence = 0
        # 上电默认禁止驱动，必须显式调用 /chassis/enable 才允许执行速度命令。
        self._drive_enabled = False
        self._emergency_stop = False
        self._latest_command = Twist()
        self._has_received_cmd_vel = False
        self._last_cmd_vel_time = 0.0
        self._last_heartbeat_time = 0.0
        self._latest_heartbeat = None
        self._previous_stm32_uptime_ms: Optional[int] = None
        self._latest_status: Optional[StatusData] = None
        self._latest_speed: Optional[WheelSpeed] = None
        self._latest_output: Optional[MotorOutput] = None
        self._left_encoder: Optional[EncoderData] = None
        self._right_encoder: Optional[EncoderData] = None
        self._last_odometry_sequence: Optional[int] = None
        self._last_odometry_update_time: Optional[float] = None
        self._odometry = WheelOdometry()
        self._latest_imu_acceleration: Optional[protocol.ImuVector] = None
        self._latest_imu_gyroscope: Optional[protocol.ImuVector] = None
        self._latest_imu_status: Optional[protocol.ImuStatus] = None
        self._last_published_imu_sequence: Optional[int] = None
        self._last_imu_time = 0.0
        self._latest_gyro_z_radps: Optional[float] = None
        self._imu_fusion_active = False
        self._parameter_queue: Deque[Tuple[int, float]] = deque()
        # 当前等待 STM32 应答的参数。字典字段少，调试时直接打印也容易看懂。
        self._pending_parameter: Optional[PendingParameter] = None
        self._parameter_transfer_failed = False

        self._odom_publisher = self.create_publisher(Odometry, "odom", 20)
        self._joint_publisher = self.create_publisher(JointState, "joint_states", 20)
        self._imu_publisher = self.create_publisher(Imu, "imu/data_raw", 20)
        self._diagnostic_publisher = self.create_publisher(
            DiagnosticArray, "diagnostics", 10
        )
        self._debug_publisher = self.create_publisher(
            Float32MultiArray, "motor/debug", 20
        )
        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
        self.create_service(SetBool, "chassis/enable", self._on_enable)
        self.create_service(SetBool, "chassis/emergency_stop", self._on_estop)
        self.create_service(Trigger, "chassis/clear_faults", self._on_clear_faults)
        self.create_service(Trigger, "chassis/save_parameters", self._on_save_parameters)
        self.create_service(Trigger, "chassis/push_parameters", self._on_push_parameters)
        self.create_service(Trigger, "chassis/reset_odometry", self._on_reset_odometry)

        command_rate = float(self.get_parameter("command_rate_hz").value)
        if command_rate < 10.0:
            command_rate = 10.0
        self.create_timer(1.0 / command_rate, self._send_motion_command)
        self.create_timer(0.005, self._poll_can)
        self.create_timer(0.05, self._send_one_pending_parameter)
        self.create_timer(0.20, self._publish_diagnostics)
        self.add_on_set_parameters_callback(self._on_parameters_changed)

        if bool(self.get_parameter("push_parameters_on_start").value):
            self._queue_all_parameters()

        self.get_logger().info(
            f"Chassis {self._transport_name} node started on {self._can.interface}; "
            "motor output remains protected by both ROS and STM32 timeouts."
        )

    def destroy_node(self) -> bool:
        # 退出前尽最大可能发三次禁止运行命令，随后关闭 CAN socket。
        self._drive_enabled = False
        self._emergency_stop = False
        for _ in range(3):
            self._send_motion_command()
        self._can.close()
        return super().destroy_node()

    def _next_sequence(self) -> int:
        self._sequence = (self._sequence + 1) & 0xFF
        return self._sequence

    def _open_can_if_needed(self) -> bool:
        if self._can.is_open:
            return True
        now = time.monotonic()
        if now - self._last_can_open_attempt < 1.0:
            return False
        self._last_can_open_attempt = now
        try:
            self._can.open()
            self.get_logger().info(
                f"{self._transport_name} interface {self._can.interface} is open"
            )
            return True
        except OSError as error:
            self._log_can_error(f"Cannot open {self._can.interface}: {error}")
            return False

    def _log_can_error(self, text: str) -> None:
        # 通讯故障时定时器仍会运行，限制日志频率以免刷屏。
        now = time.monotonic()
        if now - self._last_error_log_time >= 2.0:
            self.get_logger().error(text)
            self._last_error_log_time = now

    def _send(self, frame: CanFrame) -> bool:
        if not self._open_can_if_needed():
            return False
        try:
            self._can.send(frame)
            return True
        except OSError as error:
            self._log_can_error(f"{self._transport_name} send failed: {error}")
            self._can.close()
            return False

    def _on_cmd_vel(self, message: Twist) -> None:
        self._latest_command = message
        self._has_received_cmd_vel = True
        self._last_cmd_vel_time = time.monotonic()

    def _send_motion_command(self) -> None:
        now = time.monotonic()
        timeout = float(self.get_parameter("ros_command_timeout_s").value)
        command_is_fresh = (
            self._has_received_cmd_vel and now - self._last_cmd_vel_time <= timeout
        )

        if command_is_fresh and self._drive_enabled and not self._emergency_stop:
            linear = float(self._latest_command.linear.x)
            angular = float(self._latest_command.angular.z)
            enable = True
        else:
            linear = 0.0
            angular = 0.0
            # 急停需要 enable 位也为零；STM32 根据 estop 位锁存故障。
            enable = False

        frame = protocol.make_motion_command(
            linear,
            angular,
            enable,
            self._emergency_stop,
            self._next_sequence(),
        )
        self._send(frame)

    def _poll_can(self) -> None:
        if not self._open_can_if_needed():
            return
        try:
            frames = self._can.receive_available()
        except OSError as error:
            self._log_can_error(f"{self._transport_name} receive failed: {error}")
            self._can.close()
            return

        for frame in frames:
            self._handle_frame(frame)

    def _handle_frame(self, frame: CanFrame) -> None:
        if frame.can_id == protocol.ID_WHEEL_SPEED:
            self._latest_speed = protocol.decode_wheel_speed(frame)
            self._publish_motor_debug()
        elif frame.can_id == protocol.ID_MOTOR_OUTPUT:
            self._latest_output = protocol.decode_motor_output(frame)
            self._publish_motor_debug()
        elif frame.can_id == protocol.ID_LEFT_ENCODER:
            decoded = protocol.decode_encoder(frame)
            if decoded is not None:
                self._left_encoder = decoded
                self._update_odometry_if_pair_ready()
        elif frame.can_id == protocol.ID_RIGHT_ENCODER:
            decoded = protocol.decode_encoder(frame)
            if decoded is not None:
                self._right_encoder = decoded
                self._update_odometry_if_pair_ready()
        elif frame.can_id == protocol.ID_STATUS:
            self._latest_status = protocol.decode_status(frame)
        elif frame.can_id == protocol.ID_IMU_ACCELERATION:
            decoded_imu = protocol.decode_imu_vector(frame)
            if decoded_imu is not None:
                self._latest_imu_acceleration = decoded_imu
                self._publish_imu_if_pair_ready()
        elif frame.can_id == protocol.ID_IMU_GYROSCOPE:
            decoded_imu = protocol.decode_imu_vector(frame)
            if decoded_imu is not None:
                self._latest_imu_gyroscope = decoded_imu
                self._publish_imu_if_pair_ready()
        elif frame.can_id == protocol.ID_IMU_STATUS:
            decoded_status = protocol.decode_imu_status(frame)
            if decoded_status is not None:
                self._latest_imu_status = decoded_status
                if not decoded_status.calibrated:
                    self._imu_fusion_active = False
        elif frame.can_id == protocol.ID_PARAMETER_REPLY:
            reply = protocol.decode_parameter_reply(frame)
            if reply is not None:
                self.get_logger().info(
                    f"Parameter reply: id={reply.parameter_id}, "
                    f"status={reply.status}, value={reply.value:.4f}, "
                    f"sequence={reply.sequence}"
                )
                self._handle_parameter_reply(reply)
        elif frame.can_id == protocol.ID_HEARTBEAT:
            heartbeat = protocol.decode_heartbeat(frame)
            if heartbeat is not None:
                if (
                    self._previous_stm32_uptime_ms is not None
                    and heartbeat.uptime_ms < self._previous_stm32_uptime_ms
                ):
                    # STM32 复位后累计编码器通常也从零开始，重新建立里程计基准。
                    self.get_logger().warning(
                        "STM32 restart detected; encoder odometry baseline reset"
                    )
                    self._odometry.reset()
                    self._last_odometry_sequence = None
                    self._last_odometry_update_time = None
                    self._latest_imu_acceleration = None
                    self._latest_imu_gyroscope = None
                    self._latest_imu_status = None
                    self._last_published_imu_sequence = None
                    self._latest_gyro_z_radps = None
                    self._imu_fusion_active = False
                self._previous_stm32_uptime_ms = heartbeat.uptime_ms
                self._latest_heartbeat = heartbeat
                self._last_heartbeat_time = time.monotonic()

    def _map_imu_vector(self, vector: protocol.ImuVector) -> Tuple[float, float, float]:
        """Map sensor chip axes to ROS base axes using six readable parameters."""
        source_values = (float(vector.x), float(vector.y), float(vector.z))
        mapped = []
        for axis_name in ("x", "y", "z"):
            source = int(self.get_parameter(f"imu.{axis_name}_source").value)
            sign = float(self.get_parameter(f"imu.{axis_name}_sign").value)
            mapped.append(sign * source_values[source])
        return mapped[0], mapped[1], mapped[2]

    def _publish_imu_if_pair_ready(self) -> None:
        acceleration = self._latest_imu_acceleration
        gyroscope = self._latest_imu_gyroscope
        status = self._latest_imu_status
        if acceleration is None or gyroscope is None:
            return
        if acceleration.sequence != gyroscope.sequence:
            return
        if self._last_published_imu_sequence == acceleration.sequence:
            return
        # 下位机明确报告校准完成后，才发布和参与融合。
        if status is None or not status.calibrated:
            return

        accel_scale = float(self.get_parameter("imu.accel_lsb_per_g").value)
        gyro_scale = float(self.get_parameter("imu.gyro_lsb_per_dps").value)
        accel_counts = self._map_imu_vector(acceleration)
        gyro_counts = self._map_imu_vector(gyroscope)
        standard_gravity = 9.80665
        degrees_to_radians = math.pi / 180.0
        linear_acceleration = tuple(
            value * standard_gravity / accel_scale for value in accel_counts
        )
        angular_velocity = tuple(
            value * degrees_to_radians / gyro_scale for value in gyro_counts
        )

        message = Imu()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(self.get_parameter("imu.frame_id").value)
        # data_raw does not contain an absolute orientation estimate.
        message.orientation_covariance[0] = -1.0
        message.angular_velocity.x = angular_velocity[0]
        message.angular_velocity.y = angular_velocity[1]
        message.angular_velocity.z = angular_velocity[2]
        message.linear_acceleration.x = linear_acceleration[0]
        message.linear_acceleration.y = linear_acceleration[1]
        message.linear_acceleration.z = linear_acceleration[2]

        gyro_stddev = float(
            self.get_parameter("imu.angular_velocity_stddev").value
        )
        accel_stddev = float(
            self.get_parameter("imu.linear_acceleration_stddev").value
        )
        gyro_variance = gyro_stddev * gyro_stddev
        accel_variance = accel_stddev * accel_stddev
        message.angular_velocity_covariance = [
            gyro_variance, 0.0, 0.0,
            0.0, gyro_variance, 0.0,
            0.0, 0.0, gyro_variance,
        ]
        message.linear_acceleration_covariance = [
            accel_variance, 0.0, 0.0,
            0.0, accel_variance, 0.0,
            0.0, 0.0, accel_variance,
        ]
        self._imu_publisher.publish(message)

        self._latest_gyro_z_radps = angular_velocity[2]
        self._last_imu_time = time.monotonic()
        self._last_published_imu_sequence = acceleration.sequence

    def _update_odometry_if_pair_ready(self) -> None:
        left = self._left_encoder
        right = self._right_encoder
        if left is None or right is None or left.sequence != right.sequence:
            return
        if self._last_odometry_sequence == left.sequence:
            return
        self._last_odometry_sequence = left.sequence

        diameter = float(self.get_parameter("geometry.wheel_diameter_m").value)
        counts_per_rev = float(
            self.get_parameter("geometry.counts_per_wheel_rev").value
        )
        wheel_track = float(self.get_parameter("geometry.wheel_track_m").value)
        now = time.monotonic()
        dt_s = None
        if self._last_odometry_update_time is not None:
            dt_s = now - self._last_odometry_update_time
        self._last_odometry_update_time = now

        imu_timeout = float(self.get_parameter("imu.timeout_s").value)
        imu_enabled = bool(self.get_parameter("imu.fusion_enabled").value)
        imu_calibrated = (
            self._latest_imu_status is not None
            and self._latest_imu_status.calibrated
        )
        imu_fresh = (
            self._latest_gyro_z_radps is not None
            and (now - self._last_imu_time) <= imu_timeout
        )
        self._imu_fusion_active = imu_enabled and imu_calibrated and imu_fresh
        imu_weight = float(self.get_parameter("imu.fusion_weight").value)
        sample = self._odometry.update(
            left.total_count,
            right.total_count,
            diameter,
            counts_per_rev,
            wheel_track,
            imu_yaw_rate_radps=(
                self._latest_gyro_z_radps if self._imu_fusion_active else None
            ),
            dt_s=dt_s,
            imu_weight=imu_weight,
        )
        if sample is not None:
            self._publish_odometry_and_joints(sample)

    def _publish_odometry_and_joints(self, sample: OdometrySample) -> None:
        stamp = self.get_clock().now().to_msg()
        odom_frame = str(self.get_parameter("odom_frame").value)
        base_frame = str(self.get_parameter("base_frame").value)
        half_yaw = 0.5 * sample.yaw_rad
        quaternion_z = math.sin(half_yaw)
        quaternion_w = math.cos(half_yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = odom_frame
        odom.child_frame_id = base_frame
        odom.pose.pose.position.x = sample.x_m
        odom.pose.pose.position.y = sample.y_m
        odom.pose.pose.orientation.z = quaternion_z
        odom.pose.pose.orientation.w = quaternion_w
        if self._latest_speed is not None:
            odom.twist.twist.linear.x = 0.5 * (
                self._latest_speed.left_measured_mps
                + self._latest_speed.right_measured_mps
            )
            wheel_track = float(self.get_parameter("geometry.wheel_track_m").value)
            wheel_yaw_rate = (
                self._latest_speed.right_measured_mps
                - self._latest_speed.left_measured_mps
            ) / wheel_track
            odom.twist.twist.angular.z = wheel_yaw_rate
            if self._imu_fusion_active and self._latest_gyro_z_radps is not None:
                imu_weight = float(self.get_parameter("imu.fusion_weight").value)
                odom.twist.twist.angular.z = (
                    (1.0 - imu_weight) * wheel_yaw_rate
                    + imu_weight * self._latest_gyro_z_radps
                )
        self._odom_publisher.publish(odom)

        joint = JointState()
        joint.header.stamp = stamp
        joint.name = [
            str(self.get_parameter("left_joint_name").value),
            str(self.get_parameter("right_joint_name").value),
        ]
        joint.position = [
            sample.left_wheel_angle_rad,
            sample.right_wheel_angle_rad,
        ]
        if self._latest_speed is not None:
            radius = 0.5 * float(
                self.get_parameter("geometry.wheel_diameter_m").value
            )
            joint.velocity = [
                self._latest_speed.left_measured_mps / radius,
                self._latest_speed.right_measured_mps / radius,
            ]
        self._joint_publisher.publish(joint)

        if bool(self.get_parameter("publish_tf").value):
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = odom_frame
            transform.child_frame_id = base_frame
            transform.transform.translation.x = sample.x_m
            transform.transform.translation.y = sample.y_m
            transform.transform.rotation.z = quaternion_z
            transform.transform.rotation.w = quaternion_w
            self._tf_broadcaster.sendTransform(transform)

    def _publish_motor_debug(self) -> None:
        if self._latest_speed is None or self._latest_output is None:
            return
        message = Float32MultiArray()
        message.data = [
            self._latest_speed.left_target_mps,
            self._latest_speed.left_measured_mps,
            self._latest_speed.right_target_mps,
            self._latest_speed.right_measured_mps,
            float(self._latest_output.left_pwm),
            float(self._latest_output.right_pwm),
            self._latest_output.left_error_mps,
            self._latest_output.right_error_mps,
        ]
        self._debug_publisher.publish(message)

    def _publish_diagnostics(self) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "C30D motor controller"
        status.hardware_id = "STM32F407-C30D"

        heartbeat_age = time.monotonic() - self._last_heartbeat_time
        if self._latest_heartbeat is None or heartbeat_age > 2.0:
            status.level = DiagnosticStatus.ERROR
            status.message = "STM32 heartbeat missing"
        elif self._latest_status is None:
            status.level = DiagnosticStatus.WARN
            status.message = "Heartbeat received, status frame missing"
        else:
            faults = self._latest_status.faults
            state_name = protocol.STATE_NAMES.get(
                self._latest_status.state, f"UNKNOWN({self._latest_status.state})"
            )
            if faults == 0:
                status.level = DiagnosticStatus.OK
                status.message = state_name
            elif faults & (0x0001 | 0x0080 | 0x0100):
                status.level = DiagnosticStatus.WARN
                status.message = protocol.fault_text(faults)
            else:
                status.level = DiagnosticStatus.ERROR
                status.message = protocol.fault_text(faults)

            status.values.extend(
                [
                    KeyValue(key="state", value=state_name),
                    KeyValue(key="faults", value=protocol.fault_text(faults)),
                    KeyValue(
                        key="battery_voltage",
                        value=f"{self._latest_status.battery_mv / 1000.0:.3f} V",
                    ),
                    KeyValue(
                        key="last_command_sequence",
                        value=str(self._latest_status.last_command_sequence),
                    ),
                    KeyValue(
                        key="control_overruns",
                        value=str(self._latest_status.overrun_count),
                    ),
                ]
            )
        status.values.append(KeyValue(key="transport", value=self._transport_name))
        status.values.append(KeyValue(key="interface", value=self._can.interface))
        status.values.append(KeyValue(key="heartbeat_age", value=f"{heartbeat_age:.3f} s"))
        pending_count = len(self._parameter_queue)
        if self._pending_parameter is not None and pending_count == 0:
            pending_count = 1
        status.values.append(
            KeyValue(key="pending_parameters", value=str(pending_count))
        )
        status.values.append(
            KeyValue(
                key="parameter_transfer_failed",
                value=str(self._parameter_transfer_failed),
            )
        )

        imu_age = time.monotonic() - self._last_imu_time
        sensor_names = {0: "NONE", 1: "MPU6050", 2: "ICM20948"}
        if self._latest_imu_status is None:
            imu_sensor = "UNKNOWN"
            imu_calibrated = False
            calibration_progress = "0/unknown"
        else:
            imu_sensor = sensor_names.get(
                self._latest_imu_status.sensor_type,
                f"UNKNOWN({self._latest_imu_status.sensor_type})",
            )
            imu_calibrated = self._latest_imu_status.calibrated
            calibration_progress = (
                f"{self._latest_imu_status.calibration_samples}/"
                f"{self._latest_imu_status.calibration_target}"
            )

        status.values.extend(
            [
                KeyValue(key="imu_sensor", value=imu_sensor),
                KeyValue(key="imu_calibrated", value=str(imu_calibrated)),
                KeyValue(key="imu_calibration", value=calibration_progress),
                KeyValue(key="imu_data_age", value=f"{imu_age:.3f} s"),
                KeyValue(
                    key="imu_fusion_active", value=str(self._imu_fusion_active)
                ),
            ]
        )

        # IMU problems must be visible, but they never stop motor control.
        if bool(self.get_parameter("imu.fusion_enabled").value):
            imu_timeout = float(self.get_parameter("imu.timeout_s").value)
            imu_problem = None
            if self._latest_imu_status is None:
                imu_problem = "IMU status missing"
            elif not imu_calibrated:
                imu_problem = "IMU calibrating; keep chassis still"
            elif imu_age > imu_timeout:
                imu_problem = "IMU data stale; wheel odometry fallback active"
            if imu_problem is not None and status.level < DiagnosticStatus.WARN:
                status.level = DiagnosticStatus.WARN
                status.message = imu_problem

        array.status.append(status)
        self._diagnostic_publisher.publish(array)

    def _on_enable(self, request: SetBool.Request, response: SetBool.Response):
        if (
            request.data
            and bool(self.get_parameter("imu.fusion_enabled").value)
            and (
                self._latest_imu_status is None
                or not self._latest_imu_status.calibrated
            )
        ):
            self._drive_enabled = False
            response.success = False
            response.message = "wait for stationary IMU calibration to complete"
            self._send_motion_command()
            return response
        self._drive_enabled = bool(request.data)
        response.success = True
        response.message = "drive enabled" if request.data else "drive disabled"
        self._send_motion_command()
        return response

    def _on_estop(self, request: SetBool.Request, response: SetBool.Response):
        self._emergency_stop = bool(request.data)
        if request.data:
            self._drive_enabled = False
        response.success = True
        response.message = (
            "emergency stop sent; clear_faults is required before re-enable"
            if request.data
            else "estop request released; STM32 latch is still active"
        )
        self._send_motion_command()
        return response

    def _on_clear_faults(self, request: Trigger.Request, response: Trigger.Response):
        del request
        self._drive_enabled = False
        self._emergency_stop = False
        # 按协议顺序：先下发普通禁止命令，再发送带钥匙的清故障命令。
        self._send_motion_command()
        response.success = self._send(
            protocol.make_system_command(
                protocol.SYSTEM_CLEAR_FAULT, self._next_sequence()
            )
        )
        response.message = "clear-fault command sent" if response.success else "CAN unavailable"
        return response

    def _on_save_parameters(self, request: Trigger.Request, response: Trigger.Response):
        del request
        if self._pending_parameter is not None or self._parameter_queue:
            response.success = False
            response.message = "wait until all parameter replies are received"
            return response
        if self._parameter_transfer_failed:
            response.success = False
            response.message = "parameter transfer failed; push parameters again"
            return response
        response.success = self._send(
            protocol.make_system_command(
                protocol.SYSTEM_SAVE_PARAMETERS, self._next_sequence()
            )
        )
        response.message = "save command sent; check parameter reply" if response.success else "CAN unavailable"
        return response

    def _queue_all_parameters(self) -> None:
        self._parameter_queue.clear()
        self._parameter_transfer_failed = False
        for name, parameter_id in protocol.PARAMETER_IDS.items():
            self._parameter_queue.append(
                (parameter_id, float(self.get_parameter(name).value))
            )

    def _on_push_parameters(self, request: Trigger.Request, response: Trigger.Response):
        del request
        if self._latest_status is not None and self._latest_status.state == 2:
            response.success = False
            response.message = "disable chassis before changing motor parameters"
            return response
        if self._pending_parameter is not None:
            response.success = False
            response.message = "a parameter is still waiting for reply"
            return response
        self._queue_all_parameters()
        response.success = True
        response.message = f"queued {len(self._parameter_queue)} parameters"
        return response

    def _send_one_pending_parameter(self) -> None:
        now = time.monotonic()

        # 已发送的参数先等待应答；200 ms 没有应答才重发。
        if self._pending_parameter is not None:
            elapsed = now - self._pending_parameter.sent_time
            if elapsed < 0.20:
                return
            if self._pending_parameter.attempts >= 3:
                parameter_id = self._pending_parameter.parameter_id
                self.get_logger().error(
                    f"Parameter id={parameter_id} timed out after 3 attempts"
                )
                self._parameter_transfer_failed = True
                self._pending_parameter = None
                self._parameter_queue.clear()
                return

            retry_frame = protocol.make_parameter_command(
                self._pending_parameter.parameter_id,
                protocol.PARAM_WRITE,
                self._pending_parameter.value,
                self._pending_parameter.sequence,
            )
            if self._send(retry_frame):
                self._pending_parameter.attempts += 1
                self._pending_parameter.sent_time = now
            return

        if not self._parameter_queue:
            return

        parameter_id, value = self._parameter_queue[0]
        sequence = self._next_sequence()
        frame = protocol.make_parameter_command(
            parameter_id, protocol.PARAM_WRITE, value, sequence
        )
        if self._send(frame):
            self._pending_parameter = PendingParameter(
                parameter_id=parameter_id,
                value=value,
                sequence=sequence,
                attempts=1,
                sent_time=now,
            )

    def _handle_parameter_reply(self, reply: protocol.ParameterReply) -> None:
        """只接受与当前等待参数的 ID 和序号都匹配的应答。"""
        pending = self._pending_parameter
        if pending is None:
            return
        if (
            reply.parameter_id != pending.parameter_id
            or reply.sequence != pending.sequence
        ):
            return

        if reply.status == 0:
            # 当前参数确认成功，移除队首，下个定时周期再发送下一个。
            if self._parameter_queue:
                self._parameter_queue.popleft()
            self._pending_parameter = None
        else:
            self.get_logger().error(
                f"STM32 rejected parameter id={reply.parameter_id}, "
                f"status={reply.status}"
            )
            self._parameter_transfer_failed = True
            self._pending_parameter = None
            self._parameter_queue.clear()

    def _on_parameters_changed(self, parameters) -> SetParametersResult:
        restart_required = {
            "can_interface",
            "transport",
            "serial_port",
            "serial_baud_rate",
            "command_rate_hz",
            "odom_frame",
            "base_frame",
            "left_joint_name",
            "right_joint_name",
            "push_parameters_on_start",
        }
        for parameter in parameters:
            if parameter.name in restart_required:
                return SetParametersResult(
                    successful=False,
                    reason=f"edit YAML and restart node to change {parameter.name}",
                )

        if self._latest_status is not None and self._latest_status.state == 2:
            for parameter in parameters:
                if parameter.name in protocol.PARAMETER_IDS:
                    return SetParametersResult(
                        successful=False,
                        reason="disable chassis before changing motor parameters",
                    )

        imu_positive_parameters = {
            "imu.accel_lsb_per_g",
            "imu.gyro_lsb_per_dps",
            "imu.angular_velocity_stddev",
            "imu.linear_acceleration_stddev",
            "imu.timeout_s",
        }
        imu_source_parameters = {
            "imu.x_source",
            "imu.y_source",
            "imu.z_source",
        }
        imu_sign_parameters = {"imu.x_sign", "imu.y_sign", "imu.z_sign"}
        for parameter in parameters:
            if parameter.name in imu_positive_parameters:
                imu_value = float(parameter.value)
                if not math.isfinite(imu_value) or imu_value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{parameter.name} must be a finite positive number",
                    )
            elif parameter.name in imu_source_parameters:
                if int(parameter.value) not in (0, 1, 2):
                    return SetParametersResult(
                        successful=False,
                        reason=f"{parameter.name} must be 0, 1, or 2",
                    )
            elif parameter.name in imu_sign_parameters:
                if float(parameter.value) not in (-1.0, 1.0):
                    return SetParametersResult(
                        successful=False,
                        reason=f"{parameter.name} must be -1.0 or 1.0",
                    )
            elif parameter.name == "imu.fusion_weight":
                weight = float(parameter.value)
                if not math.isfinite(weight) or not 0.0 <= weight <= 1.0:
                    return SetParametersResult(
                        successful=False,
                        reason="imu.fusion_weight must be between 0.0 and 1.0",
                    )
            elif parameter.name == "imu.frame_id" and not str(parameter.value):
                return SetParametersResult(
                    successful=False,
                    reason="imu.frame_id must not be empty",
                )

        changed: Dict[int, float] = {}
        for parameter in parameters:
            if parameter.name in protocol.PARAMETER_IDS:
                try:
                    value = float(parameter.value)
                except (TypeError, ValueError):
                    return SetParametersResult(
                        successful=False,
                        reason=f"{parameter.name} must be numeric",
                    )
                if not math.isfinite(value):
                    return SetParametersResult(
                        successful=False,
                        reason=f"{parameter.name} must be finite",
                    )
                minimum, maximum = protocol.PARAMETER_RANGES[parameter.name]
                if not minimum <= value <= maximum:
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            f"{parameter.name} must be in "
                            f"[{minimum}, {maximum}]"
                        ),
                    )
                if parameter.name == "geometry.chassis_type" and value not in (0.0, 1.0):
                    return SetParametersResult(
                        successful=False,
                        reason="geometry.chassis_type must be exactly 0 or 1",
                    )
                if parameter.name == "geometry.servo_pwm_per_rad" and abs(value) < 50.0:
                    return SetParametersResult(
                        successful=False,
                        reason="absolute servo_pwm_per_rad must be at least 50",
                    )
                changed[protocol.PARAMETER_IDS[parameter.name]] = value

        for parameter_id, value in changed.items():
            self._parameter_queue.append((parameter_id, value))
            self._parameter_transfer_failed = False
        return SetParametersResult(successful=True)

    def _on_reset_odometry(self, request: Trigger.Request, response: Trigger.Response):
        del request
        if self._latest_status is not None and self._latest_status.state == 2:
            response.success = False
            response.message = "disable chassis before resetting odometry"
            return response
        sent = self._send(
            protocol.make_system_command(
                protocol.SYSTEM_RESET_ODOMETRY, self._next_sequence()
            )
        )
        if sent:
            self._odometry.reset()
            self._last_odometry_sequence = None
            self._last_odometry_update_time = None
        response.success = sent
        response.message = "odometry reset command sent" if sent else "CAN unavailable"
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ChassisCanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # A launch process can shut the context down before this finally block
        # runs. Calling shutdown twice raises RCLError on ROS 2 Humble.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

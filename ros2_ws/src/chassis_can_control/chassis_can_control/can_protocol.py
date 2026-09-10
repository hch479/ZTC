"""与 STM32 `chassis_can_protocol.c` 一一对应的 Python 协议实现。"""

from dataclasses import dataclass
import math
import struct
from typing import Optional


CAN_DLC = 8

ID_MOTION_COMMAND = 0x101
ID_PARAMETER_COMMAND = 0x102
ID_SYSTEM_COMMAND = 0x103
ID_WHEEL_SPEED = 0x181
ID_MOTOR_OUTPUT = 0x182
ID_LEFT_ENCODER = 0x183
ID_RIGHT_ENCODER = 0x184
ID_STATUS = 0x185
ID_PARAMETER_REPLY = 0x186
ID_IMU_ACCELERATION = 0x187
ID_IMU_GYROSCOPE = 0x188
ID_IMU_STATUS = 0x189
ID_HEARTBEAT = 0x700

IMU_CALIBRATED = 0x01

CMD_ENABLE = 0x01
CMD_ESTOP = 0x02
SYSTEM_KEY0 = 0xA5
SYSTEM_KEY1 = 0x5A

PARAM_READ = 0
PARAM_WRITE = 1

SYSTEM_CLEAR_FAULT = 1
SYSTEM_SAVE_PARAMETERS = 2
SYSTEM_RESET_ODOMETRY = 3

STATE_NAMES = {
    0: "BOOT",
    1: "IDLE",
    2: "RUNNING",
    3: "TIMEOUT",
    4: "FAULT",
    5: "ESTOP",
}

FAULT_NAMES = {
    0x0001: "COMMAND_TIMEOUT",
    0x0002: "ESTOP",
    0x0004: "LOW_BATTERY",
    0x0008: "LEFT_ENCODER",
    0x0010: "RIGHT_ENCODER",
    0x0020: "LEFT_STALL",
    0x0040: "RIGHT_STALL",
    0x0080: "HARDWARE_DISABLED",
    0x0100: "CONTROL_OVERRUN",
    0x0200: "BAD_PARAMETER",
}

PARAMETER_IDS = {
    "controller.left_kp": 1,
    "controller.left_ki": 2,
    "controller.left_kff": 3,
    "controller.right_kp": 4,
    "controller.right_ki": 5,
    "controller.right_kff": 6,
    "controller.left_dead_forward": 7,
    "controller.left_dead_reverse": 8,
    "controller.right_dead_forward": 9,
    "controller.right_dead_reverse": 10,
    "limits.acceleration_mps2": 11,
    "limits.deceleration_mps2": 12,
    "controller.filter_alpha": 13,
    "controller.straight_sync_kp": 14,
    "controller.straight_sync_max_mps": 15,
    "geometry.wheel_track_m": 16,
    "geometry.wheel_diameter_m": 17,
    "geometry.counts_per_wheel_rev": 18,
    "limits.maximum_linear_speed_mps": 19,
    "safety.low_battery_mv": 20,
    "safety.command_timeout_ms": 21,
    "safety.stall_target_speed_mps": 22,
    "safety.stall_measured_speed_mps": 23,
    "safety.stall_pwm_ratio": 24,
    "safety.stall_time_ms": 25,
    "geometry.chassis_type": 26,
    "geometry.wheelbase_m": 27,
    "geometry.maximum_steering_angle_rad": 28,
    "geometry.servo_center_pwm": 29,
    "geometry.servo_pwm_per_rad": 30,
}

# 与 STM32 `chassis_params_validate()` 保持一致的上位机预检查范围。
# 这样明显错误的值不会等到 CAN 应答后才发现。
PARAMETER_RANGES = {
    "controller.left_kp": (0.0, 50000.0),
    "controller.left_ki": (0.0, 50000.0),
    "controller.left_kff": (0.0, 50000.0),
    "controller.right_kp": (0.0, 50000.0),
    "controller.right_ki": (0.0, 50000.0),
    "controller.right_kff": (0.0, 50000.0),
    "controller.left_dead_forward": (0.0, 10000.0),
    "controller.left_dead_reverse": (0.0, 10000.0),
    "controller.right_dead_forward": (0.0, 10000.0),
    "controller.right_dead_reverse": (0.0, 10000.0),
    "limits.acceleration_mps2": (0.02, 10.0),
    "limits.deceleration_mps2": (0.02, 20.0),
    "controller.filter_alpha": (0.01, 1.0),
    "controller.straight_sync_kp": (0.0, 10.0),
    "controller.straight_sync_max_mps": (0.0, 1.0),
    "geometry.wheel_track_m": (0.05, 2.0),
    "geometry.wheel_diameter_m": (0.02, 0.5),
    "geometry.counts_per_wheel_rev": (1.0, 1000000.0),
    "limits.maximum_linear_speed_mps": (0.02, 5.0),
    "safety.low_battery_mv": (6000.0, 30000.0),
    "safety.command_timeout_ms": (50.0, 5000.0),
    "safety.stall_target_speed_mps": (0.01, 2.0),
    "safety.stall_measured_speed_mps": (0.0, 1.0),
    "safety.stall_pwm_ratio": (0.10, 1.0),
    "safety.stall_time_ms": (100.0, 10000.0),
    "geometry.chassis_type": (0.0, 1.0),
    "geometry.wheelbase_m": (0.05, 2.0),
    "geometry.maximum_steering_angle_rad": (0.05, 1.4),
    "geometry.servo_center_pwm": (500.0, 2500.0),
    # 允许负数，负号表示舵机机械安装方向相反。
    "geometry.servo_pwm_per_rad": (-3000.0, 3000.0),
}


@dataclass
class CanFrame:
    can_id: int
    data: bytes

    def __post_init__(self) -> None:
        if not 0 <= self.can_id <= 0x7FF:
            raise ValueError("This project only accepts standard 11-bit CAN IDs")
        if len(self.data) != CAN_DLC:
            raise ValueError("All project CAN frames must contain exactly 8 bytes")


@dataclass
class WheelSpeed:
    left_target_mps: float
    left_measured_mps: float
    right_target_mps: float
    right_measured_mps: float


@dataclass
class MotorOutput:
    left_pwm: int
    right_pwm: int
    left_error_mps: float
    right_error_mps: float


@dataclass
class EncoderData:
    total_count: int
    delta_count: int
    sequence: int


@dataclass
class StatusData:
    state: int
    faults: int
    battery_mv: int
    last_command_sequence: int
    overrun_count: int


@dataclass
class ParameterReply:
    parameter_id: int
    status: int
    value: float
    sequence: int


@dataclass
class Heartbeat:
    uptime_ms: int
    state: int
    firmware_major: int
    firmware_minor: int


@dataclass
class ImuVector:
    """A three-axis IMU sample in the sensor driver's original ADC counts."""

    x: int
    y: int
    z: int
    sequence: int


@dataclass
class ImuStatus:
    sensor_type: int
    calibrated: bool
    calibration_samples: int
    calibration_target: int
    sequence: int


def crc8(data: bytes) -> int:
    """CRC-8/ATM：poly=0x07, init=0x00, refin=false, xorout=0x00。"""
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            if value & 0x80:
                value = ((value << 1) ^ 0x07) & 0xFF
            else:
                value = (value << 1) & 0xFF
    return value


def has_valid_crc(frame: CanFrame) -> bool:
    return crc8(frame.data[:7]) == frame.data[7]


def _clamp_i16(value: float) -> int:
    return max(-32768, min(32767, int(round(value))))


def make_motion_command(
    linear_mps: float,
    angular_radps: float,
    enable: bool,
    emergency_stop: bool,
    sequence: int,
) -> CanFrame:
    if not math.isfinite(linear_mps) or not math.isfinite(angular_radps):
        raise ValueError("Motion command must contain finite numbers")
    flags = (CMD_ENABLE if enable else 0) | (CMD_ESTOP if emergency_stop else 0)
    body = struct.pack(
        "<hhBBB",
        _clamp_i16(linear_mps * 1000.0),
        _clamp_i16(angular_radps * 1000.0),
        flags,
        1,  # 协议版本
        sequence & 0xFF,
    )
    return CanFrame(ID_MOTION_COMMAND, body + bytes([crc8(body)]))


def make_parameter_command(
    parameter_id: int, operation: int, value: float, sequence: int
) -> CanFrame:
    if not math.isfinite(value):
        raise ValueError("Parameter must be finite")
    body = struct.pack("<BBfB", parameter_id, operation, value, sequence & 0xFF)
    return CanFrame(ID_PARAMETER_COMMAND, body + bytes([crc8(body)]))


def make_system_command(opcode: int, sequence: int) -> CanFrame:
    body = struct.pack(
        "<BBBBBBB",
        opcode,
        SYSTEM_KEY0,
        SYSTEM_KEY1,
        sequence & 0xFF,
        0,
        0,
        0,
    )
    return CanFrame(ID_SYSTEM_COMMAND, body + bytes([crc8(body)]))


def decode_wheel_speed(frame: CanFrame) -> Optional[WheelSpeed]:
    if frame.can_id != ID_WHEEL_SPEED:
        return None
    left_target, left_actual, right_target, right_actual = struct.unpack(
        "<hhhh", frame.data
    )
    return WheelSpeed(
        left_target / 1000.0,
        left_actual / 1000.0,
        right_target / 1000.0,
        right_actual / 1000.0,
    )


def decode_motor_output(frame: CanFrame) -> Optional[MotorOutput]:
    if frame.can_id != ID_MOTOR_OUTPUT:
        return None
    left_pwm, right_pwm, left_error, right_error = struct.unpack("<hhhh", frame.data)
    return MotorOutput(left_pwm, right_pwm, left_error / 1000.0, right_error / 1000.0)


def decode_encoder(frame: CanFrame) -> Optional[EncoderData]:
    if frame.can_id not in (ID_LEFT_ENCODER, ID_RIGHT_ENCODER) or not has_valid_crc(frame):
        return None
    total, delta, sequence = struct.unpack("<ihB", frame.data[:7])
    return EncoderData(total, delta, sequence)


def decode_status(frame: CanFrame) -> Optional[StatusData]:
    if frame.can_id != ID_STATUS or not has_valid_crc(frame):
        return None
    state, faults, battery, command_sequence, overrun = struct.unpack(
        "<BHHBB", frame.data[:7]
    )
    return StatusData(state, faults, battery, command_sequence, overrun)


def decode_parameter_reply(frame: CanFrame) -> Optional[ParameterReply]:
    if frame.can_id != ID_PARAMETER_REPLY or not has_valid_crc(frame):
        return None
    parameter_id, status, value, sequence = struct.unpack("<BBfB", frame.data[:7])
    return ParameterReply(parameter_id, status, value, sequence)


def decode_heartbeat(frame: CanFrame) -> Optional[Heartbeat]:
    if frame.can_id != ID_HEARTBEAT or not has_valid_crc(frame):
        return None
    uptime, state, major, minor = struct.unpack("<IBBB", frame.data[:7])
    return Heartbeat(uptime, state, major, minor)


def decode_imu_vector(frame: CanFrame) -> Optional[ImuVector]:
    if frame.can_id not in (ID_IMU_ACCELERATION, ID_IMU_GYROSCOPE):
        return None
    if not has_valid_crc(frame):
        return None
    x, y, z, sequence = struct.unpack("<hhhB", frame.data[:7])
    return ImuVector(x, y, z, sequence)


def decode_imu_status(frame: CanFrame) -> Optional[ImuStatus]:
    if frame.can_id != ID_IMU_STATUS or not has_valid_crc(frame):
        return None
    sensor_type, flags, samples, target, sequence = struct.unpack(
        "<BBHHB", frame.data[:7]
    )
    return ImuStatus(
        sensor_type=sensor_type,
        calibrated=bool(flags & IMU_CALIBRATED),
        calibration_samples=samples,
        calibration_target=target,
        sequence=sequence,
    )


def fault_text(faults: int) -> str:
    names = [name for bit, name in FAULT_NAMES.items() if faults & bit]
    return "NONE" if not names else "|".join(names)

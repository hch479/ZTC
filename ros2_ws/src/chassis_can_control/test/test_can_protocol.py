import math
import struct

from chassis_can_control import can_protocol as protocol


def test_crc_known_vector():
    assert protocol.crc8(b"123456789") == 0xF4


def test_motion_command_layout():
    frame = protocol.make_motion_command(0.321, -0.456, True, False, 77)
    assert frame.can_id == protocol.ID_MOTION_COMMAND
    assert frame.data == bytes([0x41, 0x01, 0x38, 0xFE, 0x01, 0x01, 77, frame.data[7]])
    assert protocol.has_valid_crc(frame)


def test_parameter_float_round_trip():
    frame = protocol.make_parameter_command(1, protocol.PARAM_WRITE, 3000.5, 9)
    # 用应答 ID 模拟 STM32 回包；布局中的 float32 应能正确解出。
    body = frame.data[:7]
    reply = protocol.CanFrame(
        protocol.ID_PARAMETER_REPLY,
        bytes([1, 0]) + body[2:6] + bytes([9]) + bytes([protocol.crc8(bytes([1, 0]) + body[2:6] + bytes([9]))]),
    )
    decoded = protocol.decode_parameter_reply(reply)
    assert decoded is not None
    assert math.isclose(decoded.value, 3000.5, abs_tol=0.01)


def test_fault_text():
    assert protocol.fault_text(0) == "NONE"
    text = protocol.fault_text(0x0001 | 0x0020)
    assert "COMMAND_TIMEOUT" in text
    assert "LEFT_STALL" in text


def test_imu_vector_decode_and_crc_rejection():
    body = struct.pack("<hhhB", -123, 456, 16384, 27)
    frame = protocol.CanFrame(
        protocol.ID_IMU_ACCELERATION,
        body + bytes([protocol.crc8(body)]),
    )
    decoded = protocol.decode_imu_vector(frame)
    assert decoded == protocol.ImuVector(-123, 456, 16384, 27)

    damaged = bytearray(frame.data)
    damaged[0] ^= 0x01
    assert protocol.decode_imu_vector(
        protocol.CanFrame(protocol.ID_IMU_ACCELERATION, bytes(damaged))
    ) is None


def test_imu_status_decode():
    body = struct.pack("<BBHHB", 1, protocol.IMU_CALIBRATED, 500, 500, 91)
    frame = protocol.CanFrame(
        protocol.ID_IMU_STATUS,
        body + bytes([protocol.crc8(body)]),
    )
    decoded = protocol.decode_imu_status(frame)
    assert decoded is not None
    assert decoded.sensor_type == 1
    assert decoded.calibrated
    assert decoded.calibration_samples == 500
    assert decoded.calibration_target == 500
    assert decoded.sequence == 91

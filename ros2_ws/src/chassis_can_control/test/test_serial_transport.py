from chassis_can_control.can_protocol import CanFrame
from chassis_can_control.serial_transport import SerialTransport


def test_serial_packet_round_trip_without_hardware():
    transport = SerialTransport("/dev/null")
    original = CanFrame(0x185, bytes(range(8)))
    transport._receive_buffer.extend(SerialTransport._encode(original))
    assert transport.receive_available() == [original]


def test_parser_recovers_after_noise_and_bad_crc():
    transport = SerialTransport("/dev/null")
    original = CanFrame(0x700, b"12345678")
    bad = bytearray(SerialTransport._encode(original))
    bad[-1] ^= 0x55
    transport._receive_buffer.extend(b"noise" + bad + SerialTransport._encode(original))
    assert transport.receive_available() == [original]

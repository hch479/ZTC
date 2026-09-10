"""C30D 串口传输层：只依赖 Linux/Python 标准库。"""

import errno
import os
from typing import List

from .can_protocol import CanFrame, crc8


_SOF = b"\xA5\x5A"
_VERSION = 0x01
_FRAME_LENGTH = 15


class SerialTransport:
    """把 logical_id + 8 字节协议帧封装到 115200-8N1 串口。"""

    def __init__(self, device: str, baud_rate: int = 115200) -> None:
        self.interface = device
        self.baud_rate = baud_rate
        self._file_descriptor: int | None = None
        self._receive_buffer = bytearray()

    @property
    def is_open(self) -> bool:
        return self._file_descriptor is not None

    def open(self) -> None:
        # termios/tty 只存在于 Linux；放在这里导入也方便在 Windows 上做纯协议测试。
        import termios
        import tty

        self.close()
        if self.baud_rate != 115200:
            raise ValueError("This firmware currently supports 115200 baud only")

        descriptor = os.open(
            self.interface, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK
        )
        try:
            # raw 模式关闭回显、换行转换和软件流控。
            tty.setraw(descriptor)
            attributes = termios.tcgetattr(descriptor)
            attributes[4] = termios.B115200
            attributes[5] = termios.B115200
            attributes[2] |= termios.CLOCAL | termios.CREAD
            attributes[2] &= ~termios.CSTOPB
            attributes[2] &= ~termios.PARENB
            attributes[2] &= ~termios.CSIZE
            attributes[2] |= termios.CS8
            attributes[2] &= ~getattr(termios, "CRTSCTS", 0)
            termios.tcsetattr(descriptor, termios.TCSANOW, attributes)
            termios.tcflush(descriptor, termios.TCIOFLUSH)
        except Exception:
            os.close(descriptor)
            raise

        self._file_descriptor = descriptor
        self._receive_buffer.clear()

    def close(self) -> None:
        if self._file_descriptor is not None:
            os.close(self._file_descriptor)
            self._file_descriptor = None
        self._receive_buffer.clear()

    @staticmethod
    def _encode(frame: CanFrame) -> bytes:
        body = bytes(
            [
                _VERSION,
                frame.can_id & 0xFF,
                (frame.can_id >> 8) & 0xFF,
                len(frame.data),
            ]
        ) + frame.data
        return _SOF + body + bytes([crc8(body)])

    def send(self, frame: CanFrame) -> None:
        if self._file_descriptor is None:
            raise RuntimeError("Serial port is not open")

        packet = self._encode(frame)
        sent = 0
        while sent < len(packet):
            try:
                count = os.write(self._file_descriptor, packet[sent:])
            except BlockingIOError as error:
                raise OSError("Serial transmit buffer is full") from error
            if count <= 0:
                raise OSError("Serial write returned zero bytes")
            sent += count

    def _read_into_buffer(self) -> None:
        if self._file_descriptor is None:
            return
        while True:
            try:
                chunk = os.read(self._file_descriptor, 512)
            except BlockingIOError:
                break
            except OSError as error:
                if error.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    break
                raise
            if not chunk:
                break
            self._receive_buffer.extend(chunk)

        # 噪声或接错协议时限制缓存长度，避免内存无限增长。
        if len(self._receive_buffer) > 4096:
            del self._receive_buffer[:-2]

    def receive_available(self, maximum_frames: int = 64) -> List[CanFrame]:
        frames: List[CanFrame] = []
        self._read_into_buffer()

        while len(frames) < maximum_frames:
            header_index = self._receive_buffer.find(_SOF)
            if header_index < 0:
                # 最后一个 A5 可能是下一帧的首字节，应保留下来。
                keep = 1 if self._receive_buffer[-1:] == _SOF[:1] else 0
                if keep:
                    del self._receive_buffer[:-1]
                else:
                    self._receive_buffer.clear()
                break
            if header_index > 0:
                del self._receive_buffer[:header_index]
            if len(self._receive_buffer) < _FRAME_LENGTH:
                break

            packet = bytes(self._receive_buffer[:_FRAME_LENGTH])
            body = packet[2:14]
            logical_id = packet[3] | (packet[4] << 8)
            valid = (
                packet[2] == _VERSION
                and packet[5] == 8
                and logical_id <= 0x7FF
                and crc8(body) == packet[14]
            )
            if valid:
                frames.append(CanFrame(logical_id, packet[6:14]))
                del self._receive_buffer[:_FRAME_LENGTH]
            else:
                # 只丢一个字节，再搜索帧头，能尽快从错位或坏帧中恢复。
                del self._receive_buffer[0]

        return frames

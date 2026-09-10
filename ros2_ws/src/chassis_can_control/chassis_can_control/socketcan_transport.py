"""只使用 Python 标准库访问 Linux SocketCAN。"""

import socket
import struct
from typing import List

from .can_protocol import CanFrame


# Linux `struct can_frame`：can_id(4) + can_dlc(1) + padding(3) + data(8)。
_CAN_FRAME_FORMAT = "=IB3x8s"
_CAN_FRAME_SIZE = struct.calcsize(_CAN_FRAME_FORMAT)
_CAN_EFF_FLAG = 0x80000000
_CAN_RTR_FLAG = 0x40000000
_CAN_ERR_FLAG = 0x20000000
_CAN_SFF_MASK = 0x000007FF


class SocketCanTransport:
    """非阻塞 SocketCAN 封装，适合放在 ROS 定时器中轮询。"""

    def __init__(self, interface: str) -> None:
        self.interface = interface
        self._socket: socket.socket | None = None

    @property
    def is_open(self) -> bool:
        return self._socket is not None

    def open(self) -> None:
        self.close()
        can_socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        can_socket.setblocking(False)
        can_socket.bind((self.interface,))
        self._socket = can_socket

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close()
            self._socket = None

    def send(self, frame: CanFrame) -> None:
        if self._socket is None:
            raise RuntimeError("SocketCAN is not open")
        packed = struct.pack(_CAN_FRAME_FORMAT, frame.can_id, len(frame.data), frame.data)
        written = self._socket.send(packed)
        if written != _CAN_FRAME_SIZE:
            raise OSError(f"Incomplete CAN write: {written}/{_CAN_FRAME_SIZE}")

    def receive_available(self, maximum_frames: int = 64) -> List[CanFrame]:
        frames: List[CanFrame] = []
        if self._socket is None:
            return frames

        for _ in range(maximum_frames):
            try:
                raw = self._socket.recv(_CAN_FRAME_SIZE)
            except BlockingIOError:
                break

            if len(raw) != _CAN_FRAME_SIZE:
                continue
            raw_id, length, data = struct.unpack(_CAN_FRAME_FORMAT, raw)
            if raw_id & (_CAN_EFF_FLAG | _CAN_RTR_FLAG | _CAN_ERR_FLAG):
                continue
            if length != 8:
                continue
            frames.append(CanFrame(raw_id & _CAN_SFF_MASK, data))
        return frames


# 上位机第 4 模块：Linux 串口传输模块

> 本文件是该模块在旧任务中的完整归档：保留首次讲解正文，并把首次文档之后的专项追问统一融合在同一文件中。以后无需回到原任务查找。

## 本模块归档内容

- 首次讲解来源：`docs/17_RDK_X5_Linux串口传输模块零基础详解.md`
- 后续追问来源：原模块任务中的对话回答（此前未单独保存为文档）

---

# 第一部分：首次完整讲解

# RDK X5 Linux 串口传输模块零基础详解

## 1. 这份文档专门讲什么

本文只重点讲 RDK X5 上位机的第 4 个模块：

```text
ros2_ws/src/chassis_can_control/chassis_can_control/serial_transport.py
```

你已经大致理解了：

1. ROS 2 主节点模块；
2. ROS 速度命令处理模块；
3. Python 协议编解码模块。

这个串口模块正好接在第 3 个模块和 STM32 之间。它负责：

```text
Python 协议对象 ⇄ Linux 串口字节 ⇄ USB 串口芯片 ⇄ STM32 USART3
```

它不负责：

- 订阅 `/cmd_vel`；
- 决定电机是否使能；
- 计算左右轮目标速度；
- 执行 PI 闭环；
- 计算 ROS 里程计；
- 解释某个 logical ID 是轮速还是心跳。

它只解决一个问题：**怎样可靠地在 Linux 与 STM32 之间发送和接收一串字节。**

## 2. 先看模块在整个系统中的位置

发送方向：

```text
ROS 2 /cmd_vel
      ↓
chassis_can_node.py
决定实际下发速度、使能和急停
      ↓
can_protocol.py
生成 CanFrame(logical_id, data[8])
      ↓
serial_transport.py
包装成 15 字节并写入 Linux 串口
      ↓
/dev/serial/by-id/...0002-if00
      ↓ USB 线
USB 转串口芯片
      ↓ UART 电平
STM32 USART3
```

接收方向：

```text
STM32 生成轮速/编码器/状态/心跳
      ↓
USART3 发出 15 字节帧
      ↓ USB 转串口
Linux 串口设备文件
      ↓
serial_transport.py
缓存、找帧头、校验、还原 CanFrame
      ↓
chassis_can_node.py 的 _handle_frame()
      ↓
can_protocol.py 解码具体字段
      ↓
/motor/debug、/odom、/diagnostics
```

可以把 `serial_transport.py` 看成快递运输层：协议模块已经把内容装进“8 字节小盒子”，串口模块再加上地址标签、帧头和外层校验，通过 USB 运输。到达后，它拆掉外层包装，把小盒子交给主节点。

## 3. 串口到底是什么

### 3.1 “串行”是什么意思

串口将数据按位依次发送。一个字节 `0x32` 的二进制是：

```text
0011 0010
```

UART 会按照配置添加起始位、数据位和停止位，在一根发送线上依次传输。接收端必须使用相同波特率和帧格式。

### 3.2 当前串口配置：115200-8N1

```text
115200  每秒约 115200 bit
8       每个字符 8 个数据位
N       No parity，无奇偶校验
1       1 个停止位
```

UART 一个普通字节通常占：

```text
1 起始位 + 8 数据位 + 1 停止位 = 10 bit
```

理论有效字节速率约为：

```text
115200 / 10 = 11520 byte/s
```

一个外层帧是 15 字节，传输时间约：

```text
15 × 10 / 115200 ≈ 1.30 ms
```

### 3.3 UART 与 USB 串口不是同一个概念

STM32 使用 USART3。RDK 没有直接用 TTL 引脚，而是通过 USB 串口设备连接：

```text
RDK USB 主机
  ↕ USB 协议
WCH USB 转串口芯片
  ↕ UART TX/RX
STM32 USART3
```

Linux 驱动把这个 USB 设备呈现成串口设备文件。Python 不必自己实现 USB 协议，只需读写设备文件。

## 4. 为什么 Linux 串口看起来像文件

Linux 有“很多东西都是文件”的设计。硬盘文件、终端、串口都可以通过文件描述符读写。

当前稳定路径：

```text
/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00
```

执行：

```bash
ls -l /dev/serial/by-id/
```

可能看到：

```text
...0002-if00 -> ../../ttyACM1
```

`by-id` 文件其实是符号链接，最终指向 `/dev/ttyACM1`。为什么配置中优先使用 `by-id`：

- `ttyACM0`、`ttyACM1` 取决于设备发现顺序；
- 重启或重新插拔后编号可能交换；
- `by-id` 按设备序列号识别，更稳定；
- 当前资料明确 `0001` 是雷达，`0002` 是底盘。

## 5. 阅读本模块需要的 Python 基础

### 5.1 导入模块

```python
import errno
import os
from typing import List

from .can_protocol import CanFrame, crc8
```

含义：

- `os`：操作 Linux 文件描述符；
- `errno`：识别操作系统错误码；
- `List`：类型提示；
- `.can_protocol`：当前 Python 包内相邻的协议模块；
- `CanFrame`：逻辑帧数据类；
- `crc8`：CRC-8 算法。

开头的点：

```python
from .can_protocol ...
```

表示相对导入，即“从当前 `chassis_can_control` 包中导入”。

### 5.2 `bytes` 与 `bytearray`

`bytes` 是不可修改的字节序列：

```python
header = b"\xA5\x5A"
```

`bytearray` 是可修改的字节缓冲区：

```python
buffer = bytearray()
buffer.extend(b"\x01\x02")
del buffer[:1]
```

为什么发送数据常用 `bytes`：数据生成后不应被意外修改。

为什么接收缓存使用 `bytearray`：需要不断追加新字节、删除已处理部分。

### 5.3 十六进制

```python
0xA5
0x5A
0x01
```

`0x` 表示十六进制。一个十六进制字节范围是 `00`～`FF`。十六进制只是显示方式，内存中仍是二进制位。

### 5.4 切片

```python
packet[2:14]
```

取下标 2 到 13，不包含 14。

```python
packet[sent:]
```

取从 `sent` 到末尾的所有字节。

```python
del buffer[:15]
```

删除缓存开头 15 字节。

### 5.5 `None`

```python
self._file_descriptor: int | None = None
```

表示串口尚未打开时没有文件描述符；打开后它会变成整数。

### 5.6 类与 `self`

```python
class SerialTransport:
```

类是串口传输对象的模板。主节点创建：

```python
self._can = SerialTransport(device, baud_rate)
```

对象内部：

```python
self.interface
self.baud_rate
self._file_descriptor
self._receive_buffer
```

都属于这一份串口对象。

### 5.7 `@property`

```python
@property
def is_open(self) -> bool:
    return self._file_descriptor is not None
```

调用方像读取普通属性一样写：

```python
if self._can.is_open:
```

实际执行的是 `is_open()` 方法。它让外部只能查询，不必直接碰 `_file_descriptor`。

### 5.8 `@staticmethod`

```python
@staticmethod
def _encode(frame: CanFrame) -> bytes:
```

编码只需要 `frame`，不需要对象里的串口路径、文件描述符或接收缓存，所以它是静态方法，没有 `self` 参数。

## 6. 文件顶部的协议常量

```python
_SOF = b"\xA5\x5A"
_VERSION = 0x01
_FRAME_LENGTH = 15
```

### 6.1 `_SOF`

SOF 是 Start Of Frame，即帧起始标志：

```text
A5 5A
```

接收端在连续字节流中搜索它，判断一帧可能从哪里开始。

### 6.2 `_VERSION`

外层串口协议版本是 1。未来帧格式改变时可以增加版本，旧程序看到不认识的版本就拒绝解析。

### 6.3 `_FRAME_LENGTH`

当前外层帧固定 15 字节，不是可变长度。固定长度解析简单，也容易限制内存。

变量名前的单下划线表示模块内部使用，是 Python 约定，不是安全权限控制。

## 7. 15 字节串口帧格式

```text
下标  内容
0     A5，帧头第 1 字节
1     5A，帧头第 2 字节
2     01，外层协议版本
3     logical_id 低字节
4     logical_id 高字节
5     08，内部数据长度固定 8
6~13  logical data[8]
14    对下标 2~13 计算的外层 CRC-8
```

逻辑帧 `CanFrame` 只有：

```text
logical_id + data[8]
```

串口模块增加帧头、版本、长度和外层 CRC，使接收端能从无边界的字节流里重新找到帧。

## 8. 构造函数 `__init__()`

原代码：

```python
def __init__(self, device: str, baud_rate: int = 115200) -> None:
    self.interface = device
    self.baud_rate = baud_rate
    self._file_descriptor: int | None = None
    self._receive_buffer = bytearray()
```

逐行解释。

### 8.1 参数

```python
device: str
```

设备路径字符串，例如：

```text
/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00
```

```python
baud_rate: int = 115200
```

如果调用方不传波特率，默认是 115200。

```python
-> None
```

构造函数不返回业务结果。

### 8.2 保存配置，但尚未打开

```python
self.interface = device
self.baud_rate = baud_rate
```

只是记住要打开哪个设备和波特率。

```python
self._file_descriptor = None
```

此时尚未调用 Linux `open()`。

```python
self._receive_buffer = bytearray()
```

建立空接收缓存，用于保存半帧和多帧数据。

创建对象和打开硬件是两个动作：

```text
SerialTransport(...)  创建对象
transport.open()       真正打开串口
```

## 9. `open()`：打开和配置 Linux 串口

### 9.1 为什么在函数内部导入 `termios` 和 `tty`

```python
import termios
import tty
```

它们主要存在于类 Unix 系统。放在 `open()` 内部，Windows 上仍可以导入其他纯协议代码并运行不依赖硬件的测试；只有真正打开 Linux 串口才需要它们。

### 9.2 先关闭旧接口

```python
self.close()
```

如果这是断线重连，可能还有旧文件描述符。先关闭能避免泄漏和状态混乱。

### 9.3 限制波特率

```python
if self.baud_rate != 115200:
    raise ValueError("This firmware currently supports 115200 baud only")
```

STM32 当前固定配置为 115200。上位机写成其他值不会“自动协商”，只会收到乱码，因此程序直接拒绝。

`raise` 表示主动抛出异常，让调用者知道配置错误。

### 9.4 `os.open()`

```python
descriptor = os.open(
    self.interface,
    os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK,
)
```

返回的 `descriptor` 是小整数，例如 11。它不是串口数据，而是 Linux 内核分配的“已打开对象编号”。以后读写都把这个编号交给内核。

三个标志：

| 标志 | 含义 |
|---|---|
| `O_RDWR` | 以可读可写方式打开 |
| `O_NOCTTY` | 不让该串口成为进程的控制终端 |
| `O_NONBLOCK` | 非阻塞模式，无数据时立即返回/报 EAGAIN |

### 9.5 为什么必须非阻塞

主节点每 5 ms 轮询串口。如果使用阻塞读取而当前没有数据，ROS 事件循环可能一直卡在 `read()`：

- 无法继续发送运动命令；
- 无法发布诊断；
- 无法处理服务；
- 节点看起来“卡死”。

非阻塞模式下，没有数据就是“本轮没读到”，函数快速返回，下一次定时器再来。

### 9.6 `tty.setraw()`

```python
tty.setraw(descriptor)
```

终端默认可能处理回显、换行、特殊控制字符。二进制协议要求收到什么字节就保留什么字节，所以使用 raw 模式关闭文本终端转换。

否则 `0x0A` 等值可能被当作换行处理，二进制帧会被破坏。

### 9.7 读取现有串口配置

```python
attributes = termios.tcgetattr(descriptor)
```

返回一个配置列表。项目只修改需要的字段。

```python
attributes[4] = termios.B115200
attributes[5] = termios.B115200
```

下标 4 是输入波特率，下标 5 是输出波特率。

### 9.8 控制标志

```python
attributes[2] |= termios.CLOCAL | termios.CREAD
```

- `CREAD`：允许接收；
- `CLOCAL`：忽略调制解调器载波控制线，适合本地串口设备。

位运算 `|=` 表示把这些标志位置 1，同时保留其他位。

```python
attributes[2] &= ~termios.CSTOPB
```

清除双停止位标志，因此使用 1 个停止位。

```python
attributes[2] &= ~termios.PARENB
```

关闭奇偶校验。

```python
attributes[2] &= ~termios.CSIZE
attributes[2] |= termios.CS8
```

先清除数据位长度掩码，再选择 8 数据位。

```python
attributes[2] &= ~getattr(termios, "CRTSCTS", 0)
```

关闭 RTS/CTS 硬件流控。`getattr(..., 0)` 表示如果某个平台没有 `CRTSCTS` 常量就使用 0，增强兼容性。

### 9.9 应用配置与清缓存

```python
termios.tcsetattr(descriptor, termios.TCSANOW, attributes)
```

`TCSANOW` 表示立即应用。

```python
termios.tcflush(descriptor, termios.TCIOFLUSH)
```

清除打开前或重新配置期间残留的输入、输出数据，避免把旧半帧当作新通信开头。

### 9.10 异常时为什么立即关闭

```python
try:
    ...
except Exception:
    os.close(descriptor)
    raise
```

如果打开成功但配置中途失败，必须关闭临时文件描述符。最后的 `raise` 把原异常继续交给主节点处理。

### 9.11 全部成功后才记录为已打开

```python
self._file_descriptor = descriptor
self._receive_buffer.clear()
```

这样不会出现“配置只完成一半，`is_open` 却为 True”的状态。

## 10. `close()`：关闭串口

```python
def close(self) -> None:
    if self._file_descriptor is not None:
        os.close(self._file_descriptor)
        self._file_descriptor = None
    self._receive_buffer.clear()
```

步骤：

1. 已打开才调用 `os.close()`；
2. 设回 `None`，所以 `is_open` 变为 False；
3. 清空接收缓存。

关闭后清缓存很重要：重新连接时旧半帧不能和新设备字节拼在一起。

重复调用 `close()` 也安全，因为第二次发现描述符已经是 `None`。

## 11. `_encode()`：从逻辑帧到 15 字节

原代码：

```python
body = bytes(
    [
        _VERSION,
        frame.can_id & 0xFF,
        (frame.can_id >> 8) & 0xFF,
        len(frame.data),
    ]
) + frame.data

return _SOF + body + bytes([crc8(body)])
```

### 11.1 拆 logical ID

假设 ID 是 `0x0101`：

```python
frame.can_id & 0xFF
```

保留低 8 位，得到 `0x01`。

```python
(frame.can_id >> 8) & 0xFF
```

右移 8 位后取低 8 位，得到高字节 `0x01`。

因此按小端顺序发送：

```text
01 01
```

### 11.2 `bytes([...])`

列表中的每个整数必须在 0～255，转换成实际字节序列。

### 11.3 两次 `+`

```python
header + body + crc
```

连接三个不可变 `bytes`，形成一个完整 15 字节对象。

### 11.4 实际 `0.05 m/s` 示例

假设：

```text
linear.x = 0.05 m/s
angular.z = 0
enable = True
estop = False
sequence = 7
```

协议模块生成：

```text
logical_id = 0x101
data[8] = 32 00 00 00 01 01 07 A0
```

说明：

```text
32 00  = 小端 int16 的 50，即 0.05×1000
00 00  = 角速度 0
01     = enable 位为 1
01     = 内层协议版本
07     = sequence
A0     = data 前 7 字节的内层 CRC
```

串口模块包装后的真实 15 字节：

```text
A5 5A 01 01 01 08 32 00 00 00 01 01 07 A0 EB
```

其中最后 `EB` 是对下标 2～13 的外层 CRC。

两层 CRC 的作用范围不同：

```text
内层 CRC：保护 8 字节逻辑消息
外层 CRC：保护版本、ID、长度和 8 字节数据
```

## 12. `send()`：为什么要循环写

```python
def send(self, frame: CanFrame) -> None:
    if self._file_descriptor is None:
        raise RuntimeError("Serial port is not open")

    packet = self._encode(frame)
    sent = 0
    while sent < len(packet):
        count = os.write(self._file_descriptor, packet[sent:])
        sent += count
```

### 12.1 先检查是否打开

未打开时不能写。`RuntimeError` 表示程序状态不正确，不是硬件返回的数据。

### 12.2 为什么 `os.write()` 可能没写完

调用系统写 15 字节，并不保证一定一次接受全部 15 字节。操作系统发送缓冲区空间不足时，非阻塞写可能只接受前几字节。

例如第一次返回 9：

```text
sent = 9
下一次写 packet[9:]，只写剩余 6 字节
```

循环直到 `sent == 15`。

如果每次都从头写，会重复前面的字节，STM32 无法正确组帧。

### 12.3 `BlockingIOError`

```python
except BlockingIOError as error:
    raise OSError("Serial transmit buffer is full") from error
```

非阻塞模式下缓冲区完全没有空间时，Python 可能抛出 `BlockingIOError`。代码转换为更容易理解的 `OSError`，主节点捕获后关闭串口，之后尝试重连。

### 12.4 为什么检查 `count <= 0`

正常写入应返回正数。返回 0 会让循环永远不前进，因此立即报错，避免死循环。

## 13. `_read_into_buffer()`：从内核读取现有字节

### 13.1 没打开就返回

```python
if self._file_descriptor is None:
    return
```

### 13.2 一次最多请求 512 字节

```python
chunk = os.read(self._file_descriptor, 512)
```

512 是本次允许读的最大值，不代表必须收到 512。可能返回：

```text
0 字节
3 字节
15 字节
37 字节
512 字节
```

串口是字节流，没有“每次 read 正好一帧”的保证。

### 13.3 循环读到暂时没有数据

```python
while True:
```

把内核缓冲区当前已有数据尽量读完，避免高遥测频率下长期积压。

```python
except BlockingIOError:
    break
```

表示目前没有更多数据，不是严重错误。

有些系统用普通 `OSError` 携带：

```text
EAGAIN
EWOULDBLOCK
```

它们同样表示“现在没有数据”，所以也正常结束本轮读取。其他错误继续抛给主节点，例如 USB 被拔出。

### 13.4 把新数据追加到缓存

```python
self._receive_buffer.extend(chunk)
```

不是覆盖原缓存。假设上次留下半帧 6 字节，本次又读到 9 字节，追加后正好成为完整 15 字节。

### 13.5 4096 字节上限

```python
if len(self._receive_buffer) > 4096:
    del self._receive_buffer[:-2]
```

正常情况下解析器会持续删除合法帧。如果接错设备、波特率错误或输入全是噪声，缓存不能无限增长。

为什么保留最后 2 字节：它们可能恰好包含帧头 `A5 5A`，下一轮可以继续解析。

## 14. 为什么会有半包、粘包和噪声

### 14.1 半包

STM32 发送一个 15 字节帧，但 Linux 第一次只读到前 7 字节：

```text
A5 5A 01 00 07 08 12
```

剩余 8 字节下一次才到。模块不能丢掉前 7 字节，所以需要跨调用缓存。

### 14.2 粘包

Linux 一次读取到两个完整帧共 30 字节。它不能把 30 字节当成一个大消息，而应循环解析两帧。

“半包/粘包”不是串口硬件真的把包粘住，而是字节流 API 没有保存应用层消息边界。

### 14.3 噪声或错位

节点可能在 STM32 正发送到一半时启动，第一批数据从某帧中间开始。还可能存在坏字节。解析器必须搜索帧头并能重新同步。

## 15. `receive_available()`：完整解析过程

函数返回：

```python
List[CanFrame]
```

即本轮能成功解析出的 0～64 个逻辑帧。

### 15.1 创建结果列表并读入新数据

```python
frames = []
self._read_into_buffer()
```

### 15.2 限制每轮最多 64 帧

```python
while len(frames) < maximum_frames:
```

防止一次回调解析无限多数据，长期占用 ROS 事件循环。剩余数据留在缓存中，下一个 5 ms 周期继续处理。

### 15.3 搜索帧头

```python
header_index = self._receive_buffer.find(_SOF)
```

结果：

```text
0     缓存开头就是 A5 5A
5     前 5 字节是噪声，第 5 个位置找到帧头
-1    当前没有完整 A5 5A
```

### 15.4 找不到帧头时为什么可能保留一个 `A5`

假设本轮最后一个字节是：

```text
A5
```

下一轮第一个字节可能是 `5A`。如果把这个单独的 `A5` 清掉，就错过了跨两次读取的帧头。

代码：

```python
keep = 1 if buffer[-1:] == _SOF[:1] else 0
```

这里使用 `buffer[-1:]` 而不是 `buffer[-1]`，空缓存时切片仍安全返回空字节串，不会抛索引异常。

### 15.5 删除帧头前的噪声

```python
if header_index > 0:
    del self._receive_buffer[:header_index]
```

执行后缓存下标 0 必然是 `A5`，下标 1 是 `5A`。

### 15.6 不足 15 字节就等待

```python
if len(self._receive_buffer) < 15:
    break
```

不能把半帧当完整帧，也不能阻塞等待。保留缓存，下次定时器继续。

### 15.7 复制候选帧

```python
packet = bytes(self._receive_buffer[:15])
```

复制为不可变 `bytes`。即使随后删除缓存内容，当前候选帧仍保持不变。

### 15.8 取出外层 body 与 logical ID

```python
body = packet[2:14]
logical_id = packet[3] | (packet[4] << 8)
```

ID 低字节在前。假设字节是：

```text
85 01
```

计算：

```text
0x85 | (0x01 << 8) = 0x185
```

这就是状态帧 ID。

### 15.9 四项合法性检查

```python
valid = (
    packet[2] == 1
    and packet[5] == 8
    and logical_id <= 0x7FF
    and crc8(body) == packet[14]
)
```

分别检查：

1. 外层版本正确；
2. 内部数据固定 8 字节；
3. ID 在 11 位标准 CAN ID 范围内；
4. 外层 CRC 正确。

找到 `A5 5A` 不等于一定是真帧。数据区也可能偶然出现 `A5 5A`，所以还必须检查后续字段和 CRC。

### 15.10 合法帧

```python
frames.append(CanFrame(logical_id, packet[6:14]))
del self._receive_buffer[:15]
```

去掉外层包装，只把 ID 和 8 字节交给上层，然后从缓存删除已消费的整帧。

### 15.11 非法帧为什么只丢 1 字节

```python
del self._receive_buffer[0]
```

如果直接删除 15 字节，坏候选帧中间可能已经包含下一帧真正的 `A5 5A`，会连下一帧一起丢掉。

只丢第一个 `A5`，然后重新搜索，恢复同步更快。

## 16. 它怎样连接 ROS 主节点

### 16.1 从 YAML 读取串口配置

```yaml
transport: serial
serial_port: /dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00
serial_baud_rate: 115200
```

主节点：

```python
if transport == "serial":
    self._can = SerialTransport(serial_port, serial_baud_rate)
```

变量名仍叫 `_can`，因为主节点同时支持 `SerialTransport` 与 `SocketCanTransport`。更准确的通用名字可以是 `_transport`，但当前代码沿用了最早的命名。

### 16.2 两个传输对象使用相同接口

主节点只依赖：

```text
interface
is_open
open()
close()
send(frame)
receive_available()
```

串口对象和 CAN 对象都实现这些成员，所以主节点其余控制逻辑不需要写两份。

### 16.3 打开和自动重连

```python
def _open_can_if_needed(self):
```

虽然函数名有 CAN，它也负责串口。如果已经打开直接返回；否则至少间隔 1 秒才重试一次。

这样 USB 拔掉时不会每 5 ms 打印一次错误、疯狂调用 `open()`。

### 16.4 发送连接

主节点先调用协议模块：

```python
frame = protocol.make_motion_command(...)
```

再调用统一包装：

```python
self._send(frame)
```

`_send()` 内部调用：

```python
self._can.send(frame)
```

当前 `_can` 实际就是 `SerialTransport` 对象。

如果发送出现 `OSError`：

```text
记录日志
close() 串口
后续定时器再次尝试 open()
```

### 16.5 接收连接

ROS 定时器：

```python
self.create_timer(0.005, self._poll_can)
```

每 5 ms 调用：

```python
frames = self._can.receive_available()
for frame in frames:
    self._handle_frame(frame)
```

串口模块只恢复 `CanFrame`；`_handle_frame()` 再按 ID 调用协议解码函数。

### 16.6 节点退出

节点销毁前：

```text
软件使能设为 False
连续发送三次禁止运动命令
关闭串口
```

这是尽最大可能让 STM32 接收到停机命令。STM32 自己还有命令超时保护，不能只依赖这三帧。

## 17. 它怎样连接协议模块

发送时：

```text
can_protocol.py 决定 data[8] 每个字节的业务含义
serial_transport.py 不修改 data[8]，只增加外层
```

接收时：

```text
serial_transport.py 只检查外层并还原 CanFrame
can_protocol.py 根据 logical_id 解码 data[8]
```

分层的好处：

- 串口解析不需要知道速度公式；
- 业务协议不需要知道 Linux `termios`；
- 以后切 CAN 时大部分上层代码不变；
- 协议与传输可以分别单元测试。

## 18. 它怎样连接 STM32 下位机

Linux 对应文件：

```text
serial_transport.py
```

STM32 对应文件：

```text
CHASSIS_SERIAL/src/chassis_serial_transport.c
```

两端必须完全一致：

| 项目 | RDK Python | STM32 C |
|---|---|---|
| 帧头 | `_SOF = A5 5A` | `CHASSIS_SERIAL_SOF0/1` |
| 版本 | `_VERSION = 1` | `CHASSIS_SERIAL_VERSION = 1` |
| 帧长 | `_FRAME_LENGTH = 15` | `CHASSIS_SERIAL_FRAME_LENGTH = 15` |
| ID 顺序 | 低字节在前 | 低字节在前 |
| 数据长度 | 8 | `CHASSIS_CAN_DLC = 8` |
| 外层 CRC | 对 2～13 | 对 2～13 |
| 波特率 | 115200 | USART3 115200 |

STM32 接收在硬件中断中逐字节完成，合法整帧进入 8 槽环形队列；FreeRTOS 串口任务再取出处理。RDK 不知道这个队列细节，只需按照协议持续发送。

## 19. 为什么两个 ROS 节点会破坏串口接收

你之前遇到过 `STM32 heartbeat missing`，实际发现两个 `chassis_can_node` 同时打开同一个串口。

两个进程读取同一个 Linux 字节流时，不是每个进程都收到完整副本，而可能发生：

```text
STM32 发：A5 5A 01 00 07 08 ...
节点 A 读：A5 5A 01
节点 B 读：00 07 08 ...
```

两个节点都凑不齐 15 字节合法帧，因此各自心跳超时。

检查进程：

```bash
pgrep -af 'chassis_serial.launch.py|chassis_can_node'
```

查看谁占串口：

```bash
fuser -v /dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00
```

正确做法是同一底盘串口只运行一套节点。

## 20. Linux 中怎样检查串口

### 20.1 查看稳定设备名

```bash
ls -l /dev/serial/by-id/
```

### 20.2 查看 USB 设备

```bash
lsusb
```

当前 WCH 设备通常显示类似：

```text
QinHeng Electronics USB Single Serial
```

### 20.3 查看设备权限

```bash
ls -l /dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00
groups
```

串口通常属于 `dialout` 组。如果 `wheeltec` 不在该组：

```bash
sudo usermod -aG dialout wheeltec
```

然后退出 SSH 并重新登录。`usermod` 修改的是用户组数据库，当前已登录 shell 不会自动获得新组身份。

### 20.4 查看内核识别日志

```bash
dmesg | tail -50
```

某些系统读取 `dmesg` 需要：

```bash
sudo dmesg | tail -50
```

拔插 USB 后可检查设备是否注册为 `ttyACM*`。

### 20.5 查看占用进程

```bash
fuser -v /dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00
```

不要在 ROS 节点运行时再用串口助手打开同一设备。

## 21. 为什么不建议直接 `cat` 底盘串口

你可能见过：

```bash
cat /dev/ttyACM1
```

本项目传输的是二进制帧，包含不可打印字节。终端会显示乱码，而且 `cat` 会与 ROS 节点抢数据。

也不要在节点运行时执行：

```bash
screen /dev/ttyACM1 115200
minicom
```

需要观察业务数据时优先使用：

```bash
ros2 topic echo /diagnostics
ros2 topic echo /motor/debug
```

## 22. 异常和重连的完整流程

假设运行中拔掉 USB：

1. `os.read()` 或 `os.write()` 抛出 `OSError`；
2. 主节点 `_poll_can()` 或 `_send()` 捕获；
3. 最多每 2 秒记录一次错误日志；
4. 调用 `SerialTransport.close()`；
5. `is_open` 变成 False；
6. 后续发送/接收定时器调用 `_open_can_if_needed()`；
7. 至少间隔 1 秒尝试重新打开；
8. USB 恢复且路径仍存在后重新成功；
9. 接收缓存从空开始；
10. 等待 STM32 新遥测帧，心跳恢复。

电机安全不能只靠 Linux 重连。STM32 超过命令时间没有合法运动帧会独立停止输出。

## 23. 为什么有两层“非阻塞”思想

第一层是操作系统：

```text
O_NONBLOCK
```

没有字节时 `read()` 不等待。

第二层是 ROS 定时器：

```text
每 5 ms 调一次 receive_available()
```

每次只处理当前已有的数据并快速返回。这样单线程事件循环还能及时执行发送、服务和诊断回调。

这不是“串口读取不可靠”，而是事件驱动程序常用的设计。

## 24. 单元测试怎样验证解析器

测试文件：

```text
test/test_serial_transport.py
```

### 24.1 编码后再解码

测试不打开真实串口，而是：

1. 创建一个 `CanFrame`；
2. 调用 `_encode()` 生成 15 字节；
3. 直接塞入 `_receive_buffer`；
4. 调用 `receive_available()`；
5. 断言输出与原帧相等。

这叫 round trip：编码再解码应恢复原值。

### 24.2 噪声和坏 CRC 恢复

测试构造：

```text
noise + 坏CRC帧 + 正确帧
```

期望解析器跳过噪声和坏帧，最终返回正确帧。这验证了重新同步逻辑。

运行当前纯 Python 协议测试：

```bash
cd ~/chassis_project/ros2_ws/src/chassis_can_control
python3 test/run_protocol_tests.py
```

这类测试可以验证软件字节逻辑，但不能代替真实 USB、波特率、接线和 STM32 测试。

## 25. 常见错误怎样判断

### 25.1 `No such file or directory`

设备路径不存在。检查：

```bash
ls -l /dev/serial/by-id/
lsusb
```

可能原因：USB 未插、STM32 未供电、地址写错、`0002` 不存在。

### 25.2 `Permission denied`

用户没有串口权限。检查 `groups` 和设备所属组。

### 25.3 `Device or resource busy`

其他程序独占设备。检查：

```bash
fuser -v /dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00
```

### 25.4 设备存在但心跳丢失

依次检查：

1. 是否重复节点；
2. 是否把 `0001` 雷达当成底盘；
3. 两端是否都是 115200；
4. STM32 是否运行当前串口固件；
5. USB 是否间歇断连；
6. 启动日志是否显示成功打开；
7. `fuser` 是否只有一个节点占用。

### 25.5 日志里反复 open/close

多半是 USB 接触、供电或驱动问题，不是 ROS `/cmd_vel` 的问题。传输层尚未稳定时不要调 PI 参数。

## 26. 日常观察命令及其意义

加载环境：

```bash
source /opt/ros/humble/setup.bash
source ~/chassis_project/ros2_ws/install/setup.bash
```

检查节点：

```bash
ros2 node list
```

检查通信结果：

```bash
ros2 topic echo --once /diagnostics
```

正常重点：

```text
level: 0
message: IDLE 或 RUNNING
faults: NONE
transport: serial
interface: ...0002-if00
heartbeat_age: 小于 2 秒
```

检查进程和串口占用：

```bash
pgrep -af 'chassis_serial.launch.py|chassis_can_node'
fuser -v /dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00
```

检查后台日志：

```bash
tail -f ~/chassis_project/chassis_serial.log
```

`Ctrl+C` 只停止 `tail` 显示，不停止后台节点。

## 27. 逐函数总结

| 函数/属性 | 输入 | 输出 | 作用 |
|---|---|---|---|
| `__init__()` | 路径、波特率 | 新对象 | 保存配置、建立空缓存 |
| `is_open` | 无 | bool | 判断描述符是否存在 |
| `open()` | 无 | 无 | 打开设备并配置 115200-8N1 raw 非阻塞 |
| `close()` | 无 | 无 | 关闭描述符并清缓存 |
| `_encode()` | `CanFrame` | 15 字节 | 加帧头、版本、ID、长度和外层 CRC |
| `send()` | `CanFrame` | 无 | 循环写完全部 15 字节 |
| `_read_into_buffer()` | 无 | 无 | 把内核现有字节追加到缓存 |
| `receive_available()` | 最大帧数 | `List[CanFrame]` | 处理半包、粘包、噪声、CRC并返回逻辑帧 |

## 28. 你真正需要记住的主线

发送：

```text
can_protocol.py 生成 CanFrame
→ SerialTransport._encode() 生成 15 字节
→ os.write() 写 Linux 设备文件
→ USB 串口
→ STM32 USART3
```

接收：

```text
STM32 USART3 发字节
→ Linux 驱动放入内核接收缓冲
→ os.read() 非阻塞读取
→ bytearray 跨周期缓存
→ 搜索 A5 5A
→ 检查版本、长度、ID、CRC
→ 恢复 CanFrame
→ 主节点按 logical ID 处理
→ 发布 ROS 话题
```

这个模块最重要的技术点不是“调用一个串口库”，而是：

- Linux 设备文件与文件描述符；
- `termios` 的 115200-8N1 raw 配置；
- 非阻塞 I/O；
- 处理部分写入；
- 跨调用接收缓存；
- 半包、粘包和噪声恢复；
- 帧头、固定长度和双层 CRC；
- 异常关闭与限频自动重连；
- 与 ROS 定时器、协议模块和 STM32 接收中断分层连接。

---

# 第二部分：首次文档之后的追问与补充

## 后续追问：稳定串口路径与 `_SOF` 查找

### 原问题

> 当前稳定路径是怎么得到的？`_SOF` 明明是两个字节，`header_index = buffer.find(_SOF)` 和 `header_index < 0` 是什么意思？

### 1. 稳定路径来自 Linux `udev`

在 RDK 执行：

```bash
ls -l /dev/serial/by-id/
```

可以看到类似：

```text
usb-WCH.CN_USB_Single_Serial_0002-if00 -> ../../ttyACM1
```

`/dev/serial/by-id/...0002-if00` 是符号链接，类似 Windows 快捷方式。它根据 USB 设备报告的厂商、产品、序列号和接口号由 `udev` 创建。查看最终设备：

```bash
readlink -f /dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00
```

`/dev/ttyACM0`、`ttyACM1` 会随识别顺序变化；`by-id` 依据设备身份，通常更稳定。当前通过资料、设备映射和实际心跳三层确认 `0002` 是 STM32 底盘，`0001` 是雷达。更换 USB 芯片或控制板后仍应重新检查。

### 2. 两字节 `_SOF` 与一个整数下标并不矛盾

```python
_SOF = b"\xA5\x5A"
header_index = self._receive_buffer.find(_SOF)
```

`find()` 查找的是连续子串 `A5 5A`，返回这两个字节中第一个字节的起始位置：

```text
缓存 A5 5A 01 ...       → 0
缓存 11 22 A5 5A ...    → 2
缓存 11 22 33           → -1
```

正常下标从 0 开始，不可能小于 0，所以：

```python
if header_index < 0:
```

就是“当前缓存里没有完整找到 `A5 5A`”。不能写 `if not header_index`，因为帧头正好在下标 0 时，0 也会被 Python 当成假。

### 3. 为什么找不到时还可能保留最后一个 `A5`

串口读取可能把两字节帧头拆到两次读取：

```text
本次末尾：... A5
下次开头：5A ...
```

若缓存最后一个字节是 `_SOF[:1]`，程序只保留这个 `A5`，等待下一次与 `5A` 组合；否则当前缓存没有任何可能成为帧头的内容，可以清空。

因此接收状态机同时处理：

- 帧头前噪声；
- 半个帧头；
- 半帧；
- 粘包；
- CRC 错误后的重新同步。

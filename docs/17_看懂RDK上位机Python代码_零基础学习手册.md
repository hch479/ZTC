# 看懂 RDK X5 上位机 Python 代码：面向本项目的零基础学习手册

## 1. 这份手册怎样使用

这不是一份从“打印 Hello World”开始的通用 Python 教科书，而是一份专门帮助你看懂当前 RDK X5 上位机工程的阅读指南。

工程源码位于：

```text
ros2_ws/src/chassis_can_control/
```

RDK 上的实际路径是：

```text
/home/wheeltec/chassis_project/ros2_ws/src/chassis_can_control/
```

建议按下面的节奏学习：

1. 第一遍只看模块和数据流，不研究每个 Python 语法；
2. 第二遍跟着本文逐函数阅读；
3. 第三遍在 RDK 终端观察函数产生的 ROS 数据；
4. 最后再尝试修改一处很小的内容，例如诊断文字或发布频率。

你暂时不需要先学完整本 Python 书。本项目真正反复使用的语法只有：变量、条件、循环、函数、类、对象、列表、字典、字节串、异常和回调。

## 2. 先记住整个上位机在做什么

上位机程序的核心任务只有两条数据链。

### 2.1 命令下发链

```text
ROS /cmd_vel
    ↓
chassis_can_node.py 接收线速度和角速度
    ↓
can_protocol.py 编成 logical ID + 8 字节
    ↓
serial_transport.py 包成 15 字节串口帧
    ↓
Linux 串口 0002
    ↓
STM32 下位机
```

### 2.2 状态上传链

```text
STM32 发送 15 字节串口帧
    ↓
serial_transport.py 找帧头、校验、拆出 8 字节
    ↓
can_protocol.py 解码成有名字的数据对象
    ↓
chassis_can_node.py 按消息 ID 分流
    ↓
/diagnostics、/motor/debug、/odom、/joint_states、/tf
```

只要始终知道自己正在读的是哪一条链，就不容易迷路。

## 3. 六个 Python 文件分别负责什么

| 文件 | 作用 | 阅读优先级 |
|---|---|---:|
| `chassis_can_node.py` | ROS 2 主节点，组织全部功能 | 最高 |
| `can_protocol.py` | 8 字节逻辑协议的编码和解码 | 高 |
| `serial_transport.py` | Linux 串口打开、发送、缓存、组帧 | 高 |
| `wheel_odometry.py` | 用左右编码器计算位置和方向 | 中 |
| `chassis_serial.launch.py` | 用 YAML 参数启动节点 | 中 |
| `setup.py` | 告诉 ROS 怎样安装并找到 Python 入口 | 中 |

还有一个：

```text
socketcan_transport.py
```

它是以后 CAN 连接线到位时使用的传输层。当前实车走串口，第一次阅读可以先跳过它。

## 4. Python 和你可能熟悉的 C 语言有什么区别

### 4.1 Python 用缩进表示代码块

C 语言：

```c
if (enabled) {
    speed = 0.05;
}
```

Python：

```python
if enabled:
    speed = 0.05
```

Python 没有大括号和行末分号。冒号后缩进的行属于该代码块。本工程统一使用 4 个空格缩进。

### 4.2 Python 变量不需要先声明类型

```python
speed = 0.05
enabled = False
name = "chassis_can_node"
```

这里分别是浮点数、布尔值和字符串。

项目中经常出现类型提示：

```python
def _send(self, frame: CanFrame) -> bool:
```

可以读成：

```text
这个函数接收一个 CanFrame，预期返回 bool。
```

类型提示主要帮助阅读器和编辑器理解代码，不等同于 C 的强制类型声明。

### 4.3 `None` 表示“当前没有数据”

```python
self._latest_heartbeat = None
```

节点刚启动时还没收到 STM32 心跳，因此没有心跳对象。收到后它会变成一个 `Heartbeat` 对象。

常见判断：

```python
if self._latest_heartbeat is None:
    # 还没有心跳
```

### 4.4 `and`、`or`、`not`

```python
if command_is_fresh and self._drive_enabled and not self._emergency_stop:
```

等价于：

```text
命令仍然新鲜
并且 软件已经使能
并且 不处于急停
```

三个条件同时满足才执行真正速度。

### 4.5 列表

```python
values = [1.0, 2.0, 3.0]
```

列表有顺序，可以按下标访问：

```python
values[0]  # 第一个元素
```

`/motor/debug` 就使用一个浮点数列表。

### 4.6 字典

```python
STATE_NAMES = {
    0: "BOOT",
    1: "IDLE",
    2: "RUNNING",
}
```

字典用键查找值：

```python
STATE_NAMES[1]  # 得到 "IDLE"
```

代码使用更安全的 `get()`：

```python
STATE_NAMES.get(state, f"UNKNOWN({state})")
```

如果找不到该状态，就显示 `UNKNOWN(...)`，不会直接抛出错误。

### 4.7 元组和多变量赋值

```python
minimum, maximum = (0.0, 50000.0)
```

左边两个变量分别接收右边两个值。协议解码中也经常这样写：

```python
left_target, left_actual, right_target, right_actual = struct.unpack(...)
```

### 4.8 字符串格式化

```python
f"heartbeat age = {heartbeat_age:.3f} s"
```

前面的 `f` 表示格式化字符串；花括号放变量；`.3f` 表示保留三位小数。

### 4.9 切片

```python
frame.data[:7]
```

表示从开头取到下标 7 之前，也就是第 0～6 字节。协议前 7 字节是有效内容，第 8 字节是 CRC。

```python
packet[2:14]
```

表示取第 2～13 字节。

### 4.10 `bytes` 和 `bytearray`

```text
bytes      不可修改的字节序列，适合一帧已经生成的数据
bytearray  可修改的字节缓存，适合不断追加串口数据
```

例如：

```python
self._receive_buffer = bytearray()
self._receive_buffer.extend(chunk)
```

表示把刚读到的串口字节追加到接收缓存末尾。

## 5. 函数应该怎样阅读

例子：

```python
def crc8(data: bytes) -> int:
    value = 0
    ...
    return value
```

阅读一个函数时固定回答四个问题：

1. 谁调用这个函数？
2. 输入参数是什么？
3. 它改变了哪些对象状态？
4. 它返回什么，或者产生了什么外部效果？

以上例为例：

```text
调用者：协议编码和校验代码
输入：一串 bytes
修改状态：不修改节点状态，只使用局部变量
返回：0～255 的 CRC 整数
```

### 5.1 局部变量和对象成员

```python
now = time.monotonic()
self._last_command_time = now
```

- `now` 是局部变量，函数结束后不用再保留；
- `self._last_command_time` 属于节点对象，其他回调以后还会读取。

看到 `self.` 时，要意识到程序正在读写“节点长期保存的状态”。

## 6. 类、对象和 `self`

### 6.1 类是一张设计图

```python
class SerialTransport:
    ...
```

它规定串口对象具有什么数据和函数。

创建对象：

```python
transport = SerialTransport(device_path, 115200)
```

这个 `transport` 对象保存自己的设备路径、波特率、文件描述符和接收缓存。

### 6.2 `self` 就是“当前这个对象”

```python
class SerialTransport:
    def __init__(self, device, baud_rate):
        self.interface = device
        self.baud_rate = baud_rate
```

创建对象时：

```python
SerialTransport("/dev/...0002...", 115200)
```

Python 自动把新对象作为 `self` 传入。对象最后保存：

```text
interface = /dev/...0002...
baud_rate = 115200
```

### 6.3 `__init__` 是初始化函数

对象创建时自动执行：

```python
node = ChassisCanNode()
```

会进入：

```python
def __init__(self) -> None:
```

主节点在这里声明 ROS 参数、打开串口、创建发布者、订阅者、服务和定时器。

### 6.4 继承和 `super()`

```python
class ChassisCanNode(Node):
```

表示 `ChassisCanNode` 在 ROS 2 的 `Node` 基础上扩展。

```python
super().__init__("chassis_can_node")
```

先调用父类 `Node` 的初始化，创建真正的 ROS 节点。之后当前类才增加底盘功能。

### 6.5 `@dataclass`

协议文件中：

```python
@dataclass
class Heartbeat:
    uptime_ms: int
    state: int
    firmware_major: int
    firmware_minor: int
```

这是一个主要用来保存数据的类。解码后得到：

```python
heartbeat = Heartbeat(120000, 1, 1, 0)
```

可以按名字访问：

```python
heartbeat.uptime_ms
heartbeat.state
```

这比记住“数组第 0 个值是运行时间，第 1 个值是状态”更清楚。

### 6.6 `@property`

串口代码：

```python
@property
def is_open(self) -> bool:
    return self._file_descriptor is not None
```

调用时像读变量：

```python
if self._can.is_open:
```

而不是：

```python
if self._can.is_open():
```

### 6.7 `@staticmethod`

```python
@staticmethod
def _encode(frame: CanFrame) -> bytes:
```

这个函数属于 `SerialTransport` 的功能，但不需要读取任何 `self.xxx`，所以声明为静态方法。

## 7. 导入语句怎样看

```python
import math
import time
```

导入整个模块，使用时写：

```python
math.sin(...)
time.monotonic()
```

```python
from collections import deque
from dataclasses import dataclass
```

只导入指定名称，使用时直接写 `deque()`、`@dataclass`。

```python
from . import can_protocol as protocol
```

前面的点表示“当前 Python 包中的模块”。之后使用别名：

```python
protocol.make_motion_command(...)
protocol.ID_HEARTBEAT
```

```python
from .serial_transport import SerialTransport
```

表示从同一包的 `serial_transport.py` 导入 `SerialTransport` 类。

## 8. 程序真正从哪里开始

### 8.1 `setup.py` 建立命令入口

```python
entry_points={
    "console_scripts": [
        "chassis_can_node = chassis_can_control.chassis_can_node:main",
    ],
}
```

它告诉 ROS：运行名为 `chassis_can_node` 的程序时，调用：

```text
chassis_can_control/chassis_can_node.py 中的 main()
```

### 8.2 `main()`

```python
def main(args=None) -> None:
    rclpy.init(args=args)
    node = ChassisCanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
```

逐句理解：

```text
rclpy.init()             初始化 ROS 2 Python 环境
ChassisCanNode()         创建底盘节点对象
rclpy.spin(node)         进入事件循环，持续等消息/服务/定时器
KeyboardInterrupt        用户按 Ctrl+C 时进入
destroy_node()           退出前禁用电机并关闭接口
rclpy.shutdown()         关闭 ROS 2
```

最关键的认识：这个程序不是从上到下运行完就结束。它初始化后长期停在 `spin()` 中，等待不同事件触发回调函数。

## 9. 什么是“回调函数”

回调函数不是你在代码主线里直接调用的函数，而是先把函数交给 ROS，事件发生时由 ROS 调用。

### 9.1 话题回调

```python
self.create_subscription(
    Twist,
    "cmd_vel",
    self._on_cmd_vel,
    10,
)
```

可以读成：

```text
订阅 Twist 类型的 cmd_vel；每次收到消息，就调用 self._on_cmd_vel。
```

注意这里写的是：

```python
self._on_cmd_vel
```

没有括号，表示把函数本身交给 ROS，而不是现在就执行它。

### 9.2 定时器回调

```python
self.create_timer(0.02, self._send_motion_command)
```

表示每 0.02 秒由 ROS 调用一次 `_send_motion_command()`，也就是 50 Hz。

### 9.3 服务回调

```python
self.create_service(SetBool, "chassis/enable", self._on_enable)
```

当终端调用 `/chassis/enable` 时，ROS 调用 `_on_enable(request, response)`。

## 10. 主节点的初始化代码怎样拆开看

`ChassisCanNode.__init__()` 很长，但可以分成六组。

### 10.1 声明参数

```python
self.declare_parameter("transport", "can")
self.declare_parameter("serial_baud_rate", 115200)
self.declare_parameter("command_rate_hz", 50.0)
```

声明参数的名字和默认值。启动时 YAML 可以覆盖默认值。

### 10.2 根据参数创建传输对象

```python
transport = str(self.get_parameter("transport").value).lower()

if transport == "serial":
    self._can = SerialTransport(...)
elif transport == "can":
    self._can = SocketCanTransport(...)
else:
    raise ValueError(...)
```

当前 YAML 指定 `transport: serial`，所以实际创建 `SerialTransport`。

变量仍命名为 `_can` 是因为最早抽象为 CAN 逻辑帧；当前物理链路确实是串口。

### 10.3 建立内部状态

```python
self._drive_enabled = False
self._emergency_stop = False
self._latest_command = Twist()
self._last_heartbeat_time = 0.0
self._latest_heartbeat = None
```

这些变量跨回调长期保存：

```text
是否软件使能
是否急停
最近速度命令
最近心跳时间
最近解码出的心跳对象
```

### 10.4 创建发布者

```python
self._odom_publisher = self.create_publisher(Odometry, "odom", 20)
```

以后调用：

```python
self._odom_publisher.publish(message)
```

就能发布 `/odom`。

### 10.5 创建订阅者和服务

```python
self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
self.create_service(SetBool, "chassis/enable", self._on_enable)
```

### 10.6 创建定时器

| 周期 | 回调 | 功能 |
|---:|---|---|
| 20 ms | `_send_motion_command` | 给 STM32 发运动命令 |
| 5 ms | `_poll_can` | 读取并处理已有串口帧 |
| 50 ms | `_send_one_pending_parameter` | 逐项发送参数 |
| 200 ms | `_publish_diagnostics` | 发布诊断 |

## 11. 第一条主线：`/cmd_vel` 怎样发到 STM32

### 11.1 第一步：收到 ROS 消息

```python
def _on_cmd_vel(self, message: Twist) -> None:
    self._latest_command = message
    self._has_received_cmd_vel = True
    self._last_cmd_vel_time = time.monotonic()
```

这个函数不直接发串口，只保存：

```text
最近一条 Twist
已经收到过命令
收到时刻
```

为什么不直接发送？因为 `/cmd_vel` 来源可能忽快忽慢，而节点希望以固定 50 Hz 向 STM32发送。

### 11.2 第二步：20 ms 定时器检查安全条件

核心逻辑：

```python
now = time.monotonic()
timeout = float(self.get_parameter("ros_command_timeout_s").value)

command_is_fresh = (
    self._has_received_cmd_vel
    and now - self._last_cmd_vel_time <= timeout
)
```

`time.monotonic()` 是单调时钟，适合计算时间间隔。即使系统日期被校准，它也不会突然倒退。

然后判断：

```python
if command_is_fresh and self._drive_enabled and not self._emergency_stop:
    linear = float(self._latest_command.linear.x)
    angular = float(self._latest_command.angular.z)
    enable = True
else:
    linear = 0.0
    angular = 0.0
    enable = False
```

只有三项条件同时满足才下发真实速度，否则主动下发零速度和失能。

### 11.3 第三步：生成 8 字节逻辑帧

```python
frame = protocol.make_motion_command(
    linear,
    angular,
    enable,
    self._emergency_stop,
    self._next_sequence(),
)
```

这是跨模块调用：主节点不关心每个字节怎样排布，只把语义清楚的变量交给协议模块。

### 11.4 第四步：发送

```python
self._send(frame)
```

`_send()` 先确认接口已打开，再调用传输对象：

```python
self._can.send(frame)
```

因为当前 `_can` 实际指向 `SerialTransport` 对象，所以最后执行的是串口版 `send()`。

## 12. 协议模块怎样把速度变成字节

文件：

```text
can_protocol.py
```

### 12.1 运动命令函数

```python
def make_motion_command(
    linear_mps: float,
    angular_radps: float,
    enable: bool,
    emergency_stop: bool,
    sequence: int,
) -> CanFrame:
```

先检查浮点数不是无穷大或 NaN：

```python
if not math.isfinite(linear_mps) or not math.isfinite(angular_radps):
    raise ValueError(...)
```

再组成标志位：

```python
flags = (CMD_ENABLE if enable else 0) | \
        (CMD_ESTOP if emergency_stop else 0)
```

这里的 `A if 条件 else B` 是条件表达式；`|` 是按位或。

### 12.2 `struct.pack()`

```python
body = struct.pack(
    "<hhBBB",
    _clamp_i16(linear_mps * 1000.0),
    _clamp_i16(angular_radps * 1000.0),
    flags,
    1,
    sequence & 0xFF,
)
```

格式字符串解释：

| 字符 | 含义 |
|---|---|
| `<` | 小端序，低字节在前 |
| `h` | 16 位有符号整数 |
| `h` | 第二个 16 位有符号整数 |
| `B` | 8 位无符号整数 |
| `B` | 8 位无符号整数 |
| `B` | 8 位无符号整数 |

共生成 7 字节：

```text
线速度×1000       2字节
角速度×1000       2字节
标志位             1字节
协议版本           1字节
序号               1字节
```

最后追加 CRC：

```python
return CanFrame(
    ID_MOTION_COMMAND,
    body + bytes([crc8(body)]),
)
```

`bytes([crc])` 把一个 0～255 整数变成单字节；`+` 连接两段 `bytes`。

### 12.3 数值例子

假设：

```text
linear = 0.05 m/s
angular = 0 rad/s
enable = true
estop = false
sequence = 7
```

得到：

```text
0.05 × 1000 = 50 = 0x0032
小端排列为 32 00
角速度为 00 00
flags 为 01
version 为 01
sequence 为 07
最后计算 CRC
```

STM32 的 C 代码按同一规则取回数值，所以两端能互相理解。

## 13. 串口模块怎样发送 15 字节

文件：

```text
serial_transport.py
```

逻辑帧还不是最终串口帧。`_encode()` 再加一层：

```python
body = bytes([
    _VERSION,
    frame.can_id & 0xFF,
    (frame.can_id >> 8) & 0xFF,
    len(frame.data),
]) + frame.data

return _SOF + body + bytes([crc8(body)])
```

最终 15 字节：

| 位置 | 内容 |
|---:|---|
| 0～1 | `A5 5A` 帧头 |
| 2 | 外层版本 |
| 3～4 | logical ID，小端 |
| 5 | 数据长度 8 |
| 6～13 | 8 字节逻辑数据 |
| 14 | 外层 CRC |

`send()` 通过 `os.write()` 写 Linux 设备文件。一次 `write` 不保证写完所有字节，所以代码用 `while` 累加已发送长度，直到完整写完。

## 14. 第二条主线：STM32 数据怎样回到 ROS

### 14.1 5 ms 定时器轮询串口

```python
def _poll_can(self) -> None:
    if not self._open_can_if_needed():
        return

    frames = self._can.receive_available()
    for frame in frames:
        self._handle_frame(frame)
```

虽然函数名有 `can`，当前对象是串口传输。读法是：

```text
若接口没打开就退出；
取出当前已收到的所有完整帧；
逐帧交给 _handle_frame。
```

### 14.2 非阻塞读取

串口用：

```python
os.O_NONBLOCK
```

打开。没有数据时不会卡住整个 ROS 节点，而是抛出 `BlockingIOError`，代码捕获后结束本轮读取。

### 14.3 为什么要有接收缓存

一次读取可能得到：

- 半个帧；
- 一个完整帧；
- 连续几个帧；
- 从噪声或帧中间开始的数据。

因此先放进：

```python
self._receive_buffer = bytearray()
```

再反复寻找：

```python
header_index = self._receive_buffer.find(_SOF)
```

若不足 15 字节，保留缓存等下一次；若 CRC 错，只丢一个字节，再搜索下一处 `A5 5A`。这叫重新同步。

### 14.4 按 ID 分流

```python
def _handle_frame(self, frame: CanFrame) -> None:
    if frame.can_id == protocol.ID_WHEEL_SPEED:
        ...
    elif frame.can_id == protocol.ID_MOTOR_OUTPUT:
        ...
    elif frame.can_id == protocol.ID_LEFT_ENCODER:
        ...
```

`if / elif` 表示只有一个匹配分支会执行。

可以把 `_handle_frame()` 看成一个“邮件分拣中心”：根据 logical ID 把帧送到轮速、PWM、编码器、状态、参数应答或心跳处理流程。

## 15. 协议解码怎样看

轮速解码：

```python
left_target, left_actual, right_target, right_actual = \
    struct.unpack("<hhhh", frame.data)
```

`<hhhh` 表示把 8 字节按小端解析成四个 int16。

返回数据对象：

```python
return WheelSpeed(
    left_target / 1000.0,
    left_actual / 1000.0,
    right_target / 1000.0,
    right_actual / 1000.0,
)
```

STM32 为节约字节以 mm/s 传整数，Python 除以 1000 恢复 m/s。

编码器和状态解码还会先检查：

```python
if frame.can_id != ID_STATUS or not has_valid_crc(frame):
    return None
```

返回 `None` 表示这帧不是目标类型，或者 CRC 无效。调用者必须先判断解码结果不是 `None` 才使用。

## 16. `/motor/debug` 怎样发布

节点分别收到轮速帧和 PWM 帧。只有两者都存在才发布：

```python
if self._latest_speed is None or self._latest_output is None:
    return
```

然后建立 ROS 消息：

```python
message = Float32MultiArray()
message.data = [
    左目标速度,
    左实测速度,
    右目标速度,
    右实测速度,
    左PWM,
    右PWM,
    左速度误差,
    右速度误差,
]
self._debug_publisher.publish(message)
```

这段代码展示了 ROS Python 的典型步骤：

```text
创建消息对象 → 给字段赋值 → publisher.publish(message)
```

## 17. 编码器里程计模块怎样看

文件：

```text
wheel_odometry.py
```

它故意不依赖 ROS，只负责数学计算，所以比主节点更容易阅读。

### 17.1 初始化和复位

```python
def __init__(self) -> None:
    self.reset()
```

创建对象时直接调用 `reset()`，把 `x`、`y`、`yaw` 和轮角清零。

```python
self._previous_left_count: Optional[int] = None
```

表示刚开始还没有上一批编码器计数。

### 17.2 第一批数据只建立基准

```python
if self._previous_left_count is None:
    self._previous_left_count = left_count
    self._previous_right_count = right_count
    return self.sample()
```

STM32 可能已经运行很久，累计计数不为 0。RDK 节点刚启动时不能把 STM32 历史计数当作本次行驶距离。

### 17.3 计算位移

```python
meters_per_count = math.pi * wheel_diameter_m / counts_per_wheel_rev
left_distance = left_delta * meters_per_count
right_distance = right_delta * meters_per_count
```

### 17.4 计算车体中心和转角

```python
center_distance = 0.5 * (left_distance + right_distance)
yaw_delta = (right_distance - left_distance) / wheel_track_m
```

左右走得一样，`yaw_delta` 为 0；右轮走得更多，小车向左转，角度改变。

### 17.5 累计位置

```python
middle_yaw = self.yaw_rad + 0.5 * yaw_delta
self.x_m += center_distance * math.cos(middle_yaw)
self.y_m += center_distance * math.sin(middle_yaw)
```

`+=` 表示在原值基础上增加。

### 17.6 主节点发布 ROS 里程计

`WheelOdometry.update()` 返回 `OdometrySample`，主节点把它装入：

```text
nav_msgs/msg/Odometry
sensor_msgs/msg/JointState
TransformStamped
```

当前 `/odom` 来自编码器，不是 IMU 融合里程计。

## 18. 诊断回调怎样看

```python
heartbeat_age = time.monotonic() - self._last_heartbeat_time
```

如果心跳不存在或超过 2 秒：

```python
status.level = DiagnosticStatus.ERROR
status.message = "STM32 heartbeat missing"
```

否则读取 STM32 状态和故障位，组成：

```text
state
faults
battery_voltage
last_command_sequence
control_overruns
transport
interface
heartbeat_age
pending_parameters
parameter_transfer_failed
```

最后：

```python
array.status.append(status)
self._diagnostic_publisher.publish(array)
```

`append()` 是把一个元素追加到列表末尾。`DiagnosticArray.status` 本身是一个列表，所以要先加入 `status`。

## 19. 服务代码怎样看

### 19.1 使能服务

```python
def _on_enable(self, request, response):
    self._drive_enabled = bool(request.data)
    response.success = True
    response.message = (
        "drive enabled" if request.data else "drive disabled"
    )
    self._send_motion_command()
    return response
```

`request` 是调用者发来的请求，`response` 是节点要填写并返回的响应。

使能服务只改变 RDK 软件门控。真正运动还必须持续收到新鲜 `/cmd_vel`，而且 STM32 没有故障。

### 19.2 急停服务

急停请求为真时：

```python
self._emergency_stop = True
self._drive_enabled = False
```

然后立即发运动帧，STM32 收到急停位后锁存故障。

### 19.3 清故障服务

代码先把软件使能和急停请求清除，发送一帧普通失能命令，再发送带钥匙的系统命令。这样符合 STM32 的清故障前置条件。

## 20. 参数队列为什么看起来复杂

一次有约 30 个参数，不能发完就假设 STM32 全收到了。代码使用：

```python
self._parameter_queue = deque()
self._pending_parameter = None
```

`deque` 是双端队列，这里只按先进先出使用。

处理流程：

```text
从队头取一个参数
→ 发送并记录 ID、值、sequence、时间、次数
→ 暂停，等待 STM32 应答
→ ID 和 sequence 都匹配才确认
→ 成功后弹出队头
→ 发送下一个
```

若 200 ms 没应答则重发，最多 3 次。仍失败：

```python
self._parameter_transfer_failed = True
self._pending_parameter = None
self._parameter_queue.clear()
```

阅读这部分时，先把 `PendingParameter` 当作一张“等待签收的快递单”。

## 21. `try / except / finally` 怎样看

打开或操作串口可能失败：设备拔掉、权限不足、被占用等。

```python
try:
    self._can.open()
    return True
except OSError as error:
    self._log_can_error(...)
    return False
```

含义：

```text
尝试打开；
成功就返回 True；
若出现 OSError，就记录错误并返回 False。
```

`finally` 表示无论正常退出还是发生异常，都执行清理：

```python
finally:
    node.destroy_node()
```

这对电机控制很重要，退出时应尽最大可能发失能命令并关闭串口。

## 22. 几种容易看不懂的简写

### 22.1 条件表达式

```python
"enabled" if request.data else "disabled"
```

等价于完整的 `if/else` 赋值。

### 22.2 列表推导式

```python
names = [name for bit, name in FAULT_NAMES.items() if faults & bit]
```

可先展开理解：

```python
names = []
for bit, name in FAULT_NAMES.items():
    if faults & bit:
        names.append(name)
```

### 22.3 位运算

```python
sequence & 0xFF
```

只保留低 8 位，确保 sequence 可装进一个字节。

```python
faults & bit
```

检查某个故障位是否被置 1。

```python
frame.can_id >> 8
```

右移 8 位，用来取 ID 的高字节。

### 22.4 模运算

```python
self._sequence = (self._sequence + 1) & 0xFF
```

序号从 0 增到 255，下一次回到 0。

### 22.5 `del request`

```python
def _on_clear_faults(self, request, response):
    del request
```

`Trigger` 请求没有字段要使用。删除局部引用是为了明确表示参数故意不用，并减少静态检查警告。

## 23. launch 和 YAML 怎样配合

`chassis_serial.launch.py`：

```python
package_directory = get_package_share_directory("chassis_can_control")
parameter_file = os.path.join(
    package_directory,
    "config",
    "chassis_serial.yaml",
)
```

它寻找的是安装目录中的功能包，而不是直接写死源码绝对路径。

然后：

```python
Node(
    package="chassis_can_control",
    executable="chassis_can_node",
    name="chassis_can_node",
    parameters=[parameter_file],
)
```

所以修改源码中的 YAML 后需要重新构建，让它复制到 `install`：

```bash
cd ~/chassis_project/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select chassis_can_control
source install/setup.bash
```

## 24. 上下位机代码对应表

| Python 上位机 | STM32 下位机 | 对应关系 |
|---|---|---|
| `make_motion_command()` | `chassis_decode_motion_command()` | 上位机编码、下位机解码 |
| `make_parameter_command()` | `chassis_decode_parameter_command()` | 参数命令 |
| `make_system_command()` | `chassis_decode_system_command()` | 系统命令 |
| `decode_wheel_speed()` | `chassis_encode_wheel_speed()` | 下位机编码、上位机解码 |
| `decode_status()` | `chassis_encode_status()` | 状态和故障 |
| `decode_heartbeat()` | `chassis_encode_heartbeat()` | 心跳 |
| `SerialTransport` | `chassis_serial_transport.c` | 15 字节外层串口帧 |

阅读协议时应两边配对看：发送方怎样放字节，接收方就必须怎样取字节。

## 25. 当前 Python 代码没有做什么

为了避免产生错误理解，当前代码没有：

- 在 RDK 上执行电机 PI；PI 在 STM32；
- 直接控制 PWM；PWM 在 STM32；
- 读取 RDK 自己的 IMU；
- 解码 STM32 IMU 帧；当前协议没有 IMU 帧；
- 发布 `/imu/data`；
- 做编码器与 IMU 融合；
- 自动使能电机。

RDK Python 当前的准确定位是：

```text
ROS 2 接口 + 安全门控 + 协议编解码 + 串口传输
+ 状态诊断 + 参数管理 + 编码器里程计
```

## 26. 在 Ubuntu 终端边运行边理解代码

以下命令主要用于观察，不修改程序。

### 26.1 查看节点接口

```bash
source /opt/ros/humble/setup.bash
source ~/chassis_project/ros2_ws/install/setup.bash
ros2 node info /chassis_can_node
```

把输出与 `create_publisher`、`create_subscription`、`create_service` 对照。

### 26.2 观察接收链

```bash
ros2 topic echo /diagnostics
```

对应 `_publish_diagnostics()`。

```bash
ros2 topic echo /motor/debug
```

对应 `_publish_motor_debug()`。

```bash
ros2 topic echo /odom
```

对应 `_publish_odometry_and_joints()`。

### 26.3 查看发布频率

```bash
ros2 topic hz /motor/debug
ros2 topic hz /odom
```

可以把实际频率与 STM32 遥测周期、RDK 定时器周期联系起来。

### 26.4 查看参数

```bash
ros2 param list /chassis_can_node
ros2 param get /chassis_can_node transport
ros2 param get /chassis_can_node serial_port
```

把结果与 `declare_parameter()` 和 YAML 对照。

### 26.5 查看后台日志

```bash
tail -f ~/chassis_project/chassis_serial.log
```

按 `Ctrl+C` 只退出日志查看，不会停止后台节点。

## 27. 推荐的实际阅读顺序

### 第一阶段：理解入口，不看细节

1. `setup.py` 的 `entry_points`；
2. `chassis_serial.launch.py`；
3. `chassis_can_node.py` 最后的 `main()`；
4. `ChassisCanNode.__init__()` 中的发布、订阅、服务和定时器。

目标：知道程序怎样启动、有哪些事件入口。

### 第二阶段：只追命令下发

1. `_on_cmd_vel()`；
2. `_send_motion_command()`；
3. `make_motion_command()`；
4. `SerialTransport._encode()`；
5. `SerialTransport.send()`。

目标：能口述 `/cmd_vel` 如何变成 15 字节。

### 第三阶段：只追状态上传

1. `SerialTransport.receive_available()`；
2. `_poll_can()`；
3. `_handle_frame()`；
4. 各个 `decode_*()`；
5. `_publish_motor_debug()` 和 `_publish_diagnostics()`。

目标：能口述 STM32 字节如何变成 ROS 消息。

### 第四阶段：里程计

1. `WheelOdometry.reset()`；
2. `WheelOdometry.update()`；
3. `_update_odometry_if_pair_ready()`；
4. `_publish_odometry_and_joints()`。

### 第五阶段：服务和参数队列

最后再看 `_on_enable()`、急停、清故障和参数重试。它们依赖你已理解对象状态和回调。

## 28. 学完后做四个不危险的小练习

这些练习不需要使能电机。

### 练习一：指出对象成员

在 `_on_cmd_vel()` 中找出：

- 哪个是函数参数；
- 哪些是长期保存的 `self` 成员；
- 哪个函数取得当前单调时间。

### 练习二：手工编码速度

回答 `linear.x = 0.10 m/s` 在运动命令前两个字节是什么：

```text
0.10 × 1000 = 100 = 0x0064
小端字节：64 00
```

### 练习三：增加一条启动日志

在节点初始化最后附近增加：

```python
self.get_logger().info("Python learning test: node initialization complete")
```

重新构建并前台启动，观察日志。这个修改不改变电机控制。

### 练习四：改变诊断显示文字

把硬件名称显示文字改成更容易识别的名字，再重新构建，观察 `/diagnostics`。不要修改状态判断和故障等级。

## 29. 阅读时经常会问的十个问题

### 1. 为什么代码到处是 `self`？

因为 ROS 节点长期运行，多个回调要共享最近命令、心跳、串口对象和发布者。

### 2. 为什么 `_on_cmd_vel()` 不直接发串口？

它只记录最新输入；固定 50 Hz 定时器统一完成安全检查和下发。

### 3. 为什么有两个 CRC？

8 字节逻辑协议和 15 字节串口外层各自校验，便于同一逻辑协议复用到 CAN 与串口。

### 4. 为什么文件名还有 `can`？

它表示逻辑帧最初按 CAN ID + 8 字节设计。当前外层物理传输是串口。

### 5. 为什么要用 `None`？

它明确表示“尚未收到该类数据”，避免拿默认的全零对象误认为真实数据。

### 6. 为什么回调函数没有被直接调用？

它们先注册给 ROS，由消息、服务请求或定时器事件触发。

### 7. 为什么 Python 能控制 STM32？

不是因为两者语言相同，而是因为两端严格遵守相同字节协议。

### 8. 为什么修改 YAML 后要重新构建？

launch 读取安装目录的参数副本，不是直接读取源码目录。

### 9. 为什么服务返回成功但车不动？

使能只是打开一层软件门，还需要持续 `/cmd_vel`、正常心跳、硬件使能和 STM32 无故障。

### 10. 为什么两个节点会导致心跳丢失？

两个进程同时从同一串口读取，各自只拿到部分字节，无法稳定拼出完整帧。

## 30. 最终应该掌握的代码主线

当你能够不看文档说出下面两条链，说明已经真正开始看懂项目。

命令链：

```text
main
→ ChassisCanNode.__init__
→ _on_cmd_vel
→ _send_motion_command
→ protocol.make_motion_command
→ SerialTransport.send
→ STM32
```

反馈链：

```text
STM32
→ SerialTransport.receive_available
→ _poll_can
→ _handle_frame
→ protocol.decode_*
→ _publish_diagnostics / _publish_motor_debug / WheelOdometry
→ ROS 话题
```

学习这个项目时，不需要一开始就理解所有 700 多行主节点代码。先能沿着这两条主线走通，再逐步加入服务、参数和里程计，代码就会从“一大团 Python”变成几个相互连接、职责明确的小模块。

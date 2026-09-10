# 上位机 Python 协议编解码模块零基础超详细讲解

## 1. 这份文档讲的是哪个文件

本文专门讲解 RDK X5 ROS 2 功能包中的：

```text
ros2_ws/src/chassis_can_control/
└── chassis_can_control/
    └── can_protocol.py
```

RDK X5 上对应位置是：

```text
/home/wheeltec/chassis_project/ros2_ws/src/chassis_can_control/
└── chassis_can_control/can_protocol.py
```

这个文件就是上位机的“Python 协议编解码模块”。

它不负责：

- 直接订阅 ROS 2 `/cmd_vel`；
- 直接打开 Linux 串口；
- 直接控制电机；
- 计算电机 PI；
- 计算 ROS 里程计。

它只负责一件核心工作：

```text
在“有明确意义的 Python 数据”和“固定排列的 8 个二进制字节”之间转换。
```

例如，上层节点理解的是：

```text
线速度 = 0.05 m/s
角速度 = 0 rad/s
电机允许使能 = 是
急停 = 否
命令序号 = 7
```

但 STM32 串口最终只能收到一个个字节。协议模块会把这些含义编码为：

```text
logical ID = 0x101
data[8]    = 32 00 00 00 01 01 07 A0
```

反方向也是一样。STM32 上传：

```text
01 00 00 68 29 35 00 FB
```

协议模块可以将其解码为：

```text
状态 = IDLE
故障 = NONE
电池电压 = 10600 mV
最后命令序号 = 53
控制超限次数 = 0
```

## 2. 这个模块在整个项目中的位置

先把它放回完整数据链路中：

```text
ROS 2 其他节点
发布 /cmd_vel
       ↓
chassis_can_node.py
提取 linear.x、angular.z、软件使能和急停状态
       ↓
can_protocol.py                 ← 本文讲解的模块
将有意义的数据编码成 logical ID + data[8]
       ↓
serial_transport.py
再包装成 A5 5A 开头的 15 字节串口帧
       ↓
Linux USB 串口 0002
       ↓
STM32 chassis_serial_transport.c
拆除 15 字节串口外壳
       ↓
STM32 chassis_can_protocol.c
按照同一规则解码 data[8]
       ↓
STM32 电机控制和安全状态机
```

STM32 上传数据时，方向完全相反：

```text
STM32 状态、轮速、编码器
       ↓
chassis_can_protocol.c 编码成 logical ID + data[8]
       ↓
chassis_serial_transport.c 包成 15 字节
       ↓
USB 串口
       ↓
serial_transport.py 拆出 logical ID + data[8]
       ↓
can_protocol.py 解码为 Python 数据对象
       ↓
chassis_can_node.py 发布 /diagnostics、/motor/debug、/odom
```

因此，这个模块位于 ROS 含义和二进制通信之间。

## 3. “协议”到底是什么

### 3.1 人与人交流也需要协议

如果一个人说数字 `50`，另一个人不知道它表示什么：

- 50 米？
- 50 m/s？
- 50 PWM？
- 50 mV？
- 50 是状态码？

双方必须提前约定：

```text
第 0～1 字节表示线速度，单位为 mm/s，小端、有符号整数。
```

这样收到整数 `50` 时，STM32 才知道：

```text
50 mm/s = 0.05 m/s
```

### 3.2 通信协议需要约定什么

本项目至少约定了：

1. 每种报文的 logical ID；
2. 每个字段位于第几个字节；
3. 每个字段占几个字节；
4. 是有符号还是无符号；
5. 使用小端还是大端；
6. 使用整数还是 IEEE 754 浮点数；
7. 物理单位和缩放比例；
8. 哪些位表示使能和急停；
9. CRC 怎样计算；
10. 请求和应答怎样通过序号匹配。

只要 Python 和 STM32 任何一项理解不一致，双方就不能正确交流。

### 3.3 编码与解码

编码可以理解为“打包”：

```text
0.05 m/s、已使能 → 8 个字节
```

解码可以理解为“拆包”：

```text
8 个字节 → 电压 10.600 V、状态 IDLE
```

在计算机领域，这个过程也叫序列化和反序列化。

## 4. 阅读该文件需要的 Python 基础

### 4.1 `import` 是什么

文件开头：

```python
from dataclasses import dataclass
import math
import struct
from typing import Optional
```

Python 将常用功能分在不同模块中。`import` 表示把模块提供的功能引入当前文件。

这里分别用于：

| 导入内容 | 作用 |
|---|---|
| `dataclass` | 快速定义“只负责保存字段”的数据类 |
| `math` | 使用 `math.isfinite()` 检查数值是否正常 |
| `struct` | 在 Python 数值和二进制字节之间转换 |
| `Optional` | 类型提示：返回值可能是对象，也可能是 `None` |

### 4.2 Python 变量和常量

```python
CAN_DLC = 8
ID_MOTION_COMMAND = 0x101
```

Python 没有 C 语言那种严格的 `const` 关键字。全部大写是一种约定，提醒程序员这些值是常量，不应在运行中随意修改。

### 4.3 函数定义

```python
def crc8(data: bytes) -> int:
```

逐部分理解：

```text
def       定义函数
crc8      函数名称
data      传入参数名称
: bytes   类型提示，希望 data 是字节串
-> int    类型提示，返回一个整数
:         后面缩进内容属于函数体
```

类型提示主要帮助人阅读和编辑器检查，Python 默认不会像 C 编译器那样强制执行所有类型提示。

### 4.4 `return`

```python
return value
```

表示函数执行结束，并把 `value` 交回调用者。

例如：

```python
result = crc8(b"123456789")
```

`result` 会得到 `0xF4`，十进制为 244。

### 4.5 `if`、`elif` 和 `else`

```python
if condition1:
    ...
elif condition2:
    ...
else:
    ...
```

依次判断条件。Python 使用缩进表示条件内部的代码，不使用 C 的大括号。

### 4.6 `for` 循环

```python
for byte in data:
```

如果 `data` 是：

```python
b"ABC"
```

循环三次，`byte` 依次为：

```text
65、66、67
```

Python 遍历 `bytes` 时，每个元素直接是 0～255 的整数。

### 4.7 类和对象

```python
class CanFrame:
    ...
```

类是一种数据结构的设计图。创建对象：

```python
frame = CanFrame(0x101, b"12345678")
```

之后可以访问：

```python
frame.can_id
frame.data
```

### 4.8 `None`

`None` 表示没有有效对象或没有结果。

例如：

```python
def decode_status(frame):
    if frame.can_id != ID_STATUS:
        return None
```

意思是：这不是状态帧，我无法给出 `StatusData`。

调用者必须先判断：

```python
decoded = decode_status(frame)
if decoded is not None:
    print(decoded.battery_mv)
```

### 4.9 字典

```python
STATE_NAMES = {
    0: "BOOT",
    1: "IDLE",
    2: "RUNNING",
}
```

字典保存“键 → 值”的映射。例如：

```python
STATE_NAMES[1]
```

得到：

```text
IDLE
```

### 4.10 元组

```python
(0.0, 50000.0)
```

这是一个有两个元素的元组。项目用它表示参数允许范围：

```text
最小值、最大值
```

可以拆分：

```python
minimum, maximum = PARAMETER_RANGES["controller.left_kp"]
```

### 4.11 列表推导式

文件最后：

```python
names = [name for bit, name in FAULT_NAMES.items() if faults & bit]
```

这是简写。它等价于：

```python
names = []
for bit, name in FAULT_NAMES.items():
    if faults & bit:
        names.append(name)
```

初学时先看展开版本更容易。

## 5. 字节、二进制和十六进制基础

### 5.1 一个字节是什么

1 个字节等于 8 个二进制位：

```text
00000000 ～ 11111111
```

如果按无符号整数理解，它可以表示：

```text
0 ～ 255
```

### 5.2 为什么经常使用十六进制

4 个二进制位恰好对应 1 个十六进制数字：

```text
二进制 1010 = 十六进制 A
二进制 1111 = 十六进制 F
```

一个字节可以方便地写成两个十六进制数字：

```text
11100101 = E5
```

Python 中十六进制整数以 `0x` 开头：

```python
0x101
0xA5
0xFF
```

它们仍然是普通整数，只是书写形式不同：

```text
0xA5 = 十进制 165
0xFF = 十进制 255
```

### 5.3 Python 的 `bytes`

```python
data = bytes([0x32, 0x00, 0x01])
```

这是一个长度为 3 的不可变字节串。

显示为十六进制：

```python
print(data.hex(" "))
```

输出：

```text
32 00 01
```

取单个字节：

```python
data[0]
```

得到整数 `50`，因为 `0x32 = 50`。

切片：

```python
data[:2]
```

得到前两个字节。切片左边包含、右边不包含，因此 `[:7]` 表示下标 0～6，共 7 个字节。

### 5.4 `bytes([crc])`

假设：

```python
crc = 0xA0
```

下面代码生成一个长度为 1 的字节串：

```python
bytes([crc])
```

结果是：

```text
A0
```

然后可以拼接：

```python
full_data = body + bytes([crc])
```

### 5.5 小端序是什么

如果整数 `0x1234` 占两个字节：

```text
高字节 = 0x12
低字节 = 0x34
```

小端序把低字节放前面：

```text
34 12
```

大端序把高字节放前面：

```text
12 34
```

本项目多字节字段统一使用小端序。

例如十进制 50：

```text
十六进制 = 0x0032
小端两字节 = 32 00
```

例如电池电压 10600 mV：

```text
十六进制 = 0x2968
小端两字节 = 68 29
```

### 5.6 有符号整数与补码

16 位有符号整数范围：

```text
-32768 ～ 32767
```

正数 50：

```text
0x0032 → 小端 32 00
```

负数使用补码。例如 -456 的 16 位补码为：

```text
0xFE38 → 小端 38 FE
```

Python 的 `struct` 会自动完成补码转换，不需要自己手算。

### 5.7 位与位掩码

一个字节有 8 个 bit：

```text
bit7 bit6 bit5 bit4 bit3 bit2 bit1 bit0
```

项目约定：

```python
CMD_ENABLE = 0x01  # 00000001，bit0
CMD_ESTOP  = 0x02  # 00000010，bit1
```

按位或 `|` 可以同时设置多个位：

```text
00000001
00000010
-------- OR
00000011
```

所以使能和急停都为真时，flags 为 `0x03`。

按位与 `&` 用于检查某个位：

```python
if faults & 0x0020:
    print("左轮堵转")
```

## 6. 文件开头的协议常量

### 6.1 `CAN_DLC = 8`

```python
CAN_DLC = 8
```

`DLC` 可以理解为数据长度。本项目的逻辑报文数据区固定为 8 字节。

注意：这里说的是逻辑帧的数据区，不是完整串口帧。完整串口帧为 15 字节。

### 6.2 logical ID

```python
ID_MOTION_COMMAND = 0x101
ID_PARAMETER_COMMAND = 0x102
ID_SYSTEM_COMMAND = 0x103

ID_WHEEL_SPEED = 0x181
ID_MOTOR_OUTPUT = 0x182
ID_LEFT_ENCODER = 0x183
ID_RIGHT_ENCODER = 0x184
ID_STATUS = 0x185
ID_PARAMETER_REPLY = 0x186
ID_HEARTBEAT = 0x700
```

可以把 ID 理解为报文类型编号。

| ID | 方向 | 含义 |
|---:|---|---|
| `0x101` | RDK→STM32 | 运动命令 |
| `0x102` | RDK→STM32 | 参数读写 |
| `0x103` | RDK→STM32 | 系统命令 |
| `0x181` | STM32→RDK | 目标/实测轮速 |
| `0x182` | STM32→RDK | PWM 和速度误差 |
| `0x183` | STM32→RDK | 左编码器 |
| `0x184` | STM32→RDK | 右编码器 |
| `0x185` | STM32→RDK | 状态、故障和电池 |
| `0x186` | STM32→RDK | 参数或系统命令应答 |
| `0x700` | STM32→RDK | 心跳 |

当主节点收到帧时，先检查 ID 再选择解码函数。

### 6.3 为什么叫 `CAN`，现在却走串口

逻辑帧最初按照标准 CAN 帧设计：

```text
11 位 ID + 8 字节 data
```

当前没有使用 CAN 物理线，而是在外层添加串口封装。因此：

```text
逻辑层仍然使用 CanFrame
物理传输层当前使用 USB 串口
```

名字保留为 CAN 不代表现在正在使用 CAN 接口。

### 6.4 系统命令编号

```python
SYSTEM_CLEAR_FAULT = 1
SYSTEM_SAVE_PARAMETERS = 2
SYSTEM_RESET_ODOMETRY = 3
```

相同 `0x103` ID 下，通过 opcode 区分三种系统操作。

### 6.5 系统命令钥匙

```python
SYSTEM_KEY0 = 0xA5
SYSTEM_KEY1 = 0x5A
```

清故障、保存 Flash 等系统命令必须包含两个固定钥匙字节。它用于减少随机坏数据被误认为危险系统命令的概率。

这里的 `A5 5A` 和串口帧头恰好相同，但位置和作用不同：

- 串口帧最外面的 `A5 5A`：用于寻找一帧从哪里开始；
- 系统命令 data 内的 `A5 5A`：用于确认系统命令格式正确。

不要把这两组混为一谈。

## 7. 状态、故障和参数字典

### 7.1 `STATE_NAMES`

STM32 为节省带宽只上传一个状态数字，例如 `1`。Python 使用字典转成人能读懂的文字：

```python
STATE_NAMES = {
    0: "BOOT",
    1: "IDLE",
    2: "RUNNING",
    3: "TIMEOUT",
    4: "FAULT",
    5: "ESTOP",
}
```

主节点中：

```python
state_name = protocol.STATE_NAMES.get(status.state, "UNKNOWN")
```

`.get()` 在键不存在时可以提供默认值，避免直接报错。

### 7.2 `FAULT_NAMES`

故障不是“一次只能有一个”。16 位故障字段的每一位表示一种故障：

```text
0x0001 COMMAND_TIMEOUT
0x0002 ESTOP
0x0004 LOW_BATTERY
0x0008 LEFT_ENCODER
0x0010 RIGHT_ENCODER
0x0020 LEFT_STALL
...
```

例如：

```text
COMMAND_TIMEOUT = 0x0001
LEFT_STALL      = 0x0020
两个同时存在  = 0x0021
```

`fault_text(0x0021)` 会得到：

```text
COMMAND_TIMEOUT|LEFT_STALL
```

### 7.3 `PARAMETER_IDS`

ROS 参数名是字符串：

```text
controller.left_kp
geometry.wheel_diameter_m
```

但通信中传完整字符串太浪费带宽，所以映射为一个字节 ID：

```python
"controller.left_kp": 1
"geometry.wheel_diameter_m": 17
```

主节点下发所有参数时：

```python
for name, parameter_id in protocol.PARAMETER_IDS.items():
    value = self.get_parameter(name).value
```

于是它能将 ROS 参数名、参数值和 STM32 参数编号连接起来。

### 7.4 `PARAMETER_RANGES`

```python
"controller.left_kp": (0.0, 50000.0)
```

主节点在下发前先检查范围：

```python
minimum, maximum = protocol.PARAMETER_RANGES[parameter.name]
```

这叫上位机预检查。STM32 仍然会再次检查，形成两层保护：

```text
RDK 检查：尽早发现用户输入错误
STM32 检查：不信任外部数据，保护最终控制器
```

## 8. `dataclass` 数据类

### 8.1 `CanFrame`

```python
@dataclass
class CanFrame:
    can_id: int
    data: bytes
```

`@dataclass` 会自动生成初始化函数，因此可以直接写：

```python
frame = CanFrame(0x101, b"12345678")
```

不用手写：

```python
def __init__(self, can_id, data):
    self.can_id = can_id
    self.data = data
```

### 8.2 `__post_init__`

```python
def __post_init__(self) -> None:
    if not 0 <= self.can_id <= 0x7FF:
        raise ValueError(...)
    if len(self.data) != CAN_DLC:
        raise ValueError(...)
```

dataclass 自动初始化完成后，会调用 `__post_init__()` 做额外验证。

本项目只接受标准 11 位 ID：

```text
0x000 ～ 0x7FF
```

数据必须恰好 8 字节。

如果创建错误对象：

```python
CanFrame(0x101, b"abc")
```

会立即抛出 `ValueError`，而不是让错误数据继续流入串口。

### 8.3 为什么不直接使用字典

当然可以写：

```python
{"can_id": 0x101, "data": b"12345678"}
```

但数据类有这些优点：

- 字段名固定；
- 编辑器能提示；
- 创建时可以验证；
- 打印内容清楚；
- 两个对象容易比较，便于测试。

### 8.4 其他解码结果类

例如：

```python
@dataclass
class StatusData:
    state: int
    faults: int
    battery_mv: int
    last_command_sequence: int
    overrun_count: int
```

解码后不是返回一串难记顺序的数字，而是：

```python
status.state
status.battery_mv
status.overrun_count
```

其他数据类：

| 类 | 保存内容 |
|---|---|
| `WheelSpeed` | 左右目标与实测轮速 |
| `MotorOutput` | 左右 PWM 与误差 |
| `EncoderData` | 累计计数、周期增量和序号 |
| `StatusData` | 状态、故障、电池和诊断计数 |
| `ParameterReply` | 参数 ID、回应状态、值和序号 |
| `Heartbeat` | STM32 运行时间、状态和固件版本 |

## 9. CRC-8 校验详细讲解

### 9.1 CRC 解决什么问题

串口传输可能受到噪声影响。例如原始字节是：

```text
32 00 00 00 01 01 07
```

如果其中一个 bit 翻转，接收方可能得到：

```text
32 00 00 00 01 01 03
```

如果不校验，序号或其他字段就会被错误接受。

发送方根据数据计算 CRC 并附在最后。接收方重新计算，如果不相等就丢弃。

CRC 不是加密，不能防止恶意伪造，只用于检测传输错误。

### 9.2 当前 CRC 参数

```text
名称：CRC-8/ATM
多项式：0x07
初始值：0x00
不反射
最终不异或
```

### 9.3 函数逐行解释

```python
def crc8(data: bytes) -> int:
    value = 0
```

CRC 寄存器从 0 开始。

```python
    for byte in data:
        value ^= byte
```

依次处理每个输入字节。`^` 是按位异或。

```python
        for _ in range(8):
```

一个字节有 8 个 bit，所以每个字节继续循环 8 次。变量名 `_` 表示循环次数本身不需要使用。

```python
            if value & 0x80:
```

检查当前最高位 bit7 是否为 1。

```python
                value = ((value << 1) ^ 0x07) & 0xFF
```

最高位为 1 时，左移后与多项式 `0x07` 异或。

```python
            else:
                value = (value << 1) & 0xFF
```

最高位为 0 时只左移。

`& 0xFF` 将结果限制在 8 位。Python 整数不会自动溢出，如果不限制，它会不断变大；STM32 的 `uint8_t` 则天然只保留 8 位。

### 9.4 已知测试向量

协议测试使用通用验证字符串：

```python
crc8(b"123456789") == 0xF4
```

这个已知结果用来确认算法参数没有写错。

### 9.5 检查内部 CRC

```python
def has_valid_crc(frame: CanFrame) -> bool:
    return crc8(frame.data[:7]) == frame.data[7]
```

含义：

```text
重新计算 data[0]～data[6] 的 CRC
与 data[7] 中收到的 CRC 比较
```

相同返回 `True`，不同返回 `False`。

## 10. `struct` 模块详细讲解

### 10.1 为什么需要 `struct`

Python 整数和浮点数是高级对象，不能直接把“对象本身”写入串口。必须明确转换成固定数量的字节。

```python
struct.pack(format, values...)
```

将 Python 数值打包为 `bytes`。

```python
struct.unpack(format, data)
```

将 `bytes` 解包为 Python 数值元组。

### 10.2 本项目使用的格式字符

| 字符 | 含义 | 字节数 |
|---|---|---:|
| `<` | 后续多字节字段使用小端序 | 0 |
| `B` | 8 位无符号整数 | 1 |
| `h` | 16 位有符号整数 | 2 |
| `H` | 16 位无符号整数 | 2 |
| `i` | 32 位有符号整数 | 4 |
| `I` | 32 位无符号整数 | 4 |
| `f` | IEEE 754 单精度浮点数 | 4 |

### 10.3 格式长度必须匹配

例如：

```python
struct.pack("<hhBBB", ...)
```

长度为：

```text
h 2 字节
h 2 字节
B 1 字节
B 1 字节
B 1 字节
总计 7 字节
```

再添加一个 CRC，正好 8 字节。

### 10.4 打包和解包必须完全对称

Python 发送：

```python
struct.pack("<hhBBB", ...)
```

STM32 必须按同样结构读取：

```text
int16、int16、uint8、uint8、uint8、CRC
```

如果一端认为第一个字段是 4 字节整数，另一端认为是 2 字节，后面所有字段都会错位。

## 11. `_clamp_i16()` 为什么需要限幅

```python
def _clamp_i16(value: float) -> int:
    return max(-32768, min(32767, int(round(value))))
```

从内往外理解：

```python
round(value)
```

四舍五入。

```python
int(...)
```

转换为整数。

```python
min(32767, result)
```

不允许超过 32767。

```python
max(-32768, result)
```

不允许低于 -32768。

因为格式 `h` 只能表示 int16 范围。如果把更大数直接交给 `struct.pack()`，会抛出异常。

函数名前的单下划线表示它主要供当前模块内部使用。

## 12. 运动命令编码 `make_motion_command()`

这是 ROS 速度命令连接 STM32 的关键函数。

### 12.1 函数输入

```python
def make_motion_command(
    linear_mps: float,
    angular_radps: float,
    enable: bool,
    emergency_stop: bool,
    sequence: int,
) -> CanFrame:
```

| 参数 | 含义 |
|---|---|
| `linear_mps` | 车体线速度，m/s |
| `angular_radps` | 车体角速度，rad/s |
| `enable` | 软件是否允许驱动 |
| `emergency_stop` | 是否请求急停 |
| `sequence` | 0～255 循环序号 |

### 12.2 检查 NaN 和无穷大

```python
if not math.isfinite(linear_mps) or not math.isfinite(angular_radps):
    raise ValueError(...)
```

`math.isfinite()` 只有普通有限数值才返回 `True`。下面这些会被拒绝：

```text
NaN
+∞
-∞
```

原因是异常浮点数不能成为安全电机命令。

### 12.3 生成 flags

```python
flags = (CMD_ENABLE if enable else 0) | \
        (CMD_ESTOP if emergency_stop else 0)
```

条件表达式：

```python
CMD_ENABLE if enable else 0
```

等价于：

```python
if enable:
    first = CMD_ENABLE
else:
    first = 0
```

两部分再用按位或合并。

结果：

| enable | estop | flags |
|---|---|---:|
| False | False | `0x00` |
| True | False | `0x01` |
| False | True | `0x02` |
| True | True | `0x03` |

正常逻辑不会主动同时下发使能和急停，但协议位本身允许表达。

### 12.4 单位缩放

```python
_clamp_i16(linear_mps * 1000.0)
_clamp_i16(angular_radps * 1000.0)
```

线速度从 m/s 转为 mm/s 整数：

```text
0.05 m/s × 1000 = 50 mm/s
```

角速度从 rad/s 转为 mrad/s：

```text
0.15 rad/s × 1000 = 150 mrad/s
```

接收端再除以 1000 恢复 SI 单位。

这样可以用 2 字节有符号整数表达速度，分辨率为：

```text
0.001 m/s
0.001 rad/s
```

### 12.5 `sequence & 0xFF`

```python
sequence & 0xFF
```

只保留最低 8 位，保证放得进 `B` 字段。例如：

```text
sequence = 256 = 0x100
0x100 & 0xFF = 0
```

所以序号在 255 后回到 0。

### 12.6 data[8] 布局

```python
body = struct.pack(
    "<hhBBB",
    linear,
    angular,
    flags,
    1,
    sequence,
)
```

| data 下标 | 字节数 | 含义 |
|---:|---:|---|
| 0～1 | 2 | 线速度 ×1000，int16，小端 |
| 2～3 | 2 | 角速度 ×1000，int16，小端 |
| 4 | 1 | flags：bit0 使能、bit1 急停 |
| 5 | 1 | 协议版本，当前为 1 |
| 6 | 1 | sequence |
| 7 | 1 | 前 7 字节 CRC-8 |

### 12.7 返回 `CanFrame`

```python
return CanFrame(
    ID_MOTION_COMMAND,
    body + bytes([crc8(body)]),
)
```

返回：

```text
ID = 0x101
data = 7 字节 body + 1 字节 CRC
```

此时还没有 `A5 5A` 串口帧头。串口模块下一步才添加。

## 13. 完整手算：0.05 m/s 前进命令

调用：

```python
frame = make_motion_command(
    linear_mps=0.05,
    angular_radps=0.0,
    enable=True,
    emergency_stop=False,
    sequence=7,
)
```

### 第一步：线速度

```text
0.05 × 1000 = 50
50 = 0x0032
小端 = 32 00
```

### 第二步：角速度

```text
0 × 1000 = 0
0 = 0x0000
小端 = 00 00
```

### 第三步：flags

```text
enable=True  → bit0=1
estop=False  → bit1=0
flags = 00000001 = 01
```

### 第四步：版本和序号

```text
version = 01
sequence = 07
```

### 第五步：前 7 字节

```text
32 00 00 00 01 01 07
```

### 第六步：内部 CRC

CRC-8/ATM 结果：

```text
A0
```

### 最终逻辑帧

```text
ID      = 0x101
data[8] = 32 00 00 00 01 01 07 A0
```

### 串口模块继续包装

`serial_transport.py` 添加：

```text
A5 5A                         外层帧头
01                            外层版本
01 01                         logical ID 0x101，小端
08                            data 长度
32 00 00 00 01 01 07 A0       逻辑 data[8]
EB                            外层 CRC
```

完整 15 字节：

```text
A5 5A 01 01 01 08 32 00 00 00 01 01 07 A0 EB
```

这才是 Linux 串口实际写出的字节。

## 14. 停止与急停命令的区别

### 14.1 普通失能停止

```python
make_motion_command(0.0, 0.0, False, False, 8)
```

逻辑 data：

```text
00 00 00 00 00 01 08 2D
```

含义：

```text
速度为零
enable=0
estop=0
```

STM32 进入空闲，不锁存急停。

### 14.2 急停

```python
make_motion_command(0.0, 0.0, False, True, 9)
```

逻辑 data：

```text
00 00 00 00 02 01 09 FC
```

flags 为 `0x02`。STM32 收到后锁存 ESTOP，后续普通速度命令不能自动解除，必须执行清故障流程。

因此“速度写零”“软件失能”“急停”不是同一含义。

## 15. 参数命令编码 `make_parameter_command()`

函数：

```python
def make_parameter_command(
    parameter_id: int,
    operation: int,
    value: float,
    sequence: int,
) -> CanFrame:
```

### 15.1 布局

格式：

```python
"<BBfB"
```

| data 下标 | 含义 |
|---:|---|
| 0 | 参数 ID |
| 1 | 操作：0 读、1 写 |
| 2～5 | float32 参数值，小端 |
| 6 | sequence |
| 7 | CRC-8 |

### 15.2 为什么参数使用 float32

控制参数既可能是整数形式的 3000，也可能是：

```text
0.35
0.162
636.56
```

使用 IEEE 754 单精度浮点数能统一表示。

STM32F407 和 Python 都能处理 IEEE 754 float32。Python 自身 `float` 通常是 64 位双精度，但 `struct.pack("f", value)` 会明确压缩为 32 位。

这可能产生微小精度误差，例如某些小数解包后并非数学上的完全精确值。参数比较应允许很小误差。

### 15.3 示例：写左轮 Kp=3000

```python
make_parameter_command(
    parameter_id=1,
    operation=PARAM_WRITE,
    value=3000.0,
    sequence=10,
)
```

逻辑 data：

```text
01 01 00 80 3B 45 0A E6
```

解释：

```text
01             参数 ID 1 = controller.left_kp
01             写操作
00 80 3B 45    float32 小端表示 3000.0
0A             sequence 10
E6             CRC
```

### 15.4 它和 ROS 参数模块怎样连接

主节点收到 `ros2 param set` 后：

1. 在 `PARAMETER_IDS` 找到 ID；
2. 在 `PARAMETER_RANGES` 检查范围；
3. 将 `(parameter_id, value)` 放入参数队列；
4. 定时器调用 `make_parameter_command()`；
5. 串口发送；
6. STM32 修改 RAM 参数并发送 `0x186` 应答；
7. Python 用 `decode_parameter_reply()` 检查 ID 和 sequence；
8. 成功后才处理下一个参数。

## 16. 系统命令编码 `make_system_command()`

```python
def make_system_command(opcode: int, sequence: int) -> CanFrame:
```

格式：

```python
"<BBBBBBB"
```

7 个字段都是 1 字节无符号整数。

| data 下标 | 含义 |
|---:|---|
| 0 | opcode |
| 1 | 固定钥匙 `0xA5` |
| 2 | 固定钥匙 `0x5A` |
| 3 | sequence |
| 4～6 | 保留参数，当前为 0 |
| 7 | CRC |

清故障示例：

```python
make_system_command(SYSTEM_CLEAR_FAULT, 11)
```

逻辑 data：

```text
01 A5 5A 0B 00 00 00 28
```

它由这些 ROS 服务间接调用：

```text
/chassis/clear_faults
/chassis/save_parameters
/chassis/reset_odometry
```

## 17. 解码函数为什么返回 `Optional`

例如：

```python
def decode_status(frame: CanFrame) -> Optional[StatusData]:
```

意思是返回值有两种可能：

```text
合法状态帧 → StatusData 对象
ID 不对或 CRC 错 → None
```

这种设计让调用者能明确判断数据是否有效。

但要注意，不同解码函数的校验方式并不完全相同，后面会逐个说明。

## 18. 轮速解码 `decode_wheel_speed()`

```python
left_target, left_actual, right_target, right_actual = \
    struct.unpack("<hhhh", frame.data)
```

格式 `<hhhh` 表示 4 个小端 int16，恰好用满 8 字节。

| 字节 | 含义 |
|---:|---|
| 0～1 | 左轮斜坡目标速度 ×1000 |
| 2～3 | 左轮滤波实测速度 ×1000 |
| 4～5 | 右轮斜坡目标速度 ×1000 |
| 6～7 | 右轮滤波实测速度 ×1000 |

示例数据：

```text
32 00 2F 00 32 00 31 00
```

解开整数：

```text
50、47、50、49
```

除以 1000 后：

```python
WheelSpeed(
    left_target_mps=0.050,
    left_measured_mps=0.047,
    right_target_mps=0.050,
    right_measured_mps=0.049,
)
```

主节点将它与电机输出组合后发布 `/motor/debug`。

### 18.1 为什么这帧没有内部 CRC

4 个 int16 已经占满 8 字节，没有第 8 字节留给内部 CRC。

当前串口版依靠 15 字节外层帧的 CRC 检查整帧，因此仍受保护。将来使用真正 CAN 总线时，CAN 控制器本身也有帧级 CRC 和错误检测。

## 19. 电机输出解码 `decode_motor_output()`

同样使用：

```python
"<hhhh"
```

| 字节 | 含义 |
|---:|---|
| 0～1 | 左 PWM，int16 |
| 2～3 | 右 PWM，int16 |
| 4～5 | 左速度误差 ×1000 |
| 6～7 | 右速度误差 ×1000 |

解码后：

```python
MotorOutput(
    left_pwm=...,
    right_pwm=...,
    left_error_mps=...,
    right_error_mps=...,
)
```

PWM 不除以 1000，因为它本身就是 STM32 PWM 计数值。速度误差需要除以 1000 恢复 m/s。

这帧也用满 8 字节，因此没有内部 CRC，依靠串口外层 CRC。

## 20. 编码器解码 `decode_encoder()`

先检查：

```python
frame.can_id in (ID_LEFT_ENCODER, ID_RIGHT_ENCODER)
```

表示同一个函数既能解左编码器，也能解右编码器。

再检查内部 CRC：

```python
has_valid_crc(frame)
```

解包：

```python
struct.unpack("<ihB", frame.data[:7])
```

| 字节 | 格式 | 含义 |
|---:|---|---|
| 0～3 | `i` | int32 累计编码器计数 |
| 4～5 | `h` | int16 本周期增量 |
| 6 | `B` | sequence |
| 7 | CRC | 前 7 字节校验 |

为什么需要累计计数和周期增量两种数据？

- 累计计数适合 RDK 计算长期里程计；
- 周期增量适合诊断当前编码器变化；
- sequence 用于确认左右两帧属于同一批数据。

主节点只有左右 sequence 相同才更新 `/odom`，避免拿“新左轮 + 旧右轮”计算错误转角。

## 21. 状态解码 `decode_status()`

格式：

```python
"<BHHBB"
```

长度：

```text
B 1
H 2
H 2
B 1
B 1
共 7 字节，再加内部 CRC
```

| 字节 | 含义 |
|---:|---|
| 0 | 状态编号 |
| 1～2 | 16 位故障标志 |
| 3～4 | 电池电压，mV |
| 5 | STM32 最后处理的运动命令 sequence |
| 6 | 控制周期异常累计次数，最大上传 255 |
| 7 | CRC |

### 21.1 使用你实际看到的状态举例

逻辑数据：

```text
01 00 00 68 29 35 00 FB
```

逐项解释：

```text
01       状态 1 = IDLE
00 00    故障位 = 0 = NONE
68 29    小端 0x2968 = 10600 mV = 10.600 V
35       十六进制 0x35 = 十进制 53，最后命令序号
00       control_overruns = 0
FB       内部 CRC
```

解码结果：

```python
StatusData(
    state=1,
    faults=0,
    battery_mv=10600,
    last_command_sequence=53,
    overrun_count=0,
)
```

主节点再将这些内容放进 `/diagnostics`。

## 22. 参数应答解码 `decode_parameter_reply()`

格式：

```python
"<BBfB"
```

| 字节 | 含义 |
|---:|---|
| 0 | 参数 ID，系统命令应答时可能为 `0xFE` |
| 1 | 应答状态 |
| 2～5 | 实际参数值或系统 opcode，float32 |
| 6 | sequence |
| 7 | CRC |

应答状态：

| 数值 | 含义 |
|---:|---|
| 0 | 成功 |
| 1 | CRC 错误 |
| 2 | 参数/操作编号错误 |
| 3 | 参数值不合法 |
| 4 | 当前状态不允许操作 |
| 5 | Flash 保存错误 |

主节点不会仅凭“收到一帧”就认为参数成功。它还比较：

```text
参数 ID 是否匹配
sequence 是否匹配
status 是否为 0
```

这防止迟到的旧应答被当成本次请求的结果。

## 23. 心跳解码 `decode_heartbeat()`

格式：

```python
"<IBBB"
```

| 字节 | 含义 |
|---:|---|
| 0～3 | STM32 上电运行时间，ms，uint32 |
| 4 | 当前状态 |
| 5 | 固件主版本 |
| 6 | 固件次版本 |
| 7 | CRC |

示例：

```text
40 E2 01 00 01 01 00 AA
```

解码：

```text
40 E2 01 00 → 123456 ms
01          → IDLE
01          → 固件主版本 1
00          → 固件次版本 0
AA          → CRC
```

心跳每秒上传一次。主节点收到后记录本机单调时间：

```python
self._last_heartbeat_time = time.monotonic()
```

如果超过 2 秒没收到新心跳，就发布：

```text
STM32 heartbeat missing
```

如果新心跳的 `uptime_ms` 比上一次小，主节点推断 STM32 发生过复位，于是重新建立编码器里程计基准。

## 24. `fault_text()` 怎样组合故障文字

原代码：

```python
def fault_text(faults: int) -> str:
    names = [name for bit, name in FAULT_NAMES.items() if faults & bit]
    return "NONE" if not names else "|".join(names)
```

### 24.1 展开理解

```python
names = []

for bit, name in FAULT_NAMES.items():
    if faults & bit:
        names.append(name)

if not names:
    return "NONE"
else:
    return "|".join(names)
```

### 24.2 `"|".join(names)`

如果：

```python
names = ["COMMAND_TIMEOUT", "LEFT_STALL"]
```

结果：

```text
COMMAND_TIMEOUT|LEFT_STALL
```

这种文字最后进入 `/diagnostics`，方便人查看。

## 25. 它怎样连接主节点 `chassis_can_node.py`

主节点导入：

```python
from . import can_protocol as protocol
from .can_protocol import CanFrame, EncoderData, MotorOutput, StatusData, WheelSpeed
```

### 25.1 点号相对导入

```python
from . import can_protocol
```

开头的 `.` 表示从当前 Python 包 `chassis_can_control` 内导入，不是从系统中寻找另一个同名模块。

### 25.2 发送运动命令

```python
frame = protocol.make_motion_command(
    linear,
    angular,
    enable,
    self._emergency_stop,
    self._next_sequence(),
)
self._send(frame)
```

流程：

```text
ROS Twist
→ 提取 linear.x 和 angular.z
→ 协议模块生成 CanFrame
→ 主节点交给传输模块 send()
```

### 25.3 接收数据分发

主节点接收 `CanFrame` 后：

```python
if frame.can_id == protocol.ID_WHEEL_SPEED:
    self._latest_speed = protocol.decode_wheel_speed(frame)
elif frame.can_id == protocol.ID_STATUS:
    self._latest_status = protocol.decode_status(frame)
elif frame.can_id == protocol.ID_HEARTBEAT:
    heartbeat = protocol.decode_heartbeat(frame)
```

这是典型的按 ID 分发。

### 25.4 解码结果怎样进入 ROS

```text
WheelSpeed + MotorOutput
→ Float32MultiArray
→ /motor/debug

EncoderData
→ WheelOdometry
→ nav_msgs/Odometry
→ /odom

StatusData + Heartbeat
→ DiagnosticArray
→ /diagnostics
```

协议模块自身不认识 ROS 消息类型。它只返回普通 Python 数据对象，使协议层和 ROS 层分离。

## 26. 它怎样连接串口模块 `serial_transport.py`

协议模块输出：

```python
CanFrame(can_id=0x101, data=8字节)
```

串口模块 `_encode()` 再生成：

```text
0      A5
1      5A
2      外层协议版本 01
3～4   logical ID，小端
5      数据长度 08
6～13  CanFrame.data
14     外层 CRC
```

接收方向中，串口模块验证完整 15 字节后，只把：

```text
logical ID
data[8]
```

交给协议模块。

因此分工是：

```text
can_protocol.py     决定 8 字节里面是什么意思
serial_transport.py 决定怎样在连续串口字节流中找到并运输这 8 字节
```

## 27. 它怎样连接 STM32 C 协议代码

对应文件：

```text
STM32：CHASSIS_SERIAL/src/chassis_can_protocol.c
RDK：  chassis_can_control/can_protocol.py
```

对应关系：

| Python | STM32 C |
|---|---|
| `crc8()` | `chassis_crc8()` |
| `make_motion_command()` | `chassis_decode_motion_command()` |
| `make_parameter_command()` | `chassis_decode_parameter_command()` |
| `make_system_command()` | `chassis_decode_system_command()` |
| `decode_wheel_speed()` | `chassis_encode_wheel_speed()` |
| `decode_motor_output()` | `chassis_encode_motor_output()` |
| `decode_encoder()` | `chassis_encode_encoder()` |
| `decode_status()` | `chassis_encode_status()` |
| `decode_parameter_reply()` | `chassis_encode_parameter_reply()` |
| `decode_heartbeat()` | `chassis_encode_heartbeat()` |

Python `struct.pack("<hhBBB")` 和 C 中的逐字节写入必须形成完全相同的结果。

例如 Python 线速度：

```python
linear_mps * 1000
```

STM32 解码：

```c
command->linear_mps = get_i16_le(&frame->data[0]) / 1000.0f;
```

一端乘 1000，一端除 1000，形成对称关系。

## 28. 内部 CRC 与外层 CRC

这是本模块最容易混淆的知识点之一。

### 28.1 逻辑帧内部 CRC

部分 logical data 使用：

```text
data[0]～data[6] = 有效内容
data[7] = 内部 CRC
```

包括：

- 运动命令；
- 参数命令；
- 系统命令；
- 编码器；
- 状态；
- 参数应答；
- 心跳。

### 28.2 串口外层 CRC

完整 15 字节串口帧最后还有一个 CRC，覆盖：

```text
外层版本 + ID + 长度 + 完整 data[8]
```

它由 `serial_transport.py` 和 STM32 `chassis_serial_transport.c` 处理。

### 28.3 为什么轮速和 PWM 帧没有内部 CRC

轮速需要 4 个 int16，已经占满 8 字节；PWM/误差也占满 8 字节。因此没有内部 CRC，但串口外层 CRC仍覆盖它们。

当前串口版本中，任何完整帧在交给 `can_protocol.py` 前都已经通过外层 CRC。

## 29. sequence 序号解决什么问题

### 29.1 序号不是时间戳

sequence 只是一个 8 位递增编号：

```text
0、1、2、...、254、255、0、1...
```

### 29.2 参数请求匹配

假设参数请求 9 超时后重试，之后又发送请求 10。如果请求 9 的迟到应答突然到达，只比较参数 ID 可能误认为是请求 10 的结果。

同时比较 ID 和 sequence，可以丢弃旧应答。

### 29.3 左右编码器配对

STM32 同一次生成左、右编码器帧时使用相同 sequence。RDK 只用相同 sequence 的左右数据计算里程计。

### 29.4 诊断最后命令

状态帧上传 STM32 最后处理的运动命令 sequence，可以帮助判断 STM32 是否持续接收新命令。

## 30. 为什么协议模块不直接使用 ROS 消息

可以想象一种写法：让 `make_motion_command()` 直接接收 `Twist`。但当前设计没有这样做，而是接收普通数值：

```python
make_motion_command(linear_mps, angular_radps, ...)
```

优点：

1. 协议模块不依赖 ROS 2；
2. Windows 上也能单独测试；
3. 更容易写单元测试；
4. 将来别的程序也能复用协议；
5. ROS 消息变化不会直接影响二进制协议层。

这叫低耦合：模块之间知道得尽量少。

## 31. 为什么协议模块不直接打开串口

同理，它只产生和解析 `CanFrame`，不关心传输方式。

因此同一个协议模块可以配合：

```text
SerialTransport
SocketCanTransport
未来可能的 UDP 或测试用内存传输
```

只要传输层最后交付相同的 ID + 8 字节，协议层不需要改变。

## 32. 离线运行协议测试

协议测试不需要让电机运行，也不需要打开真实串口。

### 32.1 在 RDK X5 上

```bash
cd ~/chassis_project/ros2_ws/src/chassis_can_control
python3 test/run_protocol_tests.py
```

正常输出：

```text
All Python protocol tests passed.
```

这条命令只运行 Python 计算，不会使能电机。

### 32.2 命令逐部分解释

```bash
cd ~/chassis_project/ros2_ws/src/chassis_can_control
```

进入功能包源码目录。

```bash
python3 test/run_protocol_tests.py
```

用 Python 3 解释器执行测试脚本。

此测试脚本主动把源码目录加入 `sys.path`，所以不要求先运行 ROS 节点。

### 32.3 当前测试检查什么

```text
CRC 已知向量是否得到 0xF4
运动命令每个字节是否正确
float32 参数能否打包并解包
多个故障能否转成文字
里程计基本计算是否正确
```

## 33. 在 Python 交互环境中观察一帧

在 RDK 或 Ubuntu 工程源码目录：

```bash
cd ~/chassis_project/ros2_ws/src/chassis_can_control
python3
```

出现 `>>>` 后逐行输入：

```python
from chassis_can_control import can_protocol as p

frame = p.make_motion_command(0.05, 0.0, True, False, 7)

hex(frame.can_id)
frame.data.hex(" ")
p.has_valid_crc(frame)
```

预期：

```text
'0x101'
'32 00 00 00 01 01 07 a0'
True
```

退出 Python：

```python
exit()
```

以上操作只生成内存中的字节，没有调用串口 `send()`，因此不会让电机运动。

## 34. 查看完整 15 字节串口帧

仍在源码目录进入 Python：

```bash
python3
```

输入：

```python
from chassis_can_control import can_protocol as p
from chassis_can_control.serial_transport import SerialTransport

frame = p.make_motion_command(0.05, 0.0, True, False, 7)
packet = SerialTransport._encode(frame)
packet.hex(" ")
len(packet)
```

预期：

```text
'a5 5a 01 01 01 08 32 00 00 00 01 01 07 a0 eb'
15
```

`_encode()` 只返回字节，不打开串口。仍然是离线安全操作。

## 35. 如何理解协议单元测试

例如：

```python
def test_motion_command_layout():
    frame = protocol.make_motion_command(0.321, -0.456, True, False, 77)
    assert frame.can_id == protocol.ID_MOTION_COMMAND
    assert frame.data == bytes([...])
    assert protocol.has_valid_crc(frame)
```

`assert` 表示“这个条件必须为真”。如果为假，测试立即失败并指出位置。

这个测试不只检查函数有没有返回，还检查：

- ID 是否正确；
- 正速度字节顺序是否正确；
- 负速度补码和小端是否正确；
- flags、版本和序号位置是否正确；
- CRC 是否正确。

协议特别适合单元测试，因为它是纯计算，不依赖硬件。

## 36. 常见错误与排查方法

### 36.1 ID 正确但数据解释错误

检查：

- `struct` 格式是否一致；
- 字段顺序是否一致；
- 小端/大端是否一致；
- 缩放比例是否一致；
- 有符号/无符号是否一致。

### 36.2 正数正常，负数错误

重点检查两端是否都使用有符号 `int16`，以及是否正确处理补码。

### 36.3 参数值接近但不完全相等

float32 存在正常舍入误差。不要用字符串或绝对完全相等判断小数，测试中应使用允许误差的比较，例如 `math.isclose()`。

### 36.4 CRC 一直失败

检查：

- CRC 覆盖的是哪些字节；
- 是否把 CRC 字节本身也算进去了；
- 多项式是否为 `0x07`；
- 初始值是否为 0；
- 外层 CRC 和内部 CRC 是否混淆；
- Python 和 STM32 是否使用同一算法。

### 36.5 收到数据但解码函数返回 `None`

可能原因：

- logical ID 不匹配；
- 内部 CRC 错；
- 调用了错误的解码函数。

### 36.6 心跳丢失不一定是协议函数错误

还可能是：

- STM32 未供电；
- 0002 串口不存在；
- 两个 ROS 节点抢同一串口；
- 串口模块不能形成完整 15 字节；
- 下位机固件不是匹配版本。

需要按照完整链路逐层排查，而不是只看 `decode_heartbeat()`。

## 37. 修改协议时必须遵守的规则

协议不是只改 Python 一处就完成。任何字段改变都要同步检查：

```text
RDK can_protocol.py
STM32 chassis_can_protocol.h/.c
RDK 主节点调用处
STM32 控制模块调用处
串口外层是否仍能容纳
协议单元测试
文档
协议版本号
```

例如想新增 IMU 报文，应至少决定：

1. 新 logical ID；
2. 一帧 8 字节能放哪些轴；
3. 加速度和角速度使用什么单位；
4. 使用整数缩放还是 float32；
5. 如何添加时间或 sequence；
6. 是否需要多帧配对；
7. CRC 如何保护；
8. Python 解码数据类；
9. ROS `sensor_msgs/msg/Imu` 发布；
10. 坐标系和协方差。

不能只把六个原始 `short` 随便发出来就称为完整 ROS IMU。

## 38. 推荐的代码阅读顺序

第一次阅读不要从第一行一直死磕到最后一行，建议分五轮：

### 第一轮：只看整体分类

```text
常量
字典
数据类
CRC
3 个编码函数
5 个解码函数
故障文字函数
```

### 第二轮：只研究运动命令

```text
make_motion_command()
struct.pack("<hhBBB")
CRC
0.05 m/s 示例
STM32 对应解码
```

### 第三轮：只研究状态反馈

```text
decode_status()
StatusData
故障位
电池 mV
/diagnostics
```

### 第四轮：研究参数

```text
PARAMETER_IDS
PARAMETER_RANGES
make_parameter_command()
decode_parameter_reply()
主节点参数队列
```

### 第五轮：研究模块连接

```text
chassis_can_node.py
can_protocol.py
serial_transport.py
STM32 C 协议
```

## 39. 本模块的输入和输出总结

### RDK 向 STM32 发送

| 函数 | Python 输入 | 输出 logical ID |
|---|---|---:|
| `make_motion_command()` | 速度、使能、急停、序号 | `0x101` |
| `make_parameter_command()` | 参数 ID、操作、值、序号 | `0x102` |
| `make_system_command()` | 操作码、序号 | `0x103` |

### RDK 解码 STM32 上传

| 函数 | 输入 logical ID | Python 输出对象 |
|---|---:|---|
| `decode_wheel_speed()` | `0x181` | `WheelSpeed` |
| `decode_motor_output()` | `0x182` | `MotorOutput` |
| `decode_encoder()` | `0x183/0x184` | `EncoderData` |
| `decode_status()` | `0x185` | `StatusData` |
| `decode_parameter_reply()` | `0x186` | `ParameterReply` |
| `decode_heartbeat()` | `0x700` | `Heartbeat` |

## 40. 最后用一句话理解这个模块

```text
can_protocol.py 是上下位机共同语言在 RDK Python 端的实现：
它不负责运输字节，也不负责控制电机，只负责保证每个数值按双方约定的位置、类型、单位、字节序和 CRC 变成 8 字节，或者从 8 字节恢复成清晰的 Python 数据对象。
```

理解这个定位后，再阅读主节点和串口模块就不会混乱：

```text
主节点决定“什么时候发、收到后做什么”；
协议模块决定“8 个字节分别表示什么”；
串口模块决定“这 8 个字节怎样可靠地通过连续串口字节流传输”；
STM32 协议模块按完全相同的约定进行相反转换。
```

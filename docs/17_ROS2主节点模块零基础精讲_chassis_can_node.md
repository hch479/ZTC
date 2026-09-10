# ROS 2 主节点模块零基础精讲：`chassis_can_node.py`

## 1. 本文讲解对象

本文只集中讲解 RDK X5 上位机的 ROS 2 主节点：

```text
ros2_ws/src/chassis_can_control/
└── chassis_can_control/
    └── chassis_can_node.py
```

实际 RDK 路径是：

```text
/home/wheeltec/chassis_project/ros2_ws/src/
chassis_can_control/chassis_can_control/chassis_can_node.py
```

这个文件是整个上位机程序的“总协调者”，主要负责：

1. 创建 ROS 2 节点；
2. 读取 YAML 参数；
3. 创建并打开串口传输对象；
4. 接收 ROS `/cmd_vel`；
5. 定时向 STM32 发送运动命令；
6. 接收 STM32 轮速、编码器、状态和心跳；
7. 发布 `/odom`、`/diagnostics` 等 ROS 数据；
8. 提供使能、急停、清故障和参数管理服务；
9. 节点退出前安全关闭电机输出。

它不会直接产生 PWM，也不会直接读 STM32 的定时器。电机 PI、PWM、编码器硬件读取和安全状态机的最终执行者仍是 STM32。

## 2. 先用一句话理解这个节点

```text
chassis_can_node.py = ROS 2 世界与 STM32 底盘世界之间的翻译员和调度员
```

从 ROS 到 STM32：

```text
/cmd_vel
  ↓
ROS Twist 消息
  ↓ chassis_can_node.py
协议逻辑帧
  ↓ serial_transport.py
USB 串口字节
  ↓
STM32
```

从 STM32 到 ROS：

```text
STM32 遥测字节
  ↓ serial_transport.py
协议逻辑帧
  ↓ can_protocol.py
有名字的 Python 数据对象
  ↓ chassis_can_node.py
/odom、/diagnostics、/motor/debug、/joint_states、TF
```

## 3. 在读代码前必须知道的 Python 基础

### 3.1 Python 用缩进表示代码块

Python 没有用 `{}` 包住函数和判断，而是用缩进：

```python
if ready:
    run_motor()
    send_status()
```

两行都缩进 4 个空格，所以都属于 `if`。缩进恢复后，代码块结束：

```python
if ready:
    run_motor()

send_status()  # 不属于 if，无论 ready 是真是假都会执行
```

### 3.2 什么是函数

```python
def _on_cmd_vel(self, message: Twist) -> None:
    self._latest_command = message
```

逐项解释：

- `def`：定义函数；
- `_on_cmd_vel`：函数名称；
- `self`：当前节点对象；
- `message`：别人传进来的参数；
- `: Twist`：类型提示，表示希望它是 Twist 消息；
- `-> None`：函数不返回业务结果；
- 冒号后缩进的代码是函数体。

定义函数不代表它马上执行。要么由其他代码调用，要么由 ROS 收到事件后调用。

### 3.3 什么是类和对象

代码定义：

```python
class ChassisCanNode(Node):
```

可以把“类”理解为设计图。创建对象：

```python
node = ChassisCanNode()
```

相当于按设计图创建一个真正运行的底盘节点对象。

`ChassisCanNode(Node)` 表示它继承 ROS 2 的 `Node` 类。继承后才能使用：

```python
self.create_publisher(...)
self.create_subscription(...)
self.create_service(...)
self.create_timer(...)
self.get_logger()
```

### 3.4 `self` 到底是什么

假设：

```python
node = ChassisCanNode()
```

那么类方法中的 `self` 就是 `node` 这个对象。

```python
self._drive_enabled = False
```

表示在当前节点对象中保存一个名为 `_drive_enabled` 的变量。

不同节点对象各自拥有自己的变量。当前项目正常情况下只创建一个节点对象。

### 3.5 对象属性和局部变量的区别

局部变量：

```python
now = time.monotonic()
```

只在当前函数执行期间使用。

对象属性：

```python
self._last_cmd_vel_time = time.monotonic()
```

保存到节点对象中，其他回调以后还能读取。

节点需要记住最新命令、最新状态和心跳时间，因此大量使用 `self.xxx`。

### 3.6 `None`、布尔值和判断

```text
None   当前没有有效对象
True   真
False  假
```

启动时：

```python
self._latest_heartbeat = None
```

表示尚未收到 STM32 心跳。

之后：

```python
if self._latest_heartbeat is None:
```

就能判断通信是否从未建立。

### 3.7 列表、字典和元组

列表：

```python
message.data = [left_target, left_measured, right_target, right_measured]
```

有顺序，可通过下标访问。

字典：

```python
defaults = {
    "controller.left_kp": 3000.0,
    "controller.left_ki": 1500.0,
}
```

通过名字查值。

元组：

```python
(parameter_id, value)
```

这里表示固定的一对数据。

### 3.8 `for` 循环

```python
for name, value in defaults.items():
    self.declare_parameter(name, value)
```

意思是逐个取出字典中的参数名和默认值，然后声明 ROS 参数。

### 3.9 `try/except`

```python
try:
    self._can.open()
except OSError as error:
    self.get_logger().error(str(error))
```

`try` 中执行可能失败的操作。如果串口打开失败，会跳到 `except`，记录错误而不是无说明地崩溃。

### 3.10 类型提示不是另一种运算

```python
self._latest_status: Optional[StatusData] = None
```

意思是这个变量以后可能保存 `StatusData`，也可能是 `None`。`Optional` 主要帮助阅读和编辑器检查，不会自动产生数据。

## 4. ROS 2 到底是什么

ROS 2 不是普通的“一个程序”，更像一套让多个机器人程序相互通信的框架。

例如：

```text
键盘控制节点 ──发布──> /cmd_vel
                           ↓
                    底盘主节点
                           ↓
底盘主节点 ──发布──> /odom ──> 导航节点
```

每个独立运行的 ROS 程序称为“节点”。节点之间主要通过话题、服务、参数和 TF 交换信息。

## 5. ROS 2 的四个核心概念

### 5.1 节点 Node

本项目节点名称：

```text
/chassis_can_node
```

查看：

```bash
ros2 node list
```

查看它订阅、发布和提供什么服务：

```bash
ros2 node info /chassis_can_node
```

### 5.2 话题 Topic

话题适合持续数据流。

发布者像广播电台，不断向某个频道发送消息；订阅者像收音机，订阅对应频道后接收消息。

本节点订阅：

```text
/cmd_vel
```

本节点发布：

```text
/odom
/joint_states
/diagnostics
/motor/debug
/tf
```

### 5.3 服务 Service

服务适合“一次请求，一次回答”。例如：

```text
客户端：请使能电机
服务端：请求已接受，drive enabled
```

本节点提供：

```text
/chassis/enable
/chassis/emergency_stop
/chassis/clear_faults
/chassis/push_parameters
/chassis/save_parameters
/chassis/reset_odometry
```

### 5.4 参数 Parameter

参数是节点的可配置数据，例如：

```text
串口路径
波特率
轮径
轮距
PI 参数
超时阈值
```

它们主要从 `chassis_serial.yaml` 加载。

## 6. 主节点与其他文件的关系

主节点没有把所有功能都写在一个文件里，而是调用其他模块：

| 文件 | 主节点怎样使用它 |
|---|---|
| `can_protocol.py` | 编码运动/参数命令，解码 STM32 遥测 |
| `serial_transport.py` | 打开串口、发送和接收 15 字节包 |
| `socketcan_transport.py` | 将来 CAN 版使用 |
| `wheel_odometry.py` | 根据左右编码器累计计数计算位姿 |
| `chassis_serial.yaml` | 提供串口、几何、控制和安全参数 |
| `chassis_serial.launch.py` | 加载 YAML 并启动主节点 |
| `setup.py` | 把 `chassis_can_node` 命令连接到 `main()` |
| STM32 `chassis_can_protocol.c` | 与 Python 协议定义一一对应 |

主节点顶部的导入：

```python
from . import can_protocol as protocol
from .serial_transport import SerialTransport
from .wheel_odometry import WheelOdometry
```

开头的点 `.` 表示从当前 Python 包内部导入。

## 7. 程序真正从哪里开始

文件底部：

```python
if __name__ == "__main__":
    main()
```

当 Python 直接运行这个文件时，调用 `main()`。

ROS 安装入口由 `setup.py` 定义：

```python
"chassis_can_node = chassis_can_control.chassis_can_node:main"
```

所以 ROS 启动可执行程序时，最终也进入：

```python
def main(args=None) -> None:
    rclpy.init(args=args)
    node = ChassisCanNode()
    try:
        rclpy.spin(node)
    ...
```

逐句理解：

### 7.1 初始化 ROS

```python
rclpy.init(args=args)
```

`rclpy` 是 ROS 2 的 Python 客户端库。它建立 ROS 运行环境，让程序能够创建节点和通信接口。

### 7.2 创建节点对象

```python
node = ChassisCanNode()
```

自动调用 `ChassisCanNode.__init__()`，节点的大部分初始化都在这里完成。

### 7.3 进入事件循环

```python
rclpy.spin(node)
```

这是理解 ROS 代码最关键的一句。

程序不会按文件顺序把所有函数逐个执行。它完成初始化后进入 `spin()`，等待事件：

```text
收到 /cmd_vel       → 调用 _on_cmd_vel()
收到服务请求        → 调用对应服务回调
20 ms 定时器到期    → 调用 _send_motion_command()
5 ms 定时器到期     → 调用 _poll_can()
50 ms 定时器到期    → 调用 _send_one_pending_parameter()
200 ms 定时器到期   → 调用 _publish_diagnostics()
```

这种“事件发生后调用指定函数”的函数叫回调函数。

### 7.4 退出

按 `Ctrl+C` 会产生 `KeyboardInterrupt`，之后进入 `finally`：

```python
node.destroy_node()
```

主节点重写了 `destroy_node()`，退出前：

1. 软件使能清零；
2. 急停发送位清零；
3. 连续发送三次失能运动命令；
4. 关闭串口；
5. 销毁 ROS 节点。

这是为了降低节点退出瞬间 STM32 仍保留旧命令的风险。STM32 自己还有命令超时作为第二道保护。

## 8. `__init__()`：节点的总初始化

```python
class ChassisCanNode(Node):
    def __init__(self) -> None:
        super().__init__("chassis_can_node")
```

`super()` 表示调用父类 `Node` 的初始化。字符串决定节点名为：

```text
chassis_can_node
```

如果没有这一步，后面的 `create_publisher()` 等 ROS 方法没有可用的底层节点对象。

初始化可以分成六个阶段：

```text
声明并读取参数
→ 创建传输对象并打开接口
→ 建立内部状态变量
→ 创建发布者
→ 创建订阅者和服务
→ 创建定时器与参数回调
```

## 9. 第一阶段：声明和读取参数

### 9.1 为什么必须声明

```python
self.declare_parameter("serial_baud_rate", 115200)
```

表示节点认识一个名为 `serial_baud_rate` 的参数，默认值为 115200。

启动时 launch 加载 YAML，如果 YAML 提供该参数，就用 YAML 的值覆盖默认值。

### 9.2 参数来自哪里

启动链：

```text
ros2 launch
  ↓
chassis_serial.launch.py
  ↓ 找到安装目录里的配置文件
install/.../config/chassis_serial.yaml
  ↓
将参数传给 ChassisCanNode
```

注意：launch 读取的是 `install` 中的配置副本。因此修改源码目录 YAML 后必须重新编译，才能更新安装副本。

### 9.3 读取参数

```python
self.get_parameter("serial_baud_rate").value
```

返回的是 ROS 参数对象的值。为了明确类型，代码又转换：

```python
int(...)
str(...)
float(...)
bool(...)
```

### 9.4 参数为什么在 Python 和 YAML 各写一遍

Python 默认值保证没有 YAML 时仍能启动；YAML 便于不改代码就针对实车配置。

正常以 YAML 为当前实车配置来源，Python 默认值应和合理安全值保持一致。

## 10. 第二阶段：选择串口或 CAN 传输

```python
transport = str(self.get_parameter("transport").value).lower()
```

当前 YAML：

```yaml
transport: serial
```

因此执行：

```python
self._can = SerialTransport(serial_port, baud_rate)
```

这里的变量名 `_can` 是项目早期为了统一 CAN/串口接口保留的名称。当前 `_can` 变量中实际保存的是 `SerialTransport` 对象，不代表物理接口正在使用 CAN。

同理：

```python
_poll_can()
_open_can_if_needed()
```

在当前串口配置下实际是在轮询和打开串口。这只是命名未重构，不影响运行逻辑。

### 10.1 为什么这样设计

两个传输类都提供同一组接口：

```text
open()
close()
send(frame)
receive_available()
is_open
interface
```

主节点只调用这些统一接口，不需要在每个业务函数里写一遍“如果串口……否则 CAN……”。这种思想叫抽象或接口统一。

### 10.2 打开失败怎么办

`_open_can_if_needed()`：

```text
已经打开 → 直接成功
距离上次尝试不到 1 秒 → 暂不重试
否则尝试 open()
失败 → 记录日志并返回 False
```

限制为每秒一次，避免设备不存在时疯狂打开和刷屏。

错误日志每 2 秒最多打印一次，避免终端被相同报错淹没。

## 11. 第三阶段：内部状态变量

### 11.1 上位机安全状态

```python
self._drive_enabled = False
self._emergency_stop = False
```

启动默认失能。即使节点收到 `/cmd_vel`，没有显式调用 `/chassis/enable`，也只会给 STM32 发零速度和失能位。

### 11.2 最新速度命令

```python
self._latest_command = Twist()
self._has_received_cmd_vel = False
self._last_cmd_vel_time = 0.0
```

分别保存：最新 Twist、是否收到过、最后接收时间。

### 11.3 最新 STM32 遥测

```python
self._latest_heartbeat = None
self._latest_status = None
self._latest_speed = None
self._latest_output = None
self._left_encoder = None
self._right_encoder = None
```

主节点把不同报文最新值分别缓存。某些 ROS 消息需要组合两种数据，例如 `/motor/debug` 同时需要轮速和 PWM，所以必须暂存。

### 11.4 里程计对象

```python
self._odometry = WheelOdometry()
```

真正的编码器里程计公式放在 `wheel_odometry.py`，主节点负责把编码器数据和参数交给它，再发布结果。

### 11.5 参数队列

```python
self._parameter_queue = deque()
self._pending_parameter = None
self._parameter_transfer_failed = False
```

`deque` 是双端队列。本项目从右边加入参数，从左边依次发送和移除。

## 12. 第四阶段：创建发布者

### 12.1 `/odom`

```python
self._odom_publisher = self.create_publisher(Odometry, "odom", 20)
```

三个参数：

```text
Odometry   消息类型
"odom"     话题名，最终显示为 /odom
20         队列深度
```

队列深度不是发布频率。它大致表示通信短暂拥堵时可以缓存多少条消息。

### 12.2 `/joint_states`

发布左右轮的：

```text
关节名称
累计转角
角速度
```

可供机器人模型和可视化使用。

### 12.3 `/diagnostics`

发布：

```text
STM32 状态
故障
电池电压
心跳新鲜度
传输类型和设备路径
参数传输状态
```

### 12.4 `/motor/debug`

发布数组：

```text
[左目标速度,
 左实测速度,
 右目标速度,
 右实测速度,
 左PWM,
 右PWM,
 左速度误差,
 右速度误差]
```

### 12.5 TF 广播器

```python
self._tf_broadcaster = TransformBroadcaster(self)
```

负责发布：

```text
odom → base_link
```

## 13. 第五阶段：创建订阅者

```python
self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
```

含义：

```text
订阅话题 /cmd_vel
消息类型必须为 geometry_msgs/msg/Twist
收到消息时调用 self._on_cmd_vel
队列深度为 10
```

### 13.1 Twist 是什么

Twist 包含线速度和角速度：

```text
linear.x  前后速度，m/s
linear.y  左右速度，普通阿克曼/差速车通常不用
linear.z  上下速度，地面车不用
angular.x 绕 X 轴角速度
angular.y 绕 Y 轴角速度
angular.z 绕 Z 轴转向角速度，rad/s
```

本项目只读取：

```python
self._latest_command.linear.x
self._latest_command.angular.z
```

### 13.2 `/cmd_vel` 回调为什么只保存不发送

```python
def _on_cmd_vel(self, message):
    self._latest_command = message
    self._has_received_cmd_vel = True
    self._last_cmd_vel_time = time.monotonic()
```

它不立刻向串口发送，因为不同 ROS 节点的发布频率可能不稳定。主节点统一由 50 Hz 定时器发送，从而：

- 串口命令频率相对固定；
- 很容易判断命令是否超时；
- 软件使能和急停检查集中在一个地方；
- ROS 发布者短暂抖动不会直接改变串口调度结构。

## 14. 第六阶段：创建服务

### 14.1 `SetBool` 服务

`SetBool` 请求只有一个布尔字段：

```text
data: true 或 false
```

响应包含：

```text
success
message
```

用于：

```text
/chassis/enable
/chassis/emergency_stop
```

### 14.2 `Trigger` 服务

Trigger 请求没有业务字段，只表示“请执行一次”。响应也有：

```text
success
message
```

用于：

```text
/chassis/clear_faults
/chassis/save_parameters
/chassis/push_parameters
/chassis/reset_odometry
```

### 14.3 服务回调中的 `request` 和 `response`

```python
def _on_enable(self, request, response):
    self._drive_enabled = bool(request.data)
    response.success = True
    response.message = "drive enabled"
    return response
```

ROS 把请求对象传进来，函数填写响应对象并返回。

## 15. 第七阶段：创建定时器

```python
self.create_timer(1.0 / command_rate, self._send_motion_command)
self.create_timer(0.005, self._poll_can)
self.create_timer(0.05, self._send_one_pending_parameter)
self.create_timer(0.20, self._publish_diagnostics)
```

当前默认：

| 周期 | 频率 | 回调 | 功能 |
|---:|---:|---|---|
| 0.020 s | 50 Hz | `_send_motion_command` | 给 STM32 发送运动命令 |
| 0.005 s | 200 Hz | `_poll_can` | 读取已有串口帧 |
| 0.050 s | 20 Hz | `_send_one_pending_parameter` | 参数队列/重试 |
| 0.200 s | 5 Hz | `_publish_diagnostics` | 发布诊断 |

`_poll_can` 只是函数旧名。当前串口配置下实际轮询串口。

### 15.1 定时器是否绝对准时

不是硬实时保证。ROS 2 Python 默认执行器会按事件调度回调，如果某个回调耗时太久，其他回调会延迟。

因此：

- 主节点回调应保持短小；
- 电机 100 Hz 实时 PI 放在 STM32；
- Python 不承担最后一级硬件超时保护。

## 16. 运动命令发送回调详解

核心函数：

```python
def _send_motion_command(self) -> None:
```

### 16.1 取得当前单调时间

```python
now = time.monotonic()
```

`monotonic()` 专门测量时间间隔。即使用户校准系统日期，它也不会倒退。

### 16.2 判断命令是否新鲜

```python
command_is_fresh = (
    self._has_received_cmd_vel
    and now - self._last_cmd_vel_time <= timeout
)
```

逻辑：

```text
以前收到过 /cmd_vel
并且
距离最近一条不超过 0.20 s
```

### 16.3 三重允许条件

```python
if command_is_fresh and self._drive_enabled and not self._emergency_stop:
```

三个条件全部成立才发送真实速度：

```text
ROS速度命令新鲜
软件使能已经打开
没有急停
```

否则：

```python
linear = 0.0
angular = 0.0
enable = False
```

### 16.4 编码逻辑帧

```python
frame = protocol.make_motion_command(
    linear,
    angular,
    enable,
    self._emergency_stop,
    self._next_sequence(),
)
```

这会调用 `can_protocol.py`：

```text
linear m/s × 1000 → int16
angular rad/s × 1000 → int16
使能/急停 → flags 位
加入协议版本和 sequence
计算 8 字节逻辑帧 CRC
```

### 16.5 发送

```python
self._send(frame)
```

`_send()`：

1. 确认接口打开；
2. 调用当前传输对象的 `send()`；
3. 串口异常时打印错误、关闭接口；
4. 后续定时器再尝试重新打开。

## 17. 串口接收和帧分发详解

### 17.1 读取所有当前完整帧

```python
frames = self._can.receive_available()
```

当前 `_can` 是 `SerialTransport`。它负责：

- 从 Linux 设备文件非阻塞读字节；
- 缓存半帧和多帧；
- 搜索 `A5 5A`；
- 检查外层版本、长度和 CRC；
- 返回若干 `CanFrame` 逻辑帧。

### 17.2 遍历

```python
for frame in frames:
    self._handle_frame(frame)
```

如果一次读到 5 帧，就逐个处理 5 次。

### 17.3 根据 ID 分发

```python
if frame.can_id == protocol.ID_WHEEL_SPEED:
    ...
elif frame.can_id == protocol.ID_MOTOR_OUTPUT:
    ...
```

这相当于收到信封后根据“业务编号”交给不同处理逻辑。

| ID | 解码结果 | 后续用途 |
|---:|---|---|
| `0x181` | 左右目标/实测轮速 | `/motor/debug`、`/odom` 速度 |
| `0x182` | 左右 PWM/误差 | `/motor/debug` |
| `0x183` | 左累计编码器 | 里程计 |
| `0x184` | 右累计编码器 | 里程计 |
| `0x185` | 状态/故障/电压 | `/diagnostics` |
| `0x186` | 参数应答 | 参数队列确认 |
| `0x700` | 心跳/运行时间 | 通信诊断、复位检测 |

## 18. 为什么接收数据需要缓存“最新值”

轮速和 PWM 是两种不同报文，可能先后到达：

```text
先到轮速帧 → 只更新 _latest_speed
后到PWM帧  → 更新 _latest_output
两者都存在 → 发布完整 /motor/debug
```

如果不缓存，就很难组合来自不同报文的数据。

编码器也类似：必须等左、右两帧 sequence 相同，才确定它们是同一批测量。

## 19. 心跳处理和 STM32 重启检测

心跳中包含 STM32 上电运行时间 `uptime_ms`。

正常情况下它一直增加：

```text
1000, 2000, 3000, ...
```

如果新值小于旧值：

```python
heartbeat.uptime_ms < self._previous_stm32_uptime_ms
```

通常说明 STM32 复位了。复位后编码器累计计数也可能归零，因此主节点：

```python
self._odometry.reset()
self._last_odometry_sequence = None
```

避免把“计数从很大值突然回到 0”错误解释为小车瞬间倒退很远。

收到合法心跳后还记录：

```python
self._last_heartbeat_time = time.monotonic()
```

诊断模块据此计算 `heartbeat_age`。

## 20. 里程计处理与 `wheel_odometry.py` 的连接

### 20.1 等待同批左右计数

```python
if left is None or right is None or left.sequence != right.sequence:
    return
```

`return` 表示条件不满足就结束本次函数，不更新里程计。

还要避免同一个 sequence 重复计算：

```python
if self._last_odometry_sequence == left.sequence:
    return
```

### 20.2 调用独立里程计类

```python
sample = self._odometry.update(
    left.total_count,
    right.total_count,
    diameter,
    counts_per_rev,
    wheel_track,
)
```

主节点提供：

- 左右累计计数；
- 轮径；
- 每圈计数；
- 轮距。

`WheelOdometry` 计算并返回：

```text
x
y
yaw
左右轮累计角度
```

### 20.3 为什么单独放一个文件

里程计公式与 ROS 通信本身无关。独立以后：

- 文件更容易读；
- 可以在没有 ROS 的 Windows Python 中测试；
- 主节点不至于同时塞入所有数学细节；
- 以后替换算法更清晰。

## 21. 发布 `/odom`

创建消息：

```python
odom = Odometry()
```

### 21.1 时间戳

```python
odom.header.stamp = self.get_clock().now().to_msg()
```

表示这条 ROS 消息的生成时间。

### 21.2 坐标系

```python
odom.header.frame_id = "odom"
odom.child_frame_id = "base_link"
```

含义：底盘 `base_link` 位姿是相对于 `odom` 坐标系描述的。

### 21.3 位置

```python
odom.pose.pose.position.x = sample.x_m
odom.pose.pose.position.y = sample.y_m
```

地面车 z 保持默认 0。

### 21.4 姿态为什么使用四元数

当前只有二维偏航角 yaw：

```python
half_yaw = 0.5 * yaw
quaternion_z = sin(half_yaw)
quaternion_w = cos(half_yaw)
```

然后：

```text
qx = 0
qy = 0
qz = sin(yaw/2)
qw = cos(yaw/2)
```

ROS 姿态字段统一使用四元数。当前二维场景只需要 z 和 w 非零。

### 21.5 速度

```text
车体线速度 = (左实测 + 右实测) / 2
车体角速度 = (右实测 - 左实测) / 轮距
```

### 21.6 真正发布

```python
self._odom_publisher.publish(odom)
```

执行后所有订阅 `/odom` 的 ROS 节点都有机会收到。

## 22. 发布轮关节和 TF

`JointState` 中：

```text
name      左右轮关节名称
position  左右轮累计角度，rad
velocity  左右轮角速度，rad/s
```

轮角速度换算：

```text
角速度 = 轮缘线速度 / 轮半径
```

TF 使用同一组 `x、y、yaw` 广播 `odom → base_link`。导航和 RViz 通过 TF 了解不同坐标系之间的关系。

注意：当前里程计只来自编码器，没有融合 IMU。

## 23. `/motor/debug` 发布逻辑

```python
if self._latest_speed is None or self._latest_output is None:
    return
```

两种遥测缺一种就不发布，避免用不存在的数据。

数组顺序必须固定，因为 `Float32MultiArray` 本身没有字段名称：

```text
0 左目标速度
1 左实测速度
2 右目标速度
3 右实测速度
4 左 PWM
5 右 PWM
6 左速度误差
7 右速度误差
```

缺点是下标可读性一般；优点是结构简单。更正式的项目可以自定义 ROS 消息类型，让字段有明确名称。

## 24. `/diagnostics` 发布逻辑

### 24.1 创建诊断容器

```python
array = DiagnosticArray()
status = DiagnosticStatus()
```

一个 `DiagnosticArray` 可以包含多个设备状态；当前只加入一个 C30D 电机控制器状态。

### 24.2 心跳错误

```python
if self._latest_heartbeat is None or heartbeat_age > 2.0:
    status.level = DiagnosticStatus.ERROR
    status.message = "STM32 heartbeat missing"
```

对应输出：

```text
level: "\x02"
message: STM32 heartbeat missing
```

数值 2 代表 ERROR。

### 24.3 正常状态

如果故障位为 0：

```python
status.level = DiagnosticStatus.OK
status.message = state_name
```

对应：

```text
level: "\0"
message: IDLE
faults: NONE
```

`"\0"` 实际表示数值 0，即 OK。

### 24.4 WARN 与 ERROR

命令超时、硬件失能、控制周期超限被归为警告；急停、欠压、编码器、堵转、坏参数等被归为错误。

诊断中还附加：

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

### 24.5 为什么 `pending_parameters=0` 不等于通信正常

它只说明当前没有等待下发的参数。是否通信正常必须看：

```text
level
message
heartbeat_age
state
faults
```

## 25. 使能服务详解

```python
def _on_enable(self, request, response):
    self._drive_enabled = bool(request.data)
    response.success = True
    response.message = ...
    self._send_motion_command()
    return response
```

调用：

```bash
ros2 service call /chassis/enable \
std_srvs/srv/SetBool "{data: true}"
```

流程：

```text
终端发服务请求
→ ROS 调用 _on_enable
→ _drive_enabled = True
→ 立即执行一次发送检查
→ 返回 drive enabled
```

重要：`drive enabled` 表示 RDK 软件允许，不等于电机一定转动。还需要：

- 有 0.20 s 内的新 `/cmd_vel`；
- 没有急停；
- STM32 无严重故障；
- 硬件使能有效。

## 26. 急停与清故障服务

### 26.1 急停

```python
self._emergency_stop = bool(request.data)
if request.data:
    self._drive_enabled = False
```

触发急停时立即撤销软件使能，再发送带急停位的运动帧。STM32 收到后锁存 ESTOP。

仅调用：

```bash
... emergency_stop ... "{data: false}"
```

只会释放 RDK 本地急停请求，STM32 锁存仍存在。

### 26.2 清故障

`_on_clear_faults()` 的顺序：

1. RDK 软件失能；
2. RDK 急停请求清零；
3. 发送普通失能运动命令；
4. 再发送带钥匙的清故障系统命令。

STM32 只有确认当前命令既不使能也不急停，才接受清故障。

## 27. 参数下发模块为何比较复杂

控制参数不能像普通 ROS 变量一样只改上位机，因为真正执行 PI 的是 STM32。必须通过协议下发。

### 27.1 `PendingParameter`

```python
@dataclass
class PendingParameter:
    parameter_id: int
    value: float
    sequence: int
    attempts: int
    sent_time: float
```

保存“已经发送、正在等待 STM32 回应”的一个参数。

### 27.2 为什么一次只等待一个

流程最容易保证正确：

```text
发参数 1 → 等确认
确认后发参数 2 → 等确认
...
```

如果同时发 30 个，处理丢帧、乱序、重复会复杂很多。

### 27.3 参数下发定时器

每 50 ms 检查一次：

- 有 pending 且不到 200 ms：继续等；
- 超过 200 ms：重发；
- 已尝试 3 次：宣布失败、清空队列；
- 没有 pending 且队列非空：发送队首参数。

### 27.4 应答匹配

只有同时匹配：

```text
parameter_id
sequence
```

才认为是当前请求的应答。过期或其他请求的应答会被忽略。

### 27.5 保存参数

`/chassis/save_parameters` 在以下情况拒绝：

- 还有参数正在等待应答；
- 参数传输已经失败。

真正保存由 STM32 写 Flash。RDK 只是发送保存请求。

## 28. ROS 参数动态修改回调

```python
self.add_on_set_parameters_callback(self._on_parameters_changed)
```

当执行：

```bash
ros2 param set /chassis_can_node controller.left_kp 3100.0
```

ROS 在真正接受修改前调用 `_on_parameters_changed()`。

### 28.1 需要重启的参数

串口路径、波特率、transport、坐标系名称等不允许运行时修改，要求改 YAML 后重启。

原因例如：串口对象已经按旧路径打开，单纯改参数值不会自动重建整个传输对象。

### 28.2 运行中禁止改电机参数

如果 STM32 状态为 `RUNNING`，PI、死区、几何等受支持参数会被拒绝，避免运动中突然改变控制器。

### 28.3 数值检查

代码检查：

```text
能转换为数字
不是 NaN 或无穷大
处于允许范围
chassis_type 只能为 0 或 1
舵机换算系数绝对值不能太小
```

验证通过后把参数 ID和值加入下发队列。

### 28.4 动态修改不会自动改 YAML

`ros2 param set` 只修改当前运行节点，并排队下发 STM32。节点重启后，上位机仍从 YAML 加载。

要永久保持一致，应：

1. 修改源码 YAML；
2. 重新 `colcon build`；
3. 重启节点；
4. 失能状态下 push 参数；
5. 验证后才保存 STM32 Flash。

## 29. 复位里程计服务

如果 STM32 报告状态为 `RUNNING`，主节点拒绝复位。

允许时：

1. 给 STM32 发送复位里程计系统命令；
2. 清空 RDK 的 `WheelOdometry`；
3. 清除上一次编码器 sequence；
4. 下一批编码器只重新建立基准。

上下位机同时复位，避免两端位姿不一致。

## 30. 一个完整命令如何穿过所有模块

假设发布：

```text
linear.x = 0.05 m/s
angular.z = 0.0 rad/s
```

完整过程：

1. ROS 命令行或导航节点发布 `/cmd_vel`；
2. ROS 执行器调用 `_on_cmd_vel()`；
3. 主节点保存 Twist 和当前单调时间；
4. 20 ms 定时器调用 `_send_motion_command()`；
5. 检查命令新鲜、软件使能、无急停；
6. `can_protocol.make_motion_command()` 将 0.05 变为整数 50；
7. 生成 ID `0x101` 的 8 字节逻辑帧；
8. `SerialTransport.send()` 包成 15 字节串口帧；
9. Linux 将字节写入 `...0002-if00`；
10. STM32 USART3 中断接收并组帧；
11. STM32 通信任务解码运动命令；
12. STM32 100 Hz 控制任务计算目标轮速和 PI；
13. STM32 输出 PWM；
14. 编码器形成实测反馈；
15. STM32 回传轮速、PWM、编码器和状态；
16. RDK `_poll_can()` 收到逻辑帧；
17. `_handle_frame()` 按 ID 分发；
18. 主节点发布 `/motor/debug`、`/odom` 和 `/diagnostics`。

## 31. 主节点目前与 IMU 没有连接

当前主节点没有导入：

```python
from sensor_msgs.msg import Imu
```

也没有创建：

```text
/imu/data 发布者
IMU 协议 ID 解码
IMU 坐标系和单位换算
编码器/IMU 融合
```

所以当前 `/odom` 是纯编码器里程计。STM32 虽有 IMU 原始采集任务，但目前没有通过本协议上传到这个节点。

## 32. 如何在 Ubuntu 中阅读主节点代码

登录 RDK：

```bash
ssh wheeltec@192.168.0.100
```

进入代码目录：

```bash
cd ~/chassis_project/ros2_ws/src/chassis_can_control/
cd chassis_can_control
```

确认文件：

```bash
ls -l chassis_can_node.py
```

分页阅读：

```bash
less chassis_can_node.py
```

`less` 中：

```text
上下方向键   滚动
/关键词      搜索
n            下一个匹配
q            退出
```

带行号阅读：

```bash
nl -ba chassis_can_node.py | less
```

搜索函数：

```bash
grep -n '^    def ' chassis_can_node.py
```

不要为了阅读就随意保存修改。需要编辑时建议先复制备份。

## 33. 如何观察节点在运行时的行为

每个新终端先加载：

```bash
source /opt/ros/humble/setup.bash
source ~/chassis_project/ros2_ws/install/setup.bash
```

### 33.1 查看节点

```bash
ros2 node list
ros2 node info /chassis_can_node
```

### 33.2 查看话题结构

```bash
ros2 topic list -t
ros2 topic info /cmd_vel -v
ros2 topic info /odom -v
```

### 33.3 查看消息类型定义

```bash
ros2 interface show geometry_msgs/msg/Twist
ros2 interface show nav_msgs/msg/Odometry
ros2 interface show std_srvs/srv/SetBool
```

这个命令非常适合初学者，它会显示字段结构。

### 33.4 查看主节点输出

```bash
ros2 topic echo --once /diagnostics
ros2 topic echo /motor/debug
ros2 topic echo /odom
```

### 33.5 查看频率

```bash
ros2 topic hz /motor/debug
ros2 topic hz /odom
```

### 33.6 查看当前参数

```bash
ros2 param list /chassis_can_node
ros2 param get /chassis_can_node transport
ros2 param get /chassis_can_node serial_port
ros2 param dump /chassis_can_node
```

## 34. 怎样安全启动和避免重复节点

先检查：

```bash
pgrep -af 'chassis_serial.launch.py|chassis_can_node'
```

如果已经看到一套 `ros2 launch` 和一套 `chassis_can_node`，不要再次启动。

如果没有，再执行：

```bash
cd ~/chassis_project/ros2_ws
bash src/chassis_can_control/scripts/start_chassis_serial_safe.sh
```

重复节点会同时读同一个串口。Linux 串口数据不是广播复制给两个进程，而可能被两个进程分着读，导致双方都拼不出稳定完整帧，最后出现：

```text
STM32 heartbeat missing
```

安全停止（整行复制执行）：

```bash
bash ~/chassis_project/ros2_ws/src/chassis_can_control/scripts/stop_chassis_serial_safe.sh
```

## 35. 修改主节点代码后的流程

### 35.1 先停止节点

```bash
bash ~/chassis_project/ros2_ws/src/chassis_can_control/scripts/stop_chassis_serial_safe.sh
```

### 35.2 修改源码

真正应修改的是（下面只是文件路径，不是终端命令）：

```text
~/chassis_project/ros2_ws/src/chassis_can_control/chassis_can_control/chassis_can_node.py
```

不要直接修改 `install` 中自动生成/复制的文件，因为下次编译会覆盖。

### 35.3 检查 Python 语法

整行复制执行：

```bash
python3 -m py_compile ~/chassis_project/ros2_ws/src/chassis_can_control/chassis_can_control/chassis_can_node.py
```

没有输出通常表示语法通过。

### 35.4 重新编译

```bash
cd ~/chassis_project/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select chassis_can_control
source install/setup.bash
```

应看到：

```text
Summary: 1 package finished
```

### 35.5 重新安全启动并检查

```bash
bash src/chassis_can_control/scripts/start_chassis_serial_safe.sh
```

新终端：

```bash
source /opt/ros/humble/setup.bash
source ~/chassis_project/ros2_ws/install/setup.bash
ros2 topic echo --once /diagnostics
```

修改主节点不需要重新烧录 STM32；只有修改 STM32 C 工程才需要 Keil 编译和 ST-Link 烧录。

## 36. 适合初学者的函数阅读顺序

建议不要从第 1 行一路硬读到第 722 行，而按信息流阅读：

### 第一轮：启动骨架

```text
main()
ChassisCanNode.__init__()
destroy_node()
```

先弄清程序何时创建、何时长期运行、如何退出。

### 第二轮：运动命令

```text
_on_cmd_vel()
_send_motion_command()
_send()
```

再同时看：

```text
can_protocol.make_motion_command()
serial_transport.send()
```

### 第三轮：STM32 遥测

```text
_poll_can()
_handle_frame()
```

同时看 `can_protocol.py` 中各 `decode_*()`。

### 第四轮：里程计与发布

```text
_update_odometry_if_pair_ready()
_publish_odometry_and_joints()
_publish_motor_debug()
_publish_diagnostics()
```

### 第五轮：服务和参数

```text
_on_enable()
_on_estop()
_on_clear_faults()
_on_push_parameters()
_send_one_pending_parameter()
_handle_parameter_reply()
_on_parameters_changed()
```

## 37. 阅读每个函数时问自己五个问题

例如 `_send_motion_command()`：

1. 谁调用它？——20 ms ROS 定时器，也会被部分服务立即调用；
2. 输入从哪里来？——节点缓存的最新 `/cmd_vel`、使能和急停状态；
3. 它读取哪些对象属性？——`_latest_command`、`_last_cmd_vel_time` 等；
4. 它调用谁？——`protocol.make_motion_command()` 和 `_send()`；
5. 结果到哪里？——经过传输模块发送给 STM32。

用这种方法比逐句背代码更容易形成模块关系。

## 38. 主节点的安全设计总结

主节点包含以下上位机保护：

- 启动默认软件失能；
- 必须显式调用使能服务；
- `/cmd_vel` 超过 0.20 s 自动发送零速度和失能；
- 急停会撤销软件使能；
- 退出前发送三次失能帧；
- 通信失败自动关闭并定时重开；
- 参数运行中禁止修改；
- 参数有范围检查、应答和最多三次重试；
- 心跳超过 2 秒发布 ERROR；
- STM32 复位后自动重置编码器里程计基准；
- 复位里程计和保存参数要求非运行状态。

它不是唯一安全层。STM32 还有独立的命令超时、急停、欠压、堵转、编码器和参数保护。

## 39. 当前模块的技术边界

主节点已经实现：

- ROS 2 速度接口；
- 串口/CAN 统一传输抽象；
- 协议编解码调用；
- 编码器里程计；
- 电机调试数据；
- 状态诊断；
- 软件使能和急停；
- 参数传输、确认与重试；
- STM32 重启识别。

主节点没有实现：

- 电机 PI 本身，PI 在 STM32；
- PWM 硬件输出；
- IMU 消息 `/imu/data`；
- 编码器与 IMU 融合；
- 完整导航功能；
- 激光雷达处理。

因此它最准确的定位是：

```text
一个把 ROS 2 速度命令安全下发给 STM32，
并把 STM32 底盘遥测转换为 ROS 2 标准接口的桥接节点。
```

## 40. 最后再看一次整体运行图

```text
                     ROS 2 世界

键盘/导航 ── /cmd_vel ──> ChassisCanNode
                              │
              /chassis/enable │ 软件使能、超时、急停检查
                              │
                              ▼
                     can_protocol.py
                     运动命令编码
                              │
                              ▼
                   serial_transport.py
                    15 字节串口封装
                              │
========================== USB 串口 ==========================
                              │
                              ▼
                     STM32 实时控制
             运动学 → 编码器速度 → 前馈+PI → PWM
                              │
                轮速/编码器/状态/心跳回传
                              │
========================== USB 串口 ==========================
                              │
                              ▼
                   serial_transport.py
                              │
                              ▼
                     can_protocol.py
                        遥测解码
                              │
                              ▼
                      ChassisCanNode
                ┌─────────────┼─────────────┐
                ▼             ▼             ▼
              /odom     /diagnostics   /motor/debug
                │
                ├── /joint_states
                └── TF: odom → base_link
```

理解这张图后，再回头看 `chassis_can_node.py`，每个函数都可以放到某一条箭头上，而不是 722 行互不相关的 Python 代码。

# 上位机第 6 模块：ROS 状态与调试数据模块

> 本文件是该模块在旧任务中的完整归档：保留首次讲解正文，并把首次文档之后的专项追问统一融合在同一文件中。以后无需回到原任务查找。

## 本模块归档内容

- 首次讲解来源：`docs/17_ROS2状态与调试数据模块零基础详解.md`
- 首次文档之后没有新增专项追问。

---

# 第一部分：首次完整讲解

# ROS 2 状态与调试数据模块零基础详解

## 1. 这份文档专门讲哪个模块

这份文档专门讲 RDK X5 上位机的第 6 个模块：**ROS 状态与调试数据模块**。

对应的主要代码位于：

```text
ros2_ws/src/chassis_can_control/chassis_can_control/chassis_can_node.py
```

它发布五类 ROS 2 数据：

```text
/diagnostics     底盘是否正常、有什么故障、心跳是否存在
/motor/debug     左右轮目标速度、实测速度、PWM 和误差
/odom            小车在里程计坐标系中的位置、姿态和速度
/joint_states    左右车轮转过的角度和当前角速度
/tf              odom 坐标系到 base_link 坐标系的变换
```

这个模块本身不直接控制 PWM，也不直接读取 STM32 寄存器。它的工作是：

```text
接收前面模块已经解码好的 Python 数据
        ↓
整理成 ROS 2 标准消息
        ↓
通过 ROS 2 话题或 TF 发布出去
        ↓
终端、RViz、导航节点或其他程序订阅使用
```

## 2. 它和前五个模块是什么关系

你已经大致理解了前五个模块，可以把第 6 个模块看成它们的“输出展示层”。

完整数据链：

```text
STM32 编码器、电机控制器和状态机
        ↓ 生成逻辑报文
STM32 串口发送
        ↓ 15 字节串口帧
模块 4：serial_transport.py
        ↓ 找帧头、校验并恢复 CanFrame
模块 3：can_protocol.py
        ↓ 按 logical ID 解码成 Python 数据对象
模块 1：chassis_can_node.py 的 _handle_frame()
        ↓ 保存最新数据并调用发布函数
模块 5：wheel_odometry.py
        ↓ 由编码器计算 x、y、yaw
模块 6：状态与调试数据发布
        ↓
/diagnostics、/motor/debug、/odom、/joint_states、/tf
```

对应关系：

| 模块 6 的输出 | 直接数据来源 | 间接依赖 |
|---|---|---|
| `/motor/debug` | 轮速帧、PWM/误差帧 | STM32 电机闭环、协议解码 |
| `/diagnostics` | 状态帧、心跳帧、RDK 本地通信状态 | STM32 状态机、安全模块、参数模块 |
| `/odom` | 左右编码器累计计数、最新轮速 | 协议解码、`wheel_odometry.py`、几何参数 |
| `/joint_states` | 编码器累计计数、最新轮速 | 轮径和每圈计数参数 |
| `/tf` | 里程计的 x、y、yaw | `/odom` 同一份计算结果 |

所以模块 6 不是孤立的。前面的串口、协议或里程计出错，模块 6 发布的数据自然也会异常。

## 3. 先理解 ROS 2 的“发布—订阅”

### 3.1 什么是话题

ROS 2 话题可以理解成一个有名字的数据频道：

```text
发布者 Publisher  →  /diagnostics  →  订阅者 Subscriber
```

底盘节点负责发布。终端命令、RViz、导航程序或你以后写的节点都可以订阅。

发布者不关心当前有几个订阅者，也不等待订阅者回答。它只负责按照消息格式发布。

### 3.2 话题和服务的区别

话题适合连续数据：

```text
轮速、里程计、状态、传感器数据
```

服务适合一次请求、一次响应：

```text
请使能底盘 → 成功
请清除故障 → 已发送
```

因此：

```text
/motor/debug       是话题
/chassis/enable    是服务
```

### 3.3 消息类型是什么

ROS 2 不能只说“发布一组数据”，还必须约定结构。例如：

```text
/odom 的类型是 nav_msgs/msg/Odometry
/joint_states 的类型是 sensor_msgs/msg/JointState
```

类型决定消息有哪些字段、字段是什么类型。发布者和订阅者必须使用相同类型。

查看当前话题及类型：

```bash
ros2 topic list -t
```

查看某个消息类型的结构：

```bash
ros2 interface show nav_msgs/msg/Odometry
ros2 interface show diagnostic_msgs/msg/DiagnosticArray
```

## 4. 先理解相关 Python 基础

### 4.1 `self` 是什么

代码：

```python
self._latest_status = None
```

`self` 代表当前这个 `ChassisCanNode` 对象。`_latest_status` 是这个节点对象保存的一项状态。

可以把对象想成一个柜子：

```text
ChassisCanNode 对象
├── _latest_status
├── _latest_heartbeat
├── _latest_speed
├── _latest_output
├── _left_encoder
├── _right_encoder
└── 多个 publisher
```

### 4.2 `None` 为什么很多

节点刚启动时还没有收到 STM32 数据：

```python
self._latest_heartbeat = None
self._latest_status = None
self._latest_speed = None
```

`None` 表示“现在还没有有效对象”。收到并成功解码后，才会变成：

```text
Heartbeat 对象
StatusData 对象
WheelSpeed 对象
```

因此发布函数经常先检查：

```python
if self._latest_speed is None:
    return
```

这表示数据不完整时暂时不发布，避免访问不存在的字段。

### 4.3 创建消息对象

```python
message = Float32MultiArray()
```

这一行调用消息类，创建一个空的消息对象。随后填写：

```python
message.data = [1.0, 2.0]
```

最后发布：

```python
self._debug_publisher.publish(message)
```

共同模式是：

```text
创建消息 → 填字段 → publish()
```

### 4.4 点号和多层字段

```python
odom.pose.pose.position.x = sample.x_m
```

不要被多个点吓到。它只是逐层进入结构：

```text
odom
└── pose
    └── pose
        └── position
            └── x
```

为什么有两个 `pose`？外层 `pose` 是带协方差的 `PoseWithCovariance`，内层 `pose` 才是真正的位置和姿态。

### 4.5 列表

```python
message.data = [
    left_target,
    left_measured,
    right_target,
    right_measured,
]
```

方括号表示列表，元素按固定顺序排列。`/motor/debug` 没有字段名，因此阅读者必须知道每个下标含义。

### 4.6 f-string

```python
value=f"{heartbeat_age:.3f} s"
```

`f` 表示字符串中可以插入变量；`:.3f` 表示保留 3 位小数。例如：

```text
0.11472 → "0.115 s"
```

### 4.7 `append()` 与 `extend()`

加入一个元素：

```python
status.values.append(KeyValue(...))
```

一次加入多个元素：

```python
status.values.extend([item1, item2, item3])
```

## 5. 发布者在节点中如何创建

节点初始化时执行：

```python
self._odom_publisher = self.create_publisher(Odometry, "odom", 20)
self._joint_publisher = self.create_publisher(JointState, "joint_states", 20)
self._diagnostic_publisher = self.create_publisher(
    DiagnosticArray, "diagnostics", 10
)
self._debug_publisher = self.create_publisher(
    Float32MultiArray, "motor/debug", 20
)
self._tf_broadcaster = TransformBroadcaster(self)
```

以第一行为例：

```python
self.create_publisher(Odometry, "odom", 20)
```

三个参数：

1. `Odometry`：消息类型；
2. `"odom"`：话题名，最终通常显示为 `/odom`；
3. `20`：QoS 队列深度。

### 5.1 队列深度是什么

如果订阅端一时处理不过来，ROS 可以暂存若干条消息。深度 20 表示保留有限的最近消息，并不是每次发布 20 条。

状态和控制数据更看重“最新值”，历史积压太多通常没有意义。

### 5.2 为什么 TF 不是普通 publisher

TF 有专门的广播器：

```python
TransformBroadcaster(self)
```

使用：

```python
self._tf_broadcaster.sendTransform(transform)
```

底层仍会进入 ROS 话题系统，但 `tf2_ros` 帮你按照 TF 约定发布和管理。

## 6. 这些数据不是同一时刻发布的

这是理解该模块最重要的一点。

### 6.1 `/diagnostics`：定时发布

初始化：

```python
self.create_timer(0.20, self._publish_diagnostics)
```

每 0.20 秒调用一次，也就是约 5 Hz。

即使这 0.20 秒内没有新状态帧，它仍会用保存的最新数据发布，并重新计算心跳已经多久没来。

### 6.2 `/motor/debug`：由轮速/PWM帧触发

收到 `ID_WHEEL_SPEED`：

```python
self._latest_speed = protocol.decode_wheel_speed(frame)
self._publish_motor_debug()
```

收到 `ID_MOTOR_OUTPUT`：

```python
self._latest_output = protocol.decode_motor_output(frame)
self._publish_motor_debug()
```

每次收到其中一类都会尝试发布，但只有两类数据都存在才真正发布。

### 6.3 `/odom`、`/joint_states`、`/tf`：编码器对触发

收到左编码器后保存并尝试配对；收到右编码器后也保存并尝试配对。只有左右 sequence 相同、且这一批尚未使用，才计算和发布。

因此它们三者使用同一批里程计结果和同一个时间戳。

## 7. 数据怎样进入状态与调试模块

串口轮询函数：

```python
def _poll_can(self):
    frames = self._can.receive_available()
    for frame in frames:
        self._handle_frame(frame)
```

虽然函数名仍叫 `_poll_can`，串口模式下 `self._can` 实际是 `SerialTransport`。这是为了统一传输接口。

`_handle_frame()` 是分发中心：

```python
if frame.can_id == protocol.ID_WHEEL_SPEED:
    ...
elif frame.can_id == protocol.ID_MOTOR_OUTPUT:
    ...
elif frame.can_id == protocol.ID_LEFT_ENCODER:
    ...
```

它像快递分拣：根据 logical ID 把数据送到不同处理分支。

### 7.1 为什么要保存“最新值”

轮速帧和 PWM 帧不一定在同一个 Python 调用中到达。所以代码先分别保存：

```python
self._latest_speed
self._latest_output
```

当两者都存在时组合成 `/motor/debug`。

状态和心跳也分别保存，因为诊断需要同时参考：

```python
self._latest_status
self._latest_heartbeat
self._last_heartbeat_time
```

## 8. `/motor/debug` 详解

### 8.1 它解决什么问题

只看小车是否转动，很难知道问题在目标、编码器还是 PWM。`/motor/debug` 把 STM32 电机闭环内部的重要变量上传出来。

它回答：

- STM32 收到的左右目标速度是多少？
- 编码器测到的速度是多少？
- 控制器输出多少 PWM？
- 当前速度误差有多大？

### 8.2 发布代码逐句解释

```python
def _publish_motor_debug(self) -> None:
```

定义节点内部函数，不返回有用结果。

```python
if self._latest_speed is None or self._latest_output is None:
    return
```

轮速和输出任意一个不存在，就直接退出。

```python
message = Float32MultiArray()
```

创建一个浮点数组 ROS 消息。

```python
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
```

按固定顺序填 8 个值。PWM 原本是整数，为满足浮点数组统一转成 `float`。

```python
self._debug_publisher.publish(message)
```

发布到 `/motor/debug`。

### 8.3 数组下标

| 下标 | 数据 | 单位 | 来自哪里 |
|---:|---|---|---|
| 0 | 左轮斜坡后的目标速度 | m/s | STM32 左轮控制器 |
| 1 | 左轮滤波后的实测速度 | m/s | 左编码器 |
| 2 | 右轮斜坡后的目标速度 | m/s | STM32 右轮控制器 |
| 3 | 右轮滤波后的实测速度 | m/s | 右编码器 |
| 4 | 左轮 PWM | PWM 计数 | STM32 PI 输出 |
| 5 | 右轮 PWM | PWM 计数 | STM32 PI 输出 |
| 6 | 左轮速度误差 | m/s | 目标减实测 |
| 7 | 右轮速度误差 | m/s | 目标减实测 |

注意：下标 0/2 是经过加减速斜坡的轮速目标，不一定瞬间等于 ROS `/cmd_vel.linear.x`。

### 8.4 怎样查看

在已加载 ROS 环境的终端：

```bash
ros2 topic echo /motor/debug
```

只看一条：

```bash
ros2 topic echo --once /motor/debug
```

查看频率：

```bash
ros2 topic hz /motor/debug
```

### 8.5 一个例子

```text
data: [0.05, 0.047, 0.05, 0.052, 1160.0, 1090.0, 0.003, -0.002]
```

解释：

- 左右目标都是 0.05 m/s；
- 左轮稍慢 0.003 m/s；
- 右轮稍快 0.002 m/s；
- 左轮 PWM 比右轮略高，控制器在补偿左右差异；
- 如果实测长期远离目标，同时 PWM 顶到约 16700，可能是负载太大、参数不合适、编码器方向错误或堵转。

### 8.6 当前实现的局限

`Float32MultiArray` 容易实现，但消息本身不记录每个下标的名字。更大型项目可定义自定义消息，例如：

```text
left_target_mps
left_measured_mps
left_pwm
...
```

当前项目为了代码简单使用数组，必须依赖文档记住顺序。

## 9. `/diagnostics` 详解

### 9.1 为什么需要诊断话题

`/motor/debug` 更关心控制器数值；`/diagnostics` 更关心系统健康：

- STM32 是否在线；
- 当前是 IDLE、RUNNING 还是 FAULT；
- 有哪些故障位；
- 电池电压是多少；
- 控制任务有没有超时；
- 使用哪个传输接口；
- 参数传输是否完成。

### 9.2 消息层级

```text
DiagnosticArray
├── header
└── status[]
    └── DiagnosticStatus
        ├── level
        ├── name
        ├── message
        ├── hardware_id
        └── values[]
            └── KeyValue(key, value)
```

本项目每次只在 `status[]` 中放一个底盘状态，但标准消息允许一个数组同时描述多块硬件。

### 9.3 创建外层和状态对象

```python
array = DiagnosticArray()
array.header.stamp = self.get_clock().now().to_msg()
status = DiagnosticStatus()
status.name = "C30D motor controller"
status.hardware_id = "STM32F407-C30D"
```

`header.stamp` 是 RDK 生成该 ROS 消息的时间。

`name` 是诊断项目名，`hardware_id` 标识硬件类别。

### 9.4 心跳年龄怎样计算

收到有效心跳时保存：

```python
self._last_heartbeat_time = time.monotonic()
```

发布诊断时：

```python
heartbeat_age = time.monotonic() - self._last_heartbeat_time
```

它代表“距离最后一帧心跳已经过去多少秒”。STM32 正常每 1 秒发送一帧，所以通常小于约 1 秒；考虑调度和通信，代码以 2 秒为故障门限。

### 9.5 为什么使用 `time.monotonic()`

它是单调时钟，只保证持续向前，适合测时间间隔。若使用墙上时钟，系统联网校时后时间可能突然跳变，心跳年龄就可能变负数或突然很大。

### 9.6 诊断分支

第一种：没有心跳或超过 2 秒：

```python
status.level = DiagnosticStatus.ERROR
status.message = "STM32 heartbeat missing"
```

第二种：有心跳，但还没有状态帧：

```python
status.level = DiagnosticStatus.WARN
status.message = "Heartbeat received, status frame missing"
```

第三种：心跳和状态帧都有，检查故障位。

### 9.7 level 数值

| 数值 | Python 常量 | 含义 | ROS 输出可能显示 |
|---:|---|---|---|
| 0 | `DiagnosticStatus.OK` | 正常 | `"\0"` |
| 1 | `DiagnosticStatus.WARN` | 警告 | `"\x01"` |
| 2 | `DiagnosticStatus.ERROR` | 错误 | `"\x02"` |

你之前看到的：

```text
level: "\0"
message: IDLE
```

就是 level 数值 0，不是乱码。

### 9.8 状态名称

协议字典：

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

```python
state_name = protocol.STATE_NAMES.get(
    self._latest_status.state,
    f"UNKNOWN({self._latest_status.state})",
)
```

`dict.get(key, default)`：找到就返回名称，找不到就返回 `UNKNOWN(数值)`，避免未知固件状态使 Python 报错退出。

### 9.9 故障位和位掩码

一个 16 位整数可以同时表示多个故障：

```text
0x0001 COMMAND_TIMEOUT
0x0002 ESTOP
0x0004 LOW_BATTERY
0x0020 LEFT_STALL
...
```

位或 `|` 可以组合故障，位与 `&` 可以检查某一位。

```python
faults & 0x0004
```

非零表示包含欠压故障。

`fault_text()` 遍历字典，把所有置位名称用 `|` 拼接，例如：

```text
LOW_BATTERY|LEFT_STALL
```

### 9.10 为什么某些故障是 WARN

当前代码只要发现故障字中包含以下任意一位，就进入警告分支：

```text
COMMAND_TIMEOUT
HARDWARE_DISABLED
CONTROL_OVERRUN
```

如果完全不包含这三位、但存在其他故障，才判为 ERROR。

这里要特别注意代码分支顺序：假设 `COMMAND_TIMEOUT` 和 `LOW_BATTERY` 同时存在，
当前写法会因为先匹配到 `COMMAND_TIMEOUT` 而显示 WARN，即使 `LOW_BATTERY` 本应属于严重故障。
所以实际操作不能只看 `level`，还必须阅读完整的 `faults` 字符串。这是当前诊断分级逻辑的一个可改进点，
更严谨的写法应该先检查严重故障，再检查警告故障。

这意味着 `WARN` 不等于“可以忽略”。例如命令超时已经导致停止，只是它可能是操作者停止发命令产生的可恢复状态。

### 9.11 KeyValue 字段逐项解释

```text
state
```

STM32 状态机名称。

```text
faults
```

故障位翻译后的字符串。

```text
battery_voltage
```

STM32 上报 mV，Python 除以 1000 并格式化为 V。

```text
last_command_sequence
```

STM32 最后接收并保存的运动命令序号。可辅助判断下发链路是否持续更新。

```text
control_overruns
```

STM32 100 Hz 控制周期异常的累计计数，正常应为 0。

```text
transport
```

当前为 `serial`，以后 CAN 版可能为 `can`。

```text
interface
```

实际串口路径，例如 `...0002-if00`。

```text
heartbeat_age
```

最后心跳到现在的间隔。

```text
pending_parameters
```

还有多少参数等待传输或确认。

```text
parameter_transfer_failed
```

参数传输是否在三次重试后失败。

注意：后两个参数字段正常，不能证明电机通信整体正常。必须同时看 `level`、`message` 和 `heartbeat_age`。

### 9.12 最后怎样发布

```python
array.status.append(status)
self._diagnostic_publisher.publish(array)
```

先把单个 `DiagnosticStatus` 放进数组，再发布整个 `DiagnosticArray`。

### 9.13 正常例子

```text
level: "\0"
message: IDLE
state: IDLE
faults: NONE
battery_voltage: 10.600 V
control_overruns: 0
transport: serial
heartbeat_age: 0.115 s
pending_parameters: 0
parameter_transfer_failed: False
```

解释：STM32 在线、串口正常、底盘空闲、无故障。

### 9.14 异常例子

```text
level: "\x02"
message: STM32 heartbeat missing
heartbeat_age: 75.309 s
```

解释：RDK 已 75 秒没有收到有效心跳。可能原因：

- STM32 掉电或复位失败；
- USB 串口拔掉；
- 串口路径错误；
- 固件协议不匹配；
- 两个节点同时抢读串口；
- 串口数据严重损坏。

此时不要使能电机。

## 10. `/odom` 详解

### 10.1 `/odom` 是什么

`/odom` 是轮式里程计估计。它描述：

- 小车从里程计起点走到了哪里；
- 当前朝向；
- 当前线速度和角速度。

当前 `/odom` 只由左右编码器计算，没有融合 IMU、激光或视觉。

### 10.2 数据触发流程

收到左编码器：

```python
self._left_encoder = decoded
self._update_odometry_if_pair_ready()
```

收到右编码器也是相同逻辑。

配对条件：

```python
if left is None or right is None or left.sequence != right.sequence:
    return
```

为什么检查 sequence？左右帧可能前后到达，甚至丢一帧。只有相同序号才表示来自 STM32 同一批采样。

还要防止重复处理：

```python
if self._last_odometry_sequence == left.sequence:
    return
```

### 10.3 调用里程计模块

```python
sample = self._odometry.update(
    left.total_count,
    right.total_count,
    diameter,
    counts_per_rev,
    wheel_track,
)
```

输入：

- 左右累计编码器计数；
- 轮径；
- 每轮一圈计数；
- 左右轮距。

输出 `OdometrySample`：

```text
x_m
y_m
yaw_rad
left_wheel_angle_rad
right_wheel_angle_rad
```

### 10.4 创建时间戳和坐标系

```python
stamp = self.get_clock().now().to_msg()
odom_frame = "odom"
base_frame = "base_link"
```

这里的时间戳是 RDK 发布时刻，不是 STM32 采样的硬件时间戳。对于当前低速项目足够简单，但高精度融合时需要更严格的时间同步设计。

### 10.5 yaw 转四元数

ROS 姿态使用四元数。平面小车只有绕 Z 轴的 yaw：

```python
half_yaw = 0.5 * sample.yaw_rad
quaternion_z = math.sin(half_yaw)
quaternion_w = math.cos(half_yaw)
```

X、Y 分量保持默认 0。

例如 yaw 为 90°，即 `π/2`：

```text
z = sin(π/4) ≈ 0.7071
w = cos(π/4) ≈ 0.7071
```

### 10.6 填写位姿

```python
odom = Odometry()
odom.header.stamp = stamp
odom.header.frame_id = odom_frame
odom.child_frame_id = base_frame
odom.pose.pose.position.x = sample.x_m
odom.pose.pose.position.y = sample.y_m
odom.pose.pose.orientation.z = quaternion_z
odom.pose.pose.orientation.w = quaternion_w
```

解释：

```text
header.frame_id = odom
```

表示位姿是相对于 `odom` 坐标系描述的。

```text
child_frame_id = base_link
```

表示被描述的对象是底盘坐标系。

### 10.7 填写速度

如果已经收到轮速：

```text
车体线速度 = (左实测 + 右实测) / 2
车体角速度 = (右实测 - 左实测) / 轮距
```

代码：

```python
odom.twist.twist.linear.x = 0.5 * (left + right)
odom.twist.twist.angular.z = (right - left) / wheel_track
```

位姿来自累计编码器计数，速度来自 STM32 上报的滤波轮速，两者数据源相关但形式不同。

### 10.8 当前没有填写 covariance

`Odometry` 消息还包含位姿和速度协方差，用来表示不确定度。当前代码保持默认全零。

在严格的传感器融合中，全零可能被解释为“非常确定”，并不理想。后续接 `robot_localization` 时应根据编码器误差、打滑情况填写合理协方差。

### 10.9 查看命令

```bash
ros2 topic echo /odom
ros2 topic hz /odom
```

只看位置字段：

```bash
ros2 topic echo /odom --field pose.pose.position
```

## 11. `/joint_states` 详解

### 11.1 它的用途

`/joint_states` 描述机器人各关节的：

```text
名称 name
角度 position
角速度 velocity
力/力矩 effort
```

当前只发布左右车轮关节，用于 robot_state_publisher、RViz 轮子动画或其他机器人模型组件。

### 11.2 填写代码

```python
joint = JointState()
joint.header.stamp = stamp
joint.name = ["left_wheel_joint", "right_wheel_joint"]
joint.position = [left_angle, right_angle]
```

列表顺序必须严格对应：

```text
name[0] 对应 position[0] 和 velocity[0]
name[1] 对应 position[1] 和 velocity[1]
```

### 11.3 线速度转角速度

圆周线速度公式：

```text
v = ωr
```

所以：

```text
ω = v/r
```

代码：

```python
radius = 0.5 * wheel_diameter
joint.velocity = [left_mps / radius, right_mps / radius]
```

单位是 rad/s。

### 11.4 为什么 `effort` 没填

当前硬件没有电机电流或转矩传感器，也没有可靠的转矩估计，因此保持为空。PWM 不是力矩，不能直接作为 `effort` 填进去。

### 11.5 查看

```bash
ros2 topic echo /joint_states
```

## 12. `/tf` 详解

### 12.1 坐标系为什么重要

机器人系统中，不同数据在不同坐标系：

```text
odom       里程计起始参考系
base_link  小车底盘中心坐标系
laser      激光雷达坐标系
imu_link   IMU 坐标系
```

如果不知道坐标系关系，无法把雷达点、小车位置和地图放到一起。

### 12.2 当前发布的变换

```text
父坐标系：odom
子坐标系：base_link
```

它表示底盘相对于里程计起点的位置和朝向。

### 12.3 发布条件

```python
if bool(self.get_parameter("publish_tf").value):
```

YAML 中 `publish_tf: true` 时发布；设为 `false` 时停止，避免系统中另一个定位节点也发布同一变换造成冲突。

### 12.4 填写 TransformStamped

```python
transform = TransformStamped()
transform.header.stamp = stamp
transform.header.frame_id = odom_frame
transform.child_frame_id = base_frame
transform.transform.translation.x = sample.x_m
transform.transform.translation.y = sample.y_m
transform.transform.rotation.z = quaternion_z
transform.transform.rotation.w = quaternion_w
```

它和 `/odom` 使用同一个 `sample`、同一个时间戳和同一个四元数，因此两者应该一致。

### 12.5 查看 TF

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

查看 TF 树，可尝试：

```bash
ros2 run tf2_tools view_frames
```

该命令需要系统已安装对应工具，通常生成 `frames.pdf`。

## 13. 五种输出怎样相互验证

不要孤立看一个话题。可以交叉检查：

### 情况一：小车静止且健康

```text
/diagnostics  IDLE / NONE
/motor/debug  目标、实测、PWM 接近 0
/odom         位置基本不变
/joint_states 角度基本不变
/tf           odom→base_link 基本不变
```

### 情况二：低速直行

```text
/diagnostics  RUNNING / NONE
/motor/debug  左右目标相近，实测跟随目标
/odom         x 持续变化，yaw 变化较小
/joint_states 左右轮角度持续变化
/tf           base_link 沿 odom X 方向移动
```

### 情况三：编码器方向错

可能看到：

```text
PWM 正向增加
某一轮实测速度为负
速度误差持续很大
/odom 方向异常
最终可能编码器或堵转故障
```

### 情况四：重复节点抢串口

可能看到：

```text
存在两个 /chassis_can_node 或两个进程
heartbeat_age 不断增大
STM32 heartbeat missing
/motor/debug、/odom 数据断断续续
```

## 14. 在 Ubuntu/RDK 终端中怎样操作

### 14.1 先登录 RDK

在 Ubuntu 虚拟机终端：

```bash
ssh wheeltec@192.168.0.100
```

如果 IP 已变化，应先在 RDK 本机运行：

```bash
hostname -I
```

### 14.2 每个新终端加载环境

```bash
source /opt/ros/humble/setup.bash
source ~/chassis_project/ros2_ws/install/setup.bash
```

第一行加载系统 ROS 2 Humble；第二行加载本项目工作空间。

### 14.3 确认节点只运行一个

```bash
pgrep -af 'chassis_serial.launch.py|chassis_can_node'
```

正常是一套 launch 进程加一个 Python node。若已经运行，不要再次执行启动脚本。

### 14.4 查看话题

```bash
ros2 topic list -t
```

确认至少存在：

```text
/diagnostics
/motor/debug
/odom
/joint_states
/tf
```

### 14.5 查看节点发布了什么

```bash
ros2 node info /chassis_can_node
```

在 `Publishers` 部分可看到话题及类型。

### 14.6 查看一条诊断

```bash
ros2 topic echo --once /diagnostics
```

`--once` 表示收到一条后退出。

### 14.7 持续观察电机调试数据

```bash
ros2 topic echo /motor/debug
```

按 `Ctrl+C` 只停止终端显示，不会停止后台底盘节点。

### 14.8 查看发布频率

```bash
ros2 topic hz /diagnostics
ros2 topic hz /motor/debug
ros2 topic hz /odom
```

预期大致：

```text
/diagnostics 约 5 Hz
/motor/debug 约 50～100 Hz，取决于两类帧触发方式和实际调度
/odom 约 20 Hz
```

`/motor/debug` 当前在轮速帧和输出帧到达时都可能发布，所以实际频率可能高于每类单独的 50 Hz，并且某一帧可能搭配上一时刻的另一类最新数据。这是“最新值组合”设计的结果。

### 14.9 查看话题信息

```bash
ros2 topic info /diagnostics --verbose
ros2 topic info /odom --verbose
```

可看到发布者、订阅者和 QoS 信息。

### 14.10 记录数据

如果要保留实验数据：

```bash
mkdir -p ~/chassis_project/bags
cd ~/chassis_project/bags
ros2 bag record /diagnostics /motor/debug /odom /joint_states /tf
```

按 `Ctrl+C` 正常结束记录。回放前先保持电机失能，回放这些输出话题本身不会控制电机，但不要在同一 ROS 域中混淆实时数据和回放数据。

查看 bag 信息：

```bash
ros2 bag info 生成的目录名
```

回放：

```bash
ros2 bag play 生成的目录名
```

## 15. 这个模块怎样与 RViz 和导航连接

### 15.1 RViz

RViz 可以显示：

- TF 坐标轴；
- Odometry 轨迹或箭头；
- RobotModel，需要 URDF 和 `/joint_states`；
- 后续激光雷达点云。

当前模块提供了底层基础数据，但完整 RobotModel 还需要 URDF 和静态 TF。

### 15.2 导航

导航通常需要：

```text
/cmd_vel       导航输出给底盘
/odom          底盘运动估计
odom→base_link TF
激光/视觉数据
map→odom TF，由定位系统提供
```

当前模块提供 `/odom` 和 `odom→base_link`，但尚不等于完整导航已经完成。

### 15.3 将来接 IMU

当前没有 `/imu/data`。以后若实现：

```text
STM32 IMU 原始数据
→ 协议编码
→ Python 解码
→ sensor_msgs/msg/Imu
→ /imu/data
→ robot_localization 与 /odom 融合
```

融合后可能由 `robot_localization` 发布新的融合里程计和 TF。那时要避免原节点和融合节点同时发布相同 `odom→base_link`。

## 16. 常见问题与排查顺序

### 16.1 话题不存在

```bash
ros2 node list
ros2 topic list
```

若节点不存在，检查进程和日志：

```bash
pgrep -af 'chassis_serial.launch.py|chassis_can_node'
tail -100 ~/chassis_project/chassis_serial.log
```

### 16.2 `/diagnostics` 有数据，但 `/motor/debug` 没数据

说明至少 ROS 节点和诊断定时器在运行，但轮速/PWM 遥测可能没有完整收到。检查：

- 心跳是否正常；
- 串口是否被两个进程占用；
- STM32 固件版本是否匹配；
- 日志是否有串口接收错误。

### 16.3 `/motor/debug` 有数据，但 `/odom` 没更新

轮速帧正常不代表左右编码器帧都正常。检查：

- `/odom` 是否有话题但数值不变；
- 左右编码器 sequence 是否能配对；
- 轮径、轮距、每圈计数是否有效；
- STM32 是否回传累计编码器。

### 16.4 `/odom` 漂移

轮式里程计会受：

- 轮径误差；
- 轮距误差；
- 左右轮不一致；
- 地面打滑；
- 阿克曼模型近似；
- 编码器计数标定误差。

当前没有 IMU/激光融合，长时间漂移是预期现象，不一定是 Python 代码故障。

### 16.5 TF 冲突或抖动

检查是否有多个节点发布 `odom→base_link`：

```bash
ros2 topic info /tf --verbose
```

如果后续加入定位融合，应决定由哪一个节点负责这条 TF，并将另一个节点的 `publish_tf` 关闭。

### 16.6 `heartbeat_age` 正常但 faults 不为 NONE

这表示通信正常，但 STM32 确实报告了控制故障。不要把通信恢复等同于故障已经消失。根据 `faults` 排查电池、编码器、堵转、急停或参数。

## 17. 建议你怎样学习这部分代码

按以下顺序，不要一次看完整个 `chassis_can_node.py`：

1. 看第 136～144 行：发布者怎样创建；
2. 看第 250～306 行：帧怎样分发和保存；
3. 看第 391～405 行：最简单的 `/motor/debug`；
4. 看第 407～470 行：分支较多的 `/diagnostics`；
5. 回看 `can_protocol.py` 中四种解码后的数据类；
6. 看第 308～330 行：左右编码器配对；
7. 看 `wheel_odometry.py`：位姿怎样计算；
8. 看第 332～389 行：同一结果怎样变成 odom、joint 和 TF；
9. 在终端逐个 `echo`，把代码字段和实际输出对上。

阅读每个发布函数时回答：

```text
谁触发它？
它依赖哪些最新数据？
数据不完整时怎样处理？
创建什么 ROS 消息？
填写了哪些字段？
发布到哪个话题？
其他节点怎样使用？
```

## 18. 一次不让电机转动的学习实验

这个实验只观察状态，不使能电机。

### 终端 A：确认节点

```bash
ssh wheeltec@RDK的实际IP
source /opt/ros/humble/setup.bash
source ~/chassis_project/ros2_ws/install/setup.bash
ros2 node list
ros2 node info /chassis_can_node
```

### 终端 B：观察诊断

```bash
ssh wheeltec@RDK的实际IP
source /opt/ros/humble/setup.bash
source ~/chassis_project/ros2_ws/install/setup.bash
ros2 topic echo /diagnostics
```

观察 `heartbeat_age` 从约 0 增长到约 1 秒，再因新心跳回到较小值。

### 终端 C：观察其他话题

```bash
ssh wheeltec@RDK的实际IP
source /opt/ros/humble/setup.bash
source ~/chassis_project/ros2_ws/install/setup.bash
ros2 topic echo /motor/debug
```

底盘失能时，目标和 PWM 应接近 0。手动轻转架空车轮时，编码器实测值和 `/joint_states` 可能变化，但具体取决于驱动机构和硬件状态。

依次查看：

```bash
ros2 topic echo --once /odom
ros2 topic echo --once /joint_states
ros2 run tf2_ros tf2_echo odom base_link
```

这个实验不调用 `/chassis/enable`，不会主动让电机转动。

## 19. 当前模块的实现优点与改进方向

### 已实现的优点

- 使用 ROS 标准 Odometry、JointState 和 Diagnostic 消息；
- 心跳与状态分开判断；
- 编码器按 sequence 配对；
- STM32 重启时重置 RDK 里程计基准；
- 同一里程计样本同步生成 odom、joint 和 TF；
- 将电机闭环内部量通过 `/motor/debug` 暴露出来；
- 支持串口和 CAN 复用同一发布逻辑。

### 可继续改进

- 为 `/motor/debug` 定义带字段名的自定义消息；
- 给 `/odom` 填写合理协方差；
- 在编码器和状态数据中传输 STM32 采样时间戳；
- 在诊断中增加串口 CRC 错误和队列溢出计数；
- 调整诊断故障判断顺序，让任意严重故障都优先显示 ERROR；
- 增加明确的 RDK 软件使能状态字段；
- 完成 `/imu/data` 和传感器融合；
- 用 systemd 管理唯一后台节点，避免重复启动；
- 使用诊断聚合器在界面中分组显示。

## 20. 最后用一句话概括每个输出

```text
/diagnostics  告诉你“系统健康不健康，为什么”
/motor/debug  告诉你“电机闭环现在算出了什么”
/odom         告诉你“小车估计自己走到了哪里、速度多大”
/joint_states 告诉你“左右车轮分别转到了哪里、转多快”
/tf           告诉整个 ROS 系统“base_link 相对 odom 在哪里”
```

它们都不是凭空产生的，而是把 STM32 的轮速、PWM、编码器、状态和心跳，经过协议解码与里程计计算后，转换成 ROS 2 能统一理解的数据形式。

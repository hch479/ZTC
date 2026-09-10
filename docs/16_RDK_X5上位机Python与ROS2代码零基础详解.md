# RDK X5 上位机 Python 与 ROS 2 代码零基础详解

## 1. 这份文档的目标

这份文档面向 Python 基础薄弱、第一次接触 ROS 2 的读者。讲解对象是实际部署在 RDK X5 上的功能包：

```text
/home/wheeltec/chassis_project/ros2_ws/src/chassis_can_control
```

你读完后应能回答：

1. Python 程序从哪里启动；
2. ROS 2 节点怎样接收 `/cmd_vel`；
3. Python 怎样把速度编码为串口字节；
4. STM32 遥测怎样变成 `/odom` 和 `/diagnostics`；
5. 话题、服务、参数和 launch 各自是什么；
6. 上下位机为什么能理解同一个报文；
7. Ubuntu 终端中每条常用命令是什么意思；
8. 为什么后台节点看不见窗口却仍在运行；
9. 当前 IMU 在整个系统中的真实状态。

## 2. 上位机和下位机如何分工

```text
导航/键盘/其他 ROS 节点
        ↓ /cmd_vel
RDK X5：Python ROS 2 节点
        ↓ 15 字节串口帧
STM32：实时控制与安全
        ↓ PWM
电机

编码器
  ↓
STM32 计算轮速、状态、累计计数
  ↓ 15 字节串口帧
RDK Python 解码
  ├─/motor/debug
  ├─/odom
  ├─/joint_states
  ├─/tf
  └─/diagnostics
```

为什么不把全部电机控制放在 RDK？Linux 和 Python 适合网络、ROS 和复杂算法，但不是严格硬实时系统。STM32 的 100 Hz 控制任务更稳定，并且在 RDK 程序卡住时还能独立执行超时停机。

为什么不把全部 ROS 功能放在 STM32？STM32 内存和算力有限，运行完整 ROS 2、导航、可视化和日志并不合适。

## 3. 先学会看 Python

### 3.1 缩进就是语法

C 用大括号表示代码块：

```c
if (ready) {
    run();
}
```

Python 用缩进：

```python
if ready:
    run()
```

同一个代码块通常统一缩进 4 个空格。缩进错误会直接导致语法错误或逻辑改变。

### 3.2 变量不需要写类型

```python
speed = 0.05
name = "chassis_can_node"
enabled = False
```

Python 会在运行时记录类型。项目仍使用类型提示帮助阅读：

```python
def crc8(data: bytes) -> int:
```

它表示希望 `data` 是字节串，返回整数；通常不是强制检查。

### 3.3 `None`、`True` 和 `False`

```text
None   没有对象/尚无数据
True   真
False  假
```

例如节点启动时还没收到心跳：

```python
self._latest_heartbeat = None
```

### 3.4 列表、元组和字典

列表可修改：

```python
values = [1, 2, 3]
```

元组通常表达固定组合：

```python
minimum, maximum = (0.0, 50000.0)
```

字典用键查值：

```python
STATE_NAMES = {
    1: "IDLE",
    2: "RUNNING",
}
```

### 3.5 函数

```python
def fault_text(faults: int) -> str:
    ...
```

`def` 定义函数，括号中是参数，`return` 返回结果。

### 3.6 类与对象

```python
class WheelOdometry:
    def __init__(self):
        self.x_m = 0.0
```

类像设计图，对象是按设计图创建出的实例：

```python
odometry = WheelOdometry()
```

`self` 指当前对象本身。`self.x_m` 是每个对象自己的状态。

### 3.7 `__init__`

对象创建时自动调用：

```python
node = ChassisCanNode()
```

这会进入：

```python
def __init__(self):
```

项目在其中声明参数、打开串口、创建 ROS 话题/服务/定时器。

### 3.8 `dataclass`

```python
@dataclass
class Heartbeat:
    uptime_ms: int
    state: int
    firmware_major: int
    firmware_minor: int
```

它是专门保存数据的简洁类。解码后可以写：

```python
heartbeat.uptime_ms
heartbeat.state
```

不用到处记 `list[0]`、`list[1]`。

### 3.9 `Optional`

```python
self._latest_speed: Optional[WheelSpeed] = None
```

表示该变量可能是一个 `WheelSpeed`，也可能暂时是 `None`。

### 3.10 下划线命名

```python
self._send_motion_command()
```

单下划线是一种约定，表示“类内部使用”，不是绝对禁止外部访问。

### 3.11 异常处理

```python
try:
    self._can.open()
except OSError as error:
    self.get_logger().error(str(error))
```

如果打开串口失败，Python 不会直接让整个节点毫无说明地退出，而是进入 `except` 记录错误。节点随后每秒重试打开。

## 4. ROS 2 最基础的五个概念

### 4.1 节点 Node

节点是一个长期运行、参与 ROS 网络的程序。本项目节点名：

```text
/chassis_can_node
```

查看：

```bash
ros2 node list
ros2 node info /chassis_can_node
```

### 4.2 话题 Topic

话题适合连续数据流。发布者不断发布，订阅者被动接收：

```text
/cmd_vel      其他节点 → 底盘节点
/odom         底盘节点 → 导航/显示
/diagnostics  底盘节点 → 操作者
```

发布者不需要等待每个订阅者回复。

### 4.3 服务 Service

服务适合一次请求、一次响应：

```text
请求：请使能底盘
响应：成功，drive enabled
```

本项目的 `/chassis/enable`、`/chassis/clear_faults` 都是服务。

### 4.4 参数 Parameter

参数是节点配置，例如串口路径、轮径和 PI：

```text
serial_port
geometry.wheel_diameter_m
controller.left_kp
```

### 4.5 TF 坐标变换

TF 描述坐标系之间的空间关系。本项目发布：

```text
odom → base_link
```

意思是底盘坐标系 `base_link` 当前在里程计坐标系 `odom` 的哪个位置和方向。

## 5. ROS 工作空间与功能包目录

```text
ros2_ws/
├── src/chassis_can_control/
│   ├── chassis_can_control/
│   │   ├── chassis_can_node.py
│   │   ├── can_protocol.py
│   │   ├── serial_transport.py
│   │   ├── socketcan_transport.py
│   │   └── wheel_odometry.py
│   ├── config/chassis_serial.yaml
│   ├── launch/chassis_serial.launch.py
│   ├── scripts/
│   ├── test/
│   ├── package.xml
│   ├── setup.py
│   └── setup.cfg
├── build/
├── install/
└── log/
```

含义：

- `src`：你编辑的源码；
- `build`：编译过程的中间文件；
- `install`：ROS 实际查找和运行的安装结果；
- `log`：编译日志。

修改源码 YAML 后只重启不一定生效，因为 launch 从 `install/.../share/.../config` 读取安装副本。应重新 `colcon build`。

## 6. Python 程序从哪里启动

`setup.py` 中：

```python
entry_points={
    "console_scripts": [
        "chassis_can_node = chassis_can_control.chassis_can_node:main",
    ],
}
```

含义：ROS 执行 `chassis_can_node` 时，调用：

```text
chassis_can_control/chassis_can_node.py 中的 main()
```

`main()`：

```python
def main(args=None):
    rclpy.init(args=args)
    node = ChassisCanNode()
    rclpy.spin(node)
```

逐句解释：

1. `rclpy.init()` 初始化 ROS 2 Python 客户端；
2. `ChassisCanNode()` 创建节点对象并完成所有初始化；
3. `rclpy.spin(node)` 进入事件循环；
4. 之后回调函数在收到消息、服务请求或定时器到期时被调用；
5. `Ctrl+C` 后销毁节点、发送几次禁用命令并关闭串口。

Python 代码不是从上到下执行一次就结束，它初始化后长期停在事件循环中。

## 7. launch 文件怎样启动节点

文件：

```text
launch/chassis_serial.launch.py
```

它先找到安装后的包目录：

```python
package_directory = get_package_share_directory("chassis_can_control")
```

再拼接参数文件路径：

```python
parameter_file = os.path.join(
    package_directory, "config", "chassis_serial.yaml"
)
```

最后创建节点：

```python
Node(
    package="chassis_can_control",
    executable="chassis_can_node",
    name="chassis_can_node",
    output="screen",
    parameters=[parameter_file],
)
```

命令：

```bash
ros2 launch chassis_can_control chassis_serial.launch.py
```

## 8. YAML 参数文件怎样工作

文件：

```text
config/chassis_serial.yaml
```

开头：

```yaml
chassis_can_node:
  ros__parameters:
    transport: serial
    serial_baud_rate: 115200
```

第一层必须和节点名匹配；`ros__parameters` 是 ROS 2 固定写法；下面才是键值参数。

在 Python 中先声明：

```python
self.declare_parameter("serial_baud_rate", 115200)
```

再读取：

```python
int(self.get_parameter("serial_baud_rate").value)
```

YAML 有对应值时覆盖默认值；没有时使用 Python 中的默认值。

## 9. 节点初始化做了什么

`ChassisCanNode.__init__()` 的顺序：

1. 调用父类 `Node` 初始化节点名；
2. 声明通信、坐标系、控制和安全参数；
3. 根据 `transport` 创建串口或 SocketCAN 对象；
4. 尝试打开接口；
5. 建立所有内部状态；
6. 创建发布者；
7. 创建 `/cmd_vel` 订阅者；
8. 创建各项服务；
9. 创建多个定时器；
10. 注册参数变化检查；
11. 输出启动日志。

### 9.1 发布者

```python
self._odom_publisher = self.create_publisher(Odometry, "odom", 20)
```

三项分别是：消息类型、话题名、队列深度。

### 9.2 订阅者

```python
self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
```

收到 `/cmd_vel` 时，ROS 自动调用：

```python
def _on_cmd_vel(self, message):
```

这就是回调函数。

### 9.3 服务

```python
self.create_service(SetBool, "chassis/enable", self._on_enable)
```

收到服务请求后调用 `_on_enable()`，它修改软件使能状态并填写响应。

### 9.4 定时器

```python
self.create_timer(0.02, self._send_motion_command)
self.create_timer(0.005, self._poll_can)
self.create_timer(0.05, self._send_one_pending_parameter)
self.create_timer(0.20, self._publish_diagnostics)
```

分别表示：

| 周期 | 作用 |
|---:|---|
| 20 ms | 以 50 Hz 向 STM32 发送运动命令 |
| 5 ms | 读取并处理串口已有帧 |
| 50 ms | 参数队列处理 |
| 200 ms | 发布诊断 |

## 10. `/cmd_vel` 怎样变成串口命令

### 10.1 接收 Twist

`geometry_msgs/msg/Twist` 包含：

```text
linear.x/y/z
angular.x/y/z
```

平面小车主要使用：

```text
linear.x   前后速度，m/s
angular.z  绕竖直 Z 轴角速度，rad/s
```

回调只保存最新消息和收到时间：

```python
self._latest_command = message
self._has_received_cmd_vel = True
self._last_cmd_vel_time = time.monotonic()
```

回调不直接写串口，真正发送由固定 50 Hz 定时器完成。这样即使 `/cmd_vel` 发布频率不稳定，下发周期仍相对稳定。

### 10.2 软件使能与新鲜度检查

定时器计算：

```python
command_is_fresh = now - last_time <= 0.20
```

只有同时满足：

```text
命令未超时
软件使能为 True
没有急停
```

才把最新 `linear.x`、`angular.z` 和 `enable=True` 下发。否则下发：

```text
linear = 0
angular = 0
enable = False
```

所以“调用使能一次后永远保持最后速度”不会发生。

### 10.3 Python 怎样把数字打成字节

`can_protocol.py` 中：

```python
body = struct.pack(
    "<hhBBB",
    linear_mps * 1000,
    angular_radps * 1000,
    flags,
    1,
    sequence,
)
```

格式字符串：

```text
<   小端序
h   16 位有符号整数
h   16 位有符号整数
B   8 位无符号整数
B   8 位无符号整数
B   8 位无符号整数
```

总共 7 字节，再追加 CRC 成为 8 字节。

例如：

```text
linear.x = 0.05 m/s → 50 → 0x0032 → 小端 32 00
angular.z = 0       → 0  → 00 00
flags = 1           → 已使能
version = 1
sequence = 假设 7
最后追加 CRC
```

STM32 C 代码使用相同字节位置和小端解释，所以能恢复出原值。

## 11. `CanFrame` 为什么也用于串口

```python
@dataclass
class CanFrame:
    can_id: int
    data: bytes
```

这是项目的逻辑帧：ID + 固定 8 字节。CAN 版直接发送它；串口版由 `SerialTransport` 包成 15 字节：

```text
A5 5A + version + logical_id + length + data[8] + CRC
```

这样上层协议和 ROS 节点无需知道底层究竟是 CAN 还是串口。只有传输对象不同：

```python
SerialTransport(...)
SocketCanTransport(...)
```

这种把“传什么”和“怎么传”分开的设计叫分层或抽象。

## 12. `SerialTransport` 逐步讲解

文件：

```text
chassis_can_control/serial_transport.py
```

### 12.1 打开设备文件

Linux 把串口表示成文件：

```python
os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
```

含义：

- `O_RDWR`：可读可写；
- `O_NOCTTY`：不让它成为进程控制终端；
- `O_NONBLOCK`：没有数据时立即返回，不把 ROS 回调卡死。

### 12.2 配置 115200-8N1

`termios` 和 `tty` 设置：

```text
115200 波特率
8 数据位
无校验
1 停止位
关闭软/硬件流控
raw 模式，不做换行或回显转换
```

### 12.3 发送

`_encode()` 生成 15 字节包，`os.write()` 可能一次只写出部分内容，所以代码用循环直到全部写完。

### 12.4 接收缓存

非阻塞读取可能得到半帧、数帧或从帧中间开始的数据。代码先追加到 `bytearray` 缓存，然后：

1. 搜索 `A5 5A`；
2. 数据不足 15 字节就等待下次；
3. 检查版本、长度、ID 和 CRC；
4. 合法则转成 `CanFrame`；
5. 非法则只丢一个字节，继续重新同步。

缓存超过 4096 字节时会裁剪，避免接错协议或噪声导致内存无限增长。

## 13. STM32 遥测怎样变成 ROS 数据

5 ms 定时器调用：

```python
frames = self._can.receive_available()
for frame in frames:
    self._handle_frame(frame)
```

`_handle_frame()` 根据 ID 分流：

```text
0x181 → 轮速
0x182 → PWM/误差
0x183 → 左编码器
0x184 → 右编码器
0x185 → 状态
0x186 → 参数应答
0x700 → 心跳
```

### 13.1 `struct.unpack`

例如轮速：

```python
left_target, left_actual, right_target, right_actual = \
    struct.unpack("<hhhh", frame.data)
```

把 8 字节按四个 int16 小端整数解开，再除以 1000 恢复 m/s。

### 13.2 为什么检查 ID、CRC 和 sequence

- ID：确认这帧是什么类型；
- CRC：确认数据未损坏；
- sequence：确认左右编码器属于同一批，参数应答属于当前请求。

只校验长度而不校验这些内容，偶然错位可能被当成真实速度。

## 14. `/motor/debug` 怎样生成

只有轮速帧和电机输出帧都已收到才发布：

```python
message.data = [
    left_target,
    left_measured,
    right_target,
    right_measured,
    left_pwm,
    right_pwm,
    left_error,
    right_error,
]
```

使用命令：

```bash
ros2 topic echo /motor/debug
```

这相当于远程查看 STM32 速度闭环内部变量，适合调参和判断电机是否跟随目标。

## 15. RDK 里程计怎样计算

文件：

```text
wheel_odometry.py
```

STM32 每批发送左右累计编码器计数和相同 sequence。RDK 等两侧 sequence 相同后才更新。

### 15.1 第一批只建立基准

节点启动时 STM32 累计计数不一定为 0。第一次收到计数只保存为 previous，不把它当成节点启动后走过的距离。

### 15.2 处理 int32 环绕

累计计数超过 `2147483647` 会绕回负数。`wrapped_count_delta()` 用 32 位模运算得到正确差值。

### 15.3 计数到位移

```text
meters_per_count = π × wheel_diameter / counts_per_rev
left_distance = left_delta × meters_per_count
right_distance = right_delta × meters_per_count
```

### 15.4 差速里程计

```text
center_distance = (left_distance + right_distance) / 2
yaw_delta = (right_distance - left_distance) / wheel_track
```

使用中点方向积分：

```text
middle_yaw = yaw + yaw_delta/2
x += center_distance × cos(middle_yaw)
y += center_distance × sin(middle_yaw)
yaw += yaw_delta
```

中点法比始终用周期起点角度更准确，同时仍容易理解。

### 15.5 发布 `/odom`、`/joint_states` 和 `/tf`

二维 yaw 转四元数：

```text
qz = sin(yaw/2)
qw = cos(yaw/2)
qx = qy = 0
```

ROS 中通常不用单个欧拉角存姿态，而使用四元数避免三维姿态奇异问题。

`/joint_states` 中轮子位置使用弧度，轮子角速度：

```text
angular_velocity = linear_wheel_speed / wheel_radius
```

## 16. 诊断信息怎样生成

每 0.20 s 发布 `/diagnostics`。

如果从未收到心跳，或心跳超过 2 s：

```text
level = ERROR = 2
message = STM32 heartbeat missing
```

收到心跳和状态后，根据状态、故障位生成：

```text
level: 0
message: IDLE
faults: NONE
battery_voltage: 10.600 V
heartbeat_age: 0.115 s
```

`heartbeat_age` 用 `time.monotonic()` 计算。单调时钟只关心间隔，不会因为系统时间校准而突然倒退。

诊断等级：

```text
0 = OK
1 = WARN
2 = ERROR
```

`pending_parameters` 和 `parameter_transfer_failed` 只描述参数传输，不能代替心跳状态。

## 17. 服务回调怎样工作

### 17.1 软件使能

```python
def _on_enable(self, request, response):
    self._drive_enabled = bool(request.data)
    response.success = True
```

命令：

```bash
ros2 service call /chassis/enable std_srvs/srv/SetBool "{data: true}"
```

注意：服务返回 `success=True` 表示 RDK 接受了请求，不等于硬件一定已经运动。还必须有新鲜 `/cmd_vel`、STM32 无故障、硬件使能有效。

### 17.2 急停

急停置位时：

```text
RDK 软件使能立即清零
运动帧带 emergency_stop=1
STM32 锁存 ESTOP
```

### 17.3 清故障

RDK 先发送普通失能运动帧，再发送带 `A5 5A` 钥匙的清故障系统命令。STM32 只有看到使能和急停位都清零才允许清故障。

### 17.4 复位里程计

STM32 不在 `RUNNING` 时才能复位。成功后 RDK 同时清空自己的里程计状态，重新建立编码器基准。

## 18. 参数队列、应答和重试

参数不是 30 个一起塞进串口，而是排队逐个发送：

```python
self._parameter_queue = deque()
self._pending_parameter = None
```

流程：

1. 从队头取一个参数；
2. 分配 sequence；
3. 发送并记录发送时间、次数；
4. 等待 STM32 回应相同参数 ID 和 sequence；
5. 成功才弹出队头并发送下一个；
6. 200 ms 没回应则重试；
7. 最多 3 次；
8. 仍失败则清队列并设置 `parameter_transfer_failed=True`。

这是一种简单的停止等待 ARQ：一次只保持一个未确认请求，逻辑清晰，适合低速控制参数传输。

## 19. 上下位机代码怎样一一对应

| 功能 | RDK Python | STM32 C |
|---|---|---|
| ID 常量 | `can_protocol.py` | `chassis_can_protocol.h` |
| CRC-8 | `crc8()` | `chassis_crc8()` |
| 运动编码 | `make_motion_command()` | `chassis_decode_motion_command()` |
| 参数编码 | `make_parameter_command()` | `chassis_decode_parameter_command()` |
| 系统命令 | `make_system_command()` | `chassis_decode_system_command()` |
| 轮速解码 | `decode_wheel_speed()` | `chassis_encode_wheel_speed()` |
| 状态解码 | `decode_status()` | `chassis_encode_status()` |
| 心跳解码 | `decode_heartbeat()` | `chassis_encode_heartbeat()` |
| 串口外层 | `serial_transport.py` | `chassis_serial_transport.c` |
| 参数范围 | `PARAMETER_RANGES` | `chassis_params_validate()` |

如果修改协议，必须同时修改两端。例如 Python 把轮速改成大端，而 STM32 仍按小端读取，就会得到完全错误的值。

### 19.1 一个完整前进命令的生命周期

1. 某 ROS 节点发布 `/cmd_vel`：`linear.x=0.05`；
2. `_on_cmd_vel()` 保存消息和接收时间；
3. 50 Hz 定时器确认命令新鲜、软件已使能；
4. `make_motion_command()` 编码 logical ID `0x101` 和 8 字节；
5. `SerialTransport` 包成 15 字节并写入 `0002` 串口；
6. STM32 USART3 中断逐字节组帧并入队；
7. `chassis_serial_task` 取帧、校验并保存命令；
8. 100 Hz 控制任务根据底盘类型计算左右目标；
9. 编码器得到实测速度；
10. 前馈 + PI 计算 PWM；
11. `Set_Pwm()` 写硬件；
12. STM32 回传目标、实测、PWM、编码器、状态；
13. RDK 解码并发布 ROS 话题。

## 20. 当前 IMU 与 RDK 的关系

当前 STM32 固件有 MPU6050/ICM20948 读取任务，但协议表中没有 IMU logical ID，Python 中也没有：

```text
sensor_msgs/msg/Imu 发布者
/imu/data 话题
IMU 解码函数
编码器 + IMU 融合
```

因此当前 `/odom` 完全来自编码器，不是 IMU 融合里程计。

如果后续完善，推荐链路：

```text
STM32 修复静止平均校准
→ 定义 IMU 报文（时间、加速度、角速度）
→ RDK 解码并换成 SI 单位
→ 发布 sensor_msgs/msg/Imu
→ 配置 frame_id 和协方差
→ 静态标定 IMU 到 base_link
→ robot_localization 融合 /odom 与 /imu/data
```

不要在原始单位和坐标轴没确认前直接加入导航滤波器。

## 21. Ubuntu 终端基础

### 21.1 提示符怎么看

常见：

```text
wheeltec@ubuntu:~$
```

含义：

```text
wheeltec  当前用户
ubuntu    主机名
~         当前目录是用户主目录
$         普通用户
```

### 21.2 路径

```text
/home/wheeltec/chassis_project  绝对路径
~/chassis_project               ~ 代表 /home/wheeltec
src/chassis_can_control         相对当前目录
..                              上一级目录
.                               当前目录
```

### 21.3 常用文件命令

```bash
pwd
```

显示当前目录。

```bash
ls
ls -l
ls -la
```

列出文件；`-l` 显示详细信息；`-a` 包含隐藏文件。

```bash
cd ~/chassis_project/ros2_ws
```

进入目录。

```bash
less 文件名
```

分页阅读；按方向键滚动，按 `q` 退出。

```bash
nano 文件名
```

简单文本编辑器；`Ctrl+O` 保存，回车确认，`Ctrl+X` 退出。

### 21.4 `source` 是什么

```bash
source /opt/ros/humble/setup.bash
```

让当前 shell 执行文件内容，从而增加 ROS 相关环境变量和命令路径。若直接运行脚本，它设置的环境通常只属于子进程，当前终端得不到。

```bash
source ~/chassis_project/ros2_ws/install/setup.bash
```

再把自己的工作空间叠加到系统 ROS 环境上。

每个新终端都要重新 source，因为环境变量属于各自 shell 进程。

### 21.5 `bash` 和 `sh`

安全启动脚本以 Bash 语法编写：

```bash
bash start_chassis_serial_safe.sh
```

不要写成：

```bash
sh start_chassis_serial_safe.sh
```

Ubuntu 的 `sh` 可能是更精简的 `dash`，不保证支持 Bash 特性。

### 21.6 管道、重定向与后台符号

管道：

```bash
ros2 service list | grep chassis
```

把左侧输出交给 `grep` 筛选。

覆盖重定向：

```bash
command > output.log
```

追加重定向：

```bash
command >> output.log
```

把错误也写入同一文件：

```bash
command > output.log 2>&1
```

后台执行：

```bash
command &
```

`nohup` 让进程在 SSH 断开后仍继续：

```bash
nohup command > output.log 2>&1 </dev/null &
```

这就是你看不到终端窗口、节点却仍在运行的原因。

### 21.7 `Ctrl+C` 的含义

它向前台程序发送中断信号。若正在运行 `ros2 topic echo`，只会停止显示；若前台运行 launch，会停止 launch 和其节点。它不会自动杀掉另一个 `nohup` 后台节点。

## 22. RDK 日常命令逐条解释

### 22.1 SSH 登录

在 Ubuntu 虚拟机：

```bash
ssh wheeltec@192.168.0.100
```

`ssh` 是加密远程终端，后面是 `用户名@IP地址`。

### 22.2 检查串口

```bash
ls -l /dev/serial/by-id/
```

`/dev` 是设备文件目录，`by-id` 提供稳定名称。当前：

```text
0001 = 雷达
0002 = STM32 底盘
```

### 22.3 检查重复节点

```bash
pgrep -af 'chassis_serial.launch.py|chassis_can_node'
```

- `pgrep`：按名字查进程；
- `-a`：显示完整命令行；
- `-f`：匹配完整命令行；
- `|` 在引号中的正则表示“或”。

正常只有一套 launch 和 node。命令自身偶尔可能出现在匹配结果中，要看完整命令行判断。

### 22.4 安全启动

```bash
cd ~/chassis_project/ros2_ws
bash src/chassis_can_control/scripts/start_chassis_serial_safe.sh
```

脚本自动 source ROS 环境、列出稳定串口并前台启动。它不会调用使能服务。

### 22.5 查看诊断

新终端执行：

```bash
source /opt/ros/humble/setup.bash
source ~/chassis_project/ros2_ws/install/setup.bash
ros2 topic echo --once /diagnostics
```

`--once` 表示收到一条后退出。没有它会持续输出，按 `Ctrl+C` 结束显示。

### 22.6 安全停止

```bash
bash ~/chassis_project/ros2_ws/src/chassis_can_control/scripts/stop_chassis_serial_safe.sh
```

脚本先调用失能，再停止本项目节点。

### 22.7 查看后台日志

```bash
tail -f ~/chassis_project/chassis_serial.log
```

- `tail`：看文件末尾；
- `-f`：文件新增内容时持续跟随；
- `Ctrl+C`：停止跟随，不会停止后台节点。

### 22.8 查看谁占用串口

```bash
fuser -v /dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00
```

如果显示两个 `chassis_can_node` PID，就是重复节点在抢读同一串口。

## 23. ROS 命令逐条解释

### 23.1 查看节点与接口

```bash
ros2 node list
ros2 node info /chassis_can_node
ros2 topic list -t
ros2 service list -t | grep chassis
```

`-t` 同时显示消息或服务类型。

### 23.2 软件使能/失能

```bash
ros2 service call /chassis/enable std_srvs/srv/SetBool "{data: true}"
```

结构：

```text
ros2 service call
服务名
服务类型
YAML 格式的请求数据
```

失能只把 `true` 改为 `false`。

### 23.3 发布速度

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.05}, angular: {z: 0.0}}"
```

解释：

- `topic pub`：发布话题；
- `-r 10`：每秒发布 10 次；
- `/cmd_vel`：话题名；
- `geometry_msgs/msg/Twist`：消息类型；
- 最后是 YAML 消息内容；
- 行末 `\` 表示命令下一行继续。

必须持续发布，因为 RDK 只接受 0.20 s 内的新鲜命令。

### 23.4 急停和清故障

```bash
ros2 service call /chassis/emergency_stop std_srvs/srv/SetBool "{data: true}"
ros2 service call /chassis/clear_faults std_srvs/srv/Trigger "{}"
```

`Trigger` 请求没有字段，所以用空字典 `{}`。

### 23.5 查看频率

```bash
ros2 topic hz /motor/debug
ros2 topic hz /odom
```

它测量实际收到的发布频率，不改变节点。

### 23.6 查看 TF

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

`ros2 run` 直接运行包中的某个可执行程序；这里持续显示两个坐标系变换。

### 23.7 查看和修改参数

```bash
ros2 param list /chassis_can_node
ros2 param get /chassis_can_node controller.left_kp
ros2 param set /chassis_can_node controller.left_kp 3100.0
ros2 param dump /chassis_can_node
```

修改控制参数前必须失能。运行时 `param set` 不会自动写回 YAML，也不会自动保存 STM32 Flash。

## 24. 编译命令逐条解释

```bash
cd ~/chassis_project/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select chassis_can_control
source install/setup.bash
```

解释：

1. 进入工作空间根目录；
2. 加载系统 ROS 2 Humble；
3. `colcon` 只编译指定包；
4. 把新生成的 `install` 环境加载进当前终端。

成功应看到：

```text
Summary: 1 package finished
```

修改以下内容后需要重新编译：

```text
Python 文件
launch 文件
YAML 文件
setup.py
package.xml
```

普通开机启动不需要每次编译。

## 25. 前台与后台节点

### 25.1 前台

```bash
ros2 launch chassis_can_control chassis_serial.launch.py
```

日志在当前窗口，`Ctrl+C` 正常停止。开发调试优先使用前台。

### 25.2 后台

```bash
nohup ros2 launch chassis_can_control chassis_serial.launch.py \
  > ~/chassis_project/chassis_serial.log 2>&1 </dev/null &
```

SSH 退出后仍运行。再次登录不会看到原窗口，必须通过：

```bash
pgrep -af 'chassis_serial.launch.py|chassis_can_node'
ros2 node list
tail -f ~/chassis_project/chassis_serial.log
```

来确认。

重复启动两个节点会造成：

- 同名 ROS 节点；
- 两个进程抢读同一串口；
- 每个进程只读到部分字节；
- 无法稳定拼成 15 字节帧；
- 出现 `STM32 heartbeat missing`。

## 26. 常见输出应该怎样理解

### 26.1 正常

```text
level: "\0"
message: IDLE
faults: NONE
heartbeat_age: 0.115 s
```

`"\0"` 实际是数值 0，表示 OK。`IDLE` 表示通信正常但电机当前不运行。

### 26.2 心跳错误

```text
level: "\x02"
message: STM32 heartbeat missing
heartbeat_age: 75.309 s
```

数值 2 表示 ERROR。可能是 STM32 掉电、串口拔掉、固件不匹配或两个节点抢串口。

### 26.3 `AMENT_TRACE_SETUP_FILES: unbound variable`

原因是旧脚本在 source ROS 前开启 `set -u`。当前安全脚本已修复。应使用：

```bash
bash ~/chassis_project/ros2_ws/src/chassis_can_control/scripts/start_chassis_serial_safe.sh
```

### 26.4 Package not found

说明当前终端没加载工作空间，或没有成功编译：

```bash
source /opt/ros/humble/setup.bash
source ~/chassis_project/ros2_ws/install/setup.bash
ros2 pkg prefix chassis_can_control
```

## 27. 推荐的阅读顺序

Python 基础薄弱时按以下顺序：

1. `config/chassis_serial.yaml`：先看有哪些参数；
2. `launch/chassis_serial.launch.py`：看节点如何启动；
3. `setup.py`：看可执行入口怎样指向 `main()`；
4. `chassis_can_node.py:main()` 和 `__init__()`；
5. `_on_cmd_vel()`、`_send_motion_command()`；
6. `can_protocol.py:make_motion_command()`；
7. `serial_transport.py:send()`；
8. 再反向阅读 `receive_available()` 和 `_handle_frame()`；
9. `wheel_odometry.py`；
10. 服务和参数队列；
11. 最后看 SocketCAN 保留版本。

每读一个函数，回答四个问题：

```text
谁调用它？
输入是什么？
内部状态改了什么？
输出或副作用是什么？
```

## 28. 一次完整安全实验

### 终端 1：登录并检查

```bash
ssh wheeltec@192.168.0.100
pgrep -af 'chassis_serial.launch.py|chassis_can_node'
ls -l /dev/serial/by-id/
```

若节点已运行，不要重复启动。若没有：

```bash
cd ~/chassis_project/ros2_ws
bash src/chassis_can_control/scripts/start_chassis_serial_safe.sh
```

### 终端 2：加载环境和检查诊断

```bash
ssh wheeltec@192.168.0.100
source /opt/ros/humble/setup.bash
source ~/chassis_project/ros2_ws/install/setup.bash
ros2 topic echo --once /diagnostics
```

确认 `IDLE / NONE` 和心跳小于 2 s。

### 终端 3：观察闭环

```bash
ssh wheeltec@192.168.0.100
source /opt/ros/humble/setup.bash
source ~/chassis_project/ros2_ws/install/setup.bash
ros2 topic echo /motor/debug
```

### 终端 2：车轮架空后使能并低速发送

```bash
ros2 service call /chassis/enable std_srvs/srv/SetBool "{data: true}"
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.05}, angular: {z: 0.0}}"
```

停止发布按 `Ctrl+C`，立即失能：

```bash
ros2 service call /chassis/enable std_srvs/srv/SetBool "{data: false}"
```

观察终端 3 中目标、实测和 PWM 是否合理。

## 29. 当前上位机实现边界

已经实现：

- ROS 2 `/cmd_vel` 接口；
- 串口/CAN 可切换传输抽象；
- 速度命令定时下发和双层超时；
- 轮速、PWM、编码器、状态和心跳解码；
- `/odom`、`/joint_states`、TF；
- `/diagnostics`、`/motor/debug`；
- 使能、急停、清故障、参数、保存、复位服务；
- 参数范围检查、排队、应答和重试；
- STM32 重启后里程计基准复位；
- 协议和里程计单元测试。

尚未实现：

- `/imu/data`；
- IMU 单位换算和坐标系标定；
- 编码器与 IMU 融合；
- 导航栈的完整配置；
- 后台进程的 systemd 正式服务管理。

因此当前上位机准确定位是“ROS 2 与 STM32 电机闭环之间的安全通信桥接、遥测和编码器里程计节点”。

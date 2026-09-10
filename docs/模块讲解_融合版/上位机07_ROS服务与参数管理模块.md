# 上位机第 7 模块：ROS 服务与参数管理模块

> 本文件是该模块在旧任务中的完整归档：保留首次讲解正文，并把首次文档之后的专项追问统一融合在同一文件中。以后无需回到原任务查找。

## 本模块归档内容

- 首次讲解来源：`docs/17_ROS2服务与参数管理模块零基础详解.md`
- 首次文档之后没有新增专项追问。

---

# 第一部分：首次完整讲解

# ROS 2 服务与参数管理模块零基础详解

## 1. 这份文档讲什么

本文专门讲解 RDK X5 上位机的“ROS 2 服务与参数管理模块”。对应代码主要位于：

```text
ros2_ws/src/chassis_can_control/
├── chassis_can_control/
│   ├── chassis_can_node.py     服务、参数、队列、应答和诊断
│   └── can_protocol.py         参数 ID、范围和协议编码/解码
├── config/
│   └── chassis_serial.yaml     启动时加载的参数值
└── launch/
    └── chassis_serial.launch.py
```

STM32 对应代码主要是：

```text
CHASSIS_SERIAL/
├── inc/
│   ├── chassis_can_protocol.h
│   └── chassis_params.h
└── src/
    ├── chassis_app.c
    ├── chassis_can_protocol.c
    ├── chassis_params.c
    └── c30d_chassis_port.c
```

这部分代码解决两个问题：

1. 操作者怎样通过 ROS 2 发出“使能、急停、清故障、保存参数”等一次性操作；
2. RDK 中的 PI、轮径、安全阈值等参数，怎样检查、下发、确认并保存到 STM32。

## 2. 先区分话题、服务和参数

第一次接触 ROS 2 时，最容易混淆这三种接口。

### 2.1 话题 Topic：连续广播数据

话题适合持续变化的数据：

```text
/cmd_vel       连续速度命令
/odom          连续里程计
/diagnostics   连续诊断信息
/motor/debug   连续电机调试数据
```

它像广播：发布者不断发送，订阅者收到就处理。发布者通常不会为每条消息等待响应。

本项目持续发送速度，使用话题最合适：

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.05}, angular: {z: 0.0}}"
```

### 2.2 服务 Service：一次请求、一次响应

服务适合明确的一次性操作：

```text
请求：把底盘软件使能打开
响应：RDK 已接受，drive enabled
```

它像打电话：客户端发出一个请求，服务端处理后返回一个响应。

例如：

```bash
ros2 service call /chassis/enable std_srvs/srv/SetBool "{data: true}"
```

### 2.3 参数 Parameter：节点的配置状态

参数是附属于某个 ROS 节点的配置：

```text
controller.left_kp = 3000.0
geometry.wheel_diameter_m = 0.065
serial_baud_rate = 115200
```

参数不是持续广播的传感器数据，也不是单纯的一次命令。参数具有名称、类型和当前值，可以查看或修改。

### 2.4 三者的直观类比

| ROS 2 接口 | 类比 | 本项目例子 |
|---|---|---|
| 话题 | 广播频道 | 持续发布速度和诊断 |
| 服务 | 打电话询问并得到回应 | 使能、急停、清故障 |
| 参数 | 设备设置页面 | PI、轮径、串口路径 |

## 3. 服务中的客户端与服务端

执行：

```bash
ros2 service call /chassis/enable std_srvs/srv/SetBool "{data: true}"
```

ROS 2 CLI 会临时创建一个客户端。RDK 上长期运行的 `/chassis_can_node` 是服务端：

```text
终端 ros2 service call
        ↓ 请求
/chassis_can_node 服务回调
        ↓ 响应
终端显示 response
```

服务端必须已经运行。检查：

```bash
ros2 node list
ros2 service list -t | grep chassis
```

## 4. Python 怎样创建 ROS 2 服务

代码导入两个标准服务类型：

```python
from std_srvs.srv import SetBool, Trigger
```

在节点初始化函数 `__init__()` 中创建服务：

```python
self.create_service(SetBool, "chassis/enable", self._on_enable)
self.create_service(SetBool, "chassis/emergency_stop", self._on_estop)
self.create_service(Trigger, "chassis/clear_faults", self._on_clear_faults)
self.create_service(Trigger, "chassis/save_parameters", self._on_save_parameters)
self.create_service(Trigger, "chassis/push_parameters", self._on_push_parameters)
self.create_service(Trigger, "chassis/reset_odometry", self._on_reset_odometry)
```

以第一行说明三个参数：

```python
self.create_service(
    SetBool,              # 服务类型
    "chassis/enable",     # 服务名称
    self._on_enable,      # 收到请求时调用的函数
)
```

节点默认命名空间为根目录，因此最终看到的完整名称是：

```text
/chassis/enable
```

## 5. Python 回调函数是什么

回调函数不是由我们在主程序中不断手工调用。节点创建服务时，把函数交给 ROS：

```python
self.create_service(SetBool, "chassis/enable", self._on_enable)
```

之后 `rclpy.spin(node)` 持续等待事件。当服务请求到达时，ROS 自动调用：

```python
def _on_enable(self, request, response):
```

三个参数：

- `self`：当前 `ChassisCanNode` 对象；
- `request`：客户端发来的请求对象；
- `response`：ROS 已创建、等待我们填写的响应对象。

函数最后：

```python
return response
```

ROS 将这个响应传回客户端。

## 6. 两种服务类型

### 6.1 `SetBool`

它的概念结构是：

```text
请求：
  bool data

响应：
  bool success
  string message
```

所以命令必须提供 `data`：

```bash
ros2 service call /chassis/enable std_srvs/srv/SetBool "{data: true}"
```

查看类型定义：

```bash
ros2 interface show std_srvs/srv/SetBool
```

### 6.2 `Trigger`

它的请求没有字段，只表达“执行一次这个动作”：

```text
请求：空

响应：
  bool success
  string message
```

命令使用空字典：

```bash
ros2 service call /chassis/clear_faults std_srvs/srv/Trigger "{}"
```

查看定义：

```bash
ros2 interface show std_srvs/srv/Trigger
```

### 6.3 为什么代码里有 `del request`

Trigger 请求没有数据，函数仍必须接收 `request` 参数：

```python
def _on_clear_faults(self, request, response):
    del request
```

`del request` 表示后面不使用这个局部变量，也能避免静态检查工具提示“参数未使用”。它不是在删除 ROS 网络中的请求。

## 7. 本项目的六个服务概览

| 服务 | 类型 | 主要作用 |
|---|---|---|
| `/chassis/enable` | `SetBool` | RDK 软件使能/失能 |
| `/chassis/emergency_stop` | `SetBool` | 发送急停并撤销软件使能 |
| `/chassis/clear_faults` | `Trigger` | 清除 STM32 锁存故障 |
| `/chassis/push_parameters` | `Trigger` | 把 RDK 当前参数排队下发 |
| `/chassis/save_parameters` | `Trigger` | 请求 STM32 保存参数到 Flash |
| `/chassis/reset_odometry` | `Trigger` | 复位 STM32 和 RDK 里程计 |

下面逐个讲解。

## 8. `/chassis/enable` 软件使能服务

代码：

```python
def _on_enable(self, request: SetBool.Request,
               response: SetBool.Response):
    self._drive_enabled = bool(request.data)
    response.success = True
    response.message = (
        "drive enabled" if request.data else "drive disabled"
    )
    self._send_motion_command()
    return response
```

### 8.1 第一行：保存 RDK 软件使能状态

```python
self._drive_enabled = bool(request.data)
```

如果请求是 `true`：

```python
self._drive_enabled = True
```

如果请求是 `false`：

```python
self._drive_enabled = False
```

`self._drive_enabled` 是 Python 节点内存中的状态，不是 STM32 GPIO。

### 8.2 Python 条件表达式

```python
"drive enabled" if request.data else "drive disabled"
```

等价于：

```python
if request.data:
    response.message = "drive enabled"
else:
    response.message = "drive disabled"
```

### 8.3 为什么立即调用 `_send_motion_command()`

平时有一个 50 Hz 定时器每 20 ms 发送运动命令。服务回调主动调用一次，可以不必等到下个定时周期，让失能更快生效。

### 8.4 使能后为什么不一定马上转

真正发送非零命令必须同时满足：

```python
command_is_fresh and self._drive_enabled and not self._emergency_stop
```

也就是：

```text
最近 0.20 s 内有 /cmd_vel
AND RDK 软件使能打开
AND 没有急停
```

STM32 还会继续检查：

```text
硬件使能是否有效
命令是否超时
是否存在锁存故障
电池、编码器和堵转是否正常
```

因此使能服务只是多道安全条件中的一道。

### 8.5 使用方法

车轮架空后使能：

```bash
ros2 service call /chassis/enable std_srvs/srv/SetBool "{data: true}"
```

失能：

```bash
ros2 service call /chassis/enable std_srvs/srv/SetBool "{data: false}"
```

### 8.6 `success=True` 到底证明了什么

它证明：

- ROS 服务回调运行成功；
- RDK 内部 `_drive_enabled` 已修改；
- 节点尝试立即发送了一次运动命令。

它不单独证明：

- STM32 已收到；
- 底盘硬件使能有效；
- 电机一定会运动。

最终要结合：

```bash
ros2 topic echo --once /diagnostics
ros2 topic echo /motor/debug
```

## 9. `/chassis/emergency_stop` 急停服务

代码：

```python
def _on_estop(self, request, response):
    self._emergency_stop = bool(request.data)
    if request.data:
        self._drive_enabled = False
    response.success = True
    response.message = (...)
    self._send_motion_command()
    return response
```

### 9.1 触发急停

```bash
ros2 service call /chassis/emergency_stop std_srvs/srv/SetBool \
"{data: true}"
```

RDK 会：

1. 设置 `_emergency_stop=True`；
2. 同时设置 `_drive_enabled=False`；
3. 立即发送一帧 `enable=0, emergency_stop=1` 的运动命令；
4. STM32 收到后锁存 `ESTOP` 故障；
5. 后续普通速度命令不能自动解除锁存故障。

### 9.2 将急停请求改回 false 不等于清故障

```bash
ros2 service call /chassis/emergency_stop std_srvs/srv/SetBool \
"{data: false}"
```

它只让后续运动帧不再携带急停位。STM32 中已经锁存的 `ESTOP` 仍然存在。

正确恢复顺序：

```text
排除危险原因
→ 车轮架空
→ emergency_stop false
→ clear_faults
→ 检查 IDLE / NONE
→ 必要时重新 enable
```

### 9.3 为什么急停还要撤销软件使能

即使随后清除了 STM32 故障，RDK 也不会自动恢复运动。操作者必须重新调用使能服务，这防止清故障后突然执行旧命令。

## 10. `/chassis/clear_faults` 清故障服务

代码核心：

```python
self._drive_enabled = False
self._emergency_stop = False
self._send_motion_command()
response.success = self._send(
    protocol.make_system_command(
        protocol.SYSTEM_CLEAR_FAULT,
        self._next_sequence(),
    )
)
```

### 10.1 为什么先发普通失能命令

STM32 的安全规则是：只有最新运动命令同时满足：

```text
enable = 0
emergency_stop = 0
```

才接受清故障系统命令。

所以 RDK 按顺序发送：

```text
第一帧：普通失能运动帧
第二帧：清故障系统命令
```

### 10.2 系统命令为什么有钥匙字节

`make_system_command()` 生成：

```text
opcode
0xA5
0x5A
sequence
三个保留字节
CRC
```

STM32 只有钥匙字节和 CRC 都正确才处理。这样随机串口噪声更难被误认为清故障或写 Flash 命令。

### 10.3 使用方法

```bash
ros2 service call /chassis/clear_faults std_srvs/srv/Trigger "{}"
```

随后必须检查：

```bash
ros2 topic echo --once /diagnostics
```

只有看到：

```text
state: IDLE
faults: NONE
```

才说明状态已经恢复。

### 10.4 当前 `success=True` 的实际含义

当前代码的 `success` 来自 `_send()`：它表示系统命令已经成功写入串口传输层。

STM32 会用 `0x186` 参数/系统应答帧回应，但当前服务回调不会阻塞等待这帧。因此服务响应不是 STM32 最终确认。最终确认应看：

- 节点日志中的 `Parameter reply`；
- 后续 `/diagnostics` 是否恢复为 `IDLE / NONE`。

## 11. `/chassis/reset_odometry` 里程计复位服务

代码先检查最新 STM32 状态：

```python
if self._latest_status is not None \
        and self._latest_status.state == 2:
    response.success = False
    response.message = "disable chassis before resetting odometry"
    return response
```

状态数值 `2` 表示 `RUNNING`。运行中拒绝复位，避免位置突然跳回原点。

允许时发送 STM32 复位命令：

```python
sent = self._send(
    protocol.make_system_command(
        protocol.SYSTEM_RESET_ODOMETRY,
        self._next_sequence(),
    )
)
```

如果串口写入成功，RDK 同时清空自己的里程计：

```python
self._odometry.reset()
self._last_odometry_sequence = None
```

### 11.1 为什么两边都要复位

STM32 保存累计编码器计数，RDK 也保存由计数积分得到的 `x、y、yaw`。只复位一边会使两边基准不一致。

### 11.2 使用方法

先失能并保证静止：

```bash
ros2 service call /chassis/enable std_srvs/srv/SetBool "{data: false}"
ros2 service call /chassis/reset_odometry std_srvs/srv/Trigger "{}"
```

再查看：

```bash
ros2 topic echo --once /odom
```

### 11.3 当前实现的确认边界

和清故障一样，服务成功表示命令写入成功，回调没有等待 STM32 最终应答。正常通信下两端会几乎同时复位，但严谨验证还应检查日志、诊断和后续 `/odom`。

## 12. 参数到底存在几个地方

本项目不是只有“一份参数”。至少要区分四层：

| 层级 | 位置 | 是否掉电保留 | 含义 |
|---|---|---|---|
| Python 默认值 | `chassis_can_node.py` | 代码中保留 | YAML 缺项时备用 |
| YAML 源码 | `config/chassis_serial.yaml` | 文件保留 | 希望下次启动加载的配置 |
| RDK 节点运行时参数 | Python 进程内存 | 节点退出即消失 | 当前 ROS 参数值 |
| STM32 RAM 参数 | MCU 内存 | MCU 掉电即消失 | 当前真正用于电机控制 |
| STM32 Flash 参数 | MCU 内部 Flash | 掉电保留 | STM32 下次上电加载值 |

这里实际列出了五种存储形态，因为“RDK 参数”和“STM32 参数”不能混为一谈。

### 12.1 节点启动时的 RDK 参数来源

Python 先声明默认值：

```python
self.declare_parameter("controller.left_kp", 3000.0)
```

launch 文件加载安装目录里的 YAML：

```text
install/chassis_can_control/share/chassis_can_control/
config/chassis_serial.yaml
```

YAML 有对应值时覆盖 Python 默认值。

### 12.2 STM32 上电时的参数来源

STM32 从自己的 Flash 加载保存参数；如果参数区为空，使用 C 代码默认值。

### 12.3 为什么 RDK 和 STM32 参数可能不同

配置中：

```yaml
push_parameters_on_start: false
```

意味着 ROS 节点启动时不会自动用 YAML 覆盖 STM32。

所以：

```bash
ros2 param get /chassis_can_node controller.left_kp
```

只能证明 RDK 当前参数是多少，不能单独证明 STM32 当前使用相同数值。

需要执行参数下发并确认成功，才能让 STM32 RAM 与 RDK 当前参数同步。

## 13. ROS 参数怎样声明和读取

### 13.1 单独声明

```python
self.declare_parameter("serial_baud_rate", 115200)
```

名称是字符串，默认值为整数 115200。

### 13.2 批量声明控制参数

代码先建立字典：

```python
defaults = {
    "controller.left_kp": 3000.0,
    "controller.left_ki": 1500.0,
    ...
}
```

再循环：

```python
for name, value in defaults.items():
    self.declare_parameter(name, value)
```

`defaults.items()` 每次给出一对名称和值。

### 13.3 读取参数

```python
value = self.get_parameter("controller.left_kp").value
```

`get_parameter()` 返回参数对象，`.value` 才是具体数值。

## 14. 查看 ROS 参数

每个新终端先加载环境：

```bash
source /opt/ros/humble/setup.bash
source ~/chassis_project/ros2_ws/install/setup.bash
```

列出全部参数：

```bash
ros2 param list /chassis_can_node
```

读取一个参数：

```bash
ros2 param get /chassis_can_node controller.left_kp
```

导出当前 RDK 参数：

```bash
ros2 param dump /chassis_can_node
```

保存到文件：

```bash
ros2 param dump /chassis_can_node \
  > ~/chassis_project/chassis_params_runtime.yaml
```

这只是导出 RDK 节点参数，不是读取 STM32 Flash。

## 15. 参数名称与 STM32 ID 的映射

RDK 使用容易阅读的字符串名称，串口协议为了紧凑使用 1 字节 ID：

```python
PARAMETER_IDS = {
    "controller.left_kp": 1,
    "controller.left_ki": 2,
    ...
}
```

| ID | ROS 参数名称 | 作用 |
|---:|---|---|
| 1 | `controller.left_kp` | 左轮比例增益 |
| 2 | `controller.left_ki` | 左轮积分增益 |
| 3 | `controller.left_kff` | 左轮前馈 |
| 4 | `controller.right_kp` | 右轮比例增益 |
| 5 | `controller.right_ki` | 右轮积分增益 |
| 6 | `controller.right_kff` | 右轮前馈 |
| 7～10 | `controller.*dead*` | 左右轮正反转死区 |
| 11 | `limits.acceleration_mps2` | 加速度限制 |
| 12 | `limits.deceleration_mps2` | 减速度限制 |
| 13 | `controller.filter_alpha` | 速度低通系数 |
| 14～15 | `controller.straight_sync_*` | 直行同步修正 |
| 16 | `geometry.wheel_track_m` | 轮距 |
| 17 | `geometry.wheel_diameter_m` | 轮径 |
| 18 | `geometry.counts_per_wheel_rev` | 一圈编码器计数 |
| 19 | `limits.maximum_linear_speed_mps` | 最大线速度 |
| 20 | `safety.low_battery_mv` | 欠压阈值，mV |
| 21 | `safety.command_timeout_ms` | STM32 命令超时 |
| 22～25 | `safety.stall_*` | 堵转判定参数 |
| 26 | `geometry.chassis_type` | 0=差速，1=阿克曼 |
| 27 | `geometry.wheelbase_m` | 轴距 |
| 28 | `geometry.maximum_steering_angle_rad` | 最大舵角 |
| 29 | `geometry.servo_center_pwm` | 舵机中值 |
| 30 | `geometry.servo_pwm_per_rad` | 舵角到 PWM 换算 |

上下位机必须使用相同 ID，否则 RDK 以为在修改 Kp，STM32 可能把同一数字解释成轮径或安全阈值。

## 16. 参数范围检查

RDK `can_protocol.py` 定义 `PARAMETER_RANGES`。例如：

```python
"controller.left_kp": (0.0, 50000.0)
"geometry.wheel_diameter_m": (0.02, 0.5)
"safety.command_timeout_ms": (50.0, 5000.0)
```

修改参数时，回调依次检查：

1. 参数是不是允许动态修改；
2. STM32 是否正在 `RUNNING`；
3. 参数名是否属于下发列表；
4. 值能否转换为浮点数；
5. 值是否是有限数，不允许 NaN 或无穷大；
6. 是否在最小值和最大值之间；
7. 特殊离散值是否满足要求。

例如底盘类型必须恰好为 0 或 1：

```python
if parameter.name == "geometry.chassis_type" \
        and value not in (0.0, 1.0):
    return SetParametersResult(
        successful=False,
        reason="geometry.chassis_type must be exactly 0 or 1",
    )
```

RDK 先检查一次，STM32 收到后还会用 C 代码再次检查。双端检查避免错误值直接进入电机控制。

## 17. 参数变化回调是什么

节点注册：

```python
self.add_on_set_parameters_callback(
    self._on_parameters_changed
)
```

当执行：

```bash
ros2 param set /chassis_can_node controller.left_kp 3100.0
```

ROS 在真正接受新值前，自动调用：

```python
def _on_parameters_changed(self, parameters):
```

`parameters` 是一个列表，因为一次请求可能同时修改多个参数。

回调返回：

```python
SetParametersResult(successful=True)
```

表示 RDK 本地检查通过，ROS 可以接受这些新值。

返回失败：

```python
SetParametersResult(
    successful=False,
    reason="...",
)
```

ROS 拒绝修改并向终端显示原因。

## 18. 哪些参数需要重启

代码列出：

```python
restart_required = {
    "can_interface",
    "transport",
    "serial_port",
    "serial_baud_rate",
    "command_rate_hz",
    "odom_frame",
    "base_frame",
    "left_joint_name",
    "right_joint_name",
    "push_parameters_on_start",
}
```

这些参数在节点初始化时用于创建串口对象、定时器、发布坐标系等。运行中直接修改，旧对象不会自动重建，所以回调拒绝：

```text
edit YAML and restart node to change ...
```

正确流程：

```bash
bash ~/chassis_project/ros2_ws/src/chassis_can_control/scripts/stop_chassis_serial_safe.sh

nano ~/chassis_project/ros2_ws/src/chassis_can_control/config/chassis_serial.yaml

cd ~/chassis_project/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select chassis_can_control
source install/setup.bash

bash src/chassis_can_control/scripts/start_chassis_serial_safe.sh
```

为什么需要重新编译？launch 读取的是 `install` 目录中 YAML 的安装副本，不是直接读取 `src` 中刚修改的文件。

## 19. 动态修改单个控制参数的完整流程

以修改左轮 Kp 为例。

### 19.1 操作前检查

```bash
ros2 topic echo --once /diagnostics
```

确认：

```text
message: IDLE
faults: NONE
heartbeat_age: 小于 2 秒
```

再明确失能：

```bash
ros2 service call /chassis/enable std_srvs/srv/SetBool "{data: false}"
```

### 19.2 读取旧值

```bash
ros2 param get /chassis_can_node controller.left_kp
```

### 19.3 小幅修改

```bash
ros2 param set /chassis_can_node controller.left_kp 3100.0
```

内部发生：

```text
CLI 发参数修改请求
→ ROS 调用 _on_parameters_changed()
→ 检查当前不是 RUNNING
→ 检查 3100 是有限数且在范围内
→ 将 (参数ID=1, 值=3100) 放入发送队列
→ 回调返回 successful=True
→ RDK 节点参数更新为 3100
→ 50 ms 参数定时器稍后发给 STM32
→ STM32 检查并更新 RAM 参数
→ STM32 返回 ID、状态、实际值、sequence
→ RDK 匹配应答并移除队列项
```

### 19.4 不要只相信 `ros2 param set` 的成功输出

该输出主要表示 RDK 本地预检查通过。STM32 应答发生在稍后的定时器和串口回调中。

等待一小段时间，再检查：

```bash
ros2 topic echo --once /diagnostics
```

必须满足：

```text
pending_parameters: 0
parameter_transfer_failed: False
```

同时可查看节点日志中的：

```text
Parameter reply: id=1, status=0, value=3100.0000, sequence=...
```

## 20. 参数发送队列为什么存在

如果 30 个参数瞬间全部写入串口：

- STM32 接收队列可能拥塞；
- 应答顺序难追踪；
- 不知道哪一个失败；
- 重试逻辑复杂。

所以使用双端队列：

```python
from collections import deque

self._parameter_queue = deque()
self._pending_parameter = None
```

`deque` 是适合从头部取数据的队列。

### 20.1 队列元素

```python
(parameter_id, value)
```

例如：

```python
(1, 3100.0)
```

表示左 Kp。

### 20.2 `PendingParameter`

```python
@dataclass
class PendingParameter:
    parameter_id: int
    value: float
    sequence: int
    attempts: int
    sent_time: float
```

它记录当前已经发出、正在等待应答的一个参数：

- `parameter_id`：哪个参数；
- `value`：发送的值；
- `sequence`：本次请求序号；
- `attempts`：已发送几次；
- `sent_time`：最近发送时刻。

任何时刻最多只有一个未确认参数。这种方式叫“停止等待”：收到当前应答后才发送下一个。

## 21. 参数帧怎样编码

Python：

```python
body = struct.pack(
    "<BBfB",
    parameter_id,
    operation,
    value,
    sequence,
)
```

格式含义：

```text
<   小端序
B   参数 ID，1 字节
B   操作，1 字节
f   float，4 字节
B   sequence，1 字节
```

共 7 字节，再加内部 CRC 成 8 字节。logical ID 为：

```text
0x102 = 参数命令
```

当前写参数操作：

```python
PARAM_WRITE = 1
```

8 字节逻辑帧再由 `serial_transport.py` 包成 15 字节串口帧，发送给 STM32。

## 22. 定时发送、超时与三次重试

节点创建 50 ms 定时器：

```python
self.create_timer(0.05, self._send_one_pending_parameter)
```

### 22.1 没有等待应答时

取队首但暂时不删除：

```python
parameter_id, value = self._parameter_queue[0]
```

分配 sequence、生成帧并发送。发送成功后建立 `PendingParameter`。

### 22.2 正在等待应答时

```python
elapsed = now - self._pending_parameter.sent_time
```

小于 0.20 s：继续等待。

超过 0.20 s 且发送次数少于 3：使用相同 ID、值和 sequence 重发。

已经发送 3 次仍无正确应答：

```python
self._parameter_transfer_failed = True
self._pending_parameter = None
self._parameter_queue.clear()
```

并记录日志：

```text
Parameter id=... timed out after 3 attempts
```

### 22.3 为什么重发使用相同 sequence

STM32 和 RDK 可以知道这是同一个请求的重试，而不是一个新的参数操作。

## 23. STM32 参数应答怎样处理

STM32 返回 logical ID：

```text
0x186 = 参数/系统命令应答
```

Python 解码得到：

```python
ParameterReply(
    parameter_id,
    status,
    value,
    sequence,
)
```

RDK 只接受同时匹配：

```text
reply.parameter_id == pending.parameter_id
reply.sequence == pending.sequence
```

不匹配的旧应答或其他应答不能误确认当前参数。

`status == 0` 表示成功：

```python
self._parameter_queue.popleft()
self._pending_parameter = None
```

非零表示 STM32 拒绝：

```python
self._parameter_transfer_failed = True
self._parameter_queue.clear()
```

### 23.1 STM32 应答状态

| 状态 | 含义 |
|---:|---|
| 0 | `OK`，成功 |
| 1 | `BAD_CRC`，CRC 错误 |
| 2 | `BAD_PARAMETER`，ID/命令不认识 |
| 3 | `BAD_VALUE`，数值非法 |
| 4 | `DENIED`，当前状态不允许 |
| 5 | `STORAGE_ERROR`，Flash 保存错误 |

## 24. `/chassis/push_parameters` 全量下发服务

代码先拒绝运行中下发：

```python
if self._latest_status is not None \
        and self._latest_status.state == 2:
    response.success = False
```

再检查是否已有参数等待应答。如果安全，则：

```python
self._queue_all_parameters()
```

`_queue_all_parameters()` 遍历 `PARAMETER_IDS`：

```python
for name, parameter_id in protocol.PARAMETER_IDS.items():
    self._parameter_queue.append(
        (parameter_id,
         float(self.get_parameter(name).value))
    )
```

它使用的是 RDK 节点当前运行时参数，不一定等于磁盘 YAML，因为运行中可能使用 `ros2 param set` 改过。

### 24.1 使用方法

先明确失能并确认 `IDLE / NONE`：

```bash
ros2 service call /chassis/enable std_srvs/srv/SetBool "{data: false}"
ros2 topic echo --once /diagnostics
```

再下发：

```bash
ros2 service call /chassis/push_parameters std_srvs/srv/Trigger "{}"
```

响应类似：

```text
success=True
message='queued 30 parameters'
```

这只表示 30 个参数已进入 RDK 队列，不表示 30 个都被 STM32 接受。

等待数秒后检查：

```bash
ros2 topic echo --once /diagnostics
```

完成标准：

```text
pending_parameters: 0
parameter_transfer_failed: False
```

### 24.2 诊断中的两个字段

```text
pending_parameters
```

表示还有多少参数未完成确认。正常会从约 30 逐步减到 0。

```text
parameter_transfer_failed
```

表示本轮参数传输是否发生超时或 STM32 拒绝。

## 25. `/chassis/save_parameters` 保存服务

在保存前，代码先检查：

```text
当前没有 pending 参数
参数队列为空
parameter_transfer_failed 为 False
```

否则拒绝保存。

通过检查后发送：

```text
SYSTEM_SAVE_PARAMETERS
```

STM32 收到后：

1. 确认底盘不在 `RUNNING`；
2. 复制当前 RAM 参数；
3. 计算参数 CRC32；
4. 电机输出归零；
5. 将参数写入内部 Flash；
6. 返回成功或存储错误应答。

### 25.1 正确使用顺序

```text
失能
→ 参数修改/全量下发
→ pending=0
→ transfer_failed=False
→ 车轮架空验证
→ 确认控制正确
→ 再保存 Flash
```

命令：

```bash
ros2 service call /chassis/save_parameters std_srvs/srv/Trigger "{}"
```

### 25.2 重要：不要把“save command sent”理解成保存完成

当前服务回调只确认保存命令成功写到串口：

```text
save command sent; check parameter reply
```

STM32 的最终 Flash 写入结果通过 `0x186` 应答进入节点日志。查看前台启动终端，或后台日志：

```bash
tail -f ~/chassis_project/chassis_serial.log
```

成功应答中的 `status=0`；`status=5` 表示存储错误。

### 25.3 如果从未下发参数就直接保存

只要队列为空且没有传输失败，服务会发送保存命令。此时 STM32 保存的是它当前 RAM 中已有的参数，不一定是 RDK YAML 值。

所以推荐始终先明确执行 `/chassis/push_parameters`、确认完成、架空验证，再保存。

## 26. `ros2 param set`、`push` 和 `save` 的区别

| 操作 | RDK 运行参数 | STM32 RAM | STM32 Flash | 重启后结果 |
|---|---|---|---|---|
| `ros2 param set` | 修改 | 对支持参数排队下发 | 不修改 | RDK 重启后恢复 YAML；STM32 掉电后恢复 Flash |
| `push_parameters` | 不修改 | 全量同步当前 RDK 参数 | 不修改 | STM32 掉电后仍恢复旧 Flash |
| `save_parameters` | 不修改 | 不修改 | 保存 STM32 当前 RAM | STM32 下次上电使用新值 |
| 修改 YAML + build | 下次启动加载新值 | 不自动修改 | 不修改 | RDK 重启后使用新 YAML |

要实现永久一致，一般需要：

```text
修改 YAML
→ build
→ 重启 RDK 节点
→ push_parameters
→ 确认
→ 架空验证
→ save_parameters
```

## 27. 为什么 `push_parameters_on_start` 默认是 false

如果设为 true，每次节点启动都会立即排队下发全部参数。这样虽然方便同步，但风险是：

- YAML 填错会在每次启动时覆盖 STM32 RAM；
- 尚未验证的参数会自动进入电机控制；
- 不容易分辨当前 STM32 参数来自 Flash 还是 RDK。

当前使用：

```yaml
push_parameters_on_start: false
```

让操作者先检查通信和底盘状态，再主动下发，更适合调试阶段。

## 28. 服务、协议、串口和下位机的连接

### 28.1 使能服务链路

```text
ros2 service call /chassis/enable
→ _on_enable()
→ 修改 RDK _drive_enabled
→ _send_motion_command()
→ can_protocol.make_motion_command()
→ serial_transport.send()
→ STM32 USART3
→ chassis_app_process_can_frame()
→ STM32 状态机判断是否允许输出
→ wheel_controller 计算 PWM
```

### 28.2 参数修改链路

```text
ros2 param set
→ _on_parameters_changed()
→ 范围检查
→ _parameter_queue
→ 50 ms 参数定时器
→ make_parameter_command()
→ 串口
→ STM32 chassis_params_set_value()
→ 参数应答 0x186
→ decode_parameter_reply()
→ _handle_parameter_reply()
→ 更新 diagnostics 中的传输状态
```

### 28.3 保存参数链路

```text
/chassis/save_parameters
→ make_system_command(SAVE)
→ 串口
→ STM32 检查状态
→ c30d_port_save_parameters()
→ CRC32 + Flash 写入
→ 0x186 应答
→ RDK 日志
```

## 29. 服务模块与其他上位机模块的关系

### 29.1 与 ROS 主节点模块

所有服务、参数、定时器和共享状态都属于同一个 `ChassisCanNode` 对象。`self._drive_enabled`、`self._latest_status` 等变量可以被不同回调访问。

### 29.2 与速度命令模块

`/chassis/enable` 和 `/chassis/emergency_stop` 不直接生成 PWM，而是改变 `_send_motion_command()` 的判断条件。

### 29.3 与协议模块

ROS 服务对象不能直接发给 STM32。必须转换为：

- 运动命令 `0x101`；
- 参数命令 `0x102`；
- 系统命令 `0x103`。

### 29.4 与串口模块

服务回调调用 `_send()`，最终由 `SerialTransport.send()` 写入 `0002` 串口。如果串口未打开，服务可能返回 `CAN unavailable`。这里的文字沿用了统一 CAN/串口抽象，串口版实际表示传输接口不可用。

### 29.5 与诊断模块

参数队列和失败标志被周期发布到 `/diagnostics`。STM32 状态和故障也通过诊断决定参数修改、复位操作是否安全。

### 29.6 与里程计模块

`reset_odometry` 服务同时操作 STM32 系统计数和 Python `WheelOdometry` 对象。

### 29.7 与下位机电机闭环

PI、前馈、死区等参数最终只有进入 STM32 RAM，才会改变真正的电机闭环。仅修改 RDK 节点显示值并不等于下位机已经应用。

## 30. 三种“成功”必须分开理解

这是本模块最重要的概念。

### 第一层：ROS 请求成功

终端找到服务，回调执行并返回：

```text
success=True
```

### 第二层：RDK 传输成功

Python 将逻辑帧包装后成功写入 Linux 串口。`_send()` 返回 True。

### 第三层：STM32 执行成功

STM32 收到合法帧，通过状态和范围检查，真正执行并返回成功应答或在遥测中体现状态变化。

它们不是同一件事。例如：

```text
ROS push 服务 success=True
```

只表示已排队；必须等 30 个参数逐个应答。

判断第三层成功的方法：

| 操作 | 最终确认方式 |
|---|---|
| 使能/失能 | `/diagnostics` 状态 + `/motor/debug` + 实际命令 |
| 急停 | `/diagnostics` 出现 `ESTOP` |
| 清故障 | `/diagnostics` 恢复 `IDLE / NONE` |
| 参数下发 | `pending=0` 且 `transfer_failed=False`，日志 `status=0` |
| 保存 Flash | 节点日志中的系统应答 `status=0`，必要时重启验证 |
| 里程计复位 | `/odom` 回到新基准且日志无拒绝 |

## 31. 安全的参数实验流程

下面适合第一次学习，不建议一开始修改 PI。

### 31.1 准备三个终端

每个终端都登录 RDK，并加载环境：

```bash
ssh wheeltec@192.168.0.100
source /opt/ros/humble/setup.bash
source ~/chassis_project/ros2_ws/install/setup.bash
```

如果 IP 已变化，以 `hostname -I` 的实际结果为准。

### 31.2 终端 A：查看诊断

```bash
ros2 topic echo /diagnostics
```

### 31.3 终端 B：查看节点日志

后台节点：

```bash
tail -f ~/chassis_project/chassis_serial.log
```

如果节点在前台运行，直接观察启动终端。

### 31.4 终端 C：只读练习

```bash
ros2 service list -t | grep chassis
ros2 param list /chassis_can_node
ros2 param get /chassis_can_node controller.left_kp
ros2 param get /chassis_can_node geometry.wheel_diameter_m
```

这些命令不修改状态。

### 31.5 练习失能

```bash
ros2 service call /chassis/enable std_srvs/srv/SetBool "{data: false}"
```

检查诊断仍为 `IDLE`。

### 31.6 初次不要做的事情

在还未理解并备份参数前，不要立即执行：

```text
大幅修改 Kp/Ki/Kff
push 全部参数
save 到 Flash
车轮落地调参
开启 push_parameters_on_start
```

## 32. 修改一个参数后的恢复方法

假设运行时把左 Kp 从 3000 改成 3100，想恢复：

```bash
ros2 param set /chassis_can_node controller.left_kp 3000.0
```

等待：

```text
pending_parameters: 0
parameter_transfer_failed: False
```

如果只是 RDK 运行时参数混乱，又没有保存 STM32 Flash，可以安全停止并重新启动节点，RDK 参数会从 YAML 重载。但注意：已经成功下发到 STM32 RAM 的值不会因为重启 RDK 自动恢复，除非：

- 重新 push YAML 参数；或者
- STM32 重新上电，从 Flash 恢复已保存值。

这再次说明 RDK 和 STM32 是两套独立内存。

## 33. 常见问题

### 33.1 `Service not available`

检查：

```bash
ros2 node list
ros2 service list | grep chassis
```

确认当前终端已加载 ROS 环境，且节点确实运行。

### 33.2 服务成功但电机不转

使能只是一个条件，还要持续发送新鲜 `/cmd_vel`，并确保 STM32 无故障和硬件使能有效。

### 33.3 `ros2 param set` 成功但闭环没变化

可能原因：

- 参数还在队列；
- STM32 应答超时；
- 修改的参数需要重启而被拒绝；
- 只查看了 RDK 参数，没有确认 STM32；
- 改变量太小，现象不明显。

检查：

```bash
ros2 topic echo --once /diagnostics
tail -100 ~/chassis_project/chassis_serial.log
```

### 33.4 `pending_parameters` 一直不为 0

检查心跳、串口、是否有重复节点、STM32 固件是否匹配。不要在传输未完成时保存参数。

### 33.5 `parameter_transfer_failed: True`

表示某个参数三次超时或被 STM32 拒绝。不要保存。先看日志找到参数 ID 和状态，排查后重新执行 push。

### 33.6 修改串口路径被拒绝

`serial_port` 属于需要重启的参数。修改源码 YAML、重新 build、停止旧节点后再启动。

### 33.7 修改 YAML 后参数仍是旧值

因为 launch 加载安装副本。执行：

```bash
cd ~/chassis_project/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select chassis_can_control
source install/setup.bash
```

然后停止旧节点并重新启动。

### 33.8 两个同名节点为什么危险

两个节点会同时开放相同服务名、参数服务并抢读同一个串口，客户端可能找到多个服务端，串口帧也被拆分。参数管理和诊断都会变得不可信。

启动前检查：

```bash
pgrep -af 'chassis_serial.launch.py|chassis_can_node'
```

## 34. 推荐阅读代码顺序

1. `chassis_can_node.py` 中六行 `create_service()`；
2. `_on_enable()`；
3. `_on_estop()`；
4. `_on_clear_faults()`；
5. 参数默认值和 `declare_parameter()`；
6. `can_protocol.py` 的 `PARAMETER_IDS` 和范围；
7. `_on_parameters_changed()`；
8. `_queue_all_parameters()`；
9. `_send_one_pending_parameter()`；
10. `_handle_parameter_reply()`；
11. `_on_save_parameters()`；
12. STM32 `chassis_app_process_can_frame()`；
13. STM32 `chassis_params_set_value()`；
14. STM32 `c30d_port_save_parameters()`。

阅读每个函数时问：

```text
谁触发它？
它读取哪些 self 状态？
它修改哪些 self 状态？
它是否发串口帧？
它有没有等待 STM32 回应？
最终成功应该去哪里确认？
```

## 35. 最后总结

服务模块负责一次性操作：

```text
使能、急停、清故障、参数全量下发、保存和里程计复位
```

参数管理模块负责：

```text
声明和读取配置
→ 运行时预检查
→ 名称映射为参数 ID
→ 排队发送
→ 200 ms 超时重试
→ ID + sequence 匹配应答
→ 通过 diagnostics 报告进度和失败
→ 验证后请求 STM32 保存 Flash
```

最需要记住的四句话：

1. ROS 参数值、STM32 RAM 参数和 STM32 Flash 参数不是同一份内存；
2. `push_parameters` 是下发到 STM32 RAM，`save_parameters` 才是保存到 Flash；
3. ROS 服务的 `success=True` 不总等于 STM32 已执行完成；
4. 修改电机参数必须先失能、架空、小步修改、确认应答，最后才保存。

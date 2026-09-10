# 上位机第 2 模块：ROS 速度命令处理模块

> 本文件是该模块在旧任务中的完整归档：保留首次讲解正文，并把首次文档之后的专项追问统一融合在同一文件中。以后无需回到原任务查找。

## 本模块归档内容

- 首次讲解来源：`docs/17_RDK_X5_ROS速度命令处理模块详解.md`
- 后续追问来源：原模块任务中的对话回答（此前未单独保存为文档）

---

# 第一部分：首次完整讲解

# RDK X5 上位机：ROS 速度命令处理模块零基础详解

## 1. 本文讲解范围

本文只讲 RDK X5 上位机中的“ROS 速度命令处理模块”，也就是：

```text
ROS 2 /cmd_vel
    ↓
RDK Python 节点接收并保存速度
    ↓
检查命令是否超时、软件是否使能、是否急停
    ↓
以 50 Hz 生成运动协议帧
    ↓
通过 USB 串口发送给 STM32
```

涉及的主要文件：

```text
ros2_ws/src/chassis_can_control/
├── chassis_can_control/
│   ├── chassis_can_node.py     ROS 订阅、状态判断和定时发送
│   ├── can_protocol.py         把速度命令编码成 8 字节逻辑帧
│   └── serial_transport.py     把逻辑帧包装成 15 字节串口帧
└── config/
    └── chassis_serial.yaml     发送频率和超时时间
```

下位机对应文件：

```text
CHASSIS_SERIAL/src/chassis_serial_transport.c  接收 15 字节串口帧
CHASSIS_SERIAL/src/chassis_can_protocol.c      解码 8 字节运动命令
CHASSIS_SERIAL/src/chassis_app.c               状态机和左右轮目标计算
CHASSIS_SERIAL/src/wheel_controller.c          电机速度闭环
```

本文先补足必要的 ROS 2 和 Python 基础，再逐句讲解实际代码。

## 2. 先用一句话理解这个模块

这个模块不是“收到一次速度就立刻让电机一直转”，而是：

> 保存 ROS 最新速度，并由定时器不断检查安全条件；只有命令足够新、软件已经使能、没有急停时，才周期性地把速度发给 STM32。

核心安全判断可简化为：

```text
允许发送实际速度 =
    收到过 /cmd_vel
    AND 最近一次 /cmd_vel 没超时
    AND 软件使能已打开
    AND 没有急停
```

任意一项不满足，就发送：

```text
线速度 = 0
角速度 = 0
enable = false
```

## 3. 为什么机器人通常使用 `/cmd_vel`

`cmd_vel` 是 command velocity 的缩写，即“速度命令”。在 ROS 移动机器人中，它通常用来表达：

```text
我希望机器人现在以多大的线速度和角速度运动
```

它表达的是速度目标，而不是：

- 电机 PWM；
- 左轮转速；
- 右轮转速；
- 机器人最终要到达的位置；
- 要运行多少秒。

举例：

```text
linear.x = 0.10 m/s
angular.z = 0.00 rad/s
```

表示希望机器人以 `0.10 m/s` 向前直行。

```text
linear.x = 0.10 m/s
angular.z = 0.30 rad/s
```

表示希望机器人一边前进，一边以 `0.30 rad/s` 改变航向。

上层命令只描述车体运动目标。左右轮应该分别多快、PWM 应该多大，由 STM32 继续计算。

## 4. ROS 2 话题基础

### 4.1 什么是话题

ROS 2 话题可以理解成一个带名字的数据频道：

```text
发布者 Publisher → 话题 Topic → 订阅者 Subscriber
```

例如：

```text
键盘控制节点
    ↓ 发布 Twist
/cmd_vel
    ↓ 订阅 Twist
chassis_can_node
```

发布者不直接调用底盘节点中的 Python 函数。它只向 ROS 网络发布消息，ROS 2 负责把消息送给所有订阅者。

### 4.2 发布者可能是谁

`/cmd_vel` 可以来自：

- `ros2 topic pub` 手工测试命令；
- 键盘遥控节点；
- 手柄节点；
- ROS 2 导航控制器；
- 你自己编写的路径跟踪程序；
- 其他自动驾驶算法。

底盘节点不关心发布者是谁，只关心收到的消息类型和数值。

### 4.3 一个话题可以有多个发布者

ROS 允许多个节点同时向 `/cmd_vel` 发布。此时底盘节点会收到它们交错的数据，最终使用“最后收到的一条”。这对电机控制很危险，例如导航发前进，键盘节点同时发停止，两者会互相覆盖。

查看发布者数量：

```bash
ros2 topic info /cmd_vel -v
```

正式测试前应确认只有你预期的速度发布者。

## 5. `Twist` 消息是什么

本项目订阅的消息类型是：

```text
geometry_msgs/msg/Twist
```

它的结构可以理解为：

```text
Twist
├── linear
│   ├── x
│   ├── y
│   └── z
└── angular
    ├── x
    ├── y
    └── z
```

查看 ROS 中的正式定义：

```bash
ros2 interface show geometry_msgs/msg/Twist
```

会看到两个 `Vector3`：

```text
Vector3 linear
Vector3 angular
```

对于在地面上运动的小车，本项目只使用：

```text
message.linear.x   前后线速度，单位 m/s
message.angular.z  绕竖直轴的角速度，单位 rad/s
```

其他字段当前被忽略：

```text
linear.y、linear.z
angular.x、angular.y
```

### 5.1 正负号的含义

按照本项目约定：

```text
linear.x > 0   向前
linear.x < 0   向后
angular.z > 0  逆时针/左转方向
angular.z < 0  顺时针/右转方向
```

实际车轮和舵机方向还受机械安装、底盘类型和下位机方向参数影响，第一次必须架空测试。

### 5.2 rad/s 是什么

弧度是角度单位：

```text
π rad = 180°
1 rad ≈ 57.3°
```

`0.30 rad/s` 表示如果角速度保持不变，理想情况下每秒航向改变约：

```text
0.30 × 57.3 ≈ 17.2°
```

它不是舵机角度。STM32 会结合线速度、轴距和底盘类型计算真正舵角或左右轮速度。

## 6. Python 中与本模块相关的最少基础

### 6.1 `self` 是当前节点对象

代码：

```python
self._drive_enabled = False
```

可以理解为：

```text
在当前 chassis_can_node 对象内部，保存一个名为 _drive_enabled 的状态
```

它不是临时局部变量。只要节点还在运行，这个值就一直存在，并能被订阅回调、定时器回调和服务回调共同访问。

### 6.2 `False`、`True` 和 `not`

```python
self._drive_enabled = False
```

表示软件使能关闭。

```python
not self._emergency_stop
```

表示“没有急停”。如果 `_emergency_stop` 为 `False`，`not False` 就是 `True`。

### 6.3 `and`

```python
if A and B and C:
```

只有 A、B、C 全为真，条件才成立。这正适合多个安全条件同时判断。

### 6.4 属性的逐层访问

```python
self._latest_command.linear.x
```

从左到右理解：

```text
self
  当前节点对象
._latest_command
  节点保存的最新 Twist 消息
.linear
  Twist 中的线速度向量
.x
  线速度向量的 x 分量
```

### 6.5 函数和回调

```python
def _on_cmd_vel(self, message: Twist) -> None:
```

- `def`：定义函数；
- `_on_cmd_vel`：函数名；
- `self`：当前节点；
- `message`：ROS 送来的消息；
- `: Twist`：类型提示；
- `-> None`：不返回业务结果。

所谓回调，就是你先把函数交给 ROS，等事件发生时由 ROS 调用它。

## 7. 节点启动时先建立哪些状态

`chassis_can_node.py` 初始化时：

```python
self._drive_enabled = False
self._emergency_stop = False
self._latest_command = Twist()
self._has_received_cmd_vel = False
self._last_cmd_vel_time = 0.0
```

逐项解释。

### 7.1 `_drive_enabled = False`

软件使能默认关闭。即使上电后立刻收到非零 `/cmd_vel`，也不能直接让车运动。

必须显式调用：

```bash
ros2 service call /chassis/enable std_srvs/srv/SetBool "{data: true}"
```

### 7.2 `_emergency_stop = False`

RDK 自己当前没有发出急停请求。

注意，这不一定代表 STM32 没有历史锁存急停。STM32 的锁存故障还要通过 `/diagnostics` 判断，并用清故障服务处理。

### 7.3 `_latest_command = Twist()`

创建一个空的 `Twist` 对象。它所有数值字段默认是 0，因此初始内容大致为：

```text
linear.x = 0
linear.y = 0
linear.z = 0
angular.x = 0
angular.y = 0
angular.z = 0
```

### 7.4 `_has_received_cmd_vel = False`

虽然 `_latest_command` 默认全零，但程序仍要区分：

```text
真的收到过一条零速度命令
```

和：

```text
节点启动后从未收到任何速度命令
```

所以使用独立布尔变量记录。

### 7.5 `_last_cmd_vel_time = 0.0`

保存最近一次 `/cmd_vel` 到达的单调时间。第一次消息到达前这个值没有实际业务意义，因为 `_has_received_cmd_vel` 仍为 `False`。

## 8. 如何创建 `/cmd_vel` 订阅者

实际代码：

```python
self.create_subscription(
    Twist,
    "cmd_vel",
    self._on_cmd_vel,
    10,
)
```

原代码写在一行，拆开后更容易理解。

四个参数分别是：

| 参数 | 内容 | 含义 |
|---|---|---|
| 1 | `Twist` | 只接收这种消息类型 |
| 2 | `"cmd_vel"` | 话题名 |
| 3 | `self._on_cmd_vel` | 收到消息后调用的函数 |
| 4 | `10` | QoS 历史队列深度 |

### 8.1 为什么代码写 `cmd_vel`，终端显示 `/cmd_vel`

`"cmd_vel"` 是相对名称。当前节点位于根命名空间，所以解析后是：

```text
/cmd_vel
```

如果未来把节点放到 `/robot1` 命名空间，可能解析成：

```text
/robot1/cmd_vel
```

### 8.2 队列深度 10 是什么意思

如果消息到达速度短时间超过节点处理速度，ROS 最多保留一定数量的最近消息。速度控制本质上最关心最新命令，而不是把几秒前积压的命令全部补执行。

当前代码最终只保存最新消息，因此旧消息一旦回调处理完成就不会进入运动命令历史队列。

## 9. `/cmd_vel` 回调逐句讲解

完整代码只有三行：

```python
def _on_cmd_vel(self, message: Twist) -> None:
    self._latest_command = message
    self._has_received_cmd_vel = True
    self._last_cmd_vel_time = time.monotonic()
```

虽然很短，但职责非常明确。

### 9.1 保存最新消息

```python
self._latest_command = message
```

新的消息覆盖旧消息。假设连续收到：

```text
第1条：0.05 m/s
第2条：0.10 m/s
第3条：0.00 m/s
```

最终 `_latest_command` 保存第 3 条零速度。

### 9.2 标记已经收到过

```python
self._has_received_cmd_vel = True
```

从此程序知道通信链上至少出现过速度命令。

### 9.3 记录到达时间

```python
self._last_cmd_vel_time = time.monotonic()
```

`time.monotonic()` 返回只能向前增长的时间，适合计算间隔：

```text
命令年龄 = 当前单调时间 - 最近命令单调时间
```

为什么不用系统日期时间？如果网络校时让系统时钟突然跳变，普通时间差可能错误；单调时钟不会因日期校准而倒退。

### 9.4 回调为什么不直接发送串口

如果直接在回调中发送，串口发送频率就完全跟随上游：

- 上游 5 Hz，就只发 5 Hz；
- 上游 200 Hz，就尝试发 200 Hz；
- 上游抖动，串口周期也抖动；
- 多个发布者交错，发送节奏更混乱。

当前设计把工作分开：

```text
订阅回调：只负责接收和保存
定时器回调：负责固定频率判断和发送
```

这样结构更清晰，也更容易实现超时保护。

## 10. 50 Hz 定时器如何创建

参数文件中：

```yaml
command_rate_hz: 50.0
ros_command_timeout_s: 0.20
```

Python 读取频率：

```python
command_rate = float(self.get_parameter("command_rate_hz").value)
if command_rate < 10.0:
    command_rate = 10.0
self.create_timer(1.0 / command_rate, self._send_motion_command)
```

当频率为 50 Hz：

```text
周期 = 1 / 50 = 0.02 s = 20 ms
```

所以 ROS 大约每 20 ms 调用一次 `_send_motion_command()`。

### 10.1 为什么最低限制为 10 Hz

如果频率配置得太低，STM32 可能经常触发命令超时。代码将小于 10 Hz 的配置强制提高到 10 Hz，避免非常明显的错误配置。

当前 STM32 命令超时为约 200 ms，50 Hz 表示理论上在超时窗口内应收到约 10 帧，留有一定余量。

### 10.2 ROS 定时器是不是严格硬实时

不是。RDK 运行 Linux 和 Python，系统忙时回调可能延迟。因此最终电机实时闭环和独立超时仍放在 STM32 中。

50 Hz 是期望调度频率，不应理解为每次都精确到微秒。

## 11. 定时发送函数逐句讲解

完整代码：

```python
def _send_motion_command(self) -> None:
    now = time.monotonic()
    timeout = float(self.get_parameter("ros_command_timeout_s").value)
    command_is_fresh = (
        self._has_received_cmd_vel
        and now - self._last_cmd_vel_time <= timeout
    )

    if command_is_fresh and self._drive_enabled and not self._emergency_stop:
        linear = float(self._latest_command.linear.x)
        angular = float(self._latest_command.angular.z)
        enable = True
    else:
        linear = 0.0
        angular = 0.0
        enable = False

    frame = protocol.make_motion_command(
        linear,
        angular,
        enable,
        self._emergency_stop,
        self._next_sequence(),
    )
    self._send(frame)
```

下面逐段解释。

### 11.1 获取当前时间

```python
now = time.monotonic()
```

当前只用于计算最近速度命令的年龄。

### 11.2 读取超时参数

```python
timeout = float(self.get_parameter("ros_command_timeout_s").value)
```

当前为：

```text
0.20 s = 200 ms
```

`float(...)` 把 ROS 参数值明确转为 Python 浮点数。

### 11.3 判断命令是否新鲜

```python
command_is_fresh = (
    self._has_received_cmd_vel
    and now - self._last_cmd_vel_time <= timeout
)
```

分成两个条件：

```text
条件 A：节点确实收到过 /cmd_vel
条件 B：最近命令年龄不超过 0.20 s
```

只有 A 和 B 同时满足，`command_is_fresh` 才是 `True`。

示例：

```text
now = 100.35
last = 100.20
命令年龄 = 0.15 s
0.15 <= 0.20 → 新鲜
```

```text
now = 100.45
last = 100.20
命令年龄 = 0.25 s
0.25 <= 0.20 → 假，已超时
```

### 11.4 最核心的四条件安全判断

```python
if command_is_fresh and self._drive_enabled and not self._emergency_stop:
```

真值表：

| 命令新鲜 | 软件使能 | 急停 | 发送结果 |
|---|---|---|---|
| 否 | 任意 | 任意 | 零速度，禁止输出 |
| 是 | 否 | 否 | 零速度，禁止输出 |
| 是 | 是 | 是 | 零速度，急停标志有效 |
| 是 | 是 | 否 | 发送实际速度，允许输出 |

### 11.5 取出真正使用的 Twist 字段

```python
linear = float(self._latest_command.linear.x)
angular = float(self._latest_command.angular.z)
enable = True
```

注意，上位机这里不计算左右轮速度，也不计算 PWM。它只是把车体线速度和角速度交给 STM32。

### 11.6 任一条件不满足就构造安全命令

```python
else:
    linear = 0.0
    angular = 0.0
    enable = False
```

程序不是简单地“不发送”。它仍以 50 Hz 主动发送零速度和禁止位，让 STM32 明确知道 RDK 当前不允许输出。

这比沉默更明确；同时 STM32 仍保留自己的通信超时作为第二层保护。

## 12. 软件使能服务如何影响速度模块

服务创建：

```python
self.create_service(SetBool, "chassis/enable", self._on_enable)
```

服务回调：

```python
def _on_enable(self, request, response):
    self._drive_enabled = bool(request.data)
    response.success = True
    response.message = "drive enabled" if request.data else "drive disabled"
    self._send_motion_command()
    return response
```

调用使能：

```bash
ros2 service call /chassis/enable std_srvs/srv/SetBool "{data: true}"
```

### 12.1 服务请求的数据怎样进入 Python

命令中的：

```text
{data: true}
```

会成为：

```python
request.data == True
```

然后：

```python
self._drive_enabled = True
```

### 12.2 为什么回调中立即发送一次

不必等下一个 20 ms 定时器，使能或失能变化可以立即反映到 STM32。

特别是失能时，立即发送零速度比最多再等 20 ms 更安全。

### 12.3 `success=True` 不等于电机一定转动

它只表示 RDK 接受了软件使能请求。电机实际运动还要求：

- 存在新鲜 `/cmd_vel`；
- RDK 没有急停；
- 串口发送成功；
- STM32 正常接收；
- STM32 硬件使能有效；
- STM32 没有堵转、欠压等故障；
- 运动命令不是零速度。

## 13. 一个非常重要的使能细节

`/cmd_vel` 即使在软件失能时也会被回调保存。

假设：

```text
时刻 0 ms：软件失能
时刻 10 ms：收到 linear.x = 0.20
时刻 50 ms：操作者调用 enable=true
```

因为 50 ms 时保存的非零命令仍在 200 ms 新鲜期内，使能服务会立即调用 `_send_motion_command()`，于是车辆可能立即执行 `0.20 m/s`。

因此使能前必须确认：

1. 车轮架空；
2. 没有未知节点在发布 `/cmd_vel`；
3. 最好先持续发布零速度；
4. 再调用使能；
5. 最后缓慢发布非零速度。

查看发布者：

```bash
ros2 topic info /cmd_vel -v
```

安全的手工实验顺序可以是：

终端 A 先发布零速度：

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.0}}"
```

终端 B 确认架空后使能：

```bash
ros2 service call /chassis/enable std_srvs/srv/SetBool "{data: true}"
```

停止终端 A 的零速度发布，然后从一个明确终端低速发布非零命令。

## 14. 急停服务如何影响速度模块

回调：

```python
def _on_estop(self, request, response):
    self._emergency_stop = bool(request.data)
    if request.data:
        self._drive_enabled = False
    ...
    self._send_motion_command()
```

触发急停：

```bash
ros2 service call /chassis/emergency_stop std_srvs/srv/SetBool "{data: true}"
```

发生三件事：

1. `_emergency_stop=True`；
2. `_drive_enabled=False`；
3. 立即发送零速度、`enable=false`、`estop=true`。

STM32 收到后锁存急停故障。后续普通速度命令不能自动解除。

### 14.1 为什么把急停改为 false 仍不够

执行：

```bash
ros2 service call /chassis/emergency_stop std_srvs/srv/SetBool "{data: false}"
```

只表示 RDK 不再继续请求急停，但 STM32 的锁存故障仍存在。正确恢复流程：

```text
排除真实危险
→ 保持车轮架空
→ 调用 clear_faults
→ 查看 diagnostics 恢复 IDLE / NONE
→ 重新显式使能
```

清故障命令：

```bash
ros2 service call /chassis/clear_faults std_srvs/srv/Trigger "{}"
```

## 15. 超时到底会做什么

这是本模块最容易误解的地方。

当 `/cmd_vel` 超过 0.20 s 没更新：

```text
command_is_fresh = False
```

于是上位机发送：

```text
linear = 0
angular = 0
enable = false
```

但是当前 Python 代码不会执行：

```python
self._drive_enabled = False
```

也就是说，RDK 内部的软件使能记忆仍可能是 `True`。

### 15.1 后续新命令会怎样

如果没有急停，之后再次收到一条新 `/cmd_vel`：

```text
command_is_fresh 重新变为 True
_drive_enabled 仍为 True
```

那么下一个 50 Hz 周期可以恢复发送实际速度。

因此：

```text
停止发布 /cmd_vel
```

表示依靠超时暂时停止；而：

```text
调用 /chassis/enable false
```

表示明确、持续地软件失能。

完成实验或离开车辆前必须显式失能，不能只依赖停止话题发布。

## 16. 序号 sequence 如何生成

代码：

```python
def _next_sequence(self) -> int:
    self._sequence = (self._sequence + 1) & 0xFF
    return self._sequence
```

`0xFF` 是十六进制的 255，按位与后只保留低 8 位，所以序号范围：

```text
0, 1, 2, ..., 254, 255, 0, 1, ...
```

每次构造运动命令都会使用新序号。STM32 将最后处理的命令序号上传到诊断：

```text
last_command_sequence
```

它有助于判断下位机是否持续处理新命令，但不是加密或身份认证。

## 17. 速度如何编码为 8 字节逻辑帧

定时器调用：

```python
frame = protocol.make_motion_command(
    linear,
    angular,
    enable,
    self._emergency_stop,
    self._next_sequence(),
)
```

进入 `can_protocol.py`：

```python
def make_motion_command(
    linear_mps,
    angular_radps,
    enable,
    emergency_stop,
    sequence,
):
```

### 17.1 先拒绝非法浮点数

```python
if not math.isfinite(linear_mps) or not math.isfinite(angular_radps):
    raise ValueError(...)
```

`NaN` 和正负无穷不是合法运动命令。

### 17.2 生成 flags 字节

```python
flags = (CMD_ENABLE if enable else 0) | \
        (CMD_ESTOP if emergency_stop else 0)
```

位定义：

```text
bit 0 = enable
bit 1 = emergency stop
```

可能值：

| enable | estop | flags |
|---|---|---:|
| false | false | 0 |
| true | false | 1 |
| false | true | 2 |
| true | true | 3，但当前安全逻辑不会主动形成这种运动状态 |

`|` 是按位或，把不同标志组合进一个字节。

### 17.3 为什么乘以 1000

```python
linear_mps * 1000.0
angular_radps * 1000.0
```

转换：

```text
m/s → mm/s 数值
rad/s → mrad/s 数值
```

例如：

```text
0.05 m/s × 1000 = 50
0.30 rad/s × 1000 = 300
```

然后四舍五入并限制在 int16 范围。

### 17.4 `struct.pack("<hhBBB", ...)`

```python
body = struct.pack(
    "<hhBBB",
    linear_integer,
    angular_integer,
    flags,
    1,
    sequence,
)
```

格式含义：

| 符号 | 含义 | 字节数 |
|---|---|---:|
| `<` | 小端序 | 0 |
| `h` | 16 位有符号整数 | 2 |
| `h` | 16 位有符号整数 | 2 |
| `B` | 8 位无符号整数 | 1 |
| `B` | 8 位无符号整数 | 1 |
| `B` | 8 位无符号整数 | 1 |

合计 7 字节。

最后：

```python
body + bytes([crc8(body)])
```

追加 1 字节 CRC，得到固定 8 字节逻辑数据。

### 17.5 逻辑帧布局

逻辑 ID 为：

```text
0x101 = 运动命令
```

数据布局：

| 字节 | 内容 |
|---:|---|
| 0～1 | 线速度 ×1000，int16，小端 |
| 2～3 | 角速度 ×1000，int16，小端 |
| 4 | enable/estop flags |
| 5 | 协议版本 1 |
| 6 | sequence |
| 7 | 前 7 字节 CRC-8 |

## 18. 用一个实际数值做编码示例

假设：

```text
linear = 0.05 m/s
angular = 0.00 rad/s
enable = true
estop = false
sequence = 7
```

转换为整数：

```text
linear_integer = 50 = 0x0032
angular_integer = 0 = 0x0000
```

小端序表示低字节在前：

```text
50 → 32 00
0  → 00 00
```

前 7 字节大致为：

```text
32 00 00 00 01 01 07
```

最后再计算 CRC 放在第 8 字节。

这里的 `01` 分别可能代表 flags=1 和 version=1，位置不同、含义不同。

## 19. 从逻辑帧到 15 字节串口帧

`self._send(frame)` 最终调用当前传输对象：

```python
self._can.send(frame)
```

变量名 `_can` 是历史上的统一传输接口名。当前 YAML 设置：

```yaml
transport: serial
```

所以真实对象是：

```python
SerialTransport(...)
```

不是当前正在使用物理 CAN。

串口模块在 8 字节逻辑帧外增加：

| 字节 | 内容 |
|---:|---|
| 0 | `A5` |
| 1 | `5A` |
| 2 | 串口外层版本 `01` |
| 3～4 | logical ID `0x101`，小端 |
| 5 | 数据长度 `08` |
| 6～13 | 8 字节运动数据 |
| 14 | 外层 CRC-8 |

然后写入：

```text
/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00
```

串口参数为 `115200-8N1`。

## 20. `_send()` 如何处理串口错误

```python
def _send(self, frame):
    if not self._open_can_if_needed():
        return False
    try:
        self._can.send(frame)
        return True
    except OSError as error:
        self._log_can_error(...)
        self._can.close()
        return False
```

逻辑：

1. 接口未打开时尝试打开；
2. 成功则发送；
3. 写串口异常时记录错误；
4. 关闭当前句柄；
5. 后续定时器再尝试重连。

错误日志被限制为最多约每 2 秒一条，避免 50 Hz 失败时每秒刷 50 条日志。

即使 RDK 发不出去，STM32 还会因收不到新命令而独立进入超时停止。

## 21. STM32 收到后做什么

### 21.1 串口层收帧

STM32 USART3 中断：

```text
逐字节接收
→ 寻找 A5 5A
→ 收满 15 字节
→ 检查外层 CRC
→ 放入接收队列
```

### 21.2 协议层解码

`chassis_decode_motion_command()` 检查：

- logical ID 是否为 `0x101`；
- 内层 CRC 是否正确；
- 协议版本是否为 1；
- 然后按相同小端格式恢复线速度、角速度、flags 和 sequence。

### 21.3 保存最新命令和时间

```c
app->command = motion;
app->last_command_ms = now_ms;
app->has_received_command = 1U;
```

如果急停位为 1，STM32 锁存急停故障。

### 21.4 100 Hz 控制任务判断是否允许输出

STM32 自己再次检查：

```text
硬件使能
收到过命令
命令 enable 位
命令超时
急停和严重故障
```

只有全部安全才进入 `RUNNING`。

### 21.5 运动学和 PI 不在 RDK 速度模块中

STM32 根据线速度、角速度和底盘类型计算左右目标，再执行前馈 + PI，最终得到 PWM。

所以整个职责分工是：

```text
RDK：我要车体怎么运动
STM32：左右轮该多快、PWM 应该多少
```

## 22. 双层超时保护

### 22.1 RDK 的 ROS 命令超时

对象：上游 `/cmd_vel`。

```text
如果 ROS 超过 0.20 s 没发新速度
→ RDK 主动发零速度和 enable=false
```

它防止键盘节点、导航控制器或网络输入停止更新时继续沿用旧速度。

### 22.2 STM32 的通信命令超时

对象：RDK 到 STM32 的运动帧。

```text
如果 STM32 超过约 200 ms 没收到新运动帧
→ STM32 进入 TIMEOUT 并停止 PWM
```

它防止 RDK 节点崩溃、USB 线断开或 Python 卡死。

### 22.3 为什么需要两层

故障可能发生在不同位置：

```text
上游控制器停止发布，但 RDK 节点还活着
→ RDK 超时负责

RDK 整个程序或串口失效
→ STM32 超时负责
```

安全设计不应只依赖一台处理器的一段代码。

## 23. 节点关闭时为什么发送三次禁用命令

节点销毁代码：

```python
self._drive_enabled = False
self._emergency_stop = False
for _ in range(3):
    self._send_motion_command()
self._can.close()
```

`for _ in range(3)` 表示重复 3 次。下划线 `_` 表示循环变量本身不需要使用。

目的：退出前尽最大可能让 STM32 收到禁止输出命令。即使某一帧因瞬时问题丢失，后面还有两帧。

这仍不是唯一保护；如果三帧都没送到，STM32 自身超时最终也会停止。

## 24. ROS 执行这些回调的方式

程序启动后：

```python
rclpy.spin(node)
```

可以把它理解成一个事件循环：

```text
等待事件
├─收到 /cmd_vel → 调用 _on_cmd_vel
├─20 ms 定时器到期 → 调用 _send_motion_command
├─收到 enable 服务 → 调用 _on_enable
├─收到 estop 服务 → 调用 _on_estop
└─其他定时器和服务 → 调用对应函数
```

当前主程序使用普通 `rclpy.spin()`，没有显式创建多线程执行器。对入门理解而言，可以先认为这些短回调由事件循环依次执行，而不是每个回调都自己启动一个线程。

这也是为什么回调应尽量短，不能在 `/cmd_vel` 回调里阻塞几秒。

## 25. 一条速度命令的完整时间线

假设节点已启动、车轮架空、软件已使能，发布：

```text
linear.x = 0.05
angular.z = 0.00
```

完整流程：

1. `ros2 topic pub` 或其他节点构造 `Twist`；
2. ROS 2 将消息送到 `/cmd_vel` 订阅者；
3. `_on_cmd_vel()` 保存消息；
4. `_has_received_cmd_vel=True`；
5. 记录单调时间；
6. 下一个 20 ms 定时器到期；
7. 计算命令年龄，小于 0.20 s；
8. 确认 `_drive_enabled=True`；
9. 确认 `_emergency_stop=False`；
10. 读取 `linear.x=0.05`、`angular.z=0`；
11. 生成新 sequence；
12. `make_motion_command()` 生成 ID `0x101` 和 8 字节数据；
13. `SerialTransport` 包成 15 字节；
14. Linux 写入底盘 USB 串口 `0002`；
15. STM32 USART3 中断组帧；
16. STM32 解析并保存运动命令；
17. STM32 100 Hz 控制任务计算左右目标；
18. 编码器提供实测轮速；
19. 前馈 + PI 计算 PWM；
20. 电机驱动输出；
21. STM32 回传轮速、PWM、状态；
22. RDK 发布 `/motor/debug` 和 `/diagnostics`。

## 26. 不同场景下会发生什么

### 场景 A：节点刚启动，没有 `/cmd_vel`

```text
has_received = false
drive_enabled = false
```

结果：50 Hz 发送零速度、禁止输出。

### 场景 B：收到非零 `/cmd_vel`，但未使能

消息会保存，但发送给 STM32 的仍是零速度、禁止输出。

### 场景 C：先收到非零命令，200 ms 内使能

保存的非零命令仍新鲜，可能在使能回调中立即发送。必须理解这一点。

### 场景 D：已使能并持续以 10 Hz 发布

10 Hz 表示每 100 ms 一条，小于 200 ms 超时，RDK 以 50 Hz 重发当前最新速度。

### 场景 E：发布程序停止

最后一条命令超过 200 ms 后，RDK 发送零速度和禁止位。

### 场景 F：之后重新开始发布

如果 RDK 的 `_drive_enabled` 仍为 `True` 且没有急停，新命令可以恢复执行。所以实验结束应明确失能。

### 场景 G：USB 串口断开

RDK 发送失败并尝试重新打开；STM32 因收不到新帧执行下位机超时停止。

### 场景 H：两个底盘节点同时运行

两个进程抢读一个串口，可能各自只拿到部分字节，导致心跳丢失。必须保证只运行一套节点。

## 27. Ubuntu/ROS 观察命令详解

以下命令只用于观察或在失能状态下测试模块。

### 27.1 每个新终端加载环境

```bash
source /opt/ros/humble/setup.bash
source ~/chassis_project/ros2_ws/install/setup.bash
```

第一行加载系统 ROS 2 Humble，第二行加载本项目编译安装结果。

### 27.2 确认底盘节点存在

```bash
ros2 node list
```

应看到：

```text
/chassis_can_node
```

### 27.3 查看节点订阅和服务

```bash
ros2 node info /chassis_can_node
```

在 `Subscribers` 中应看到 `/cmd_vel`，在 `Service Servers` 中应看到 `/chassis/enable` 等服务。

### 27.4 查看 Twist 定义

```bash
ros2 interface show geometry_msgs/msg/Twist
```

这是学习消息字段最可靠的方式。

### 27.5 查看 `/cmd_vel` 发布者和订阅者

```bash
ros2 topic info /cmd_vel -v
```

重点看：

```text
Publisher count
Subscription count
```

如果发布者数量比预期多，不要使能。

### 27.6 观察速度命令

```bash
ros2 topic echo /cmd_vel
```

只显示 ROS 网络中的命令，不代表 STM32 一定正在执行。是否执行还受使能、超时、急停和故障影响。

### 27.7 查看发布频率

```bash
ros2 topic hz /cmd_vel
```

如果频率低于 5 Hz，间隔可能达到或超过 200 ms，不适合当前超时设置。手工测试推荐 10 Hz。

### 27.8 查看话题带宽

```bash
ros2 topic bw /cmd_vel
```

可以观察 ROS 消息数据量；对 `Twist` 来说带宽很小。

## 28. 安全地测试速度处理模块

### 28.1 测试前提

- 车轮架空；
- `/diagnostics` 为 `IDLE / NONE`；
- 心跳小于 2 秒；
- 没有重复节点；
- `/cmd_vel` 没有未知发布者；
- 有人现场观察。

### 28.2 确认当前只有一套底盘节点

```bash
pgrep -af 'chassis_serial.launch.py|chassis_can_node'
```

应只有一个 launch 和一个 node 主进程。

### 28.3 先保持失能观察 `/cmd_vel`

```bash
ros2 service call /chassis/enable std_srvs/srv/SetBool "{data: false}"
```

然后发布低速命令：

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.05}, angular: {z: 0.0}}"
```

此时回调会收到并保存命令，但因为软件失能，电机不应输出。

### 28.4 停止上述非零发布并等待至少 1 秒

按 `Ctrl+C`，然后等待超过 0.20 s，让非零命令失效。

### 28.5 先发布零速度

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.0}}"
```

### 28.6 架空后使能

另一个终端：

```bash
ros2 service call /chassis/enable std_srvs/srv/SetBool "{data: true}"
```

因为最新命令是零，正常不会运动。

### 28.7 切换为低速非零命令

停止零速度发布，再明确启动：

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.05}, angular: {z: 0.0}}"
```

观察 `/motor/debug`：

```bash
ros2 topic echo /motor/debug
```

### 28.8 结束时明确失能

先停止速度发布，再执行：

```bash
ros2 service call /chassis/enable std_srvs/srv/SetBool "{data: false}"
```

不要只停止 `/cmd_vel` 就离开。

## 29. 如何判断这一模块是否正常

### 29.1 ROS 订阅正常

```bash
ros2 topic info /cmd_vel -v
```

应能看到 `/chassis_can_node` 订阅。

### 29.2 消息内容正常

```bash
ros2 topic echo /cmd_vel
```

应看到你发布的 `linear.x`、`angular.z`。

### 29.3 发布频率正常

```bash
ros2 topic hz /cmd_vel
```

手工持续测试约 10 Hz。

### 29.4 STM32 通信正常

```bash
ros2 topic echo --once /diagnostics
```

应看到：

```text
faults: NONE
transport: serial
heartbeat_age: 小于 2 秒
```

### 29.5 闭环目标正常

软件使能且发布低速后：

```bash
ros2 topic echo /motor/debug
```

左右目标速度应随命令变化。若 `/cmd_vel` 有数据但目标始终为零，检查使能、命令频率和急停状态。

## 30. 常见问题与原因

### 30.1 `/cmd_vel` 能看到，但电机不转

可能原因：

1. 软件没有使能；
2. 发布频率太低，命令已超时；
3. 发送的是零速度；
4. 急停未清除；
5. STM32 故障；
6. 硬件使能无效；
7. 串口心跳丢失；
8. 电池欠压。

先看：

```bash
ros2 topic echo --once /diagnostics
ros2 topic hz /cmd_vel
ros2 topic echo /motor/debug
```

### 30.2 调用使能后车辆立刻动了

说明使能前 0.20 s 内存在新鲜非零 `/cmd_vel`。检查所有发布者，并按“先零速度、再使能、再非零”的顺序操作。

### 30.3 停止发布后车停了，重新发布又动了

这是当前设计：超时只暂时发送禁止命令，不会把 `_drive_enabled` 永久清零。需要持续禁止时调用 `/chassis/enable false`。

### 30.4 `/cmd_vel` 发布频率是 1 Hz

每条间隔 1 秒，远大于 0.20 s。结果会变成短暂执行后超时停止，反复启停。使用至少高于 5 Hz，手工测试推荐 10 Hz。

### 30.5 诊断显示 `heartbeat missing`

这通常不是 `/cmd_vel` 消息格式错误，而是 RDK 与 STM32 遥测通信有问题。检查串口、STM32 供电和重复节点。

### 30.6 同时运行键盘和导航

两个发布者会交错覆盖。正式系统一般需要速度仲裁器统一多个来源，而不是直接让多个控制器无管理地发布到同一个 `/cmd_vel`。

## 31. 当前模块的参数边界

### 31.1 `command_rate_hz`

决定 RDK 向 STM32 生成运动帧的频率，当前 50 Hz。它在节点初始化时用于创建定时器，修改 YAML 后需要重新编译并重启节点。

### 31.2 `ros_command_timeout_s`

决定 ROS `/cmd_vel` 多久未更新就视为过期，当前 0.20 s。

超时不宜太短，否则系统偶发调度延迟就会停；也不宜太长，否则上游失效后旧命令保持太久。

### 31.3 最大速度限制在哪里

RDK 协议层会限制到 int16 可表达范围，但真正面向小车的安全速度限制主要在 STM32 参数：

```text
limits.maximum_linear_speed_mps
maximum_angular_speed_radps
```

也就是说，即使 ROS 错误发布很大的有限数值，STM32 仍会按配置限幅。但不能因此故意发送危险值。

## 32. 这个模块与其他上位机模块的连接

### 32.1 与 ROS 主节点模块

主节点初始化订阅者、服务、参数和定时器；速度处理模块依赖它们才能被 ROS 调用。

### 32.2 与协议模块 `can_protocol.py`

速度处理负责决定“发什么”，协议模块负责决定“每个字节怎样排列”。

### 32.3 与串口模块 `serial_transport.py`

协议模块得到逻辑帧后，串口模块负责“怎样送到 STM32”。

### 32.4 与诊断模块

速度命令中的 sequence 会由 STM32 状态帧回传；串口和 STM32 状态则显示在 `/diagnostics`。

### 32.5 与 `/motor/debug`

速度处理发出车体目标；STM32 运动学和 PI 运行后回传左右目标、实测和 PWM，RDK 发布 `/motor/debug`。它是验证命令最终执行情况的重要反馈。

### 32.6 与里程计模块

速度命令是“想怎么走”，编码器里程计是“估计实际走了多少”。里程计不会直接使用 `/cmd_vel` 积分，而是使用 STM32 回传的编码器累计计数。

### 32.7 与参数模块

轮径、轮距、底盘类型、速度上限等参数会影响 STM32 如何把车体速度转换为轮速，但 `/cmd_vel` 回调本身只保存消息。

### 32.8 与 IMU

当前速度处理模块不读取 IMU，RDK 也还没有 `/imu/data`。当前闭环反馈来自编码器。

## 33. 建议你亲自做的只读练习

这些练习不使能电机，也能帮助理解 ROS 速度模块。

### 练习 1：看消息结构

```bash
ros2 interface show geometry_msgs/msg/Twist
```

### 练习 2：看订阅关系

```bash
ros2 node info /chassis_can_node
```

找到 `/cmd_vel`。

### 练习 3：保持失能并发布命令

```bash
ros2 service call /chassis/enable std_srvs/srv/SetBool "{data: false}"
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.05}, angular: {z: 0.0}}"
```

另一个终端：

```bash
ros2 topic echo /cmd_vel
```

理解“ROS 收到命令”和“电机被允许执行”是两件事。

### 练习 4：观察发布停止

停止 `topic pub`，理解 0.20 s 后命令变过期。因为当前没有直接发布 `command_is_fresh`，可结合代码和 `/motor/debug` 的目标值理解，但全程保持软件失能。

### 练习 5：检查发布者数量

```bash
ros2 topic info /cmd_vel -v
```

分别在发布程序运行和停止时观察 `Publisher count`。

## 34. 阅读本模块代码的推荐顺序

1. `chassis_serial.yaml` 中的 `command_rate_hz` 和 `ros_command_timeout_s`；
2. `chassis_can_node.py` 中五个初始状态变量；
3. `create_subscription()`；
4. `_on_cmd_vel()`；
5. `create_timer()`；
6. `_send_motion_command()`；
7. `_on_enable()`；
8. `_on_estop()`；
9. `_next_sequence()`；
10. `can_protocol.py:make_motion_command()`；
11. `_send()`；
12. `serial_transport.py:send()`；
13. STM32 `chassis_decode_motion_command()`；
14. STM32 `chassis_app_control_step()`。

每个函数都问四个问题：

```text
谁在什么时候调用它？
输入数据是什么？
它修改了哪些状态？
它把结果交给了谁？
```

## 35. 最终总结

ROS 速度命令处理模块可以浓缩成六步：

```text
1. 订阅 /cmd_vel
2. 回调保存最新 Twist 和到达时间
3. 50 Hz 定时器检查新鲜度、使能和急停
4. 不安全时主动生成零速度禁止帧
5. 安全时把 linear.x、angular.z 编码成运动帧
6. 串口发送给 STM32，由 STM32 完成运动学、PI 和 PWM
```

必须牢记三个安全结论：

1. 收到 `/cmd_vel` 不等于电机一定执行，必须同时满足使能和安全状态；
2. 使能前如果存在新鲜非零命令，车辆可能立即运动；
3. 命令超时只是暂时禁止输出，不等于永久软件失能，实验结束必须显式调用 `/chassis/enable false`。

---

# 第二部分：首次文档之后的追问与补充

## 后续追问：命令新鲜度、服务回调、超时与 `make_motion_command()`

### 原问题

> 为什么要判断命令是否新鲜？服务回调中的 request 和 response 是什么？超时为什么要发送停止指令，是不是必须持续发送命令？flags 的位运算是什么意思？请详细解释 make_motion_command()。

### 1. “命令新鲜”指时间没有过期

`_latest_command` 只是普通 Python 变量。收到一次 `0.2 m/s` 后，即使导航、键盘节点或网络已经中断，这个变量仍会保存旧值，不会自动变成零。

节点因此记录：

```text
是否收到过 /cmd_vel
最后一次收到 /cmd_vel 的单调时钟时间
```

判断近似为：

```python
command_is_fresh = (
    self._has_received_cmd_vel
    and time.monotonic() - self._last_cmd_vel_time <= timeout
)
```

当前超时约为 0.20 s。上层正常控制时应持续更新 `/cmd_vel`；停发超过超时时间就代表控制来源可能已经失效，不能继续执行旧速度。

### 2. 超时后究竟发生什么

RDK 的发送定时器仍以约 50 Hz 工作，但会把输出改成：

```text
linear = 0
angular = 0
enable = false
```

所以“停发 `/cmd_vel`”会导致小车自动停止。这样即使控制程序崩溃，也不会因为内存里还保存着最后一条前进命令而持续运动。STM32 还有独立的约 200 ms 合法命令超时保护，形成双层安全链。

### 3. 服务回调的 request 与 response

ROS 2 服务是一次请求、一次响应。例如：

```python
self.create_service(SetBool, "chassis/enable", self._on_enable)
```

`SetBool` 的接口可理解为：

```text
bool data       # request
---
bool success    # response
string message  # response
```

调用：

```bash
ros2 service call /chassis/enable std_srvs/srv/SetBool "{data: true}"
```

ROS 2 框架创建 `request` 和 `response` 对象，再调用：

```python
_on_enable(request, response)
```

回调读取 `request.data`，修改 `_drive_enabled`，填写 `response.success` 与 `response.message`，最后 `return response`。立即调用 `_send_motion_command()` 可以让失能或急停不必等待下一个 20 ms 定时周期。

### 4. `flags` 条件表达式和按位或

```python
flags = (
    (CMD_ENABLE if enable else 0)
    | (CMD_ESTOP if emergency_stop else 0)
)
```

可拆成：

```python
enable_bits = CMD_ENABLE if enable else 0
estop_bits = CMD_ESTOP if emergency_stop else 0
flags = enable_bits | estop_bits
```

假设：

```text
CMD_ENABLE = 0b00000001
CMD_ESTOP  = 0b00000010
```

那么：

| enable | estop | flags |
|---|---|---|
| false | false | `00000000` |
| true | false | `00000001` |
| false | true | `00000010` |
| true | true | `00000011` |

按位或 `|` 能把两个互不冲突的开关装进同一个字节。

### 5. `make_motion_command()` 的完整工作

函数收到：

```text
linear_mps
angular_rps
enable
emergency_stop
sequence
```

主要步骤是：

1. 把 m/s 换成整数 mm/s，把 rad/s 换成整数 mrad/s；
2. 限制到 `int16` 和项目允许范围；
3. 生成上面的 `flags`；
4. 写入协议版本和滚动序号；
5. 使用小端序打包前 7 字节；
6. 对前 7 字节计算 CRC8，形成第 8 字节；
7. 返回逻辑 ID 为 `0x101` 的 `CanFrame` 对象。

布局为：

```text
data[0..1]  linear，mm/s，int16，小端
data[2..3]  angular，mrad/s，int16，小端
data[4]     flags
data[5]     protocol version
data[6]     sequence
data[7]     CRC8
```

随后串口传输模块再把 `0x101 + data[8]` 包装为 15 字节外层串口帧。逻辑编码与物理运输是两个步骤。

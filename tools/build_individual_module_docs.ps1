$ErrorActionPreference = 'Stop'

$workspaceRoot = Split-Path -Parent $PSScriptRoot
$docsRoot = Join-Path $workspaceRoot 'docs'
$outputRoot = Join-Path $docsRoot '模块讲解_融合版'
$utf8NoBom = [System.Text.UTF8Encoding]::new($false)

if (-not (Test-Path -LiteralPath $outputRoot)) {
    [System.IO.Directory]::CreateDirectory($outputRoot) | Out-Null
}

$lower8Followup = @'
## 后续追问：PWM 为什么能让电机转动，为什么一个电机需要两个输入

### 原问题

> 输入 PWM 波是怎么控制电机转动的？为什么一个电机要两个输入？电机的组成结构是怎样的？

### 1. 从电机内部结构开始理解

当前底盘使用的可理解为“有刷直流减速电机 + AB 相编码器”。主要结构包括：

```text
定子永久磁铁
转子铁芯与铜线绕组
电刷与换向器
电机轴与轴承
减速齿轮箱
AB 相编码器
```

绕组通电后产生磁场，与定子磁场作用形成电磁转矩。换向器和电刷随转子转动改变绕组电流方向，使转矩持续推动转子向同一方向旋转。电机本体通常转速高、转矩小，减速箱以降低输出转速为代价放大输出转矩。编码器不负责驱动，只负责把实际转动转换成 A/B 相脉冲反馈给 STM32。

近似关系是：

```text
电机转矩 ≈ 转矩常数 × 电机电流
```

所以 PWM 不是直接“规定转速”，而是先改变驱动器施加到绕组上的有效电压和电流，电流改变转矩，转矩再在负载、摩擦和反电动势共同作用下形成实际速度。

### 2. STM32 为什么不能直接连接电机

STM32 GPIO 只能输出 3.3 V、小电流逻辑信号，既不能提供电机所需的大电流，也无法安全承受绕组关断时产生的反向电压。真实路径是：

```text
STM32 两路 PWM/逻辑信号
        ↓
AT8236 电机驱动器/H 桥功率级
        ↓
电池大电流
        ↓
电机绕组
```

### 3. PWM 实际改变什么

PWM 在一个固定周期内高速切换开和关：

```text
占空比 = 高电平持续时间 / PWM 周期
```

占空比越大，驱动器在一个周期中施加有效电压的时间通常越长。电机绕组具有电感，电流不能瞬间突变，所以约 10 kHz 的高速开关会形成相对平滑的平均电流。

但 PWM 与转速不是固定的一一对应关系。电池电压、负载、地面摩擦、左右电机差异和反电动势都会改变结果，因此必须使用编码器闭环：

```text
目标速度
→ 控制器计算 PWM
→ H 桥驱动电机
→ 编码器测量实际速度
→ 实际速度反馈给控制器
```

### 4. 为什么需要两个驱动输入

电机本体只有两根动力线，但 STM32 控制的是驱动器的两个逻辑输入。直流电机方向取决于电机两端电压极性：

```text
端子 A 为正、B 为负 → 一个方向
端子 B 为正、A 为负 → 相反方向
```

H 桥用四个功率开关交换电流方向：

```text
          电池正极
        Q1       Q2
          A—电机—B
        Q3       Q4
          电池负极
```

- 打开 Q1、Q4：电流从 A 流向 B；
- 打开 Q2、Q3：电流从 B 流向 A；
- 两端处于相同电位时可形成滑行或电气制动，具体取决于驱动芯片真值表。

当前底层为一个电机提供两路 PWM。`Set_Pwm()` 根据 PWM 数值正负决定哪一路接近全高、哪一路被调制：

```text
PWM 绝对值 → 有效驱动比例
PWM 正负号 → 两路信号的组合 → 电机电流方向
```

这就是代码中“两路桥臂”和正负 PWM 的物理含义。

### 5. 为什么堵转危险

电机转动时会产生与转速近似成正比的反电动势：

```text
电流 ≈（有效供电电压 - 反电动势）/ 绕组电阻
```

堵转时转速接近 0，反电动势也接近 0，电流可能非常大。因此项目同时观察目标速度、实际速度和 PWM，持续满足“目标较大、PWM 较大但实际速度接近 0”时触发堵转保护。
'@

$upper2Followup = @'
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
'@

$upper3Followup = @'
## 后续追问：`make_parameter_command()` 怎样与 ROS 参数模块连接

### 原问题

> `make_parameter_command()` 和 ROS 参数模块怎样连接？

### 1. 它们不是直接连接，而是由主节点协调

三个模块的职责不同：

```text
ROS 参数系统：保存“参数名 → 当前值”
主节点：验证、查 ID、排队、超时重试和匹配应答
协议模块：把数字 ID、操作和值编码成 8 字节
```

例如终端执行：

```bash
ros2 param set /chassis_can_node controller.left_kp 3100.0
```

完整链路是：

```text
ROS 参数修改请求
→ _on_parameters_changed()
→ 检查底盘状态与数值范围
→ PARAMETER_IDS 把 controller.left_kp 映射为 ID 1
→ 把 (1, 3100.0) 放入参数队列
→ 参数发送定时器取队头
→ make_parameter_command(1, WRITE, 3100.0, sequence)
→ 串口发送 0x102
→ STM32 修改 RAM 参数并返回 0x186
→ RDK 核对 parameter_id、sequence 和 status
→ 成功后才从队列删除
```

### 2. `make_parameter_command()` 的字节布局

其核心打包格式可表示为：

```python
struct.pack("<BBfB", parameter_id, operation, value, sequence)
```

```text
<   小端序
B   1 字节参数 ID
B   1 字节读/写操作
f   4 字节 float32 数值
B   1 字节序号
```

前 7 字节再添加 CRC8，得到 `0x102` 的 8 字节逻辑载荷。

### 3. 为什么要有队列、pending 和 sequence

参数不是一股脑同时发送，而是一次只等待一个应答：

```text
队列头参数已发送
→ 保存 PendingParameter
→ 等待相同 ID 和 sequence 的 0x186
→ 成功后 pop 左侧队头
→ 再发送下一个
```

超过约 200 ms 没有正确应答会重发，最多尝试 3 次。仍失败则设置 `parameter_transfer_failed=True`，避免悄悄丢参数。

### 4. ROS 参数、YAML、STM32 RAM 与 Flash 必须分开

```text
ROS 当前参数：节点运行时的值
YAML：下次节点启动时加载的配置文件
STM32 RAM：本次运行使用的控制参数
STM32 Flash：掉电后仍保留的参数
```

`ros2 param set` 成功不代表 YAML 已改，也不代表 Flash 已保存。推荐流程：

```text
底盘失能
→ param set 临时修改
→ 参数下发成功
→ 架空验证
→ 同步修改 YAML
→ 最后显式调用 save_parameters 写 Flash
```

`/chassis/push_parameters` 会把 RDK 当前全部 ROS 参数重新排队下发；是否启动时自动下发由 `push_parameters_on_start` 决定，当前默认关闭以避免无意覆盖 STM32 已保存参数。
'@

$upper4Followup = @'
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
'@

$upper8Followup = @'
## 后续追问一：`setup.bash`、代码部署与 `bash` 命令

### 1. `setup.bash` 是什么

`setup.bash` 是为当前 Bash 终端加载环境变量的脚本，不直接控制电机。

```bash
source /opt/ros/humble/setup.bash
```

让当前终端认识系统安装的 ROS 2 Humble、ROS 命令、消息和 Python 库。

```bash
source ~/chassis_project/ros2_ws/install/setup.bash
```

让终端在系统 ROS 2 基础上继续认识本工作空间安装的 `chassis_can_control` 包、节点、Launch 和配置文件。每个新终端都需要重新加载，除非写入终端启动配置。

`setup.py` 与 `setup.bash` 不同：前者描述 Python 功能包如何安装，后者是构建后生成的环境加载脚本。

### 2. Windows 代码怎样进入 RDK X5

```text
Windows 工作区中的 ros2_ws/src/chassis_can_control
→ 通过 SSH/SFTP 复制
RDK: ~/chassis_project/ros2_ws/src/chassis_can_control
→ colcon build
RDK: ~/chassis_project/ros2_ws/install/chassis_can_control
→ source install/setup.bash
ROS 2 能够查找并启动节点
```

常用重新构建流程：

```bash
cd ~/chassis_project/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select chassis_can_control
source install/setup.bash
```

Python 虽不编译成 STM32 那样的机器码，但 ROS 2 仍需安装模块、登记入口、复制 Launch/YAML 并生成环境脚本。

### 3. 以 `bash` 开头是什么意思

```bash
bash path/to/start_chassis_serial_safe.sh
```

表示启动 Bash 解释器，依次执行 `.sh` 文件中的命令。`source script.sh` 则在当前终端执行，使环境变量保留；`./script.sh` 要求脚本有执行权限和正确的 shebang。当前明确写 `bash script.sh` 可避免执行权限和 `sh/dash` 兼容问题。

## 后续追问二：`STM32 heartbeat missing` 是否正常

以下状态不正常，不能使能电机：

```text
level: 2
message: STM32 heartbeat missing
heartbeat_age: 很大
```

`pending_parameters: 0` 只表示没有排队参数，`parameter_transfer_failed: False` 只表示参数通道未报告失败，都不能证明串口正常。

安全排查顺序：

1. 调用 `/chassis/enable` 设为 false；
2. 用 `pgrep` 确认只有一套 Launch/节点；
3. 用 `ls -l /dev/serial/by-id/` 确认 `0002` 存在；
4. 用 `fuser -v` 检查串口是否只被一个节点占用；
5. 查看节点日志中的 `Permission denied`、`Input/output error` 等；
6. 安全停止旧节点，只启动一套串口版节点；
7. 正常诊断应为 `IDLE / NONE` 且 `heartbeat_age` 很小。

## 后续追问三：本次实机故障的最终根因与修复

当时进一步检查确认：

- RDK 地址、`0002` 设备、权限和单节点均正常；
- 停止 ROS 后直接读取串口仍为 0 字节；
- 发送合法零速失能探测帧也无响应；
- ST-Link 读取 Flash 后发现 STM32 中实际仍是 CAN 版固件。

最终处理为：

```text
备份完整 512 KiB Flash
→ 重新编译串口版，0 Error / 0 Warning
→ 使用扇区擦除重新烧录串口版
→ 烧录后回读校验
→ 保持 0x08060000 参数 sector 不变
→ 复位 MCU
```

恢复后诊断为：

```text
message: IDLE
state: IDLE
faults: NONE
heartbeat_age: 约 0.05 s
transport: serial
```

这次故障说明：RDK 选择 `transport: serial` 并不意味着 STM32 会自动变成串口固件；上下位机物理运输方式必须一致。若 STM32 烧的是 CAN 版，RDK 串口设备即使能正常打开，也收不到协议心跳。
'@

$modules = @(
    [pscustomobject]@{ Order = 1; Side = '下位机'; Number = 1; Name = '系统启动与硬件初始化模块'; Output = '下位机01_系统启动与硬件初始化模块_含后续问答.md'; Primary = '17_STM32下位机系统启动与硬件初始化深度详解.md'; Supplements = @('18_delay_init_CAN通信与PWM定时器答疑.md'); Inline = '' },
    [pscustomobject]@{ Order = 2; Side = '下位机'; Number = 2; Name = 'FreeRTOS 任务调度模块'; Output = '下位机02_FreeRTOS任务调度模块_含后续问答.md'; Primary = '18_STM32下位机FreeRTOS任务调度模块深度详解.md'; Supplements = @('19_USART3串口任务_DMA_dt_s与遥测专题详解.md'); Inline = '' },
    [pscustomobject]@{ Order = 3; Side = '下位机'; Number = 3; Name = 'STM32 串口通信模块'; Output = '下位机03_STM32串口通信模块_含后续问答.md'; Primary = '17_STM32下位机串口通信代码深度详解.md'; Supplements = @('18_小端序_0x101_CAN与上下位机数据流详解.md'); Inline = '' },
    [pscustomobject]@{ Order = 4; Side = '下位机'; Number = 4; Name = '上下位机逻辑协议与通信模块'; Output = '下位机04_上下位机逻辑协议与通信模块.md'; Primary = '17_上下位机串口通信代码深度详解.md'; Supplements = @(); Inline = '' },
    [pscustomobject]@{ Order = 5; Side = '下位机'; Number = 5; Name = '底盘核心控制 chassis_app 模块'; Output = '下位机05_底盘核心控制chassis_app模块_含后续问答.md'; Primary = '17_STM32底盘核心控制模块chassis_app深入讲解.md'; Supplements = @('18_底盘控制五个核心疑问_从物理原理到代码计算.md'); Inline = '' },
    [pscustomobject]@{ Order = 6; Side = '下位机'; Number = 6; Name = '单轮电机闭环控制模块'; Output = '下位机06_单轮电机闭环控制模块_含后续问答.md'; Primary = '17_单轮电机闭环控制模块逐行深入讲解.md'; Supplements = @('18_单轮闭环五个问题补充说明.md','19_目标加减速度与完整闭环逐周期算例.md'); Inline = '' },
    [pscustomobject]@{ Order = 7; Side = '下位机'; Number = 7; Name = 'C30D 硬件接口适配模块'; Output = '下位机07_C30D硬件接口适配模块.md'; Primary = '17_STM32下位机硬件接口适配模块深入讲解.md'; Supplements = @(); Inline = '' },
    [pscustomobject]@{ Order = 8; Side = '下位机'; Number = 8; Name = '底层硬件驱动与 IMU 模块'; Output = '下位机08_底层硬件驱动与IMU模块_含后续问答.md'; Primary = '17_STM32底层硬件驱动与ICM20948专项详解.md'; Supplements = @(); Inline = $lower8Followup },

    [pscustomobject]@{ Order = 9; Side = '上位机'; Number = 1; Name = 'ROS 2 主节点模块'; Output = '上位机01_ROS2主节点模块_含后续问答.md'; Primary = '17_ROS2主节点模块零基础精讲_chassis_can_node.md'; Supplements = @('18_从零理解类对象ROS节点Launch_YAML与cmd_vel订阅.md'); Inline = '' },
    [pscustomobject]@{ Order = 10; Side = '上位机'; Number = 2; Name = 'ROS 速度命令处理模块'; Output = '上位机02_ROS速度命令处理模块_含后续问答.md'; Primary = '17_RDK_X5_ROS速度命令处理模块详解.md'; Supplements = @(); Inline = $upper2Followup },
    [pscustomobject]@{ Order = 11; Side = '上位机'; Number = 3; Name = 'Python 协议编解码模块'; Output = '上位机03_Python协议编解码模块_含后续问答.md'; Primary = '17_上位机Python协议编解码模块零基础超详细讲解.md'; Supplements = @(); Inline = $upper3Followup },
    [pscustomobject]@{ Order = 12; Side = '上位机'; Number = 4; Name = 'Linux 串口传输模块'; Output = '上位机04_Linux串口传输模块_含后续问答.md'; Primary = '17_RDK_X5_Linux串口传输模块零基础详解.md'; Supplements = @(); Inline = $upper4Followup },
    [pscustomobject]@{ Order = 13; Side = '上位机'; Number = 5; Name = '编码器里程计模块'; Output = '上位机05_编码器里程计模块_含后续问答.md'; Primary = '17_上位机编码器里程计模块零基础详解.md'; Supplements = @('19_wrapped_count_delta_中点法_base_link与TF专项详解.md'); Inline = '' },
    [pscustomobject]@{ Order = 14; Side = '上位机'; Number = 6; Name = 'ROS 状态与调试数据模块'; Output = '上位机06_ROS状态与调试数据模块.md'; Primary = '17_ROS2状态与调试数据模块零基础详解.md'; Supplements = @(); Inline = '' },
    [pscustomobject]@{ Order = 15; Side = '上位机'; Number = 7; Name = 'ROS 服务与参数管理模块'; Output = '上位机07_ROS服务与参数管理模块.md'; Primary = '17_ROS2服务与参数管理模块零基础详解.md'; Supplements = @(); Inline = '' },
    [pscustomobject]@{ Order = 16; Side = '上位机'; Number = 8; Name = '配置、安装与启动模块'; Output = '上位机08_配置安装与启动模块_含后续问答.md'; Primary = '17_RDK_X5配置启动与安装模块零基础详解.md'; Supplements = @(); Inline = $upper8Followup }
)

foreach ($module in $modules) {
    $outputPath = Join-Path $outputRoot $module.Output
    $writer = [System.IO.StreamWriter]::new($outputPath, $false, $utf8NoBom)
    try {
        $writer.NewLine = "`n"
        $writer.WriteLine("# $($module.Side)第 $($module.Number) 模块：$($module.Name)")
        $writer.WriteLine()
        $writer.WriteLine('> 本文件是该模块在旧任务中的完整归档：保留首次讲解正文，并把首次文档之后的专项追问统一融合在同一文件中。以后无需回到原任务查找。')
        $writer.WriteLine()
        $writer.WriteLine('## 本模块归档内容')
        $writer.WriteLine()
        $writer.WriteLine(('- 首次讲解来源：`docs/{0}`' -f $module.Primary))
        foreach ($supplement in $module.Supplements) {
            $writer.WriteLine(('- 后续追问来源：`docs/{0}`' -f $supplement))
        }
        if ($module.Inline) {
            $writer.WriteLine('- 后续追问来源：原模块任务中的对话回答（此前未单独保存为文档）')
        }
        if (($module.Supplements.Count -eq 0) -and -not $module.Inline) {
            $writer.WriteLine('- 首次文档之后没有新增专项追问。')
        }
        $writer.WriteLine()
        $writer.WriteLine('---')
        $writer.WriteLine()
        $writer.WriteLine('# 第一部分：首次完整讲解')
        $writer.WriteLine()
        $primaryPath = Join-Path $docsRoot $module.Primary
        $primaryText = [System.IO.File]::ReadAllText($primaryPath, $utf8NoBom)
        $writer.Write($primaryText)
        if (-not $primaryText.EndsWith("`n")) { $writer.WriteLine() }

        if (($module.Supplements.Count -gt 0) -or $module.Inline) {
            $writer.WriteLine()
            $writer.WriteLine('---')
            $writer.WriteLine()
            $writer.WriteLine('# 第二部分：首次文档之后的追问与补充')
            $writer.WriteLine()
        }

        $supplementIndex = 0
        foreach ($supplement in $module.Supplements) {
            $supplementIndex++
            $writer.WriteLine("## 补充文档 $supplementIndex")
            $writer.WriteLine()
            $writer.WriteLine(('来源：`docs/{0}`' -f $supplement))
            $writer.WriteLine()
            $supplementPath = Join-Path $docsRoot $supplement
            $supplementText = [System.IO.File]::ReadAllText($supplementPath, $utf8NoBom)
            $writer.Write($supplementText)
            if (-not $supplementText.EndsWith("`n")) { $writer.WriteLine() }
            $writer.WriteLine()
        }

        if ($module.Inline) {
            $writer.WriteLine($module.Inline)
        }
    }
    finally {
        $writer.Dispose()
    }
}

Write-Output "OUTPUT_ROOT=$outputRoot"
Write-Output "MODULE_COUNT=$($modules.Count)"
foreach ($module in $modules) {
    $path = Join-Path $outputRoot $module.Output
    Write-Output ("{0}`t{1}`t{2}" -f $module.Side, $module.Number, $path)
}

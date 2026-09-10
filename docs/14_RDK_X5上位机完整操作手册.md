# RDK X5 上位机完整操作手册（C30D 串口电机控制版）

## 1. 文档适用范围

本文档只说明当前已经部署到 **RDK X5** 的 ROS 2 Humble 上位机程序，适用于：

- RDK X5 通过 USB 串口与小车 STM32 底盘板通信；
- 串口设备序列号为 `0002`，波特率为 `115200`；
- ROS 2 功能包名为 `chassis_can_control`；
- RDK 用户名为 `wheeltec`；
- 工程目录为 `~/chassis_project/ros2_ws`。

当前实车使用的是**串口版本**。CAN 版本虽然保留在工程中，但没有 CAN 连接线之前不要切换，也不要同时启动串口版和 CAN 版节点。

## 2. 上位机程序实现了什么

RDK X5 上的 `chassis_can_node` 是 ROS 2 和 STM32 电机控制程序之间的桥梁：

```text
ROS 2 /cmd_vel
       ↓
RDK X5 chassis_can_node
       ↓ USB 串口，115200-8N1
STM32F407 底盘控制程序
       ↓
左右电机、编码器和舵机
```

RDK 节点负责：

1. 接收 ROS 2 `/cmd_vel` 速度命令；
2. 通过串口向 STM32 下发线速度、角速度、使能和急停命令；
3. 接收轮速、编码器、电机 PWM、故障、心跳和电池电压；
4. 发布 `/odom`、`/joint_states`、`/tf`、`/diagnostics` 和 `/motor/debug`；
5. 提供软件使能、急停、清故障、参数下发、参数保存和里程计复位服务；
6. 在 ROS 命令超时或通信异常时自动发送零速度和禁止输出命令。

电机 PI 闭环、编码器测速、堵转保护和最终 PWM 输出运行在 STM32 上。RDK 负责 ROS 接口、数据记录、状态监控和参数管理。

## 3. 必须遵守的安全规则

第一次测试、修改参数后第一次测试、恢复故障后的第一次测试，都必须把车轮架空。

1. 启动 ROS 节点不会自动使能电机；
2. 必须显式调用 `/chassis/enable` 才允许执行速度；
3. `/cmd_vel` 停止超过 `0.20 s` 后，RDK 自动下发零速度；
4. STM32 还有独立的 `200 ms` 命令超时保护；
5. 离开小车前先软件失能，再停止节点；
6. 电机运行时禁止修改控制参数、复位里程计或插拔底盘串口；
7. 不要同时启动两个 `chassis_can_node`；
8. 不要把雷达串口 `0001` 当成底盘串口 `0002`；
9. 不要使用单次高速命令做首次测试；
10. 出现方向错误、异响、持续堵转或失控迹象时立即执行急停。

## 4. 硬件连接与网络连接

### 4.1 底盘串口

当前底盘控制板已经通过 USB 接到 RDK X5。RDK 使用稳定设备名：

```text
/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00
```

不要在配置中改成 `/dev/ttyACM0` 或 `/dev/ttyUSB0`，这些编号可能在重启后变化。

### 4.2 从 Ubuntu 虚拟机登录 RDK

确保虚拟机和 RDK 在同一局域网，然后在虚拟机终端执行：

```bash
ping -c 4 192.168.0.100
ssh wheeltec@192.168.0.100
```

第一次连接可能出现主机指纹确认，输入：

```text
yes
```

然后输入 RDK 登录密码。输入密码时终端不显示字符，这是正常现象。

`192.168.0.100` 是当前使用的地址。如果路由器重新分配了地址，应在 RDK 本机执行下面命令确认：

```bash
hostname -I
```

## 5. RDK 上的工程位置

登录 RDK 后：

```bash
cd ~/chassis_project/ros2_ws
pwd
```

正常输出：

```text
/home/wheeltec/chassis_project/ros2_ws
```

主要目录：

```text
ros2_ws/
├── src/chassis_can_control/
│   ├── chassis_can_control/       Python 节点、串口、协议和里程计
│   ├── config/
│   │   └── chassis_serial.yaml    当前串口版参数
│   ├── launch/
│   │   └── chassis_serial.launch.py
│   ├── scripts/
│   │   ├── start_chassis_serial_safe.sh
│   │   └── stop_chassis_serial_safe.sh
│   ├── test/
│   ├── package.xml
│   └── setup.py
├── build/                         编译中间文件
├── install/                       ROS 实际加载的安装结果
└── log/                           colcon 编译日志
```

## 6. 每次登录终端后加载 ROS 环境

普通操作终端执行：

```bash
source /opt/ros/humble/setup.bash
source ~/chassis_project/ros2_ws/install/setup.bash
```

验证功能包是否可见：

```bash
ros2 pkg prefix chassis_can_control
```

正常应输出类似：

```text
/home/wheeltec/chassis_project/ros2_ws/install/chassis_can_control
```

安全启动脚本会自动加载环境，因此执行安全启动脚本前不必手工 `source`。但是查看话题、调用服务和发送命令的新终端仍需加载环境。

## 7. 开机后的标准安全启动流程

### 7.1 确认 USB 串口

```bash
ls -l /dev/serial/by-id/
```

必须能找到：

```text
usb-WCH.CN_USB_Single_Serial_0002-if00
```

本车资料中的对应关系：

```text
0001 = 雷达
0002 = 底盘控制器
```

### 7.2 检查节点是否已经运行

```bash
pgrep -af 'chassis_serial.launch.py|chassis_can_node'
```

- 没有输出：节点没有运行，可以继续启动；
- 已出现 `chassis_serial.launch.py` 和 `chassis_can_node`：节点已经运行，不要重复启动。

当前 RDK 上可能已经有后台节点。判断是否需要启动必须以本命令结果为准。

### 7.3 安全启动

```bash
cd ~/chassis_project/ros2_ws
bash src/chassis_can_control/scripts/start_chassis_serial_safe.sh
```

必须使用 `bash`，不要使用 `sh`。正常会看到：

```text
Available stable serial device names:
Starting serial chassis node with software motor enable OFF.
Chassis serial node started ...
```

这个终端要保持打开。`Ctrl+C` 会停止节点。启动节点以后，电机仍然处于软件失能状态。

### 7.4 另开终端查看诊断

重新 SSH 登录 RDK，并执行：

```bash
source /opt/ros/humble/setup.bash
source ~/chassis_project/ros2_ws/install/setup.bash
ros2 topic echo /diagnostics
```

正常状态应满足：

```text
message: IDLE
faults: NONE
transport: serial
interface: /dev/serial/by-id/...0002-if00
heartbeat_age: 小于 2 秒
pending_parameters: 0
parameter_transfer_failed: False
```

按 `Ctrl+C` 退出持续显示。

只查看一帧可以执行：

```bash
ros2 topic echo --once /diagnostics
```

## 8. 第一次低速架空测试

### 8.1 测试前确认

必须同时满足：

- 车轮已经架空；
- 串口使用 `0002`；
- `/diagnostics` 为 `IDLE`；
- `faults` 为 `NONE`；
- `heartbeat_age` 小于 2 秒；
- 周围没有人接触车轮。

### 8.2 软件使能

```bash
ros2 service call /chassis/enable std_srvs/srv/SetBool "{data: true}"
```

正常响应：

```text
success=True
message='drive enabled'
```

使能并不等于立即运动。只有持续收到新鲜的 `/cmd_vel`，节点才会允许速度输出。

### 8.3 低速前进测试

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.05}, angular: {z: 0.0}}"
```

含义：以 10 Hz 持续发布 `0.05 m/s` 的前进命令。观察两侧车轮方向是否正确，有异常立即按 `Ctrl+C`。

### 8.4 停止运动并失能

先在速度发布终端按 `Ctrl+C`，然后执行：

```bash
ros2 service call /chassis/enable std_srvs/srv/SetBool "{data: false}"
```

正常响应：

```text
success=True
message='drive disabled'
```

### 8.5 其他测试命令

低速后退：

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: -0.05}, angular: {z: 0.0}}"
```

低速转向测试：

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.05}, angular: {z: 0.15}}"
```

每次测试结束都执行软件失能。第一次测试不要超过 `0.05 m/s`。

## 9. 正常停止、急停与清故障

### 9.1 日常停止

首先停止 `/cmd_vel` 发布，再执行：

```bash
ros2 service call /chassis/enable std_srvs/srv/SetBool "{data: false}"
```

如果还要关闭 ROS 节点：

```bash
bash ~/chassis_project/ros2_ws/src/chassis_can_control/scripts/stop_chassis_serial_safe.sh
```

该脚本会先尝试软件失能，再结束本项目节点。

### 9.2 紧急停止

发生异常时执行：

```bash
ros2 service call /chassis/emergency_stop std_srvs/srv/SetBool "{data: true}"
```

急停会同时撤销软件使能，并让 STM32 锁存急停故障。仅把服务参数改回 `false` 不会清除 STM32 已锁存的故障。

### 9.3 清除故障

先排除真实故障原因并保持车轮架空，然后：

```bash
ros2 service call /chassis/clear_faults std_srvs/srv/Trigger "{}"
```

再检查：

```bash
ros2 topic echo --once /diagnostics
```

只有恢复到 `IDLE / NONE` 后才可以重新调用 `/chassis/enable`。

## 10. ROS 2 接口说明

### 10.1 订阅的话题

| 话题 | 类型 | 作用 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 接收线速度 `linear.x` 和角速度 `angular.z` |

### 10.2 发布的话题

| 话题 | 类型 | 作用 |
|---|---|---|
| `/odom` | `nav_msgs/msg/Odometry` | 编码器积分得到的底盘里程计 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 左右轮角度和角速度 |
| `/tf` | `tf2_msgs/msg/TFMessage` | `odom → base_link` 变换 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 状态、故障、电压、心跳和通信信息 |
| `/motor/debug` | `std_msgs/msg/Float32MultiArray` | 电机闭环调试数据 |

列出当前接口：

```bash
ros2 node info /chassis_can_node
ros2 topic list -t
ros2 service list -t | grep chassis
```

### 10.3 `/motor/debug` 数组含义

```bash
ros2 topic echo /motor/debug
```

数组顺序固定为：

| 下标 | 含义 | 单位 |
|---:|---|---|
| 0 | 左轮目标速度 | m/s |
| 1 | 左轮实测速度 | m/s |
| 2 | 右轮目标速度 | m/s |
| 3 | 右轮实测速度 | m/s |
| 4 | 左电机 PWM 输出 | STM32 PWM 计数 |
| 5 | 右电机 PWM 输出 | STM32 PWM 计数 |
| 6 | 左轮速度误差 | m/s |
| 7 | 右轮速度误差 | m/s |

查看频率：

```bash
ros2 topic hz /motor/debug
ros2 topic hz /odom
```

### 10.4 服务列表

| 服务 | 类型 | 作用 |
|---|---|---|
| `/chassis/enable` | `std_srvs/srv/SetBool` | 软件使能或失能 |
| `/chassis/emergency_stop` | `std_srvs/srv/SetBool` | 触发急停 |
| `/chassis/clear_faults` | `std_srvs/srv/Trigger` | 清除锁存故障 |
| `/chassis/push_parameters` | `std_srvs/srv/Trigger` | 把节点参数下发到 STM32 |
| `/chassis/save_parameters` | `std_srvs/srv/Trigger` | 要求 STM32 把参数保存到 Flash |
| `/chassis/reset_odometry` | `std_srvs/srv/Trigger` | 清零 STM32 与 RDK 里程计基准 |

## 11. 查看里程计和 TF

查看里程计：

```bash
ros2 topic echo /odom
```

查看左右轮关节：

```bash
ros2 topic echo /joint_states
```

查看 `odom` 到 `base_link`：

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

在底盘失能且静止时复位里程计：

```bash
ros2 service call /chassis/reset_odometry std_srvs/srv/Trigger "{}"
```

如果底盘处于 `RUNNING`，节点会拒绝复位。

## 12. 参数查看、修改、下发和保存

### 12.1 参数文件

源码参数文件：

```text
~/chassis_project/ros2_ws/src/chassis_can_control/config/chassis_serial.yaml
```

节点启动时通过 ROS 安装目录加载参数。因此修改源码 YAML 后必须重新编译，才能更新 `install` 中的副本。

查看当前节点参数：

```bash
ros2 param list /chassis_can_node
ros2 param dump /chassis_can_node
```

查看单个参数：

```bash
ros2 param get /chassis_can_node controller.left_kp
```

### 12.2 主要参数分类

- `controller.left_kp/right_kp`：比例增益；
- `controller.left_ki/right_ki`：积分增益；
- `controller.left_kff/right_kff`：速度前馈；
- `controller.*dead*`：左右电机正反转死区；
- `controller.filter_alpha`：速度滤波系数；
- `controller.straight_sync_*`：直行同步修正；
- `geometry.*`：底盘类型、轮径、轮距、轴距、编码器计数和舵机几何；
- `limits.*`：最大速度和加减速度；
- `safety.*`：命令超时、欠压和堵转保护。

不要照搬其他小车的 PI、死区、轮径或编码器参数。

### 12.3 临时修改一个参数

先软件失能：

```bash
ros2 service call /chassis/enable std_srvs/srv/SetBool "{data: false}"
```

例如临时修改左轮比例增益：

```bash
ros2 param set /chassis_can_node controller.left_kp 3100.0
```

节点会检查取值范围，并把受支持的电机参数排队下发给 STM32。检查传输状态：

```bash
ros2 topic echo --once /diagnostics
```

必须满足：

```text
pending_parameters: 0
parameter_transfer_failed: False
```

运行时 `ros2 param set` 不会修改 YAML，节点重启后 RDK 参数会恢复为 YAML 中的值；也不会自动把 STM32 参数保存进 Flash。

### 12.4 将 YAML 中的全部控制参数下发给 STM32

保持失能，然后执行：

```bash
ros2 service call /chassis/push_parameters std_srvs/srv/Trigger "{}"
```

命令应提示已经排队约 30 个参数。等待数秒后检查 `/diagnostics`，必须确认：

```text
pending_parameters: 0
parameter_transfer_failed: False
```

如果传输失败，不要保存，先检查串口和日志。

### 12.5 保存到 STM32 Flash

只有参数已经架空验证正确、全部下发成功后才执行：

```bash
ros2 service call /chassis/save_parameters std_srvs/srv/Trigger "{}"
```

保存 Flash 不是日常启动步骤，不要频繁执行。

### 12.6 推荐的永久修改流程

1. 架空车轮并软件失能；
2. 备份原 YAML；
3. 小幅修改 `src/.../chassis_serial.yaml`；
4. 重新编译；
5. 重新启动节点；
6. 调用 `/chassis/push_parameters`；
7. 确认传输成功；
8. 架空低速验证；
9. 确认无误后调用 `/chassis/save_parameters`。

## 13. 修改代码或 YAML 后重新编译

普通开机启动不需要重新编译。只有修改了 Python、launch、YAML、`setup.py` 或 `package.xml` 后才需要执行：

```bash
cd ~/chassis_project/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select chassis_can_control
source install/setup.bash
```

看到 `Summary: 1 package finished` 才算成功。

编译前如果旧节点仍在运行，应先失能并停止旧节点：

```bash
bash ~/chassis_project/ros2_ws/src/chassis_can_control/scripts/stop_chassis_serial_safe.sh
```

编译完成后再执行安全启动。不要通过重新烧录 STM32 来解决 RDK Python 代码或 YAML 修改问题，这两者是不同程序。

## 14. 前台运行与后台运行

### 14.1 推荐的调试方式：前台运行

```bash
bash ~/chassis_project/ros2_ws/src/chassis_can_control/scripts/start_chassis_serial_safe.sh
```

优点是日志直接显示，按 `Ctrl+C` 可以正常退出。

### 14.2 临时后台运行

确认节点没有运行后：

```bash
source /opt/ros/humble/setup.bash
source ~/chassis_project/ros2_ws/install/setup.bash
nohup ros2 launch chassis_can_control chassis_serial.launch.py \
  > ~/chassis_project/chassis_serial.log 2>&1 </dev/null &
```

查看日志：

```bash
tail -f ~/chassis_project/chassis_serial.log
```

后台启动仍然默认软件失能。不要重复执行后台启动命令。

本手册不建议在尚未完成架空测试前配置开机自动使能。即使以后配置节点自启动，也不得配置电机自动使能。

## 15. 常见故障排查

### 15.1 `AMENT_TRACE_SETUP_FILES: unbound variable`

旧脚本如果在加载 ROS 前执行 `set -u`，会出现：

```text
/opt/ros/humble/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable
```

当前安全启动脚本已经修复。请直接使用：

```bash
bash ~/chassis_project/ros2_ws/src/chassis_can_control/scripts/start_chassis_serial_safe.sh
```

临时手工加载环境时可执行：

```bash
set +u
source /opt/ros/humble/setup.bash
source ~/chassis_project/ros2_ws/install/setup.bash
```

### 15.2 找不到 `chassis_can_control`

```text
Package 'chassis_can_control' not found
```

依次执行：

```bash
cd ~/chassis_project/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select chassis_can_control
source install/setup.bash
ros2 pkg prefix chassis_can_control
```

### 15.3 找不到 `0002` 串口

```bash
lsusb
ls -l /dev/serial/by-id/
```

检查 USB 线、底盘供电和接头。不要自动改用 `0001`，因为它是雷达。

### 15.4 串口权限不足

查看：

```bash
ls -l /dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00
groups
```

如果用户不在 `dialout` 组：

```bash
sudo usermod -aG dialout wheeltec
```

然后退出 SSH 并重新登录，组权限才会生效。

### 15.5 串口被占用或出现重复节点

检查：

```bash
pgrep -af 'chassis_serial.launch.py|chassis_can_node'
```

先安全停止：

```bash
bash ~/chassis_project/ros2_ws/src/chassis_can_control/scripts/stop_chassis_serial_safe.sh
```

再次检查没有旧节点后，只启动一次。不要直接反复执行启动脚本。

### 15.6 `/diagnostics` 没有数据

检查节点和话题：

```bash
ros2 node list
ros2 topic list
ros2 node info /chassis_can_node
```

如果节点不存在，检查启动终端或后台日志。如果节点存在但话题命令看不到它，确认两个终端都加载了同一个 ROS 2 Humble 环境。

### 15.7 `STM32 heartbeat missing`

依次检查：

1. STM32 底盘板是否供电；
2. `0002` 串口是否存在；
3. 是否错误启动了两个节点；
4. RDK 参数是否仍为 `115200`；
5. STM32 是否运行匹配的串口版固件；
6. 查看启动终端或 `chassis_serial.log`。

心跳恢复前不要使能电机。

### 15.8 服务显示使能成功，但电机不转

检查：

1. 是否正在以至少 5 Hz 持续发布 `/cmd_vel`；
2. `/cmd_vel` 是否在 `0.20 s` 内持续更新；
3. `/diagnostics` 是否为 `RUNNING` 或无故障；
4. 电池电压是否正常；
5. 硬件电机使能是否开启；
6. `/motor/debug` 的目标速度和 PWM 是否变化。

只调用 `/chassis/enable` 而不持续发送 `/cmd_vel`，电机不会转，这是正常的安全设计。

### 15.9 修改 YAML 后没有生效

因为节点读取的是 `install` 中的安装副本。修改源码 YAML 后必须：

```bash
cd ~/chassis_project/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select chassis_can_control
source install/setup.bash
```

然后停止旧节点并重新启动。

### 15.10 查看 ROS 日志

后台日志：

```bash
tail -100 ~/chassis_project/chassis_serial.log
```

ROS 2 日志目录：

```bash
ls -lt ~/.ros/log/ | head
```

编译日志：

```bash
ls -lt ~/chassis_project/ros2_ws/log/ | head
```

## 16. 每日操作速查

### 启动

```bash
ssh wheeltec@192.168.0.100
pgrep -af 'chassis_serial.launch.py|chassis_can_node'
ls -l /dev/serial/by-id/
cd ~/chassis_project/ros2_ws
bash src/chassis_can_control/scripts/start_chassis_serial_safe.sh
```

如果 `pgrep` 已经显示节点，则不要再次执行最后两行。

### 查看状态

```bash
source /opt/ros/humble/setup.bash
source ~/chassis_project/ros2_ws/install/setup.bash
ros2 topic echo --once /diagnostics
```

### 架空低速测试

```bash
ros2 service call /chassis/enable std_srvs/srv/SetBool "{data: true}"
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.05}, angular: {z: 0.0}}"
```

按 `Ctrl+C` 后：

```bash
ros2 service call /chassis/enable std_srvs/srv/SetBool "{data: false}"
```

### 停止节点

```bash
bash ~/chassis_project/ros2_ws/src/chassis_can_control/scripts/stop_chassis_serial_safe.sh
```

### 紧急停止

```bash
ros2 service call /chassis/emergency_stop std_srvs/srv/SetBool "{data: true}"
```

## 17. 当前已验证的实车状态

本文档生成前已在实际 RDK X5 上确认：

- 工程目录存在且 ROS 包可以运行；
- 底盘串口为 `...0002-if00`；
- 通信方式为 `serial`；
- STM32 状态为 `IDLE`；
- 故障为 `NONE`；
- 软件使能为关闭；
- ROS 与 STM32 心跳正常；
- `control_overruns` 为 `0`；
- 安全启动脚本已修复 ROS 环境变量报错；
- 蜂鸣器保持关闭。

首次实际运动仍必须由操作者在车轮架空、现场有人看护的条件下完成。

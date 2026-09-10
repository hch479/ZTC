# RDK X5、SocketCAN 与 ROS 2 使用说明

## 1. 硬件连接

RDK X5 开发板自身集成了 TCAN4550 CAN FD 控制器/收发器，板载接口同时兼容经典 CAN 和 CAN FD，不需要再串第二级收发器。当前 C30D 的 STM32F407 和 VP230 只支持经典 CAN，因此本项目必须把 RDK X5 配成经典 CAN 1 Mbit/s，不能开启 `fd on`。详细依据、烧录和恢复步骤见 `09_CANFD与烧录恢复.md`。

```text
RDK CAN_H  -------- C30D CAN_H
RDK CAN_L  -------- C30D CAN_L
RDK GND    -------- C30D GND
```

总线两端各保留一个 120 Ω 终端电阻。断电后测 CAN_H 与 CAN_L 之间总电阻，应接近 60 Ω。不要把 CAN_H/CAN_L 直接接到普通 UART 或裸 GPIO。RDK X5 与电机电源要共信号地，但电机大电流回路布线应避免经过上位机地线。

## 2. 启动 SocketCAN

C30D 当前配置为 1 Mbit/s：

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up
ip -details -statistics link show can0
```

工程也提供了等价脚本：

```bash
sudo bash ros2_ws/src/chassis_can_control/scripts/setup_can.sh can0 1000000
```

先只观察 STM32 心跳：

```bash
candump can0,700:7FF
```

如果无心跳，依次检查供电、共地、H/L 是否反接、终端电阻、两端位速率以及 `ip -statistics` 的 error-passive/bus-off 计数。

## 3. 构建 ROS 2 包

假设 RDK X5 已安装对应版本 ROS 2：

```bash
cd ~/chassis_project/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-select chassis_can_control
source install/setup.bash
```

启动：

```bash
ros2 launch chassis_can_control chassis_can.launch.py
```

若没有权限打开 CAN，可用 udev/capability 做永久配置，调试阶段也可使用有相应权限的终端。不要直接长期以 root 运行整套 ROS 图。

## 4. 第一次无电机动作检查

保持硬件使能关闭：

```bash
ros2 topic echo /diagnostics
ros2 topic echo /motor/debug
ros2 topic echo /joint_states
```

预期看到：

- heartbeat age 小于 2 s；
- state 为 IDLE；
- fault 中可能有 HARDWARE_DISABLED；
- 电池电压符合万用表读数；
- 手转车轮时 encoder/joint position 会变化。

## 5. 驱动轮离地试转

先发零速度并保持发布：

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

另开终端使能：

```bash
ros2 service call /chassis/enable std_srvs/srv/SetBool "{data: true}"
```

然后用很小速度：

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.08}, angular: {z: 0.0}}"
```

停止发布后，RDK 节点约 200 ms 内改发禁止运行的零速命令；即使节点整个崩溃，STM32 自身也会在约 200 ms 后进入 TIMEOUT。

## 6. 服务

```bash
# 软件使能/失能
ros2 service call /chassis/enable std_srvs/srv/SetBool "{data: false}"

# 触发急停（会在 STM32 锁存）
ros2 service call /chassis/emergency_stop std_srvs/srv/SetBool "{data: true}"

# 解除上位机急停请求后，清 STM32 锁存故障
ros2 service call /chassis/emergency_stop std_srvs/srv/SetBool "{data: false}"
ros2 service call /chassis/clear_faults std_srvs/srv/Trigger "{}"

# 将 YAML 中全部电机参数排队写到 STM32 RAM
ros2 service call /chassis/push_parameters std_srvs/srv/Trigger "{}"

# 确认参数正确后写入 STM32 Flash
ros2 service call /chassis/save_parameters std_srvs/srv/Trigger "{}"

# 车体停止时清两端里程计
ros2 service call /chassis/reset_odometry std_srvs/srv/Trigger "{}"
```

服务“发送成功”只表示报文进入 CAN socket。最终结果看 `0x186` 应答日志；status=0 才表示 STM32 接受。

## 7. 动态参数

例如只改左轮比例系数：

```bash
ros2 param set /chassis_can_node controller.left_kp 3500.0
```

节点会把修改排入队列，每 50 ms 发送一个，避免突发填满 CAN FIFO。STM32 在 RUNNING 状态会拒绝更改；先失能并等状态变为 IDLE。

配置文件 `config/chassis.yaml` 默认 `push_parameters_on_start: false`，这是为了防止尚未确认的 YAML 覆盖 STM32 中已经调好的参数。调试稳定后可改为 true。

## 8. ROS 接口定义

| 名称 | 类型 | 方向/说明 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | 输入线速度 x、角速度 z |
| `/odom` | `nav_msgs/Odometry` | 编码器里程计 |
| `/joint_states` | `sensor_msgs/JointState` | 左右轮角度/角速度 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 状态、故障、电压、心跳 |
| `/motor/debug` | `std_msgs/Float32MultiArray` | `[L目标,L实测,R目标,R实测,L_PWM,R_PWM,L误差,R误差]` |

如果机器人已有 EKF/IMU 节点，应根据系统 TF 设计决定是否把 `publish_tf` 设为 false，避免两个节点同时发布 `odom -> base_link`。

节点还会比较 STM32 心跳中的运行时间。如果运行时间回退，说明控制板大概率发生了复位，ROS 会重新建立编码器里程计基准，避免累计计数归零造成一次明显的 `/odom` 跳变。

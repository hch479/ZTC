# IMU 融合实现与实车操作手册

> 本文侧重烧录、部署和实车验收。若要逐函数理解改动内容，以及 ICM20948 从寄存器读取到 ROS 融合的完整过程，请配合阅读 `22_IMU改动清单与ICM20948数据读取使用详解.md`。

## 1. 本次完成了什么

当前串口版项目已经把 C30D 板载 IMU 接入完整上下位机链路：

```text
MPU6050 / ICM20948
        ↓ I2C 读取，100 Hz
STM32 静止零偏校准
        ↓ 0x187 / 0x188 / 0x189 逻辑帧
USB 串口
        ↓
RDK X5 ROS 2 节点
        ├─ 发布 /imu/data_raw
        └─ 编码器距离 + IMU 航向角速度融合 → /odom 和 odom→base_link TF
```

这里采用的是适合当前小车的轻量融合：

- 前进距离仍由左右编码器计算；
- 车体每周期的航向变化由“编码器转角”和“陀螺仪 z 轴角速度积分”加权得到；
- IMU 未校准、超时或断流时，自动退回纯编码器里程计；
- IMU 不直接修改 PWM，也不改变 STM32 电机闭环和急停逻辑。
- 启用融合时，ROS 服务会在 IMU 尚未校准完成时拒绝电机使能，防止校准过程中车辆移动。

这不是完整 EKF，也没有把加速度二次积分成位置。低成本 IMU 的微小零偏经过二次积分会快速变成很大的位置误差，因此当前做法更可靠，也更容易讲清楚和调试。

## 2. 是否需要购买额外设备

### 2.1 传感器本身通常不需要再买

C30D 原理和代码已经支持两种板载 IMU：

- 老版本板：MPU6050；
- 新版本板：ICM20948。

`systemInit()` 会读取传感器 `WHO_AM_I`，自动判断硬件版本，然后创建对应的 IMU 任务。所以上位机不需要预先知道你是哪一种芯片，下位机会在 IMU 状态帧中报告类型。

### 2.2 实际需要的现有设备

- C30D STM32F407VE 下位机板；
- RDK X5；
- 当前已经可用的 USB 串口连接；
- Windows 电脑、Keil MDK 5、STM32F4 Device Pack；
- ST-Link 和 SWD 接线，用于更新 STM32 固件；
- 稳固、水平、不会振动的桌面；
- 首次测试时用于架空驱动轮的支架。

### 2.3 建议但不是必须购买

- 卷尺或地面胶带：检查直行距离；
- 直角尺或地面 90° 标记：检查转向角；
- 能显示角度的手机工具：仅作为粗略对照；
- 如果以后做高精度定位，再考虑轮速更好的编码器、外置低噪声 IMU、磁力计或激光/视觉定位。当前阶段不必先买。

## 3. 改动的代码模块

### 3.1 STM32 IMU 任务

文件：

```text
keil_project/R550_C30D_SERIAL_MOTOR/BALANCE/imu_task.h
keil_project/R550_C30D_SERIAL_MOTOR/BALANCE/imu_task.c
```

主要功能：

1. IMU 任务保持 100 Hz 读取；
2. 开机先丢弃 50 个预热样本，约 0.5 秒；
3. 再平均 500 个静止样本，约 5 秒；
4. 计算三轴加速度和三轴陀螺仪零偏；
5. 让原厂驱动在后续读取中减去零偏；
6. 通过很短的 FreeRTOS 临界区发布一份完整快照，避免串口任务读到一半新、一半旧的数据。

为什么不沿用原来的校准流程：原程序依靠旧 `Balance_task` 增加 `SysVal.Time_count`，而当前电机工程为了避免重复读取编码器和重复写 PWM，没有创建这个旧任务。因此原校准流程不会正常结束。本次让 IMU 任务自己管理预热、平均和完成标志。

### 3.2 STM32 逻辑协议

文件：

```text
common/chassis_can_protocol.h
common/chassis_can_protocol.c
keil_project/R550_C30D_SERIAL_MOTOR/CHASSIS_SERIAL/inc/chassis_can_protocol.h
keil_project/R550_C30D_SERIAL_MOTOR/CHASSIS_SERIAL/src/chassis_can_protocol.c
```

新增三种逻辑帧：

| ID | 名称 | 内容 | 周期 |
|---|---|---|---|
| `0x187` | IMU acceleration | x、y、z 原始计数、序号、CRC | 20 ms |
| `0x188` | IMU gyroscope | x、y、z 原始计数、序号、CRC | 20 ms |
| `0x189` | IMU status | 传感器类型、校准状态、进度、序号、CRC | 1000 ms |

`0x187` 和 `0x188` 使用相同的样本序号。RDK 只有在两个序号相同时才组成一条 ROS IMU 消息，避免把不同时刻的加速度和角速度拼在一起。

### 3.3 STM32 串口遥测任务

文件：

```text
keil_project/R550_C30D_SERIAL_MOTOR/CHASSIS_SERIAL/src/chassis_tasks.c
```

新增行为：

- 校准期间只发送状态进度，不发送未经校正的向量；
- 校准完成后以 50 Hz 发送加速度和角速度；
- 发送失败只丢弃该次遥测，不阻塞 10 ms 电机控制任务。

### 3.4 RDK 协议解码

文件：

```text
ros2_ws/src/chassis_can_control/chassis_can_control/can_protocol.py
```

新增 `ImuVector`、`ImuStatus` 两类数据和对应解码函数。每帧先检查 ID、长度和 CRC，错误数据不会进入 ROS 发布和里程计融合。

### 3.5 ROS IMU 发布与融合

文件：

```text
ros2_ws/src/chassis_can_control/chassis_can_control/chassis_can_node.py
ros2_ws/src/chassis_can_control/chassis_can_control/wheel_odometry.py
```

ROS 节点完成以下处理：

1. 等待同序号的加速度和角速度；
2. 检查下位机是否报告校准完成；
3. 把 ADC 计数转换为 SI 单位；
4. 发布 `sensor_msgs/msg/Imu` 到 `/imu/data_raw`；
5. 保存最新 z 轴角速度；
6. 编码器数据到来时检查 IMU 是否新鲜；
7. IMU 可用时融合航向，不可用时使用原编码器公式；
8. 在 `/diagnostics` 中报告 IMU 类型、校准、数据年龄和融合状态。

## 4. 单位转换

当前两个传感器驱动都配置为：

- 加速度量程：±2 g，对应 `16384 LSB/g`；
- 陀螺仪量程：±500 °/s，对应 `65.5 LSB/(°/s)`。

RDK 使用：

```text
加速度 m/s² = 原始计数 ÷ 16384 × 9.80665
角速度 rad/s = 原始计数 ÷ 65.5 × π ÷ 180
```

ROS 的 `/imu/data_raw` 不包含绝对朝向，因此：

```text
orientation_covariance[0] = -1
```

这表示“本消息没有可用的 orientation”，不是程序错误。

## 5. 融合公式

编码器先计算：

```text
wheel_yaw_delta = (right_distance - left_distance) / wheel_track
```

陀螺仪计算：

```text
imu_yaw_delta = gyro_z_radps × dt
```

最终使用：

```text
yaw_delta = (1 - weight) × wheel_yaw_delta
          + weight × imu_yaw_delta
```

默认 `weight = 0.85`。它表示当前周期航向变化更信任陀螺仪，但并不代表位置的 85% 来自 IMU。前进距离仍然全部来自编码器。

## 6. 烧录和部署前的安全准备

1. 保存已经验证可恢复的原厂固件备份；
2. 小车轮子架空；
3. 电机硬件使能保持关闭；
4. 蜂鸣器保持当前禁用状态；
5. 首次上电后约 6 秒内不要移动、触碰或晃动底盘；
6. 未完成 z 轴正负验证前，不要高速运行。

## 7. Windows 上编译 STM32

在 PowerShell 中执行：

```powershell
cd C:\Users\User\Documents\ChatGPT\ZTC\keil_project\R550_C30D_SERIAL_MOTOR
.\BUILD_WITH_KEIL.bat
```

本次修改已经在本机用 ARM Compiler 5.06 update 5 完整构建通过：

```text
0 Error(s), 0 Warning(s)
```

生成文件：

```text
OBJ_SERIAL\C30D_SERIAL_MOTOR.hex
```

## 8. 使用 ST-Link 更新 STM32

推荐在 Keil 图形界面下载：

1. 打开 `USER/WHEELTEC.uvprojx`；
2. Target 选择 `C30D_SERIAL_MOTOR`；
3. Debug 选择 `ST-Link Debugger`；
4. Flash Algorithm 选择 `STM32F4xx 512KB Flash`；
5. 选择 `Erase Sectors`，不要选择整片擦除；
6. 勾选 Program、Verify、Reset and Run；
7. 点击 Download；
8. 确认 Program 和 Verify 成功；
9. 重新上电，把车放稳并静止至少 6 秒。

如果需要命令行下载，可在确认 ST-Link 接线和目标正确后运行：

```powershell
cd C:\Users\User\Documents\ChatGPT\ZTC\keil_project\R550_C30D_SERIAL_MOTOR
.\FLASH_WITH_KEIL.bat
```

## 9. 把 RDK 源码更新到板上

在 Windows PowerShell 中，把 `<RDK_IP>` 替换成 RDK 当前 IP：

```powershell
scp -r "C:\Users\User\Documents\ChatGPT\ZTC\ros2_ws\src\chassis_can_control" wheeltec@<RDK_IP>:/home/wheeltec/chassis_project/chassis_can_control_upload
```

登录 RDK 后执行：

```bash
mkdir -p ~/chassis_project/ros2_ws/src/chassis_can_control
cp -a ~/chassis_project/chassis_can_control_upload/. \
  ~/chassis_project/ros2_ws/src/chassis_can_control/

bash ~/chassis_project/ros2_ws/src/chassis_can_control/scripts/stop_chassis_serial_safe.sh

cd ~/chassis_project/ros2_ws
set +u
source /opt/ros/humble/setup.bash
colcon build --packages-select chassis_can_control
source install/setup.bash
```

成功标志应包含：

```text
Summary: 1 package finished
```

## 10. 安全启动

确认小车静止，然后在 RDK 执行：

```bash
cd ~/chassis_project/ros2_ws
bash src/chassis_can_control/scripts/start_chassis_serial_safe.sh
```

打开另一个终端：

```bash
set +u
source /opt/ros/humble/setup.bash
source ~/chassis_project/ros2_ws/install/setup.bash
```

不要使用 `set -u` 后直接 source ROS 环境。你以前遇到的 `AMENT_TRACE_SETUP_FILES: unbound variable` 就是这个原因，`set +u` 会先关闭未定义变量即报错的模式。

## 11. 验证 IMU 是否工作

### 11.1 检查话题

```bash
ros2 topic list | grep -E '^/imu/data_raw$|^/odom$|^/diagnostics$'
```

应该看到：

```text
/imu/data_raw
/odom
/diagnostics
```

### 11.2 查看诊断

```bash
ros2 topic echo --once /diagnostics
```

重点字段：

```text
imu_sensor: MPU6050 或 ICM20948
imu_calibrated: True
imu_calibration: 500/500
imu_data_age: 通常小于 0.20 s
imu_fusion_active: True
```

刚启动时短暂出现：

```text
IMU calibrating; keep chassis still
```

属于正常现象。约 6 秒后必须变成校准完成。

### 11.3 查看原始 IMU

```bash
ros2 topic echo /imu/data_raw
```

静止水平放置时，理想趋势是：

- `angular_velocity.x/y/z` 接近 0；
- 水平方向加速度接近 0；
- 竖直方向加速度的绝对值接近 `9.80665 m/s²`。

允许有噪声，不要求每个瞬时值完全相同。

查看频率：

```bash
timeout 8 ros2 topic hz /imu/data_raw
```

预期约 50 Hz。

## 12. 必做的坐标轴验证

ROS 约定：

- x：车头向前；
- y：车体向左；
- z：车体向上；
- 从上往下看，逆时针左转时 `angular_velocity.z` 应为正。

### 12.1 先禁止电机

```bash
ros2 service call /chassis/enable std_srvs/srv/SetBool "{data: false}"
```

### 12.2 手动转动车身

保持节点运行，手动把整车缓慢向左转，同时观察：

```bash
ros2 topic echo /imu/data_raw --field angular_velocity.z
```

判断：

- 左转为正：默认 z 轴符号正确；
- 左转为负：编辑 `config/chassis_serial.yaml`，把 `imu.z_sign` 从 `1.0` 改为 `-1.0`。

改 YAML 后重新编译并重启节点：

```bash
cd ~/chassis_project/ros2_ws
set +u
source /opt/ros/humble/setup.bash
colcon build --packages-select chassis_can_control
bash src/chassis_can_control/scripts/stop_chassis_serial_safe.sh
bash src/chassis_can_control/scripts/start_chassis_serial_safe.sh
```

如果发现芯片轴并非车体轴，可用：

```yaml
imu.x_source: 0
imu.y_source: 1
imu.z_source: 2
```

其中 `0/1/2` 分别表示芯片原始 x/y/z。三个 source 应分别使用 0、1、2，不应重复。每个轴再用 `x_sign/y_sign/z_sign` 选择正负。

## 13. 融合效果验证

### 13.1 先只验证手动旋转

电机保持禁止，执行：

```bash
ros2 topic echo /odom --field pose.pose.orientation
```

手动缓慢向左转，orientation 应连续变化。若方向相反，先修正 `imu.z_sign`，不要用其他参数掩盖方向错误。

### 13.2 低速实车验证

1. 轮子架空验证无异常输出；
2. 放回地面；
3. 设置很低的速度；
4. 做直行和约 90° 转向；
5. 对比真实方向与 `/odom`；
6. 确认 `/diagnostics` 中 `imu_fusion_active=True`。

建议录包：

```bash
ros2 bag record /imu/data_raw /odom /joint_states /motor/debug /diagnostics /cmd_vel
```

## 14. 参数怎么调

配置文件：

```text
~/chassis_project/ros2_ws/src/chassis_can_control/config/chassis_serial.yaml
```

最重要参数：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `imu.fusion_enabled` | `true` | 是否允许 IMU 参与里程计航向 |
| `imu.fusion_weight` | `0.85` | IMU 航向增量的权重 |
| `imu.timeout_s` | `0.20` | 超过该时间未收到有效 IMU 就降级 |
| `imu.z_sign` | `1.0` | 调整左转时 z 角速度的正负 |
| `imu.gyro_lsb_per_dps` | `65.5` | ±500 dps 量程换算值 |
| `imu.accel_lsb_per_g` | `16384.0` | ±2 g 量程换算值 |

调权重建议：

- `0.0`：关闭融合效果，等同纯编码器航向；
- `0.5`：两者各占一半；
- `0.85`：当前推荐起点；
- 太靠近 `1.0`：短时间转向灵敏，但长期陀螺仪漂移更明显；
- 太靠近 `0.0`：仍容易受轮胎打滑和左右轮误差影响。

不要随意改两个 LSB 换算参数，除非你同时修改了 STM32 传感器量程寄存器。

## 15. 自动降级行为

以下任意条件成立时，本周期不使用 IMU：

- 未收到 IMU 状态帧；
- 下位机报告未校准；
- 没有完整的加速度/角速度同序号数据；
- CRC 错误；
- IMU 数据年龄超过 `imu.timeout_s`；
- YAML 中 `imu.fusion_enabled=false`。

自动降级只影响 `/odom` 航向来源，不会停止电机。诊断会显示：

```text
IMU data stale; wheel odometry fallback active
imu_fusion_active: False
```

## 16. 常见问题

### 16.1 一直显示 IMU calibrating

检查：

1. STM32 是否烧录了本次新 HEX；
2. 车体开机后是否保持静止；
3. 是否能持续收到心跳；
4. `imu_calibration` 是否从小数值增加到 `500/500`。

### 16.2 完全没有 IMU status

如果电机心跳正常但没有 `0x189`，最常见原因是 STM32 仍在运行旧固件。重新核对 Keil Build、Download、Verify 和 Reset and Run。

### 16.3 `/imu/data_raw` 存在但没有数据

话题由节点创建，所以 `topic list` 中存在不等于已经收到数据。检查：

```bash
ros2 topic echo --once /diagnostics
timeout 8 ros2 topic hz /imu/data_raw
```

### 16.4 静止时 z 角速度不为零

少量噪声正常。持续明显偏置通常由以下原因造成：

- 开机校准期间移动了车；
- 桌面振动；
- 电机在校准期间运行；
- 传感器温度变化较大。

重新给下位机上电并静止 6 秒再测。

### 16.5 直行时里程计仍慢慢偏航

可能原因不只 IMU：

- z 轴符号或轴映射错误；
- 左右轮直径不同；
- 编码器计数比例不准；
- 舵机中位不准；
- 地面打滑；
- 陀螺仪温漂。

先验证坐标轴，再分别标定机械和融合权重。

## 17. 本次软件验证结果

已经执行：

- Python 协议、CRC、IMU 解码、编码器里程计和融合数学测试；
- Python 主节点与 launch 文件语法编译检查；
- Windows GCC 的 C 协议与控制核心测试；
- Keil ARM Compiler 5.06 完整工程真实构建。

结果：

```text
All Python protocol tests passed.
All chassis core tests passed.
Keil: 0 Error(s), 0 Warning(s).
```

尚需在你的实物上完成的项目：

- 烧录新 STM32 HEX；
- 更新 RDK ROS 包；
- 开机静止校准；
- 确认 IMU 型号；
- 确认 z 轴正负和必要的轴映射；
- 低速验证融合效果。

只有实车完成这些步骤后，才能把“融合实际有效”作为最终验收结论。

# C30D 五状态 EKF：实现、使用与验证结果

日期：2026-09-10。开发分支：`codex/planar-ekf`。

## 1. 本次完成了什么

**已经基于现有项目实现平面扩展卡尔曼滤波，并接入原 ROS 节点。串口 YAML 默认使用 EKF。**

新增算法状态为：

```text
X = [x, y, yaw, v, omega]
     位置   朝向  前向速度 角速度
单位：[m, m, rad, m/s, rad/s]
```

编码器提供速度观测，IMU 提供角速度观测。滤波器包含非线性运动预测、解析 Jacobian、协方差传播、观测残差、Kalman 增益和 Joseph 协方差更新。这次不再是把两个航向增量按 0.85 相加。

本次范围：

- 新增不依赖 ROS/NumPy 的 Python EKF 核心。
- 接入现有串口 ROS 节点，每条有效 IMU 数据对只更新一次。
- `/odom` 输出 EKF 位姿、速度及协方差。
- `/wheel/odom_raw` 输出不含 IMU 的纯编码器对照数据。
- 保留 `weighted` 模式，便于回到原固定权重方法比较。
- 增加异常观测、断流、重置与诊断处理。
- 完成离线数学测试、协议到节点发布的替身测试及四组对照仿真。

**验证边界：尚未在真实 ROS 2 Humble 环境和实车上运行。模拟测试通过不代表实车精度已经提高。** 当前五状态没有 gyro bias、实际舵角或绝对位置观测，仍存在相应误差来源。

本次没有改动 STM32 源码、协议 ID 或电机控制逻辑，没有执行烧录。软件电机使能默认关闭的行为保留。本次改动已写入本地开发分支的工作目录，尚未提交或推送新的 Git 版本。

## 2. 主要文件

以下路径相对项目根目录：

| 文件 | 用途 |
|---|---|
| `ros2_ws/src/chassis_can_control/chassis_can_control/planar_ekf.py` | EKF 数学核心、编码器观测适配、协方差映射 |
| `ros2_ws/src/chassis_can_control/chassis_can_control/chassis_can_node.py` | 在现有节点中接收观测、发布结果、诊断与重置 |
| `ros2_ws/src/chassis_can_control/chassis_can_control/wheel_odometry.py` | 保留固定权重算法，新增只重建计数基准的方法 |
| `ros2_ws/src/chassis_can_control/config/chassis_serial.yaml` | 串口主线默认模式及 EKF 参数 |
| `ros2_ws/src/chassis_can_control/config/chassis.yaml` | CAN 保持 `weighted` 模式 |
| `ros2_ws/src/chassis_can_control/test/test_planar_ekf.py` | 数学、异常输入及异步观测测试 |
| `ros2_ws/src/chassis_can_control/test/test_node_ekf.py` | 用消息和传输替身执行真实 ROS 节点逻辑 |
| `tools/simulate_ekf.py` | 三种算法在同一合成数据上的对照实验 |

建议按 `PlanarEkf.motion()` → `_predict_step()` → `_correct()` → `EkfOdometry` → ROS 节点的顺序阅读。

## 3. 数据怎样流动

```text
左右编码器累计计数
  → 左右帧序号配对
  → 累计计数差 / 接收时间差
  → 左右轮速
  → v_wheel、omega_wheel
  ├→ 纯编码器积分 → /wheel/odom_raw（无 TF）
  └→ EKF 速度观测更新

IMU 加速度帧 + 角速度帧
  → CRC 检查、同序号配对、校准状态检查
  → 轴映射与单位换算
  ├→ /imu/data_raw
  └→ gyro_z → EKF 角速度观测更新（每组只用一次）

EKF
  → /odom：x、y、yaw、v、omega、协方差
  → odom → base_link TF（publish_tf=true 时）
```

纯编码器话题在 EKF 模式下发布；`weighted` 模式保持原路径，不额外发布这个对照话题。这里的“raw”指未融合 IMU，仍会处理帧配对、计数环绕、长间隔和不合理的计数跳变。

当前 EKF 不订阅已经融合后的 `/odom`，也不把原固定权重结果当成观测。轮速遥测 `_latest_speed` 继续用于电机调试和关节速度，不重复输入 EKF；EKF 的轮速输入来自累计计数差。

## 4. 编码器提供什么观测

设左右累计计数增量为 `Δcount_left` 和 `Δcount_right`，时间间隔为 `dt`：

```text
meters_per_count = π × wheel_diameter / counts_per_rev
v_left  = Δcount_left  × meters_per_count / dt
v_right = Δcount_right × meters_per_count / dt

v_wheel     = (v_left + v_right) / 2
omega_wheel = (v_right - v_left) / wheel_track
```

例如沿用本项目默认几何参数：

```text
轮径 0.065 m，每圈 60000 count，轮距 0.162 m
左增量 6000，右增量 7200，dt=0.20 s

v_left      ≈ 0.102102 m/s
v_right     ≈ 0.122522 m/s
v_wheel     ≈ 0.112312 m/s
omega_wheel ≈ 0.126052 rad/s
```

如果同一阶段 gyro_z 为 `0.15 rad/s`，滤波器收到的是两个关于角速度的观测。具体怎样修正，要根据当时的状态协方差和观测噪声计算，不再直接指定 0.85。

编码器速度是一个时间区间的平均速度；本版把它近似作为接收时刻的速度观测。快速加减速、转向及通信延迟会影响这个近似，需要以后用采样时间戳与实车数据完善。

## 5. EKF 怎样预测

预测使用上一次估计的 `v` 和 `omega`，不是把同一轮速既作为独立预测输入又作为观测重复使用。

`PlanarEkf.motion()` 使用中点模型：

```text
middle_yaw = yaw + omega × dt / 2
x_next     = x + v × dt × cos(middle_yaw)
y_next     = y + v × dt × sin(middle_yaw)
yaw_next   = yaw + omega × dt
v_next     = v
omega_next = omega
```

`v_next=v` 和 `omega_next=omega` 是短时预测模型，不表示车辆必须匀速；变化通过过程噪声与后续观测更新反映。

因为位置公式有 sin/cos，运动模型是非线性的。程序使用对状态求导得到的 `F` 传播协方差：

```text
P_predicted = F × P × Fᵀ + Q
```

预测按不超过 20 ms 的小步推进。过程噪声采用局部线性化的连续白加速度模型：速度方差随 `dt` 增长，位置方差含 `dt³/3`，位置与速度交叉项含 `dt²/2`。这样改变消息频率时，噪声不会简单地按“收到多少条消息”机械累加。

## 6. EKF 怎样更新

轮速观测和 gyro 观测分别选择状态的对应分量：

```text
wheel observation: [v_wheel, omega_wheel]
H_wheel = [[0, 0, 0, 1, 0],
           [0, 0, 0, 0, 1]]

gyro observation: [gyro_z]
H_gyro = [[0, 0, 0, 0, 1]]
```

核心计算为：

```text
residual = z - H × X
S = H × P × Hᵀ + R
K = P × Hᵀ × inverse(S)
X_updated = X + K × residual

P_updated = (I-KH) × P × (I-KH)ᵀ + K × R × Kᵀ
```

最后一行是 Joseph 形式。还会对角度归一化，并消除浮点误差导致的协方差轻微不对称。

一个单变量算例：若角速度先验方差是 1，gyro 观测方差是 0.25，则 `K=1/(1+0.25)=0.8`；先验角速度为 0、观测为 1，更新后为 0.8。这只是某次特定条件下的增益，下一次会随 `P` 和 `R` 改变。

当前轮速观测的 `R` 使用对角阵，近似假设 `v_wheel` 与 `omega_wheel` 的观测误差独立。左右轮误差不对称时，这个假设未必成立；未来可由标定数据构造完整的 2×2 `R`。IMU 底层滑动平均也会让相邻样本相关，本版尚未建立有色噪声模型。

## 7. 为什么每条 IMU 只用一次

原固定权重模式在每次编码器到达时，使用最近一次 gyro 乘编码器周期。

EKF 模式在每组 IMU 消息配对完成时调用 `add_imu()`，预测到该接收时刻并更新角速度；编码器到达时只执行编码器更新。节点通过帧序号去重，核心也检查 IMU 时间，避免重复使用同一条观测而使协方差虚假缩小。

这表示使用每条有效接收到的约 50 Hz IMU 向量消息，不表示 MCU 内部约 100 Hz 样本都已传到上位机，也不是用真实 MCU 时间戳完成了精确积分。

## 8. 异常、断流和重置

| 情况 | 本版处理 |
|---|---|
| IMU 未校准 | 不进入 EKF；保留原使能服务校准保护 |
| `imu.fusion_enabled=false` | 不接收新的 IMU 更新，EKF 继续使用车轮观测 |
| IMU 断流 | 不重复使用旧 gyro，由模型预测和轮速观测继续估计 |
| NaN、Inf、超出硬阈值 | 拒绝观测 |
| 创新过大 | 根据 `residualᵀ S⁻¹ residual` 与门限平方比较，拒绝该观测 |
| 编码器 int32 环绕 | 使用现有环绕差值公式 |
| 大幅计数跳变 | 重建计数基准，不把跳变累计成一次巨大位移 |
| 编码器间隔超过 `ekf.max_gap_s` | 保留位姿、速度置零、增大不确定性，并重建基准 |
| MCU heartbeat 表明重启 | 清空滤波器、计数配对、相关 IMU 状态 |
| `/chassis/reset_odometry` 发送成功 | 同步重置本地原算法和 EKF，清除旧编码器配对 |

长时间中断期间的真实运动未知，本版选择不补算这段轨迹，因此可能丢失真实位移；协方差增加是保守的工程处理，不是对中断误差的精确统计建模。

IMU 断流后的“车轮更新”仍然是 EKF，不会瞬间变成原纯编码器积分结果；之前 IMU 对状态的影响会保留。`imu_fusion_active` 表示最近是否接受过有效 IMU 更新，不代表历史影响已被清零。

这些处理只影响状态估计，不改变电机控制保护，也不会因为里程计降级而自动启动或停止电机。

## 9. 参数怎样使用

串口参数文件：`ros2_ws/src/chassis_can_control/config/chassis_serial.yaml`。

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `odometry.mode` | `ekf` | `ekf` 或 `weighted` |
| `ekf.acceleration_noise` | 0.8 | 连续白线加速度噪声幅度，单位 m/s^(3/2) |
| `ekf.angular_acceleration_noise` | 2.0 | 连续白角加速度噪声幅度，单位 rad/s^(3/2) |
| `ekf.wheel_speed_stddev` | 0.04 | 轮速观测标准差，m/s |
| `ekf.wheel_yaw_rate_stddev` | 0.15 | 车轮角速度观测标准差，rad/s |
| `ekf.gyro_stddev` | 0.03 | EKF 使用的 gyro 观测标准差，rad/s |
| `ekf.innovation_gate` | 5.0 | 使用平方门限 25 检查 NIS，不承诺特定误报概率 |
| `ekf.max_gap_s` | 0.5 | 编码器长间隔重建基准阈值，s |
| `ekf.max_wheel_speed_mps` | 2.0 | 单轮观测合理性上限，m/s，不是电机速度限制 |
| `ekf.max_yaw_rate_radps` | 10.0 | 角速度观测合理性上限，rad/s |

这些数值是待标定的起点。标准差平方后进入 `R`；不要把标准差和方差混用。原 `imu.angular_velocity_stddev` 仍用于 `/imu/data_raw` 消息字段，内部 EKF 明确使用 `ekf.gyro_stddev`，二者不自动同步。

EKF 模式下，模式、EKF 参数、轮径/轮距/计数比例和 IMU 轴映射等改变要求编辑 YAML 并重启节点，避免新旧模型混在同一状态中。重启会重新从局部原点开始估计。

原 `imu.fusion_weight` 仅在 `weighted` 模式生效，EKF 不读取它作为 Kalman 增益。

## 10. 话题、协方差和 TF

`/odom` 的 pose 使用状态 x/y/yaw；twist 使用同一个 EKF 的 v/omega，不再一边发布 EKF 位姿、一边发布固定加权速度。

5×5 协方差中的位置、航向及交叉项映射到 ROS 的 6×6 pose covariance，速度及角速度交叉项映射到 twist covariance。未估计的 z/roll/pitch 等轴使用大方差，避免全零表示过度自信。

`/wheel/odom_raw` 的速度协方差来自编码器观测参数。其纯轮积分位姿没有单独传播协方差，因此 pose 使用大方差；它主要用于对照和检查原始速度观测，不能把其位姿当作高精度绝对位置。

当前节点只发布一份 `odom → base_link` TF。将来如果再接入其他定位节点，应明确 TF 所有权，不能同时发布同一条变换。

前向运动模型假设所选参考点没有侧向速度；阿克曼车应核对编码器车轴及 `base_link` 是否对应合适的车轴中点。程序没有收到实际舵角，也没有在线计算参考点偏移补偿。

## 11. 本次离线验证结果

执行环境：本机可用的 Python 3.14.0；不要求 ROS、pytest 或 NumPy。

```text
All 28 Python tests passed (protocol, odometry, EKF, node wiring).
Python 3.10 syntax compatible files: 17
```

验证包括：

- 解析 Jacobian 与有限差分导数比较。
- 单变量 Kalman 增益与协方差的可计算例子。
- 匀速圆弧预测与解析轨迹比较。
- 异步噪声观测下，协方差对称且能完成 Cholesky 正定分解。
- NaN/Inf、异常创新、时间倒序、计数跳变、环绕和长间隔。
- 无 IMU 时的车轮更新以及恢复。
- 真实节点方法中的协议配对、IMU 去重、纯编码器输出、协方差字段、TF 数量、校准保护、重置和模式检查。

节点测试使用最小 ROS 消息和传输替身，能覆盖 Python 数据流，但不能证明真实 DDS 通信、ROS 消息类型绑定及启动环境已正确。本机还没有完成真实 Humble/colcon 运行验收。

源码使用 Python 3.10 可解析语法的检查通过，但这不等于已经在 Python 3.10 解释器下完整运行测试。

### 同一合成数据上的算法对照

每个场景 20 秒、400 次编码器更新，固定随机种子，IMU 50 Hz、编码器 20 Hz。误差来自脚本人为设定的左右轮尺度误差、随机扰动和 gyro 噪声。

| 场景 | 固定权重位置 RMSE / m | EKF 位置 RMSE / m | 固定权重航向 RMSE / rad | EKF 航向 RMSE / rad |
|---|---:|---:|---:|---:|
| 转弯与噪声 | 0.07908 | 0.03583 | 0.03728 | 0.01474 |
| gyro 加 0.015 rad/s 零偏 | 0.17976 | 0.25272 | 0.11272 | 0.15155 |
| IMU 在第 8～12 秒断流 | 0.19397 | 0.15614 | 0.12274 | 0.10043 |
| 静止、模拟噪声及 gyro 零偏 | 0.00552 | 0.00544 | 0.16766 | 0.17319 |

**持续零偏场景下，EKF 比固定权重更差。** 本版没有在线 gyro bias 状态，给 gyro 较小的观测噪声会更信任它，也可能更信任它的偏置。这个结果完整保留，不能只挑有利场景宣传精度。

这些数值不代表你的实际传感器。仿真中的独立噪声与真实 IMU 滑动平均后的相关噪声也不同。默认参数没有按这些结果反复调到“保证获胜”。

## 12. 你现在怎样运行

在项目根目录运行离线测试：

```powershell
python -B ros2_ws/src/chassis_can_control/test/run_protocol_tests.py
```

如果本机 `python` 仍指向 WindowsApps 别名，可以在 PowerShell 中使用本次实际解释器：

```powershell
$pythonPath = 'C:\Users\xixun\AppData\Local\Autodesk\webdeploy\production\257040cabc1dffce734a8079453b19b0ffe2b735\Python\python.exe'
& $pythonPath -B ros2_ws/src/chassis_can_control/test/run_protocol_tests.py
& $pythonPath -B tools/simulate_ekf.py --output build/ekf_demo
```

对照结果生成在 `build/ekf_demo/`：四份 CSV 和 `summary.json`，属于可再生文件，不纳入 Git。

以后在已经准备好 ROS 2 Humble 的 RDK/Linux 环境中更新源码、构建并启动：

```bash
cd ~/chassis_project/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select chassis_can_control --symlink-install
source install/setup.bash
ros2 launch chassis_can_control chassis_serial.launch.py
```

上面的目录需与实际部署位置一致；这些命令本次没有在实物执行。需要包含已支持 IMU 逻辑帧的 STM32 固件，旧纯电机固件不会因为更新 Python 就自动开始发送 IMU。

首次观察 `/imu/data_raw`、`/wheel/odom_raw`、`/odom` 和 `/diagnostics`。新增诊断有 `odometry_mode`、`ekf_rejected_wheel`、`ekf_rejected_imu`、`ekf_gap_count`、`ekf_last_nis`、`ekf_wheel_accepted` 和 `ekf_wheel_age_s`。

切回原算法：把串口 YAML 中 `odometry.mode` 改为 `weighted`，按部署方式更新配置并重启节点。这样便于在相同录制数据上比较两种算法，而不是覆盖掉原实现。

## 13. 接下来最值得做什么

先读 `motion()` 和 `_correct()`，运行离线演示，观察状态与协方差；拿到实物后优先验证坐标轴、时间延迟和静止 gyro 偏置，再录制真实数据进行对照。

如果实测主要问题是零偏，再设计包含 gyro bias 的模型，并说明它如何从现有观测中被识别；如果主要是时间误差，先改进采样时间戳与时序。两轮共同打滑的平移误差仍可能需要外部位置观测，不能单靠更换滤波器名称解决。

参考：Joseph 更新和创新门控是常见滤波实现方式，也可对照阅读 [robot_localization 官方 EKF 源码](https://raw.githubusercontent.com/cra-ros-pkg/robot_localization/rolling-devel/src/ekf.cpp)。本次实现为项目内的独立五状态 Python 模块，并未安装或调用该 ROS 包。

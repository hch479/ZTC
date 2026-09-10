# 零基础逐段理解：这个项目的 EKF 融合代码究竟在做什么

日期：2026-09-10。对应当前本地 `codex/planar-ekf` 工作目录中的实现。

这份文档从你已经接触过的编码器和 IMU 出发，解释我新增和修改的代码。上一份说明直接出现了 Jacobian、协方差、Joseph 等术语，对第一次接触的人跨度太大。这里先解释这些东西为什么需要，再对应代码，不要求你提前学过卡尔曼滤波或矩阵。

**这次实现的功能是：根据左右轮编码器和 IMU 的 z 轴角速度，持续估计小车的二维位置、朝向、前进速度和转弯角速度。** 算法写在上位机 Python 程序里，接入原 ROS 节点；没有把 EKF 写进 STM32，也没有改电机 PID。

文中的真实代码片段来自当前工程；标为“教学例子”的数值用于帮助理解。第 12 节的完整时间线使用当前真实函数离线计算过，但输入仍是人为构造的数据，不是实车采集数据。

## 阅读路线

建议分三次读，不必一次记住所有公式。

1. **第一次：第 1～5 节。** 明白输入、输出、预测、修正，以及“不确定性”是什么。
2. **第二次：第 6～12 节。** 对着实际代码看一轮 EKF 怎样算出来；矩阵部分可以反复查。
3. **第三次：第 13～18 节。** 理解它怎样接进你的节点，以及怎样在没有实物时自己运行、观察。

文档比较长，可以用 VS Code 的 Markdown 预览和“大纲”按标题跳转。

## 1. 先明确：传感器告诉我们什么，程序想知道什么

### 1.1 编码器没有直接告诉我们“小车在房间里的位置”

编码器给程序的是左右轮累计计数，例如：

```text
上一次：左轮 100000，右轮 100000
这一次：左轮 103000，右轮 103420
间隔：0.05 秒
```

相减后，本次左轮增加 3000 个计数，右轮增加 3420 个计数。知道每圈计数和轮径，就能换算左右轮滚动了多远，再除以时间得到轮速。

但是“轮子滚动了多远”和“车身相对地面真的移动了多远”可能不同，例如打滑时。所以把编码器换算出的量称为**观测**：它提供关于真实运动的信息，但不等于绝对真值。

### 1.2 本次使用的 IMU 量是角速度，不是绝对角度

例如 gyro z 换算后为：

```text
0.174533 rad/s ≈ 10 °/s
```

表示小车当前绕 z 轴每秒大约转 10 度，不表示它现在朝向 10 度。

如果这一角速度持续 0.02 秒，这段时间增加的转角约为：

```text
角度增量 = 角速度 × 时间
         = 0.174533 × 0.02
         = 0.00349066 rad
         ≈ 0.2°
```

角速度像“转动的速度表”；要知道朝向变化，需要把它随时间积累。累积也会把误差积累进去。

本次 EKF **没有使用 IMU 加速度进行速度或位置积分**。加速度依然采集并发布，但没有进入这五个状态的观测更新。也没有磁力计、GPS 或外部定位提供绝对航向。

### 1.3 程序实际维护五个数

源代码：

```python
self.state = [0.0] * 5
```

之后这五个位置的意义永远固定：

| 索引 | 数学记号 | 代码常用名称 | 意义 | 单位 |
|---:|---|---|---|---|
| 0 | x | `x` | 局部坐标中的 x 位置 | m |
| 1 | y | `y` | 局部坐标中的 y 位置 | m |
| 2 | θ | `yaw` | 车头相对局部坐标 x 轴的朝向角 | rad |
| 3 | v | `speed` | 车体向前的有符号线速度，倒车为负 | m/s |
| 4 | ω | `rate` | 绕 z 轴的有符号角速度 | rad/s |

为避免把“整组状态”和“x 位置”混在一起，本文用大写 `X` 表示整组：

```text
X = [x, y, θ, v, ω]ᵀ
```

`ᵀ` 在这里表示竖着排列。代码里用普通 Python 列表存，不要求真的写成五行。

例如：

```python
self.state = [1.2, 0.3, 0.5, 0.2, 0.1]
```

解释为：估计位置是 `(1.2 m, 0.3 m)`，车头朝向约 28.65°，正在以 0.2 m/s 向前走，以 0.1 rad/s 转弯。

这里的 `(0, 0, 0)` 是启动或重置时定义的局部原点和朝向。它不知道你房间的东南西北。通常平面坐标约定 x 向前、y 向左、z 向上，从上往下看左转为正；实际轴映射和编码器符号需要与这个约定一致。

`v` 是沿车身前方的速度，并不是固定坐标系中的 x 方向速度。车头旋转后，前进会同时改变局部坐标的 x 和 y，这也是后面需要 sin/cos 的原因。

### 1.4 这五个数都是“估计”，不是五个直接读到的传感器数

输入是：

```text
编码器 → 观测 v、ω
陀螺仪 → 观测 ω
```

输出是：

```text
估计 x、y、θ、v、ω
```

位置和朝向主要靠运动模型与连续观测累积得到。没有绝对位置传感器，因此不能保证长期不漂移。

## 2. 原来的融合和现在的融合区别在哪里

### 2.1 原算法：预先决定一个固定比例

原来的 `WheelOdometry.update()` 在启用 IMU 时，大致做：

```python
imu_yaw_delta = imu_yaw_rate_radps * dt_s
yaw_delta = (
    (1.0 - safe_weight) * wheel_yaw_delta
    + safe_weight * imu_yaw_delta
)
```

默认权重 0.85，即本周期转角取 15% 编码器结果加 85% gyro 结果。

它简单直观，但没有另外维护“上次估计的不确定性有多大”。数据正常时和数据比较不确定时，也不会通过协方差自动重新算融合增益。

### 2.2 现在的算法：同时维护估计值与估计误差的模型

EKF 多维护一张叫 `P` 的表，记录五个估计量的不确定性及它们之间的误差关联。每收到一个新观测：

```text
① 根据上次估计和过去的时间，预测现在的运动状态。
② 计算这次预测的不确定性。
③ 比较传感器新观测与预测，得到差值。
④ 根据预测不确定性和观测噪声，计算本次修正量。
⑤ 更新状态，也更新不确定性。
⑥ 保存，等待下一次数据。
```

这就是“预测—修正”的循环。

“自动算增益”不表示程序自动知道哪个传感器是真的。你给它的运动模型和噪声参数仍然影响结果。如果错误地把一个有偏置的传感器设置得很可靠，它也会更相信这个错误读数。

### 2.3 本次算法放在哪里

```text
STM32：编码器、IMU 采集及已有处理，电机控制，发送遥测
                         ↓
上位机：现有串口/协议解析与数据配对
                         ↓
Python 的 EkfOdometry：把累计计数变成速度观测
                         ↓
Python 的 PlanarEkf：预测 + 观测修正
                         ↓
ROS 节点发布 /odom 和 TF
```

EKF 不负责向电机输出 PWM，不修改 PID 参数，也不会通过这几个估计函数自动使能电机。

## 3. 先不用矩阵：手算一次只有角速度的卡尔曼修正

这一节是教学例子，数值特意选得容易算，不使用项目默认参数。

### 3.1 预测与新观测发生分歧

假设：

```text
我预测角速度为 0.10 rad/s。
新传感器读数为 0.20 rad/s。
```

两个值不一致。更新可以写成：

```text
新估计 = 预测 + K × (观测 - 预测)
```

`K` 叫 Kalman gain，卡尔曼增益。

若 `K=0`，这次完全不根据观测修正；若 `K=1`，这次直接取观测；若 `K=0.8`：

```text
新估计 = 0.10 + 0.8 × (0.20 - 0.10)
       = 0.18 rad/s
```

现在真正要解释的是：`K` 怎样得到？

### 3.2 “不确定性”先用标准差理解

假设静止时连续测量某个量，结果有高有低。标准差 `σ` 是描述这种波动大小的一种数值。标准差大表示波动尺度大，标准差小表示波动尺度小；它不是每次误差的最大值。

滤波公式主要使用**方差**：

```text
方差 = 标准差的平方 = σ²
```

对于这次教学例子：

```text
预测的角速度标准差：0.20 rad/s → 预测方差 P=0.04
观测的角速度标准差：0.10 rad/s → 观测方差 R=0.01
```

`P` 描述当前估计误差，`R` 描述本次观测误差。都不是直接把传感器读数平方：角速度观测为 0.20，不代表 `R=0.20²`。

在单变量、直接观测且预测误差与观测误差独立的模型中：

```text
K = P / (P + R)
  = 0.04 / (0.04 + 0.01)
  = 0.8
```

观测更可靠，所以修正更接近它。反过来若预测方差为 0.01，观测方差为 0.04，增益变成 0.2，新估计为 0.12。

注意：**这段“0 到 1 的权重”直觉适用于这里的标量直接观测。完整矩阵 K 中的跨状态元素有单位，也可能为负，不能都解释为百分比。**

### 3.3 修正后，不确定性也更新

上述单变量情况下，可以简写为：

```text
P_new = (1 - K) × P
      = 0.2 × 0.04
      = 0.008
```

新标准差为 `sqrt(0.008)≈0.08944 rad/s`。融合后在这套独立误差模型下，比只看任意一个信息源更确定。

如果不经过新的运动预测，又来了一个**独立的新观测**，其 `R` 仍为 0.01，则下次增益为：

```text
K_next = 0.008 / (0.008 + 0.01) ≈ 0.44444
```

可见增益不是一直 0.8。若时间过去，运动模型又增加了不确定性，增益也会随之变化。

不能把缓存里的同一条观测反复当作独立新观测，否则程序会以为得到了更多证据，方差会被不合理地压小。这是后面“一条 IMU 只用一次”的原因。

### 3.4 为什么是这个 K：可以先跳过的小推导

设预测误差为 `e_p`，观测误差为 `e_z`。新估计的误差为：

```text
e_new = (1-K)e_p + K e_z
```

如果两种误差零均值且互不相关，新误差的方差为：

```text
P_new = (1-K)²P + K²R
```

寻找使这个二次式最小的 K，得到 `K=P/(P+R)`。这也说明卡尔曼增益来自一个误差模型，模型假设不符合实际时，效果就会受影响。

项目使用的 Joseph 更新，正是上面这个方差表达式的矩阵形式。

## 4. 从一个数变成五个数：P、F、Q、H、R、K 各是什么

### 4.1 P 是一张 5×5 表

行和列都按 `[x, y, θ, v, ω]` 排列：

```text
            x       y       θ       v       ω
      x   Pxx     Pxy     Pxθ     Pxv     Pxω
      y   Pyx     Pyy     Pyθ     Pyv     Pyω
      θ   Pθx     Pθy     Pθθ     Pθv     Pθω
      v   Pvx     Pvy     Pvθ     Pvv     Pvω
      ω   Pωx     Pωy     Pωθ     Pωv     Pωω
```

代码里：

```python
self.covariance[0][0]  # Pxx，x 估计误差的方差
self.covariance[4][4]  # Pωω，角速度估计误差的方差
self.covariance[2][4]  # Pθω，朝向误差与角速度误差的协方差
```

主对角线是每个状态自己的方差。非对角线叫协方差，表示两种估计误差是否倾向于一起偏大、一起偏小，或者反方向变化。

例如：如果上一段角速度估计偏大，累计出来的朝向通常也会偏大。于是 `θ` 与 `ω` 的误差有联系，`Pθω` 可以不为零。

**正是这种联系，让一次 gyro 角速度观测可以同时修正朝向估计。** 后面会把具体乘法算给你看。

这张表不是百分比表。`Pxx` 的单位是 m²，`Pωω` 是 `(rad/s)²`，`Pθω` 是 rad²/s。协方差也不等于归一化到 `[-1,1]` 的相关系数。

### 4.2 其他字母的用途

| 记号 | 代码名称 | 用一句话理解 |
|---|---|---|
| X | `self.state` | 当前估计的小车运动状态 |
| P | `self.covariance` | 这份估计的不确定性和误差关联 |
| f | `motion()` 中的状态公式 | 如果维持当前运动，过 dt 后会在哪里 |
| F | `jacobian` | 状态稍微估错一点，会怎样影响下一步估计 |
| Q | `noise` | 这一段预测中新增加的运动不确定性 |
| z | `values` | 本次传感器换算后的观测值 |
| H | 由 `indices` 隐式表示 | 传感器直接观察状态中的哪几个量 |
| R | 由 `variances` 表示对角线 | 观测误差的方差 |
| r | `residual` | 观测减预测，双方差多少 |
| S | `innovation` | 对“双方差多少”这件事，本来容许多大不确定性 |
| K | `gain` | 本次差值怎样分配为五个状态的修正量 |

容易看错的一点：当前代码中的 `innovation` 变量存的是 **S 矩阵**，差值本身存在 `residual`，不要把这两个变量当成同一个东西。

### 4.3 阅读代码够用的矩阵运算

Python 的矩阵就是列表里放列表：

```python
matrix = [[1, 2],
          [3, 4]]
matrix[1][0]  # 第二行第一列，值为 3；索引从 0 开始
```

转置把行和列交换：

```text
[[1, 2],      转置后   [[1, 3],
 [3, 4]]               [2, 4]]
```

矩阵乘法是“左边一行 × 右边一列，对应元素相乘再相加”，不是对应位置简单相乘。例如：

```text
[[1, 2],   × [[5], = [[1×5 + 2×6], = [[17],
 [3, 4]]      [6]]    [3×5 + 4×6]]    [39]]
```

左边列数必须等于右边行数。`5×5` 乘 `5×1`，结果是 `5×1`。

单位矩阵 I 的对角线为 1，其余为 0，相当于普通乘法中的 1：`I×X=X`。

后面使用 `S⁻¹`，意思是 S 的逆矩阵。单个数时就是倒数；矩阵通常不能把每个元素分别取倒数。当前只需要对 1×1 和 2×2 求逆，所以代码没有引入 NumPy。

## 5. 先认识文件和 Python 对象，避免把数学和语法同时看乱

### 5.1 主要文件位置

项目根目录：`C:/Users/xixun/Desktop/C30D_Development_Handoff_20260909`。

| 文件 | 这次的作用 |
|---|---|
| [planar_ekf.py](C:/Users/xixun/Desktop/C30D_Development_Handoff_20260909/ros2_ws/src/chassis_can_control/chassis_can_control/planar_ekf.py) | 新增，五状态 EKF 数学和编码器接入封装 |
| [chassis_can_node.py](C:/Users/xixun/Desktop/C30D_Development_Handoff_20260909/ros2_ws/src/chassis_can_control/chassis_can_control/chassis_can_node.py) | 修改，接收数据时调用 EKF，发布估计结果 |
| [wheel_odometry.py](C:/Users/xixun/Desktop/C30D_Development_Handoff_20260909/ros2_ws/src/chassis_can_control/chassis_can_control/wheel_odometry.py) | 保留原算法，新增重建累计计数基准的方法 |
| [chassis_serial.yaml](C:/Users/xixun/Desktop/C30D_Development_Handoff_20260909/ros2_ws/src/chassis_can_control/config/chassis_serial.yaml) | 串口主线选择 EKF 并给出参数 |
| [chassis.yaml](C:/Users/xixun/Desktop/C30D_Development_Handoff_20260909/ros2_ws/src/chassis_can_control/config/chassis.yaml) | CAN 配置仍选择原 `weighted` 模式 |
| [test_planar_ekf.py](C:/Users/xixun/Desktop/C30D_Development_Handoff_20260909/ros2_ws/src/chassis_can_control/test/test_planar_ekf.py) | 新增，独立算法测试 |
| [test_node_ekf.py](C:/Users/xixun/Desktop/C30D_Development_Handoff_20260909/ros2_ws/src/chassis_can_control/test/test_node_ekf.py) | 新增，节点接线与消息逻辑的离线测试 |
| [run_protocol_tests.py](C:/Users/xixun/Desktop/C30D_Development_Handoff_20260909/ros2_ws/src/chassis_can_control/test/run_protocol_tests.py) | 修改，把新增测试接入统一运行入口 |
| [simulate_ekf.py](C:/Users/xixun/Desktop/C30D_Development_Handoff_20260909/tools/simulate_ekf.py) | 新增，合成数据上的三种算法对照 |

README、AGENTS 和第 25、27 号文档也更新了现状说明，它们不参与滤波计算。本次讲解新增的是你正在看的第 28 号文档。

### 5.2 三个类分别负责什么

```text
EkfConfig：一组配置参数。
PlanarEkf：只负责运动状态、矩阵、预测与观测修正。
EkfOdometry：把累计计数变成 PlanarEkf 能用的速度观测，并处理时序和中断。
```

节点中有：

```python
self._ekf_odometry = EkfOdometry(EkfConfig(**ekf_values))
```

你可以把对象理解为“一份带数据的程序实体”：

```text
节点对象
 └─ _ekf_odometry 对象
     ├─ filter：PlanarEkf 对象
     │   ├─ state：五个状态
     │   ├─ covariance：5×5 协方差
     │   ├─ time_s：状态已经推进到哪个时间
     │   └─ config：参数
     ├─ raw：纯编码器 WheelOdometry 对象
     └─ previous：上一对累计计数和接收时间
```

所以：

```python
self._ekf_odometry.filter.state[4]
```

就是“节点保存的里程计对象里的滤波器对象，当前估计的角速度”。

`self` 表示当前对象自己。方法前的单个下划线，如 `_correct()`，主要表示内部使用的约定，不会赋予特殊数学含义。`@staticmethod` 表示函数不需要自动传入 `self`，例如 `motion(state, dt)` 只用给定参数计算，不直接改某个滤波器。

### 5.3 这几个 Python 写法怎么读

```python
x, y, yaw, speed, rate = state
```

把列表的五个元素分别取出命名。

```python
self.state[3:] = [0.0, 0.0]
```

只替换从索引 3 到末尾的两个数，即速度和角速度；前面的位姿不变。

```python
accepted = self.filter.observe_gyro(rate, timestamp)
```

调用方法，返回 `True` 表示接受了观测修正，`False` 表示没有接受。

`None` 表示“没有这个值/还没建立这个信息”，不同于数值 0。`time_s=None` 是还不知道时间起点；`time_s=0.0` 是一个已经确定的有效起点。

```python
residual = [value - self.state[i] for i, value in zip(indices, values)]
```

这是简写，等价于：

```python
residual = []
for i, value in zip(indices, values):
    residual.append(value - self.state[i])
```

`zip([3,4], [0.2,0.1])` 依次取出 `(3,0.2)`、`(4,0.1)`。`enumerate(...)` 会额外带上从 0 开始的编号。

```python
self.covariance = [[0.0] * 5 for _ in range(5)]
```

创建五个相互独立的行列表。这里不能随手改成 `[[0.0]*5]*5`，后者会重复引用同一个行对象，修改某行元素可能导致五行一起改变。

## 6. 初始化：EkfConfig 与 reset()

### 6.1 EkfConfig 保存“我们对系统的假设”

部分真实代码：

```python
@dataclass(frozen=True)
class EkfConfig:
    acceleration_noise: float = 0.8
    angular_acceleration_noise: float = 2.0
    wheel_speed_stddev: float = 0.04
    wheel_yaw_rate_stddev: float = 0.15
    gyro_stddev: float = 0.03
    innovation_gate: float = 5.0
    max_gap_s: float = 0.5
    max_wheel_speed_mps: float = 2.0
    max_yaw_rate_radps: float = 10.0
```

`dataclass` 帮我们自动生成接收这些参数的初始化方法；`frozen=True` 限制建立后直接修改字段，避免运行中随手改动参数而没有同步处理滤波状态。

`__post_init__()` 在配置构造后检查每个值是否有限且在 `[1e-6,100]` 范围内。它是基础输入检查，不表示这个范围里的每个值都适合你的车。

`vars(self).items()` 就是依次取出“字段名、字段值”；节点中的 `**ekf_values` 则把字典展开为命名参数，例如 `EkfConfig(gyro_stddev=0.03, ...)`。

三个观测标准差会平方进入 R：

```text
轮速方差         = 0.04² = 0.0016
车轮角速度方差   = 0.15² = 0.0225
gyro 角速度方差  = 0.03² = 0.0009
```

在这套默认假设里，同一个时刻的 gyro 角速度观测比车轮角速度观测更可靠。但两者更新频率和更新前 P 也影响增益，不能据此说“EKF 永远给 gyro 一个固定百分比”。

### 6.2 reset() 为什么这样设置

```python
self.state = [0.0] * 5
self.covariance = [[0.0] * 5 for _ in range(5)]
for i, variance in enumerate([1e-6, 1e-6, 1e-6, 1.0, 1.0]):
    self.covariance[i][i] = variance
self.time_s = None
```

状态先为零。P 初始为：

```text
diag(0.000001, 0.000001, 0.000001, 1, 1)
```

前面三项小，是因为把当前位置和朝向定义为局部原点，并不是宣称已经测出了毫米级绝对位置。

后面速度方差设为 1，是表达“虽然初始化数值写成零，但我对启动时的真实速度不确定”。这样第一批有效速度观测能够明显影响估计。

此外清零：

```text
rejected_wheel：核心/适配层计入的编码器拒绝次数
rejected_imu：核心计入的 IMU 拒绝次数
gap_count：触发中断处理的次数
last_nis：最近一次计算出的创新统计量
```

这些是诊断数据，不属于五个状态。注意适配层提前拦下的某些 IMU（例如重复时间）不会进入核心，所以拒绝计数并不统计所有被丢弃的消息。

## 7. 第一步：motion() 怎样预测小车运动

### 7.1 先只看状态预测

真实代码：

```python
x, y, yaw, speed, rate = state
middle = yaw + 0.5 * rate * dt
cosine, sine = math.cos(middle), math.sin(middle)
predicted = [x + speed * dt * cosine,
             y + speed * dt * sine,
             wrap_angle(yaw + rate * dt),
             speed,
             rate]
```

如果短时间内速度和角速度暂时保持不变：

```text
这段时间走的距离 ≈ speed × dt
这段时间转的角度 ≈ rate × dt
```

距离沿车头方向展开到 x、y：

```text
x 增量 = 距离 × cos(运动方向)
y 增量 = 距离 × sin(运动方向)
```

这里采用时间段中间的朝向：

```text
middle = 起始朝向 + 总转角的一半
```

例如本段从 0° 转到 2°，用大约 1° 的方向计算平移，比整段都用 0° 更能反映转弯。它仍是短步近似，不是任意运动下的精确轨迹。

### 7.2 一个可以手算的运动例子

假设：

```text
当前 x=y=θ=0
v=0.2 m/s，ω=0.1 rad/s，dt=0.1 s
```

则：

```text
middle = 0 + 0.5×0.1×0.1 = 0.005 rad
距离 = 0.2×0.1 = 0.02 m
x_new = 0.02×cos(0.005) ≈ 0.01999975 m
y_new = 0.02×sin(0.005) ≈ 0.00010000 m
θ_new = 0.01 rad
v_new = 0.2 m/s
ω_new = 0.1 rad/s
```

这是单独调用 `motion(state,0.1)` 的结果。实际 `predict_to()` 会拆小步，所以末位可能不同。

`v_new=v`、`ω_new=ω` 不要求车真的只能匀速。它表示“还没看到新数据时，暂时延续上一次速度估计”；车辆的变化通过过程噪声和新观测体现。

### 7.3 wrap_angle() 是做什么的

```python
return math.atan2(math.sin(angle), math.cos(angle))
```

把角度变成大致 `[-π,π]` 范围内的等价方向。例如 181° 和 -179° 指向同一方向，代码用后者表示。

因此持续左转跨过 180° 时，输出 yaw 数值可能跳到接近 -180°；不表示小车突然反向转了 360°。画误差或计算角度差也要处理这个环绕。

### 7.4 为什么叫“扩展”卡尔曼滤波

普通线性模型可以写成 `X_next=A×X`；这里有 `v×cos(θ+ωdt/2)`、`v×sin(...)`，不能用一个固定矩阵对所有状态精确表达。

EKF 的处理是：

```text
状态：仍用原来的非线性运动公式计算。
不确定性：在当前状态附近，用导数得到一个局部线性近似来传播。
```

这个导数矩阵叫 Jacobian，雅可比矩阵，也就是 F。“扩展”指对非线性问题使用这种局部线性化，并不是因为用了更多传感器或更多状态。

**代码没有用 `F×X` 替代实际状态预测。** 状态用 `predicted`，F 用于更新 P。这个区别非常重要。

## 8. F 和 Q：预测时怎样计算不确定性

### 8.1 先把“导数”理解成小扰动的影响

假设当前速度多估了 `δv`，经过 dt，x 会额外偏多少？

由 `x_next=x+vdt cos(middle)` 得到：

```text
x 额外偏差 ≈ dt×cos(middle)×δv
```

所以 F 中“下一步 x 对当前 v 的敏感程度”为：

```python
jacobian[0][3] = dt * cosine
```

`[0][3]` 的行 0 表示下一步 x，列 3 表示当前 v。不是“第 0 个传感器对第 3 个传感器”。

同理，角速度多估 `δω`，下一步朝向会多估 `dt×δω`：

```python
jacobian[2][4] = dt
```

这条关系会让 θ 和 ω 的误差产生关联。

### 8.2 对照完整 F

记 `c=cos(middle)`，`s=sin(middle)`：

```text
F = [1  0  -v·dt·s    dt·c    -v·dt²·s/2]
    [0  1   v·dt·c    dt·s     v·dt²·c/2]
    [0  0      1       0          dt    ]
    [0  0      0       1           0    ]
    [0  0      0       0           1    ]
```

代码先 `identity(5)`，设置对角线为 1，再写七个非零交叉项。

| 代码 | 为什么有这一项 |
|---|---|
| `F[0][2] = -v*dt*s` | 朝向估计变化会改变 x 方向投影；cos 的导数是 -sin |
| `F[0][3] = dt*c` | 前进速度变化会改变 x 方向位移 |
| `F[0][4] = -0.5*v*dt*dt*s` | 角速度改变中间朝向，再改变 x 投影 |
| `F[1][2] = v*dt*c` | 朝向变化会改变 y 投影；sin 的导数是 cos |
| `F[1][3] = dt*s` | 前进速度变化会改变 y 位移 |
| `F[1][4] = 0.5*v*dt*dt*c` | 角速度改变中间朝向，再改变 y 投影 |
| `F[2][4] = dt` | 角速度误差随 dt 积累为朝向误差 |

例如第三列的位置项为什么没有额外 `dt/2`？因为 middle 对 yaw 的导数为 1；第五列有，是因为 middle 对 rate 的导数为 `dt/2`。这是复合函数求导的结果。

### 8.3 P_predicted = F P Fᵀ + Q 的含义

先看一维：若下一步误差约为原误差的两倍，方差会变成四倍，因为方差按误差平方计。矩阵中的 `F P Fᵀ` 是把这一规则扩展到多个相互关联的量。

但车不一定维持原速度，它可能加速、减速或改变转弯速度；这些没有直接写进匀速预测的变化也会增加不确定性。于是再加 Q：

```text
预测后的不确定性
 = 旧的不确定性经过运动关系传播
 + 这段时间新增的运动不确定性
```

真实代码：

```python
self.covariance = add(
    multiply(multiply(jacobian, self.covariance), transpose(jacobian)),
    noise)
self.state = predicted
```

先 `F×P`，再乘 `Fᵀ`，最后加 `noise`，它就是 Q。

### 8.4 Q 中为什么出现加速度，但又说没有用加速度计

```python
qv = self.config.acceleration_noise ** 2
qw = self.config.angular_acceleration_noise ** 2
```

这里的两个参数描述**模型里没有准确预测的速度变化**。它们是配置数值，不是 `message.linear_acceleration.x` 之类的 IMU 读数。

例如小车实际加速了，匀速预测会落后。设置适当的过程噪声，就是承认速度可能发生变化，让 P 增长，从而给新观测留下修正空间。

当前采用连续白加速度噪声的近似模型：

```text
速度新增方差：        qv × dt
位置与速度新增协方差：qv × dt² / 2
位置新增方差：        qv × dt³ / 3
```

角速度和朝向也使用对应形式。因为位置是速度随时间的积累，新增速度扰动会进一步影响位置，这就出现不同次方的 dt。

更具体一点，在连续白噪声假设中，这些项来自时间积分：

```text
∫₀ᵈᵗ q dτ          = q·dt
∫₀ᵈᵗ q(dt-τ) dτ    = q·dt²/2
∫₀ᵈᵗ q(dt-τ)² dτ   = q·dt³/3
```

不需要现在掌握随机过程，但要知道这不是随便选的三个经验系数，也不是“取一次加速度读数乘 dt”。

因此 `acceleration_noise` 的单位为 `m/s^(3/2)`，`angular_acceleration_noise` 为 `rad/s^(3/2)`；平方后是对应连续噪声谱密度。**不要直接把加速度计一次测量的标准差填进来，认为单位和含义相同。**

### 8.5 方向如何进入 Q

```python
direction = [math.cos(angle), math.sin(angle)]
```

假设车身前后方向的速度有不确定变化，需要投影到局部坐标 x、y。

```python
noise[i][j] = direction[i] * direction[j] * qv * dt ** 3 / 3.0
noise[i][3] = noise[3][i] = direction[i] * qv * dt ** 2 / 2.0
```

这段循环填入平移部分及它与 v 的关联。然后单独填入朝向和角速度部分：

```python
noise[3][3] = qv * dt
noise[2][2] = qw * dt ** 3 / 3.0
noise[2][4] = noise[4][2] = qw * dt ** 2 / 2.0
noise[4][4] = qw * dt
```

这是每个短步使用当前方向的近似，不是包含任意侧滑、俯仰和全部动力学的完整噪声模型。

### 8.6 一组默认参数下的 Q 数字

当 `dt=0.02 s`，中间朝向为 0：

```text
qv=0.8²=0.64；qw=2.0²=4

Qxx = 0.64×0.02³/3 ≈ 0.000001706667
Qxv = Qvx = 0.64×0.02²/2 = 0.000128
Qvv = 0.64×0.02 = 0.0128

Qθθ = 4×0.02³/3 ≈ 0.000010666667
Qθω = Qωθ = 4×0.02²/2 = 0.0008
Qωω = 4×0.02 = 0.08
```

这一方向下，Q 其余元素为零。但 P 并不因此全部为零：P 还包含上一时刻的不确定性及 `F P Fᵀ` 的传播。

这也解释了：即使 gyro 观测刚把角速度方差压得很小，经过 0.02 秒后，默认角加速度过程噪声又会给它增加 0.08，所以后续 gyro 增益仍可能很大。默认参数需要结合实车变化速度和噪声标定。

## 9. 第二步：_correct() 怎样把新观测变成修正

这是数学核心。理解这一节时，可以一直拿第 3 节的公式作对照：

```text
新估计 = 预测 + 增益 × 差值
```

### 9.1 谁调用 _correct()，三个参数是什么

编码器入口：

```python
accepted = self._correct(
    [3, 4],
    [speed, rate],
    [self.config.wheel_speed_stddev ** 2,
     self.config.wheel_yaw_rate_stddev ** 2])
```

对应：

```text
indices   = [3,4]         → 本次直接观察状态里的 v、ω
values    = [speed,rate]  → 本次编码器算出的 v、ω
variances = [0.0016,0.0225] → 它们的观测方差
```

gyro 入口：

```python
accepted = self._correct([4], [rate], [self.config.gyro_stddev ** 2])
```

对应只观察 `state[4]`，即 ω，观测方差为 0.0009。

两个传感器共用同一个 `_correct()`，没有分别维护互相独立的五状态滤波器。它们更新的是同一个 `self.state` 和 `self.covariance`。

### 9.2 H 为什么没有写成一个变量

数学上编码器观测矩阵是：

```text
H_wheel = [0 0 0 1 0]
          [0 0 0 0 1]
```

与五状态相乘，正好选出 `[v,ω]ᵀ`。gyro 则为：

```text
H_gyro = [0 0 0 0 1]
```

因为当前观测关系只是“从五个数里挑几个”，直接用 `indices=[3,4]` 或 `[4]` 选取即可，不必真的生成一堆 0 再相乘。

这是针对当前线性观测关系的实现简化。若以后观测是距离、方位或带偏置的组合，就不能未经修改继续只用挑索引的方法。

运动模型非线性，而观测模型线性，这样仍然是 EKF；不要求预测和观测两个部分都非线性。

### 9.3 计算差值 residual

```python
residual = [value - self.state[i]
            for i, value in zip(indices, values)]
```

假设预测 `v=0.20, ω=0.10`，编码器观测 `v=0.23, ω=0.14`：

```text
residual = [0.03, 0.04]
```

意思是：编码器认为你比预测前进得更快、转弯也更快。

对 gyro，只得到一个差值：

```text
residual = [gyro_z - predicted_omega]
```

此时 `self.state` 已经由 `predict_to(timestamp)` 推进到观测时刻，因此比较的是“本时刻预测”和“本时刻观测”，不是拿新读数直接减很久以前的状态。

### 9.4 计算 S：差多少才算大

```python
size = len(indices)
innovation = [[self.covariance[i][j] for j in indices]
              for i in indices]
for k in range(size):
    innovation[k][k] += variances[k]
```

这一段就是：

```text
S = H P Hᵀ + R
```

对 gyro：

```text
S = Pωω + Rgyro
```

对编码器：

```text
S = [Pvv + Rv     Pvω      ]
    [Pωv          Pωω + Rω]
```

预测本身有误差，传感器也有误差，二者相减的差值自然也有不确定性。在假设预测误差与当前观测误差互不相关时，它们的方差相加。

### 9.5 为什么要求逆

单变量增益是 `P/(P+R)`，也就是 `P×S⁻¹`。矩阵里没有普通意义上的“除以一张表”，所以使用逆矩阵。

代码对单个 gyro 很简单：

```python
inverse = [[1.0 / innovation[0][0]]]
```

对编码器的 2×2：

```python
a, b = innovation[0]
c, d = innovation[1]
determinant = a * d - b * c
inverse = [[d / determinant, -b / determinant],
           [-c / determinant, a / determinant]]
```

数学形式：

```text
[a b]⁻¹       1      [ d -b]
[c d]   = -------- × [-c  a]
           ad - bc
```

`determinant` 是行列式。当前代码在它非正或不是有限数时拒绝更新，避免使用失效的结果。正常有效协方差与正观测噪声下，S 应为正定。

### 9.6 NIS：先检查这次观测是否与当前估计差得过分

```python
nis = sum(residual[i] * inverse[i][j] * residual[j]
          for i in range(size) for j in range(size))
self.last_nis = nis
if not math.isfinite(nis) or nis > self.config.innovation_gate ** 2:
    return False
```

这就是：

```text
NIS = residualᵀ × S⁻¹ × residual
```

对一个 gyro 数：

```text
NIS = 差值² / S
```

例如同样差 0.2 rad/s：

```text
S=0.01 时，NIS=4
S=0.001 时，NIS=40
```

默认比较门限是 `5²=25`。前者可以继续修正，后者会被拒绝。理由是后者的预测与观测都声称自己很确定，却出现了相对特别大的分歧。

它并没有证明读数一定坏了。真实快速运动变化、时间错位、噪声参数过小，也会使有效数据被拒绝。只有参数合理、统计假设成立时，统计门控才有相应意义。

对于两个编码器观测，NIS 联合考虑两个差值及它们的关联。不要把默认 5 简单说成“两项各自都做严格 5σ 检查”；这里统一使用平方门限 25，没有针对不同观测维度给出固定误报概率保证。

### 9.7 计算 K：五个状态各自应该修正多少

```python
pht = [[row[i] for i in indices] for row in self.covariance]
gain = multiply(pht, inverse)
```

`pht` 表示 `P Hᵀ`。挑选 P 的对应列，就得到每个状态与被观测量的误差联系。

对 gyro，K 是 5×1：

```text
K = [Pxω / S]
    [Pyω / S]
    [Pθω / S]
    [Pvω / S]
    [Pωω / S]
```

最后一行是你熟悉的 `P/(P+R)`，前四行告诉程序怎样把角速度差值传递到其他状态。

例如 `Pθω>0` 且 gyro 观测比预测大，那么朝向也会被向大的方向修正。这不是把 gyro 伪装成了绝对角度传感器，而是利用运动积累形成的误差关联。

对编码器，K 是 5×2，每一行分别对应“轮速差值的影响”和“车轮角速度差值的影响”。

### 9.8 修正五个状态

```python
for i in range(5):
    self.state[i] += sum(gain[i][j] * residual[j]
                         for j in range(size))
self.state[2] = wrap_angle(self.state[2])
```

对编码器，第 i 个状态的修正为：

```text
X_i_new = X_i_predicted
          + K[i][0] × 轮速差值
          + K[i][1] × 车轮角速度差值
```

因此更新循环遍历所有五个状态。传感器只直接观察两个分量，不意味着其余三个分量永远不能通过关联得到修正。

如果某个交叉协方差确实为零，对应增益也可能为零，那次观测就不会通过这一条关联改变那个状态。

### 9.9 为什么还要更新 P

如果只改状态不改 P，下次计算增益时就不知道“刚刚已经得到一条新证据”。当前代码使用：

```text
A = I - K H
P_new = A P Aᵀ + K R Kᵀ
```

这就是 Joseph 形式。对应源代码的第一段：

```python
residual_matrix = identity(5)
for i in range(5):
    for column, state_index in enumerate(indices):
        residual_matrix[i][state_index] -= gain[i][column]
```

`residual_matrix` 实际保存 A，不是之前的 residual 向量。它构造单位矩阵后，在被 H 选中的列减去 K，等价于 `I-KH`。

下一段计算 `K R Kᵀ`：

```python
krkt = [[sum(gain[i][k] * variances[k] * gain[j][k]
             for k in range(size))
         for j in range(5)] for i in range(5)]
```

当前 R 是对角阵，所以只需要对各观测分量累加 `Kik×Rkk×Kjk`。

然后：

```python
self.covariance = add(
    multiply(multiply(residual_matrix, self.covariance),
             transpose(residual_matrix)),
    krkt)
```

你可以用第 3 节的数字检查一维情况：

```text
P_new = (1-0.8)²×0.04 + 0.8²×0.01
      = 0.0016 + 0.0064
      = 0.008
```

Joseph 形式更完整地保留了预测误差和观测误差两部分，在浮点计算中有利于保持协方差性质。它不是另一套融合传感器的方法，只是计算更新后 P 的一种形式。

最后对称化：

```python
value = 0.5 * (self.covariance[i][j] + self.covariance[j][i])
self.covariance[i][j] = self.covariance[j][i] = value
```

理论上 `Pij=Pji`；浮点舍入可能有很小偏差，就取二者平均。对称化不能修复错误的数学模型，也不能单独保证任意输入下都正定。

### 9.10 整段 _correct() 的阅读顺序

```text
选出被观测的状态
 → z - 预测 = residual
 → 预测观测方差 + 传感器方差 = S
 → 求 S 的逆
 → NIS 检查，太异常则返回 False
 → K = P Hᵀ S⁻¹
 → X += K residual
 → 更新 P
 → 角度归一化、P 对称化
 → 返回 True
```

源代码中角度归一化在更新 P 之前执行；上面把末尾的数值整理写在一起，帮助你记忆。

## 10. 时间怎样处理：predict_to() 和两个观测入口

### 10.1 observe_wheels() 与 observe_gyro() 的共同结构

把异常检查暂时省略后，两者都是：

```python
self.predict_to(timestamp)   # 先把状态推进到这个观测的时间
self._correct(...)          # 再用这条观测修正
```

因此滤波器是事件驱动的。IMU 到来可以更新，编码器到来也可以更新，不要求两种传感器每次同时到达。

### 10.2 predict_to() 中的 dt 是什么

```python
dt = timestamp - self.time_s
```

`time_s` 是滤波状态已经推进到的时间，不一定是上次编码器时间。

例如：

| 时间 | 事件 | EKF 本次预测需要推进 |
|---:|---|---:|
| 0.00 s | 第一对编码器建基准 | 只建立时间 |
| 0.02 s | IMU | 0.02 s |
| 0.04 s | IMU | 0.02 s |
| 0.05 s | 编码器 | 0.01 s |
| 0.06 s | IMU | 0.01 s |

但 0.05 s 时编码器换算速度使用的是从 0.00 到 0.05 的计数变化，所以**编码器求轮速的 dt=0.05 s，而 EKF 此次预测的 dt=0.01 s**。这是两种不同用途的时间差，不是算错。

### 10.3 为什么拆成最多 20 ms

```python
while dt > 1e-12:
    step = min(dt, 0.02)
    self._predict_step(step)
    dt -= step
```

比如要推进 0.05 秒，拆成 `0.02+0.02+0.01`，每小步都重新计算中间方向、F 和 Q。这样有助于减小转弯时的离散化误差。

它没有制造新传感器数据。拆成三步只是三次数学预测，不等于观测了三次，也不会调用三次 `_correct()`。

`1e-12` 是很小的数，用于避免浮点减法留下极小尾数而多做无意义循环。

### 10.4 首次、倒序与同一时刻

```text
time_s 为 None：先记住时间，不知道更早发生了什么，因此不向前补算。
timestamp 比 time_s 小：拒绝倒序，不把新状态倒着推进。
timestamp 等于 time_s：可以直接观测修正，预测时间为零。
```

两个不同传感器可以在同一时刻各提供一次有效观测；但不能因此重复使用同一个 gyro 样本。后一件事由配对序号和 `EkfOdometry.add_imu()` 的时间检查另外保证。

### 10.5 观测被拒绝后，状态一定完全没变吗

不一定，要看在哪一步被拒绝。

```text
数值非法或超过硬阈值 → 在预测前被拦下，不做这次预测和修正。
时间合法，先完成预测，随后 NIS 太大 → 保留已推进的预测，不接受观测修正。
```

所以 `accepted=False` 主要表示“没有接受这一条观测修正”，不能统一理解为“整个对象和调用前逐字节相同”。

## 11. EkfOdometry：累计计数怎样变成可用观测

### 11.1 为什么不让 PlanarEkf 直接读串口和累计计数

数学核心只接受已换算好的 `m/s`、`rad/s` 和秒。这样即使没有 ROS、串口、实车，也能构造数值验证公式。

累计计数、轮径、轮距、计数重置等工程问题放在 `EkfOdometry`；协议解析和 ROS 消息放在节点。你排查问题时可以沿这三层往上追。

### 11.2 reset() 的几项时间变量

| 变量 | 保存什么 |
|---|---|
| `previous` | 上一对用于计数差分基准的左计数、右计数、时间 |
| `last_wheel_time` | 最近接受的轮观测时间，或初始化/长间隔重新建基准的时间 |
| `last_gyro_time` | 最近一次通过适配层时间检查、尝试送入核心的 IMU 时间 |
| `last_accepted_gyro_time` | 最近真正接受 gyro 修正的时间 |
| `filter.time_s` | EKF 状态已经推进到的时间 |

这些变量有不同职责，不要全部改成一个 `last_time`。例如计数差分需要上一对计数的时间，IMU 新鲜度需要最近接受 IMU 的时间。

`raw_speed`、`raw_rate` 是纯编码器速度，不是滤波后的速度。滤波结果存在 `filter.state[3]`、`filter.state[4]`。

### 11.3 第一次累计计数只建基准

```python
if self.previous is None:
    self.previous = current
    self.last_wheel_time = timestamp
    self.raw.set_count_baseline(left, right)
    self.filter.predict_to(timestamp)
    return self.sample()
```

如果程序启动时累计计数已经是 100000，不能假设这是刚刚这一个周期走出来的 100000。必须记住它，等下一次相减。

所以第一次不能算出轮速观测，也不会把当前位置突然加上 100000 个计数对应的距离。

### 11.4 第二次起怎样换算

真实代码：

```python
scale = math.pi * diameter / counts_per_rev
left_speed = WheelOdometry.wrapped_count_delta(left, old_left) * scale / dt
right_speed = WheelOdometry.wrapped_count_delta(right, old_right) * scale / dt
speed = 0.5 * (left_speed + right_speed)
rate = (right_speed - left_speed) / track
```

按你的默认参数和前面的例子：

```text
diameter = 0.065 m
counts_per_rev = 60000
track = 0.162 m
dt = 0.05 s

scale = π×0.065/60000 ≈ 0.000003403392 m/count

左轮距离 = 3000×scale ≈ 0.01021018 m
右轮距离 = 3420×scale ≈ 0.01163960 m

左轮速度 ≈ 0.20420352 m/s
右轮速度 ≈ 0.23279202 m/s

speed = (0.20420352+0.23279202)/2
      ≈ 0.218497769 m/s

rate = (0.23279202-0.20420352)/0.162
     ≈ 0.176472180 rad/s
```

直觉上，左右轮一样快时角速度为零；右轮走得更快时车向左转，对应正角速度。

这个换算依赖滚动与几何假设。对实际阿克曼结构，还要确认左右编码器所在车轴、轮距，以及作为运动参考点的 `base_link` 是否合适。本版没有实际舵角观测，也没有参考点偏移补偿。

编码器换算的是上个区间的平均速度，本版近似把它作为当前接收时刻的速度观测。快速加减速时这个近似可能产生延迟或误差。

### 11.5 wrapped_count_delta() 为什么不用直接相减

下位机累计计数是 int32，达到最大正数后可能变成负数。假设：

```text
上一次： 2147483640
这一次：-2147483640
```

直接相减会得到一个巨大的负数，实际上跨过边界只增加了 16。

已有函数用：

```python
((current - previous + (1 << 31)) % (1 << 32)) - (1 << 31)
```

在 32 位环绕空间里计算差值。`1<<31` 是 `2³¹`，`%` 是取模。它适用于相邻采样的真实增量远小于半个计数范围的正常情况。

MCU 重新启动把计数归零不等于正常环绕，因此还需要心跳重启识别和不合理轮速检查。

### 11.6 为什么同时算一份 raw

```python
self.raw.update(left, right, diameter, counts_per_rev, track)
self.raw_speed, self.raw_rate = speed, rate
self.wheel_accepted = self.filter.observe_wheels(speed, rate, timestamp)
```

这里 `self.raw.update()` 没有传 gyro，默认 IMU 权重为 0，所以得到纯编码器里程计。

两条路径是：

```text
同一组累计计数
 ├─ 纯编码器积分 → raw，供对照发布
 └─ 换算 v、ω → EKF 观测
```

**raw 的 x/y/yaw 不再喂回当前 EKF。** 也没有把原来已经按 0.85 融合 IMU 的结果再次喂进来，避免把同一份信息当成多份独立观测。

如果观测仅因 EKF 的 NIS 门控被拒绝，raw 仍可能记录这次编码器运动。这正好能帮助比较“原始轮观测在说什么”和“滤波器接受了什么”。但巨大计数跳变会在 raw 积分之前被拦下。

### 11.7 add_imu() 为什么还有一层检查

进入核心之前，它检查：

```text
时间必须有限。
必须先有编码器基准。
不能早于最近车轮有效时间。
距离最近车轮有效时间不能超过 max_gap_s。
不能重复或倒序使用 IMU 时间。
```

通过后：

```python
self.last_gyro_time = timestamp
accepted = self.filter.observe_gyro(rate, timestamp)
if accepted:
    self.last_accepted_gyro_time = timestamp
return accepted
```

这样轮数据长时间丢失时，不会只靠 gyro 不断沿陈旧前进速度推算小车移动。

即使某次 gyro 因 NIS 被拒绝，`last_gyro_time` 也记住尝试过的时间，不把同一个时间的样本重新处理一遍。

### 11.8 中断与跳变为什么要 suspend()

设最后一次估计前进速度为 0.3 m/s，然后通信断了 10 秒。真实情况可能是继续走、停下、被人搬走。直接补算 3 米没有足够证据。

本版选择：

```python
self.state[3:] = [0.0, 0.0]
```

保留目前位姿，重建速度不确定性，清掉速度与其他状态的旧关联，给三个姿态位置方差各增加 1，再把时间设为新时间。

对于编码器长间隔，还会把当前累计计数作为新基准。因此**不会补回通信空白期间的位移**。这会损失未知段的真实运动，但避免把一段不可确认的间隔按旧速度无限外推。

速度置零是这套中断策略的计算选择，不是传感器已经证明车停住；方差加 1 也是工程上的保守处理，不是精确推导出的中断误差。

新增的 `WheelOdometry.set_count_baseline()` 只更新上一对计数，保留现有位姿和轮角。它与 `reset()` 把整份里程计归零不同。

### 11.9 sample() 为什么混合了两类结果

```python
x, y, yaw, _, _ = self.filter.state
raw = self.raw.sample()
return OdometrySample(x, y, yaw,
                      raw.left_wheel_angle_rad,
                      raw.right_wheel_angle_rad)
```

前三个量是车身的融合位姿；后两个量是编码器积累得到的左右轮自身转角。

轮子模型在 ROS 中显示转了多少圈，应来自编码器计数，不应拿 gyro 修正后的车身朝向去假装车轮旋转。`_` 在这里表示这次解包不使用的两个元素。

## 12. 用真实函数跑一遍：从 0 秒到 0.10 秒

这一节把所有部分串起来。使用当前默认配置、默认几何参数；输入是教学构造数据。表中数值由当前实现计算后四舍五入。

### 12.1 输入时间线

```text
t=0.00：左右编码器都是 100000，建立基准。
t=0.02：gyro z 对应 10°/s。
t=0.04：再次收到同样大小的新 gyro 读数。
t=0.05：编码器左 103000，右 103420。
t=0.06：再次收到同样大小的新 gyro 读数。
t=0.10：编码器左 106000，右 106840。
```

这里没有 t=0.08 的 IMU 事件，表示这组教学输入只提供上面这些事件。不能把后面的数值用于另一个添加了 t=0.08 观测的事件序列。

原始 gyro 计数取 655，轴映射为默认同向时：

```text
655 / 65.5 = 10 °/s
10 × π / 180 = 0.174532925 rad/s
```

“再次收到同样大小的新读数”是新样本；不是把同一条缓存读数重复使用。

### 12.2 t=0.00：为什么五个状态都还是零

调用：

```python
odom.update(100000, 100000, 0.065, 60000, 0.162, 0.0)
```

只建立计数基准与时间。没有上一对计数可差分，不能产生有效速度观测。

```text
X = [0,0,0,0,0]
P 对角线 = [0.000001,0.000001,0.000001,1,1]
```

### 12.3 t=0.02：第一条 gyro 如何改变朝向和角速度

先从 0.00 预测到 0.02。因为初始速度和角速度估计都是零，预测的 X 仍是零，但 P 已经变化：

```text
Pωω_predicted = 1 + 4×0.02 = 1.08

Pθω_predicted = 原角速度不确定性通过 F 传播 + Qθω
             = 1×0.02 + 0.0008
             = 0.0208
```

注意第二行：即使当前角速度数值为零，其误差方差并不为零。过去这段时间可能存在未掌握的角速度，因此朝向误差与角速度误差产生了联系。

gyro 修正：

```text
residual = 0.174532925 - 0 = 0.174532925
S = 1.08 + 0.0009 = 1.0809

Kω = 1.08/1.0809 ≈ 0.999167361
Kθ = 0.0208/1.0809 ≈ 0.019243223
```

完整 K 的其他三项此时为零，因此：

```text
ω_new = 0 + 0.999167361×0.174532925
      ≈ 0.174387602 rad/s

θ_new = 0 + 0.019243223×0.174532925
      ≈ 0.003358576 rad
```

新状态：

```text
[0, 0, 0.003358576, 0, 0.174387602]
```

`Kθ` 的单位为秒，乘 rad/s 得到 rad。它不是“1.924% 的朝向可信度”。

gyro 没有给绝对角度，但通过 θ 与 ω 的关联修正了当前相对朝向。这也不是回放历史轨迹的平滑器：当前实现只更新当前五状态和 P，没有保存、回写整个历史状态序列。

此时 NIS 约为 0.02818，小于 25，接受观测。角速度方差从 1.08 降为约 0.000899251。

### 12.4 t=0.04：为什么不是再次加固定 0.02×gyro

先预测：

```text
θ_predicted = 0.003358576 + 0.174387602×0.02
            ≈ 0.006846328 rad
ω_predicted = 0.174387602 rad/s
```

再与新的 gyro 比较：

```text
residual ≈ 0.000145323 rad/s
S ≈ 0.081799251
Kω ≈ 0.988997454
Kθ ≈ 0.010211633
```

修正后：

```text
θ ≈ 0.006847812 rad
ω ≈ 0.174531326 rad/s
```

预测先依据之前的估计积累角度，再根据新读数修正。这个过程与每次拿最新 gyro 直接乘整个编码器周期不同。

### 12.5 t=0.05：第一条真正的轮速观测

两次编码器间隔为 0.05 秒，得到：

```text
v_wheel ≈ 0.218497769 m/s
ω_wheel ≈ 0.176472180 rad/s
```

EKF 上次已到 0.04，所以只再预测 0.01 秒。预测状态为：

```text
X_predicted ≈ [0, 0, 0.008593125, 0, 0.174531326]
```

此时还没收到过有效轮速修正，v 估计仍为零，但 v 的不确定性大。轮观测差值：

```text
residual_v ≈ 0.218497769
residual_ω ≈ 0.001940854
```

本次 K 约为：

```text
                 轮速差值的系数    车轮角速度差值的系数
x                 0.049148053       0
y                 0.000177472       0
θ                 0                 0.003440466
v                 0.998452012       0
ω                 0                 0.645054972
```

例如位置与速度的交叉项，此时为：

```text
Pxv ≈ 0.050799428
Svv = 1.032 + 0.0016 = 1.0336
Kx,v = 0.050799428 / 1.0336 ≈ 0.049148053
```

所以 x 更新为：

```text
x_new = 0 + 0.049148053×0.218497769
      ≈ 0.010738740 m
```

你可能疑惑：“刚才预测 x 还是零，怎么来了速度观测，x 就不是零了？”

原因是：这段时间里 x 的误差与未知 v 的误差已经通过预测产生关联。新观测表明车确实在前进，就会同时修正当前 x 和 v 的估计。这不是直接把编码器位置观测融合进来了，也不是再次额外积分一遍位移。

完整更新结果：

```text
x ≈ 0.010738740 m
y ≈ 0.000038777 m
θ ≈ 0.008599803 rad
v ≈ 0.218159537 m/s
ω ≈ 0.175783284 rad/s
```

### 12.6 t=0.06：有前进速度后，gyro 也能轻微影响位置

预测到 0.06：

```text
x ≈ 0.012920237400 m
y ≈ 0.000059455712 m
θ ≈ 0.010357635610 rad
v ≈ 0.218159537217 m/s
ω ≈ 0.175783283572 rad/s
```

gyro 观测比预测角速度小约 0.001250358 rad/s。此时计算出的 K 约为：

```text
Kx = -0.000000055967
Ky =  0.000005904297
Kθ =  0.007625326628
Kv =  0
Kω =  0.983758539835
```

更新后：

```text
x ≈ 0.012920237470 m
y ≈ 0.000059448330 m
θ ≈ 0.010348101219 rad
v ≈ 0.218159537217 m/s
ω ≈ 0.174553232845 rad/s
```

这个例子里位置修正极小，但不严格为零；它来自运动方向、角速度与位置误差的关联。不能由此推断 gyro 已经能提供可靠的独立平移定位。

### 12.7 整体结果表

| 时间/s | 事件 | x/m | y/m | yaw/rad | v/(m/s) | ω/(rad/s) |
|---:|---|---:|---:|---:|---:|---:|
| 0.00 | 编码器建基准 | 0 | 0 | 0 | 0 | 0 |
| 0.02 | gyro 更新 | 0 | 0 | 0.003358576 | 0 | 0.174387602 |
| 0.04 | gyro 更新 | 0 | 0 | 0.006847812 | 0 | 0.174531326 |
| 0.05 | 编码器更新 | 0.010738740 | 0.000038777 | 0.008599803 | 0.218159537 | 0.175783284 |
| 0.06 | gyro 更新 | 0.012920237 | 0.000059448 | 0.010348101 | 0.218159537 | 0.174553233 |
| 0.10 | 编码器更新 | 0.021654979 | 0.000180407 | 0.017364158 | 0.218482394 | 0.176236740 |

到 0.10 秒，纯编码器路径给出：

```text
x ≈ 0.021848714 m
y ≈ 0.000192790 m
yaw ≈ 0.017647218 rad
```

两种结果不同，因为 EKF 同时考虑了时间预测、角速度观测和不确定性。这一小组数值的差异不能用来宣称哪一种更接近实车真值；这里只是在验证与解释程序计算过程。

## 13. chassis_can_node.py：这套数学怎样接到你的原程序

### 13.1 节点启动时读取模式与参数

```python
self.declare_parameter("odometry.mode", "ekf")
self._fusion_mode = str(self.get_parameter("odometry.mode").value)
```

只接受 `ekf` 和 `weighted`。随后把 `ekf.*` 参数读取成 `EkfConfig`，创建 `EkfOdometry`。

这样保留了旧算法的选择入口，你不需要删除新代码才能回到固定权重方式。串口配置默认选择 EKF，CAN 配置仍是 weighted，不能认为两个启动入口都使用了相同新流程。

EKF 模式还检查 `imu.frame_id` 与 `base_frame` 一致。这是要求目前传入的 gyro 已按车身坐标轴解释；程序没有在这里查询任意 TF 自动完成 IMU 安装旋转。帧名字相同也不能证明物理安装正确，轴映射仍需核对。

### 13.2 IMU 到来时：沿着实际调用顺序看

相关函数是 `_publish_imu_if_pair_ready()`。已有流程先检查：

```text
加速度帧和角速度帧都存在。
两帧 sequence 一样，属于同一组遥测。
这组 sequence 还没有发布过。
下位机状态确认 calibrated。
```

只有这些成立才进行轴映射和单位换算。

```python
angular_velocity = tuple(
    value * degrees_to_radians / gyro_scale
    for value in gyro_counts
)
```

节点也按已有逻辑发布完整的 `/imu/data_raw`。随后新增的 EKF 接入点为：

```python
self._latest_gyro_z_radps = angular_velocity[2]
self._last_imu_time = time.monotonic()
self._last_published_imu_sequence = acceleration.sequence
if self._fusion_mode == "ekf" and bool(self.get_parameter("imu.fusion_enabled").value):
    self._ekf_odometry.add_imu(self._latest_gyro_z_radps, self._last_imu_time)
```

展开为：

```text
本组 gyro z，已经是 rad/s
 → add_imu()
 → 时间与车轮数据新鲜度检查
 → observe_gyro()
 → predict_to()
 → _correct([4], ...)
```

只有 z 角速度进入 EKF；加速度帧在这一流程中仍参与数据配对，并被发布，但数值没有传入滤波核心。

当前实现直接在同一个节点的接收流程中调用数学对象，**不是发布 `/imu/data_raw` 后再用另一个 EKF 订阅器订阅它**。也没有调用外部 `robot_localization` 包。

### 13.3 编码器到来时：新增的模式分支

`_update_odometry_if_pair_ready()` 先确认左右编码器帧同序号、未重复处理，再读轮径、计数比例和轮距。

新增 EKF 分支：

```python
if self._fusion_mode == "ekf":
    sample = self._ekf_odometry.update(
        left.total_count, right.total_count,
        diameter, counts_per_rev, wheel_track, now)
    self._imu_fusion_active = self._ekf_imu_active(now)
    if sample is not None:
        self._publish_odometry_and_joints(sample)
    return
```

这里调用的正是第 11 节解释的累计计数适配器。分支末尾 `return`，所以不会再执行下面的旧固定权重算法。

也不会在这里把 `_latest_gyro_z_radps` 再送入 EKF 一遍；gyro 已经在其自己的消息事件中使用过了。

### 13.4 内部更新频率与 /odom 发布频率不是同一件事

当前代码中：

```text
IMU 到来：内部 EKF 可以预测并修正，同时发布 IMU 消息。
编码器到来且返回 sample：发布 odom、joint_states 和可选 TF。
```

因此不能因为 IMU 约 50 Hz 更新，就宣称 `/odom` 已经被改成 50 Hz 发布。正常情况下里程计发布主要跟随约 20 Hz 的编码器配对事件。

下位机约 100 Hz 采集和上位机约 50 Hz IMU 遥测也不是同一个频率，滤波器只处理真正收到且有效的数据组。

### 13.5 时间戳目前仍有什么近似

| 时间用途 | 当前来源 |
|---|---|
| EKF 的 `timestamp` | 上位机 `time.monotonic()` 接收处理时间 |
| 编码器计算速度的时间差 | 两次配对编码器接收处理时间差 |
| ROS 消息 `header.stamp` | 节点 ROS 时钟，发布时读取 |

`time.monotonic()` 是单调时钟，适合测量经过了多少秒，不把普通系统时间校正直接当成运动时间变化。

但它不是 MCU 采样时间。串口传输延迟、缓冲和不同数据帧到达时间会影响融合时序。本次没有实现 MCU 时间戳同步或历史状态回放，也不能把 ROS 发布时刻等同于真实传感器采样时刻。

### 13.6 _publish_odometry_and_joints() 怎样输出五状态

```python
odom.pose.pose.position.x = sample.x_m
odom.pose.pose.position.y = sample.y_m
odom.twist.twist.linear.x = self._ekf_odometry.filter.state[3]
odom.twist.twist.angular.z = self._ekf_odometry.filter.state[4]
```

`pose` 是位姿，`twist` 是线速度与角速度。两者现在都来自同一个 EKF 估计，避免发布的速度仍停留在旧固定权重计算结果。

ROS 姿态字段用四元数表示。在只有 yaw 的平面情况下：

```python
quaternion_z = math.sin(0.5 * sample.yaw_rad)
quaternion_w = math.cos(0.5 * sample.yaw_rad)
```

另外两个分量为零。这一步只是把同一个朝向换一种消息表达，不是又进行了一次姿态融合。

`joint.position` 使用左右轮累计转角。`joint.velocity` 保留原来从 `_latest_speed` 轮速遥测除以半径的来源，它不是五状态 EKF 输出的左右轮速度。调试时要分清这几条不同来源。

### 13.7 ros_covariances() 为什么把 5×5 变成两个 6×6

ROS 的位姿协方差按以下顺序：

```text
[x, y, z, roll, pitch, yaw]
```

我们的位姿只估计 x、y、yaw，所以内部索引 `[0,1,2]` 对应 ROS 索引 `[0,1,5]`。

速度协方差的排列是：

```text
[vx, vy, vz, wx, wy, wz]
```

我们的 v、ω 对应索引 0、5。代码：

```python
pose[ros_i * 6 + ros_j] = self.filter.covariance[i][j]
```

因为 ROS 用长度为 36 的一维列表保存 6×6 表，行 r、列 c 放在 `r*6+c`：

```text
pose[0]  ← Pxx
pose[7]  ← Pyy
pose[35] ← Pθθ
pose[5]  ← Pxθ
twist[0] ← Pvv
twist[35]← Pωω
```

不只是三个对角线，已有的位姿交叉项和 v/ω 交叉项也会映射。

没有估计的轴，其对角线先设 `1e6`，表明这里不能提供可信的该轴估计；不要填零误导下游认为完全没有误差。位姿与速度之间的交叉协方差仍保留在内部 P，但 `Odometry` 分开的这两个字段不能承载所有跨块关联。

### 13.8 为什么新增 /wheel/odom_raw，但不让它发布 TF

`_publish_raw_wheel_odometry()` 发布纯编码器结果，便于你把两条轨迹画在一起比较。

raw 位姿没有单独传播 P，所以用大方差标识，不能冒充已经统计建模好的高精度位姿。有效 raw 速度的方差来自配置里的编码器观测噪声。

TF 表达坐标系之间的当前变换。若两个算法都发布同一条 `odom → base_link`，下游会收到互相竞争的位置结果。因此当前仍只有主 `/odom` 对应的一份 TF，raw 只做消息对照。

这里写的 `/odom` 等是假定默认无额外命名空间时的名称；节点创建的是相对话题名，实际部署使用命名空间或重映射后名称可能不同。

### 13.9 _ekf_imu_active() 和 reset 的新增处理

`_ekf_imu_active()` 检查启用、校准状态，以及最近接受 gyro 的时间是否在 `imu.timeout_s` 内。

它表示最近是否成功接受新 IMU 信息，不表示 `False` 时算法完全不含任何历史 IMU 影响。关闭新 IMU 更新后，已有状态仍保存以前融合的结果。

`_reset_local_odometry()` 同时清理：

```text
原 WheelOdometry
新 EkfOdometry 和其中的 PlanarEkf
缓存的左右编码器配对
上一次里程计序号与时间
IMU 融合活跃标志
```

MCU 心跳识别重启时会进入重置并清理相关 IMU 缓存；里程计重置服务成功发送重置请求后，也会同步清理本地估计。这样下一对计数可以重新建基准，不接着用重置前的旧累计计数做差。

### 13.10 参数回调为什么要求某些修改重启

`_on_parameters_changed()` 对模式、`ekf.*`，以及 EKF 模式下的关键几何和 IMU 映射参数要求重启。

例如旧轮径下累计出的 x 和 P，不能在中途换一个轮径后就假装所有历史都由新模型产生。当前选用较简单的方式：编辑 YAML，重启节点，重新建立局部估计。

运行时开关 `imu.fusion_enabled` 仍可控制是否接受新的 IMU 更新。`imu.fusion_weight` 只在 weighted 模式使用。

## 14. 参数不是越小越好：怎样理解以后要调的数

### 14.1 R：你声称传感器有多可靠

| 参数变化 | 模型中的主要含义 |
|---|---|
| 增大 `ekf.gyro_stddev` | 对 gyro 观测更谨慎；在相同先验下，直接角速度增益一般变小 |
| 减小 `ekf.gyro_stddev` | 更相信 gyro，也可能更相信其未消除的偏置 |
| 增大 `ekf.wheel_speed_stddev` | 更不确定轮速观测；可能减弱对速度的修正 |
| 增大 `ekf.wheel_yaw_rate_stddev` | 更不确定左右轮差推导出的角速度 |

如果把噪声设得极小，程序可能显得很“相信”传感器，但相信不等于正确。也可能因为 S 太小，把真实变化当成异常拒绝。

当前 R 把 `v_wheel` 与 `ω_wheel` 的观测误差近似当作独立。实际上它们都来自同一对轮速。当左右轮误差方差不一样时，二者可能相关；未来可以从标定数据构建完整的 2×2 R。

### 14.2 Q：你允许运动偏离匀速预测到什么程度

增大过程噪声，表示更承认车速或转弯速度会在预测期间变化。通常会让估计更愿意响应新观测，也可能更容易跟随噪声。

减小过程噪声，表示更信任当前速度的延续，可能更平滑，也可能跟不上启动、刹车或快速转弯。

这种影响是整个递推共同作用的结果，不能保证某个参数调大后所有状态的每一个增益都单调变化。

### 14.3 三种容易混淆的参数

```text
imu.angular_velocity_stddev
    → 发布 /imu/data_raw 消息时填写的角速度协方差。

ekf.gyro_stddev
    → 当前内部 EKF 真正用于 gyro 更新的 R。

imu.fusion_weight
    → 旧 weighted 模式的固定权重，不是 EKF 的 K。
```

前两项当前是分别配置的，不会自动同步。只改 IMU 消息的协方差，不会自动改变内部 `_correct()` 使用的 `ekf.gyro_stddev`。

`ekf.max_wheel_speed_mps=2.0` 是观测合理性检查上限，**不是命令电机允许跑到 2 m/s**；电机命令限速由原控制配置负责。

### 14.4 你现在没有实物，先不要凭轨迹好看就调参

现在可以学会参数的作用、复现数学例子、检查输出结构。拿到实物后再检查安装方向、静止 gyro、轮径/计数比例和时间延迟，并记录数据与参考轨迹比较。

平滑的曲线也可能整体方向错误；很小的 P 也可能只是模型过度自信，不能据此宣称精度高。

## 15. 当前实现能做到什么，还没做到什么

### 15.1 它是真正的五状态 EKF 递推

代码包含非线性运动函数、F 的线性化、P/Q 传播、观测残差、动态 K 和 P 更新。这些是它区别于原固定权重的具体计算内容。

但它是面向二维里程计的一版简化实现：

```text
没有 z 高度、roll、pitch 状态。
没有加速度观测进入状态更新。
没有 gyro bias 状态。
没有 GPS、视觉或雷达绝对位置观测。
没有实际舵角观测。
没有 MCU 采样时钟同步或延迟观测回放。
```

### 15.2 为什么 gyro 零偏仍然会导致错误

假设车静止，gyro 却长期读到 `0.015 rad/s`。这叫偏置：它不是正负随机波动围绕零，而是持续偏向一侧。

目前观测模型写成：

```text
gyro = ω + 噪声
```

如果要明确估计偏置，模型可能需要变为：

```text
gyro = ω + bias + 噪声
```

并新增 bias 的状态、运动假设和可识别性分析。当前五个状态里没有它，所以不能把持续偏差自动分离为“真实转弯”和“陀螺仪偏置”。编码器会提供制约，但不保证能彻底消除漂移。

下位机已有的静止标定可以减去启动时估计的零偏，但不等于上位机已经有在线 bias 跟踪；温度变化和残余偏差仍可能存在。

之前的合成实验中，“gyro 带持续偏置”场景的 EKF 误差比固定权重更大，这个结果保留在第 27 号文档。它提醒我们当前默认模型和参数有局限，不能只因叫 EKF 就认为所有情况都会改善。

### 15.3 两个轮子一起打滑，gyro 为什么救不了平移

如果两轮以差不多的程度一起空转，编码器可能报告前进，gyro 却正确报告没有转弯。这两件事并不矛盾，滤波器可能仍估计车辆在直行。

gyro 主要提供转动信息，不能直接确认车到底沿地面前进了几米。因此没有外部参考时，两轮共同打滑的平移误差仍可能保留下来。

### 15.4 没有 IMU 时不是彻底不能工作

只要轮观测有效，EKF 可以只接收编码器的 v 和 ω 并继续估计。IMU 恢复后又可以接受 gyro 更新。

这时依然是同一个 EKF，不会突然把当前位姿替换为 raw 轨迹。若要从零重新做不含历史 gyro 的对照，应从一致初始条件重置并运行，而不是只观察当前 `imu_fusion_active=False`。

### 15.5 概率假设也有近似

卡尔曼计算基于一定的误差统计模型。实际编码器比例误差、打滑和 IMU 滑动平均后的相邻样本相关性，都可能偏离当前简化假设。

“每条观测只用一次”避免了最直接的重复利用问题，但不代表真实相邻传感器噪声已经严格独立。P 是模型推算出的不确定性，需要真实数据检查它是否与实际误差相称。

## 16. 新增测试代码在验证什么

### 16.1 test_planar_ekf.py：检查公式与算法行为

其中一个测试直接对应第 3 节：

```python
ekf = PlanarEkf(EkfConfig(gyro_stddev=0.5))
ekf.observe_gyro(1.0, 0.0)
```

初始 `Pωω=1`，这里特意令 `R=0.5²=0.25`，所以：

```text
K=1/(1+0.25)=0.8
ω_new=0.8
Pωω_new=0.2
```

测试用断言检查这些数。注意这里直接调用 `PlanarEkf`，不经过要求先有轮基准的 `EkfOdometry`；它是在单独检查数学函数。

`test_motion_jacobian_matches_numerical_derivative()` 检查手写 F 有没有求导或索引错误：

```text
某个状态加一点 ε，运行 motion。
同一个状态减一点 ε，再运行 motion。
两次输出的差 / (2ε)，应接近 F 对应列。
```

这叫有限差分验证。例如测试第 3 列，就是用两种稍有不同的速度看位置预测怎样变化，再与解析导数比较。

其他测试覆盖圆弧预测、协方差对称与正定、非法数值、异常观测、重复 IMU、纯编码器对照、无 IMU、恢复、长间隔、计数环绕、跳变、重置及 ROS 协方差索引。

### 16.2 test_node_ekf.py：检查算法是否接到了正确数据上

公式正确仍可能接错线，例如把度每秒当弧度每秒、同一 IMU 用了两次、raw 意外包含 IMU、两个结果同时发 TF。

节点测试用最小 ROS 消息和传输替身，让真实节点方法处理构造的协议帧，检查配对、去重、发布、校准保护、模式和重置等逻辑。

这是离线替身测试，不等于真正启动了 ROS 2 DDS 通信，也不能证明 RDK 的串口和部署环境全部正常。

### 16.3 simulate_ekf.py：看连续运动时的表现

它人工生成运动真值、编码器和 gyro，再比较：

```text
纯编码器
原固定权重
新 EKF
```

四个场景包括转弯噪声、gyro 偏置、IMU 断流和静止偏置。脚本输出 CSV 和汇总误差，用来观察算法对假设问题的反应。

RMSE 是均方根误差：把各次误差平方后取平均再开方，是概括一段轨迹误差的指标。它依赖这里人为生成的真值和噪声，不能直接当成你实际车辆的定位精度。

先前实现验证共有 28 项 Python 测试通过；Python 3.10 语法可解析检查也通过。实际执行解释器为本机可用的 Python 3.14，尚没有真实 Humble 环境和实车验收记录。

## 17. 没有实物时，怎样自己复现并看懂这些值

### 17.1 可以只运行数学代码，不用启动 ROS

在 VS Code 中打开项目根目录，终端使用 PowerShell。先切到实际项目：

```powershell
Set-Location -LiteralPath 'C:\Users\xixun\Desktop\C30D_Development_Handoff_20260909'
```

本机本次已经使用过的解释器为：

```powershell
$ekfPython = 'C:\Users\xixun\AppData\Local\Autodesk\webdeploy\production\257040cabc1dffce734a8079453b19b0ffe2b735\Python\python.exe'
```

这个路径是当前机器现有应用附带的 Python；以后该应用更新可能改变路径。如已配置自己的 Python，可改用对应解释器。

下面整段粘贴到 PowerShell，会把代码传给 Python 运行，不需要保存新 `.py` 文件：

```powershell
@'
import math
import sys
from pathlib import Path

# 让 Python 找到项目里的包，不需要安装 ROS 或 NumPy。
sys.path.insert(0, str(Path.cwd() / 'ros2_ws/src/chassis_can_control'))
from chassis_can_control.planar_ekf import EkfOdometry

odom = EkfOdometry()
gyro = 655 / 65.5 * math.pi / 180.0

def show(label):
    print('\n' + label)
    print('state [x,y,yaw,v,omega]:')
    print(['%.9f' % value for value in odom.filter.state])
    print('P diagonal:')
    print(['%.9f' % odom.filter.covariance[i][i] for i in range(5)])
    print('last NIS:', odom.filter.last_nis)

odom.update(100000, 100000, 0.065, 60000, 0.162, 0.00)
show('t=0.00 baseline')
odom.add_imu(gyro, 0.02)
show('t=0.02 gyro')
odom.add_imu(gyro, 0.04)
show('t=0.04 gyro')
odom.update(103000, 103420, 0.065, 60000, 0.162, 0.05)
show('t=0.05 wheel')
odom.add_imu(gyro, 0.06)
show('t=0.06 gyro')
odom.update(106000, 106840, 0.065, 60000, 0.162, 0.10)
show('t=0.10 wheel')
print('\nraw:', odom.raw.sample())
'@ | & $ekfPython -B -
```

`-B` 避免生成 Python 字节码缓存，最后的 `-` 表示从标准输入读取这段代码。`show()` 只是显示内部数值，没有改写 EKF 公式。

第 12 节结果表就是这一事件序列。最末五状态应接近：

```text
[0.021654979, 0.000180407, 0.017364158, 0.218482394, 0.176236740]
```

### 17.2 如果想看 K，应该在哪里下断点

打开 `planar_ekf.py`，在 `_correct()` 中这一行之后观察：

```python
gain = multiply(pht, inverse)
```

单步调试时重点看：

```text
indices       → 当前是哪种观测
values        → 新观测
self.state    → 修正前预测
residual      → 差值
innovation    → S
gain          → K
self.covariance → P
```

跨过更新 `self.state[i]` 的循环，再看 `self.state` 怎样变化。教学脚本若保存为单独文件并由对应解释器调试，就能用 VS Code 断点观察；不需要实物串口参与。

不建议初学时同时单步进入串口轮询、ROS 初始化和矩阵计算。先用上面的少量确定输入看清数学，再顺着第 13 节看节点调用。

### 17.3 运行已有测试和连续仿真

仍在项目根目录：

```powershell
& $ekfPython -B ros2_ws/src/chassis_can_control/test/run_protocol_tests.py
& $ekfPython -B tools/simulate_ekf.py --output build/ekf_demo
```

测试入口检查已有协议、里程计与 EKF 测试。仿真生成在 `build/ekf_demo` 中，属于可再生输出，不需要纳入 Git。

这些命令只运行本地离线计算，不会连接小车或发电机控制指令。

### 17.4 四个小练习及答案

**练习 A：左右编码器增量完全相同，轮观测角速度是多少？**

答案：0，因为 `(right_speed-left_speed)/track=0`。若同时 gyro 给出非零值，EKF 会根据 P、R 和门控结果处理分歧，不会直接规定必须取零。

**练习 B：为什么把 `gyro_stddev` 从 0.03 改成 0.06，R 会变成原来的四倍？**

答案：进入 R 的是标准差平方，`0.06²/0.03²=4`。不是两倍。

**练习 C：第一次收到累计计数 500000，为什么 x 仍为零？**

答案：只有一个计数没有增量与时间差，先建立基准。累计计数不一定从上位机启动那一刻开始。

**练习 D：gyro 增益只有最后一项非零，可能是什么原因？**

答案：当时其他状态与 ω 的交叉协方差为零。例如直接用刚 reset 的 `PlanarEkf` 在初始同一时刻做 gyro 修正，还没有经过运动传播形成关联。

## 18. 读完后，用这张对照表回看代码

| 你在问的问题 | 对应位置 | 应该抓住的内容 |
|---|---|---|
| 车现在估计在哪里、怎么运动？ | `PlanarEkf.state` | 五状态固定顺序 |
| 程序对估计有多确定？ | `PlanarEkf.covariance` | 方差与交叉协方差 |
| 过一小段时间会怎样？ | `motion()` | 中间朝向、距离投影、角速度积分 |
| 预测误差怎样传播？ | `_predict_step()` | `F P Fᵀ + Q` |
| 两个传感器不是同时到达怎么办？ | `predict_to()` 与观测入口 | 按事件推进同一状态 |
| 这条观测与预测差多少？ | `_correct()` 的 `residual` | `z-HX` |
| 这个差值是否过大？ | `_correct()` 的 `nis` | 相对于 S 的归一化差异 |
| 本次更相信观测多少？ | `_correct()` 的 `gain` | 由 P、H、R 计算 K |
| 为什么角速度能修正位置？ | P 的交叉项与 K 的对应行 | 运动形成的误差关联 |
| 编码器原始数怎样变成速度？ | `EkfOdometry.update()` | 计数差、比例、dt、轮距 |
| 为什么不能反复用最新 gyro？ | `add_imu()` 与节点序号检查 | 同一观测不能重复当新证据 |
| 串口数据在哪接进来？ | 节点的两个 `*_if_pair_ready()` | 配对、校准、单位、调用 |
| 结果怎样发给 ROS？ | `_publish_odometry_and_joints()` | 五状态、协方差、四元数、单一 TF |
| 没数据时如何处理？ | `suspend()` 和计数重建 | 保留已知位姿，承认未知运动 |

你不需要先背会所有矩阵公式才开始读代码。先能解释一轮“预测现在—比较观测—按不确定性修正—保存结果”，再用第 12 节的数字去对应每个变量；当这些变量不再只是字母时，再看 F 的导数和 Joseph 更新就会容易很多。

本文只新增讲解文档，没有继续改变上一轮的 EKF 算法。真实 ROS 环境与实车精度仍需后续验证。

# STM32 下位机电机与 IMU 代码零基础详解

## 1. 这份文档讲什么

这份文档面向第一次接触电机控制、编码器、IMU 和 FreeRTOS 的读者。讲解对象不是一个抽象例程，而是当前已经编译、烧录到 C30D 底盘控制板上的串口版工程：

```text
keil_project/R550_C30D_SERIAL_MOTOR/
```

芯片为 STM32F407，主要代码语言是 C，实时任务由 FreeRTOS 调度。RDK X5 通过 USB 串口向它发送 ROS 速度命令。

先说明最重要的事实：

- 电机闭环、编码器测速、PWM、安全状态机已经实际接入当前控制链；
- IMU 芯片识别和 100 Hz 原始采集任务存在；
- 当前协议没有 IMU 报文，RDK 也没有 `/imu/data`；
- 当前电机 PI 和 ROS `/odom` 均未使用 IMU；
- 原厂 IMU 校准依赖的 `SysVal.Time_count` 原本由旧 `Balance_task` 更新，而当前工程没有创建这个旧任务，因此目前不能把 IMU 部分称为已经完整可用的姿态系统。

这不是说硬件没有 IMU，而是要区分“能读到传感器寄存器”和“完成了可靠校准、姿态解算、上传 ROS、参与融合”这几个不同阶段。

## 2. 先建立整体认识

一次电机控制的信息流是：

```text
RDK /cmd_vel
  ↓ 线速度 v、角速度 w、使能位
USART3 串口中断收字节
  ↓ 拼成完整 15 字节帧
chassis_serial_task 解析命令
  ↓ 保存最新目标和时间
chassis_control_task 每 10 ms 执行一次
  ├─读取左右编码器增量
  ├─换算左右实测轮速
  ├─底盘运动学得到左右目标轮速
  ├─斜坡、滤波、前馈和 PI
  ├─安全检查
  └─输出左右 PWM
       ↓
H 桥/电机驱动电路
       ↓
直流减速电机转动
       ↓
编码器产生 A/B 相脉冲，形成闭环
```

所谓“闭环”，就是控制器不只发出一个 PWM 就结束，而是不断比较：

```text
误差 = 目标速度 - 实测速度
```

如果车轮实际偏慢，就增加输出；如果偏快，就减小输出。这个过程每秒执行 100 次。

## 3. 读懂工程前需要的 C 语言知识

### 3.1 头文件与源文件

常见组合：

```text
wheel_controller.h    对外声明：有哪些结构体和函数
wheel_controller.c    具体实现：函数内部怎样计算
```

其他 `.c` 文件只有包含头文件后，才能知道这些函数怎样调用：

```c
#include "wheel_controller.h"
```

### 3.2 结构体

结构体把一组相关变量放在一起：

```c
typedef struct
{
    float ramped_target_mps;
    float filtered_speed_mps;
    float integral_output;
    int16_t output_pwm;
} wheel_controller_t;
```

可以把它理解成“一个车轮控制器的档案袋”。左轮和右轮分别有一份：

```c
wheel_controller_t left_controller;
wheel_controller_t right_controller;
```

访问结构体成员使用点号：

```c
controller.output_pwm
```

如果拿到的是结构体指针，则使用箭头：

```c
controller->output_pwm
```

### 3.3 指针为什么会出现

函数如果接收结构体副本，修改的只是副本。传入地址后，函数可以修改原对象：

```c
void wheel_controller_reset(wheel_controller_t *controller)
```

调用：

```c
wheel_controller_reset(&app->left_controller);
```

`&` 表示取地址，`*controller` 表示该地址指向的对象。

### 3.4 整数类型

项目使用明确位宽：

```text
uint8_t   8 位无符号，0～255
int16_t   16 位有符号，-32768～32767
uint16_t  16 位无符号，0～65535
int32_t   32 位有符号
uint32_t  32 位无符号
float     单精度浮点数
```

协议特别需要明确位宽，因为上下位机必须对“每个字段占几个字节”达成一致。

### 3.5 `static` 的两种常见含义

文件级函数前的 `static` 表示只在当前 `.c` 文件可见：

```c
static float clamp_float(...)
```

函数内静态变量则会在多次调用之间保留值。当前控制状态主要保存在结构体中，阅读更直接。

### 3.6 `volatile`

串口队列指针同时被中断和任务访问：

```c
static volatile uint8_t g_queue_write;
```

`volatile` 告诉编译器：这个变量可能在当前代码看不到的地方被改变，不要把它永远缓存到寄存器里。

## 4. 程序从哪里开始

入口是：

```text
USER/main.c
```

核心流程：

```c
int main(void)
{
    systemInit();
    xTaskCreate(start_task, ...);
    vTaskStartScheduler();
}
```

含义：

1. `systemInit()` 初始化串口、IMU、编码器、PWM、ADC 等硬件；
2. 创建一个临时的启动任务；
3. 启动 FreeRTOS 调度器；
4. 调度器开始按照优先级和周期运行各任务。

`start_task()` 在正常模式下调用：

```c
chassis_tasks_start();
```

它创建两个与本项目直接相关的任务：

```text
chassis_control_task  优先级 6，100 Hz 电机控制
chassis_serial_task   优先级 5，500 Hz 轮询通信和发送遥测
```

还会根据硬件版本创建：

```text
MPU6050_task 或 ICM20948_task，100 Hz IMU 采集
show_task，OLED 显示
led_task，LED 状态
```

旧 `Balance_task` 没有被创建。原因是它也会读取并清空编码器、写同一组 PWM；新旧两个控制任务同时运行会互相干扰。

## 5. `systemInit()` 初始化了什么

文件：

```text
BALANCE/system.c
```

关键初始化包括：

1. 中断优先级和延时；
2. I²C GPIO，供 IMU 使用；
3. USART1、USART2、USART3、USART5；
4. 读取 IMU 设备 ID，识别硬件版本；
5. 蜂鸣器、硬件使能、OLED、按键；
6. ADC 和电池电压；
7. CAN 外设（当前串口控制链没有使用它传速度）；
8. 底盘型号选择；
9. TIM2～TIM5 编码器接口；
10. TIM1、TIM9、TIM10、TIM11 电机 PWM；
11. TIM12 舵机 PWM；
12. Flash 参数读取。

### 5.1 PWM 为什么是 10 kHz

代码使用：

```c
TIM1_PWM_Init(16799, 0);
```

定时器输入时钟约为 168 MHz：

```text
PWM频率 = 168000000 / ((16799 + 1) × (0 + 1))
        = 10000 Hz
```

所以一个 PWM 周期为 0.1 ms。比较寄存器大致从 0 到 16799：

```text
PWM = 0      占空比约 0%
PWM = 8400   占空比约 50%
PWM = 16700  占空比接近 100%
```

10 kHz 的开关信号不是“电机每秒转一万圈”，而是驱动管每秒开关一万次。电机绕组的电感和机械惯性把它等效成平均电压。

## 6. 电机、驱动器和 PWM 的关系

### 6.1 MCU 不能直接带电机

STM32 GPIO 只能输出很小的电流，不能直接驱动电机。实际链路是：

```text
STM32 PWM/方向信号 → H 桥驱动芯片 → 电池大电流 → 电机
```

H 桥可以改变电机两端电压方向，因此支持正转和反转。

### 6.2 当前硬件输出入口

抽象接口位于：

```text
CHASSIS_SERIAL/src/c30d_chassis_port.c
```

```c
void c30d_port_set_output(int16_t left_pwm,
                          int16_t right_pwm,
                          uint16_t servo_pwm)
{
    Set_Pwm(left, right, 0, 0, servo_pwm);
}
```

R550 使用 A、B 两路电机，C、D 保持为 0。真正把正负 PWM 写到定时器和方向引脚的原厂函数是：

```text
BALANCE/balance.c 中的 Set_Pwm()
HARDWARE/motor.c 中的定时器 PWM 初始化
```

### 6.3 为什么把方向符号集中配置

```c
#define C30D_LEFT_PWM_SIGN  1
#define C30D_RIGHT_PWM_SIGN 1
#define C30D_RIGHT_ENCODER_SIGN (-1)
```

机械安装会让左右电机或编码器的物理正方向不同。集中配置符号后，控制算法内部始终遵循统一约定：

```text
正速度 = 小车向前
正编码器增量 = 车轮向前
```

如果方向错误，应检查接口层符号和接线，不要在 PI 公式各处随意加负号。

## 7. 编码器是什么

### 7.1 A/B 相正交编码器

每个电机带编码器，通常输出相位相差 90° 的 A、B 两路方波：

```text
A: __--__--__--
B: _--__--__--_
```

根据谁先变化，可以判断旋转方向；根据边沿数量，可以判断转过多少角度。

STM32 定时器的编码器模式由硬件自动计数，不需要每个边沿都进入软件中断。

### 7.2 定时器配置

文件：

```text
HARDWARE/encoder.c
```

当前左右轮通过接口层选择：

```text
左轮：TIM2
右轮：TIM3
```

读取函数：

```c
Encoder_TIM = (short)TIM2->CNT;
TIM2->CNT = 0;
```

它做了两件事：

1. 读取上一个控制周期内累计的正负脉冲数；
2. 立即清零，为下一个周期重新计数。

因此只能让一个控制任务调用它。若两个任务都读，先读的任务会清零，后读的任务几乎总得到 0。

### 7.3 编码器增量换算距离

设：

```text
D = 轮径，当前为 0.065 m
N = 车轮一圈对应计数，当前为 60000
Δcount = 本周期编码器增量
```

轮子一圈的周长：

```text
C = πD
```

一个计数对应的距离：

```text
meters_per_count = πD / N
```

代入当前参数：

```text
π × 0.065 / 60000 ≈ 0.000003403 m
                      ≈ 3.403 μm/计数
```

如果 10 ms 内收到 147 个计数：

```text
距离 ≈ 147 × 3.403 μm = 0.000500 m
速度 ≈ 0.000500 / 0.01 = 0.050 m/s
```

代码就是：

```c
speed = count_to_distance(delta) / dt_s;
```

### 7.4 为什么要检查编码器尖峰

通信干扰、接触不良或计数器异常可能让单周期增量突然很大。代码先判断实测速度是否超过物理上限：

- 单次异常：不进入 PI 和里程计；
- 连续三次异常：锁存 `LEFT_ENCODER` 或 `RIGHT_ENCODER` 故障。

这样既不会因一次毛刺让 PWM 突然反打，也不会长期忽略真实故障。

## 8. 100 Hz 控制任务

文件：

```text
CHASSIS_SERIAL/src/chassis_tasks.c
```

```c
#define CONTROL_PERIOD_MS 10U
```

所以任务每 10 ms 运行一次：

```c
vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
```

`vTaskDelayUntil()` 按绝对周期唤醒，比“运行完再延时 10 ms”更稳定。若本次计算花了 1 ms，下一次仍以原计划时刻为基准，不会逐渐漂移。

每周期步骤：

```text
读取当前时间
计算真实 dt
读取并清零左右编码器
获取互斥锁
执行 chassis_app_control_step()
复制 PWM 和舵机输出
释放互斥锁
调用 c30d_port_set_output()
```

如果拿不到互斥锁，本周期直接输出零，安全优先。

### 8.1 为什么需要互斥锁

控制任务和串口任务都会访问 `g_chassis_app`：

- 控制任务修改轮速、PI、状态和 PWM；
- 串口任务修改最新命令、参数并读取遥测。

如果两者同时修改同一结构体，可能出现半旧半新的数据。互斥锁像一把钥匙，同一时刻只允许一个任务进入共享区。

## 9. 从车体速度到左右轮速度

上位机发来的主要量是：

```text
v = linear.x，车体中心线速度，m/s
w = angular.z，车体偏航角速度，rad/s
L = 左右轮距，m
```

差速运动学：

```text
v_left  = v - wL/2
v_right = v + wL/2
```

例如：

```text
v = 0.20 m/s
w = 0.50 rad/s
L = 0.162 m
```

得到：

```text
v_left  = 0.20 - 0.50×0.162/2 = 0.1595 m/s
v_right = 0.20 + 0.50×0.162/2 = 0.2405 m/s
```

右轮更快，所以车辆向左转。

### 9.1 阿克曼模式

当前 YAML 中：

```text
geometry.chassis_type: 1.0
```

协议规定：

```text
0 = 差速
1 = 阿克曼
```

阿克曼车用舵机转向，不能像差速车一样原地自转。代码先计算舵角：

```text
steering_angle = atan(wheelbase × w / v)
```

线速度接近 0 时，代码会把角速度也置 0，避免发出不可能执行的原地旋转命令。舵角还会限幅，再换算为舵机 PWM：

```text
servo_pwm = center_pwm + pwm_per_rad × steering_angle
```

当前中心值为 1500，换算系数约为 636.56 PWM/rad。

## 10. 速度闭环的完整计算

文件：

```text
CHASSIS_SERIAL/src/wheel_controller.c
```

每个车轮各运行一套完全独立的控制器。

### 10.1 第一步：实测速度低通滤波

编码器在低速时一个周期可能是 14 个计数，下一个周期是 15 个计数，因此瞬时速度会跳动。使用一阶低通：

```text
filtered = filtered + α × (measured - filtered)
```

当前 `α = 0.35`。假设旧滤波速度 0.040，当前实测 0.060：

```text
filtered = 0.040 + 0.35 × (0.060 - 0.040)
         = 0.047 m/s
```

`α` 越大越灵敏但更抖，越小越平滑但延迟更明显。

### 10.2 第二步：目标速度斜坡

如果目标从 0 突然跳到 0.6 m/s，直接控制会产生很大电流和机械冲击。代码限制每周期目标变化量：

```text
本周期最大加速变化 = acceleration × dt
本周期最大减速变化 = deceleration × dt
```

当前：

```text
加速度 0.50 m/s²
减速度 0.80 m/s²
dt = 0.01 s
```

所以每周期最多：

```text
加速 0.005 m/s
减速 0.008 m/s
```

换向时先使用减速度回到零，再向反方向加速，避免瞬间反打。

### 10.3 第三步：死区补偿与前馈

电机存在静摩擦。很小的 PWM 只能让电机嗡响，无法转动。代码根据正反方向加入死区：

```text
feedforward = Kff × target + deadzone
```

前馈是在误差出现前，根据目标速度预估所需 PWM。它让 PI 不必从零慢慢“追”出输出。

当前示例：

```text
Kff = 9000 PWM/(m/s)
target = 0.05 m/s
deadzone = 700
```

```text
feedforward = 9000 × 0.05 + 700 = 1150 PWM
```

这个数值只是控制器计算值，不等于一定正确；最终应通过架空和落地实验标定。

### 10.4 第四步：比例 P

```text
error = ramped_target - filtered_speed
P = Kp × error
```

如果目标 0.05、实测 0.04、`Kp = 3000`：

```text
error = 0.01 m/s
P = 3000 × 0.01 = 30 PWM
```

P 只看当前误差。Kp 太小，响应软；Kp 太大，容易振荡和噪声放大。

### 10.5 第五步：积分 I

```text
I_new = I_old + Ki × error × dt
```

积分把过去误差累加起来，用于消除长期小偏差。若 `Ki = 1500`、误差 0.01、周期 0.01 s：

```text
每周期积分增加 = 1500 × 0.01 × 0.01 = 0.15 PWM
```

积分不会立即很大，但持续有误差时会逐步增加。

### 10.6 为什么没有 D 项

PID 的 D 项根据误差变化率工作，对编码器量化噪声比较敏感。当前速度环采用：

```text
前馈 + P + I
```

再配合速度低通、目标斜坡，比较容易理解和调试。这是工程选择，不是说 D 项永远没用。

### 10.7 输出限幅和积分抗饱和

总输出：

```text
PWM_raw = feedforward + P + I
PWM = clamp(PWM_raw, -limit, +limit)
```

如果理论输出要求 20000，而硬件只能给 16700，继续累积积分会导致解除堵塞后突然冲出去，这叫积分饱和。

当前代码使用条件积分：

- 输出已顶到上限，误差还想让它更大：暂停积分；
- 误差方向有助于退出饱和：允许积分更新。

### 10.8 目标归零时清积分

当目标和斜坡目标都接近零，代码主动：

```text
目标 = 0
积分 = 0
PWM = 0
```

这样静止时不会因为残留积分继续通电、发热或啸叫。

## 11. 直行同步修正

即使左右轮目标相同，电机差异、轮胎阻力和地面摩擦也会让小车逐渐跑偏。

直行时记录左右起始距离，然后计算：

```text
sync_error = 左轮已走距离 - 右轮已走距离
sync_correction = Ksync × sync_error
```

若左轮走得更多，代码稍微降低左轮目标、提高右轮目标。修正量有限幅，避免该辅助环压过主速度环。

它只在角速度接近零且线速度不太小时启用，正常转弯时关闭。

## 12. 安全状态机

状态定义：

| 数值 | 状态 | 含义 |
|---:|---|---|
| 0 | `BOOT` | 启动阶段 |
| 1 | `IDLE` | 空闲，PWM 为零 |
| 2 | `RUNNING` | 允许输出且命令正常 |
| 3 | `TIMEOUT` | 命令超时 |
| 4 | `FAULT` | 严重故障 |
| 5 | `ESTOP` | 急停锁存 |

只有同时满足下列条件才进入 `RUNNING`：

```text
硬件使能有效
收到过运动命令
运动命令 enable 位为 1
命令未超时
没有急停
没有严重锁存故障
```

否则 PI 的 `allow_output` 为 0，控制器状态被清空，PWM 返回 0。

### 12.1 动态故障与锁存故障

动态故障随条件消失可以自动消失，例如：

```text
COMMAND_TIMEOUT
HARDWARE_DISABLED
CONTROL_OVERRUN
```

锁存故障不会因为瞬间恢复自动清除，例如：

```text
ESTOP
LOW_BATTERY
LEFT/RIGHT_ENCODER
LEFT/RIGHT_STALL
BAD_PARAMETER
```

锁存是为了避免故障刚刚消失就自动重新转动，必须由操作者排查后调用清故障命令。

### 12.2 双重命令超时

RDK 层：超过 0.20 s 没有新 `/cmd_vel`，就发送零速度和 `enable=0`。

STM32 层：超过 `command_timeout_ms` 没收到新运动帧，进入 `TIMEOUT`。

两层互不依赖。即使 RDK Python 卡死，STM32 仍能独立停止。

### 12.3 堵转检测

只有同时满足下列条件才累计堵转时间：

```text
目标速度足够大
实测速度很小或方向相反
PWM 已经很高
持续时间达到阈值
```

当前默认大致为：

```text
目标 ≥ 0.15 m/s
实测 ≤ 0.02 m/s
PWM ≥ 上限的 80%
持续 ≥ 600 ms
```

锁存堵转后，本周期立即把左右 PWM 置零。

### 12.4 欠压保护

ADC 读到的 `Voltage` 被换算为 mV。若非零电压低于阈值，锁存 `LOW_BATTERY`。设置合理阈值可以避免电池过放和低电压时电机控制异常。

## 13. 串口通信怎样实现

### 13.1 为什么内部还叫 CAN Frame

项目先设计了一套：

```text
logical_id + 8 字节数据
```

逻辑 ID 与 CAN 标准帧兼容，因此结构名保留为 `chassis_can_frame_t`。当前串口版只是在外面再包一层 15 字节串口帧。这不代表当前物理链路正在走 CAN。

### 13.2 15 字节串口帧

文件：

```text
CHASSIS_SERIAL/inc/chassis_serial_transport.h
```

| 字节 | 内容 |
|---:|---|
| 0 | `0xA5` 帧头 |
| 1 | `0x5A` 帧头 |
| 2 | `0x01` 外层协议版本 |
| 3～4 | logical ID，小端序 |
| 5 | 数据长度，固定 `8` |
| 6～13 | 8 字节逻辑数据 |
| 14 | 对字节 2～13 计算的 CRC-8/ATM |

串口配置是：

```text
115200 bit/s，8 数据位，1 停止位，无校验，无流控
```

### 13.3 串口中断为什么只做简单工作

每来一个字节，USART3 接收中断执行：

```text
读取字节
寻找 A5 5A 帧头
检查版本和长度
收满 15 字节
检查外层 CRC
将完整帧放入 8 槽环形队列
```

它不做 PI、Flash 擦写或复杂日志。中断应尽快退出，否则会阻塞其他实时工作。

### 13.4 环形队列

队列有读指针和写指针：

```text
中断：向 write 位置放帧，然后推进 write
任务：从 read 位置取帧，然后推进 read
```

`__DMB()` 是内存屏障，保证“帧内容写完”先于“写指针更新”被另一个执行上下文看到。

队列满时丢弃最新帧并增加 overrun 计数。运动命令仍有超时保护，不会因为丢一帧永久保持运动。

## 14. 逻辑协议怎样定义

主要 ID：

| ID | 方向 | 含义 |
|---:|---|---|
| `0x101` | RDK→STM32 | 运动命令 |
| `0x102` | RDK→STM32 | 参数读写 |
| `0x103` | RDK→STM32 | 清故障、保存、复位里程计 |
| `0x181` | STM32→RDK | 左右目标/实测轮速 |
| `0x182` | STM32→RDK | 左右 PWM 和速度误差 |
| `0x183` | STM32→RDK | 左编码器 |
| `0x184` | STM32→RDK | 右编码器 |
| `0x185` | STM32→RDK | 状态、故障、电压 |
| `0x186` | STM32→RDK | 参数/系统命令应答 |
| `0x700` | STM32→RDK | 心跳和固件版本 |

### 14.1 运动命令的 8 字节

| 字节 | 内容 |
|---:|---|
| 0～1 | `linear.x × 1000`，int16，小端 |
| 2～3 | `angular.z × 1000`，int16，小端 |
| 4 | bit0=使能，bit1=急停 |
| 5 | 协议版本 1 |
| 6 | 序号 sequence |
| 7 | 对前 7 字节的 CRC-8 |

例如 `0.05 m/s` 被编码为整数 `50`。使用毫米每秒和毫弧度每秒可以在 8 字节内传输且避免不同编译器直接传浮点结构的布局问题。

### 14.2 CRC 做什么

CRC 是差错检测，不是加密。如果某一位在传输中翻转，接收端重新计算的 CRC 通常不匹配，就丢弃该帧。

本项目使用：

```text
CRC-8/ATM
多项式 0x07
初始值 0x00
```

外层 15 字节帧有一次 CRC，部分 8 字节逻辑报文内部又有一次 CRC，形成两层检查。

### 14.3 sequence 的作用

序号每发一帧递增并在 255 后回到 0。它用于：

- 判断参数应答是不是对应本次请求；
- 区分左右编码器是否来自同一批数据；
- 诊断下位机最后处理到了哪条命令。

## 15. 遥测任务

`chassis_serial_task` 每 2 ms 醒来一次，先处理所有已接收命令，再按不同周期发送：

| 数据 | 周期 | 频率 |
|---|---:|---:|
| 轮速 + PWM/误差 | 20 ms | 50 Hz |
| 左右编码器 | 50 ms | 20 Hz |
| 状态/故障/电压 | 100 ms | 10 Hz |
| 心跳 | 1000 ms | 1 Hz |

RDK 根据这些帧发布 `/motor/debug`、`/odom` 和 `/diagnostics`。

## 16. 参数为什么可以从 RDK 修改

参数结构定义在：

```text
CHASSIS_SERIAL/inc/chassis_params.h
```

包含 PI、前馈、死区、轮径、轮距、编码器计数、速度限制、堵转和欠压阈值等。

参数修改流程：

```text
RDK 发送参数 ID + 操作 + float 值 + sequence
STM32 检查 CRC、运行状态、参数 ID 和范围
成功后修改 RAM 中的参数并清空 PI 历史状态
STM32 原样回应实际值和 sequence
RDK 确认应答
操作者验证正确后才发送保存命令
STM32 计算 CRC32 并写入 Flash
```

运行中拒绝改参数，是为了避免左右轮控制特性在一个周期内突然改变。

### 16.1 RAM 与 Flash

- RAM 参数：掉电丢失，适合临时测试；
- Flash 参数：掉电保留，但擦写寿命有限，验证后再保存。

保存前电机先被置零，进入临界区完成 Flash 写入，避免任务在半写入状态读取参数。

参数带有：

```text
magic = 0x43483330（ASCII CH30）
版本号
结构体大小
CRC32
```

这样可以识别空白 Flash、旧格式和损坏内容。

## 17. IMU 基础知识

### 17.1 IMU 是什么

IMU 是惯性测量单元。当前板可能搭载：

- 旧版：MPU6050，三轴加速度计 + 三轴陀螺仪；
- 新版：ICM20948，三轴加速度计 + 三轴陀螺仪 + 三轴磁力计。

启动时读取设备 ID：

```text
识别 MPU6050 → SysVal.HardWare_Ver = V1_0
识别 ICM20948 → SysVal.HardWare_Ver = V1_1
都不识别 → 系统复位
```

### 17.2 加速度计测什么

加速度计测的是比力。小车静止平放时仍会在重力方向看到约 1 g，而不是三个轴全为零。

当前量程为 ±2 g，典型灵敏度：

```text
16384 LSB ≈ 1 g
```

所以代码在校准 Z 轴后加回 16384，保留静止时的重力分量。

加速度计可以长期判断“重力朝哪个方向”，但运动加速度、震动和颠簸会干扰倾角判断。

### 17.3 陀螺仪测什么

陀螺仪测角速度，不是角度。当前量程为 ±500 °/s。要得到角度，需要积分：

```text
angle_new = angle_old + angular_velocity × dt
```

即使静止时只有很小的零偏，长时间积分也会累积成明显角度误差，称为漂移。

### 17.4 磁力计测什么

磁力计可以感知地磁方向，辅助修正航向长期漂移，但电机电流、钢铁结构和磁铁会严重干扰它。ICM20948 代码中磁力计采集目前被 `#if 0` 关闭。

## 18. 当前 IMU 代码怎样运行

文件：

```text
BALANCE/imu_task.c
BALANCE/imu_task.h
HARDWARE/MPU6050/MPU6050.c
HARDWARE/ICM20948/ICM20948.c
```

任务频率为 100 Hz，每周期读取三轴加速度和三轴角速度。数据结构：

```c
typedef struct {
    IMU_BASE_t Deviation_accel;
    IMU_BASE_t Deviation_gyro;
    IMU_BASE_t Original_accel;
    IMU_BASE_t Original_gyro;
    IMU_BASE_t gyro;
    IMU_BASE_t accel;
} IMU_DATA_t;
```

含义：

- `Original_*`：传感器原始读数；
- `Deviation_*`：启动时估计的零偏；
- `gyro/accel`：减去零偏后的读数。

### 18.1 当前校准实现的问题

代码条件是：

```c
if (SysVal.Time_count < CONTROL_DELAY)
{
    ImuData_copy(&imu.Deviation_gyro, &imu.gyro);
    ImuData_copy(&imu.Deviation_accel, &imu.accel);
}
```

但 `SysVal.Time_count` 的递增位于旧 `Balance_task`，而当前工程没有创建该任务。因此它可能一直小于 `CONTROL_DELAY`，偏差值会持续被改写。

另外，这里是“把最近一次值当偏差”，不是对数百或数千个静止样本求平均。严格的 IMU 启动校准应：

1. 使用 IMU 任务自己的样本计数或时间戳；
2. 要求小车静止；
3. 累加例如 1000 个样本；
4. 陀螺仪三轴偏差取平均；
5. 加速度计 X/Y 取平均，Z 轴需要正确处理 1 g；
6. 校准结束后固定偏差，不再每周期覆盖；
7. 对原始值做单位换算和滤波；
8. 定义坐标系方向；
9. 增加协议帧上传 RDK；
10. 在 RDK 发布 `sensor_msgs/msg/Imu`，再考虑 EKF 融合。

### 18.2 当前 IMU 不参与电机 PI

电机 PI 的反馈变量明确来自编码器：

```text
left_encoder_delta  → left_measured_speed
right_encoder_delta → right_measured_speed
```

IMU 不适合替代轮速编码器做单轮速度闭环，因为它测的是车体惯性量，无法直接告诉你每个轮子转多快。

IMU 更适合：

- 与轮式里程计融合，改善航向估计；
- 检测剧烈碰撞或异常倾斜；
- 平衡车姿态控制；
- 为导航提供角速度和线加速度。

## 19. 编码器里程计与 IMU 融合的区别

编码器里程计的优点：短时间平滑、能直接得到轮子走过的距离。缺点：打滑时它以为车走了，实际可能没动。

IMU 陀螺仪的优点：短时间旋转响应快，不依赖轮胎抓地。缺点：积分会漂移。

二者互补：

```text
编码器：长期距离约束
陀螺仪：短期转动变化
```

常用融合方法包括互补滤波、扩展卡尔曼滤波。ROS 里通常可由 `robot_localization` 接收 `/odom` 和 `/imu/data`，输出融合后的 `/odometry/filtered`。但必须先保证时间戳、单位、坐标系、协方差和静态标定正确。

## 20. 推荐阅读代码顺序

不要从底层库随意跳读，建议按控制链：

1. `USER/main.c`：任务从哪里创建；
2. `BALANCE/system.c`：硬件初始化；
3. `CHASSIS_SERIAL/src/chassis_tasks.c`：周期和任务分工；
4. `CHASSIS_SERIAL/src/c30d_chassis_port.c`：算法如何接硬件；
5. `CHASSIS_SERIAL/src/chassis_app.c`：运动学、状态机和安全；
6. `CHASSIS_SERIAL/src/wheel_controller.c`：单轮前馈 + PI；
7. `CHASSIS_SERIAL/src/chassis_serial_transport.c`：15 字节串口；
8. `CHASSIS_SERIAL/src/chassis_can_protocol.c`：8 字节协议编码；
9. `CHASSIS_SERIAL/src/chassis_params.c`：默认值、范围和 Flash；
10. `HARDWARE/encoder.c`：定时器编码器；
11. `HARDWARE/motor.c` 与 `BALANCE/balance.c:Set_Pwm()`：最终 PWM；
12. `BALANCE/imu_task.c`：IMU 当前采集和缺口。

## 21. 用调试数据理解闭环

RDK 上执行：

```bash
ros2 topic echo /motor/debug
```

数组顺序：

```text
[左目标, 左实测, 右目标, 右实测, 左PWM, 右PWM, 左误差, 右误差]
```

观察方法：

- 目标变化后实测是否快速跟随；
- 稳态误差是否接近零；
- PWM 是否长期顶到上限；
- 左右轮同目标时差异是否明显；
- 目标归零后 PWM 是否归零；
- 电机堵住时是否在阈值时间后进入故障。

调 PI 时必须车轮架空、一次只小幅修改一个参数、记录前后曲线。不要靠听声音同时大改 Kp、Ki、Kff 和死区。

## 22. 常见理解误区

### 误区一：PWM 就是速度

PWM 是控制输入，不是速度测量。同一个 PWM 在不同电压、负载、地面上得到的速度不同，所以需要编码器闭环。

### 误区二：有编码器就自动形成闭环

只有“读取编码器 → 计算误差 → 调整 PWM”形成周期反馈，才叫闭环。

### 误区三：PI 参数越大响应越好

参数过大会振荡、啸叫、电流冲击或放大编码器噪声。

### 误区四：IMU 能直接给出位置

加速度二次积分才能得到位置，偏置和噪声会迅速累积。小车定位通常需要编码器、IMU、激光/视觉等多传感器融合。

### 误区五：读取到 IMU 原始数值就等于姿态可用

还需要校准、单位换算、坐标系、滤波、姿态算法、时间戳和验证。

### 误区六：当前文件叫 CAN 协议，所以串口版走 CAN

当前只是复用 logical ID + 8 字节逻辑层，外层实际由 USART3 发送 15 字节串口帧。

## 23. 当前实现边界总结

已经实现并接入实车：

- 双电机编码器速度反馈；
- 100 Hz 前馈 + PI 速度闭环；
- 目标加减速斜坡；
- 速度低通；
- 输出限幅和积分抗饱和；
- 差速/阿克曼运动学；
- 舵机目标换算；
- 直行同步；
- 命令超时、急停、欠压、编码器、堵转保护；
- 参数范围检查、应答和 Flash 保存；
- 串口命令和遥测；
- 编码器里程计数据上传。

存在代码但尚未完成当前项目闭环：

- MPU6050/ICM20948 原始采集；
- IMU 启动校准需要重写为独立计时和样本平均；
- 没有 IMU 协议帧；
- 没有 ROS `/imu/data`；
- 没有编码器 + IMU 融合定位；
- ICM20948 磁力计当前关闭。

所以当前项目可以准确描述为“STM32 双电机编码器闭环与 ROS 串口底盘控制”，不能描述为“已经完成 IMU 融合导航”。后者可以作为下一阶段明确、很有技术含量的扩展目标。

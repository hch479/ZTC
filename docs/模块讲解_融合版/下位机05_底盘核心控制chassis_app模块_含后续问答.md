# 下位机第 5 模块：底盘核心控制 chassis_app 模块

> 本文件是该模块在旧任务中的完整归档：保留首次讲解正文，并把首次文档之后的专项追问统一融合在同一文件中。以后无需回到原任务查找。

## 本模块归档内容

- 首次讲解来源：`docs/17_STM32底盘核心控制模块chassis_app深入讲解.md`
- 后续追问来源：`docs/18_底盘控制五个核心疑问_从物理原理到代码计算.md`

---

# 第一部分：首次完整讲解

# STM32 底盘核心控制模块 `chassis_app` 深入讲解

## 1. 本文范围

本文专门讲解下位机第 5 个模块——底盘核心控制模块：

```text
CHASSIS_SERIAL/inc/chassis_app.h
CHASSIS_SERIAL/src/chassis_app.c
```

对应完整工程位置：

```text
keil_project/R550_C30D_SERIAL_MOTOR/
```

你已经理解了前四个模块：

1. 系统启动与硬件初始化；
2. FreeRTOS 任务调度；
3. 串口传输；
4. 上下位机逻辑协议。

所以本文从“协议已经得到一帧合法逻辑消息、控制任务已经得到编码器增量”开始，重点回答：

- `chassis_app` 在整个系统里处于什么位置；
- `chassis_app_t` 为什么要保存这么多变量；
- 上位机命令怎样进入控制状态；
- 每 10 ms 的控制步骤到底做了什么；
- 线速度和角速度怎样变成左右轮目标与舵机目标；
- 什么时候允许输出 PWM，什么时候必须输出零；
- 编码器尖峰、欠压、超时、堵转怎样检测；
- 它如何调用 `wheel_controller_update()`；
- 里程计怎样积分；
- 状态和遥测怎样交给通信模块；
- 它与任务、协议、参数、硬件接口和 RDK 上位机怎样连接。

本文数值例子采用源码中的安全默认参数，例如轮径 `0.065 m`、轮距 `0.162 m`、编码器一圈 `60000` 计数。实际运行时，`chassis_app_init()` 会优先加载 STM32 Flash 中通过校验的参数；如果你曾经从 RDK 下发并保存过新参数，实车数值可能与默认值不同。算法和调用关系不受这个区别影响。

## 2. 一句话理解这个模块

`chassis_app` 是底盘的“控制决策中枢”。

它的输入是：

```text
上位机运动/参数/系统命令
当前时间
控制周期 dt
左右编码器增量
电池电压
硬件使能状态
```

它的内部处理是：

```text
协议命令状态保存
参数管理
安全状态机
底盘运动学
直行同步
调用左右轮速度控制器
堵转/编码器/欠压检测
里程计计算
遥测数据组织
```

它的输出是：

```text
左电机 PWM
右电机 PWM
舵机 PWM
底盘状态和故障
轮速、编码器、PWM、心跳等遥测帧
保存参数等硬件动作请求
```

它本身不直接读取 `TIM2->CNT`，也不直接写 `TIMx->CCR`。读取编码器和写 PWM 由外部接口模块完成。这种设计使“控制策略”和“具体硬件寄存器”尽量分开。

## 3. 它在整个系统中的位置

### 3.1 控制方向

```text
RDK /cmd_vel
    ↓
Python 编码运动帧
    ↓
STM32 串口传输模块组帧
    ↓
协议模块解码
    ↓
chassis_app_process_can_frame()
    ↓ 保存最新命令、使能、急停和接收时间
chassis_app_control_step() 每 10 ms 读取这些状态
    ↓
运动学 + 安全状态机 + 单轮控制器
    ↓
left_pwm / right_pwm / servo_pwm
    ↓
chassis_tasks.c 复制输出
    ↓
c30d_chassis_port.c
    ↓
Set_Pwm()
    ↓
电机驱动硬件
```

### 3.2 反馈方向

```text
编码器定时器
    ↓ Read_Encoder()
chassis_control_task 获得左右增量
    ↓
chassis_app_control_step()
    ├─换算实测轮速
    ├─反馈给 PI
    ├─累计里程
    └─更新故障和状态
    ↓
chassis_app_make_*_frame()
    ↓
串口发送到 RDK
    ↓
/motor/debug、/odom、/diagnostics
```

闭环之所以成立，是因为 PWM 影响电机转速，电机转速又由编码器反馈给下一周期的控制计算。

## 4. 为什么单独设计 `chassis_app` 层

如果把全部逻辑都写进 FreeRTOS 任务，会出现：

- 控制算法与任务延时混在一起；
- 寄存器操作与状态机混在一起；
- 很难在没有实车时做单元测试；
- 更换串口为 CAN 时需要修改控制算法；
- 代码中到处出现左右方向负号；
- 故障判断、参数和遥测难以统一。

现在分层后：

```text
chassis_tasks.c          决定什么时候调用、互斥锁怎样使用
c30d_chassis_port.c      决定怎样读取真实硬件、怎样输出 PWM
chassis_can_protocol.c   决定字节怎样编码与校验
wheel_controller.c       决定一个车轮怎样由速度误差算 PWM
chassis_app.c            决定整车当前应该做什么
```

`chassis_app.c` 仍然知道逻辑消息 ID，并负责生成遥测帧，因此它不是完全与协议无关的纯算法库；更准确地说，它是“底盘应用层”。但它已经避免直接依赖 STM32 定时器、USART 寄存器和具体电机引脚。

## 5. 头文件中的两个核心定义

### 5.1 `chassis_action_t`

```c
typedef enum
{
    CHASSIS_ACTION_NONE = 0,
    CHASSIS_ACTION_SAVE_PARAMETERS = 1
} chassis_action_t;
```

为什么保存 Flash 不直接在 `chassis_app.c` 里完成？

因为 Flash 擦写是具体硬件行为，而核心控制模块不应该知道使用哪块 Flash、哪个扇区、怎样进入临界区。于是它只返回一个“动作请求”：

```text
CHASSIS_ACTION_NONE             不需要外部硬件动作
CHASSIS_ACTION_SAVE_PARAMETERS  请接口层保存参数
```

`chassis_serial_task` 收到该返回值后，调用：

```c
c30d_port_save_parameters(&params_to_save);
```

这叫控制反转或命令返回：核心层决定“要保存”，外层决定“怎样保存”。

### 5.2 `chassis_app_t`

这个结构体保存整个底盘核心模块的运行状态。它不是一组随意堆放的变量，可以分成九类。

## 6. `chassis_app_t` 逐组解释

### 6.1 参数和左右轮控制器

```c
chassis_params_t params;
wheel_controller_t left_controller;
wheel_controller_t right_controller;
```

`params` 包含：

- 左右轮 `kp/ki/kff`；
- 正反转死区；
- PWM 上限；
- 轮径、轮距、轴距；
- 编码器一圈计数；
- 最高速度、加减速度；
- 滤波系数；
- 舵机标定；
- 命令超时；
- 欠压和堵转阈值；
- 底盘类型。

`left_controller` 和 `right_controller` 分别保存一个轮子的动态历史量：

```text
ramped_target_mps   斜坡后的目标速度
filtered_speed_mps  滤波后的实测速度
integral_output     PI 的积分历史
last_error_mps      最近一次速度误差
output_pwm          最近一次 PWM
```

为什么这些不能只是函数内局部变量？因为 PI 积分、低通滤波和目标斜坡都依赖上一周期的状态。函数每 10 ms 被重新调用，但这些变量必须跨周期保留。

### 6.2 最新上位机运动命令

```c
chassis_motion_command_t command;
uint32_t last_command_ms;
uint8_t has_received_command;
```

`command` 保存：

```text
linear_mps       车体中心线速度
angular_radps    车体角速度
enable           软件使能位
emergency_stop   急停位
sequence         命令序号
```

`last_command_ms` 保存合法运动帧到达时刻，用于命令超时。

`has_received_command` 区分：

```text
上电后从未收到命令
曾经收到命令，但现在可能超时
```

如果没有这个标志，`last_command_ms` 初始为 0，系统启动超过 200 ms 后就可能误报命令超时。现在从未收到命令时保持 `IDLE`，不会把正常等待上位机当成通信故障。

### 6.3 故障时间和运行诊断

```c
uint32_t left_stall_ms;
uint32_t right_stall_ms;
uint32_t control_overrun_count;
```

- `left_stall_ms/right_stall_ms`：左右轮满足堵转条件的累计时间；
- `control_overrun_count`：控制周期不在合理范围内的累计次数。

堵转判断不能只看某一个周期，因为车轮启动时短时间“有 PWM、速度仍很小”很正常。必须连续满足一段时间才判故障。

### 6.4 编码器累计量

```c
int32_t left_total_count;
int32_t right_total_count;
int16_t left_delta_count;
int16_t right_delta_count;
float left_distance_m;
float right_distance_m;
```

两种计数必须区分：

```text
delta_count  本次 10 ms 周期内新增加多少计数，用于速度
total_count  从复位开始累计多少计数，用于里程计和上传
```

`left_distance_m/right_distance_m` 是累计物理距离，主要用于直行同步修正。

### 6.5 STM32 内部里程计

```c
float x_m;
float y_m;
float yaw_rad;
float linear_speed_mps;
float angular_speed_radps;
```

它们表示二维平面中的底盘估计状态：

```text
x_m/y_m               相对起点位置
yaw_rad               航向角
linear_speed_mps       车体中心线速度
angular_speed_radps    车体角速度
```

目前这些 STM32 内部 `x/y/yaw` 没有通过现有帧直接上传；RDK 会根据累计编码器计数再独立计算一份 `/odom`。保留下位机里程计有利于以后增加本地诊断或新的遥测帧。

### 6.6 目标、原始速度和直行同步

```c
float left_target_mps;
float right_target_mps;
float left_raw_speed_mps;
float right_raw_speed_mps;
float straight_left_start_m;
float straight_right_start_m;
```

- `left/right_target_mps`：经过整车运动学和直行修正后的轮速目标；
- `left/right_raw_speed_mps`：由本周期编码器直接计算、尚未低通的速度；
- `straight_*_start_m`：本次开始直行时的左右累计距离基准。

### 6.7 电源、故障和最终输出

```c
uint16_t battery_mv;
uint16_t latched_faults;
uint16_t dynamic_faults;
uint16_t servo_pwm;
int16_t left_pwm;
int16_t right_pwm;
```

故障分两类：

```text
latched_faults  锁存故障，需要明确清故障
dynamic_faults  动态故障，条件消失后可以自动消失
```

最终 PWM 存在结构体中，任务拿到互斥锁后复制出来，再交给硬件接口。这样串口遥测任务也可以安全读取同一份输出。

### 6.8 状态与布尔标志

```c
uint8_t state;
uint8_t has_received_command;
uint8_t straight_sync_active;
uint8_t left_encoder_spike_count;
uint8_t right_encoder_spike_count;
```

`state` 是当前底盘状态机结果。`straight_sync_active` 表示当前是否已经建立直行距离基准。编码器尖峰计数器要求连续三次超物理速度才锁存故障。

### 6.9 遥测序号

```c
uint8_t telemetry_sequence;
```

左右编码器帧共用同一个 sequence。RDK 只有收到相同 sequence 的左右帧，才把它们当成同一批数据计算里程计，防止左轮是上一批、右轮是下一批。

## 7. 文件顶部的辅助函数

这些函数是 `static`，只供 `chassis_app.c` 内部使用。

### 7.1 `clamp_float()`：限幅

```c
static float clamp_float(float value, float minimum, float maximum)
```

含义：

```text
value > maximum → 返回 maximum
value < minimum → 返回 minimum
否则返回 value
```

控制系统中限幅非常重要：

- 上位机即使发送异常大速度，也不能超过底盘限制；
- 舵角不能超过机械范围；
- 直行修正不能无限增大；
- 左右轮目标不能超过允许速度。

### 7.2 `clamp_servo_pwm()`：舵机安全范围

```c
static uint16_t clamp_servo_pwm(float value,
                                const chassis_params_t *params)
```

它先限制在 `servo_min_pwm` 和 `servo_max_pwm` 之间，再加 `0.5f` 转为整数，相当于对正数四舍五入。

假设：

```text
servo_min_pwm = 1000
servo_max_pwm = 2000
计算值 = 2175
```

最终只能得到 2000，避免舵机撞机械限位。

### 7.3 `count_to_distance_m()`：计数换距离

```c
perimeter = π × wheel_diameter_m;
distance = count × perimeter / counts_per_wheel_rev;
```

当前默认：

```text
轮径 D = 0.065 m
一圈计数 N = 60000
```

一计数对应：

```text
π × 0.065 / 60000 ≈ 3.403×10⁻⁶ m
```

正计数表示向前距离，负计数表示向后距离。方向统一已由 `c30d_chassis_port.c` 的编码器符号完成。

### 7.4 `delta_to_speed_mps()`：距离除以时间

```c
speed = count_to_distance_m(delta) / dt_s;
```

例如 10 ms 内 147 个计数：

```text
distance ≈ 147 × 3.403 μm ≈ 0.000500 m
speed ≈ 0.000500 / 0.010 = 0.050 m/s
```

如果 `dt_s <= 0`，返回 0，避免除零。

### 7.5 `wrap_angle()`：角度归一化

它把角度限制到：

```text
[-π, π]
```

例如：

```text
3.20 rad → 3.20 - 2π ≈ -3.083 rad
```

物理方向相同，但数值不会无限增长，方便比较和发送。

### 7.6 `set_reply()`：统一生成应答

它调用协议层：

```c
chassis_encode_parameter_reply(...)
```

并把：

```c
*reply_ready = 1U;
```

这表示外层任务应把 `reply` 发送出去。它同时检查两个指针不为空，避免空指针访问。

## 8. 初始化函数 `chassis_app_init()`

函数原型：

```c
void chassis_app_init(chassis_app_t *app,
                      const chassis_params_t *stored_params)
```

### 8.1 第一步：检查对象地址

```c
if (app == 0)
{
    return;
}
```

C 中 `0` 在这里表示空指针。如果调用者没有提供有效结构体，函数直接返回。

### 8.2 第二步：全部清零

```c
memset(app, 0, sizeof(*app));
```

清零后：

- 还没收到命令；
- 编码器累计为 0；
- PWM 为 0；
- 故障为 0；
- PI 历史为 0；
- 时间和里程计为 0。

显式清零比依赖未初始化内存安全得多。

### 8.3 第三步：选择 Flash 参数或默认参数

只有同时满足：

```text
stored_params 不是空指针
参数结构合法
CRC 正确
```

才复制 Flash 参数。

否则加载安全默认参数。如果传入了非空参数但验证失败，还会锁存：

```text
CHASSIS_FAULT_BAD_PARAMETER
```

这里区分两种情况：

```text
Flash 从未保存新格式参数 → 外层传 NULL → 用默认值，不报错
Flash 声称有新格式但内容损坏 → 传非空无效结构 → 默认值 + BAD_PARAMETER
```

### 8.4 第四步：复位左右轮控制器

```c
wheel_controller_reset(&app->left_controller);
wheel_controller_reset(&app->right_controller);
```

清空积分、滤波、斜坡和 PWM 历史。

### 8.5 第五步：进入安全初始状态

```c
app->state = CHASSIS_STATE_IDLE;
app->servo_pwm = clamp_servo_pwm(servo_center_pwm, ...);
```

上电后不会直接进入 `RUNNING`。阿克曼舵机目标先处于中心位置。

## 9. 清故障 `chassis_app_clear_faults()`

它清除：

```text
所有锁存故障
左右堵转累计时间
左右编码器连续尖峰计数
左右轮控制器历史
```

它没有直接把状态写成 `IDLE`。原因是状态应在下一次 `control_step()` 根据真实条件重新计算。例如硬件仍未使能、命令仍超时，它就不应该被强行标成正常运行。

它也不会清除 `dynamic_faults`，因为动态故障由下一周期根据当前硬件条件重新计算。

## 10. 复位里程计 `chassis_app_reset_odometry()`

它清除：

```text
左右累计计数和本周期增量
左右累计距离
x、y、yaw
直行同步状态
```

它不清除：

```text
PI 参数
最新运动命令
故障状态
电池电压
控制器滤波速度
```

系统命令处理会保证运行中拒绝复位，避免运动过程中里程基准突然跳变。

## 11. 命令处理主函数 `chassis_app_process_can_frame()`

函数原型：

```c
chassis_action_t chassis_app_process_can_frame(
    chassis_app_t *app,
    const chassis_can_frame_t *frame,
    uint32_t now_ms,
    chassis_can_frame_t *reply,
    uint8_t *reply_ready);
```

输入和输出：

| 参数 | 含义 |
|---|---|
| `app` | 整个底盘状态 |
| `frame` | 已由串口外层恢复出的逻辑帧 |
| `now_ms` | 收到并处理该帧的当前时间 |
| `reply` | 如果需要应答，写到这里 |
| `reply_ready` | 是否应该发送应答 |
| 返回值 | 是否需要外层执行保存 Flash 等动作 |

这个函数不是每 10 ms 固定执行，而是串口任务每取出一帧就调用一次。

### 11.1 为什么先把 `reply_ready` 清零

```c
if (reply_ready != 0)
{
    *reply_ready = 0U;
}
```

防止调用者上一次留下的 `1` 被误认为本次也有新应答。

### 11.2 运动命令 `0x101`

流程：

```c
if (chassis_decode_motion_command(frame, &motion))
{
    app->command = motion;
    app->last_command_ms = now_ms;
    app->has_received_command = 1U;
}
```

协议解码会检查：

- logical ID 是否为运动命令；
- 内层 CRC 是否正确；
- 协议版本是否为 1；
- 线速度和角速度字段怎样换算。

只有合法帧才覆盖最新命令和刷新时间。坏帧不会延长命令有效期，这是安全关键点。否则噪声帧可能让旧速度一直不过期。

如果急停位为 1：

```c
app->latched_faults |= CHASSIS_FAULT_ESTOP;
```

`|=` 是按位或赋值。它只把 ESTOP 对应位设置为 1，不影响其他故障位。

为什么急停要锁存？即使后续普通运动命令中急停位恢复 0，也不会自动恢复运动。操作者必须排除问题并发清故障命令。

运动命令不需要逐帧应答，因为 RDK 以 50 Hz 连续发送。逐帧应答会增加通信负担；状态帧和 `last_command_sequence` 已能用于诊断。

### 11.3 参数读取 `0x102 + READ`

```text
解码参数 ID、操作、float 值和 sequence
根据 ID 从 app->params 读取实际值
编码参数应答
返回相同 sequence
```

若 ID 不存在，响应 `CHASSIS_REPLY_BAD_PARAMETER`。

### 11.4 参数写入 `0x102 + WRITE`

先判断：

```c
if (app->state == CHASSIS_STATE_RUNNING)
```

运行中拒绝改控制参数。否则调用：

```c
chassis_params_set_value(...)
```

它会：

- 按参数 ID 选择字段；
- 先在候选副本中修改；
- 验证范围和结构整体一致性；
- 合法才提交。

参数成功后同时复位左右轮控制器：

```c
wheel_controller_reset(...);
```

为什么？假设 `Ki` 改小，但仍保留旧控制器积累的大积分，重新使能时会产生与新参数不匹配的冲击。清空历史量可让新参数从已知状态开始。

应答返回的是参数系统实际保存的值，而不只是把请求原封不动返回。这样上位机可以确认类型转换和范围检查后的真实结果。

### 11.5 清故障系统命令 `0x103`

系统命令必须经过协议层的 CRC 和 `A5 5A` 钥匙检查。

核心层还要求：

```text
command.enable == 0
command.emergency_stop == 0
```

为什么清故障前必须先发“普通失能命令”？

假如保留的最新命令仍然是：

```text
enable=1，速度=0.4 m/s
```

如果清故障立即恢复，车可能突然继续运动。RDK 的清故障服务会先发送一次失能且非急停的运动帧，再发送清故障系统命令。

### 11.6 复位里程计系统命令

只有：

```text
state != RUNNING
```

才允许执行。否则返回 `DENIED`。

### 11.7 保存参数系统命令

同样要求不在 `RUNNING`。满足时：

1. 先准备成功应答；
2. 返回 `CHASSIS_ACTION_SAVE_PARAMETERS`；
3. 外层复制参数并执行 Flash 写入；
4. 如果写入失败，外层把应答改为 `STORAGE_ERROR`。

`chassis_app` 不直接写 Flash，这正是应用层与硬件层的边界。

## 12. 两个故障辅助函数

### 12.1 `update_encoder_fault()`

判断条件：

```c
fabsf(measured_speed) > physical_limit
```

`fabsf()` 取浮点绝对值，因此正向和反向异常都能检测。

默认物理上限为 `2.0 m/s`，明显高于正常最高命令 `0.6 m/s`。它不是正常速度限幅，而是判断编码器单周期计数是否不可信。

逻辑：

```text
本周期超限 → counter + 1，最大不超过 255
本周期正常 → counter = 0
连续达到 3 次 → 锁存编码器故障
```

为什么是“连续三次”而不是“累计三次”？正常一次就清零，所以必须连续异常约 30 ms。这能过滤孤立干扰，同时仍较快停机。

### 12.2 `update_stall_fault()`

堵转必须同时满足三个方面：

#### 条件 A：确实要求它转

```text
|target_speed| ≥ stall_target_speed_mps
```

默认 `0.15 m/s`。极低速时编码器量化明显，不适合判断堵转。

#### 条件 B：实际没有正常跟随

```text
|measured_speed| ≤ stall_measured_speed_mps
```

或者：

```text
target_speed × measured_speed < -0.002
```

乘积为负表示目标和实测方向相反；再用 `-0.002` 而不是简单 `<0`，可以避免零点附近微小噪声被当成反转。

#### 条件 C：控制器已经很努力

```text
|PWM| ≥ stall_pwm_ratio × pwm_limit
```

默认比例 0.80、上限 16700：

```text
阈值 = 0.8 × 16700 = 13360
```

只有 PWM 很高但轮子仍不动，才像真正堵转。如果 PWM 很低，可能只是斜坡刚开始或控制器没要求大输出。

#### 时间累计

三个条件同时成立才：

```c
*elapsed_ms += step_ms;
```

任一条件不成立立即清零。达到默认 `600 ms` 才锁存堵转。

## 13. 控制主函数 `chassis_app_control_step()`

这是整个模块最重要的函数。它由 `chassis_control_task` 每 10 ms 调用一次。

函数输入：

```c
chassis_app_t *app
uint32_t now_ms
float dt_s
int16_t left_encoder_delta
int16_t right_encoder_delta
uint16_t battery_mv
uint8_t hardware_enable
```

注意：它不自己读硬件。任务先通过硬件接口取得这些值，再传进来。

下面按照真实执行顺序讲解。

## 14. 控制步骤 1：验证真实控制周期

正常周期为：

```text
dt_s = 0.010 s
```

代码接受范围：

```text
0.005 s ≤ dt_s ≤ 0.020 s
```

同时检查：

```c
dt_s != dt_s
```

这是检测 NaN。IEEE 浮点数中 NaN 是唯一不等于自己的值。

如果周期异常：

```text
control_overrun_count + 1
设置动态 CONTROL_OVERRUN
本次计算强制使用 dt = 0.010 s
```

为什么不能把异常大的真实 `dt` 直接带入 PI 和里程计？

- 积分项是 `Ki × error × dt`，dt 很大会让积分突然跳变；
- 里程计角度增量是 `angular × dt`，会出现大跳；
- 堵转时间也会被一次异常周期快速累加。

`CONTROL_OVERRUN` 当前属于诊断型动态故障，不会单独强制进入 `FAULT`；控制仍使用替代周期继续。这是“记录调度异常，但不因单次抖动立即停机”的策略。

随后：

```c
step_ms = dt_s × 1000 + 0.5;
```

把秒换成整数毫秒，供堵转计时使用。

## 15. 控制步骤 2：编码器增量换算原始速度

```c
left_raw_speed_mps = delta_to_speed_mps(...);
right_raw_speed_mps = delta_to_speed_mps(...);
```

这是本周期原始测量，还没有进入单轮控制器的一阶低通滤波。

同时保存：

```c
app->battery_mv = battery_mv;
```

供安全判断和状态遥测使用。

## 16. 控制步骤 3：拒绝不可信编码器尖峰

如果原始速度未超过物理上限：

```text
保存本周期 delta
累计 total_count
累计实际距离
把原始速度交给 PI
```

如果超过物理上限：

```text
本周期有效 delta 记为 0
不累计总计数
不累计距离
PI 暂时使用上一周期滤波速度
```

这里有两层保护：

1. 立即保护：一个尖峰不进入里程计，不驱动 PI 反向打大 PWM；
2. 持续保护：连续三次尖峰锁存编码器故障。

### 16.1 为什么累计计数使用无符号环绕写法

```c
app->left_total_count =
    (int32_t)((uint32_t)app->left_total_count +
              (uint32_t)(int32_t)left_encoder_delta);
```

C 语言中有符号整数溢出属于未定义行为。先转换为 `uint32_t` 做明确的模 2³² 加法，再转回 `int32_t`，可得到定义明确的环绕行为。

RDK 的 `wrapped_count_delta()` 也按 32 位环绕计算，所以两端配套。

## 17. 控制步骤 4：更新故障集合

### 17.1 动态故障先重算

```c
app->dynamic_faults &= CHASSIS_FAULT_CONTROL_OVERRUN;
```

含义是：保留本周期刚判断出的 `CONTROL_OVERRUN`，清除其他需要重新计算的动态位。然后按当前条件重新设置。

### 17.2 硬件失能

```c
if (hardware_enable == 0U)
{
    dynamic_faults |= HARDWARE_DISABLED;
}
```

硬件使能通常来自实体开关或使能引脚。软件使能不能绕过硬件失能。

它会让底盘进入 `IDLE` 且禁止输出，但作为动态故障，硬件重新使能后可以自动消失。

### 17.3 欠压

```c
if ((battery_mv != 0U) &&
    (battery_mv < low_battery_mv))
```

电压为 0 时不立即判断欠压，因为 0 也可能表示 ADC 数据尚未有效。非零且低于默认 9800 mV 才锁存 `LOW_BATTERY`。

### 17.4 命令超时

```c
elapsed_since_command = now_ms - last_command_ms;
```

`uint32_t` 无符号减法能在系统毫秒计数回绕时保持短时间差正确，这是嵌入式中常见写法。

只有已经收到过命令且间隔超过阈值，才设置 `COMMAND_TIMEOUT`。

## 18. 控制步骤 5：安全状态机

### 18.1 严重故障集合

```text
LOW_BATTERY
LEFT_ENCODER
RIGHT_ENCODER
LEFT_STALL
RIGHT_STALL
BAD_PARAMETER
```

急停单独具有更高优先级。

### 18.2 状态判断顺序

实际优先级：

```text
1. ESTOP 锁存存在       → ESTOP
2. 严重锁存故障存在     → FAULT
3. 命令超时             → TIMEOUT
4. 硬件失能             → IDLE
5. 从未收到命令         → IDLE
6. 软件 enable 为 0     → IDLE
7. 以上都没有           → RUNNING，允许输出
```

用伪代码表示：

```text
output_allowed = false

if 急停:
    state = ESTOP
else if 严重故障:
    state = FAULT
else if 超时:
    state = TIMEOUT
else if 硬件未使能 or 没命令 or 软件未使能:
    state = IDLE
else:
    state = RUNNING
    output_allowed = true
```

最关键的不变量是：

```text
只有 state == RUNNING 时，output_allowed 才可能为 1。
```

后续即使运动学仍计算变量，只要 `output_allowed=0`，左右轮控制器就会清状态并返回零 PWM。

### 18.3 为什么 `HARDWARE_DISABLED` 的诊断仍可能显示 `IDLE`

硬件失能位不属于 severe_faults，所以状态是 `IDLE`，但故障位中仍包含 `HARDWARE_DISABLED`。这表达的是：系统没有严重锁存故障，只是物理使能条件当前不满足。

## 19. 控制步骤 6：限制上位机命令

不管上位机发多大数值，先限制：

```text
linear ∈ [-maximum_linear_speed, +maximum_linear_speed]
angular ∈ [-maximum_angular_speed, +maximum_angular_speed]
```

当前默认：

```text
最大线速度 0.60 m/s
最大角速度 1.50 rad/s
```

这叫输入饱和或命令限幅。上位机限制不能替代下位机限制，因为通信错误或上位机程序错误不能被绝对排除。

## 20. 控制步骤 7：阿克曼运动学

当前默认底盘类型：

```text
CHASSIS_TYPE_ACKERMANN = 1
```

### 20.1 阿克曼车为什么不能原地旋转

差速车可让左轮后退、右轮前进原地转向。阿克曼结构靠前轮舵角转向，必须有一定前后速度才能沿圆弧运动。

所以当：

```text
|linear_command| < 0.02 m/s
```

代码把线速度和角速度都置零。

### 20.2 从角速度换算舵角

车辆理想单轨模型：

```text
w = v × tan(δ) / wheelbase
```

反解舵角：

```text
δ = atan(wheelbase × w / v)
```

其中：

```text
δ          舵角，rad
v          车体中心线速度，m/s
w          期望角速度，rad/s
wheelbase  轴距，当前 0.144 m
```

舵角会限制在：

```text
[-maximum_steering_angle, +maximum_steering_angle]
```

当前最大约 `0.52 rad ≈ 29.8°`。

### 20.3 为什么限幅后重新计算角速度

假如期望转弯很急，公式得到 50°，但机械最多只能 30°。如果后面仍用原来的角速度计算左右轮目标，就会假设车辆能达到不可能的曲率。

因此根据限幅后的真实舵角重新计算：

```text
w_actual_command = v × tan(δ_limited) / wheelbase
```

这让驱动轮差速目标与实际舵角保持一致。

### 20.4 数值例子

假设：

```text
v = 0.20 m/s
w = 0.50 rad/s
wheelbase = 0.144 m
```

```text
δ = atan(0.144 × 0.50 / 0.20)
  = atan(0.36)
  ≈ 0.3456 rad
  ≈ 19.8°
```

未超过 0.52 rad，所以不用限幅。

## 21. 控制步骤 8：计算左右轮目标

公式：

```text
v_left  = v - w × wheel_track / 2
v_right = v + w × wheel_track / 2
```

当前轮距 `0.162 m`，继续使用上例：

```text
v_left  = 0.20 - 0.50 × 0.162 / 2
        = 0.1595 m/s

v_right = 0.20 + 0.50 × 0.162 / 2
        = 0.2405 m/s
```

右轮更快，车辆产生正方向角速度。

对于阿克曼车，这里可理解为左右驱动轮沿不同半径运动，需要不同线速度；舵机同时决定转向几何。

## 22. 控制步骤 9：左右轮等比例限速

转弯时某一轮可能超过 `maximum_linear_speed_mps`。代码找出左右目标绝对值最大者：

```text
maximum_wheel_speed = max(|left_target|, |right_target|)
```

若超限：

```text
scale = maximum_allowed / maximum_wheel_speed
left_target  *= scale
right_target *= scale
```

为什么不是只截断超限的那一轮？

如果只截断右轮，左右速度比例改变，车辆曲率也改变。等比例缩小可以保持：

```text
left_target : right_target
```

基本不变，因此保持期望转弯形状，只降低整体速度。

## 23. 控制步骤 10：直行同步修正

左右电机、轮胎直径和摩擦力不可能完全一致。即使两个速度环都接近目标，长距离也可能出现累计偏差。

### 23.1 何时启用

同时满足：

```text
|angular_command| ≤ straight_angular_threshold
|linear_command| ≥ 0.05 m/s
```

即“确实想直行，而且速度不是接近零”。当前角速度阈值默认 `0.03 rad/s`。

### 23.2 第一次进入直行

保存基准：

```text
straight_left_start  = 当前左累计距离
straight_right_start = 当前右累计距离
```

这样比较的是本次直行开始以后各自走了多少，而不是上电以来的全部历史差异。

### 23.3 同步误差

```text
left_progress  = left_distance  - left_start
right_progress = right_distance - right_start
sync_error = left_progress - right_progress
```

如果 `sync_error > 0`，左轮在本次直行中走得更多。

### 23.4 修正方向

```text
sync_correction = clamp(Ksync × sync_error,
                        -sync_max,
                        +sync_max)

left_target  -= sync_correction
right_target += sync_correction
```

左轮走多时降低左目标、提高右目标，使两侧累计距离重新靠拢。

这是一个外层位置同步环，里面仍有各自的速度 PI 环：

```text
直行累计距离同步（慢外环）
            ↓ 修正左右目标速度
左右轮速度 PI（快内环）
            ↓
PWM
```

离开直行条件后，`straight_sync_active=0`，下次重新建立基准。

## 24. 控制步骤 11：调用单轮控制器

左轮调用：

```c
wheel_controller_update(
    &app->left_controller,
    &app->params.left,
    &app->params,
    left_target,
    left_speed_for_control,
    dt_s,
    output_allowed);
```

七个参数分别是：

1. 左轮动态历史状态；
2. 左轮 `kp/ki/kff/死区/PWM上限`；
3. 全车加减速度和滤波等参数；
4. 整车模块计算出的左轮目标；
5. 编码器得到的左轮实测速度；
6. 本周期真实时间；
7. 安全状态机是否允许输出。

右轮完全相同，只换成右侧对象和参数。

### 24.1 深入 `wheel_controller_update()`

它执行：

#### A. 实测速度低通

```text
filtered += alpha × (measured - filtered)
```

#### B. 如果不允许输出

```text
ramped_target = 0
integral = 0
last_error = 0
PWM = 0
返回 0
```

所以 `chassis_app` 的 `output_allowed` 是最终安全闸门。

#### C. 目标速度斜坡

```text
同方向加速 → acceleration × dt
减速或换向 → deceleration × dt
```

#### D. 目标回零时清积分

避免静止仍通电或啸叫。

#### E. 前馈和死区

```text
feedforward = kff × ramped_target + deadzone
```

#### F. PI

```text
error = ramped_target - filtered_speed
P = kp × error
I_candidate = I_old + ki × error × dt
```

#### G. 限幅和积分抗饱和

```text
PWM = clamp(feedforward + P + I, -limit, +limit)
```

输出已饱和且误差还想继续推向饱和时，不接受新的积分。

这个单轮控制器只知道“我这个轮子的目标和实测”，不知道整车是差速还是阿克曼，也不知道急停原因。整车层负责目标和安全，单轮层负责速度闭环。这就是职责分离。

## 25. 控制步骤 12：舵机 PWM

阿克曼模式：

```text
servo_pwm = servo_center_pwm
          + servo_pwm_per_rad × steering_angle
```

继续使用 `δ = 0.3456 rad`：

```text
center = 1500
pwm_per_rad = 636.56

servo_pwm ≈ 1500 + 636.56 × 0.3456
          ≈ 1720
```

然后限制到 `[1000, 2000]`。

如果机械安装方向相反，可以让 `servo_pwm_per_rad` 为负，不需要修改运动学公式。

差速模式在核心结构中把舵机保持中心，但 `chassis_control_task` 在实际输出前会把非阿克曼车型的舵机值改为 0，以保持原硬件行为。

## 26. 控制步骤 13：堵转检测与本周期立即断输出

只有 `output_allowed != 0` 才累计堵转。失能时清零堵转计时，避免下次启动继承旧时间。

堵转可能在本周期调用单轮控制器之后刚达到 600 ms。如果只等待下一个周期的状态机判断，PWM 还会多输出一个周期。

所以代码紧接着检查：

```c
if (LEFT_STALL 或 RIGHT_STALL 已锁存)
{
    left_pwm = 0;
    right_pwm = 0;
    state = FAULT;
    reset 两个轮控制器;
}
```

这叫同周期故障响应。即使只有一侧堵转，也同时停止两侧，避免车辆突然单轮驱动偏转。

## 27. 控制步骤 14：STM32 内部里程计

车体中心速度：

```text
v = (v_left + v_right) / 2
```

车体角速度：

```text
w = (v_right - v_left) / wheel_track
```

这里使用的是单轮控制器低通后的实测速度，不是目标速度。目标速度只代表“希望怎样走”，不能证明实际真的走了。

### 27.1 中点积分

```text
delta_yaw = w × dt
average_yaw = yaw + delta_yaw/2

x += v × cos(average_yaw) × dt
y += v × sin(average_yaw) × dt
yaw = wrap(yaw + delta_yaw)
```

为什么使用周期中间的方向？转弯时，一个周期内方向从旧 yaw 变化到新 yaw；用中间角近似整段运动方向，误差小于始终使用周期起点方向。

### 27.2 里程计不是绝对定位

编码器里程计会受到：

- 轮径参数误差；
- 轮胎打滑；
- 左右轮实际轮径不一致；
- 阿克曼舵角和简化模型误差；
- 地面不平；
- 编码器量化。

因此 `x/y/yaw` 会逐渐漂移。当前模块没有使用 IMU；RDK 的 `/odom` 也由编码器计算。未来可把可靠 IMU 上传后在上位机做融合。

## 28. 遥测生成函数

这些函数不是直接发送串口，只生成逻辑帧。`chassis_serial_task` 再调用传输层发送。

### 28.1 `chassis_app_make_wheel_speed_frame()`

上传：

```text
左轮斜坡目标
左轮滤波实测
右轮斜坡目标
右轮滤波实测
```

RDK 将其与电机输出帧组合为 `/motor/debug`。

为什么上传斜坡目标而不是最初的命令目标？因为 PI 真正追踪的是斜坡后的目标。调试时比较它和实测值才有意义。

### 28.2 `chassis_app_make_motor_output_frame()`

上传：

```text
左右 PWM
左右最近速度误差
```

### 28.3 `chassis_app_make_encoder_frames()`

先：

```c
telemetry_sequence++;
```

再用同一 sequence 生成左右两帧。每帧含累计计数、本周期有效增量和序号。

### 28.4 `chassis_app_make_status_frame()`

上传：

```text
当前状态
latched_faults | dynamic_faults
电池电压
最后运动命令 sequence
控制周期异常计数
```

异常计数上传字段只有 8 位，因此大于 255 时截为 255；内部仍保留 32 位累计值。

### 28.5 `chassis_app_make_heartbeat_frame()`

上传：

```text
STM32 运行毫秒数
当前状态
固件主版本 1
固件次版本 0
```

RDK 每收到心跳就记录本机单调时间。如果超过 2 秒没有新心跳，发布 `STM32 heartbeat missing`。

如果 STM32 `uptime_ms` 变小，RDK 判断 STM32 刚刚重启，清除上位机编码器里程计基准，避免累计计数从零开始造成巨大位置跳变。

## 29. 与 `chassis_tasks.c` 的连接

### 29.1 启动时

```text
从 Flash 读取参数
→ chassis_app_init()
→ 创建互斥锁
→ 创建串口任务
→ 创建控制任务
```

### 29.2 控制任务中

```text
每 10 ms
→ 读取左右编码器（读取后硬件计数清零）
→ 获取 g_chassis_mutex
→ chassis_app_control_step()
→ 复制 left_pwm/right_pwm/servo_pwm/chassis_type
→ 释放互斥锁
→ c30d_port_set_output()
```

为什么复制后释放锁再写硬件？共享状态已经得到一致快照，耗时的外部硬件调用不必一直占用互斥锁，串口任务可以更快读取状态。

### 29.3 串口任务中

```text
取一帧
→ 获取互斥锁
→ chassis_app_process_can_frame()
→ 如需保存则复制 params
→ 释放锁
→ 外层执行 Flash 保存或发送应答
```

遥测生成也在互斥锁内，确保不会读到“状态刚更新一半”的组合。

## 30. 与参数模块的连接

`chassis_app` 不负责每个参数的范围细节，而是调用：

```text
chassis_params_set_defaults()
chassis_params_validate()
chassis_params_crc_is_valid()
chassis_params_get_value()
chassis_params_set_value()
```

控制模块只决定：

- 什么时候可修改；
- 修改后是否复位控制器；
- 什么时候可保存；
- 怎样回应上位机。

参数模块决定：

- ID 对应哪个字段；
- 数值允许范围；
- 结构版本、大小和 CRC；
- 默认值。

## 31. 与硬件接口模块的连接

`chassis_app_control_step()` 需要的输入来自：

```text
c30d_port_get_time_ms()
c30d_port_read_left_encoder()
c30d_port_read_right_encoder()
c30d_port_read_battery_mv()
c30d_port_hardware_is_enabled()
```

计算结果最终交给：

```text
c30d_port_set_output()
```

Flash 动作交给：

```text
c30d_port_load_parameters()
c30d_port_save_parameters()
```

因此以后如果更换电机通道、编码器定时器或硬件方向，优先修改 `c30d_chassis_port`，不要污染 `chassis_app` 的运动学和状态机。

## 32. 与 RDK 上位机的连接

| `chassis_app` 内容 | RDK 对应行为 |
|---|---|
| `command.linear_mps/angular_radps` | 来自 `/cmd_vel` |
| `command.enable` | 来自 `/chassis/enable` 和命令新鲜度 |
| `command.emergency_stop` | 来自 `/chassis/emergency_stop` |
| `last_command_ms` | STM32 独立超时保护 |
| 参数读写 | ROS 参数与 `/chassis/push_parameters` |
| 清故障 | `/chassis/clear_faults` |
| 复位里程计 | `/chassis/reset_odometry` |
| 保存参数动作 | `/chassis/save_parameters` |
| 轮速/PWM/误差 | `/motor/debug` |
| 累计编码器 | RDK 计算 `/odom` |
| 状态/故障/电压 | `/diagnostics` |
| 心跳 | `heartbeat_age` 与 STM32 重启检测 |

安全是两层实现：

```text
RDK：/cmd_vel 超过 0.20 s 就发送失能零速度
STM32：运动帧超过 command_timeout_ms 就进入 TIMEOUT
```

RDK 软件出错不能取消 STM32 的本地保护。

## 33. 一帧运动命令在本模块内的完整生命周期

假设上位机持续发送：

```text
linear = 0.20 m/s
angular = 0.50 rad/s
enable = 1
estop = 0
```

本模块内发生：

1. `process_can_frame()` 校验解码成功；
2. 覆盖 `app->command`；
3. 更新 `last_command_ms`；
4. 设置 `has_received_command=1`；
5. 下一次 10 ms 控制周期检查没有急停、严重故障和超时；
6. 硬件已使能，所以进入 `RUNNING`；
7. 限制线速度和角速度；
8. 阿克曼公式得到舵角约 0.3456 rad；
9. 得到左右目标约 0.1595 和 0.2405 m/s；
10. 若近似直行则执行同步修正，本例为转弯所以不启用；
11. 左右目标分别送入两个 `wheel_controller_update()`；
12. 两个控制器用编码器实测、斜坡、低通、前馈和 PI 得到 PWM；
13. 舵机目标约为 1720；
14. 检查堵转；
15. 更新里程计；
16. 任务释放互斥锁并把 PWM 写到硬件；
17. 后续串口任务从该结构生成轮速、PWM、状态和编码器遥测。

如果 RDK 停止发送：

1. `last_command_ms` 不再刷新；
2. 超过 200 ms；
3. 动态故障加入 `COMMAND_TIMEOUT`；
4. 状态变为 `TIMEOUT`；
5. `output_allowed=0`；
6. 两个轮控制器清空历史并返回 0；
7. 硬件收到零 PWM。

## 34. 如何阅读和调试这个模块

### 34.1 建议设置的断点

车轮架空并确保调试安全后，可观察：

```text
chassis_app_process_can_frame()
chassis_app_control_step()
wheel_controller_update()
update_stall_fault()
```

重点变量：

```text
app->state
app->latched_faults
app->dynamic_faults
app->command
elapsed_since_command
output_allowed
left/right_raw_speed_mps
left/right_target_mps
left/right_controller.filtered_speed_mps
left/right_controller.integral_output
left/right_pwm
steering_angle
servo_pwm
```

### 34.2 不接调试器时

RDK 查看：

```bash
ros2 topic echo /motor/debug
ros2 topic echo /diagnostics
ros2 topic echo /odom
```

`/motor/debug` 顺序：

```text
[左斜坡目标, 左滤波实测,
 右斜坡目标, 右滤波实测,
 左PWM, 右PWM,
 左误差, 右误差]
```

### 34.3 调试现象与可能位置

| 现象 | 优先检查 |
|---|---|
| 目标为正但实测为负 | 编码器符号、PWM符号 |
| 两侧目标正常但 PWM 始终 0 | `state`、`output_allowed`、硬件/软件使能 |
| 心跳正常但状态 TIMEOUT | RDK 是否持续发运动帧 |
| 启动瞬间冲击 | 积分是否清零、斜坡、死区/Kff |
| 直线逐渐跑偏 | 轮径、编码器计数、PI、直行同步 |
| PWM 很高但轮速接近 0 | 机械堵转、接线、堵转阈值 |
| 舵机方向相反 | `servo_pwm_per_rad` 符号 |
| `/odom` 跳变 | 编码器尖峰、累计计数环绕、重复节点 |

## 35. 几个容易误解的概念

### 35.1 `left_target_mps` 不是 PWM

它是希望车轮达到的物理速度。真正 PWM 由单轮控制器根据目标和实测计算。

### 35.2 `left_raw_speed_mps` 不等于 `filtered_speed_mps`

前者是本周期编码器直接换算，后者经过跨周期低通，更适合 PI 和遥测。

### 35.3 软件使能不等于一定运动

还必须满足硬件使能、命令新鲜、无故障，并且速度命令非零。

### 35.4 `IDLE` 不等于通信断开

正常连接且未使能时就是 `IDLE`。通信断开由心跳和命令超时分别反映。

### 35.5 锁存故障不应自动恢复

堵转、欠压、编码器异常或急停消失后自动重启可能造成危险，所以必须人工清除。

### 35.6 里程计是估计，不是真值

代码数学正确不代表实车永不漂移，机械标定和传感器融合仍然重要。

## 36. 本模块最核心的设计思想

可以总结为六点：

1. **单一状态对象**：`chassis_app_t` 集中保存控制所需状态；
2. **固定周期控制**：每 10 ms 用真实 `dt` 更新；
3. **安全先于控制**：先判状态和 `output_allowed`，再允许 PWM；
4. **整车与单轮分层**：整车层算目标，单轮层算 PWM；
5. **算法与硬件分层**：核心层不直接碰寄存器和 Flash；
6. **控制与遥测同源**：上传的是控制器真实使用的目标、实测、故障和输出。

理解这个模块时，始终沿着下面的主线追踪变量：

```text
command
  ↓ 安全状态机
output_allowed
  ↓ 运动学
left/right_target_mps
  ↓ 单轮闭环 + encoder feedback
left/right_pwm
  ↓ 硬件输出
motor movement
  ↓ encoder delta
下一控制周期
```

这就是当前 STM32 底盘核心控制模块的完整闭环。

---

# 第二部分：首次文档之后的追问与补充

## 补充文档 1

来源：`docs/18_底盘控制五个核心疑问_从物理原理到代码计算.md`

# 底盘控制五个核心疑问：从物理原理到代码计算

## 1. 先说结论：你现在感到混乱是正常的，学习方法需要调整

你现在面对的并不是一个简单的“让电机转起来”例程，而是一套同时包含以下内容的完整底盘控制程序：

```text
编码器测量
单位换算
底盘运动学
目标速度斜坡
速度滤波
前馈控制
PI 反馈控制
积分抗饱和
左右轮直行同步
阿克曼舵机换算
里程计积分
故障状态机
通信和参数管理
```

这些内容本来就是多个课程的交叉点。第一次阅读时“看着后面忘着前面”，不代表你不适合做这个项目，而是因为你试图同时在脑中保存太多层次。

你没有必要继续按照“从第一行看到最后一行，并记住所有参数”的方式阅读。那种方法对现在的你效率很低。

现阶段只需要先建立下面这一条主线：

```text
目标车速
   ↓ 运动学
左右轮目标速度
   ↓ 与编码器实测速度比较
速度误差
   ↓ 前馈 + PI
左右 PWM
   ↓ 电机转动
编码器产生新计数
   ↓
下一次控制周期重新修正
```

第一阶段只盯住五个变量：

```text
target_mps          目标速度
measured_mps        编码器实测速度
error               目标与实测之差
output_pwm          控制器输出
dt_s                两次控制之间的时间
```

其他参数先知道“属于哪一类”，不需要背数值。

本文按照你的五个问题重新讲解，而且每一部分都采用相同顺序：

```text
物理现象是什么
为什么要处理它
数学公式怎样得到
代码怎样实现
代入具体数字算一次
参数从哪里来
怎样在实车上验证
```

---

## 2. 问题一：车轮一圈计数 N=60000 是怎样得到的

### 2.1 N 到底代表什么

代码中的：

```c
params->counts_per_wheel_rev = 60000.0f;
```

它代表：

```text
驱动车轮最终转完整一圈时，STM32 编码器计数器累计多少个 count
```

注意这里说的是“车轮一圈”，不是电机内部转子一圈。

这个参数非常重要，因为代码用它把编码器计数换算成实际距离：

```text
一个计数对应距离 = 车轮周长 / 车轮一圈计数 N
```

如果 N 错一倍：

- 实测速度会错一倍；
- PI 会根据错误速度控制；
- 里程计距离会错一倍；
- 堵转和编码器故障判断也会受影响。

### 2.2 当前 60000 的代码来源

原厂工程中可以找到：

```c
#define Photoelectric_500 500
#define EncoderMultiples  4
#define HALL_30F          30
```

原厂计算：

```c
Encoder_precision =
    EncoderMultiples *
    Robot_Parament.EncoderAccuracy *
    Robot_Parament.GearRatio;
```

代入：

```text
编码器线数             = 500
AB 相四倍频            = 4
电机减速比             = 30

车轮一圈计数 N = 500 × 4 × 30
                 = 60000 count/rev
```

所以新控制代码把相同结果写成默认参数：

```c
params->counts_per_wheel_rev = 60000.0f;
/* 500 线 × 4 倍频 × 30 减速比 */
```

### 2.3 为什么 AB 相会有 4 倍频

正交编码器有 A、B 两路方波，相位相差 90°。

一个完整电气周期中会出现四个有效边沿：

```text
A 上升沿
B 上升沿
A 下降沿
B 下降沿
```

STM32 定时器使用：

```c
TIM_EncoderMode_TI12
```

会利用 A/B 两相边沿计数并判断方向。因此一个标称“500线”的编码器，在四倍频口径下，电机轴一圈得到：

```text
500 × 4 = 2000 count
```

### 2.4 为什么还要乘以减速比 30

假设减速比为 30:1：

```text
电机轴转 30 圈
输出轴/车轮转 1 圈
```

编码器安装在电机侧时，车轮转一圈期间电机轴转了 30 圈：

```text
2000 count/电机轴圈 × 30 电机轴圈/车轮圈
= 60000 count/车轮圈
```

如果编码器安装在减速箱输出轴上，就不应该再乘 30。因此必须知道编码器的真实安装位置。

### 2.5 “500线”这个说法为什么仍然可能有歧义

不同厂家会使用不同术语：

```text
PPR：每圈脉冲数
LPR：每圈线数
CPR：每圈计数数
```

有些厂家写“500 PPR”，指 A 相一圈 500 个周期，需要再乘 4；有些资料写“500 CPR”，可能已经包含四倍频。

因此：

```text
60000 是依据当前原厂源码得到的理论默认值，
但严谨工程中还要用实车验证。
```

### 2.6 怎样实测验证 N

最可靠的方法不是根据 `/odom` 反推，因为 `/odom` 本身已经使用了 N。应该直接观察原始累计编码器计数。

推荐步骤：

1. 电机软件失能；
2. 把车轮架空；
3. 在轮胎和车架上各做一个对齐标记；
4. 记录 `left_total_count` 或 `right_total_count` 初值；
5. 用手将车轮沿一个方向准确转 10 圈；
6. 记录末值；
7. 计算：

```text
N实测 = |末值 - 初值| / 10
```

为什么转 10 圈而不是 1 圈？

- 对齐标记误差会被除以 10；
- 齿轮间隙的相对影响减小；
- 更容易判断接近 60000 还是 15000、30000、120000。

可以通过 Keil Watch 观察累计计数，或者临时增加原始计数调试输出。现在 ROS `/odom` 没有直接显示原始 count，所以不能只看 `/odom` 完成这个独立验证。

### 2.7 N 怎样参与速度计算

当前轮径：

```text
D = 0.065 m
```

车轮周长：

```text
C = πD
  = π × 0.065
  ≈ 0.204204 m
```

一计数代表：

```text
meters_per_count = C / N
                 = π × 0.065 / 60000
                 ≈ 0.00000340339 m
                 ≈ 3.403 μm
```

若 10 ms 内读到 147 个计数：

```text
本周期距离 = 147 × 3.403 μm
           ≈ 0.0005003 m

速度 = 距离 / 时间
     = 0.0005003 / 0.010
     ≈ 0.05003 m/s
```

对应代码：

```c
static float count_to_distance_m(...)
{
    perimeter = PI * wheel_diameter_m;
    return count * perimeter / counts_per_wheel_rev;
}

static float delta_to_speed_mps(...)
{
    return count_to_distance_m(delta) / dt_s;
}
```

---

## 3. 问题二：程序刚开始时参数到底怎样初始化

### 3.1 不是所有参数都在 `main()` 中逐个设置

参数初始化分成三个层次：

```text
第一优先级：STM32 Flash 中已经保存且校验正确的参数
第二优先级：源码中的安全默认参数
第三种变化：运行后由 RDK 临时下发的新参数
```

其中 RDK YAML 不会在每次启动时自动覆盖 STM32，因为当前：

```yaml
push_parameters_on_start: false
```

所以不能把“RDK YAML 默认值”和“STM32 当前实际参数”简单理解成永远相同。

### 3.2 完整初始化调用链

程序大致执行：

```text
main()
  ↓
systemInit()
  ↓ 初始化编码器、PWM、串口等硬件
start_task()
  ↓
chassis_tasks_start()
  ↓
c30d_port_load_parameters(&stored_params)
  ↓
根据返回值调用 chassis_app_init()
```

`chassis_tasks_start()` 中的逻辑是：

```text
load_result > 0
    Flash 中是有效的新格式参数
    → chassis_app_init(app, &stored_params)

load_result == 0
    Flash 空白，或者还是原厂旧格式
    → chassis_app_init(app, NULL)

load_result < 0
    Flash 看起来是新格式，但范围或 CRC 错误
    → 传入坏参数，让 init 使用默认值并锁存 BAD_PARAMETER
```

### 3.3 `c30d_port_load_parameters()` 怎样判断 Flash 是否有效

它从 STM32 内部 Flash 参数区读取整个 `chassis_params_t`，然后检查：

```text
magic          是否等于固定标记 CH30
version        参数结构版本是否正确
size           结构体大小是否匹配
字段范围       例如轮径、Kp、超时是否处于允许范围
CRC32          内容是否损坏
```

固定标记：

```c
#define CHASSIS_PARAMETER_MAGIC 0x43483330
```

它对应 ASCII 的 `CH30`。目的类似文件头：看到这个标记才认为 Flash 中可能是本项目的新参数结构。

### 3.4 `chassis_app_init()` 第一步为什么是 `memset`

```c
memset(app, 0, sizeof(*app));
```

这会把整个底盘状态对象清零：

```text
PWM = 0
还没收到命令
故障位 = 0
编码器累计 = 0
里程计 = 0
积分历史 = 0
滤波速度 = 0
```

为什么要这样做？

局部或动态内存中原有内容可能是随机值。如果积分初值碰巧很大，第一次使能就可能出现突然输出。清零建立了确定、安全的起点。

### 3.5 有有效 Flash 参数时

```c
if (stored_params != 0 &&
    chassis_params_validate(stored_params) &&
    chassis_params_crc_is_valid(stored_params))
{
    app->params = *stored_params;
}
```

`app->params = *stored_params` 是结构体整体复制。之后控制器使用的是 RAM 中的 `app->params`，不会每 10 ms 直接读 Flash。

原因：

- RAM 读取更快；
- Flash 不适合频繁写；
- 参数修改可以先在 RAM 中试验，确认后再保存。

### 3.6 没有有效 Flash 参数时

调用：

```c
chassis_params_set_defaults(&app->params);
```

默认参数分成五组：

#### A. 电机控制初始值

```text
Kp = 3000
Ki = 1500
Kff = 9000
正转死区 = 700
反转死区 = 700
PWM 上限 = 16700
```

源码注释明确说：

```text
这些值以“车轮离地可安全验证”为目标，
不代表已经针对这台实车调到最佳。
```

#### B. 机械几何值

```text
轮径 = 0.065 m
轮距 = 0.162 m
轴距 = 0.144 m
一圈计数 = 60000
```

这些来自 R550 Mini 原厂资料和原厂代码宏：

```c
#define Black_WheelDiameter 0.065
#define Akm_wheelspacing    0.162
#define Akm_axlespacing     0.144
```

它们应该用尺子、编码器规格和实车行驶结果再次标定。

#### C. 速度和滤波值

```text
最大线速度 = 0.60 m/s
最大角速度 = 1.50 rad/s
加速度 = 0.50 m/s²
减速度 = 0.80 m/s²
滤波 alpha = 0.35
```

这些属于控制策略选择，不是机械结构直接决定的常数。

#### D. 舵机初值

```text
最大舵角 = 0.52 rad
中心 PWM = 1500
PWM/rad = 636.56
允许 PWM = 1000～2000
```

其中 1500 和 636.56 的来源会在第 6 章深入解释。

#### E. 安全阈值

```text
命令超时 = 200 ms
欠压 = 9800 mV
物理轮速异常阈值 = 2.0 m/s
堵转目标阈值 = 0.15 m/s
堵转实测阈值 = 0.02 m/s
堵转 PWM 比例 = 80%
堵转时间 = 600 ms
```

这些是保护策略，也需要根据电池类型、电机能力和实车试验确认。

### 3.7 为什么初始化后还要重置左右轮控制器

```c
wheel_controller_reset(&app->left_controller);
wheel_controller_reset(&app->right_controller);
```

这会清除：

```text
斜坡目标
滤波速度
积分输出
上次误差
PWM
```

它与前面的整个 `memset` 有一点重复，但好处是表达清晰：无论未来怎样修改初始化逻辑，两个控制器都明确从零状态开始。

### 3.8 为什么初始状态是 IDLE，而不是 RUNNING

```c
app->state = CHASSIS_STATE_IDLE;
```

要进入 `RUNNING`，后续控制周期必须同时满足：

```text
硬件使能有效
已经收到合法运动命令
软件 enable=1
命令未超时
没有急停
没有严重故障
```

所以加载完参数并不等于允许电机转动。

### 3.9 RDK 修改参数后发生什么

如果调用 `ros2 param set` 或参数下发服务：

```text
RDK 发送一个参数 ID 和新值
STM32 在 RAM 副本中尝试修改
检查范围
成功后清空两个轮子的控制历史
返回实际值和 sequence
```

此时只是 RAM 变化，掉电可能恢复旧 Flash 值。

只有再调用保存参数服务，STM32 才把当前结构写入 Flash。

---

## 4. 问题三：`sync_correction = clamp(...)` 怎样修正直行

### 4.1 先理解它要解决的物理问题

假设给左右轮相同目标：

```text
左目标 = 0.20 m/s
右目标 = 0.20 m/s
```

即使两个速度 PI 都工作，车辆仍可能慢慢跑偏，原因包括：

- 左右轮真实直径略有差异；
- 左右电机和减速箱效率不同；
- 左右轮负载不同；
- 地面摩擦不同；
- 编码器标定存在小误差；
- 两侧 PI 的稳态误差略有差异。

单轮速度环只关心“各自当前速度是否接近目标”，但直行还关心“从本次直行开始，两侧累计走过的距离是否一致”。

因此又增加一个较慢的外层同步环。

### 4.2 什么情况下才启用直行同步

代码要求：

```text
|angular_command| ≤ 0.03 rad/s
|linear_command| ≥ 0.05 m/s
```

原因：

- 明确要求转弯时，左右轮本来就应该走不同距离，不能强行拉平；
- 速度太接近零时，编码器量化误差占比大，也没有必要同步。

### 4.3 为什么先记录本次直行的起点

第一次进入直行状态时：

```c
straight_left_start_m  = left_distance_m;
straight_right_start_m = right_distance_m;
```

之后计算本次直行各自走了多少：

```text
left_progress  = left_distance  - left_start
right_progress = right_distance - right_start
```

为什么不直接比较上电以来的总距离？

车辆之前可能刚转过弯，转弯本来就会让左右轮累计距离不同。直行修正不应该试图消除以前正常转弯造成的差异，所以要在每次开始直行时重新建立零点。

### 4.4 `sync_error` 的定义

```text
sync_error = left_progress - right_progress
```

三种情况：

```text
sync_error > 0   左轮比右轮走得多
sync_error = 0   两轮进度相同
sync_error < 0   右轮比左轮走得多
```

单位是米。

### 4.5 `Ksync × sync_error` 为什么能变成速度修正

代码：

```c
sync_correction =
    straight_sync_kp * sync_error;
```

其中：

```text
sync_error       单位 m
sync_correction  希望单位 m/s
```

所以 `straight_sync_kp` 的量纲实际上近似：

```text
(m/s) / m = 1/s
```

当前值 `0.40` 可理解成：每出现 1 m 的累计距离差，理论上要求 0.40 m/s 的差速修正。当然实际误差通常只有毫米或厘米。

### 4.6 `clamp()` 到底做什么

```c
sync_correction = clamp_float(
    Ksync * sync_error,
    -sync_max,
    +sync_max);
```

`clamp` 就是限幅：

```text
如果结果大于 +sync_max → 只取 +sync_max
如果结果小于 -sync_max → 只取 -sync_max
否则保持原值
```

当前：

```text
Ksync = 0.40
sync_max = 0.08 m/s
```

为什么要限幅？

如果编码器异常、打滑或参数错误造成很大距离差，不允许同步外环突然给出巨大的左右速度差。它只能做有限的小修正，不能压过主运动命令。

### 4.7 为什么左轮减、右轮加

代码：

```c
left_target  -= sync_correction;
right_target += sync_correction;
```

假设左轮走得多：

```text
sync_error > 0
sync_correction > 0
```

于是：

```text
左目标 = 原目标 - 正修正 → 左轮减速
右目标 = 原目标 + 正修正 → 右轮加速
```

这会让距离差朝零方向变化。

这叫负反馈：

```text
出现正误差
→ 控制动作让误差减小
```

如果符号写反：

```text
左轮已经走多了，反而继续加快左轮
```

误差会越来越大，那就是正反馈，会发散。

### 4.8 具体算一遍

假设本次直行开始后：

```text
左轮走了 1.020 m
右轮走了 1.000 m
基础目标都是 0.200 m/s
```

距离误差：

```text
sync_error = 1.020 - 1.000
           = 0.020 m
```

未限幅修正：

```text
correction = 0.40 × 0.020
           = 0.008 m/s
```

`0.008` 没超过 `0.08`，所以 clamp 后仍为 `0.008`。

修正后的目标：

```text
left_target  = 0.200 - 0.008 = 0.192 m/s
right_target = 0.200 + 0.008 = 0.208 m/s
```

然后这两个新目标分别交给左右 `wheel_controller_update()`。同步模块本身不直接改 PWM，它只改目标速度；真正的 PWM 修正由各自的速度控制器完成。

### 4.9 倒车时符号仍然正确吗

假设倒车时左轮走得更远：

```text
left_progress  = -1.020 m
right_progress = -1.000 m
```

```text
sync_error = -1.020 - (-1.000)
           = -0.020 m
correction = -0.008 m/s
```

原目标都是 `-0.200 m/s`：

```text
left_target  = -0.200 - (-0.008) = -0.192
right_target = -0.200 + (-0.008) = -0.208
```

左轮速度绝对值变小，右轮绝对值变大，仍然会消除距离差。因此当前符号在前进和倒车中都构成负反馈。

### 4.10 为什么它是外环，速度 PI 是内环

```text
外环：比较累计距离
      ↓ 产生小的目标速度修正
内环：比较目标速度和当前实测速度
      ↓ 产生 PWM
```

外环处理慢慢累积的跑偏；内环处理每 10 ms 的速度误差。这种“慢位置环套快速度环”的结构在电机系统中很常见。

---

## 5. 问题四之一：`wheel_controller_update()` 到底怎样修正速度

### 5.1 先不要看代码，先理解控制原理

电机存在一个大致关系：

```text
PWM 增大 → 电机平均电压增大 → 电机通常转得更快
PWM 减小 → 电机通常转得更慢
```

但相同 PWM 并不总得到相同速度，因为还受以下因素影响：

```text
电池电压
车重
坡度
摩擦
齿轮效率
电机个体差异
```

所以不能只做：

```text
速度 0.2 m/s 永远对应 PWM 2500
```

必须反复测量实际速度并修正：

```text
目标 0.20，实际 0.15 → 太慢 → 增加 PWM
目标 0.20，实际 0.23 → 太快 → 减小 PWM
```

这就是反馈控制。

### 5.2 函数七个输入分别是什么

```c
wheel_controller_update(
    controller,
    control_params,
    chassis_params,
    target_mps,
    measured_mps,
    dt_s,
    allow_output);
```

| 输入 | 含义 |
|---|---|
| `controller` | 本轮跨周期保存的滤波、积分、斜坡状态 |
| `control_params` | 本轮 Kp、Ki、Kff、死区、PWM 上限 |
| `chassis_params` | 全车加速度、减速度、滤波等参数 |
| `target_mps` | 希望本轮达到的速度 |
| `measured_mps` | 编码器测出的本轮速度 |
| `dt_s` | 距离上次计算过去多少秒，正常 0.01 |
| `allow_output` | 安全状态机是否允许电机输出 |

输出是 `int16_t PWM`。

### 5.3 第一步：实测速度低通滤波

代码：

```c
filtered += alpha * (measured - filtered);
```

改写为：

```text
新滤波值 = (1-alpha) × 旧滤波值
           + alpha × 本次测量值
```

当前 `alpha=0.35`：

```text
新值 = 65%旧值 + 35%本次测量
```

为什么要滤波？

低速时编码器一个周期可能读到 14 个 count，下一个周期读到 15 个 count。真实机械速度可能很平稳，但离散计数换算出的瞬时速度会跳动。若 PI 完全跟随这个跳动，PWM 也会抖动。

例子：

```text
旧滤波速度 = 0.040 m/s
本次原始速度 = 0.060 m/s
alpha = 0.35
```

```text
新滤波速度
= 0.040 + 0.35 × (0.060 - 0.040)
= 0.047 m/s
```

它没有立即跳到 0.060，而是向测量值靠近。

代价是产生一定延迟。所以 alpha 不是越小越好：

```text
alpha 大 → 响应快，但噪声多
alpha 小 → 更平滑，但反馈迟钝
```

### 5.4 第二步：安全失能时彻底清历史

```c
if (allow_output == 0)
{
    ramped_target = 0;
    integral = 0;
    last_error = 0;
    PWM = 0;
    return 0;
}
```

为什么不只是返回 0，而要清积分和斜坡？

假设故障前积分已经积累到 4000。失能期间若保留，故障清除重新使能时，这个旧积分会立刻重新施加，车辆可能突然冲击。

所以失能不仅是“这一拍 PWM 为零”，也是“清除控制器记忆”。

### 5.5 第三步：目标限幅

```text
target ∈ [-maximum_speed, +maximum_speed]
```

即使整车层已经限过一次，单轮控制器再限一次属于防御式编程，防止未来其他调用者传入异常目标。

### 5.6 第四步：目标速度斜坡

如果命令从 0 突然变成 0.6 m/s，直接让 PI 追 0.6 会产生很大误差和 PWM。斜坡把目标逐步增加：

```text
每周期最大加速变化 = acceleration × dt
```

当前：

```text
acceleration = 0.50 m/s²
dt = 0.01 s
```

```text
每 10 ms 最多增加 0.005 m/s
```

目标 0.05 m/s 时，斜坡大致为：

```text
第1周期  0.005
第2周期  0.010
第3周期  0.015
...
第10周期 0.050 m/s
```

约 0.1 s 到达目标。

为什么减速度设置为 0.80，比加速度 0.50 大？

```text
启动柔和一些
停车更快一些
换向时先较快回到零，再向相反方向加速
```

斜坡不是反馈，它是对目标命令做整形。

### 5.7 第五步：目标归零时清积分

当原目标和斜坡目标都接近零：

```text
PWM = 0
积分 = 0
```

为什么不是让 PI 在零速度附近继续工作？

编码器低速量化、静摩擦和微小噪声可能让 PI 为维持“严格零速度”持续给出正负小 PWM，造成：

- 电机嗡响；
- 发热；
- 车辆轻微抖动；
- 积分慢慢积累。

静止命令下直接断输出更符合当前底盘需求。

### 5.8 第六步：死区补偿

直流电机和齿轮箱有静摩擦。PWM 从 0 慢慢增加时，可能在 0～600 范围内都不转，到约 700 才克服静摩擦。

所以只要斜坡目标为正：

```text
deadzone = +dead_forward
```

为负：

```text
deadzone = -dead_reverse
```

死区补偿提供一个基础推动量。

这个值应该实测：架空车轮，从 0 缓慢增加 PWM，记录刚刚能持续转动的值；正转和反转分别测，因为机械摩擦可能不对称。

### 5.9 第七步：计算速度误差

```text
error = ramped_target - filtered_speed
```

如果：

```text
error > 0 → 实际偏慢，需要增加输出
error < 0 → 实际偏快，需要减小输出
error = 0 → 当前速度正好等于斜坡目标
```

### 5.10 第八步：前馈 Kff

```text
feedforward = Kff × ramped_target + deadzone
```

前馈的思想是：

```text
在误差还没出现前，根据目标速度先猜一个大致需要的 PWM。
```

如果实验发现稳定空载时：

```text
0.10 m/s 大约需要总 PWM 1600
死区约 700
```

扣除死区后速度相关部分约 900：

```text
Kff ≈ (1600 - 700) / 0.10
    ≈ 9000 PWM/(m/s)
```

这解释了默认 `Kff=9000` 的量级。它不是由理论电机参数严格计算出来的，而是初始经验值，应该通过“稳态速度—PWM”实验标定。

前馈不能独立保证准确，因为负载和电压会变化，所以还需要反馈 PI。

### 5.11 第九步：比例项 P

```text
P = Kp × error
```

比例项对当前误差立即反应。

假设：

```text
目标 = 0.050
滤波实测 = 0.047
error = 0.003 m/s
Kp = 3000
```

```text
P = 3000 × 0.003 = 9 PWM
```

如果误差变成 0.010，P 就变成 30。

为什么 P 能修正速度？

- 实际偏慢时 error 为正，P 为正，总 PWM 增大；
- 实际偏快时 error 为负，P 为负，总 PWM 减小。

这同样是负反馈。

Kp 太小：修正无力、响应慢。

Kp 太大：一次微小速度噪声就引起大 PWM 变化，可能振荡、啸叫。

### 5.12 第十步：积分项 I

比例控制常会留下稳态误差。例如坡道上需要额外 100 PWM 才能维持速度。如果误差已经很小，P 提供的额外输出也很小，系统可能长期略慢。

积分将每个周期的小误差累加：

```text
I_new = I_old + Ki × error × dt
```

假设：

```text
I_old = 20 PWM
Ki = 1500
error = 0.003 m/s
dt = 0.01 s
```

```text
本周期积分增加
= 1500 × 0.003 × 0.01
= 0.045 PWM

I_new = 20.045 PWM
```

单周期增加很小，但只要偏慢持续存在，积分会逐渐增加，直到消除长期误差。

为什么公式必须乘 `dt`？

积分在数学上是误差对时间的累积。若控制频率从 100 Hz 变成 200 Hz，每秒调用次数翻倍；不乘 dt 就会让积分每秒增长翻倍。乘以真实时间后，控制频率变化时物理意义更一致。

### 5.13 总输出

```text
PWM_raw = feedforward + P + I
```

使用前面的例子：

```text
ramped_target = 0.050
filtered = 0.047
deadzone = 700
Kff = 9000
Kp = 3000
I_new = 20.045
```

```text
feedforward = 9000 × 0.050 + 700
            = 1150

P = 3000 × (0.050 - 0.047)
  = 9

PWM_raw = 1150 + 9 + 20.045
        = 1179.045
```

最终四舍五入为约 `1179`。

下一周期编码器会告诉控制器这个 PWM 产生了怎样的新速度，再重新计算。这就是持续修正，而不是一次算完。

### 5.14 PWM 限幅

```text
PWM = clamp(PWM_raw, -16700, +16700)
```

原因：定时器和电机驱动有物理上限，软件不能输出无限大。

### 5.15 为什么需要积分抗饱和

假设车轮被卡住，目标很大，误差持续为正。理论输出可能已经超过 16700，但实际只能输出 16700。如果积分仍不断增加，可能积累到几万。

障碍一旦移除，积分不会立刻消失，电机会长时间全速冲出。这叫积分饱和或 windup。

当前代码先计算候选积分和候选输出。如果：

```text
输出没有饱和
```

就接受新积分。

如果输出已经正向饱和，但误差变成负数，负误差有助于退出饱和，也允许更新积分。

如果输出正向饱和且误差仍为正，就暂停积分。

负向饱和同理。

### 5.16 一张图理解各部分职责

```text
原始目标速度
    ↓ 斜坡：防突然加速/换向
斜坡目标 ───────────────┐
    ↓                    │
Kff×目标 + 死区：        │
预估基础 PWM             │
                         ↓
编码器 → 低通 → 实测速度 → 误差
                              ├→ P：立即修正当前误差
                              └→ I：慢慢消除长期误差

基础 PWM + P + I
    ↓ 限幅与抗积分饱和
最终 PWM
```

### 5.17 这些参数该怎样得到，而不是死记

推荐调试顺序：

1. 车轮架空；
2. 分别测正反转最小持续转动 PWM，得到死区；
3. 暂时让 Ki 很小或为 0；
4. 测几个稳定速度点对应的 PWM，估计 Kff；
5. 逐步增加 Kp，使速度能较快跟随但不明显振荡；
6. 最后逐步增加 Ki，消除负载下稳态误差；
7. 检查启动、停车、换向和电池电压变化；
8. 左右轮分别记录，不假设完全相同。

不要同时大幅修改 Kff、Kp、Ki 和死区，否则即使结果变好，也不知道是哪一个参数起作用。

---

## 6. 问题四之二：舵机公式为什么这样算，参数从哪里来

### 6.1 先区分三个不同量

```text
车体线速度 v                m/s
车体期望角速度 w            rad/s
虚拟中心前轮角 δc           rad
右前轮实际目标角 δR         rad
```

`angular.z` 是希望车体每秒转过多少弧度，不是直接给舵机的 PWM，也不是直接的前轮角度。

这里必须区分 `δc` 和 `δR`。当前新代码计算的是单轨模型中的虚拟中心前轮角 `δc`；原厂舵机多项式真正接收的是右前轮角 `δR`，二者并不完全相同。

### 6.2 阿克曼/单轨模型

将左右前轮近似成车体中心的一只虚拟前轮，得到单轨模型：

```text
        前轮方向
            /
           / δ
----------O----------  前轴
          |
          | L（轴距）
          |
----------O----------  后轴中心
          |
          | R（转弯半径）
          ● 瞬时转动中心
```

几何关系：

```text
tan(δ) = L / R
```

车辆沿半径 R 的圆运动时：

```text
w = v / R
```

所以：

```text
R = v / w
```

代入：

```text
tan(δ) = L / (v/w)
       = Lw/v
```

因此：

```text
δc = atan(Lw/v)
```

这就是代码：

```c
steering_angle = atanf(
    wheelbase_m * angular_command / linear_command);
```

但原厂代码还多了一步。先计算车体中心转弯半径：

```text
R = v / w
```

再根据轮距 `W` 计算右前轮角：

```text
δR = atan(L / (R + W/2))
```

对应原厂 `Vz_to_Akm_Angle()`：

```c
R = Vx / Vz;
AngleR = atan(Axle_spacing / (R + 0.5f * Wheel_spacing));
```

随后原厂三次多项式使用的是这个 `AngleR`，不是虚拟中心角 `δc`。

### 6.3 每个参数什么意思

```text
steering_angle / δc   当前新代码的虚拟中心前轮角，rad
AngleR / δR           原厂用于舵机映射的右前轮角，rad
wheelbase_m / L       前后轴中心距离，当前默认 0.144 m
wheel_track_m / W     左右轮间距，当前默认 0.162 m
angular_command / w   希望车体角速度，rad/s
linear_command / v    希望车体线速度，m/s
atanf                  浮点反正切函数
```

### 6.4 为什么线速度太小时不计算

公式包含：

```text
Lw/v
```

当 v 接近 0：

- 会除以非常小的数；
- 算出接近 90° 的不可能舵角；
- 阿克曼车本身也不能靠舵机原地旋转。

所以当前代码在：

```text
|v| < 0.02 m/s
```

时将 v 和 w 都置零。

### 6.5 舵角数值例子

```text
v = 0.20 m/s
w = 0.50 rad/s
L = 0.144 m
```

```text
δc = atan(0.144 × 0.50 / 0.20)
   = atan(0.36)
   ≈ 0.34556 rad
   ≈ 19.80°
```

这是当前新代码使用的虚拟中心前轮角。完整原厂链路还会计算：

```text
R = v/w = 0.20/0.50 = 0.40 m

δR = atan(0.144 / (0.40 + 0.162/2))
   = atan(0.144 / 0.481)
   ≈ 0.29088 rad
   ≈ 16.67°
```

所以同一条运动命令下，`δc≈0.34556 rad`，而进入原厂舵机多项式的右前轮角是 `δR≈0.29088 rad`。不能把两个角混为一个量。

最大舵角默认：

```text
0.52 rad ≈ 29.8°
```

本例未超过机械限制。

### 6.6 为什么限幅后还要重新计算 w

若理论得到的前轮角超过机械极限，真实车辆达不到原期望曲率。

代码先限制 δ，再用：

```text
w_limited = v × tan(δ_limited) / L
```

重新得到真实可实现的角速度，然后根据它计算左右驱动轮目标。这样驱动轮差速与舵角相互一致。

上式适用于虚拟中心角 `δc`。若恢复原厂以右前轮角 `δR` 为基准的模型，限幅后应先反算中心转弯半径：

```text
R_limited = L / tan(δR_limited) - W/2
w_limited = v / R_limited
```

这样舵机使用的右前轮角、车体角速度和左右后轮速度才来自同一条几何关系。

### 6.7 从前轮角度到舵机 PWM

当前新代码使用简化线性关系：

```text
servo_pwm = servo_center_pwm
          + servo_pwm_per_rad × steering_angle
```

参数：

```text
servo_center_pwm = 1500
servo_pwm_per_rad = 636.56
```

本例：

```text
servo_pwm
= 1500 + 636.56 × 0.34556
≈ 1719.97
≈ 1720
```

最后再限制在 1000～2000。

### 6.8 1500 是怎么来的

原厂 TIM12 舵机定时器配置使计数单位约为 1 μs：

```text
CCR = 1000 → 约 1.0 ms 高电平
CCR = 1500 → 约 1.5 ms 高电平
CCR = 2000 → 约 2.0 ms 高电平
```

常见 RC 舵机使用约 1～2 ms 脉宽，1.5 ms 附近作为机械中心，所以原厂定义：

```c
#define SERVO_INIT 1500
```

但装配误差可能让“车轮真正朝正前方”对应 1470、1520 等数值。中心值需要实车校准，不应盲信 1500。

### 6.9 636.56 是怎么来的

原厂代码中存在：

```c
float Ratio = 636.56;
```

数值接近：

```text
1000 / (π/2) ≈ 636.62 count/rad
```

可以理解为一种近似：舵机脉宽变化 1000 μs 对应舵机轴约 90°，把弧度换成 PWM 计数。

但这里有一个非常重要的事实：

```text
636.56 描述的是舵机轴角度到 PWM 的比例，
不一定等于前轮转角到 PWM 的比例。
```

舵机轴与前轮之间还有摇臂、拉杆、转向节，机械关系通常不是严格 1:1，也可能不是线性。

### 6.10 当前新代码的舵机映射不适用于这台原厂机械结构

原厂阿克曼代码不是直接：

```text
PWM = 1500 + 636.56 × 前轮角
```

原厂先把上位机角速度换算成右前轮角 `δR`，再使用三次多项式近似转向连杆：

```text
Angle_Servo =
    -0.628 × δR³
    +1.269 × δR²
    -1.772 × δR
    +1.573

PWM = 1500 + (Angle_Servo - 1.572) × 636.56
```

而当前新代码把虚拟中心角 `δc` 直接代入简单线性关系：

```text
PWM = center + pwm_per_rad × δc
```

这不是对原厂机械关系的等价简化，原因有三点：

1. 输入角不同：原厂用右前轮角 `δR`，当前代码用虚拟中心角 `δc`；
2. 映射不同：原厂包含摇臂、拉杆和转向节的三次非线性补偿；
3. 默认方向相反：正角度时，原厂 PWM 减小，当前 `+636.56` 却让 PWM 增大。

仍使用 `v=0.20 m/s、w=0.50 rad/s`：

```text
当前代码：δc≈0.34556 → PWM≈1720
原厂链路：δR≈0.29088 → PWM≈1231
```

两者相差约 489 个 1 μs 计数，而且位于中心值 1500 的相反两侧。因此，针对用户提供的这台原厂小车，当前 `servo_pwm_per_rad=+636.56` 的线性实现应判定为错误，不能继续用于实际转向。

此外，原厂还使用非对称前轮角限制 `-0.49～+0.32 rad` 和最终 PWM 限制 `800～2200`；当前新代码使用对称角度限制和 `1000～2000`，也不是原厂标定的完整复现。修正时不能只给线性斜率加一个负号，必须恢复“右前轮角计算 + 三次多项式 + 非对称限幅”整条链路。

### 6.11 正确的舵机标定方法

推荐不要先猜公式，而是采集实测点：

1. 电机失能；
2. 车轮落在可测角度的平台上，或使用角度尺；
3. 给舵机若干安全 PWM，例如 1100、1200、1300……1900；
4. 每个 PWM 测量实际等效前轮转角 δ；
5. 找到车轮正前方对应的 `center_pwm`；
6. 检查左右方向符号；
7. 画出 `PWM—前轮转角` 数据；
8. 若近似直线，用：

```text
slope = (PWM2 - PWM1) / (δ2 - δ1)
```

9. 若明显非线性，应恢复查表或多项式映射；
10. 测量机械安全极限，再设置 min/max 和 maximum angle。

参数负号可以表达安装方向：

```text
正 δ 需要 PWM 增大 → slope 为正
正 δ 需要 PWM 减小 → slope 为负
```

---

## 7. 问题五：中点积分到底怎样计算

### 7.1 里程计要解决什么问题

编码器只告诉我们左右轮在一小段时间里分别走了多少。我们希望估计车辆在平面上的：

```text
x       前后/全局 X 位置
y       左右/全局 Y 位置
yaw     车头方向角
```

假设坐标约定：

```text
车头向前是局部 x 正方向
左侧是局部 y 正方向
逆时针转动是 yaw 正方向
```

### 7.2 先由左右轮速度得到车体速度

设：

```text
vL = 左轮线速度
vR = 右轮线速度
W  = 左右轮距
```

车体中心线速度：

```text
v = (vL + vR) / 2
```

为什么取平均？

- 两轮同速前进时，中心速度就是共同速度；
- 左轮静止、右轮前进时，车体中心速度约为右轮的一半；
- 两轮等速反向时，平均为零，车辆中心近似原地旋转。

车体角速度：

```text
w = (vR - vL) / W
```

为什么是速度差除以轮距？

转弯时两轮绕同一个瞬时中心运动：

```text
vL = w × RL
vR = w × RR
```

且：

```text
RR - RL = W
```

所以：

```text
vR - vL = w(RR - RL) = wW
w = (vR-vL)/W
```

### 7.3 一个周期内走了多少和转了多少

控制周期 `dt` 内：

```text
中心位移 ds = v × dt
角度变化 dθ = w × dt
```

也可直接由左右轮位移得到：

```text
ds = (dL + dR) / 2
dθ = (dR - dL) / W
```

### 7.4 最简单的欧拉积分

如果车辆周期开始时方向为 θ，最简单写法：

```text
x_new = x + ds × cos(θ)
y_new = y + ds × sin(θ)
θ_new = θ + dθ
```

问题是：车辆在这个周期中正在转弯，方向并不是一直等于周期开始的 θ。使用旧方向会把整段位移都投影到错误方向上。

### 7.5 中点积分的思想

周期开始方向：

```text
θ
```

周期结束方向：

```text
θ + dθ
```

周期中间的方向近似：

```text
θ_mid = θ + dθ/2
```

假设这一小段距离主要沿中间方向前进：

```text
x_new = x + ds × cos(θ + dθ/2)
y_new = y + ds × sin(θ + dθ/2)
θ_new = θ + dθ
```

这就是中点积分。

### 7.6 为什么中点法比旧角度更合理

假设周期内车头从 0° 转到 10°。

- 欧拉法假设全部距离都沿 0°；
- 使用新角度则假设全部距离都沿 10°；
- 实际路径方向在 0°～10° 之间变化；
- 使用 5° 的中间方向更接近整段平均方向。

控制周期很短、单周期转角很小时，中点法对圆弧运动的近似非常好。

### 7.7 与精确圆弧公式的关系

若 `dθ ≠ 0`，圆弧的精确积分可写为：

```text
R = ds / dθ

Δx = R[sin(θ+dθ) - sin(θ)]
Δy = -R[cos(θ+dθ) - cos(θ)]
```

使用三角恒等式：

```text
sin(A)-sin(B)
= 2cos((A+B)/2)sin((A-B)/2)
```

得到：

```text
Δx = ds × [sin(dθ/2)/(dθ/2)]
          × cos(θ+dθ/2)
```

当 dθ 很小时：

```text
sin(dθ/2)/(dθ/2) ≈ 1
```

所以：

```text
Δx ≈ ds × cos(θ+dθ/2)
Δy ≈ ds × sin(θ+dθ/2)
```

这正是中点公式。

它还有一个工程优势：当 dθ 接近 0 时，不需要计算 `R=ds/dθ`，避免除以接近零的数。

### 7.8 完整数值例子

假设本周期：

```text
左轮位移 dL = 0.009 m
右轮位移 dR = 0.011 m
轮距 W = 0.162 m
旧航向 θ = 0.300 rad
```

中心位移：

```text
ds = (0.009 + 0.011) / 2
   = 0.010 m
```

航向变化：

```text
dθ = (0.011 - 0.009) / 0.162
   ≈ 0.0123457 rad
   ≈ 0.707°
```

中间航向：

```text
θ_mid = 0.300 + 0.0123457/2
      ≈ 0.3061728 rad
```

位置增量：

```text
Δx = 0.010 × cos(0.3061728)
   ≈ 0.00953494 m

Δy = 0.010 × sin(0.3061728)
   ≈ 0.00301412 m
```

新航向：

```text
θ_new = 0.300 + 0.0123457
      = 0.3123457 rad
```

精确圆弧公式在该例中得到：

```text
Δx ≈ 0.00953488 m
Δy ≈ 0.00301410 m
```

与中点法极其接近，因为单周期转角很小。

### 7.9 当前 STM32 代码具体用了什么数据

代码先使用滤波轮速：

```c
linear_speed =
    (left_filtered + right_filtered) / 2;

angular_speed =
    (right_filtered - left_filtered) / wheel_track;
```

再计算：

```c
delta_yaw = angular_speed * dt_s;
average_yaw = yaw + 0.5f * delta_yaw;

x += linear_speed * cos(average_yaw) * dt_s;
y += linear_speed * sin(average_yaw) * dt_s;
yaw = wrap_angle(yaw + delta_yaw);
```

这里：

```text
ds = linear_speed × dt
dθ = angular_speed × dt
```

所以与上面的中点公式完全对应。

### 7.10 为什么 STM32 用滤波速度，而 RDK 用累计 count

STM32 这里使用滤波速度，优点是轨迹和速度更平滑；缺点是滤波会引入相位延迟，快速加减速时可能产生一点积分偏差。

RDK 的 `wheel_odometry.py` 使用左右累计计数差直接计算每批位移，更接近原始位移积分。

这两份里程计的目的不同：

```text
STM32 内部里程计：本地控制状态和未来扩展
RDK 里程计：发布 ROS /odom
```

当前 STM32 的 `x/y/yaw` 没有直接通过现有协议上传，ROS `/odom` 主要看 RDK 那一份。

### 7.11 中点积分为什么仍然会漂移

积分公式正确不等于位置永远正确。输入的轮距、轮径和轮位移可能有误差：

```text
轮胎打滑
实际轮径不准
左右轮径不同
轮距参数不准
阿克曼模型简化
编码器量化
地面不平
```

积分会把小误差不断累加。因此轮式里程计是相对估计，不是绝对定位。未来可结合正确校准的 IMU、激光或视觉进行融合。

---

## 8. 这些参数不要一起记，按四类整理

你感到“参数超级多”，主要是因为不同性质的参数混在同一个结构体里。可以只按四类理解。

### 8.1 机械/传感器参数：应该测量或查规格

```text
wheel_diameter_m
wheel_track_m
wheelbase_m
counts_per_wheel_rev
servo_center_pwm
servo_pwm_per_rad 或舵机映射表
```

它们描述“这台车实际长什么样、传感器怎样计数”。

### 8.2 控制器参数：应该通过实验调节

```text
Kff
Kp
Ki
dead_forward/reverse
filter_alpha
straight_sync_kp
straight_sync_max
```

它们描述“控制器怎样修正”。

### 8.3 命令整形参数：根据希望的驾驶感受选择

```text
maximum speed
acceleration
deceleration
maximum steering angle
```

它们描述“允许多快、启动停车多柔和”。

### 8.4 安全参数：根据硬件能力和风险验证

```text
command timeout
low battery
physical speed limit
stall thresholds
stall time
PWM limit
```

它们描述“什么情况必须停止或报警”。

现阶段每一类只记住它解决什么问题，不要背全部默认数值。

---

## 9. 你还有必要继续这样看代码吗

### 9.1 有必要继续学这个项目，但没有必要继续现在这种读法

这个项目值得学，因为它包含真正的：

```text
传感器反馈
实时周期控制
物理单位换算
运动学
闭环控制
安全状态机
上下位机通信
ROS 数据接口
```

这些内容比“只给驱动器发送速度命令”更有技术含量。

但你不需要现在把 700 行 `chassis_app.c` 和所有参数都记住。工程师也不会靠记忆保存整套代码，而是靠：

```text
模块图
数据流
变量命名
调试工具
实验记录
需要时重新定位代码
```

### 9.2 先暂停阅读这些内容

现在可以暂时跳过：

- Flash CRC 具体每一位怎样算；
- 参数 ID 的全部 30 个分支；
- 所有遥测帧的每个字节；
- IMU 驱动寄存器；
- OLED、手柄、旧原厂任务；
- SocketCAN 保留版本；
- 所有故障边界条件。

它们不是不重要，而是不属于你当前建立电机闭环主线所必需的第一层知识。

### 9.3 第一轮只学“一个轮子恒速”

只回答：

```text
目标速度从哪里来？
编码器怎样得到实测速度？
error 怎样算？
Kff、P、I 各自产生多少 PWM？
PWM 怎样影响下一周期速度？
```

建议只看：

```text
count_to_distance_m()
delta_to_speed_mps()
wheel_controller_update()
```

先完全不看转向、直行同步和里程计。

### 9.4 第二轮再学“两个轮子怎样组成底盘”

只看：

```text
left_target = v - wW/2
right_target = v + wW/2
舵角 atan(Lw/v)
```

然后用纸笔算三个命令：

```text
直行：v=0.1, w=0
左转：v=0.1, w=0.3
后退：v=-0.1, w=0
```

### 9.5 第三轮才看保护

只跟踪：

```text
output_allowed
state
latched_faults
dynamic_faults
```

回答“为什么 PWM 会被置零”。

### 9.6 第四轮再看里程计和同步

到这时再学习：

```text
sync_error
sync_correction
ds
dθ
中点积分
```

这些就会落到已有的左右轮概念上，而不是孤立公式。

### 9.7 最有效的方法不是继续读，而是做一个小实验

建议第一个实验只做架空 `0.05 m/s`：

```bash
ros2 topic echo /motor/debug
```

只记录：

```text
左目标
左实测
左 PWM
左误差
```

观察 2～3 秒：

1. 斜坡目标怎样从 0 增长到 0.05；
2. 实测怎样追上目标；
3. 误差变小时 P 怎样变小；
4. PWM 最后稳定在什么范围。

把代码中的一次计算和真实一组数据对上，比继续阅读几十页更容易形成理解。

### 9.8 你不需要“看完才算会”

真正掌握的标准不是能背出代码，而是能回答：

```text
车轮偏慢时，哪几个量会怎样变化？
为什么失能后 PWM 必须为零？
为什么编码器计数可以换成 m/s？
为什么左轮走多后要降低左目标？
为什么转弯时左右轮速度不同？
为什么位置估计会漂移？
```

能用自己的话解释并通过数据验证，才是掌握。

---

## 10. 最后把五个问题压缩成一张主线图

```text
编码器规格：500线 × 4倍频 × 30减速比
                  ↓
            N = 60000 count/轮圈
                  ↓
编码器 delta → 距离 → measured_mps
                  ↓
            一阶低通滤波
                  ↓
目标 v,w → 阿克曼/左右轮运动学 → target_mps
                  ↓
直行时累计距离差 → sync_correction → 修正左右 target
                  ↓
target 经过加减速斜坡
                  ↓
error = target - measured
                  ↓
PWM = 死区 + Kff×target + Kp×error + 积分
                  ↓
          限幅和抗积分饱和
                  ↓
              电机转动
                  ↓
           下一周期编码器反馈

同时：
左右轮实测位移
  → ds=(dL+dR)/2
  → dθ=(dR-dL)/轮距
  → 使用 θ+dθ/2 做中点积分
  → 更新 x、y、yaw
```

这张图就是当前底盘核心算法。其余大量代码主要是在保证这条主线能够安全、可靠地长期运行。


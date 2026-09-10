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

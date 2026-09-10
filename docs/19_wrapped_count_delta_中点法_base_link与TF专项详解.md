# `wrapped_count_delta`、中点法、`base_link` 与 TF 专项详解

> 适用对象：刚开始学习 Python、ROS 2、编码器里程计的读者。
>
> 对应本项目代码：`ros2_ws/src/chassis_can_control/chassis_can_control/wheel_odometry.py` 和 `chassis_can_node.py`。

---

## 0. 先给出三个问题的最简答案

### 问题 1：`wrapped_count_delta` 是干什么的？

编码器累计计数用的是有范围的 32 位整数。车轮一直转时，数字走到最大值会突然跳到最小值，反向转时也可能从最小值跳到最大值。

`wrapped_count_delta(current, previous)` 的作用是：

**即使计数器发生了这种跳变，也能算出车轮这一次真正增加或减少了多少个计数。**

它输出的不是编码器总计数，而是“这次和上次相比变化了多少”。

### 问题 2：中点法为什么除以 2？

假设机器人这一小段运动开始时朝向 30°，结束时朝向 50°，那么它在这段路程中的平均朝向大约是 40°：

```text
30° + (50° - 30°) / 2 = 40°
```

所以除以 2 是为了找到**这一小段运动过程的中间朝向**，用这个方向把路程分解到 x、y 轴。

机器人最终朝向仍然增加完整的 20°，并没有把实际转角减半。

### 问题 3：`base_link` 和 TF 是什么？

- `base_link` 是固定在机器人车身上的一个坐标系，可以把它想成贴在车身上的一张坐标纸。
- `odom` 是里程计建立的局部参考坐标系，可以把它想成机器人启动地点附近固定不动的一张坐标纸。
- TF 不是另一个坐标系。TF 是 ROS 2 中负责记录、发布和查询“不同坐标系之间位置关系”的机制。

本项目发布的核心关系是：

```text
odom  ──动态变换──>  base_link
固定参考坐标系          机器人车身坐标系
```

它表达的是：**此时此刻，机器人车身 `base_link` 在 `odom` 坐标系中的位置和方向是什么。**

---

# 第一部分：彻底理解 `wrapped_count_delta`

## 1.1 先理解“累计计数”和“本次增量”

编码器每转过一点就产生计数。程序收到的值通常是累计值，例如：

```text
时刻 1：1000
时刻 2：1015
时刻 3：1032
```

从时刻 1 到时刻 2，变化量为：

```text
1015 - 1000 = 15 个计数
```

从时刻 2 到时刻 3，变化量为：

```text
1032 - 1015 = 17 个计数
```

`wheel_odometry.py` 中的 `previous` 是上一帧累计计数，`current` 是当前帧累计计数，函数要返回本次增量。

正常情况下，直接写下面这句似乎就够了：

```python
delta = current - previous
```

但是 32 位整数有边界，跨过边界时直接相减就会出错。

## 1.2 为什么计数会从最大值突然跳到最小值

本项目把编码器累计计数理解为 32 位有符号整数，它的范围是：

```text
最小值：-2,147,483,648
最大值： 2,147,483,647
```

最大值再增加 1 时，因为 32 位空间已经用完，数值会回到最小值：

```text
2,147,483,646
2,147,483,647
-2,147,483,648   ← 发生回绕
-2,147,483,647
```

这和时钟很像：23:59 的下一分钟是 00:00，而不是 24:00。也像汽车机械里程表从 99999 继续走会回到 00000。

如果上一帧是最大值，当前帧是最小值，车轮实际上只前进了 1 个计数。但直接相减会得到：

```text
-2,147,483,648 - 2,147,483,647
= -4,294,967,295
```

这个结果显然不代表车轮突然反转了几十亿个计数。

## 1.3 先用 8 位小例子理解“环”

32 位数字太大，我们先把问题缩小成 8 位有符号整数。8 位有符号整数范围为：

```text
-128 ～ 127
```

它不是一条无限延伸的直线，而可以想成一个首尾连接的圆环：

```text
... 124 → 125 → 126 → 127 → -128 → -127 → -126 ...
```

如果：

```text
previous = 127
current  = -128
```

沿着圆环向前只走了一格，因此正确增量是 `+1`。

反过来，如果：

```text
previous = -128
current  = 127
```

沿着圆环向后只走了一格，因此正确增量是 `-1`。

`wrapped_count_delta` 做的就是在这个首尾相连的数字环上寻找正确的短距离。

## 1.4 项目里的实际函数

代码是：

```python
@staticmethod
def wrapped_count_delta(current: int, previous: int) -> int:
    return ((current - previous + (1 << 31)) % (1 << 32)) - (1 << 31)
```

`@staticmethod` 只说明这个函数不需要使用对象中的其他变量；理解计算公式时可以暂时忽略它。

`current: int`、`previous: int` 和最后的 `-> int` 是类型提示，表示输入和输出预计都是整数，不会改变计算过程。

## 1.5 `1 << 31` 和 `1 << 32` 是什么

`<<` 是二进制左移运算。对数字 1 左移 n 位，相当于乘以 `2^n`：

```text
1 << 3  = 2³  = 8
1 << 31 = 2³¹ = 2,147,483,648
1 << 32 = 2³² = 4,294,967,296
```

为了便于阅读，可以把原函数临时改写成：

```python
HALF_RANGE = 2147483648       # 2^31
FULL_RANGE = 4294967296       # 2^32

delta = current - previous
delta = delta + HALF_RANGE
delta = delta % FULL_RANGE
delta = delta - HALF_RANGE
return delta
```

这里的 `%` 是取模运算。在 Python 中，`a % 4294967296` 会把结果整理到 `0 ～ 4294967295` 这个区间。

最后再减去 `2147483648`，就把它平移到：

```text
-2,147,483,648 ～ 2,147,483,647
```

也就是一个正常的 32 位有符号增量范围。

## 1.6 把公式拆成四步操作

原公式：

```python
((current - previous + (1 << 31)) % (1 << 32)) - (1 << 31)
```

可以按下面四步从最里面向外看：

1. `current - previous`：先计算最原始的差。
2. `+ 2^31`：把希望保留的有符号区间整体向右移动。
3. `% 2^32`：把超出一整圈的部分折回 32 位计数环。
4. `- 2^31`：再把结果移回有正有负的范围。

它的本质是：**把原始差值规范到 `[-2^31, 2^31 - 1]` 这个区间。**

## 1.7 四个实际例子

### 例 1：正常向前

```text
previous = 1000
current  = 1015
结果     = +15
```

没有跨边界，函数返回普通相减结果。

### 例 2：正常向后

```text
previous = 1000
current  = 980
结果     = -20
```

负数表示编码器向反方向变化。

### 例 3：正向跨过最大值

```text
previous =  2,147,483,647
current  = -2,147,483,648
结果     = +1
```

虽然数值表面上从很大的正数跳到了很小的负数，但函数知道这只是沿计数环向前走了一格。

### 例 4：反向跨过最小值

```text
previous = -2,147,483,648
current  =  2,147,483,647
结果     = -1
```

这次是沿计数环向后走了一格。

## 1.8 更容易看懂的等价写法

如果暂时不喜欢一行公式，可以把它理解成下面这段判断代码：

```python
def wrapped_count_delta_easy(current: int, previous: int) -> int:
    delta = current - previous

    # 差值大得不合理，说明发生了“从最小值跳到最大值”的反向回绕。
    if delta > 2147483647:
        delta = delta - 4294967296

    # 差值小得不合理，说明发生了“从最大值跳到最小值”的正向回绕。
    elif delta < -2147483648:
        delta = delta + 4294967296

    return delta
```

这一版本和项目中的取模公式表达的是同一个思想。项目使用一行公式，是因为它不需要写两个分支。

## 1.9 函数在里程计流程中的位置

它只负责把累计计数变成本周期计数增量，后面的流程才把增量变成距离和位姿：

```text
当前左右轮累计计数
        │
        ▼
wrapped_count_delta
        │
        ▼
左右轮本周期计数增量
        │ × 每个计数对应的米数
        ▼
左右轮本周期行驶距离
        │
        ▼
差速底盘运动学
        │
        ▼
x、y、yaw
```

对应代码关系是：

```python
left_delta = self.wrapped_count_delta(left_count, self._previous_left_count)
right_delta = self.wrapped_count_delta(right_count, self._previous_right_count)

meters_per_count = math.pi * wheel_diameter_m / counts_per_wheel_rev
left_distance = left_delta * meters_per_count
right_distance = right_delta * meters_per_count
```

其中：

```text
车轮一圈的周长 = π × 车轮直径
每个计数对应的距离 = 车轮一圈的周长 ÷ 车轮一圈的编码器计数
```

## 1.10 这个函数有一个重要前提

它默认相邻两次采样之间的真实变化量小于半圈数字范围，也就是小于 `2^31` 个计数。

这是合理的，因为正常控制周期只有几毫秒或几十毫秒，车轮不可能在一次采样间隔内走几十亿个计数。

如果通信断开非常久，或者编码器计数突然被下位机清零，那么这不再只是普通回绕问题，系统还需要通过超时、重新初始化或故障状态单独处理。

---

# 第二部分：彻底理解里程计中的中点法

## 2.1 先看项目中的五行核心代码

```python
center_distance = 0.5 * (left_distance + right_distance)
yaw_delta = (right_distance - left_distance) / wheel_track_m
middle_yaw = self.yaw_rad + 0.5 * yaw_delta

self.x_m += center_distance * math.cos(middle_yaw)
self.y_m += center_distance * math.sin(middle_yaw)
```

随后程序才更新最终朝向：

```python
self.yaw_rad = math.atan2(
    math.sin(self.yaw_rad + yaw_delta),
    math.cos(self.yaw_rad + yaw_delta),
)
```

先记住两个不同的量：

```text
middle_yaw                 用于估计这段路程朝哪个方向走
self.yaw_rad + yaw_delta   机器人走完这段路之后的最终朝向
```

`middle_yaw` 只是计算 x、y 时临时使用的方向，不是最终保存的方向。

## 2.2 左右轮距离为什么能算出车体运动

这是差速底盘：左轮和右轮可以走不同距离。

设：

```text
dL = left_distance       左轮本周期行驶距离
dR = right_distance      右轮本周期行驶距离
L  = wheel_track_m       左右轮接地点之间的轮距
```

车体中心大约前进：

```text
center_distance = (dL + dR) / 2
```

车体朝向变化：

```text
yaw_delta = (dR - dL) / L
```

为什么是右轮减左轮？

- 两轮走得一样远，`dR - dL = 0`，车体不转弯。
- 右轮比左轮走得远，右侧推得更多，机器人向左转，按照 ROS 常用约定 yaw 增加。
- 左轮比右轮走得远，机器人向右转，yaw 减小。

这里的 `yaw_delta` 单位是弧度，因为“距离差 ÷ 轮距”的两个长度单位会相互约掉。

## 2.3 如果边走边转，不能只用起始角度

假设机器人在一个周期中：

```text
开始朝向 = 30°
本次转角 = 20°
结束朝向 = 50°
本次中心前进距离 = 0.10 m
```

真实运动不是先沿 30° 走完 0.10 m，再原地转到 50°。它是在前进的同时逐渐从 30° 转向 50°，轨迹是一小段圆弧。

如果全程都用开始角 30° 计算，会把运动方向估得偏早；如果全程都用结束角 50° 计算，又会估得偏晚。

中点法选择二者中间的 40°，作为这一小段圆弧的代表方向。

```text
开始                            结束
30° ----------- 40° ----------- 50°
 ^               ^               ^
旧 yaw        中点 yaw        新 yaw
```

因此：

```text
中点角度 = (开始角度 + 结束角度) / 2
         = (旧 yaw + 旧 yaw + yaw_delta) / 2
         = 旧 yaw + yaw_delta / 2
```

这就是代码中：

```python
middle_yaw = self.yaw_rad + 0.5 * yaw_delta
```

角度除以 2 的完整原因。

## 2.4 除以 2 没有把机器人转角减半

这是最容易误解的地方。

在刚才的例子中：

```text
旧 yaw       = 30°
yaw_delta    = 20°
middle_yaw   = 30° + 20° / 2 = 40°
最终新 yaw   = 30° + 20°     = 50°
```

程序计算 x、y 时使用 40°，但更新最终朝向时仍然加完整的 20°。

换成项目里的代码就是：

```python
# 只在计算这一段位移方向时使用一半增量
middle_yaw = self.yaw_rad + 0.5 * yaw_delta

# 保存新方向时使用完整增量
self.yaw_rad = self.yaw_rad + yaw_delta
```

实际代码外面又套了 `sin`、`cos`、`atan2`，是为了把角度限制在常用范围，原理仍然是加完整的 `yaw_delta`。

## 2.5 用具体数字算一次 x、y

继续使用：

```text
center_distance = 0.10 m
middle_yaw      = 40°
```

把 0.10 m 分解到 x、y 方向：

```text
Δx = 0.10 × cos(40°) ≈ 0.0766 m
Δy = 0.10 × sin(40°) ≈ 0.0643 m
```

假设原来 `x = 1.0000 m`、`y = 2.0000 m`，更新后大约是：

```text
x = 1.0000 + 0.0766 = 1.0766 m
y = 2.0000 + 0.0643 = 2.0643 m
```

最终朝向则是 50°。

注意，Python 的 `math.sin()` 和 `math.cos()` 使用弧度。上面用角度只是为了方便人理解；代码中的 `yaw_rad` 本来就是弧度。

## 2.6 三种特殊运动情况

### 情况 A：两轮等速直行

```text
dL = 0.10 m
dR = 0.10 m
center_distance = 0.10 m
yaw_delta = 0
```

因为没有转弯，中点方向与起始方向相同。

### 情况 B：原地旋转

```text
dL = -0.05 m
dR =  0.05 m
center_distance = 0
yaw_delta 不等于 0
```

此时 x、y 不变化，只有 yaw 变化。这正好符合原地转向。

### 情况 C：边前进边转弯

```text
dL = 0.08 m
dR = 0.12 m
```

机器人中心前进 0.10 m，同时因为右轮多走 0.04 m 而产生转角。这时中点法最有意义。

## 2.7 中点法与精确圆弧公式的关系

严格地说，差速车在一个周期内左右轮速度不变时，车体中心走的是圆弧。精确积分公式为：

```text
Δx = s / Δθ × [sin(θ + Δθ) - sin(θ)]
Δy = s / Δθ × [-cos(θ + Δθ) + cos(θ)]
```

其中 `s` 是中心距离，`θ` 是旧 yaw，`Δθ` 是本次 yaw 变化。

当控制周期较短、每次转角较小时，下面的中点近似非常接近精确圆弧结果：

```text
Δx ≈ s × cos(θ + Δθ/2)
Δy ≈ s × sin(θ + Δθ/2)
```

项目选用中点法，是因为公式简单、计算稳定，而且适合高频更新的移动机器人里程计。

## 2.8 为什么不使用时间 `dt` 也能更新位置

这里累加的是编码器在本周期实际走过的距离，所以更新位置时直接使用 `left_distance`、`right_distance`，不需要再乘时间。

如果要计算速度，才需要：

```text
线速度 = 本周期中心距离 ÷ dt
角速度 = 本周期角度变化 ÷ dt
```

不要把“距离积分”和“速度计算”混在一起。

## 2.9 另一个角度除以 2：四元数中的 `yaw / 2`

项目发布姿态时还会看到类似：

```python
quaternion_z = math.sin(yaw / 2.0)
quaternion_w = math.cos(yaw / 2.0)
```

这也出现了除以 2，但原因和中点法完全不同。

- 中点法中的 `yaw_delta / 2`：为了取得一段运动的中间朝向。
- 四元数中的 `yaw / 2`：这是欧拉角转换为单位四元数时由四元数数学定义产生的半角公式。

四元数半角并不表示机器人只旋转了一半。ROS 中最终仍然能从这个四元数还原出完整 yaw。

初学阶段只需把二者分开记忆，不必马上推导四元数数学。

---

# 第三部分：`base_link`、`odom` 和 TF 到底是什么

## 3.1 什么是坐标系

在数学中，只说“一个点在 (1, 2)”是不完整的，因为必须先说明这个位置是相对于哪套坐标轴测量的。

例如，同一个杯子：

- 相对于桌子左下角，可能在 `(1.0 m, 0.5 m)`。
- 相对于机器人中心，可能在 `(0.3 m, -0.2 m)`。

杯子的物理位置没有变，数值不同是因为参考坐标系不同。

一个三维坐标系通常有：

- 一个原点；
- x、y、z 三根轴；
- 一个方向；
- 一个名称。

ROS 中机器人各部分、传感器和环境参考都用有名称的坐标系表示。

## 3.2 `base_link` 是什么

`base_link` 是移动机器人最常用的车身主体坐标系名称。

你可以想象在机器人底盘上牢牢贴了一张坐标纸。这张坐标纸会跟着车一起平移、旋转，通常约定：

```text
x 轴：指向机器人正前方
y 轴：指向机器人左侧
z 轴：指向机器人上方
```

俯视示意：

```text
                 x（前）
                 ↑
                 │
        y（左）←  ●  base_link 原点
```

按照右手坐标系，机器人向左转时 yaw 通常为正，向右转时 yaw 通常为负。

### `base_link` 的原点究竟在车上哪里

名字本身不会自动决定物理位置。工程中一般把原点放在：

- 左右驱动轮轴线的中点；或
- 机器人几何中心附近；或
- 机器人模型规定的底盘参考点。

本项目的差速运动学默认位姿代表左右驱动轮之间的车体参考点。以后建立 URDF/Xacro 机器人模型时，必须让模型中的 `base_link` 原点与这个约定一致，否则激光雷达、IMU、轮子和导航会出现位置偏差。

一句话记忆：

**`base_link` 是贴在机器人主体上、会随车运动的坐标系。**

## 3.3 `odom` 是什么

`odom` 是 odometry 的缩写，是编码器里程计使用的局部参考坐标系。

可以把机器人开始计算里程计时的位置暂时当成：

```text
x = 0
y = 0
yaw = 0
```

之后每收到一组左右轮计数，程序就在这个参考系中累计机器人的位置和方向。

和 `base_link` 不同，`odom` 不跟着机器人车身移动；`base_link` 会在 `odom` 中运动。

```text
odom：像铺在地面上的坐标纸
base_link：像贴在车身上的坐标纸
```

### `odom` 是不是世界地图坐标系

不是。编码器会受到轮胎打滑、轮径误差、地面不平和累计误差影响，所以 `odom` 一般具有两个特点：

- 短时间内运动连续，不应该无故瞬间跳变；
- 长时间会逐渐漂移，不能代表绝对世界位置。

如果以后加入激光 SLAM 或定位，通常还会出现 `map` 坐标系：

```text
map → odom → base_link
```

其中 `map` 负责全局位置，`odom` 保留局部连续运动，`base_link` 代表车身。

## 3.4 “`base_link` 在 `odom` 中的位姿”是什么意思

假设里程计给出：

```text
x = 1.2 m
y = 0.4 m
yaw = 30°
```

完整说法是：

**机器人车身坐标系 `base_link` 的原点，在 `odom` 坐标系中位于 `(1.2 m, 0.4 m)`，并且相对于 `odom` 旋转了 30°。**

同一时刻，如果某物体在 `base_link` 前方 0.5 m，TF 就可以帮助 ROS 把这个点换算成它在 `odom` 中的坐标。

这正是坐标变换的用途。

## 3.5 TF 到底是什么

TF 可以理解为 ROS 2 的“坐标系关系管理系统”。它负责：

1. 发布坐标系之间的平移和旋转关系；
2. 给每个关系附上时间戳；
3. 把多个关系连接成一棵坐标树；
4. 帮程序查询某个时间点两个坐标系之间的变换。

因此：

```text
base_link 是一个坐标系名称。
odom 是另一个坐标系名称。
TF 是保存和查询它们之间关系的机制。
```

不能把 TF 和 `base_link` 并列理解成两个坐标系。

## 3.6 TF 中的一条关系包含什么

一条变换至少包含：

```text
父坐标系 parent frame
子坐标系 child frame
平移 translation：x、y、z
旋转 rotation：通常用四元数 x、y、z、w
时间戳 timestamp
```

本项目发布：

```text
父坐标系：odom
子坐标系：base_link
```

读法是：给出 `base_link` 相对于 `odom` 的位置和方向。

箭头常写成：

```text
odom → base_link
```

这里的父子并不是说二者物理上有亲属关系，而是用来构建一棵不能随意成环的坐标树。

## 3.7 项目里怎样发布 TF

节点先创建广播器：

```python
self._tf_broadcaster = TransformBroadcaster(self)
```

得到里程计结果后，创建一条变换消息：

```python
transform = TransformStamped()
transform.header.stamp = stamp
transform.header.frame_id = odom_frame
transform.child_frame_id = base_frame

transform.transform.translation.x = sample.x_m
transform.transform.translation.y = sample.y_m
transform.transform.rotation.z = quaternion_z
transform.transform.rotation.w = quaternion_w

self._tf_broadcaster.sendTransform(transform)
```

逐句含义如下：

- `header.stamp`：这条位置关系对应什么时间；
- `header.frame_id`：父坐标系，这里通常是 `odom`；
- `child_frame_id`：子坐标系，这里通常是 `base_link`；
- `translation.x/y`：车体在 `odom` 中的位置；
- `rotation.z/w`：车体朝向对应的四元数；
- `sendTransform()`：把关系交给 TF 系统发布。

配置文件中的关键参数是：

```yaml
odom_frame: odom
base_frame: base_link
publish_tf: true
```

如果 `publish_tf` 为 `false`，节点仍可发布 `/odom` 话题，但不会发送 `odom → base_link` 的 TF。

## 3.8 `/odom` 话题和 TF 有什么区别

二者经常携带相似的位置和方向，但用途不同。

| 对比项 | `/odom` 话题 | TF 变换 |
|---|---|---|
| 主要作用 | 发布里程计数据 | 建立坐标系之间的空间关系 |
| 典型内容 | 位姿、线速度、角速度、协方差 | 父子坐标系、平移、旋转、时间 |
| 常见使用者 | 控制、融合、记录、诊断节点 | RViz、导航、传感器数据坐标转换 |
| 本项目关系 | 表示车体里程计状态 | 发布 `odom → base_link` |

可以把它们理解成：

- `/odom` 是一份较完整的“里程计测量报告”；
- TF 是一份给整个 ROS 坐标系统使用的“坐标关系”。

它们不是简单的二选一。移动机器人通常会同时发布 `/odom` 和对应 TF。

## 3.9 动态 TF 和静态 TF

### 动态 TF

会随时间变化，需要持续发布。例如机器人一直在移动，所以：

```text
odom → base_link
```

是动态 TF，本项目会随编码器数据更新它。

### 静态 TF

机器人装配完成后基本不变，只需表达固定安装关系。例如 IMU 固定在底盘上：

```text
base_link → imu_link
```

激光雷达固定在底盘上：

```text
base_link → laser_link
```

这些关系通常由 URDF 的 `robot_state_publisher` 或静态 TF 发布器提供，而不是编码器里程计负责。

一个更完整的机器人 TF 树可能是：

```text
map
 └── odom
      └── base_link
           ├── imu_link
           ├── laser_link
           └── camera_link
```

其中只有一部分属于当前编码器里程计模块，不要认为一个 Python 文件必须发布所有坐标关系。

## 3.10 TF 为什么需要时间戳

机器人在运动，同一传感器点在不同时刻换算出来的位置可能不同。

例如激光雷达在 10:00:00.100 扫到一个点，ROS 应当使用接近那个时刻的机器人位姿，而不是使用一秒后的位姿。因此 TF 不只记“在哪里”，还要记“什么时候在哪里”。

传感器消息时间和 TF 时间不匹配时，导航或 RViz 可能出现 extrapolation、lookup timeout 等报错。

## 3.11 在 RDK X5 上实际查看

以下命令假设已经编译工作空间。每次打开新终端，先加载 ROS 2 和本项目环境：

```bash
source /opt/ros/humble/setup.bash
source ~/chassis_project/ros2_ws/install/setup.bash
```

如果你的项目实际目录不是 `~/chassis_project/ros2_ws`，第二行要换成实际路径。

### 查看 `odom → base_link` 的实时关系

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

正常运行时，会不断显示 translation 和 rotation。推动小车或让车轮转动后，translation 或 rotation 应当发生变化。

如果显示查不到变换，依次检查：

1. 底盘 ROS 2 节点是否正在运行；
2. 串口心跳和编码器反馈是否正常；
3. 参数 `publish_tf` 是否为 `true`；
4. 实际配置的 frame 名是否真的是 `odom` 和 `base_link`。

### 查看 `/odom` 消息

```bash
ros2 topic echo --once /odom
```

持续查看可去掉 `--once`：

```bash
ros2 topic echo /odom
```

按 `Ctrl+C` 结束持续显示。这里的 `Ctrl+C` 是终止当前前台命令，不是复制。

### 查看 TF 底层话题

```bash
ros2 topic echo --once /tf
```

动态 TF 通过 `/tf` 传输，静态 TF 通常通过 `/tf_static` 传输：

```bash
ros2 topic echo --once /tf_static
```

刚开始学习时，优先使用 `tf2_echo odom base_link`，因为它比直接看四元数消息更容易理解。

## 3.12 一个传感器坐标转换例子

假设将来激光雷达坐标系叫 `laser_link`，机器人系统里有：

```text
odom → base_link → laser_link
```

虽然里程计模块并没有直接发布 `odom → laser_link`，TF 仍然可以把两段关系连接起来：

```text
odom → base_link
加上
base_link → laser_link
得到
odom → laser_link
```

这样激光雷达在自身坐标系测到的障碍物点，就能转换到 `odom` 坐标系中，用来画图或导航。

这也是为什么机器人项目不能只有 x、y 数字，还需要维护一棵一致的 TF 树。

---

# 第四部分：把三个问题连成完整数据流

现在把所有概念放回项目：

```text
STM32 发来左右轮累计编码器计数
              │
              ▼
wrapped_count_delta
处理 32 位回绕，得到本周期左右轮计数增量
              │
              ▼
计数增量 × 每计数米数
得到 left_distance、right_distance
              │
              ▼
差速运动学
得到 center_distance、yaw_delta
              │
              ▼
中点法
用 old_yaw + yaw_delta/2 估计本段位移方向
              │
              ▼
累计得到 x、y、最终 yaw
              │
              ├──────────────┐
              ▼              ▼
发布 /odom 消息       发布 odom → base_link 的 TF
里程计测量报告        ROS 坐标系关系
```

## 4.1 一句话分别记忆

- `wrapped_count_delta`：解决有限位数编码器累计计数跨边界时的增量计算。
- `middle_yaw`：用一段运动的中间朝向估计这段圆弧位移的方向。
- `base_link`：固定在机器人车身上的坐标系。
- `odom`：由轮式里程计建立的局部连续参考坐标系。
- TF：发布、保存和查询坐标系之间时变关系的 ROS 2 机制。

## 4.2 最容易犯的五个错误

1. 把 `wrapped_count_delta` 当成计数清零函数。它不会修改编码器，只计算两帧的正确差值。
2. 认为 `yaw_delta / 2` 会让最终转角少一半。它只用于计算中点方向，最终 yaw 仍加完整增量。
3. 把中点法的除以 2 和四元数半角混为一谈。二者出现位置和原因完全不同。
4. 把 `base_link` 当成一个 ROS 节点或话题。它只是坐标系名称。
5. 把 TF 当成坐标系。TF 是管理多个坐标系关系的系统。

## 4.3 建议你按这个顺序对照代码

1. 在 `wheel_odometry.py` 找到 `wrapped_count_delta()`，用正常、正向回绕、反向回绕三个例子手算。
2. 接着找 `left_distance`、`right_distance`，确认“计数如何变成米”。
3. 找 `center_distance`、`yaw_delta` 和 `middle_yaw`，用 30° 到 50° 的例子代入。
4. 找 x、y、yaw 的更新代码，确认中点角只用于 x、y，而最终 yaw 使用完整增量。
5. 在 `chassis_can_node.py` 找 `TransformStamped` 和 `sendTransform()`，逐项对应父坐标系、子坐标系、平移、旋转和时间戳。
6. 启动节点后运行 `tf2_echo odom base_link`，把终端显示和代码变量联系起来。

完成这六步后，这三个看似分散的问题就会变成同一条清晰的数据链：编码器数字先变成位移，位移再变成车体位姿，位姿最后通过 `/odom` 和 TF 提供给 ROS 2 其他模块。

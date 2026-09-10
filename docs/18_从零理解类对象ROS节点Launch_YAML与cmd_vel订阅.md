# 从零理解类、对象、ROS 2 节点、Launch、YAML 和 `/cmd_vel` 订阅

## 0. 这份文档解决什么问题

这份文档只解决下面四个基础问题：

1. Python 的类、对象和 `self` 到底是什么；
2. Python 对象和 ROS 2 节点是什么关系；
3. Launch 文件和 YAML 文件分别干什么；
4. 下面这行代码每一部分是什么意思，`/cmd_vel` 到底在哪里创建：

```python
self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
```

本文暂时不讲电机 PI、串口协议和里程计公式。先把主节点的“骨架”理解清楚，再学习后面的功能。

---

# 第一部分：先完全离开 ROS，理解 Python 的类和对象

## 1. 普通变量只能保存一个值

例如：

```python
car_name = "小车A"
car_speed = 0.0
car_enabled = False
```

这三个变量分别表示：

```text
小车名称
小车速度
小车是否使能
```

如果只有一辆小车，这样写暂时可以。

如果有两辆小车，就会变成：

```python
car1_name = "小车A"
car1_speed = 0.0
car1_enabled = False

car2_name = "小车B"
car2_speed = 0.0
car2_enabled = False
```

小车越来越多，变量会越来越混乱。因此我们需要把“同一辆小车的数据和功能”组织在一起。

## 2. 类是设计说明，不是一辆真正存在的小车

先看最简单的类：

```python
class Car:
    pass
```

`class Car:` 表示定义一个名为 `Car` 的类。

现在只有一份“什么是 Car”的设计说明，还没有真正创建任何小车。

可以把它类比成：

```text
类 Car = 小车设计图
```

设计图不是实物。即使设计图写好了，桌面上也不会自动出现一辆车。

## 3. 对象是根据类创建出来的具体实例

执行：

```python
car1 = Car()
```

这句话可以拆成两部分：

```text
Car()     根据 Car 这张设计图创建一个对象
car1 =    用变量 car1 保存这个对象
```

现在：

```text
Car   是类，是设计图
car1  是对象，是根据设计图创建出的具体实例
```

再执行：

```python
car2 = Car()
```

就创建了第二个对象。

```text
Car 类
 ├── car1 对象
 └── car2 对象
```

`car1` 和 `car2` 使用同一份类定义，但它们是两个不同对象。

## 4. `__init__()` 是对象创建时自动执行的初始化函数

修改类：

```python
class Car:
    def __init__(self):
        print("正在初始化一辆小车")
```

执行：

```python
car1 = Car()
```

Python 会自动调用：

```python
Car.__init__(car1)
```

因此终端会显示：

```text
正在初始化一辆小车
```

你一般不需要自己直接调用 `__init__()`。执行 `Car()` 时，Python 自动完成对象创建和初始化。

## 5. `self` 就是“当前正在操作的这个对象”

这是最关键的一点。

看代码：

```python
class Car:
    def __init__(self, name):
        self.name = name
        self.speed = 0.0
```

创建两个对象：

```python
car1 = Car("小车A")
car2 = Car("小车B")
```

创建 `car1` 时，`self` 指向 `car1`：

```text
self.name = name
等价理解为
car1.name = "小车A"
```

创建 `car2` 时，`self` 指向 `car2`：

```text
self.name = name
等价理解为
car2.name = "小车B"
```

因此：

```python
print(car1.name)  # 小车A
print(car2.name)  # 小车B
```

可以先把 `self.xxx` 牢牢记成一句话：

```text
self.xxx = 当前这个对象自己的 xxx
```

## 6. 为什么函数参数里必须写 `self`

继续定义一个方法：

```python
class Car:
    def __init__(self, name):
        self.name = name
        self.speed = 0.0

    def set_speed(self, new_speed):
        self.speed = new_speed
```

调用：

```python
car1.set_speed(0.5)
```

Python 在内部可以近似理解成：

```python
Car.set_speed(car1, 0.5)
```

所以：

```text
self      接收到 car1
new_speed 接收到 0.5
```

函数内部：

```python
self.speed = new_speed
```

就相当于：

```python
car1.speed = 0.5
```

如果调用：

```python
car2.set_speed(0.2)
```

此时 `self` 就是 `car2`，不会修改 `car1`。

## 7. 对象属性与局部变量的区别

看这个方法：

```python
def set_speed(self, new_speed):
    old_speed = self.speed
    self.speed = new_speed
```

这里有两种变量：

```text
old_speed       局部变量，只在这次函数执行期间使用
self.speed      对象属性，函数结束后仍保存在对象中
```

底盘节点为什么有大量 `self.xxx`？

因为节点要长期记住：

```text
最新速度命令
是否已经使能
最后收到命令的时间
最新心跳
最新编码器数据
发布者对象
串口对象
```

这些数据不能在一个函数结束后消失，所以保存在节点对象内部。

## 8. 方法后面有括号和没有括号的区别

假设：

```python
car1.set_speed
```

没有括号，表示“取得这个方法本身”，但暂时不执行。

```python
car1.set_speed(0.5)
```

有括号，表示现在立即调用方法，并传入 `0.5`。

这个区别对 ROS 回调非常重要：

```python
self._on_cmd_vel
```

表示把函数本身交给 ROS 保存，等以后收到消息时再调用。

如果错误地写成：

```python
self._on_cmd_vel()
```

就变成初始化时立即调用，而且没有传入消息，会报错。

---

# 第二部分：普通 Python 对象怎样成为 ROS 2 节点

## 9. ROS 2 节点是什么

ROS 2 节点首先仍然是一个运行中的程序或程序对象。

它与普通 Python 对象相比，多了 ROS 通信能力，例如：

```text
能够发布话题
能够订阅话题
能够提供服务
能够读取 ROS 参数
能够创建定时器
能够出现在 ROS 计算图中
```

本项目中的 ROS 节点名是：

```text
/chassis_can_node
```

## 10. ROS 2 已经提供了一个基础 `Node` 类

代码导入：

```python
from rclpy.node import Node
```

ROS 2 的开发者已经写好了 `Node` 类，它包含：

```python
create_publisher()
create_subscription()
create_service()
create_timer()
declare_parameter()
get_logger()
```

我们不需要从零重新实现 ROS 网络通信，而是在这个基础类上增加底盘功能。

## 11. `class ChassisCanNode(Node)` 是继承

项目代码：

```python
class ChassisCanNode(Node):
```

它表示：

```text
定义一个新类 ChassisCanNode
它继承 ROS 2 的 Node 类
```

可以把它理解成：

```text
Node 提供通用 ROS 能力
             +
ChassisCanNode 增加底盘专用功能
             =
一个能与 STM32 控制底盘通信的 ROS 节点类
```

继承后，`ChassisCanNode` 对象既能使用我们自己写的方法：

```python
self._on_cmd_vel()
self._send_motion_command()
```

也能使用从 `Node` 继承来的方法：

```python
self.create_subscription()
self.create_publisher()
```

## 12. `super().__init__("chassis_can_node")` 是什么

代码：

```python
class ChassisCanNode(Node):
    def __init__(self):
        super().__init__("chassis_can_node")
```

我们的 `ChassisCanNode` 有自己的初始化函数，但父类 `Node` 也有初始化函数。

```python
super().__init__("chassis_can_node")
```

表示先调用父类 `Node` 的初始化，并把 ROS 节点名称设为：

```text
chassis_can_node
```

执行完成以后，当前对象才真正具备完整的 ROS 节点底层能力。

如果省略这一步，后面的 `create_subscription()` 没有正确初始化的 ROS 节点可以使用。

## 13. 节点类、节点对象和 ROS 图中的节点要区分

这三个概念名字相近，但不完全一样。

### 13.1 节点类

```python
class ChassisCanNode(Node):
```

这是代码中的设计图。

### 13.2 节点对象

```python
node = ChassisCanNode()
```

这是程序运行时在内存里创建出的具体 Python 对象。

### 13.3 ROS 图中的节点

对象初始化时执行：

```python
super().__init__("chassis_can_node")
```

它把这个运行对象注册进 ROS 通信系统，因此执行：

```bash
ros2 node list
```

可以看到：

```text
/chassis_can_node
```

整个关系是：

```text
ChassisCanNode 类（设计图）
          ↓ ChassisCanNode()
node 对象（Python 内存中的具体对象）
          ↓ Node 父类初始化
/chassis_can_node（ROS 图里可发现的节点）
```

## 14. 为什么主程序还要执行 `rclpy.spin(node)`

创建节点对象以后，只是完成了初始化。

ROS 还需要不断等待事件：

```text
有没有新的 /cmd_vel？
有没有服务请求？
定时器到时间了吗？
```

代码：

```python
rclpy.spin(node)
```

可以理解成：

```text
让这个节点一直运行，并不断处理 ROS 事件
```

如果没有 `spin()`，程序创建完对象后很快运行到结尾并退出，就没有机会等待以后到达的 `/cmd_vel`。

## 15. 回调函数是什么

普通调用是我们主动执行函数：

```python
car1.set_speed(0.5)
```

回调则是：

```text
我们先把函数交给某个系统保存
将来某个事件发生时，由系统调用这个函数
```

例如：

```python
self.create_subscription(
    Twist,
    "cmd_vel",
    self._on_cmd_vel,
    10,
)
```

这里把 `self._on_cmd_vel` 交给 ROS。

以后收到 `/cmd_vel` 消息时，ROS 近似执行：

```python
self._on_cmd_vel(收到的Twist消息)
```

所以 `_on_cmd_vel()` 是订阅回调。

---

# 第三部分：`/cmd_vel` 到底是谁创建的

## 16. ROS 话题不是普通 Python 变量

下面的写法不会出现在代码中：

```python
cmd_vel = 某个话题对象
```

ROS 话题不是要求某个中心程序先“创建一个文件或变量”，其他节点才能使用。

ROS 2 中，各节点创建的是通信端点：

```text
发布端 Publisher
订阅端 Subscription
```

只要发布端和订阅端的：

```text
话题名称相同
消息类型兼容
QoS 兼容
```

ROS 2/DDS 就会发现并连接它们。

## 17. 本节点创建的是 `/cmd_vel` 的订阅端

代码：

```python
self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
```

这行代码就是创建 `/cmd_vel` 订阅端的地方。

它不是创建发布端，也不是生成速度数据。

可以画成：

```text
底盘节点内部

create_subscription(...)
        ↓
创建一个订阅端
        ↓
等待别人发布 /cmd_vel
```

## 18. `/cmd_vel` 的发布端通常由其他节点创建

发布端可能来自：

```text
键盘遥控节点
导航节点
游戏手柄节点
你在终端执行的 ros2 topic pub
自己编写的速度发布节点
```

例如终端执行：

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.05}, angular: {z: 0.0}}"
```

这条 `ros2 topic pub` 命令会临时启动一个 ROS 节点，并创建 `/cmd_vel` 发布端。

于是形成：

```text
ros2 topic pub 创建的发布端
            │
            │ /cmd_vel，Twist
            ▼
ChassisCanNode 创建的订阅端
            │
            ▼
      _on_cmd_vel(message)
```

## 19. 如果只有订阅端、没有发布端会怎样

节点仍然可以正常运行。

它只是在等待消息：

```text
订阅已创建
但当前没有发布者
所以 _on_cmd_vel() 不会被调用
```

查看：

```bash
ros2 topic info /cmd_vel -v
```

可能看到：

```text
Publisher count: 0
Subscription count: 1
```

这表示：

```text
没有发布者
有一个订阅者，也就是底盘节点
```

启动速度发布命令后，会变成类似：

```text
Publisher count: 1
Subscription count: 1
```

## 20. 如果先启动发布者、后启动订阅者可以吗

可以。

ROS 2 使用分布式发现机制。发布者和订阅者的启动顺序通常不是固定的：

```text
发布者先启动 → 等待兼容订阅者出现
订阅者先启动 → 等待兼容发布者出现
```

双方出现并发现彼此后开始通信。

## 21. 为什么代码写 `"cmd_vel"`，命令却写 `/cmd_vel`

代码：

```python
"cmd_vel"
```

这是相对话题名。

当前节点没有额外命名空间，因此 ROS 将它解析成绝对名称：

```text
/cmd_vel
```

开头 `/` 表示从 ROS 根命名空间开始。

如果将节点放入命名空间 `/robot1`，相对名称可能解析为：

```text
/robot1/cmd_vel
```

相对名称便于一套代码给多台机器人复用。

---

# 第四部分：逐项拆解 `create_subscription(...)`

## 22. 原始代码

```python
self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
```

先按结构拆开：

```python
self.create_subscription(
    Twist,             # 第1个参数：消息类型
    "cmd_vel",         # 第2个参数：话题名称
    self._on_cmd_vel,  # 第3个参数：收到消息后调用的函数
    10,                # 第4个参数：QoS队列深度
)
```

## 23. `self.create_subscription` 是什么意思

`self` 是当前 `ChassisCanNode` 对象。

`create_subscription` 是从 ROS `Node` 父类继承来的方法。

组合起来就是：

```text
让当前底盘节点对象创建一个 ROS 订阅端
```

## 24. 第一个参数 `Twist`

顶部导入：

```python
from geometry_msgs.msg import Twist
```

`Twist` 是 ROS 2 已经定义好的消息类型，用于描述速度。

查看它的真实结构：

```bash
ros2 interface show geometry_msgs/msg/Twist
```

输出类似：

```text
Vector3 linear
    float64 x
    float64 y
    float64 z
Vector3 angular
    float64 x
    float64 y
    float64 z
```

因此一条 Twist 消息可以理解成：

```text
message.linear.x
message.linear.y
message.linear.z
message.angular.x
message.angular.y
message.angular.z
```

本项目只使用：

```text
linear.x   小车前后线速度，m/s
angular.z  小车绕竖直轴的角速度，rad/s
```

消息类型必须一致。发布者发布 `Twist`，订阅者也必须订阅 `Twist`，不能一边使用字符串、一边使用 Twist。

## 25. 第二个参数 `"cmd_vel"`

它是话题名称。

当前解析为：

```text
/cmd_vel
```

发布者要把速度送到本节点，也应使用同一个话题名称。

例如：

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist ...
```

## 26. 第三个参数 `self._on_cmd_vel`

这是收到消息后要调用的函数。

注意没有括号：

```python
self._on_cmd_vel
```

表示把函数交给 ROS 保存。

对应函数：

```python
def _on_cmd_vel(self, message: Twist) -> None:
    self._latest_command = message
    self._has_received_cmd_vel = True
    self._last_cmd_vel_time = time.monotonic()
```

收到消息时，ROS 会把消息作为 `message` 参数传入。

## 27. 第四个参数 `10`

这里是简写的 QoS 队列深度。

初学阶段可以先这样理解：

```text
当回调暂时来不及处理时，最多保留最近 10 条待处理消息
```

它不是：

```text
不是 10 Hz
不是发送 10 次
不是 10 秒超时
不是速度大小
```

发布频率由发布者决定，例如：

```bash
-r 10
```

才表示每秒发布 10 次。

严格来说，QoS 还包括可靠性、历史策略和持久性等设置。这里传入整数 10，ROS 使用默认 QoS 配置，并将历史深度设为 10。

## 28. 为什么没有用变量保存订阅对象

代码写成：

```python
self.create_subscription(...)
```

而不是：

```python
self._subscription = self.create_subscription(...)
```

在 `rclpy` 的 Node 实现中，节点会维护所创建实体的内部引用，因此当前代码能够工作。

不过对初学者而言，写成下面这样更直观：

```python
self._cmd_vel_subscription = self.create_subscription(
    Twist,
    "cmd_vel",
    self._on_cmd_vel,
    10,
)
```

它明确告诉读者：

```text
创建订阅后返回了一个 Subscription 对象
并把它保存在当前节点的 _cmd_vel_subscription 属性里
```

两种写法的订阅功能相同，后一种更适合学习和后续管理。

---

# 第五部分：收到 `/cmd_vel` 后到底发生什么

## 29. 回调的三行代码

```python
def _on_cmd_vel(self, message: Twist) -> None:
    self._latest_command = message
    self._has_received_cmd_vel = True
    self._last_cmd_vel_time = time.monotonic()
```

逐行解释。

### 29.1 保存整条消息

```python
self._latest_command = message
```

把最新 Twist 消息保存为节点对象属性，以便之后的定时器读取。

此时可以访问：

```python
self._latest_command.linear.x
self._latest_command.angular.z
```

### 29.2 标记已经收到过命令

```python
self._has_received_cmd_vel = True
```

节点启动时它是 `False`。收到第一条后变成 `True`。

这样节点可以区分：

```text
从来没有收到过速度命令
曾经收到过，但现在已经超时
```

### 29.3 记录接收时间

```python
self._last_cmd_vel_time = time.monotonic()
```

用于之后判断速度命令是否超过 `0.20 s` 没更新。

## 30. 为什么回调没有立刻发串口

回调只保存最新命令，发送由 50 Hz 定时器完成：

```python
self.create_timer(1.0 / command_rate, self._send_motion_command)
```

当前 `command_rate = 50`：

```text
周期 = 1 / 50 = 0.02 s = 20 ms
```

每 20 ms，ROS 调用：

```python
self._send_motion_command()
```

定时器读取回调保存的最新命令。

关系是：

```text
/cmd_vel 消息到达
       ↓
_on_cmd_vel() 保存最新消息和时间
       ↓
等待下一个 20 ms 定时器
       ↓
_send_motion_command() 检查安全条件
       ↓
编码并发送给 STM32
```

## 31. 发送前的三个安全条件

代码：

```python
if command_is_fresh and self._drive_enabled and not self._emergency_stop:
```

必须同时满足：

```text
最近 0.20 秒内有新的 /cmd_vel
软件使能已经打开
没有急停
```

才读取：

```python
linear = self._latest_command.linear.x
angular = self._latest_command.angular.z
```

否则强制：

```python
linear = 0.0
angular = 0.0
enable = False
```

因此仅仅发布 `/cmd_vel` 不会自动让电机运动，还需要调用使能服务。

## 32. 一条命令的完整时间顺序

假设节点已经启动并软件使能，终端发布 `0.05 m/s`：

```text
时刻1：ros2 topic pub 创建 /cmd_vel 发布端
时刻2：ROS发现发布端与底盘订阅端名称和类型匹配
时刻3：发布者发送 Twist 消息
时刻4：ROS执行器唤醒 _on_cmd_vel(message)
时刻5：回调保存 message 和接收时间
时刻6：20 ms 运动定时器到期
时刻7：检查命令新鲜、使能、急停
时刻8：读取 linear.x=0.05 和 angular.z=0
时刻9：can_protocol.py 编码逻辑帧
时刻10：serial_transport.py 包装并写串口
时刻11：STM32接收并执行电机控制
```

---

# 第六部分：Launch 文件从零解释

## 33. 不使用 Launch 也可以启动节点

ROS 节点编译安装后，可以直接运行：

```bash
ros2 run chassis_can_control chassis_can_node
```

但是这样没有自动指定本项目的 YAML 参数文件，可能使用 Python 中的默认值，例如默认 `transport` 是 `can`，不适合当前串口实车。

也可以手工带参数文件：

```bash
ros2 run chassis_can_control chassis_can_node \
--ros-args --params-file 某个参数文件.yaml
```

参数越来越多、节点越来越多以后，命令会很长。因此 ROS 提供 Launch 文件集中描述“怎样启动整套系统”。

## 34. Launch 文件是什么

Launch 文件是一份启动方案。

它可以描述：

```text
启动哪个 ROS 包中的哪个程序
节点名称是什么
给节点加载哪个参数文件
日志输出到哪里
是否启动多个节点
是否设置命名空间或重映射
```

当前文件：

```text
launch/chassis_serial.launch.py
```

它本身也是 Python 文件，但不是主节点业务代码。它的工作是“组织启动”，不是处理 `/cmd_vel` 或串口数据。

## 35. 当前 Launch 文件逐行解释

代码主体：

```python
def generate_launch_description():
    package_directory = get_package_share_directory(
        "chassis_can_control"
    )
    parameter_file = os.path.join(
        package_directory,
        "config",
        "chassis_serial.yaml",
    )

    return LaunchDescription([
        Node(
            package="chassis_can_control",
            executable="chassis_can_node",
            name="chassis_can_node",
            output="screen",
            parameters=[parameter_file],
        )
    ])
```

### 35.1 `generate_launch_description()`

这是 ROS Launch 系统约定调用的函数。它返回整套启动描述。

### 35.2 找到安装后的功能包目录

```python
get_package_share_directory("chassis_can_control")
```

返回类似：

```text
/home/wheeltec/chassis_project/ros2_ws/install/
chassis_can_control/share/chassis_can_control
```

### 35.3 拼接 YAML 路径

```python
os.path.join(package_directory, "config", "chassis_serial.yaml")
```

得到安装后的参数文件完整路径。

### 35.4 描述要启动的节点

```python
Node(
    package="chassis_can_control",
    executable="chassis_can_node",
    name="chassis_can_node",
    output="screen",
    parameters=[parameter_file],
)
```

每一项：

```text
package      ROS 功能包名
executable   setup.py 注册的可执行入口名
name         运行后的节点名
output       日志显示到终端
parameters   传给节点的参数文件列表
```

## 36. 执行 Launch 命令时发生什么

```bash
ros2 launch chassis_can_control chassis_serial.launch.py
```

过程：

```text
ros2 launch 找到功能包
       ↓
找到安装后的 launch 文件
       ↓
调用 generate_launch_description()
       ↓
找到安装后的 chassis_serial.yaml
       ↓
启动 chassis_can_node 可执行程序
       ↓
把 YAML 参数传给节点
       ↓
节点 __init__() 读取这些参数
```

## 37. Launch 文件与主节点不是谁调用谁的普通函数关系

不是主节点内部写：

```python
import chassis_serial.launch.py
```

而是外部 ROS Launch 系统：

```text
先读取 Launch 描述
再创建一个新的主节点进程
并把启动参数交给这个进程
```

Launch 像“导演和启动清单”，主节点才是“真正长期工作的演员”。

---

# 第七部分：YAML 文件从零解释

## 38. YAML 是什么

YAML 是一种用来保存配置数据的文本格式，不是 Python 程序。

它主要由：

```text
键: 值
缩进层级
```

组成。

简单例子：

```yaml
name: 小车A
speed: 0.05
enabled: false
```

对应的概念是：

```text
name 的值是字符串“小车A”
speed 的值是数字 0.05
enabled 的值是布尔值 false
```

## 39. YAML 的缩进非常重要

```yaml
car:
  name: 小车A
  speed: 0.05
```

`name` 和 `speed` 缩进在 `car` 下面，表示它们属于 `car`。

YAML 通常使用空格缩进，不要用 Tab 混排。

## 40. 当前 ROS 参数 YAML 的固定结构

```yaml
chassis_can_node:
  ros__parameters:
    transport: serial
    serial_port: /dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00
    serial_baud_rate: 115200
```

逐层解释：

### 40.1 `chassis_can_node:`

表示下面这组参数是给哪个节点的。

### 40.2 `ros__parameters:`

这是 ROS 2 参数文件的固定关键字。

### 40.3 具体参数

```yaml
transport: serial
```

告诉节点当前使用串口传输。

```yaml
serial_port: /dev/serial/by-id/...
```

告诉节点打开哪个 Linux 串口设备。

```yaml
serial_baud_rate: 115200
```

告诉节点串口速度。

## 41. YAML 参数怎样进入 `self.get_parameter()`

主节点先声明：

```python
self.declare_parameter("transport", "can")
```

这里 Python 默认值是 `can`。

Launch 又加载 YAML：

```yaml
transport: serial
```

创建节点时，YAML 覆盖默认值。

之后：

```python
self.get_parameter("transport").value
```

读到的是：

```text
serial
```

因此节点创建 `SerialTransport`，而不是 `SocketCanTransport`。

## 42. 为什么不把所有参数直接写死在 Python 中

如果全部写死，改变轮径或串口路径就需要修改代码。

分开后：

```text
Python 负责程序逻辑
YAML 负责可调整配置
Launch 负责启动时把二者连接起来
```

这种分工使同一份 Python 代码可以适配不同小车。

## 43. 为什么改完源码 YAML 还要重新编译

当前 Launch 使用的是功能包安装目录里的 YAML：

```text
install/chassis_can_control/share/
chassis_can_control/config/chassis_serial.yaml
```

你平时编辑的是源码：

```text
src/chassis_can_control/config/chassis_serial.yaml
```

`colcon build` 会把源码配置复制到安装目录。

因此修改源码 YAML 后要执行：

```bash
cd ~/chassis_project/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select chassis_can_control
source install/setup.bash
```

再停止旧节点并重新启动。

---

# 第八部分：把类、对象、Launch、YAML 和订阅连接起来

## 44. 一次完整启动过程

执行：

```bash
ros2 launch chassis_can_control chassis_serial.launch.py
```

完整过程：

```text
第1步：ROS读取 Launch 文件
第2步：Launch 找到 YAML 参数文件
第3步：Launch 启动 chassis_can_node 可执行程序
第4步：程序进入 main()
第5步：rclpy.init() 初始化 ROS Python 环境
第6步：ChassisCanNode() 根据类创建节点对象
第7步：__init__() 自动执行
第8步：super().__init__() 初始化 ROS Node 父类
第9步：YAML 覆盖声明的默认参数
第10步：按 transport=serial 创建串口对象
第11步：create_subscription() 创建 /cmd_vel 订阅端
第12步：创建发布者、服务和定时器
第13步：rclpy.spin(node) 开始等待事件
```

## 45. 一次完整 `/cmd_vel` 通信过程

另一个终端执行：

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.05}, angular: {z: 0.0}}"
```

过程：

```text
第1步：命令创建临时 ROS 发布节点
第2步：发布节点创建 /cmd_vel 的 Twist 发布端
第3步：ROS发现底盘节点已经有同名同类型订阅端
第4步：发布端每秒发 10 条 Twist
第5步：ROS将消息交给底盘节点
第6步：spin() 调度 _on_cmd_vel(message)
第7步：回调保存最新消息和接收时间
第8步：50 Hz 定时器检查安全条件
第9步：条件满足后编码并发给 STM32
```

## 46. 四个概念最终对应关系

```text
类 ChassisCanNode
    = 底盘节点的 Python 设计图

对象 node = ChassisCanNode()
    = 程序运行时根据设计图创建的具体对象

Launch 文件
    = 告诉 ROS 启动哪个程序、叫什么、加载哪个配置

YAML 文件
    = 保存串口、轮径、PI和安全阈值等配置值

create_subscription(...)
    = 让当前节点对象创建 /cmd_vel 的订阅端
```

---

# 第九部分：可以亲自验证的实验

以下实验只观察 ROS 通信。电机保持软件失能，不调用使能服务。

## 47. 加载环境

```bash
source /opt/ros/humble/setup.bash
source ~/chassis_project/ros2_ws/install/setup.bash
```

## 48. 确认底盘节点

```bash
ros2 node list
```

应看到：

```text
/chassis_can_node
```

## 49. 查看节点的订阅端

```bash
ros2 node info /chassis_can_node
```

在 `Subscribers` 下应看到：

```text
/cmd_vel: geometry_msgs/msg/Twist
```

这就是 `create_subscription()` 创建成功的证据。

## 50. 查看 `/cmd_vel` 当前两端数量

未启动发布命令时：

```bash
ros2 topic info /cmd_vel -v
```

观察：

```text
Publisher count
Subscription count
```

通常底盘节点已运行时，订阅数量至少为 1。

## 51. 创建一个发布端，但不使能电机

新终端同样加载 ROS 环境，再执行：

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.0}}"
```

这里只发布零速度，并且没有调用 `/chassis/enable`，电机不会因为这个实验运行。

再次查看：

```bash
ros2 topic info /cmd_vel -v
```

现在应能看到发布者数量增加。

按 `Ctrl+C` 停止临时发布端，再查看时发布者数量会减少。

## 52. 查看 Twist 的字段，而不是靠背诵

```bash
ros2 interface show geometry_msgs/msg/Twist
```

再查看一条正在发布的消息：

```bash
ros2 topic echo --once /cmd_vel
```

这样可以把消息类型定义与真实数据对应起来。

---

# 第十部分：最容易混淆的十个问题

## 53. `ChassisCanNode` 是节点吗

它首先是一个 Python 类，也就是节点设计图。`ChassisCanNode()` 创建对象并初始化父类后，这个运行对象才作为 ROS 节点参与通信。

## 54. `node` 变量是 ROS 节点名称吗

不是。

```python
node = ChassisCanNode()
```

`node` 只是 Python 变量名。

ROS 节点名称来自：

```python
super().__init__("chassis_can_node")
```

## 55. `self` 是 ROS 特有的吗

不是。`self` 是 Python 类实例方法的基础概念，普通 Python 类也一样使用。

## 56. `/cmd_vel` 是谁创建的

没有一个必须先创建整个话题的中心节点。

底盘节点通过 `create_subscription()` 创建订阅端；键盘、导航或命令行通过 publisher 创建发布端。ROS 发现并连接兼容的两端。

## 57. `create_subscription()` 会自己生成速度吗

不会。它只创建接收端并登记回调。速度必须由其他发布者发送。

## 58. `Twist` 是变量名吗

它是 ROS 消息类，从 `geometry_msgs.msg` 导入。收到的具体消息对象才保存在 `message` 中。

## 59. `10` 是 10 Hz 吗

不是。这里是订阅 QoS 队列深度。发布频率由发布者控制。

## 60. Launch 是主节点吗

不是。Launch 负责组织启动，主节点负责长期处理业务。

## 61. YAML 是 Python 代码吗

不是。YAML 是配置数据文本，不能执行回调或发送串口。

## 62. 改 YAML 为什么主节点不知道

正在运行的节点不会持续监视源码 YAML。当前 Launch 还读取安装副本，所以修改源码 YAML 后需要重新编译并重启节点。

---

# 第十一部分：建议你现在只记住的内容

第一次阅读不需要记住所有 ROS API。先掌握下面八句话：

1. 类是设计图，对象是根据类创建出的具体实例；
2. `self` 表示当前这个对象；
3. `ChassisCanNode` 继承了 ROS 的 `Node` 类；
4. `node = ChassisCanNode()` 创建一个具体节点对象；
5. `rclpy.spin(node)` 让节点长期等待消息、服务和定时器；
6. Launch 决定如何启动节点并加载哪个配置；
7. YAML 保存参数值，不负责执行程序；
8. `create_subscription()` 创建 `/cmd_vel` 接收端，速度由其他发布者产生。

最核心的一行现在可以读成一句完整中文：

```python
self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
```

```text
让当前底盘节点创建一个订阅端，
接收名为 /cmd_vel、类型为 Twist 的消息；
每当收到消息，就由 ROS 调用当前对象的 _on_cmd_vel 方法；
订阅使用默认 QoS，并保留深度为 10 的消息历史。
```

这才是后续理解主节点其他代码的起点。

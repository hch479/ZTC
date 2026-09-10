# Codex 项目交接入口

你正在接手 C30D STM32F407 + RDK X5 ROS 2 小车项目。先完整阅读 `迁移与继续开发说明.md`，再阅读 README 和 `docs/21`、`docs/22`、`docs/25` 对应文档。路径都相对此目录；原文档中的旧电脑路径仅是历史记录。

用户偏好：使用中文；后续技术回答交付为实际 `.md` 文件并提供链接；代码要简单易懂、注释充分，讲解要结合真实函数、变量、数据流和数值例子。用户已理解基础电机代码，但 IMU、滤波、Python 和状态估计还需要细致说明。不使用“移植”描述本项目工作。

当前主线：`keil_project/R550_C30D_SERIAL_MOTOR/USER/WHEELTEC.uvprojx`，目标 `C30D_SERIAL_MOTOR`；ROS 源码在 `ros2_ws/src/chassis_can_control`。保留 CAN 工程，但它不是与串口主线同步的最新版 IMU 工程。不要直接以旧 releases、historical_builds 或 GD32 工程覆盖当前主线。

现状（2026-09-10更新）：串口电机闭环曾有历史实车记录；IMU 采集、500点标定及 ROS 发布已有实现。用户已授权并新增 Python 五状态平面 EKF，串口 YAML 默认 odometry.mode=ekf；weighted 保留原固定权重方式，CAN YAML 仍为 weighted。EKF 状态为 x/y/yaw/v/omega，不含在线 gyro bias；每个有效 IMU 对只更新一次，使用上位机接收时间。发布纯编码器 wheel/odom_raw 与 EKF odom，仅后者发布 TF。详见 docs/27；离线数学与节点替身测试不等于 ROS Humble 或实车验收。MCU 时间戳、在线偏置、ESKF、舵角反馈仍未实现。文档21～25中“当前算法”属于历史版本描述。

约束：保持软件电机使能默认关闭；保留蜂鸣器关闭状态。写硬件前核对实际连接板、当前固件及备份；历史Flash和选项字节不是通用固件。不要把本包称为 RDK SD 卡或虚拟机完整镜像。不要根据过时 IP、旧账号或旧设备路径直接连接并启动电机。

修改前核对共享核心与 Keil 内嵌副本：CMake 测试用 `common/`、`firmware/stm32f407/`；实际串口构建用主线 `CHASSIS_SERIAL/`。修改这些逻辑时检查并同步相应副本，测试通过不等于主线 Keil 自动通过。

接入 EKF 前必须提供纯编码器观测，避免将已有 IMU 融合的 /odom 与同一 IMU 当作独立输入；同一条 odom→base_link TF 只保留一个发布者。编码器+gyro不能保证识别两轮共同打滑的平移误差。不要把算法名称当作精度证据。

首次接手先只读核对工程入口、依赖和验证记录，运行离线测试并输出中文 `.md` 接手报告。普通修改用 apply_patch，保留用户改动。不要默认自动烧录、擦除Flash或启动电机。

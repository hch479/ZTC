# CAN FD、程序下载、备份与恢复手册

本文只针对当前资料中已经确认的组合：

- 下位机：WHEELTEC C30D 2.0；
- MCU：STM32F407VET6；
- CAN 收发器：VP230，即 TI SN65HVD230；
- 上位机：D-Robotics RDK X5；
- 当前工程通信：经典 CAN，标准 11 位 ID，1 Mbit/s，每帧最多 8 字节。

## 1. 先给出结论

### 1.1 当前整车不能直接改成 CAN FD

C30D 下位机有两处硬件限制：

1. STM32F407 内置的是 `bxCAN`。芯片手册明确说明它符合 CAN 2.0A/B，最高 1 Mbit/s，不是 `FDCAN` 外设，不能收发 CAN FD 帧。
2. C30D 原理图中的物理层芯片是 `VP230/SN65HVD230`。其数据手册只规定经典 CAN、最高 1 Mbit/s，并没有 CAN FD 合规指标。

因此，只改 C/C++、ROS 2 或 SocketCAN 参数不能让 C30D 支持 CAN FD。即使上位机发出 FD 帧，下位机控制器也无法按 FD 格式解析。

### 1.2 RDK X5 本身支持 CAN FD

RDK X5 官方资料说明板上集成了 TCAN4550 CAN FD 控制器和收发器，兼容经典 CAN 与 CAN FD。因此当前系统的限制来自 C30D，不是 RDK X5。

官方说明：

- [RDK X5 CAN 使用](https://d-robotics.github.io/rdk_doc/Advanced_development/hardware_development/rdk_x5/can/)
- [RDK X5 硬件接口](https://d-robotics.github.io/rdk_doc/Quick_start/hardware_introduction/rdk_x5/)

### 1.3 本项目推荐继续使用经典 CAN

本项目的命令、反馈、诊断和参数帧都已拆成 8 字节经典 CAN 帧，1 Mbit/s 对两轮底盘的带宽完全足够。CAN FD 会增加硬件改造、底层驱动和兼容性工作，但不会直接提高轮速闭环质量。电机项目的主要技术含量仍然来自：编码器采样、前馈加 PI、运动学、保护状态机、故障诊断、参数管理和 ROS 2 集成。

RDK X5 应按经典 CAN 启动：

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up
ip -details -statistics link show can0
```

不要对当前 C30D 总线执行 `fd on`，也不要发送 `123##...` 形式的 CAN FD 帧。

### 1.4 如果以后一定要做 CAN FD

有两条硬件路线：

1. 更换为带 FDCAN 外设的下位机控制板，并使用明确标注 CAN FD 的收发器。这是推荐路线。
2. 给 STM32F407 外接 SPI CAN FD 控制器，再配 CAN FD 收发器。可选器件例如 MCP2517FD/MCP2518FD 加 MCP2562FD。该路线还要做 SPI 驱动、中断、缓存、错误状态和实时性验证，复杂度明显更高。

无论哪条路线，上下位机都要统一仲裁段位速率、数据段位速率、采样点和终端电阻。到那时还要把 Linux 程序从 `struct can_frame` 改为 `struct canfd_frame`，并重写 STM32 侧底层驱动；不能只改 `fd on`。

## 2. 必须区分的三个“Bootloader”

### 2.1 STM32 芯片内部的系统 Bootloader

这是 ST 固化在 System Memory ROM 中的程序，用于串口等系统下载。普通用户 Flash 的整片擦除不会擦掉它。

### 2.2 WHEELTEC 无线 BootLoader

这是厂家放在 STM32 用户 Flash 开头的自定义程序。本资料对应文件为：

```text
C:\Users\User\Desktop\ROS机器人小车资料\
2.WHEELTEC R550-V550 ROS教育机器人运动底盘资料\
5.无线烧录教程与工具\2.烧录工具\
C30D_BootLoader_64kb_V1.0.hex
```

它占用 `0x08000000..0x0800FFFF` 共 64 KiB。整片擦除会把它删掉；删掉以后无线烧录工具无法直接连接，必须先通过 ST-Link 或 USB 串口有线恢复。

### 2.3 RDK X5 的启动固件和 Linux 系统

RDK X5 是 Linux 计算机。ROS 包通常只是复制、编译和运行，不是在每次修改时重刷整机系统。只有系统损坏或需要更换系统版本时，才对 TF 卡/存储介质重烧镜像。不要把 RDK X5 的系统烧录与 STM32 的 Keil 下载混在一起理解。

## 3. 修改任何程序前的备份

### 3.1 备份原厂源码和 HEX

把厂家最新 C30D 工程整体复制到另一个磁盘或压缩包中。当前已找到的普通原厂恢复文件是：

```text
C:\Users\User\Desktop\ROS机器人小车资料\
2.WHEELTEC R550-V550 ROS教育机器人运动底盘资料\
3.STM32运动底盘源码\
当前版本代码（适配STM32控制板_D版_2022.05.17-现在）\
C30D-2.0版\
R550_C30D(2.0)_Mini小车STM32源码_GMR编码器_2025.12.26\
OBJ\WHEELTEC.hex
```

该 HEX 的有效地址从 `0x08000000` 开始，是“普通直接启动应用”，不是保留厂家无线 BootLoader 的应用镜像。

### 3.2 用 ST-Link 读取当前整片 Flash

只保存厂家 HEX 不一定能保存你这台车当前已经标定的参数。最稳妥的方法是在第一次下载前读取整片 512 KiB 用户 Flash：

1. 安装 STM32CubeProgrammer 和 ST-Link 驱动。
2. C30D 正常供电并关闭电机使能。
3. 连接 `GND、SWDIO(PA13)、SWCLK(PA14)`；建议再接 `NRST` 和 3.3 V 目标电压参考。
4. 在 CubeProgrammer 中选择 ST-Link、SWD，连接芯片。
5. 从 `0x08000000` 读取长度 `0x00080000`。
6. 保存为例如 `C30D_factory_full_2026xxxx.bin`。
7. 再读取一次并比较文件哈希，确认备份稳定。
8. 截图保存 Option Bytes，尤其是 RDP 等级。

不要给同一块板同时从多个来源强行供电。通常让 C30D 使用自己的正常电源，ST-Link 的 3.3 V 只作为目标电压参考；具体以你的 ST-Link 型号接线定义为准。

如果芯片已经设置读保护，读取可能失败。不要在没备份时随意解除 RDP Level 1，因为解除会触发整片擦除。绝对不要设置 RDP Level 2，它通常不可逆。

### 3.3 备份 RDK X5

至少做两层备份：

1. 把自己的 ROS 工作空间、配置文件、launch 文件和 systemd 服务复制到电脑或 Git 仓库。
2. 如果系统运行在 TF 卡上，关机后拔卡，用读卡器制作整卡镜像。

Linux 备份示例：

```bash
sudo fdisk -l
sudo dd if=/dev/sdX bs=4M status=progress | gzip > rdk_x5_factory.img.gz
sync
```

`/dev/sdX` 必须替换成经过容量和分区信息确认的 TF 卡整盘设备。恢复命令中的 `of=` 会覆盖目标盘；选错设备会造成数据丢失，所以恢复时更推荐使用 balenaEtcher 的图形界面并反复核对目标卡。

如果你的 RDK X5 是 eMMC/模组版本，不能照抄 TF 卡步骤，应先确认具体板型、启动介质和官方 `rdk-backup`/USB 烧录流程。

## 4. 下位机正常编译和 ST-Link 下载

### 4.1 接线和安全状态

1. 把车架起，驱动轮离地。
2. 关闭 C30D 电机硬件使能，最好断开电机主电源。
3. C30D 与 ST-Link 共地。
4. 连接 `PA13-SWDIO`、`PA14-SWCLK`，推荐连接 `NRST`。
5. 给 C30D 正常逻辑电源。

### 4.2 Keil 设置

1. 打开 `USER/WHEELTEC.uvprojx`。
2. 选择 `Options for Target -> Debug`。
3. 调试器选择 `ST-Link Debugger`。
4. 点击 `Settings`，确认能识别 ST-LINK/V2 和 STM32F407。
5. 进入 `Flash Download`。
6. 添加 `STM32F4xx 512KB Flash` 算法。
7. 正常开发时选择 `Erase Sectors`，同时勾选 `Program、Verify、Reset and Run`。
8. 编译为 0 error 后点击 Download。

`Erase Full Chip` 只用于确实无法下载或计划完整恢复时，不应成为每次下载的默认选项，因为它会删除厂家无线 BootLoader和保存在用户 Flash 中的参数。

### 4.3 当前自研工程的 Flash 布局

当前工程按“ST-Link 直接启动”设计：

```text
0x08000000..0x0805FFFF  应用程序，最大 384 KiB
0x08060000..0x0807FFFF  参数区 sector 7，128 KiB
```

它使用 `WHEELTEC_reserve_sector7.sct`，入口在 `0x08000000`。因此当前构建出的程序会覆盖位于 Flash 开头的厂家无线 BootLoader。使用当前版本时，把 ST-Link 作为主要恢复通道即可。

## 5. USB 一键下载

C30D 原理图有 CH9102F、DTR/RTS 自动复位和进入系统 Bootloader 的电路。厂家资料针对 F4 建议使用 FlyMCU：

1. 连接 C30D USB 下载口和电脑。
2. 安装 CH9102/串口驱动，确认出现 COM 口。
3. 打开 FlyMCU。
4. 选择正确 COM 口和 HEX 文件。
5. 选择与厂家教程一致的控制方式：DTR 低电平复位、RTS 高电平进入 Bootloader。
6. 启动编程并等待校验成功。
7. 重新断电上电。

USB 下载依赖 BOOT0/复位控制和串口链路。出现异常时，ST-Link 的 SWD 方式通常更适合作为最终救援通道。

## 6. 厂家无线烧录布局

资料中的 C30D 无线 BootLoader 名称含 `64kb`，已配置无线工程也验证了如下布局：

```text
0x08000000..0x0800FFFF  WHEELTEC C30D 无线 BootLoader，64 KiB
0x08010000..            无线版应用程序 APP
```

厂家压缩包中的无线应用 `wheeltec.bin` 首个复位向量指向 `0x08010331`，对应应用从 `0x08010000` 启动；其 `system_stm32f4xx.c` 中 `VECT_TAB_OFFSET` 为 `0x10000`。

无线烧录前置步骤：

1. 通过 FlyMCU 或 ST-Link 把 `C30D_BootLoader_64kb_V1.0.hex` 烧到 `0x08000000`。
2. 完全断电再上电一次。
3. 使用厂家“已配置无线烧录”工程生成的 `wheeltec.bin`。
4. 使用 WHEELTEC 无线烧录工具通过受支持的 JDY-33 蓝牙或 Wi-Fi 模块发送 BIN。
5. 厂家资料明确说明旧 BT04-A 不支持该无线烧录功能。

如果要让当前自研工程兼容这个无线 BootLoader，必须另外制作无线版 Target：

- IROM 起始地址改为 `0x08010000`；
- 中断向量偏移改为 `0x10000`；
- 链接上限仍不得进入参数区 `0x08060000`；
- 生成 BIN；
- 应用还要按厂家协议在无线串口接收 `reset` 并调用 `NVIC_SystemReset()`。

在完成并验证这些改动之前，不要把当前从 `0x08000000` 链接的 BIN 当作无线应用发送。

## 7. 整片擦除后的恢复

### 7.1 只需要恢复厂家普通功能

这是最简单、最稳妥的方案：

1. 使用 ST-Link 或 FlyMCU 有线连接 C30D。
2. 对用户 Flash 执行整片擦除。
3. 选择第 3.1 节的原厂 `OBJ/WHEELTEC.hex`。
4. Program、Verify、Reset and Run。
5. 断电重启，保持车轮离地测试手柄、串口、电机、编码器和舵机。

这会恢复厂家普通应用，但不会恢复厂家无线烧录功能，因为普通 HEX 从 `0x08000000` 开始并占用 BootLoader 区域。

### 7.2 恢复到擦除前一模一样

如果第 3.2 节保存过完整 512 KiB BIN：

1. 用 ST-Link 连接。
2. 整片擦除用户 Flash。
3. 从 `0x08000000` 写入完整备份 BIN。
4. Verify。
5. 检查 Option Bytes 与备份记录一致。
6. 复位并离地测试。

该方法能同时恢复当时的 BootLoader、应用和内部 Flash 参数，是最接近“一键回到购买状态”的方法。

### 7.3 恢复厂家无线烧录功能

1. 用 ST-Link 或 FlyMCU 烧 `C30D_BootLoader_64kb_V1.0.hex`。
2. 断电重启一次。
3. 使用厂家压缩包内已配置无线烧录的 `wheeltec.bin` 进行无线下载。
4. 或用 ST-Link 把该 APP BIN 明确写到 `0x08010000`，不能写到 `0x08000000`。
5. Verify 后复位测试。

无线资料包的应用版本与当前最新普通源码日期并不相同。要追求“原购买版本完全一致”，仍应优先使用你在改动前读取出的完整 Flash 备份，或向厂家索取与你订单、板卡版本和车型完全对应的固件。

### 7.4 擦除后连不上怎么办

按以下顺序处理：

1. 检查 C30D 逻辑电源和 ST-Link 目标电压。
2. 检查 GND、PA13、PA14、NRST 是否接对。
3. 将 SWD 频率降到 100 kHz～1 MHz。
4. 选择 `Connect under reset`，按住复位再连接。
5. 检查 Option Bytes；不要设置 RDP Level 2。
6. 能连上后先烧最小的已知正常原厂 HEX，再排查自研程序。

用户 Flash 被擦空通常不会永久损坏 MCU；只要 SWD、供电和 Option Bytes 没有被错误锁死，ST-Link 仍能重新写入程序。

## 8. RDK X5 上位机程序如何更新

### 8.1 更新 ROS 2 代码不需要重刷系统

把本工程复制到 RDK X5，例如：

```bash
mkdir -p ~/chassis_project
# 从电脑复制本项目的 ros2_ws 到上述目录后执行：
cd ~/chassis_project/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-select chassis_can_control
source install/setup.bash
ros2 launch chassis_can_control chassis_can.launch.py
```

这只是部署应用。要回退时停止节点，恢复备份的工作空间并重新 `colcon build`，不会擦除 RDK OS。

### 8.2 系统损坏时重刷 TF 卡

1. 先确认板型、系统版本和启动介质：

```bash
cat /etc/version
rdkos_info 2>/dev/null || true
lsblk
```

2. 正常关机，拔出 TF 卡；禁止运行时热拔卡。
3. 在 Windows 中打开 balenaEtcher。
4. 选择与 RDK X5、ROS/TROS 环境匹配的官方或厂家 `.img.gz` 镜像。
5. 仔细选择目标 TF 卡，开始烧录并等待校验完成。
6. 插回 RDK X5，首次启动后恢复网络、ROS 工作空间和配置。

官方文档提醒 RDK X5 出厂 miniboot 与硬件版本匹配，不应降级写入旧版 miniboot。重刷系统镜像和更新 NAND 中的 miniboot 是两个不同动作；没有明确版本要求时不要随意执行底层启动固件更新。

## 9. 推荐的第一次实操顺序

1. 先不擦除，读取 C30D 完整 512 KiB Flash 并保存两份。
2. 保存 C30D Option Bytes 截图和当前 RDK 系统版本。
3. 对 RDK TF 卡做整卡镜像，另行备份 ROS 工作空间。
4. 车轮离地并关闭电机使能。
5. 用 ST-Link 的 `Erase Sectors` 下载自研固件，不选 `Erase Full Chip`。
6. 先验证心跳、编码器和故障状态，再允许很小 PWM。
7. RDK X5 使用经典 CAN 1 Mbit/s，不开启 FD。
8. 只有当普通下载失败或明确执行恢复时才整片擦除。

这样即使新固件不能运行，也可以用 ST-Link 把完整备份或原厂 `WHEELTEC.hex` 写回去。

# 这是可以直接用 Keil 打开的串口版完整下位机工程

## 最简单的打开方法

双击本目录下的：

```text
OPEN_KEIL_PROJECT.bat
```

或者直接双击：

```text
USER\WHEELTEC.uvprojx
```

打开后，Keil 左侧工程树应出现 `CHASSIS_SERIAL` 分组，其中有 7 个已经加入编译的 C 文件。

## 编译

1. 安装 Keil MDK 5。
2. 安装 `Keil.STM32F4xx_DFP` 芯片包。本机已经确认安装 2.11.0，工程也已对齐到该版本。
3. 本工程沿用厂家 ARM Compiler 5.06 配置。本机已经确认安装 ARM Compiler 5.06 update 5，可以直接编译。
4. 打开工程后点击 `Rebuild`。
5. 正确结果应为 `0 Error`。
6. 新生成文件位于：

```text
OBJ_SERIAL\C30D_SERIAL_MOTOR.hex
```

本机已于 2026-08-13 用 Keil µVision 5.24a 和 ARM Compiler 5.06 update 5 完成真实构建，结果为 `0 Error(s), 0 Warning(s)`。

CAN 版本仍完整保留在相邻目录 `R550_C30D_CAN_MOTOR`，串口版不会覆盖它。

`OBJ` 目录中的 `WHEELTEC.hex` 是复制过来的厂家原始构建结果，不是本项目的新固件。不要把两个 HEX 混淆。

## 下载前必须做的事

1. 先用 ST-Link 读取并保存原来完整的 512 KiB Flash。
2. 把小车架起，所有轮子离地。
3. 关闭电机硬件使能，最好暂时断开电机主电源。
4. ST-Link 连接 GND、PA13/SWDIO、PA14/SWCLK，推荐再接 NRST。
5. 正常下载选择 `Erase Sectors`，不要默认选择 `Erase Full Chip`。

详细备份和恢复步骤见项目根目录：

```text
docs\09_CANFD与烧录恢复.md
```

## 这个 Keil 工程已经替你完成的接入

- 厂家完整 STM32 库、启动代码、FreeRTOS 和 C30D 硬件驱动都已保留；
- 新电机控制代码已复制到 `CHASSIS_SERIAL`；
- 7 个新 C 文件（包括串口传输层）已经加入 Keil 工程；
- `CHASSIS_SERIAL\inc` 已加入头文件路径；
- `main.c` 已改为创建 `chassis_control_task` 和 `chassis_can_task`；
- 原 `Balance_task`、手柄任务和旧 `data_task` 不再创建，避免重复控制电机和旧 CAN 协议冲突；
- USART3 使用 115200、8N1；旧厂家串口解析入口保留但不参与当前构建逻辑；
- 链接文件已限制应用在 `0x08000000..0x0805FFFF`，保留 sector 7 参数区；
- 目标芯片改正为 STM32F407VE、512 KiB Flash；
- 新输出单独放在 `OBJ_CAN`，不会覆盖厂家 `OBJ` 中的原始 HEX。

## 第一次打开时必须看一眼的 Keil 设置

进入 `Options for Target`：

1. `Device` 应显示 `STM32F407VE`。
2. `C/C++ -> Include Paths` 中应有 `..\CHASSIS_SERIAL\inc`。
3. `Linker` 应使用 `..\CHASSIS_SERIAL\WHEELTEC_reserve_sector7.sct`。
4. `Output` 应勾选 `Create HEX File`。
5. `Debug` 应选择 `ST-Link Debugger`。
6. `Utilities/Flash Download` 应使用 STM32F4xx 512 KB Flash 算法。

如果 Keil 因为本机芯片包版本不同而自动更新工程配置，更新后重新核对上面六项。

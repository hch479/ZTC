# 本机 Keil 环境检查与安装说明

## 1. 本机实际检查结果

本机 Keil 位于：

```text
C:\Users\User\Desktop\keil5
```

已经确认：

| 项目 | 本机状态 |
|---|---|
| Keil µVision | 5.24a（5.24.2.0） |
| STM32F1 芯片包 | `STM32F1xx_DFP 2.2.0`，已安装 |
| STM32F4 芯片包 | `STM32F4xx_DFP 2.11.0`，已安装 |
| STM32F407VETx 设备描述 | 已包含 |
| STM32F4 512 KiB Flash 下载算法 | `STM32F4xx_512.FLM`，已安装 |
| ARM Compiler 5 | 5.06 update 5，已安装 |
| ARM Compiler 6 | 6.7，已安装 |
| 当前工程实际使用 | ARM Compiler 5.06 update 5 |

因此当前工程不需要再安装 STM32F407 芯片包，也不需要更换 Keil。看到 STM32F103/STM32F1 只说明 F1 包也装着，并不代表只能开发 F1。

## 2. 已完成真实编译验证

使用本机的 `UV4.exe` 编译：

```text
keil_project\R550_C30D_CAN_MOTOR\USER\WHEELTEC.uvprojx
```

结果：

```text
Program Size: Code=66008 RO-data=8032 RW-data=836 ZI-data=29772
0 Error(s), 0 Warning(s)
```

生成文件：

```text
keil_project\R550_C30D_CAN_MOTOR\OBJ_CAN\C30D_CAN_MOTOR.hex
```

也可以双击：

```text
keil_project\R550_C30D_CAN_MOTOR\BUILD_WITH_KEIL.bat
```

重新编译并查看 `OBJ_CAN\keil_build.log`。

## 3. 在 Keil 图形界面自己确认

1. 打开工程。
2. 点击工具栏的魔术棒 `Options for Target`。
3. 在 `Device` 页确认显示 `STM32F407VE`。
4. 打开 `Project -> Manage -> Pack Installer`。
5. 左侧搜索 `STM32F407VE` 或 `STM32F4`。
6. 右侧 `Packs` 中应看到 `Keil::STM32F4xx_DFP` 为 Installed。
7. 在 `Project -> Manage -> Project Items -> Folders/Extensions` 或编译器选择位置，确认 ARM Compiler 5 可选。
8. 在 `Utilities -> Settings -> Flash Download` 中确认使用 `STM32F4xx 512KB Flash`。

## 4. 如果换电脑后没有 F4 芯片包

### 方法一：Pack Installer 在线安装

1. 打开 Keil。
2. 选择 `Project -> Manage -> Pack Installer`。
3. 等待左下角 Pack 索引更新完成。
4. 搜索 `STM32F4xx_DFP`。
5. 选择 `Keil::STM32F4xx_DFP`，点击 `Install`。
6. 安装完成后重新打开工程，在 Device 页选择 `STM32F407VE`。

### 方法二：离线安装 `.pack`

1. 从 Arm Keil 官方 Pack 页面下载 `Keil.STM32F4xx_DFP.x.x.x.pack`。
2. 关闭正在编译的工程。
3. 双击 `.pack` 文件，使用 Pack Installer 安装。
4. 重新打开 Keil。
5. 在 Pack Installer 中确认状态为 Installed。

官方 Pack 页面：

```text
https://www.keil.arm.com/packs/stm32f4xx_dfp-keil/overview/
```

对于这个基于老厂家工程、使用 ARM Compiler 5 的项目，优先使用 2.x 系列 DFP。新 3.x Pack 的内容组织有明显变化，不必为了“版本最新”而升级一个已经能稳定编译的旧工程。

## 5. 如果缺少 ARM Compiler 5

常见提示包括：

```text
ARM Compiler V5.06 update ... not available
Missing Compiler Version 5
```

处理方法：

1. 在 Keil 中打开 `Project -> Manage -> Project Items -> Folders/Extensions`。
2. 查看 ARM Compiler 5 是否已经列出。
3. 如果没有，从 Arm/Keil 官方历史编译器下载页取得 ARM Compiler 5.06 安装包。
4. 安装后，在 Keil 的 `Folders/Extensions` 中添加 ARMCC 安装目录。
5. 回到工程 `Options for Target`，选择 ARM Compiler 5。

本机已有以下文件，不需要执行上述安装：

```text
C:\Users\User\Desktop\keil5\ARM\ARMCC\bin\armcc.exe
```

## 6. 芯片包与厂家源码库不是同一个概念

- `STM32F4xx_DFP`：让 Keil 认识 STM32F407VE，提供设备信息、寄存器视图、Flash 下载算法和调试描述。
- 工程中的 `FWLIB`：厂家随源码附带的 STM32F4 标准外设库源码，实际参与 C 文件编译。
- `STM32F1xx_DFP`：给 STM32F103/F107 等 F1 芯片使用，不能替代 F4 包。

当前工程两部分都齐全：Keil 已安装 F4 DFP，项目目录中也有厂家使用的 STM32F4 `FWLIB`。

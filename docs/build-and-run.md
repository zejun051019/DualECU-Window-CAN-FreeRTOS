# 接线、构建与运行

## 台架

| 部件 | 使用方式 |
|---|---|
| F407 | M144Z-M4 Mini Board，STM32F407ZGT6，J-Link / SWD |
| 执行端 | 天猛星 MSPM0G3507，XDS110 / SWD |
| CAN | 两个3.3 V逻辑收发器，Classic CAN，500 kbit/s，两端端接 |
| 电机 | 带内部驱动器的 X42S，UART 命令及位置/状态反馈，空轴演示 |

按信号名称连接，不按线色推断：

| MCU | 模块 / 电机端 | 功能 |
|---|---|---|
| F407 PA12 | 本侧 CAN 模块 TX | CAN1_TX |
| F407 PA11 | 本侧 CAN 模块 RX | CAN1_RX |
| G3507 PA12 | 本侧 CAN 模块 TX | MCAN_TX |
| G3507 PA13 | 本侧 CAN 模块 RX | MCAN_RX |
| 每块板3.3 V、GND | 本侧 CAN 模块3.3 V、GND | 逻辑供电 |
| 两模块 CANH、CANL | 对侧同名 CANH、CANL | 总线 |
| G3507 PB6 / UART1_TX | 电机 R/A/H | 电机接收 |
| G3507 PB7 / UART1_RX | 电机 T/B/L | 电机发送 |
| G3507 GND | 电机 Gnd，并与其他节点共地 | 信号参考 |

电机使用台架独立12 V电源，不由开发板3.3 V/5 V供电。MCU直连接法适用于本台架可通信的TTL UART接口；RS485等其他电气版本需要对应收发器。电机与执行端之间使用UART，电机CAN端子不在本应用链路中。改线先断电，空轴保持净空。

## 工具与配置源

- CMake、Ninja、原生 GCC（PC 检查）。
- F407：ST GNU Tools for STM32 14.3.1+st.2；CubeMX 6.11.1 / CubeF4 1.28.3；配置源为 [f407.ioc](../firmware/f407/f407.ioc)。
- G3507：Arm GNU 10.3.1；MSPM0 SDK 2.10.00.04 / SysConfig 1.27.1；配置源为 [mspm0_can.syscfg](../firmware/g3507/mspm0_can.syscfg)。已包含生成的配置及所需 DriverLib 源码，普通构建不需要重新生成。
- J-Link 和带 MSPM0 支持的 OpenOCD 0.12.0 / XDS110。

本仓库的下载入口面向 Windows。将 [.env.example](../.env.example) 中的工具目录配置为本机环境变量后，重新打开 VS Code；该文件本身不会自动加载。

## VS Code

打开仓库根目录，安装 CMake Tools、C/C++、Cortex-Debug。执行 **Terminal → Run Build Task** 选择 F407 或 G3507 构建；执行 **Terminal → Run Task** 可选择主机检查或对应板卡下载。两种探针严格区分：F407 使用 J-Link，G3507 使用 XDS110。

## 等价命令

在仓库根目录的 PowerShell 执行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\run-project-host-checks.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Target F407
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Target G3507
```

预期为主机检查14/14通过，生成 `firmware/f407/build/Debug/f407.elf` 和 `firmware/g3507/build-zdt-uart-control/MSPM0.elf`。G3507 电机演示使用 `DUALECU_ENABLE_ZDT_UART_CONTROL=ON`，正式构建的故障注入开关为OFF。

连接指定探针及已核对的台架后下载：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\flash.ps1 -Target F407
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\flash.ps1 -Target G3507
```

核对 J-Link 校验输出和 OpenOCD `Verified OK`。分开刷写会暂时中断双板通信，应先回到STOP并完成显式恢复；刷写/重启不恢复旧运动。运行时先长按 KEY_UP / WKUP 建立临时范围，再以新短按执行演示。

## 调试观察

先观察 F407 在线状态及 STOP 锁存，再观察 G3507 的 fault、range valid 和 UART 反馈；只有这些条件成立，按键才可能转换成运动。

- F407：`g_can_physical_button_event_count`、`g_can_physical_button_rejected_count`、远端状态及接收年龄。
- G3507：`g_window_can_state`、`g_window_can_fault`、`g_window_can_demo_active`、`g_zdt_uart_range_valid`、`g_zdt_uart_position_tenths`。
- 通信异常：F407 CAN ESR、G3507 MCAN PSR/ECR；本台架曾观察到间歇Bus-Off，需要区分离线锁存、总线错误与按键事件拒绝。

调试器暂停一端会改变时序；用于读数的ELF必须与板上镜像匹配。不能用旧符号地址解释新版RAM。构建、下载校验和实际运动是三个不同验证层次。

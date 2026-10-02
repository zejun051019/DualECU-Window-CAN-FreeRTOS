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

### 构建选项

| 目标 / 选项 | 默认演示配置 | 用途 |
|---|---|---|
| F407 / `DUALECU_ENABLE_BOOT_CAN_PROBE` | OFF | ON 时运行原1000帧上电链路诊断；会延后正常按键服务，日常演示关闭 |
| G3507 / `DUALECU_ENABLE_ZDT_UART_CONTROL` | ON（构建入口设置） | UART 电机命令与反馈、演示和本地保护 |
| G3507 / `DUALECU_ENABLE_TEST_FAULT_INJECTION` | OFF | 诊断构建可软件丢弃反馈等；不等同物理故障 |

F407 如需单独运行链路诊断，在已有构建目录执行 `cmake -S firmware/f407 -B firmware/f407/build/Debug -DDUALECU_ENABLE_BOOT_CAN_PROBE=ON` 后构建。回到演示时将同一选项设为OFF并重新构建。OFF 时启动诊断计数为0是预期行为，正常命令/状态计数仍应增长。

连接指定探针及已核对的台架后下载：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\flash.ps1 -Target F407
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\flash.ps1 -Target G3507
```

核对 J-Link 校验输出和 OpenOCD `Verified OK`。分开刷写会暂时中断双板通信，应先回到STOP并完成显式恢复；刷写/重启不恢复旧运动。运行时先长按 KEY_UP / WKUP 约2.5秒后松开，完成恢复并建立临时范围，再以新短按执行演示。电机重新上电后把当前位置当作其内部零点，不沿用旧绝对坐标；先重新长按设零。若故障原因仍活动，运动应被拒绝，先查状态而非反复短按。

## 调试观察

先观察 F407 在线状态及 STOP 锁存，再观察 G3507 的 fault、range valid 和 UART 反馈；只有这些条件成立，按键才可能转换成运动。

- F407：`g_can_physical_button_event_count`、`g_can_physical_button_rejected_count`、远端状态及接收年龄。
- G3507：`g_window_can_state`、`g_window_can_fault`、`g_window_can_demo_active`、`g_zdt_uart_range_valid`、`g_zdt_uart_position_tenths`。
- 通信异常：F407 CAN ESR、G3507 MCAN PSR/ECR；本台架曾观察到间歇Bus-Off，需要区分离线锁存、总线错误与按键事件拒绝。
- G3507 总线服务：`g_can_port_bus_off_count`、`g_can_port_recovery_attempt_count`、`g_can_port_recovery_complete_count`、`g_can_port_last_error_code`。错误码是历史观察值，不代表当前仍故障；总线恢复计数增长也不等于业务解锁。先确认状态报文持续更新，再观察握手、fault和range valid。

调试器暂停一端会改变时序；用于读数的ELF必须与板上镜像匹配。不能用旧符号地址解释新版RAM。构建、下载校验和实际运动是三个不同验证层次。

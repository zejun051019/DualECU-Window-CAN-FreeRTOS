# 双 ECU 车窗执行器控制原型

**STM32F407 · FreeRTOS · Classic CAN · MSPM0G3507 · UART 电机反馈**

[![Host checks](https://github.com/zejun051019/DualECU-Window-CAN-FreeRTOS/actions/workflows/host-checks.yml/badge.svg)](https://github.com/zejun051019/DualECU-Window-CAN-FreeRTOS/actions/workflows/host-checks.yml)

用一套双节点台架模拟车窗升降：**F407 处理按键和通信请求，G3507 执行本地故障门控，再通过 UART 控制 X42S 闭环步进电机。** 当前演示使用空轴，以相对 90° 行程呈现升降、停止与回程。

## 功能演示

| 操作 | 台架行为 |
|---|---|
| 长按 F407 的 KEY_UP / WKUP 约 2.5 秒，松开 | 显式恢复并建立临时软件零点；轴保持静止 |
| 设零后短按一次 | 向上端运动，约 90° 后停止 |
| 到上端后再短按 | 回到下端并停止 |
| 运动过程中短按 | 中途 STOP，不自动续动 |
| 中途停止后再短按 | 以新操作返回下端 |

设零基准保存在 RAM。复位、故障或电机重新上电后的恢复流程从 STOP 开始，运动需要新的用户操作。长按后紧接短按的交互由一次性、有限期的待办意图处理，等待设零确认后仅执行一次；离线、故障、STOP 或超时会取消该意图。

```mermaid
flowchart LR
    KEY[KEY_UP / WKUP] --> F407[F407 / FreeRTOS<br/>RX ISR → Queue → RX任务<br/>TX任务统一决策]
    F407 <-->|Classic CAN · 500 kbit/s| G3507[G3507 / 裸机周期循环<br/>协议状态机 · 本地故障门控]
    G3507 <-->|UART 命令与反馈| MOTOR[X42S 空轴电机<br/>内部驱动器与闭环]
```

## 工程重点

| 设计 | 解决的问题 | 源码入口 |
|---|---|---|
| ISR 有界取帧，Queue 按值复制，接收时刻随帧传递 | 中断不等待；积压的旧状态不能被误当成刚收到 | [F407 CAN 应用](firmware/f407/App/Src/app_can_physical.c) |
| TX 任务统一管理目标和序号，STOP 独立锁存 | 满队列、并发操作和旧待发运动不能覆盖 STOP | [请求管理](shared/window_request.c) |
| 版本、DLC、序号、重复/冲突/回绕检查 | 区分合法报文、新操作和允许执行的动作 | [协议编解码](shared/can_protocol.c) |
| STOP → CLEAR → STOP，匹配新鲜状态确认 | CAN 发送成功不等于业务执行；清错不自动运动 | [恢复客户端](shared/can_recovery_client.c) |
| 执行端周期检查失联、反馈与故障 | 本地保护不依赖另一块 MCU 或下一帧到达 | [执行端状态机](shared/window_state.c) |
| 有限小段运动、位置反馈与软件行程 | 每段都重新检查门控；停止后不重放旧目标 | [演示规划器](shared/window_demo.c)、[UART 电机端口](firmware/g3507/user/zdt_uart_mspm0.c) |

电机内部完成闭环控制；本项目实现的是跨节点请求、反馈监督、任务协作与故障处理。

## 实测结果

2026-10-02 的正式镜像完成设零、上升、回程、中途停止及停止后回程。UART 反馈的临时范围为 **1.0°–91.0°**，上升终点 **90.5°**，回程终点 **1.5°**；中途停止在 **18.1°**，再次请求后返回 **1.8°**。四段均最终为 `STOP / NONE`。

- **14 组项目主机检查**覆盖协议、序号、恢复、超时回绕、请求锁存、按键、行程、演示规划与 UART 解析。
- 两端工程从公开目录独立构建，保留 `.ioc`、`.syscfg`、生成代码和厂商许可声明。
- 失联、设备错误及软件反馈丢弃路径有版本绑定的台架记录，详见[验证记录](docs/verification.md)。

位置来自电机驱动器反馈，轴运动/停止由现场观察确认。本项目是空轴控制原型，不代表实车车窗、防夹或量产汽车 ECU。台架曾出现间歇 CAN Bus-Off；提供错误状态诊断与显式恢复，实测结果仅支持各自记录的运行窗口。

## 读代码与复现

- [架构与 FreeRTOS 设计](docs/architecture.md)
- [CAN 协议与恢复流程](docs/protocol.md)
- [接线、构建、下载与运行](docs/build-and-run.md)
- [功能与故障验证记录](docs/verification.md)
- [厂商代码许可说明](NOTICE.md)

```text
firmware/f407/       F407 HAL、FreeRTOS、应用及 CubeMX 配置
firmware/g3507/      G3507 DriverLib、UART 电机端口及 SysConfig 配置
shared/             协议、状态机、恢复、行程和演示规划
tests/host/         可在 PC 运行的 C 检查
tools/              主机检查与下载入口
docs/               架构、协议、复现步骤与实测数据
```

开发方式：作者提供硬件台架、确认需求并完成现场按键/轴动作观察；代码实现和部分调试使用 AI 辅助。仓库中的验证记录用于追溯工程结果。

# 架构与 FreeRTOS 设计

## 两端职责

F407 运行 FreeRTOS V10.3.1，使用 CMSIS-RTOS2 接口、GCC ARM_CM4F 端口与 heap_4。它负责输入、协议客户端和 CAN 收发。G3507 运行裸机周期循环，拥有执行端状态和电机命令，按本地时间检查故障。

```mermaid
flowchart TD
    FIFO[bxCAN FIFO0] --> ISR[RX ISR<br/>每次最多3帧 · 保存接收时刻]
    ISR -->|按值复制 · 非阻塞| Q[8项固定大小 Queue]
    Q --> RX[canPhyRx<br/>解析 · 更新状态快照]
    RX -->|Mutex保护 · 保留原接收时间| TX[canTx<br/>每10ms服务 · 唯一TX所有者]
    KEY[按键事件] --> TX
    ISR -->|Queue满| LATCH[独立溢出锁存]
    LATCH --> TX
    TX --> CAN[CAN发送 · STOP优先]
```

## 任务、资源与中断边界

| 上下文 | 实际工作 | 配置 |
|---|---|---|
| CAN RX ISR | 有界复制、时间戳、计数和非阻塞 Queue 投递 | NVIC 优先级5；每次最多3帧 |
| `canPhyRx` | 阻塞等待 Queue，校验并发布状态快照 | BelowNormal；512 B 栈 |
| `canTx` | 按键、目标/序号、离线、STOP、发送和恢复 | Normal；每10 ms；512 B 栈 |

Tick 为1 kHz；HAL 时基使用 TIM6。Heap 配置为15360 B。NVIC 分组为 GROUP_4，4位抢占优先级；`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5`，内核移位值为 `0x50`。本地 CMSIS 适配层在 ISR 调用 `osMessageQueuePut(..., 0)` 时使用 FromISR 分支及必要的调度请求。

## 所有权与故障优先级

- TX 任务唯一拥有请求、目标、seq 和发送决策；RX 任务发布带接收时刻的状态快照。
- 状态快照用 Mutex 保护；ISR 共享锁存的操作使用与 IRQ 边界匹配的临界区，`volatile` 只提供可观察性。
- Queue 满时独立锁存并暂停 FIFO pending 通知；TX 直接检查锁存建立 STOP，后续排空和原因清除再按流程恢复通知。
- 旧状态的年龄从 ISR 接收时刻计算，不从出队时刻计算。
- 发送采用最新请求合并，不积存历史运动；恢复前清理旧目标。
- 运行期 CAN/控制路径不打印阻塞式日志，观察字段通过调试器读取。

## 执行端

[window_can_app.c](../firmware/g3507/user/window_can_app.c) 连接 MCAN、协议状态、演示目标和 UART 电机端口；[window_state.c](../shared/window_state.c) 执行失联、序号与锁存门控；[window_demo.c](../shared/window_demo.c) 规划 RAM 范围内的小段目标。

每段动作结束后重新检查反馈和门控，再考虑下一段。普通 UP/DOWN 仍是有限单步；演示事件使用同一底层保护链路。STOP、故障与失联优先于普通运动；反馈恢复或通信重连不能自行重放旧运动。

当 UART 链路失效时，软件 STOP 可能无法送到电机。有限目标限制已发送的运动范围，不能视作独立硬件切断能力。

## 通信恢复与业务恢复

恢复逻辑复用上述任务和执行端周期循环，没有新增任务或协议层。CAN ID、DLC、序号和 STOP→CLEAR→STOP 语义保持不变。

```mermaid
flowchart TD
    BO[检测到 Bus-Off] --> LOCAL[执行端本地 STOP<br/>取消演示 · 作废 RAM 行程]
    BO --> LINK[撤销旧待发帧<br/>非阻塞服务总线恢复]
    LINK --> ONLINE[通信重新可用<br/>业务仍锁定且无旧目标]
    LOCAL --> ONLINE
    ONLINE --> LONG[原因解除后新的长按]
    LONG --> HANDSHAKE[原 STOP → CLEAR → STOP<br/>每步等待新鲜匹配确认]
    HANDSHAKE --> ZERO[静止且反馈有效时重新设零<br/>轴不动]
    ZERO --> NEW[新的短按才请求运动]
```

- **CAN 端口拥有硬件恢复。** F407 开启 bxCAN 自动 Bus-Off 恢复；G3507 在周期服务中检查硬件状态、撤销旧待发帧，并以至少250 ms间隔尝试异步恢复。不等待总线恢复完成，不阻塞本地保护；恢复活动期间拒绝普通发送。见 [CAN 端口](../firmware/g3507/user/can_port_mspm0.c#L144)。
- **执行端拥有安全输出。** Bus-Off 首次出现时建立本地 STOP、撤销演示、清除旧通信基准并使行程失效；已有更高优先级故障保持。见 [周期服务](../firmware/g3507/user/window_can_app.c#L1097)和[状态机](../shared/window_state.c#L228)。
- **F407 TX 任务拥有显式恢复请求。** 新的长按在新鲜 STOP 状态下走既有握手，再请求重新设零；本次长按之后的新短按最多等待1500 ms，故障、离线、STOP或超时取消。通信重连本身不产生运动请求。
- **UART 端口拥有反馈与范围。** 新的显式设零允许重建已有静止范围，要求反馈就绪、到位、无活动/待处理运动，并检查整数范围。见 [行程设定](../firmware/g3507/user/zdt_uart_mspm0.c#L118)。

默认关闭上电1000帧链路诊断，正式演示直接进入正常周期服务。诊断通过构建选项保留，见[构建说明](build-and-run.md#构建选项)。总线诊断计数与业务执行计数分别观察，不能用恢复尝试次数证明物理故障根因消失。

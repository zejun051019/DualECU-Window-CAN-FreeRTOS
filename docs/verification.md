# 版本绑定的台架验证

## 当前版本：演示恢复更新（2026-10-02）

测试ID为 `DEMO-RELIABILITY-20261002`。两节点实体CAN连接，F407/J-Link、G3507/XDS110；X42S空轴经UART受控。位置来自电机驱动器反馈，不是外部仪器机械角度测量。下面分别注明输入方式，调试事件经同一按键处理器与业务门控，但不代替GPIO按键测试。

| 场景 / 输入方式 | 可观察结果 | 样本 |
|---|---|---|
| 调试事件：长按设零、短按上升与回程 | 设零不动，范围0°–90°；终点89.5°、0.7°；最终STOP/NONE | [10份动作前后状态](evidence/demo-recovery-motion.json) |
| 调试事件：再上升、中途STOP、观察、新短按回程 | 停于11.2°并保持；新操作回到0.6°；最后active=0、STOP/NONE | 同上；[上行轨迹](evidence/demo-recovery-rise-trace.json)、[回程轨迹](evidence/demo-recovery-return-trace.json)、[停止后回程轨迹](evidence/demo-recovery-stop-return-trace.json) |
| 实际复位G3507 MCU，随后调试长按 | 旧范围失效；长按重新建立范围，位置0.7°保持，无自动运动 | [9份恢复状态](evidence/demo-recovery-reset-error.json) |
| 软件注入已解析设备错误E2，随后调试长按 | MOTOR_LOCAL/范围失效、F407锁存STOP；原握手后重新设零，STOP/NONE，位置保持 | 同上 |
| 操作者实体按键复测并反馈“可以了”，随后只读快照 | F407事件由此前8增至14、拒绝维持0；范围1.1°–91.1°，反馈1.8°、STOP/NONE、active=0，CAN在线 | [配对快照](evidence/demo-recovery-user-button.json)；未逐键采样，不由末态推导每次动作 |
| 连续只读双板观察 | 17份快照覆盖约291.5 s，CAN RX 2292→8092、状态TX 1151→4067；各轮离线/Bus-Off/故障为0，位置0.7°保持 | [连续状态](evidence/demo-recovery-monitor.json) |

动作轨迹中的角度以0.1°为单位保存。上下行各约十八秒；到位容差与电机内部闭环有关，不作为机械绝对精度声明。主机UTC标注读取顺序，不提供跨MCU共同时间基准。

### 当前台架镜像

| 节点 | 实际刷入的 ELF SHA-256 |
|---|---|
| F407 | `53DF58554398DDC3B8AF21F06B0ACD8BD981CBC8700BB20FCF49DA43F9B035B9` |
| G3507正式UART-control | `04F054600F38FCC259E60D859F4B0D609777A56C3257EDE9EB6FC04CAED61881` |

公开目录的644个固件、共享模块、配置和主机测试文件与本次维护源码逐文件校对一致，见[当前源码哈希](evidence/source-sha256.json)。重新构建的ELF包含构建路径等信息，哈希不要求与台架ELF相同。发布检查使用公开目录独立构建，两端编译链接通过；F407 FLASH 44368 B / RAM 21176 B，G3507 FLASH 19672 B / SRAM 6376 B。

## 前一发布版本：实体按键记录

测试ID为 `CAN-WINDOW-BUTTON-DEMO-001`。操作者完成实体按键和轴动作观察，调试器保存两端运行状态。这些记录绑定下方原镜像，用于追溯，不冒充新版的逐键轨迹。

| 场景 | 电机UART反馈与最终状态 | 样本 |
|---|---|---|
| 长按设零，紧接短按上升 | 范围1.0°–91.0°；终点90.5°；STOP/NONE | [150个样本](evidence/rise.json) |
| 上端新短按回程 | 终点1.5°；STOP/NONE | [47个样本](evidence/return.json) |
| 上升中再短按STOP | 中途18.1°，之后保持，不自行续转 | [26个样本](evidence/stop.json) |
| 中途停止后新短按 | 回到1.8°；STOP/NONE | [25个样本](evidence/return-after-stop.json) |

四段实体按键的拒绝计数均为0，最终无活动运动或故障。反馈终点差为0.5°、0.5°和0.8°，不等同外部仪器精度。

### 前版故障与旧路径回归

| 输入 | 可观察结果 | 证据类型 |
|---|---|---|
| 原UP/DOWN单步 | 1.8°→2.2°→1.8°，两次最终STOP/NONE | 原正式镜像运行状态 |
| 执行端已STOP时复位F407 | G3507 COMM_TIMEOUT/STOP，位置不变；握手后STOP/NONE，不自动续动 | [失联](evidence/can-loss.json)、[恢复](evidence/can-recovery.json) |
| 有界动作后软件丢弃UART回复 | G3507 MOTOR_LOCAL/STOP，范围失效，F407锁存STOP；恢复回复不自动解锁 | [原同源诊断构建](evidence/feedback-loss.json) |
| 注入已解析设备错误事件E2 | G3507 MOTOR_LOCAL/STOP，范围失效，F407锁存STOP | [原正式镜像应用事件注入](evidence/device-error.json) |

### 前版镜像与来源

| 镜像 | ELF SHA-256 |
|---|---|
| F407原正式 | `AE8EB936260BDC85D77E6AC0F05BAF59868FCFBD8518EB9D67368B4C420D7595` |
| G3507原正式 | `32CDA168C136B6F8FCB8B3F46086E15D43707D73702E0FF19EB3A05FDFE918D2` |
| G3507原同源诊断，启用故障注入 | `08CDCB97D79665E4F199F86829831454727A5581AE4873282F9F4A0135E98B80` |

原发布提交为 `4a09ba9fe314b0f58f8a23024c8d9940d5cae456`，[历史源码哈希](evidence/source-sha256-original.json)保留原版本来源。当前源码相较原版修改15个固件/配置/测试文件，未改变报文格式、任务分工与原保护链路。

## 主机与目标构建

[主机入口](../tools/run-project-host-checks.ps1)运行14组C检查，覆盖协议、恢复/新鲜度、MCAN时间戳、行程、请求锁存、按键、状态机、演示规划、UART解析等。新版补充Bus-Off本地STOP、旧运动拒绝、握手恢复不运动和长按通信恢复用例。[GitHub Actions](../.github/workflows/host-checks.yml)运行同一入口。

PC检查验证可移植C逻辑，目标构建验证编译链接，均不替代实体按键、电机和故障记录。F407构建仍有链接器RWX段警告；本次发布未重新刷写或操作电机，实物结果复用上述同源镜像记录。

## 验证范围

- 软件回复丢弃不等同物理UART拔线，解析后E2注入不等同电机真实报警，执行端MCU复位不等同电机12 V断电。
- 本地STOP是控制指令与逻辑状态，不等同独立硬件切断或仪器测得的机械停止时延。
- 间歇Bus-Off的物理触发根因尚未确定。早期补丁在受控位率失配中观察到进入/恢复，随后加入跨故障回合限速；该结果不作为最终版完整Bus-Off注入验收。最终版正常运行窗口Bus-Off计数为0，不代表长期可靠性已证明。
- 空轴演示不代表真实车窗、防夹、绝对位置、ASIL或量产车规能力。

样本保留UTC、输入动作和两端状态字段，去掉本机路径及脚本元数据。[证据索引](evidence/index.json)记录原始样本哈希、公开文件及输入类型；完整本地记录按测试ID归档。当前页展示工程证据，不代替作者个人独立调试能力验收。

# 版本绑定的台架验证

## 应用测试：2026-10-02

测试ID为 `CAN-WINDOW-BUTTON-DEMO-001`。两节点实体CAN连接，F407/J-Link、G3507/XDS110；X42S空轴经UART受控。操作者完成按键与轴动作观察，调试器保存两端运行状态。

| 场景 | 电机UART反馈与最终状态 | 运行样本 |
|---|---|---|
| 长按设零，紧接短按上升 | 范围1.0°–91.0°；终点90.5°；STOP/NONE | [150个样本](evidence/rise.json) |
| 上端新短按回程 | 终点1.5°；STOP/NONE | [47个样本](evidence/return.json) |
| 上升中再短按 STOP | 中途18.1°，之后保持，不自行续转 | [26个样本](evidence/stop.json) |
| 中途停止后新短按 | 回到1.8°；STOP/NONE | [25个样本](evidence/return-after-stop.json) |

四段实体按键的拒绝计数均为0，最终无活动运动或故障。终点反馈差为0.5°、0.5°和0.8°。这是电机驱动器反馈，不是外部仪器机械角度精度；主机UTC只标注读取顺序，不提供跨MCU共同时间基准。

## 故障与旧路径回归

| 输入 | 可观察结果 | 证据类型 |
|---|---|---|
| 原UP/DOWN单步 | 1.8°→2.2°→1.8°，两次最终STOP/NONE | 正式镜像运行状态 |
| 执行端已STOP时复位F407 | G3507 COMM_TIMEOUT/STOP，位置不变；握手后STOP/NONE，不自动续动 | [失联](evidence/can-loss.json)、[恢复](evidence/can-recovery.json) |
| 有界动作后软件丢弃UART回复 | G3507 MOTOR_LOCAL/STOP，范围失效，F407锁存STOP；恢复回复不自动解锁 | [诊断构建](evidence/feedback-loss.json) |
| 注入已解析设备错误事件0xE2 | G3507 MOTOR_LOCAL/STOP，范围失效，F407锁存STOP | [正式镜像应用事件注入](evidence/device-error.json) |

软件回复丢弃不等同物理UART拔线，解析后事件注入不等同电机真的上报报警。执行端本地STOP是控制指令/逻辑状态，不等同独立硬件切断或仪器测得的机械停止时延。

## 镜像与来源

| 镜像 | SHA-256 |
|---|---|
| F407正式 | `AE8EB936260BDC85D77E6AC0F05BAF59868FCFBD8518EB9D67368B4C420D7595` |
| G3507正式 | `32CDA168C136B6F8FCB8B3F46086E15D43707D73702E0FF19EB3A05FDFE918D2` |
| G3507同源诊断，启用故障注入 | `08CDCB97D79665E4F199F86829831454727A5581AE4873282F9F4A0135E98B80` |

表中是台架测试时刷入镜像的哈希；重新构建的ELF包含构建路径和工具信息，不要求哈希相同。公开目录中的固件/共享源码来自上述台架版本，逐文件校对一致，[源码哈希清单](evidence/source-sha256.json)可用于复核。样本保留UTC和两端状态字段，去掉本机路径及执行脚本元数据；[索引](evidence/index.json)记录各原始样本文件哈希，完整本地记录按测试ID归档。

## 主机与目标构建

[主机入口](../tools/run-project-host-checks.ps1)运行14组C检查，覆盖协议、恢复/新鲜度、MCAN时间戳、行程、请求锁存、按键、状态机、演示规划、UART解析等。[GitHub Actions](../.github/workflows/host-checks.yml)运行同一入口。

两板从公开目录独立配置构建；PC检查仅验证可移植C逻辑，目标构建仅验证编译链接，不代替上面的实体按键、电机和故障记录。

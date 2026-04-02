# h7_tof_safety_bridge

面向纯 TOF STM32 H7 安全桥的 ROS1 SDK。

## 功能

- 发布 4 路 TOF 状态、急停状态、各类掩码、阈值和人工解除剩余时间
- 通过 `dynamic_reconfigure` 管理 `baseline_mm / tolerance_mm / release_hold_ms`
- 提供 `/h7_tof/release_estop` 服务触发定时人工解除
- 当 TOF 通道无效、超量程或越阈值时，通过 `/diagnostics` 发布告警
- 即使板端没有回报 `B/T/TH`，ROS 仍可继续下发参数，但会明确标记为“未确认”

## ROS 接口

发布：

- `/h7_tof/status`，类型 `h7_tof_safety_bridge/TofSafetyStatus`
- `/h7_tof/estop`，类型 `std_msgs/Bool`
- `/h7_tof/raw_line`，类型 `std_msgs/String`
- `/h7_tof/distances_mm`，类型 `std_msgs/UInt16MultiArray`
- `/diagnostics`，类型 `diagnostic_msgs/DiagnosticArray`

服务：

- `/h7_tof/release_estop`，类型 `std_srvs/Trigger`

## Dynamic Reconfigure 参数

- `baseline_mm`
- `tolerance_mm`
- `release_hold_ms`

## 启动方式

主桥接节点：

```bash
roslaunch h7_tof_safety_bridge h7_tof_safety_bridge.launch
```

常用参数：

- `port`
- `baud`
- `send_rate_hz`
- `prefer_board_persisted_params`
- `distance_topic`
- `diagnostics_topic`
- `baseline_mm`
- `tolerance_mm`
- `release_hold_ms`

## 说明

- 无效或超量程 TOF 数据统一发布为 `65535`
- `has_board_params=false` 表示板端状态帧没有回报 `B/T/TH`；这时 ROS 仍可下发参数，但无法仅凭状态帧确认板端是否已采纳
- 当 `release_remaining_ms > 0` 时，板端会临时抑制 `self_estop`；如果时间到后故障仍在，急停会再次拉起

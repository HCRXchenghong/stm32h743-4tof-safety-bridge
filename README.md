# stm32h743-4tof-safety-bridge

基于 STM32H743 的纯 TOF 安全桥，支持 4 路 UART TOF 传感器、本地急停判定、USB CDC 控制、ROS1 SDK、ROS1 `rqt` 调试面板，以及 Windows 串口调试工具。

这是一次破坏性协议升级。固件、ROS SDK、ROS `rqt` 和 Windows 工具需要整套一起升级；旧版基于运动方向和 `takeover` 语义的工具不再兼容当前纯 TOF 版本。

## 项目概览

- STM32 板端采集 4 路 TOF 距离
- 板端本地完成阈值判定和急停输出
- 板端通过 USB CDC 持续上报纯 TOF `H7TOF` 状态帧
- 上位机可通过 `H7CTL` 下发 `baseline_mm`、`tolerance_mm` 和人工解除时间
- 无效、超量程或故障距离统一按 `65535` 上报

## 控制协议

控制帧格式：

```text
$H7CTL,seq,release_req,baseline_mm,tolerance_mm,release_hold_ms*CS
```

字段说明：

- `seq`：控制帧序号
- `release_req`：是否请求人工解除，`1` 表示请求，`0` 表示不请求
- `baseline_mm`：基准距离，单位 `mm`
- `tolerance_mm`：容差距离，单位 `mm`
- `release_hold_ms`：人工解除持续时间，单位 `ms`

## 状态协议

状态帧格式：

```text
$H7TOF,seq,tof1,tof2,tof3,tof4,estop,valid_mask,fault_mask,R=...,F=...,S=...,C=...,E=...,L=...,TM=trip_mask,A=active_mask,SE=self_estop,EE=external_estop,RT=release_remaining_ms,B=baseline_mm,T=tolerance_mm,TH=threshold_mm*CS
```

重点字段：

- `tof1..tof4`：4 路 TOF 距离，单位 `mm`
- `valid_mask`：当前有效通道掩码
- `fault_mask`：当前导致板端安全故障的通道掩码
- `TM`：纯越阈值掩码
- `A`：生效通道掩码，目前固定为 `0x0F`
- `SE`：板端急停标志
- `EE`：外部急停兼容标志，目前固定为 `0`
- `RT`：人工解除剩余时间，单位 `ms`
- `B` / `T` / `TH`：当前运行时基准值、容差值和阈值

行为说明：

- 原始 TOF 读数为 `0` 或无效值时，板端按无效/超量程处理，并对外上报 `65535`
- 无效、掉线、超量程通道会从 `valid_mask` 中移除，并进入 `fault_mask`
- 只要任一生效通道无效或超过阈值，且当前没有人工解除保护，板端就会急停
- 人工解除计时结束后如果故障仍在，板端会再次急停

## Windows 调试工具

路径：

- `tools/windows_serial_console`

包含内容：

- 源码：`tools/windows_serial_console/Program.cs`
- 构建脚本：`tools/windows_serial_console/build.ps1`
- 已编译程序：`tools/windows_serial_console/bin/H7TofSerialConsole.exe`

主要功能：

- 串口连接、断开、自动重连
- 4 路 TOF 距离显示
- 急停状态、掩码、阈值、人工解除剩余时间显示
- 在线下发 `baseline_mm / tolerance_mm`
- 触发人工解除
- 根据 `B/T/TH` 回读判断板端是否真正确认参数
- 本地保存串口、波特率和人工解除时间

## ROS1 SDK

路径：

- `sdk/h7_tof_safety_bridge_ros1`

主要功能：

- 将纯 TOF 协议桥接为 ROS 话题、服务和诊断信息
- 通过 `dynamic_reconfigure` 管理 `baseline_mm / tolerance_mm / release_hold_ms`
- 提供 `/h7_tof/release_estop` 服务触发人工解除
- 在状态消息里提供 `has_board_params`，用于区分“板端真实回读参数”和“仅本地配置值”

## ROS1 rqt 调试面板

路径：

- `sdk/h7_tof_safety_bridge_rqt`

主要功能：

- 显示 4 路 TOF、急停状态、掩码、阈值和人工解除剩余时间
- 通过 ROS SDK 下发 `baseline_mm / tolerance_mm`
- 通过 ROS SDK 服务触发人工解除
- 显示 `/h7_tof/raw_line` 原始日志
- 不直接管理串口

直接启动：

```bash
roslaunch h7_tof_safety_bridge_rqt h7_tof_safety_bridge_rqt.launch
```

同时启动 SDK 和 rqt：

```bash
roslaunch h7_tof_safety_bridge_rqt h7_tof_safety_bridge_with_rqt.launch
```

手动独立打开插件：

```bash
rqt --standalone h7_tof_safety_bridge_rqt/H7TofSafetyBridge
```

## 固件产物

主构建输出：

- `firmware/h743_tof_usb_bridge_cubeide/build/Release/stm32h743-4tof-safety-bridge.hex`

发布目录副本：

- `release/stm32h743-4tof-safety-bridge.hex`

## 目录结构

```text
stm32h743-4tof-safety-bridge/
|-- firmware/
|-- release/
|-- sdk/
|   |-- h7_tof_safety_bridge_ros1/
|   `-- h7_tof_safety_bridge_rqt/
|-- tools/
|   `-- windows_serial_console/
`-- README.md
```

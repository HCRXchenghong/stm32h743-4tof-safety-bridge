# stm32h743-4tof-safety-bridge

基于 `STM32H743ZITx` 的 4 路 UART TOF 安全桥接固件，提供 USB CDC 状态上报、板端急停逻辑、阈值在线下发，以及 Windows 串口调试工具。

## 当前实现

- 4 路 TOF 物理定义固定为：
  - `TOF1 = 右前`
  - `TOF2 = 右后`
  - `TOF3 = 左后`
  - `TOF4 = 左前`
- 位图统一按 `bit0..bit3 = TOF1..TOF4`
- `active_mask` 当前固定为 `0x0F`
- 默认参数：
  - `baseline_mm = 391`
  - `tolerance_mm = 28`
  - `threshold_mm = baseline_mm + tolerance_mm`
- 板端急停规则：
  - 任一有效 TOF 距离 `> threshold_mm` 时，置位 `self_estop`
  - 任一激活 TOF 超量程、无有效距离、掉线或故障时，也会置位 `self_estop`
  - ROS 或 Windows 可以发送“人工解除 + 持续时间”，在剩余时间内即使故障仍存在，也会临时保持 `self_estop=0`
  - 人工解除时间结束后，如果 TOF 故障或超阈值仍存在，会再次进入 `self_estop`
  - `estop = self_estop`
  - `external_estop` 当前保留兼容字段，固定为 `0`
- `self_estop` 可由 `release_req` 人工临时解除，解除时长由 `release_hold_ms` 指定

## 掉电保存

`baseline_mm / tolerance_mm` 已支持掉电保存。

- 板端收到新的阈值参数后，不会立即在接收回调中写 Flash
- 固件会在主循环中延迟约 `1s` 落盘，降低频繁擦写和中断上下文风险
- 参数保存在 STM32 内部 Flash 预留区：`0x081E0000`，大小 `128KB`
- 重新上电后，固件会自动加载上次成功保存的 `baseline_mm / tolerance_mm`

## 参数初始化来源

系统当前统一以 STM32 板端已保存参数作为初始化真值。

- Win 串口工具：
  - 启动软件时，不把本地缓存当成板端当前值
  - 连接板子后，先等待板端状态帧中的 `B/T/TH`
  - 收到板端参数后，再同步到界面输入框
  - 在板端参数同步完成前，“下发参数”按钮不可用，避免误覆盖
- ROS1 SDK：
  - 默认 `prefer_board_persisted_params=true`
  - 节点启动后，先读取板端状态帧里的 `baseline_mm / tolerance_mm`
  - 把板端当前值写回 ROS1 参数和 `dynamic_reconfigure`
  - 在完成这一步之前，ROS1 不会先用 launch 默认值抢先覆盖 STM32
- 典型使用流程：
  - 先用 Windows 工具修改 `baseline_mm / tolerance_mm`
  - STM32 自动掉电保存
  - 再把 USB 接到 ROS1 上位机
  - ROS1 启动后自动继承 STM32 上次保存值，作为初始化值继续运行

## 控制输入协议

固件通过 USB CDC 接收文本行，支持 `\n` 或 `\r\n` 结尾。

推荐控制帧：

```text
$H7CTL,seq,vx_mmps,vy_mmps,wz_mradps,release_req,takeover_enable,baseline_mm,tolerance_mm,release_hold_ms*CS
```

说明：

- `vx_mmps / vy_mmps / wz_mradps` 仅用于推导 `motion_class`
- `release_req` 用于触发人工解除
- `takeover_enable` 仅透传到状态帧，供上位机显示
- `baseline_mm / tolerance_mm` 用于运行时更新板端阈值
- `release_hold_ms` 为人工解除持续时间，单位 `ms`

兼容输入：

- 文本：`FWD` / `REV` / `TURN` / `STRAFE` / `MIXED` / `IDLE` / `RELEASE`
- 三元组：`vx,vy,wz`
- JSON：`{"vx":...,"vy":...,"wz":...,"baseline_mm":...,"tolerance_mm":...}`

## STM32 输出协议

当前主状态帧格式：

```text
$H7TOF,seq,tof1,tof2,tof3,tof4,estop,valid_mask,fault_mask,R=...,F=...,S=...,C=...,E=...,L=...,TM=trip_mask,A=active_mask,M=motion_class,SE=self_estop,EE=external_estop,TK=takeover_enable,RT=release_remaining_ms,B=baseline_mm,T=tolerance_mm,TH=threshold_mm*CS
```

关键字段：

- `tof1..tof4`：距离，单位 `mm`
- `estop`：总急停
- `valid_mask`：有效 TOF 位图
- `fault_mask`：导致板端急停的位图
- `TM`：纯距离超阈值位图
- `A`：当前生效位图，当前固定 `0x0F`
- `M`：运动分类
- `SE`：板端 `self_estop`
- `EE`：外部急停兼容位，当前固定 `0`
- `RT`：人工解除剩余时间，单位 `ms`
- `B`：当前 `baseline_mm`
- `T`：当前 `tolerance_mm`
- `TH`：当前阈值 `threshold_mm`
- 无效距离统一为 `65535`
- TOF 原始上报若为 `0`，板端会按“超量程/无效”处理，并同样上报为 `65535`
- 当某路 TOF 超量程、掉线或坏掉时，对应位会从 `valid_mask` 清除，并进入 `fault_mask`

## Windows 串口工具

路径：`tools/windows_serial_console`

已提供：

- 源码：`tools/windows_serial_console/Program.cs`
- 构建脚本：`tools/windows_serial_console/build.ps1`
- 已编译程序：`tools/windows_serial_console/bin/H7TofSerialConsole.exe`

功能：

- 串口连接 / 断开 / 自动重连
- 实时显示 `tof1~tof4`
- 显示 `estop / self_estop / external_estop`
- 显示 `valid_mask / fault_mask / trip_mask / active_mask / motion_class`
- 在线修改 `baseline_mm / tolerance_mm`
- 配置“人工解除持续时间(ms)”并发送真实板端解除命令
- 发送原始文本命令
- 快捷发送 `FWD / REV / TURN / STRAFE / IDLE`
- 提供“恢复急停”按钮，可真实发送带持续时间的人工解除命令
- 本地记住上次串口、波特率、原始指令
- 本地记住上次人工解除持续时间
- 阈值显示与下发以板端当前回读值为准，不再把本地缓存当成板端真值
- 参数下发后等待板端回读 `B/T/TH` 作为确认

## 目录结构

```text
stm32h743-4tof-safety-bridge/
|-- README.md
|-- firmware/
|   `-- h743_tof_usb_bridge_cubeide/
|-- sdk/
|   `-- h7_tof_safety_bridge_ros1/
|-- tools/
|   `-- windows_serial_console/
`-- release/
```

## 构建固件

固件工程路径：

- `firmware/h743_tof_usb_bridge_cubeide`

命令行构建：

```powershell
powershell -ExecutionPolicy Bypass -File scripts/build_firmware.ps1 -Configuration Release
```

产物：

- `firmware/h743_tof_usb_bridge_cubeide/build/Release/stm32h743-4tof-safety-bridge.hex`
- `release/stm32h743-4tof-safety-bridge.hex`

## 预编译产物

仓库当前包含：

- 固件：`release/stm32h743-4tof-safety-bridge.hex`
- Windows 工具：`tools/windows_serial_console/bin/H7TofSerialConsole.exe`

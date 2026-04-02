# h7_tof_safety_bridge_rqt

面向纯 TOF STM32 H7 安全桥的 ROS1 `rqt` 调试插件。

## 功能

- 显示 4 路 TOF 距离
- 显示急停状态、有效掩码、故障掩码、越界掩码、生效掩码
- 显示阈值和人工解除剩余时间
- 通过 `dynamic_reconfigure` 下发 `baseline_mm / tolerance_mm`
- 通过 `/h7_tof/release_estop` 触发定时人工解除
- 显示 `/h7_tof/raw_line` 原始日志

## 启动方式

直接启动：

```bash
roslaunch h7_tof_safety_bridge_rqt h7_tof_safety_bridge_rqt.launch
```

手动打开插件：

```bash
rqt --standalone h7_tof_safety_bridge_rqt/H7TofSafetyBridge
```

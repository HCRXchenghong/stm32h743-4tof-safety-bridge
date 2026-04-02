# h7_tof_safety_bridge_rqt

ROS1 `rqt` plugin for the pure-TOF STM32 H7 safety bridge.

- Displays 4 TOF channels, estop state, masks, threshold, and manual-release remaining time
- Applies `baseline_mm` / `tolerance_mm` through `dynamic_reconfigure`
- Triggers timed manual release through `/h7_tof/release_estop`
- Shows `/h7_tof/raw_line` logs

Launch directly:

```bash
roslaunch h7_tof_safety_bridge_rqt h7_tof_safety_bridge_rqt.launch
```

Or open the plugin manually:

```bash
rqt --standalone h7_tof_safety_bridge_rqt/H7TofSafetyBridge
```

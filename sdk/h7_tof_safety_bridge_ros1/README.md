# h7_tof_safety_bridge

ROS1 SDK for the pure-TOF STM32 H7 safety bridge.

## Features

- Publishes 4-channel TOF status, estop state, masks, threshold, and manual-release remaining time
- Applies `baseline_mm` / `tolerance_mm` / `release_hold_ms` through `dynamic_reconfigure`
- Exposes `/h7_tof/release_estop` for timed manual release
- Publishes `/diagnostics` warnings when TOF channels are invalid, out of range, or threshold-tripped
- Keeps working even if the board does not report `B/T/TH`, but marks that state as unconfirmed

## ROS Interfaces

Published:

- `/h7_tof/status` (`h7_tof_safety_bridge/TofSafetyStatus`)
- `/h7_tof/estop` (`std_msgs/Bool`)
- `/h7_tof/raw_line` (`std_msgs/String`)
- `/h7_tof/distances_mm` (`std_msgs/UInt16MultiArray`)
- `/diagnostics` (`diagnostic_msgs/DiagnosticArray`)

Service:

- `/h7_tof/release_estop` (`std_srvs/Trigger`)

## Dynamic Reconfigure

- `baseline_mm`
- `tolerance_mm`
- `release_hold_ms`

## Launch

Main bridge launch:

```bash
roslaunch h7_tof_safety_bridge h7_tof_safety_bridge.launch
```

Useful args:

- `port`
- `baud`
- `send_rate_hz`
- `prefer_board_persisted_params`
- `distance_topic`
- `diagnostics_topic`
- `baseline_mm`
- `tolerance_mm`
- `release_hold_ms`

## Notes

- Invalid or out-of-range TOF readings are published as `65535`.
- `has_board_params=false` means the board did not report `B/T/TH`; ROS can still send parameters, but cannot confirm board acceptance from status frames alone.
- When `release_remaining_ms > 0`, the board temporarily suppresses `self_estop`. If the fault remains after the timer expires, estop asserts again.

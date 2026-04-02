# stm32h743-4tof-safety-bridge

Pure-TOF safety bridge for an STM32H743 board with 4 UART TOF sensors, USB CDC control, a ROS1 SDK, a ROS1 `rqt` panel, and a Windows serial console.

This is a breaking protocol update. Firmware, ROS SDK, ROS `rqt`, and the Windows tool should be upgraded together; older motion/takeover-oriented tools are not compatible with this pure-TOF release.

## Overview

- 4 TOF channels are sampled on the STM32 board
- The board evaluates threshold-based safety locally and drives an estop output
- The board reports status over USB CDC with a pure-TOF `H7TOF` frame
- Upper layers can update `baseline_mm` / `tolerance_mm` and trigger timed manual release with `H7CTL`
- Invalid, out-of-range, or failed TOF values are reported as `65535`

## Control Protocol

Control frame:

```text
$H7CTL,seq,release_req,baseline_mm,tolerance_mm,release_hold_ms*CS
```

Fields:

- `release_req`: `1` to trigger timed manual release, otherwise `0`
- `baseline_mm`: base distance used for threshold evaluation
- `tolerance_mm`: extra tolerance above baseline
- `release_hold_ms`: manual-release duration in milliseconds

## Status Protocol

Status frame:

```text
$H7TOF,seq,tof1,tof2,tof3,tof4,estop,valid_mask,fault_mask,R=...,F=...,S=...,C=...,E=...,L=...,TM=trip_mask,A=active_mask,SE=self_estop,EE=external_estop,RT=release_remaining_ms,B=baseline_mm,T=tolerance_mm,TH=threshold_mm*CS
```

Important fields:

- `tof1..tof4`: TOF distances in `mm`
- `valid_mask`: valid TOF channels
- `fault_mask`: channels currently causing board-side safety fault
- `TM`: pure threshold-trip mask
- `A`: active channel mask, currently fixed to `0x0F`
- `SE`: board-side estop flag
- `EE`: external estop compatibility flag, currently fixed to `0`
- `RT`: remaining manual-release time in `ms`
- `B` / `T` / `TH`: current runtime baseline, tolerance, and threshold

Behavior:

- A TOF value of `0` is treated as invalid/out-of-range and reported as `65535`
- Missing, failed, or out-of-range channels are removed from `valid_mask` and added to `fault_mask`
- If any active channel is invalid or above threshold, the board asserts estop unless a timed manual release is currently active
- When the release timer expires and the fault still exists, the board asserts estop again

## Windows Tool

Path:

- `tools/windows_serial_console`

Provided:

- Source: `tools/windows_serial_console/Program.cs`
- Build script: `tools/windows_serial_console/build.ps1`
- Built app: `tools/windows_serial_console/bin/H7TofSerialConsole.exe`

Features:

- Serial connect / disconnect / auto reconnect
- 4-channel TOF display
- Estop, masks, threshold, and release-remaining display
- Runtime update of `baseline_mm / tolerance_mm`
- Timed manual release
- Board confirmation workflow using `B/T/TH` readback
- Local persistence of port, baud rate, and manual-release duration

## ROS1 SDK

Path:

- `sdk/h7_tof_safety_bridge_ros1`

Features:

- ROS topic and diagnostics bridge for the pure-TOF protocol
- `dynamic_reconfigure` for `baseline_mm / tolerance_mm / release_hold_ms`
- `/h7_tof/release_estop` service for timed manual release
- Status includes `has_board_params` so tools can distinguish real board readback from unconfirmed local values

## ROS1 rqt Panel

Path:

- `sdk/h7_tof_safety_bridge_rqt`

Features:

- Displays TOF channels, estop state, masks, threshold, and release remaining time
- Applies `baseline_mm / tolerance_mm` through the ROS SDK
- Triggers timed manual release through the ROS SDK service
- Shows `/h7_tof/raw_line` logs
- Does not manage serial ports directly

Launch:

```bash
roslaunch h7_tof_safety_bridge_rqt h7_tof_safety_bridge_rqt.launch
```

Or start both bridge + rqt together:

```bash
roslaunch h7_tof_safety_bridge_rqt h7_tof_safety_bridge_with_rqt.launch
```

Standalone plugin ID:

```bash
rqt --standalone h7_tof_safety_bridge_rqt/H7TofSafetyBridge
```

## Firmware Build Output

Main build output:

- `firmware/h743_tof_usb_bridge_cubeide/build/Release/stm32h743-4tof-safety-bridge.hex`

Release copy:

- `release/stm32h743-4tof-safety-bridge.hex`

## Directory Layout

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

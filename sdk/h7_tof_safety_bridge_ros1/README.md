# h7_tof_safety_bridge ROS1 SDK

ROS1 bridge for the STM32 H7 4-TOF safety board.

## What Changed

- All 4 TOF channels stay active all the time.
- The H7 now trips `self_estop` when a valid TOF distance is greater than `baseline_mm + tolerance_mm`.
- Missing, out-of-range, or failed TOF data on any active channel also trips `self_estop`.
- Distances lower than the baseline do not trigger `self_estop`.
- ROS can queue a timed manual release; while `release_remaining_ms > 0`, the H7 keeps `self_estop=0` even if TOF faults still exist.
- When the manual-release timer expires, the H7 asserts `self_estop` again if any TOF fault is still present.
- `baseline_mm` and `tolerance_mm` can be changed at runtime from `rqt_reconfigure`.

## ROS Interfaces

- Subscribed: `/cmd_vel`
- Subscribed: `/h7_tof/takeover_enable`
- Published: `/h7_tof/status`
- Published: `/h7_tof/estop`
- Published: `/h7_tof/raw_line`
- Published: `/h7_tof/distances_mm`
- Published: `/diagnostics`
- Service: `/h7_tof/release_estop` (queues an H7 timed manual-release command)

`/h7_tof/status` now also includes:

- `trip_mask`
- `missing_data_mask`
- `missing_data_labels`
- `baseline_mm`
- `tolerance_mm`
- `threshold_mm`
- `release_remaining_ms`

The SDK derives missing-data errors with:

```text
missing_data_mask = active_mask & (~valid_mask)
```

Channel labels are:

- `TOF1 = right_front`
- `TOF2 = right_rear`
- `TOF3 = left_rear`
- `TOF4 = left_front`

When a TOF channel is out of range, missing, or failed, the node still uploads the status to ROS through `/h7_tof/status`, `/h7_tof/distances_mm`, and `/diagnostics`.
The H7 asserts `self_estop` in that situation unless a timed manual release is currently active; `/diagnostics` also reports the remaining manual-release time.

## Dynamic Reconfigure

After launching the node, open:

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

Then adjust:

- `baseline_mm`
- `tolerance_mm`
- `release_hold_ms`

The bridge sends the updated values to the H7 in the periodic `H7CTL` frame.

## Launch

```bash
roslaunch h7_tof_safety_bridge h7_tof_safety_bridge.launch port:=/dev/ttyACM0
```

Optional launch args:

- `distance_topic`
- `diagnostics_topic`
- `baseline_mm`
- `tolerance_mm`
- `cmd_vel_topic`
- `takeover_topic`

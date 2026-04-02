#!/usr/bin/env python3
import threading

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rospy
import serial
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String, UInt16MultiArray
from std_srvs.srv import Trigger, TriggerResponse

from h7_tof_safety_bridge.cfg import SafetyBridgeConfig
from h7_tof_safety_bridge.msg import TofSafetyStatus
from h7_tof_safety_bridge.protocol import (
    DEFAULT_BASELINE_MM,
    DEFAULT_TOLERANCE_MM,
    ControlCommand,
    TOF_CHANNEL_LABELS,
    active_missing_mask,
    build_control_frame,
    mask_to_tof_labels,
    parse_status_line,
)


class H7TofBridgeNode:
    def __init__(self) -> None:
        self.port_name = rospy.get_param("~port", "/dev/ttyACM0")
        self.baud_rate = int(rospy.get_param("~baud", 115200))
        self.send_rate_hz = float(rospy.get_param("~send_rate_hz", 20.0))
        self.prefer_board_persisted_params = bool(rospy.get_param("~prefer_board_persisted_params", True))
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.takeover_topic = rospy.get_param("~takeover_topic", "/h7_tof/takeover_enable")
        self.status_topic = rospy.get_param("~status_topic", "/h7_tof/status")
        self.estop_topic = rospy.get_param("~estop_topic", "/h7_tof/estop")
        self.raw_topic = rospy.get_param("~raw_topic", "/h7_tof/raw_line")
        self.distance_topic = rospy.get_param("~distance_topic", "/h7_tof/distances_mm")
        self.diagnostics_topic = rospy.get_param("~diagnostics_topic", "/diagnostics")
        self.release_hold_ms = int(rospy.get_param("~release_hold_ms", 3000))

        self.seq = 0
        self.vx_mmps = 0
        self.vy_mmps = 0
        self.wz_mradps = 0
        self.takeover_enabled = False
        self.release_requested = False
        self.baseline_mm = int(rospy.get_param("~baseline_mm", DEFAULT_BASELINE_MM))
        self.tolerance_mm = int(rospy.get_param("~tolerance_mm", DEFAULT_TOLERANCE_MM))
        self.params_bootstrapped = not self.prefer_board_persisted_params
        self.lock = threading.Lock()
        self.running = True

        self.status_pub = rospy.Publisher(self.status_topic, TofSafetyStatus, queue_size=20)
        self.estop_pub = rospy.Publisher(self.estop_topic, Bool, queue_size=20)
        self.raw_pub = rospy.Publisher(self.raw_topic, String, queue_size=50)
        self.distance_pub = rospy.Publisher(self.distance_topic, UInt16MultiArray, queue_size=20)
        self.diagnostics_pub = rospy.Publisher(self.diagnostics_topic, DiagnosticArray, queue_size=20)

        rospy.Subscriber(self.cmd_vel_topic, Twist, self.on_cmd_vel, queue_size=10)
        rospy.Subscriber(self.takeover_topic, Bool, self.on_takeover, queue_size=10)
        rospy.Service("/h7_tof/release_estop", Trigger, self.handle_release_estop)

        self.dynamic_reconfigure_server = Server(SafetyBridgeConfig, self.on_reconfigure)

        self.serial_port = serial.Serial(
            port=self.port_name,
            baudrate=self.baud_rate,
            timeout=0.05,
            write_timeout=0.05,
        )

        self.reader_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.reader_thread.start()
        self.send_timer = rospy.Timer(rospy.Duration.from_sec(1.0 / self.send_rate_hz), self.send_control_frame)
        rospy.on_shutdown(self.shutdown)

    def on_cmd_vel(self, msg: Twist) -> None:
        with self.lock:
            self.vx_mmps = int(round(msg.linear.x * 1000.0))
            self.vy_mmps = int(round(msg.linear.y * 1000.0))
            self.wz_mradps = int(round(msg.angular.z * 1000.0))

    def on_takeover(self, msg: Bool) -> None:
        with self.lock:
            self.takeover_enabled = bool(msg.data)

    def on_reconfigure(self, config: SafetyBridgeConfig, _level: int) -> SafetyBridgeConfig:
        with self.lock:
            self.baseline_mm = max(0, int(config.baseline_mm))
            self.tolerance_mm = max(0, int(config.tolerance_mm))
            self.params_bootstrapped = True
            config.baseline_mm = self.baseline_mm
            config.tolerance_mm = self.tolerance_mm
        return config

    def handle_release_estop(self, _req: Trigger) -> TriggerResponse:
        with self.lock:
            self.release_requested = True
            release_hold_ms = self.release_hold_ms
        return TriggerResponse(success=True, message="queued H7 estop release request for %d ms" % release_hold_ms)

    def send_control_frame(self, _event) -> None:
        with self.lock:
            if not self.params_bootstrapped:
                return
            release_req = self.release_requested
            if release_req:
                self.release_requested = False
            command = ControlCommand(
                seq=self.seq,
                vx_mmps=self.vx_mmps,
                vy_mmps=self.vy_mmps,
                wz_mradps=self.wz_mradps,
                release_req=release_req,
                release_hold_ms=self.release_hold_ms,
                takeover_enable=self.takeover_enabled,
                baseline_mm=self.baseline_mm,
                tolerance_mm=self.tolerance_mm,
            )
            self.seq += 1

        frame = build_control_frame(command)
        try:
            self.serial_port.write(frame.encode("ascii"))
        except serial.SerialException as exc:
            if command.release_req:
                with self.lock:
                    self.release_requested = True
            rospy.logerr_throttle(1.0, "failed to write H7CTL: %s", exc)

    def read_loop(self) -> None:
        while self.running and not rospy.is_shutdown():
            try:
                raw = self.serial_port.readline()
            except serial.SerialException as exc:
                rospy.logerr_throttle(1.0, "failed to read H7TOF: %s", exc)
                rospy.sleep(0.05)
                continue

            if not raw:
                continue

            try:
                line = raw.decode("utf-8", errors="replace").strip()
            except Exception:
                continue

            if not line:
                continue

            self.raw_pub.publish(String(data=line))

            try:
                frame = parse_status_line(line)
            except ValueError as exc:
                rospy.logwarn_throttle(1.0, "drop invalid H7TOF frame: %s", exc)
                continue

            self.bootstrap_params_from_board(frame)

            status_msg = TofSafetyStatus()
            status_msg.header.stamp = rospy.Time.now()
            status_msg.seq = frame.seq
            status_msg.tof_mm = frame.tof_mm
            status_msg.estop = frame.estop
            status_msg.self_estop = frame.self_estop
            status_msg.external_estop = frame.external_estop
            status_msg.active_mask = frame.active_mask
            status_msg.trip_mask = frame.trip_mask
            status_msg.valid_mask = frame.valid_mask
            status_msg.fault_mask = frame.fault_mask
            status_msg.missing_data_mask = active_missing_mask(frame.active_mask, frame.valid_mask)
            status_msg.motion_mode = frame.motion_mode
            status_msg.takeover_enabled = frame.takeover_enabled
            status_msg.baseline_mm = frame.baseline_mm
            status_msg.tolerance_mm = frame.tolerance_mm
            status_msg.threshold_mm = frame.threshold_mm
            status_msg.release_remaining_ms = frame.release_remaining_ms
            status_msg.missing_data_labels = mask_to_tof_labels(status_msg.missing_data_mask)

            self.status_pub.publish(status_msg)
            self.estop_pub.publish(Bool(data=frame.estop))
            self.distance_pub.publish(UInt16MultiArray(data=list(frame.tof_mm)))
            self.publish_tof_diagnostics(frame, status_msg.missing_data_mask, status_msg.missing_data_labels)

            if frame.estop:
                rospy.logerr_throttle(
                    1.0,
                    "H7 estop active: fault_mask=0x%02X trip_mask=0x%02X missing=%s",
                    frame.fault_mask,
                    frame.trip_mask,
                    ", ".join(status_msg.missing_data_labels) if status_msg.missing_data_labels else "none",
                )
            elif frame.fault_mask and frame.release_remaining_ms > 0:
                rospy.logwarn_throttle(
                    1.0,
                    "H7 manual release active: remaining=%d ms fault_mask=0x%02X trip_mask=0x%02X",
                    frame.release_remaining_ms,
                    frame.fault_mask,
                    frame.trip_mask,
                )
            elif status_msg.missing_data_labels:
                rospy.logwarn_throttle(
                    1.0,
                    "TOF needs inspection on %s (active_mask=0x%02X valid_mask=0x%02X)",
                    ", ".join(status_msg.missing_data_labels),
                    frame.active_mask,
                    frame.valid_mask,
                )

    def bootstrap_params_from_board(self, frame) -> None:
        with self.lock:
            if self.params_bootstrapped:
                return
            if not frame.has_board_params:
                rospy.logwarn_throttle(
                    5.0,
                    "H7 status frames do not include B/T/TH; board may still be running older firmware, so ROS will keep waiting for real board parameter readback",
                )
                return

            self.baseline_mm = int(frame.baseline_mm)
            self.tolerance_mm = int(frame.tolerance_mm)
            self.params_bootstrapped = True
            baseline_mm = self.baseline_mm
            tolerance_mm = self.tolerance_mm

        try:
            self.dynamic_reconfigure_server.update_configuration(
                {
                    "baseline_mm": baseline_mm,
                    "tolerance_mm": tolerance_mm,
                }
            )
        except Exception as exc:
            rospy.logwarn("failed to update dynamic_reconfigure from board params: %s", exc)

        rospy.set_param("~baseline_mm", baseline_mm)
        rospy.set_param("~tolerance_mm", tolerance_mm)
        rospy.loginfo(
            "initialized ROS1 baseline/tolerance from H7 persisted config: baseline_mm=%d tolerance_mm=%d",
            baseline_mm,
            tolerance_mm,
        )

    def publish_tof_diagnostics(self, frame, missing_mask: int, missing_labels) -> None:
        diagnostic_array = DiagnosticArray()
        diagnostic_array.header.stamp = rospy.Time.now()

        diagnostic_status = DiagnosticStatus()
        diagnostic_status.name = "h7_tof_bridge/tof_data"
        diagnostic_status.hardware_id = self.port_name
        trip_labels = mask_to_tof_labels(frame.trip_mask)
        fault_labels = mask_to_tof_labels(frame.fault_mask)

        if frame.fault_mask and frame.release_remaining_ms <= 0:
            diagnostic_status.level = DiagnosticStatus.ERROR
            if missing_labels:
                diagnostic_status.message = "Safety stop: TOF missing/out-of-range on " + ", ".join(missing_labels)
            elif trip_labels:
                diagnostic_status.message = "Safety stop: threshold exceeded on " + ", ".join(trip_labels)
            else:
                diagnostic_status.message = "Safety stop: TOF fault detected on " + ", ".join(fault_labels)
        elif frame.fault_mask and frame.release_remaining_ms > 0:
            diagnostic_status.level = DiagnosticStatus.WARN
            diagnostic_status.message = (
                "Safety override active for %d ms on %s"
                % (frame.release_remaining_ms, ", ".join(fault_labels) if fault_labels else "active faults")
            )
        elif missing_labels:
            diagnostic_status.level = DiagnosticStatus.WARN
            diagnostic_status.message = "TOF needs inspection: " + ", ".join(missing_labels)
        else:
            diagnostic_status.level = DiagnosticStatus.OK
            diagnostic_status.message = "All active TOF channels reporting valid data"

        diagnostic_status.values = [
            KeyValue(key="estop", value=str(bool(frame.estop)).lower()),
            KeyValue(key="self_estop", value=str(bool(frame.self_estop)).lower()),
            KeyValue(key="missing_data_mask", value=f"0x{missing_mask:02X}"),
            KeyValue(key="missing_data_labels", value=", ".join(missing_labels) if missing_labels else "none"),
            KeyValue(key="active_mask", value=f"0x{frame.active_mask:02X}"),
            KeyValue(key="valid_mask", value=f"0x{frame.valid_mask:02X}"),
            KeyValue(key="fault_mask", value=f"0x{frame.fault_mask:02X}"),
            KeyValue(key="trip_mask", value=f"0x{frame.trip_mask:02X}"),
            KeyValue(key="fault_labels", value=", ".join(fault_labels) if fault_labels else "none"),
            KeyValue(key="trip_labels", value=", ".join(trip_labels) if trip_labels else "none"),
            KeyValue(key="threshold_mm", value=str(frame.threshold_mm)),
            KeyValue(key="release_hold_ms", value=str(self.release_hold_ms)),
            KeyValue(key="release_remaining_ms", value=str(frame.release_remaining_ms)),
        ]

        for index, label in enumerate(TOF_CHANNEL_LABELS):
            bit = 1 << index
            if (frame.valid_mask & bit) == 0:
                channel_state = "missing_or_out_of_range"
            elif (frame.trip_mask & bit) != 0:
                channel_state = "threshold_exceeded"
            else:
                channel_state = "valid"
            diagnostic_status.values.append(KeyValue(key=f"tof{index + 1}_{label}", value=channel_state))

        diagnostic_array.status = [diagnostic_status]
        self.diagnostics_pub.publish(diagnostic_array)

    def shutdown(self) -> None:
        if not self.running:
            return
        self.running = False
        try:
            self.send_timer.shutdown()
        except Exception:
            pass
        try:
            if self.serial_port.is_open:
                self.serial_port.close()
        except Exception:
            pass


def main() -> None:
    rospy.init_node("h7_tof_bridge")
    H7TofBridgeNode()
    rospy.spin()


if __name__ == "__main__":
    main()

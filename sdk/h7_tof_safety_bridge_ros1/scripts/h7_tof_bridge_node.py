#!/usr/bin/env python3
import threading

import rospy
import serial
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse

from h7_tof_safety_bridge.msg import TofSafetyStatus
from h7_tof_safety_bridge.protocol import ControlCommand, build_control_frame, parse_status_line


class H7TofBridgeNode:
    def __init__(self) -> None:
        self.port_name = rospy.get_param("~port", "/dev/ttyACM0")
        self.baud_rate = int(rospy.get_param("~baud", 115200))
        self.send_rate_hz = float(rospy.get_param("~send_rate_hz", 20.0))
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.takeover_topic = rospy.get_param("~takeover_topic", "/h7_tof/takeover_enable")
        self.status_topic = rospy.get_param("~status_topic", "/h7_tof/status")
        self.estop_topic = rospy.get_param("~estop_topic", "/h7_tof/estop")
        self.raw_topic = rospy.get_param("~raw_topic", "/h7_tof/raw_line")

        self.seq = 0
        self.vx_mmps = 0
        self.vy_mmps = 0
        self.wz_mradps = 0
        self.takeover_enabled = False
        self.release_pending = False
        self.lock = threading.Lock()
        self.running = True

        self.status_pub = rospy.Publisher(self.status_topic, TofSafetyStatus, queue_size=20)
        self.estop_pub = rospy.Publisher(self.estop_topic, Bool, queue_size=20)
        self.raw_pub = rospy.Publisher(self.raw_topic, String, queue_size=50)

        rospy.Subscriber(self.cmd_vel_topic, Twist, self.on_cmd_vel, queue_size=10)
        rospy.Subscriber(self.takeover_topic, Bool, self.on_takeover, queue_size=10)
        rospy.Service("/h7_tof/release_estop", Trigger, self.handle_release_estop)

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

    def handle_release_estop(self, _req: Trigger) -> TriggerResponse:
        with self.lock:
            self.release_pending = True
        return TriggerResponse(success=True, message="release pulse queued for next H7CTL frame")

    def send_control_frame(self, _event) -> None:
        with self.lock:
            command = ControlCommand(
                seq=self.seq,
                vx_mmps=self.vx_mmps,
                vy_mmps=self.vy_mmps,
                wz_mradps=self.wz_mradps,
                release_req=self.release_pending,
                takeover_enable=self.takeover_enabled,
            )
            self.seq += 1
            self.release_pending = False

        frame = build_control_frame(command)
        try:
            self.serial_port.write(frame.encode("ascii"))
        except serial.SerialException as exc:
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

            msg = TofSafetyStatus()
            msg.header.stamp = rospy.Time.now()
            msg.seq = frame.seq
            msg.tof_mm = frame.tof_mm
            msg.estop = frame.estop
            msg.self_estop = frame.self_estop
            msg.external_estop = frame.external_estop
            msg.active_mask = frame.active_mask
            msg.trip_mask = frame.trip_mask
            msg.valid_mask = frame.valid_mask
            msg.fault_mask = frame.fault_mask
            msg.motion_mode = frame.motion_mode
            msg.takeover_enabled = frame.takeover_enabled

            self.status_pub.publish(msg)
            self.estop_pub.publish(Bool(data=frame.estop))

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

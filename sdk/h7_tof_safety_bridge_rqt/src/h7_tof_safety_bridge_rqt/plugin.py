import time
from collections import deque

import dynamic_reconfigure.client
import rospy
from h7_tof_safety_bridge.msg import TofSafetyStatus
from python_qt_binding.QtCore import QObject, Qt, QTimer, Signal
from python_qt_binding.QtWidgets import (
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPlainTextEdit,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)
from rqt_gui_py.plugin import Plugin
from std_msgs.msg import String
from std_srvs.srv import Trigger


CARD_OK = "#67f0a9"
CARD_WARN = "#f6c760"
CARD_ERROR = "#ff6b6b"
CARD_INFO = "#7ad7ff"
FG = "#ecf4ff"
MUTED = "#96acc4"
PANEL = "#141f2d"
CARD = "#0f1824"


class _BridgeSignals(QObject):
    status_received = Signal(object)
    raw_received = Signal(str)


class H7TofSafetyBridgePlugin(Plugin):
    def __init__(self, context):
        super(H7TofSafetyBridgePlugin, self).__init__(context)
        self.setObjectName("H7TofSafetyBridgePlugin")

        self.status_topic = rospy.get_param("~status_topic", "/h7_tof/status")
        self.raw_topic = rospy.get_param("~raw_topic", "/h7_tof/raw_line")
        self.release_service_name = rospy.get_param("~release_service", "/h7_tof/release_estop")
        self.reconfigure_target = rospy.get_param("~reconfigure_target", "/h7_tof_bridge")
        self.stale_timeout_ms = int(rospy.get_param("~stale_timeout_ms", 1500))
        self.apply_confirm_timeout_ms = int(rospy.get_param("~apply_confirm_timeout_ms", 3000))

        self._signals = _BridgeSignals()
        self._signals.status_received.connect(self._handle_status_update)
        self._signals.raw_received.connect(self._append_log_line)

        self._reconfigure_client = None
        self._release_service = None
        self._last_status_time = 0.0
        self._last_board_baseline = 391
        self._last_board_tolerance = 28
        self._has_board_params = False
        self._input_sync_guard = False
        self._inputs_dirty = False
        self._pending_apply = None
        self._warned_missing_board_params = False
        self._log_lines = deque(maxlen=500)

        self._widget = QWidget()
        self._widget.setObjectName("H7TofSafetyBridgePanel")
        self._widget.setWindowTitle("H7 TOF Safety")
        self._widget.setStyleSheet(
            """
            QWidget#H7TofSafetyBridgePanel { background: #0f1824; color: #ecf4ff; }
            QGroupBox { border: 1px solid #304156; border-radius: 10px; margin-top: 12px; background: #141f2d; }
            QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 4px; color: #51e5ff; font-weight: bold; }
            QLabel[role="muted"] { color: #96acc4; }
            QLabel[role="value"] { color: #ecf4ff; font-family: Consolas; font-weight: bold; }
            QLabel[role="tof"] { color: #ecf4ff; font-family: Consolas; font-size: 28px; font-weight: bold; }
            QFrame[role="card"] { background: #0f1824; border: 1px solid #304156; border-radius: 10px; }
            QSpinBox, QPlainTextEdit { background: #0a121c; color: #ecf4ff; border: 1px solid #304156; border-radius: 6px; }
            QPushButton { background: #123449; color: #ecf4ff; border: 1px solid #466584; border-radius: 6px; padding: 6px 10px; }
            QPushButton:disabled { color: #70859b; background: #132230; border-color: #24394d; }
            """
        )

        self._build_ui()
        context.add_widget(self._widget)

        self._status_sub = rospy.Subscriber(self.status_topic, TofSafetyStatus, self._on_status_msg, queue_size=20)
        self._raw_sub = rospy.Subscriber(self.raw_topic, String, self._on_raw_msg, queue_size=50)

        self._ui_timer = QTimer(self._widget)
        self._ui_timer.timeout.connect(self._refresh_online_state)
        self._ui_timer.start(250)
        self._refresh_online_state()

    def _build_ui(self):
        root = QVBoxLayout(self._widget)
        root.setContentsMargins(14, 14, 14, 14)
        root.setSpacing(12)

        top_bar = QHBoxLayout()
        self._sdk_state_label = QLabel("SDK 状态：离线")
        self._sdk_state_label.setStyleSheet("font-weight: bold; color: %s;" % CARD_ERROR)
        self._apply_state_label = QLabel("参数状态：等待状态帧")
        self._apply_state_label.setProperty("role", "muted")
        top_bar.addWidget(self._sdk_state_label)
        top_bar.addStretch(1)
        top_bar.addWidget(self._apply_state_label)
        root.addLayout(top_bar)

        content = QHBoxLayout()
        content.setSpacing(12)
        root.addLayout(content, 1)

        left = QVBoxLayout()
        left.setSpacing(12)
        content.addLayout(left, 3)

        left.addWidget(self._build_sensor_group())
        left.addWidget(self._build_control_group())

        right = QVBoxLayout()
        right.setSpacing(12)
        content.addLayout(right, 2)

        right.addWidget(self._build_status_group())
        right.addWidget(self._build_log_group(), 1)

    def _build_sensor_group(self):
        group = QGroupBox("TOF 距离")
        layout = QGridLayout(group)
        layout.setSpacing(10)

        self._tof_value_labels = []
        self._tof_cards = []
        names = ["TOF1 右前", "TOF2 右后", "TOF3 左后", "TOF4 左前"]
        for index, name in enumerate(names):
            card = QFrame()
            card.setProperty("role", "card")
            card_layout = QVBoxLayout(card)
            card_layout.setContentsMargins(12, 12, 12, 12)
            card_layout.setSpacing(4)

            title = QLabel(name)
            title.setProperty("role", "muted")
            value = QLabel("--")
            value.setProperty("role", "tof")
            unit = QLabel("mm")
            unit.setProperty("role", "muted")

            card_layout.addWidget(title)
            card_layout.addWidget(value)
            card_layout.addWidget(unit)
            layout.addWidget(card, index // 2, index % 2)
            self._tof_cards.append(card)
            self._tof_value_labels.append(value)

        return group

    def _build_control_group(self):
        group = QGroupBox("参数与解除")
        layout = QGridLayout(group)
        layout.setHorizontalSpacing(10)
        layout.setVerticalSpacing(8)

        layout.addWidget(self._caption("Baseline (mm)"), 0, 0)
        self._baseline_input = self._make_spinbox(0, 65534, 391)
        self._baseline_input.valueChanged.connect(self._on_param_input_changed)
        layout.addWidget(self._baseline_input, 0, 1)

        layout.addWidget(self._caption("Tolerance (mm)"), 0, 2)
        self._tolerance_input = self._make_spinbox(0, 65534, 28)
        self._tolerance_input.valueChanged.connect(self._on_param_input_changed)
        layout.addWidget(self._tolerance_input, 0, 3)

        self._apply_button = QPushButton("下发参数")
        self._apply_button.clicked.connect(self._on_apply_clicked)
        layout.addWidget(self._apply_button, 0, 4)

        layout.addWidget(self._caption("人工解除(ms)"), 1, 0)
        self._release_hold_input = self._make_spinbox(1, 600000, 3000)
        layout.addWidget(self._release_hold_input, 1, 1)

        self._release_button = QPushButton("恢复急停")
        self._release_button.clicked.connect(self._on_release_clicked)
        layout.addWidget(self._release_button, 1, 4)

        note = QLabel("允许直接下发参数；只有板端回读 B/T/TH 与目标值匹配后，界面才会标成“板端确认”。")
        note.setWordWrap(True)
        note.setProperty("role", "muted")
        layout.addWidget(note, 2, 0, 1, 5)
        return group

    def _build_status_group(self):
        group = QGroupBox("状态与参数")
        layout = QVBoxLayout(group)
        layout.setSpacing(10)

        self._estop_banner = QLabel("当前状态：等待状态帧")
        self._estop_banner.setStyleSheet("font-size: 18px; font-weight: bold; color: %s;" % CARD_WARN)
        self._estop_detail = QLabel("暂无板端状态。")
        self._estop_detail.setWordWrap(True)
        self._estop_detail.setProperty("role", "muted")
        layout.addWidget(self._estop_banner)
        layout.addWidget(self._estop_detail)

        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignLeft)
        form.setFormAlignment(Qt.AlignTop | Qt.AlignLeft)
        form.setHorizontalSpacing(18)
        form.setVerticalSpacing(8)
        self._status_values = {}
        for key, label in (
            ("seq", "帧序号"),
            ("estop", "总急停"),
            ("self_estop", "板端急停"),
            ("external_estop", "外部急停"),
            ("valid_mask", "有效掩码"),
            ("fault_mask", "故障掩码"),
            ("trip_mask", "越界掩码"),
            ("active_mask", "生效掩码"),
            ("params", "Baseline / Tol"),
            ("threshold", "Threshold"),
            ("release_remaining", "解除剩余"),
        ):
            left = self._caption(label)
            right = QLabel("--")
            right.setProperty("role", "value")
            form.addRow(left, right)
            self._status_values[key] = right

        layout.addLayout(form)
        return group

    def _build_log_group(self):
        group = QGroupBox("日志")
        layout = QVBoxLayout(group)
        self._log_box = QPlainTextEdit()
        self._log_box.setReadOnly(True)
        layout.addWidget(self._log_box, 1)
        return group

    def _caption(self, text):
        label = QLabel(text)
        label.setProperty("role", "muted")
        return label

    def _make_spinbox(self, minimum, maximum, value):
        spin = QSpinBox()
        spin.setRange(minimum, maximum)
        spin.setValue(value)
        return spin

    def _on_status_msg(self, msg):
        self._signals.status_received.emit(msg)

    def _on_raw_msg(self, msg):
        self._signals.raw_received.emit(msg.data)

    def _handle_status_update(self, msg):
        self._last_status_time = time.time()
        self._has_board_params = bool(msg.has_board_params)

        if msg.has_board_params:
            self._last_board_baseline = int(msg.baseline_mm)
            self._last_board_tolerance = int(msg.tolerance_mm)
            self._warned_missing_board_params = False
            if self._pending_apply is None and not self._inputs_dirty:
                self._set_inputs(msg.baseline_mm, msg.tolerance_mm)
                self._set_apply_state("参数状态：已同步板端参数", CARD_OK)
        elif not self._warned_missing_board_params:
            self._warned_missing_board_params = True
            self._append_log_line("板端状态帧未携带 B/T/TH，参数下发仍可执行，但界面无法确认板端是否已采纳。")
            if self._pending_apply is None and not self._inputs_dirty:
                self._set_apply_state("参数状态：板端参数未确认", CARD_WARN)

        self._set_status_value("seq", str(msg.seq))
        self._set_status_value("estop", "1" if msg.estop else "0", CARD_ERROR if msg.estop else CARD_OK)
        self._set_status_value("self_estop", "1" if msg.self_estop else "0", CARD_ERROR if msg.self_estop else CARD_OK)
        self._set_status_value("external_estop", "1" if msg.external_estop else "0", CARD_ERROR if msg.external_estop else CARD_OK)
        self._set_status_value("valid_mask", self._format_mask(msg.valid_mask))
        self._set_status_value("fault_mask", self._format_mask(msg.fault_mask))
        self._set_status_value("trip_mask", self._format_mask(msg.trip_mask))
        self._set_status_value("active_mask", self._format_mask(msg.active_mask))
        self._set_status_value("params", ("%d / %d" % (msg.baseline_mm, msg.tolerance_mm)) if msg.has_board_params else "-- / --")
        self._set_status_value("threshold", str(msg.threshold_mm) if (msg.has_board_params or msg.threshold_mm > 0) else "--")
        self._set_status_value("release_remaining", "%d ms" % max(0, int(msg.release_remaining_ms)))
        self._update_estop_banner(msg)
        self._update_tof_cards(msg)
        self._update_apply_confirmation(msg)
        self._refresh_online_state()

    def _update_tof_cards(self, msg):
        for index in range(4):
            value = int(msg.tof_mm[index])
            if value >= 65535:
                self._tof_value_labels[index].setText("--")
                self._tof_value_labels[index].setStyleSheet("color: %s;" % CARD_WARN)
            else:
                tripped = ((int(msg.trip_mask) >> index) & 0x01) == 1
                self._tof_value_labels[index].setText(str(value))
                self._tof_value_labels[index].setStyleSheet("color: %s;" % (CARD_ERROR if tripped else FG))

    def _update_apply_confirmation(self, msg):
        if self._pending_apply is None:
            return

        if not msg.has_board_params:
            self._set_apply_state("参数状态：已下发，未获板端确认", CARD_WARN)
            return

        target_baseline, target_tolerance, _requested_at = self._pending_apply
        if int(msg.baseline_mm) == target_baseline and int(msg.tolerance_mm) == target_tolerance:
            self._pending_apply = None
            self._inputs_dirty = False
            self._set_inputs(target_baseline, target_tolerance)
            self._set_apply_state("参数状态：板端已确认", CARD_OK)
        else:
            self._set_apply_state("参数状态：等待板端回读确认", "#f1cf54")

    def _update_estop_banner(self, msg):
        if msg.estop:
            self._estop_banner.setText("当前状态：急停中")
            self._estop_banner.setStyleSheet("font-size: 18px; font-weight: bold; color: %s;" % CARD_ERROR)
        elif int(msg.release_remaining_ms) > 0:
            self._estop_banner.setText("当前状态：人工解除生效")
            self._estop_banner.setStyleSheet("font-size: 18px; font-weight: bold; color: %s;" % CARD_INFO)
        else:
            self._estop_banner.setText("当前状态：运行允许")
            self._estop_banner.setStyleSheet("font-size: 18px; font-weight: bold; color: %s;" % CARD_OK)

        if int(msg.release_remaining_ms) > 0 and int(msg.fault_mask) != 0:
            detail = "人工解除剩余 %d ms；时间到且故障未恢复会再次急停。" % int(msg.release_remaining_ms)
        elif msg.self_estop and msg.external_estop:
            detail = "板端安全逻辑和外部急停同时触发。"
        elif msg.self_estop:
            detail = "板端安全逻辑触发急停。"
        elif msg.external_estop:
            detail = "外部急停输入触发。"
        else:
            detail = "未检测到急停触发条件。"
        self._estop_detail.setText(
            "总急停=%d  板端急停=%d  外部急停=%d  人工解除剩余=%dms  |  %s"
            % (int(msg.estop), int(msg.self_estop), int(msg.external_estop), int(msg.release_remaining_ms), detail)
        )

    def _on_param_input_changed(self, _value):
        if self._input_sync_guard or self._pending_apply is not None:
            return

        baseline = int(self._baseline_input.value())
        tolerance = int(self._tolerance_input.value())
        self._inputs_dirty = (baseline != self._last_board_baseline) or (tolerance != self._last_board_tolerance)
        if self._inputs_dirty:
            text = "参数状态：本地参数已修改，待下发"
            if not self._has_board_params:
                text += "（板端未确认）"
            self._set_apply_state(text, "#f1cf54")
        else:
            self._set_apply_state("参数状态：已同步板端参数" if self._has_board_params else "参数状态：板端参数未确认", CARD_OK if self._has_board_params else CARD_WARN)

    def _on_apply_clicked(self):
        if not self._is_online():
            self._append_log_line("SDK 离线或状态超时，无法下发参数。")
            return
        if not self._ensure_reconfigure_client():
            self._append_log_line("dynamic_reconfigure 不可用，无法下发参数。")
            return

        baseline = int(self._baseline_input.value())
        tolerance = int(self._tolerance_input.value())
        try:
            self._reconfigure_client.update_configuration(
                {
                    "baseline_mm": baseline,
                    "tolerance_mm": tolerance,
                }
            )
        except Exception as exc:
            self._append_log_line("参数下发失败: %s" % exc)
            return

        self._pending_apply = (baseline, tolerance, time.time())
        self._set_apply_state("参数状态：已发送，等待板端确认", "#f1cf54")
        self._append_log_line("已请求下发参数: baseline_mm=%d tolerance_mm=%d" % (baseline, tolerance))
        if not self._has_board_params:
            self._append_log_line("板端尚未回读 B/T/TH；本次参数会继续下发，但界面可能无法确认成功。")

    def _on_release_clicked(self):
        if not self._is_online():
            self._append_log_line("SDK 离线或状态超时，无法发送人工解除。")
            return
        if not self._ensure_reconfigure_client():
            self._append_log_line("dynamic_reconfigure 不可用，无法配置人工解除时间。")
            return
        if not self._ensure_release_service():
            self._append_log_line("恢复急停服务不可用。")
            return

        release_hold_ms = int(self._release_hold_input.value())
        try:
            self._reconfigure_client.update_configuration({"release_hold_ms": release_hold_ms})
            response = self._release_service()
        except Exception as exc:
            self._append_log_line("人工解除失败: %s" % exc)
            return

        if response.success:
            self._append_log_line("已发送人工解除命令，持续 %d ms。" % release_hold_ms)
        else:
            self._append_log_line("人工解除被拒绝: %s" % response.message)

    def _ensure_reconfigure_client(self):
        if self._reconfigure_client is not None:
            return True
        try:
            self._reconfigure_client = dynamic_reconfigure.client.Client(self.reconfigure_target, timeout=1.0)
            return True
        except Exception:
            self._reconfigure_client = None
            return False

    def _ensure_release_service(self):
        if self._release_service is not None:
            return True
        try:
            self._release_service = rospy.ServiceProxy(self.release_service_name, Trigger)
            return True
        except Exception:
            self._release_service = None
            return False

    def _refresh_online_state(self):
        online = self._is_online()
        if online:
            age_ms = int((time.time() - self._last_status_time) * 1000.0)
            self._sdk_state_label.setText("SDK 状态：在线（最近状态 %d ms 前）" % max(0, age_ms))
            self._sdk_state_label.setStyleSheet("font-weight: bold; color: %s;" % CARD_OK)
        else:
            self._sdk_state_label.setText("SDK 状态：离线 / 状态超时")
            self._sdk_state_label.setStyleSheet("font-weight: bold; color: %s;" % CARD_ERROR)

        self._apply_button.setEnabled(online)
        self._release_button.setEnabled(online)

        if self._pending_apply is not None and ((time.time() - self._pending_apply[2]) * 1000.0) > self.apply_confirm_timeout_ms:
            self._set_apply_state(
                "参数状态：等待超时，请看板端回读" if self._has_board_params else "参数状态：已下发，未获板端确认",
                CARD_WARN,
            )

    def _is_online(self):
        if self._last_status_time <= 0.0:
            return False
        return ((time.time() - self._last_status_time) * 1000.0) <= self.stale_timeout_ms

    def _set_status_value(self, key, value, color=FG):
        label = self._status_values[key]
        label.setText(value)
        label.setStyleSheet("color: %s;" % color)

    def _set_apply_state(self, text, color):
        self._apply_state_label.setText(text)
        self._apply_state_label.setStyleSheet("font-weight: bold; color: %s;" % color)

    def _set_inputs(self, baseline, tolerance):
        self._input_sync_guard = True
        try:
            self._baseline_input.setValue(int(baseline))
            self._tolerance_input.setValue(int(tolerance))
        finally:
            self._input_sync_guard = False

    def _append_log_line(self, line):
        timestamp = time.strftime("%H:%M:%S")
        self._log_lines.append("[%s] %s" % (timestamp, line))
        self._log_box.setPlainText("\n".join(self._log_lines))
        self._log_box.verticalScrollBar().setValue(self._log_box.verticalScrollBar().maximum())

    @staticmethod
    def _format_mask(value):
        return "0x%02X" % (int(value) & 0xFF)

    def shutdown_plugin(self):
        try:
            self._ui_timer.stop()
        except Exception:
            pass
        try:
            self._status_sub.unregister()
        except Exception:
            pass
        try:
            self._raw_sub.unregister()
        except Exception:
            pass

    def save_settings(self, plugin_settings, instance_settings):
        del plugin_settings
        del instance_settings

    def restore_settings(self, plugin_settings, instance_settings):
        del plugin_settings
        del instance_settings

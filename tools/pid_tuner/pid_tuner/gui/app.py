from __future__ import annotations

import math
from datetime import datetime
from pathlib import Path
import sys

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QFileDialog,
    QFormLayout,
    QInputDialog,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from ..models import MotionGoal, PidConfig, Telemetry
from ..storage import (DEFAULT_LOGS_DIR, export_c_defaults, list_profiles,
                       load_profile, save_profile, write_telemetry_csv)
from .buffer import TelemetryBuffer
from .plots import TelemetryPlots
from .session import SessionController
from .smooth_scroll import SmoothScrollArea
from .widgets import (GOTO_TIMEOUT_MS, GOTO_VMAX_MM_S, GOTO_WMAX_DEG_S,
                      GOTO_YAW_LABEL, HEADING_MODE_NONE,
                      ConnectionMotionPanel, PidControlPanel)


HEARTBEAT_INTERVAL_MS = 250

MOTION_STATE_TEXT = {
    0: "空闲",
    1: "运行中",
    2: "已到达，无需运动",
    3: "运动超时",
    4: "位姿不可用",
    5: "世界原点不可用",
    6: "已取消",
}


def validate_motion_goal(goal: MotionGoal) -> str | None:
    if not all(math.isfinite(value) for value in (
            goal.x_mm, goal.y_mm, goal.yaw_deg, goal.vmax_mm_s, goal.wmax_deg_s)):
        return "GOTO 参数必须是有限数值"
    if not goal.use_position and not goal.use_yaw:
        return "GOTO 至少需要启用位置或航向"
    if goal.use_position and not 0.0 < goal.vmax_mm_s <= GOTO_VMAX_MM_S:
        return "vmax 必须在 0-1200 mm/s 之间"
    if goal.use_yaw and not 0.0 < goal.wmax_deg_s <= GOTO_WMAX_DEG_S:
        return "wmax 必须在 0-120 deg/s 之间"
    if not 0 < goal.timeout_ms <= GOTO_TIMEOUT_MS:
        return "超时必须在 1-15000 ms 之间"
    return None


def format_pid_apply_log(revision: int, pid: PidConfig) -> str:
    values = ", ".join(f"{name}={value:.4f}" for name, value in pid.to_dict().items())
    return f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] PID 已上传 r{revision}: {values}"


def format_telemetry_status(item: Telemetry) -> str:
    pose_state = "位姿有效" if (item.flags & 0x01) else "位姿无效"
    motion_state = MOTION_STATE_TEXT.get(item.state, f"未知状态 {item.state}")
    target = ", ".join(f"{value:.1f}" for value in item.target)
    actual = ", ".join(f"{value:.1f}" for value in item.actual)
    err_val = ", ".join(f"{value:.1f}" for value in item.error)
    phase = " 航向对准中" if item.yaw_aligning else ""
    return (f"{motion_state} {pose_state}{phase} 目标=({target}) 实际=({actual}) 误差=({err_val}) "
            f"PID r{item.pid_revision} 标志=0x{item.flags:02X} 覆盖={item.overwritten_count}")


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("PID 在线调参")
        self.resize(1440, 900)
        self.buffer = TelemetryBuffer()
        self.session = SessionController()
        self.recording = False
        self.recorded: list[Telemetry] = []
        self._active_heading_mode = "WIT"
        self._build()
        self._wire()
        self.timer = QTimer(self)
        self.timer.setInterval(40)
        self.timer.timeout.connect(self.refresh)
        self.timer.start()
        self.heartbeat_timer = QTimer(self)
        self.heartbeat_timer.setInterval(HEARTBEAT_INTERVAL_MS)
        self.heartbeat_timer.timeout.connect(self.session.heartbeat)
        self.heartbeat_timer.start()

    def _build(self) -> None:
        root = QSplitter(Qt.Orientation.Horizontal)
        root.setChildrenCollapsible(False)
        controls = QWidget()
        controls.setMinimumWidth(300)
        left = QVBoxLayout(controls)
        left.setContentsMargins(12, 12, 12, 12)

        self.connection_motion = ConnectionMotionPanel()
        self.pid_control = PidControlPanel()
        # 保留窗口属性，避免现有调用方依赖具体控件位置。
        self.port = self.connection_motion.port
        self.refresh_ports_button = self.connection_motion.refresh_ports_button
        self.baud = self.connection_motion.baud
        self.connect_button = self.connection_motion.connect_button
        self.status = self.connection_motion.status
        self.goal = self.connection_motion.goal
        self.timeout = self.connection_motion.timeout
        self.large_yaw_align = self.connection_motion.large_yaw_align
        self.yaw_source = self.connection_motion.yaw_source
        self.reset_origin = self.connection_motion.reset_origin
        self.goto = self.connection_motion.goto
        self.goto_position = self.connection_motion.goto_position
        self.goto_yaw = self.connection_motion.goto_yaw
        self.stop = self.connection_motion.stop
        self.pid = self.pid_control.pid
        self.read_pid = self.pid_control.read_pid
        self.apply_pid = self.pid_control.apply_pid
        self.restore_pid = self.pid_control.restore_pid
        left.addWidget(self.connection_motion)
        left.addWidget(self.pid_control)

        self.profile = QComboBox()
        self.load_profile = QPushButton("加载方案")
        self.save_profile = QPushButton("另存方案")
        self.export_c = QPushButton("导出 C")
        profiles = QFormLayout()
        profiles.addRow(self.profile)
        profiles.addRow(self.load_profile)
        profiles.addRow(self.save_profile)
        profiles.addRow(self.export_c)
        left.addLayout(profiles)

        self.new_experiment = QPushButton("新实验")
        self.record = QPushButton("开始记录 CSV")
        self.window = QSpinBox()
        self.window.setRange(5, 120)
        self.window.setValue(30)
        left.addWidget(self.new_experiment)
        left.addWidget(self.record)
        left.addWidget(QLabel("时间窗口 (秒)"))
        left.addWidget(self.window)
        left.addStretch()

        self.controls_scroll = SmoothScrollArea()
        self.controls_scroll.setWidgetResizable(True)
        self.controls_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.controls_scroll.setWidget(controls)

        self.plots = TelemetryPlots()
        self.plots_scroll = QScrollArea()
        self.plots_scroll.setWidgetResizable(False)
        self.plots_scroll.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop)
        self.plots_scroll.setWidget(self.plots)
        root.addWidget(self.controls_scroll)
        root.addWidget(self.plots_scroll)
        root.setStretchFactor(0, 0)
        root.setStretchFactor(1, 1)
        root.setSizes([330, 1110])
        self.setCentralWidget(root)
        self.refresh_ports()
        self.reload_profiles()

    def _wire(self) -> None:
        self.connection_motion.refresh_ports_requested.connect(self.refresh_ports)
        self.connection_motion.connect_requested.connect(self.connect_port)
        self.connection_motion.disconnect_requested.connect(self.disconnect_port)
        self.connection_motion.motion_requested.connect(self.request_motion)
        self.connection_motion.stop_requested.connect(self.session.stop)
        self.connection_motion.yaw_source_requested.connect(self.request_heading_mode)
        self.connection_motion.origin_reset_requested.connect(self.session.reset_origin)
        self.connection_motion.goto_strategy_requested.connect(self.session.set_goto_strategy)
        self.pid_control.read_requested.connect(self.session.read_pid)
        self.pid_control.apply_requested.connect(self.session.apply_pid)
        self.pid_control.restore_requested.connect(self.session.restore_pid)
        self.new_experiment.clicked.connect(self.new_experiment_clicked)
        self.record.clicked.connect(self.toggle_record)
        self.load_profile.clicked.connect(self.load_selected_profile)
        self.save_profile.clicked.connect(self.save_current_profile)
        self.export_c.clicked.connect(self.export_current_c)
        self.window.valueChanged.connect(lambda value: self.plots.set_window(float(value)))
        self.session.telemetry.connect(self.on_telemetry)
        self.session.status.connect(self.status.setText)
        self.session.failure.connect(self.on_failure)
        self.session.pid_read.connect(self.on_pid)
        self.session.pid_applied.connect(self.on_pid_applied)
        self.session.yaw_source_changed.connect(self.on_yaw_source_changed)
        self.session.goto_strategy_read.connect(self.on_goto_strategy_changed)
        self.session.goto_strategy_changed.connect(self.on_goto_strategy_changed)
        self.session.origin_reset.connect(self.on_origin_reset)
        self.session.motion_changed.connect(self.on_motion_changed)

    def refresh_ports(self) -> None:
        from serial.tools import list_ports
        self.connection_motion.set_available_ports([item.device for item in list_ports.comports()])

    def connect_port(self, port: str, baud: int) -> None:
        self.session.connect_port(port, baud)
        self.connection_motion.set_connected(True)

    def disconnect_port(self) -> None:
        self.session.disconnect()
        self.large_yaw_align.setEnabled(False)
        self.connection_motion.set_connected(False)

    def toggle_connection(self) -> None:
        if self.session.connected:
            self.disconnect_port()
        elif self.port.currentText():
            self.connect_port(self.port.currentText(), int(self.baud.currentText()))
        else:
            self.on_failure("未发现可用 COM 口")

    def current_pid(self) -> PidConfig:
        return self.pid_control.current_pid()

    def apply_current_pid(self) -> None:
        self.session.apply_pid(self.current_pid())

    def on_pid(self, revision: int, pid: PidConfig) -> None:
        self.pid_control.set_pid(pid)
        self.status.setText(f"PID 修订号 {revision}")

    def start_motion(self) -> None:
        self.request_motion(self.connection_motion.current_motion_goal(
            True, self.connection_motion.uses_yaw()))

    def start_position_motion(self) -> None:
        self.request_motion(self.connection_motion.current_motion_goal(True, False))

    def start_yaw_motion(self) -> None:
        self.request_motion(self.connection_motion.current_motion_goal(False, True))

    def request_motion(self, goal: MotionGoal) -> None:
        error = validate_motion_goal(goal)
        if error is not None:
            self.status.setText(f"错误: {error}")
            return
        self.session.start_motion(goal)
        self._active_heading_mode = (
            self.connection_motion.heading_mode() if goal.use_yaw else HEADING_MODE_NONE
        )
        self.plots.set_heading_mode(self._active_heading_mode)
        kind = "组合" if goal.use_position and goal.use_yaw else ("位置" if goal.use_position else "角度")
        self.status.setText(f"已请求 {kind} GOTO")
        self.buffer.add_event(f"{kind} GOTO")

    def request_heading_mode(self, mode: str) -> None:
        """将三态 UI 映射为板端航向源和 GOTO 航向约束。"""
        self._active_heading_mode = mode
        self.plots.set_heading_mode(mode)
        if mode == HEADING_MODE_NONE:
            self.status.setText("已关闭航向控制；后续组合 GOTO 仅控制 X/Y")
            self.buffer.add_event("航向控制关闭")
            return
        self.session.set_yaw_source(mode)

    def on_telemetry(self, item: Telemetry) -> None:
        self.buffer.append(item)
        if self.recording:
            self.recorded.append(item)
        status = format_telemetry_status(item)
        status += (" 航向控制=关闭" if self._active_heading_mode == HEADING_MODE_NONE
                   else f" 航向控制={self._active_heading_mode}")
        if item.heartbeat_timed_out:
            status += " 心跳超时停车"
        elif item.remote_goal_active:
            status += f" 心跳正常/{item.heartbeat_age_ms}ms"
        self.status.setText(status)
        if (self.connection_motion.heading_mode() != HEADING_MODE_NONE and
                item.yaw_source != self.connection_motion.heading_mode()):
            self.connection_motion.set_heading_mode(item.yaw_source)
            if self._active_heading_mode != HEADING_MODE_NONE:
                self._active_heading_mode = item.yaw_source
                self.plots.set_heading_mode(item.yaw_source)

    def refresh(self) -> None:
        self.plots.refresh(self.buffer)

    def on_pid_applied(self, revision: int, pid: PidConfig) -> None:
        print(format_pid_apply_log(revision, pid), flush=True)
        self.buffer.add_event(f"PID r{revision}")
        self.status.setText(f"PID 已应用，修订号 {revision}")

    def on_yaw_source_changed(self, source: str) -> None:
        if self.connection_motion.heading_mode() != HEADING_MODE_NONE:
            self.connection_motion.set_heading_mode(source)
        if self._active_heading_mode != HEADING_MODE_NONE:
            self._active_heading_mode = source
            self.plots.set_heading_mode(source)
        self.status.setText(f"航向 PID 数据源已切换为 {source}")
        self.buffer.add_event(f"航向源 {source}")

    def on_goto_strategy_changed(self, enabled: bool) -> None:
        self.large_yaw_align.blockSignals(True)
        self.large_yaw_align.setChecked(enabled)
        self.large_yaw_align.blockSignals(False)
        self.connection_motion.set_connected(self.session.connected)
        self.connection_motion.set_motion_active(self.session.motion_active)
        self.status.setText("大航向误差先对准已启用" if enabled else "大航向误差先对准已关闭")
        self.buffer.add_event("GOTO 策略: 先对准航向" if enabled else "GOTO 策略: 并行控制")

    def on_motion_changed(self, active: bool) -> None:
        self.connection_motion.set_motion_active(active)
        self.buffer.add_event("运动状态改变")

    def on_origin_reset(self) -> None:
        for widget in self.goal[:3]:
            widget.setValue(0.0)
        self.buffer.add_event("零点重置")
        self.status.setText("零点已重置")

    def new_experiment_clicked(self) -> None:
        self.buffer.clear()
        self.recorded.clear()
        self.buffer.add_event("新实验")

    def toggle_record(self) -> None:
        self.recording = not self.recording
        self.record.setText("停止记录并保存" if self.recording else "开始记录 CSV")
        if self.recording:
            self.recorded = []
            self.buffer.add_event("开始记录")
        else:
            path = DEFAULT_LOGS_DIR / "gui_telemetry.csv"
            write_telemetry_csv(path, self.recorded)
            self.status.setText(f"已保存 {path}")

    def reload_profiles(self) -> None:
        self.profile.clear()
        self.profile.addItems(list_profiles())

    def load_selected_profile(self) -> None:
        if not self.profile.currentText():
            return
        pid, _ = load_profile(self.profile.currentText())
        self.on_pid(0, pid)
        self.status.setText("已加载本地方案")

    def save_current_profile(self) -> None:
        name, accepted = QInputDialog.getText(self, "保存 PID 方案", "名称")
        if accepted and name:
            save_profile(name, self.current_pid())
            self.reload_profiles()
            self.profile.setCurrentText(name)

    def export_current_c(self) -> None:
        path, _ = QFileDialog.getSaveFileName(
            self, "导出 C 宏", "pid_defaults.h", "Header (*.h);;Text (*.txt)")
        if path:
            Path(path).write_text(export_c_defaults(self.current_pid()), encoding="utf-8")

    def on_failure(self, message: str) -> None:
        self.status.setText(f"错误: {message}")
        if self.session.connected and self.session.motion_active:
            self.session.stop()
        QMessageBox.warning(self, "通信错误", message)

    def closeEvent(self, event: object) -> None:
        self.timer.stop()
        self.heartbeat_timer.stop()
        self.session.shutdown()
        event.accept()  # type: ignore[attr-defined]


def main() -> int:
    app = QApplication(sys.argv)
    app.setStyleSheet(
        "QWidget{background:#161b22;color:#d8dee9;} "
        "QLineEdit,QComboBox,QSpinBox,QDoubleSpinBox{background:#222b36;padding:4px;} "
        "QPushButton{background:#2b3a4b;padding:6px;} "
        "QPushButton#stopButton{background:#d1495b;color:white;font-weight:bold;}"
    )
    window = MainWindow()
    window.show()
    return app.exec()

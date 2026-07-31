from __future__ import annotations

import math
from pathlib import Path
import sys

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (QApplication, QComboBox, QDoubleSpinBox, QFileDialog, QFormLayout,
                               QGridLayout, QHBoxLayout, QLabel, QLineEdit, QMainWindow, QMessageBox,
                               QPushButton, QSpinBox, QSplitter, QVBoxLayout, QWidget, QInputDialog)

from ..models import MotionGoal, PidConfig
from ..storage import DEFAULT_PROFILES_DIR, export_c_defaults, list_profiles, load_profile, save_profile, write_telemetry_csv
from .buffer import TelemetryBuffer
from .plots import TelemetryPlots
from .session import SessionController


GOTO_VMAX_MM_S = 1500.0
GOTO_WMAX_DEG_S = 90.0
GOTO_TIMEOUT_MS = 15000
GOTO_YAW_LABEL = "yaw 相对初始化零点 deg"

MOTION_STATE_TEXT = {
    0: "空闲",
    1: "运行中",
    2: "已到达，无需运动",
    3: "运动超时",
    4: "位姿不可用",
    5: "世界原点不可用",
    6: "已取消",
}


def number(value: float = 0.0, minimum: float = -100000.0, maximum: float = 100000.0) -> QDoubleSpinBox:
    box = QDoubleSpinBox(); box.setRange(minimum, maximum); box.setDecimals(4); box.setValue(value); box.setSingleStep(0.1)
    return box


def validate_motion_goal(goal: MotionGoal) -> str | None:
    if not all(math.isfinite(value) for value in (goal.x_mm, goal.y_mm, goal.yaw_deg, goal.vmax_mm_s, goal.wmax_deg_s)):
        return "GOTO 参数必须是有限数值"
    if not 0.0 < goal.vmax_mm_s <= GOTO_VMAX_MM_S:
        return "vmax 必须在 0-1500 mm/s 之间"
    if not 0.0 < goal.wmax_deg_s <= GOTO_WMAX_DEG_S:
        return "wmax 必须在 0-90 deg/s 之间"
    if not 0 < goal.timeout_ms <= GOTO_TIMEOUT_MS:
        return "超时必须在 1-15000 ms 之间"
    return None


def format_telemetry_status(item: Telemetry) -> str:
    pose_state = "位姿有效" if (item.flags & 0x01) else "位姿无效"
    motion_state = MOTION_STATE_TEXT.get(item.state, f"未知状态 {item.state}")
    target = ", ".join(f"{value:.1f}" for value in item.target)
    actual = ", ".join(f"{value:.1f}" for value in item.actual)
    return (f"{motion_state} {pose_state} 目标=({target}) 实际=({actual}) "
            f"PID r{item.pid_revision} 标志=0x{item.flags:02X} 覆盖={item.overwritten_count}")


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__(); self.setWindowTitle("PID 在线调参"); self.resize(1440, 900)
        self.buffer = TelemetryBuffer(); self.session = SessionController(); self.recording = False; self.recorded = []
        self._build(); self._wire()
        self.timer = QTimer(self); self.timer.setInterval(40); self.timer.timeout.connect(self.refresh); self.timer.start()

    def _build(self) -> None:
        root = QSplitter(); controls = QWidget(); left = QVBoxLayout(controls); left.setContentsMargins(12, 12, 12, 12)
        self.port = QComboBox(); self.refresh_ports_button = QPushButton("刷新 COM"); self.baud = QComboBox(); self.baud.addItems(["115200", "230400"]); self.connect_button = QPushButton("连接")
        connection = QFormLayout(); connection.addRow("串口", self.port); connection.addRow(self.refresh_ports_button); connection.addRow("波特率", self.baud); connection.addRow(self.connect_button); left.addLayout(connection)
        self.status = QLabel("未连接"); left.addWidget(self.status)
        self.pid = [number(value) for value in (1, .03, .1, 2, .05, .08)]
        pid_form = QFormLayout(); [pid_form.addRow(name, widget) for name, widget in zip(("Kp 位置", "Ki 位置", "Kd 位置", "Kp 航向", "Ki 航向", "Kd 航向"), self.pid)]
        self.read_pid = QPushButton("读取 PID"); self.apply_pid = QPushButton("应用 PID"); self.restore_pid = QPushButton("恢复默认")
        pid_form.addRow(self.read_pid); pid_form.addRow(self.apply_pid); pid_form.addRow(self.restore_pid); left.addLayout(pid_form)
        self.profile = QComboBox(); self.load_profile = QPushButton("加载方案"); self.save_profile = QPushButton("另存方案"); self.export_c = QPushButton("导出 C")
        left.addWidget(self.profile); left.addWidget(self.load_profile); left.addWidget(self.save_profile); left.addWidget(self.export_c)
        self.goal = [number(), number(), number(), number(50.0, 0.1, GOTO_VMAX_MM_S), number(30.0, 0.1, GOTO_WMAX_DEG_S)]; self.timeout = QSpinBox(); self.timeout.setRange(1, GOTO_TIMEOUT_MS); self.timeout.setValue(5000)
        goal_form = QFormLayout(); [goal_form.addRow(name, widget) for name, widget in zip(("X mm", "Y mm", GOTO_YAW_LABEL, "vmax mm/s", "wmax deg/s"), self.goal)]
        goal_form.addRow("超时 ms", self.timeout); self.goto = QPushButton("开始 GOTO"); self.stop = QPushButton("STOP"); self.stop.setObjectName("stopButton"); self.new_experiment = QPushButton("新实验")
        goal_form.addRow(self.goto); goal_form.addRow(self.stop); goal_form.addRow(self.new_experiment); left.addLayout(goal_form)
        self.record = QPushButton("开始记录 CSV"); self.window = QSpinBox(); self.window.setRange(5, 120); self.window.setValue(30); left.addWidget(self.record); left.addWidget(QLabel("时间窗口 (秒)")); left.addWidget(self.window); left.addStretch()
        self.plots = TelemetryPlots(); root.addWidget(controls); root.addWidget(self.plots); root.setSizes([330, 1110]); self.setCentralWidget(root); self.refresh_ports(); self.reload_profiles()

    def _wire(self) -> None:
        self.connect_button.clicked.connect(self.toggle_connection); self.refresh_ports_button.clicked.connect(self.refresh_ports); self.read_pid.clicked.connect(self.session.read_pid); self.apply_pid.clicked.connect(self.apply_current_pid); self.restore_pid.clicked.connect(self.session.restore_pid)
        self.goto.clicked.connect(self.start_motion); self.stop.clicked.connect(self.session.stop); self.new_experiment.clicked.connect(self.new_experiment_clicked); self.record.clicked.connect(self.toggle_record)
        self.load_profile.clicked.connect(self.load_selected_profile); self.save_profile.clicked.connect(self.save_current_profile); self.export_c.clicked.connect(self.export_current_c)
        self.window.valueChanged.connect(lambda value: self.plots.set_window(float(value))); self.session.telemetry.connect(self.on_telemetry); self.session.status.connect(self.status.setText); self.session.failure.connect(self.on_failure); self.session.pid_read.connect(self.on_pid); self.session.pid_applied.connect(lambda revision: self.buffer.add_event(f"PID r{revision}")); self.session.motion_changed.connect(lambda _: self.buffer.add_event("运动状态改变"))

    def refresh_ports(self) -> None:
        from serial.tools import list_ports
        self.port.clear(); self.port.addItems([item.device for item in list_ports.comports()])

    def toggle_connection(self) -> None:
        if self.session.connected: self.session.disconnect(); self.connect_button.setText("连接"); return
        if not self.port.currentText(): self.on_failure("未发现可用 COM 口"); return
        self.session.connect_port(self.port.currentText(), int(self.baud.currentText())); self.connect_button.setText("断开")

    def current_pid(self) -> PidConfig: return PidConfig(*(widget.value() for widget in self.pid))
    def apply_current_pid(self) -> None: self.session.apply_pid(self.current_pid())
    def on_pid(self, revision: int, pid: PidConfig) -> None:
        [widget.setValue(value) for widget, value in zip(self.pid, pid.to_dict().values())]; self.status.setText(f"PID 修订号 {revision}")
    def start_motion(self) -> None:
        goal = MotionGoal(*(widget.value() for widget in self.goal), self.timeout.value())
        error = validate_motion_goal(goal)
        if error is not None:
            self.status.setText(f"错误: {error}")
            return
        self.session.start_motion(goal)
        self.status.setText(f"已请求 GOTO 世界目标=({goal.x_mm:.1f}, {goal.y_mm:.1f}, {goal.yaw_deg:.1f})")
        self.buffer.add_event("GOTO")
    def on_telemetry(self, item: object) -> None:
        self.buffer.append(item); self.recorded.append(item) if self.recording else None
        self.status.setText(format_telemetry_status(item))
    def refresh(self) -> None: self.plots.refresh(self.buffer); self.session.heartbeat()
    def new_experiment_clicked(self) -> None: self.buffer.clear(); self.recorded.clear(); self.buffer.add_event("新实验")
    def toggle_record(self) -> None:
        self.recording = not self.recording; self.record.setText("停止记录并保存" if self.recording else "开始记录 CSV")
        if self.recording: self.recorded = []; self.buffer.add_event("开始记录")
        if not self.recording:
            path = Path("logs") / "gui_telemetry.csv"; write_telemetry_csv(path, self.recorded); self.status.setText(f"已保存 {path}")
    def reload_profiles(self) -> None: self.profile.clear(); self.profile.addItems(list_profiles())
    def load_selected_profile(self) -> None:
        if not self.profile.currentText(): return
        pid, _ = load_profile(self.profile.currentText()); self.on_pid(0, pid); self.status.setText("已加载本地方案")
    def save_current_profile(self) -> None:
        name, accepted = QInputDialog.getText(self, "保存 PID 方案", "名称")
        if accepted and name:
            save_profile(name, self.current_pid()); self.reload_profiles(); self.profile.setCurrentText(name)
    def export_current_c(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "导出 C 宏", "pid_defaults.h", "Header (*.h);;Text (*.txt)")
        if path: Path(path).write_text(export_c_defaults(self.current_pid()), encoding="utf-8")
    def on_failure(self, message: str) -> None:
        self.status.setText(f"错误: {message}")
        if self.session.connected and self.session.motion_active: self.session.stop()
        QMessageBox.warning(self, "通信错误", message)
    def closeEvent(self, event: object) -> None: self.session.shutdown(); event.accept()  # type: ignore[attr-defined]


def main() -> int:
    app = QApplication(sys.argv); app.setStyleSheet("QWidget{background:#161b22;color:#d8dee9;} QLineEdit,QComboBox,QSpinBox,QDoubleSpinBox{background:#222b36;padding:4px;} QPushButton{background:#2b3a4b;padding:6px;} QPushButton#stopButton{background:#d1495b;color:white;font-weight:bold;}")
    window = MainWindow(); window.show(); return app.exec()

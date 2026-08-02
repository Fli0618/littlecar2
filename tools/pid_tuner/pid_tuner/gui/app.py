from __future__ import annotations

import math
from pathlib import Path
import sys
from datetime import datetime

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (QAbstractSpinBox, QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QFormLayout,
                               QGridLayout, QHBoxLayout, QLabel, QLineEdit, QMainWindow, QMessageBox,
                               QPushButton, QScrollArea, QSpinBox, QSplitter, QVBoxLayout, QWidget, QInputDialog)

from ..models import MotionGoal, PidConfig
from ..storage import DEFAULT_PROFILES_DIR, export_c_defaults, list_profiles, load_profile, save_profile, write_telemetry_csv
from .buffer import TelemetryBuffer
from .plots import TelemetryPlots
from .session import SessionController


GOTO_VMAX_MM_S = 1200.0
GOTO_WMAX_DEG_S = 120.0
GOTO_TIMEOUT_MS = 15000
HEARTBEAT_INTERVAL_MS = 250
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
    box.setButtonSymbols(QAbstractSpinBox.ButtonSymbols.NoButtons)
    return box


def validate_motion_goal(goal: MotionGoal) -> str | None:
    if not all(math.isfinite(value) for value in (goal.x_mm, goal.y_mm, goal.yaw_deg, goal.vmax_mm_s, goal.wmax_deg_s)):
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
    phase = " 航向对准中" if item.yaw_aligning else ""
    return (f"{motion_state} {pose_state}{phase} 目标=({target}) 实际=({actual}) "
            f"PID r{item.pid_revision} 标志=0x{item.flags:02X} 覆盖={item.overwritten_count}")


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__(); self.setWindowTitle("PID 在线调参"); self.resize(1440, 900)
        self.buffer = TelemetryBuffer(); self.session = SessionController(); self.recording = False; self.recorded = []
        self._build(); self._wire()
        self.timer = QTimer(self); self.timer.setInterval(40); self.timer.timeout.connect(self.refresh); self.timer.start()
        self.heartbeat_timer = QTimer(self); self.heartbeat_timer.setInterval(HEARTBEAT_INTERVAL_MS)
        self.heartbeat_timer.timeout.connect(self.session.heartbeat); self.heartbeat_timer.start()

    def _build(self) -> None:
        root = QSplitter(Qt.Orientation.Horizontal)
        root.setChildrenCollapsible(False)
        controls = QWidget()
        controls.setMinimumWidth(300)
        left = QVBoxLayout(controls); left.setContentsMargins(12, 12, 12, 12)
        self.port = QComboBox(); self.refresh_ports_button = QPushButton("刷新 COM"); self.baud = QComboBox(); self.baud.addItems(["115200", "230400"]); self.connect_button = QPushButton("连接")
        connection = QFormLayout(); connection.addRow("串口", self.port); connection.addRow(self.refresh_ports_button); connection.addRow("波特率", self.baud); connection.addRow(self.connect_button); left.addLayout(connection)
        self.status = QLabel("未连接"); left.addWidget(self.status)
        self.pid = [number(value) for value in (1, .03, .1, 2, .05, .08)]
        pid_form = QFormLayout(); [pid_form.addRow(name, widget) for name, widget in zip(("Kp 位置", "Ki 位置", "Kd 位置", "Kp 航向", "Ki 航向", "Kd 航向"), self.pid)]
        self.read_pid = QPushButton("读取 PID"); self.apply_pid = QPushButton("应用 PID"); self.restore_pid = QPushButton("恢复默认")
        pid_form.addRow(self.read_pid); pid_form.addRow(self.apply_pid); pid_form.addRow(self.restore_pid); left.addLayout(pid_form)
        self.profile = QComboBox(); self.load_profile = QPushButton("加载方案"); self.save_profile = QPushButton("另存方案"); self.export_c = QPushButton("导出 C")
        left.addWidget(self.profile); left.addWidget(self.load_profile); left.addWidget(self.save_profile); left.addWidget(self.export_c)
        self.goal = [number(), number(), number(), number(600.0, 0.1, GOTO_VMAX_MM_S), number(120.0, 0.1, GOTO_WMAX_DEG_S)]; self.timeout = QSpinBox(); self.timeout.setRange(1, GOTO_TIMEOUT_MS); self.timeout.setValue(GOTO_TIMEOUT_MS)
        self.use_yaw = QCheckBox("启用航向约束"); self.use_yaw.setChecked(True)
        self.large_yaw_align = QCheckBox("航向误差大时先对准"); self.large_yaw_align.setEnabled(False)
        self.large_yaw_align.setToolTip("仅对位置和航向同时启用的 GOTO 生效；板端复位后恢复固件默认值")
        self.yaw_source = QComboBox(); self.yaw_source.addItems(["WIT", "OPS"]); self.reset_origin = QPushButton("重置零点")
        goal_form = QFormLayout(); [goal_form.addRow(name, widget) for name, widget in zip(("X mm", "Y mm", GOTO_YAW_LABEL, "vmax mm/s", "wmax deg/s"), self.goal)]; goal_form.addRow(self.use_yaw)
        goal_form.addRow(self.large_yaw_align)
        goal_form.addRow("航向 PID 源", self.yaw_source); goal_form.addRow(self.reset_origin)
        goal_form.addRow("超时 ms", self.timeout); self.goto = QPushButton("开始组合 GOTO"); self.goto_position = QPushButton("发送位置 GOTO"); self.goto_yaw = QPushButton("发送角度 GOTO"); self.stop = QPushButton("STOP"); self.stop.setObjectName("stopButton"); self.new_experiment = QPushButton("新实验")
        goal_form.addRow(self.goto); goal_form.addRow(self.goto_position); goal_form.addRow(self.goto_yaw); goal_form.addRow(self.stop); goal_form.addRow(self.new_experiment); left.addLayout(goal_form)
        self.record = QPushButton("开始记录 CSV"); self.window = QSpinBox(); self.window.setRange(5, 120); self.window.setValue(30); left.addWidget(self.record); left.addWidget(QLabel("时间窗口 (秒)")); left.addWidget(self.window); left.addStretch()
        self.controls_scroll = QScrollArea()
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
        self.setCentralWidget(root); self.refresh_ports(); self.reload_profiles()

    def _wire(self) -> None:
        self.connect_button.clicked.connect(self.toggle_connection); self.refresh_ports_button.clicked.connect(self.refresh_ports); self.read_pid.clicked.connect(self.session.read_pid); self.apply_pid.clicked.connect(self.apply_current_pid); self.restore_pid.clicked.connect(self.session.restore_pid)
        self.goto.clicked.connect(self.start_motion); self.goto_position.clicked.connect(self.start_position_motion); self.goto_yaw.clicked.connect(self.start_yaw_motion); self.stop.clicked.connect(self.session.stop); self.new_experiment.clicked.connect(self.new_experiment_clicked); self.record.clicked.connect(self.toggle_record)
        self.yaw_source.currentTextChanged.connect(self.session.set_yaw_source); self.reset_origin.clicked.connect(self.session.reset_origin); self.large_yaw_align.toggled.connect(self.session.set_goto_strategy)
        self.load_profile.clicked.connect(self.load_selected_profile); self.save_profile.clicked.connect(self.save_current_profile); self.export_c.clicked.connect(self.export_current_c)
        self.window.valueChanged.connect(lambda value: self.plots.set_window(float(value))); self.session.telemetry.connect(self.on_telemetry); self.session.status.connect(self.status.setText); self.session.failure.connect(self.on_failure); self.session.pid_read.connect(self.on_pid); self.session.pid_applied.connect(self.on_pid_applied); self.session.yaw_source_changed.connect(self.on_yaw_source_changed); self.session.goto_strategy_read.connect(self.on_goto_strategy_changed); self.session.goto_strategy_changed.connect(self.on_goto_strategy_changed); self.session.origin_reset.connect(self.on_origin_reset); self.session.motion_changed.connect(self.on_motion_changed)

    def refresh_ports(self) -> None:
        from serial.tools import list_ports
        self.port.clear(); self.port.addItems([item.device for item in list_ports.comports()])

    def toggle_connection(self) -> None:
        if self.session.connected: self.session.disconnect(); self.large_yaw_align.setEnabled(False); self.connect_button.setText("连接"); return
        if not self.port.currentText(): self.on_failure("未发现可用 COM 口"); return
        self.session.connect_port(self.port.currentText(), int(self.baud.currentText())); self.connect_button.setText("断开")

    def current_pid(self) -> PidConfig: return PidConfig(*(widget.value() for widget in self.pid))
    def apply_current_pid(self) -> None: self.session.apply_pid(self.current_pid())
    def on_pid(self, revision: int, pid: PidConfig) -> None:
        [widget.setValue(value) for widget, value in zip(self.pid, pid.to_dict().values())]; self.status.setText(f"PID 修订号 {revision}")
    def start_motion(self) -> None:
        self._start_motion(use_position=True, use_yaw=self.use_yaw.isChecked())

    def start_position_motion(self) -> None:
        self._start_motion(use_position=True, use_yaw=False)

    def start_yaw_motion(self) -> None:
        self._start_motion(use_position=False, use_yaw=True)

    def _start_motion(self, use_position: bool, use_yaw: bool) -> None:
        goal = MotionGoal(*(widget.value() for widget in self.goal), self.timeout.value(),
                          use_yaw=use_yaw, use_position=use_position)
        error = validate_motion_goal(goal)
        if error is not None:
            self.status.setText(f"错误: {error}")
            return
        self.session.start_motion(goal)
        kind = "组合" if use_position and use_yaw else ("位置" if use_position else "角度")
        self.status.setText(f"已请求{kind} GOTO")
        self.buffer.add_event(f"{kind} GOTO")
    def on_telemetry(self, item: object) -> None:
        self.buffer.append(item); self.recorded.append(item) if self.recording else None
        status = format_telemetry_status(item)
        if item.heartbeat_timed_out:
            status += " 心跳超时停车"
        elif item.remote_goal_active:
            status += f" 心跳正常/{item.heartbeat_age_ms}ms"
        self.status.setText(status)
        if item.yaw_source != self.yaw_source.currentText():
            self.yaw_source.blockSignals(True); self.yaw_source.setCurrentText(item.yaw_source); self.yaw_source.blockSignals(False)
    def refresh(self) -> None: self.plots.refresh(self.buffer)
    def on_pid_applied(self, revision: int, pid: PidConfig) -> None:
        print(format_pid_apply_log(revision, pid), flush=True)
        self.buffer.add_event(f"PID r{revision}")
        self.status.setText(f"PID 已应用，修订号 {revision}")
    def on_yaw_source_changed(self, source: str) -> None:
        self.status.setText(f"航向 PID 数据源已切换为 {source}"); self.buffer.add_event(f"航向源 {source}")
    def on_goto_strategy_changed(self, enabled: bool) -> None:
        self.large_yaw_align.blockSignals(True); self.large_yaw_align.setChecked(enabled); self.large_yaw_align.blockSignals(False)
        self.large_yaw_align.setEnabled(self.session.connected and not self.session.motion_active)
        self.status.setText("大航向误差先对准已启用" if enabled else "大航向误差先对准已关闭")
        self.buffer.add_event("GOTO 策略: 先对准航向" if enabled else "GOTO 策略: 并行控制")
    def on_motion_changed(self, active: bool) -> None:
        self.large_yaw_align.setEnabled(self.session.connected and not active)
        self.buffer.add_event("运动状态改变")
    def on_origin_reset(self) -> None:
        [widget.setValue(0.0) for widget in self.goal[:3]]; self.buffer.add_event("零点重置"); self.status.setText("零点已重置")
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

"""可复用的 PID 调参与连接、单点运动控制组件。"""

from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QAbstractSpinBox,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QLabel,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from ..models import MotionGoal, PidConfig


GOTO_VMAX_MM_S = 1200.0
GOTO_WMAX_DEG_S = 120.0
GOTO_TIMEOUT_MS = 15000
GOTO_YAW_LABEL = "yaw 相对初始化零点 deg"
HEADING_MODE_WIT = "WIT"
HEADING_MODE_OPS = "OPS"
HEADING_MODE_NONE = "NONE"


def number(value: float = 0.0, minimum: float = -100000.0,
           maximum: float = 100000.0) -> QDoubleSpinBox:
    """创建统一的浮点输入框，避免方向按钮挤占控制区。"""
    box = QDoubleSpinBox()
    box.setRange(minimum, maximum)
    box.setDecimals(4)
    box.setValue(value)
    box.setSingleStep(0.1)
    box.setButtonSymbols(QAbstractSpinBox.ButtonSymbols.NoButtons)
    return box


class ConnectionPanel(QWidget):
    """Reusable serial-port selector; it never opens a port itself."""

    refresh_ports_requested = Signal()
    connect_requested = Signal(str, int)
    disconnect_requested = Signal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.port = QComboBox()
        self.port.setObjectName("comPortCombo")
        self.baud = QComboBox()
        self.baud.setObjectName("baudRateCombo")
        self.baud.addItems(["115200", "230400"])
        self.refresh_ports_button = QPushButton("刷新 COM")
        self.connect_button = QPushButton("连接")
        self.status = QLabel("未连接")
        self._connected = False
        self._connecting = False

        form = QFormLayout(self)
        form.addRow("串口", self.port)
        form.addRow("波特率", self.baud)
        form.addRow(self.refresh_ports_button)
        form.addRow(self.connect_button)
        form.addRow("状态", self.status)
        self.refresh_ports_button.clicked.connect(self.refresh_ports_requested)
        self.connect_button.clicked.connect(self._toggle_connection)
        self._update_enabled()

    def set_available_ports(self, ports: list[str]) -> None:
        previous = self.port.currentText()
        self.port.clear()
        self.port.addItems(ports)
        if previous in ports:
            self.port.setCurrentText(previous)
        self._update_enabled()

    def set_connecting(self, connecting: bool) -> None:
        self.port.setEnabled(not connecting and not self._connected)
        self.baud.setEnabled(not connecting and not self._connected)
        self.refresh_ports_button.setEnabled(not connecting and not self._connected)
        self.connect_button.setEnabled(not connecting and (self._connected or bool(self.port.currentText())))
        self.connect_button.setText("连接中" if connecting else ("断开" if self._connected else "连接"))
        if connecting:
            self.status.setText("正在连接")

    def set_connected(self, connected: bool, status: str = "") -> None:
        self._connected = connected
        self.set_connecting(False)
        self.status.setText(status or ("已连接" if connected else "未连接"))

    def _update_enabled(self) -> None:
        if not self._connected:
            self.connect_button.setEnabled(bool(self.port.currentText()))

    def _toggle_connection(self) -> None:
        if self._connected:
            self.disconnect_requested.emit()
        elif self.port.currentText():
            self.connect_requested.emit(self.port.currentText(), int(self.baud.currentText()))


class PidControlPanel(QWidget):
    """PID 参数编辑与读写操作组件，不直接依赖串口或会话对象。"""

    read_requested = Signal()
    apply_requested = Signal(object)
    restore_requested = Signal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.pid = [number(value) for value in (1, .03, .1, 2, .05, .08)]
        self.read_pid = QPushButton("读取 PID")
        self.apply_pid = QPushButton("应用 PID")
        self.restore_pid = QPushButton("恢复默认")

        form = QFormLayout(self)
        for name, widget in zip(
                ("Kp 位置", "Ki 位置", "Kd 位置", "Kp 航向", "Ki 航向", "Kd 航向"),
                self.pid):
            form.addRow(name, widget)
        form.addRow(self.read_pid)
        form.addRow(self.apply_pid)
        form.addRow(self.restore_pid)

        self.read_pid.clicked.connect(self.read_requested)
        self.apply_pid.clicked.connect(lambda: self.apply_requested.emit(self.current_pid()))
        self.restore_pid.clicked.connect(self.restore_requested)

    def current_pid(self) -> PidConfig:
        return PidConfig(*(widget.value() for widget in self.pid))

    def set_pid(self, pid: PidConfig) -> None:
        for widget, value in zip(self.pid, pid.to_dict().values()):
            widget.setValue(value)


class ConnectionMotionPanel(QWidget):
    """串口连接与单点 GOTO 控制组件，通过信号交由外部会话执行。"""

    refresh_ports_requested = Signal()
    connect_requested = Signal(str, int)
    disconnect_requested = Signal()
    motion_requested = Signal(object)
    stop_requested = Signal()
    yaw_source_requested = Signal(str)
    origin_reset_requested = Signal()
    goto_strategy_requested = Signal(bool)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.port = QComboBox()
        self.refresh_ports_button = QPushButton("刷新 COM")
        self.baud = QComboBox()
        self.baud.addItems(["115200", "230400"])
        self.connect_button = QPushButton("连接")
        self.status = QLabel("未连接")

        self.goal = [
            number(), number(), number(),
            number(600.0, 0.1, GOTO_VMAX_MM_S),
            number(120.0, 0.1, GOTO_WMAX_DEG_S),
        ]
        self.timeout = QSpinBox()
        self.timeout.setRange(1, GOTO_TIMEOUT_MS)
        self.timeout.setValue(GOTO_TIMEOUT_MS)
        self.large_yaw_align = QCheckBox("航向误差大时先对准")
        self.large_yaw_align.setEnabled(False)
        self.large_yaw_align.setToolTip("仅对位置和航向同时启用的 GOTO 生效。")
        self.yaw_source = QComboBox()
        self.yaw_source.addItem("WIT yaw（用于航向控制）", HEADING_MODE_WIT)
        self.yaw_source.addItem("OPS yaw（用于航向控制）", HEADING_MODE_OPS)
        self.yaw_source.addItem("不使用航向（仅控制 X/Y）", HEADING_MODE_NONE)
        self._connected = False
        self._connecting = False
        self._motion_active = False
        self.reset_origin = QPushButton("重置零点")
        self.goto = QPushButton("开始组合 GOTO")
        self.goto_position = QPushButton("发送位置 GOTO")
        self.goto_yaw = QPushButton("发送角度 GOTO")
        self.stop = QPushButton("STOP")
        self.stop.setObjectName("stopButton")

        layout = QVBoxLayout(self)
        connection = QFormLayout()
        connection.addRow("串口", self.port)
        connection.addRow(self.refresh_ports_button)
        connection.addRow("波特率", self.baud)
        connection.addRow(self.connect_button)
        layout.addLayout(connection)
        layout.addWidget(self.status)

        motion = QFormLayout()
        for name, widget in zip(
                ("X mm", "Y mm", GOTO_YAW_LABEL, "vmax mm/s", "wmax deg/s"), self.goal):
            motion.addRow(name, widget)
        motion.addRow(self.large_yaw_align)
        motion.addRow("航向控制模式", self.yaw_source)
        motion.addRow(self.reset_origin)
        motion.addRow("超时 ms", self.timeout)
        motion.addRow(self.goto)
        motion.addRow(self.goto_position)
        motion.addRow(self.goto_yaw)
        motion.addRow(self.stop)
        layout.addLayout(motion)

        self.refresh_ports_button.clicked.connect(self.refresh_ports_requested)
        self.connect_button.clicked.connect(self._toggle_connection)
        self.goto.clicked.connect(lambda: self._emit_motion(True, self.uses_yaw()))
        self.goto_position.clicked.connect(lambda: self._emit_motion(True, False))
        self.goto_yaw.clicked.connect(lambda: self._emit_motion(False, True))
        self.stop.clicked.connect(self.stop_requested)
        self.yaw_source.currentIndexChanged.connect(self._heading_mode_changed)
        self.reset_origin.clicked.connect(self.origin_reset_requested)
        self.large_yaw_align.toggled.connect(self.goto_strategy_requested)
        self._update_connection_controls()
        self._update_heading_controls()

    def set_available_ports(self, ports: list[str]) -> None:
        self.port.clear()
        self.port.addItems(ports)
        self._update_connection_controls()

    def set_connected(self, connected: bool) -> None:
        self._connected = connected
        self._connecting = False
        self._update_connection_controls()
        self._update_heading_controls()

    def set_connecting(self, connecting: bool) -> None:
        self._connecting = connecting
        self._update_connection_controls()

    def set_motion_active(self, active: bool) -> None:
        """锁定运行中的航向模式，避免一次 GOTO 中途改变控制语义。"""
        self._motion_active = active
        self._update_heading_controls()

    def heading_mode(self) -> str:
        """返回协议使用的航向模式标识。"""
        return str(self.yaw_source.currentData())

    def uses_yaw(self) -> bool:
        return self.heading_mode() != HEADING_MODE_NONE

    def set_heading_mode(self, mode: str) -> None:
        """同步板端实际航向源，但不再次发送设置命令。"""
        index = self.yaw_source.findData(mode.upper())
        if index < 0:
            raise ValueError(f"unknown heading mode: {mode}")
        self.yaw_source.blockSignals(True)
        self.yaw_source.setCurrentIndex(index)
        self.yaw_source.blockSignals(False)
        self._update_heading_controls()

    def _heading_mode_changed(self) -> None:
        mode = self.heading_mode()
        self._update_heading_controls()
        self.yaw_source_requested.emit(mode)

    def _update_heading_controls(self) -> None:
        uses_yaw = self.uses_yaw()
        self.goal[2].setEnabled(uses_yaw)
        self.goal[4].setEnabled(uses_yaw)
        self.goto_yaw.setEnabled(uses_yaw)
        self.large_yaw_align.setEnabled(
            uses_yaw and self._connected and not self._motion_active
        )
        self.yaw_source.setEnabled(not self._motion_active)
        self.goto.setText("开始位置+航向 GOTO" if uses_yaw else "开始仅位置 GOTO")

    def _update_connection_controls(self) -> None:
        self.port.setEnabled(not self._connected and not self._connecting)
        self.baud.setEnabled(not self._connected and not self._connecting)
        self.refresh_ports_button.setEnabled(not self._connected and not self._connecting)
        self.connect_button.setEnabled(self._connected or (
            not self._connecting and bool(self.port.currentText())))
        self.connect_button.setText("连接中" if self._connecting else (
            "断开" if self._connected else "连接"))

    def current_motion_goal(self, use_position: bool, use_yaw: bool) -> MotionGoal:
        return MotionGoal(*(widget.value() for widget in self.goal), self.timeout.value(),
                          use_yaw=use_yaw, use_position=use_position)

    def _toggle_connection(self) -> None:
        if self.connect_button.text() == "断开":
            self.disconnect_requested.emit()
        elif self.port.currentText():
            self.connect_requested.emit(self.port.currentText(), int(self.baud.currentText()))

    def _emit_motion(self, use_position: bool, use_yaw: bool) -> None:
        self.motion_requested.emit(self.current_motion_goal(use_position, use_yaw))

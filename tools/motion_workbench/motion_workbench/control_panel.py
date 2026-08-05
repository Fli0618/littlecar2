"""Workbench-specific point and path command controls."""

from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtGui import QFocusEvent, QMouseEvent, QWheelEvent
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from pid_tuner.models import HolonomicConfig, HolonomicConfigState, MotionGoal, PidConfig, PidConfigState

from .models import TargetPose


HEADING_MODE_WIT = "WIT"
HEADING_MODE_OPS = "OPS"
HEADING_MODE_NONE = "NONE"


class DoubleClickLineEdit(QLineEdit):
    """只在双击后开放键盘输入，失焦或提交后重新锁定。"""

    def __init__(self) -> None:
        super().__init__()
        self.setReadOnly(True)

    def mouseDoubleClickEvent(self, event: QMouseEvent) -> None:  # noqa: N802 - Qt override
        self.setReadOnly(False)
        super().mouseDoubleClickEvent(event)
        self.selectAll()

    def focusOutEvent(self, event: QFocusEvent) -> None:  # noqa: N802 - Qt override
        self.lock()
        super().focusOutEvent(event)

    def lock(self) -> None:
        self.setReadOnly(True)


class ProtectedDoubleSpinBox(QDoubleSpinBox):
    """允许双击键入或按钮步进，但把滚轮事件交给外层滚动区。"""

    def __init__(self) -> None:
        super().__init__()
        editor = DoubleClickLineEdit()
        self.setLineEdit(editor)
        self.setKeyboardTracking(False)
        self.editingFinished.connect(editor.lock)
        self.setToolTip("双击数值后键盘输入；也可使用右侧按钮。鼠标滚轮不会修改参数。")

    def wheelEvent(self, event: QWheelEvent) -> None:  # noqa: N802 - Qt override
        event.ignore()


def protected_number(
    value: float,
    minimum: float,
    maximum: float,
    step: float,
    decimals: int = 4,
) -> ProtectedDoubleSpinBox:
    """创建 workbench 专用的受保护浮点参数框。"""
    box = ProtectedDoubleSpinBox()
    box.setRange(minimum, maximum)
    box.setDecimals(decimals)
    box.setSingleStep(step)
    box.setValue(value)
    return box


class WorkbenchPidControlPanel(QWidget):
    """PID 参数页；数值仅允许双击输入或使用步进按钮修改。"""

    read_requested = Signal()
    apply_requested = Signal(object)
    restore_requested = Signal()
    goto_strategy_changed = Signal(bool)

    def __init__(self) -> None:
        super().__init__()
        defaults = (1.5, 0.10, 0.78, 2.50, 1.0, 0.80)
        self.pid = [protected_number(value, 0.0, 20.0, 0.01) for value in defaults]
        self.read_pid = QPushButton("读取 PID")
        self.apply_pid = QPushButton("应用 PID")
        self.restore_pid = QPushButton("恢复默认")
        self.status = QLabel("PID 未同步")
        self._connected = False
        self._motion_active = False
        form = QFormLayout(self)
        for name, widget in zip(
            ("Kp 位置", "Ki 位置", "Kd 位置", "Kp 航向", "Ki 航向", "Kd 航向"),
            self.pid,
        ):
            form.addRow(name, widget)
        form.addRow(self.read_pid)
        form.addRow(self.apply_pid)
        form.addRow(self.restore_pid)
        form.addRow("状态", self.status)
        self.large_yaw_align = QCheckBox("大航向误差时先对准航向")
        form.addRow(self.large_yaw_align)
        self.read_pid.clicked.connect(self.read_requested)
        self.apply_pid.clicked.connect(lambda: self.apply_requested.emit(self.current_pid()))
        self.restore_pid.clicked.connect(self.restore_requested)
        self.large_yaw_align.toggled.connect(self.goto_strategy_changed)
        self.set_connected(False)

    def current_pid(self) -> PidConfig:
        return PidConfig(*(widget.value() for widget in self.pid))

    def set_pid(self, pid: PidConfig) -> None:
        for widget, value in zip(self.pid, pid.to_dict().values()):
            widget.setValue(value)

    def set_pid_state(self, state: PidConfigState) -> None:
        self.set_pid(state.config)
        self.status.setText(f"PID r{state.revision} 已同步")

    def set_connected(self, connected: bool) -> None:
        self._connected = connected
        self.apply_pid.setEnabled(connected)
        self.restore_pid.setEnabled(connected)
        self.large_yaw_align.setEnabled(connected and not self._motion_active)
        if not connected:
            self.status.setText("PID 未同步")

    def set_goto_strategy(self, enabled: bool) -> None:
        self.large_yaw_align.blockSignals(True)
        self.large_yaw_align.setChecked(enabled)
        self.large_yaw_align.blockSignals(False)

    def set_motion_active(self, active: bool) -> None:
        self._motion_active = active
        self.large_yaw_align.setEnabled(self._connected and not active)


HOLONOMIC_FIELD_GROUPS = (
    ("运动轮廓", (
        ("linear_accel_mm_s2", "平移加速度 mm/s²", 600.0),
        ("linear_decel_mm_s2", "平移减速度 mm/s²", 800.0),
        ("yaw_accel_deg_s2", "航向角加速度 deg/s²", 150.0),
    )),
    ("前向反馈", (
        ("kp_forward", "前向位置 Kp (1/s)", 0.8),
        ("kv_forward", "前向速度 Kv", 0.3),
    )),
    ("横向反馈", (
        ("kp_lateral", "横向位置 Kp (1/s)", 0.8),
        ("kv_lateral", "横向速度 Kv", 0.3),
    )),
    ("航向反馈", (
        ("kp_yaw", "航向位置 Kp (1/s)", 2.0),
        ("kv_yaw", "航向速度 Kv", 0.3),
    )),
    ("驱动比例校准", (
        ("forward_scale", "前向 scale", 1.0),
        ("lateral_scale", "横向 scale", 1.0),
        ("yaw_scale", "旋转 scale", 1.0),
    )),
)


class HolonomicControlPanel(QWidget):
    """全向位置控制器 12 项参数页；数值仅允许双击输入或使用步进按钮修改。"""

    read_requested = Signal()
    apply_requested = Signal(object)
    restore_requested = Signal()

    def __init__(self) -> None:
        super().__init__()
        self.inputs: dict[str, ProtectedDoubleSpinBox] = {}
        layout = QVBoxLayout(self)
        for title, fields in HOLONOMIC_FIELD_GROUPS:
            group = QGroupBox(title)
            form = QFormLayout(group)
            for name, label, value in fields:
                if name in ("linear_accel_mm_s2", "linear_decel_mm_s2", "yaw_accel_deg_s2"):
                    box = protected_number(value, 0.01, 5000.0, 10.0)
                elif name.endswith("_scale"):
                    box = protected_number(value, 0.5, 2.0, 0.01)
                else:
                    box = protected_number(value, 0.0, 20.0, 0.01)
                self.inputs[name] = box
                form.addRow(label, box)
            layout.addWidget(group)
        self.read_button = QPushButton("读取参数")
        self.apply_button = QPushButton("应用参数")
        self.restore_button = QPushButton("恢复默认")
        buttons = QHBoxLayout()
        buttons.addWidget(self.read_button)
        buttons.addWidget(self.apply_button)
        buttons.addWidget(self.restore_button)
        layout.addLayout(buttons)
        self.revision_label = QLabel("当前修订号：-")
        self.status = QLabel("全向参数：未读取")
        self.status.setWordWrap(True)
        layout.addWidget(self.revision_label)
        layout.addWidget(self.status)
        layout.addStretch()
        self.read_button.clicked.connect(self.read_requested)
        self.apply_button.clicked.connect(
            lambda: self.apply_requested.emit(self.current_config()))
        self.restore_button.clicked.connect(self.restore_requested)
        self._connected = False
        self._unsupported = False
        self.set_connected(False)

    def current_config(self) -> HolonomicConfig:
        return HolonomicConfig(*(box.value() for box in self.inputs.values()))

    def set_config(self, state: HolonomicConfigState) -> None:
        for name, box in self.inputs.items():
            box.setValue(getattr(state.config, name))
        self.revision_label.setText(f"当前修订号：{state.revision}")
        self.status.setText(f"全向参数 r{state.revision} 已同步")

    def set_connected(self, connected: bool) -> None:
        self._connected = connected
        enabled = connected and not self._unsupported
        self.read_button.setEnabled(enabled)
        self.apply_button.setEnabled(enabled)
        self.restore_button.setEnabled(enabled)
        self.setEnabled(not self._unsupported)
        if not connected:
            self.revision_label.setText("当前修订号：-")
            self.status.setText("全向参数：未读取")

    def set_unsupported(self, unsupported: bool) -> None:
        self._unsupported = unsupported
        self.set_connected(self._connected)
        if unsupported:
            self.status.setText("当前固件不支持全向调参")

    def set_applying(self) -> None:
        self.status.setText("全向参数：等待下位机应用…")

    def set_applied(self, state: HolonomicConfigState) -> None:
        self.set_config(state)
        self.status.setText(
            f"已生效 r{state.revision}；Kp/Kv/scale 下一控制周期生效，"
            "accel/decel 下一次 Start 生效（当前轮廓不重算）")


def _number(value: float = 0.0, minimum: float = -5000.0, maximum: float = 5000.0) -> QDoubleSpinBox:
    box = QDoubleSpinBox()
    box.setRange(minimum, maximum)
    box.setDecimals(2)
    box.setSingleStep(1.0)
    box.setValue(value)
    return box


class PointControlPanel(QWidget):
    candidate_edited = Signal(object)
    send_requested = Signal(object)
    stop_requested = Signal()
    clear_requested = Signal()
    heading_mode_requested = Signal(str)
    controller_changed = Signal(str)

    def __init__(self) -> None:
        super().__init__()
        self.x, self.y, self.yaw = _number(), _number(), _number(0.0, -180.0, 180.0)
        self.vmax, self.wmax = _number(600.0, 0.1, 1500.0), _number(120.0, 0.1, 180.0)
        self.timeout = QSpinBox(); self.timeout.setRange(1, 60000); self.timeout.setValue(10000)
        self.heading_mode = QComboBox()
        self.heading_mode.addItem("WIT yaw（用于航向控制）", HEADING_MODE_WIT)
        self.heading_mode.addItem("OPS yaw（用于航向控制）", HEADING_MODE_OPS)
        self.heading_mode.addItem("不使用航向（仅控制 X/Y）", HEADING_MODE_NONE)
        self.controller = QComboBox()
        self.controller.addItem("经典位置 PID", "classic")
        self.controller.addItem("全向位置控制", "holonomic")
        self._motion_active = False
        form = QFormLayout(); form.addRow("X (mm)", self.x); form.addRow("Y (mm)", self.y); form.addRow("yaw (deg)", self.yaw)
        form.addRow("vmax (mm/s)", self.vmax); form.addRow("wmax (deg/s)", self.wmax); form.addRow("超时 (ms)", self.timeout)
        form.addRow("航向控制模式", self.heading_mode)
        form.addRow("控制器", self.controller)
        self.goto = QPushButton("发送组合 GOTO")
        self.position = QPushButton("发送位置 GOTO")
        self.rotate = QPushButton("仅旋转")
        self.clear = QPushButton("清除候选目标")
        self.stop = QPushButton("STOP"); self.stop.setObjectName("stopButton")
        layout = QVBoxLayout(self); layout.addLayout(form)
        for button in (self.goto, self.position, self.rotate, self.clear, self.stop): layout.addWidget(button)
        layout.addStretch()
        for box in (self.x, self.y, self.yaw): box.valueChanged.connect(self._emit_candidate)
        self.goto.clicked.connect(lambda: self._emit_goal(True, self.uses_yaw()))
        self.position.clicked.connect(lambda: self._emit_goal(True, False))
        self.rotate.clicked.connect(lambda: self._emit_goal(False, True))
        self.clear.clicked.connect(self.clear_requested)
        self.stop.clicked.connect(self.stop_requested)
        self.heading_mode.currentIndexChanged.connect(self._heading_mode_changed)
        self.controller.currentIndexChanged.connect(self._controller_mode_changed)
        self._update_heading_controls()

    def current_heading_mode(self) -> str:
        return str(self.heading_mode.currentData())

    def uses_yaw(self) -> bool:
        return self.current_heading_mode() != HEADING_MODE_NONE

    def current_controller(self) -> str:
        return str(self.controller.currentData())

    def set_heading_mode(self, mode: str) -> None:
        index = self.heading_mode.findData(mode.upper())
        if index < 0:
            raise ValueError(f"unknown heading mode: {mode}")
        self.heading_mode.blockSignals(True)
        self.heading_mode.setCurrentIndex(index)
        self.heading_mode.blockSignals(False)
        self._update_heading_controls()

    def set_motion_active(self, active: bool) -> None:
        self._motion_active = active
        self._update_heading_controls()

    def _heading_mode_changed(self, _index: int) -> None:
        self._update_heading_controls()
        self.heading_mode_requested.emit(self.current_heading_mode())

    def _update_heading_controls(self) -> None:
        uses_yaw = self.uses_yaw()
        self.yaw.setEnabled(uses_yaw)
        self.wmax.setEnabled(uses_yaw)
        self.rotate.setEnabled(uses_yaw)
        self.heading_mode.setEnabled(not self._motion_active)
        self.controller.setEnabled(not self._motion_active)
        self.goto.setText("发送位置+航向 GOTO" if uses_yaw else "发送仅位置 GOTO")

    def _controller_mode_changed(self, _index: int) -> None:
        self.controller_changed.emit(self.current_controller())

    def set_candidate(self, pose: TargetPose | None) -> None:
        if pose is None:
            return
        for box, value in zip((self.x, self.y, self.yaw), (pose.x_mm, pose.y_mm, pose.yaw_deg)):
            box.blockSignals(True); box.setValue(value); box.blockSignals(False)

    def _emit_candidate(self) -> None:
        self.candidate_edited.emit(TargetPose(self.x.value(), self.y.value(), self.yaw.value()))

    def _emit_goal(self, use_position: bool, use_yaw: bool) -> None:
        self.send_requested.emit(MotionGoal(self.x.value(), self.y.value(), self.yaw.value(), self.vmax.value(),
                                            self.wmax.value(), self.timeout.value(), use_yaw, use_position))

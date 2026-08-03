"""Workbench-specific point and path command controls."""

from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (QCheckBox, QDoubleSpinBox, QFormLayout, QHBoxLayout, QLabel, QPushButton,
                               QSpinBox, QVBoxLayout, QWidget)

from pid_tuner.models import MotionGoal

from .models import TargetPose


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

    def __init__(self) -> None:
        super().__init__()
        self.x, self.y, self.yaw = _number(), _number(), _number(0.0, -180.0, 180.0)
        self.vmax, self.wmax = _number(600.0, 0.1, 1500.0), _number(120.0, 0.1, 180.0)
        self.timeout = QSpinBox(); self.timeout.setRange(1, 60000); self.timeout.setValue(10000)
        self.use_yaw = QCheckBox("启用航向约束"); self.use_yaw.setChecked(True)
        form = QFormLayout(); form.addRow("X (mm)", self.x); form.addRow("Y (mm)", self.y); form.addRow("yaw (deg)", self.yaw)
        form.addRow("vmax (mm/s)", self.vmax); form.addRow("wmax (deg/s)", self.wmax); form.addRow("超时 (ms)", self.timeout)
        form.addRow(self.use_yaw)
        self.goto = QPushButton("发送组合 GOTO")
        self.position = QPushButton("发送位置 GOTO")
        self.rotate = QPushButton("仅旋转")
        self.clear = QPushButton("清除候选目标")
        self.stop = QPushButton("STOP"); self.stop.setObjectName("stopButton")
        layout = QVBoxLayout(self); layout.addLayout(form)
        for button in (self.goto, self.position, self.rotate, self.clear, self.stop): layout.addWidget(button)
        layout.addStretch()
        for box in (self.x, self.y, self.yaw): box.valueChanged.connect(self._emit_candidate)
        self.goto.clicked.connect(lambda: self._emit_goal(True, self.use_yaw.isChecked()))
        self.position.clicked.connect(lambda: self._emit_goal(True, False))
        self.rotate.clicked.connect(lambda: self._emit_goal(False, True))
        self.clear.clicked.connect(self.clear_requested)
        self.stop.clicked.connect(self.stop_requested)

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

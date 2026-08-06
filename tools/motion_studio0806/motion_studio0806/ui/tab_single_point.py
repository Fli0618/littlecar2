"""Tab 1: Single Point & Holonomic Position Tuning Panel (Clean compact left panel)."""

from __future__ import annotations

from typing import Optional

from PySide6.QtCore import Qt
from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QComboBox, QDoubleSpinBox, QFormLayout, QGroupBox, QHBoxLayout,
    QPushButton, QVBoxLayout, QWidget
)

from motion_studio0806.core.models import ControlMode, HolonomicParams, TargetPose
from motion_studio0806.core.session import StudioSession


class SinglePointTab(QWidget):
    """Tab 1: Compact Single Point & Holonomic Control Panel."""

    target_changed = Signal(object)  # Emits TargetPose to update map canvas marker

    def __init__(self, session: StudioSession, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.session = session
        self._init_ui()

    def _init_ui(self) -> None:
        main_layout = QVBoxLayout(self)

        # Mode Selection
        mode_group = QGroupBox("控制模式选择 (Mode Select)")
        mode_layout = QFormLayout(mode_group)
        self.combo_mode = QComboBox()
        self.combo_mode.addItem("全向位置控制器 (Holonomic Profile)", ControlMode.HOLONOMIC)
        self.combo_mode.addItem("经典单点 PID (Classic P2P)", ControlMode.CLASSIC_PID)
        self.combo_mode.currentIndexChanged.connect(self._on_mode_changed)
        mode_layout.addRow("运行模式:", self.combo_mode)
        main_layout.addWidget(mode_group)

        # Target Pose Input Group
        target_group = QGroupBox("目标位姿设置 (Target Pose)")
        target_layout = QFormLayout(target_group)

        self.spin_x = QDoubleSpinBox()
        self.spin_x.setRange(-5000.0, 5000.0)
        self.spin_x.setSuffix(" mm")
        self.spin_x.setValue(0.0)

        self.spin_y = QDoubleSpinBox()
        self.spin_y.setRange(-5000.0, 5000.0)
        self.spin_y.setSuffix(" mm")
        self.spin_y.setValue(0.0)

        self.spin_yaw = QDoubleSpinBox()
        self.spin_yaw.setRange(-180.0, 180.0)
        self.spin_yaw.setSuffix(" deg")
        self.spin_yaw.setValue(0.0)

        self.spin_vmax = QDoubleSpinBox()
        self.spin_vmax.setRange(50.0, 1500.0)
        self.spin_vmax.setSuffix(" mm/s")
        self.spin_vmax.setValue(800.0)

        target_layout.addRow("目标 X 坐标:", self.spin_x)
        target_layout.addRow("目标 Y 坐标:", self.spin_y)
        target_layout.addRow("目标 Yaw 角度:", self.spin_yaw)
        target_layout.addRow("最大线速度:", self.spin_vmax)

        btn_layout = QHBoxLayout()
        self.btn_send = QPushButton("发送目标 (Send Goal)")
        self.btn_send.setStyleSheet("font-weight: bold; background-color: #2b5797; color: white;")
        self.btn_send.clicked.connect(self._on_send_goal)

        self.btn_cancel = QPushButton("取消并停车 (Cancel)")
        self.btn_cancel.clicked.connect(self._on_cancel)

        btn_layout.addWidget(self.btn_send)
        btn_layout.addWidget(self.btn_cancel)
        target_layout.addRow(btn_layout)

        main_layout.addWidget(target_group)

        # Holonomic 12-Params Group
        self.group_holonomic = QGroupBox("Holonomic 12项运行时热加载参数")
        holo_layout = QFormLayout(self.group_holonomic)

        self.spin_acc_x = QDoubleSpinBox()
        self.spin_acc_x.setRange(100, 3000)
        self.spin_acc_x.setValue(1000)

        self.spin_kp_x = QDoubleSpinBox()
        self.spin_kp_x.setRange(0, 50)
        self.spin_kp_x.setValue(2.0)

        self.spin_kv_x = QDoubleSpinBox()
        self.spin_kv_x.setRange(0, 50)
        self.spin_kv_x.setValue(1.0)

        holo_layout.addRow("Acc X (mm/s²):", self.spin_acc_x)
        holo_layout.addRow("Kp X:", self.spin_kp_x)
        holo_layout.addRow("Kv X:", self.spin_kv_x)

        btn_holo = QPushButton("应用全向参数 (Apply Holonomic Params)")
        btn_holo.clicked.connect(self._on_apply_holonomic)
        holo_layout.addRow(btn_holo)

        main_layout.addWidget(self.group_holonomic)
        main_layout.addStretch()

    def _on_mode_changed(self) -> None:
        mode = self.combo_mode.currentData()
        self.group_holonomic.setVisible(mode == ControlMode.HOLONOMIC)

    def _on_send_goal(self) -> None:
        target = TargetPose(
            x_mm=self.spin_x.value(),
            y_mm=self.spin_y.value(),
            yaw_deg=self.spin_yaw.value(),
            v_max_mm_s=self.spin_vmax.value(),
        )
        mode = self.combo_mode.currentData()
        self.target_changed.emit(target)
        self.session.set_single_point(target, mode)

    def _on_cancel(self) -> None:
        self.session.emergency_stop()

    def _on_apply_holonomic(self) -> None:
        self.session.status_message.emit("全向参数已提交更新！")

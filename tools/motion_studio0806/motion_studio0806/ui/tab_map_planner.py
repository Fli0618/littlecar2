"""Tab 3: Full Competition Map & Costmap Configurations Panel (100% exact copy of full costmap parameters)."""

from __future__ import annotations

from typing import Optional

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QDoubleSpinBox, QFormLayout, QGroupBox, QHBoxLayout,
    QPushButton, QScrollArea, QVBoxLayout, QWidget, QLabel
)

from motion_studio0806.core.session import StudioSession


class MapPlannerTab(QWidget):
    """Tab 3: Comprehensive Map & Costmap Parameters Matching Full Workbench Spec."""

    config_changed = Signal(dict)  # Emits configuration dict to update right-side map

    def __init__(self, session: StudioSession, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.session = session
        self.obstacle_count = 0
        self._init_ui()

    def _init_ui(self) -> None:
        outer_layout = QVBoxLayout(self)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        layout = QVBoxLayout(scroll_content)

        # 1. Random Obstacle Editor Panel
        group_obs = QGroupBox("随机障碍物编辑")
        obs_box = QVBoxLayout(group_obs)
        btn_row = QHBoxLayout()

        self.btn_place_obs = QPushButton("放置障碍物")
        self.btn_select_obs = QPushButton("选择/拖动")
        self.btn_del_obs = QPushButton("删除选中障碍物")
        self.lbl_obs_count = QLabel("障碍物: 0 个")

        btn_row.addWidget(self.btn_place_obs)
        btn_row.addWidget(self.btn_select_obs)
        btn_row.addWidget(self.btn_del_obs)
        obs_box.addLayout(btn_row)
        obs_box.addWidget(self.lbl_obs_count)
        layout.addWidget(group_obs)

        # 2. Main Costmap Container
        group_cost = QGroupBox("代价地图: 红色物理轮廓 / 粉色硬禁区 / 黄色软代价区")
        cost_box = QVBoxLayout(group_cost)

        # Sub 1: Vehicle Dimensions
        group_car = QGroupBox("车体尺寸 (用于规划与完整车体扫描)")
        form_car = QFormLayout(group_car)
        self.spin_length = QDoubleSpinBox(); self.spin_length.setRange(100, 1000); self.spin_length.setValue(300.0); self.spin_length.setSuffix(" mm")
        self.spin_width = QDoubleSpinBox(); self.spin_width.setRange(100, 1000); self.spin_width.setValue(300.0); self.spin_width.setSuffix(" mm")
        form_car.addRow("车身长度 (mm)", self.spin_length)
        form_car.addRow("车身宽度 (mm)", self.spin_width)
        cost_box.addWidget(group_car)

        # Sub 2: Field Border
        group_border = QGroupBox("场地边线 (向场内膨胀)")
        form_border = QFormLayout(group_border)
        self.spin_border_safe = QDoubleSpinBox(); self.spin_border_safe.setValue(20.0); self.spin_border_safe.setSuffix(" mm")
        self.spin_border_soft = QDoubleSpinBox(); self.spin_border_soft.setValue(120.0); self.spin_border_soft.setSuffix(" mm")
        self.spin_border_weight = QDoubleSpinBox(); self.spin_border_weight.setValue(3.00)
        form_border.addRow("安全距离 (mm)", self.spin_border_safe)
        form_border.addRow("软膨胀距离 (mm)", self.spin_border_soft)
        form_border.addRow("软代价权重", self.spin_border_weight)
        cost_box.addWidget(group_border)

        # Sub 3: Functional Zones
        group_zone = QGroupBox("边界功能区内突")
        form_zone = QFormLayout(group_zone)
        self.spin_raw_half = QDoubleSpinBox(); self.spin_raw_half.setValue(200.0); self.spin_raw_half.setSuffix(" mm")
        self.spin_raw_depth = QDoubleSpinBox(); self.spin_raw_depth.setValue(85.0); self.spin_raw_depth.setSuffix(" mm")
        self.spin_temp_half = QDoubleSpinBox(); self.spin_temp_half.setValue(290.0); self.spin_temp_half.setSuffix(" mm")
        self.spin_temp_width = QDoubleSpinBox(); self.spin_temp_width.setValue(150.0); self.spin_temp_width.setSuffix(" mm")
        self.spin_temp_soft = QDoubleSpinBox(); self.spin_temp_soft.setValue(35.0); self.spin_temp_soft.setSuffix(" mm")
        form_zone.addRow("原料区半宽 (mm)", self.spin_raw_half)
        form_zone.addRow("原料区深度 (mm)", self.spin_raw_depth)
        form_zone.addRow("暂存/粗加工区半长 (mm)", self.spin_temp_half)
        form_zone.addRow("暂存/粗加工区宽度 (mm)", self.spin_temp_width)
        form_zone.addRow("软膨胀距离 (mm)", self.spin_temp_soft)
        cost_box.addWidget(group_zone)

        # Sub 4: Yellow Platforms (Inner)
        group_plat_in = QGroupBox("四个黄色平台 (组内侧)")
        form_plat_in = QFormLayout(group_plat_in)
        self.spin_in_safe = QDoubleSpinBox(); self.spin_in_safe.setValue(20.0); self.spin_in_safe.setSuffix(" mm")
        self.spin_in_soft = QDoubleSpinBox(); self.spin_in_soft.setValue(20.0); self.spin_in_soft.setSuffix(" mm")
        self.spin_in_weight = QDoubleSpinBox(); self.spin_in_weight.setValue(3.00)
        form_plat_in.addRow("安全距离 (mm)", self.spin_in_safe)
        form_plat_in.addRow("软膨胀距离 (mm)", self.spin_in_soft)
        form_plat_in.addRow("软代价权重", self.spin_in_weight)
        cost_box.addWidget(group_plat_in)

        # Sub 5: Yellow Platforms (Outer)
        group_plat_out = QGroupBox("四个黄色平台 (组外侧)")
        form_plat_out = QFormLayout(group_plat_out)
        self.spin_out_soft = QDoubleSpinBox(); self.spin_out_soft.setValue(240.0); self.spin_out_soft.setSuffix(" mm")
        self.spin_out_weight = QDoubleSpinBox(); self.spin_out_weight.setValue(3.80)
        form_plat_out.addRow("软膨胀距离 (mm)", self.spin_out_soft)
        form_plat_out.addRow("软代价权重", self.spin_out_weight)
        cost_box.addWidget(group_plat_out)

        layout.addWidget(group_cost)

        # Apply & Sync Button
        btn_apply = QPushButton("应用所有地图代价参数 (Apply Costmap Config)")
        btn_apply.setStyleSheet("font-weight: bold; background-color: #2b5797; color: white; height: 34px;")
        btn_apply.clicked.connect(self._emit_config)
        layout.addWidget(btn_apply)

        layout.addStretch()
        scroll.setWidget(scroll_content)
        outer_layout.addWidget(scroll)

        # Connect live value changed signals
        for spin_box in [
            self.spin_length, self.spin_width, self.spin_border_safe, self.spin_border_soft,
            self.spin_border_weight, self.spin_raw_half, self.spin_raw_depth, self.spin_temp_half,
            self.spin_temp_width, self.spin_temp_soft, self.spin_in_safe, self.spin_in_soft,
            self.spin_in_weight, self.spin_out_soft, self.spin_out_weight
        ]:
            spin_box.valueChanged.connect(self._emit_config)

    def _emit_config(self) -> None:
        cfg = {
            "car_length": self.spin_length.value(),
            "car_width": self.spin_width.value(),
            "border_safe": self.spin_border_safe.value(),
            "border_soft": self.spin_border_soft.value(),
            "border_weight": self.spin_border_weight.value(),
            "raw_half": self.spin_raw_half.value(),
            "raw_depth": self.spin_raw_depth.value(),
            "temp_half": self.spin_temp_half.value(),
            "temp_width": self.spin_temp_width.value(),
            "temp_soft": self.spin_temp_soft.value(),
            "in_safe": self.spin_in_safe.value(),
            "in_soft": self.spin_in_soft.value(),
            "in_weight": self.spin_in_weight.value(),
            "out_soft": self.spin_out_soft.value(),
            "out_weight": self.spin_out_weight.value(),
        }
        self.config_changed.emit(cfg)

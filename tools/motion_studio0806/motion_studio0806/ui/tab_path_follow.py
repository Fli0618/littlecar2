"""Tab 2: Path Planning, RViz Waypoint Marking & Benchmark Testing Page."""

from __future__ import annotations

from typing import List, Optional

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QDoubleSpinBox, QFormLayout, QGroupBox, QHBoxLayout,
    QHeaderView, QListWidget, QListWidgetItem, QMessageBox,
    QPushButton, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget, QLabel
)

from motion_studio0806.core.models import PathTemplate, PathWaypoint
from motion_studio0806.core.session import StudioSession


class PathFollowTab(QWidget):
    """Tab 2: Route creation, RViz style waypoint marking, and A* path execution."""

    waypoints_changed = Signal(list)  # Broadcasts list of PathWaypoint for map rendering

    def __init__(self, session: StudioSession, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.session = session

        # Default 3 waypoints: Start (1), Mid (2), Goal (3)
        self.waypoints: List[PathWaypoint] = [
            PathWaypoint(x_mm=2250.0, y_mm=150.0, yaw_deg=0.0, v_max_mm_s=800.0),
            PathWaypoint(x_mm=1500.0, y_mm=1500.0, yaw_deg=45.0, v_max_mm_s=800.0),
            PathWaypoint(x_mm=500.0, y_mm=2250.0, yaw_deg=90.0, v_max_mm_s=800.0),
        ]

        self._init_ui()
        self.refresh_table()

    def _init_ui(self) -> None:
        main_layout = QVBoxLayout(self)

        # 1. RViz-style Waypoint Marker Table (Default 3 Points)
        wp_group = QGroupBox("路径标定点位 (默认3个点位，可增删标定)")
        wp_layout = QVBoxLayout(wp_group)

        self.table_wp = QTableWidget(0, 4)
        self.table_wp.setHorizontalHeaderLabels(["X (mm)", "Y (mm)", "Yaw (°)", "速度"])
        self.table_wp.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table_wp.cellChanged.connect(self._on_cell_changed)
        wp_layout.addWidget(self.table_wp)

        btn_box = QHBoxLayout()
        btn_add = QPushButton("插入中间点 (+)")
        btn_add.clicked.connect(self._on_add_point)

        btn_del = QPushButton("删除选中点 (-)")
        btn_del.clicked.connect(self._on_del_point)

        btn_box.addWidget(btn_add)
        btn_box.addWidget(btn_del)
        wp_layout.addLayout(btn_box)

        main_layout.addWidget(wp_group)

        # 2. Path Motion & Tracking Tuning
        param_group = QGroupBox("A* 平滑与轨迹追踪参数")
        param_layout = QFormLayout(param_group)

        self.spin_lookahead = QDoubleSpinBox()
        self.spin_lookahead.setRange(50, 1000)
        self.spin_lookahead.setValue(250)
        self.spin_lookahead.setSuffix(" mm")

        self.spin_kp_cross = QDoubleSpinBox()
        self.spin_kp_cross.setRange(0, 20)
        self.spin_kp_cross.setValue(1.5)

        self.spin_max_vel = QDoubleSpinBox()
        self.spin_max_vel.setRange(100, 2000)
        self.spin_max_vel.setValue(800)
        self.spin_max_vel.setSuffix(" mm/s")

        param_layout.addRow("前瞻距离 (Lookahead):", self.spin_lookahead)
        param_layout.addRow("横向纠偏 Kp:", self.spin_kp_cross)
        param_layout.addRow("最大巡航速度:", self.spin_max_vel)

        main_layout.addWidget(param_group)

        # 3. Action Execution Buttons
        btn_exec_layout = QVBoxLayout()
        self.btn_send_path = QPushButton("🚀 一键发送路径至小车 (Send Path)")
        self.btn_send_path.setStyleSheet("font-weight: bold; background-color: #0078d7; color: white; height: 36px;")
        self.btn_send_path.clicked.connect(self._on_send_path)

        self.btn_stop = QPushButton("🛑 停止/急停 (Stop)")
        self.btn_stop.setStyleSheet("background-color: #d13438; color: white; height: 32px;")
        self.btn_stop.clicked.connect(self.session.emergency_stop)

        btn_exec_layout.addWidget(self.btn_send_path)
        btn_exec_layout.addWidget(self.btn_stop)
        main_layout.addLayout(btn_exec_layout)

        main_layout.addStretch()

    def refresh_table(self) -> None:
        self.table_wp.blockSignals(True)
        self.table_wp.setRowCount(len(self.waypoints))
        for i, wp in enumerate(self.waypoints):
            self.table_wp.setItem(i, 0, QTableWidgetItem(str(wp.x_mm)))
            self.table_wp.setItem(i, 1, QTableWidgetItem(str(wp.y_mm)))
            self.table_wp.setItem(i, 2, QTableWidgetItem(str(wp.yaw_deg)))
            self.table_wp.setItem(i, 3, QTableWidgetItem(str(wp.v_max_mm_s)))
        self.table_wp.blockSignals(False)

        # Notify map to redraw path
        self.waypoints_changed.emit(self.waypoints)

    def _on_add_point(self) -> None:
        new_wp = PathWaypoint(x_mm=1000.0, y_mm=1000.0, yaw_deg=0.0, v_max_mm_s=800.0)
        self.waypoints.append(new_wp)
        self.refresh_table()

    def _on_del_point(self) -> None:
        row = self.table_wp.currentRow()
        if 0 <= row < len(self.waypoints):
            if len(self.waypoints) <= 2:
                QMessageBox.warning(self, "提示", "至少保留 2 个路径点！")
                return
            self.waypoints.pop(row)
            self.refresh_table()

    def _on_cell_changed(self, row: int, col: int) -> None:
        if 0 <= row < len(self.waypoints):
            try:
                val = float(self.table_wp.item(row, col).text())
                if col == 0:
                    self.waypoints[row].x_mm = val
                elif col == 1:
                    self.waypoints[row].y_mm = val
                elif col == 2:
                    self.waypoints[row].yaw_deg = val
                elif col == 3:
                    self.waypoints[row].v_max_mm_s = val
                self.waypoints_changed.emit(self.waypoints)
            except ValueError:
                pass

    def on_map_clicked_add_point(self, x_mm: float, y_mm: float) -> None:
        """Called when user clicks on right-side map canvas to add/update point."""
        new_wp = PathWaypoint(x_mm=x_mm, y_mm=y_mm, yaw_deg=0.0, v_max_mm_s=800.0)
        self.waypoints.append(new_wp)
        self.refresh_table()

    def _on_send_path(self) -> None:
        if not self.waypoints:
            QMessageBox.warning(self, "提示", "路径不能为空！")
            return
        self.session.send_path(self.waypoints)
        self.session.status_message.emit(f"已下发 {len(self.waypoints)} 个路径点到小车！")

    def set_waypoints(self, waypoints: List[PathWaypoint]) -> None:
        self.waypoints = list(waypoints)
        self.refresh_table()

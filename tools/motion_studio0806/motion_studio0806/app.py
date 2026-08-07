"""Main application entry point for Motion Studio 0806."""

from __future__ import annotations

import sys
from typing import Optional

from PySide6.QtCore import Qt
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QApplication, QComboBox, QHBoxLayout, QLabel,
    QMainWindow, QPushButton, QStatusBar, QStackedWidget, QTabWidget,
    QVBoxLayout, QWidget, QSplitter, QButtonGroup
)
import serial
import serial.tools.list_ports

from motion_studio0806.core.session import StudioSession
from motion_studio0806.ui.tab_single_point import SinglePointTab
from motion_studio0806.ui.tab_path_follow import PathFollowTab
from motion_studio0806.ui.tab_map_planner import MapPlannerTab
from motion_studio0806.ui.map_view import CleanMapView
from motion_studio0806.ui.plots_view import CleanPlotsView


class MotionStudioWindow(QMainWindow):
    """Main window of Motion Studio 0806."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("LittleCar2 Motion Studio 0806 - 底盘运动调试工作台")
        self.resize(1360, 850)

        self.session = StudioSession(self)
        self._init_ui()
        self._refresh_serial_ports()

        # Route telemetry to right-side active display views
        self.session.telemetry_updated.connect(self.map_view.update_telemetry)
        self.session.raw_telemetry_updated.connect(self.plots_view.update_telemetry)

    def _init_ui(self) -> None:
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # Global Header Panel
        header_widget = QWidget()
        header_widget.setStyleSheet("background-color: #252526; color: white; padding: 6px;")
        header_layout = QHBoxLayout(header_widget)

        # Title Label
        title_label = QLabel("Motion Studio 0806")
        title_font = QFont("Segoe UI", 12, QFont.Bold)
        title_label.setFont(title_font)
        header_layout.addWidget(title_label)
        header_layout.addSpacing(15)

        # Serial Select
        header_layout.addWidget(QLabel("串口 (Port):"))
        self.combo_ports = QComboBox()
        self.combo_ports.setMinimumWidth(120)
        header_layout.addWidget(self.combo_ports)

        btn_refresh = QPushButton("刷新")
        btn_refresh.clicked.connect(self._refresh_serial_ports)
        header_layout.addWidget(btn_refresh)

        # Baudrate
        header_layout.addWidget(QLabel("波特率:"))
        self.combo_baud = QComboBox()
        self.combo_baud.addItems(["115200", "921600", "460800", "57600"])
        header_layout.addWidget(self.combo_baud)

        # Connect Button
        self.btn_connect = QPushButton("连接串口 (Connect)")
        self.btn_connect.setStyleSheet("background-color: #107c41; color: white; font-weight: bold;")
        self.btn_connect.clicked.connect(self._toggle_connection)
        header_layout.addWidget(self.btn_connect)

        header_layout.addStretch()

        # Emergency Stop Button
        self.btn_estop = QPushButton("🚨 STOP 机械急停")
        self.btn_estop.setStyleSheet("background-color: #e81123; color: white; font-weight: bold; font-size: 13px; padding: 6px 14px;")
        self.btn_estop.clicked.connect(self.session.emergency_stop)
        header_layout.addWidget(self.btn_estop)

        main_layout.addWidget(header_widget)

        # Main Splitter (Left Controls vs Right Display Views)
        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter)

        # Left Column: 3 Dedicated Control Tabs
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet("QTabBar::tab { font-size: 12px; padding: 7px 12px; }")

        self.tab_single = SinglePointTab(self.session)
        self.tab_path = PathFollowTab(self.session)
        self.tab_map = MapPlannerTab(self.session)

        # Display Stack (Plots View vs Map View)
        self.display_stack = QStackedWidget()
        self.plots_view = CleanPlotsView()
        self.map_view = CleanMapView()

        # Full Bidirectional Signal Routing Hub
        self.tab_single.target_changed.connect(self.map_view.set_single_target)
        self.tab_path.waypoints_changed.connect(self.map_view.set_waypoints)
        self.tab_map.config_changed.connect(self.map_view.update_map_config)
        self.map_view.point_added.connect(self.tab_path.on_map_clicked_add_point)

        # Initial sync
        self.map_view.set_waypoints(self.tab_path.waypoints)

        self.display_stack.addWidget(self.plots_view)
        self.display_stack.addWidget(self.map_view)

        self.tabs.addTab(self.tab_single, "📌 单点/定点")
        self.tabs.addTab(self.tab_path, "🛣️ 路径测试与制作 (A*)")
        self.tabs.addTab(self.tab_map, "⚙️ 地图配置")

        splitter.addWidget(self.tabs)

        # Right Column: View Mode Switcher + Display Stack
        right_container = QWidget()
        right_layout = QVBoxLayout(right_container)
        right_layout.setContentsMargins(0, 0, 0, 0)

        # Top View Switcher Toolbar
        view_bar = QHBoxLayout()
        view_bar.setContentsMargins(4, 4, 4, 4)

        lbl_view = QLabel("右侧主显示视图 (Right View):")
        lbl_view.setStyleSheet("font-weight: bold;")
        view_bar.addWidget(lbl_view)

        self.btn_group_view = QButtonGroup(self)
        self.btn_show_plots = QPushButton("📊 遥测波形图表 (Telemetry Waveforms)")
        self.btn_show_plots.setCheckable(True)
        self.btn_show_plots.setChecked(True)
        self.btn_show_plots.setStyleSheet("padding: 5px 12px; font-weight: bold;")

        self.btn_show_map = QPushButton("🗺️ 2D 赛场地图 (2D Field Map)")
        self.btn_show_map.setCheckable(True)
        self.btn_show_map.setStyleSheet("padding: 5px 12px; font-weight: bold;")

        self.btn_group_view.addButton(self.btn_show_plots, 0)
        self.btn_group_view.addButton(self.btn_show_map, 1)
        self.btn_group_view.idClicked.connect(self._switch_right_view)

        view_bar.addWidget(self.btn_show_plots)
        view_bar.addWidget(self.btn_show_map)
        view_bar.addStretch()

        right_layout.addLayout(view_bar)
        right_layout.addWidget(self.display_stack)
        splitter.addWidget(right_container)

        splitter.setSizes([450, 910])

        # Status Bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.session.status_message.connect(self.status_bar.showMessage)
        self.session.connection_changed.connect(self._on_connection_changed)

        self.status_bar.showMessage("就绪 (Ready)")

    def _switch_right_view(self, view_id: int) -> None:
        self.display_stack.setCurrentIndex(view_id)

    def _refresh_serial_ports(self) -> None:
        self.combo_ports.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.combo_ports.addItem(f"{p.device} ({p.description})", p.device)
        if not ports:
            self.combo_ports.addItem("无可用串口", "")

    def _toggle_connection(self) -> None:
        if self.session.connected:
            self.session.disconnect_serial()
        else:
            port = self.combo_ports.currentData()
            baud = int(self.combo_baud.currentText())
            if port:
                self.session.connect_serial(port, baud)

    def _on_connection_changed(self, connected: bool, port: str) -> None:
        if connected:
            self.btn_connect.setText(f"断开 {port} (Disconnect)")
            self.btn_connect.setStyleSheet("background-color: #d13438; color: white; font-weight: bold;")
        else:
            self.btn_connect.setText("连接串口 (Connect)")
            self.btn_connect.setStyleSheet("background-color: #107c41; color: white; font-weight: bold;")

    def _on_path_from_map(self, waypoints: list) -> None:
        self.tab_path.set_waypoints(waypoints)
        self.map_view.set_waypoints(waypoints)
        self.tabs.setCurrentWidget(self.tab_path)


def main() -> None:
    app = QApplication(sys.argv)
    window = MotionStudioWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

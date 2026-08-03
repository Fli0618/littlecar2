"""Desktop entry point for the unified chassis motion workbench."""

from __future__ import annotations

import sys

from PySide6.QtCore import QTimer, Qt, Signal
from PySide6.QtWidgets import (QApplication, QComboBox, QFormLayout, QHBoxLayout, QLabel, QMainWindow, QPushButton,
                               QScrollArea, QSplitter, QStackedWidget, QTabWidget, QVBoxLayout, QWidget)

from map_planner.gui import MapEditorWidget
from map_planner.models import ContinuousPathSegment, Pose
from pid_tuner.gui.plots import TelemetryPlots
from pid_tuner.gui.widgets import PidControlPanel

from .control_panel import PointControlPanel
from .controller import MotionWorkbenchController
from .models import TargetPose


class PathControlPanel(QWidget):
    """Small path command surface; serialization remains in the controller layer."""

    upload_requested = Signal()
    start_requested = Signal()
    abort_requested = Signal()

    def __init__(self) -> None:
        super().__init__()
        self.source = QComboBox(); self.source.addItems(["当前连续路径", "当前贝塞尔路径"])
        self.upload = QPushButton("上传路径")
        self.start = QPushButton("启动路径")
        self.abort = QPushButton("中止路径")
        self.status = QLabel("未上传")
        form = QFormLayout(self)
        form.addRow("路径来源", self.source); form.addRow(self.upload); form.addRow(self.start); form.addRow(self.abort); form.addRow("状态", self.status)
        self.upload.clicked.connect(self.upload_requested)
        self.start.clicked.connect(self.start_requested)
        self.abort.clicked.connect(self.abort_requested)


class MotionWorkbenchWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("底盘运动调试工作台")
        self.resize(1500, 920)
        self.controller = MotionWorkbenchController()
        self._build()
        self._wire()
        self.refresh_timer = QTimer(self); self.refresh_timer.setInterval(40); self.refresh_timer.timeout.connect(self._refresh); self.refresh_timer.start()
        self.heartbeat_timer = QTimer(self); self.heartbeat_timer.setInterval(500); self.heartbeat_timer.timeout.connect(self.controller.session.heartbeat); self.heartbeat_timer.start()

    def _build(self) -> None:
        central = QWidget(); root = QVBoxLayout(central); root.setContentsMargins(8, 8, 8, 8)
        status_row = QHBoxLayout()
        self.connection = QLabel("未连接")
        self.pose_status = QLabel("位姿: 等待遥测")
        self.motion_status = QLabel("运动: NO_TARGET")
        self.upload_status = QLabel("路径: 未上传")
        self.stop = QPushButton("STOP"); self.stop.setObjectName("stopButton")
        for widget in (self.connection, self.pose_status, self.motion_status, self.upload_status): status_row.addWidget(widget)
        status_row.addStretch(); status_row.addWidget(self.stop)
        root.addLayout(status_row)

        self.pid_panel = PidControlPanel()
        self.point_panel = PointControlPanel()
        self.path_panel = PathControlPanel()
        tabs = QTabWidget(); tabs.addTab(self.pid_panel, "PID"); tabs.addTab(self.point_panel, "单点"); tabs.addTab(self.path_panel, "路径")
        left_scroll = QScrollArea(); left_scroll.setWidgetResizable(True); left_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff); left_scroll.setWidget(tabs); left_scroll.setMinimumWidth(320)

        self.plots = TelemetryPlots()
        plots_scroll = QScrollArea(); plots_scroll.setWidget(self.plots); plots_scroll.setWidgetResizable(False); plots_scroll.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop)
        self.map_editor = MapEditorWidget()
        self.workspace = QStackedWidget(); self.workspace.addWidget(plots_scroll); self.workspace.addWidget(self.map_editor)
        self.view_switch = QPushButton("切换至地图")
        workspace_container = QWidget(); workspace_layout = QVBoxLayout(workspace_container); workspace_layout.setContentsMargins(0, 0, 0, 0); workspace_layout.addWidget(self.view_switch); workspace_layout.addWidget(self.workspace)
        splitter = QSplitter(Qt.Orientation.Horizontal); splitter.setChildrenCollapsible(False); splitter.addWidget(left_scroll); splitter.addWidget(workspace_container); splitter.setStretchFactor(1, 1); splitter.setSizes([340, 1160])
        root.addWidget(splitter)
        self.setCentralWidget(central)

    def _wire(self) -> None:
        self.stop.clicked.connect(self.controller.stop)
        self.point_panel.candidate_edited.connect(self.controller.select_candidate)
        self.point_panel.send_requested.connect(self.controller.start_goal)
        self.point_panel.stop_requested.connect(self.controller.stop)
        self.point_panel.clear_requested.connect(self.controller.clear_candidate)
        self.controller.candidate_changed.connect(self.point_panel.set_candidate)
        self.controller.actual_pose_changed.connect(self._set_actual_pose)
        self.controller.motion_state_changed.connect(lambda value: self.motion_status.setText(f"运动: {value}"))
        self.controller.status_changed.connect(self.connection.setText)
        self.pid_panel.read_requested.connect(self.controller.session.read_pid)
        self.pid_panel.apply_requested.connect(self.controller.session.apply_pid)
        self.pid_panel.restore_requested.connect(self.controller.session.restore_pid)
        self.controller.session.pid_read.connect(lambda _revision, pid: self.pid_panel.set_pid(pid))
        self.controller.session.pid_applied.connect(lambda _revision, pid: self.pid_panel.set_pid(pid))
        self.controller.session.path_upload_changed.connect(self.path_panel.status.setText)
        self.path_panel.upload_requested.connect(self._upload_selected_path)
        self.path_panel.start_requested.connect(lambda: self.controller.start_path(self._path_id))
        self.path_panel.abort_requested.connect(self.controller.abort_path)
        self.view_switch.clicked.connect(self._switch_workspace)
        self._path_id = 1

    def _set_actual_pose(self, target: TargetPose, valid: bool) -> None:
        self.pose_status.setText("位姿: 有效" if valid else "位姿: 无效")
        self.map_editor.set_runtime_pose(Pose(target.x_mm, target.y_mm, target.yaw_deg) if valid else None)

    def _switch_workspace(self) -> None:
        next_index = 1 - self.workspace.currentIndex(); self.workspace.setCurrentIndex(next_index)
        self.view_switch.setText("切换至图表" if next_index else "切换至地图")

    def _refresh(self) -> None:
        if self.workspace.currentIndex() == 0:
            self.plots.refresh(self.controller.buffer)

    def _upload_selected_path(self) -> None:
        plan = self.map_editor.get_plan()
        for step in plan.steps:
            if isinstance(step, ContinuousPathSegment):
                self.controller.upload_path(self._path_id, step.points)
                self.path_panel.status.setText(f"上传路径 {self._path_id}")
                return
        self.path_panel.status.setText("当前方案没有连续路径")

    def closeEvent(self, event: object) -> None:
        self.controller.session.shutdown()
        event.accept()  # type: ignore[attr-defined]


def main() -> int:
    app = QApplication(sys.argv)
    app.setStyleSheet("QWidget{background:#171b21;color:#e6edf3;} QLineEdit,QComboBox,QSpinBox,QDoubleSpinBox{background:#242b34;padding:4px;} QPushButton{background:#365169;padding:6px;} QPushButton#stopButton{background:#c83b3b;color:white;font-weight:bold;}")
    window = MotionWorkbenchWindow(); window.show()
    return app.exec()

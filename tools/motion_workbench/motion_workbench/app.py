"""Desktop entry point for the unified chassis motion workbench."""

from __future__ import annotations

import sys
from pathlib import Path

from PySide6.QtCore import QTimer, Qt, Signal
from PySide6.QtGui import QFontDatabase
from PySide6.QtWidgets import (QApplication, QComboBox, QDialog, QFileDialog, QFormLayout, QHBoxLayout, QLabel,
                               QMainWindow, QPlainTextEdit, QPushButton, QDoubleSpinBox, QGroupBox, QScrollArea,
                               QSplitter, QStackedWidget, QTabWidget, QVBoxLayout, QWidget)

from map_planner.gui import MapEditorWidget
from pid_tuner.gui.plots import TelemetryPlots
from pid_tuner.gui.widgets import ConnectionPanel
from pid_tuner.models import (GotoStrategySnapshot, HolonomicConfigState, MotionGoal,
                               PathConfigState, PathControlConfig, PidConfigState, Telemetry)
from pid_tuner.storage import export_motion_config_header

from .control_panel import (
    HEADING_MODE_NONE,
    HolonomicControlPanel,
    PointControlPanel,
    WorkbenchPidControlPanel,
    protected_number,
)
from .controller import MotionWorkbenchController
from .models import CoordinateSyncState, PathUploadSnapshot, TargetPose

MOTION_WORKBENCH_REFRESH_MS = 40


class PathControlPanel(QWidget):
    """Continuous-path parameter editor; execution lives in the realtime map page."""

    read_config_requested = Signal()
    apply_config_requested = Signal(object)
    restore_config_requested = Signal()

    def __init__(self) -> None:
        super().__init__()
        self._pending_config_action: str | None = None
        layout = QVBoxLayout(self)
        hint = QLabel("本页只管理路径控制参数；实机执行统一在“3 实时运行”中操作。")
        hint.setWordWrap(True)
        layout.addWidget(hint)
        self.config_inputs: dict[str, QDoubleSpinBox] = {}
        groups = (
            ("路径 PD", (
                ("kp_cross_track", "横向 Kp", 0.98, 0.0, 20.0, 0.01),
                ("kd_cross_track_velocity", "横向速度 Kd", 0.62, 0.0, 20.0, 0.01),
                ("kp_yaw", "航向 Kp", 1.42, 0.0, 20.0, 0.01),
                ("kd_yaw_rate", "航向角速度 Kd", 0.427, 0.0, 20.0, 0.01),
            )),
            ("速度规划", (
                ("cruise_speed_mm_s", "巡航速度 mm/s", 600.0, 1.0, 1500.0, 10.0),
                ("max_yaw_rate_deg_s", "最大角速度 deg/s", 100.0, 1.0, 180.0, 5.0),
                ("accel_mm_s2", "加速度 mm/s²", 450.0, 1.0, 5000.0, 10.0),
                ("decel_mm_s2", "减速度 mm/s²", 650.0, 1.0, 5000.0, 10.0),
                ("max_lateral_accel_mm_s2", "横向加速度 mm/s²", 240.0, 1.0, 5000.0, 10.0),
                ("hardware_acc", "硬件 ACC档位", 2.0, 0.0, 255.0, 1.0),
            )),
            ("曲率前馈", (
                ("curvature_preview_mm", "曲率预览 mm", 450.0, 1.0, 2000.0, 5.0),
                ("curvature_ff_time_s", "曲率前馈等效时间 s", 0.05, 0.0, 2.0, 0.01),
            )),
            ("前视规划", (
                ("lookahead_min_mm", "最小前视 mm", 90.0, 1.0, 1000.0, 5.0),
                ("lookahead_base_mm", "基础前视 mm", 90.0, 1.0, 1000.0, 5.0),
                ("lookahead_speed_gain_s", "速度增益 s", 0.15, 0.0, 2.0, 0.01),
                ("lookahead_curve_gain_mm", "曲率增益 mm", 120.0, 0.0, 1000.0, 5.0),
                ("lookahead_max_mm", "最大前视 mm", 220.0, 1.0, 1000.0, 5.0),
                ("lookahead_rate_mm_s", "前视变化率 mm/s", 400.0, 1.0, 2000.0, 10.0),
                ("initial_lookahead_mm", "初始前视 mm", 90.0, 1.0, 1000.0, 5.0),
            )),
            ("末段捕获", (
                ("final_capture_distance_mm", "捕获距离 mm", 80.0, 0.0, 2000.0, 5.0),
                ("final_capture_speed_mm_s", "捕获速度 mm/s", 50.0, 0.0, 1500.0, 5.0),
            )),
        )
        for title, fields in groups:
            group = QGroupBox(title); form = QFormLayout(group)
            for name, label, value, minimum, maximum, step in fields:
                box = protected_number(value, minimum, maximum, step, decimals=3)
                self.config_inputs[name] = box; form.addRow(label, box)
            layout.addWidget(group)
        buttons = QHBoxLayout()
        self.read_config = QPushButton("读取参数")
        self.apply_config = QPushButton("应用参数")
        self.restore_config = QPushButton("恢复默认")
        self.config_status = QLabel("路径参数：未读取")
        self.config_status.setWordWrap(True)
        self._set_config_status("路径参数：未读取", "neutral")
        layout.addWidget(self.config_status)
        for button in (self.read_config, self.apply_config, self.restore_config): buttons.addWidget(button)
        layout.addLayout(buttons)
        layout.addStretch()
        self.read_config.clicked.connect(self._emit_read_config)
        self.apply_config.clicked.connect(self._emit_config)
        self.restore_config.clicked.connect(self._emit_restore_config)

    def current_config(self) -> PathControlConfig:
        values = {name: box.value() for name, box in self.config_inputs.items()}
        if not values["lookahead_min_mm"] <= values["lookahead_base_mm"] <= values["lookahead_max_mm"]:
            raise ValueError("前视距离必须满足：最小 ≤ 基础 ≤ 最大")
        return PathControlConfig(**values)

    def set_config(self, revision: int, config: PathControlConfig) -> None:
        """Compatibility entry point for a normal parameter read."""
        self.set_read_config(revision, config)

    def set_read_config(self, revision: int, config: PathControlConfig) -> None:
        for name, value in config.to_dict().items():
            self.config_inputs[name].setValue(value)
        action = "恢复默认成功" if self._pending_config_action == "restore" else "读取成功"
        self._pending_config_action = None
        self._set_config_status(f"✓ {action}｜下位机修订号：{revision}", "success")

    def set_applied_config(self, revision: int, config: PathControlConfig) -> None:
        for name, value in config.to_dict().items():
            self.config_inputs[name].setValue(value)
        self._pending_config_action = None
        self._set_config_status(f"✓ 应用成功｜下位机修订号：{revision}", "success")

    def show_operation_error(self, message: str) -> None:
        if self._pending_config_action is None:
            return
        self._pending_config_action = None
        self._set_config_status(f"✗ 操作失败：{message}", "error")

    def _set_config_status(self, text: str, state: str) -> None:
        colors = {
            "neutral": ("#263238", "#b0bec5"),
            "pending": ("#4e342e", "#ffb74d"),
            "success": ("#1b5e20", "#a5d6a7"),
            "error": ("#7f1d1d", "#ffcdd2"),
        }
        background, foreground = colors[state]
        self.config_status.setText(text)
        self.config_status.setStyleSheet(
            f"padding: 8px; border-radius: 4px; background: {background}; color: {foreground}; font-weight: 700;")

    def _emit_read_config(self) -> None:
        self._pending_config_action = "read"
        self._set_config_status("正在读取下位机参数…", "pending")
        self.read_config_requested.emit()

    def _emit_restore_config(self) -> None:
        self._pending_config_action = "restore"
        self._set_config_status("正在恢复下位机默认参数…", "pending")
        self.restore_config_requested.emit()

    def _emit_config(self) -> None:
        try:
            config = self.current_config()
        except ValueError as error:
            self._set_config_status(f"✗ 参数无效：{error}", "error"); return
        self._pending_config_action = "apply"
        self._set_config_status("正在发送，等待 STM32 确认…", "pending")
        self.apply_config_requested.emit(config)


class MotionConfigExportDialog(QDialog):
    """Preview and save one immutable motion configuration header export."""

    def __init__(self, header_text: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._header_text = header_text
        self.setWindowTitle("导出固化参数")
        self.resize(840, 680)
        layout = QVBoxLayout(self)
        self.editor = QPlainTextEdit()
        self.editor.setReadOnly(True)
        self.editor.setPlainText(header_text)
        self.editor.setFont(QFontDatabase.systemFont(QFontDatabase.SystemFont.FixedFont))
        layout.addWidget(self.editor)
        self.status = QLabel()
        layout.addWidget(self.status)
        buttons = QHBoxLayout()
        self.copy_all = QPushButton("复制全部")
        self.save_file = QPushButton("保存文件")
        self.close_button = QPushButton("关闭")
        for button in (self.copy_all, self.save_file):
            buttons.addWidget(button)
        buttons.addStretch()
        buttons.addWidget(self.close_button)
        layout.addLayout(buttons)
        self.copy_all.clicked.connect(self._copy_all)
        self.save_file.clicked.connect(self._choose_save_path)
        self.close_button.clicked.connect(self.accept)

    def _copy_all(self) -> None:
        QApplication.clipboard().setText(self._header_text)
        self.status.setText("已复制 advance_motion_config.h")

    def _choose_save_path(self) -> None:
        filename, _ = QFileDialog.getSaveFileName(
            self, "保存固化参数", "advance_motion_config.h", "C Header Files (*.h);;All Files (*)")
        if filename:
            self._save_to_path(Path(filename))

    def _save_to_path(self, path: Path) -> bool:
        try:
            with path.open("w", encoding="utf-8", newline="\n") as handle:
                handle.write(self._header_text)
        except OSError as error:
            self.status.setText(f"保存失败：{error}")
            return False
        self.status.setText(f"已保存 {path.name}")
        return True


class MotionWorkbenchWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("底盘运动调试工作台")
        self.resize(1500, 920)
        self.controller = MotionWorkbenchController()
        self._active_heading_mode = "WIT"
        self._active_pid_state: PidConfigState | None = None
        self._active_path_state: PathConfigState | None = None
        self._active_goto_strategy: GotoStrategySnapshot | None = None
        self._active_holonomic_state: HolonomicConfigState | None = None
        self._build()
        self._wire()
        self.refresh_timer = QTimer(self); self.refresh_timer.setInterval(MOTION_WORKBENCH_REFRESH_MS); self.refresh_timer.timeout.connect(self._refresh); self.refresh_timer.start()
        self.heartbeat_timer = QTimer(self); self.heartbeat_timer.setInterval(500); self.heartbeat_timer.timeout.connect(self.controller.session.heartbeat); self.heartbeat_timer.start()

    def _build(self) -> None:
        central = QWidget(); root = QVBoxLayout(central); root.setContentsMargins(8, 8, 8, 8)
        status_row = QHBoxLayout()
        self.connection = QLabel("未连接")
        self.pose_status = QLabel("位姿: 等待遥测")
        self.motion_status = QLabel("运动: NO_TARGET")
        self.controller_status = QLabel("控制器: 经典位置 PID")
        self.upload_status = QLabel("路径: 未上传")
        self.heading_status = QLabel("航向控制: WIT")
        self.map_start_status = QLabel("地图起点: 未标定")
        self.board_origin_status = QLabel("板端原点: 未知")
        self.coordinate_sync_status = QLabel("坐标同步: 未就绪")
        self.reset_origin_button = QPushButton("重置零点")
        self.return_origin_button = QPushButton("返回零点")
        self.reset_origin_button.setEnabled(False)
        self.return_origin_button.setEnabled(False)
        self.stop = QPushButton("STOP"); self.stop.setObjectName("stopButton")
        self.export_motion_config = QPushButton("导出固化参数")
        self.export_motion_config.setToolTip("导出 STM32 当前已生效的 PID、路径、GOTO 和全向参数")
        self.export_motion_config.setEnabled(False)
        for widget in (self.connection, self.pose_status, self.motion_status,
                       self.controller_status, self.upload_status, self.heading_status, self.map_start_status,
                       self.board_origin_status, self.coordinate_sync_status):
            status_row.addWidget(widget)
        status_row.addStretch()
        status_row.addWidget(self.reset_origin_button)
        status_row.addWidget(self.return_origin_button)
        status_row.addWidget(self.export_motion_config)
        status_row.addWidget(self.stop)
        root.addLayout(status_row)

        self.pid_panel = WorkbenchPidControlPanel()
        self.holonomic_panel = HolonomicControlPanel()
        self.connection_panel = ConnectionPanel()
        self.point_panel = PointControlPanel()
        self.path_panel = PathControlPanel()
        self.tabs = QTabWidget()
        self.tabs.addTab(self.connection_panel, "连接")
        self.tabs.addTab(self.pid_panel, "PID")
        self.tabs.addTab(self.holonomic_panel, "全向位置")
        self.tabs.addTab(self.point_panel, "单点")
        self.tabs.addTab(self.path_panel, "路径")
        left_scroll = QScrollArea(); left_scroll.setWidgetResizable(True); left_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded); left_scroll.setWidget(self.tabs); left_scroll.setMinimumWidth(320)

        self.plots = TelemetryPlots()
        plots_scroll = QScrollArea(); plots_scroll.setWidget(self.plots); plots_scroll.setWidgetResizable(False); plots_scroll.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop)
        self.map_editor = MapEditorWidget()
        self.workspace = QStackedWidget(); self.workspace.addWidget(plots_scroll); self.workspace.addWidget(self.map_editor)
        self.view_switch = QPushButton("切换至地图")
        workspace_container = QWidget(); workspace_layout = QVBoxLayout(workspace_container); workspace_layout.setContentsMargins(0, 0, 0, 0); workspace_layout.addWidget(self.view_switch); workspace_layout.addWidget(self.workspace)
        splitter = QSplitter(Qt.Orientation.Horizontal); self.main_splitter = splitter
        splitter.setChildrenCollapsible(False); splitter.setHandleWidth(8); splitter.setOpaqueResize(True)
        splitter.setStyleSheet("QSplitter::handle:horizontal{background:#455a64;margin:1px 2px;border-radius:2px;} QSplitter::handle:horizontal:hover{background:#29b6f6;}")
        splitter.addWidget(left_scroll); splitter.addWidget(workspace_container)
        splitter.setStretchFactor(0, 0); splitter.setStretchFactor(1, 1); splitter.setSizes([340, 1160])
        root.addWidget(splitter)
        self.setCentralWidget(central)

    def _wire(self) -> None:
        self.stop.clicked.connect(self.controller.stop)
        self.export_motion_config.clicked.connect(self._export_motion_config)
        self.reset_origin_button.clicked.connect(self._request_origin_reset)
        self.return_origin_button.clicked.connect(self._return_to_origin)
        self.connection_panel.refresh_ports_requested.connect(self._refresh_ports)
        self.connection_panel.connect_requested.connect(self._connect_port)
        self.connection_panel.disconnect_requested.connect(self._disconnect_port)
        self.point_panel.candidate_edited.connect(self.controller.select_candidate)
        self.point_panel.send_requested.connect(self._start_point_goal)
        self.point_panel.heading_mode_requested.connect(self._request_heading_mode)
        self.point_panel.controller_changed.connect(self._on_controller_changed)
        self.point_panel.stop_requested.connect(self.controller.stop)
        self.point_panel.clear_requested.connect(self.controller.clear_candidate)
        self.controller.candidate_changed.connect(self.point_panel.set_candidate)
        self.controller.upload_changed.connect(self._set_upload_status)
        self.controller.coordinate_sync_changed.connect(self._set_coordinate_sync_status)
        self.controller.plan_execution_changed.connect(self._set_plan_execution_status)
        self.controller.plan_finished.connect(self._set_plan_execution_status)
        self.controller.motion_state_changed.connect(lambda value: self.motion_status.setText(f"运动: {value}"))
        self.controller.status_changed.connect(self._on_session_status)
        self.controller.session.connection_failed.connect(self._on_connection_failure)
        self.controller.session.connection_changed.connect(self._on_connection_changed)
        self.pid_panel.read_requested.connect(self.controller.session.read_pid)
        self.pid_panel.apply_requested.connect(self.controller.session.apply_pid)
        self.pid_panel.restore_requested.connect(self.controller.session.restore_pid)
        self.controller.session.pid_read.connect(self.pid_panel.set_pid_state)
        self.controller.session.pid_read.connect(self._cache_pid_state)
        self.controller.session.pid_applied.connect(self.pid_panel.set_pid_state)
        self.controller.session.pid_applied.connect(self._cache_pid_state)
        self.controller.session.connection_changed.connect(self.pid_panel.set_connected)
        self.controller.session.connection_changed.connect(self.holonomic_panel.set_connected)
        self.holonomic_panel.read_requested.connect(self.controller.session.read_holonomic_config)
        self.holonomic_panel.apply_requested.connect(self.controller.session.apply_holonomic_config)
        self.holonomic_panel.apply_requested.connect(lambda _config: self.holonomic_panel.set_applying())
        self.holonomic_panel.restore_requested.connect(self.controller.session.restore_holonomic_config)
        self.holonomic_panel.restore_requested.connect(self.holonomic_panel.set_applying)
        self.controller.session.holonomic_config_read.connect(self.holonomic_panel.set_config)
        self.controller.session.holonomic_config_read.connect(self._cache_holonomic_state)
        self.controller.session.holonomic_config_applied.connect(self.holonomic_panel.set_applied)
        self.controller.session.holonomic_config_applied.connect(self._cache_holonomic_state)
        self.controller.session.holonomic_unsupported.connect(
            lambda: self.holonomic_panel.set_unsupported(True))
        self.pid_panel.goto_strategy_changed.connect(self.controller.session.set_goto_strategy)
        self.controller.session.goto_strategy_read.connect(
            lambda strategy: self.pid_panel.set_goto_strategy(strategy.large_yaw_align_enabled))
        self.controller.session.goto_strategy_read.connect(self._cache_goto_strategy)
        self.controller.session.goto_strategy_changed.connect(
            lambda strategy: self.pid_panel.set_goto_strategy(strategy.large_yaw_align_enabled))
        self.controller.session.goto_strategy_changed.connect(self._cache_goto_strategy)
        self.controller.session.yaw_source_changed.connect(self._on_yaw_source_changed)
        self.controller.session.motion_changed.connect(self.point_panel.set_motion_active)
        self.controller.session.motion_changed.connect(self.pid_panel.set_motion_active)
        self.controller.session.motion_changed.connect(self.map_editor.set_hardware_motion_active)
        self.controller.session.motion_changed.connect(
            lambda _active: self._refresh_origin_controls())
        self.controller.session.origin_reset.connect(self._on_origin_reset)
        self.controller.session.telemetry.connect(self._sync_heading_source)
        self.path_panel.read_config_requested.connect(self.controller.session.read_path_config)
        self.path_panel.apply_config_requested.connect(self.controller.session.apply_path_config)
        self.path_panel.restore_config_requested.connect(self.controller.session.restore_path_config)
        self.controller.session.path_config_read.connect(
            lambda state: self.path_panel.set_read_config(state.revision, state.config))
        self.controller.session.path_config_read.connect(self._cache_path_state)
        self.controller.session.path_config_applied.connect(
            lambda state: self.path_panel.set_applied_config(state.revision, state.config))
        self.controller.session.path_config_applied.connect(self._cache_path_state)
        self.controller.session.operation_failed.connect(self.path_panel.show_operation_error)
        self.map_editor.plan_changed.connect(self.controller.set_plan)
        self.map_editor.start_frame_changed.connect(lambda _frame: self.controller.invalidate_coordinate_sync())
        self.map_editor.calibration_state_changed.connect(self.controller.set_map_calibrated)
        self.map_editor.candidate_selected.connect(self.controller.set_plan_cursor)
        self.map_editor.hardware_enabled_changed.connect(self._set_hardware_execution)
        self.map_editor.single_step_requested.connect(self.controller.start_single)
        self.map_editor.continuous_requested.connect(self.controller.start_continuous)
        self.map_editor.execution_stop_requested.connect(self.controller.stop)
        self.map_editor.execution_controller_changed.connect(
            self.controller.set_plan_point_controller)
        self.view_switch.clicked.connect(self._switch_workspace)
        self.controller.set_plan(self.map_editor.get_plan())
        self.controller.set_map_calibrated(not self.map_editor.calibration_pending)

    def _refresh_ports(self) -> None:
        from serial.tools import list_ports
        self.connection_panel.set_available_ports([item.device for item in list_ports.comports()])
        if not self.connection_panel.port.currentText():
            self.connection_panel.set_connected(False, "未发现可用 COM 口")

    def _connect_port(self, port: str, baud: int) -> None:
        self._clear_motion_config_sync()
        self.connection_panel.set_connecting(True)
        self.controller.session.connect_port(port, baud)

    def _disconnect_port(self) -> None:
        self.controller.session.disconnect()

    def _on_connection_changed(self, connected: bool) -> None:
        if not connected:
            self._clear_motion_config_sync()
            self._on_controller_changed("classic")
        self.connection_panel.set_connected(
            connected, "已连接，参数已同步" if connected else "已断开")
        self._refresh_origin_controls()

    def _on_session_status(self, status: str) -> None:
        self.connection.setText(status)
        self.connection_panel.set_connected(self.controller.session.connected, status)
        self._refresh_origin_controls()

    def _on_connection_failure(self, message: str) -> None:
        self._clear_motion_config_sync()
        self._refresh_origin_controls()
        self.connection_panel.set_connected(False, f"连接失败: {message}")

    def _clear_motion_config_sync(self) -> None:
        self._active_pid_state = None
        self._active_path_state = None
        self._active_goto_strategy = None
        self._active_holonomic_state = None
        self.holonomic_panel.set_unsupported(False)
        self._refresh_motion_config_export()

    def _cache_pid_state(self, state: PidConfigState) -> None:
        if self.controller.session.connected:
            self._active_pid_state = state
            self._refresh_motion_config_export()

    def _cache_path_state(self, state: PathConfigState) -> None:
        if self.controller.session.connected:
            self._active_path_state = state
            self._refresh_motion_config_export()

    def _cache_goto_strategy(self, strategy: GotoStrategySnapshot) -> None:
        if self.controller.session.connected:
            self._active_goto_strategy = strategy
            self._refresh_motion_config_export()

    def _cache_holonomic_state(self, state: HolonomicConfigState) -> None:
        if self.controller.session.connected:
            self.holonomic_panel.set_unsupported(False)
            self._active_holonomic_state = state
            self._refresh_motion_config_export()

    def _refresh_motion_config_export(self) -> None:
        self.export_motion_config.setEnabled(
            self.controller.session.connected
            and self._active_pid_state is not None
            and self._active_path_state is not None
            and self._active_goto_strategy is not None
            and self._active_holonomic_state is not None)

    def _export_motion_config(self) -> None:
        if not self.export_motion_config.isEnabled():
            return
        assert self._active_pid_state is not None
        assert self._active_path_state is not None
        assert self._active_goto_strategy is not None
        assert self._active_holonomic_state is not None
        try:
            header = export_motion_config_header(
                self._active_pid_state, self._active_path_state,
                self._active_goto_strategy, self._active_holonomic_state)
        except ValueError as error:
            self.connection.setText(f"导出失败：{error}")
            return
        MotionConfigExportDialog(header, self).exec()

    def _refresh_origin_controls(self) -> None:
        available = (self.controller.session.connected
                     and not self.controller.session.motion_active
                     and self.controller.coordinate_sync_state != CoordinateSyncState.RESET_PENDING)
        self.reset_origin_button.setEnabled(available)
        self.return_origin_button.setEnabled(available)

    def _request_origin_reset(self) -> None:
        self.controller.start_origin_reset()
        self._refresh_origin_controls()

    def _on_origin_reset(self) -> None:
        self.controller.confirm_origin_reset()
        self.controller.select_candidate(TargetPose(0.0, 0.0, 0.0))
        self.pose_status.setText("位姿: 零点已重置，等待遥测")
        self._refresh_origin_controls()

    def _return_to_origin(self) -> None:
        if not self.controller.session.connected or self.controller.session.motion_active:
            return
        uses_yaw = getattr(self.point_panel, "uses_yaw", None)
        use_yaw = (uses_yaw() if callable(uses_yaw)
                   else self.point_panel.use_yaw.isChecked())
        target = TargetPose(0.0, 0.0, 0.0)
        goal = MotionGoal(
            target.x_mm, target.y_mm, target.yaw_deg,
            self.point_panel.vmax.value(), self.point_panel.wmax.value(),
            self.point_panel.timeout.value(), use_yaw, True)
        self.controller.select_candidate(target)
        self.controller.start_goal(goal)

    def _request_heading_mode(self, mode: str) -> None:
        """联动单点 GOTO、板端航向源和图表角色标记。"""
        self._active_heading_mode = mode
        self.plots.set_heading_mode(mode)
        if mode == HEADING_MODE_NONE:
            self.heading_status.setText("航向控制: 关闭（WIT/OPS 仅观测）")
            return
        self.heading_status.setText(f"航向控制: {mode}")
        if self.controller.session.connected:
            self.controller.session.set_yaw_source(mode)

    def _on_yaw_source_changed(self, source: str) -> None:
        if self.point_panel.current_heading_mode() == HEADING_MODE_NONE:
            return
        self.point_panel.set_heading_mode(source)
        self._active_heading_mode = source
        self.plots.set_heading_mode(source)
        self.heading_status.setText(f"航向控制: {source}")

    def _sync_heading_source(self, telemetry: Telemetry) -> None:
        if self.point_panel.current_heading_mode() == HEADING_MODE_NONE:
            return
        if telemetry.yaw_source != self.point_panel.current_heading_mode():
            self.point_panel.set_heading_mode(telemetry.yaw_source)
            self._active_heading_mode = telemetry.yaw_source
            self.plots.set_heading_mode(telemetry.yaw_source)
            self.heading_status.setText(f"航向控制: {telemetry.yaw_source}")

    def _start_point_goal(self, goal: MotionGoal) -> None:
        controller = self.point_panel.current_controller()
        if not self.controller.start_goal(goal, controller):
            return
        self._active_heading_mode = (
            self.point_panel.current_heading_mode() if goal.use_yaw else HEADING_MODE_NONE
        )
        self.plots.set_heading_mode(self._active_heading_mode)
        if self._active_heading_mode == HEADING_MODE_NONE:
            self.heading_status.setText("航向控制: 关闭（WIT/OPS 仅观测）")
        else:
            self.heading_status.setText(f"航向控制: {self._active_heading_mode}")

    def _on_controller_changed(self, controller: str) -> None:
        self.controller_status.setText(
            "控制器: 全向位置" if controller == "holonomic" else "控制器: 经典位置 PID")

    def _set_hardware_execution(self, enabled: bool) -> None:
        if enabled and not self.controller.session.connected:
            self.map_editor.set_execution_enabled(False)
            self.map_editor.set_execution_status("请先连接串口后再启用实机运动")

    def _set_plan_execution_status(self, execution: object) -> None:
        state = getattr(execution, "state", "")
        cursor = getattr(execution, "cursor", 0)
        count = getattr(execution, "step_count", 0)
        reason = getattr(execution, "reason", "")
        self.map_editor.set_execution_status(f"{state} {cursor}/{count} {reason}".strip())

    def _switch_workspace(self) -> None:
        next_index = 1 - self.workspace.currentIndex(); self.workspace.setCurrentIndex(next_index)
        self.view_switch.setText("切换至图表" if next_index else "切换至地图")

    def _refresh(self) -> None:
        snapshot = self.controller.consume_runtime_ui_snapshot()
        self.plots.refresh(self.controller.buffer,
                           self.controller.holonomic_telemetry_buffer,
                           self.controller.path_telemetry_buffer)
        self.map_editor.apply_runtime_snapshot(snapshot)
        self.pose_status.setText("位姿: 有效" if snapshot.pose_valid else "位姿: 无效")

    def _set_upload_status(self, snapshot: PathUploadSnapshot) -> None:
        self.upload_status.setText(f"路径: {snapshot.state.value}")

    def _set_coordinate_sync_status(self, state: CoordinateSyncState) -> None:
        mapping = {
            CoordinateSyncState.MAP_UNCALIBRATED: ("地图起点: 未标定", "板端原点: 未知", "坐标同步: 未就绪"),
            CoordinateSyncState.BOARD_ORIGIN_UNKNOWN: ("地图起点: 已标定", "板端原点: 未知", "坐标同步: 等待重置"),
            CoordinateSyncState.RESET_PENDING: ("地图起点: 已标定", "板端原点: 重置中", "坐标同步: 等待重置"),
            CoordinateSyncState.WAITING_ZERO_TELEMETRY: ("地图起点: 已标定", "板端原点: 等待零点遥测", "坐标同步: 等待遥测"),
            CoordinateSyncState.SYNCED: ("地图起点: 已标定", "板端原点: 已建立", "坐标同步: 正常"),
            CoordinateSyncState.MISMATCH: ("地图起点: 已标定", "板端原点: 异常", "坐标同步: 不一致"),
        }
        map_text, board_text, sync_text = mapping[state]
        self.map_start_status.setText(map_text)
        self.board_origin_status.setText(board_text)
        self.coordinate_sync_status.setText(sync_text)
        self._refresh_origin_controls()

    def closeEvent(self, event: object) -> None:
        self.refresh_timer.stop()
        self.heartbeat_timer.stop()
        self.controller.session.shutdown()
        event.accept()  # type: ignore[attr-defined]


def main() -> int:
    app = QApplication(sys.argv)
    app.setStyleSheet("QWidget{background:#171b21;color:#e6edf3;} QLineEdit,QComboBox,QSpinBox,QDoubleSpinBox{background:#242b34;padding:4px;} QPushButton{background:#365169;padding:6px;} QPushButton#stopButton{background:#c83b3b;color:white;font-weight:bold;}")
    window = MotionWorkbenchWindow(); window.show()
    return app.exec()

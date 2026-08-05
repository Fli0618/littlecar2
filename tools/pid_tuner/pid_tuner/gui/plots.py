from __future__ import annotations

import pyqtgraph as pg
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (QComboBox, QHBoxLayout, QPushButton, QSizePolicy,
                               QStackedWidget, QVBoxLayout, QWidget)

from .buffer import TelemetryBuffer


HEADING_MODES = {"WIT", "OPS", "NONE"}


class TelemetryPlots(QWidget):
    """Display pose and per-axis diagnostic values on a shared time axis."""

    _LINEAR_DEFAULT_RANGE_MM = 500.0
    _SPEED_DEFAULT_RANGE_MM_S = 100.0
    _YAW_DEFAULT_RANGE_DEG = 180.0
    _MINIMUM_WIDTH = 900
    _MINIMUM_HEIGHT = 820

    _DIAGNOSTIC_TITLES = (
        ("X 误差 (mm)", "Y 误差 (mm)"),
        ("X 误差积分累计 (mm*s)", "Y 误差积分累计 (mm*s)"),
    )

    def __init__(self) -> None:
        super().__init__()
        self.setMinimumSize(self._MINIMUM_WIDTH, self._MINIMUM_HEIGHT)
        self.window_s = 30.0
        self.follow_latest = True
        self.heading_mode = "WIT"
        self.source = QComboBox()
        self.source.addItem("经典单点", "classic")
        self.source.addItem("连续路径", "path")
        self.source.addItem("全向位置", "holonomic")
        self.mode = QComboBox()
        self.mode.addItems(["误差", "积分累计"])
        self.follow = QPushButton("跟随最新")
        self.fit = QPushButton("适配纵轴")
        controls = QHBoxLayout()
        controls.addWidget(self.source)
        controls.addWidget(self.mode)
        controls.addWidget(self.follow)
        controls.addWidget(self.fit)
        controls.addStretch()

        self.graphics = pg.GraphicsLayoutWidget()
        self.graphics.setMinimumSize(self._MINIMUM_WIDTH, self._MINIMUM_HEIGHT - 56)
        self.graphics.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.position_x = self.graphics.addPlot(row=0, col=0, title="OPS X 位置 (世界坐标, mm)")
        self.error_x = self.graphics.addPlot(row=0, col=1, title=self._DIAGNOSTIC_TITLES[0][0])
        self.position_y = self.graphics.addPlot(row=1, col=0, title="OPS Y 位置 (世界坐标, mm)")
        self.error_y = self.graphics.addPlot(row=1, col=1, title=self._DIAGNOSTIC_TITLES[0][1])
        self.speed_x = self.graphics.addPlot(row=2, col=0, title="X 速度 (mm/s)")
        self.speed_y = self.graphics.addPlot(row=2, col=1, title="Y 速度 (mm/s)")
        self.wit_yaw = self.graphics.addPlot(row=3, col=0, title="WIT 相对航向 (deg)")
        self.error_wit_yaw = self.graphics.addPlot(row=3, col=1, title="WIT 航向误差 (deg)")
        self.ops_yaw = self.graphics.addPlot(row=4, col=0, title="OPS 相对 Z 航向 (deg)")
        self.error_ops_yaw = self.graphics.addPlot(row=4, col=1, title="OPS 航向误差 (deg)")
        self.plots = (self.position_x, self.position_y, self.speed_x, self.speed_y,
                      self.wit_yaw, self.ops_yaw, self.error_x, self.error_y,
                      self.error_wit_yaw, self.error_ops_yaw)
        for plot in self.plots:
            plot.setXLink(self.position_x)
            plot.showGrid(x=True, y=True, alpha=0.25)
            plot.addLegend()
            plot.setLabel("bottom", "时间", units="s")

        self.curves = {
            "target_x": self.position_x.plot(pen=pg.mkPen("#f6c85f", width=2), name="目标 X"),
            "actual_x": self.position_x.plot(pen=pg.mkPen("#4ecdc4", width=2), name="实际 X"),
            "target_y": self.position_y.plot(pen=pg.mkPen("#c792ea", width=2), name="目标 Y"),
            "actual_y": self.position_y.plot(pen=pg.mkPen("#82aaff", width=2), name="实际 Y"),
            "command_vx": self.speed_x.plot(pen=pg.mkPen("#f6c85f", width=2), name="命令 vx"),
            "measured_vx": self.speed_x.plot(pen=pg.mkPen("#4ecdc4", width=2), name="实测 vx"),
            "command_vy": self.speed_y.plot(pen=pg.mkPen("#c792ea", width=2), name="命令 vy"),
            "measured_vy": self.speed_y.plot(pen=pg.mkPen("#82aaff", width=2), name="实测 vy"),
            "target_wit_yaw": self.wit_yaw.plot(pen=pg.mkPen("#f6c85f", width=2), name="目标 yaw"),
            "actual_wit_yaw": self.wit_yaw.plot(pen=pg.mkPen("#4ecdc4", width=2), name="WIT yaw"),
            "target_ops_yaw": self.ops_yaw.plot(pen=pg.mkPen("#f6c85f", width=2), name="目标 yaw"),
            "actual_ops_yaw": self.ops_yaw.plot(pen=pg.mkPen("#82aaff", width=2), name="OPS Z yaw"),
        }
        self.diag = (
            self.error_x.plot(pen=pg.mkPen("#ff6b6b", width=2), name="X 误差"),
            self.error_y.plot(pen=pg.mkPen("#ffd166", width=2), name="Y 误差"),
            self.error_wit_yaw.plot(pen=pg.mkPen("#a8dadc", width=2), name="WIT 航向误差"),
            self.error_ops_yaw.plot(pen=pg.mkPen("#c792ea", width=2), name="OPS 航向误差"),
        )
        self.zero_lines = tuple(
            pg.InfiniteLine(pos=0.0, angle=0,
                            pen=pg.mkPen("#8a8f98", width=1, style=Qt.PenStyle.DashLine))
            for _ in self.diag
        )
        for plot, line in zip(self.diag_plots, self.zero_lines):
            plot.addItem(line)

        self.holonomic_graphics = pg.GraphicsLayoutWidget()
        self.holonomic_graphics.setMinimumSize(self._MINIMUM_WIDTH, self._MINIMUM_HEIGHT - 56)
        self.holonomic_graphics.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.holo_position = self.holonomic_graphics.addPlot(
            row=0, col=0, title="全向参考与实际位置 (mm)")
        self.holo_error = self.holonomic_graphics.addPlot(
            row=1, col=0, title="全向误差 (mm / deg)")
        self.holo_speed = self.holonomic_graphics.addPlot(
            row=2, col=0, title="全向平移速度 (mm/s)")
        self.holo_wz = self.holonomic_graphics.addPlot(
            row=3, col=0, title="全向角速度 (deg/s)")
        self.holo_profile = self.holonomic_graphics.addPlot(
            row=4, col=0, title="全向运动轮廓")
        for plot in (self.holo_position, self.holo_error, self.holo_speed,
                     self.holo_wz, self.holo_profile):
            plot.setXLink(self.holo_position)
            plot.showGrid(x=True, y=True, alpha=0.25)
            plot.addLegend()
            plot.setLabel("bottom", "时间", units="s")
        self.holo_curves = {
            "ref_x": self.holo_position.plot(pen=pg.mkPen("#f6c85f", width=2), name="参考 X"),
            "actual_x": self.holo_position.plot(pen=pg.mkPen("#4ecdc4", width=2), name="实际 X"),
            "ref_y": self.holo_position.plot(pen=pg.mkPen("#c792ea", width=2), name="参考 Y"),
            "actual_y": self.holo_position.plot(pen=pg.mkPen("#82aaff", width=2), name="实际 Y"),
            "error_forward": self.holo_error.plot(pen=pg.mkPen("#ff6b6b", width=2), name="前向误差"),
            "error_lateral": self.holo_error.plot(pen=pg.mkPen("#ffd166", width=2), name="横向误差"),
            "error_yaw": self.holo_error.plot(pen=pg.mkPen("#c792ea", width=2), name="航向误差"),
            "measured_forward": self.holo_speed.plot(pen=pg.mkPen("#f6c85f", width=2), name="实测前向"),
            "drive_forward": self.holo_speed.plot(pen=pg.mkPen("#4ecdc4", width=2), name="驱动前向"),
            "measured_lateral": self.holo_speed.plot(pen=pg.mkPen("#c792ea", width=2), name="实测横向"),
            "drive_lateral": self.holo_speed.plot(pen=pg.mkPen("#82aaff", width=2), name="驱动横向"),
            "measured_wz": self.holo_wz.plot(pen=pg.mkPen("#f6c85f", width=2), name="实测角速度"),
            "drive_wz": self.holo_wz.plot(pen=pg.mkPen("#4ecdc4", width=2), name="驱动角速度"),
            "profile_speed": self.holo_profile.plot(pen=pg.mkPen("#f6c85f", width=2), name="参考速度"),
            "profile_progress": self.holo_profile.plot(pen=pg.mkPen("#4ecdc4", width=2), name="轮廓进度"),
            "profile_remaining": self.holo_profile.plot(pen=pg.mkPen("#82aaff", width=2), name="轮廓剩余"),
        }

        self.path_graphics = pg.GraphicsLayoutWidget()
        self.path_graphics.setMinimumSize(self._MINIMUM_WIDTH, self._MINIMUM_HEIGHT - 56)
        self.path_graphics.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.path_progress = self.path_graphics.addPlot(
            row=0, col=0, title="路径进度 (mm)")
        self.path_speed = self.path_graphics.addPlot(
            row=1, col=0, title="路径参考速度 (mm/s)")
        self.path_cross = self.path_graphics.addPlot(
            row=2, col=0, title="横向误差 (mm)")
        self.path_wz = self.path_graphics.addPlot(
            row=3, col=0, title="角速度命令 (deg/s)")
        for plot in (self.path_progress, self.path_speed, self.path_cross, self.path_wz):
            plot.setXLink(self.path_progress)
            plot.showGrid(x=True, y=True, alpha=0.25)
            plot.addLegend()
            plot.setLabel("bottom", "时间", units="s")
        self.path_curves = {
            "progress": self.path_progress.plot(pen=pg.mkPen("#4ecdc4", width=2), name="进度"),
            "remaining": self.path_progress.plot(pen=pg.mkPen("#82aaff", width=2), name="剩余"),
            "reference_speed": self.path_speed.plot(pen=pg.mkPen("#f6c85f", width=2), name="参考速度"),
            "cross_track": self.path_cross.plot(pen=pg.mkPen("#ff6b6b", width=2), name="横向误差"),
            "command_wz": self.path_wz.plot(pen=pg.mkPen("#c792ea", width=2), name="角速度命令"),
        }

        self.graphics_stack = QStackedWidget()
        self.graphics_stack.addWidget(self.graphics)
        self.graphics_stack.addWidget(self.path_graphics)
        self.graphics_stack.addWidget(self.holonomic_graphics)

        layout = QVBoxLayout(self)
        layout.addLayout(controls)
        layout.addWidget(self.graphics_stack)
        self.mode.currentIndexChanged.connect(
            lambda _: self.refresh(self._buffer if hasattr(self, "_buffer") else TelemetryBuffer())
        )
        self.source.currentIndexChanged.connect(
            lambda _: self.refresh(self._buffer if hasattr(self, "_buffer") else TelemetryBuffer())
        )
        self.follow.clicked.connect(self.enable_follow)
        self.fit.clicked.connect(self.enable_auto_y)

    def set_window(self, seconds: float) -> None:
        self.window_s = seconds
        self.follow_latest = True

    def set_heading_mode(self, mode: str) -> None:
        """标记当前真正参与控制的航向源；NONE 时两路只用于观察。"""
        normalized = mode.upper()
        if normalized not in HEADING_MODES:
            raise ValueError(f"unknown heading mode: {mode}")
        self.heading_mode = normalized
        if hasattr(self, "_buffer"):
            self.refresh(self._buffer)

    def enable_follow(self) -> None:
        self.follow_latest = True

    def enable_auto_y(self) -> None:
        if hasattr(self, "_buffer"):
            self.refresh(self._buffer)

    def refresh(self, buffer: TelemetryBuffer,
                holonomic=None, path=None) -> None:
        """按数据源选择渲染；隐藏 Dock 时调用方跳过本方法，数据采集不受影响。"""
        self._buffer = buffer
        source = str(self.source.currentData())
        if source == "holonomic":
            self.graphics_stack.setCurrentWidget(self.holonomic_graphics)
            self._refresh_holonomic(list(holonomic) if holonomic else [])
            return
        if source == "path":
            self.graphics_stack.setCurrentWidget(self.path_graphics)
            self._refresh_path(list(path) if path else [])
            return
        self.graphics_stack.setCurrentWidget(self.graphics)
        self._refresh_classic(buffer)

    def _refresh_classic(self, buffer: TelemetryBuffer) -> None:
        rows = buffer.visible(self.window_s)
        if not rows:
            return
        times = [row[0] for row in rows]
        samples = [row[1] for row in rows]
        self.curves["target_x"].setData(times, [sample.target[0] for sample in samples])
        self.curves["actual_x"].setData(times, [sample.actual[0] for sample in samples])
        self.curves["target_y"].setData(times, [sample.target[1] for sample in samples])
        self.curves["actual_y"].setData(times, [sample.actual[1] for sample in samples])
        self.curves["command_vx"].setData(times, [sample.command_velocity[0] for sample in samples])
        self.curves["measured_vx"].setData(times, [sample.measured_velocity[0] for sample in samples])
        self.curves["command_vy"].setData(times, [sample.command_velocity[1] for sample in samples])
        self.curves["measured_vy"].setData(times, [sample.measured_velocity[1] for sample in samples])
        targets = [sample.target[2] for sample in samples]
        wit_values = [sample.wit_yaw_deg for sample in samples]
        ops_values = [sample.ops_yaw_deg for sample in samples]
        self.curves["target_wit_yaw"].setData(times, targets)
        self.curves["actual_wit_yaw"].setData(times, wit_values)
        self.curves["target_ops_yaw"].setData(times, targets)
        self.curves["actual_ops_yaw"].setData(times, ops_values)

        if self.mode.currentIndex() == 0:
            data = [sample.error for sample in samples]
        else:
            data = [sample.integrals for sample in samples]
        for index, curve in enumerate(self.diag[:2]):
            curve.setData(times, [item[index] for item in data])
        current_linear = data[-1]
        for plot, title, value in zip(
                (self.error_x, self.error_y),
                self._DIAGNOSTIC_TITLES[self.mode.currentIndex()],
                current_linear):
            plot.setTitle(f"{title} | 当前 {value:+.2f}")
        wit_errors = [self._wrap_angle(target - actual) for target, actual in zip(targets, wit_values)]
        ops_errors = [self._wrap_angle(target - actual) for target, actual in zip(targets, ops_values)]
        self.diag[2].setData(times, wit_errors)
        self.diag[3].setData(times, ops_errors)
        self.error_wit_yaw.setTitle(self._yaw_error_title("WIT", wit_errors[-1]))
        self.error_ops_yaw.setTitle(self._yaw_error_title("OPS", ops_errors[-1]))

        if self.follow_latest:
            self.position_x.setXRange(max(0.0, times[-1] - self.window_s), max(self.window_s, times[-1]), padding=0)
        self._update_y_ranges(samples, data)

    def _refresh_holonomic(self, samples: list) -> None:
        if not samples:
            return
        times = [sample.tick / 1000.0 for sample in samples]
        curves = self.holo_curves
        curves["ref_x"].setData(times, [sample.reference[0] for sample in samples])
        curves["actual_x"].setData(times, [sample.actual[0] for sample in samples])
        curves["ref_y"].setData(times, [sample.reference[1] for sample in samples])
        curves["actual_y"].setData(times, [sample.actual[1] for sample in samples])
        curves["error_forward"].setData(times, [sample.error[0] for sample in samples])
        curves["error_lateral"].setData(times, [sample.error[1] for sample in samples])
        curves["error_yaw"].setData(times, [sample.error[2] for sample in samples])
        curves["measured_forward"].setData(times, [sample.measured[0] for sample in samples])
        curves["drive_forward"].setData(times, [sample.drive[0] for sample in samples])
        curves["measured_lateral"].setData(times, [sample.measured[1] for sample in samples])
        curves["drive_lateral"].setData(times, [sample.drive[1] for sample in samples])
        curves["measured_wz"].setData(times, [sample.measured[2] for sample in samples])
        curves["drive_wz"].setData(times, [sample.drive[2] for sample in samples])
        curves["profile_speed"].setData(
            times, [sample.profile_reference_speed_mm_s for sample in samples])
        curves["profile_progress"].setData(
            times, [sample.profile_progress_mm for sample in samples])
        curves["profile_remaining"].setData(
            times, [sample.profile_remaining_mm for sample in samples])
        if self.follow_latest:
            self.holo_position.setXRange(
                max(0.0, times[-1] - self.window_s), max(self.window_s, times[-1]), padding=0)

    def _refresh_path(self, samples: list) -> None:
        if not samples:
            return
        times = [sample.tick / 1000.0 for sample in samples]
        curves = self.path_curves
        curves["progress"].setData(times, [sample.progress_mm for sample in samples])
        curves["remaining"].setData(times, [sample.remaining_mm for sample in samples])
        curves["reference_speed"].setData(
            times, [sample.reference_speed_mm_s for sample in samples])
        curves["cross_track"].setData(times, [sample.cross_track_mm for sample in samples])
        curves["command_wz"].setData(
            times, [sample.command_wz_deg_s for sample in samples])
        if self.follow_latest:
            self.path_progress.setXRange(
                max(0.0, times[-1] - self.window_s), max(self.window_s, times[-1]), padding=0)

    def _yaw_error_title(self, source: str, value: float) -> str:
        if self.heading_mode == "NONE":
            role = "仅观测，未参与控制"
        elif self.heading_mode == source:
            role = "当前控制源"
        else:
            role = "对照源"
        return f"{source} 航向误差 (deg，{role}) | 当前 {value:+.2f}"

    @staticmethod
    def _wrap_angle(value: float) -> float:
        return ((value + 180.0) % 360.0) - 180.0

    @staticmethod
    def _set_default_or_adaptive_y_range(plot: pg.PlotItem, values: list[float], default_limit: float) -> None:
        """Keep normal motion on a stable scale and expand only for outliers."""
        minimum = min(values)
        maximum = max(values)
        if -default_limit <= minimum and maximum <= default_limit:
            plot.setYRange(-default_limit, default_limit, padding=0)
            return

        span = maximum - minimum
        padding = max(span * 0.08, default_limit * 0.05)
        plot.setYRange(minimum - padding, maximum + padding, padding=0)

    def _update_y_ranges(self, samples: list, diagnostic_data: list[tuple[float, float, float]]) -> None:
        self._set_default_or_adaptive_y_range(
            self.position_x,
            [value for sample in samples for value in (sample.target[0], sample.actual[0])],
            self._LINEAR_DEFAULT_RANGE_MM,
        )
        self._set_default_or_adaptive_y_range(
            self.position_y,
            [value for sample in samples for value in (sample.target[1], sample.actual[1])],
            self._LINEAR_DEFAULT_RANGE_MM,
        )
        self._set_default_or_adaptive_y_range(
            self.speed_x,
            [value for sample in samples for value in (sample.command_velocity[0], sample.measured_velocity[0])],
            self._SPEED_DEFAULT_RANGE_MM_S,
        )
        self._set_default_or_adaptive_y_range(
            self.speed_y,
            [value for sample in samples for value in (sample.command_velocity[1], sample.measured_velocity[1])],
            self._SPEED_DEFAULT_RANGE_MM_S,
        )
        self._set_default_or_adaptive_y_range(
            self.wit_yaw,
            [value for sample in samples for value in (sample.target[2], sample.wit_yaw_deg)],
            self._YAW_DEFAULT_RANGE_DEG,
        )
        self._set_default_or_adaptive_y_range(
            self.ops_yaw,
            [value for sample in samples for value in (sample.target[2], sample.ops_yaw_deg)],
            self._YAW_DEFAULT_RANGE_DEG,
        )

        linear_limit = self._LINEAR_DEFAULT_RANGE_MM if self.mode.currentIndex() == 0 else 1000.0
        self._set_default_or_adaptive_y_range(self.error_x, [item[0] for item in diagnostic_data], linear_limit)
        self._set_default_or_adaptive_y_range(self.error_y, [item[1] for item in diagnostic_data], linear_limit)
        self._set_default_or_adaptive_y_range(self.error_wit_yaw, [self._wrap_angle(sample.target[2] - sample.wit_yaw_deg) for sample in samples], self._YAW_DEFAULT_RANGE_DEG)
        self._set_default_or_adaptive_y_range(self.error_ops_yaw, [self._wrap_angle(sample.target[2] - sample.ops_yaw_deg) for sample in samples], self._YAW_DEFAULT_RANGE_DEG)

    @property
    def diag_plots(self) -> tuple[pg.PlotItem, pg.PlotItem, pg.PlotItem, pg.PlotItem]:
        return self.error_x, self.error_y, self.error_wit_yaw, self.error_ops_yaw

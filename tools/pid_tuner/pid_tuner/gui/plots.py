from __future__ import annotations

import pyqtgraph as pg
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QComboBox, QHBoxLayout, QPushButton, QSizePolicy, QVBoxLayout, QWidget

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
        self.mode = QComboBox()
        self.mode.addItems(["误差", "积分累计"])
        self.follow = QPushButton("跟随最新")
        self.fit = QPushButton("适配纵轴")
        controls = QHBoxLayout()
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

        layout = QVBoxLayout(self)
        layout.addLayout(controls)
        layout.addWidget(self.graphics)
        self.mode.currentIndexChanged.connect(
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

    def refresh(self, buffer: TelemetryBuffer) -> None:
        self._buffer = buffer
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

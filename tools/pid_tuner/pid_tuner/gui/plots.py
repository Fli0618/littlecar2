from __future__ import annotations

import pyqtgraph as pg
from PySide6.QtWidgets import QComboBox, QHBoxLayout, QPushButton, QSizePolicy, QVBoxLayout, QWidget

from .buffer import TelemetryBuffer


class TelemetryPlots(QWidget):
    """Display pose and per-axis diagnostic values on a shared time axis."""

    _LINEAR_DEFAULT_RANGE_MM = 500.0
    _YAW_DEFAULT_RANGE_DEG = 180.0
    _MINIMUM_WIDTH = 900
    _MINIMUM_HEIGHT = 820

    _DIAGNOSTIC_TITLES = (
        ("X 误差 (mm)", "Y 误差 (mm)"),
        ("X 命令-实际速度 (mm/s)", "Y 命令-实际速度 (mm/s)"),
        ("X PID 积分项", "Y PID 积分项"),
    )

    def __init__(self) -> None:
        super().__init__()
        self.setMinimumSize(self._MINIMUM_WIDTH, self._MINIMUM_HEIGHT)
        self.window_s = 30.0
        self.follow_latest = True
        self.mode = QComboBox()
        self.mode.addItems(["误差", "速度", "积分项"])
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
        self.wit_yaw = self.graphics.addPlot(row=2, col=0, title="WIT 相对航向 (deg)")
        self.error_wit_yaw = self.graphics.addPlot(row=2, col=1, title="WIT 航向误差 (deg)")
        self.ops_yaw = self.graphics.addPlot(row=3, col=0, title="OPS 相对 Z 航向 (deg)")
        self.error_ops_yaw = self.graphics.addPlot(row=3, col=1, title="OPS 航向误差 (deg)")
        self.plots = (self.position_x, self.position_y, self.wit_yaw, self.ops_yaw, self.error_x, self.error_y, self.error_wit_yaw, self.error_ops_yaw)
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
            "target_wit_yaw": self.wit_yaw.plot(pen=pg.mkPen("#f6c85f", width=2), name="目标 yaw"),
            "actual_wit_yaw": self.wit_yaw.plot(pen=pg.mkPen("#4ecdc4", width=2), name="WIT yaw"),
            "target_ops_yaw": self.ops_yaw.plot(pen=pg.mkPen("#f6c85f", width=2), name="目标 yaw"),
            "actual_ops_yaw": self.ops_yaw.plot(pen=pg.mkPen("#82aaff", width=2), name="OPS Z yaw"),
        }
        self.diag = (
            self.error_x.plot(pen=pg.mkPen("#ff6b6b", width=2), name="X"),
            self.error_y.plot(pen=pg.mkPen("#ffd166", width=2), name="Y"),
            self.error_wit_yaw.plot(pen=pg.mkPen("#a8dadc", width=2), name="WIT 航向"),
            self.error_ops_yaw.plot(pen=pg.mkPen("#c792ea", width=2), name="OPS 航向"),
        )

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
        targets = [sample.target[2] for sample in samples]
        wit_values = [sample.wit_yaw_deg for sample in samples]
        ops_values = [sample.ops_yaw_deg for sample in samples]
        self.curves["target_wit_yaw"].setData(times, targets)
        self.curves["actual_wit_yaw"].setData(times, wit_values)
        self.curves["target_ops_yaw"].setData(times, targets)
        self.curves["actual_ops_yaw"].setData(times, ops_values)

        if self.mode.currentIndex() == 0:
            data = [sample.error for sample in samples]
        elif self.mode.currentIndex() == 1:
            data = [
                tuple(command - measured for command, measured in zip(sample.command_velocity, sample.measured_velocity))
                for sample in samples
            ]
        else:
            data = [sample.integrals for sample in samples]
        for index, curve in enumerate(self.diag[:2]):
            curve.setData(times, [item[index] for item in data])
        for plot, title in zip((self.error_x, self.error_y), self._DIAGNOSTIC_TITLES[self.mode.currentIndex()]):
            plot.setTitle(title)
        self.diag[2].setData(times, [self._wrap_angle(target - actual) for target, actual in zip(targets, wit_values)])
        self.diag[3].setData(times, [self._wrap_angle(target - actual) for target, actual in zip(targets, ops_values)])

        if self.follow_latest:
            self.position_x.setXRange(max(0.0, times[-1] - self.window_s), max(self.window_s, times[-1]), padding=0)
        self._update_y_ranges(samples, data)

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
            self.wit_yaw,
            [value for sample in samples for value in (sample.target[2], sample.wit_yaw_deg)],
            self._YAW_DEFAULT_RANGE_DEG,
        )
        self._set_default_or_adaptive_y_range(
            self.ops_yaw,
            [value for sample in samples for value in (sample.target[2], sample.ops_yaw_deg)],
            self._YAW_DEFAULT_RANGE_DEG,
        )

        if self.mode.currentIndex() == 0:
            self._set_default_or_adaptive_y_range(self.error_x, [item[0] for item in diagnostic_data], self._LINEAR_DEFAULT_RANGE_MM)
            self._set_default_or_adaptive_y_range(self.error_y, [item[1] for item in diagnostic_data], self._LINEAR_DEFAULT_RANGE_MM)
            self._set_default_or_adaptive_y_range(self.error_wit_yaw, [self._wrap_angle(sample.target[2] - sample.wit_yaw_deg) for sample in samples], self._YAW_DEFAULT_RANGE_DEG)
            self._set_default_or_adaptive_y_range(self.error_ops_yaw, [self._wrap_angle(sample.target[2] - sample.ops_yaw_deg) for sample in samples], self._YAW_DEFAULT_RANGE_DEG)
        else:
            for plot in self.diag_plots:
                plot.enableAutoRange(axis="y", enable=True)

    @property
    def diag_plots(self) -> tuple[pg.PlotItem, pg.PlotItem, pg.PlotItem, pg.PlotItem]:
        return self.error_x, self.error_y, self.error_wit_yaw, self.error_ops_yaw

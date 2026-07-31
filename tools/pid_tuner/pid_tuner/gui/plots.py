from __future__ import annotations

import pyqtgraph as pg
from PySide6.QtWidgets import QComboBox, QHBoxLayout, QPushButton, QVBoxLayout, QWidget

from .buffer import TelemetryBuffer


class TelemetryPlots(QWidget):
    """Display pose and per-axis diagnostic values on a shared time axis."""

    _DIAGNOSTIC_TITLES = (
        ("X 误差 (mm)", "Y 误差 (mm)", "航向误差 (deg)"),
        ("X 命令-实际速度 (mm/s)", "Y 命令-实际速度 (mm/s)", "航向命令-实际速度 (deg/s)"),
        ("X PID 积分项", "Y PID 积分项", "航向 PID 积分项"),
    )

    def __init__(self) -> None:
        super().__init__()
        self.window_s = 30.0
        self.follow_latest = True
        self.auto_y = True
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
        self.position_x = self.graphics.addPlot(row=0, col=0, title="OPS X 位置 (世界坐标, mm)")
        self.error_x = self.graphics.addPlot(row=0, col=1, title=self._DIAGNOSTIC_TITLES[0][0])
        self.position_y = self.graphics.addPlot(row=1, col=0, title="OPS Y 位置 (世界坐标, mm)")
        self.error_y = self.graphics.addPlot(row=1, col=1, title=self._DIAGNOSTIC_TITLES[0][1])
        self.yaw = self.graphics.addPlot(row=2, col=0, title="相对航向 (WIT 优先, deg)")
        self.error_yaw = self.graphics.addPlot(row=2, col=1, title=self._DIAGNOSTIC_TITLES[0][2])
        self.plots = (self.position_x, self.position_y, self.yaw, self.error_x, self.error_y, self.error_yaw)
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
            "target_yaw": self.yaw.plot(pen=pg.mkPen("#f6c85f", width=2), name="目标 yaw"),
            "actual_yaw": self.yaw.plot(pen=pg.mkPen("#4ecdc4", width=2), name="实际 yaw"),
        }
        self.diag = (
            self.error_x.plot(pen=pg.mkPen("#ff6b6b", width=2), name="X"),
            self.error_y.plot(pen=pg.mkPen("#ffd166", width=2), name="Y"),
            self.error_yaw.plot(pen=pg.mkPen("#a8dadc", width=2), name="航向"),
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
        self.auto_y = True
        for plot in self.plots:
            plot.enableAutoRange(axis="y", enable=True)

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
        self.curves["target_yaw"].setData(times, [sample.target[2] for sample in samples])
        self.curves["actual_yaw"].setData(times, [sample.actual[2] for sample in samples])

        if self.mode.currentIndex() == 0:
            data = [sample.error for sample in samples]
        elif self.mode.currentIndex() == 1:
            data = [
                tuple(command - measured for command, measured in zip(sample.command_velocity, sample.measured_velocity))
                for sample in samples
            ]
        else:
            data = [sample.integrals for sample in samples]
        for index, curve in enumerate(self.diag):
            curve.setData(times, [item[index] for item in data])
        for plot, title in zip(self.diag_plots, self._DIAGNOSTIC_TITLES[self.mode.currentIndex()]):
            plot.setTitle(title)

        if self.follow_latest:
            self.position_x.setXRange(max(0.0, times[-1] - self.window_s), max(self.window_s, times[-1]), padding=0)
        if self.auto_y:
            for plot in self.plots:
                plot.enableAutoRange(axis="y", enable=True)

    @property
    def diag_plots(self) -> tuple[pg.PlotItem, pg.PlotItem, pg.PlotItem]:
        return self.error_x, self.error_y, self.error_yaw

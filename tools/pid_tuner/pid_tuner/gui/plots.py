from __future__ import annotations

import pyqtgraph as pg
from PySide6.QtWidgets import QComboBox, QHBoxLayout, QPushButton, QVBoxLayout, QWidget

from .buffer import TelemetryBuffer


class TelemetryPlots(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.window_s = 30.0; self.follow_latest = True; self.auto_y = True
        self.mode = QComboBox(); self.mode.addItems(["误差", "速度", "积分项"])
        self.follow = QPushButton("跟随最新"); self.fit = QPushButton("适配纵轴")
        controls = QHBoxLayout(); controls.addWidget(self.mode); controls.addWidget(self.follow); controls.addWidget(self.fit); controls.addStretch()
        self.graphics = pg.GraphicsLayoutWidget()
        self.position = self.graphics.addPlot(row=0, col=0, title="位置 (OPS 世界位姿, mm)")
        self.yaw = self.graphics.addPlot(row=1, col=0, title="航向 (WIT 优先, deg)")
        self.diagnostic = self.graphics.addPlot(row=2, col=0, title="误差")
        self.yaw.setXLink(self.position); self.diagnostic.setXLink(self.position)
        for plot in (self.position, self.yaw, self.diagnostic):
            plot.showGrid(x=True, y=True, alpha=0.25); plot.addLegend(); plot.setLabel("bottom", "时间", units="s")
        self.curves = {
            "target_x": self.position.plot(pen=pg.mkPen("#f6c85f", width=2), name="目标 X"),
            "actual_x": self.position.plot(pen=pg.mkPen("#4ecdc4", width=2), name="实际 X"),
            "target_y": self.position.plot(pen=pg.mkPen("#c792ea", width=2), name="目标 Y"),
            "actual_y": self.position.plot(pen=pg.mkPen("#82aaff", width=2), name="实际 Y"),
            "target_yaw": self.yaw.plot(pen=pg.mkPen("#f6c85f", width=2), name="目标 yaw"),
            "actual_yaw": self.yaw.plot(pen=pg.mkPen("#4ecdc4", width=2), name="实际 yaw"),
        }
        self.diag = [self.diagnostic.plot(pen=pg.mkPen(color, width=2), name=name)
                     for color, name in (("#ff6b6b", "X"), ("#ffd166", "Y"), ("#a8dadc", "yaw"))]
        layout = QVBoxLayout(self); layout.addLayout(controls); layout.addWidget(self.graphics)
        self.mode.currentIndexChanged.connect(lambda _: self.refresh(self._buffer if hasattr(self, "_buffer") else TelemetryBuffer()))
        self.follow.clicked.connect(self.enable_follow); self.fit.clicked.connect(self.enable_auto_y)

    def set_window(self, seconds: float) -> None:
        self.window_s = seconds; self.follow_latest = True

    def enable_follow(self) -> None: self.follow_latest = True
    def enable_auto_y(self) -> None:
        self.auto_y = True
        for plot in (self.position, self.yaw, self.diagnostic): plot.enableAutoRange(axis="y", enable=True)

    def refresh(self, buffer: TelemetryBuffer) -> None:
        self._buffer = buffer
        rows = buffer.visible(self.window_s)
        if not rows: return
        times = [row[0] for row in rows]; samples = [row[1] for row in rows]
        self.curves["target_x"].setData(times, [s.target[0] for s in samples]); self.curves["actual_x"].setData(times, [s.actual[0] for s in samples])
        self.curves["target_y"].setData(times, [s.target[1] for s in samples]); self.curves["actual_y"].setData(times, [s.actual[1] for s in samples])
        self.curves["target_yaw"].setData(times, [s.target[2] for s in samples]); self.curves["actual_yaw"].setData(times, [s.actual[2] for s in samples])
        data = [s.error for s in samples] if self.mode.currentIndex() == 0 else ([s.command_velocity[i] - s.measured_velocity[i] for s in samples] for i in range(3))
        if self.mode.currentIndex() == 2: data = [s.integrals for s in samples]
        if self.mode.currentIndex() == 1: data = [list(values) for values in zip(*data)]
        for index, curve in enumerate(self.diag): curve.setData(times, [item[index] for item in data])
        self.diagnostic.setTitle(["误差", "命令-实际速度", "PID 积分项"][self.mode.currentIndex()])
        if self.follow_latest:
            self.position.setXRange(max(0.0, times[-1] - self.window_s), max(self.window_s, times[-1]), padding=0)
        if self.auto_y:
            for plot in (self.position, self.yaw, self.diagnostic): plot.enableAutoRange(axis="y", enable=True)

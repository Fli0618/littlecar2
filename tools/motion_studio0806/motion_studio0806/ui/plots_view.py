"""Clean Telemetry Waveform Plot View for Motion Studio 0806 (Integrates full TelemetryPlots)."""

from __future__ import annotations

from typing import Optional

from PySide6.QtWidgets import QVBoxLayout, QWidget

from pid_tuner.gui.plots import TelemetryPlots
from pid_tuner.gui.buffer import TelemetryBuffer


class CleanPlotsView(QWidget):
    """View wrapper embedding full 8-plot TelemetryPlots component."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.buffer = TelemetryBuffer()
        self.plots = TelemetryPlots()
        self._init_ui()

    def _init_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.plots)

    def update_telemetry(self, raw_tel: Any) -> None:
        if hasattr(raw_tel, "timestamp"):
            self.buffer.append(raw_tel)
            self.plots.update_plots(self.buffer)

    def clear_plots(self) -> None:
        self.buffer.clear()
        self.plots.update_plots(self.buffer)

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

from ..models import Telemetry


@dataclass(frozen=True)
class EventMarker:
    time_s: float
    label: str


class TelemetryBuffer:
    def __init__(self, retention_s: float = 120.0) -> None:
        self.retention_s = retention_s
        self.samples: deque[tuple[float, Telemetry]] = deque()
        self.events: deque[EventMarker] = deque()
        self._origin_tick: int | None = None

    def clear(self) -> None:
        self.samples.clear(); self.events.clear(); self._origin_tick = None

    def append(self, telemetry: Telemetry) -> float:
        if self._origin_tick is None:
            self._origin_tick = telemetry.tick
        time_s = (telemetry.tick - self._origin_tick) / 1000.0
        self.samples.append((time_s, telemetry))
        while self.samples and time_s - self.samples[0][0] > self.retention_s:
            self.samples.popleft()
        while self.events and time_s - self.events[0].time_s > self.retention_s:
            self.events.popleft()
        return time_s

    def add_event(self, label: str) -> None:
        time_s = self.samples[-1][0] if self.samples else 0.0
        self.events.append(EventMarker(time_s, label))

    def latest_time(self) -> float:
        return self.samples[-1][0] if self.samples else 0.0

    def visible(self, window_s: float) -> list[tuple[float, Telemetry]]:
        start = max(0.0, self.latest_time() - window_s)
        return [(time_s, item) for time_s, item in self.samples if time_s >= start]

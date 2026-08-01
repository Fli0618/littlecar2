from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from typing import Callable

from PySide6.QtCore import QObject, Signal

from ..models import MotionGoal, PidConfig, Telemetry
from ..serial_client import SerialClient


class SessionController(QObject):
    telemetry = Signal(object)
    status = Signal(str)
    failure = Signal(str)
    pid_read = Signal(int, object)
    pid_applied = Signal(int, object)
    yaw_source_changed = Signal(str)
    origin_reset = Signal()
    motion_changed = Signal(bool)

    def __init__(self) -> None:
        super().__init__()
        self._client: SerialClient | None = None
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="pid-tuner-ui")
        self.connected = False
        self.motion_active = False
        self._heartbeat_in_flight = False
        self._motion_generation = 0

    def connect_port(self, port: str, baud: int) -> None:
        def action() -> SerialClient:
            client = SerialClient.open_port(port, baud)
            client.start(); client.add_telemetry_callback(self._handle_telemetry)
            client.get_pid()
            return client
        future = self._executor.submit(action)
        future.add_done_callback(self._connected)

    def _connected(self, future: object) -> None:
        try:
            self._client = future.result()  # type: ignore[attr-defined]
            self.connected = True; self.status.emit("已连接")
        except Exception as error:
            self.failure.emit(str(error))

    def disconnect(self) -> None:
        client = self._client; self._client = None; self.connected = False
        self._motion_generation += 1
        if client is not None:
            self._executor.submit(self._stop_and_close, client, self.motion_active)
        self.motion_active = False; self.motion_changed.emit(False); self.status.emit("已断开")

    @staticmethod
    def _stop_and_close(client: SerialClient, stop_motion: bool) -> None:
        try:
            if stop_motion:
                client.stop()
        finally:
            client.close()

    def _set_motion_active(self, active: bool) -> None:
        if self.motion_active == active:
            return
        self.motion_active = active
        self.motion_changed.emit(active)

    def _submit(self, operation: Callable[[SerialClient], object], callback: Callable[[object], None] | None = None) -> None:
        if self._client is None:
            self.failure.emit("未连接串口"); return
        future = self._executor.submit(operation, self._client)
        def done(result: object) -> None:
            try:
                value = result.result()  # type: ignore[attr-defined]
                if callback: callback(value)
            except Exception as error:
                self.failure.emit(str(error))
        future.add_done_callback(done)

    def read_pid(self) -> None:
        self._submit(lambda client: client.get_pid(), lambda value: self.pid_read.emit(value[0], value[1]))

    def apply_pid(self, pid: PidConfig) -> None:
        self._submit(lambda client: client.set_pid(pid), lambda value: self.pid_applied.emit(value, pid))

    def restore_pid(self) -> None:
        self._submit(lambda client: client.restore_pid(), lambda value: self.status.emit(f"PID 已恢复默认，修订号 {value}"))

    def set_yaw_source(self, source: str) -> None:
        self._submit(lambda client: client.set_yaw_source(source), lambda _: self.yaw_source_changed.emit(source))

    def reset_origin(self) -> None:
        self._submit(lambda client: client.reset_origin(), lambda _: self.origin_reset.emit())

    def start_motion(self, goal: MotionGoal) -> None:
        self._motion_generation += 1
        generation = self._motion_generation
        self._set_motion_active(False)

        def done(_: object) -> None:
            if generation != self._motion_generation or not self.connected:
                return
            self.motion_active = True; self.motion_changed.emit(True); self.status.emit("远程运动中")
        self._submit(lambda client: client.goto(goal), done)

    def _handle_telemetry(self, item: Telemetry) -> None:
        self.telemetry.emit(item)
        if self.motion_active and (item.state not in (0, 1) or item.heartbeat_timed_out):
            self._set_motion_active(False)

    def heartbeat(self) -> None:
        if not self.motion_active or self._heartbeat_in_flight or self._client is None:
            return
        client = self._client
        generation = self._motion_generation
        self._heartbeat_in_flight = True
        future = self._executor.submit(client.heartbeat)

        def done(result: object) -> None:
            self._heartbeat_in_flight = False
            try:
                result.result()  # type: ignore[attr-defined]
            except Exception as error:
                if generation == self._motion_generation:
                    self._set_motion_active(False)
                    self.failure.emit(str(error))

        future.add_done_callback(done)

    def stop(self) -> None:
        self._motion_generation += 1
        self._set_motion_active(False)

        def done(_: object) -> None:
            self.motion_active = False; self.motion_changed.emit(False); self.status.emit("已发送 STOP")
        self._submit(lambda client: client.stop(), done)

    def shutdown(self) -> None:
        self.disconnect(); self._executor.shutdown(wait=False)

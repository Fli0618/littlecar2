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
    pid_applied = Signal(int)
    motion_changed = Signal(bool)

    def __init__(self) -> None:
        super().__init__()
        self._client: SerialClient | None = None
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="pid-tuner-ui")
        self.connected = False
        self.motion_active = False

    def connect_port(self, port: str, baud: int) -> None:
        def action() -> SerialClient:
            client = SerialClient.open_port(port, baud)
            client.start(); client.add_telemetry_callback(self.telemetry.emit)
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
        if client is not None:
            self._executor.submit(client.close)
        self.motion_active = False; self.motion_changed.emit(False); self.status.emit("已断开")

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
        self._submit(lambda client: client.set_pid(pid), lambda value: self.pid_applied.emit(value))

    def restore_pid(self) -> None:
        self._submit(lambda client: client.restore_pid(), lambda value: self.pid_applied.emit(value))

    def start_motion(self, goal: MotionGoal) -> None:
        def done(_: object) -> None:
            self.motion_active = True; self.motion_changed.emit(True); self.status.emit("远程运动中")
        self._submit(lambda client: client.goto(goal), done)

    def heartbeat(self) -> None:
        if self.motion_active: self._submit(lambda client: client.heartbeat())

    def stop(self) -> None:
        def done(_: object) -> None:
            self.motion_active = False; self.motion_changed.emit(False); self.status.emit("已发送 STOP")
        self._submit(lambda client: client.stop(), done)

    def shutdown(self) -> None:
        self.disconnect(); self._executor.shutdown(wait=False, cancel_futures=True)

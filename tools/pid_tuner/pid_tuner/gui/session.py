from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import time
from typing import Callable, cast

from PySide6.QtCore import QObject, Signal

from ..models import MotionGoal, PathControlConfig, PidConfig, Telemetry
from ..serial_client import SerialClient
from ..protocol import Frame


class SessionController(QObject):
    telemetry = Signal(object)
    status = Signal(str)
    failure = Signal(str)
    pid_read = Signal(int, object)
    pid_applied = Signal(int, object)
    yaw_source_changed = Signal(str)
    goto_strategy_read = Signal(bool)
    goto_strategy_changed = Signal(bool)
    origin_reset = Signal()
    motion_changed = Signal(bool)
    path_telemetry = Signal(object)
    path_upload_changed = Signal(str)
    path_config_read = Signal(int, object)
    path_config_applied = Signal(int, object)

    def __init__(self) -> None:
        super().__init__()
        self._client: SerialClient | None = None
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="pid-tuner-ui")
        self.connected = False
        self.motion_active = False
        self._heartbeat_in_flight = False
        self._motion_generation = 0

    def connect_port(self, port: str, baud: int) -> None:
        def action() -> tuple[SerialClient, tuple[int, PidConfig], bool]:
            client = SerialClient.open_port(port, baud)
            client.start(); client.add_telemetry_callback(self._handle_telemetry)
            client.add_path_telemetry_callback(self._handle_path_telemetry)
            return client, client.get_pid(), client.get_goto_strategy()
        future = self._executor.submit(action)
        future.add_done_callback(self._connected)

    def _connected(self, future: object) -> None:
        try:
            client, pid, goto_strategy = future.result()  # type: ignore[attr-defined]
            self._client = client
            self.connected = True; self.pid_read.emit(*pid)
            self.goto_strategy_read.emit(goto_strategy); self.status.emit("已连接")
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

    def read_path_config(self) -> None:
        self._submit(lambda client: client.get_path_config(),
                     lambda value: self.path_config_read.emit(value[0], value[1]))

    def apply_path_config(self, config: PathControlConfig) -> None:
        self._submit(lambda client: client.set_path_config(config),
                     lambda value: self.path_config_applied.emit(value, config))

    def restore_path_config(self) -> None:
        def restore_and_read(client: SerialClient) -> tuple[int, tuple[int, PathControlConfig]]:
            revision = client.restore_path_config()
            time.sleep(0.05)
            return revision, client.get_path_config()

        def restored(value: object) -> None:
            revision, active = cast(tuple[int, tuple[int, PathControlConfig]], value)
            self.status.emit(f"路径参数已恢复默认，修订号 {revision}")
            self.path_config_read.emit(active[0], active[1])

        self._submit(restore_and_read, restored)

    def set_yaw_source(self, source: str) -> None:
        self._submit(lambda client: client.set_yaw_source(source), lambda _: self.yaw_source_changed.emit(source))

    def set_goto_strategy(self, large_yaw_align_enabled: bool) -> None:
        self._submit(lambda client: client.set_goto_strategy(large_yaw_align_enabled),
                     lambda _: self.goto_strategy_changed.emit(large_yaw_align_enabled))

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

    def upload_path(self, begin: bytes, chunks: list[bytes], commit: bytes) -> None:
        """Upload one path through the existing single-session executor."""
        def upload(client: SerialClient) -> None:
            client.path_begin(begin)
            for chunk in chunks:
                client.path_chunk(chunk)
            client.path_commit(commit)
        self.path_upload_changed.emit("正在上传")
        self._submit(upload, lambda _: self.path_upload_changed.emit("路径已提交"))

    def start_path(self, payload: bytes) -> None:
        self._submit(lambda client: client.path_start(payload), lambda _: self._set_motion_active(True))

    def abort_path(self) -> None:
        self._motion_generation += 1
        self._set_motion_active(False)
        self._submit(lambda client: client.path_abort(), lambda _: self.path_upload_changed.emit("路径已中止"))

    def _handle_path_telemetry(self, frame: Frame) -> None:
        self.path_telemetry.emit(frame)

    def shutdown(self) -> None:
        self.disconnect(); self._executor.shutdown(wait=False)

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import time
from typing import Callable, TypeVar

from PySide6.QtCore import QObject, Signal

from ..models import (AckResponse, GotoStrategySnapshot, MotionGoal, PathBeginCommand, PathChunkCommand,
                      PathCommitCommand, PathConfigState, PathControlConfig, PathStartCommand,
                      PathTelemetry, PidConfig, PidConfigState, Telemetry)
from ..serial_client import SerialClient


ConfigState = TypeVar("ConfigState", PidConfigState, PathConfigState)


class SessionController(QObject):
    telemetry = Signal(object)
    status = Signal(str)
    connection_changed = Signal(bool)
    connection_failed = Signal(str)
    operation_failed = Signal(str)
    pid_read = Signal(object)
    pid_applied = Signal(object)
    yaw_source_changed = Signal(str)
    goto_strategy_read = Signal(object)
    goto_strategy_changed = Signal(object)
    origin_reset = Signal()
    motion_changed = Signal(bool)
    path_telemetry = Signal(object)
    path_committed = Signal(int)
    path_started = Signal(int)
    path_config_read = Signal(object)
    path_config_applied = Signal(object)

    def __init__(self) -> None:
        super().__init__()
        self._client: SerialClient | None = None
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="pid-tuner-ui")
        self.connected = False
        self.motion_active = False
        self._heartbeat_in_flight = False
        self._motion_generation = 0

    def connect_port(self, port: str, baud: int) -> None:
        def action() -> tuple[SerialClient, PidConfigState, PathConfigState, GotoStrategySnapshot]:
            client = SerialClient.open_port(port, baud)
            try:
                client.start()
                client.add_telemetry_callback(self._handle_telemetry)
                client.add_path_telemetry_callback(self._handle_path_telemetry)
                return client, client.get_pid(), client.get_path_config(), client.get_goto_strategy()
            except Exception:
                client.close()
                raise

        future = self._executor.submit(action)
        future.add_done_callback(self._connected)

    def _connected(self, future: object) -> None:
        try:
            client, pid, path_config, goto_strategy = future.result()  # type: ignore[attr-defined]
            self._client = client
            self.connected = True
            self.pid_read.emit(pid)
            self.path_config_read.emit(path_config)
            self.goto_strategy_read.emit(goto_strategy)
            self.connection_changed.emit(True)
            self.status.emit("已连接，参数已同步")
        except Exception as error:
            self._client = None
            self.connected = False
            self.connection_changed.emit(False)
            self.connection_failed.emit(str(error))

    def disconnect(self) -> None:
        client = self._client
        self._client = None
        self.connected = False
        self.connection_changed.emit(False)
        self._motion_generation += 1
        if client is not None:
            self._executor.submit(self._stop_and_close, client, self.motion_active)
        self.motion_active = False
        self.motion_changed.emit(False)
        self.status.emit("已断开")

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

    def _submit(self, operation: Callable[[SerialClient], object],
                callback: Callable[[object], None] | None = None) -> None:
        if self._client is None:
            self.operation_failed.emit("未连接串口")
            return
        future = self._executor.submit(operation, self._client)

        def done(result: object) -> None:
            try:
                value = result.result()  # type: ignore[attr-defined]
                if callback:
                    callback(value)
            except Exception as error:
                self.operation_failed.emit(str(error))

        future.add_done_callback(done)

    @staticmethod
    def _wait_for_revision(response: AckResponse, read_active: Callable[[], ConfigState],
                           label: str) -> ConfigState:
        """Wait until the board reports the ACK revision as its active configuration."""
        if response.revision is None:
            raise RuntimeError(f"{label} ACK 缺少修订号")
        deadline = time.monotonic() + 1.0
        while True:
            active = read_active()
            if active.revision == response.revision:
                return active
            if time.monotonic() >= deadline:
                break
            time.sleep(0.05)
        raise RuntimeError(f"{label} 修订号 {response.revision} 未在周期边界生效")

    def read_pid(self) -> None:
        self._submit(lambda client: client.get_pid(), self.pid_read.emit)

    def apply_pid(self, pid: PidConfig) -> None:
        self._submit(lambda client: self._wait_for_revision(
            client.set_pid(pid), client.get_pid, "PID"), self.pid_applied.emit)

    def restore_pid(self) -> None:
        def restored(state: object) -> None:
            active = state  # Signal callbacks receive object, narrowed at this boundary.
            assert isinstance(active, PidConfigState)
            self.pid_read.emit(active)
            self.status.emit(f"PID 已恢复默认，修订号 {active.revision}")

        self._submit(lambda client: self._wait_for_revision(
            client.restore_pid(), client.get_pid, "PID"), restored)

    def read_path_config(self) -> None:
        self._submit(lambda client: client.get_path_config(), self.path_config_read.emit)

    def apply_path_config(self, config: PathControlConfig) -> None:
        self._submit(lambda client: self._wait_for_revision(
            client.set_path_config(config), client.get_path_config, "路径参数"),
            self.path_config_applied.emit)

    def restore_path_config(self) -> None:
        def restored(value: object) -> None:
            assert isinstance(value, PathConfigState)
            active = value
            self.status.emit(f"路径参数已恢复默认，修订号 {active.revision}")
            self.path_config_read.emit(active)

        self._submit(lambda client: self._wait_for_revision(
            client.restore_path_config(), client.get_path_config, "路径参数"), restored)

    def set_yaw_source(self, source: str) -> None:
        self._submit(lambda client: client.set_yaw_source(source),
                     lambda _: self.yaw_source_changed.emit(source))

    def set_goto_strategy(self, large_yaw_align_enabled: bool) -> None:
        self._submit(lambda client: client.set_goto_strategy(large_yaw_align_enabled),
                     lambda _: self.goto_strategy_changed.emit(
                         GotoStrategySnapshot(large_yaw_align_enabled)))

    def reset_origin(self) -> None:
        self._submit(lambda client: client.reset_origin(), lambda _: self.origin_reset.emit())

    def start_motion(self, goal: MotionGoal) -> None:
        self._motion_generation += 1
        generation = self._motion_generation
        self._set_motion_active(False)

        def done(_: object) -> None:
            if generation != self._motion_generation or not self.connected:
                return
            self.motion_active = True
            self.motion_changed.emit(True)
            self.status.emit("远程运动中")

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
                    self.operation_failed.emit(str(error))

        future.add_done_callback(done)

    def stop(self) -> None:
        self._motion_generation += 1
        self._set_motion_active(False)

        def done(_: object) -> None:
            self.motion_active = False
            self.motion_changed.emit(False)
            self.status.emit("已发送 STOP")

        self._submit(lambda client: client.stop(), done)

    def upload_path(self, begin: PathBeginCommand, chunks: tuple[PathChunkCommand, ...],
                    commit: PathCommitCommand) -> None:
        def upload(client: SerialClient) -> None:
            client.path_begin(begin)
            for chunk in chunks:
                client.path_chunk(chunk)
            client.path_commit(commit)

        self._submit(upload, lambda _: self.path_committed.emit(commit.path_id))

    def start_path(self, command: PathStartCommand) -> None:
        def started(_: object) -> None:
            self._set_motion_active(True)
            self.path_started.emit(command.path_id)
        self._submit(lambda client: client.path_start(command), started)

    def upload_and_start_path(self, begin: PathBeginCommand, chunks: tuple[PathChunkCommand, ...],
                              commit: PathCommitCommand, start: PathStartCommand) -> None:
        def action(client: SerialClient) -> None:
            client.path_begin(begin)
            for chunk in chunks:
                client.path_chunk(chunk)
            client.path_commit(commit)
            client.path_start(start)
        def done(_: object) -> None:
            self.path_committed.emit(commit.path_id)
            self.path_started.emit(start.path_id)
            self._set_motion_active(True)
        self._submit(action, done)

    def abort_path(self) -> None:
        self._motion_generation += 1
        self._set_motion_active(False)
        self._submit(lambda client: client.path_abort())

    def _handle_path_telemetry(self, telemetry: PathTelemetry) -> None:
        self.path_telemetry.emit(telemetry)

    def shutdown(self) -> None:
        self.disconnect()
        self._executor.shutdown(wait=False)

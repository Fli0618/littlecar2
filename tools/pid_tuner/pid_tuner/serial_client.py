"""Threaded serial client for the STM32 PID tuner protocol."""

from __future__ import annotations

from collections.abc import Callable
from queue import Empty, Queue
import threading
import time
from typing import Protocol

from .models import BoardError, MotionGoal, PidConfig, RequestTimeout, Telemetry
from .protocol import (
    CMD_ACK, CMD_ERROR, CMD_GET_GOTO_STRATEGY, CMD_GET_PID, CMD_GOTO_POSE,
    CMD_GOTO_STRATEGY, CMD_HEARTBEAT, CMD_PID, CMD_RESET_ORIGIN, CMD_RESTORE_PID,
    CMD_SET_GOTO_STRATEGY, CMD_SET_PID, CMD_SET_YAW_SOURCE, CMD_STOP, CMD_TELEMETRY,
    Frame, ProtocolError, StreamDecoder, decode_goto_strategy, decode_pid,
    decode_telemetry, encode_frame, encode_goal, encode_goto_strategy, encode_pid,
    encode_yaw_source,
)


class SerialTransport(Protocol):
    @property
    def in_waiting(self) -> int: ...

    def read(self, size: int = 1) -> bytes: ...
    def write(self, data: bytes) -> int: ...
    def flush(self) -> None: ...
    def close(self) -> None: ...


class SerialClient:
    """One request stream plus one background reader for a single serial port."""

    def __init__(self, transport: SerialTransport, request_timeout_s: float = 1.0,
                 max_attempts: int = 3) -> None:
        self._transport = transport
        self._request_timeout_s = request_timeout_s
        self._max_attempts = max_attempts
        self._decoder = StreamDecoder()
        self._responses: Queue[Frame] = Queue()
        self._telemetry: Queue[Telemetry] = Queue()
        self._callbacks: list[Callable[[Telemetry], None]] = []
        self._stop_event = threading.Event()
        self._request_lock = threading.Lock()
        self._sequence_lock = threading.Lock()
        self._sequence = 0
        self._reader: threading.Thread | None = None
        self._reader_error: BaseException | None = None

    @classmethod
    def open_port(cls, port: str, baud: int = 115200, request_timeout_s: float = 1.0,
                  max_attempts: int = 3) -> "SerialClient":
        import serial

        return cls(serial.Serial(port, baud, timeout=0.02), request_timeout_s, max_attempts)

    def __enter__(self) -> "SerialClient":
        self.start()
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.close()

    @property
    def crc_errors(self) -> int:
        return self._decoder.crc_errors

    @property
    def format_errors(self) -> int:
        return self._decoder.format_errors

    def start(self) -> None:
        if self._reader is not None:
            return
        self._reader = threading.Thread(target=self._reader_main, name="pid-tuner-rx", daemon=True)
        self._reader.start()

    def close(self) -> None:
        self._stop_event.set()
        reader = self._reader
        if reader is not None and reader is not threading.current_thread():
            reader.join(timeout=1.0)
        self._transport.close()
        self._reader = None

    def add_telemetry_callback(self, callback: Callable[[Telemetry], None]) -> None:
        self._callbacks.append(callback)

    def get_telemetry(self, timeout_s: float | None = None) -> Telemetry:
        return self._telemetry.get(timeout=timeout_s)

    def get_pid(self) -> tuple[int, PidConfig]:
        return decode_pid(self.request(CMD_GET_PID, expected_command=CMD_PID).payload)

    def set_pid(self, config: PidConfig) -> int:
        frame = self.request(CMD_SET_PID, encode_pid(config))
        if len(frame.payload) != 5:
            raise ProtocolError("SET_PID ACK payload must contain a revision")
        return int.from_bytes(frame.payload[1:], "little")

    def restore_pid(self) -> int:
        frame = self.request(CMD_RESTORE_PID)
        if len(frame.payload) != 5:
            raise ProtocolError("RESTORE_PID ACK payload must contain a revision")
        return int.from_bytes(frame.payload[1:], "little")

    def goto(self, goal: MotionGoal) -> None:
        self.request(CMD_GOTO_POSE, encode_goal(goal))

    def set_yaw_source(self, source: str) -> None:
        self.request(CMD_SET_YAW_SOURCE, encode_yaw_source(source))

    def get_goto_strategy(self) -> bool:
        return decode_goto_strategy(
            self.request(CMD_GET_GOTO_STRATEGY, expected_command=CMD_GOTO_STRATEGY).payload)

    def set_goto_strategy(self, large_yaw_align_enabled: bool) -> None:
        self.request(CMD_SET_GOTO_STRATEGY, encode_goto_strategy(large_yaw_align_enabled))

    def reset_origin(self) -> None:
        self.request(CMD_RESET_ORIGIN)

    def heartbeat(self) -> None:
        self.request(CMD_HEARTBEAT)

    def stop(self) -> None:
        self.request(CMD_STOP)

    def request(self, command: int, payload: bytes = b"", expected_command: int = CMD_ACK) -> Frame:
        self.start()
        with self._request_lock:
            sequence = self._next_sequence()
            raw = encode_frame(command, sequence, payload)
            for _ in range(self._max_attempts):
                self._raise_reader_error()
                self._transport.write(raw)
                self._transport.flush()
                deadline = time.monotonic() + self._request_timeout_s
                while True:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    try:
                        frame = self._responses.get(timeout=remaining)
                    except Empty:
                        break
                    if frame.sequence != sequence:
                        continue
                    if frame.command == CMD_ERROR:
                        code = frame.payload[1] if len(frame.payload) >= 2 else None
                        raise BoardError(command, code)
                    if frame.command != expected_command:
                        raise ProtocolError(
                            f"unexpected response 0x{frame.command:02X} for command 0x{command:02X}"
                        )
                    return frame
            raise RequestTimeout(f"timeout waiting for command 0x{command:02X}")

    def _next_sequence(self) -> int:
        with self._sequence_lock:
            sequence = self._sequence
            self._sequence = (self._sequence + 1) & 0xFF
            return sequence

    def _raise_reader_error(self) -> None:
        if self._reader_error is not None:
            raise RuntimeError("serial reader stopped") from self._reader_error

    def _reader_main(self) -> None:
        try:
            while not self._stop_event.is_set():
                data = self._transport.read(max(self._transport.in_waiting, 1))
                if not data:
                    continue
                for frame in self._decoder.feed(data):
                    if frame.command == CMD_TELEMETRY:
                        telemetry = decode_telemetry(frame)
                        self._telemetry.put(telemetry)
                        for callback in tuple(self._callbacks):
                            try:
                                callback(telemetry)
                            except Exception:
                                pass
                    else:
                        self._responses.put(frame)
        except BaseException as error:
            if not self._stop_event.is_set():
                self._reader_error = error

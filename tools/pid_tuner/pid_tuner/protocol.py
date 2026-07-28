"""Binary protocol shared by the board verifier and later PC tooling."""

from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import Iterable

SYNC = b"\xA5\x5A"
VERSION = 1
MAX_PAYLOAD = 96
FRAME_OVERHEAD = 9

CMD_GET_PID = 0x01
CMD_SET_PID = 0x02
CMD_RESTORE_PID = 0x03
CMD_GOTO_POSE = 0x10
CMD_STOP = 0x11
CMD_HEARTBEAT = 0x12
CMD_ACK = 0x80
CMD_PID = 0x81
CMD_TELEMETRY = 0x82
CMD_ERROR = 0xE0

TELEMETRY_PAYLOAD_SIZE = 88


class ProtocolError(ValueError):
    """The peer sent a syntactically valid frame with an invalid meaning."""


@dataclass(frozen=True)
class Frame:
    command: int
    sequence: int
    payload: bytes = b""
    version: int = VERSION


@dataclass(frozen=True)
class PidConfig:
    kp_pos: float
    ki_pos: float
    kd_pos: float
    kp_yaw: float
    ki_yaw: float
    kd_yaw: float


@dataclass(frozen=True)
class Telemetry:
    tick: int
    pid_revision: int
    overwritten_count: int
    state: int
    flags: int
    target: tuple[float, float, float]
    actual: tuple[float, float, float]
    error: tuple[float, float, float]
    command_velocity: tuple[float, float, float]
    measured_velocity: tuple[float, float, float]
    integrals: tuple[float, float, float]


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(command: int, sequence: int, payload: bytes = b"", version: int = VERSION) -> bytes:
    if not 0 <= command <= 0xFF or not 0 <= sequence <= 0xFF or not 0 <= version <= 0xFF:
        raise ProtocolError("frame header byte is out of range")
    if len(payload) > MAX_PAYLOAD:
        raise ProtocolError("payload exceeds 96 bytes")
    body = struct.pack("<BBBH", version, command, sequence, len(payload)) + payload
    return SYNC + body + struct.pack("<H", crc16_ccitt_false(body))


def encode_pid(config: PidConfig) -> bytes:
    return struct.pack("<6f", config.kp_pos, config.ki_pos, config.kd_pos,
                       config.kp_yaw, config.ki_yaw, config.kd_yaw)


def decode_pid(payload: bytes) -> tuple[int, PidConfig]:
    if len(payload) != 28:
        raise ProtocolError("PID payload must be 28 bytes")
    revision, *values = struct.unpack("<I6f", payload)
    return revision, PidConfig(*values)


def decode_telemetry(frame: Frame) -> Telemetry:
    if frame.command != CMD_TELEMETRY or len(frame.payload) != TELEMETRY_PAYLOAD_SIZE:
        raise ProtocolError("invalid telemetry frame")
    values = struct.unpack("<IIIBBH18f", frame.payload)
    return Telemetry(
        tick=values[0],
        pid_revision=values[1],
        overwritten_count=values[2],
        state=values[3],
        flags=values[4],
        target=tuple(values[6:9]),
        actual=tuple(values[9:12]),
        error=tuple(values[12:15]),
        command_velocity=tuple(values[15:18]),
        measured_velocity=tuple(values[18:21]),
        integrals=tuple(values[21:24]),
    )


class StreamDecoder:
    """Recover valid frames from arbitrary serial read boundaries."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.crc_errors = 0
        self.format_errors = 0

    def feed(self, data: bytes) -> list[Frame]:
        self._buffer.extend(data)
        frames: list[Frame] = []
        while True:
            start = self._buffer.find(SYNC)
            if start < 0:
                self._buffer[:] = self._buffer[-1:] if self._buffer.endswith(SYNC[:1]) else b""
                break
            if start:
                del self._buffer[:start]
            if len(self._buffer) < 7:
                break
            payload_length = self._buffer[5] | (self._buffer[6] << 8)
            if payload_length > MAX_PAYLOAD:
                self.format_errors += 1
                del self._buffer[0]
                continue
            frame_length = FRAME_OVERHEAD + payload_length
            if len(self._buffer) < frame_length:
                break
            raw = bytes(self._buffer[:frame_length])
            received_crc = struct.unpack_from("<H", raw, frame_length - 2)[0]
            if crc16_ccitt_false(raw[2:-2]) != received_crc:
                self.crc_errors += 1
                del self._buffer[0]
                continue
            frames.append(Frame(raw[3], raw[4], raw[7:-2], raw[2]))
            del self._buffer[:frame_length]
        return frames


def telemetry_csv_row(telemetry: Telemetry) -> Iterable[float | int]:
    return (
        telemetry.tick, telemetry.pid_revision, telemetry.overwritten_count,
        telemetry.state, telemetry.flags, *telemetry.target, *telemetry.actual,
        *telemetry.error, *telemetry.command_velocity, *telemetry.measured_velocity,
        *telemetry.integrals,
    )

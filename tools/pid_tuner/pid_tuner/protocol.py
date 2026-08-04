"""Binary protocol shared by the board verifier and later PC tooling."""

from __future__ import annotations

import struct
import math
from typing import Iterable

from .models import (
    AckResponse,
    GotoStrategySnapshot,
    PathConfigSnapshot,
    PathControlConfig,
    PathStatus,
    PathTelemetry,
    PathBeginCommand,
    PathChunkCommand,
    PathCommitCommand,
    PathPointSnapshot,
    PathStartCommand,
    PidConfig,
    Telemetry,
)

SYNC = b"\xA5\x5A"
VERSION = 3
MAX_PAYLOAD = 96
FRAME_OVERHEAD = 9

CMD_GET_PID = 0x01
CMD_SET_PID = 0x02
CMD_RESTORE_PID = 0x03
CMD_GOTO_POSE = 0x10
CMD_STOP = 0x11
CMD_HEARTBEAT = 0x12
CMD_SET_YAW_SOURCE = 0x13
CMD_RESET_ORIGIN = 0x14
CMD_GET_GOTO_STRATEGY = 0x15
CMD_SET_GOTO_STRATEGY = 0x16
CMD_PATH_BEGIN = 0x20
CMD_PATH_CHUNK = 0x21
CMD_PATH_COMMIT = 0x22
CMD_PATH_START = 0x23
CMD_PATH_ABORT = 0x24
CMD_PATH_STATUS = 0x25
CMD_GET_PATH_CONFIG = 0x26
CMD_SET_PATH_CONFIG = 0x27
CMD_RESTORE_PATH_CONFIG = 0x28
CMD_ACK = 0x80
CMD_PID = 0x81
CMD_TELEMETRY = 0x82
CMD_GOTO_STRATEGY = 0x83
CMD_PATH_STATUS_RESPONSE = 0x84
CMD_PATH_TELEMETRY = 0x85
CMD_PATH_CONFIG = 0x86
CMD_ERROR = 0xE0

TELEMETRY_PAYLOAD_SIZE = 96
PATH_TELEMETRY_PAYLOAD_SIZE = 94
PATH_CONFIG_FIELDS = (
    "kp_cross_track", "kd_cross_track_velocity", "kp_yaw", "kd_yaw_rate",
    "cruise_speed_mm_s", "max_yaw_rate_deg_s", "accel_mm_s2", "decel_mm_s2",
    "max_lateral_accel_mm_s2", "curvature_preview_mm", "curvature_ff_time_s",
    "lookahead_min_mm", "lookahead_base_mm", "lookahead_speed_gain_s",
    "lookahead_curve_gain_mm", "lookahead_max_mm", "lookahead_rate_mm_s",
    "initial_lookahead_mm", "final_capture_distance_mm", "final_capture_speed_mm_s",
)
PATH_MAX_POINTS = 256
PATH_CHUNK_MAX_POINTS = 7


class ProtocolError(ValueError):
    """The peer sent a syntactically valid frame with an invalid meaning."""


from dataclasses import dataclass


@dataclass(frozen=True)
class Frame:
    command: int
    sequence: int
    payload: bytes = b""
    version: int = VERSION


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


def encode_path_config(config: PathControlConfig) -> bytes:
    """Encode all path-control values as one atomic little-endian group."""
    values = tuple(getattr(config, field) for field in PATH_CONFIG_FIELDS)
    if not all(math.isfinite(value) for value in values):
        raise ProtocolError("path config values must be finite")
    nonnegative = (
        ("kp_cross_track", 20.0), ("kd_cross_track_velocity", 20.0),
        ("kp_yaw", 20.0), ("kd_yaw_rate", 20.0),
        ("lookahead_speed_gain_s", 2.0), ("lookahead_curve_gain_mm", 1000.0),
        ("curvature_ff_time_s", 2.0),
    )
    positive = (
        ("cruise_speed_mm_s", 1500.0), ("max_yaw_rate_deg_s", 180.0),
        ("accel_mm_s2", 5000.0), ("decel_mm_s2", 5000.0),
        ("max_lateral_accel_mm_s2", 5000.0), ("curvature_preview_mm", 2000.0),
        ("lookahead_min_mm", 1000.0),
        ("lookahead_base_mm", 1000.0), ("lookahead_max_mm", 1000.0),
        ("lookahead_rate_mm_s", 2000.0), ("initial_lookahead_mm", 1000.0),
        ("final_capture_distance_mm", 2000.0), ("final_capture_speed_mm_s", 1500.0),
    )
    mapping = dict(zip(PATH_CONFIG_FIELDS, values))
    if (any(not 0.0 <= mapping[name] <= high for name, high in nonnegative) or
            any(not 0.0 < mapping[name] <= high for name, high in positive)):
        raise ProtocolError("path config value is outside its supported range")
    if not (mapping["lookahead_min_mm"] <= mapping["lookahead_base_mm"] <=
            mapping["lookahead_max_mm"] and
            mapping["initial_lookahead_mm"] >= mapping["lookahead_min_mm"] and
            mapping["initial_lookahead_mm"] <= mapping["lookahead_max_mm"]):
        raise ProtocolError("path lookahead must satisfy min <= base <= max")
    return struct.pack("<20f", *values)


def encode_goal(goal: "MotionGoal") -> bytes:
    from .models import MotionGoal

    if not isinstance(goal, MotionGoal):
        raise ProtocolError("goal must be a MotionGoal")
    return struct.pack("<5fIB", goal.x_mm, goal.y_mm, goal.yaw_deg,
                       goal.vmax_mm_s, goal.wmax_deg_s, goal.timeout_ms,
                       (0x01 if goal.use_yaw else 0x00) | (0x02 if goal.use_position else 0x00))


def _path_point_bytes(points: Iterable[object]) -> tuple[PathPointSnapshot, bytes]:
    snapshots = tuple(PathPointSnapshot(float(point.x_mm), float(point.y_mm), float(point.yaw_deg))
                      for point in points)
    if not 2 <= len(snapshots) <= PATH_MAX_POINTS:
        raise ProtocolError(f"path point count must be between 2 and {PATH_MAX_POINTS}")
    if not all(math.isfinite(value) for point in snapshots for value in
               (point.x_mm, point.y_mm, point.yaw_deg)):
        raise ProtocolError("path point values must be finite")
    raw = b"".join(struct.pack("<fff", point.x_mm, point.y_mm, point.yaw_deg)
                   for point in snapshots)
    return snapshots, raw


def build_path_upload(path_id: int, points: Iterable[object]) -> tuple[
        PathBeginCommand, tuple[PathChunkCommand, ...], PathCommitCommand]:
    snapshots, raw = _path_point_bytes(points)
    if not 0 <= path_id <= 0xFFFFFFFF:
        raise ProtocolError("path id is out of range")
    chunks: list[PathChunkCommand] = []
    for offset in range(0, len(snapshots), PATH_CHUNK_MAX_POINTS):
        chunks.append(PathChunkCommand(path_id, offset,
                                       snapshots[offset:offset + PATH_CHUNK_MAX_POINTS]))
    return (PathBeginCommand(path_id, len(snapshots), crc16_ccitt_false(raw)),
            tuple(chunks), PathCommitCommand(path_id))


def encode_path_begin(command: PathBeginCommand) -> bytes:
    return struct.pack("<IHH", command.path_id, command.point_count, command.crc16)


def encode_path_chunk(command: PathChunkCommand) -> bytes:
    if not 1 <= len(command.points) <= PATH_CHUNK_MAX_POINTS:
        raise ProtocolError("path chunk point count is out of range")
    return (struct.pack("<IHB", command.path_id, command.first_index, len(command.points)) +
            b"".join(struct.pack("<fff", point.x_mm, point.y_mm, point.yaw_deg)
                     for point in command.points))


def encode_path_commit(command: PathCommitCommand) -> bytes:
    return struct.pack("<I", command.path_id)


def encode_path_start(command: PathStartCommand) -> bytes:
    return struct.pack("<I", command.path_id)


def encode_yaw_source(source: str) -> bytes:
    values = {"WIT": 0, "OPS": 1}
    try:
        return bytes([values[source.upper()]])
    except (AttributeError, KeyError) as error:
        raise ProtocolError("yaw source must be WIT or OPS") from error


def encode_goto_strategy(large_yaw_align_enabled: bool) -> bytes:
    return bytes([1 if large_yaw_align_enabled else 0])


def decode_goto_strategy(payload: bytes) -> bool:
    if len(payload) != 1 or payload[0] not in (0, 1):
        raise ProtocolError("GOTO strategy payload must be one boolean byte")
    return bool(payload[0])


def decode_pid(payload: bytes) -> tuple[int, PidConfig]:
    if len(payload) != 28:
        raise ProtocolError("PID payload must be 28 bytes")
    revision, *values = struct.unpack("<I6f", payload)
    return revision, PidConfig(*values)


def decode_path_config(payload: bytes) -> tuple[int, PathControlConfig]:
    """Decode a revision followed by the complete path-control group."""
    if len(payload) != 84:
        raise ProtocolError("path config payload must be 84 bytes")
    revision, *values = struct.unpack("<I20f", payload)
    return revision, PathConfigSnapshot(*values)


def decode_telemetry(frame: Frame) -> Telemetry:
    if (frame.version != VERSION or frame.command != CMD_TELEMETRY or
            len(frame.payload) != TELEMETRY_PAYLOAD_SIZE):
        raise ProtocolError("invalid telemetry frame")
    values = struct.unpack("<IIIBBH20f", frame.payload)
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
        remote_link_status=values[5],
        wit_yaw_deg=values[24],
        ops_yaw_deg=values[25],
    )


def decode_path_telemetry(frame: Frame) -> PathTelemetry:
    """Decode the fixed V3 path diagnostics frame into a typed snapshot."""
    if (frame.version != VERSION or frame.command != CMD_PATH_TELEMETRY or
            len(frame.payload) != PATH_TELEMETRY_PAYLOAD_SIZE):
        raise ProtocolError("invalid path telemetry frame")
    values = struct.unpack("<IIIBHHB19f", frame.payload)
    floats = values[7:]
    return PathTelemetry(
        tick=values[0], path_id=values[1], path_config_revision=values[2],
        state=values[3], nearest_segment_index=values[4],
        target_segment_index=values[5], final_stage=values[6],
        progress_mm=floats[0], remaining_mm=floats[1],
        projection_x_mm=floats[2], projection_y_mm=floats[3],
        lookahead_x_mm=floats[4], lookahead_y_mm=floats[5],
        signed_curvature_1_mm=floats[6], curvature_preview_1_mm=floats[7],
        yaw_gradient_deg_per_mm=floats[8], reference_speed_mm_s=floats[9],
        lookahead_mm=floats[10], feedforward_vx_mm_s=floats[11],
        feedforward_vy_mm_s=floats[12], feedforward_wz_deg_s=floats[13],
        cross_track_mm=floats[14], measured_normal_velocity_mm_s=floats[15],
        normal_velocity_ff_mm_s=floats[16], normal_feedback_mm_s=floats[17],
        command_wz_deg_s=floats[18],
    )


def decode_path_status(frame: Frame) -> PathStatus:
    if (frame.version != VERSION or frame.command != CMD_PATH_STATUS_RESPONSE or
            len(frame.payload) != 17):
        raise ProtocolError("invalid path status frame")
    motion_state = frame.payload[0]
    active_present = bool(frame.payload[1])
    staging_state = frame.payload[2]
    active_count, staging_count, received_count = struct.unpack_from("<HHH", frame.payload, 3)
    active_id, staging_id = struct.unpack_from("<II", frame.payload, 9)
    return PathStatus(motion_state, active_present, staging_state, active_count,
                      staging_count, received_count, active_id, staging_id)


def decode_ack(frame: Frame) -> AckResponse:
    if frame.version != VERSION or frame.command != CMD_ACK or len(frame.payload) not in (1, 5):
        raise ProtocolError("invalid ACK frame")
    revision = struct.unpack("<I", frame.payload[1:5])[0] if len(frame.payload) == 5 else 0
    return AckResponse(frame.payload[0], frame.sequence, revision, False)


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
            if raw[2] != VERSION:
                self.format_errors += 1
                del self._buffer[:frame_length]
                continue
            frames.append(Frame(raw[3], raw[4], raw[7:-2], raw[2]))
            del self._buffer[:frame_length]
        return frames


def telemetry_csv_row(telemetry: Telemetry) -> Iterable[float | int]:
    return (
        telemetry.tick, telemetry.pid_revision, telemetry.overwritten_count,
        telemetry.state, telemetry.flags, *telemetry.target, *telemetry.actual,
        *telemetry.error, *telemetry.command_velocity, *telemetry.measured_velocity,
        *telemetry.integrals, telemetry.wit_yaw_deg, telemetry.ops_yaw_deg,
    )

"""Continuous-path packet construction shared by the workbench controller."""

from __future__ import annotations

import math
import struct
from collections.abc import Iterable

from pid_tuner.protocol import ProtocolError, crc16_ccitt_false

from map_planner.models import PathPosePoint

PATH_MAX_POINTS = 256
PATH_CHUNK_MAX_POINTS = 7


def pack_path_points(points: Iterable[PathPosePoint]) -> bytes:
    values = list(points)
    if not 2 <= len(values) <= PATH_MAX_POINTS:
        raise ProtocolError(f"path point count must be between 2 and {PATH_MAX_POINTS}")
    raw = bytearray()
    for point in values:
        if not all(math.isfinite(value) for value in (point.x_mm, point.y_mm, point.yaw_deg)):
            raise ProtocolError("path point values must be finite")
        raw.extend(struct.pack("<fff", point.x_mm, point.y_mm, point.yaw_deg))
    return bytes(raw)


def build_path_begin(path_id: int, points: list[PathPosePoint]) -> bytes:
    raw = pack_path_points(points)
    return struct.pack("<IHH", path_id, len(points), crc16_ccitt_false(raw))


def build_path_chunks(path_id: int, points: list[PathPosePoint]) -> list[bytes]:
    raw = pack_path_points(points)
    chunks: list[bytes] = []
    for offset in range(0, len(points), PATH_CHUNK_MAX_POINTS):
        count = min(PATH_CHUNK_MAX_POINTS, len(points) - offset)
        chunks.append(struct.pack("<IHB", path_id, offset, count) + raw[offset * 12:(offset + count) * 12])
    return chunks


def build_path_commit(path_id: int) -> bytes:
    return struct.pack("<I", path_id)


def build_path_start(path_id: int) -> bytes:
    return struct.pack("<I", path_id)

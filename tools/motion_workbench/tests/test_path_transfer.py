from __future__ import annotations

import struct
import unittest

from map_planner.models import PathPosePoint
from pid_tuner.protocol import ProtocolError

from motion_workbench.path_transfer import (PATH_CHUNK_MAX_POINTS, PATH_MAX_POINTS, build_path_begin,
                                            build_path_chunks, pack_path_points)


class PathTransferTests(unittest.TestCase):
    def points(self, count: int) -> list[PathPosePoint]:
        return [PathPosePoint(float(index), float(index * 2), float(index)) for index in range(count)]

    def test_begin_contains_point_count_and_crc(self) -> None:
        points = self.points(2)
        path_id, count, checksum = struct.unpack("<IHH", build_path_begin(17, points))
        self.assertEqual((path_id, count), (17, 2))
        self.assertNotEqual(checksum, 0)

    def test_chunks_obey_payload_budget_and_offsets(self) -> None:
        chunks = build_path_chunks(5, self.points(15))
        self.assertEqual(len(chunks), 3)
        self.assertEqual([struct.unpack_from("<H", chunk, 4)[0] for chunk in chunks], [0, 7, 14])
        self.assertTrue(all(chunk[6] <= PATH_CHUNK_MAX_POINTS and len(chunk) <= 96 for chunk in chunks))

    def test_capacity_is_enforced(self) -> None:
        with self.assertRaises(ProtocolError):
            pack_path_points(self.points(PATH_MAX_POINTS + 1))


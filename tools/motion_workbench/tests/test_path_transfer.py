from __future__ import annotations

import struct
import unittest

from map_planner.models import PathPosePoint
from pid_tuner.models import PathBeginCommand
from pid_tuner.protocol import (PATH_CHUNK_MAX_POINTS, PATH_MAX_POINTS, ProtocolError,
                                build_path_upload, encode_path_begin, encode_path_chunk)


class PathTransferTests(unittest.TestCase):
    def points(self, count: int) -> list[PathPosePoint]:
        return [PathPosePoint(float(index), float(index * 2), float(index)) for index in range(count)]

    def test_begin_contains_point_count_and_crc(self) -> None:
        begin, chunks, commit = build_path_upload(17, self.points(2))
        path_id, count, checksum = struct.unpack("<IHH", encode_path_begin(begin))
        self.assertEqual((path_id, count), (17, 2))
        self.assertNotEqual(checksum, 0)
        self.assertEqual((len(chunks), commit.path_id), (1, 17))

    def test_chunks_obey_payload_budget_and_offsets(self) -> None:
        _, chunks, _ = build_path_upload(5, self.points(15))
        self.assertEqual(len(chunks), 3)
        self.assertEqual([item.first_index for item in chunks], [0, 7, 14])
        self.assertTrue(all(len(item.points) <= PATH_CHUNK_MAX_POINTS and
                            len(encode_path_chunk(item)) <= 96 for item in chunks))

    def test_capacity_is_enforced(self) -> None:
        with self.assertRaises(ProtocolError):
            build_path_upload(1, self.points(PATH_MAX_POINTS + 1))


if __name__ == "__main__":
    unittest.main()

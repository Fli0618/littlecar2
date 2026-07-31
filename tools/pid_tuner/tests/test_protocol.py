import struct
import unittest

from pid_tuner.protocol import (
    CMD_TELEMETRY,
    Frame,
    StreamDecoder,
    crc16_ccitt_false,
    decode_telemetry,
    encode_frame,
)


class ProtocolTests(unittest.TestCase):
    def test_crc_reference_vector(self) -> None:
        self.assertEqual(crc16_ccitt_false(b"123456789"), 0x29B1)

    def test_split_and_concatenated_frames(self) -> None:
        first = encode_frame(0x01, 7)
        second = encode_frame(0x12, 8)
        decoder = StreamDecoder()
        self.assertEqual(decoder.feed(first[:4]), [])
        frames = decoder.feed(first[4:] + second)
        self.assertEqual([(item.command, item.sequence) for item in frames], [(1, 7), (0x12, 8)])

    def test_bad_crc_resynchronizes(self) -> None:
        bad = bytearray(encode_frame(0x01, 1))
        bad[-1] ^= 0x01
        good = encode_frame(0x12, 2)
        decoder = StreamDecoder()
        frames = decoder.feed(bytes(bad) + good)
        self.assertEqual(decoder.crc_errors, 1)
        self.assertEqual([(item.command, item.sequence) for item in frames], [(0x12, 2)])

    def test_telemetry_layout(self) -> None:
        payload = struct.pack("<IIIBBH18f", 100, 3, 5, 1, 0x1F, 0xC12C, *range(18))
        telemetry = decode_telemetry(Frame(CMD_TELEMETRY, 9, payload))
        self.assertEqual(telemetry.tick, 100)
        self.assertEqual(telemetry.pid_revision, 3)
        self.assertEqual(telemetry.overwritten_count, 5)
        self.assertEqual(telemetry.target, (0.0, 1.0, 2.0))
        self.assertEqual(telemetry.integrals, (15.0, 16.0, 17.0))
        self.assertTrue(telemetry.remote_goal_active)
        self.assertTrue(telemetry.heartbeat_timed_out)
        self.assertEqual(telemetry.heartbeat_age_ms, 300)


if __name__ == "__main__":
    unittest.main()

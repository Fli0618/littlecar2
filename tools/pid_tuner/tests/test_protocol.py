import struct
import unittest

from pid_tuner.protocol import (
    CMD_TELEMETRY, decode_goto_strategy, decode_path_config, encode_goal,
    encode_goto_strategy, encode_path_config, encode_yaw_source,
    Frame,
    StreamDecoder,
    crc16_ccitt_false,
    decode_path_telemetry, decode_telemetry,
    encode_frame,
)
from pid_tuner.models import PathControlConfig


PATH_CONFIG = PathControlConfig(
    0.98, 0.62, 1.42, 0.427, 820.0, 100.0, 800.0, 1000.0,
    600.0, 300.0, 0.05, 60.0, 60.0, 0.15, 120.0, 180.0,
    400.0, 80.0, 60.0, 150.0,
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
        payload = struct.pack("<IIIBBH20f", 100, 3, 5, 1, 0xBF, 0xC12C, *range(20))
        telemetry = decode_telemetry(Frame(CMD_TELEMETRY, 9, payload))
        self.assertEqual(telemetry.tick, 100)
        self.assertEqual(telemetry.pid_revision, 3)
        self.assertEqual(telemetry.overwritten_count, 5)
        self.assertEqual(telemetry.target, (0.0, 1.0, 2.0))
        self.assertEqual(telemetry.integrals, (15.0, 16.0, 17.0))
        self.assertEqual((telemetry.wit_yaw_deg, telemetry.ops_yaw_deg), (18.0, 19.0))
        self.assertEqual(telemetry.yaw_source, "OPS")
        self.assertTrue(telemetry.yaw_aligning)
        self.assertTrue(telemetry.remote_goal_active)
        self.assertTrue(telemetry.heartbeat_timed_out)
        self.assertEqual(telemetry.heartbeat_age_ms, 300)

    def test_goal_flags_and_yaw_source_encoding(self) -> None:
        from pid_tuner.models import MotionGoal
        self.assertEqual(encode_goal(MotionGoal(0, 0, 0, 1, 1, 1))[-1], 0x03)
        self.assertEqual(encode_goal(MotionGoal(0, 0, 0, 1, 1, 1, use_yaw=False))[-1], 0x02)
        self.assertEqual(encode_goal(MotionGoal(0, 0, 0, 1, 1, 1, use_position=False))[-1], 0x01)
        self.assertEqual(encode_yaw_source("ops"), b"\x01")

    def test_goto_strategy_encoding_requires_one_boolean_byte(self) -> None:
        self.assertEqual(encode_goto_strategy(False), b"\x00")
        self.assertEqual(encode_goto_strategy(True), b"\x01")
        self.assertFalse(decode_goto_strategy(b"\x00"))
        self.assertTrue(decode_goto_strategy(b"\x01"))
        with self.assertRaises(ValueError):
            decode_goto_strategy(b"\x02")

    def test_path_config_round_trip_and_validation(self) -> None:
        encoded = encode_path_config(PATH_CONFIG)
        self.assertEqual(len(encoded), 80)
        revision, decoded = decode_path_config(struct.pack("<I", 7) + encoded)
        self.assertEqual(revision, 7)
        for actual, expected in zip(decoded.to_dict().values(), PATH_CONFIG.to_dict().values()):
            self.assertAlmostEqual(actual, expected, places=5)
        with self.assertRaises(ValueError):
            encode_path_config(PathControlConfig(
                **{**PATH_CONFIG.to_dict(), "lookahead_min_mm": 200.0,
                   "lookahead_max_mm": 100.0}
            ))

    def test_path_telemetry_is_typed_and_within_frame_budget(self) -> None:
        values = [float(index) for index in range(19)]
        payload = struct.pack("<IIIBHHB19f", 1, 2, 3, 1, 4, 5, 0, *values)
        self.assertEqual(len(payload), 94)
        item = decode_path_telemetry(Frame(0x85, 0, payload))
        self.assertEqual((item.path_id, item.path_config_revision), (2, 3))
        self.assertEqual(item.command_wz_deg_s, 18.0)

    def test_decoder_rejects_legacy_protocol_version(self) -> None:
        legacy = encode_frame(0x01, 1, version=2)
        decoder = StreamDecoder()
        self.assertEqual(decoder.feed(legacy), [])
        self.assertEqual(decoder.format_errors, 1)


if __name__ == "__main__":
    unittest.main()

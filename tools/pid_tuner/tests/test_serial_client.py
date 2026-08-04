import inspect
import struct
import time
import unittest

from pid_tuner.models import BoardError, GotoControlConfigSnapshot, MotionGoal, PathControlConfig, PidConfig
from pid_tuner.protocol import (CMD_ACK, CMD_ERROR, CMD_GET_GOTO_CONFIG, CMD_GET_GOTO_STRATEGY, CMD_GOTO_CONFIG,
                                CMD_GET_PATH_CONFIG, CMD_GET_PID, CMD_GOTO_STRATEGY,
                                CMD_PATH_CONFIG, CMD_PID, CMD_RESET_ORIGIN,
                                CMD_RESTORE_GOTO_CONFIG, CMD_RESTORE_PATH_CONFIG, CMD_SET_GOTO_CONFIG, CMD_SET_GOTO_STRATEGY,
                                CMD_SET_PATH_CONFIG, CMD_SET_YAW_SOURCE, CMD_TELEMETRY,
                                StreamDecoder, encode_frame)
from pid_tuner.serial_client import SerialClient

from fake_transport import FakeTransport


class SerialClientTests(unittest.TestCase):
    PATH_CONFIG = PathControlConfig(
        0.98, 0.62, 1.42, 0.427, 820.0, 100.0, 800.0, 1000.0,
        600.0, 300.0, 0.05, 60.0, 60.0, 0.15, 120.0, 180.0,
        400.0, 80.0, 60.0, 150.0,
    )
    GOTO_CONFIG = GotoControlConfigSnapshot(
        500.0, 820.0, 800.0, 1000.0, 180.0, 150.0, 300.0, 0.8, 0.2, 180.0,
        90.0, 180.0, 220.0, 25.0, 20.0, 60.0, 1.2, 0.3, 45.0, 250, 500,
    )
    def test_get_pid_and_close(self) -> None:
        def on_write(raw, _attempt):
            request = StreamDecoder().feed(raw)[0]
            self.assertEqual(request.command, CMD_GET_PID)
            payload = struct.pack("<I6f", 9, 1, 2, 3, 4, 5, 6)
            return [encode_frame(CMD_PID, request.sequence, payload)]

        transport = FakeTransport(on_write)
        with SerialClient(transport) as client:
            state = client.get_pid()
        self.assertEqual(state.revision, 9)
        self.assertEqual(state.config, PidConfig(1, 2, 3, 4, 5, 6))
        self.assertTrue(transport.closed)

    def test_timeout_retry_reuses_sequence(self) -> None:
        decoder = StreamDecoder()

        def on_write(raw, attempt):
            request = decoder.feed(raw)[0]
            if attempt == 1:
                return []
            return [encode_frame(CMD_ACK, request.sequence, bytes([request.command]))]

        transport = FakeTransport(on_write)
        with SerialClient(transport, request_timeout_s=0.03, max_attempts=3) as client:
            client.heartbeat()
        self.assertEqual(len(transport.writes), 2)
        first = StreamDecoder().feed(transport.writes[0])[0]
        second = StreamDecoder().feed(transport.writes[1])[0]
        self.assertEqual(first.sequence, second.sequence)

    def test_telemetry_callback(self) -> None:
        telemetry_payload = struct.pack("<IIIBBH20f", 10, 2, 0, 1, 3, 0, *range(20))

        def on_write(raw, _attempt):
            request = StreamDecoder().feed(raw)[0]
            return [
                encode_frame(CMD_TELEMETRY, 7, telemetry_payload),
                encode_frame(CMD_ACK, request.sequence, bytes([request.command])),
            ]

        received = []
        with SerialClient(FakeTransport(on_write)) as client:
            client.add_telemetry_callback(received.append)
            client.heartbeat()
            telemetry = client.get_telemetry(timeout_s=0.2)
        self.assertEqual(telemetry.tick, 10)
        self.assertEqual(len(received), 1)

    def test_goto_encodes_all_goal_fields(self) -> None:
        captured = []

        def on_write(raw, _attempt):
            request = StreamDecoder().feed(raw)[0]
            captured.append(request)
            return [encode_frame(CMD_ACK, request.sequence, bytes([request.command]))]

        goal = MotionGoal(1, 2, 3, 4, 5, 6000)
        with SerialClient(FakeTransport(on_write)) as client:
            client.goto(goal)
        self.assertEqual(struct.unpack("<5fIB", captured[0].payload), (1, 2, 3, 4, 5, 6000, 3))

    def test_yaw_source_and_origin_commands(self) -> None:
        captured = []

        def on_write(raw, _attempt):
            request = StreamDecoder().feed(raw)[0]
            captured.append(request)
            return [encode_frame(CMD_ACK, request.sequence, bytes([request.command]))]

        with SerialClient(FakeTransport(on_write)) as client:
            client.set_yaw_source("OPS")
            client.reset_origin()
        self.assertEqual((captured[0].command, captured[0].payload), (CMD_SET_YAW_SOURCE, b"\x01"))
        self.assertEqual((captured[1].command, captured[1].payload), (CMD_RESET_ORIGIN, b""))

    def test_goto_strategy_commands(self) -> None:
        captured = []

        def on_write(raw, _attempt):
            request = StreamDecoder().feed(raw)[0]
            captured.append(request)
            if request.command == CMD_GET_GOTO_STRATEGY:
                return [encode_frame(CMD_GOTO_STRATEGY, request.sequence, b"\x01")]
            return [encode_frame(CMD_ACK, request.sequence, bytes([request.command]))]

        with SerialClient(FakeTransport(on_write)) as client:
            self.assertTrue(client.get_goto_strategy().large_yaw_align_enabled)
            response = client.set_goto_strategy(False)
        self.assertEqual(response.command, CMD_SET_GOTO_STRATEGY)
        self.assertEqual((captured[0].command, captured[0].payload), (CMD_GET_GOTO_STRATEGY, b""))
        self.assertEqual((captured[1].command, captured[1].payload), (CMD_SET_GOTO_STRATEGY, b"\x00"))

    def test_board_error_is_exposed(self) -> None:
        def on_write(raw, _attempt):
            request = StreamDecoder().feed(raw)[0]
            return [encode_frame(CMD_ERROR, request.sequence, bytes([request.command, 5]))]

        with SerialClient(FakeTransport(on_write)) as client:
            with self.assertRaises(BoardError) as raised:
                client.heartbeat()
        self.assertEqual(raised.exception.code, 5)

    def test_path_config_get_set_and_restore(self) -> None:
        captured = []

        def on_write(raw, _attempt):
            request = StreamDecoder().feed(raw)[0]
            captured.append(request)
            if request.command == CMD_GET_PATH_CONFIG:
                payload = struct.pack("<I20f", 4, *self.PATH_CONFIG.to_dict().values())
                return [encode_frame(CMD_PATH_CONFIG, request.sequence, payload)]
            revision = 5 if request.command == CMD_SET_PATH_CONFIG else 6
            return [encode_frame(CMD_ACK, request.sequence,
                                 bytes([request.command]) + revision.to_bytes(4, "little"))]

        with SerialClient(FakeTransport(on_write)) as client:
            state = client.get_path_config()
            self.assertEqual(state.revision, 4)
            for actual, expected in zip(state.config.to_dict().values(), self.PATH_CONFIG.to_dict().values()):
                self.assertAlmostEqual(actual, expected, places=5)
            self.assertEqual(client.set_path_config(self.PATH_CONFIG).revision, 5)
            self.assertEqual(client.restore_path_config().revision, 6)
        self.assertEqual([item.command for item in captured], [
            CMD_GET_PATH_CONFIG, CMD_SET_PATH_CONFIG, CMD_RESTORE_PATH_CONFIG,
        ])

    def test_goto_config_get_set_and_restore(self) -> None:
        captured = []

        def on_write(raw, _attempt):
            request = StreamDecoder().feed(raw)[0]
            captured.append(request)
            if request.command == CMD_GET_GOTO_CONFIG:
                payload = struct.pack("<I19f2I", 7, *self.GOTO_CONFIG.to_dict().values())
                return [encode_frame(CMD_GOTO_CONFIG, request.sequence, payload)]
            revision = 8 if request.command == CMD_SET_GOTO_CONFIG else 9
            return [encode_frame(CMD_ACK, request.sequence,
                                 bytes([request.command]) + revision.to_bytes(4, "little"))]

        with SerialClient(FakeTransport(on_write)) as client:
            self.assertEqual(client.get_goto_control_config().revision, 7)
            self.assertEqual(client.set_goto_control_config(self.GOTO_CONFIG).revision, 8)
            self.assertEqual(client.restore_goto_control_config().revision, 9)
        self.assertEqual([item.command for item in captured], [
            CMD_GET_GOTO_CONFIG, CMD_SET_GOTO_CONFIG, CMD_RESTORE_GOTO_CONFIG,
        ])

    def test_public_requests_return_typed_results_and_raw_request_is_private(self) -> None:
        self.assertFalse(hasattr(SerialClient, "request"))
        self.assertTrue(hasattr(SerialClient, "_request_frame"))
        self.assertEqual(inspect.signature(SerialClient.get_pid).return_annotation, "PidConfigState")
        self.assertEqual(inspect.signature(SerialClient.set_pid).return_annotation, "AckResponse")


if __name__ == "__main__":
    unittest.main()

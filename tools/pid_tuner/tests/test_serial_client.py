import struct
import time
import unittest

from pid_tuner.models import MotionGoal, PidConfig
from pid_tuner.protocol import CMD_ACK, CMD_GET_PID, CMD_PID, CMD_TELEMETRY, StreamDecoder, encode_frame
from pid_tuner.serial_client import SerialClient

from fake_transport import FakeTransport


class SerialClientTests(unittest.TestCase):
    def test_get_pid_and_close(self) -> None:
        def on_write(raw, _attempt):
            request = StreamDecoder().feed(raw)[0]
            self.assertEqual(request.command, CMD_GET_PID)
            payload = struct.pack("<I6f", 9, 1, 2, 3, 4, 5, 6)
            return [encode_frame(CMD_PID, request.sequence, payload)]

        transport = FakeTransport(on_write)
        with SerialClient(transport) as client:
            revision, pid = client.get_pid()
        self.assertEqual(revision, 9)
        self.assertEqual(pid, PidConfig(1, 2, 3, 4, 5, 6))
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
        telemetry_payload = struct.pack("<IIIBBH18f", 10, 2, 0, 1, 3, 0, *range(18))

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
        self.assertEqual(struct.unpack("<5fIB", captured[0].payload), (1, 2, 3, 4, 5, 6000, 1))


if __name__ == "__main__":
    unittest.main()

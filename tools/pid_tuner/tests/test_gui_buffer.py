import unittest

from pid_tuner.gui.buffer import TelemetryBuffer
from pid_tuner.models import Telemetry


def sample(tick: int) -> Telemetry:
    return Telemetry(tick, 1, 0, 1, 3, (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0))


class TelemetryBufferTests(unittest.TestCase):
    def test_retention_and_visible_window(self) -> None:
        buffer = TelemetryBuffer(120)
        for second in range(125): buffer.append(sample(second * 1000))
        self.assertLessEqual(buffer.samples[0][0], 5.0)
        self.assertEqual(len(buffer.visible(30)), 31)

    def test_clear_resets_relative_time(self) -> None:
        buffer = TelemetryBuffer(); buffer.append(sample(1000)); buffer.clear()
        self.assertEqual(buffer.append(sample(9000)), 0.0)

import tempfile
import unittest
from pathlib import Path

from pid_tuner.models import PidConfig, Telemetry
from pid_tuner.storage import export_c_defaults, list_profiles, load_profile, save_profile, write_telemetry_csv


class StorageTests(unittest.TestCase):
    def test_profile_round_trip_and_c_export(self) -> None:
        pid = PidConfig(1, 0.03, 0.1, 2, 0.05, 0.08)
        with tempfile.TemporaryDirectory() as directory:
            path = save_profile("baseline", pid, "test", 7, Path(directory))
            loaded, document = load_profile("baseline", Path(directory))
            self.assertEqual(path.name, "baseline.json")
            self.assertEqual(loaded, pid)
            self.assertEqual(document["firmware_revision"], 7)
            self.assertEqual(list_profiles(Path(directory)), ["baseline"])
        self.assertIn("ADVANCE_MOTION_DEFAULT_KP_POS", export_c_defaults(pid))

    def test_csv_output(self) -> None:
        sample = Telemetry(1, 2, 0, 1, 3, (1, 2, 3), (4, 5, 6), (7, 8, 9),
                           (10, 11, 12), (13, 14, 15), (16, 17, 18))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "run.csv"
            write_telemetry_csv(path, [sample])
            text = path.read_text(encoding="utf-8")
        self.assertIn("target_x_mm", text)
        self.assertIn("18", text)

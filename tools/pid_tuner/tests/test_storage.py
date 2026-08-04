import tempfile
import unittest
from pathlib import Path
import math
import re

from pid_tuner.models import (GotoStrategySnapshot, PathConfigSnapshot, PathConfigState, PidConfig,
                               PidConfigState, Telemetry)
from pid_tuner.storage import (PATH_C_MACROS, PID_C_MACROS, export_c_defaults,
                               export_motion_config_header, list_profiles, load_profile, save_profile,
                               write_telemetry_csv)


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
        self.assertIn("wit_yaw_deg", text)
        self.assertIn("ops_yaw_deg", text)
        self.assertIn("18", text)

    def test_export_motion_config_header_is_complete_and_deterministic(self) -> None:
        pid = PidConfigState(12, PidConfig(1.0, -0.0, 3.25, 4.0, 5.0, 6.0))
        path = PathConfigState(8, PathConfigSnapshot(*[float(value) for value in range(10, 30)]))
        exported = export_motion_config_header(pid, path, GotoStrategySnapshot(True))

        macros = re.findall(r"^#define (ADVANCE_MOTION_[A-Z0-9_]+) ", exported, re.MULTILINE)
        expected = [name for name, _ in PID_C_MACROS + PATH_C_MACROS]
        expected.append("ADVANCE_MOTION_DEFAULT_LARGE_YAW_ALIGN_ENABLE")
        self.assertEqual([name for name in macros if name != "ADVANCE_MOTION_CONFIG_SCHEMA_VERSION"], expected)
        self.assertEqual(len(macros), len(set(macros)))
        self.assertTrue(exported.startswith("#ifndef __ADVANCE_MOTION_CONFIG_H__\n"))
        self.assertIn("#define ADVANCE_MOTION_CONFIG_SCHEMA_VERSION ((uint32_t)1U)", exported)
        self.assertIn("#define ADVANCE_MOTION_PATH_CRUISE_SPEED_MM_S (14.0f)", exported)
        self.assertIn("#define ADVANCE_MOTION_DEFAULT_KI_POS (0.0f)", exported)
        self.assertIn("#define ADVANCE_MOTION_DEFAULT_LARGE_YAW_ALIGN_ENABLE ((uint8_t)1U)", exported)
        self.assertEqual(exported, export_motion_config_header(pid, path, GotoStrategySnapshot(True)))
        self.assertTrue(exported.endswith("\n"))
        self.assertFalse(exported.endswith("\n\n"))

    def test_export_motion_config_header_writes_false_strategy_and_rejects_nonfinite_values(self) -> None:
        pid = PidConfigState(1, PidConfig(*(float(value) for value in range(1, 7))))
        path_values = [float(value) for value in range(1, 21)]
        path = PathConfigState(2, PathConfigSnapshot(*path_values))
        self.assertIn("((uint8_t)0U)", export_motion_config_header(pid, path, GotoStrategySnapshot(False)))
        for invalid in (math.nan, math.inf, -math.inf):
            invalid_path = PathConfigState(2, PathConfigSnapshot(invalid, *path_values[1:]))
            with self.assertRaises(ValueError):
                export_motion_config_header(pid, invalid_path, GotoStrategySnapshot(False))

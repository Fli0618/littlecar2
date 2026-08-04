from __future__ import annotations

from pathlib import Path
import re
import unittest

from pid_tuner import protocol


ROOT = Path(__file__).resolve().parents[3]


class MotionCompatibilityTests(unittest.TestCase):
    def test_public_motion_api_signatures_are_present(self) -> None:
        header = (ROOT / "Core" / "Inc" / "advance_motion.h").read_text(encoding="utf-8")
        expected = (
            "AdvanceMotion_Init(void)",
            "AdvanceMotion_Update(void)",
            "AdvanceMotion_SetWorldVelocityEx(float vx_world_mm_s, float vy_world_mm_s, float wz_ccw_deg_s, uint8_t acc)",
            "AdvanceMotion_GotoPoseEx(const WorldGoalPose2D_t *goal, uint8_t acc)",
            "AdvanceMotion_GotoGoalBlocking(const WorldGoalPose2D_t *goal, uint8_t acc)",
            "AdvanceMotion_GotoPoseBlocking(float x_mm, float y_mm,",
            "AdvanceMotion_FollowPathEx(const AdvanceMotion_PathPoint_t *points,",
            "AdvanceMotion_FollowPathBlocking(const AdvanceMotion_PathPoint_t *points,",
            "AdvanceMotion_Cancel(void)",
            "AdvanceMotion_CancelIfActive(void)",
            "AdvanceMotion_GetStatus(AdvanceMotion_RuntimeStatus_t *status)",
            "AdvanceMotion_GetDebugSnapshot(AdvanceMotion_DebugSnapshot_t *snapshot)",
            "AdvanceMotion_GetPidConfig(AdvanceMotion_PidConfig_t *config,",
            "AdvanceMotion_RequestPidConfig(const AdvanceMotion_PidConfig_t *config,",
            "AdvanceMotion_GetPathControlConfig(",
            "AdvanceMotion_RequestPathControlConfig(",
            "AdvanceMotion_SetLargeYawAlignEnabled(uint8_t enabled)",
            "AdvanceMotion_GetLargeYawAlignEnabled(uint8_t *enabled)",
            "AdvanceMotion_ResetYawControl(void)",
        )
        for signature in expected:
            self.assertIn(signature, header)

    def test_protocol_v3_wire_invariants_match_firmware(self) -> None:
        source = (ROOT / "Core" / "Src" / "comm_tuner.c").read_text(encoding="utf-8")
        self.assertEqual(protocol.VERSION, 3)
        self.assertEqual(protocol.TELEMETRY_PAYLOAD_SIZE, 96)
        self.assertEqual(protocol.PATH_TELEMETRY_PAYLOAD_SIZE, 94)
        self.assertEqual(len(protocol.PATH_CONFIG_FIELDS), 20)
        self.assertEqual(len(protocol.encode_path_config(self._path_config())), 80)
        self.assertEqual(len(protocol.encode_path_config(self._path_config())) + 4, 84)
        self.assertRegex(source, r"#define COMM_TUNER_PROTOCOL_VERSION \(\(uint8_t\)3U\)")
        for name, value in vars(protocol).items():
            if name.startswith("CMD_") and isinstance(value, int):
                macro = "COMM_TUNER_" + name
                self.assertRegex(source, rf"#define {macro} \(\(uint8_t\)0x{value:02X}U\)")

    @staticmethod
    def _path_config():
        from pid_tuner.models import PathControlConfig

        return PathControlConfig(*([1.0] * 20))


if __name__ == "__main__":
    unittest.main()

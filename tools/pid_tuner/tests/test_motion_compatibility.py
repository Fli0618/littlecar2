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
        self.assertEqual(protocol.PATH_CONFIG_FIELDS, (
            "kp_cross_track", "kd_cross_track_velocity", "kp_yaw", "kd_yaw_rate",
            "cruise_speed_mm_s", "max_yaw_rate_deg_s", "accel_mm_s2", "decel_mm_s2",
            "max_lateral_accel_mm_s2", "curvature_preview_mm", "curvature_ff_time_s",
            "lookahead_min_mm", "lookahead_base_mm", "lookahead_speed_gain_s",
            "lookahead_curve_gain_mm", "lookahead_max_mm", "lookahead_rate_mm_s",
            "initial_lookahead_mm", "final_capture_distance_mm", "final_capture_speed_mm_s",
        ))
        self.assertEqual(len(protocol.encode_path_config(self._path_config())), 80)
        self.assertEqual(len(protocol.encode_path_config(self._path_config())) + 4, 84)
        self.assertRegex(source, r"#define COMM_TUNER_PROTOCOL_VERSION \(\(uint8_t\)3U\)")
        commands = {
            "CMD_GET_PID": 0x01, "CMD_SET_PID": 0x02, "CMD_RESTORE_PID": 0x03,
            "CMD_GOTO_POSE": 0x10, "CMD_STOP": 0x11, "CMD_HEARTBEAT": 0x12,
            "CMD_SET_YAW_SOURCE": 0x13, "CMD_RESET_ORIGIN": 0x14,
            "CMD_GET_GOTO_STRATEGY": 0x15, "CMD_SET_GOTO_STRATEGY": 0x16,
            "CMD_PATH_BEGIN": 0x20, "CMD_PATH_CHUNK": 0x21, "CMD_PATH_COMMIT": 0x22,
            "CMD_PATH_START": 0x23, "CMD_PATH_ABORT": 0x24, "CMD_PATH_STATUS": 0x25,
            "CMD_GET_PATH_CONFIG": 0x26, "CMD_SET_PATH_CONFIG": 0x27,
            "CMD_RESTORE_PATH_CONFIG": 0x28, "CMD_ACK": 0x80, "CMD_PID": 0x81,
            "CMD_TELEMETRY": 0x82, "CMD_GOTO_STRATEGY": 0x83,
            "CMD_PATH_STATUS_RESPONSE": 0x84, "CMD_PATH_TELEMETRY": 0x85,
            "CMD_PATH_CONFIG": 0x86, "CMD_ERROR": 0xE0,
        }
        for name, value in commands.items():
            self.assertEqual(getattr(protocol, name), value)
            macro = "COMM_TUNER_" + name
            self.assertRegex(source, rf"#define {macro} \(\(uint8_t\)0x{value:02X}U\)")

    @staticmethod
    def _path_config():
        from pid_tuner.models import PathControlConfig

        return PathControlConfig(*([1.0] * 20))


if __name__ == "__main__":
    unittest.main()

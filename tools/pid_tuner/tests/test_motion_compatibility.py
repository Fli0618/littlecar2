from __future__ import annotations

from pathlib import Path
import re
import unittest

from pid_tuner import protocol


ROOT = Path(__file__).resolve().parents[3]


class MotionCompatibilityTests(unittest.TestCase):
    CONFIG_MACROS = (
        "ADVANCE_MOTION_DEFAULT_KP_POS", "ADVANCE_MOTION_DEFAULT_KI_POS",
        "ADVANCE_MOTION_DEFAULT_KD_POS", "ADVANCE_MOTION_DEFAULT_KP_YAW",
        "ADVANCE_MOTION_DEFAULT_KI_YAW", "ADVANCE_MOTION_DEFAULT_KD_YAW",
        "ADVANCE_MOTION_PATH_KP_POS", "ADVANCE_MOTION_PATH_KD_VEL",
        "ADVANCE_MOTION_PATH_KP_YAW", "ADVANCE_MOTION_PATH_KD_YAW",
        "ADVANCE_MOTION_PATH_CRUISE_SPEED_MM_S", "ADVANCE_MOTION_PATH_MAX_WZ_DEG_S",
        "ADVANCE_MOTION_PATH_ACCEL_MM_S2", "ADVANCE_MOTION_PATH_DECEL_MM_S2",
        "ADVANCE_MOTION_PATH_MAX_LATERAL_ACC_MM_S2",
        "ADVANCE_MOTION_PATH_CURVATURE_PREVIEW_MM", "ADVANCE_MOTION_PATH_CURVATURE_FF_TIME_S",
        "ADVANCE_MOTION_PATH_LOOKAHEAD_MIN_MM", "ADVANCE_MOTION_PATH_LOOKAHEAD_BASE_MM",
        "ADVANCE_MOTION_PATH_LOOKAHEAD_SPEED_GAIN_S", "ADVANCE_MOTION_PATH_LOOKAHEAD_CURVE_GAIN_MM",
        "ADVANCE_MOTION_PATH_LOOKAHEAD_MAX_MM", "ADVANCE_MOTION_PATH_LOOKAHEAD_RATE_MM_S",
        "ADVANCE_MOTION_PATH_INITIAL_LOOKAHEAD_MM",
        "ADVANCE_MOTION_PATH_FINAL_CAPTURE_DISTANCE_MM",
        "ADVANCE_MOTION_PATH_FINAL_CAPTURE_SPEED_MM_S",
        "ADVANCE_MOTION_DEFAULT_LARGE_YAW_ALIGN_ENABLE",
        "ADVANCE_MOTION_GOTO_PROFILE_THRESHOLD_MM", "ADVANCE_MOTION_GOTO_CRUISE_SPEED_MM_S",
        "ADVANCE_MOTION_GOTO_ACCEL_MM_S2", "ADVANCE_MOTION_GOTO_DECEL_MM_S2",
        "ADVANCE_MOTION_GOTO_CAPTURE_DISTANCE_MM", "ADVANCE_MOTION_GOTO_CAPTURE_SPEED_MM_S",
        "ADVANCE_MOTION_GOTO_FINAL_MAX_SPEED_MM_S", "ADVANCE_MOTION_GOTO_CROSS_TRACK_KP",
        "ADVANCE_MOTION_GOTO_CROSS_TRACK_KD", "ADVANCE_MOTION_GOTO_CROSS_TRACK_MAX_MM_S",
        "ADVANCE_MOTION_GOTO_YAW_CRUISE_DEG_S", "ADVANCE_MOTION_GOTO_YAW_ACCEL_DEG_S2",
        "ADVANCE_MOTION_GOTO_YAW_DECEL_DEG_S2", "ADVANCE_MOTION_GOTO_YAW_CAPTURE_EQUIVALENT_MM",
        "ADVANCE_MOTION_GOTO_YAW_CAPTURE_RATE_DEG_S", "ADVANCE_MOTION_GOTO_YAW_FINAL_MAX_DEG_S",
        "ADVANCE_MOTION_GOTO_YAW_CORRECTION_KP", "ADVANCE_MOTION_GOTO_YAW_CORRECTION_KD",
        "ADVANCE_MOTION_GOTO_YAW_CORRECTION_MAX_DEG_S", "ADVANCE_MOTION_GOTO_CORRECTION_OPEN_LOOP_MS",
        "ADVANCE_MOTION_GOTO_CORRECTION_BLEND_MS",
    )

    def test_motion_config_header_owns_all_exportable_defaults(self) -> None:
        header = (ROOT / "Core" / "Inc" / "advance_motion.h").read_text(encoding="utf-8")
        config = (ROOT / "Core" / "Inc" / "advance_motion_config.h").read_text(encoding="utf-8")
        source = (ROOT / "Core" / "Src" / "advance_motion.c").read_text(encoding="utf-8")

        for name in self.CONFIG_MACROS:
            self.assertEqual(len(re.findall(rf"^#define {name}\b", config, re.MULTILINE)), 1)
            self.assertNotRegex(header, rf"^#define {name}\b")
        self.assertIn('#include "advance_motion_config.h"', source)
        self.assertNotIn("ADVANCE_MOTION_CONFIG_SCHEMA_VERSION", config)
        self.assertNotIn("ADVANCE_MOTION_CONFIG_SCHEMA_VERSION", source)
        self.assertRegex(source, r"g_pid_default = \{\s*\.kp_pos =", re.DOTALL)
        self.assertRegex(source, r"g_path_config_default = \{\s*\.kp_cross_track =", re.DOTALL)
        for field in (
            "kp_cross_track", "kd_cross_track_velocity", "kp_yaw", "kd_yaw_rate",
            "cruise_speed_mm_s", "max_yaw_rate_deg_s", "accel_mm_s2", "decel_mm_s2",
            "max_lateral_accel_mm_s2", "curvature_preview_mm", "curvature_ff_time_s",
            "lookahead_min_mm", "lookahead_base_mm", "lookahead_speed_gain_s",
            "lookahead_curve_gain_mm", "lookahead_max_mm", "lookahead_rate_mm_s",
            "initial_lookahead_mm", "final_capture_distance_mm", "final_capture_speed_mm_s",
        ):
            self.assertRegex(source, rf"\.{field} =")

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
            "AdvanceMotion_GetGotoControlConfig(",
            "AdvanceMotion_RequestGotoControlConfig(",
            "AdvanceMotion_RestoreDefaultGotoControlConfig(uint32_t *revision)",
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
            "CMD_GET_GOTO_CONFIG": 0x29, "CMD_SET_GOTO_CONFIG": 0x2A,
            "CMD_RESTORE_GOTO_CONFIG": 0x2B, "CMD_GOTO_CONFIG": 0x87,
        }
        for name, value in commands.items():
            self.assertEqual(getattr(protocol, name), value)
            macro = "COMM_TUNER_" + name
            self.assertRegex(source, rf"#define {macro} \(\(uint8_t\)0x{value:02X}U\)")

    def test_goto_config_wire_layout_is_explicit_and_stable(self) -> None:
        header = (ROOT / "Core" / "Inc" / "advance_motion.h").read_text(encoding="utf-8")
        source = (ROOT / "Core" / "Src" / "comm_tuner.c").read_text(encoding="utf-8")
        expected_fields = (
            "profile_threshold_mm", "cruise_speed_mm_s", "accel_mm_s2", "decel_mm_s2",
            "capture_distance_mm", "capture_speed_mm_s", "final_max_speed_mm_s",
            "cross_track_kp", "cross_track_kd", "cross_track_correction_max_mm_s",
            "yaw_cruise_rate_deg_s", "yaw_accel_deg_s2", "yaw_decel_deg_s2",
            "yaw_capture_equivalent_mm", "yaw_capture_rate_deg_s", "yaw_final_max_rate_deg_s",
            "yaw_correction_kp", "yaw_correction_kd", "yaw_correction_max_deg_s",
            "correction_open_loop_ms", "correction_blend_ms",
        )
        struct_body = re.search(
            r"typedef struct\s*\{(?P<body>[^}]*)\} AdvanceMotion_GotoControlConfig_t;", header, re.DOTALL)
        self.assertIsNotNone(struct_body)
        positions = [struct_body.group("body").index(field) for field in expected_fields]  # type: ignore[union-attr]
        self.assertEqual(positions, sorted(positions))
        self.assertEqual(protocol.GOTO_CONFIG_FIELDS, expected_fields[:19])
        self.assertEqual(len(protocol.encode_goto_config(self._goto_config())), 84)
        self.assertRegex(source, r"COMM_TUNER_SET_GOTO_CONFIG_PAYLOAD_SIZE \(\(uint16_t\)84U\)")
        self.assertRegex(source, r"COMM_TUNER_GOTO_CONFIG_PAYLOAD_SIZE \(\(uint16_t\)88U\)")

    @staticmethod
    def _path_config():
        from pid_tuner.models import PathControlConfig

        return PathControlConfig(*([1.0] * 20))

    @staticmethod
    def _goto_config():
        from pid_tuner.models import GotoControlConfig
        return GotoControlConfig(40.0, 700.0, 1200.0, 1500.0, 40.0, 100.0, 160.0,
                                 0.98, 0.62, 150.0, 80.0, 200.0, 280.0, 40.0, 15.0,
                                 25.0, 1.42, 0.427, 20.0, 500, 1000)


if __name__ == "__main__":
    unittest.main()

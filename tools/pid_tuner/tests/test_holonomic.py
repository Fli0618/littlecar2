"""全向位置控制器的协议、串口客户端、会话与导出测试。"""

from __future__ import annotations

from concurrent.futures import Future
import math
import os
import struct
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from pid_tuner.gui.session import ActiveMotionKind, SessionController
from pid_tuner.models import (AckResponse, BoardError, GotoStrategySnapshot, HolonomicConfig,
                               HolonomicConfigState, HolonomicTelemetry, MotionGoal, PathConfigSnapshot,
                               PathConfigState, PathStartCommand, PidConfig, PidConfigState, Telemetry)
from pid_tuner.protocol import (CMD_ACK, CMD_HOLONOMIC_CONFIG, CMD_HOLONOMIC_GOTO_POSE,
                                CMD_HOLONOMIC_TELEMETRY, CMD_GET_HOLONOMIC_CONFIG,
                                CMD_ERROR, ERROR_BAD_COMMAND,
                                CMD_RESTORE_HOLONOMIC_CONFIG, CMD_SET_HOLONOMIC_CONFIG,
                                Frame, ProtocolError, StreamDecoder, decode_holonomic_config,
                                decode_holonomic_telemetry, encode_frame, encode_holonomic_config)
from pid_tuner.serial_client import SerialClient
from pid_tuner.storage import export_motion_config_header

from fake_transport import FakeTransport


HOLONOMIC = HolonomicConfig(
    linear_accel_mm_s2=600.0, linear_decel_mm_s2=800.0, yaw_accel_deg_s2=150.0,
    kp_forward=0.8, kv_forward=0.3, kp_lateral=0.8, kv_lateral=0.3,
    kp_yaw=2.0, kv_yaw=0.3, forward_scale=1.0, lateral_scale=1.0, yaw_scale=1.0,
)


class HolonomicProtocolTests(unittest.TestCase):
    def test_holonomic_config_encode_is_48_bytes_and_roundtrips(self) -> None:
        payload = encode_holonomic_config(HOLONOMIC)
        self.assertEqual(len(payload), 48)
        state = decode_holonomic_config(
            Frame(CMD_HOLONOMIC_CONFIG, 1, struct.pack("<I", 7) + payload))
        self.assertEqual(state.revision, 7)
        for name, expected in HOLONOMIC.to_dict().items():
            self.assertAlmostEqual(getattr(state.config, name), expected, places=5)

    def test_holonomic_config_frame_is_52_bytes(self) -> None:
        raw = struct.pack("<I12f", 3, *HOLONOMIC.to_dict().values())
        self.assertEqual(len(raw), 52)
        self.assertEqual(len(encode_frame(CMD_HOLONOMIC_CONFIG, 1, raw)), 61)

    def test_holonomic_telemetry_frame_is_96_bytes(self) -> None:
        values = (1, 2, 3, 4, 5) + tuple(float(value) for value in range(21))
        payload = struct.pack("<IIBBH21f", *values)
        self.assertEqual(len(payload), 96)
        telemetry = decode_holonomic_telemetry(Frame(CMD_HOLONOMIC_TELEMETRY, 1, payload))
        self.assertEqual(telemetry.tick, 1)
        self.assertEqual(telemetry.config_revision, 2)
        self.assertEqual(telemetry.state, 3)
        self.assertEqual(telemetry.flags, 4)
        self.assertEqual(telemetry.remote_link_status, 5)
        self.assertEqual(telemetry.goal, (0.0, 1.0, 2.0))
        self.assertEqual(telemetry.actual, (3.0, 4.0, 5.0))
        self.assertEqual(telemetry.reference, (6.0, 7.0, 8.0))
        self.assertEqual(telemetry.error, (9.0, 10.0, 11.0))
        self.assertEqual(telemetry.measured, (12.0, 13.0, 14.0))
        self.assertEqual(telemetry.drive, (15.0, 16.0, 17.0))
        self.assertEqual(telemetry.profile_progress_mm, 18.0)
        self.assertEqual(telemetry.profile_remaining_mm, 19.0)
        self.assertEqual(telemetry.profile_reference_speed_mm_s, 20.0)
        self.assertFalse(telemetry.position_constraint_enabled)
        self.assertFalse(telemetry.yaw_constraint_enabled)
        self.assertTrue(telemetry.controller_active)

    def test_holonomic_telemetry_flag_bit2_is_controller_active(self) -> None:
        payload = struct.pack("<IIBBH21f", 0, 0, 0, 0x04, 0, *([0.0] * 21))
        telemetry = decode_holonomic_telemetry(Frame(CMD_HOLONOMIC_TELEMETRY, 1, payload))
        self.assertTrue(telemetry.controller_active)
        self.assertFalse(telemetry.position_constraint_enabled)

    def test_holonomic_config_rejects_out_of_range_values(self) -> None:
        bad_values = (
            (0.0, 800.0, 150.0, 0.8, 0.3, 0.8, 0.3, 2.0, 0.3, 1.0, 1.0, 1.0),
            (600.0, 5001.0, 150.0, 0.8, 0.3, 0.8, 0.3, 2.0, 0.3, 1.0, 1.0, 1.0),
            (600.0, 800.0, 150.0, -0.1, 0.3, 0.8, 0.3, 2.0, 0.3, 1.0, 1.0, 1.0),
            (600.0, 800.0, 150.0, 0.8, 0.3, 0.8, 0.3, 21.0, 0.3, 1.0, 1.0, 1.0),
            (600.0, 800.0, 150.0, 0.8, 0.3, 0.8, 0.3, 2.0, 0.3, 0.4, 1.0, 1.0),
            (600.0, 800.0, 150.0, 0.8, 0.3, 0.8, 0.3, 2.0, 0.3, 1.0, 2.1, 1.0),
            (math.nan, 800.0, 150.0, 0.8, 0.3, 0.8, 0.3, 2.0, 0.3, 1.0, 1.0, 1.0),
        )
        for values in bad_values:
            with self.assertRaises(ProtocolError):
                encode_holonomic_config(HolonomicConfig(*values))

    def test_holonomic_config_rejects_wrong_length(self) -> None:
        with self.assertRaises(ProtocolError):
            decode_holonomic_config(Frame(CMD_HOLONOMIC_CONFIG, 1, b"\x00" * 51))
        with self.assertRaises(ProtocolError):
            decode_holonomic_config(Frame(CMD_HOLONOMIC_CONFIG, 1, b"\x00" * 53))
        with self.assertRaises(ProtocolError):
            decode_holonomic_telemetry(Frame(CMD_HOLONOMIC_TELEMETRY, 1, b"\x00" * 95))

    def test_v3_frame_structure_is_unchanged(self) -> None:
        raw = encode_frame(CMD_HOLONOMIC_CONFIG, 1, struct.pack("<I12f", 3, *HOLONOMIC.to_dict().values()))
        self.assertEqual(raw[:2], b"\xA5\x5A")
        self.assertEqual(raw[2], 3)


class HolonomicSerialClientTests(unittest.TestCase):
    def test_old_firmware_bad_command_is_reported_without_protocol_disconnect(self) -> None:
        def on_write(raw, _attempt):
            request = StreamDecoder().feed(raw)[0]
            return [encode_frame(CMD_ERROR, request.sequence,
                                 bytes([request.command, ERROR_BAD_COMMAND]))]

        with SerialClient(FakeTransport(on_write)) as client:
            with self.assertRaises(BoardError) as raised:
                client.get_holonomic_config()
        self.assertEqual(raised.exception.command, CMD_GET_HOLONOMIC_CONFIG)
        self.assertEqual(raised.exception.code, ERROR_BAD_COMMAND)

    def test_holonomic_commands_use_new_ids_and_payloads(self) -> None:
        captured = []

        def on_write(raw, _attempt):
            request = StreamDecoder().feed(raw)[0]
            captured.append(request)
            if request.command == CMD_GET_HOLONOMIC_CONFIG:
                payload = struct.pack("<I12f", 3, *HOLONOMIC.to_dict().values())
                return [encode_frame(CMD_HOLONOMIC_CONFIG, request.sequence, payload)]
            revision = 4 if request.command == CMD_SET_HOLONOMIC_CONFIG else 5
            return [encode_frame(CMD_ACK, request.sequence,
                                 bytes([request.command]) + revision.to_bytes(4, "little"))]

        with SerialClient(FakeTransport(on_write)) as client:
            state = client.get_holonomic_config()
            self.assertEqual(state.revision, 3)
            for name, expected in HOLONOMIC.to_dict().items():
                self.assertAlmostEqual(getattr(state.config, name), expected, places=5)
            self.assertEqual(client.set_holonomic_config(HOLONOMIC).revision, 4)
            self.assertEqual(client.restore_holonomic_config().revision, 5)
        self.assertEqual([item.command for item in captured], [
            CMD_GET_HOLONOMIC_CONFIG, CMD_SET_HOLONOMIC_CONFIG, CMD_RESTORE_HOLONOMIC_CONFIG,
        ])
        self.assertEqual(len(captured[1].payload), 48)

    def test_holonomic_goto_reuses_goal_payload(self) -> None:
        captured = []

        def on_write(raw, _attempt):
            request = StreamDecoder().feed(raw)[0]
            captured.append(request)
            return [encode_frame(CMD_ACK, request.sequence, bytes([request.command]))]

        goal = MotionGoal(1, 2, 3, 4, 5, 6000)
        with SerialClient(FakeTransport(on_write)) as client:
            client.holonomic_goto(goal)
        self.assertEqual(captured[0].command, CMD_HOLONOMIC_GOTO_POSE)
        self.assertEqual(struct.unpack("<5fIB", captured[0].payload), (1, 2, 3, 4, 5, 6000, 3))

    def test_holonomic_telemetry_routes_to_callbacks(self) -> None:
        payload = struct.pack("<IIBBH21f", 1, 2, 3, 4, 0, *([0.0] * 21))

        def on_write(raw, _attempt):
            request = StreamDecoder().feed(raw)[0]
            return [
                encode_frame(CMD_HOLONOMIC_TELEMETRY, 7, payload),
                encode_frame(CMD_ACK, request.sequence, bytes([request.command])),
            ]

        received = []
        with SerialClient(FakeTransport(on_write)) as client:
            client.add_holonomic_telemetry_callback(received.append)
            client.heartbeat()
        self.assertEqual(len(received), 1)
        self.assertEqual(received[0].tick, 1)
        self.assertEqual(received[0].config_revision, 2)


class HolonomicSessionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    @staticmethod
    def _connected_result(holonomic):
        return (object(),
                PidConfigState(1, PidConfig(1, 2, 3, 4, 5, 6)),
                PathConfigState(2, PathConfigSnapshot(*[float(value) for value in range(1, 21)])),
                GotoStrategySnapshot(True),
                holonomic)

    def test_connected_emits_holonomic_state(self) -> None:
        controller = SessionController()
        received = []
        controller.holonomic_config_read.connect(received.append)
        future = Future()
        future.set_result(self._connected_result(HolonomicConfigState(5, HOLONOMIC)))
        try:
            controller._connected(future)
            self.assertEqual(len(received), 1)
            self.assertEqual(received[0].revision, 5)
            self.assertTrue(controller.connected)
        finally:
            controller._executor.shutdown(wait=False)

    def test_connected_unsupported_firmware_keeps_connection(self) -> None:
        controller = SessionController()
        unsupported = []
        statuses = []
        controller.holonomic_unsupported.connect(lambda: unsupported.append(1))
        controller.status.connect(statuses.append)
        future = Future()
        future.set_result(self._connected_result(None))
        try:
            controller._connected(future)
            self.assertEqual(unsupported, [1])
            self.assertTrue(controller.connected)
            self.assertTrue(any("不支持全向调参" in text for text in statuses))
        finally:
            controller._executor.shutdown(wait=False)

    def test_wait_for_revision_holonomic(self) -> None:
        controller = SessionController()

        class FakeClient:
            def __init__(self) -> None:
                self.active = 8

            def get_holonomic_config(self) -> HolonomicConfigState:
                if self.active == 8:
                    self.active = 9
                return HolonomicConfigState(self.active, HOLONOMIC)

        response = AckResponse(CMD_SET_HOLONOMIC_CONFIG, 1, 9)
        try:
            state = controller._wait_for_revision(
                response, FakeClient().get_holonomic_config, "全向参数")
            self.assertEqual(state.revision, 9)
        finally:
            controller._executor.shutdown(wait=False)

    @staticmethod
    def _classic_telemetry(state: int = 2, remote_link_status: int = 0,
                           actual: tuple[float, float, float] = (0.0, 0.0, 0.0),
                           error: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> Telemetry:
        return Telemetry(1, 0, 0, state, 0x03, (0.0, 0.0, 0.0), actual, error,
                         (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                         remote_link_status=remote_link_status)

    @staticmethod
    def _holonomic_telemetry(state: int = 3, remote_link_status: int = 0) -> HolonomicTelemetry:
        return HolonomicTelemetry(
            1, 0, state, 0x04, remote_link_status,
            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
            0.0, 0.0, 0.0)

    def test_motion_ack_sets_controller_kind(self) -> None:
        controller = SessionController()
        submitted = []
        controller._submit = lambda operation, callback=None, failure_callback=None: submitted.append(
            (callback, failure_callback))  # type: ignore[method-assign]
        controller.connected = True
        try:
            controller.start_motion(MotionGoal(1, 2, 3, 4, 5, 1000))
            submitted.pop()[0](None)
            self.assertEqual(controller._active_motion_kind, ActiveMotionKind.CLASSIC)
            controller.start_holonomic_motion(MotionGoal(1, 2, 3, 4, 5, 1000))
            submitted.pop()[0](None)
            self.assertEqual(controller._active_motion_kind, ActiveMotionKind.HOLONOMIC)
            controller.start_path(PathStartCommand(7))
            submitted.pop()[0](None)
            self.assertEqual(controller._active_motion_kind, ActiveMotionKind.PATH)
        finally:
            controller._executor.shutdown(wait=False)

    def test_unrelated_terminal_telemetry_does_not_clear_motion(self) -> None:
        controller = SessionController()
        classic_received = []
        holonomic_received = []
        controller.telemetry.connect(classic_received.append)
        controller.holonomic_telemetry.connect(holonomic_received.append)
        try:
            controller._set_active_motion(ActiveMotionKind.HOLONOMIC)
            controller._handle_telemetry(self._classic_telemetry())
            self.assertTrue(controller.motion_active)
            self.assertEqual(controller._active_motion_kind, ActiveMotionKind.HOLONOMIC)
            self.assertEqual(len(classic_received), 1)

            controller._set_active_motion(ActiveMotionKind.PATH)
            controller._handle_holonomic_telemetry(self._holonomic_telemetry())
            self.assertTrue(controller.motion_active)
            self.assertEqual(controller._active_motion_kind, ActiveMotionKind.PATH)
            self.assertEqual(len(holonomic_received), 1)
        finally:
            controller._executor.shutdown(wait=False)

    def test_matching_terminal_and_link_timeout_clear_motion(self) -> None:
        controller = SessionController()
        try:
            controller._set_active_motion(ActiveMotionKind.HOLONOMIC)
            controller._handle_holonomic_telemetry(self._holonomic_telemetry())
            self.assertFalse(controller.motion_active)

            controller._set_active_motion(ActiveMotionKind.CLASSIC)
            controller._handle_holonomic_telemetry(self._holonomic_telemetry(remote_link_status=0x4000))
            self.assertFalse(controller.motion_active)

            controller._set_active_motion(ActiveMotionKind.PATH)
            controller._handle_telemetry(self._classic_telemetry())
            self.assertFalse(controller.motion_active)

            controller._set_active_motion(ActiveMotionKind.HOLONOMIC)
            controller._handle_telemetry(self._classic_telemetry(state=1, remote_link_status=0x4000))
            self.assertFalse(controller.motion_active)
        finally:
            controller._executor.shutdown(wait=False)

    def test_stop_disconnect_and_abort_reset_motion_kind(self) -> None:
        controller = SessionController()
        try:
            for action in (controller.stop, controller.abort_path, controller.disconnect):
                controller._set_active_motion(ActiveMotionKind.HOLONOMIC)
                action()
                self.assertEqual(controller._active_motion_kind, ActiveMotionKind.NONE)
                self.assertFalse(controller.motion_active)
        finally:
            controller._executor.shutdown(wait=False)


class HolonomicExportTests(unittest.TestCase):
    def test_export_motion_config_header_contains_holonomic_defaults(self) -> None:
        text = export_motion_config_header(
            PidConfigState(1, PidConfig(1, 2, 3, 4, 5, 6)),
            PathConfigState(2, PathConfigSnapshot(*([1.0] * 20))),
            GotoStrategySnapshot(False),
            HolonomicConfigState(4, HOLONOMIC),
        )
        self.assertIn("ADVANCE_HOLONOMIC_DEFAULT_LINEAR_ACCEL_MM_S2 (600.0f)", text)
        self.assertIn("ADVANCE_HOLONOMIC_DEFAULT_YAW_SCALE (1.0f)", text)
        self.assertIn("Holonomic revision: 4", text)
        self.assertIn("ADVANCE_MOTION_DEFAULT_KP_POS", text)
        self.assertEqual(text.count("#ifndef __ADVANCE_MOTION_CONFIG_H__"), 1)
        self.assertEqual(text.count("#endif"), 1)


if __name__ == "__main__":
    unittest.main()

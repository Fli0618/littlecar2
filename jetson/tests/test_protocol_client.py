import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "src"))

import main
from protocol.commands import (
    ACK_BAD_PERIOD,
    ACK_OK,
    CMD_ACK,
    CMD_COLOR_RESULT,
    CMD_COMPETITION_START,
    CMD_START_CIRCLE,
    CMD_START_DISK_CENTER,
    CMD_QR_RESULT,
    CMD_START_COLOR,
    CMD_START_QR,
    CMD_STOP,
    START_AREA_1,
    START_AREA_2,
    TASK_CODE_LENGTH,
)
from protocol.frame import pack_frame, parse_frames


class Port:
    def __init__(self):
        self.in_waiting = 0
        self.input = bytearray()
        self.output = bytearray()

    def read(self, count):
        data = bytes(self.input[:count])
        del self.input[:count]
        self.in_waiting = len(self.input)
        return data

    def write(self, data):
        self.output.extend(data)
        return len(data)


class Camera:
    def __init__(self, reads=None, error=None):
        self.reads = 0
        self.error = error
        self._frames = iter(reads or [object()])

    def read(self):
        self.reads += 1
        if self.error is not None:
            raise self.error
        return True, next(self._frames)


def frames(port):
    return parse_frames(bytearray(), port.output)


def test_only_color_and_circle_modes_require_fill_light():
    assert main._requires_fill_light(CMD_START_COLOR)
    assert main._requires_fill_light(CMD_START_CIRCLE)
    assert not main._requires_fill_light(CMD_START_QR)
    assert not main._requires_fill_light(CMD_START_DISK_CENTER)
    assert not main._requires_fill_light(CMD_STOP)


def test_detection_log_is_emitted_only_when_result_changes(capsys):
    state = main.make_service_state()
    result = {
        "detections": [
            {"type": 2, "center": [10, 20], "confidence": 0.9012, "measured": True, "support_count": 3}
        ]
    }

    main._log_detection_result(state, CMD_START_COLOR, result)
    main._log_detection_result(state, CMD_START_COLOR, result)
    changed = {"detections": [{**result["detections"][0], "center": [11, 20]}]}
    main._log_detection_result(state, CMD_START_COLOR, changed)

    lines = [line for line in capsys.readouterr().out.splitlines() if line.startswith("vision_result")]
    assert len(lines) == 2
    assert '"center":[10,20]' in lines[0]
    assert '"center":[11,20]' in lines[1]


def test_qr_detection_log_contains_status_and_code(capsys):
    state = main.make_service_state()
    main._log_detection_result(
        state,
        CMD_START_QR,
        {"raw_code": "156+123+516+231", "status": "FIRST_DETECTED", "code": "156+123+516+231"},
    )

    output = capsys.readouterr().out
    assert "vision_result mode=二维码检测" in output
    assert '"status":"FIRST_DETECTED"' in output
    assert '"code":"156+123+516+231"' in output


def test_visual_detection_waits_for_fill_light_settlement():
    assert not main._visual_detection_ready(CMD_START_COLOR, True, 10.3, 10.0)
    assert main._visual_detection_ready(CMD_START_COLOR, True, 10.3, 10.3)
    assert main._visual_detection_ready(CMD_START_CIRCLE, False, None, 10.0)
    assert main._visual_detection_ready(CMD_START_QR, True, 10.3, 10.0)
    assert main._visual_detection_ready(CMD_START_DISK_CENTER, True, 10.3, 10.0)


def test_start_period_stop_and_session_replacement(monkeypatch):
    port, state = Port(), main.make_service_state()
    calls = []
    monkeypatch.setattr(main, "reset_advance_tracking", lambda: calls.append("vision"))
    monkeypatch.setattr(main, "reset_qr_tracking", lambda: calls.append("qr"))
    main.handle_command(port, state, CMD_START_COLOR, 7, b"\x28\x00")
    assert (state["mode"], state["session"], state["period_ms"]) == (CMD_START_COLOR, 7, 40)
    assert frames(port) == [(CMD_ACK, 7, bytes((CMD_START_COLOR, ACK_OK)))]
    port.output.clear()
    main.handle_command(port, state, CMD_START_COLOR, 8, b"\x00\x00")
    assert state["session"] == 7
    assert frames(port) == [(CMD_ACK, 8, bytes((CMD_START_COLOR, ACK_BAD_PERIOD)))]
    port.output.clear()
    main.handle_command(port, state, CMD_STOP, 8, b"")
    assert state["mode"] == 0 and calls == ["vision", "vision", "qr"]
    assert frames(port) == [(CMD_ACK, 8, bytes((CMD_STOP, ACK_OK)))]


def test_qr_start_resets_qr_tracking(monkeypatch):
    port, state = Port(), main.make_service_state()
    calls = []
    monkeypatch.setattr(main, "reset_advance_tracking", lambda: calls.append("vision"))
    monkeypatch.setattr(main, "reset_qr_tracking", lambda: calls.append("qr"))
    main.handle_command(port, state, CMD_START_QR, 7, b"\x28\x00")
    assert state["mode"] == CMD_START_QR
    assert calls == ["vision", "qr"]


def test_old_inference_result_is_dropped_after_session_change(monkeypatch):
    port, state = Port(), main.make_service_state()
    state.update(mode=CMD_START_COLOR, session=1, period_ms=1)
    cameras = {"qr": Camera(), "vision": Camera()}
    monkeypatch.setattr(main, "advance_detect_color", lambda frame: {"detections": []})
    monkeypatch.setattr(main, "poll_commands", lambda p, s: s.update(session=2))
    main.run_detection(port, cameras, state, 1.0)
    assert port.output == b""


def test_result_payload_is_truncated_to_stm32_limit():
    result = {
        "detections": [
            {"type": index, "center": [index, index + 1], "confidence": 1.0, "measured": True, "support_count": 2}
            for index in range(main.MAX_TARGETS + 3)
        ]
    }
    payload = main._target_payload(result)
    assert payload[0] == main.MAX_TARGETS
    assert len(payload) == 1 + main.MAX_TARGETS * 8


def test_period_controls_camera_reads_and_detection(monkeypatch):
    port, state = Port(), main.make_service_state()
    state.update(mode=CMD_START_COLOR, session=4, period_ms=40, last_run=-1.0)
    camera = Camera(reads=[object(), object()])
    cameras = {"qr": Camera(), "vision": camera}
    results = iter([{"detections": [{"type": 1, "center": [0, 0], "confidence": 1.0, "measured": True, "support_count": 1}]}, {"detections": [{"type": 1, "center": [1, 0], "confidence": 1.0, "measured": True, "support_count": 1}]}])
    monkeypatch.setattr(main, "advance_detect_color", lambda frame: next(results))

    for now in (0.0, 0.01, 0.02, 0.041):
        main.run_detection(port, cameras, state, now)

    sent = [frame for frame in frames(port) if frame[0] == CMD_COLOR_RESULT]
    assert camera.reads == 2 and len(sent) == 2
    assert int.from_bytes(sent[-1][2][2:4], "little", signed=True) == 1


def test_qr_uses_qr_camera_and_sends_only_confirmed_code(monkeypatch):
    port, state = Port(), main.make_service_state()
    state.update(mode=CMD_START_QR, session=4, period_ms=1, last_run=-1.0)
    qr_camera, vision_camera = Camera(), Camera()
    code = "156+123+516+231"
    monkeypatch.setattr(main, "advance_detect_qr", lambda frame: {"raw_code": code, "code": code, "status": "FIRST_DETECTED"})
    main.run_detection(port, {"qr": qr_camera, "vision": vision_camera}, state, 0.0)
    assert qr_camera.reads == 1 and vision_camera.reads == 0
    assert frames(port) == [(CMD_QR_RESULT, 4, code.encode("ascii"))]
    assert len(code.encode("ascii")) == TASK_CODE_LENGTH


def test_qr_does_not_send_without_code_or_with_invalid_payload(monkeypatch):
    port, state = Port(), main.make_service_state()
    state.update(mode=CMD_START_QR, session=4, period_ms=1, last_run=-1.0)
    qr_camera = Camera(reads=[object(), object()])
    results = iter([{"raw_code": None, "code": None, "status": "MISSING"}, {"raw_code": "bad", "code": "bad", "status": "INVALID"}])
    monkeypatch.setattr(main, "advance_detect_qr", lambda frame: next(results))
    main.run_detection(port, {"qr": qr_camera, "vision": Camera()}, state, 0.0)
    main.run_detection(port, {"qr": qr_camera, "vision": Camera()}, state, 0.002)
    assert port.output == b""


def test_qr_camera_and_detector_failures_do_not_exit(monkeypatch):
    port, state = Port(), main.make_service_state()
    state.update(mode=CMD_START_QR, session=4, period_ms=1, last_run=-1.0)
    main.run_detection(port, {"qr": Camera(error=RuntimeError("camera")), "vision": Camera()}, state, 0.0)
    qr_camera = Camera()
    monkeypatch.setattr(main, "advance_detect_qr", lambda frame: (_ for _ in ()).throw(RuntimeError("detector")))
    main.run_detection(port, {"qr": qr_camera, "vision": Camera()}, state, 0.002)
    assert port.output == b"" and qr_camera.reads == 1


def test_start_competition_sends_once_and_allows_retry_after_write_failure():
    class Gui:
        def __init__(self):
            self.running = 0
            self.elapsed = []

        def show_running_page(self):
            self.running += 1

        def set_elapsed(self, seconds):
            self.elapsed.append(seconds)

    class FailingPort(Port):
        def write(self, data):
            raise OSError("serial unavailable")

    gui, state = Gui(), main.make_service_state()
    assert not main.start_competition(FailingPort(), state, gui, START_AREA_1, 1.0)
    assert not state.get("competition_started", False)

    port = Port()
    assert not main.start_competition(port, state, gui, 0, 1.5)
    assert main.start_competition(port, state, gui, START_AREA_1, 2.0)
    assert main.start_competition(port, state, gui, START_AREA_1, 3.0)
    assert frames(port) == [(CMD_COMPETITION_START, 0, b"\x01")]
    assert state["start_area"] == START_AREA_1
    assert gui.running == 1 and gui.elapsed == [0]


def test_start_competition_sends_selected_area_2():
    class Gui:
        def show_running_page(self):
            pass

        def set_elapsed(self, seconds):
            assert seconds == 0

    port, state = Port(), main.make_service_state()
    assert main.start_competition(port, state, Gui(), START_AREA_2, 1.0)
    assert frames(port) == [(CMD_COMPETITION_START, 0, b"\x02")]


def test_qr_result_updates_gui_once_and_keeps_previous_value_after_missing(monkeypatch):
    port, state = Port(), main.make_service_state()
    state.update(mode=CMD_START_QR, session=4, period_ms=1, last_run=-1.0)
    qr_camera = Camera(reads=[object(), object()])
    code = "156+123+516+231"
    results = iter([
        {"raw_code": code, "code": code, "status": "FIRST_DETECTED"},
        {"raw_code": None, "code": None, "status": "MISSING"},
    ])
    updates = []
    monkeypatch.setattr(main, "advance_detect_qr", lambda frame: next(results))

    main.run_detection(port, {"qr": qr_camera, "vision": Camera()}, state, 0.0, updates.append)
    main.run_detection(port, {"qr": qr_camera, "vision": Camera()}, state, 0.002, updates.append)

    assert updates == [code]
    assert frames(port) == [(CMD_QR_RESULT, 4, code.encode("ascii"))]


def test_visual_page_action_uses_mode_and_session_changes():
    assert main.visual_page_action((0, 0), (CMD_START_QR, 1)) == "show_camera"
    assert main.visual_page_action((0, 0), (CMD_START_COLOR, 1)) == "show_camera"
    assert main.visual_page_action((CMD_START_COLOR, 1), (0, 2)) == "show_running"
    assert main.visual_page_action((CMD_START_COLOR, 1), (CMD_START_COLOR, 2)) == "show_camera"
    assert main.visual_page_action((CMD_START_COLOR, 1), (CMD_START_COLOR, 1)) is None
    assert main.visual_page_action((0, 0), (99, 1)) is None


def test_preview_callback_receives_detection_frame_after_result_send(monkeypatch):
    port, state = Port(), main.make_service_state()
    state.update(mode=CMD_START_COLOR, session=4, period_ms=1, last_run=-1.0)
    frame = object()
    seen = []
    monkeypatch.setattr(main, "advance_detect_color", lambda detected_frame: {
        "detections": [{"type": 1, "center": [0, 0], "confidence": 1.0, "measured": True, "support_count": 1}]
    })

    main.run_detection(
        port,
        {"qr": Camera(), "vision": Camera(reads=[frame])},
        state,
        0.0,
        preview_callback=lambda preview_frame, mode, result: seen.append((preview_frame, mode, result, bytes(port.output))),
    )

    assert seen[0][0] is frame
    assert seen[0][1] == CMD_START_COLOR
    assert seen[0][2]["detections"][0]["type"] == 1
    assert frames(port)[0][0] == CMD_COLOR_RESULT
    assert seen[0][3] == bytes(port.output)


def test_preview_callback_error_does_not_block_result_send(monkeypatch):
    port, state = Port(), main.make_service_state()
    state.update(mode=CMD_START_COLOR, session=4, period_ms=1, last_run=-1.0)
    monkeypatch.setattr(main, "advance_detect_color", lambda frame: {"detections": []})

    main.run_detection(
        port,
        {"qr": Camera(), "vision": Camera()},
        state,
        0.0,
        preview_callback=lambda *_args: (_ for _ in ()).throw(RuntimeError("preview failed")),
    )

    assert frames(port) == [(CMD_COLOR_RESULT, 4, b"\x00")]


def test_qr_preview_updates_before_task_code_is_confirmed(monkeypatch):
    port, state = Port(), main.make_service_state()
    state.update(mode=CMD_START_QR, session=4, period_ms=1, last_run=-1.0)
    frame = object()
    result = {"raw_code": "156+123+516+231", "code": None, "status": "CONFIRMING"}
    seen = []
    monkeypatch.setattr(main, "advance_detect_qr", lambda detected_frame: result)

    main.run_detection(
        port,
        {"qr": Camera(reads=[frame]), "vision": Camera()},
        state,
        0.0,
        preview_callback=lambda preview_frame, mode, preview_result: seen.append((preview_frame, mode, preview_result)),
    )

    assert port.output == b""
    assert seen == [(frame, CMD_START_QR, result)]


def test_idle_preview_conditions_are_period_limited_without_hardware_side_effects():
    assert not main._should_update_idle_preview(False, 0, True, 1.0, 0.0)
    assert not main._should_update_idle_preview(True, CMD_START_COLOR, True, 1.0, 0.0)
    assert not main._should_update_idle_preview(True, 0, False, 1.0, 0.0)
    assert not main._should_update_idle_preview(True, 0, True, 1.099, 1.0)
    assert main._should_update_idle_preview(True, 0, True, 1.1, 1.0)

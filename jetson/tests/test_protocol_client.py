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
    CMD_QR_RESULT,
    CMD_START_COLOR,
    CMD_START_QR,
    CMD_STOP,
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

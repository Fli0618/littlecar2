import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "src"))

import main
from protocol.commands import ACK_BAD_PERIOD, ACK_OK, CMD_ACK, CMD_COLOR_RESULT, CMD_START_COLOR, CMD_STOP
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


def frames(port):
    return parse_frames(bytearray(), port.output)


def test_start_period_stop_and_session_replacement(monkeypatch):
    port, state = Port(), main.make_service_state()
    calls = []
    monkeypatch.setattr(main, "reset_advance_tracking", lambda: calls.append(True))
    main.handle_command(port, state, CMD_START_COLOR, 7, b"\x28\x00")
    assert (state["mode"], state["session"], state["period_ms"]) == (CMD_START_COLOR, 7, 40)
    assert frames(port) == [(CMD_ACK, 7, bytes((CMD_START_COLOR, ACK_OK)))]
    port.output.clear()
    main.handle_command(port, state, CMD_START_COLOR, 8, b"\x00\x00")
    assert state["session"] == 7
    assert frames(port) == [(CMD_ACK, 8, bytes((CMD_START_COLOR, ACK_BAD_PERIOD)))]
    port.output.clear()
    main.handle_command(port, state, CMD_STOP, 8, b"")
    assert state["mode"] == 0 and calls == [True, True]
    assert frames(port) == [(CMD_ACK, 8, bytes((CMD_STOP, ACK_OK)))]


def test_old_inference_result_is_dropped_after_session_change(monkeypatch):
    port, state = Port(), main.make_service_state()
    state.update(mode=CMD_START_COLOR, session=1, period_ms=1)
    camera = type("Camera", (), {"read": lambda self: (True, object())})()
    monkeypatch.setattr(main, "advance_detect_color", lambda frame: {"detections": []})
    monkeypatch.setattr(main, "poll_commands", lambda p, s: s.update(session=2))
    main.run_detection(port, camera, state, 1.0)
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


def test_period_sends_only_latest_result_without_queue(monkeypatch):
    port, state = Port(), main.make_service_state()
    state.update(mode=CMD_START_COLOR, session=4, period_ms=40, last_send=-1.0)
    camera = type("Camera", (), {"read": lambda self: (True, object())})()
    results = [{"detections": [{"type": 1, "center": [x, 0], "confidence": 1.0, "measured": True, "support_count": 1}]} for x in range(4)]
    result_iterator = iter(results)
    monkeypatch.setattr(main, "advance_detect_color", lambda frame: next(result_iterator))
    monkeypatch.setattr(main, "poll_commands", lambda p, s: None)

    for now in (0.0, 0.01, 0.02, 0.041):
        main.run_detection(port, camera, state, now)

    sent = [frame for frame in frames(port) if frame[0] == CMD_COLOR_RESULT]
    assert len(sent) == 2
    assert int.from_bytes(sent[-1][2][2:4], "little", signed=True) == 3

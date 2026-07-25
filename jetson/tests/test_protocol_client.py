import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "src"))

import main
from protocol.commands import ACK_BAD_PERIOD, ACK_OK, CMD_ACK, CMD_START_COLOR, CMD_STOP
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

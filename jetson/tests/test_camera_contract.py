import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "src"))

import main
from protocol.commands import CMD_START_COLOR


class FakeCapture:
    def __init__(
        self,
        *,
        opened=True,
        width=640,
        height=480,
        frame=None,
        set_result=True,
        read_result=True,
    ):
        self.opened = opened
        self.width = width
        self.height = height
        self.frame = np.zeros((height, width, 3), dtype=np.uint8) if frame is None else frame
        self.set_result = set_result
        self.read_result = read_result
        self.set_calls = []
        self.released = False

    def isOpened(self):
        return self.opened

    def set(self, prop, value):
        self.set_calls.append((prop, value))
        return self.set_result

    def get(self, prop):
        if prop == main.cv2.CAP_PROP_FRAME_WIDTH:
            return self.width
        return self.height

    def read(self):
        return self.read_result, self.frame if self.read_result else None

    def release(self):
        self.released = True


class Port:
    def __init__(self):
        self.in_waiting = 0
        self.output = bytearray()

    def read(self, _count):
        return b""

    def write(self, data):
        self.output.extend(data)
        return len(data)


def test_open_camera_sets_and_validates_640x480(monkeypatch):
    capture = FakeCapture()
    monkeypatch.setattr(main.cv2, "VideoCapture", lambda *_args: capture)

    assert main.open_camera("/dev/video2", 640, 480, main.CAMERA_VISION) is capture
    assert capture.set_calls == [
        (main.cv2.CAP_PROP_FRAME_WIDTH, 640),
        (main.cv2.CAP_PROP_FRAME_HEIGHT, 480),
    ]
    assert not capture.released


@pytest.mark.parametrize(
    "capture",
    [
        FakeCapture(set_result=False),
        FakeCapture(width=1280, height=720),
        FakeCapture(frame=np.zeros((720, 1280, 3), dtype=np.uint8)),
        FakeCapture(read_result=False),
    ],
)
def test_open_camera_releases_on_contract_failure(monkeypatch, capture):
    monkeypatch.setattr(main.cv2, "VideoCapture", lambda *_args: capture)

    with pytest.raises(RuntimeError):
        main.open_camera("/dev/video2", 640, 480, main.CAMERA_VISION)
    assert capture.released


def test_open_cameras_releases_first_camera_when_second_fails(monkeypatch):
    qr_capture = FakeCapture()
    vision_capture = FakeCapture(opened=False)
    captures = iter((qr_capture, vision_capture))
    monkeypatch.setattr(main.cv2, "VideoCapture", lambda *_args: next(captures))

    with pytest.raises(RuntimeError):
        main.open_cameras()
    assert qr_capture.released
    assert vision_capture.released


def test_invalid_runtime_frame_skips_detection_and_serial_send(monkeypatch):
    state = main.make_service_state()
    state.update(mode=CMD_START_COLOR, session=3, period_ms=1, last_run=-1.0)
    port = Port()
    detected = []
    monkeypatch.setattr(main, "advance_detect_color", lambda _frame: detected.append(True))
    cameras = {
        main.CAMERA_QR: FakeCapture(),
        main.CAMERA_VISION: FakeCapture(frame=np.zeros((720, 1280, 3), dtype=np.uint8)),
    }

    main.run_detection(port, cameras, state, 0.0)

    assert not detected
    assert port.output == b""

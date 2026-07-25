"""STM32 驱动的单线程持续视觉服务。"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parent
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))

from protocol.commands import (
    ACK_BAD_CMD, ACK_BAD_LENGTH, ACK_BAD_PERIOD, ACK_OK,
    CMD_ACK, CMD_CIRCLE_RESULT, CMD_COLOR_RESULT, CMD_DISK_CENTER_RESULT,
    CMD_START_CIRCLE, CMD_START_COLOR, CMD_START_DISK_CENTER, CMD_STOP,
    DISK_CENTER_NO_TARGET, DISK_CENTER_OK, START_COMMANDS,
)
from protocol.frame import pack_frame, parse_frames
from vision import advance_detect_circle, advance_detect_color, advance_detect_disk_center, reset_advance_tracking

SERIAL_PORT = "/dev/ttyTHS1"
SERIAL_BAUDRATE = 115200
CAMERA_ID = 1
DEFAULT_PERIOD_MS = 40
MAX_TARGETS = 8


def make_service_state() -> dict[str, object]:
    return {"mode": 0, "session": 0, "period_ms": DEFAULT_PERIOD_MS, "last_send": 0.0, "rx": bytearray()}


def _write_ack(port: object, session: int, command: int, status: int) -> None:
    port.write(pack_frame(CMD_ACK, session, bytes((command, status))))


def handle_command(port: object, state: dict[str, object], command: int, session: int, payload: bytes) -> None:
    if command in START_COMMANDS:
        if len(payload) != 2:
            _write_ack(port, session, command, ACK_BAD_LENGTH)
            return
        period_ms = int.from_bytes(payload, "little")
        if period_ms == 0:
            _write_ack(port, session, command, ACK_BAD_PERIOD)
            return
        reset_advance_tracking()
        state.update(mode=command, session=session, period_ms=period_ms, last_send=0.0)
        _write_ack(port, session, command, ACK_OK)
    elif command == CMD_STOP:
        if payload:
            _write_ack(port, session, command, ACK_BAD_LENGTH)
            return
        reset_advance_tracking()
        state.update(mode=0, session=session, last_send=0.0)
        _write_ack(port, session, command, ACK_OK)
    else:
        _write_ack(port, session, command, ACK_BAD_CMD)


def poll_commands(port: object, state: dict[str, object]) -> None:
    pending = getattr(port, "in_waiting", 0)
    data = port.read(pending) if pending else b""
    for command, session, payload in parse_frames(state["rx"], data):
        handle_command(port, state, command, session, payload)


def _target_payload(result: dict[str, object]) -> bytes:
    targets = result.get("detections", [])
    payload = bytearray((min(len(targets), MAX_TARGETS),))
    for item in targets[:MAX_TARGETS]:
        x, y = item["center"]
        confidence = max(0, min(255, int(round(float(item["confidence"]) * 255))))
        payload.extend((int(item["type"]) & 0xFF,))
        payload.extend(int(x).to_bytes(2, "little", signed=True))
        payload.extend(int(y).to_bytes(2, "little", signed=True))
        payload.extend((confidence, int(bool(item.get("measured", False))), int(item.get("support_count", 0)) & 0xFF))
    return bytes(payload)


def _disk_payload(result: dict[str, object]) -> bytes:
    support_count = int(result.get("support_count", 0))
    measured_count = int(result.get("measured_count", 0))
    if support_count == 0:
        status, x, y = DISK_CENTER_NO_TARGET, 0, 0
    else:
        status = DISK_CENTER_OK
        x, y = result["center"]
    return bytes((status,)) + int(x).to_bytes(2, "little", signed=True) + int(y).to_bytes(2, "little", signed=True) + bytes((support_count & 0xFF, measured_count & 0xFF))


def run_detection(port: object, camera: object, state: dict[str, object], now: float) -> None:
    mode = int(state["mode"])
    if not mode:
        return
    ok, frame = camera.read()
    if not ok:
        return
    session = int(state["session"])
    if mode == CMD_START_COLOR:
        result, response = advance_detect_color(frame), CMD_COLOR_RESULT
    elif mode == CMD_START_CIRCLE:
        result, response = advance_detect_circle(frame), CMD_CIRCLE_RESULT
    else:
        result, response = advance_detect_disk_center(frame), CMD_DISK_CENTER_RESULT
    poll_commands(port, state)
    if int(state["mode"]) != mode or int(state["session"]) != session:
        return
    if now - float(state["last_send"]) < int(state["period_ms"]) / 1000.0:
        return
    payload = _disk_payload(result) if response == CMD_DISK_CENTER_RESULT else _target_payload(result)
    port.write(pack_frame(response, session, payload))
    state["last_send"] = now


def main() -> None:
    import serial

    state = make_service_state()
    port = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=0, write_timeout=0)
    camera = cv2.VideoCapture(CAMERA_ID)
    if not camera.isOpened():
        port.close()
        raise RuntimeError(f"cannot open camera {CAMERA_ID}")
    try:
        while True:
            poll_commands(port, state)
            run_detection(port, camera, state, time.monotonic())
    finally:
        reset_advance_tracking()
        camera.release()
        port.close()


if __name__ == "__main__":
    main()

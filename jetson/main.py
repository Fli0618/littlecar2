"""STM32 驱动的单线程常驻视觉服务。"""

from __future__ import annotations

import time

import cv2
import numpy as np

from protocol.commands import (
    ACK_BAD_CMD,
    ACK_BAD_LENGTH,
    ACK_BAD_PERIOD,
    ACK_OK,
    CMD_ACK,
    CMD_CIRCLE_RESULT,
    CMD_COLOR_RESULT,
    CMD_DISK_CENTER_RESULT,
    CMD_QR_RESULT,
    CMD_START_CIRCLE,
    CMD_START_COLOR,
    CMD_START_DISK_CENTER,
    CMD_START_QR,
    CMD_STOP,
    DISK_CENTER_NO_TARGET,
    DISK_CENTER_OK,
    START_COMMANDS,
    TASK_CODE_LENGTH,
)
from protocol.frame import pack_frame, parse_frames
from vision import (
    advance_detect_circle,
    advance_detect_color,
    advance_detect_disk_center,
    advance_detect_qr,
    configure_model_backend,
    detect_circle,
    detect_color,
    reset_advance_tracking,
    reset_qr_tracking,
)

SERIAL_PORT = "/dev/ttyTHS1"
SERIAL_BAUDRATE = 115200
CAMERA_QR_DEVICE = "/dev/video0"
CAMERA_VISION_DEVICE = "/dev/video1"
MODEL_BACKEND = "pt"  # "pt" or "engine"
DEFAULT_PERIOD_MS = 40
MAX_TARGETS = 8
IDLE_SLEEP_SECONDS = 0.001
MODEL_WARMUP_FRAME_SIZE = 640


def make_service_state() -> dict[str, object]:
    return {"mode": 0, "session": 0, "period_ms": DEFAULT_PERIOD_MS, "last_run": float("-inf"), "rx": bytearray()}


def warmup_vision_models() -> None:
    """启动时加载并预热颜色、数字圆环两个 YOLO 模型。"""
    frame = np.zeros((MODEL_WARMUP_FRAME_SIZE, MODEL_WARMUP_FRAME_SIZE, 3), dtype=np.uint8)

    started = time.perf_counter()
    detect_color(frame)
    color_ms = (time.perf_counter() - started) * 1000.0

    started = time.perf_counter()
    detect_circle(frame)
    circle_ms = (time.perf_counter() - started) * 1000.0

    print(
        f"vision models ready backend={MODEL_BACKEND} "
        f"color_warmup_ms={color_ms:.1f} circle_warmup_ms={circle_ms:.1f}",
        flush=True,
    )


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
        if command == CMD_START_QR:
            reset_qr_tracking()
        state.update(mode=command, session=session, period_ms=period_ms, last_run=float("-inf"))
        _write_ack(port, session, command, ACK_OK)
    elif command == CMD_STOP:
        if payload:
            _write_ack(port, session, command, ACK_BAD_LENGTH)
            return
        reset_advance_tracking()
        reset_qr_tracking()
        state.update(mode=0, session=session, last_run=float("-inf"))
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


def _qr_payload(code: object) -> bytes | None:
    if not isinstance(code, str):
        return None
    try:
        payload = code.encode("ascii")
    except UnicodeEncodeError:
        return None
    return payload if len(payload) == TASK_CODE_LENGTH else None


def run_detection(port: object, cameras: dict[str, object], state: dict[str, object], now: float) -> None:
    mode = int(state["mode"])
    if not mode or now - float(state["last_run"]) < int(state["period_ms"]) / 1000.0:
        return
    state["last_run"] = now
    camera = cameras["qr" if mode == CMD_START_QR else "vision"]
    try:
        ok, frame = camera.read()
    except Exception:
        return
    if not ok:
        return

    session = int(state["session"])
    if mode == CMD_START_QR:
        try:
            result = advance_detect_qr(frame)
        except Exception:
            return
        response = CMD_QR_RESULT
    elif mode == CMD_START_COLOR:
        result, response = advance_detect_color(frame), CMD_COLOR_RESULT
    elif mode == CMD_START_CIRCLE:
        result, response = advance_detect_circle(frame), CMD_CIRCLE_RESULT
    else:
        result, response = advance_detect_disk_center(frame), CMD_DISK_CENTER_RESULT

    poll_commands(port, state)
    if int(state["mode"]) != mode or int(state["session"]) != session:
        return
    if response == CMD_QR_RESULT:
        payload = _qr_payload(result.get("code"))
        if payload is None:
            return
    elif response == CMD_DISK_CENTER_RESULT:
        payload = _disk_payload(result)
    else:
        payload = _target_payload(result)
    port.write(pack_frame(response, session, payload))


def main() -> None:
    import serial

    configure_model_backend(MODEL_BACKEND)
    warmup_vision_models()
    state = make_service_state()
    port = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=0, write_timeout=0)
    qr_camera = cv2.VideoCapture(CAMERA_QR_DEVICE, cv2.CAP_V4L2)
    vision_camera = cv2.VideoCapture(CAMERA_VISION_DEVICE, cv2.CAP_V4L2)
    if not qr_camera.isOpened() or not vision_camera.isOpened():
        qr_camera.release()
        vision_camera.release()
        port.close()
        raise RuntimeError(f"cannot open cameras qr={CAMERA_QR_DEVICE}, vision={CAMERA_VISION_DEVICE}")
    cameras = {"qr": qr_camera, "vision": vision_camera}
    try:
        while True:
            poll_commands(port, state)
            run_detection(port, cameras, state, time.monotonic())
            time.sleep(IDLE_SLEEP_SECONDS)
    finally:
        reset_advance_tracking()
        reset_qr_tracking()
        qr_camera.release()
        vision_camera.release()
        port.close()


if __name__ == "__main__":
    main()

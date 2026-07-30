"""STM32 驱动的单线程常驻视觉服务。"""

from __future__ import annotations

import time
from collections.abc import Callable

import cv2
import numpy as np

from protocol.commands import (
    ACK_BAD_CMD,
    ACK_BAD_LENGTH,
    ACK_BAD_PERIOD,
    ACK_OK,
    CMD_ACK,
    CMD_CIRCLE_RESULT,
    CMD_COMPETITION_START,
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
    VALID_START_AREAS,
    TASK_CODE_LENGTH,
)
from protocol.frame import pack_frame, parse_frames
from ui import CompetitionGUI
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
    render_camera_preview,
)

SERIAL_PORT = "/dev/ttyTHS1"
SERIAL_BAUDRATE = 115200
CAMERA_QR_DEVICE = "/dev/video0"
CAMERA_VISION_DEVICE = "/dev/video1"
MODEL_BACKEND = "engine"  # "pt" or "engine"
DEFAULT_PERIOD_MS = 40
MAX_TARGETS = 8
MODEL_WARMUP_FRAME_SIZE = 640
SERVICE_POLL_INTERVAL_MS = 1
ELAPSED_UPDATE_INTERVAL_MS = 200
PWM_PIN = 33
PWM_FREQUENCY_HZ = 10000
PWM_DUTY_CYCLE_PERCENT = 50.0
LIGHT_SETTLE_SECONDS = 0.3
ENABLE_CAMERA_PREVIEW_UI = True
CAMERA_PREVIEW_PERIOD_MS = 100
QR_AIM_OFFSET_X_PX = 0
QR_AIM_OFFSET_Y_PX = 0
VISION_AIM_OFFSET_X_PX = 0
VISION_AIM_OFFSET_Y_PX = 0
_FILL_LIGHT_MODES = frozenset((CMD_START_COLOR, CMD_START_CIRCLE))


def _requires_fill_light(mode: int) -> bool:
    """仅颜色物料和同心圆检测会话需要补光灯。"""
    return mode in _FILL_LIGHT_MODES


def _visual_detection_ready(mode: int, fill_light_active: bool, ready_at: float | None, now: float) -> bool:
    """补光灯稳定前继续轮询命令，但暂不执行对应的视觉推理。"""
    return not (
        _requires_fill_light(mode)
        and fill_light_active
        and ready_at is not None
        and now < ready_at
    )


def make_service_state() -> dict[str, object]:
    return {
        "mode": 0,
        "session": 0,
        "period_ms": DEFAULT_PERIOD_MS,
        "last_run": float("-inf"),
        "last_task_code": "",
        "rx": bytearray(),
    }


def _visual_key(state: dict[str, object]) -> tuple[int, int]:
    return int(state["mode"]), int(state["session"])


def visual_page_action(previous_key: tuple[int, int], current_key: tuple[int, int]) -> str | None:
    """根据有效视觉会话变化返回需要执行的 GUI 页面操作。"""
    if previous_key == current_key:
        return None
    if current_key[0] in START_COMMANDS:
        return "show_camera"
    if current_key[0] == 0:
        return "show_running"
    return None


def _should_update_idle_preview(
    enabled: bool,
    mode: int,
    camera_page_visible: bool,
    now: float,
    last_preview_at: float,
) -> bool:
    return (
        enabled
        and mode == 0
        and camera_page_visible
        and (now - last_preview_at) * 1000.0 >= CAMERA_PREVIEW_PERIOD_MS
    )


def _preview_mode_text(mode: int) -> str:
    return {
        CMD_START_QR: "二维码检测",
        CMD_START_COLOR: "颜色检测",
        CMD_START_CIRCLE: "同心圆检测",
        CMD_START_DISK_CENTER: "物料盘中心检测",
    }.get(mode, "手动相机预览")


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


def run_detection(
    port: object,
    cameras: dict[str, object],
    state: dict[str, object],
    now: float,
    task_code_callback: Callable[[str], None] | None = None,
    preview_callback: Callable[[np.ndarray, int, dict[str, object]], None] | None = None,
) -> None:
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
        code = result.get("code")
        payload = _qr_payload(code)
        if payload is not None:
            port.write(pack_frame(response, session, payload))
            if task_code_callback is not None and code != state["last_task_code"]:
                task_code_callback(str(code))
                state["last_task_code"] = code
    elif response == CMD_DISK_CENTER_RESULT:
        payload = _disk_payload(result)
        port.write(pack_frame(response, session, payload))
    else:
        payload = _target_payload(result)
        port.write(pack_frame(response, session, payload))
    if preview_callback is not None:
        try:
            preview_callback(frame, mode, result)
        except Exception as error:
            print(f"camera preview update failed: {error}", flush=True)


def start_competition(
    port: object,
    state: dict[str, object],
    gui: CompetitionGUI,
    start_area: int,
    now: float,
) -> bool:
    """发送启动帧；串口写入成功后才切换比赛显示页面。"""
    if start_area not in VALID_START_AREAS:
        return False
    if bool(state.get("competition_started", False)):
        return True
    try:
        port.write(pack_frame(CMD_COMPETITION_START, 0, bytes((start_area,))))
    except Exception as error:
        print(f"competition start send failed: {error}", flush=True)
        return False
    state["competition_started"] = True
    state["competition_started_at"] = now
    state["start_area"] = start_area
    gui.show_running_page()
    gui.set_elapsed(0)
    return True


def main() -> None:
    import serial
    import tkinter as tk

    gpio = None
    pwm = None
    gpio_configured = False
    fill_light_active = False
    fill_light_unavailable = False
    fill_light_ready_at: float | None = None

    def stop_fill_light() -> None:
        nonlocal pwm, fill_light_active, fill_light_ready_at
        if not fill_light_active and pwm is None:
            return
        if pwm is not None:
            try:
                pwm.ChangeDutyCycle(0)
                pwm.stop()
            except Exception as error:
                print(f"fill light stop failed: {error}", flush=True)
            finally:
                pwm = None
        if gpio_configured and gpio is not None:
            try:
                gpio.output(PWM_PIN, gpio.LOW)
            except Exception as error:
                print(f"fill light pin reset failed: {error}", flush=True)
        fill_light_active = False
        fill_light_ready_at = None

    def enable_fill_light(now: float) -> None:
        nonlocal pwm, fill_light_active, fill_light_unavailable, fill_light_ready_at
        if fill_light_active or fill_light_unavailable:
            return
        try:
            pwm = gpio.PWM(PWM_PIN, PWM_FREQUENCY_HZ)
            pwm.start(PWM_DUTY_CYCLE_PERCENT)
            fill_light_active = True
            fill_light_ready_at = now + LIGHT_SETTLE_SECONDS
            print(
                f"fill light enabled pin={PWM_PIN} frequency_hz={PWM_FREQUENCY_HZ} "
                f"duty_cycle_percent={PWM_DUTY_CYCLE_PERCENT:.1f}",
                flush=True,
            )
        except Exception as error:
            print(
                "fill light start failed; continuing visual detection without light. "
                "Check Jetson-IO PWM configuration for Pin 33 and GPIO permissions: "
                f"{error}",
                flush=True,
            )
            fill_light_unavailable = True
            stop_fill_light()

    def sync_fill_light(now: float) -> None:
        if _requires_fill_light(int(state["mode"])):
            enable_fill_light(now)
        else:
            stop_fill_light()

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
        import Jetson.GPIO as jetson_gpio

        gpio = jetson_gpio
        gpio.setmode(gpio.BOARD)
        gpio.setup(PWM_PIN, gpio.OUT, initial=gpio.LOW)
        gpio_configured = True
        # 补光灯独立供电时，控制地必须与 Jetson GND 共地。
    except Exception as error:
        fill_light_unavailable = True
        print(
            "fill light initialization failed; continuing visual detection without light. "
            "Check Jetson-IO PWM configuration for Pin 33 and GPIO permissions: "
            f"{error}",
            flush=True,
        )

    root = tk.Tk()
    gui = CompetitionGUI(root, camera_preview_enabled=ENABLE_CAMERA_PREVIEW_UI)
    last_manual_preview_at = float("-inf")

    def apply_visual_page_action(previous_key: tuple[int, int], current_key: tuple[int, int]) -> None:
        if not ENABLE_CAMERA_PREVIEW_UI:
            return
        action = visual_page_action(previous_key, current_key)
        if action == "show_camera":
            gui.show_camera_page()
        elif action == "show_running":
            gui.show_running_page()

    def update_detection_preview(frame_bgr: np.ndarray, mode: int, result: dict[str, object]) -> None:
        if not ENABLE_CAMERA_PREVIEW_UI or not gui.is_camera_page_visible():
            return
        aim_offset = (
            (QR_AIM_OFFSET_X_PX, QR_AIM_OFFSET_Y_PX)
            if mode == CMD_START_QR
            else (VISION_AIM_OFFSET_X_PX, VISION_AIM_OFFSET_Y_PX)
        )
        try:
            status_text = _preview_mode_text(mode)
            gui.set_camera_frame(
                render_camera_preview(
                    frame_bgr,
                    mode,
                    result,
                    aim_offset=aim_offset,
                    status_text=status_text,
                ),
                status_text=status_text,
            )
        except Exception as error:
            print(f"camera preview rendering failed: {error}", flush=True)

    def update_idle_preview(now: float) -> None:
        nonlocal last_manual_preview_at
        if not _should_update_idle_preview(
            ENABLE_CAMERA_PREVIEW_UI,
            int(state["mode"]),
            gui.is_camera_page_visible(),
            now,
            last_manual_preview_at,
        ):
            return
        last_manual_preview_at = now
        try:
            ok, frame = vision_camera.read()
            if not ok:
                return
            gui.set_camera_frame(
                render_camera_preview(
                    frame,
                    0,
                    aim_offset=(VISION_AIM_OFFSET_X_PX, VISION_AIM_OFFSET_Y_PX),
                    status_text="手动相机预览",
                ),
                status_text="手动相机预览",
            )
        except Exception as error:
            print(f"idle camera preview failed: {error}", flush=True)

    def update_elapsed() -> None:
        started_at = state.get("competition_started_at")
        if isinstance(started_at, float):
            gui.set_elapsed(int(time.monotonic() - started_at))
            root.after(ELAPSED_UPDATE_INTERVAL_MS, update_elapsed)

    def on_start(start_area: int) -> bool:
        started = start_competition(port, state, gui, start_area, time.monotonic())
        if started:
            update_elapsed()
        return started

    def service_tick() -> None:
        # 场地标注页只改变 GUI 视图；服务轮询始终持续运行。
        try:
            previous_visual_key = _visual_key(state)
            poll_commands(port, state)
            current_visual_key = _visual_key(state)
            apply_visual_page_action(previous_visual_key, current_visual_key)
            now = time.monotonic()
            sync_fill_light(now)
            if _visual_detection_ready(int(state["mode"]), fill_light_active, fill_light_ready_at, now):
                preview_callback = update_detection_preview if (
                    ENABLE_CAMERA_PREVIEW_UI and gui.is_camera_page_visible()
                ) else None
                run_detection(port, cameras, state, now, gui.set_task_code, preview_callback)
            apply_visual_page_action(current_visual_key, _visual_key(state))
            sync_fill_light(time.monotonic())
            update_idle_preview(time.monotonic())
        except Exception:
            gui.close()
            raise
        root.after(SERVICE_POLL_INTERVAL_MS, service_tick)

    gui.set_start_callback(on_start)
    root.after(SERVICE_POLL_INTERVAL_MS, service_tick)
    try:
        gui.run()
    finally:
        stop_fill_light()
        if gpio_configured and gpio is not None:
            try:
                gpio.output(PWM_PIN, gpio.LOW)
            finally:
                gpio.cleanup(PWM_PIN)
        reset_advance_tracking()
        reset_qr_tracking()
        qr_camera.release()
        vision_camera.release()
        port.close()


if __name__ == "__main__":
    main()

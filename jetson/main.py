"""STM32 驱动的单线程常驻视觉服务。"""

from __future__ import annotations

import json
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
from ui import CAMERA_QR, CAMERA_VISION, CompetitionGUI
from vision import (
    advance_detect_circle,
    advance_detect_color,
    advance_detect_color_hsv,
    advance_detect_color_hybrid,
    advance_detect_disk_center,
    advance_detect_qr,
    configure_model_backend,
    detect_circle,
    detect_color,
    detect_color_hsv,
    get_hsv_config,
    reset_advance_tracking,
    reset_qr_tracking,
    render_camera_preview,
)

# Jetson 与 STM32 的固定串口链路；修改时需与下位机串口配置保持一致。
SERIAL_PORT = "/dev/ttyTHS1"
SERIAL_BAUDRATE = 115200
# 两路相机的 V4L2 设备路径，二维码任务使用 QR，相机视觉任务使用 VISION。
CAMERA_QR_DEVICE = "/dev/video0"
CAMERA_VISION_DEVICE = "/dev/video2"
# 原始相机帧尺寸契约；STM32 按 640x480 像素坐标解释检测结果，不能改为模型 imgsz。
QR_FRAME_WIDTH = 640
QR_FRAME_HEIGHT = 480
VISION_FRAME_WIDTH = 640
VISION_FRAME_HEIGHT = 480
# 颜色物料只在画面上方 3/4 内检测；同心圆检测使用完整画面。
COLOR_ROI_TOP = 0
COLOR_ROI_BOTTOM_RATIO = 0.75
COLOR_DETECTION_BACKEND = "yolo_hsv"
MODEL_BACKEND = "engine"  # "pt" or "engine"
DEFAULT_PERIOD_MS = 40
MAX_TARGETS = 8
# 仅用于方形模型预热输入，不代表相机原始帧尺寸或 STM32 坐标系。
MODEL_WARMUP_FRAME_SIZE = 640
SERVICE_POLL_INTERVAL_MS = 1
ELAPSED_UPDATE_INTERVAL_MS = 200
PWM_PIN = 32
PWM_FREQUENCY_HZ = 10000
PWM_DUTY_CYCLE_PERCENT = 100.0
LIGHT_SETTLE_SECONDS = 0.3
ENABLE_CAMERA_PREVIEW_UI = True
CAMERA_PREVIEW_PERIOD_MS = 100
# 仅调整 GUI 准星显示位置；不会修改检测结果或 STM32 视觉伺服参考点。
QR_PREVIEW_AIM_OFFSET_X_PX = 0
QR_PREVIEW_AIM_OFFSET_Y_PX = 0
VISION_PREVIEW_AIM_OFFSET_X_PX = 0
VISION_PREVIEW_AIM_OFFSET_Y_PX = 0
_FILL_LIGHT_MODES = frozenset((CMD_START_COLOR, CMD_START_CIRCLE))


def _color_detection_backend() -> Callable[[np.ndarray], dict[str, object]]:
    """返回颜色任务检测器；混合模式先完成 HSV 分类再进入跟踪。"""
    backends: dict[str, Callable[[np.ndarray], dict[str, object]]] = {
        "yolo": advance_detect_color,
        "hsv": advance_detect_color_hsv,
        "yolo_hsv": advance_detect_color_hybrid,
    }
    try:
        return backends[COLOR_DETECTION_BACKEND]
    except KeyError as exc:
        raise ValueError(
            f"unsupported color detection backend: {COLOR_DETECTION_BACKEND!r}; "
            "expected 'yolo', 'hsv' or 'yolo_hsv'"
        ) from exc


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
        "last_detection_log_signature": None,
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


def _frame_size(frame: object) -> tuple[int, int] | None:
    """返回 OpenCV 图像的宽高；非图像对象留给既有轻量测试替身处理。"""
    shape = getattr(frame, "shape", None)
    if not isinstance(shape, tuple) or len(shape) < 2:
        return None
    return int(shape[1]), int(shape[0])


def _camera_frame_size(camera_name: str) -> tuple[int, int]:
    return (
        (QR_FRAME_WIDTH, QR_FRAME_HEIGHT)
        if camera_name == CAMERA_QR
        else (VISION_FRAME_WIDTH, VISION_FRAME_HEIGHT)
    )


def _frame_matches_camera_contract(frame: object, camera_name: str) -> bool:
    """仅允许按启动契约协商成功的原始相机帧进入检测与坐标发送。"""
    actual_size = _frame_size(frame)
    return actual_size is None or actual_size == _camera_frame_size(camera_name)


def _detection_roi_bounds(frame: np.ndarray, mode: int) -> tuple[int, int, int, int] | None:
    """返回检测 ROI 的全局像素边界，格式为 ``(x1, y1, x2, y2)``。"""
    if not isinstance(frame, np.ndarray):
        return None
    height, width = frame.shape[:2]
    if mode == CMD_START_COLOR:
        bottom = max(COLOR_ROI_TOP + 1, min(height, int(height * COLOR_ROI_BOTTOM_RATIO)))
        return 0, COLOR_ROI_TOP, width, bottom
    if mode == CMD_START_CIRCLE:
        return 0, 0, width, height
    return None


def _detection_frame(frame: np.ndarray, mode: int) -> tuple[np.ndarray, tuple[int, int, int, int] | None]:
    """按任务选择模型输入帧，并返回该帧在原图中的 ROI 边界。"""
    bounds = _detection_roi_bounds(frame, mode)
    if bounds is None:
        return frame, None
    x1, y1, x2, y2 = bounds
    return frame[y1:y2, x1:x2], bounds


def _restore_detection_coordinates(
    result: dict[str, object], roi_bounds: tuple[int, int, int, int] | None,
) -> dict[str, object]:
    """将 ROI 内检测结果复制并恢复到原始相机坐标系。"""
    if roi_bounds is None:
        return result
    offset_x, offset_y = roi_bounds[:2]
    if offset_x == 0 and offset_y == 0:
        return result

    detections = result.get("detections")
    if not isinstance(detections, list):
        return result
    restored: list[dict[str, object]] = []
    for item in detections:
        if not isinstance(item, dict):
            continue
        restored_item = dict(item)
        center = item.get("center")
        if isinstance(center, (list, tuple)) and len(center) == 2:
            restored_item["center"] = [int(center[0]) + offset_x, int(center[1]) + offset_y]
        bbox = item.get("bbox")
        if isinstance(bbox, (list, tuple)) and len(bbox) == 4:
            restored_item["bbox"] = [
                int(bbox[0]) + offset_x,
                int(bbox[1]) + offset_y,
                int(bbox[2]) + offset_x,
                int(bbox[3]) + offset_y,
            ]
        restored.append(restored_item)
    return {**result, "detections": restored}


def _release_camera(camera: object | None) -> None:
    if camera is not None:
        try:
            camera.release()
        except Exception:
            pass


def open_camera(device: str, width: int, height: int, camera_name: str) -> object:
    """以 V4L2 固定打开相机，并在启动阶段验证驱动与实际帧尺寸。"""
    camera = cv2.VideoCapture(device, cv2.CAP_V4L2)
    reported_width = 0
    reported_height = 0
    frame_width = 0
    frame_height = 0
    try:
        if not camera.isOpened():
            raise RuntimeError(f"camera open failed name={camera_name} device={device}")
        if not camera.set(cv2.CAP_PROP_FRAME_WIDTH, width) or not camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height):
            raise RuntimeError(f"camera size set failed name={camera_name} device={device} requested={width}x{height}")

        reported_width = int(round(camera.get(cv2.CAP_PROP_FRAME_WIDTH)))
        reported_height = int(round(camera.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        ok, frame = camera.read()
        frame_size = _frame_size(frame) if ok else None
        if frame_size is not None:
            frame_width, frame_height = frame_size
        if (
            not ok
            or frame_size is None
            or (reported_width, reported_height) != (width, height)
            or frame_size != (width, height)
        ):
            raise RuntimeError(
                f"camera size mismatch name={camera_name} device={device} requested={width}x{height} "
                f"reported={reported_width}x{reported_height} frame={frame_width}x{frame_height}"
            )
    except Exception:
        _release_camera(camera)
        raise

    print(
        f"camera_ready name={camera_name} device={device} requested={width}x{height} "
        f"reported={reported_width}x{reported_height} frame={frame_width}x{frame_height}",
        flush=True,
    )
    return camera


def open_cameras() -> dict[str, object]:
    """初始化两台相机；任意一步失败均释放已经打开的相机。"""
    qr_camera: object | None = None
    vision_camera: object | None = None
    try:
        qr_camera = open_camera(CAMERA_QR_DEVICE, QR_FRAME_WIDTH, QR_FRAME_HEIGHT, CAMERA_QR)
        vision_camera = open_camera(
            CAMERA_VISION_DEVICE,
            VISION_FRAME_WIDTH,
            VISION_FRAME_HEIGHT,
            CAMERA_VISION,
        )
        return {CAMERA_QR: qr_camera, CAMERA_VISION: vision_camera}
    except Exception:
        _release_camera(vision_camera)
        _release_camera(qr_camera)
        raise


def _preview_mode_text(mode: int) -> str:
    return {
        CMD_START_QR: "二维码检测",
        CMD_START_COLOR: "颜色检测",
        CMD_START_CIRCLE: "同心圆检测",
        CMD_START_DISK_CENTER: "物料盘中心检测",
    }.get(mode, "手动相机预览")


def _detection_log_payload(mode: int, result: dict[str, object]) -> dict[str, object]:
    """提取稳定、紧凑的检测字段，避免日志被模型内部对象污染。"""
    if mode == CMD_START_QR:
        return {
            "raw_code": result.get("raw_code"),
            "status": result.get("status"),
            "code": result.get("code"),
        }
    if mode in (CMD_START_COLOR, CMD_START_CIRCLE):
        detections = result.get("detections", [])
        normalized: list[dict[str, object]] = []
        if isinstance(detections, list):
            for item in detections:
                if not isinstance(item, dict):
                    continue
                center = item.get("center")
                entry: dict[str, object] = {
                    "type": item.get("type"),
                    "center": list(center) if isinstance(center, (list, tuple)) else None,
                    "confidence": round(float(item["confidence"]), 3)
                    if isinstance(item.get("confidence"), (int, float))
                    else None,
                    "measured": bool(item.get("measured", False)),
                    "support_count": int(item.get("support_count", 0)),
                }
                if mode == CMD_START_COLOR:
                    entry.update(
                        {
                            "yolo_type": item.get("yolo_type"),
                            "yolo_confidence": round(float(item["yolo_confidence"]), 3)
                            if isinstance(item.get("yolo_confidence"), (int, float))
                            else None,
                            "hsv_color": item.get("hsv_color"),
                            "hsv_coverage": round(float(item["hsv_coverage"]), 3)
                            if isinstance(item.get("hsv_coverage"), (int, float))
                            else None,
                            "hsv_purity": round(float(item["hsv_purity"]), 3)
                            if isinstance(item.get("hsv_purity"), (int, float))
                            else None,
                            "hsv_margin": round(float(item["hsv_margin"]), 3)
                            if isinstance(item.get("hsv_margin"), (int, float))
                            else None,
                            "classification_source": item.get("classification_source"),
                        }
                    )
                normalized.append(entry)
        normalized.sort(key=lambda item: json.dumps(item, sort_keys=True, ensure_ascii=False))
        return {"detections": normalized}
    return {
        "center": result.get("center"),
        "status": result.get("status"),
        "support_count": result.get("support_count", 0),
        "measured_count": result.get("measured_count", 0),
    }


def _log_detection_result(state: dict[str, object], mode: int, result: dict[str, object]) -> None:
    """仅在检测结果发生变化时向终端输出一行 JSON 日志。"""
    payload = _detection_log_payload(mode, result)
    signature = json.dumps(payload, sort_keys=True, ensure_ascii=False, separators=(",", ":"))
    if signature == state.get("last_detection_log_signature"):
        return
    state["last_detection_log_signature"] = signature
    print(
        f"vision_result mode={_preview_mode_text(mode)} session={int(state['session'])} "
        f"result={signature}",
        flush=True,
    )


def warmup_vision_models() -> None:
    """启动时预热当前颜色后端和数字圆环 YOLO 模型。"""
    frame = np.zeros((MODEL_WARMUP_FRAME_SIZE, MODEL_WARMUP_FRAME_SIZE, 3), dtype=np.uint8)

    started = time.perf_counter()
    if COLOR_DETECTION_BACKEND == "hsv":
        get_hsv_config()
        detect_color_hsv(frame)
    elif COLOR_DETECTION_BACKEND == "yolo":
        detect_color(frame)
    elif COLOR_DETECTION_BACKEND == "yolo_hsv":
        get_hsv_config()
        detect_color(frame)
    else:
        _color_detection_backend()
    color_ms = (time.perf_counter() - started) * 1000.0

    started = time.perf_counter()
    detect_circle(frame)
    circle_ms = (time.perf_counter() - started) * 1000.0

    print(
        f"vision models ready backend={MODEL_BACKEND} "
        f"color_detection={COLOR_DETECTION_BACKEND} "
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
        state.update(
            mode=command,
            session=session,
            period_ms=period_ms,
            last_run=float("-inf"),
            last_detection_log_signature=None,
        )
        _write_ack(port, session, command, ACK_OK)
    elif command == CMD_STOP:
        if payload:
            _write_ack(port, session, command, ACK_BAD_LENGTH)
            return
        reset_advance_tracking()
        reset_qr_tracking()
        state.update(mode=0, session=session, last_run=float("-inf"), last_detection_log_signature=None)
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
    camera_name = CAMERA_QR if mode == CMD_START_QR else CAMERA_VISION
    camera = cameras[camera_name]
    try:
        ok, frame = camera.read()
    except Exception:
        return
    if not ok:
        return
    if not _frame_matches_camera_contract(frame, camera_name):
        actual_size = _frame_size(frame)
        expected_width, expected_height = _camera_frame_size(camera_name)
        print(
            f"camera_frame_size_invalid name={camera_name} expected={expected_width}x{expected_height} "
            f"actual={actual_size}",
            flush=True,
        )
        return

    session = int(state["session"])
    if mode == CMD_START_QR:
        try:
            result = advance_detect_qr(frame)
        except Exception:
            return
        response = CMD_QR_RESULT
    elif mode == CMD_START_COLOR:
        detection_frame, roi_bounds = _detection_frame(frame, mode)
        result = _restore_detection_coordinates(_color_detection_backend()(detection_frame), roi_bounds)
        response = CMD_COLOR_RESULT
    elif mode == CMD_START_CIRCLE:
        detection_frame, _ = _detection_frame(frame, mode)
        result, response = advance_detect_circle(detection_frame), CMD_CIRCLE_RESULT
    else:
        result, response = (
            advance_detect_disk_center(frame, color_detector=advance_detect_color),
            CMD_DISK_CENTER_RESULT,
        )

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
    _log_detection_result(state, mode, result)
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
                "Check Jetson-IO PWM configuration for Pin 32 and GPIO permissions: "
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
    try:
        cameras = open_cameras()
    except Exception:
        port.close()
        raise

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
            "Check Jetson-IO PWM configuration for Pin 32 and GPIO permissions: "
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
        active_camera_id = CAMERA_QR if mode == CMD_START_QR else CAMERA_VISION
        selected_camera_id = gui.get_selected_camera()
        preview_mode = mode
        preview_result = result
        preview_frame = frame_bgr
        if selected_camera_id != active_camera_id:
            ok, selected_frame = cameras[selected_camera_id].read()
            if not ok or not _frame_matches_camera_contract(selected_frame, selected_camera_id):
                return
            preview_frame = selected_frame
            preview_mode = 0
            preview_result = {}
        aim_offset = (
            (QR_PREVIEW_AIM_OFFSET_X_PX, QR_PREVIEW_AIM_OFFSET_Y_PX)
            if selected_camera_id == CAMERA_QR
            else (VISION_PREVIEW_AIM_OFFSET_X_PX, VISION_PREVIEW_AIM_OFFSET_Y_PX)
        )
        try:
            status_text = f"{selected_camera_id} 相机预览 | {_preview_mode_text(preview_mode)}"
            preview_roi = _detection_roi_bounds(preview_frame, preview_mode)
            gui.set_camera_frame(
                render_camera_preview(
                    preview_frame,
                    preview_mode,
                    preview_result,
                    aim_offset=aim_offset,
                    roi_bounds=preview_roi,
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
            selected_camera_id = gui.get_selected_camera()
            ok, frame = cameras[selected_camera_id].read()
            if not ok or not _frame_matches_camera_contract(frame, selected_camera_id):
                return
            status_text = f"{selected_camera_id} 相机预览 | {_preview_mode_text(0)}"
            gui.set_camera_frame(
                render_camera_preview(
                    frame,
                    0,
                    aim_offset=(
                        (QR_PREVIEW_AIM_OFFSET_X_PX, QR_PREVIEW_AIM_OFFSET_Y_PX)
                        if selected_camera_id == CAMERA_QR
                        else (VISION_PREVIEW_AIM_OFFSET_X_PX, VISION_PREVIEW_AIM_OFFSET_Y_PX)
                    ),
                    status_text=status_text,
                ),
                status_text=status_text,
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
        _release_camera(cameras.get(CAMERA_QR))
        _release_camera(cameras.get(CAMERA_VISION))
        port.close()


if __name__ == "__main__":
    main()

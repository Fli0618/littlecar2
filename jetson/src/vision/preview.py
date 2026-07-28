"""将视觉识别结果绘制为可供 GUI 使用的 RGB 预览帧。"""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np

from protocol.commands import CMD_START_CIRCLE, CMD_START_COLOR, CMD_START_DISK_CENTER, CMD_START_QR


_CROSSHAIR_COLOR = (0, 255, 0)
_QR_COLOR = (0, 255, 255)
_COLOR_TARGET_COLOR = (255, 180, 0)
_CIRCLE_TARGET_COLOR = (255, 0, 255)
_DISK_CENTER_COLOR = (0, 0, 255)
_SUPPORT_POINT_COLOR = (255, 255, 0)
_TEXT_COLOR = (255, 255, 255)
_FONT = cv2.FONT_HERSHEY_SIMPLEX


def render_camera_preview(
    frame_bgr: np.ndarray,
    mode: int,
    result: dict[str, object] | None = None,
    aim_offset: tuple[int, int] = (0, 0),
    status_text: str = "",
) -> np.ndarray:
    """渲染独立 RGB 预览图，不修改相机输入帧或识别结果。"""
    _validate_frame(frame_bgr)
    preview_bgr = frame_bgr.copy()
    height, width = preview_bgr.shape[:2]
    aim_x = width // 2 + int(aim_offset[0])
    aim_y = height // 2 + int(aim_offset[1])
    _draw_crosshair(preview_bgr, aim_x, aim_y)

    normalized_result: dict[str, object] = result if isinstance(result, dict) else {}
    if mode == CMD_START_QR:
        _draw_qr_result(preview_bgr, normalized_result)
    elif mode in (CMD_START_COLOR, CMD_START_CIRCLE):
        _draw_detections(preview_bgr, normalized_result, mode)
    elif mode == CMD_START_DISK_CENTER:
        _draw_disk_center(preview_bgr, normalized_result)
    elif mode == 0:
        _draw_text(preview_bgr, "MANUAL CAMERA PREVIEW", (12, 26), _TEXT_COLOR)

    if status_text:
        _draw_text(preview_bgr, status_text, (12, height - 14), _TEXT_COLOR)
    return cv2.cvtColor(preview_bgr, cv2.COLOR_BGR2RGB)


def _validate_frame(frame_bgr: np.ndarray) -> None:
    if not isinstance(frame_bgr, np.ndarray) or frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
        raise TypeError("frame_bgr must be a BGR numpy.ndarray")


def _draw_crosshair(frame: np.ndarray, center_x: int, center_y: int) -> None:
    point = _bounded_point(frame, center_x, center_y)
    cv2.drawMarker(frame, point, _CROSSHAIR_COLOR, markerType=cv2.MARKER_CROSS, markerSize=26, thickness=1)
    _draw_text(frame, f"aim=({center_x - frame.shape[1] // 2:+d},{center_y - frame.shape[0] // 2:+d})", (12, 52), _CROSSHAIR_COLOR)


def _draw_qr_result(frame: np.ndarray, result: dict[str, object]) -> None:
    raw_code = result.get("raw_code")
    status = result.get("status")
    code = result.get("code")
    _draw_text(frame, f"QR raw: {_display_value(raw_code)}", (12, 82), _QR_COLOR)
    _draw_text(frame, f"QR status: {_display_value(status)}", (12, 106), _QR_COLOR)
    _draw_text(frame, f"QR code: {_display_value(code)}", (12, 130), _QR_COLOR)


def _draw_detections(frame: np.ndarray, result: dict[str, object], mode: int) -> None:
    color = _COLOR_TARGET_COLOR if mode == CMD_START_COLOR else _CIRCLE_TARGET_COLOR
    detections = result.get("detections")
    if not isinstance(detections, list):
        return
    for item in detections:
        if not isinstance(item, dict):
            continue
        center = _point_from_value(item.get("center"))
        if center is None:
            continue
        point = _bounded_point(frame, *center)
        cv2.drawMarker(frame, point, color, markerType=cv2.MARKER_TILTED_CROSS, markerSize=20, thickness=2)
        label = (
            f"type={_display_value(item.get('type'))} "
            f"conf={_format_confidence(item.get('confidence'))} "
            f"measured={int(bool(item.get('measured', False)))} "
            f"support={_display_value(item.get('support_count', 0))}"
        )
        _draw_text(frame, label, (point[0] + 8, point[1] - 8), color)


def _draw_disk_center(frame: np.ndarray, result: dict[str, object]) -> None:
    support_points = result.get("support_points")
    if isinstance(support_points, list):
        for support_point in support_points:
            point = _point_from_value(support_point)
            if point is not None:
                cv2.circle(frame, _bounded_point(frame, *point), 6, _SUPPORT_POINT_COLOR, 2)

    center = _point_from_value(result.get("center"))
    if center is None:
        return
    point = _bounded_point(frame, *center)
    cv2.drawMarker(frame, point, _DISK_CENTER_COLOR, markerType=cv2.MARKER_CROSS, markerSize=30, thickness=2)
    label = (
        f"disk center=({center[0]},{center[1]}) "
        f"support={_display_value(result.get('support_count', 0))} "
        f"measured={_display_value(result.get('measured_count', 0))}"
    )
    _draw_text(frame, label, (point[0] + 10, point[1] - 10), _DISK_CENTER_COLOR)


def _bounded_point(frame: np.ndarray, x: int, y: int) -> tuple[int, int]:
    return max(0, min(frame.shape[1] - 1, x)), max(0, min(frame.shape[0] - 1, y))


def _point_from_value(value: object) -> tuple[int, int] | None:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        return None
    if not all(isinstance(coordinate, (int, float)) for coordinate in value):
        return None
    return int(round(value[0])), int(round(value[1]))


def _draw_text(frame: np.ndarray, text: str, origin: tuple[int, int], color: tuple[int, int, int]) -> None:
    x, y = _bounded_point(frame, *origin)
    cv2.putText(frame, str(text), (x, y), _FONT, 0.48, color, 1, cv2.LINE_AA)


def _display_value(value: object) -> str:
    return "-" if value is None else str(value)


def _format_confidence(value: object) -> str:
    return f"{float(value):.2f}" if isinstance(value, (int, float)) else "-"

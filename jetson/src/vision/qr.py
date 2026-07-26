"""二维码任务码的原始识别与稳定确认接口。"""

from __future__ import annotations

from collections import deque
import re

import cv2
import numpy as np


QR_WINDOW_SIZE = 5
QR_CONFIRM_COUNT = 3
QR_MISSING_THRESHOLD = 5

_TASK_CODE_PATTERN = re.compile(r"^\d{3}\+\d{3}\+\d{3}\+\d{3}$", re.ASCII)
_DETECTOR = cv2.QRCodeDetector()
_RECENT_CODES: deque[str | None] = deque(maxlen=QR_WINDOW_SIZE)
_LATCHED_CODE: str | None = None
_LAST_DISAPPEARED_CODE: str | None = None
_MISSING_FRAMES = 0


def detect_qr(frame_bgr: np.ndarray) -> str | None:
    """从 BGR 图像中识别二维码，并直接返回原始内容。"""
    if not isinstance(frame_bgr, np.ndarray) or frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
        raise TypeError("frame_bgr must be a BGR numpy.ndarray")

    data, points, _ = _DETECTOR.detectAndDecode(frame_bgr)
    if not data or points is None:
        return None
    return data


def _is_task_code(value: str) -> bool:
    try:
        encoded = value.encode("ascii")
    except UnicodeEncodeError:
        return False
    return len(encoded) == 15 and _TASK_CODE_PATTERN.fullmatch(value) is not None


def _missing_result(raw_code: str | None, status: str) -> dict[str, str | None]:
    global _LATCHED_CODE, _LAST_DISAPPEARED_CODE, _MISSING_FRAMES

    _RECENT_CODES.append(None)
    if _LATCHED_CODE is not None:
        _MISSING_FRAMES += 1
        if _MISSING_FRAMES >= QR_MISSING_THRESHOLD:
            _LAST_DISAPPEARED_CODE = _LATCHED_CODE
            _LATCHED_CODE = None
            _MISSING_FRAMES = 0
            _RECENT_CODES.clear()
            return {"raw_code": raw_code, "code": None, "status": "DISAPPEARED"}
        if raw_code is None:
            status = "MISSING"
    return {"raw_code": raw_code, "code": None, "status": status}


def advance_detect_qr(frame_bgr: np.ndarray) -> dict[str, str | None]:
    """识别并在五帧窗口内确认一次稳定的二维码任务码。"""
    global _LATCHED_CODE, _MISSING_FRAMES

    raw_code = detect_qr(frame_bgr)
    if raw_code is None:
        return _missing_result(None, "NO_QR")
    if not _is_task_code(raw_code):
        return _missing_result(raw_code, "INVALID")

    _RECENT_CODES.append(raw_code)
    _MISSING_FRAMES = 0
    if raw_code == _LATCHED_CODE:
        return {"raw_code": raw_code, "code": None, "status": "REPEATED"}
    if _RECENT_CODES.count(raw_code) < QR_CONFIRM_COUNT:
        return {"raw_code": raw_code, "code": None, "status": "CONFIRMING"}

    previous_latched = _LATCHED_CODE
    _LATCHED_CODE = raw_code
    if previous_latched is not None:
        status = "CHANGED"
    elif _LAST_DISAPPEARED_CODE == raw_code:
        status = "REAPPEARED"
    elif _LAST_DISAPPEARED_CODE is not None:
        status = "CHANGED"
    else:
        status = "FIRST_DETECTED"
    return {"raw_code": raw_code, "code": raw_code, "status": status}


def reset_qr_tracking() -> None:
    """清空二维码确认窗口、锁存任务码与消失状态。"""
    global _LATCHED_CODE, _LAST_DISAPPEARED_CODE, _MISSING_FRAMES

    _RECENT_CODES.clear()
    _LATCHED_CODE = None
    _LAST_DISAPPEARED_CODE = None
    _MISSING_FRAMES = 0

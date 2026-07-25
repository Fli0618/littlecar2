"""Jetson 应用入口。

摄像头、视觉、通信和小车控制流程由用户在此处自行组合。
"""

from __future__ import annotations

import sys
from pathlib import Path


try:
    from vision import (
        CAMERA_INDEX_COLOR_CIRCLE,
        CAMERA_INDEX_QR,
        advance_detect_circle,
        advance_detect_color,
        advance_detect_disk_center,
        detect_circle,
        detect_color,
        detect_disk_center,
        detect_qr,
    )
except ModuleNotFoundError:
    sys.path.insert(0, str(Path(__file__).resolve().parent / "src"))
    from vision import (
        CAMERA_INDEX_COLOR_CIRCLE,
        CAMERA_INDEX_QR,
        advance_detect_circle,
        advance_detect_color,
        advance_detect_disk_center,
        detect_circle,
        detect_color,
        detect_disk_center,
        detect_qr,
    )

__all__ = [
    "detect_circle",
    "detect_color",
    "detect_disk_center",
    "advance_detect_circle",
    "advance_detect_color",
    "advance_detect_disk_center",
    "CAMERA_INDEX_COLOR_CIRCLE",
    "CAMERA_INDEX_QR",
    "detect_qr",
    "main",
]


def main() -> None:
    """预留主流程入口。"""
    pass


if __name__ == "__main__":
    main()

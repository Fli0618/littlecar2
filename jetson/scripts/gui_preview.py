"""PC 端比赛界面预览。

该脚本只生成内存中的合成画面，用于联调 GUI 和视觉标注；不会打开相机、加载模型或访问串口。
"""

from __future__ import annotations

import time
import tkinter as tk
from collections.abc import Callable

import numpy as np

from protocol.commands import CMD_START_CIRCLE, CMD_START_COLOR, CMD_START_DISK_CENTER, CMD_START_QR
from ui import CompetitionGUI
from vision import render_camera_preview

FRAME_WIDTH = 960
FRAME_HEIGHT = 540
REFRESH_INTERVAL_MS = 50

MODE_LABELS = {
    0: "手动预览",
    CMD_START_COLOR: "彩色物料识别",
    CMD_START_CIRCLE: "同心圆识别",
    CMD_START_DISK_CENTER: "物料盘中心定位",
    CMD_START_QR: "二维码识别",
}


def compose_preview_frame(frame_index: int, session: int) -> np.ndarray:
    """生成带有轻微运动效果的 BGR 合成相机画面。"""
    y, x = np.ogrid[:FRAME_HEIGHT, :FRAME_WIDTH]
    frame = np.empty((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)
    frame[..., 0] = 48
    frame[..., 1] = 58
    frame[..., 2] = 68

    frame[54:486, 105:855] = (88, 103, 114)
    shift = int(36 * np.sin((frame_index + session * 13) / 18))
    centers = ((260 + shift, 190), (480 - shift, 265), (690 + shift // 2, 345))
    colors = ((46, 66, 222), (49, 184, 73), (225, 112, 42))
    for (center_x, center_y), color in zip(centers, colors):
        mask = (x - center_x) ** 2 + (y - center_y) ** 2 <= 43**2
        frame[mask] = color
        ring = (x - center_x) ** 2 + (y - center_y) ** 2 <= 18**2
        frame[ring] = (230, 230, 230)

    return frame


def simulate_visual_result(mode: int, frame_index: int, session: int) -> dict[str, object] | None:
    """为每种视觉任务提供可重复的模拟结果，便于观察标注刷新。"""
    offset = int(8 * np.sin((frame_index + session) / 12))
    if mode == CMD_START_QR:
        return {"raw_code": "156+123+516+231", "status": "confirmed", "code": "156+123+516+231"}
    if mode == CMD_START_DISK_CENTER:
        return {
            "support_points": [[445 + offset, 247], [515 + offset, 247], [480 + offset, 310]],
            "center": [480 + offset, 268],
            "support_count": 3,
            "measured_count": 3,
        }
    if mode in (CMD_START_COLOR, CMD_START_CIRCLE):
        return {
            "detections": [
                {"type": 1, "center": [260 + offset, 190], "confidence": 0.96, "measured": True, "support_count": 4},
                {"type": 3, "center": [480 - offset, 265], "confidence": 0.93, "measured": True, "support_count": 4},
                {"type": 5, "center": [690 + offset, 345], "confidence": 0.91, "measured": True, "support_count": 3},
            ]
        }
    return None


class PreviewController:
    """管理预览会话、定时刷新和快捷键，不连接任何真实设备。"""

    def __init__(
        self,
        root: tk.Misc,
        gui: CompetitionGUI,
        *,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.root = root
        self.gui = gui
        self.clock = clock
        self.started_at: float | None = None
        self.mode = 0
        self.session = 0
        self.frame_index = 0

    def start(self, _start_area: int) -> bool:
        self.gui.show_running_page()
        self.started_at = self.clock()
        self.update_elapsed()
        self.root.after(3000, lambda: self.gui.set_task_code("156+123+516+231"))
        self.root.after(3000, lambda: self.gui.set_counts(1, 1))
        self.refresh_camera()
        return True

    def update_elapsed(self) -> None:
        if self.started_at is not None:
            self.gui.set_elapsed(int(self.clock() - self.started_at))
            self.root.after(200, self.update_elapsed)

    def activate_camera_mode(self, mode: int) -> None:
        self.mode = mode
        self.session += 1
        self.gui.show_camera_page()
        self.refresh_camera()

    def show_field(self) -> None:
        self.gui.show_field_page()

    def stop_camera_mode(self) -> None:
        self.mode = 0
        self.session += 1
        self.gui.show_running_page()

    def refresh_camera(self, refresh_session: int | None = None) -> None:
        """合成 BGR 帧并经统一视觉渲染函数转换为 RGB 后交给 GUI。"""
        refresh_session = self.session if refresh_session is None else refresh_session
        if refresh_session != self.session:
            return
        frame_bgr = compose_preview_frame(self.frame_index, self.session)
        status = f"{MODE_LABELS[self.mode]} | 模拟会话 {self.session}"
        result = simulate_visual_result(self.mode, self.frame_index, self.session)
        frame_rgb = render_camera_preview(frame_bgr, self.mode, result=result, status_text=status)
        self.gui.set_camera_frame(frame_rgb, status)
        self.frame_index += 1
        if self.gui.is_camera_page_visible():
            self.root.after(REFRESH_INTERVAL_MS, lambda: self.refresh_camera(refresh_session))


def main() -> None:
    root = tk.Tk()
    gui = CompetitionGUI(root, camera_preview_enabled=True)
    controller = PreviewController(root, gui)

    gui.set_start_callback(controller.start)
    root.bind("<F2>", lambda _event: controller.show_field())
    root.bind("<F3>", lambda _event: controller.activate_camera_mode(CMD_START_COLOR))
    root.bind("<F4>", lambda _event: controller.activate_camera_mode(CMD_START_CIRCLE))
    root.bind("<F5>", lambda _event: controller.activate_camera_mode(CMD_START_DISK_CENTER))
    root.bind("<F6>", lambda _event: controller.activate_camera_mode(CMD_START_QR))
    root.bind("<F7>", lambda _event: controller.stop_camera_mode())
    gui.run()


if __name__ == "__main__":
    main()

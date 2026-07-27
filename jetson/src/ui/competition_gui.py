"""基于 Tkinter 的比赛显示窗口。"""

from __future__ import annotations

import tkinter as tk
import tkinter.font as tkfont
from collections.abc import Callable

WINDOW_WIDTH = 1600
WINDOW_HEIGHT = 900
TASK_CODE_FONT_SIZE = 150
START_BUTTON_FONT_SIZE = 72

BACKGROUND_COLOR = "#000000"
TEXT_COLOR = "#FFFFFF"
LABEL_COLOR = "#A8A8A8"
DIVIDER_COLOR = "#303030"


class CompetitionGUI:
    """提供比赛启动、任务码和基础统计显示的轻量窗口。"""

    def __init__(self, root: tk.Tk | None = None) -> None:
        self.root = root or tk.Tk()
        self._start_callback: Callable[[], bool | None] | None = None
        self._start_clicked = False
        self._closed = False

        self.root.title("LittleCar2 比赛")
        self.root.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")
        self.root.minsize(800, 450)
        self.root.configure(bg=BACKGROUND_COLOR)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self._start_frame = tk.Frame(self.root, bg=BACKGROUND_COLOR)
        self._running_frame = tk.Frame(self.root, bg=BACKGROUND_COLOR)
        self._build_start_page()
        self._build_running_page()
        self.show_start_page()

    def _build_start_page(self) -> None:
        font_family = self._pick_font("Noto Sans CJK SC", "Noto Sans CJK", "Microsoft YaHei")
        button = tk.Button(
            self._start_frame,
            text="开始比赛",
            command=self._on_start,
            bg=TEXT_COLOR,
            fg=BACKGROUND_COLOR,
            activebackground="#D8D8D8",
            activeforeground=BACKGROUND_COLOR,
            borderwidth=0,
            font=(font_family, START_BUTTON_FONT_SIZE, "bold"),
        )
        button.place(relx=0.5, rely=0.5, anchor="center", relwidth=0.475, relheight=0.267)

    def _build_running_page(self) -> None:
        self._running_frame.grid_rowconfigure(0, weight=3)
        self._running_frame.grid_rowconfigure(1, weight=2)
        self._running_frame.grid_columnconfigure(0, weight=1)

        task_area = tk.Frame(self._running_frame, bg=BACKGROUND_COLOR)
        task_area.grid(row=0, column=0, sticky="nsew")
        self._task_code = tk.Label(
            task_area,
            text="",
            bg=BACKGROUND_COLOR,
            fg=TEXT_COLOR,
            font=("DejaVu Sans Mono", TASK_CODE_FONT_SIZE, "bold"),
        )
        self._task_code.place(relx=0.5, rely=0.5, anchor="center")

        stats_area = tk.Frame(self._running_frame, bg=BACKGROUND_COLOR, highlightbackground=DIVIDER_COLOR, highlightthickness=1)
        stats_area.grid(row=1, column=0, sticky="nsew")
        self._count_values: list[tk.Label] = []
        for column, label in enumerate(("正确抓取", "正确放置", "已运行时间")):
            stats_area.grid_columnconfigure(column, weight=1, uniform="stats")
            stats_area.grid_rowconfigure(0, weight=1)
            stats_area.grid_rowconfigure(1, weight=1)
            cell = tk.Frame(stats_area, bg=BACKGROUND_COLOR)
            cell.grid(row=0, column=column, rowspan=2, sticky="nsew")
            if column:
                cell.configure(highlightbackground=DIVIDER_COLOR, highlightthickness=1, highlightcolor=DIVIDER_COLOR)
            tk.Label(cell, text=label, bg=BACKGROUND_COLOR, fg=LABEL_COLOR, font=("Arial", 24)).place(relx=0.5, rely=0.33, anchor="center")
            value = tk.Label(cell, bg=BACKGROUND_COLOR, fg=TEXT_COLOR, font=("Arial", 42, "bold"))
            value.place(relx=0.5, rely=0.66, anchor="center")
            self._count_values.append(value)
        self.set_counts(0, 0)
        self.set_elapsed(0)

    def _pick_font(self, *preferred_fonts: str) -> str:
        available = set(tkfont.families(self.root))
        return next((font for font in preferred_fonts if font in available), preferred_fonts[-1])

    def _on_start(self) -> None:
        if self._start_clicked or self._start_callback is None:
            return
        self._start_clicked = True
        try:
            started = self._start_callback()
        except Exception:
            self._start_clicked = False
            raise
        if started is False:
            self._start_clicked = False

    def show_start_page(self) -> None:
        """显示只含开始按钮的初始页面。"""
        self._running_frame.pack_forget()
        self._start_frame.pack(fill="both", expand=True)

    def show_running_page(self) -> None:
        """显示任务码、统计和计时区域。"""
        self._start_frame.pack_forget()
        self._running_frame.pack(fill="both", expand=True)

    def set_task_code(self, code: str) -> None:
        """更新当前任务码显示。"""
        self._task_code.configure(text=code)

    def set_counts(self, pick_count: int, place_count: int) -> None:
        """更新抓取和放置计数。"""
        self._count_values[0].configure(text=f"{pick_count} / 6")
        self._count_values[1].configure(text=f"{place_count} / 6")

    def set_elapsed(self, seconds: int) -> None:
        """按分秒格式更新已运行时间。"""
        elapsed = max(0, int(seconds))
        minutes, remaining_seconds = divmod(elapsed, 60)
        self._count_values[2].configure(text=f"{minutes:02d}:{remaining_seconds:02d}")

    def set_start_callback(self, callback: Callable[[], bool | None]) -> None:
        """设置开始按钮回调；返回 False 表示本次启动失败。"""
        self._start_callback = callback

    def run(self) -> None:
        """进入 Tkinter 主事件循环。"""
        self.root.mainloop()

    def close(self) -> None:
        """关闭窗口，随后由主服务释放硬件资源。"""
        if self._closed:
            return
        self._closed = True
        self.root.destroy()

"""基于 Tkinter 的比赛显示窗口。"""

from __future__ import annotations

import tkinter as tk
import tkinter.font as tkfont
from collections.abc import Callable

WINDOW_WIDTH = 1600
WINDOW_HEIGHT = 900
TASK_CODE_FONT_SIZE = 150
START_BUTTON_FONT_SIZE = 72
FIELD_BUTTON_FONT_SIZE = 28

BACKGROUND_COLOR = "#000000"
TEXT_COLOR = "#FFFFFF"
LABEL_COLOR = "#A8A8A8"
DIVIDER_COLOR = "#303030"
FIELD_BACKGROUND_COLOR = "#161616"
FIELD_SURFACE_COLOR = "#DCDCDC"
FIELD_PLATFORM_COLOR = "#FFFCE2"
DIMENSION_COLOR = "#2254D8"
START_ZONE_COLOR = "#1239D6"


class CompetitionGUI:
    """提供比赛启动、任务码和基础统计显示的轻量窗口。"""

    def __init__(self, root: tk.Tk | None = None) -> None:
        self.root = root or tk.Tk()
        self._start_callback: Callable[[], bool | None] | None = None
        self._start_clicked = False
        self._closed = False
        self._font_family = self._pick_font("Noto Sans CJK SC", "Noto Sans CJK", "Microsoft YaHei")

        self.root.title("LittleCar2 比赛")
        self.root.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")
        self.root.minsize(800, 450)
        self.root.configure(bg=BACKGROUND_COLOR)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self._start_frame = tk.Frame(self.root, bg=BACKGROUND_COLOR)
        self._running_frame = tk.Frame(self.root, bg=BACKGROUND_COLOR)
        self._field_frame = tk.Frame(self.root, bg=FIELD_BACKGROUND_COLOR)
        self._build_start_page()
        self._build_running_page()
        self._build_field_page()
        self.show_start_page()

    def _build_start_page(self) -> None:
        button = tk.Button(
            self._start_frame,
            text="开始比赛",
            command=self._on_start,
            bg=TEXT_COLOR,
            fg=BACKGROUND_COLOR,
            activebackground="#D8D8D8",
            activeforeground=BACKGROUND_COLOR,
            borderwidth=0,
            font=(self._font_family, START_BUTTON_FONT_SIZE, "bold"),
        )
        button.place(relx=0.5, rely=0.5, anchor="center", relwidth=0.475, relheight=0.267)
        field_button = tk.Button(
            self._start_frame,
            text="场地标注",
            command=self.show_field_page,
            bg="#252525",
            fg=TEXT_COLOR,
            activebackground="#454545",
            activeforeground=TEXT_COLOR,
            borderwidth=0,
            font=(self._font_family, FIELD_BUTTON_FONT_SIZE, "bold"),
        )
        field_button.place(relx=0.04, rely=0.92, anchor="sw", relwidth=0.16, relheight=0.09)

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

    def _build_field_page(self) -> None:
        self._field_canvas = tk.Canvas(self._field_frame, bg=FIELD_BACKGROUND_COLOR, highlightthickness=0)
        self._field_canvas.pack(fill="both", expand=True)
        self._field_canvas.bind("<Configure>", self._draw_field_annotation)
        self._field_canvas.bind("<Escape>", lambda _event: self.show_start_page())

        back_button = tk.Button(
            self._field_frame,
            text="返回",
            command=self.show_start_page,
            bg="#252525",
            fg=TEXT_COLOR,
            activebackground="#454545",
            activeforeground=TEXT_COLOR,
            borderwidth=0,
            font=(self._font_family, FIELD_BUTTON_FONT_SIZE, "bold"),
        )
        back_button.place(x=24, y=24, width=120, height=52)

    def _draw_field_annotation(self, _event: tk.Event[tk.Misc] | None = None) -> None:
        """按 2400 mm 的逻辑坐标绘制赛场及关键尺寸。"""
        canvas = self._field_canvas
        width, height = canvas.winfo_width(), canvas.winfo_height()
        if width <= 1 or height <= 1:
            return
        canvas.delete("all")

        reserved_top, reserved_bottom, reserved_side = 84, 110, 108
        scale = min((width - reserved_side * 2) / 2400, (height - reserved_top - reserved_bottom) / 2400)
        field_size = 2400 * scale
        left = (width - field_size) / 2
        top = reserved_top + max(0, (height - reserved_top - reserved_bottom - field_size) / 2)

        def point(x: float, y: float) -> tuple[float, float]:
            return left + x * scale, top + y * scale

        def rectangle(x1: float, y1: float, x2: float, y2: float, **options: object) -> int:
            return canvas.create_rectangle(*point(x1, y1), *point(x2, y2), **options)

        def text(x: float, y: float, value: str, size: int = 20, **options: object) -> int:
            options.setdefault("fill", "#202020")
            return canvas.create_text(*point(x, y), text=value, font=(self._font_family, size, "bold"), **options)

        def line(x1: float, y1: float, x2: float, y2: float, **options: object) -> int:
            return canvas.create_line(*point(x1, y1), *point(x2, y2), **options)

        rectangle(0, 0, 2400, 2400, fill=FIELD_SURFACE_COLOR, outline="#666666", width=2)
        for x1, y1 in ((550, 500), (1400, 500), (550, 1450), (1400, 1450)):
            rectangle(x1, y1, x1 + 450, y1 + 450, fill=FIELD_PLATFORM_COLOR, outline="")

        line(1200, 0, 1200, 2400, fill="#5B5B5B", width=2, dash=(18, 12))
        line(0, 1200, 2400, 1200, fill="#5B5B5B", width=2, dash=(18, 12))
        for x, y in ((2250, 150), (2250, 2250)):
            rectangle(x - 150, y - 150, x + 150, y + 150, fill=START_ZONE_COLOR, outline="")

        source_x, source_y = point(1200, 0)
        radius = 140 * scale
        canvas.create_oval(source_x - radius, source_y - radius, source_x + radius, source_y + radius, fill="#F7F7F7", outline="#444444", width=2)
        for x, y in ((1130, 20), (1270, 20), (1200, 110)):
            cx, cy = point(x, y)
            canvas.create_oval(cx - 12 * scale, cy - 12 * scale, cx + 12 * scale, cy + 12 * scale, fill="#FFFFFF", outline="#444444")

        for x, y in ((75, 1050), (75, 1200), (75, 1350), (1050, 2325), (1200, 2325), (1350, 2325)):
            cx, cy = point(x, y)
            target_radius = max(6, 40 * scale)
            canvas.create_oval(cx - target_radius, cy - target_radius, cx + target_radius, cy + target_radius, fill="#FFFFFF", outline="#222222", width=2)
            canvas.create_oval(cx - target_radius / 3, cy - target_radius / 3, cx + target_radius / 3, cy + target_radius / 3, fill="#222222", outline="")

        text(720, 110, "原料区", 22)
        text(210, 1200, "暂存区", 20, angle=90)
        text(1380, 2200, "粗加工区", 22)
        text(2300, 1200, "二次编码区", 20, angle=90)
        text(2100, 420, "启停区1", 20)
        text(2100, 1980, "启停区2", 20)

        dimension = {"fill": DIMENSION_COLOR, "width": 2, "arrow": "both"}
        line(0, 2575, 2400, 2575, **dimension)
        text(1200, 2650, "2400", 18, fill=DIMENSION_COLOR)
        line(1000, 420, 1400, 420, **dimension)
        text(1200, 350, "400", 18, fill=DIMENSION_COLOR)
        line(550, 1080, 1000, 1080, **dimension)
        text(775, 1010, "450", 18, fill=DIMENSION_COLOR)
        line(1950, 60, 2250, 60, **dimension)
        text(2100, 120, "300", 18, fill=DIMENSION_COLOR)
        line(2460, 0, 2460, 2400, **dimension)
        text(2545, 1200, "2400", 18, fill=DIMENSION_COLOR, angle=90)
        line(-70, 0, -70, 150, **dimension)
        text(-145, 75, "150", 16, fill=DIMENSION_COLOR, angle=90)
        line(2525, 0, 2525, 1200, **dimension)
        text(2610, 600, "1100-1300", 16, fill=DIMENSION_COLOR, angle=90)

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
        self._field_frame.pack_forget()
        self._start_frame.pack(fill="both", expand=True)

    def show_running_page(self) -> None:
        """显示任务码、统计和计时区域。"""
        self._start_frame.pack_forget()
        self._field_frame.pack_forget()
        self._running_frame.pack(fill="both", expand=True)

    def show_field_page(self) -> None:
        """显示静态场地标注页，不改变比赛或硬件状态。"""
        self._start_frame.pack_forget()
        self._running_frame.pack_forget()
        self._field_frame.pack(fill="both", expand=True)
        self._field_canvas.focus_set()
        self._draw_field_annotation()

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

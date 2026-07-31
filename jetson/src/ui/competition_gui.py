"""基于 Tkinter 的比赛显示窗口。"""

from __future__ import annotations

import tkinter as tk
import tkinter.font as tkfont
from collections.abc import Callable

import numpy as np
from PIL import Image, ImageTk

from protocol.commands import START_AREA_1, START_AREA_2, VALID_START_AREAS

WINDOW_WIDTH = 1600
WINDOW_HEIGHT = 900
TASK_CODE_FONT_SIZE = 150
START_BUTTON_FONT_SIZE = 72
FIELD_BUTTON_FONT_SIZE = 28
CJK_FONT_FAMILIES = (
    "Noto Sans CJK SC",
    "Noto Sans SC",
    "Noto Sans CJK",
    "WenQuanYi Zen Hei",
    "Droid Sans Fallback",
    "Microsoft YaHei",
)

BACKGROUND_COLOR = "#000000"
TEXT_COLOR = "#FFFFFF"
LABEL_COLOR = "#A8A8A8"
DIVIDER_COLOR = "#303030"
FIELD_BACKGROUND_COLOR = "#161616"
FIELD_SURFACE_COLOR = "#DCDCDC"
FIELD_PLATFORM_COLOR = "#FFFCE2"
DIMENSION_COLOR = "#2254D8"
START_ZONE_COLOR = "#1239D6"
CAMERA_QR = "qr"
CAMERA_VISION = "vision"
VALID_CAMERAS = (CAMERA_QR, CAMERA_VISION)


class CompetitionGUI:
    """提供比赛启动、任务码和基础统计显示的轻量窗口。"""

    def __init__(self, root: tk.Tk | None = None, camera_preview_enabled: bool = True) -> None:
        self.root = root or tk.Tk()
        self._start_callback: Callable[[int], bool | None] | None = None
        self._selected_start_area: int | None = None
        self._start_clicked = False
        self._closed = False
        self._camera_preview_enabled = camera_preview_enabled
        self._camera_frame: tk.Frame | None = None
        self._camera_label: tk.Label | None = None
        self._camera_status: tk.Label | None = None
        self._camera_back_button: tk.Button | None = None
        self._camera_buttons: dict[str, tk.Button] = {}
        self._selected_camera = CAMERA_VISION
        self._camera_photo: ImageTk.PhotoImage | None = None
        self._camera_image: Image.Image | None = None
        self._current_page = "start"
        self._font_family = self._pick_font(*CJK_FONT_FAMILIES)

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
        if self._camera_preview_enabled:
            self._build_camera_page()
        self.show_start_page()

    def _build_start_page(self) -> None:
        self._start_selection = tk.Label(
            self._start_frame,
            bg=BACKGROUND_COLOR,
            fg=LABEL_COLOR,
            font=(self._font_family, FIELD_BUTTON_FONT_SIZE),
        )
        self._start_selection.place(relx=0.5, rely=0.25, anchor="center")
        self._start_button = tk.Button(
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
        self._start_button.place(relx=0.5, rely=0.54, anchor="center", relwidth=0.475, relheight=0.245)
        field_button = tk.Button(
            self._start_frame,
            text="选择启停区",
            command=self.show_field_page,
            bg="#252525",
            fg=TEXT_COLOR,
            activebackground="#454545",
            activeforeground=TEXT_COLOR,
            borderwidth=0,
            font=(self._font_family, FIELD_BUTTON_FONT_SIZE, "bold"),
        )
        field_button.place(relx=0.04, rely=0.92, anchor="sw", relwidth=0.16, relheight=0.09)
        self._update_start_selection()

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
        if self._camera_preview_enabled:
            camera_button = tk.Button(
                task_area,
                text="查看相机",
                command=self.show_camera_page,
                bg="#252525",
                fg=TEXT_COLOR,
                activebackground="#454545",
                activeforeground=TEXT_COLOR,
                borderwidth=0,
                font=(self._font_family, FIELD_BUTTON_FONT_SIZE, "bold"),
            )
            camera_button.place(relx=0.04, rely=0.92, anchor="sw", relwidth=0.16, relheight=0.18)

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
            tk.Label(
                cell,
                text=label,
                bg=BACKGROUND_COLOR,
                fg=LABEL_COLOR,
                font=(self._font_family, 24),
            ).place(relx=0.5, rely=0.33, anchor="center")
            value = tk.Label(
                cell,
                bg=BACKGROUND_COLOR,
                fg=TEXT_COLOR,
                font=(self._font_family, 42, "bold"),
            )
            value.place(relx=0.5, rely=0.66, anchor="center")
            self._count_values.append(value)
        self.set_counts(0, 0)
        self.set_elapsed(0)

    def _build_camera_page(self) -> None:
        """构建仅展示主服务提供帧数据的相机预览页。"""
        self._camera_frame = tk.Frame(self.root, bg=BACKGROUND_COLOR)
        self._camera_frame.grid_rowconfigure(0, weight=1)
        self._camera_frame.grid_rowconfigure(1, weight=0)
        self._camera_frame.grid_columnconfigure(0, weight=1)

        self._camera_label = tk.Label(self._camera_frame, bg=BACKGROUND_COLOR, borderwidth=0)
        self._camera_label.grid(row=0, column=0, sticky="nsew", padx=24, pady=(24, 8))
        self._camera_label.bind("<Configure>", self._on_camera_resize)

        footer = tk.Frame(self._camera_frame, bg=BACKGROUND_COLOR)
        footer.grid(row=1, column=0, sticky="ew", padx=24, pady=(8, 24))
        footer.grid_columnconfigure(0, weight=1)
        self._camera_status = tk.Label(
            footer,
            text="",
            bg=BACKGROUND_COLOR,
            fg=LABEL_COLOR,
            anchor="w",
            font=(self._font_family, FIELD_BUTTON_FONT_SIZE),
        )
        self._camera_status.grid(row=0, column=0, sticky="ew")
        camera_selector = tk.Frame(footer, bg=BACKGROUND_COLOR)
        camera_selector.grid(row=0, column=1, padx=(16, 0))
        for column, (camera_id, label) in enumerate(
            ((CAMERA_QR, "二维码相机"), (CAMERA_VISION, "视觉相机"))
        ):
            button = tk.Button(
                camera_selector,
                text=label,
                command=lambda selected=camera_id: self.select_camera(selected),
                bg="#454545" if camera_id == self._selected_camera else "#252525",
                fg=TEXT_COLOR,
                activebackground="#5A5A5A",
                activeforeground=TEXT_COLOR,
                borderwidth=0,
                font=(self._font_family, FIELD_BUTTON_FONT_SIZE, "bold"),
            )
            button.grid(row=0, column=column, padx=(0 if column == 0 else 8, 0), ipadx=12, ipady=8)
            self._camera_buttons[camera_id] = button
        self._camera_back_button = tk.Button(
            footer,
            text="返回任务码",
            command=self.show_running_page,
            bg="#252525",
            fg=TEXT_COLOR,
            activebackground="#454545",
            activeforeground=TEXT_COLOR,
            borderwidth=0,
            font=(self._font_family, FIELD_BUTTON_FONT_SIZE, "bold"),
        )
        self._camera_back_button.grid(row=0, column=2, padx=(16, 0), ipadx=18, ipady=8)

    def _on_camera_resize(self, _event: tk.Event[tk.Misc] | None = None) -> None:
        self._render_camera_frame()

    def _render_camera_frame(self) -> None:
        """将最新帧等比置于当前预览区域，余下区域保持黑色。"""
        if self._camera_image is None or self._camera_label is None:
            return
        width = self._camera_label.winfo_width()
        height = self._camera_label.winfo_height()
        if width <= 1 or height <= 1:
            return

        image = self._camera_image
        scale = min(width / image.width, height / image.height)
        scaled_size = (max(1, round(image.width * scale)), max(1, round(image.height * scale)))
        resampling = getattr(Image, "Resampling", Image).LANCZOS
        scaled_image = image.resize(scaled_size, resampling)
        preview = Image.new("RGB", (width, height), BACKGROUND_COLOR)
        preview.paste(scaled_image, ((width - scaled_size[0]) // 2, (height - scaled_size[1]) // 2))
        self._camera_photo = ImageTk.PhotoImage(preview)
        self._camera_label.configure(image=self._camera_photo)

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

        reserved_top, reserved_bottom, reserved_side = 84, 110, 220
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

        def horizontal_dimension(x1: float, x2: float, edge_y: float, dimension_y: float, label: str) -> None:
            """绘制与水平边界端点对齐的尺寸线及延长线。"""
            line(x1, edge_y, x1, dimension_y, fill=DIMENSION_COLOR, width=1)
            line(x2, edge_y, x2, dimension_y, fill=DIMENSION_COLOR, width=1)
            line(x1, dimension_y, x2, dimension_y, fill=DIMENSION_COLOR, width=2, arrow="both")
            text((x1 + x2) / 2, dimension_y - 70, label, 18, fill=DIMENSION_COLOR)

        def vertical_dimension(y1: float, y2: float, edge_x: float, dimension_x: float, label: str) -> None:
            """绘制与垂直边界端点对齐的尺寸线及延长线。"""
            line(edge_x, y1, dimension_x, y1, fill=DIMENSION_COLOR, width=1)
            line(edge_x, y2, dimension_x, y2, fill=DIMENSION_COLOR, width=1)
            line(dimension_x, y1, dimension_x, y2, fill=DIMENSION_COLOR, width=2, arrow="both")
            text(dimension_x + 75, (y1 + y2) / 2, label, 16, fill=DIMENSION_COLOR, angle=90)

        rectangle(0, 0, 2400, 2400, fill=FIELD_SURFACE_COLOR, outline="#666666", width=2)
        for x1, y1 in ((550, 500), (1400, 500), (550, 1450), (1400, 1450)):
            rectangle(x1, y1, x1 + 450, y1 + 450, fill=FIELD_PLATFORM_COLOR, outline="")

        line(1200, 0, 1200, 2400, fill="#5B5B5B", width=2, dash=(18, 12))
        line(0, 1200, 2400, 1200, fill="#5B5B5B", width=2, dash=(18, 12))
        for start_area, x, y in (
            (START_AREA_1, 2250, 150),
            (START_AREA_2, 2250, 2250),
        ):
            tag = f"start_area_{start_area}"
            selected = self._selected_start_area == start_area
            rectangle(
                x - 150,
                y - 150,
                x + 150,
                y + 150,
                fill=START_ZONE_COLOR,
                outline="#FFD23F" if selected else "",
                width=8 if selected else 1,
                tags=(tag,),
            )
            text(
                2100,
                420 if start_area == START_AREA_1 else 1980,
                f"启停区{start_area}",
                20,
                tags=(tag,),
            )
            canvas.tag_bind(tag, "<Button-1>", lambda _event, area=start_area: self._select_start_area(area))

        # 白色底板使暂存区和粗加工区的同心圆与场地底色清晰区分。
        rectangle(0, 950, 155, 1450, fill=TEXT_COLOR, outline="")
        rectangle(960, 2250, 1440, 2400, fill=TEXT_COLOR, outline="")

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

        horizontal_dimension(0, 2400, 2400, 2525, "2400")
        horizontal_dimension(1000, 1400, 500, 420, "400")
        horizontal_dimension(550, 1000, 950, 1020, "450")
        horizontal_dimension(2100, 2400, 0, -100, "300×300")
        vertical_dimension(0, 2400, 2400, 2480, "2400")
        vertical_dimension(0, 1200, 2400, 2560, "1100-1300")

    def _pick_font(self, *preferred_fonts: str) -> str:
        available = set(tkfont.families(self.root))
        selected = next((font for font in preferred_fonts if font in available), None)
        if selected is not None:
            return selected
        return str(tkfont.nametofont("TkDefaultFont").actual("family"))

    def _select_start_area(self, start_area: int) -> None:
        """保存地图选择，并刷新启动页提示和场地高亮。"""
        if start_area not in VALID_START_AREAS:
            return
        self._selected_start_area = start_area
        self._update_start_selection()
        self._draw_field_annotation()

    def _update_start_selection(self) -> None:
        if self._selected_start_area in VALID_START_AREAS:
            self._start_selection.configure(text=f"已选择：启停区 {self._selected_start_area}")
            self._start_button.configure(state="normal")
        else:
            self._start_selection.configure(text="未选择启停区")
            self._start_button.configure(state="disabled")

    def _on_start(self) -> None:
        if (
            self._start_clicked
            or self._start_callback is None
            or self._selected_start_area not in VALID_START_AREAS
        ):
            return
        self._start_clicked = True
        self._start_button.configure(state="disabled")
        try:
            started = self._start_callback(self._selected_start_area)
        except Exception:
            self._start_clicked = False
            self._update_start_selection()
            raise
        if started is False:
            self._start_clicked = False
            self._update_start_selection()

    def show_start_page(self) -> None:
        """显示启停区选择结果和开始按钮。"""
        self._running_frame.pack_forget()
        self._field_frame.pack_forget()
        if self._camera_frame is not None:
            self._camera_frame.pack_forget()
        self._start_frame.pack(fill="both", expand=True)
        self._current_page = "start"

    def show_running_page(self) -> None:
        """显示任务码、统计和计时区域。"""
        self._start_frame.pack_forget()
        self._field_frame.pack_forget()
        if self._camera_frame is not None:
            self._camera_frame.pack_forget()
        self._running_frame.pack(fill="both", expand=True)
        self._current_page = "running"

    def show_field_page(self) -> None:
        """显示启停区选择页，不改变比赛或硬件状态。"""
        self._start_frame.pack_forget()
        self._running_frame.pack_forget()
        if self._camera_frame is not None:
            self._camera_frame.pack_forget()
        self._field_frame.pack(fill="both", expand=True)
        self._current_page = "field"
        self._field_canvas.focus_set()
        self._draw_field_annotation()

    def show_camera_page(self) -> None:
        """显示相机预览；预览总开关关闭时保持当前页面。"""
        if not self._camera_preview_enabled or self._camera_frame is None:
            return
        self._start_frame.pack_forget()
        self._running_frame.pack_forget()
        self._field_frame.pack_forget()
        self._camera_frame.pack(fill="both", expand=True)
        self._current_page = "camera"
        self._render_camera_frame()

    def is_camera_page_visible(self) -> bool:
        """返回相机预览页是否为当前显示页面。"""
        return self._camera_preview_enabled and self._current_page == "camera"

    def select_camera(self, camera_id: str) -> None:
        """选择相机预览源；实际采集仍由主服务统一调度。"""
        if camera_id not in VALID_CAMERAS:
            return
        self._selected_camera = camera_id
        for button_id, button in self._camera_buttons.items():
            button.configure(bg="#454545" if button_id == camera_id else "#252525")

    def get_selected_camera(self) -> str:
        """返回当前需要展示的相机标识。"""
        return self._selected_camera

    def set_camera_frame(self, frame_rgb: np.ndarray, status_text: str = "") -> None:
        """更新 RGB 相机帧和状态文字，不访问相机设备。"""
        if not self._camera_preview_enabled or self._camera_label is None:
            return
        if not isinstance(frame_rgb, np.ndarray) or frame_rgb.ndim != 3 or frame_rgb.shape[2] != 3:
            raise ValueError("frame_rgb 必须是形状为 (height, width, 3) 的 RGB NumPy 数组")
        self._camera_image = Image.fromarray(frame_rgb)
        if self._camera_status is not None:
            self._camera_status.configure(text=status_text)
        self._render_camera_frame()

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

    def set_start_callback(self, callback: Callable[[int], bool | None]) -> None:
        """设置接收启停区编号的开始回调；返回 False 表示本次启动失败。"""
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

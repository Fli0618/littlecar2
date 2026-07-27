"""独立预览比赛显示界面。"""

from __future__ import annotations

import time
import tkinter as tk

from ui import CompetitionGUI


def main() -> None:
    root = tk.Tk()
    gui = CompetitionGUI(root)
    started_at: float | None = None

    def update_elapsed() -> None:
        if started_at is not None:
            gui.set_elapsed(int(time.monotonic() - started_at))
            root.after(200, update_elapsed)

    def start_preview() -> bool:
        nonlocal started_at
        gui.show_running_page()
        started_at = time.monotonic()
        update_elapsed()
        root.after(3000, lambda: gui.set_task_code("156+123+516+231"))
        root.after(3000, lambda: gui.set_counts(1, 1))
        return True

    gui.set_start_callback(start_preview)
    gui.run()


if __name__ == "__main__":
    main()

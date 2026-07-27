import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from ui import CompetitionGUI


class Widget:
    def __init__(self):
        self.calls = []
        self.options = {}

    def pack(self, **kwargs):
        self.calls.append(("pack", kwargs))

    def pack_forget(self):
        self.calls.append(("pack_forget", {}))

    def configure(self, **kwargs):
        self.options.update(kwargs)


def make_gui():
    gui = CompetitionGUI.__new__(CompetitionGUI)
    gui._start_frame = Widget()
    gui._running_frame = Widget()
    gui._task_code = Widget()
    gui._count_values = [Widget(), Widget(), Widget()]
    gui._start_clicked = False
    gui._start_callback = None
    return gui


def test_pages_switch_and_running_values_update():
    gui = make_gui()
    gui.show_start_page()
    gui.show_running_page()
    gui.set_task_code("156+123+516+231")
    gui.set_counts(1, 2)
    gui.set_elapsed(65)

    assert gui._task_code.options["text"] == "156+123+516+231"
    assert [value.options["text"] for value in gui._count_values] == ["1 / 6", "2 / 6", "01:05"]
    assert gui._start_frame.calls[-1][0] == "pack_forget"
    assert gui._running_frame.calls[-1][0] == "pack"


def test_start_callback_locks_after_success_and_allows_retry_after_failure():
    gui = make_gui()
    calls = []
    gui.set_start_callback(lambda: calls.append("failed") or False)
    gui._on_start()
    gui._on_start()
    assert calls == ["failed", "failed"]

    gui.set_start_callback(lambda: calls.append("started") or True)
    gui._on_start()
    gui._on_start()
    assert calls == ["failed", "failed", "started"]

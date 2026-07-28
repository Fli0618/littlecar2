import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from ui import CompetitionGUI
from protocol.commands import START_AREA_1, START_AREA_2


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

    def focus_set(self):
        self.calls.append(("focus_set", {}))


def make_gui():
    gui = CompetitionGUI.__new__(CompetitionGUI)
    gui._start_frame = Widget()
    gui._running_frame = Widget()
    gui._field_frame = Widget()
    gui._field_canvas = Widget()
    gui._task_code = Widget()
    gui._count_values = [Widget(), Widget(), Widget()]
    gui._start_button = Widget()
    gui._start_selection = Widget()
    gui._start_clicked = False
    gui._start_callback = None
    gui._selected_start_area = None
    gui._draw_field_annotation = lambda: None
    gui._update_start_selection()
    return gui


def test_pages_switch_and_running_values_update():
    gui = make_gui()
    gui.show_start_page()
    gui.show_field_page()
    gui.show_running_page()
    gui.set_task_code("156+123+516+231")
    gui.set_counts(1, 2)
    gui.set_elapsed(65)

    assert gui._task_code.options["text"] == "156+123+516+231"
    assert [value.options["text"] for value in gui._count_values] == ["1 / 6", "2 / 6", "01:05"]
    assert gui._start_frame.calls[-1][0] == "pack_forget"
    assert gui._running_frame.calls[-1][0] == "pack"
    assert gui._field_frame.calls[-1][0] == "pack_forget"


def test_field_page_returns_without_starting_competition():
    gui = make_gui()
    calls = []
    gui.set_start_callback(lambda start_area: calls.append(start_area) or True)
    gui.show_field_page()
    gui.show_start_page()

    assert calls == []
    assert gui._field_frame.calls[-1][0] == "pack_forget"
    assert gui._start_frame.calls[-1][0] == "pack"


def test_start_callback_locks_after_success_and_allows_retry_after_failure():
    gui = make_gui()
    calls = []
    gui._select_start_area(START_AREA_1)
    gui.set_start_callback(lambda start_area: calls.append(("failed", start_area)) or False)
    gui._on_start()
    gui._on_start()
    assert calls == [("failed", START_AREA_1), ("failed", START_AREA_1)]
    assert gui._start_button.options["state"] == "normal"

    gui.set_start_callback(lambda start_area: calls.append(("started", start_area)) or True)
    gui._on_start()
    gui._on_start()
    assert calls == [("failed", START_AREA_1), ("failed", START_AREA_1), ("started", START_AREA_1)]


def test_start_area_selection_updates_hint_enables_start_and_replaces_previous_choice():
    gui = make_gui()
    calls = []
    gui.set_start_callback(lambda start_area: calls.append(start_area) or True)

    assert gui._start_button.options["state"] == "disabled"
    assert gui._start_selection.options["text"] == "未选择启停区"

    gui._select_start_area(START_AREA_1)
    assert gui._selected_start_area == START_AREA_1
    assert gui._start_selection.options["text"] == "已选择：启停区 1"
    assert gui._start_button.options["state"] == "normal"

    gui._select_start_area(START_AREA_2)
    gui._on_start()
    assert gui._selected_start_area == START_AREA_2
    assert calls == [START_AREA_2]

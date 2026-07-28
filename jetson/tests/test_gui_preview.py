import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


def test_preview_start_callback_accepts_selected_start_area(monkeypatch):
    module_path = ROOT / "scripts" / "gui_preview.py"
    spec = importlib.util.spec_from_file_location("gui_preview", module_path)
    assert spec is not None and spec.loader is not None
    preview = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(preview)

    class Root:
        def __init__(self):
            self.after_calls = []

        def after(self, delay, callback):
            self.after_calls.append((delay, callback))

        def bind(self, event, callback):
            self.bound_event = (event, callback)

    class Gui:
        instance = None

        def __init__(self, root):
            self.root = root
            self.running = 0
            self.callback = None
            Gui.instance = self

        def show_running_page(self):
            self.running += 1

        def set_elapsed(self, seconds):
            self.elapsed = seconds

        def set_task_code(self, code):
            self.task_code = code

        def set_counts(self, pick_count, place_count):
            self.counts = (pick_count, place_count)

        def set_start_callback(self, callback):
            self.callback = callback

        def show_field_page(self):
            pass

        def run(self):
            pass

    monkeypatch.setattr(preview.tk, "Tk", Root)
    monkeypatch.setattr(preview, "CompetitionGUI", Gui)

    preview.main()

    assert Gui.instance is not None
    assert Gui.instance.callback(1) is True
    assert Gui.instance.running == 1

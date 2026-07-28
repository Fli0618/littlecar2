import importlib.util
import sys
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


def load_preview_module():
    module_path = ROOT / "scripts" / "gui_preview.py"
    spec = importlib.util.spec_from_file_location("gui_preview", module_path)
    assert spec is not None and spec.loader is not None
    preview = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(preview)
    return preview


class Root:
    def __init__(self):
        self.after_calls = []
        self.bindings = {}

    def after(self, delay, callback):
        self.after_calls.append((delay, callback))

    def bind(self, event, callback):
        self.bindings[event] = callback


class Gui:
    instance = None

    def __init__(self, root, *, camera_preview_enabled):
        self.root = root
        self.camera_preview_enabled = camera_preview_enabled
        self.running = 0
        self.camera = 0
        self.field = 0
        self.camera_visible = False
        self.callback = None
        Gui.instance = self

    def show_running_page(self):
        self.running += 1
        self.camera_visible = False

    def show_camera_page(self):
        self.camera += 1
        self.camera_visible = True

    def is_camera_page_visible(self):
        return self.camera_visible

    def show_field_page(self):
        self.field += 1
        self.camera_visible = False

    def set_elapsed(self, seconds):
        self.elapsed = seconds

    def set_task_code(self, code):
        self.task_code = code

    def set_counts(self, pick_count, place_count):
        self.counts = (pick_count, place_count)

    def set_camera_frame(self, frame_rgb, status_text):
        self.frame_rgb = frame_rgb
        self.status_text = status_text

    def set_start_callback(self, callback):
        self.callback = callback

    def run(self):
        pass


def test_preview_start_callback_accepts_selected_start_area(monkeypatch):
    preview = load_preview_module()
    root = Root()
    monkeypatch.setattr(preview.tk, "Tk", lambda: root)
    monkeypatch.setattr(preview, "CompetitionGUI", Gui)
    monkeypatch.setattr(preview, "render_camera_preview", lambda frame, mode, **_kwargs: frame[..., ::-1])

    preview.main()

    assert Gui.instance is not None
    assert Gui.instance.callback(1) is True
    assert Gui.instance.running == 1
    assert Gui.instance.frame_rgb.shape == (preview.FRAME_HEIGHT, preview.FRAME_WIDTH, 3)


def test_function_keys_switch_modes_sessions_and_pages(monkeypatch):
    preview = load_preview_module()
    root = Root()
    monkeypatch.setattr(preview.tk, "Tk", lambda: root)
    monkeypatch.setattr(preview, "CompetitionGUI", Gui)
    calls = []
    monkeypatch.setattr(
        preview,
        "render_camera_preview",
        lambda frame, mode, **kwargs: calls.append((mode, kwargs["status_text"], kwargs["result"])) or frame[..., ::-1],
    )

    preview.main()
    for key, expected_mode in (
        ("<F3>", preview.CMD_START_COLOR),
        ("<F4>", preview.CMD_START_CIRCLE),
        ("<F5>", preview.CMD_START_DISK_CENTER),
        ("<F6>", preview.CMD_START_QR),
    ):
        root.bindings[key](None)
        assert calls[-1][0] == expected_mode
        assert "模拟会话" in calls[-1][1]
        assert calls[-1][2] is not None
        assert Gui.instance.camera_visible is True

    root.bindings["<F2>"](None)
    assert Gui.instance.field == 1
    root.bindings["<F7>"](None)
    assert Gui.instance.running == 1
    assert Gui.instance.camera_visible is False


def test_compose_preview_frame_is_bgr_uint8_and_changes_with_frame_index():
    preview = load_preview_module()

    first = preview.compose_preview_frame(0, 1)
    later = preview.compose_preview_frame(9, 1)

    assert first.dtype == np.uint8
    assert first.shape == (preview.FRAME_HEIGHT, preview.FRAME_WIDTH, 3)
    assert not np.array_equal(first, later)


def test_simulate_visual_result_covers_each_camera_mode():
    preview = load_preview_module()

    assert "detections" in preview.simulate_visual_result(preview.CMD_START_COLOR, 0, 1)
    assert "detections" in preview.simulate_visual_result(preview.CMD_START_CIRCLE, 0, 1)
    assert "center" in preview.simulate_visual_result(preview.CMD_START_DISK_CENTER, 0, 1)
    assert preview.simulate_visual_result(preview.CMD_START_QR, 0, 1)["code"] == "156+123+516+231"


def test_legacy_preview_aliases_remain_usable():
    preview = load_preview_module()

    assert preview.compose_preview_frame(0).shape == (preview.FRAME_HEIGHT, preview.FRAME_WIDTH, 3)
    assert preview.simulate_result(preview.CMD_START_QR, 0)["code"] == "156+123+516+231"

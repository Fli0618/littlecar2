from pathlib import Path

import numpy as np
import pytest
from PIL import ImageFont

from protocol.commands import CMD_START_CIRCLE, CMD_START_COLOR, CMD_START_DISK_CENTER, CMD_START_QR
from vision import render_camera_preview
import vision.preview as preview


def test_preview_returns_independent_rgb_frame_and_preserves_input():
    frame_bgr = np.zeros((80, 120, 3), dtype=np.uint8)
    frame_bgr[0, 0] = (1, 2, 3)
    original = frame_bgr.copy()

    preview_rgb = render_camera_preview(frame_bgr, 0, aim_offset=(9, -4))

    assert np.array_equal(frame_bgr, original)
    assert preview_rgb.shape == frame_bgr.shape
    assert preview_rgb.dtype == np.uint8
    assert preview_rgb[0, 0].tolist() == [3, 2, 1]
    assert not np.shares_memory(preview_rgb, frame_bgr)
    assert preview_rgb[36, 69].tolist() == [0, 255, 0]


@pytest.mark.parametrize(
    ("mode", "result"),
    [
        (CMD_START_QR, {"raw_code": "156+123+516+231", "status": "FIRST_DETECTED", "code": "156+123+516+231"}),
        (CMD_START_COLOR, {"detections": [{"type": 2, "center": [25, 25], "confidence": 0.9, "measured": True, "support_count": 3}]}),
        (CMD_START_CIRCLE, {"detections": [{"type": 4, "center": [30, 30], "confidence": 0.8, "measured": False, "support_count": 2}]}),
        (CMD_START_DISK_CENTER, {"support_points": [[20, 20], [40, 20]], "center": [30, 30], "support_count": 2, "measured_count": 1}),
    ],
)
def test_preview_renders_each_detection_mode(mode, result):
    preview_rgb = render_camera_preview(np.zeros((100, 140, 3), dtype=np.uint8), mode, result)

    assert preview_rgb.any()


def test_preview_draws_roi_boundary_for_color_and_global_label_for_circle():
    frame = np.zeros((100, 140, 3), dtype=np.uint8)

    color_preview = render_camera_preview(frame, CMD_START_COLOR, roi_bounds=(0, 0, 140, 75))
    circle_preview = render_camera_preview(frame, CMD_START_CIRCLE, roi_bounds=(0, 0, 140, 100))

    # ROI line is drawn in BGR yellow and then converted to RGB.
    assert color_preview[74, 20].tolist() == [255, 200, 0]
    assert circle_preview.any()


def test_preview_rejects_invalid_camera_frame():
    with pytest.raises(TypeError, match="BGR numpy.ndarray"):
        render_camera_preview(np.zeros((10, 10), dtype=np.uint8), 0)


def test_text_renderer_routes_unicode_away_from_opencv(monkeypatch):
    frame = np.zeros((40, 120, 3), dtype=np.uint8)
    unicode_calls = []
    opencv_calls = []
    monkeypatch.setattr(preview, "_draw_unicode_text", lambda *args: unicode_calls.append(args))
    monkeypatch.setattr(preview.cv2, "putText", lambda *args: opencv_calls.append(args))

    preview._draw_text(frame, "相机预览", (4, 24), (255, 255, 255))

    assert len(unicode_calls) == 1
    assert opencv_calls == []


def test_text_renderer_keeps_ascii_on_opencv(monkeypatch):
    frame = np.zeros((40, 120, 3), dtype=np.uint8)
    unicode_calls = []
    opencv_calls = []
    monkeypatch.setattr(preview, "_draw_unicode_text", lambda *args: unicode_calls.append(args))
    monkeypatch.setattr(preview.cv2, "putText", lambda *args: opencv_calls.append(args))

    preview._draw_text(frame, "QR status: confirmed", (4, 24), (255, 255, 255))

    assert len(opencv_calls) == 1
    assert unicode_calls == []


def test_fontconfig_resolver_returns_existing_font_path(monkeypatch):
    font_path = Path(preview.__file__)

    class Result:
        stdout = f"{font_path}\n"

    monkeypatch.setattr(preview.subprocess, "run", lambda *args, **kwargs: Result())
    preview._resolve_cjk_font_path.cache_clear()

    assert preview._resolve_cjk_font_path() == font_path


def test_fontconfig_resolver_returns_none_when_command_is_unavailable(monkeypatch):
    def unavailable(*_args, **_kwargs):
        raise FileNotFoundError

    monkeypatch.setattr(preview.subprocess, "run", unavailable)
    preview._resolve_cjk_font_path.cache_clear()

    assert preview._resolve_cjk_font_path() is None


def test_unicode_renderer_clips_text_at_frame_edges(monkeypatch):
    frame = np.zeros((12, 12, 3), dtype=np.uint8)
    font = ImageFont.load_default()
    monkeypatch.setattr(preview, "_load_unicode_font", lambda _size: font)

    preview._draw_unicode_text(frame, "A", (-2, 8), (255, 255, 255))

    assert frame.any()

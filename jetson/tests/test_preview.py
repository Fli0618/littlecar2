import numpy as np
import pytest

from protocol.commands import CMD_START_CIRCLE, CMD_START_COLOR, CMD_START_DISK_CENTER, CMD_START_QR
from vision import render_camera_preview


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


def test_preview_rejects_invalid_camera_frame():
    with pytest.raises(TypeError, match="BGR numpy.ndarray"):
        render_camera_preview(np.zeros((10, 10), dtype=np.uint8), 0)

from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tools.hsv_tuner.hsv_tuner_gui import (
    build_histogram_dashboard,
    canvas_point_to_image,
    compose_square_tile,
    create_tuner_state,
    image_point_to_canvas,
    imwrite_unicode,
    render_hs_heatmap_tile,
    render_hsv_histogram_tile,
    safe_hsv_range,
)
from vision.hsv_color import HSVColorProfile, HSVConfig, HSVProcessingConfig, HSVRange, HSVSamplingConfig


def _config() -> HSVConfig:
    return HSVConfig(
        1,
        (HSVColorProfile("red", 0, True, (HSVRange((0, 70, 50), (10, 255, 255)),)),),
        HSVProcessingConfig(0, 0, 0, 0, 0),
        HSVSamplingConfig(1.0, 1.0, 1, 0.1, 0.1),
    )


def test_create_tuner_state_uses_requested_color_and_config(tmp_path):
    config = _config()
    state = create_tuner_state([tmp_path / "frame1.png"], tmp_path / "hsv.json", config=config, color_name="red")

    assert state.config_path == tmp_path / "hsv.json"
    assert state.color_name == "red"
    assert state.working_config == config


def test_histogram_dashboard_renders_roi_and_threshold_guides():
    hsv = np.zeros((40, 60, 3), dtype=np.uint8)
    hsv[:, :, 0] = 5
    hsv[:, :, 1] = 180
    hsv[:, :, 2] = 220
    profile = _config().colors[0]

    dashboard = build_histogram_dashboard(hsv, (10, 10, 30, 30), profile, width=640, height=480)

    assert dashboard.shape == (480, 640, 3)
    assert dashboard.dtype == np.uint8
    assert int(np.count_nonzero(dashboard)) > 0


def test_safe_hsv_range_repairs_crossed_slider_bounds_independently():
    repaired = safe_hsv_range((80, 200, 240), (20, 40, 100))

    assert repaired.lower == (20, 40, 100)
    assert repaired.upper == (80, 200, 240)


def test_square_tile_contains_and_round_trips_coordinates():
    image = np.zeros((40, 120, 3), dtype=np.uint8)
    tile, transform = compose_square_tile(image, 220, "原图")

    assert tile.shape == (220, 220, 3)
    canvas_point = image_point_to_canvas((60, 20), transform)
    assert canvas_point is not None
    image_point = canvas_point_to_image(canvas_point, transform)
    assert image_point is not None
    assert abs(image_point[0] - 60) <= 1
    assert abs(image_point[1] - 20) <= 1
    assert canvas_point_to_image((0, 0), transform) is None


def test_direct_chart_tiles_use_roi_mask_and_have_requested_size():
    hsv = np.zeros((30, 50, 3), dtype=np.uint8)
    hsv[:, :, 0] = 10
    hsv[:, :, 1] = 220
    hsv[:, :, 2] = 200
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    mask[5:25, 10:40] = 255
    profile = _config().colors[0]

    histogram = render_hsv_histogram_tile(hsv, mask, profile, 180)
    heatmap = render_hs_heatmap_tile(hsv, mask, profile, 180)

    assert histogram.shape == (180, 180, 3)
    assert heatmap.shape == (180, 180, 3)
    assert np.count_nonzero(histogram) > 0
    assert np.count_nonzero(heatmap) > 0


def test_unicode_image_export(tmp_path):
    target = tmp_path / "中文" / "mask.png"
    target.parent.mkdir()
    assert imwrite_unicode(target, np.zeros((4, 5), dtype=np.uint8))
    assert target.is_file()

import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tools.hsv_tuner.hsv_tuner import (
    DEFAULT_CONFIG_PATH,
    import_hsv_config,
    clamp_index,
    discover_image_paths,
    natural_sort_key,
    normalize_roi,
    output_filename,
    read_image_file,
    resolve_default_config_path,
    scale_preview,
)

from vision.hsv_color import HSVColorProfile, HSVConfig, HSVProcessingConfig, HSVRange, HSVSamplingConfig, save_hsv_config


def test_natural_sort_key_orders_numeric_fragments_as_numbers():
    paths = [Path("image10.jpg"), Path("image2.jpg"), Path("image1.jpg")]

    assert sorted(paths, key=natural_sort_key) == [Path("image1.jpg"), Path("image2.jpg"), Path("image10.jpg")]


def test_discover_image_paths_filters_extensions_and_uses_natural_order(tmp_path):
    for name in ("frame10.JPG", "frame2.png", "frame1.jpeg", "notes.txt"):
        (tmp_path / name).write_bytes(b"")
    nested = tmp_path / "nested"
    nested.mkdir()
    (nested / "frame3.jpg").write_bytes(b"")

    direct = discover_image_paths(tmp_path)
    recursive = discover_image_paths(tmp_path, recursive=True)

    assert [path.name for path in direct] == ["frame1.jpeg", "frame2.png", "frame10.JPG"]
    assert [path.name for path in recursive] == ["frame1.jpeg", "frame2.png", "frame10.JPG", "frame3.jpg"]


def test_discover_image_paths_rejects_non_directory(tmp_path):
    with pytest.raises(ValueError, match="directory"):
        discover_image_paths(tmp_path / "missing")


@pytest.mark.parametrize(
    ("index", "count", "expected"),
    [(-2, 3, 0), (1, 3, 1), (9, 3, 2), (1, 0, 0)],
)
def test_clamp_index(index, count, expected):
    assert clamp_index(index, count) == expected


@pytest.mark.parametrize(
    ("start", "end", "expected"),
    [((8, 7), (2, 1), (2, 1, 9, 8)), ((-1, -1), (12, 12), (0, 0, 10, 10)), ((2, 2), (3, 3), None)],
)
def test_normalize_roi_clips_reverse_drag_and_rejects_tiny_areas(start, end, expected):
    assert normalize_roi(start, end, 10, 10) == expected


def test_scale_preview_preserves_aspect_ratio_and_input_independence():
    image = np.zeros((200, 800, 3), dtype=np.uint8)

    preview, scale = scale_preview(image, max_width=400, max_height=300)

    assert scale == pytest.approx(0.5)
    assert preview.shape == (100, 400, 3)
    assert not np.shares_memory(image, preview)


def test_scale_preview_keeps_small_image_at_original_size():
    image = np.zeros((10, 20, 3), dtype=np.uint8)

    preview, scale = scale_preview(image)

    assert scale == 1.0
    assert preview.shape == image.shape
    assert not np.shares_memory(image, preview)


@pytest.mark.parametrize(
    ("path", "kind", "color", "expected"),
    [
        ("frame12.jpg", "preview", None, "frame12_preview.png"),
        (Path("nested/scene.png"), "mask", "red", "scene_red_mask.png"),
    ],
)
def test_output_filename_is_stable_and_does_not_reuse_input_extension(path, kind, color, expected):
    assert output_filename(path, kind, color) == expected


def test_read_image_file_supports_unicode_windows_paths(tmp_path):
    image = np.zeros((8, 10, 3), dtype=np.uint8)
    image[:, :, 1] = 180
    path = tmp_path / "物料照片" / "测试图片.jpg"
    path.parent.mkdir()
    ok, encoded = cv2.imencode(".jpg", image)
    assert ok
    path.write_bytes(encoded.tobytes())

    loaded = read_image_file(path)

    assert loaded is not None
    assert loaded.shape == image.shape


def _import_config() -> HSVConfig:
    return HSVConfig(
        1,
        (
            HSVColorProfile("red", 0, True, (HSVRange((0, 100, 100), (10, 255, 255)),)),
            HSVColorProfile("blue", 2, False, (HSVRange((100, 100, 100), (130, 255, 255)),)),
        ),
        HSVProcessingConfig(0, 0, 0, 0, 0),
        HSVSamplingConfig(1.0, 1.0, 1, 0.1, 0.1),
    )


def test_default_config_path_is_based_on_module_location_not_cwd():
    assert resolve_default_config_path() == DEFAULT_CONFIG_PATH
    assert resolve_default_config_path().name == "hsv_colors.json"
    assert resolve_default_config_path().parts[-4:] == ("jetson", "assets", "config", "hsv_colors.json")


def test_import_hsv_config_preserves_or_falls_back_selected_color(tmp_path):
    target = tmp_path / "config.json"
    save_hsv_config(_import_config(), target)

    imported = import_hsv_config(target, "red")
    fallback = import_hsv_config(target, "missing")

    assert imported.config_path == target.resolve()
    assert imported.selected_color == "red"
    assert fallback.selected_color == "red"
    assert imported.is_default is False

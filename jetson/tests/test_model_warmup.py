"""Jetson 主服务模型预热测试。"""

from __future__ import annotations

import numpy as np

import main as jetson_main


def test_warmup_vision_models_calls_both_detectors(monkeypatch) -> None:
    calls: list[tuple[str, tuple[int, ...], np.dtype]] = []

    monkeypatch.setattr(
        jetson_main,
        "detect_color",
        lambda frame: calls.append(("color", frame.shape, frame.dtype)),
    )
    monkeypatch.setattr(
        jetson_main,
        "detect_circle",
        lambda frame: calls.append(("circle", frame.shape, frame.dtype)),
    )

    jetson_main.warmup_vision_models()

    assert calls == [
        ("color", (640, 640, 3), np.dtype(np.uint8)),
        ("circle", (640, 640, 3), np.dtype(np.uint8)),
    ]

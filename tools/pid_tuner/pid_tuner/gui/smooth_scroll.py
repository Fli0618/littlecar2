"""Smooth, scroll-wheel-friendly areas for long control panels."""

from __future__ import annotations

from PySide6.QtCore import QAbstractAnimation, QEasingCurve, QPropertyAnimation
from PySide6.QtGui import QWheelEvent
from PySide6.QtWidgets import QScrollArea, QWidget


class SmoothScrollArea(QScrollArea):
    """Animate discrete mouse-wheel steps while preserving touchpad pixels."""

    WHEEL_DISTANCE_PX = 84
    ANIMATION_DURATION_MS = 140

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._target_value = 0
        self._animation = QPropertyAnimation(
            self.verticalScrollBar(), b"value", self
        )
        self._animation.setDuration(self.ANIMATION_DURATION_MS)
        self._animation.setEasingCurve(QEasingCurve.Type.OutCubic)
        self.verticalScrollBar().setSingleStep(24)
        self.verticalScrollBar().sliderPressed.connect(self.stop_smooth_scroll)

    def stop_smooth_scroll(self) -> None:
        """Stop an in-flight animation before direct scrollbar interaction."""
        self._animation.stop()
        self._target_value = self.verticalScrollBar().value()

    def scroll_by_pixels(self, distance: int, *, animated: bool) -> None:
        """Move vertically by content pixels, clamped to the scrollbar range."""
        bar = self.verticalScrollBar()
        if (animated and
                self._animation.state() == QAbstractAnimation.State.Running):
            start_target = self._target_value
        else:
            start_target = bar.value()
        target = max(bar.minimum(), min(bar.maximum(), start_target + distance))
        self._target_value = target

        if not animated:
            self._animation.stop()
            bar.setValue(target)
            return

        self._animation.stop()
        self._animation.setStartValue(bar.value())
        self._animation.setEndValue(target)
        self._animation.start()

    def wheelEvent(self, event: QWheelEvent) -> None:  # noqa: N802 - Qt override
        """Handle high-resolution touchpads directly and animate wheel notches."""
        pixel_delta = event.pixelDelta().y()
        if pixel_delta:
            self.scroll_by_pixels(-pixel_delta, animated=False)
            event.accept()
            return

        angle_delta = event.angleDelta().y()
        if angle_delta:
            distance = round(-angle_delta * self.WHEEL_DISTANCE_PX / 120.0)
            self.scroll_by_pixels(distance, animated=True)
            event.accept()
            return

        super().wheelEvent(event)

import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication, QLabel, QWidget

from pid_tuner.gui.smooth_scroll import SmoothScrollArea


class SmoothScrollAreaTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def test_discrete_scroll_is_animated_and_clamped(self) -> None:
        area = SmoothScrollArea()
        content = QWidget()
        content.setMinimumSize(200, 1200)
        QLabel("content", content)
        area.setWidget(content)
        area.resize(220, 240)
        area.show()
        self.app.processEvents()
        try:
            area.scroll_by_pixels(180, animated=True)
            QTest.qWait(area.ANIMATION_DURATION_MS + 30)
            self.assertEqual(area.verticalScrollBar().value(), 180)

            area.scroll_by_pixels(100_000, animated=False)
            self.assertEqual(
                area.verticalScrollBar().value(), area.verticalScrollBar().maximum()
            )
        finally:
            area.close()


if __name__ == "__main__":
    unittest.main()

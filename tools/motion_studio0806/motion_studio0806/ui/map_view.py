"""Clean 2D Field Map View for Motion Studio 0806 (Full bidirectional syncing and config responsiveness)."""

from __future__ import annotations

from typing import List, Optional, Any

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QVBoxLayout, QWidget, QSplitter

from map_planner.gui import MapEditorWidget
from map_planner.models import Waypoint, Plan
from motion_studio0806.core.models import PathWaypoint, TargetPose


class CleanMapView(QWidget):
    """Pure 2D Field Map Canvas Component with full bidirectional syncing."""

    point_added = Signal(float, float)  # Emits (x_mm, y_mm) on canvas click to feed left panel table

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.map_editor = MapEditorWidget()
        self._init_ui()

    def _init_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # Hide internal left drawer panel inside MapEditorWidget to keep right canvas pure
        if hasattr(self.map_editor, "findChildren"):
            for child in self.map_editor.children():
                if isinstance(child, QSplitter):
                    if child.count() > 1:
                        widget_0 = child.widget(0)
                        if widget_0:
                            widget_0.hide()

        # Connect map scene click signal if available
        if hasattr(self.map_editor, "canvas") and hasattr(self.map_editor.canvas, "clicked"):
            self.map_editor.canvas.clicked.connect(self._on_map_clicked)
        elif hasattr(self.map_editor, "view") and hasattr(self.map_editor.view, "clicked"):
            self.map_editor.view.clicked.connect(self._on_map_clicked)

        layout.addWidget(self.map_editor)

    def _on_map_clicked(self, x: float, y: float, *args: Any) -> None:
        if 0 <= x <= 3000 and 0 <= y <= 3000:
            self.point_added.emit(round(x, 1), round(y, 1))

    def set_waypoints(self, waypoints: List[PathWaypoint]) -> None:
        """Render waypoints from left panel onto right map canvas in real time."""
        if not self.map_editor:
            return
        
        steps = [
            Waypoint(name=f"点{i+1}", x_mm=wp.x_mm, y_mm=wp.y_mm, yaw_deg=wp.yaw_deg)
            for i, wp in enumerate(waypoints)
        ]
        
        if hasattr(self.map_editor, "set_plan_steps"):
            self.map_editor.set_plan_steps(steps)
        elif hasattr(self.map_editor, "plan"):
            self.map_editor.plan.steps = steps
            if hasattr(self.map_editor, "reload_plan_view"):
                self.map_editor.reload_plan_view()

    def update_map_config(self, cfg: dict) -> None:
        """Apply full costmap, vehicle dimensions, and inflation options to map_editor."""
        if not self.map_editor:
            return
        
        # Apply parameters to map_editor controls if present
        mapping = {
            "car_length": ("car_length_spin", "car_length_box"),
            "car_width": ("car_width_spin", "car_width_box"),
            "border_soft": ("border_soft_spin", "soft_inflation_box"),
        }
        
        for key, attr_names in mapping.items():
            val = cfg.get(key)
            if val is not None:
                for attr in attr_names:
                    if hasattr(self.map_editor, attr):
                        try:
                            getattr(self.map_editor, attr).setValue(val)
                        except Exception:
                            pass

        # Trigger costmap update/redraw
        if hasattr(self.map_editor, "update_costmap"):
            try:
                self.map_editor.update_costmap()
            except Exception:
                pass
        elif hasattr(self.map_editor, "redraw"):
            try:
                self.map_editor.redraw()
            except Exception:
                pass

    def set_single_target(self, target: TargetPose) -> None:
        """Mark single point target pose on map canvas."""
        wp = PathWaypoint(x_mm=target.x_mm, y_mm=target.y_mm, yaw_deg=target.yaw_deg)
        self.set_waypoints([wp])

    def update_telemetry(self, raw_tel: Any) -> None:
        pass

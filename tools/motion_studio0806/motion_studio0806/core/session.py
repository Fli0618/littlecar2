"""Session adapter for Motion Studio 0806.

Provides a clean interface connecting UI widgets with underlying serial session controller.
"""

from __future__ import annotations

import logging
from typing import List, Optional

from PySide6.QtCore import QObject, Signal, QTimer

from pid_tuner.gui.session import SessionController
from pid_tuner.models import HolonomicTelemetry, Telemetry, MotionGoal
from motion_studio0806.core.models import (
    ControlMode, HolonomicParams, PathWaypoint, TargetPose, TelemetryFrame
)

logger = logging.getLogger(__name__)


class StudioSession(QObject):
    """Facade for serial communication and telemetry routing."""

    telemetry_updated = Signal(object)      # Emits TelemetryFrame
    raw_telemetry_updated = Signal(object)  # Emits raw Telemetry object for TelemetryPlots
    connection_changed = Signal(bool, str) # connected, port_name
    status_message = Signal(str)           # Status bar message
    mode_changed = Signal(str)             # Active mode

    def __init__(self, parent: Optional[QObject] = None) -> None:
        super().__init__(parent)
        self.session = SessionController()
        self.active_mode: ControlMode = ControlMode.NONE
        self.connected = False

        # Connect signals from session controller
        self.session.telemetry.connect(self._on_pid_telemetry)
        self.session.status.connect(self.status_message)
        if hasattr(self.session, "holonomic_telemetry"):
            self.session.holonomic_telemetry.connect(self._on_holonomic_telemetry)
        if hasattr(self.session, "path_telemetry"):
            self.session.path_telemetry.connect(self._on_path_telemetry)

    def connect_serial(self, port: str, baudrate: int = 115200) -> bool:
        """Connect to target serial port."""
        try:
            self.session.connect_port(port, baudrate)
            self.connected = True
            self.connection_changed.emit(True, port)
            return True
        except Exception as e:
            logger.exception("Failed to connect serial port: %s", e)
            self.connection_changed.emit(False, "")
            return False

    def disconnect_serial(self) -> None:
        """Disconnect active serial port."""
        try:
            self.session.disconnect()
        except Exception as e:
            logger.warning("Error disconnecting serial: %s", e)
        self.connected = False
        self.connection_changed.emit(False, "")

    def emergency_stop(self) -> None:
        """Send emergency stop command to car."""
        try:
            self.session.cancel_motion()
            self.session.emergency_stop()
        except Exception as e:
            logger.error("Error executing emergency stop: %s", e)
        self.status_message.emit("EMERGENCY STOP EXECUTED!")

    def set_single_point(self, target: TargetPose, mode: ControlMode = ControlMode.HOLONOMIC) -> bool:
        """Send a single target point to car."""
        self.active_mode = mode
        self.mode_changed.emit(mode.value)
        try:
            goal = MotionGoal(
                x_mm=target.x_mm,
                y_mm=target.y_mm,
                yaw_deg=target.yaw_deg,
                v_max_mm_s=target.v_max_mm_s,
                w_max_deg_s=target.w_max_deg_s,
                use_position=target.use_position,
                use_yaw=target.use_yaw,
            )
            if mode == ControlMode.HOLONOMIC:
                return self.session.send_holonomic_goal(goal)
            else:
                return self.session.send_goal(goal)
        except Exception as e:
            logger.error("Failed to send single point goal: %s", e)
            return False

    def send_path(self, waypoints: List[PathWaypoint]) -> bool:
        """Send path waypoint array to car."""
        self.active_mode = ControlMode.WORLD_PATH
        self.mode_changed.emit(ControlMode.WORLD_PATH.value)
        try:
            # Map PathWaypoint to MotionGoal format expected by session
            goals = [
                MotionGoal(
                    x_mm=wp.x_mm,
                    y_mm=wp.y_mm,
                    yaw_deg=wp.yaw_deg,
                    v_max_mm_s=wp.v_max_mm_s,
                    use_position=True,
                    use_yaw=wp.lock_yaw,
                )
                for wp in waypoints
            ]
            if hasattr(self.session, "send_path_waypoints"):
                return self.session.send_path_waypoints(goals)
            elif hasattr(self.session, "upload_path"):
                return self.session.upload_path(goals)
            return False
        except Exception as e:
            logger.error("Failed to send path: %s", e)
            return False

    def _on_pid_telemetry(self, raw_tel: Telemetry) -> None:
        self.raw_telemetry_updated.emit(raw_tel)
        frame = TelemetryFrame(
            timestamp=raw_tel.timestamp,
            x_mm=getattr(raw_tel, "x_mm", 0.0),
            y_mm=getattr(raw_tel, "y_mm", 0.0),
            yaw_deg=getattr(raw_tel, "yaw_deg", 0.0),
            v_actual_mm_s=getattr(raw_tel, "v_actual_mm_s", 0.0),
            active_mode=self.active_mode.value,
        )
        self.telemetry_updated.emit(frame)

    def _on_holonomic_telemetry(self, raw_tel: HolonomicTelemetry) -> None:
        frame = TelemetryFrame(
            timestamp=getattr(raw_tel, "timestamp", 0.0),
            x_mm=getattr(raw_tel, "x_mm", 0.0),
            y_mm=getattr(raw_tel, "y_mm", 0.0),
            yaw_deg=getattr(raw_tel, "yaw_deg", 0.0),
            v_actual_mm_s=getattr(raw_tel, "v_actual_mm_s", 0.0),
            active_mode=ControlMode.HOLONOMIC.value,
        )
        self.telemetry_updated.emit(frame)

    def _on_path_telemetry(self, raw_path_tel: Any) -> None:
        frame = TelemetryFrame(
            timestamp=getattr(raw_path_tel, "timestamp", 0.0),
            x_mm=getattr(raw_path_tel, "x_mm", 0.0),
            y_mm=getattr(raw_path_tel, "y_mm", 0.0),
            yaw_deg=getattr(raw_path_tel, "yaw_deg", 0.0),
            cross_track_error_mm=getattr(raw_path_tel, "cross_track_error_mm", 0.0),
            v_actual_mm_s=getattr(raw_path_tel, "v_actual_mm_s", 0.0),
            active_mode=ControlMode.WORLD_PATH.value,
        )
        self.telemetry_updated.emit(frame)

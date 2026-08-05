"""比赛地图直线 GOTO Pose 编辑器。"""

from __future__ import annotations

import copy
import math
import sys
from dataclasses import replace
from typing import TYPE_CHECKING

from PySide6.QtCore import QPoint, QPointF, QRect, QRectF, QTimer, Qt, Signal
from PySide6.QtGui import QColor, QFont, QKeySequence, QPainter, QPainterPath, QPen, QPolygonF, QShortcut
from PySide6.QtWidgets import (QApplication, QButtonGroup, QCheckBox, QDoubleSpinBox, QFormLayout,
    QGraphicsEllipseItem, QGraphicsItem, QGraphicsLineItem, QGraphicsPathItem, QGraphicsPolygonItem, QGraphicsRectItem,
    QGraphicsScene, QGraphicsTextItem, QGraphicsView, QHBoxLayout, QInputDialog, QLabel, QListWidget,
    QGroupBox, QMainWindow, QMessageBox, QPushButton, QScrollArea, QSlider,
    QSplitter, QTabWidget, QVBoxLayout, QWidget, QRubberBand, QComboBox)

from .geometry import (StartFrame, paper_heading_to_world_yaw, paper_to_world, paper_vector_to_heading,
                       qgraphics_rotation_deg, world_to_paper, world_yaw_to_paper_heading,
                       rebase_plan_world_frame, wrap_deg)
from .codegen_c import CodeGenerationError, validate_plan_for_blocking_codegen
from .codegen_dialog import CodeGenerationDialog
from .bezier import bezier_tangent_yaw, generate_bezier_path_points
from .auto_path import (PLATFORM_GROUP_BOUNDS, AutoPathError, AutoPathSettings,
                        boundary_inset_rects, build_inflated_obstacles,
                        plan_auto_path)
from .models import (AutoSegmentSettings, BezierPathSegment, FIELD_SIZE_MM,
                     VEHICLE_WIDTH_MM,
                     ContinuousPathSegment, CostmapSettings, Obstacle,
                     PathPosePoint, Plan, Pose, RotateInPlace, Waypoint)
from .sim import SimulationFrame, build_plan_timeline
from .sweep import SweepGeometry, build_continuous_segment_sweep, build_goto_sweep, build_rotation_sweep, car_polygon
from .storage import list_plans, load_plan, rename_plan, save_plan

if TYPE_CHECKING:
    from motion_workbench.models import RuntimeUiSnapshot

PLATFORMS = ((550, 550), (1400, 550), (550, 1400), (1400, 1400))
MATERIAL_SLOTS = ((75, 1050), (75, 1200), (75, 1350), (1050, 2325), (1200, 2325), (1350, 2325))
START_PRESETS = {"启停区 1": (2250.0, 150.0), "启停区 2": (2250.0, 2250.0)}
RAW_CENTER_Y_MM = -70.0
RAW_CENTER_X_RANGE = (1100.0, 1300.0)
QR_CENTER_X_MM = 2396.0
QR_CENTER_Y_RANGE = (1100.0, 1300.0)
CENTERLINE_SNAP_MM = 35.0
RIGHT_ANGLE_SNAP_DEG = 8.0


def snap_to_field_centerlines(point: QPointF) -> QPointF:
    """Snap near-center clicks to the official X/Y centerlines."""
    x = 1200.0 if abs(point.x() - 1200.0) <= CENTERLINE_SNAP_MM else point.x()
    y = 1200.0 if abs(point.y() - 1200.0) <= CENTERLINE_SNAP_MM else point.y()
    return QPointF(x, y)


def snap_paper_heading_to_right_angle(heading_deg: float) -> float:
    nearest = round(heading_deg / 90.0) * 90.0
    delta = (heading_deg - nearest + 180.0) % 360.0 - 180.0
    return nearest if abs(delta) <= RIGHT_ANGLE_SNAP_DEG else heading_deg


class NumericSpinBox(QDoubleSpinBox):
    def __init__(self) -> None:
        super().__init__()
        self.setButtonSymbols(QDoubleSpinBox.ButtonSymbols.NoButtons)

    def wheelEvent(self, event):  # type: ignore[no-untyped-def]
        event.ignore()


def spin(value=0.0, minimum=-10000.0, maximum=10000.0, step=10.0) -> NumericSpinBox:
    result = NumericSpinBox(); result.setRange(minimum, maximum); result.setValue(value)
    result.setSingleStep(step); result.setDecimals(2); return result


def snap_to_45(anchor: QPointF, target: QPointF) -> QPointF:
    """把目标点正交投影到距锚点最近的 45 度方向线。"""
    dx, dy = target.x() - anchor.x(), target.y() - anchor.y()
    if dx == 0.0 and dy == 0.0:
        return QPointF(target)
    angle = round(math.atan2(dy, dx) / (math.pi / 4.0)) * (math.pi / 4.0)
    ux, uy = math.cos(angle), math.sin(angle)
    distance = dx * ux + dy * uy
    return QPointF(anchor.x() + distance * ux, anchor.y() + distance * uy)


class MapView(QGraphicsView):
    clicked = Signal(float, float, bool)
    hovered = Signal(float, float, bool)
    released = Signal(float, float, bool)
    preview_rotated = Signal()
    box_selected = Signal(QRectF, bool)
    view_changed = Signal()
    cancel_requested = Signal()

    def __init__(self) -> None:
        super().__init__(); self.mode = "select"; self._rubber = None; self._origin = None; self._panning = False
        self._space_pressed = False; self._pan_origin = QPoint(); self._pan_scroll = QPoint(); self._add_pressed = False
        self.setRenderHint(QPainter.RenderHint.Antialiasing); self.setDragMode(QGraphicsView.DragMode.NoDrag)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus); self.setMouseTracking(True); self.viewport().setMouseTracking(True)

    def wheelEvent(self, event):  # type: ignore[no-untyped-def]
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15; self.scale(factor, factor); self.view_changed.emit()

    def mousePressEvent(self, event):  # type: ignore[no-untyped-def]
        if event.button() == Qt.MouseButton.MiddleButton or (event.button() == Qt.MouseButton.LeftButton and self._space_pressed):
            self.hovered.emit(float("nan"), float("nan"), False)
            self._panning = True; self._pan_origin = event.position().toPoint()
            self._pan_scroll = QPoint(self.horizontalScrollBar().value(), self.verticalScrollBar().value())
            self.viewport().setCursor(Qt.CursorShape.ClosedHandCursor); event.accept(); return
        item = self.itemAt(event.position().toPoint())
        if event.button() == Qt.MouseButton.RightButton and self.mode in ("add", "mark_pose"):
            if self.mode == "mark_pose":
                self.cancel_requested.emit()
            else:
                self.preview_rotated.emit()
            event.accept(); return
        if event.button() == Qt.MouseButton.RightButton and isinstance(item, StartItem):
            item.rotated(); event.accept(); return
        editable = isinstance(item, (WaypointItem, RotationHandleItem, StartItem, DraggableEllipseItem, DraggableRectItem))
        if event.button() == Qt.MouseButton.LeftButton and self.mode == "obstacle" and not editable:
            point = self.mapToScene(event.position().toPoint())
            self.clicked.emit(point.x(), point.y(), bool(event.modifiers() & Qt.KeyboardModifier.ShiftModifier))
            event.accept(); return
        if event.button() == Qt.MouseButton.LeftButton and self.mode == "mark_pose" and not editable:
            point = self.mapToScene(event.position().toPoint())
            self.clicked.emit(point.x(), point.y(), bool(event.modifiers() & Qt.KeyboardModifier.ShiftModifier))
            event.accept(); return
        if event.button() == Qt.MouseButton.LeftButton and self.mode == "add" and not editable:
            self._add_pressed = True
            event.accept(); return
        if event.button() == Qt.MouseButton.LeftButton and self.mode in ("calibrate", "measure", "auto_plan") and not editable:
            point = self.mapToScene(event.position().toPoint()); self.clicked.emit(point.x(), point.y(), bool(event.modifiers() & Qt.KeyboardModifier.ShiftModifier)); event.accept(); return
        if event.button() == Qt.MouseButton.LeftButton and self.mode == "select" and not editable:
            self._origin = event.position().toPoint(); self._rubber = QRubberBand(QRubberBand.Shape.Rectangle, self.viewport())
            self._rubber.setGeometry(QRect(self._origin, self._origin)); self._rubber.show(); event.accept(); return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):  # type: ignore[no-untyped-def]
        if self._panning:
            delta = event.position().toPoint() - self._pan_origin
            self.horizontalScrollBar().setValue(self._pan_scroll.x() - delta.x())
            self.verticalScrollBar().setValue(self._pan_scroll.y() - delta.y())
            self.view_changed.emit()
            event.accept(); return
        if self._rubber is not None:
            self._rubber.setGeometry(QRect(self._origin, event.position().toPoint()).normalized()); return
        point = self.mapToScene(event.position().toPoint())
        self.hovered.emit(point.x(), point.y(), bool(event.modifiers() & Qt.KeyboardModifier.ShiftModifier))
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):  # type: ignore[no-untyped-def]
        if self._panning:
            self._panning = False
            self.viewport().setCursor(Qt.CursorShape.OpenHandCursor if self._space_pressed else Qt.CursorShape.ArrowCursor)
            event.accept(); return
        if self._rubber is not None:
            rect = self._rubber.geometry(); self._rubber.hide(); self._rubber = None
            if rect.width() > 3 and rect.height() > 3: self.box_selected.emit(self.mapToScene(rect).boundingRect(), bool(event.modifiers() & Qt.KeyboardModifier.ControlModifier))
            return
        if event.button() == Qt.MouseButton.LeftButton and self._add_pressed:
            self._add_pressed = False
            point = self.mapToScene(event.position().toPoint())
            self.released.emit(point.x(), point.y(), bool(event.modifiers() & Qt.KeyboardModifier.ShiftModifier))
            event.accept(); return
        super().mouseReleaseEvent(event)

    def leaveEvent(self, event):  # type: ignore[no-untyped-def]
        self._add_pressed = False; self.hovered.emit(float("nan"), float("nan"), False); super().leaveEvent(event)

    def keyPressEvent(self, event):  # type: ignore[no-untyped-def]
        if event.key() == Qt.Key.Key_Escape and self.mode in ("measure", "mark_pose", "add"):
            self.cancel_requested.emit(); event.accept(); return
        if event.key() == Qt.Key.Key_Space and not event.isAutoRepeat():
            self._space_pressed = True; self.viewport().setCursor(Qt.CursorShape.OpenHandCursor); event.accept(); return
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):  # type: ignore[no-untyped-def]
        if event.key() == Qt.Key.Key_Space and not event.isAutoRepeat():
            self._space_pressed = False
            if not self._panning: self.viewport().setCursor(Qt.CursorShape.ArrowCursor)
            event.accept(); return
        super().keyReleaseEvent(event)

    def focusOutEvent(self, event):  # type: ignore[no-untyped-def]
        self._space_pressed = False
        if not self._panning: self.viewport().setCursor(Qt.CursorShape.ArrowCursor)
        super().focusOutEvent(event)

    def resizeEvent(self, event):  # type: ignore[no-untyped-def]
        super().resizeEvent(event); self.view_changed.emit()


class RotationHandleItem(QGraphicsEllipseItem):
    def __init__(self, owner, changed):  # type: ignore[no-untyped-def]
        super().__init__(-11, -11, 22, 22, owner); self.changed = changed
        self.setBrush(QColor("#ffffff")); self.setPen(QPen(QColor("#1565c0"), 3)); self.setZValue(2)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable)
        icon=QGraphicsTextItem("↻",self); icon.setFont(QFont("Microsoft YaHei",12)); icon.setDefaultTextColor(QColor("#1565c0")); icon.setPos(-9,-13); icon.setAcceptedMouseButtons(Qt.MouseButton.NoButton); self.setToolTip("拖动设置目标航向")

    def mouseMoveEvent(self, event):  # type: ignore[no-untyped-def]
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):  # type: ignore[no-untyped-def]
        super().mouseReleaseEvent(event); self.changed(self)


class WaypointItem(QGraphicsEllipseItem):
    def __init__(self, index, x, y, active, invalid, moved, rotated, activate):  # type: ignore[no-untyped-def]
        super().__init__(-18, -18, 36, 36); self.index = index; self.moved = moved; self.activate = activate; self._before = QPointF(x, y)
        self.setPos(x, y); self.setBrush(QColor("#4caf50") if active else QColor("#f28c28")); self.setPen(QPen(QColor("#c62828") if invalid else QColor("#333333"), 5 if invalid else 3)); self.setZValue(10)
        self.setFlags(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable | (QGraphicsItem.GraphicsItemFlag.ItemIsMovable if active else QGraphicsItem.GraphicsItemFlag(0)))
        self.handle = RotationHandleItem(self, rotated); self.handle.setPos(0, -62); self.handle.setVisible(active)

    def mousePressEvent(self, event):  # type: ignore[no-untyped-def]
        self._before = self.scenePos(); super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):  # type: ignore[no-untyped-def]
        super().mouseReleaseEvent(event); self.moved(self.index, self._before, self.scenePos(), bool(event.modifiers() & Qt.KeyboardModifier.ShiftModifier))

    def contextMenuEvent(self, event):  # type: ignore[no-untyped-def]
        self.activate(self.index); event.accept()


class DraggableEllipseItem(QGraphicsEllipseItem):
    def __init__(self, rect, moved):  # type: ignore[no-untyped-def]
        super().__init__(*rect); self.moved = moved
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable); self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable)

    def mousePressEvent(self, event):  # type: ignore[no-untyped-def]
        self._before = QPointF(self.scenePos())
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):  # type: ignore[no-untyped-def]
        super().mouseReleaseEvent(event)
        if getattr(self, "_before", self.scenePos()) != self.scenePos():
            self.moved(self.scenePos())


class DraggableRectItem(QGraphicsRectItem):
    def __init__(self, rect, moved):  # type: ignore[no-untyped-def]
        super().__init__(*rect); self.moved = moved
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable); self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable)

    def mouseReleaseEvent(self, event):  # type: ignore[no-untyped-def]
        super().mouseReleaseEvent(event); self.moved(self.scenePos())


class CarOutlineItem(QGraphicsRectItem):
    def __init__(self, rotated, vehicle_length_mm=300.0,
                 vehicle_width_mm=VEHICLE_WIDTH_MM):  # type: ignore[no-untyped-def]
        super().__init__(-vehicle_length_mm / 2.0, -vehicle_width_mm / 2.0,
                         vehicle_length_mm, vehicle_width_mm)
        self.rotated = rotated
        self.setToolTip("右击顺时针旋转 90 度")

    def mousePressEvent(self, event):  # type: ignore[no-untyped-def]
        if event.button() == Qt.MouseButton.RightButton:
            self.rotated(); event.accept(); return
        super().mousePressEvent(event)


class StartItem(QGraphicsPolygonItem):
    """起点箭头通过右击以 90 度为单位设置初始朝向。"""
    def __init__(self, heading, rotated):  # type: ignore[no-untyped-def]
        poly = QPolygonF([QPointF(-28, -18), QPointF(10, -18), QPointF(10, -34), QPointF(42, 0), QPointF(10, 34), QPointF(10, 18), QPointF(-28, 18)])
        super().__init__(poly); self.setRotation(-heading); self.setBrush(QColor("#1976d2")); self.setPen(QPen(QColor("#0d47a1"), 3)); self.setZValue(30)
        self.rotated = rotated; self.setToolTip("右击顺时针旋转 90 度")

    def mousePressEvent(self, event):  # type: ignore[no-untyped-def]
        if event.button() == Qt.MouseButton.RightButton:
            self.rotated(); event.accept(); return
        super().mousePressEvent(event)


class MapEditorWidget(QWidget):
    """可嵌入的地图编辑器，保留原有地图、规划与仿真交互。"""

    plan_changed = Signal(object)
    candidate_selected = Signal(int)
    hardware_enabled_changed = Signal(bool)
    single_step_requested = Signal(int)
    continuous_requested = Signal(int)
    execution_stop_requested = Signal()
    start_frame_changed = Signal(object)
    calibration_state_changed = Signal(bool)

    def __init__(self, parent: QWidget | None = None) -> None:
            super().__init__(parent)
            self.plan = Plan(); self.active_index = -1; self.active_point_index = -1; self.mode = "select"; self.pending_action: str | None = None; self.calibration_pending = True; self.calibration_stage = "choose"; self.undo_stack = []; self.redo_stack = []
            self.selected_indices: set[int] = set(); self.measurement_points: list[QPointF] = []
            self.preview_paper: QPointF | None = None; self.preview_yaw_deg: float | None = None; self.preview_anchor_index: int | None = None; self.preview_anchor_signature = None; self.preview_shift = False
            self._rviz_pose_anchor: QPointF | None = None
            self._rviz_drag_point: QPointF | None = None
            self._pending_navigation_goal_paper: QPointF | None = None
            self._pending_navigation_goal_yaw: float | None = None
            self._selected_auto_segment_dirty = False
            # 曲线在确认前只保存在此临时对象中，避免半成品进入 Plan.steps。
            self.bezier_draft: BezierPathSegment | None = None
            self.bezier_draft_start_yaw = 0.0
            self._bezier_preview_cache_key = None; self._bezier_preview_points = None
            self.timeline: list[SimulationFrame] = []; self.timeline_position = 0; self.current_frame = None; self.actual_trace = []
            self._layout_slider_before: Plan | None = None; self._layout_slider_changed = False
            self._last_plan_signature = None
            self._auto_paths_stale = False
            self._loading_costmap_controls = False
            self._runtime_pose: Pose | None = None
            self._execution_target: Pose | None = None
            self._execution_error: tuple[float, float, float] | None = None
            self._execution_trace: list[Pose] = []
            self._runtime_trace_path = QPainterPath()
            self._runtime_trace_path_point_count = 0
            self._path_runtime = None
            self._execution_enabled = False
            self._hardware_motion_active = False
            self._start_preview_paper: QPointF | None = None
            self._runtime_car_item = None
            self._runtime_direction_item = None
            self._runtime_target_item = None
            self._runtime_target_direction_item = None
            self._runtime_trace_item = None
            self.scene = QGraphicsScene(self); self.view = MapView(); self.view.setScene(self.scene)
            self.view.clicked.connect(self.on_map_click); self.view.hovered.connect(self.update_preview); self.view.released.connect(self.on_map_release); self.view.preview_rotated.connect(self.rotate_preview_clockwise); self.view.box_selected.connect(self.select_box); self.view.cancel_requested.connect(self.cancel_active_interaction); self.timer = QTimer(self); self.timer.setInterval(20); self.timer.timeout.connect(self.tick)
            self._nav_preview_timer = QTimer(self); self._nav_preview_timer.setSingleShot(True)
            self._nav_preview_timer.setInterval(50); self._nav_preview_timer.timeout.connect(self._flush_navigation_preview)
            self._build(); self._build_layout_sliders(); self.view.view_changed.connect(self.position_layout_sliders); QShortcut(QKeySequence.StandardKey.Undo, self, activated=self.undo); QShortcut(QKeySequence.StandardKey.Redo, self, activated=self.redo); QShortcut(QKeySequence.StandardKey.SelectAll, self, activated=self.select_all); QShortcut(QKeySequence(Qt.Key.Key_Delete), self, activated=self.remove_selected_step); QShortcut(QKeySequence(Qt.Key.Key_Return), self, activated=self.confirm_bezier_draft); QShortcut(QKeySequence(Qt.Key.Key_Escape), self, activated=self.cancel_active_interaction)
            self.refresh_mode_ui(); self.redraw(); self.refresh_plans()
            self._initial_fit_timer = QTimer(self); self._initial_fit_timer.setSingleShot(True)
            self._initial_fit_timer.timeout.connect(self.fit_map); self._initial_fit_timer.start(0)
            QShortcut(QKeySequence(Qt.Key.Key_Home),self,activated=self.fit_map)

    @property
    def canvas(self) -> MapView:
            """工作台可直接嵌入或观察的地图画布。"""
            return self.view

    @property
    def selected_candidate_index(self) -> int:
            return self.active_index

    def get_plan(self) -> Plan:
            """返回当前方案副本，避免外部绕过编辑器状态直接修改。"""
            return copy.deepcopy(self.plan)

    def selected_step(self):
            """返回当前选中动作的副本；未选中时返回 None。"""
            step = self.plan.steps[self.active_index] if 0 <= self.active_index < len(self.plan.steps) else None
            return copy.deepcopy(step)

    def selected_step_path_points(self) -> list[PathPosePoint]:
            """按当前选中步骤生成待上传的世界坐标路径点。"""
            step = self.selected_step()
            if (self._auto_paths_stale and isinstance(step, ContinuousPathSegment)
                    and step.name.startswith("自动规划")):
                raise ValueError("代价地图参数已改变，请先重新规划全部自动航点")
            if isinstance(step, ContinuousPathSegment):
                return step.points
            if isinstance(step, BezierPathSegment):
                start = self._step_end_pose(self.active_index)
                return generate_bezier_path_points(
                    start,
                    (step.control_1_x_mm, step.control_1_y_mm),
                    (step.control_2_x_mm, step.control_2_y_mm),
                    Pose(step.end_x_mm, step.end_y_mm, step.end_yaw_deg),
                    step.yaw_mode,
                    step.sample_spacing_mm,
                )
            raise ValueError("当前步骤不是可上传的路径")

    def set_plan(self, plan: Plan, *, calibrated: bool = True) -> None:
            """装载工作台提供的方案，并重置编辑器的短暂交互状态。"""
            if not isinstance(plan, Plan):
                raise TypeError("plan 必须是 Plan 实例")
            self._discard_bezier_draft(); self._invalidate_timeline()
            self.plan = copy.deepcopy(plan); self.active_index = -1; self.active_point_index = -1
            self._pending_navigation_goal_paper = None; self._pending_navigation_goal_yaw = None
            self.selected_indices.clear(); self.undo_stack.clear(); self.redo_stack.clear()
            self.calibration_pending = not calibrated; self.calibration_stage = "complete" if calibrated else "choose"
            self._auto_paths_stale = False
            self._load_costmap_controls()
            self.update_calibration_ui(); self.refresh_waypoints(); self.redraw()
            self.plan_changed.emit(copy.deepcopy(self.plan))

    def select_candidate(self, index: int) -> None:
            """选择工作台当前关注的流程候选项。"""
            if not 0 <= index < len(self.plan.steps):
                raise IndexError("候选项索引超出范围")
            self.activate_node(index)

    @property
    def execution_enabled(self) -> bool:
            """实机执行许可状态；控件仅发射信号，不直接访问串口。"""
            return self._execution_enabled

    def set_execution_enabled(self, enabled: bool) -> None:
            """由工作台设置实机执行许可，并同步禁用或启用执行控件。"""
            enabled = bool(enabled)
            if self._execution_enabled == enabled:
                return
            self._execution_enabled = enabled
            self.execution_enabled_switch.blockSignals(True)
            self.execution_enabled_switch.setChecked(enabled)
            self.execution_enabled_switch.blockSignals(False)
            self._refresh_execution_controls()
            self.hardware_enabled_changed.emit(enabled)

    def set_hardware_motion_active(self, active: bool) -> None:
            self._hardware_motion_active = bool(active)

    def apply_runtime_snapshot(self, snapshot: RuntimeUiSnapshot) -> None:
            """Apply the controller's 40 ms snapshot without rebuilding the scene."""
            actual = snapshot.actual_pose
            target = snapshot.target_pose
            self._runtime_pose = None if actual is None else Pose(actual.x_mm, actual.y_mm, actual.yaw_deg)
            self._execution_target = None if target is None else Pose(target.x_mm, target.y_mm, target.yaw_deg)
            self._execution_error = snapshot.error
            self._path_runtime = snapshot.path_telemetry
            self._hardware_motion_active = snapshot.motion_active
            if hasattr(self, "current_position_label"):
                if self._runtime_pose is None:
                    self.current_position_label.setText("实车当前位置：等待遥测")
                else:
                    paper = self.paper_of(self._runtime_pose)
                    self.current_position_label.setText(
                        f"实车当前位置：世界 X={self._runtime_pose.x_mm:.1f}, Y={self._runtime_pose.y_mm:.1f}, "
                        f"航向={self._runtime_pose.yaw_deg:.1f}°；图纸 X={paper.x_mm:.1f}, Y={paper.y_mm:.1f} mm")
            if hasattr(self, "runtime_position_label"):
                self.runtime_position_label.setText(
                    "实车位置：等待遥测" if self._runtime_pose is None else
                    f"实车位置：X={self._runtime_pose.x_mm:.1f} mm，"
                    f"Y={self._runtime_pose.y_mm:.1f} mm，航向={self._runtime_pose.yaw_deg:.1f}°")
                if self._path_runtime is None:
                    self.runtime_tracking_label.setText("局部跟踪：等待路径遥测")
                else:
                    self.runtime_tracking_label.setText(
                        f"局部跟踪：剩余={self._path_runtime.remaining_mm:.1f} mm，"
                        f"横向误差={self._path_runtime.cross_track_mm:.1f} mm，"
                        f"前视={self._path_runtime.lookahead_mm:.1f} mm，"
                        f"参考速度={self._path_runtime.reference_speed_mm_s:.1f} mm/s")
            if snapshot.trace_reset:
                self._clear_runtime_trace()
            new_trace_points = [Pose(point.x_mm, point.y_mm, point.yaw_deg)
                                for point in snapshot.new_trace_points]
            self._execution_trace.extend(new_trace_points)
            self._append_runtime_trace_points(new_trace_points)
            self._refresh_runtime_overlay()

    def set_execution_status(self, status: str) -> None:
            """更新工作台提供的运行状态文本，不触发运动或串口操作。"""
            if not isinstance(status, str):
                raise TypeError("status 必须是字符串")
            self._execution_status = status
            self._refresh_execution_status()

    def _refresh_execution_controls(self) -> None:
            for button in (self.execution_step_button, self.execution_run_button, self.execution_stop_button):
                button.setEnabled(self._execution_enabled)
            self._refresh_execution_status()

    def _request_single_execution(self) -> None:
            if not self._execution_enabled:
                return
            if not 0 <= self.active_index < len(self.plan.steps):
                self.set_execution_status("请先选择一个有效动作")
                return
            self.single_step_requested.emit(self.active_index)

    def _request_continuous_execution(self) -> None:
            if not self._execution_enabled:
                return
            if not 0 <= self.active_index < len(self.plan.steps):
                self.set_execution_status("请先选择连续执行的起始动作")
                return
            self.continuous_requested.emit(self.active_index)

    def _set_yellow_zone_passage(self, allowed: bool) -> None:
            self.yellow_zone_status_label.setText(
                "黄色区限制已关闭：实车运行前请确认平台可安全通行。"
                if allowed else "黄色区限制已启用：车体扫掠不得接触黄色平台。")
            self.redraw()
            self.rebuild_timeline_after_edit()

    def _refresh_execution_status(self) -> None:
            if not hasattr(self, "execution_status_label"):
                return
            if not self._execution_enabled:
                self.execution_status_label.setText("实机执行未启用")
                return
            detail = getattr(self, "_execution_status", "等待工作台命令")
            if self._execution_error is not None:
                x_mm, y_mm, yaw_deg = self._execution_error
                detail = f"{detail}；误差 X={x_mm:.1f} mm, Y={y_mm:.1f} mm, 航向={yaw_deg:.1f} deg"
            self.execution_status_label.setText(detail)

    def fit_map(self):
            self.view.fitInView(QRectF(-260,-260,2920,2920),Qt.AspectRatioMode.KeepAspectRatio); self.position_layout_sliders()

    def _build_layout_sliders(self):
            """场地对象使用视图叠加滑块，避免场景重绘中断拖动。"""
            self.raw_slider = QSlider(Qt.Orientation.Horizontal, self.view.viewport())
            self.qr_slider = QSlider(Qt.Orientation.Vertical, self.view.viewport())
            for slider, value, callback in ((self.raw_slider, 1200, self._set_raw_slider_value), (self.qr_slider, 1200, self._set_qr_slider_value)):
                slider.setRange(1100, 1300); slider.setSingleStep(1); slider.setPageStep(10); slider.setValue(value)
                slider.setToolTip("范围：1100-1300 mm，步长：1 mm")
                slider.sliderPressed.connect(self._begin_layout_slider_edit)
                slider.valueChanged.connect(callback)
                slider.sliderReleased.connect(self._finish_layout_slider_edit)
                slider.setStyleSheet("QSlider::groove:horizontal,QSlider::groove:vertical{background:#cfd8dc;border-radius:3px;} QSlider::handle{background:#1565c0;border:1px solid #0d47a1;border-radius:6px;}")
            self.raw_slider.setFixedSize(160, 20); self.qr_slider.setFixedSize(20, 160)

    def position_layout_sliders(self):
            if not hasattr(self, "raw_slider"): return
            raw = self.view.mapFromScene(QPointF(self.plan.layout.raw_center_x_mm, -250))
            qr = self.view.mapFromScene(QPointF(2475, self.plan.layout.qr_center_y_mm))
            self.raw_slider.move(raw.x() - self.raw_slider.width() // 2, raw.y())
            self.qr_slider.move(qr.x() - self.qr_slider.width(), qr.y() - self.qr_slider.height() // 2)
            self.raw_slider.setVisible(self.view.viewport().rect().intersects(self.raw_slider.geometry()))
            self.qr_slider.setVisible(self.view.viewport().rect().intersects(self.qr_slider.geometry()))

    def _begin_layout_slider_edit(self):
            self._layout_slider_before = copy.deepcopy(self.plan); self._layout_slider_changed = False

    def _set_raw_slider_value(self, value):
            if self.plan.layout.raw_center_x_mm == float(value): return
            self.plan.layout.raw_center_x_mm = float(value); self._layout_slider_changed = True; self.redraw()

    def _set_qr_slider_value(self, value):
            if self.plan.layout.qr_center_y_mm == float(value): return
            self.plan.layout.qr_center_y_mm = float(value); self._layout_slider_changed = True; self.redraw()

    def _finish_layout_slider_edit(self):
            if self._layout_slider_changed and self._layout_slider_before is not None:
                self.undo_stack.append(self._layout_slider_before); self.undo_stack = self.undo_stack[-100:]; self.redo_stack.clear()
            self._layout_slider_before = None; self._layout_slider_changed = False

    def _build(self) -> None:
            root = QSplitter(Qt.Orientation.Horizontal); self.editor_splitter = root
            root.setChildrenCollapsible(False); root.setHandleWidth(8); root.setOpaqueResize(True)
            root.setStyleSheet("QSplitter::handle:horizontal{background:#455a64;margin:1px 2px;border-radius:2px;} QSplitter::handle:horizontal:hover{background:#29b6f6;}")
            left = QWidget(); left.setMinimumWidth(420)
            outer_box = QVBoxLayout(left); outer_box.setContentsMargins(10, 10, 10, 10); outer_box.setSpacing(6)
            self.editor_tabs = QTabWidget()
            cost_page, waypoint_page, output_page = QWidget(), QWidget(), QWidget()
            cost_box, waypoint_box, output_box = QVBoxLayout(cost_page), QVBoxLayout(waypoint_page), QVBoxLayout(output_page)
            for page_box in (cost_box, waypoint_box, output_box):
                page_box.setContentsMargins(6, 8, 6, 8); page_box.setSpacing(6)
            self.editor_tabs.addTab(cost_page, "1 代价地图")
            self.editor_tabs.addTab(waypoint_page, "2 点位与航点")
            self.editor_tabs.addTab(output_page, "3 方案与输出")
            self.navigation_page = cost_page
            self.basic_actions_page = waypoint_page
            self.runtime_page = QWidget()
            runtime_box = QVBoxLayout(self.runtime_page)
            runtime_box.setContentsMargins(6, 8, 6, 8)
            runtime_box.setSpacing(8)
            self.output_page = output_page
            self.editor_tabs.insertTab(2, self.runtime_page, "3 位姿与运行")
            self.editor_tabs.setTabText(0, "1 自动导航")
            self.editor_tabs.setTabText(1, "2 路径制作")
            self.editor_tabs.setTabText(2, "3 实时运行")
            self.editor_tabs.setTabText(3, "4 方案与输出")
            self.editor_tabs.currentChanged.connect(self._on_editor_tab_changed)
            outer_box.addWidget(self.editor_tabs)
            box = waypoint_box
            toolbar = QHBoxLayout(); self.tool_group = QButtonGroup(self); self.tool_group.setExclusive(True)
            self.select_button = QPushButton("选择/编辑"); self.add_button = QPushButton("添加节点"); self.measure_button = QPushButton("测距"); self.obstacle_button = QPushButton("放置障碍物"); self.auto_plan_button = QPushButton("设置终点（使用参数航向）")
            self.mark_pose_button = QPushButton("设置导航目标（点位置 → 点方向）")
            tool_style = ("QPushButton{padding:7px 10px;}"
                          "QPushButton:checked{background:#00897b;color:white;"
                          "border:2px solid #80cbc4;font-weight:700;}")
            for button, value in ((self.select_button, "select"), (self.mark_pose_button, "mark_pose"), (self.measure_button, "measure"), (self.auto_plan_button, "auto_plan")):
                button.setCheckable(True); button.setStyleSheet(tool_style); self.tool_group.addButton(button)
                button.toggled.connect(lambda checked=False, mode=value, source=button: checked and self.set_mode(mode, source)); toolbar.addWidget(button)
            self.obstacle_button.setCheckable(True); self.obstacle_button.setStyleSheet(tool_style); self.tool_group.addButton(self.obstacle_button)
            self.obstacle_button.toggled.connect(lambda checked=False: checked and self.set_mode("obstacle", self.obstacle_button))
            self.select_button.setChecked(True); box.addLayout(toolbar)
            self.continuous_goal_selection = QCheckBox("连续选择自动规划航点")
            self.continuous_goal_selection.setChecked(True)
            self.continuous_goal_selection.setToolTip("开启后，每次点击都从上一航点继续规划；关闭后生成一个航点即回到选择模式。")
            box.addWidget(self.continuous_goal_selection)
            waypoint_coordinate_note = QLabel(
                "自动航点标注使用图纸坐标差：Δ启1 = 当前航点 − 启停区1中心 (2250, 150) mm。")
            waypoint_coordinate_note.setWordWrap(True)
            box.addWidget(waypoint_coordinate_note)
            self.cursor_position_label = QLabel("地图指针：--")
            self.current_position_label = QLabel("实车当前位置：等待遥测")
            self.selected_pose_label = QLabel("当前航点：未选择")
            for label in (self.cursor_position_label, self.current_position_label, self.selected_pose_label):
                label.setWordWrap(True); box.addWidget(label)
            self.navigation_strategy = QComboBox()
            self.navigation_strategy.addItem("稳定自动（优先保持航向，必要时末端原地转向）", "auto")
            self.navigation_strategy.addItem("保持当前车身航向", "fixed")
            self.navigation_strategy.addItem("车头跟随路径切线", "tangent")
            self.navigation_strategy.addItem("移动中连续转向", "interpolate")
            self.navigation_strategy.addItem("仅在终点原地转向", "terminal")
            self.navigation_strategy.setToolTip(
                "唯一的航向策略：决定移动过程中是否转动车头，以及是否在终点原地对准 Yaw。")
            strategy_form = QFormLayout(); strategy_form.addRow("运动与航向策略", self.navigation_strategy)
            box.addLayout(strategy_form)
            # 方案存储、代码生成和执行集中在第三页。
            box = output_box
            box.addWidget(QLabel("方案")); self.plan_list = QListWidget(); self.plan_list.itemDoubleClicked.connect(self.load_selected); self.plan_list.setMinimumHeight(76); box.addWidget(self.plan_list)
            row = QHBoxLayout()
            for label, fn in (("新建", self.new_plan), ("保存", self.save), ("另存", self.save_as), ("重命名", self.rename_selected), ("加载", self.load_selected)):
                button = QPushButton(label); button.clicked.connect(fn); row.addWidget(button)
                if label == "保存": self.save_button = button
                elif label == "另存": self.save_as_button = button
            self.codegen_button = QPushButton("生成业务函数")
            self.codegen_button.clicked.connect(self.open_code_generator)
            row.addWidget(self.codegen_button)
            box.addLayout(row)
            code_note = QLabel(
                "代码使用：点击“生成业务函数” → 保持“完整 competition_route.c” → “复制代码”；"
                "把内容保存到 Core/Src/competition_route.c，加入 EIDE/Keil 工程，并在业务状态机中调用生成的 Task_xxx()。")
            code_note.setWordWrap(True); box.addWidget(code_note)
            box = waypoint_box
            self.steps_label=QLabel("导航目标与手动动作"); box.addWidget(self.steps_label)
            self.show_generated_details = QCheckBox("展开自动生成的路径点与末端转向")
            self.show_generated_details.setChecked(False)
            self.show_generated_details.toggled.connect(self.refresh_waypoints)
            box.addWidget(self.show_generated_details)
            self.waypoint_list = QListWidget(); self.waypoint_list.currentRowChanged.connect(self.activate_list_row); self.waypoint_list.setMinimumHeight(105); box.addWidget(self.waypoint_list)
            primary_actions = QHBoxLayout()
            self.replan_auto_button = QPushButton("重新规划全部自动航点")
            self.replan_auto_button.clicked.connect(self.replan_all_auto_paths)
            self.delete_button = QPushButton("删除选中航点/动作"); self.delete_button.clicked.connect(self.remove_selected_step)
            primary_actions.addWidget(self.replan_auto_button); primary_actions.addWidget(self.delete_button)
            box.addLayout(primary_actions)
            self.advanced_group = QGroupBox("高级手动编辑")
            self.advanced_group.setCheckable(True); self.advanced_group.setChecked(False)
            advanced_content = QWidget(); advanced_box = QVBoxLayout(advanced_content); advanced_box.setContentsMargins(0, 4, 0, 0)
            advanced_group_box = QVBoxLayout(self.advanced_group); advanced_group_box.addWidget(advanced_content)
            self.advanced_group.toggled.connect(advanced_content.setVisible)
            advanced_content.setVisible(False); box.addWidget(self.advanced_group)
            box = advanced_box
            self.add_button.setText("手动添加节点")
            self.add_button.setCheckable(True); self.add_button.setStyleSheet(tool_style); self.tool_group.addButton(self.add_button)
            self.add_button.toggled.connect(lambda checked=False: checked and self.set_mode("add"))
            box.addWidget(self.add_button)
            action_row=QHBoxLayout(); self.add_goto_button=QPushButton("新增点到点"); self.append_rotation_button=QPushButton("新增原地转向"); self.add_continuous_button=QPushButton("新增连续段"); self.add_bezier_button=QPushButton("新增曲线路径")
            self.add_goto_button.clicked.connect(self.begin_goto_add); self.append_rotation_button.clicked.connect(self.append_rotation); self.add_continuous_button.clicked.connect(self.add_continuous_segment); self.add_bezier_button.clicked.connect(self.begin_bezier_add)
            action_row.addWidget(self.add_goto_button); action_row.addWidget(self.append_rotation_button); action_row.addWidget(self.add_continuous_button); action_row.addWidget(self.add_bezier_button); box.addLayout(action_row)
            bezier_row=QHBoxLayout(); self.confirm_bezier_button=QPushButton("确认曲线"); self.cancel_bezier_button=QPushButton("取消曲线")
            self.confirm_bezier_button.clicked.connect(self.confirm_bezier_draft); self.cancel_bezier_button.clicked.connect(self.cancel_bezier_draft)
            bezier_row.addWidget(self.confirm_bezier_button); bezier_row.addWidget(self.cancel_bezier_button); box.addLayout(bezier_row)
            self.confirm_bezier_button.setVisible(False); self.cancel_bezier_button.setVisible(False)
            order_row=QHBoxLayout(); self.move_up_button=QPushButton("上移"); self.move_down_button=QPushButton("下移"); self.move_up_button.clicked.connect(lambda:self.move_step(-1)); self.move_down_button.clicked.connect(lambda:self.move_step(1)); order_row.addWidget(self.move_up_button); order_row.addWidget(self.move_down_button); box.addLayout(order_row)
            form = QFormLayout(); self.x=spin(); self.y=spin(); self.yaw=spin(0,-360,360,5); self.use_yaw=QCheckBox("启用航向约束（GOTO Pose）")
            self.stop=QCheckBox("到点停止"); self.stop.setChecked(True); self.dwell=spin(.5,0,120,.1); self.node_vmax=spin(820,1,1500,10); self.vmax=self.node_vmax; self.node_wmax=spin(90,1,360,5); self.timeout=spin(15, .1, 300, 1)
            self.goto_form_widgets=[]
            for label, widget in (("X (mm)",self.x),("Y (mm)",self.y),("停留 (s)",self.dwell),("最大线速度 (mm/s)",self.node_vmax)):
                text=QLabel(label); form.addRow(text,widget); self.goto_form_widgets.extend((text,widget))
            for label, widget in (("目标航向 (deg)",self.yaw),("最大角速度 (deg/s)",self.node_wmax),("超时 (s)",self.timeout)):
                form.addRow(label,widget)
            form.addRow(self.use_yaw); form.addRow(self.stop); update=QPushButton("更新当前节点"); update.clicked.connect(self.update_waypoint); form.addRow(update); box.addLayout(form)
            self.goto_form_widgets.extend((self.use_yaw,self.stop)); self.update_action_button=update
            self.stop_point_form = form
            self.continuous_label = QLabel("连续位姿点（几何路径，不代表中途停车）"); self.continuous_list = QListWidget(); self.continuous_list.setMinimumHeight(105)
            self.continuous_list.currentRowChanged.connect(self.activate_continuous_point)
            self.continuous_x=spin(); self.continuous_y=spin(); self.continuous_yaw=spin(0,-360,360,5)
            self.update_continuous_button=QPushButton("更新连续位姿点"); self.update_continuous_button.clicked.connect(self.update_continuous_point)
            self.delete_continuous_button=QPushButton("删除连续位姿点"); self.delete_continuous_button.clicked.connect(self.remove_continuous_point)
            self.continuous_panel=QWidget(); continuous_box=QVBoxLayout(self.continuous_panel); continuous_box.setContentsMargins(0,0,0,0); continuous_box.addWidget(self.continuous_label); continuous_box.addWidget(self.continuous_list)
            continuous_form=QFormLayout(); continuous_form.addRow("X (mm)",self.continuous_x); continuous_form.addRow("Y (mm)",self.continuous_y); continuous_form.addRow("目标航向 (deg)",self.continuous_yaw); continuous_form.addRow(self.update_continuous_button); continuous_box.addLayout(continuous_form); continuous_box.addWidget(self.delete_continuous_button); box.addWidget(self.continuous_panel); self.continuous_panel.setVisible(False)
            obstacle_edit_group = QGroupBox("随机障碍物编辑")
            obstacle_edit_box = QHBoxLayout(obstacle_edit_group)
            self.select_obstacle_button = QPushButton("选择/拖动")
            self.select_obstacle_button.setCheckable(True); self.select_obstacle_button.setStyleSheet(tool_style); self.tool_group.addButton(self.select_obstacle_button)
            self.delete_obstacle_button = QPushButton("删除选中障碍物")
            self.select_obstacle_button.toggled.connect(
                lambda checked=False: checked and self.set_mode("select", self.select_obstacle_button))
            self.delete_obstacle_button.clicked.connect(self.remove_selected_obstacles)
            obstacle_edit_box.addWidget(self.obstacle_button); obstacle_edit_box.addWidget(self.select_obstacle_button); obstacle_edit_box.addWidget(self.delete_obstacle_button)
            self.obstacle_count_label = QLabel("障碍物：0 个"); obstacle_edit_box.addWidget(self.obstacle_count_label)
            cost_box.addWidget(obstacle_edit_group)
            self.auto_plan_panel = QGroupBox("代价地图：红色物理轮廓 / 粉色硬禁区 / 黄色软代价区"); auto_box = QVBoxLayout(self.auto_plan_panel)
            self._costmap_controls = []
            vehicle_group = QGroupBox("车体尺寸（用于规划与完整车体扫掠）")
            vehicle_form = QFormLayout(vehicle_group)
            self.vehicle_length = spin(300.0, 50.0, 500.0, 5.0)
            self.vehicle_width = spin(300.0, 50.0, 500.0, 5.0)
            vehicle_form.addRow("车身长度 (mm)", self.vehicle_length)
            vehicle_form.addRow("车身宽度 (mm)", self.vehicle_width)
            self._costmap_controls.extend((self.vehicle_length, self.vehicle_width)); auto_box.addWidget(vehicle_group)
            def cost_group(title, safety, inflation, weight, *, radius=None):
                group = QGroupBox(title); form = QFormLayout(group)
                controls = []
                if radius is not None:
                    radius_box = spin(radius, 0.0, 100.0, 1.0); form.addRow("物理半径 (mm)", radius_box); controls.append(radius_box)
                safety_box = spin(safety, 0.0, 150.0, 5.0)
                inflation_box = spin(inflation, 0.0, 500.0, 10.0)
                weight_box = spin(weight, 0.0, 20.0, 0.25)
                form.addRow("安全距离 (mm)", safety_box)
                form.addRow("软膨胀距离 (mm)", inflation_box)
                form.addRow("软代价权重", weight_box)
                controls.extend((safety_box, inflation_box, weight_box))
                self._costmap_controls.extend(controls); auto_box.addWidget(group)
                return controls
            boundary = cost_group("场地边线（向场内膨胀）", 20.0, 120.0, 3.0)
            boundary_zone_group = QGroupBox("边界功能区内突")
            boundary_zone_form = QFormLayout(boundary_zone_group)
            self.boundary_zone_half_width = spin(200.0, 0.0, 600.0, 5.0)
            self.boundary_zone_depth = spin(85.0, 0.0, 300.0, 5.0)
            self.side_zone_half_length = spin(290.0, 0.0, 600.0, 5.0)
            self.side_zone_depth = spin(150.0, 0.0, 300.0, 5.0)
            self.boundary_zone_inflation = spin(35.0, 0.0, 500.0, 5.0)
            boundary_zone_form.addRow("原料区半宽 (mm)", self.boundary_zone_half_width)
            boundary_zone_form.addRow("原料区深度 (mm)", self.boundary_zone_depth)
            boundary_zone_form.addRow("暂存/粗加工区半长 (mm)", self.side_zone_half_length)
            boundary_zone_form.addRow("暂存/粗加工区宽度 (mm)", self.side_zone_depth)
            boundary_zone_form.addRow("软膨胀距离 (mm)", self.boundary_zone_inflation)
            self._costmap_controls.extend((self.boundary_zone_half_width,
                                           self.boundary_zone_depth,
                                           self.side_zone_half_length,
                                           self.side_zone_depth,
                                           self.boundary_zone_inflation))
            auto_box.addWidget(boundary_zone_group)
            platform = cost_group("四个黄色平台（组内侧）", 20.0, 20.0, 3.0)
            platform_outer_group = QGroupBox("四个黄色平台（组外侧）")
            platform_outer_form = QFormLayout(platform_outer_group)
            self.platform_outer_inflation = spin(240.0, 0.0, 500.0, 10.0)
            self.platform_outer_weight = spin(3.8, 0.0, 20.0, 0.1)
            platform_outer_form.addRow("软膨胀距离 (mm)", self.platform_outer_inflation)
            platform_outer_form.addRow("软代价权重", self.platform_outer_weight)
            self._costmap_controls.extend((self.platform_outer_inflation,
                                           self.platform_outer_weight))
            auto_box.addWidget(platform_outer_group)
            obstacle = cost_group("黑色随机障碍物（向外膨胀）", 20.0, 80.0, 3.0, radius=25.0)
            self.boundary_safety, self.boundary_inflation, self.boundary_weight = boundary
            self.platform_safety, self.platform_inflation, self.platform_weight = platform
            self.obstacle_radius, self.obstacle_safety, self.obstacle_inflation, self.obstacle_weight = obstacle
            # Compatibility name used by older UI tests and integrations.
            self.auto_safety_margin = self.platform_safety
            self.trajectory_group = QGroupBox("轨迹生成"); auto_form = QFormLayout(self.trajectory_group)
            self.auto_corner_radius = spin(120.0, 0.0, 400.0, 10.0); self.auto_sample_spacing = spin(20.0, 10.0, 50.0, 5.0); self.auto_terminal_straight = spin(300.0, 0.0, 1000.0, 25.0)
            self.auto_goal_x = spin(0.0, 0.0, FIELD_SIZE_MM, 5.0)
            self.auto_goal_y = spin(0.0, 0.0, FIELD_SIZE_MM, 5.0)
            self.auto_goal_yaw = spin(0.0, -180.0, 180.0, 5.0)
            self.auto_yaw_mode = QComboBox(); self.auto_yaw_mode.addItem("保持当前车身航向", "fixed"); self.auto_yaw_mode.addItem("起终点航向插值", "interpolate"); self.auto_yaw_mode.addItem("车头跟随轨迹切线", "tangent")
            self.show_inflated_zones = QCheckBox("显示障碍膨胀区和边缘代价带"); self.show_inflated_zones.setChecked(True)
            auto_form.addRow("目标 X (mm)", self.auto_goal_x); auto_form.addRow("目标 Y (mm)", self.auto_goal_y); auto_form.addRow("目标 Yaw (deg)", self.auto_goal_yaw)
            auto_form.addRow("五次并线半径 (mm)", self.auto_corner_radius); auto_form.addRow("采样间距 (mm)", self.auto_sample_spacing); auto_form.addRow("末端直线对接长度 (mm)", self.auto_terminal_straight); auto_form.addRow(self.show_inflated_zones)
            self.generate_segment_button = QPushButton("生成/更新本小段路径")
            self.generate_segment_button.setStyleSheet(
                "QPushButton{background:#f9a825;color:#1b1b1b;padding:10px;"
                "font-weight:700;border:2px solid #ffd95a;}"
                "QPushButton:hover{background:#fbc02d;}"
                "QPushButton:disabled{background:#5f5f5f;color:#bdbdbd;border-color:#777;}")
            self.generate_segment_button.clicked.connect(self.generate_navigation_segment)
            auto_form.addRow(self.generate_segment_button)
            waypoint_box.insertWidget(6, self.trajectory_group)
            costmap_note = QLabel("安全距离是车身轮廓之外额外预留的间隙。粉色区 = 车体尺寸基准 + 安全距离；黄色软区允许规划器进入但会增加 A* 代价。最终会按当前车身长宽和实际航向逐段扫掠，车角碰线就拒绝。")
            costmap_note.setWordWrap(True); auto_box.addWidget(costmap_note); cost_box.addWidget(self.auto_plan_panel)
            local_group = QGroupBox("局部路径跟踪（F407，非局部代价地图）")
            local_box = QVBoxLayout(local_group)
            local_note = QLabel("F407 以 50 Hz 跟踪已上传路径：动态前视 + 横向/航向 PD + 曲率前馈。横向误差 20 mm 起降速，50 mm 持续 60 ms 则停车；可调参数在工作台左侧“路径”页。当前不会在车上重新搜索路线。")
            local_note.setWordWrap(True); local_box.addWidget(local_note); cost_box.addWidget(local_group)
            for control in self._costmap_controls:
                control.valueChanged.connect(self._on_costmap_changed)
            self.auto_corner_radius.valueChanged.connect(self._mark_selected_segment_dirty); self.auto_sample_spacing.valueChanged.connect(self._mark_selected_segment_dirty); self.auto_terminal_straight.valueChanged.connect(self._mark_selected_segment_dirty); self.auto_goal_x.valueChanged.connect(self._on_goal_control_changed); self.auto_goal_y.valueChanged.connect(self._on_goal_control_changed); self.auto_goal_yaw.valueChanged.connect(self._on_goal_control_changed); self.navigation_strategy.currentIndexChanged.connect(self._mark_selected_segment_dirty); self.show_inflated_zones.toggled.connect(self.redraw)
            self.bezier_panel = QWidget(); bezier_box = QVBoxLayout(self.bezier_panel); bezier_box.setContentsMargins(0, 0, 0, 0)
            bezier_box.addWidget(QLabel("曲线航向")); bezier_form = QFormLayout()
            self.bezier_yaw_mode = QComboBox(); self.bezier_yaw_mode.addItem("起终点航向插值", "interpolate"); self.bezier_yaw_mode.addItem("切线跟随", "tangent")
            self.bezier_start_yaw = spin(0, -360, 360, 5); self.bezier_end_yaw = spin(0, -360, 360, 5)
            self.bezier_yaw_mode.currentIndexChanged.connect(self.refresh_bezier_draft_preview); self.bezier_start_yaw.valueChanged.connect(self.refresh_bezier_draft_preview); self.bezier_end_yaw.valueChanged.connect(self.refresh_bezier_draft_preview)
            self.bezier_start_yaw_label = QLabel("起点航向 (deg)"); self.bezier_end_yaw_label = QLabel("终点航向 (deg)")
            bezier_form.addRow("航向模式", self.bezier_yaw_mode); bezier_form.addRow(self.bezier_start_yaw_label, self.bezier_start_yaw); bezier_form.addRow(self.bezier_end_yaw_label, self.bezier_end_yaw)
            self.update_bezier_button = QPushButton("应用曲线航向"); self.update_bezier_button.clicked.connect(self.apply_bezier_heading); bezier_form.addRow(self.update_bezier_button)
            bezier_box.addLayout(bezier_form); box.addWidget(self.bezier_panel); self.bezier_panel.setVisible(False)
            box = waypoint_box
            box = output_box
            box.addWidget(QLabel("仿真")); simrow=QHBoxLayout()
            for label, fn in (("播放",self.play),("暂停",self.pause),("重置",self.reset_simulation)):
                button=QPushButton(label); button.clicked.connect(fn); simrow.addWidget(button)
                if label == "播放": self.play_button = button
            box.addLayout(simrow)
            box = runtime_box
            realtime_note = QLabel(
                "地图同时显示原规划路径和实车运行轨迹。紫色实线为实车轨迹；"
                "橙点为当前投影点，青点为动态前视点。F407 只做局部跟踪，不会实时重新规划路线。")
            realtime_note.setWordWrap(True)
            box.addWidget(realtime_note)
            self.runtime_position_label = QLabel("实车位置：等待遥测")
            self.runtime_tracking_label = QLabel("局部跟踪：等待路径遥测")
            self.runtime_position_label.setWordWrap(True)
            self.runtime_tracking_label.setWordWrap(True)
            box.addWidget(self.runtime_position_label)
            box.addWidget(self.runtime_tracking_label)
            box.addWidget(QLabel("规划动作（选择连续执行的起始动作）"))
            self.runtime_waypoint_list = QListWidget()
            self.runtime_waypoint_list.setMinimumHeight(140)
            self.runtime_waypoint_list.currentRowChanged.connect(
                self.activate_list_row)
            box.addWidget(self.runtime_waypoint_list)
            box.addWidget(QLabel("实机执行"))
            execution_row = QHBoxLayout()
            self.execution_enabled_switch = QCheckBox("实机运动")
            self.execution_enabled_switch.toggled.connect(self.set_execution_enabled)
            self.execution_step_button = QPushButton("单步执行")
            self.execution_run_button = QPushButton("连贯执行")
            self.execution_stop_button = QPushButton("停止")
            self.execution_step_button.clicked.connect(self._request_single_execution)
            self.execution_run_button.clicked.connect(self._request_continuous_execution)
            self.execution_stop_button.clicked.connect(self.execution_stop_requested.emit)
            execution_row.addWidget(self.execution_enabled_switch)
            execution_row.addWidget(self.execution_step_button)
            execution_row.addWidget(self.execution_run_button)
            execution_row.addWidget(self.execution_stop_button)
            box.addLayout(execution_row)
            self.execution_status_label = QLabel("实机执行未启用")
            self.execution_status_label.setWordWrap(True)
            box.addWidget(self.execution_status_label)
            box.addStretch()
            self.allow_yellow_zone = QCheckBox("允许通过黄色区")
            self.allow_yellow_zone.setChecked(True)
            self.allow_yellow_zone.toggled.connect(self._set_yellow_zone_passage)
            cost_box.addWidget(self.allow_yellow_zone)
            self.yellow_zone_status_label = QLabel(
                "黄色区限制已关闭：实车运行前请确认平台可安全通行。")
            self.yellow_zone_status_label.setWordWrap(True)
            self.yellow_zone_status_label.setStyleSheet("color: #ef6c00;")
            cost_box.addWidget(self.yellow_zone_status_label)
            box = output_box
            self.progress = QSlider(Qt.Orientation.Horizontal); self.progress.setRange(0, 0); self.progress.setEnabled(False)
            self.progress.sliderPressed.connect(self.pause); self.progress.valueChanged.connect(self.seek_timeline)
            box.addWidget(self.progress); self.progress_label = QLabel("进度：0.00 / 0.00 s"); box.addWidget(self.progress_label)
            self.measurement_label = QLabel("水平：--  垂直：--  欧式：--"); self.measurement_label.setWordWrap(True); box.addWidget(self.measurement_label)
            self.status=QLabel("先选择起点，再右击蓝色箭头设置朝向并确认。"); self.status.setWordWrap(True); outer_box.addWidget(self.status)
            self.path_check=QLabel("路径检查：等待编辑"); self.path_check.setWordWrap(True); outer_box.addWidget(self.path_check)
            cost_box.addStretch(); waypoint_box.addStretch(); output_box.addStretch()
            scroll=QScrollArea(); self.editor_controls_scroll = scroll
            scroll.setWidgetResizable(True); scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded); scroll.setWidget(left); scroll.setMinimumWidth(430)
            map_panel=QWidget(); map_box=QVBoxLayout(map_panel); map_box.setContentsMargins(0,0,0,0); map_box.setSpacing(0)
            map_panel.setMinimumWidth(480)
            self.current_tool_label = QLabel("当前工具：选择/编辑")
            self.current_tool_label.setStyleSheet(
                "padding:7px 12px;background:#103b46;color:#80cbc4;font-weight:700;")
            map_box.addWidget(self.current_tool_label)
            self.calibration_bar=QWidget(); guide=QHBoxLayout(self.calibration_bar); guide.setContentsMargins(12,8,12,8)
            self.calibration_label=QLabel("1. 选择起点   2. 右击蓝色箭头设置朝向") ; guide.addWidget(self.calibration_label); guide.addStretch()
            for label in ("启停区 1", "启停区 2", "自定义"):
                button=QPushButton(label); button.clicked.connect(lambda checked=False,value=label:self.begin_start(value)); guide.addWidget(button)
            self.confirm_start_button=QPushButton("确认朝向"); self.confirm_start_button.clicked.connect(self.confirm_start_heading); guide.addWidget(self.confirm_start_button)
            map_box.addWidget(self.calibration_bar); map_box.addWidget(self.view)
            root.addWidget(scroll); root.addWidget(map_panel)
            root.setStretchFactor(0, 0); root.setStretchFactor(1, 1)
            root.setSizes([520,900])
            layout = QVBoxLayout(self); layout.setContentsMargins(0, 0, 0, 0); layout.addWidget(root)
            self.update_calibration_ui()
            self._refresh_execution_controls()

    def _on_editor_tab_changed(self, _index: int) -> None:
            if self.editor_tabs.currentWidget() is not self.navigation_page:
                if self.mode in ("mark_pose", "obstacle"):
                    self.set_mode("select")
                elif self.mode == "add" and self.bezier_draft is None:
                    self.set_mode("select")
            self.redraw()

    def _costmap_overlay_visible(self) -> bool:
            toggle = getattr(self, "show_inflated_zones", None)
            return toggle is not None and toggle.isChecked()

    def set_mode(self, mode, source_button=None):
            if mode != "add": self._discard_bezier_draft(); self.clear_preview()
            if mode != "mark_pose":
                self._rviz_pose_anchor = None; self._rviz_drag_point = None
            if mode in ("add", "obstacle", "auto_plan", "mark_pose") and self.calibration_pending:
                self.select_button.setChecked(True)
                if hasattr(self,"status"): self.status.setText("请先完成起点和朝向标定。")
                return
            was_measuring=self.mode == "measure"; self.mode=mode; self.view.mode=mode
            if was_measuring and mode != "measure":
                self.measurement_points=[]
                if hasattr(self,"measurement_label"): self.update_measurement_ui()
                if hasattr(self,"scene"): self.redraw()
            if hasattr(self,"status"):
                message = "选择：框选、Ctrl 多选；中键或空格+左键平移地图。"
                if mode == "add": message = "添加节点：点击地图添加，按住 Shift 吸附水平、垂直或 45 度。"
                elif mode == "mark_pose": message = "设置导航目标：第一次点击位置，移动鼠标看航向辅助线，第二次点击确定方向并自动规划；Esc/右键取消。"
                elif mode == "measure": message = "测距：左键依次选择两点，生成水平和垂直对齐线；Esc 或切换工具清除。"
                elif mode == "obstacle": message = "障碍物：左键释放添加黑色圆形标记，选择模式可拖拽。"
                elif mode == "auto_plan": message = "设置终点：点击地图确定位置，调整本小段参数后再按黄色生成按钮。"
                self.status.setText(message)
            if source_button is not None:
                source_button.setChecked(True)
            elif mode == "select": self.select_button.setChecked(True)
            elif mode == "add": self.add_button.setChecked(True)
            elif mode == "mark_pose": self.mark_pose_button.setChecked(True)
            elif mode == "measure": self.measure_button.setChecked(True)
            elif mode == "obstacle": self.obstacle_button.setChecked(True)
            elif mode == "auto_plan": self.auto_plan_button.setChecked(True)
            if hasattr(self, "current_tool_label"):
                active = source_button
                if active is None:
                    active = {"select": self.select_button, "add": self.add_button,
                              "mark_pose": self.mark_pose_button, "measure": self.measure_button,
                              "obstacle": self.obstacle_button,
                              "auto_plan": self.auto_plan_button}.get(mode)
                self.current_tool_label.setText(
                    f"当前工具：{active.text() if active is not None else mode}")

    def begin_start(self, kind):
            if self._hardware_motion_active or self.timer.isActive():
                raise RuntimeError("执行期间不能修改起点帧")
            self.calibration_pending=True
            if kind in START_PRESETS:
                paper_x_mm, paper_y_mm = START_PRESETS[kind]
                self.set_start_frame(paper_x_mm, paper_y_mm, self.plan.start_heading_deg,
                                     start_kind="zone_1" if kind.endswith("1") else "zone_2")
            else:
                self.set_start_frame(self.plan.start_paper_x_mm, self.plan.start_paper_y_mm,
                                     self.plan.start_heading_deg, start_kind="custom")
            if kind == "自定义":
                self.calibration_stage="position"; self.mode="calibrate"; self.view.mode="calibrate"; self._start_preview_paper=None; self.calibration_label.setText("在地图上点击自定义起点位置"); self.update_calibration_ui(); self.redraw(); return
            self.calibration_stage="heading"; self.mode="calibrate"; self.view.mode="calibrate"; self.update_calibration_ui(); self.redraw()

    def on_map_click(self, x, y, shift=False):
            if math.isnan(x):
                self.cancel_active_interaction(); return
            if not (0 <= x <= FIELD_SIZE_MM and 0 <= y <= FIELD_SIZE_MM): return
            if self.mode == "measure":
                self.add_measurement_point(QPointF(x, y), shift); return
            if self.mode == "mark_pose":
                if self._rviz_pose_anchor is None:
                    original = QPointF(x, y)
                    snapped = snap_to_field_centerlines(original)
                    x, y = snapped.x(), snapped.y()
                    self._pending_navigation_goal_paper = None
                    self._pending_navigation_goal_yaw = None
                    self._rviz_pose_anchor = QPointF(x, y)
                    self._rviz_drag_point = QPointF(x, y)
                    self.preview_paper = QPointF(x, y)
                    self.preview_yaw_deg = self._step_end_pose(len(self.plan.steps)).yaw_deg
                    snap_note = ("（已自动吸附场地中线）" if snapped != original else "")
                    self.status.setText(
                        f"目标位置已确定{snap_note}：移动鼠标选择车头方向，再点击一次确认。")
                    self.redraw(); return
                self.confirm_marked_pose(x, y); return
            if self.mode == "obstacle":
                self.add_obstacle(QPointF(x, y)); return
            if self.mode == "auto_plan":
                snapped = snap_to_field_centerlines(QPointF(x, y))
                x, y = snapped.x(), snapped.y()
                self._pending_navigation_goal_paper = QPointF(x, y)
                self._pending_navigation_goal_yaw = self.auto_goal_yaw.value()
                self._set_goal_controls(x, y, self._pending_navigation_goal_yaw)
                self.selected_pose_label.setText(
                    f"待生成目标：图纸 X={x:.1f}, Y={y:.1f} mm，"
                    f"航向={self._pending_navigation_goal_yaw:.1f}°")
                self.generate_segment_button.setText("生成/更新本小段路径")
                self.status.setText("终点已确定。请调整本小段参数，然后点击黄色“生成本小段路径”。")
                self.redraw(); return
            if self.mode == "calibrate":
                if self.calibration_stage != "position": return
                self.set_start_frame(x, y, self.plan.start_heading_deg)
                self._start_preview_paper=None; self.calibration_stage="heading"; self.update_calibration_ui(); self.redraw(); return
            # 添加动作统一在鼠标释放时提交，避免一次点击同时触发 clicked/released 两次。
            if self.mode != "add" or self.calibration_pending: return

    def on_map_release(self, x, y, shift=False):
            if self.mode == "add": self.confirm_preview(x, y, shift)

    def cancel_active_interaction(self) -> None:
            """Cancel only the transient tool state; committed plan data is untouched."""
            if self.mode == "measure":
                self.clear_measurement(); return
            if self.mode == "mark_pose" and self._rviz_pose_anchor is not None:
                self._nav_preview_timer.stop()
                self._rviz_pose_anchor = None; self._rviz_drag_point = None
                self.clear_preview(False); self.redraw()
                self.status.setText("已取消本次导航目标。")
                return
            self.cancel_bezier_draft()

    def paper_of(self, waypoint):
            x, y = world_to_paper(Pose(waypoint.x_mm, waypoint.y_mm, waypoint.yaw_deg),
                                  self.plan.start_paper_x_mm, self.plan.start_paper_y_mm,
                                  self.plan.start_heading_deg)
            return Pose(x, y, world_yaw_to_paper_heading(self.plan.start_heading_deg,
                                                         waypoint.yaw_deg))

    def rotate_start_clockwise(self):
            if self.calibration_pending and self.calibration_stage == "heading":
                self.set_start_frame(self.plan.start_paper_x_mm, self.plan.start_paper_y_mm,
                                     self.plan.start_heading_deg - 90.0)
            elif not self.calibration_pending:
                self.set_start_frame(self.plan.start_paper_x_mm, self.plan.start_paper_y_mm,
                                     self.plan.start_heading_deg - 90.0)
            else:
                return
            self.redraw(); self.status.setText("起点朝向已顺时针旋转 90 度，可继续右击修改或点击确认朝向。")

    def set_start_frame(self, paper_x_mm: float, paper_y_mm: float, heading_deg: float,
                        *, preserve_paper_geometry: bool = True, start_kind: str | None = None) -> None:
            """修改起点帧并对固定世界目标执行一次性重基准。"""
            if self._hardware_motion_active or self.timer.isActive():
                raise RuntimeError("执行期间不能修改起点帧")
            old = StartFrame(self.plan.start_paper_x_mm, self.plan.start_paper_y_mm,
                             self.plan.start_heading_deg)
            new = StartFrame(float(paper_x_mm), float(paper_y_mm), float(heading_deg))
            self.push_undo()
            if preserve_paper_geometry and self.plan.steps:
                self.plan = rebase_plan_world_frame(self.plan, old, new)
            else:
                self.plan.start_paper_x_mm = new.paper_x_mm
                self.plan.start_paper_y_mm = new.paper_y_mm
                self.plan.start_heading_deg = new.heading_deg
            if start_kind is not None:
                self.plan.start_kind = start_kind
            self._clear_runtime_trace()
            self._sync_continuous_entries(); self.refresh_waypoints(); self.redraw()
            self.rebuild_timeline_after_edit(); self.plan_changed.emit(copy.deepcopy(self.plan))
            self.start_frame_changed.emit(new)

    def confirm_start_heading(self):
            if not (self.calibration_pending and self.calibration_stage == "heading"): return
            if not self.is_valid_start_pose():
                self.status.setText("起点车体进入黄色禁行区或超出场地边界，无法确认。")
                return
            self.calibration_pending=False; self.calibration_stage="complete"; self.set_mode("select"); self.update_calibration_ui(); self.redraw(); self.status.setText("起点标定完成。"); self.calibration_state_changed.emit(True)

    def select_box(self,rect,append):
            if not append: self.scene.clearSelection()
            for item in self.scene.items(rect):
                if isinstance(item,WaypointItem): item.setSelected(True)
            self.selected_indices={item.index for item in self.scene.selectedItems() if isinstance(item,WaypointItem)}

    def select_all(self):
            self.scene.clearSelection()
            for item in self.scene.items():
                if isinstance(item, WaypointItem) and isinstance(self.plan.steps[item.index], Waypoint):
                    item.setSelected(True)
            self.selected_indices = {item.index for item in self.scene.selectedItems() if isinstance(item, WaypointItem)}

    def push_undo(self): self._invalidate_timeline(); self.undo_stack.append(copy.deepcopy(self.plan)); self.undo_stack=self.undo_stack[-100:]; self.redo_stack.clear()

    def push_layout_undo(self): self.undo_stack.append(copy.deepcopy(self.plan)); self.undo_stack=self.undo_stack[-100:]; self.redo_stack.clear()

    def _polygon_path(self, points):
            polygon=QPolygonF([QPointF(x,y) for x,y in points]); path=QPainterPath(); path.addPolygon(polygon); path.closeSubpath(); return path

    def route_sweep(self, start, end, start_yaw=0.0, end_yaw=0.0, vmax=820.0, wmax=90.0, timeout=15.0) -> SweepGeometry:
            heading=self.plan.start_heading_deg
            config = self.current_costmap_settings()
            return build_goto_sweep(Pose(start.x(),start.y(),start_yaw+heading),Pose(end.x(),end.y(),end_yaw+heading),vmax,wmax,timeout, config.vehicle_length_mm, config.vehicle_width_mm)

    def continuous_sweep(self, start, end, start_yaw=0.0, end_yaw=0.0) -> SweepGeometry:
            heading=self.plan.start_heading_deg
            config = self.current_costmap_settings()
            return build_continuous_segment_sweep(Pose(start.x(),start.y(),start_yaw+heading),Pose(end.x(),end.y(),end_yaw+heading), config.vehicle_length_mm, config.vehicle_width_mm)

    def rotation_sweep(self, position, start_yaw, end_yaw, wmax=90.0, timeout=15.0) -> SweepGeometry:
            heading=self.plan.start_heading_deg
            config = self.current_costmap_settings()
            return build_rotation_sweep(Pose(position.x(),position.y(),start_yaw+heading),end_yaw+heading,wmax,timeout, config.vehicle_length_mm, config.vehicle_width_mm)

    def is_valid_start_pose(self):
            start=QPointF(self.plan.start_paper_x_mm,self.plan.start_paper_y_mm)
            return self.is_valid_route_segment(start,start)

    def _is_valid_start_candidate(self, x: float, y: float) -> bool:
            pose = Pose(x, y, self.plan.start_heading_deg)
            config = self.current_costmap_settings()
            out_of_bounds, hit_platform = self.sweep_violations(
                SweepGeometry([pose], [car_polygon(
                    pose, config.vehicle_length_mm, config.vehicle_width_mm)]))
            return not out_of_bounds and hit_platform.isEmpty()

    def sweep_path(self, sweep):
            path=QPainterPath()
            for polygon in sweep.polygons: path.addPath(self._polygon_path(polygon))
            return path

    def sweep_violations(self, sweep):
            config = self.current_costmap_settings()
            platform_safety = config.platform_safety_margin_mm
            platforms = [] if self.allow_yellow_zone.isChecked() else [
                QRectF(x - platform_safety, y - platform_safety,
                       450 + 2 * platform_safety, 450 + 2 * platform_safety)
                for x, y in PLATFORMS]
            platform_paths=[QPainterPath() for _ in platforms]
            for path,rect in zip(platform_paths,platforms):
                if platform_safety > 0.0:
                    path.addRoundedRect(rect, platform_safety, platform_safety,
                                        Qt.SizeMode.AbsoluteSize)
                else:
                    path.addRect(rect)
            obstacle_paths=[]
            for obstacle in self.plan.layout.obstacles:
                path=QPainterPath(); path.addEllipse(
                    obstacle.paper_x_mm-config.obstacle_radius_mm-config.obstacle_safety_margin_mm,
                    obstacle.paper_y_mm-config.obstacle_radius_mm-config.obstacle_safety_margin_mm,
                    (config.obstacle_radius_mm+config.obstacle_safety_margin_mm)*2,
                    (config.obstacle_radius_mm+config.obstacle_safety_margin_mm)*2)
                obstacle_paths.append(path)
            hit_platform=QPainterPath(); out_of_bounds=False
            for polygon in sweep.polygons:
                center_x = sum(x for x, _ in polygon) / len(polygon)
                center_y = sum(y for _, y in polygon) / len(polygon)
                in_boundary_opening = (
                    (center_y <= 300.0 and
                     (1050.0 <= center_x <= 1350.0 or center_x >= 2100.0)) or
                    (center_y >= 2100.0 and
                     (910.0 <= center_x <= 1490.0 or center_x >= 2100.0)) or
                    (center_x <= 300.0 and 910.0 <= center_y <= 1490.0) or
                    (center_x >= 2100.0 and
                     (center_y <= 300.0 or 1100.0 <= center_y <= 1300.0 or
                      center_y >= 2100.0)))
                boundary_safety = (0.0 if in_boundary_opening else
                                   config.boundary_safety_margin_mm)
                if any(x < boundary_safety - 1e-6 or
                       x > FIELD_SIZE_MM - boundary_safety + 1e-6 or
                       y < boundary_safety - 1e-6 or
                       y > FIELD_SIZE_MM - boundary_safety + 1e-6
                       for x,y in polygon): out_of_bounds=True
                body=self._polygon_path(polygon)
                for forbidden in (*platform_paths, *obstacle_paths):
                    if body.intersects(forbidden): hit_platform.addPath(body.intersected(forbidden))
            return out_of_bounds, hit_platform

    def is_valid_route_segment(self, start, end, start_yaw=0.0, end_yaw=0.0, vmax=820.0, wmax=90.0, timeout=15.0):
            out_of_bounds, hit_platform=self.sweep_violations(self.route_sweep(start,end,start_yaw,end_yaw,vmax,wmax,timeout))
            return not out_of_bounds and hit_platform.isEmpty()

    def is_valid_continuous_segment(self, start, end, start_yaw=0.0, end_yaw=0.0):
            out_of_bounds, hit_platform=self.sweep_violations(self.continuous_sweep(start,end,start_yaw,end_yaw))
            return not out_of_bounds and hit_platform.isEmpty()

    def is_valid_rotation(self, position, start_yaw, end_yaw, wmax=90.0, timeout=15.0):
            out_of_bounds, hit_platform=self.sweep_violations(self.rotation_sweep(position,start_yaw,end_yaw,wmax,timeout))
            return not out_of_bounds and hit_platform.isEmpty()

    def segment_intersects_rect(start,end,rect):
            dx=end.x()-start.x(); dy=end.y()-start.y(); low=0.0; high=1.0
            for origin,direction,minimum,maximum in ((start.x(),dx,rect.left(),rect.right()),(start.y(),dy,rect.top(),rect.bottom())):
                if direction == 0:
                    if origin < minimum or origin > maximum: return False
                    continue
                first=(minimum-origin)/direction; second=(maximum-origin)/direction
                if first > second: first,second=second,first
                low=max(low,first); high=min(high,second)
                if low > high: return False
            return True

    def auto_path_settings(self) -> AutoPathSettings:
            return AutoPathSettings(
                costmap=self.current_costmap_settings(),
                corner_radius_mm=self.auto_corner_radius.value(),
                sample_spacing_mm=self.auto_sample_spacing.value(),
                terminal_straight_mm=self.auto_terminal_straight.value(),
                yaw_mode="fixed",
                include_fixed_platforms=not self.allow_yellow_zone.isChecked(),
            )

    def current_costmap_settings(self) -> CostmapSettings:
            return CostmapSettings(
                vehicle_length_mm=self.vehicle_length.value(),
                vehicle_width_mm=VEHICLE_WIDTH_MM,
                boundary_safety_margin_mm=self.boundary_safety.value(),
                boundary_inflation_mm=self.boundary_inflation.value(),
                boundary_cost_weight=self.boundary_weight.value(),
                boundary_zone_half_width_mm=self.boundary_zone_half_width.value(),
                boundary_zone_depth_mm=self.boundary_zone_depth.value(),
                side_zone_half_length_mm=self.side_zone_half_length.value(),
                side_zone_depth_mm=self.side_zone_depth.value(),
                boundary_zone_inflation_mm=self.boundary_zone_inflation.value(),
                platform_safety_margin_mm=self.platform_safety.value(),
                platform_inflation_mm=self.platform_inflation.value(),
                platform_cost_weight=self.platform_weight.value(),
                platform_outer_inflation_mm=self.platform_outer_inflation.value(),
                platform_outer_cost_weight=self.platform_outer_weight.value(),
                obstacle_radius_mm=self.obstacle_radius.value(),
                obstacle_safety_margin_mm=self.obstacle_safety.value(),
                obstacle_inflation_mm=self.obstacle_inflation.value(),
                obstacle_cost_weight=self.obstacle_weight.value(),
            )

    def vehicle_dimensions(self) -> tuple[float, float]:
            # The initially checked Select tool emits during UI construction,
            # before the costmap controls below have been created.
            config = (self.current_costmap_settings()
                      if hasattr(self, "vehicle_length")
                      else self.plan.layout.costmap)
            return config.vehicle_length_mm, VEHICLE_WIDTH_MM

    def _load_costmap_controls(self) -> None:
            if not hasattr(self, "boundary_safety"):
                return
            config = self.plan.layout.costmap
            mapping = (
                (self.vehicle_length, config.vehicle_length_mm),
                (self.boundary_safety, config.boundary_safety_margin_mm),
                (self.boundary_inflation, config.boundary_inflation_mm),
                (self.boundary_weight, config.boundary_cost_weight),
                (self.boundary_zone_half_width, config.boundary_zone_half_width_mm),
                (self.boundary_zone_depth, config.boundary_zone_depth_mm),
                (self.side_zone_half_length, config.side_zone_half_length_mm),
                (self.side_zone_depth, config.side_zone_depth_mm),
                (self.boundary_zone_inflation,
                 config.boundary_zone_inflation_mm),
                (self.platform_safety, config.platform_safety_margin_mm),
                (self.platform_inflation, config.platform_inflation_mm),
                (self.platform_weight, config.platform_cost_weight),
                (self.platform_outer_inflation, config.platform_outer_inflation_mm),
                (self.platform_outer_weight, config.platform_outer_cost_weight),
                (self.obstacle_radius, config.obstacle_radius_mm),
                (self.obstacle_safety, config.obstacle_safety_margin_mm),
                (self.obstacle_inflation, config.obstacle_inflation_mm),
                (self.obstacle_weight, config.obstacle_cost_weight),
            )
            self._loading_costmap_controls = True
            try:
                for control, value in mapping:
                    control.setValue(value)
            finally:
                self._loading_costmap_controls = False

    def _mark_auto_paths_stale(self, *_args) -> None:
            if self._loading_costmap_controls:
                return
            if any(isinstance(step, ContinuousPathSegment) and
                   step.name.startswith("自动规划") for step in self.plan.steps):
                self._auto_paths_stale = True
                if hasattr(self, "status"):
                    self.status.setText("规划参数已改变：请重新规划全部自动航点后再上传。")
            if hasattr(self, "replan_auto_button"):
                self.replan_auto_button.setEnabled(self._auto_paths_stale)

    def _mark_selected_segment_dirty(self, *_args) -> None:
            """Trajectory controls are applied only when the yellow button is pressed."""
            if not hasattr(self, "generate_segment_button"):
                return
            self._selected_auto_segment_dirty = True
            self.generate_segment_button.setText("应用参数并更新本小段路径")
            if hasattr(self, "status"):
                self.status.setText("本小段参数已修改；现有路径未改变，请点击黄色按钮应用。")

    def _set_goal_controls(self, x_mm: float, y_mm: float, yaw_deg: float) -> None:
            for control, value in ((self.auto_goal_x, x_mm),
                                   (self.auto_goal_y, y_mm),
                                   (self.auto_goal_yaw, yaw_deg)):
                control.blockSignals(True); control.setValue(value); control.blockSignals(False)

    def _on_goal_control_changed(self, *_args) -> None:
            self._mark_selected_segment_dirty()
            if self._pending_navigation_goal_paper is None:
                return
            self._pending_navigation_goal_paper = QPointF(
                self.auto_goal_x.value(), self.auto_goal_y.value())
            self._pending_navigation_goal_yaw = self.auto_goal_yaw.value()
            self.selected_pose_label.setText(
                f"待生成目标：图纸 X={self.auto_goal_x.value():.1f}, "
                f"Y={self.auto_goal_y.value():.1f} mm，航向={self.auto_goal_yaw.value():.1f}°")
            self.redraw()

    def _on_costmap_changed(self, *_args) -> None:
            if self._loading_costmap_controls:
                return
            self.plan.layout.costmap = self.current_costmap_settings()
            self._mark_auto_paths_stale()
            self.redraw()

    def create_auto_path(self, goal_paper: QPointF, *,
                         goal_yaw_deg: float | None = None,
                         yaw_mode: str | None = None,
                         terminal_rotation: bool = False,
                         strategy_name: str = "",
                         replace_index: int | None = None) -> bool:
            if self.calibration_pending or self._hardware_motion_active or self.timer.isActive():
                self.status.setText("请先完成标定并停止当前运动。")
                return False
            existing_paths_were_stale = self._auto_paths_stale
            insertion_index = len(self.plan.steps) if replace_index is None else replace_index
            start_world = self._step_end_pose(insertion_index)
            start_paper = self.paper_of(start_world)
            settings = self.auto_path_settings()
            if yaw_mode is not None:
                settings = replace(settings, yaw_mode=yaw_mode)
            try:
                result = plan_auto_path(
                    (start_paper.x_mm, start_paper.y_mm),
                    (goal_paper.x(), goal_paper.y()),
                    Pose(self.plan.start_paper_x_mm, self.plan.start_paper_y_mm,
                         self.plan.start_heading_deg),
                    start_world.yaw_deg,
                    (self.auto_goal_yaw.value() if goal_yaw_deg is None
                     else goal_yaw_deg),
                    self.plan.layout.obstacles,
                    settings,
                )
            except AutoPathError as error:
                self.status.setText(f"自动规划失败：{error}")
                return False

            for first, second in zip(result.world_points, result.world_points[1:]):
                start = self.paper_of(first); end = self.paper_of(second)
                if not self.is_valid_continuous_segment(
                        QPointF(start.x_mm, start.y_mm), QPointF(end.x_mm, end.y_mm),
                        first.yaw_deg, second.yaw_deg):
                    self.status.setText(
                        "自动规划失败：平滑后整车扫掠进入禁区；请保持车身航向或减小圆角半径。")
                    return False

            target_yaw = (result.world_points[-1].yaw_deg
                          if goal_yaw_deg is None else goal_yaw_deg)
            needs_terminal_rotation = (terminal_rotation and
                                       abs(wrap_deg(target_yaw -
                                                    result.world_points[-1].yaw_deg)) > 0.5)
            if needs_terminal_rotation and not self.is_valid_rotation(
                    goal_paper, result.world_points[-1].yaw_deg, target_yaw):
                self.status.setText(
                    "自动规划失败：终点位置可以到达，但该位置没有足够空间完成目标转向。")
                return False

            self.push_undo()
            count = sum(isinstance(step, ContinuousPathSegment) and
                        step.name.startswith("自动规划") for step in self.plan.steps) + 1
            suffix = f" [{strategy_name}]" if strategy_name else ""
            generated_path = ContinuousPathSegment(
                list(result.world_points), f"自动规划 {count}{suffix}",
                AutoSegmentSettings(
                    corner_radius_mm=self.auto_corner_radius.value(),
                    sample_spacing_mm=self.auto_sample_spacing.value(),
                    terminal_straight_mm=self.auto_terminal_straight.value(),
                    yaw_mode=settings.yaw_mode,
                    goal_yaw_deg=target_yaw,
                    strategy=str(self.navigation_strategy.currentData()),
                ))
            if replace_index is None:
                self.plan.steps.append(generated_path)
                path_index = len(self.plan.steps) - 1
            else:
                if not (0 <= replace_index < len(self.plan.steps) and
                        isinstance(self.plan.steps[replace_index], ContinuousPathSegment) and
                        self.plan.steps[replace_index].name.startswith("自动规划")):
                    self.status.setText("只能更新选中的自动导航小段。")
                    return False
                if (replace_index + 1 < len(self.plan.steps) and
                        isinstance(self.plan.steps[replace_index + 1], RotateInPlace) and
                        self.plan.steps[replace_index + 1].name == "导航目标朝向"):
                    del self.plan.steps[replace_index + 1]
                self.plan.steps[replace_index] = generated_path
                path_index = replace_index
            if needs_terminal_rotation:
                self.plan.steps.insert(path_index + 1, RotateInPlace(
                    target_yaw, name="导航目标朝向"))
            self.active_index = path_index
            self.active_point_index = -1
            self._sync_continuous_entries()
            self._invalidate_timeline()
            self.refresh_waypoints()
            if self.continuous_goal_selection.isChecked():
                (self.mark_pose_button if yaw_mode is not None
                 else self.auto_plan_button).setChecked(True)
            else:
                self.select_button.setChecked(True)
            self.redraw()
            self._auto_paths_stale = existing_paths_were_stale
            self.rebuild_timeline_after_edit()
            self.status.setText(
                f"自动规划完成：{result.length_mm:.0f} mm，{len(result.world_points)} 点；"
                "已通过膨胀区和整段几何复验。")
            self._selected_auto_segment_dirty = False
            self.generate_segment_button.setText("生成/更新本小段路径")
            return True

    def replan_all_auto_paths(self) -> None:
            if self.calibration_pending or self._hardware_motion_active or self.timer.isActive():
                self.status.setText("请先完成标定并停止当前运动。")
                return
            original = copy.deepcopy(self.plan)
            working = copy.deepcopy(self.plan)
            self.plan = working
            replanned = 0
            try:
                for index, step in enumerate(self.plan.steps):
                    if not (isinstance(step, ContinuousPathSegment) and
                            step.name.startswith("自动规划") and step.points):
                        continue
                    start_world = self._step_end_pose(index)
                    start_paper = self.paper_of(start_world)
                    old_goal = self.paper_of(step.points[-1])
                    segment_settings = self.auto_path_settings()
                    goal_yaw = step.points[-1].yaw_deg
                    if step.auto_settings is not None:
                        segment_settings = replace(
                            segment_settings,
                            corner_radius_mm=step.auto_settings.corner_radius_mm,
                            sample_spacing_mm=step.auto_settings.sample_spacing_mm,
                            terminal_straight_mm=step.auto_settings.terminal_straight_mm,
                            yaw_mode=step.auto_settings.yaw_mode,
                        )
                        goal_yaw = step.auto_settings.goal_yaw_deg
                    result = plan_auto_path(
                        (start_paper.x_mm, start_paper.y_mm),
                        (old_goal.x_mm, old_goal.y_mm),
                        Pose(self.plan.start_paper_x_mm,
                             self.plan.start_paper_y_mm,
                             self.plan.start_heading_deg),
                        start_world.yaw_deg, goal_yaw,
                        self.plan.layout.obstacles, segment_settings)
                    for first, second in zip(result.world_points,
                                             result.world_points[1:]):
                        first_paper, second_paper = self.paper_of(first), self.paper_of(second)
                        if not self.is_valid_continuous_segment(
                                QPointF(first_paper.x_mm, first_paper.y_mm),
                                QPointF(second_paper.x_mm, second_paper.y_mm),
                                first.yaw_deg, second.yaw_deg):
                            raise AutoPathError(
                                f"{step.name} 的整车扫掠进入物理禁区")
                    step.points = list(result.world_points)
                    replanned += 1
            except (AutoPathError, ValueError) as error:
                self.plan = original
                self.status.setText(f"重新规划失败，原路径已保留：{error}")
                self.redraw()
                return
            if replanned == 0:
                self.plan = original
                self.status.setText("当前方案没有自动规划航点。")
                return
            self.undo_stack.append(original); self.undo_stack = self.undo_stack[-100:]
            self.redo_stack.clear(); self._auto_paths_stale = False
            self._sync_continuous_entries(); self._invalidate_timeline()
            self.refresh_waypoints(); self.redraw(); self.rebuild_timeline_after_edit()
            self.status.setText(f"已按当前代价地图重新规划 {replanned} 段自动路径。")

    def draw_inflated_forbidden_zones(self) -> None:
            if not hasattr(self, "show_inflated_zones") or not self.show_inflated_zones.isChecked():
                return
            try:
                rectangles = build_inflated_obstacles(
                    self.plan.layout.obstacles, self.auto_path_settings())
            except AutoPathError:
                return
            config = self.current_costmap_settings()
            body_half = max(config.vehicle_length_mm, config.vehicle_width_mm) / 2.0
            hard_clearance = body_half + config.boundary_safety_margin_mm
            soft_end = min(FIELD_SIZE_MM / 2.0,
                           hard_clearance + config.boundary_inflation_mm)
            yellow_pen = QPen(QColor("#f9a825"), 2, Qt.PenStyle.DashLine)
            yellow_brush = QColor(255, 213, 79, 62)
            pink_pen = QPen(QColor("#ec407a"), 3)
            pink_brush = QColor(236, 64, 122, 55)
            group_left, group_top, group_right, group_bottom = PLATFORM_GROUP_BOUNDS
            platform_group_path = QPainterPath()
            platform_group_path.addRect(QRectF(
                group_left, group_top, group_right - group_left,
                group_bottom - group_top))
            if soft_end > hard_clearance and config.boundary_cost_weight > 0.0:
                soft_width = soft_end - hard_clearance
                for band_rect in (
                    QRectF(hard_clearance, hard_clearance,
                           FIELD_SIZE_MM - 2 * hard_clearance, soft_width),
                    QRectF(hard_clearance, FIELD_SIZE_MM - soft_end,
                           FIELD_SIZE_MM - 2 * hard_clearance, soft_width),
                    QRectF(hard_clearance, soft_end, soft_width,
                           FIELD_SIZE_MM - 2 * soft_end),
                    QRectF(FIELD_SIZE_MM - soft_end, soft_end, soft_width,
                           FIELD_SIZE_MM - 2 * soft_end),
                ):
                    item = self.scene.addRect(band_rect, yellow_pen, yellow_brush)
                    item.setData(0, "boundary_cost_band"); item.setZValue(1)
                    item.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
            # Physical footprint exclusion (150 mm) is always hard. Extra
            # safety keeps openings through the two official start zones.
            hard_bands = [QRectF(0, 0, body_half, FIELD_SIZE_MM),
                          QRectF(FIELD_SIZE_MM - body_half, 0, body_half, FIELD_SIZE_MM),
                          QRectF(body_half, 0, FIELD_SIZE_MM - 2 * body_half, body_half),
                          QRectF(body_half, FIELD_SIZE_MM - body_half,
                                 FIELD_SIZE_MM - 2 * body_half, body_half)]
            safety = config.boundary_safety_margin_mm
            if safety > 0.0:
                hard_bands.extend((
                    QRectF(body_half, body_half, safety, 910.0 - body_half),
                    QRectF(body_half, 1490.0, safety,
                           FIELD_SIZE_MM - body_half - 1490.0),
                    QRectF(body_half, body_half, 1050.0 - body_half, safety),
                    QRectF(1350.0, body_half, 750.0, safety),
                    QRectF(FIELD_SIZE_MM - body_half - safety, 300.0,
                           safety, 800.0),
                    QRectF(FIELD_SIZE_MM - body_half - safety, 1300.0,
                           safety, 800.0),
                    QRectF(body_half, FIELD_SIZE_MM - body_half - safety,
                           910.0 - body_half, safety),
                    QRectF(1490.0, FIELD_SIZE_MM - body_half - safety,
                           610.0, safety),
                ))
            field_path = QPainterPath()
            field_path.addRect(QRectF(0, 0, FIELD_SIZE_MM, FIELD_SIZE_MM))
            boundary_body = QPainterPath()
            for band_rect in hard_bands[:4]:
                band_path = QPainterPath(); band_path.addRect(band_rect)
                boundary_body = boundary_body.united(band_path)
            boundary_safe = boundary_body
            for band_rect in hard_bands[4:]:
                band_path = QPainterPath(); band_path.addRect(band_rect)
                boundary_safe = boundary_safe.united(band_path)
            for left, top, right, bottom in boundary_inset_rects(config):
                body_zone = QPainterPath()
                body_zone.addRoundedRect(
                    QRectF(left - body_half, top - body_half,
                           right - left + 2 * body_half,
                           bottom - top + 2 * body_half),
                    body_half, body_half, Qt.SizeMode.AbsoluteSize)
                boundary_body = boundary_body.united(
                    body_zone.intersected(field_path))
                safe_clearance = body_half + config.boundary_safety_margin_mm
                safe_zone = QPainterPath()
                safe_zone.addRoundedRect(
                    QRectF(left - safe_clearance, top - safe_clearance,
                           right - left + 2 * safe_clearance,
                           bottom - top + 2 * safe_clearance),
                    safe_clearance, safe_clearance,
                    Qt.SizeMode.AbsoluteSize)
                boundary_safe = boundary_safe.united(
                    safe_zone.intersected(field_path))
            boundary_fill = self.scene.addPath(
                boundary_safe, QPen(Qt.PenStyle.NoPen), pink_brush)
            boundary_fill.setData(0, "boundary_hard_zone")
            boundary_fill.setZValue(2)
            boundary_fill.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
            body_boundary_item = self.scene.addPath(
                boundary_body, QPen(QColor("#d32f2f"), 2))
            body_boundary_item.setData(0, "boundary_inset_body_clearance")
            body_boundary_item.setZValue(2.4)
            body_boundary_item.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
            safety_boundary_item = self.scene.addPath(
                boundary_safe,
                QPen(QColor("#d32f2f"), 2, Qt.PenStyle.DashLine))
            safety_boundary_item.setData(0, "boundary_inset_safety_clearance")
            safety_boundary_item.setZValue(2.5)
            safety_boundary_item.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
            if (config.boundary_zone_inflation_mm > 0.0 and
                    config.boundary_cost_weight > 0.0):
                for rectangle in rectangles:
                    if rectangle.source != "boundary_zone":
                        continue
                    soft_path = self._costmap_shape_path(
                        rectangle, config.boundary_zone_inflation_mm)
                    soft_path = soft_path.subtracted(
                        self._costmap_shape_path(rectangle)).intersected(field_path)
                    item = self.scene.addPath(soft_path, yellow_pen, yellow_brush)
                    item.setData(0, "boundary_cost_band")
                    item.setZValue(1)
                    item.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
            platform_bands = []
            if (config.platform_inflation_mm > 0.0 and
                    config.platform_cost_weight > 0.0):
                for rectangle in rectangles:
                    if rectangle.source == "platform":
                        platform_bands.append((
                            "inner",
                            self._costmap_shape_path(
                                rectangle, config.platform_inflation_mm)
                            .intersected(platform_group_path)))
            if (config.platform_outer_inflation_mm > 0.0 and
                    config.platform_outer_cost_weight > 0.0):
                outset = config.platform_outer_inflation_mm
                expanded = QPainterPath()
                expanded.addRoundedRect(QRectF(
                    group_left - outset, group_top - outset,
                    group_right - group_left + 2 * outset,
                    group_bottom - group_top + 2 * outset),
                    outset, outset, Qt.SizeMode.AbsoluteSize)
                platform_bands.append((
                    "outer", expanded.subtracted(platform_group_path)
                    .intersected(field_path)))
            for region, soft_path in platform_bands:
                soft_item = self.scene.addPath(soft_path, yellow_pen, yellow_brush)
                soft_item.setData(0, "soft_cost_zone")
                soft_item.setData(1, region)
                soft_item.setZValue(1)
                soft_item.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
            platform_body_pen = QPen(QColor("#ec407a"), 3)
            platform_safety_pen = QPen(
                QColor("#ec407a"), 3, Qt.PenStyle.DashLine)
            platform_clearance = body_half + config.platform_safety_margin_mm
            for x, y in PLATFORMS:
                body_path = QPainterPath()
                body_path.addRoundedRect(
                    QRectF(x - body_half, y - body_half,
                           450 + 2 * body_half, 450 + 2 * body_half),
                    body_half, body_half, Qt.SizeMode.AbsoluteSize)
                body_item = self.scene.addPath(
                    body_path, platform_body_pen, QColor(236, 64, 122, 24))
                body_item.setData(0, "platform_body_clearance")
                body_item.setZValue(2.6)
                body_item.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
                if config.platform_safety_margin_mm > 0.0:
                    safety_path = QPainterPath()
                    safety_path.addRoundedRect(
                        QRectF(x - platform_clearance, y - platform_clearance,
                               450 + 2 * platform_clearance,
                               450 + 2 * platform_clearance),
                        platform_clearance, platform_clearance,
                        Qt.SizeMode.AbsoluteSize)
                    safety_item = self.scene.addPath(
                        safety_path, platform_safety_pen,
                        QColor(236, 64, 122, 18))
                    safety_item.setData(0, "platform_safety_clearance")
                    safety_item.setZValue(2.5)
                    safety_item.setAcceptedMouseButtons(
                        Qt.MouseButton.NoButton)
            for rectangle in rectangles:
                if rectangle.source in ("boundary", "boundary_zone", "platform"):
                    continue
                if (rectangle.source == "custom" and
                        config.obstacle_inflation_mm > 0.0 and
                        config.obstacle_cost_weight > 0.0):
                    soft_path = self._costmap_shape_path(
                        rectangle, config.obstacle_inflation_mm)
                    soft_item = self.scene.addPath(soft_path, yellow_pen, yellow_brush)
                    soft_item.setData(0, "soft_cost_zone")
                    soft_item.setData(1, "obstacle")
                    soft_item.setZValue(1)
                    soft_item.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
                if rectangle.hard:
                    item = self.scene.addPath(
                        self._costmap_shape_path(rectangle), pink_pen, pink_brush)
                    item.setData(0, "inflated_forbidden")
                    item.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
                    item.setZValue(2)
            split = self.scene.addRect(
                QRectF(group_left, group_top, group_right - group_left,
                       group_bottom - group_top),
                QPen(QColor("#f9a825"), 2, Qt.PenStyle.DashLine))
            split.setData(0, "platform_cost_split_boundary")
            split.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
            split.setZValue(3)
            physical_pen = QPen(QColor("#d32f2f"), 5)
            field_outline = self.scene.addPath(
                self._physical_boundary_path(config), physical_pen)
            field_outline.setData(0, "physical_forbidden_outline")
            field_outline.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
            field_outline.setZValue(3)
            for x, y in PLATFORMS:
                outline = self.scene.addRect(x, y, 450, 450, physical_pen)
                outline.setData(0, "physical_forbidden_outline")
                outline.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
                outline.setZValue(3)
            self.text(180, 175,
                      f"粉色硬边界：车体保守半径 {body_half:.0f} + 边线安全 {config.boundary_safety_margin_mm:.0f} mm",
                      "boundary_clearance_label", 14, 0, "#c2185b").setZValue(4)

    @staticmethod
    def _costmap_shape_path(shape, extra_offset: float = 0.0) -> QPainterPath:
            path = QPainterPath()
            left, top = shape.left - extra_offset, shape.top - extra_offset
            width = shape.right - shape.left + 2.0 * extra_offset
            height = shape.bottom - shape.top + 2.0 * extra_offset
            radius = max(0.0, shape.corner_radius + extra_offset)
            if shape.source == "custom":
                path.addEllipse(left, top, width, height)
            elif radius > 0.0:
                path.addRoundedRect(left, top, width, height, radius, radius,
                                    Qt.SizeMode.AbsoluteSize)
            else:
                path.addRect(left, top, width, height)
            return path

    def redraw(self):
            if hasattr(self,"raw_slider"):
                for slider,value in ((self.raw_slider,self.plan.layout.raw_center_x_mm),(self.qr_slider,self.plan.layout.qr_center_y_mm)):
                    slider.blockSignals(True); slider.setValue(round(value)); slider.blockSignals(False)
            self._runtime_car_item = None; self._runtime_direction_item = None
            self._runtime_target_item = None; self._runtime_target_direction_item = None
            self._runtime_trace_item = None
            self._runtime_trace_path_point_count = 0
            self._runtime_projection_item = None
            self._runtime_lookahead_item = None
            self.scene.clear(); self.scene.setSceneRect(-360,-300,3100,3100); self.draw_field()
            if self._costmap_overlay_visible(): self.draw_inflated_forbidden_zones()
            self.draw_start(); self.draw_route(); self.draw_pending_navigation_goal(); self.draw_measurement(); self.draw_preview(); self.draw_car(self.current_frame.actual if self.current_frame else None); self.draw_runtime_overlay(); self.position_layout_sliders()

    def draw_pending_navigation_goal(self) -> None:
            if (self._pending_navigation_goal_paper is None or
                    self._pending_navigation_goal_yaw is None):
                return
            point = self._pending_navigation_goal_paper
            color = QColor("#f9a825")
            ring = self.scene.addEllipse(point.x() - 34, point.y() - 34, 68, 68,
                                         QPen(color, 5), QColor(249, 168, 37, 65))
            ring.setData(0, "pending_navigation_goal"); ring.setZValue(25)
            self._draw_direction_arrow(point.x(), point.y(),
                                       self._pending_navigation_goal_yaw,
                                       color, "pending_navigation_goal_arrow")
            label = self.scene.addText("待生成", QFont("Microsoft YaHei", 15))
            label.setDefaultTextColor(color); label.setPos(point.x() + 40, point.y() + 25)
            label.setData(0, "pending_navigation_goal_label"); label.setZValue(27)
            for item in self.scene.items():
                if isinstance(item,WaypointItem) and item.index in self.selected_indices: item.setSelected(True)
            signature = repr(self.plan)
            if signature != self._last_plan_signature:
                self._last_plan_signature = signature
                self.plan_changed.emit(copy.deepcopy(self.plan))

    def static(self,item,marker): item.setData(0,marker); item.setZValue(-20); item.setAcceptedMouseButtons(Qt.MouseButton.NoButton); return item

    def text(self,x,y,value,marker,size=18,rotation=0,color="#263238"):
            item=self.scene.addText(value,QFont("Microsoft YaHei",size)); item.setDefaultTextColor(QColor(color)); item.setPos(x,y); item.setRotation(rotation); return self.static(item,marker)

    def line(self,*args,marker,pen): return self.static(self.scene.addLine(*args,pen),marker)

    @staticmethod
    def _physical_boundary_path(config: CostmapSettings) -> QPainterPath:
            raw_half = config.boundary_zone_half_width_mm
            raw_depth = config.boundary_zone_depth_mm
            side_half = config.side_zone_half_length_mm
            side_depth = config.side_zone_depth_mm
            center = FIELD_SIZE_MM / 2.0
            raw_left, raw_right = center - raw_half, center + raw_half
            side_top, side_bottom = center - side_half, center + side_half
            rough_left, rough_right = side_top, side_bottom
            path = QPainterPath(QPointF(0, 0))
            for x, y in ((raw_left, 0), (raw_left, raw_depth),
                         (raw_right, raw_depth), (raw_right, 0),
                         (FIELD_SIZE_MM, 0),
                         (FIELD_SIZE_MM, FIELD_SIZE_MM),
                         (rough_right, FIELD_SIZE_MM),
                         (rough_right, FIELD_SIZE_MM - side_depth),
                         (rough_left, FIELD_SIZE_MM - side_depth),
                         (rough_left, FIELD_SIZE_MM), (0, FIELD_SIZE_MM),
                         (0, side_bottom), (side_depth, side_bottom),
                         (side_depth, side_top), (0, side_top), (0, 0)):
                path.lineTo(x, y)
            return path

    def draw_field(self):
            config = (self.current_costmap_settings()
                      if hasattr(self, "boundary_zone_half_width")
                      else self.plan.layout.costmap)
            self.static(self.scene.addRect(0,0,2400,2400,QPen(Qt.PenStyle.NoPen),QColor("#ffffff")),"field_background")
            for x in range(0,2401,200): self.line(x,0,x,2400,marker="grid",pen=QPen(QColor(80,80,80,25),1))
            for y in range(0,2401,200): self.line(0,y,2400,y,marker="grid",pen=QPen(QColor(80,80,80,25),1))
            for x,y in PLATFORMS: self.static(self.scene.addRect(x,y,450,450,QPen(QColor("#d32f2f"),5),QColor("#fffde7")),"platform")
            for left, top, right, bottom in boundary_inset_rects(config):
                self.static(self.scene.addRect(
                    left, top, right-left, bottom-top,
                    QPen(Qt.PenStyle.NoPen), QColor("#ffffff")),
                    "boundary_inset_zone")
            self.static(self.scene.addPath(
                self._physical_boundary_path(config), QPen(QColor("#d32f2f"), 6)),
                "field_boundary")
            dash=QPen(QColor("#616161"),3,Qt.PenStyle.DashLine); self.line(1200,0,1200,2400,marker="center_line",pen=dash); self.line(0,1200,2400,1200,marker="center_line",pen=dash)
            for x,y,label in ((2250,150,"启停区 1"),(2250,2250,"启停区 2")):
                zone = self.static(self.scene.addRect(x-150,y-150,300,300,QPen(Qt.PenStyle.NoPen),QColor("#114ce0")),"start_zone"); zone.setZValue(4)
                label_item = self.text(x-115,y+162,label,"start_zone_label",20); label_item.setZValue(5)
                h_center = self.line(x-18,y,x+18,y,marker="start_zone_center",pen=QPen(QColor("#ffffff"),3)); h_center.setZValue(5)
                v_center = self.line(x,y-18,x,y+18,marker="start_zone_center",pen=QPen(QColor("#ffffff"),3)); v_center.setZValue(5)
            raw_x=self.plan.layout.raw_center_x_mm; raw=DraggableEllipseItem((-150,-150,300,300),self.move_raw_area); raw.setPos(raw_x,RAW_CENTER_Y_MM); raw.setPen(QPen(QColor("#444"),4)); raw.setBrush(QColor("#f7f7f7")); raw.setData(0,"raw_turntable"); raw.setZValue(5); self.scene.addItem(raw)
            for angle in (90, 210, 330):
                radians=math.radians(angle); x=raw_x+100*math.cos(radians); y=RAW_CENTER_Y_MM+100*math.sin(radians)
                self.static(self.scene.addEllipse(x-25,y-25,50,50,QPen(QColor("#444"),3),QColor("white")),"raw_pick_hole")
            for x,y in MATERIAL_SLOTS:
                self.static(self.scene.addEllipse(x-40,y-40,80,80,QPen(QColor("#222"),4),QColor("white")),"material_slot_outer"); self.static(self.scene.addEllipse(x-13,y-13,26,26,QPen(Qt.PenStyle.NoPen),QColor("#222")),"material_slot_inner")
            qr=DraggableRectItem((-4,-100,8,200),self.move_qr_board); qr.setPos(QR_CENTER_X_MM,self.plan.layout.qr_center_y_mm); qr.setPen(QPen(Qt.PenStyle.NoPen)); qr.setBrush(QColor("#212121")); qr.setData(0,"qr_board"); qr.setZValue(5); self.scene.addItem(qr)
            self.text(720,105,"原料区","raw_label",26); self.text(180,1260,"暂存区","storage_label",24,90); self.text(1320,2180,"粗加工区","rough_label",26); self.text(2290,1300,"二维码板","coding_label",22,90); self.text(650,2420,"红线为物理禁碰轮廓；粉色为车体中心硬禁区；黄色为软代价区","movable_boundary_label",18,0,"#c62828")
            self.draw_dimensions()
            for index,obstacle in enumerate(self.plan.layout.obstacles):
                radius = self.current_costmap_settings().obstacle_radius_mm
                item=DraggableEllipseItem((-radius,-radius,radius*2,radius*2),lambda position,index=index:self.move_obstacle(index,position)); item.setPos(obstacle.paper_x_mm,obstacle.paper_y_mm); item.setBrush(QColor("#000000")); item.setPen(QPen(QColor("#d32f2f"),4)); item.setData(0,"obstacle"); item.setData(1,index); item.setZValue(20); self.scene.addItem(item)
            if hasattr(self, "obstacle_count_label"):
                self.obstacle_count_label.setText(f"障碍物：{len(self.plan.layout.obstacles)} 个")

    def draw_dimensions(self):
            blue=QColor("#1748c5"); pen=QPen(blue,2)
            def h(x1,x2,edge,dim,label,key):
                self.line(x1,edge,x1,dim,marker=key,pen=QPen(blue)); self.line(x2,edge,x2,dim,marker=key,pen=QPen(blue)); self.line(x1,dim,x2,dim,marker=key,pen=pen); self.text((x1+x2)/2-32,dim-32,label,key,18,0,"#1748c5")
            def v(y1,y2,edge,dim,label,key):
                self.line(edge,y1,dim,y1,marker=key,pen=QPen(blue)); self.line(edge,y2,dim,y2,marker=key,pen=QPen(blue)); self.line(dim,y1,dim,y2,marker=key,pen=pen); self.text(dim+8,(y1+y2)/2,label,key,18,90,"#1748c5")
            h(0,2400,2400,2530,"2400","dim_2400w"); v(0,2400,2400,2500,"2400","dim_2400h"); h(550,1000,1000,1060,"450","dim_platform_450"); v(550,1000,550,490,"450","dim_platform_450"); h(1000,1400,550,470,"400","dim_channel_400"); v(1000,1400,1400,1480,"400","dim_channel_400"); h(2100,2400,0,-100,"300","dim_start_300"); v(0,300,2400,2470,"300","dim_start_300"); h(0,150,0,-180,"150","dim_storage_150"); v(910,1490,0,-90,"580","dim_storage_580"); h(910,1490,2400,2620,"580","dim_rough_580"); v(2250,2400,1490,1570,"150","dim_rough_150"); h(1050,1350,-70,-270,"Ø300","dim_raw_diameter"); h(1100,1300,-70,-150,"Ø200","dim_raw_pitch"); v(1100,1300,2400,2570,"1100～1300","dim_qr_range")

    def draw_start(self):
            if self.calibration_pending and self.calibration_stage == "choose": return
            if self.calibration_pending and self.calibration_stage == "position":
                if self._start_preview_paper is None: return
                x, y = self._start_preview_paper.x(), self._start_preview_paper.y()
                interactive = False
            else:
                x, y = self.plan.start_paper_x_mm, self.plan.start_paper_y_mm
                interactive = self.calibration_pending and self.calibration_stage == "heading"
            valid = (self._is_valid_start_candidate(x, y)
                     if self.calibration_pending else True)
            color = QColor("#1565c0") if valid else QColor("#c62828")
            car = CarOutlineItem(self.rotate_start_clockwise if interactive else (lambda: None),
                                 *self.vehicle_dimensions())
            car.setPos(x, y); car.setRotation(qgraphics_rotation_deg(self.plan.start_heading_deg, 0.0))
            car.setPen(QPen(color, 5)); car.setBrush(QColor(color.red(), color.green(), color.blue(), 55))
            car.setAcceptedMouseButtons(Qt.MouseButton.NoButton); car.setData(0, "start_pose_preview"); car.setZValue(16); self.scene.addItem(car)
            item=StartItem(self.plan.start_heading_deg,self.rotate_start_clockwise); item.setPos(x, y)
            if not interactive: item.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
            item.setData(0, "start_direction_preview"); self.scene.addItem(item)
            self._draw_start_axes(x, y)

    def _draw_start_axes(self, paper_x: float, paper_y: float) -> None:
            """绘制固定世界坐标方向辅助线，不参与交互和路径数据。"""
            origin = QPointF(paper_x, paper_y)
            right = world_to_paper(Pose(200.0, 0.0), self.plan.start_paper_x_mm,
                                   self.plan.start_paper_y_mm, self.plan.start_heading_deg)
            forward = world_to_paper(Pose(0.0, 200.0), self.plan.start_paper_x_mm,
                                     self.plan.start_paper_y_mm, self.plan.start_heading_deg)
            for end, color in ((QPointF(*right), QColor("#ef6c00")),
                               (QPointF(*forward), QColor("#2e7d32"))):
                line = self.scene.addLine(origin.x(), origin.y(), end.x(), end.y(), QPen(color, 3))
                line.setZValue(12); line.setAcceptedMouseButtons(Qt.MouseButton.NoButton); line.setData(0, "start_axis")
            arc = self.scene.addEllipse(paper_x - 70, paper_y - 70, 140, 140,
                                        QPen(QColor("#6a1b9a"), 2))
            arc.setStartAngle(0); arc.setSpanAngle(90 * 16); arc.setZValue(12)
            arc.setAcceptedMouseButtons(Qt.MouseButton.NoButton); arc.setData(0, "start_yaw_arc")

    def add_measurement_point(self, point, shift=False):
            if len(self.measurement_points) >= 2: self.measurement_points=[]
            if shift and self.measurement_points:
                point=snap_to_45(self.measurement_points[0],point)
            self.measurement_points.append(QPointF(point)); self.update_measurement_ui(); self.redraw()

    def add_obstacle(self, point):
            if not (0 <= point.x() <= FIELD_SIZE_MM and 0 <= point.y() <= FIELD_SIZE_MM): return
            self.push_layout_undo(); self.plan.layout.obstacles.append(Obstacle(*self.clamp_obstacle(point)))
            self._mark_auto_paths_stale(); self.redraw()

    def remove_selected_obstacles(self) -> None:
            indices = sorted({int(item.data(1)) for item in self.scene.selectedItems()
                              if item.data(0) == "obstacle" and item.data(1) is not None},
                             reverse=True)
            if not indices:
                self.status.setText("请先点“选择/拖动”，再选中一个或多个黑色障碍物。")
                return
            self.push_layout_undo()
            for index in indices:
                if 0 <= index < len(self.plan.layout.obstacles):
                    del self.plan.layout.obstacles[index]
            self._mark_auto_paths_stale(); self.redraw()
            self.status.setText(f"已删除 {len(indices)} 个障碍物。")

    def clamp_obstacle(self, point):
            radius = self.current_costmap_settings().obstacle_radius_mm
            return (max(radius,min(FIELD_SIZE_MM-radius,point.x())),
                    max(radius,min(FIELD_SIZE_MM-radius,point.y())))

    def move_obstacle(self, index, position):
            if not 0 <= index < len(self.plan.layout.obstacles): return
            next_x, next_y = self.clamp_obstacle(position)
            obstacle = self.plan.layout.obstacles[index]
            if (math.isclose(obstacle.paper_x_mm, next_x, abs_tol=1e-6) and
                    math.isclose(obstacle.paper_y_mm, next_y, abs_tol=1e-6)):
                return
            self.push_layout_undo()
            obstacle.paper_x_mm, obstacle.paper_y_mm = next_x, next_y
            self._mark_auto_paths_stale(); self.redraw()

    def move_raw_area(self, position):
            self.push_layout_undo(); self.plan.layout.raw_center_x_mm=max(RAW_CENTER_X_RANGE[0],min(RAW_CENTER_X_RANGE[1],position.x())); self.redraw()

    def move_qr_board(self, position):
            self.push_layout_undo(); self.plan.layout.qr_center_y_mm=max(QR_CENTER_Y_RANGE[0],min(QR_CENTER_Y_RANGE[1],position.y())); self.redraw()

    def clear_measurement(self):
            self.measurement_points=[]; self.update_measurement_ui(); self.redraw()

    def update_measurement_ui(self):
            if len(self.measurement_points) < 2:
                self.measurement_label.setText("水平：--  垂直：--  欧式：--")
                return
            first, second=self.measurement_points
            horizontal=abs(second.x()-first.x()); vertical=abs(second.y()-first.y())
            self.measurement_label.setText(f"水平：{horizontal:.1f} mm  垂直：{vertical:.1f} mm  欧式：{math.hypot(horizontal, vertical):.1f} mm")

    def draw_measurement(self):
            if not self.measurement_points: return
            pen=QPen(QColor("#7b1fa2"),4,Qt.PenStyle.DashLine)
            for point in self.measurement_points:
                marker=self.scene.addEllipse(point.x()-13,point.y()-13,26,26,QPen(QColor("#4a148c"),3),QColor("#ffffff")); marker.setZValue(30)
            if len(self.measurement_points) == 2:
                first, second=self.measurement_points; line=self.scene.addLine(first.x(),first.y(),second.x(),second.y(),pen); line.setZValue(29)
                guide=QPen(QColor(123,31,162,90),3,Qt.PenStyle.DashLine)
                horizontal=self.scene.addLine(0,first.y(),FIELD_SIZE_MM,first.y(),guide); horizontal.setData(0,"measurement_horizontal_guide"); horizontal.setZValue(28)
                vertical=self.scene.addLine(second.x(),0,second.x(),FIELD_SIZE_MM,guide); vertical.setData(0,"measurement_vertical_guide"); vertical.setZValue(28)

    def _invalidate_timeline(self):
            if not hasattr(self,"progress"): return
            self.pause(); self.timeline=[]; self.timeline_position=0; self.current_frame=None; self.actual_trace=[]
            self.progress.blockSignals(True); self.progress.setRange(0,0); self.progress.setValue(0); self.progress.blockSignals(False); self.progress.setEnabled(False)
            self.progress_label.setText("进度：0.00 / 0.00 s")

    def _update_progress_ui(self):
            total=self.timeline[-1].time_s if self.timeline else 0.0
            current=self.current_frame.time_s if self.current_frame else 0.0
            self.progress_label.setText(f"进度：{current:.2f} / {total:.2f} s")

    def seek_timeline(self, position):
            if not self.timeline: return
            self.timeline_position=max(0,min(position,len(self.timeline))); self.current_frame=self.timeline[self.timeline_position-1] if self.timeline_position else None
            self.actual_trace=[frame.actual for frame in self.timeline[:self.timeline_position]]; self._update_progress_ui(); self.redraw()

    def pause(self): self.timer.stop()

    def reset_simulation(self): self.pause(); self.progress.setValue(0) if self.timeline else self.redraw()

    def tick(self):
            if not self.timeline: return
            next_position=min(self.timeline_position+1,len(self.timeline)); self.progress.setValue(next_position)
            frame=self.current_frame
            if frame: self.status.setText(f"t={frame.time_s:.2f}s  速度={frame.speed_mm_s:.1f} mm/s  误差={frame.error_mm:.1f} mm")
            if self.timeline_position >= len(self.timeline): self.pause()

    def refresh_plans(self): self.plan_list.clear(); self.plan_list.addItems(list_plans())

    def save(self):
            if self.calibration_pending: self.status.setText("请先完成起点标定。"); return
            if not self.is_valid_start_pose(): self.status.setText("无法保存：起点车体进入黄色禁行区或超出场地边界。"); return
            invalid=self.invalid_waypoints()
            if invalid: self.status.setText("无法保存，越界步骤："+"、".join(str(i+1) for i in invalid)); return
            try: save_plan(self.plan); self.refresh_plans(); self.status.setText(f"已保存：{self.plan.name}")
            except ValueError: self.save_as()

    def save_as(self):
            if self.calibration_pending or not self.is_valid_start_pose() or self.invalid_waypoints(): self.status.setText("完成标定并修正越界节点后才能保存。"); return
            name,ok=QInputDialog.getText(self,"另存方案","方案名称",text=self.plan.name)
            if ok and name:
                try: save_plan(self.plan,name); self.refresh_plans()
                except ValueError as error: QMessageBox.warning(self,"保存失败",str(error))

    def rename_selected(self):
            item = self.plan_list.currentItem()
            if item is None:
                self.status.setText("请先选择要重命名的方案。")
                return
            old_name = item.text()
            new_name, ok = QInputDialog.getText(self, "重命名方案", "新方案名称", text=old_name)
            new_name = new_name.strip()
            if not ok or not new_name or new_name == old_name:
                return
            try:
                rename_plan(old_name, new_name)
                if self.plan.name == old_name:
                    self.plan.name = new_name
                self.refresh_plans()
                matches = self.plan_list.findItems(new_name, Qt.MatchFlag.MatchExactly)
                if matches:
                    self.plan_list.setCurrentItem(matches[0])
                self.status.setText(f"已重命名：{old_name} → {new_name}")
            except ValueError as error:
                QMessageBox.warning(self, "重命名失败", str(error))

    def _preview_anchor(self):
            if isinstance(self._selected_step(), ContinuousPathSegment):
                step=self._selected_step()
                if step.points:
                    p=self.paper_of(step.points[-1]); return QPointF(p.x_mm,p.y_mm),step.points[-1].yaw_deg,self.active_index
            return self._step_anchor(len(self.plan.steps)) + (self.active_index,)

    def new_plan(self):
            self._discard_bezier_draft(); self._invalidate_timeline(); self.plan=Plan(); self.active_index=-1; self.active_point_index=-1; self._pending_navigation_goal_paper=None; self._pending_navigation_goal_yaw=None; self.undo_stack=[]; self.redo_stack=[]; self._auto_paths_stale=False; self._load_costmap_controls(); self.calibration_pending=True; self.calibration_stage="choose"; self.set_mode("select"); self.update_calibration_ui(); self.refresh_waypoints(); self.redraw()

    def load_selected(self):
            item=self.plan_list.currentItem()
            if item is None: return
            try:
                self._discard_bezier_draft(); self._invalidate_timeline(); self.plan=load_plan(item.text()); self.active_index=-1; self.active_point_index=-1; self._pending_navigation_goal_paper=None; self._pending_navigation_goal_yaw=None; self._auto_paths_stale=False; self._load_costmap_controls(); self.calibration_pending=False; self.calibration_stage="complete"; self.update_calibration_ui(); self.refresh_waypoints(); self.redraw()
            except ValueError as error: QMessageBox.warning(self,"加载失败",str(error))

    def _selected_step(self):
            return self.plan.steps[self.active_index] if 0 <= self.active_index < len(self.plan.steps) else None

    def _step_end_pose(self, index):
            pose = Pose()
            for step in self.plan.steps[:index]:
                if isinstance(step, Waypoint):
                    pose = Pose(step.x_mm, step.y_mm, step.yaw_deg if step.use_yaw else pose.yaw_deg)
                elif isinstance(step, RotateInPlace):
                    pose.yaw_deg = step.yaw_deg
                elif isinstance(step, ContinuousPathSegment) and step.points:
                    point = step.points[-1]
                    pose = Pose(point.x_mm, point.y_mm, point.yaw_deg)
                elif isinstance(step, BezierPathSegment):
                    end_yaw = (bezier_tangent_yaw(
                        pose, (step.control_1_x_mm, step.control_1_y_mm),
                        (step.control_2_x_mm, step.control_2_y_mm),
                        Pose(step.end_x_mm, step.end_y_mm, step.end_yaw_deg), 1.0)
                        if step.yaw_mode == "tangent" else step.end_yaw_deg)
                    pose = Pose(step.end_x_mm, step.end_y_mm, end_yaw)
            return pose

    def _step_anchor(self, index):
            pose = self._step_end_pose(index)
            paper = self.paper_of(pose)
            return QPointF(paper.x_mm, paper.y_mm), pose.yaw_deg

    def _sync_continuous_entries(self):
            for index, step in enumerate(self.plan.steps):
                if isinstance(step, ContinuousPathSegment):
                    anchor = self._step_end_pose(index)
                    if step.points:
                        entry = step.points[0]
                        entry.x_mm, entry.y_mm, entry.yaw_deg = anchor.x_mm, anchor.y_mm, anchor.yaw_deg
                        entry.name = "入口点"
                    else:
                        step.points.append(PathPosePoint(anchor.x_mm, anchor.y_mm, anchor.yaw_deg, "入口点"))

    def refresh_mode_ui(self):
            self.steps_label.setText("导航目标与手动动作")
            self.continuous_panel.setVisible(isinstance(self._selected_step(), ContinuousPathSegment))

    def refresh_waypoints(self):
            self.waypoint_list.blockSignals(True)
            self.runtime_waypoint_list.blockSignals(True)
            self.waypoint_list.clear()
            self.runtime_waypoint_list.clear()
            self._waypoint_row_to_step_index = []
            for index, step in enumerate(self.plan.steps):
                is_internal_rotation = (isinstance(step, RotateInPlace) and
                                        step.name == "导航目标朝向" and index > 0 and
                                        isinstance(self.plan.steps[index - 1], ContinuousPathSegment) and
                                        self.plan.steps[index - 1].name.startswith("自动规划"))
                if is_internal_rotation and not self.show_generated_details.isChecked():
                    continue
                if isinstance(step, Waypoint):
                    text = f"{index + 1}. 到点停靠 ({step.x_mm:.0f}, {step.y_mm:.0f})"
                elif isinstance(step, RotateInPlace):
                    text = f"{index + 1}. 原地转向 {step.yaw_deg:.0f} deg"
                elif isinstance(step, ContinuousPathSegment):
                    text = f"{index + 1}. 连续路径段 {step.name} ({max(0, len(step.points) - 1)} 点)"
                    if step.points and step.name.startswith("自动规划"):
                        endpoint = self.paper_of(step.points[-1])
                        terminal = (self.plan.steps[index + 1]
                                    if index + 1 < len(self.plan.steps) and
                                    isinstance(self.plan.steps[index + 1], RotateInPlace) and
                                    self.plan.steps[index + 1].name == "导航目标朝向" else None)
                        target_yaw = terminal.yaw_deg if terminal is not None else step.points[-1].yaw_deg
                        text = (f"{index + 1}. 导航目标  X={endpoint.x_mm:.0f}, Y={endpoint.y_mm:.0f},"
                                f" 航向={target_yaw:.0f}°  Δ启1 X={endpoint.x_mm - START_PRESETS['启停区 1'][0]:+.0f}"
                                f" Y={endpoint.y_mm - START_PRESETS['启停区 1'][1]:+.0f} mm")
                        if self.show_generated_details.isChecked():
                            text += f"  [内部路径 {max(0, len(step.points) - 1)} 点]"
                else:
                    text = f"{index + 1}. 曲线路径 {step.name}"
                self.waypoint_list.addItem(text)
                self.runtime_waypoint_list.addItem(text)
                self._waypoint_row_to_step_index.append(index)
            selected_step_index = self.active_index
            if (selected_step_index > 0 and
                    isinstance(self.plan.steps[selected_step_index], RotateInPlace) and
                    self.plan.steps[selected_step_index].name == "导航目标朝向" and
                    not self.show_generated_details.isChecked()):
                selected_step_index -= 1
            if selected_step_index in self._waypoint_row_to_step_index:
                self.waypoint_list.setCurrentRow(
                    self._waypoint_row_to_step_index.index(selected_step_index))
                self.runtime_waypoint_list.setCurrentRow(
                    self._waypoint_row_to_step_index.index(selected_step_index))
            self.waypoint_list.blockSignals(False)
            self.runtime_waypoint_list.blockSignals(False)
            has_auto = any(isinstance(step, ContinuousPathSegment) and
                           step.name.startswith("自动规划") for step in self.plan.steps)
            self.replan_auto_button.setEnabled(not self.calibration_pending and has_auto)
            self.refresh_mode_ui()
            if 0 <= self.active_index < len(self.plan.steps):
                self.show_node(self.active_index)
            self.codegen_button.setEnabled(not self.calibration_pending and bool(self.plan.steps)
                                           and not self._auto_paths_stale)

    def activate_list_row(self, row):
            if not 0 <= row < len(getattr(self, "_waypoint_row_to_step_index", [])):
                return
            self.activate_node(self._waypoint_row_to_step_index[row])

    def activate_node(self, index):
            if not 0 <= index < len(self.plan.steps):
                return
            self.active_index, self.active_point_index = index, -1
            self.show_node(index)
            self.refresh_mode_ui()
            selected_index = index
            if (selected_index > 0 and
                    isinstance(self.plan.steps[selected_index], RotateInPlace) and
                    self.plan.steps[selected_index].name == "导航目标朝向" and
                    not self.show_generated_details.isChecked()):
                selected_index -= 1
            if selected_index in self._waypoint_row_to_step_index:
                row = self._waypoint_row_to_step_index.index(selected_index)
                for step_list in (self.waypoint_list,
                                  self.runtime_waypoint_list):
                    step_list.blockSignals(True)
                    step_list.setCurrentRow(row)
                    step_list.blockSignals(False)
            self.redraw(); self.candidate_selected.emit(index)

    def _load_auto_segment_controls(self, settings: AutoSegmentSettings,
                                    endpoint: Pose) -> None:
            controls = (
                (self.auto_goal_x, endpoint.x_mm),
                (self.auto_goal_y, endpoint.y_mm),
                (self.auto_corner_radius, settings.corner_radius_mm),
                (self.auto_sample_spacing, settings.sample_spacing_mm),
                (self.auto_terminal_straight, settings.terminal_straight_mm),
                (self.auto_goal_yaw, settings.goal_yaw_deg),
            )
            for control, value in controls:
                control.blockSignals(True); control.setValue(value); control.blockSignals(False)
            yaw_index = self.auto_yaw_mode.findData(settings.yaw_mode)
            strategy_index = self.navigation_strategy.findData(settings.strategy)
            self.auto_yaw_mode.blockSignals(True)
            self.auto_yaw_mode.setCurrentIndex(yaw_index if yaw_index >= 0 else 0)
            self.auto_yaw_mode.blockSignals(False)
            self.navigation_strategy.blockSignals(True)
            self.navigation_strategy.setCurrentIndex(
                strategy_index if strategy_index >= 0 else 0)
            self.navigation_strategy.blockSignals(False)
            self._selected_auto_segment_dirty = False

    def show_node(self, index):
            step = self._selected_step()
            if step is None:
                if hasattr(self, "selected_pose_label"):
                    self.selected_pose_label.setText("当前航点：未选择")
                return
            if hasattr(self, "selected_pose_label"):
                pose = self._step_end_pose(index + 1)
                paper = self.paper_of(pose)
                self.selected_pose_label.setText(
                    f"当前航点：世界 X={pose.x_mm:.1f}, Y={pose.y_mm:.1f}, 航向={pose.yaw_deg:.1f}°；"
                    f"图纸 X={paper.x_mm:.1f}, Y={paper.y_mm:.1f} mm")
            continuous, rotating = isinstance(step, ContinuousPathSegment), isinstance(step, RotateInPlace)
            if isinstance(step, BezierPathSegment):
                self.continuous_panel.setVisible(False)
                self._show_bezier_editor(step, self._step_end_pose(index))
                for widget in self.goto_form_widgets:
                    widget.setVisible(False)
                for widget in (self.yaw, self.node_wmax, self.timeout, self.update_action_button):
                    widget.setVisible(False)
                return
            self.bezier_panel.setVisible(False)
            self.continuous_panel.setVisible(continuous)
            for widget in self.goto_form_widgets:
                widget.setVisible(not continuous and not rotating)
            for widget in (self.yaw, self.node_wmax, self.timeout, self.update_action_button):
                widget.setVisible(not continuous)
            if continuous:
                if step.name.startswith("自动规划") and step.auto_settings is not None:
                    endpoint = self.paper_of(step.points[-1]) if step.points else Pose()
                    self._load_auto_segment_controls(step.auto_settings, endpoint)
                    self.generate_segment_button.setText("生成/更新本小段路径")
                self.continuous_list.blockSignals(True)
                self.continuous_list.clear()
                for point_index, point in enumerate(step.points):
                    label = "入口点" if point_index == 0 else ("最终停车点" if point_index == len(step.points) - 1 else f"软途经点 {point_index}")
                    self.continuous_list.addItem(f"{label}: ({point.x_mm:.0f}, {point.y_mm:.0f}) {point.yaw_deg:.0f} deg")
                self.continuous_list.blockSignals(False)
                if step.points:
                    self.active_point_index = max(0, min(self.active_point_index, len(step.points) - 1))
                    self.continuous_list.setCurrentRow(self.active_point_index)
                    self.show_continuous_point(self.active_point_index)
                return
            self.update_action_button.setText("更新原地转向" if rotating else "更新到点停靠")
            self.yaw.setValue(step.yaw_deg)
            self.node_wmax.setValue(step.wmax_deg_s)
            self.timeout.setValue(step.timeout_s)
            if isinstance(step, Waypoint):
                self.x.setValue(step.x_mm); self.y.setValue(step.y_mm); self.use_yaw.setChecked(step.use_yaw)
                self.stop.setChecked(step.stop); self.dwell.setValue(step.dwell_s); self.node_vmax.setValue(step.vmax_mm_s)

    def _show_bezier_editor(self, step, preceding):
            self.bezier_panel.setVisible(True)
            mode_index = self.bezier_yaw_mode.findData(step.yaw_mode)
            self.bezier_yaw_mode.setCurrentIndex(mode_index if mode_index >= 0 else 0)
            self.bezier_start_yaw.setValue(preceding.yaw_deg if self.bezier_draft is None else self.bezier_draft_start_yaw)
            self.bezier_end_yaw.setValue(step.end_yaw_deg)
            tangent = self.bezier_yaw_mode.currentData() == "tangent"
            self.bezier_start_yaw_label.setVisible(not tangent); self.bezier_start_yaw.setVisible(not tangent)
            self.bezier_end_yaw_label.setVisible(not tangent); self.bezier_end_yaw.setVisible(not tangent)

    def _bezier_tangent_yaw(self, start, step):
            return bezier_tangent_yaw(start, (step.control_1_x_mm, step.control_1_y_mm),
                                      (step.control_2_x_mm, step.control_2_y_mm),
                                      Pose(step.end_x_mm, step.end_y_mm, step.end_yaw_deg), 0.0)

    def _insert_bezier_start_rotation(self, index, yaw):
            previous = self._step_end_pose(index)
            if abs(wrap_deg(yaw - previous.yaw_deg)) <= 0.5:
                return index
            if index > 0 and isinstance(self.plan.steps[index - 1], RotateInPlace):
                self.plan.steps[index - 1].yaw_deg = yaw
                return index
            self.plan.steps.insert(index, RotateInPlace(yaw_deg=yaw, name="曲线起点转向"))
            return index + 1

    def apply_bezier_heading(self):
            step = self.bezier_draft or self._selected_step()
            if not isinstance(step, BezierPathSegment):
                return
            mode = self.bezier_yaw_mode.currentData()
            step.yaw_mode = mode
            if mode == "interpolate":
                step.end_yaw_deg = self.bezier_end_yaw.value()
                requested_start_yaw = self.bezier_start_yaw.value()
            else:
                base = self._step_end_pose(len(self.plan.steps) if self.bezier_draft is not None else self.active_index)
                requested_start_yaw = self._bezier_tangent_yaw(base, step)
            if self.bezier_draft is not None:
                self.bezier_draft_start_yaw = requested_start_yaw
                self._bezier_preview_cache_key = None; self.redraw(); self._show_bezier_editor(step, self._step_end_pose(len(self.plan.steps)))
                return
            self.push_undo(); self.active_index = self._insert_bezier_start_rotation(self.active_index, requested_start_yaw)
            self._sync_continuous_entries(); self.refresh_waypoints(); self.redraw(); self.rebuild_timeline_after_edit()

    def refresh_bezier_draft_preview(self):
            if not isinstance(self.bezier_draft, BezierPathSegment):
                return
            self.bezier_draft.yaw_mode = self.bezier_yaw_mode.currentData()
            self.bezier_draft.end_yaw_deg = self.bezier_end_yaw.value()
            self.bezier_draft_start_yaw = self.bezier_start_yaw.value()
            self._bezier_preview_cache_key = None; self._show_bezier_editor(self.bezier_draft, self._step_end_pose(len(self.plan.steps))); self.redraw()

    def add_continuous_segment(self):
            self._discard_bezier_draft(); self.push_undo()
            anchor = self._step_end_pose(len(self.plan.steps))
            count = sum(isinstance(step, ContinuousPathSegment) for step in self.plan.steps) + 1
            self.plan.steps.append(ContinuousPathSegment([PathPosePoint(anchor.x_mm, anchor.y_mm, anchor.yaw_deg, "入口点")], f"连续段 {count}"))
            self.active_index, self.active_point_index = len(self.plan.steps) - 1, 0
            self.refresh_waypoints(); self.redraw(); self.rebuild_timeline_after_edit()

    def append_rotation(self):
            self.push_undo()
            self.plan.steps.append(RotateInPlace(yaw_deg=self._step_end_pose(len(self.plan.steps)).yaw_deg))
            self.active_index, self.active_point_index = len(self.plan.steps) - 1, -1
            self.refresh_waypoints(); self.redraw(); self.rebuild_timeline_after_edit()

    def move_step(self, delta):
            target = self.active_index + delta
            if not (0 <= self.active_index < len(self.plan.steps) and 0 <= target < len(self.plan.steps)):
                return
            self.push_undo()
            self.plan.steps[self.active_index], self.plan.steps[target] = self.plan.steps[target], self.plan.steps[self.active_index]
            self.active_index = target
            self._sync_continuous_entries()
            self.refresh_waypoints(); self.redraw(); self.rebuild_timeline_after_edit()

    def update_waypoint(self):
            step = self._selected_step()
            if not isinstance(step, (Waypoint, RotateInPlace)):
                return
            self.push_undo()
            step.yaw_deg, step.wmax_deg_s, step.timeout_s = self.yaw.value(), self.node_wmax.value(), self.timeout.value()
            if isinstance(step, Waypoint):
                step.x_mm, step.y_mm = self.x.value(), self.y.value()
                step.use_yaw, step.stop, step.dwell_s, step.vmax_mm_s = self.use_yaw.isChecked(), self.stop.isChecked(), self.dwell.value(), self.node_vmax.value()
            self._sync_continuous_entries()
            self.refresh_waypoints(); self.redraw(); self.rebuild_timeline_after_edit()

    def activate_continuous_point(self, index):
            step = self._selected_step()
            if isinstance(step, ContinuousPathSegment) and 0 <= index < len(step.points):
                self.active_point_index = index
                self.show_continuous_point(index)
                self.redraw()

    def show_continuous_point(self, index):
            step = self._selected_step()
            if not isinstance(step, ContinuousPathSegment) or not 0 <= index < len(step.points):
                return
            point, locked = step.points[index], index == 0
            if hasattr(self, "selected_pose_label"):
                paper = self.paper_of(point)
                self.selected_pose_label.setText(
                    f"当前路径点：世界 X={point.x_mm:.1f}, Y={point.y_mm:.1f}, 航向={point.yaw_deg:.1f}°；"
                    f"图纸 X={paper.x_mm:.1f}, Y={paper.y_mm:.1f} mm")
            self.continuous_x.setValue(point.x_mm); self.continuous_y.setValue(point.y_mm); self.continuous_yaw.setValue(point.yaw_deg)
            for widget in (self.continuous_x, self.continuous_y, self.continuous_yaw, self.update_continuous_button, self.delete_continuous_button):
                widget.setEnabled(not locked)

    def update_continuous_point(self):
            step = self._selected_step()
            if not isinstance(step, ContinuousPathSegment) or self.active_point_index <= 0:
                return
            self.push_undo()
            point = step.points[self.active_point_index]
            point.x_mm, point.y_mm, point.yaw_deg = self.continuous_x.value(), self.continuous_y.value(), self.continuous_yaw.value()
            self.show_node(self.active_index); self.redraw(); self.rebuild_timeline_after_edit()

    def remove_continuous_point(self):
            step = self._selected_step()
            if not isinstance(step, ContinuousPathSegment) or self.active_point_index <= 0:
                return
            self.push_undo(); del step.points[self.active_point_index]
            self.active_point_index = max(0, self.active_point_index - 1)
            self.show_node(self.active_index); self.redraw(); self.rebuild_timeline_after_edit()

    def remove_selected_step(self):
            indices = sorted(self.selected_indices or {self.active_index}, reverse=True)
            indices = [index for index in indices if 0 <= index < len(self.plan.steps)]
            if not indices:
                return
            self._discard_bezier_draft(); self.push_undo()
            for index in indices:
                del self.plan.steps[index]
            self.selected_indices.clear()
            self.active_index, self.active_point_index = min(self.active_index, len(self.plan.steps) - 1), -1
            self._sync_continuous_entries()
            self.refresh_waypoints(); self.redraw(); self.rebuild_timeline_after_edit()

    def invalid_waypoints(self):
            invalid = []
            for index, step in enumerate(self.plan.steps):
                previous = self._step_end_pose(index)
                if isinstance(step, Waypoint):
                    target_yaw = step.yaw_deg if step.use_yaw else previous.yaw_deg
                    start, end = self.paper_of(previous), self.paper_of(step)
                    if not self.is_valid_route_segment(QPointF(start.x_mm, start.y_mm), QPointF(end.x_mm, end.y_mm), previous.yaw_deg, target_yaw, step.vmax_mm_s, step.wmax_deg_s, step.timeout_s): invalid.append(index)
                elif isinstance(step, RotateInPlace):
                    if step.name == "曲线起点转向":
                        continue
                    paper = self.paper_of(previous)
                    if not self.is_valid_rotation(QPointF(paper.x_mm, paper.y_mm), previous.yaw_deg, step.yaw_deg, step.wmax_deg_s, step.timeout_s): invalid.append(index)
                elif isinstance(step, ContinuousPathSegment) and len(step.points) < 2:
                    invalid.append(index)
                elif isinstance(step, ContinuousPathSegment):
                    for first, second in zip(step.points, step.points[1:]):
                        a, b = self.paper_of(first), self.paper_of(second)
                        if not self.is_valid_continuous_segment(QPointF(a.x_mm, a.y_mm), QPointF(b.x_mm, b.y_mm), first.yaw_deg, second.yaw_deg):
                            invalid.append(index); break
                elif isinstance(step, BezierPathSegment):
                    try:
                        self._bezier_points(previous, step)
                    except ValueError:
                        invalid.append(index)
            return invalid

    def _bezier_points(self, start, step):
            return generate_bezier_path_points(start, (step.control_1_x_mm, step.control_1_y_mm), (step.control_2_x_mm, step.control_2_y_mm), Pose(step.end_x_mm, step.end_y_mm, step.end_yaw_deg), step.yaw_mode, step.sample_spacing_mm)

    def _bezier_sweep(self, points):
            polygons = []
            for first, second in zip(points, points[1:]):
                a, b = self.paper_of(first), self.paper_of(second)
                polygons.extend(self.continuous_sweep(QPointF(a.x_mm, a.y_mm), QPointF(b.x_mm, b.y_mm), first.yaw_deg, second.yaw_deg).polygons)
            return SweepGeometry(list(points), polygons)

    def _preview_bezier_points(self, start, step):
            key = (start.x_mm, start.y_mm, start.yaw_deg, step.control_1_x_mm, step.control_1_y_mm,
                   step.control_2_x_mm, step.control_2_y_mm, step.end_x_mm, step.end_y_mm,
                   step.end_yaw_deg, step.yaw_mode, step.sample_spacing_mm)
            if key != self._bezier_preview_cache_key:
                self._bezier_preview_cache_key = key
                self._bezier_preview_points = self._bezier_points(start, step)
            return self._bezier_preview_points

    def _draw_bezier_coverage(self, points):
            """仅绘制稀疏车体轮廓，不做禁区碰撞或高精度扫掠计算。"""
            if not points:
                return
            path = QPainterPath()
            stride = max(1, math.ceil(len(points) / 24))
            display_points = [*points[::stride], points[-1]]
            for point in display_points:
                paper = self.paper_of(point)
                path.addPolygon(QPolygonF([QPointF(x, y) for x, y in car_polygon(
                    Pose(paper.x_mm, paper.y_mm,
                         point.yaw_deg + self.plan.start_heading_deg),
                    *self.vehicle_dimensions())]))
            item = self.scene.addPath(path, QPen(Qt.PenStyle.NoPen), QColor(21, 101, 192, 52))
            item.setData(0, "bezier_preview_coverage"); item.setZValue(2)
            for point in display_points:
                paper = self.paper_of(point)
                self._draw_direction_arrow(paper.x_mm, paper.y_mm, point.yaw_deg, QColor("#1565c0"), "bezier_preview_direction")

    def draw_route(self):
            previous = self._step_end_pose(0)
            invalid = set(self.invalid_waypoints())
            for index, step in enumerate(self.plan.steps):
                color = QColor("#c62828") if index in invalid else (QColor("#00897b") if isinstance(step, ContinuousPathSegment) else QColor("#d27800"))
                if isinstance(step, RotateInPlace):
                    paper = self.paper_of(previous); marker = self.scene.addEllipse(paper.x_mm - 22, paper.y_mm - 22, 44, 44, QPen(QColor("#7b1fa2"), 4), QColor("#f3e5f5")); marker.setData(0, "rotate_in_place_marker"); marker.setZValue(11); previous.yaw_deg = step.yaw_deg; continue
                if isinstance(step, Waypoint):
                    paper = self.paper_of(step); current = QPointF(paper.x_mm, paper.y_mm); before = self.paper_of(previous)
                    self.scene.addLine(before.x_mm, before.y_mm, current.x(), current.y(), QPen(color, 7))
                    self.scene.addItem(WaypointItem(index, current.x(), current.y(), index == self.active_index, index in invalid, self.move_waypoint, self.rotate_waypoint, self.set_active_from_context))
                    previous = Pose(step.x_mm, step.y_mm, step.yaw_deg if step.use_yaw else previous.yaw_deg); continue
                if isinstance(step, BezierPathSegment):
                    try:
                        points = self._bezier_points(previous, step)
                    except ValueError:
                        continue
                    for first, second in zip(points, points[1:]):
                        a, b = self.paper_of(first), self.paper_of(second); self.scene.addLine(a.x_mm, a.y_mm, b.x_mm, b.y_mm, QPen(QColor("#1565c0") if index == self.active_index else QColor("#00897b"), 6))
                    if index == self.active_index:
                        self._draw_bezier_controls(previous, step, points, False)
                    previous = Pose(points[-1].x_mm, points[-1].y_mm, points[-1].yaw_deg); continue
                marker_stride = max(1, math.ceil(len(step.points) / 32))
                for point_index, point in enumerate(step.points):
                    paper = self.paper_of(point); current = QPointF(paper.x_mm, paper.y_mm)
                    if point_index:
                        prior = self.paper_of(step.points[point_index - 1]); self.scene.addLine(prior.x_mm, prior.y_mm, current.x(), current.y(), QPen(color, 6))
                    if point_index % marker_stride and point_index != len(step.points) - 1:
                        continue
                    radius = 13 if point_index == len(step.points) - 1 else 10
                    marker = self.scene.addEllipse(current.x() - radius, current.y() - radius, radius * 2, radius * 2, QPen(color, 3), QColor("#e0f2f1")); marker.setData(0, "continuous_endpoint" if point_index == len(step.points) - 1 else "continuous_waypoint"); marker.setZValue(12)
                    self._draw_direction_arrow(current.x(), current.y(), point.yaw_deg, color, "continuous_direction")
                    if (point_index == len(step.points) - 1 and
                            step.name.startswith("自动规划")):
                        delta_x = current.x() - START_PRESETS["启停区 1"][0]
                        delta_y = current.y() - START_PRESETS["启停区 1"][1]
                        label = self.scene.addText(
                            f"航点 {index + 1}  Δ启1 X={delta_x:+.0f}  Y={delta_y:+.0f} mm",
                            QFont("Microsoft YaHei", 14))
                        label.setDefaultTextColor(QColor("#00695c"))
                        label.setPos(current.x() + 18, current.y() - 34)
                        label.setData(0, "auto_goal_label"); label.setZValue(18)
                        label.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
                        terminal = (self.plan.steps[index + 1]
                                    if index + 1 < len(self.plan.steps) and
                                    isinstance(self.plan.steps[index + 1], RotateInPlace) and
                                    self.plan.steps[index + 1].name == "导航目标朝向" else None)
                        final_yaw = terminal.yaw_deg if terminal is not None else point.yaw_deg
                        self._draw_direction_arrow(
                            current.x(), current.y(), final_yaw,
                            QColor("#ad00b5"), "navigation_goal_pose")
                if step.points:
                    last = step.points[-1]; previous = Pose(last.x_mm, last.y_mm, last.yaw_deg)

    def _draw_bezier_controls(self, start, step, points, draft):
            start_p = self.paper_of(start); end_p = self.paper_of(Pose(step.end_x_mm, step.end_y_mm, step.end_yaw_deg))
            c1_p = self.paper_of(Pose(step.control_1_x_mm, step.control_1_y_mm, 0)); c2_p = self.paper_of(Pose(step.control_2_x_mm, step.control_2_y_mm, 0))
            pen = QPen(QColor("#ef6c00"), 3, Qt.PenStyle.DashLine)
            self.scene.addLine(start_p.x_mm, start_p.y_mm, c1_p.x_mm, c1_p.y_mm, pen)
            self.scene.addLine(end_p.x_mm, end_p.y_mm, c2_p.x_mm, c2_p.y_mm, pen)
            for point in points:
                paper = self.paper_of(point)
                dot = self.scene.addEllipse(paper.x_mm - 3, paper.y_mm - 3, 6, 6, QPen(Qt.PenStyle.NoPen), QColor("#1565c0")); dot.setData(0, "bezier_sample"); dot.setZValue(14)
            if points:
                first, last = self.paper_of(points[0]), self.paper_of(points[-1])
                self._draw_direction_arrow(first.x_mm, first.y_mm, points[0].yaw_deg, QColor("#1565c0"), "bezier_start_direction")
                self._draw_direction_arrow(last.x_mm, last.y_mm, points[-1].yaw_deg, QColor("#1565c0"), "bezier_end_direction")
            for control_index, control in enumerate((c1_p, c2_p)):
                handle = DraggableEllipseItem((-10, -10, 20, 20), lambda paper, i=control_index: self.move_bezier_handle(i, paper, draft))
                handle.setPos(control.x_mm, control.y_mm); handle.setBrush(QColor("#ff9800")); handle.setPen(QPen(QColor("#e65100"), 2)); handle.setZValue(20); self.scene.addItem(handle)

    def move_bezier_handle(self, control_index, paper, draft):
            target = paper_to_world(paper.x(), paper.y(), self.plan.start_paper_x_mm, self.plan.start_paper_y_mm, self.plan.start_heading_deg)
            if draft:
                step = self.bezier_draft
            else:
                step = self._selected_step()
                self.push_undo()
            if not isinstance(step, BezierPathSegment):
                return
            if QApplication.keyboardModifiers() & Qt.KeyboardModifier.ShiftModifier:
                anchor = self._step_end_pose(len(self.plan.steps)) if control_index == 0 else Pose(step.end_x_mm, step.end_y_mm, step.end_yaw_deg)
                angle = math.atan2(target.y_mm - anchor.y_mm, target.x_mm - anchor.x_mm)
                distance = math.hypot(target.x_mm - anchor.x_mm, target.y_mm - anchor.y_mm)
                angle = round(angle / (math.pi / 4)) * math.pi / 4
                target = Pose(anchor.x_mm + distance * math.cos(angle), anchor.y_mm + distance * math.sin(angle))
            if control_index == 0:
                step.control_1_x_mm, step.control_1_y_mm = target.x_mm, target.y_mm
            else:
                step.control_2_x_mm, step.control_2_y_mm = target.x_mm, target.y_mm
            self.redraw()

    def move_waypoint(self, index, before, after, shift=False):
            if not isinstance(index, int) or index != self.active_index or before == after:
                return
            step = self._selected_step()
            if not isinstance(step, Waypoint):
                return
            self.push_undo(); pose = paper_to_world(after.x(), after.y(), self.plan.start_paper_x_mm, self.plan.start_paper_y_mm, self.plan.start_heading_deg)
            step.x_mm, step.y_mm = pose.x_mm, pose.y_mm; self._sync_continuous_entries()
            self.refresh_waypoints(); self.redraw(); self.rebuild_timeline_after_edit()

    def rotate_waypoint(self, index, item):
            if index != self.active_index or not isinstance(self._selected_step(), Waypoint):
                return
            self.push_undo(); step = self._selected_step(); delta = item.pos()
            step.yaw_deg = paper_heading_to_world_yaw(
                self.plan.start_heading_deg,
                paper_vector_to_heading(delta.x(), delta.y()))
            step.use_yaw = True
            self._sync_continuous_entries(); self.show_node(index); self.redraw(); self.rebuild_timeline_after_edit()

    def set_active_from_context(self, index):
            self.activate_node(index)

    def rotate_car_clockwise(self):
            step = self._selected_step()
            if not isinstance(step, (Waypoint, RotateInPlace)):
                return
            self.push_undo(); step.yaw_deg = ((step.yaw_deg - 90 + 180) % 360) - 180
            if isinstance(step, Waypoint): step.use_yaw = True
            self._sync_continuous_entries(); self.show_node(self.active_index); self.redraw(); self.rebuild_timeline_after_edit()

    def open_code_generator(self):
            if self._auto_paths_stale:
                self.status.setText("代价地图参数已改变，请先重新规划全部自动航点。"); return
            if self.calibration_pending or not self.plan.steps or self.invalid_waypoints():
                self.status.setText("Complete calibration and correct the flow before code generation."); return
            try:
                validate_plan_for_blocking_codegen(self.plan)
            except CodeGenerationError as error:
                self.status.setText(str(error)); return
            self.codegen_dialog = CodeGenerationDialog(self.plan, self); self.codegen_dialog.show()

    def update_calibration_ui(self):
            self.calibration_bar.setVisible(self.calibration_pending)
            heading_ready = self.calibration_pending and self.calibration_stage == "heading"
            self.confirm_start_button.setVisible(heading_ready); self.confirm_start_button.setEnabled(heading_ready)
            enabled = not self.calibration_pending
            for widget in (self.add_button, self.mark_pose_button, self.add_goto_button, self.add_continuous_button, self.add_bezier_button, self.append_rotation_button, self.move_up_button, self.move_down_button, self.delete_button, self.obstacle_button, self.auto_plan_button, self.save_button, self.save_as_button, self.play_button):
                widget.setEnabled(enabled)
            self.codegen_button.setEnabled(enabled and bool(self.plan.steps)
                                           and not self._auto_paths_stale)
            self.confirm_bezier_button.setEnabled(enabled and self.bezier_draft is not None)
            self.cancel_bezier_button.setEnabled(self.bezier_draft is not None)

    def undo(self):
            if not self.undo_stack:
                return
            self._discard_bezier_draft(); self.pending_action = None; self.set_mode("select")
            self.redo_stack.append(copy.deepcopy(self.plan)); self.plan = self.undo_stack.pop()
            self._load_costmap_controls(); self._auto_paths_stale = False
            self.active_index = min(self.active_index, len(self.plan.steps) - 1); self.active_point_index = -1
            self._sync_continuous_entries(); self._invalidate_timeline(); self.refresh_waypoints(); self.redraw(); self.rebuild_timeline_after_edit()

    def redo(self):
            if not self.redo_stack:
                return
            self._discard_bezier_draft(); self.pending_action = None; self.set_mode("select")
            self.undo_stack.append(copy.deepcopy(self.plan)); self.plan = self.redo_stack.pop()
            self._load_costmap_controls(); self._auto_paths_stale = False
            self.active_index = min(self.active_index, len(self.plan.steps) - 1); self.active_point_index = -1
            self._sync_continuous_entries(); self._invalidate_timeline(); self.refresh_waypoints(); self.redraw(); self.rebuild_timeline_after_edit()

    def draw_preview(self):
            if self.preview_paper is None or self.preview_yaw_deg is None:
                return
            if self.pending_action == "bezier" or (self.pending_action == "bezier_edit" and self.bezier_draft is not None):
                draft = self.bezier_draft
                if draft is None:
                    start = self._step_end_pose(len(self.plan.steps))
                    # 选择终点阶段仅给出轻量级直线提示；控制点尚未确定，不采样也不做区域校验。
                    start_p = self.paper_of(start)
                    self.scene.addLine(start_p.x_mm, start_p.y_mm, self.preview_paper.x(), self.preview_paper.y(), QPen(QColor("#1565c0"), 4, Qt.PenStyle.DashLine))
                    car = CarOutlineItem(self.rotate_preview_clockwise,
                                         *self.vehicle_dimensions())
                    car.setPos(self.preview_paper); car.setRotation(
                        qgraphics_rotation_deg(self.plan.start_heading_deg, self.preview_yaw_deg))
                    car.setPen(QPen(QColor("#1565c0"), 4)); car.setBrush(QColor(21, 101, 192, 45)); car.setZValue(19)
                    car.setData(0, "bezier_endpoint_preview_car"); self.scene.addItem(car)
                    self._draw_direction_arrow(self.preview_paper.x(), self.preview_paper.y(), self.preview_yaw_deg, QColor("#1565c0"), "bezier_endpoint_preview_direction")
                    self._set_path_check("曲线预览", "请选择终点")
                    return
                start = self._step_end_pose(len(self.plan.steps))
                try:
                    preview_yaw = self.bezier_draft_start_yaw if draft.yaw_mode == "interpolate" else self._bezier_tangent_yaw(start, draft)
                    points = self._preview_bezier_points(Pose(start.x_mm, start.y_mm, preview_yaw), draft)
                except ValueError:
                    self._set_path_check("曲线预览", "无效曲线"); return
                color = QColor("#1565c0")
                self._draw_bezier_coverage(points)
                for first, second in zip(points, points[1:]):
                    a, b = self.paper_of(first), self.paper_of(second); self.scene.addLine(a.x_mm, a.y_mm, b.x_mm, b.y_mm, QPen(color, 6))
                self._draw_bezier_controls(start, draft, points, True)
                self._set_path_check("曲线预览", "仅供观察，不进行区域合法性检查")
                self.confirm_bezier_button.setEnabled(not self.calibration_pending and self.bezier_draft is not None)
                return
            # 鼠标悬停只预览目标姿态。车体扫掠仍在保存、播放和下发前统一校验，
            # 避免每次 mouseMove 都生成大量多边形并把地图覆盖成密集曲线。
            color = QColor("#1565c0")
            item = CarOutlineItem(self.rotate_preview_clockwise, *self.vehicle_dimensions()); item.setPos(self.preview_paper); item.setRotation(qgraphics_rotation_deg(self.plan.start_heading_deg, self.preview_yaw_deg)); item.setPen(QPen(color, 4)); item.setBrush(QColor(color.red(), color.green(), color.blue(), 42)); item.setZValue(17); item.setData(0, "preview_car"); self.scene.addItem(item)
            if self.mode == "mark_pose":
                self._draw_navigation_goal_guide(color)
            else:
                self._draw_direction_arrow(self.preview_paper.x(), self.preview_paper.y(), self.preview_yaw_deg, color, "preview_direction")
            self._set_path_check("预览", "仅显示目标姿态；保存、播放或执行前统一检查路径")

    def _draw_navigation_goal_guide(self, color: QColor) -> None:
            """RViz 风格目标位姿：当前位置参考线、拖动箭头和角度差。"""
            if self._rviz_pose_anchor is None or self.preview_yaw_deg is None:
                return
            anchor = self._rviz_pose_anchor
            current_yaw = self._step_end_pose(len(self.plan.steps)).yaw_deg

            guide_pen = QPen(QColor(0, 150, 136, 175), 3,
                             Qt.PenStyle.DashLine)
            ring_radius = 150.0
            ring = self.scene.addEllipse(anchor.x() - ring_radius,
                                         anchor.y() - ring_radius,
                                         ring_radius * 2.0,
                                         ring_radius * 2.0, guide_pen)
            ring.setData(0, "nav_goal_heading_ring"); ring.setZValue(23)
            for dx, dy in ((ring_radius, 0.0), (0.0, ring_radius)):
                cross = self.scene.addLine(anchor.x() - dx, anchor.y() - dy,
                                           anchor.x() + dx, anchor.y() + dy,
                                           QPen(QColor(0, 150, 136, 100), 2,
                                                Qt.PenStyle.DotLine))
                cross.setData(0, "nav_goal_heading_cross"); cross.setZValue(23)
            center = self.scene.addEllipse(anchor.x() - 10, anchor.y() - 10,
                                           20, 20, QPen(QColor("#00695c"), 3),
                                           QColor("#80cbc4"))
            center.setData(0, "nav_goal_center"); center.setZValue(27)
            if self._rviz_drag_point is not None:
                radial = self.scene.addLine(anchor.x(), anchor.y(),
                                            self._rviz_drag_point.x(),
                                            self._rviz_drag_point.y(), guide_pen)
                radial.setData(0, "nav_goal_heading_guide"); radial.setZValue(24)

            def endpoint(yaw_deg: float, length: float) -> QPointF:
                paper_heading = world_yaw_to_paper_heading(
                    self.plan.start_heading_deg, yaw_deg)
                radians = math.radians(paper_heading)
                return QPointF(anchor.x() + length * math.cos(radians),
                               anchor.y() - length * math.sin(radians))

            reference_end = endpoint(current_yaw, 190.0)
            reference = self.scene.addLine(
                anchor.x(), anchor.y(), reference_end.x(), reference_end.y(),
                QPen(QColor("#757575"), 3, Qt.PenStyle.DashLine))
            reference.setData(0, "nav_goal_current_heading"); reference.setZValue(24)

            arrow_length = 150.0
            head = 55.0; shaft = 13.0
            arrow = QGraphicsPolygonItem(QPolygonF([
                QPointF(0, -shaft), QPointF(arrow_length - head, -shaft),
                QPointF(arrow_length - head, -34), QPointF(arrow_length, 0),
                QPointF(arrow_length - head, 34), QPointF(arrow_length - head, shaft),
                QPointF(0, shaft),
            ]))
            arrow.setPos(anchor); arrow.setRotation(
                qgraphics_rotation_deg(self.plan.start_heading_deg,
                                       self.preview_yaw_deg))
            arrow.setPen(QPen(color, 4)); arrow.setBrush(QColor(
                color.red(), color.green(), color.blue(), 150))
            arrow.setData(0, "nav_goal_arrow"); arrow.setZValue(26)
            arrow.setAcceptedMouseButtons(Qt.MouseButton.NoButton); self.scene.addItem(arrow)

            delta = wrap_deg(self.preview_yaw_deg - current_yaw)
            label = self.scene.addText(
                f"目标 {self.preview_yaw_deg:.1f}°  Δ航向 {delta:+.1f}°")
            label.setDefaultTextColor(color); label.setFont(QFont("Microsoft YaHei", 17))
            label.setPos(anchor.x() + 20, anchor.y() + 54)
            label.setData(0, "nav_goal_angle_label"); label.setZValue(27)

    def begin_goto_add(self):
            self._discard_bezier_draft(); self.pending_action = "goto"
            self.set_mode("add")
            self.status.setText("新增点到点：在地图上点击目标位置。")

    def begin_bezier_add(self):
            if self.calibration_pending:
                return
            self._discard_bezier_draft()
            self.bezier_draft_start_yaw = self._step_end_pose(len(self.plan.steps)).yaw_deg
            self.pending_action = "bezier"
            self.confirm_bezier_button.setVisible(True); self.cancel_bezier_button.setVisible(True)
            self.set_mode("add")
            self.status.setText("新增曲线路径：点击设置曲线终点。")

    def confirm_bezier_draft(self):
            if self.bezier_draft is None or self.calibration_pending:
                return
            start = self._step_end_pose(len(self.plan.steps))
            start_yaw = self.bezier_draft_start_yaw if self.bezier_draft.yaw_mode == "interpolate" else self._bezier_tangent_yaw(start, self.bezier_draft)
            try:
                self._bezier_points(Pose(start.x_mm, start.y_mm, start_yaw), self.bezier_draft)
            except ValueError:
                self.status.setText("曲线参数无效，无法保存。")
                return
            self.push_undo(); self._insert_bezier_start_rotation(len(self.plan.steps), start_yaw); self.bezier_draft.name = f"曲线 {len(self.plan.steps) + 1}"; self.plan.steps.append(self.bezier_draft)
            self.active_index, self.active_point_index = len(self.plan.steps) - 1, -1
            self.bezier_draft = None; self.pending_action = None; self._bezier_preview_cache_key = None; self._bezier_preview_points = None; self.set_mode("select"); self.clear_preview(False)
            self._sync_continuous_entries(); self.refresh_waypoints(); self.redraw(); self.rebuild_timeline_after_edit()

    def cancel_bezier_draft(self):
            if self.bezier_draft is None and self.pending_action not in ("bezier", "bezier_edit"):
                return
            self._discard_bezier_draft(); self.set_mode("select"); self.clear_preview(False); self.update_calibration_ui(); self.status.setText("已取消曲线路径创建。")

    def _draw_direction_arrow(self, x, y, yaw, color, marker):
            arrow = QGraphicsPolygonItem(QPolygonF([QPointF(-18, -14), QPointF(20, -14), QPointF(20, -28), QPointF(52, 0), QPointF(20, 28), QPointF(20, 14), QPointF(-18, 14)]))
            arrow.setPos(x, y); arrow.setRotation(qgraphics_rotation_deg(self.plan.start_heading_deg, yaw)); arrow.setBrush(color); arrow.setPen(QPen(QColor("#0d47a1"), 2)); arrow.setData(0, marker); arrow.setZValue(18); self.scene.addItem(arrow)

    def _set_path_check(self, subject, result):
            self.path_check.setText(f"路径检查：{subject} - {result}")

    def _validation_reason(self):
            if self.calibration_pending: return "未完成起点标定"
            if not self.plan.steps: return "流程为空"
            if not self.is_valid_start_pose(): return "起点车体越界或碰撞"
            for index, step in enumerate(self.plan.steps):
                if isinstance(step, ContinuousPathSegment) and len(step.points) < 2: return f"步骤 {index + 1} 连续段至少需要入口点和最终停车点"
            invalid = self.invalid_waypoints()
            return f"步骤 {invalid[0] + 1} 越界或碰撞平台" if invalid else "可行"

    def rebuild_timeline_after_edit(self):
            self._invalidate_timeline(); self._sync_continuous_entries()
            reason = self._validation_reason(); self._set_path_check("流程", reason)
            if reason != "可行": return
            self.timeline = build_plan_timeline(copy.deepcopy(self.plan))
            self.progress.blockSignals(True); self.progress.setRange(0, len(self.timeline)); self.progress.setValue(0); self.progress.blockSignals(False)
            self.progress.setEnabled(bool(self.timeline))
            self._update_progress_ui()

    def play(self):
            self._sync_continuous_entries()
            reason = self._validation_reason()
            self._set_path_check("仿真", reason)
            if reason != "可行":
                self.status.setText(f"无法播放：{reason}")
                return
            if not self.timeline: self.rebuild_timeline_after_edit()
            if self.timeline:
                if self.timeline_position >= len(self.timeline): self.progress.setValue(0)
                self.timer.start(); self.status.setText("正在播放仿真。")

    def draw_car(self, pose):
            p = pose or self._step_end_pose(self.active_index + 1)
            x, y = world_to_paper(p, self.plan.start_paper_x_mm, self.plan.start_paper_y_mm, self.plan.start_heading_deg)
            invalid = self.active_index in self.invalid_waypoints(); color = QColor("#c62828") if invalid else QColor("#455a64")
            car = CarOutlineItem(self.rotate_car_clockwise, *self.vehicle_dimensions()); car.setPos(x, y); car.setRotation(qgraphics_rotation_deg(self.plan.start_heading_deg, p.yaw_deg)); car.setPen(QPen(color, 5)); car.setBrush(QColor(120, 144, 156, 105)); car.setZValue(15); self.scene.addItem(car)
            self._draw_direction_arrow(x, y, p.yaw_deg, QColor("#1565c0"), "car_direction")

    def _clear_runtime_trace(self) -> None:
            self._execution_trace.clear()
            self._runtime_trace_path = QPainterPath()
            self._runtime_trace_path_point_count = 0
            if self._runtime_trace_item is not None:
                self._runtime_trace_item.setPath(self._runtime_trace_path)

    def _append_runtime_trace_points(self, points: list[Pose]) -> None:
            if not points:
                return
            if self._runtime_trace_path_point_count != len(self._execution_trace) - len(points):
                self._rebuild_runtime_trace_path()
                return
            for pose in points:
                point = world_to_paper(pose, self.plan.start_paper_x_mm,
                                       self.plan.start_paper_y_mm, self.plan.start_heading_deg)
                if self._runtime_trace_path_point_count == 0:
                    self._runtime_trace_path.moveTo(*point)
                else:
                    self._runtime_trace_path.lineTo(*point)
                self._runtime_trace_path_point_count += 1

    def _rebuild_runtime_trace_path(self) -> None:
            self._runtime_trace_path = QPainterPath()
            self._runtime_trace_path_point_count = 0
            self._append_runtime_trace_points(list(self._execution_trace))

    def _ensure_runtime_overlay_items(self) -> None:
            if self._runtime_car_item is not None and self._runtime_car_item.scene() is self.scene:
                return
            self._runtime_car_item = CarOutlineItem(lambda: None,
                                                    *self.vehicle_dimensions())
            self._runtime_car_item.setPen(QPen(QColor("#e53935"), 5))
            self._runtime_car_item.setBrush(QColor(229, 57, 53, 65))
            self._runtime_car_item.setZValue(40); self._runtime_car_item.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
            self._runtime_car_item.setData(0, "runtime_car"); self.scene.addItem(self._runtime_car_item)
            arrow_shape = QPolygonF([QPointF(-18, -14), QPointF(20, -14), QPointF(20, -28),
                                     QPointF(52, 0), QPointF(20, 28), QPointF(20, 14), QPointF(-18, 14)])
            self._runtime_direction_item = QGraphicsPolygonItem(arrow_shape)
            self._runtime_direction_item.setBrush(QColor("#e53935")); self._runtime_direction_item.setPen(QPen(QColor("#0d47a1"), 2))
            self._runtime_direction_item.setZValue(41); self._runtime_direction_item.setData(0, "runtime_direction"); self.scene.addItem(self._runtime_direction_item)
            self._runtime_target_item = self.scene.addEllipse(-38, -38, 76, 76, QPen(QColor("#2e7d32"), 4), QColor(46, 125, 50, 35))
            self._runtime_target_item.setZValue(39); self._runtime_target_item.setData(0, "runtime_target")
            self._runtime_target_direction_item = QGraphicsPolygonItem(arrow_shape)
            self._runtime_target_direction_item.setBrush(QColor("#2e7d32")); self._runtime_target_direction_item.setPen(QPen(QColor("#2e7d32"), 2))
            self._runtime_target_direction_item.setZValue(40); self._runtime_target_direction_item.setData(0, "runtime_target_direction"); self.scene.addItem(self._runtime_target_direction_item)
            self._runtime_trace_item = QGraphicsPathItem()
            self._runtime_trace_item.setPen(QPen(QColor("#8e24aa"), 5, Qt.PenStyle.SolidLine)); self._runtime_trace_item.setZValue(38); self._runtime_trace_item.setData(0, "runtime_trace"); self.scene.addItem(self._runtime_trace_item)
            self._runtime_projection_item = self.scene.addEllipse(-16, -16, 32, 32, QPen(QColor("#ff9800"), 3), QColor(255, 152, 0, 100))
            self._runtime_projection_item.setZValue(40); self._runtime_projection_item.setData(0, "runtime_projection")
            self._runtime_lookahead_item = self.scene.addEllipse(-14, -14, 28, 28, QPen(QColor("#00acc1"), 3), QColor(0, 172, 193, 100))
            self._runtime_lookahead_item.setZValue(40); self._runtime_lookahead_item.setData(0, "runtime_lookahead")

    def draw_runtime_overlay(self):
            """创建并更新持久化的运行时图元。"""
            self._refresh_runtime_overlay()

    def _refresh_runtime_overlay(self) -> None:
            if not hasattr(self, "scene"):
                return
            self._ensure_runtime_overlay_items()
            for item in (self._runtime_car_item, self._runtime_direction_item,
                         self._runtime_target_item, self._runtime_target_direction_item,
                         self._runtime_trace_item, self._runtime_projection_item,
                         self._runtime_lookahead_item):
                item.setVisible(False)
            if self._runtime_trace_path_point_count != len(self._execution_trace):
                self._rebuild_runtime_trace_path()
            if len(self._execution_trace) >= 2:
                self._runtime_trace_item.setPath(self._runtime_trace_path)
                self._runtime_trace_item.setVisible(True)
            if self._execution_target is not None:
                x, y = world_to_paper(self._execution_target, self.plan.start_paper_x_mm,
                                      self.plan.start_paper_y_mm, self.plan.start_heading_deg)
                self._runtime_target_item.setPos(x, y); self._runtime_target_item.setVisible(True)
                self._runtime_target_direction_item.setPos(x, y)
                self._runtime_target_direction_item.setRotation(qgraphics_rotation_deg(self.plan.start_heading_deg, self._execution_target.yaw_deg))
                self._runtime_target_direction_item.setVisible(True)
            if self._path_runtime is not None:
                projection = Pose(self._path_runtime.projection_x_mm,
                                  self._path_runtime.projection_y_mm)
                lookahead = Pose(self._path_runtime.lookahead_x_mm,
                                 self._path_runtime.lookahead_y_mm)
                px, py = world_to_paper(projection, self.plan.start_paper_x_mm,
                                        self.plan.start_paper_y_mm, self.plan.start_heading_deg)
                lx, ly = world_to_paper(lookahead, self.plan.start_paper_x_mm,
                                        self.plan.start_paper_y_mm, self.plan.start_heading_deg)
                self._runtime_projection_item.setPos(px, py); self._runtime_projection_item.setVisible(True)
                self._runtime_lookahead_item.setPos(lx, ly); self._runtime_lookahead_item.setVisible(True)
            if self._runtime_pose is not None:
                x, y = world_to_paper(self._runtime_pose, self.plan.start_paper_x_mm,
                                      self.plan.start_paper_y_mm, self.plan.start_heading_deg)
                self._runtime_car_item.setPos(x, y)
                self._runtime_car_item.setRotation(qgraphics_rotation_deg(self.plan.start_heading_deg, self._runtime_pose.yaw_deg))
                self._runtime_car_item.setVisible(True)
                self._runtime_direction_item.setPos(x, y)
                self._runtime_direction_item.setRotation(qgraphics_rotation_deg(self.plan.start_heading_deg, self._runtime_pose.yaw_deg))
                self._runtime_direction_item.setVisible(True)
            self._refresh_execution_status()

    def clear_preview(self, redraw=True):
            self.preview_paper = None; self.preview_yaw_deg = None; self.preview_anchor_index = None; self.preview_anchor_signature = None; self.preview_shift = False
            if redraw: self.redraw()

    def _discard_bezier_draft(self):
            self.bezier_draft = None; self.bezier_draft_start_yaw = 0.0
            if self.pending_action in ("bezier", "bezier_edit"):
                self.pending_action = None
            self._bezier_preview_cache_key = None; self._bezier_preview_points = None
            if hasattr(self, "bezier_panel"):
                self.bezier_panel.setVisible(False)
            if hasattr(self, "confirm_bezier_button"):
                self.confirm_bezier_button.setVisible(False)
                self.cancel_bezier_button.setVisible(False)

    def _current_preview_anchor(self):
            step = self._selected_step()
            if isinstance(step, ContinuousPathSegment) and step.points and self.pending_action != "goto":
                point = step.points[-1]; paper = self.paper_of(point)
                return QPointF(paper.x_mm, paper.y_mm), point.yaw_deg, (self.active_index, len(step.points) - 1, point.x_mm, point.y_mm, point.yaw_deg)
            anchor, yaw = self._step_anchor(len(self.plan.steps))
            return anchor, yaw, (len(self.plan.steps), anchor.x(), anchor.y(), yaw)

    def update_preview(self, x, y, shift=False):
            if hasattr(self, "cursor_position_label"):
                if math.isnan(x) or not (0 <= x <= FIELD_SIZE_MM and 0 <= y <= FIELD_SIZE_MM):
                    self.cursor_position_label.setText("地图指针：--")
                else:
                    world = paper_to_world(x, y, self.plan.start_paper_x_mm,
                                           self.plan.start_paper_y_mm,
                                           self.plan.start_heading_deg)
                    self.cursor_position_label.setText(
                        f"地图指针：图纸 X={x:.1f}, Y={y:.1f}；世界 X={world.x_mm:.1f}, Y={world.y_mm:.1f} mm")
            if self.mode == "mark_pose" and self._rviz_pose_anchor is not None:
                if math.isnan(x):
                    return
                dx, dy = x - self._rviz_pose_anchor.x(), y - self._rviz_pose_anchor.y()
                if math.hypot(dx, dy) >= 5.0:
                    distance = math.hypot(dx, dy)
                    raw_heading = paper_vector_to_heading(dx, dy)
                    paper_heading = snap_paper_heading_to_right_angle(raw_heading)
                    radians = math.radians(paper_heading)
                    self._rviz_drag_point = QPointF(
                        self._rviz_pose_anchor.x() + distance * math.cos(radians),
                        self._rviz_pose_anchor.y() - distance * math.sin(radians))
                    self.preview_yaw_deg = paper_heading_to_world_yaw(
                        self.plan.start_heading_deg, paper_heading)
                else:
                    self._rviz_drag_point = QPointF(x, y)
                self.preview_paper = QPointF(self._rviz_pose_anchor)
                if not self._nav_preview_timer.isActive():
                    self._nav_preview_timer.start()
                return
            if self.mode not in ("add", "calibrate"):
                return
            if self.mode == "calibrate" and self.calibration_pending and self.calibration_stage == "position":
                self._start_preview_paper = (None if math.isnan(x) or not
                                             (0 <= x <= FIELD_SIZE_MM and 0 <= y <= FIELD_SIZE_MM)
                                             else QPointF(x, y))
                self.redraw()
                return
            if math.isnan(x) or self.mode != "add" or self.calibration_pending or not (0 <= x <= FIELD_SIZE_MM and 0 <= y <= FIELD_SIZE_MM):
                self.clear_preview(); return
            anchor, anchor_yaw, signature = self._current_preview_anchor()
            if signature != self.preview_anchor_signature:
                self.preview_yaw_deg = anchor_yaw
                self.preview_anchor_signature = signature
            target = QPointF(x, y)
            self.preview_paper = snap_to_45(anchor, target) if shift else target
            self.preview_anchor_index, self.preview_shift = self.active_index, shift
            self.redraw()

    def confirm_marked_pose(self, x: float, y: float) -> None:
            if self._rviz_pose_anchor is None or self.preview_yaw_deg is None:
                return
            # Release point only determines direction; the pressed point is the waypoint.
            self.update_preview(x, y)
            self._nav_preview_timer.stop()
            goal = QPointF(self._rviz_pose_anchor)
            goal_yaw = self.preview_yaw_deg
            self._rviz_pose_anchor = None
            self._rviz_drag_point = None
            self.clear_preview(False)
            self._pending_navigation_goal_paper = QPointF(goal)
            self._pending_navigation_goal_yaw = goal_yaw
            self._set_goal_controls(goal.x(), goal.y(), goal_yaw)
            self.selected_pose_label.setText(
                f"待生成目标：图纸 X={goal.x():.1f}, Y={goal.y():.1f} mm，航向={goal_yaw:.1f}°")
            self.generate_segment_button.setText("生成/更新本小段路径")
            self.status.setText("目标位姿已确定。请调整本小段参数，然后点击黄色“生成本小段路径”。")
            self.redraw()

    def _selected_auto_path_index(self) -> int | None:
            index = self.active_index
            if (0 <= index < len(self.plan.steps) and
                    isinstance(self.plan.steps[index], RotateInPlace) and
                    self.plan.steps[index].name == "导航目标朝向"):
                index -= 1
            if (0 <= index < len(self.plan.steps) and
                    isinstance(self.plan.steps[index], ContinuousPathSegment) and
                    self.plan.steps[index].name.startswith("自动规划")):
                return index
            return None

    def generate_navigation_segment(self) -> None:
            """Generate a new target segment, or replace only the selected auto segment."""
            replace_index: int | None = None
            if self._pending_navigation_goal_paper is not None:
                goal = QPointF(self.auto_goal_x.value(), self.auto_goal_y.value())
                goal_yaw = self.auto_goal_yaw.value()
            else:
                replace_index = self._selected_auto_path_index()
                if replace_index is None:
                    self.status.setText("请先用“设置导航目标”选择位置和方向，或选中已有导航目标。")
                    return
                path = self.plan.steps[replace_index]
                if not isinstance(path, ContinuousPathSegment) or not path.points:
                    return
                goal = QPointF(self.auto_goal_x.value(), self.auto_goal_y.value())
                goal_yaw = self.auto_goal_yaw.value()

            strategy = str(self.navigation_strategy.currentData())
            labels = {
                "auto": "稳定自动", "fixed": "保持航向", "tangent": "切线航向",
                "interpolate": "连续转向", "terminal": "末端转向",
            }
            if strategy in ("auto", "terminal"):
                yaw_mode, terminal_rotation = "fixed", True
            elif strategy == "fixed":
                yaw_mode, terminal_rotation = "fixed", False
                goal_yaw = self._step_end_pose(len(self.plan.steps)).yaw_deg
            elif strategy == "tangent":
                yaw_mode, terminal_rotation = "tangent", True
            else:
                yaw_mode, terminal_rotation = "interpolate", False
            if self.create_auto_path(
                    goal, goal_yaw_deg=goal_yaw, yaw_mode=yaw_mode,
                    terminal_rotation=terminal_rotation,
                    strategy_name=labels[strategy], replace_index=replace_index):
                self._pending_navigation_goal_paper = None
                self._pending_navigation_goal_yaw = None

    def _flush_navigation_preview(self) -> None:
            if self.mode == "mark_pose" and self._rviz_pose_anchor is not None:
                self.redraw()

    def rotate_preview_clockwise(self):
            if self.preview_yaw_deg is not None:
                self.preview_yaw_deg = ((self.preview_yaw_deg - 90 + 180) % 360) - 180
                self._set_path_check("预览", f"目标朝向 {self.preview_yaw_deg:.0f} deg")
                self.redraw(); return
            step = self._selected_step()
            if not isinstance(step, RotateInPlace):
                return
            self.push_undo(); step.yaw_deg = ((step.yaw_deg - 90 + 180) % 360) - 180
            self._sync_continuous_entries(); self.show_node(self.active_index); self.refresh_waypoints(); self.redraw(); self.rebuild_timeline_after_edit()

    def confirm_preview(self, x, y, shift=False):
            self.update_preview(x, y, shift)
            if self.preview_paper is None or self.preview_yaw_deg is None:
                return
            if self.pending_action == "bezier":
                start = self._step_end_pose(len(self.plan.steps))
                end = paper_to_world(self.preview_paper.x(), self.preview_paper.y(), self.plan.start_paper_x_mm, self.plan.start_paper_y_mm, self.plan.start_heading_deg)
                dx, dy = end.x_mm - start.x_mm, end.y_mm - start.y_mm
                self.bezier_draft = BezierPathSegment(start.x_mm + dx / 3, start.y_mm + dy / 3, start.x_mm + 2 * dx / 3, start.y_mm + 2 * dy / 3, end.x_mm, end.y_mm, self.preview_yaw_deg)
                self.bezier_draft_start_yaw = start.yaw_deg
                self.pending_action = "bezier_edit"; self.mode = "select"; self.view.mode = "select"; self._show_bezier_editor(self.bezier_draft, start); self.redraw(); self.update_calibration_ui()
                self.status.setText("拖动橙色手柄调整曲线，点击确认曲线保存。")
                return
            pose = paper_to_world(self.preview_paper.x(), self.preview_paper.y(), self.plan.start_paper_x_mm, self.plan.start_paper_y_mm, self.plan.start_heading_deg)
            yaw = self.preview_yaw_deg
            self.push_undo()
            step = self._selected_step()
            if isinstance(step, ContinuousPathSegment) and self.pending_action != "goto":
                step.points.append(PathPosePoint(pose.x_mm, pose.y_mm, yaw, f"路径点 {len(step.points)}"))
                self.active_point_index = len(step.points) - 1
            else:
                self.plan.steps.append(Waypoint(pose.x_mm, pose.y_mm, yaw, True, name=f"点到点 {len(self.plan.steps) + 1}"))
                self.active_index, self.active_point_index = len(self.plan.steps) - 1, -1
            self.pending_action = None
            self.clear_preview(False); self._sync_continuous_entries(); self.refresh_waypoints(); self.redraw(); self.rebuild_timeline_after_edit()


class PlannerWindow(QMainWindow):
    """独立启动器，复用可嵌入的 ``MapEditorWidget``。"""

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("LittleCar2 比赛地图路径规划")
        self.resize(1420, 860)
        self.setMinimumSize(1024, 768)
        self.editor = MapEditorWidget(self)
        self.setCentralWidget(self.editor)

    # QMainWindow 自带 x()/y()，这里显式保留旧编辑器表单属性的访问语义。
    @property
    def x(self) -> NumericSpinBox:
        return self.editor.x

    @property
    def y(self) -> NumericSpinBox:
        return self.editor.y

    def __getattr__(self, name):  # type: ignore[no-untyped-def]
        """兼容旧版 ``PlannerWindow`` 对编辑器成员的直接访问。"""
        editor = self.__dict__.get("editor")
        if editor is not None:
            return getattr(editor, name)
        raise AttributeError(name)

    def __setattr__(self, name, value):  # type: ignore[no-untyped-def]
        editor = self.__dict__.get("editor")
        if editor is not None and hasattr(editor, name):
            setattr(editor, name, value)
            return
        super().__setattr__(name, value)


def main() -> int:
    """Launch the standalone map planner window."""

    app = QApplication(sys.argv)
    app.setStyleSheet("QWidget{font-family:'Microsoft YaHei';font-size:13px;} QPushButton{min-height:28px;} QScrollArea{border:0;}")
    window = PlannerWindow()
    window.show()
    return app.exec()

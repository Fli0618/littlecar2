"""比赛地图直线 GOTO Pose 编辑器。"""

from __future__ import annotations

import copy
import math
import sys

from PySide6.QtCore import QPoint, QPointF, QRect, QRectF, QTimer, Qt, Signal
from PySide6.QtGui import QColor, QFont, QKeySequence, QPainter, QPainterPath, QPen, QPolygonF, QShortcut
from PySide6.QtWidgets import (QApplication, QButtonGroup, QCheckBox, QDoubleSpinBox, QFormLayout,
    QGraphicsEllipseItem, QGraphicsItem, QGraphicsLineItem, QGraphicsPolygonItem, QGraphicsRectItem,
    QGraphicsScene, QGraphicsTextItem, QGraphicsView, QHBoxLayout, QInputDialog, QLabel, QListWidget,
    QMainWindow, QMessageBox, QPushButton, QScrollArea, QSlider, QSplitter, QVBoxLayout, QWidget, QRubberBand, QComboBox)

from .geometry import paper_to_world, world_to_paper
from .codegen_c import CodeGenerationError, validate_plan_for_blocking_codegen
from .codegen_dialog import CodeGenerationDialog
from .models import CAR_SIZE_MM, FIELD_SIZE_MM, ContinuousPathSegment, Obstacle, PathPosePoint, Plan, Pose, RotateInPlace, Waypoint
from .sim import SimulationFrame, build_plan_timeline
from .sweep import SweepGeometry, build_continuous_segment_sweep, build_goto_sweep, build_rotation_sweep
from .storage import list_plans, load_plan, rename_plan, save_plan

PLATFORMS = ((550, 550), (1400, 550), (550, 1400), (1400, 1400))
MATERIAL_SLOTS = ((75, 1050), (75, 1200), (75, 1350), (1050, 2325), (1200, 2325), (1350, 2325))
START_PRESETS = {"启停区 1": (2250.0, 150.0), "启停区 2": (2250.0, 2250.0)}
RAW_CENTER_Y_MM = -70.0
RAW_CENTER_X_RANGE = (1100.0, 1300.0)
QR_CENTER_X_MM = 2396.0
QR_CENTER_Y_RANGE = (1100.0, 1300.0)
OBSTACLE_RADIUS_MM = 25.0


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
        if event.button() == Qt.MouseButton.RightButton and self.mode == "add":
            self.preview_rotated.emit(); event.accept(); return
        if event.button() == Qt.MouseButton.RightButton and isinstance(item, StartItem):
            item.rotated(); event.accept(); return
        editable = isinstance(item, (WaypointItem, RotationHandleItem, StartItem, DraggableEllipseItem, DraggableRectItem))
        if event.button() == Qt.MouseButton.LeftButton and self.mode in ("add", "obstacle") and not editable:
            self._add_pressed = True; event.accept(); return
        if event.button() == Qt.MouseButton.LeftButton and self.mode in ("calibrate", "measure") and not editable:
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
        if self.mode == "add":
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
        if event.key() == Qt.Key.Key_Escape and self.mode == "measure":
            self.clicked.emit(float("nan"), float("nan"), False); event.accept(); return
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

    def mouseReleaseEvent(self, event):  # type: ignore[no-untyped-def]
        super().mouseReleaseEvent(event); self.moved(self.scenePos())


class DraggableRectItem(QGraphicsRectItem):
    def __init__(self, rect, moved):  # type: ignore[no-untyped-def]
        super().__init__(*rect); self.moved = moved
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable); self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable)

    def mouseReleaseEvent(self, event):  # type: ignore[no-untyped-def]
        super().mouseReleaseEvent(event); self.moved(self.scenePos())


class CarOutlineItem(QGraphicsRectItem):
    def __init__(self, rotated):  # type: ignore[no-untyped-def]
        super().__init__(-150, -150, 300, 300)
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


class PlannerWindow(QMainWindow):

    def __init__(self) -> None:
            super().__init__(); self.setWindowTitle("LittleCar2 比赛地图路径规划"); self.resize(1420, 860); self.setMinimumSize(1024, 768)
            self.plan = Plan(); self.active_index = -1; self.active_point_index = -1; self.mode = "select"; self.pending_action: str | None = None; self.calibration_pending = True; self.calibration_stage = "choose"; self.undo_stack = []; self.redo_stack = []
            self.selected_indices: set[int] = set(); self.measurement_points: list[QPointF] = []
            self.preview_paper: QPointF | None = None; self.preview_yaw_deg: float | None = None; self.preview_anchor_index: int | None = None; self.preview_anchor_signature = None; self.preview_shift = False
            self.timeline: list[SimulationFrame] = []; self.timeline_position = 0; self.current_frame = None; self.actual_trace = []
            self._layout_slider_before: Plan | None = None; self._layout_slider_changed = False
            self.scene = QGraphicsScene(self); self.view = MapView(); self.view.setScene(self.scene)
            self.view.clicked.connect(self.on_map_click); self.view.hovered.connect(self.update_preview); self.view.released.connect(self.on_map_release); self.view.preview_rotated.connect(self.rotate_preview_clockwise); self.view.box_selected.connect(self.select_box); self.timer = QTimer(self); self.timer.setInterval(20); self.timer.timeout.connect(self.tick)
            self._build(); self._build_layout_sliders(); self.view.view_changed.connect(self.position_layout_sliders); QShortcut(QKeySequence.StandardKey.Undo, self, activated=self.undo); QShortcut(QKeySequence.StandardKey.Redo, self, activated=self.redo); QShortcut(QKeySequence.StandardKey.SelectAll, self, activated=self.select_all); QShortcut(QKeySequence(Qt.Key.Key_Delete), self, activated=self.remove_selected_step)
            self.refresh_mode_ui(); self.redraw(); self.refresh_plans(); QTimer.singleShot(0,self.fit_map); QShortcut(QKeySequence(Qt.Key.Key_Home),self,activated=self.fit_map)

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
            root = QSplitter(Qt.Orientation.Horizontal); left = QWidget(); box = QVBoxLayout(left); box.setContentsMargins(10, 10, 10, 10); box.setSpacing(6)
            toolbar = QHBoxLayout(); self.tool_group = QButtonGroup(self); self.tool_group.setExclusive(True)
            self.select_button = QPushButton("选择"); self.add_button = QPushButton("添加节点"); self.measure_button = QPushButton("测距"); self.obstacle_button = QPushButton("障碍物")
            for button, value in ((self.select_button, "select"), (self.add_button, "add"), (self.measure_button, "measure"), (self.obstacle_button, "obstacle")):
                button.setCheckable(True); self.tool_group.addButton(button)
                button.toggled.connect(lambda checked=False, mode=value: checked and self.set_mode(mode)); toolbar.addWidget(button)
            self.select_button.setChecked(True); box.addLayout(toolbar)
            # v7 将所有动作放入同一流程，连续路径是一个独立步骤而非全局模式。
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
            self.steps_label=QLabel("流程步骤"); box.addWidget(self.steps_label); self.waypoint_list = QListWidget(); self.waypoint_list.currentRowChanged.connect(self.activate_node); self.waypoint_list.setMinimumHeight(105); box.addWidget(self.waypoint_list)
            action_row=QHBoxLayout(); self.add_goto_button=QPushButton("新增点到点"); self.append_rotation_button=QPushButton("新增原地转向"); self.add_continuous_button=QPushButton("新增连续段")
            self.add_goto_button.clicked.connect(self.begin_goto_add); self.append_rotation_button.clicked.connect(self.append_rotation); self.add_continuous_button.clicked.connect(self.add_continuous_segment)
            action_row.addWidget(self.add_goto_button); action_row.addWidget(self.append_rotation_button); action_row.addWidget(self.add_continuous_button); box.addLayout(action_row)
            order_row=QHBoxLayout(); self.move_up_button=QPushButton("上移"); self.move_down_button=QPushButton("下移"); self.move_up_button.clicked.connect(lambda:self.move_step(-1)); self.move_down_button.clicked.connect(lambda:self.move_step(1)); order_row.addWidget(self.move_up_button); order_row.addWidget(self.move_down_button); box.addLayout(order_row)
            self.delete_button = QPushButton("删除选中动作"); self.delete_button.clicked.connect(self.remove_selected_step); box.addWidget(self.delete_button)
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
            box.addWidget(QLabel("仿真")); simrow=QHBoxLayout()
            for label, fn in (("播放",self.play),("暂停",self.pause),("重置",self.reset_simulation)):
                button=QPushButton(label); button.clicked.connect(fn); simrow.addWidget(button)
                if label == "播放": self.play_button = button
            box.addLayout(simrow)
            self.progress = QSlider(Qt.Orientation.Horizontal); self.progress.setRange(0, 0); self.progress.setEnabled(False)
            self.progress.sliderPressed.connect(self.pause); self.progress.valueChanged.connect(self.seek_timeline)
            box.addWidget(self.progress); self.progress_label = QLabel("进度：0.00 / 0.00 s"); box.addWidget(self.progress_label)
            self.measurement_label = QLabel("水平：--  垂直：--  欧式：--"); self.measurement_label.setWordWrap(True); box.addWidget(self.measurement_label)
            self.status=QLabel("先选择起点，再右击蓝色箭头设置朝向并确认。"); self.status.setWordWrap(True); box.addWidget(self.status)
            self.path_check=QLabel("路径检查：等待编辑"); self.path_check.setWordWrap(True); box.addWidget(self.path_check); box.addStretch()
            scroll=QScrollArea(); scroll.setWidgetResizable(True); scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff); scroll.setWidget(left)
            map_panel=QWidget(); map_box=QVBoxLayout(map_panel); map_box.setContentsMargins(0,0,0,0); map_box.setSpacing(0)
            self.calibration_bar=QWidget(); guide=QHBoxLayout(self.calibration_bar); guide.setContentsMargins(12,8,12,8)
            self.calibration_label=QLabel("1. 选择起点   2. 右击蓝色箭头设置朝向") ; guide.addWidget(self.calibration_label); guide.addStretch()
            for label in ("启停区 1", "启停区 2", "自定义"):
                button=QPushButton(label); button.clicked.connect(lambda checked=False,value=label:self.begin_start(value)); guide.addWidget(button)
            self.confirm_start_button=QPushButton("确认朝向"); self.confirm_start_button.clicked.connect(self.confirm_start_heading); guide.addWidget(self.confirm_start_button)
            map_box.addWidget(self.calibration_bar); map_box.addWidget(self.view)
            root.addWidget(scroll); root.addWidget(map_panel); root.setSizes([360,1060]); self.setCentralWidget(root); self.update_calibration_ui()

    def set_mode(self, mode):
            if mode != "add": self.clear_preview()
            if mode in ("add", "obstacle") and self.calibration_pending:
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
                elif mode == "measure": message = "测距：左键依次选择两点，生成水平和垂直对齐线；Esc 或切换工具清除。"
                elif mode == "obstacle": message = "障碍物：左键释放添加黑色圆形标记，选择模式可拖拽。"
                self.status.setText(message)
            if mode == "select": self.select_button.setChecked(True)
            elif mode == "add": self.add_button.setChecked(True)
            elif mode == "measure": self.measure_button.setChecked(True)
            elif mode == "obstacle": self.obstacle_button.setChecked(True)

    def begin_start(self, kind):
            self.push_undo(); self.calibration_pending=True
            if kind == "自定义":
                self.plan.start_kind="custom"; self.calibration_stage="position"; self.mode="calibrate"; self.view.mode="calibrate"; self.calibration_label.setText("在地图上点击自定义起点位置"); self.update_calibration_ui(); self.redraw(); return
            self.plan.start_kind="zone_1" if kind.endswith("1") else "zone_2"
            self.plan.start_paper_x_mm, self.plan.start_paper_y_mm = START_PRESETS[kind]; self.calibration_stage="heading"; self.mode="calibrate"; self.view.mode="calibrate"; self.update_calibration_ui(); self.redraw()

    def on_map_click(self, x, y, shift=False):
            if math.isnan(x):
                self.clear_measurement(); return
            if not (0 <= x <= FIELD_SIZE_MM and 0 <= y <= FIELD_SIZE_MM): return
            if self.mode == "measure":
                self.add_measurement_point(QPointF(x, y), shift); return
            if self.mode == "obstacle":
                self.add_obstacle(QPointF(x, y)); return
            if self.mode == "calibrate":
                if self.calibration_stage != "position": return
                self.plan.start_paper_x_mm, self.plan.start_paper_y_mm = x, y; self.calibration_stage="heading"; self.update_calibration_ui(); self.redraw(); return
            if self.mode != "add" or self.calibration_pending: return
            self.update_preview(x, y, shift)
            self.confirm_preview(x, y, shift)

    def on_map_release(self, x, y, shift=False):
            if self.mode == "add": self.confirm_preview(x, y, shift)
            elif self.mode == "obstacle": self.add_obstacle(QPointF(x, y))

    def paper_of(self, waypoint):
            x,y=world_to_paper(Pose(waypoint.x_mm, waypoint.y_mm, waypoint.yaw_deg),self.plan.start_paper_x_mm,self.plan.start_paper_y_mm,self.plan.start_heading_deg); return Pose(x,y,waypoint.yaw_deg+self.plan.start_heading_deg)

    def rotate_start_clockwise(self):
            if not (self.calibration_pending and self.calibration_stage == "heading"): return
            self.push_undo(); self.plan.start_heading_deg=((self.plan.start_heading_deg-90+180)%360)-180
            self.redraw(); self.status.setText("起点朝向已顺时针旋转 90 度，可继续右击修改或点击确认朝向。")

    def confirm_start_heading(self):
            if not (self.calibration_pending and self.calibration_stage == "heading"): return
            if not self.is_valid_start_pose():
                self.status.setText("起点车体进入黄色禁行区或超出场地边界，无法确认。")
                return
            self.calibration_pending=False; self.calibration_stage="complete"; self.set_mode("select"); self.update_calibration_ui(); self.redraw(); self.status.setText("起点标定完成。")

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
            return build_goto_sweep(Pose(start.x(),start.y(),start_yaw+heading),Pose(end.x(),end.y(),end_yaw+heading),vmax,wmax,timeout)

    def continuous_sweep(self, start, end, start_yaw=0.0, end_yaw=0.0) -> SweepGeometry:
            heading=self.plan.start_heading_deg
            return build_continuous_segment_sweep(Pose(start.x(),start.y(),start_yaw+heading),Pose(end.x(),end.y(),end_yaw+heading))

    def rotation_sweep(self, position, start_yaw, end_yaw, wmax=90.0, timeout=15.0) -> SweepGeometry:
            heading=self.plan.start_heading_deg
            return build_rotation_sweep(Pose(position.x(),position.y(),start_yaw+heading),end_yaw+heading,wmax,timeout)

    def is_valid_start_pose(self):
            start=QPointF(self.plan.start_paper_x_mm,self.plan.start_paper_y_mm)
            return self.is_valid_route_segment(start,start)

    def sweep_path(self, sweep):
            path=QPainterPath()
            for polygon in sweep.polygons: path.addPath(self._polygon_path(polygon))
            return path

    def sweep_violations(self, sweep):
            field=QRectF(0,0,FIELD_SIZE_MM,FIELD_SIZE_MM)
            platforms=[QRectF(x,y,450,450) for x,y in PLATFORMS]
            platform_paths=[QPainterPath() for _ in platforms]
            for path,rect in zip(platform_paths,platforms): path.addRect(rect)
            hit_platform=QPainterPath(); out_of_bounds=False
            for polygon in sweep.polygons:
                if any(x < -1e-6 or x > FIELD_SIZE_MM + 1e-6 or y < -1e-6 or y > FIELD_SIZE_MM + 1e-6 for x,y in polygon): out_of_bounds=True
                body=self._polygon_path(polygon)
                for platform in platform_paths:
                    if body.intersects(platform): hit_platform.addPath(body.intersected(platform))
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

    def redraw(self):
            if hasattr(self,"raw_slider"):
                for slider,value in ((self.raw_slider,self.plan.layout.raw_center_x_mm),(self.qr_slider,self.plan.layout.qr_center_y_mm)):
                    slider.blockSignals(True); slider.setValue(round(value)); slider.blockSignals(False)
            self.scene.clear(); self.scene.setSceneRect(-360,-300,3100,3100); self.draw_field(); self.draw_start(); self.draw_route(); self.draw_measurement(); self.draw_preview(); self.draw_car(self.current_frame.actual if self.current_frame else None); self.position_layout_sliders()
            for item in self.scene.items():
                if isinstance(item,WaypointItem) and item.index in self.selected_indices: item.setSelected(True)

    def static(self,item,marker): item.setData(0,marker); item.setZValue(-20); item.setAcceptedMouseButtons(Qt.MouseButton.NoButton); return item

    def text(self,x,y,value,marker,size=18,rotation=0,color="#263238"):
            item=self.scene.addText(value,QFont("Microsoft YaHei",size)); item.setDefaultTextColor(QColor(color)); item.setPos(x,y); item.setRotation(rotation); return self.static(item,marker)

    def line(self,*args,marker,pen): return self.static(self.scene.addLine(*args,pen),marker)

    def draw_field(self):
            self.static(self.scene.addRect(0,0,2400,2400,QPen(QColor("#424242"),5),QColor("#ffffff")),"field_boundary")
            for x in range(0,2401,200): self.line(x,0,x,2400,marker="grid",pen=QPen(QColor(80,80,80,25),1))
            for y in range(0,2401,200): self.line(0,y,2400,y,marker="grid",pen=QPen(QColor(80,80,80,25),1))
            for x,y in PLATFORMS: self.static(self.scene.addRect(x,y,450,450,QPen(Qt.PenStyle.NoPen),QColor("#fffde7")),"platform")
            dash=QPen(QColor("#616161"),3,Qt.PenStyle.DashLine); self.line(1200,0,1200,2400,marker="center_line",pen=dash); self.line(0,1200,2400,1200,marker="center_line",pen=dash)
            for x,y,label in ((2250,150,"启停区 1"),(2250,2250,"启停区 2")):
                self.static(self.scene.addRect(x-150,y-150,300,300,QPen(Qt.PenStyle.NoPen),QColor("#114ce0")),"start_zone"); self.text(x-115,y+162,label,"start_zone_label",20)
            self.static(self.scene.addRect(0,910,150,580,QPen(QColor("#9e9e9e"),2),QColor("#ffffff")),"storage_base"); self.static(self.scene.addRect(910,2250,580,150,QPen(QColor("#9e9e9e"),2),QColor("#ffffff")),"rough_base")
            raw_x=self.plan.layout.raw_center_x_mm; raw=DraggableEllipseItem((-150,-150,300,300),self.move_raw_area); raw.setPos(raw_x,RAW_CENTER_Y_MM); raw.setPen(QPen(QColor("#444"),4)); raw.setBrush(QColor("#f7f7f7")); raw.setData(0,"raw_turntable"); raw.setZValue(5); self.scene.addItem(raw)
            for angle in (90, 210, 330):
                radians=math.radians(angle); x=raw_x+100*math.cos(radians); y=RAW_CENTER_Y_MM+100*math.sin(radians)
                self.static(self.scene.addEllipse(x-25,y-25,50,50,QPen(QColor("#444"),3),QColor("white")),"raw_pick_hole")
            for x,y in MATERIAL_SLOTS:
                self.static(self.scene.addEllipse(x-40,y-40,80,80,QPen(QColor("#222"),4),QColor("white")),"material_slot_outer"); self.static(self.scene.addEllipse(x-13,y-13,26,26,QPen(Qt.PenStyle.NoPen),QColor("#222")),"material_slot_inner")
            qr=DraggableRectItem((-4,-100,8,200),self.move_qr_board); qr.setPos(QR_CENTER_X_MM,self.plan.layout.qr_center_y_mm); qr.setPen(QPen(Qt.PenStyle.NoPen)); qr.setBrush(QColor("#212121")); qr.setData(0,"qr_board"); qr.setZValue(5); self.scene.addItem(qr)
            safe=QPen(QColor("#2e7d32"),3,Qt.PenStyle.DashLine); self.static(self.scene.addRect(150,150,2100,2100,safe),"car_center_boundary")
            self.text(720,105,"原料区","raw_label",26); self.text(180,1260,"暂存区","storage_label",24,90); self.text(1320,2180,"粗加工区","rough_label",26); self.text(2290,1300,"二维码板","coding_label",22,90); self.text(690,2420,"车体中心可移动范围：X/Y 150～2250 mm","movable_boundary_label",18,0,"#2e7d32")
            self.draw_dimensions()
            for index,obstacle in enumerate(self.plan.layout.obstacles):
                item=DraggableEllipseItem((-OBSTACLE_RADIUS_MM,-OBSTACLE_RADIUS_MM,OBSTACLE_RADIUS_MM*2,OBSTACLE_RADIUS_MM*2),lambda position,index=index:self.move_obstacle(index,position)); item.setPos(obstacle.paper_x_mm,obstacle.paper_y_mm); item.setBrush(QColor("#000000")); item.setPen(QPen(QColor("#000000"),1)); item.setData(0,"obstacle"); item.setData(1,index); item.setZValue(20); self.scene.addItem(item)

    def draw_dimensions(self):
            blue=QColor("#1748c5"); pen=QPen(blue,2)
            def h(x1,x2,edge,dim,label,key):
                self.line(x1,edge,x1,dim,marker=key,pen=QPen(blue)); self.line(x2,edge,x2,dim,marker=key,pen=QPen(blue)); self.line(x1,dim,x2,dim,marker=key,pen=pen); self.text((x1+x2)/2-32,dim-32,label,key,18,0,"#1748c5")
            def v(y1,y2,edge,dim,label,key):
                self.line(edge,y1,dim,y1,marker=key,pen=QPen(blue)); self.line(edge,y2,dim,y2,marker=key,pen=QPen(blue)); self.line(dim,y1,dim,y2,marker=key,pen=pen); self.text(dim+8,(y1+y2)/2,label,key,18,90,"#1748c5")
            h(0,2400,2400,2530,"2400","dim_2400w"); v(0,2400,2400,2500,"2400","dim_2400h"); h(550,1000,1000,1060,"450","dim_platform_450"); v(550,1000,550,490,"450","dim_platform_450"); h(1000,1400,550,470,"400","dim_channel_400"); v(1000,1400,1400,1480,"400","dim_channel_400"); h(2100,2400,0,-100,"300","dim_start_300"); v(0,300,2400,2470,"300","dim_start_300"); h(0,150,0,-180,"150","dim_storage_150"); v(910,1490,0,-90,"580","dim_storage_580"); h(910,1490,2400,2620,"580","dim_rough_580"); v(2250,2400,1490,1570,"150","dim_rough_150"); h(1050,1350,-70,-270,"Ø300","dim_raw_diameter"); h(1100,1300,-70,-150,"Ø200","dim_raw_pitch"); v(1100,1300,2400,2570,"1100～1300","dim_qr_range")

    def draw_start(self):
            if self.calibration_pending and self.calibration_stage in ("choose","position"): return
            item=StartItem(self.plan.start_heading_deg,self.rotate_start_clockwise); item.setPos(self.plan.start_paper_x_mm,self.plan.start_paper_y_mm); self.scene.addItem(item)

    def add_measurement_point(self, point, shift=False):
            if len(self.measurement_points) >= 2: self.measurement_points=[]
            if shift and self.measurement_points:
                point=snap_to_45(self.measurement_points[0],point)
            self.measurement_points.append(QPointF(point)); self.update_measurement_ui(); self.redraw()

    def add_obstacle(self, point):
            if not (0 <= point.x() <= FIELD_SIZE_MM and 0 <= point.y() <= FIELD_SIZE_MM): return
            self.push_layout_undo(); self.plan.layout.obstacles.append(Obstacle(*self.clamp_obstacle(point)))
            self.redraw()

    @staticmethod
    def clamp_obstacle(point):
            return (max(OBSTACLE_RADIUS_MM,min(FIELD_SIZE_MM-OBSTACLE_RADIUS_MM,point.x())),max(OBSTACLE_RADIUS_MM,min(FIELD_SIZE_MM-OBSTACLE_RADIUS_MM,point.y())))

    def move_obstacle(self, index, position):
            if not 0 <= index < len(self.plan.layout.obstacles): return
            self.push_layout_undo(); obstacle=self.plan.layout.obstacles[index]; obstacle.paper_x_mm,obstacle.paper_y_mm=self.clamp_obstacle(position); self.redraw()

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
            self._invalidate_timeline(); self.plan=Plan(); self.active_index=-1; self.active_point_index=-1; self.undo_stack=[]; self.redo_stack=[]; self.calibration_pending=True; self.calibration_stage="choose"; self.set_mode("select"); self.update_calibration_ui(); self.refresh_waypoints(); self.redraw()

    def load_selected(self):
            item=self.plan_list.currentItem()
            if item is None: return
            try:
                self._invalidate_timeline(); self.plan=load_plan(item.text()); self.active_index=-1; self.active_point_index=-1; self.calibration_pending=False; self.calibration_stage="complete"; self.update_calibration_ui(); self.refresh_waypoints(); self.redraw()
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
            self.steps_label.setText("流程步骤")
            self.continuous_panel.setVisible(isinstance(self._selected_step(), ContinuousPathSegment))

    def refresh_waypoints(self):
            self.waypoint_list.blockSignals(True)
            self.waypoint_list.clear()
            for index, step in enumerate(self.plan.steps):
                if isinstance(step, Waypoint):
                    text = f"{index + 1}. 到点停靠 ({step.x_mm:.0f}, {step.y_mm:.0f})"
                elif isinstance(step, RotateInPlace):
                    text = f"{index + 1}. 原地转向 {step.yaw_deg:.0f} deg"
                else:
                    text = f"{index + 1}. 连续路径段 {step.name} ({max(0, len(step.points) - 1)} 点)"
                self.waypoint_list.addItem(text)
            if 0 <= self.active_index < self.waypoint_list.count():
                self.waypoint_list.setCurrentRow(self.active_index)
            self.waypoint_list.blockSignals(False)
            self.refresh_mode_ui()
            if 0 <= self.active_index < len(self.plan.steps):
                self.show_node(self.active_index)
            self.codegen_button.setEnabled(not self.calibration_pending and bool(self.plan.steps))

    def activate_node(self, index):
            if not 0 <= index < len(self.plan.steps):
                return
            self.active_index, self.active_point_index = index, -1
            self.show_node(index)
            self.refresh_mode_ui()
            self.redraw()

    def show_node(self, index):
            step = self._selected_step()
            if step is None:
                return
            continuous, rotating = isinstance(step, ContinuousPathSegment), isinstance(step, RotateInPlace)
            self.continuous_panel.setVisible(continuous)
            for widget in self.goto_form_widgets:
                widget.setVisible(not continuous and not rotating)
            for widget in (self.yaw, self.node_wmax, self.timeout, self.update_action_button):
                widget.setVisible(not continuous)
            if continuous:
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

    def add_continuous_segment(self):
            self.push_undo()
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
            self.push_undo()
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
                    paper = self.paper_of(previous)
                    if not self.is_valid_rotation(QPointF(paper.x_mm, paper.y_mm), previous.yaw_deg, step.yaw_deg, step.wmax_deg_s, step.timeout_s): invalid.append(index)
                elif len(step.points) < 2:
                    invalid.append(index)
                else:
                    for first, second in zip(step.points, step.points[1:]):
                        a, b = self.paper_of(first), self.paper_of(second)
                        if not self.is_valid_continuous_segment(QPointF(a.x_mm, a.y_mm), QPointF(b.x_mm, b.y_mm), first.yaw_deg, second.yaw_deg):
                            invalid.append(index); break
            return invalid

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
                for point_index, point in enumerate(step.points):
                    paper = self.paper_of(point); current = QPointF(paper.x_mm, paper.y_mm)
                    if point_index:
                        prior = self.paper_of(step.points[point_index - 1]); self.scene.addLine(prior.x_mm, prior.y_mm, current.x(), current.y(), QPen(color, 6))
                    radius = 13 if point_index == len(step.points) - 1 else 10
                    marker = self.scene.addEllipse(current.x() - radius, current.y() - radius, radius * 2, radius * 2, QPen(color, 3), QColor("#e0f2f1")); marker.setData(0, "continuous_endpoint" if point_index == len(step.points) - 1 else "continuous_waypoint"); marker.setZValue(12)
                    self._draw_direction_arrow(current.x(), current.y(), point.yaw_deg, color, "continuous_direction")
                if step.points:
                    last = step.points[-1]; previous = Pose(last.x_mm, last.y_mm, last.yaw_deg)

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
            step.yaw_deg = -math.degrees(math.atan2(delta.y(), delta.x())) - self.plan.start_heading_deg; step.use_yaw = True
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
            for widget in (self.add_button, self.add_goto_button, self.obstacle_button, self.save_button, self.save_as_button, self.play_button):
                widget.setEnabled(enabled)
            self.codegen_button.setEnabled(enabled and bool(self.plan.steps))

    def undo(self):
            if not self.undo_stack:
                return
            self.redo_stack.append(copy.deepcopy(self.plan)); self.plan = self.undo_stack.pop()
            self.active_index = min(self.active_index, len(self.plan.steps) - 1); self.active_point_index = -1
            self._sync_continuous_entries(); self._invalidate_timeline(); self.refresh_waypoints(); self.redraw(); self.rebuild_timeline_after_edit()

    def redo(self):
            if not self.redo_stack:
                return
            self.undo_stack.append(copy.deepcopy(self.plan)); self.plan = self.redo_stack.pop()
            self.active_index = min(self.active_index, len(self.plan.steps) - 1); self.active_point_index = -1
            self._sync_continuous_entries(); self._invalidate_timeline(); self.refresh_waypoints(); self.redraw(); self.rebuild_timeline_after_edit()

    def draw_preview(self):
            if self.preview_paper is None or self.preview_yaw_deg is None:
                return
            step = self._selected_step()
            anchor, anchor_yaw = self._step_anchor(self.active_index + 1 if isinstance(step, ContinuousPathSegment) else len(self.plan.steps))
            if isinstance(step, ContinuousPathSegment) and step.points:
                last = self.paper_of(step.points[-1]); anchor, anchor_yaw = QPointF(last.x_mm, last.y_mm), step.points[-1].yaw_deg
            sweep = self.continuous_sweep(anchor, self.preview_paper, anchor_yaw, self.preview_yaw_deg) if isinstance(step, ContinuousPathSegment) else self.route_sweep(anchor, self.preview_paper, anchor_yaw, self.preview_yaw_deg)
            out_of_bounds, hit_platform = self.sweep_violations(sweep)
            color = QColor("#c62828") if out_of_bounds or not hit_platform.isEmpty() else QColor("#1565c0")
            path = self.scene.addPath(self.sweep_path(sweep), QPen(Qt.PenStyle.NoPen), QColor(color.red(), color.green(), color.blue(), 70)); path.setData(0, "preview_sweep"); path.setZValue(2)
            if not hit_platform.isEmpty():
                collision = self.scene.addPath(hit_platform, QPen(Qt.PenStyle.NoPen), QColor("#d32f2f")); collision.setData(0, "preview_platform_collision"); collision.setZValue(3)
            item = CarOutlineItem(self.rotate_preview_clockwise); item.setPos(self.preview_paper); item.setRotation(-(self.preview_yaw_deg + self.plan.start_heading_deg)); item.setPen(QPen(color, 4)); item.setBrush(QColor(color.red(), color.green(), color.blue(), 42)); item.setZValue(17); self.scene.addItem(item)
            self._draw_direction_arrow(self.preview_paper.x(), self.preview_paper.y(), self.preview_yaw_deg, color, "preview_direction")
            self._set_path_check("预览", "可行" if not out_of_bounds and hit_platform.isEmpty() else ("越界" if out_of_bounds else "碰撞平台"))

    def begin_goto_add(self):
            self.pending_action = "goto"
            self.set_mode("add")
            self.status.setText("新增点到点：在地图上点击目标位置。")

    def _draw_direction_arrow(self, x, y, yaw, color, marker):
            arrow = QGraphicsPolygonItem(QPolygonF([QPointF(-18, -14), QPointF(20, -14), QPointF(20, -28), QPointF(52, 0), QPointF(20, 28), QPointF(20, 14), QPointF(-18, 14)]))
            arrow.setPos(x, y); arrow.setRotation(-(yaw + self.plan.start_heading_deg)); arrow.setBrush(color); arrow.setPen(QPen(QColor("#0d47a1"), 2)); arrow.setData(0, marker); arrow.setZValue(18); self.scene.addItem(arrow)

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
            car = CarOutlineItem(self.rotate_car_clockwise); car.setPos(x, y); car.setRotation(-(p.yaw_deg + self.plan.start_heading_deg)); car.setPen(QPen(color, 5)); car.setBrush(QColor(120, 144, 156, 105)); car.setZValue(15); self.scene.addItem(car)
            self._draw_direction_arrow(x, y, p.yaw_deg, QColor("#1565c0"), "car_direction")

    def clear_preview(self, redraw=True):
            self.preview_paper = None; self.preview_yaw_deg = None; self.preview_anchor_index = None; self.preview_anchor_signature = None; self.preview_shift = False
            if redraw: self.redraw()

    def _current_preview_anchor(self):
            step = self._selected_step()
            if isinstance(step, ContinuousPathSegment) and step.points and self.pending_action != "goto":
                point = step.points[-1]; paper = self.paper_of(point)
                return QPointF(paper.x_mm, paper.y_mm), point.yaw_deg, (self.active_index, len(step.points) - 1, point.x_mm, point.y_mm, point.yaw_deg)
            anchor, yaw = self._step_anchor(len(self.plan.steps))
            return anchor, yaw, (len(self.plan.steps), anchor.x(), anchor.y(), yaw)

    def update_preview(self, x, y, shift=False):
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


def main() -> int:
    """Launch the standalone map planner window."""

    app = QApplication(sys.argv)
    app.setStyleSheet("QWidget{font-family:'Microsoft YaHei';font-size:13px;} QPushButton{min-height:28px;} QScrollArea{border:0;}")
    window = PlannerWindow()
    window.show()
    return app.exec()

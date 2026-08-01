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
    QMainWindow, QMessageBox, QPushButton, QScrollArea, QSplitter, QVBoxLayout, QWidget, QRubberBand)

from .geometry import paper_to_world, world_to_paper
from .models import CAR_SIZE_MM, FIELD_SIZE_MM, Plan, Pose, Waypoint
from .sim import Simulation
from .storage import list_plans, load_plan, save_plan

PLATFORMS = ((550, 550), (1400, 550), (550, 1400), (1400, 1400))
MATERIAL_SLOTS = ((75, 1050), (75, 1200), (75, 1350), (1050, 2325), (1200, 2325), (1350, 2325))
START_PRESETS = {"启停区 1": (2250.0, 150.0), "启停区 2": (2250.0, 2250.0)}


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
    box_selected = Signal(QRectF, bool)

    def __init__(self) -> None:
        super().__init__(); self.mode = "select"; self._rubber = None; self._origin = None; self._panning = False
        self._space_pressed = False; self._pan_origin = QPoint(); self._pan_scroll = QPoint()
        self.setRenderHint(QPainter.RenderHint.Antialiasing); self.setDragMode(QGraphicsView.DragMode.NoDrag)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

    def wheelEvent(self, event):  # type: ignore[no-untyped-def]
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15; self.scale(factor, factor)

    def mousePressEvent(self, event):  # type: ignore[no-untyped-def]
        if event.button() == Qt.MouseButton.MiddleButton or (event.button() == Qt.MouseButton.LeftButton and self._space_pressed):
            self._panning = True; self._pan_origin = event.position().toPoint()
            self._pan_scroll = QPoint(self.horizontalScrollBar().value(), self.verticalScrollBar().value())
            self.viewport().setCursor(Qt.CursorShape.ClosedHandCursor); event.accept(); return
        item = self.itemAt(event.position().toPoint())
        editable = isinstance(item, (WaypointItem, RotationHandleItem, StartItem, StartHeadingHandle))
        if event.button() == Qt.MouseButton.LeftButton and self.mode in ("add", "calibrate") and not editable:
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
            event.accept(); return
        if self._rubber is not None:
            self._rubber.setGeometry(QRect(self._origin, event.position().toPoint()).normalized()); return
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
        super().mouseReleaseEvent(event)

    def keyPressEvent(self, event):  # type: ignore[no-untyped-def]
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


class StartHeadingHandle(QGraphicsEllipseItem):
    def __init__(self, owner, changed):  # type: ignore[no-untyped-def]
        super().__init__(-12, -12, 24, 24, owner); self.changed = changed
        self.setBrush(QColor("#ffffff")); self.setPen(QPen(QColor("#1565c0"), 3)); self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable)
        icon=QGraphicsTextItem("↻",self); icon.setFont(QFont("Microsoft YaHei",12)); icon.setDefaultTextColor(QColor("#1565c0")); icon.setPos(-9,-13); icon.setAcceptedMouseButtons(Qt.MouseButton.NoButton); self.setToolTip("拖动设置起始朝向")
    def mouseMoveEvent(self, event): super().mouseMoveEvent(event)
    def mouseReleaseEvent(self, event): super().mouseReleaseEvent(event); self.changed(self)


class StartItem(QGraphicsPolygonItem):
    """流程图风格的起点箭头，拖动末端圆柄完成初始朝向标定。"""
    def __init__(self, heading, changed):  # type: ignore[no-untyped-def]
        poly = QPolygonF([QPointF(-28, -18), QPointF(10, -18), QPointF(10, -34), QPointF(42, 0), QPointF(10, 34), QPointF(10, 18), QPointF(-28, 18)])
        super().__init__(poly); self.setRotation(-heading); self.setBrush(QColor("#1976d2")); self.setPen(QPen(QColor("#0d47a1"), 3)); self.setZValue(12)
        self.handle = StartHeadingHandle(self, changed); self.handle.setPos(70, 0)


class PlannerWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__(); self.setWindowTitle("LittleCar2 比赛地图路径规划"); self.resize(1420, 860); self.setMinimumSize(1024, 768)
        self.plan = Plan(); self.active_index = -1; self.mode = "select"; self.calibration_pending = True; self.calibration_stage = "choose"; self.undo_stack = []; self.redo_stack = []
        self.selected_indices: set[int] = set()
        self.simulation = None; self.actual_trace = []; self.scene = QGraphicsScene(self); self.view = MapView(); self.view.setScene(self.scene)
        self.view.clicked.connect(self.on_map_click); self.view.box_selected.connect(self.select_box); self.timer = QTimer(self); self.timer.setInterval(20); self.timer.timeout.connect(self.tick)
        self._build(); QShortcut(QKeySequence.StandardKey.Undo, self, activated=self.undo); QShortcut(QKeySequence.StandardKey.Redo, self, activated=self.redo); QShortcut(QKeySequence.StandardKey.SelectAll, self, activated=self.select_all); QShortcut(QKeySequence(Qt.Key.Key_Delete), self, activated=self.remove_waypoint)
        self.redraw(); self.refresh_plans(); QTimer.singleShot(0,self.fit_map); QShortcut(QKeySequence(Qt.Key.Key_Home),self,activated=self.fit_map)

    def fit_map(self):
        self.view.fitInView(QRectF(-260,-260,2920,2920),Qt.AspectRatioMode.KeepAspectRatio)

    def _build(self) -> None:
        root = QSplitter(Qt.Orientation.Horizontal); left = QWidget(); box = QVBoxLayout(left); box.setContentsMargins(10, 10, 10, 10); box.setSpacing(6)
        toolbar = QHBoxLayout(); self.tool_group = QButtonGroup(self); self.tool_group.setExclusive(True)
        self.select_button = QPushButton("选择"); self.add_button = QPushButton("添加节点")
        for button, value in ((self.select_button, "select"), (self.add_button, "add")):
            button.setCheckable(True); self.tool_group.addButton(button)
            button.toggled.connect(lambda checked=False, mode=value: checked and self.set_mode(mode)); toolbar.addWidget(button)
        self.select_button.setChecked(True); box.addLayout(toolbar)
        box.addWidget(QLabel("方案")); self.plan_list = QListWidget(); self.plan_list.itemDoubleClicked.connect(self.load_selected); self.plan_list.setMinimumHeight(76); box.addWidget(self.plan_list)
        row = QHBoxLayout()
        for label, fn in (("新建", self.new_plan), ("保存", self.save), ("另存", self.save_as), ("加载", self.load_selected)):
            button = QPushButton(label); button.clicked.connect(fn); row.addWidget(button)
            if label == "保存": self.save_button = button
            elif label == "另存": self.save_as_button = button
        box.addLayout(row)
        box.addWidget(QLabel("节点（绿色为当前可编辑节点；橙色节点右键切换）")); self.waypoint_list = QListWidget(); self.waypoint_list.currentRowChanged.connect(self.activate_node); self.waypoint_list.setMinimumHeight(105); box.addWidget(self.waypoint_list)
        delete = QPushButton("删除选中节点"); delete.clicked.connect(self.remove_waypoint); box.addWidget(delete)
        form = QFormLayout(); self.x=spin(); self.y=spin(); self.yaw=spin(0,-360,360,5); self.use_yaw=QCheckBox("启用航向约束（GOTO Pose）")
        self.stop=QCheckBox("到点停止"); self.stop.setChecked(True); self.dwell=spin(.5,0,120,.1); self.node_vmax=spin(200,1,1500,10); self.vmax=self.node_vmax; self.node_wmax=spin(90,1,360,5); self.timeout=spin(15, .1, 300, 1)
        for label, widget in (("X (mm)",self.x),("Y (mm)",self.y),("目标航向 (deg)",self.yaw),("停留 (s)",self.dwell),("最大线速度",self.node_vmax),("最大角速度",self.node_wmax),("超时 (s)",self.timeout)): form.addRow(label,widget)
        form.addRow(self.use_yaw); form.addRow(self.stop); update=QPushButton("更新当前节点"); update.clicked.connect(self.update_waypoint); form.addRow(update); box.addLayout(form)
        box.addWidget(QLabel("仿真")); simrow=QHBoxLayout()
        for label, fn in (("播放",self.play),("暂停",self.pause),("重置",self.reset_simulation)):
            button=QPushButton(label); button.clicked.connect(fn); simrow.addWidget(button)
            if label == "播放": self.play_button = button
        box.addLayout(simrow); self.status=QLabel("先选择起点并拖动蓝色流程箭头标定朝向。"); self.status.setWordWrap(True); box.addWidget(self.status); box.addStretch()
        scroll=QScrollArea(); scroll.setWidgetResizable(True); scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff); scroll.setWidget(left)
        map_panel=QWidget(); map_box=QVBoxLayout(map_panel); map_box.setContentsMargins(0,0,0,0); map_box.setSpacing(0)
        self.calibration_bar=QWidget(); guide=QHBoxLayout(self.calibration_bar); guide.setContentsMargins(12,8,12,8)
        self.calibration_label=QLabel("1. 选择起点   2. 拖动蓝色箭头设置朝向") ; guide.addWidget(self.calibration_label); guide.addStretch()
        for label in ("启停区 1", "启停区 2", "自定义"):
            button=QPushButton(label); button.clicked.connect(lambda checked=False,value=label:self.begin_start(value)); guide.addWidget(button)
        map_box.addWidget(self.calibration_bar); map_box.addWidget(self.view)
        root.addWidget(scroll); root.addWidget(map_panel); root.setSizes([360,1060]); self.setCentralWidget(root); self.update_calibration_ui()

    def set_mode(self, mode):
        if mode == "add" and self.calibration_pending:
            self.select_button.setChecked(True)
            if hasattr(self,"status"): self.status.setText("请先完成起点和朝向标定。")
            return
        self.mode=mode; self.view.mode=mode
        if hasattr(self,"status"): self.status.setText("选择：框选、Ctrl 多选；中键或空格+左键平移地图。" if mode=="select" else "添加节点：点击地图添加，按住 Shift 吸附水平、垂直或 45 度。")
        if mode == "select": self.select_button.setChecked(True)
        elif mode == "add": self.add_button.setChecked(True)

    def begin_start(self, kind):
        self.push_undo(); self.calibration_pending=True
        if kind == "自定义":
            self.plan.start_kind="custom"; self.calibration_stage="position"; self.mode="calibrate"; self.view.mode="calibrate"; self.calibration_label.setText("在地图上点击自定义起点位置"); self.redraw(); return
        self.plan.start_kind="zone_1" if kind.endswith("1") else "zone_2"
        self.plan.start_paper_x_mm, self.plan.start_paper_y_mm = START_PRESETS[kind]; self.calibration_stage="heading"; self.mode="calibrate"; self.view.mode="calibrate"; self.update_calibration_ui(); self.redraw()

    def update_calibration_ui(self):
        self.calibration_bar.setVisible(self.calibration_pending)
        if self.calibration_pending and self.calibration_stage == "heading": self.calibration_label.setText("2. 拖动蓝色箭头的圆柄设置朝向")
        enabled=not self.calibration_pending
        for widget in (self.add_button,self.save_button,self.save_as_button,self.play_button): widget.setEnabled(enabled)

    def on_map_click(self, x, y, shift=False):
        if not (0 <= x <= FIELD_SIZE_MM and 0 <= y <= FIELD_SIZE_MM): return
        if self.mode == "calibrate":
            if self.calibration_stage != "position": return
            self.plan.start_paper_x_mm, self.plan.start_paper_y_mm = x, y; self.calibration_stage="heading"; self.update_calibration_ui(); self.redraw(); return
        if self.mode != "add" or self.calibration_pending: return
        self.push_undo()
        if shift:
            anchor=QPointF(self.plan.start_paper_x_mm,self.plan.start_paper_y_mm)
            if self.plan.waypoints:
                prev=self.paper_of(self.plan.waypoints[-1]); anchor=QPointF(prev.x_mm,prev.y_mm)
            snapped=snap_to_45(anchor,QPointF(x,y)); x,y=snapped.x(),snapped.y()
        pose=paper_to_world(x,y,self.plan.start_paper_x_mm,self.plan.start_paper_y_mm,self.plan.start_heading_deg)
        self.plan.waypoints.append(Waypoint(pose.x_mm,pose.y_mm,name=f"节点 {len(self.plan.waypoints)+1}")); self.active_index=len(self.plan.waypoints)-1; self.refresh_waypoints(); self.redraw()

    def paper_of(self, waypoint):
        x,y=world_to_paper(Pose(waypoint.x_mm, waypoint.y_mm, waypoint.yaw_deg),self.plan.start_paper_x_mm,self.plan.start_paper_y_mm,self.plan.start_heading_deg); return Pose(x,y,waypoint.yaw_deg+self.plan.start_heading_deg)

    def refresh_waypoints(self):
        self.waypoint_list.blockSignals(True); self.waypoint_list.clear()
        for i,p in enumerate(self.plan.waypoints): self.waypoint_list.addItem(f"{'* ' if i==self.active_index else ''}{i+1}. ({p.x_mm:.0f}, {p.y_mm:.0f}){'  航向' if p.use_yaw else ''}")
        self.waypoint_list.blockSignals(False)
        if 0<=self.active_index<len(self.plan.waypoints): self.waypoint_list.setCurrentRow(self.active_index); self.show_node(self.active_index)

    def show_node(self,index):
        if not 0<=index<len(self.plan.waypoints): return
        p=self.plan.waypoints[index]; self.x.setValue(p.x_mm); self.y.setValue(p.y_mm); self.yaw.setValue(p.yaw_deg); self.use_yaw.setChecked(p.use_yaw); self.stop.setChecked(p.stop); self.dwell.setValue(p.dwell_s); self.node_vmax.setValue(p.vmax_mm_s); self.node_wmax.setValue(p.wmax_deg_s); self.timeout.setValue(p.timeout_s)

    def activate_node(self,index):
        if not 0<=index<len(self.plan.waypoints) or index==self.active_index: return
        self.push_undo(); self.active_index=index; self.show_node(index); self.redraw()

    def set_active_from_context(self,index):
        if index != self.active_index: self.push_undo(); self.active_index=index; self.refresh_waypoints(); self.redraw(); self.status.setText("已设为当前节点。")

    def update_waypoint(self):
        if not 0<=self.active_index<len(self.plan.waypoints): return
        self.push_undo(); p=self.plan.waypoints[self.active_index]; p.x_mm=self.x.value(); p.y_mm=self.y.value(); p.yaw_deg=self.yaw.value(); p.use_yaw=self.use_yaw.isChecked(); p.stop=self.stop.isChecked(); p.dwell_s=self.dwell.value(); p.vmax_mm_s=self.node_vmax.value(); p.wmax_deg_s=self.node_wmax.value(); p.timeout_s=self.timeout.value(); self.refresh_waypoints(); self.redraw()

    def move_waypoint(self,index,before,after,shift=False):
        if index != self.active_index or before==after: return
        indices={item.index for item in self.scene.selectedItems() if isinstance(item,WaypointItem)} or {index}
        self.push_undo(); target=QPointF(after)
        if shift:
            anchor=QPointF(self.plan.start_paper_x_mm,self.plan.start_paper_y_mm)
            if index: prev=self.paper_of(self.plan.waypoints[index-1]); anchor=QPointF(prev.x_mm,prev.y_mm)
            target=snap_to_45(anchor,target)
        delta=target-before
        for selected in indices:
            current=self.paper_of(self.plan.waypoints[selected]); pose=paper_to_world(current.x_mm+delta.x(),current.y_mm+delta.y(),self.plan.start_paper_x_mm,self.plan.start_paper_y_mm,self.plan.start_heading_deg)
            self.plan.waypoints[selected].x_mm,self.plan.waypoints[selected].y_mm=pose.x_mm,pose.y_mm
        self.selected_indices=set(indices); self.refresh_waypoints(); self.redraw()

    def rotate_waypoint(self,index,item):
        if index != self.active_index: return
        self.push_undo(); delta=item.pos(); yaw=-math.degrees(math.atan2(delta.y(),delta.x()))-self.plan.start_heading_deg
        p=self.plan.waypoints[index]; p.yaw_deg=yaw; p.use_yaw=True; self.show_node(index); self.redraw()

    def rotate_start(self,item):
        self.push_undo(); delta=item.pos(); self.plan.start_heading_deg=-math.degrees(math.atan2(delta.y(),delta.x())); self.calibration_pending=False; self.calibration_stage="complete"; self.set_mode("select"); self.update_calibration_ui(); self.redraw(); self.status.setText("起点标定完成。")

    def select_box(self,rect,append):
        if not append: self.scene.clearSelection()
        for item in self.scene.items(rect):
            if isinstance(item,WaypointItem): item.setSelected(True)
        self.selected_indices={item.index for item in self.scene.selectedItems() if isinstance(item,WaypointItem)}

    def select_all(self):
        for item in self.scene.items():
            if isinstance(item,WaypointItem): item.setSelected(True)
        self.selected_indices=set(range(len(self.plan.waypoints)))

    def remove_waypoint(self):
        indices=sorted((i.index for i in self.scene.selectedItems() if isinstance(i,WaypointItem)),reverse=True)
        if not indices: return
        self.push_undo()
        for index in indices: del self.plan.waypoints[index]
        self.active_index=min(self.active_index,len(self.plan.waypoints)-1); self.refresh_waypoints(); self.redraw()

    def push_undo(self): self.undo_stack.append(copy.deepcopy(self.plan)); self.undo_stack=self.undo_stack[-100:]; self.redo_stack.clear()
    def undo(self):
        if not self.undo_stack:return
        self.redo_stack.append(copy.deepcopy(self.plan)); self.plan=self.undo_stack.pop(); self.active_index=min(self.active_index,len(self.plan.waypoints)-1); self.simulation=None; self.actual_trace=[]; self.refresh_waypoints(); self.redraw()
    def redo(self):
        if not self.redo_stack:return
        self.undo_stack.append(copy.deepcopy(self.plan)); self.plan=self.redo_stack.pop(); self.active_index=min(self.active_index,len(self.plan.waypoints)-1); self.refresh_waypoints(); self.redraw()

    def invalid_waypoints(self):
        margin=CAR_SIZE_MM/2.0; invalid=[]
        for index,waypoint in enumerate(self.plan.waypoints):
            paper=self.paper_of(waypoint)
            if not (margin <= paper.x_mm <= FIELD_SIZE_MM-margin and margin <= paper.y_mm <= FIELD_SIZE_MM-margin): invalid.append(index)
        return invalid

    def redraw(self):
        self.scene.clear(); self.scene.setSceneRect(-360,-300,3100,3100); self.draw_field(); self.draw_start(); self.draw_route(); self.draw_car(self.simulation.actual if self.simulation else None)
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
        self.static(self.scene.addEllipse(1050,-220,300,300,QPen(QColor("#444"),4),QColor("#f7f7f7")),"raw_turntable")
        for x,y in ((1130,20),(1270,20),(1200,110)): self.static(self.scene.addEllipse(x-12,y-12,24,24,QPen(QColor("#444"),3),QColor("white")),"raw_pick_hole")
        for x,y in MATERIAL_SLOTS:
            self.static(self.scene.addEllipse(x-40,y-40,80,80,QPen(QColor("#222"),4),QColor("white")),"material_slot_outer"); self.static(self.scene.addEllipse(x-13,y-13,26,26,QPen(Qt.PenStyle.NoPen),QColor("#222")),"material_slot_inner")
        self.static(self.scene.addRect(2392,1100,8,200,QPen(Qt.PenStyle.NoPen),QColor("#212121")),"qr_board")
        safe=QPen(QColor("#2e7d32"),3,Qt.PenStyle.DashLine); self.static(self.scene.addRect(150,150,2100,2100,safe),"car_center_boundary")
        self.text(720,105,"原料区","raw_label",26); self.text(180,1260,"暂存区","storage_label",24,90); self.text(1320,2180,"粗加工区","rough_label",26); self.text(2290,1300,"二维码板","coding_label",22,90); self.text(690,2420,"车体中心可移动范围：X/Y 150～2250 mm","movable_boundary_label",18,0,"#2e7d32")
        self.draw_dimensions()

    def draw_dimensions(self):
        blue=QColor("#1748c5"); pen=QPen(blue,2)
        def h(x1,x2,edge,dim,label,key):
            self.line(x1,edge,x1,dim,marker=key,pen=QPen(blue)); self.line(x2,edge,x2,dim,marker=key,pen=QPen(blue)); self.line(x1,dim,x2,dim,marker=key,pen=pen); self.text((x1+x2)/2-32,dim-32,label,key,18,0,"#1748c5")
        def v(y1,y2,edge,dim,label,key):
            self.line(edge,y1,dim,y1,marker=key,pen=QPen(blue)); self.line(edge,y2,dim,y2,marker=key,pen=QPen(blue)); self.line(dim,y1,dim,y2,marker=key,pen=pen); self.text(dim+8,(y1+y2)/2,label,key,18,90,"#1748c5")
        h(0,2400,2400,2530,"2400","dim_2400w"); v(0,2400,2400,2500,"2400","dim_2400h"); h(550,1000,1000,1060,"450","dim_platform_450"); v(550,1000,550,490,"450","dim_platform_450"); h(1000,1400,550,470,"400","dim_channel_400"); v(1000,1400,1400,1480,"400","dim_channel_400"); h(2100,2400,0,-100,"300","dim_start_300"); v(0,300,2400,2470,"300","dim_start_300"); h(0,150,0,-180,"150","dim_storage_150"); v(910,1490,0,-90,"580","dim_storage_580"); h(910,1490,2400,2620,"580","dim_rough_580"); v(2250,2400,1490,1570,"150","dim_rough_150"); h(1100,1300,0,-230,"1100～1300","dim_raw_range"); v(75,85,0,-170,"75～85","dim_raw_depth"); v(1100,1300,2400,2570,"1100～1300","dim_qr_range")

    def draw_start(self):
        if self.calibration_pending and self.calibration_stage in ("choose","position"): return
        item=StartItem(self.plan.start_heading_deg,self.rotate_start); item.setPos(self.plan.start_paper_x_mm,self.plan.start_paper_y_mm); self.scene.addItem(item)

    def draw_route(self):
        invalid=set(self.invalid_waypoints())
        previous=QPointF(self.plan.start_paper_x_mm,self.plan.start_paper_y_mm)
        for i,p in enumerate(self.plan.waypoints):
            paper=self.paper_of(p); current=QPointF(paper.x_mm,paper.y_mm); self.scene.addLine(previous.x(),previous.y(),current.x(),current.y(),QPen(QColor("#d27800"),8))
            mid=(previous+current)/2; badge=self.scene.addEllipse(mid.x()-18,mid.y()-18,36,36,QPen(QColor("#8a5500"),2),QColor("#ffffff")); badge.setZValue(4); label=self.scene.addText(str(i+1),QFont("Microsoft YaHei",14)); label.setPos(mid.x()-6,mid.y()-13); label.setZValue(5); previous=current
            item=WaypointItem(i,paper.x_mm,paper.y_mm,i==self.active_index,i in invalid,lambda index,before,after,shift=False:self.move_waypoint(index,before,after,shift),lambda handle: self.rotate_waypoint(handle.parentItem().index,handle),self.set_active_from_context); self.scene.addItem(item)
        if self.actual_trace:
            path=QPainterPath(QPointF(*world_to_paper(self.actual_trace[0],self.plan.start_paper_x_mm,self.plan.start_paper_y_mm,self.plan.start_heading_deg)))
            for p in self.actual_trace[1:]: path.lineTo(*world_to_paper(p,self.plan.start_paper_x_mm,self.plan.start_paper_y_mm,self.plan.start_heading_deg))
            self.scene.addPath(path,QPen(QColor("#9e1b32"),4,Qt.PenStyle.DashLine))

    def draw_car(self,pose):
        p=pose
        if p is None and 0 <= self.active_index < len(self.plan.waypoints):
            yaw=0.0
            for waypoint in self.plan.waypoints[:self.active_index+1]:
                if waypoint.use_yaw: yaw=waypoint.yaw_deg
            active=self.plan.waypoints[self.active_index]; p=Pose(active.x_mm,active.y_mm,yaw)
        p=p or Pose(); x,y=world_to_paper(p,self.plan.start_paper_x_mm,self.plan.start_paper_y_mm,self.plan.start_heading_deg); yaw=p.yaw_deg+self.plan.start_heading_deg
        out=not (150 <= x <= 2250 and 150 <= y <= 2250); car=QGraphicsRectItem(-150,-150,300,300); car.setPos(x,y); car.setRotation(-yaw); car.setPen(QPen(QColor("#c62828") if out else QColor("#455a64"),5)); car.setBrush(QColor(239,83,80,135) if out else QColor(120,144,156,105)); car.setZValue(15); self.scene.addItem(car)
        arrow=QGraphicsPolygonItem(QPolygonF([QPointF(-24,-24),QPointF(32,-24),QPointF(32,-44),QPointF(72,0),QPointF(32,44),QPointF(32,24),QPointF(-24,24)])); arrow.setPos(x,y); arrow.setRotation(-yaw); arrow.setBrush(QColor("#1565c0")); arrow.setPen(QPen(QColor("#0d47a1"),3)); arrow.setZValue(16); self.scene.addItem(arrow)

    def play(self):
        if self.calibration_pending: self.status.setText("请先完成起点标定。"); return
        invalid=self.invalid_waypoints()
        if invalid: self.status.setText("以下步骤超出车体中心可移动范围："+"、".join(str(i+1) for i in invalid)); return
        if not self.plan.waypoints: self.status.setText("至少添加一个节点后才能仿真。 "); return
        self.simulation=Simulation(copy.deepcopy(self.plan.waypoints),self.plan.settings,self.plan.start_paper_x_mm,self.plan.start_paper_y_mm,self.plan.start_heading_deg); self.actual_trace=[]; self.timer.start()
    def pause(self): self.timer.stop()
    def reset_simulation(self): self.timer.stop(); self.simulation=None; self.actual_trace=[]; self.redraw()
    def tick(self):
        if not self.simulation:return
        frame=self.simulation.step(); self.actual_trace.append(frame.actual); self.redraw(); self.status.setText(f"t={frame.time_s:.2f}s  速度={frame.speed_mm_s:.1f} mm/s  误差={frame.error_mm:.1f} mm")
        if self.simulation.finished or self.simulation.failed:
            self.timer.stop()
            if self.simulation.failed: self.status.setText(self.simulation.failure_reason)
    def refresh_plans(self): self.plan_list.clear(); self.plan_list.addItems(list_plans())
    def new_plan(self):
        self.pause(); self.plan=Plan(); self.active_index=-1; self.actual_trace=[]; self.undo_stack=[]; self.redo_stack=[]; self.selected_indices=set(); self.calibration_pending=True; self.calibration_stage="choose"; self.set_mode("select"); self.update_calibration_ui(); self.refresh_waypoints(); self.redraw()
    def save(self):
        if self.calibration_pending: self.status.setText("请先完成起点标定。"); return
        invalid=self.invalid_waypoints()
        if invalid: self.status.setText("无法保存，越界步骤："+"、".join(str(i+1) for i in invalid)); return
        try: save_plan(self.plan); self.refresh_plans(); self.status.setText(f"已保存：{self.plan.name}")
        except ValueError: self.save_as()
    def save_as(self):
        if self.calibration_pending or self.invalid_waypoints(): self.status.setText("完成标定并修正越界节点后才能保存。"); return
        name,ok=QInputDialog.getText(self,"另存方案","方案名称",text=self.plan.name)
        if ok and name:
            try: save_plan(self.plan,name); self.refresh_plans()
            except ValueError as error: QMessageBox.warning(self,"保存失败",str(error))
    def load_selected(self):
        item=self.plan_list.currentItem()
        if item is None:return
        try: self.plan=load_plan(item.text()); self.active_index=-1; self.calibration_pending=False; self.calibration_stage="complete"; self.update_calibration_ui(); self.refresh_waypoints(); self.redraw()
        except ValueError as error: QMessageBox.warning(self,"加载失败",str(error))


def main() -> int:
    app=QApplication(sys.argv); app.setStyleSheet("QWidget{font-family:'Microsoft YaHei';font-size:13px;} QPushButton{min-height:28px;} QScrollArea{border:0;}")
    window=PlannerWindow(); window.show(); return app.exec()

"""PySide6 图形化比赛地图编辑器。"""

from __future__ import annotations

import math
import sys

from PySide6.QtCore import QPointF, QTimer, Qt, Signal
from PySide6.QtGui import QColor, QPainter, QPainterPath, QPen, QPolygonF
from PySide6.QtWidgets import (QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QFormLayout,
                               QGraphicsEllipseItem, QGraphicsItem, QGraphicsPathItem, QGraphicsPolygonItem,
                               QGraphicsRectItem, QGraphicsScene, QGraphicsView, QHBoxLayout, QInputDialog,
                               QLabel, QListWidget, QMainWindow, QMessageBox, QPushButton, QSplitter,
                               QVBoxLayout, QWidget)

from .geometry import paper_to_world, sample_route, world_to_paper
from .models import CAR_SIZE_MM, FIELD_SIZE_MM, Plan, Pose, Segment, Waypoint
from .sim import Simulation
from .storage import list_plans, load_plan, save_plan


def spin(value: float = 0.0, minimum: float = -10000.0, maximum: float = 10000.0, step: float = 10.0) -> QDoubleSpinBox:
    widget = QDoubleSpinBox(); widget.setRange(minimum, maximum); widget.setDecimals(2); widget.setSingleStep(step); widget.setValue(value)
    return widget


class MapView(QGraphicsView):
    clicked = Signal(float, float)

    def __init__(self) -> None:
        super().__init__(); self.setRenderHint(QPainter.RenderHint.Antialiasing); self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)

    def wheelEvent(self, event):  # type: ignore[no-untyped-def]
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15
        self.scale(factor, factor)

    def mousePressEvent(self, event):  # type: ignore[no-untyped-def]
        if event.button() == Qt.MouseButton.LeftButton and self.itemAt(event.position().toPoint()) is None:
            point = self.mapToScene(event.position().toPoint()); self.clicked.emit(point.x(), point.y()); event.accept(); return
        super().mousePressEvent(event)


class WaypointItem(QGraphicsEllipseItem):
    """可拖动的途经点，释放时把图纸坐标回写给规划模型。"""

    def __init__(self, index: int, x: float, y: float, moved) -> None:  # type: ignore[no-untyped-def]
        super().__init__(-16, -16, 32, 32)
        self.index = index; self.moved = moved; self.setPos(x, y)
        self.setFlags(QGraphicsItem.GraphicsItemFlag.ItemIsMovable | QGraphicsItem.GraphicsItemFlag.ItemIsSelectable)
        self.setPen(QPen(QColor("#0c5d8a"), 3)); self.setBrush(QColor("#c5ecff")); self.setZValue(5)

    def mouseReleaseEvent(self, event):  # type: ignore[no-untyped-def]
        super().mouseReleaseEvent(event); point = self.scenePos(); self.moved(self.index, point.x(), point.y())


class PlannerWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__(); self.setWindowTitle("LittleCar2 比赛地图路径规划"); self.resize(1480, 920)
        self.plan = Plan(); self.mode = "waypoint"; self.simulation: Simulation | None = None; self.actual_trace: list[Pose] = []
        self.scene = QGraphicsScene(self); self.view = MapView(); self.view.setScene(self.scene); self.view.clicked.connect(self.on_map_click)
        self.timer = QTimer(self); self.timer.setInterval(20); self.timer.timeout.connect(self.tick)
        self._build(); self.redraw(); self.refresh_plans()

    def _build(self) -> None:
        root = QSplitter(Qt.Orientation.Horizontal); root.setChildrenCollapsible(False)
        left = QWidget(); controls = QVBoxLayout(left); controls.setContentsMargins(12, 12, 12, 12)
        controls.addWidget(QLabel("方案")); self.plan_list = QListWidget(); self.plan_list.itemDoubleClicked.connect(self.load_selected); controls.addWidget(self.plan_list)
        buttons = QHBoxLayout();
        for label, handler in (("新建", self.new_plan), ("保存", self.save), ("另存", self.save_as), ("加载", self.load_selected)):
            button = QPushButton(label); button.clicked.connect(handler); buttons.addWidget(button)
        controls.addLayout(buttons)
        controls.addWidget(QLabel("地图标定")); calibration = QFormLayout(); self.start_heading = spin(self.plan.start_heading_deg, -360, 360, 5)
        calibration.addRow("初始箭头图纸角度", self.start_heading); set_start = QPushButton("在地图标定起点"); set_start.clicked.connect(lambda: self.set_mode("start")); calibration.addRow(set_start); controls.addLayout(calibration)
        controls.addWidget(QLabel("路径点")); self.waypoint_list = QListWidget(); self.waypoint_list.currentRowChanged.connect(self.select_waypoint); controls.addWidget(self.waypoint_list)
        add_point = QPushButton("在地图添加途经点"); add_point.clicked.connect(lambda: self.set_mode("waypoint")); remove_point = QPushButton("删除选中点"); remove_point.clicked.connect(self.remove_waypoint); controls.addWidget(add_point); controls.addWidget(remove_point)
        form = QFormLayout(); self.x = spin(); self.y = spin(); self.yaw = spin(0, -360, 360, 5); self.stop = QCheckBox("在此停靠"); self.dwell = spin(0, 0, 120, .1)
        form.addRow("X (mm)", self.x); form.addRow("Y (mm)", self.y); form.addRow("航向 (deg)", self.yaw); form.addRow(self.stop); form.addRow("停留 (s)", self.dwell)
        update = QPushButton("更新选中点"); update.clicked.connect(self.update_waypoint); form.addRow(update); controls.addLayout(form)
        segment_form = QFormLayout(); self.segment_kind = QComboBox(); self.segment_kind.addItems(["bezier", "arc"]); self.handle = spin(180, 1, 2000, 10); self.radius = spin(300, 1, 4000, 10); self.clockwise = QCheckBox("顺时针圆弧")
        segment_form.addRow("到下一点的段", self.segment_kind); segment_form.addRow("贝塞尔手柄", self.handle); segment_form.addRow("圆弧半径", self.radius); segment_form.addRow(self.clockwise); apply_segment = QPushButton("更新路径段"); apply_segment.clicked.connect(self.update_segment); segment_form.addRow(apply_segment); controls.addLayout(segment_form)
        controls.addWidget(QLabel("仿真")); sim_form = QFormLayout(); self.vmax = spin(200, 1, 1500, 10); self.wmax = spin(90, 1, 180, 5); self.response = spin(.18, .01, 5, .01)
        sim_form.addRow("最大线速度", self.vmax); sim_form.addRow("最大角速度", self.wmax); sim_form.addRow("线速度响应(s)", self.response); controls.addLayout(sim_form)
        pid_form = QFormLayout(); self.kp_pos = spin(1.28, 0, 20, .05); self.ki_pos = spin(.13, 0, 20, .05); self.kd_pos = spin(.72, 0, 20, .05); self.kp_yaw = spin(1.65, 0, 20, .05); self.ki_yaw = spin(1.0, 0, 20, .05); self.kd_yaw = spin(.65, 0, 20, .05); self.yaw_response = spin(.14, .01, 5, .01); self.lookahead = spin(80, 1, 1000, 10)
        for label, widget in (("Kp 位置", self.kp_pos), ("Ki 位置", self.ki_pos), ("Kd 位置", self.kd_pos), ("Kp 航向", self.kp_yaw), ("Ki 航向", self.ki_yaw), ("Kd 航向", self.kd_yaw), ("航向响应(s)", self.yaw_response), ("前瞻距离", self.lookahead)): pid_form.addRow(label, widget)
        controls.addLayout(pid_form)
        playback = QHBoxLayout()
        for label, handler in (("播放", self.play), ("暂停", self.pause), ("重置", self.reset_simulation)):
            button = QPushButton(label); button.clicked.connect(handler); playback.addWidget(button)
        controls.addLayout(playback); self.status = QLabel("点击地图标定起点或添加途经点"); self.status.setWordWrap(True); controls.addWidget(self.status); controls.addStretch()
        root.addWidget(left); root.addWidget(self.view); root.setSizes([355, 1125]); self.setCentralWidget(root)

    def set_mode(self, mode: str) -> None:
        self.mode = mode; self.status.setText("请点击地图设置起点" if mode == "start" else "请点击地图添加途经点")

    def on_map_click(self, x: float, y: float) -> None:
        if not (0 <= x <= FIELD_SIZE_MM and 0 <= y <= FIELD_SIZE_MM): return
        if self.mode == "start":
            self.plan.start_paper_x_mm, self.plan.start_paper_y_mm = x, y; self.plan.start_heading_deg = self.start_heading.value(); self.status.setText("已更新世界坐标零点")
        else:
            pose = paper_to_world(x, y, self.plan.start_paper_x_mm, self.plan.start_paper_y_mm, self.start_heading.value())
            self.plan.waypoints.append(Waypoint(pose.x_mm, pose.y_mm, 0.0, name=f"点 {len(self.plan.waypoints) + 1}")); self.plan.normalize(); self.status.setText("已添加途经点")
        self.refresh_waypoints(); self.redraw()

    def refresh_waypoints(self) -> None:
        row = self.waypoint_list.currentRow(); self.waypoint_list.blockSignals(True); self.waypoint_list.clear()
        for index, point in enumerate(self.plan.waypoints): self.waypoint_list.addItem(f"{index + 1}. ({point.x_mm:.0f}, {point.y_mm:.0f})" + (" 停" if point.stop else ""))
        self.waypoint_list.blockSignals(False)
        if self.plan.waypoints: self.waypoint_list.setCurrentRow(min(max(row, 0), len(self.plan.waypoints) - 1))

    def select_waypoint(self, index: int) -> None:
        if index < 0 or index >= len(self.plan.waypoints): return
        point = self.plan.waypoints[index]; self.x.setValue(point.x_mm); self.y.setValue(point.y_mm); self.yaw.setValue(point.yaw_deg); self.stop.setChecked(point.stop); self.dwell.setValue(point.dwell_s)
        if index < len(self.plan.segments):
            segment = self.plan.segments[index]; self.segment_kind.setCurrentText(segment.kind); self.handle.setValue(segment.handle_length_mm); self.radius.setValue(segment.arc_radius_mm); self.clockwise.setChecked(segment.clockwise)

    def update_waypoint(self) -> None:
        index = self.waypoint_list.currentRow()
        if index < 0: return
        point = self.plan.waypoints[index]; point.x_mm, point.y_mm, point.yaw_deg, point.stop, point.dwell_s = self.x.value(), self.y.value(), self.yaw.value(), self.stop.isChecked(), self.dwell.value()
        self.refresh_waypoints(); self.redraw()

    def update_segment(self) -> None:
        index = self.waypoint_list.currentRow()
        if not 0 <= index < len(self.plan.segments): self.status.setText("请选择含有下一点的途经点"); return
        self.plan.segments[index] = Segment(self.segment_kind.currentText(), self.handle.value(), self.radius.value(), self.clockwise.isChecked()); self.redraw()

    def remove_waypoint(self) -> None:
        index = self.waypoint_list.currentRow()
        if index < 0: return
        del self.plan.waypoints[index]; self.plan.normalize(); self.refresh_waypoints(); self.redraw()

    def redraw(self) -> None:
        self.scene.clear(); self.scene.setSceneRect(-250, -250, 2900, 2900); self._draw_field(); self._draw_start(); self._draw_route(); self._draw_car(self.simulation.actual if self.simulation else None)

    def _draw_field(self) -> None:
        self.scene.addRect(0, 0, FIELD_SIZE_MM, FIELD_SIZE_MM, QPen(QColor("#666666"), 5), QColor("#dcdcdc"))
        for x, y in ((550, 500), (1400, 500), (550, 1450), (1400, 1450)): self.scene.addRect(x, y, 450, 450, QPen(Qt.PenStyle.NoPen), QColor("#fffce2"))
        self.scene.addLine(1200, 0, 1200, 2400, QPen(QColor("#5b5b5b"), 3, Qt.PenStyle.DashLine)); self.scene.addLine(0, 1200, 2400, 1200, QPen(QColor("#5b5b5b"), 3, Qt.PenStyle.DashLine))
        for x, y, name in ((2250, 150, "启停区1"), (2250, 2250, "启停区2")):
            self.scene.addRect(x - 150, y - 150, 300, 300, QPen(Qt.PenStyle.NoPen), QColor("#1239d6")); label = self.scene.addText(name); label.setDefaultTextColor(QColor("#202020")); label.setPos(x - 100, y + 165)
        for x in range(0, 2401, 200): self.scene.addLine(x, 0, x, 2400, QPen(QColor(0, 0, 0, 25), 1))
        for y in range(0, 2401, 200): self.scene.addLine(0, y, 2400, y, QPen(QColor(0, 0, 0, 25), 1))

    def _draw_start(self) -> None:
        x, y = self.plan.start_paper_x_mm, self.plan.start_paper_y_mm; self.scene.addEllipse(x - 12, y - 12, 24, 24, QPen(QColor("#1256a8"), 3), QColor("#ffffff"))

    def _paper_pose(self, pose: Pose) -> Pose:
        x, y = world_to_paper(pose, self.plan.start_paper_x_mm, self.plan.start_paper_y_mm, self.start_heading.value()); return Pose(x, y, pose.yaw_deg + self.start_heading.value())

    def _draw_route(self) -> None:
        points, _, errors = sample_route(self.plan.waypoints, self.plan.segments)
        if points:
            path = QPainterPath(QPointF(*world_to_paper(points[0], self.plan.start_paper_x_mm, self.plan.start_paper_y_mm, self.start_heading.value())))
            for point in points[1:]: path.lineTo(*world_to_paper(point, self.plan.start_paper_x_mm, self.plan.start_paper_y_mm, self.start_heading.value()))
            self.scene.addPath(path, QPen(QColor("#d27800"), 12))
        for index, point in enumerate(self.plan.waypoints):
            paper = self._paper_pose(Pose(point.x_mm, point.y_mm, point.yaw_deg)); item = WaypointItem(index, paper.x_mm, paper.y_mm, self.move_waypoint); self.scene.addItem(item); item.setToolTip(f"点 {index + 1}: ({point.x_mm:.1f}, {point.y_mm:.1f})")
        if self.actual_trace:
            path = QPainterPath(QPointF(*world_to_paper(self.actual_trace[0], self.plan.start_paper_x_mm, self.plan.start_paper_y_mm, self.start_heading.value())))
            for point in self.actual_trace[1:]: path.lineTo(*world_to_paper(point, self.plan.start_paper_x_mm, self.plan.start_paper_y_mm, self.start_heading.value()))
            self.scene.addPath(path, QPen(QColor("#9e1b32"), 5, Qt.PenStyle.DashLine))
        self.status.setText("；".join(errors) if errors else self.status.text())

    def move_waypoint(self, index: int, paper_x: float, paper_y: float) -> None:
        """拖动节点后保持世界航向和停靠属性，仅更新其世界坐标。"""
        if not 0 <= index < len(self.plan.waypoints): return
        pose = paper_to_world(paper_x, paper_y, self.plan.start_paper_x_mm, self.plan.start_paper_y_mm, self.start_heading.value())
        point = self.plan.waypoints[index]; point.x_mm, point.y_mm = pose.x_mm, pose.y_mm
        self.refresh_waypoints(); self.waypoint_list.setCurrentRow(index); self.redraw(); self.status.setText(f"已移动途经点 {index + 1}")

    def _draw_car(self, actual: Pose | None) -> None:
        pose = actual or Pose(); paper = self._paper_pose(pose); side = CAR_SIZE_MM
        rect = QGraphicsRectItem(-side / 2, -side / 2, side, side); rect.setPen(QPen(QColor("#b00020"), 5)); rect.setBrush(QColor(230, 40, 40, 128)); rect.setPos(paper.x_mm, paper.y_mm); rect.setRotation(-paper.yaw_deg); self.scene.addItem(rect)
        arrow = QGraphicsPolygonItem(QPolygonF([QPointF(0, -side / 2 - 40), QPointF(-30, -side / 2 + 25), QPointF(30, -side / 2 + 25)])); arrow.setBrush(QColor("#1267c5")); arrow.setPen(QPen(QColor("#1267c5"))); arrow.setPos(paper.x_mm, paper.y_mm); arrow.setRotation(-paper.yaw_deg); self.scene.addItem(arrow)

    def play(self) -> None:
        points, stops, errors = sample_route(self.plan.waypoints, self.plan.segments)
        if errors or len(points) < 2: self.status.setText("无法仿真：" + ("；".join(errors) if errors else "至少需要两个途经点")); return
        settings = self.plan.settings; settings.vmax_mm_s, settings.wmax_deg_s, settings.linear_response_s = self.vmax.value(), self.wmax.value(), self.response.value()
        settings.kp_pos, settings.ki_pos, settings.kd_pos = self.kp_pos.value(), self.ki_pos.value(), self.kd_pos.value()
        settings.kp_yaw, settings.ki_yaw, settings.kd_yaw = self.kp_yaw.value(), self.ki_yaw.value(), self.kd_yaw.value()
        settings.yaw_response_s, settings.lookahead_mm = self.yaw_response.value(), self.lookahead.value()
        if self.simulation is None: self.simulation = Simulation(points, stops, settings, self.plan.start_paper_x_mm, self.plan.start_paper_y_mm, self.start_heading.value()); self.actual_trace = []
        self.timer.start()

    def pause(self) -> None: self.timer.stop()

    def reset_simulation(self) -> None:
        self.timer.stop(); self.simulation = None; self.actual_trace = []; self.redraw(); self.status.setText("已重置仿真")

    def tick(self) -> None:
        if self.simulation is None: return
        frame = self.simulation.step(); self.actual_trace.append(frame.actual); self.redraw(); self.status.setText(f"t={frame.time_s:.2f}s 速度={frame.speed_mm_s:.1f}mm/s 误差={frame.error_mm:.1f}mm" + (" 越界" if frame.out_of_bounds else ""))
        if self.simulation.finished: self.timer.stop()

    def refresh_plans(self) -> None:
        self.plan_list.clear(); self.plan_list.addItems(list_plans())

    def new_plan(self) -> None:
        self.pause(); self.plan = Plan(); self.start_heading.setValue(self.plan.start_heading_deg); self.actual_trace = []; self.simulation = None; self.refresh_waypoints(); self.redraw()

    def save(self) -> None:
        try: save_plan(self.plan); self.refresh_plans(); self.status.setText(f"已保存：{self.plan.name}")
        except ValueError: self.save_as()

    def save_as(self) -> None:
        name, ok = QInputDialog.getText(self, "另存方案", "方案名称", text=self.plan.name)
        if ok and name:
            try: save_plan(self.plan, name); self.refresh_plans(); self.status.setText(f"已保存：{name}")
            except ValueError as error: QMessageBox.warning(self, "保存失败", str(error))

    def load_selected(self) -> None:
        item = self.plan_list.currentItem()
        if item is None: return
        try:
            self.plan = load_plan(item.text()); self.start_heading.setValue(self.plan.start_heading_deg); self.simulation = None; self.actual_trace = []; self.refresh_waypoints(); self.redraw(); self.status.setText(f"已加载：{self.plan.name}")
        except ValueError as error: QMessageBox.warning(self, "加载失败", str(error))


def main() -> int:
    app = QApplication(sys.argv); app.setStyleSheet("QWidget{font-family:'Microsoft YaHei';} QPushButton{min-height:28px;} QListWidget{min-height:90px;}")
    window = PlannerWindow(); window.show(); return app.exec()

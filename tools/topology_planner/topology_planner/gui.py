"""拓扑路径规划工具的 PySide6 图形界面。"""

from __future__ import annotations

import sys
import time
from typing import Callable, Iterable

from PySide6.QtCore import QPointF, QRectF, QTimer, Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPainterPath, QPainterPathStroker, QPen
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGraphicsEllipseItem,
    QGraphicsItem,
    QGraphicsLineItem,
    QGraphicsPathItem,
    QGraphicsScene,
    QGraphicsTextItem,
    QGraphicsView,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QMessageBox,
    QProgressBar,
    QPushButton,
    QScrollArea,
    QSplitter,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from .mission import MissionPlanningError, build_fixed_mission_plan
from .planner import Edge, PathResult, edge_key, edges, find_best_paths, nodes
from .simulation import MissionSimulator, SimulationPhase


SCALE = 220.0
MARGIN = 150.0
START_COLOR = "#b04cff"
GOAL_COLOR = "#e08aff"


def scene_point(node_id: str) -> QPointF:
    node = nodes[node_id]
    return QPointF(MARGIN + node.x * SCALE, MARGIN + node.y * SCALE)


def display_name(node_id: str) -> str:
    node = nodes[node_id]
    return f"{node.label}（{node.id}）"


class EdgeItem(QGraphicsLineItem):
    """可点击的拓扑边，点击后切换禁用状态。"""

    def __init__(self, edge: Edge, clicked: Callable[[tuple[str, str]], None]) -> None:
        first, second = scene_point(edge.node_a), scene_point(edge.node_b)
        super().__init__(first.x(), first.y(), second.x(), second.y())
        self.edge = edge
        self._clicked = clicked
        self.setAcceptedMouseButtons(Qt.MouseButton.LeftButton)
        self.setZValue(2)
        self.setToolTip(f"左键切换道路启用状态：{edge.node_a} - {edge.node_b}")
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable, False)

    def shape(self):  # type: ignore[no-untyped-def]
        path = QPainterPath()
        path.moveTo(self.line().p1())
        path.lineTo(self.line().p2())
        stroker = QPainterPathStroker()
        stroker.setWidth(24.0)
        return stroker.createStroke(path)

    def mousePressEvent(self, event):  # type: ignore[no-untyped-def]
        if event.button() == Qt.MouseButton.LeftButton:
            self._clicked(edge_key(self.edge.node_a, self.edge.node_b))
            event.accept()
            return
        super().mousePressEvent(event)


class NodeItem(QGraphicsEllipseItem):
    """支持左、右键设置起点和终点的拓扑节点图元。"""

    def __init__(
        self,
        node_id: str,
        label: str,
        radius: float,
        set_start: Callable[[str], None],
        set_goal: Callable[[str], None],
    ) -> None:
        super().__init__(-radius, -radius, radius * 2, radius * 2)
        self.node_id = node_id
        self._set_start = set_start
        self._set_goal = set_goal
        self.setBrush(QColor("#303b46"))
        self.setPen(QPen(QColor("#f2f2f2"), 2.0))
        self.setZValue(10)
        self.setAcceptedMouseButtons(Qt.MouseButton.LeftButton | Qt.MouseButton.RightButton)
        self.setToolTip("左键：设为起始点\n右键：设为终止点")

        self.start_ring = QGraphicsEllipseItem(
            -radius - 8, -radius - 8, (radius + 8) * 2, (radius + 8) * 2, self
        )
        self.start_ring.setBrush(Qt.BrushStyle.NoBrush)
        self.start_ring.setPen(QPen(QColor(START_COLOR), 5.0, Qt.PenStyle.SolidLine))
        self.start_ring.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
        self.start_ring.setVisible(False)

        self.goal_ring = QGraphicsEllipseItem(
            -radius - 15, -radius - 15, (radius + 15) * 2, (radius + 15) * 2, self
        )
        self.goal_ring.setBrush(Qt.BrushStyle.NoBrush)
        self.goal_ring.setPen(QPen(QColor(GOAL_COLOR), 5.0, Qt.PenStyle.DashLine))
        self.goal_ring.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
        self.goal_ring.setVisible(False)

        self.label_item = QGraphicsTextItem(f"{label}\n{node_id}", self)
        self.label_item.setDefaultTextColor(QColor("#ffffff"))
        self.label_item.setFont(QFont("Microsoft YaHei", 9 if len(label) <= 2 else 8))
        self.label_item.setTextWidth(radius * 1.8)
        bounds = self.label_item.boundingRect()
        self.label_item.setPos(-bounds.width() / 2, -bounds.height() / 2)
        self.label_item.setAcceptedMouseButtons(Qt.MouseButton.NoButton)

    def set_role(self, is_start: bool, is_goal: bool) -> None:
        self.start_ring.setVisible(is_start)
        self.goal_ring.setVisible(is_goal)

    def mousePressEvent(self, event):  # type: ignore[no-untyped-def]
        if event.button() == Qt.MouseButton.LeftButton:
            self._set_start(self.node_id)
            event.accept()
            return
        if event.button() == Qt.MouseButton.RightButton:
            self._set_goal(self.node_id)
            event.accept()
            return
        super().mousePressEvent(event)


class PlannerWindow(QMainWindow):
    """单段路径规划与固定任务链离线演示窗口。"""

    _SINGLE_TAB = 0
    _MISSION_TAB = 1

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("LittleCar2 拓扑路径规划")
        self.resize(1440, 900)
        self.setMinimumSize(1100, 760)
        self.blocked_edges: set[tuple[str, str]] = set()
        self.results: list[PathResult] = []
        self.selected_result = 0
        self.route_items: list[QGraphicsItem] = []
        self.mission_route_items: list[QGraphicsItem] = []
        self.simulation_items: list[QGraphicsItem] = []
        self.mission_plan = None
        self.mission_simulator = None
        self._last_animation_time: float | None = None

        self.scene = QGraphicsScene(self)
        self.view = QGraphicsView(self.scene)
        self.view.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.view.setBackgroundBrush(QColor("#242424"))
        self.view.setDragMode(QGraphicsView.DragMode.NoDrag)
        self._build_scene()
        self._create_vehicle_item()

        self.start_combo = QComboBox()
        self.goal_combo = QComboBox()
        for node_id in nodes:
            self.start_combo.addItem(display_name(node_id), node_id)
            self.goal_combo.addItem(display_name(node_id), node_id)
        self.start_combo.setCurrentIndex(self.start_combo.findData("START1"))
        self.goal_combo.setCurrentIndex(self.goal_combo.findData("ROUGH"))
        self.start_combo.currentIndexChanged.connect(self.replan)
        self.goal_combo.currentIndexChanged.connect(self.replan)

        self.distance_weight = self._weight_spin(1.0)
        self.turn_weight = self._weight_spin(0.75)
        self.stop_weight = self._weight_spin(1.0)
        for spin in (self.distance_weight, self.turn_weight, self.stop_weight):
            spin.valueChanged.connect(self._replan_after_cost_change)

        self.path_list = QListWidget()
        self.path_list.currentRowChanged.connect(self._select_result)
        self.status_label = QLabel()
        self.status_label.setWordWrap(True)
        self.disabled_label = QLabel()
        self._build_controls()

        self.animation_timer = QTimer(self)
        self.animation_timer.setInterval(30)
        self.animation_timer.timeout.connect(self._advance_mission_animation)
        self.replan()
        self._refresh_mission_controls()

    @staticmethod
    def _weight_spin(value: float) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(0.0, 100.0)
        spin.setSingleStep(0.05)
        spin.setDecimals(2)
        spin.setValue(value)
        return spin

    def _build_scene(self) -> None:
        self.scene.setSceneRect(QRectF(0, 0, MARGIN * 2 + 3 * SCALE, MARGIN * 2 + 2 * SCALE))
        self.edge_items: dict[tuple[str, str], EdgeItem] = {}
        for edge in edges:
            item = EdgeItem(edge, self._toggle_edge)
            self.edge_items[edge_key(edge.node_a, edge.node_b)] = item
            self.scene.addItem(item)
        self.node_items: dict[str, NodeItem] = {}
        for node_id, node in nodes.items():
            radius = 34.0 if node.kind == "navigation" else 48.0
            point = scene_point(node_id)
            item = NodeItem(node_id, node.label, radius, self.set_start_node, self.set_goal_node)
            item.setPos(point)
            item.setBrush(QColor("#303b46") if node.kind == "navigation" else QColor("#6a4b32"))
            self.scene.addItem(item)
            self.node_items[node_id] = item

    def _create_vehicle_item(self) -> None:
        self.vehicle_item = QGraphicsEllipseItem(-16, -16, 32, 32)
        self.vehicle_item.setBrush(QColor("#ffcc33"))
        self.vehicle_item.setPen(QPen(QColor("#1a1a1a"), 2.0))
        self.vehicle_item.setZValue(20)
        self.vehicle_item.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
        self.vehicle_item.setVisible(False)
        self.scene.addItem(self.vehicle_item)
        self.simulation_items.append(self.vehicle_item)
        self.vehicle_text = QGraphicsTextItem("车", self.vehicle_item)
        self.vehicle_text.setDefaultTextColor(QColor("#101010"))
        self.vehicle_text.setFont(QFont("Microsoft YaHei", 9, QFont.Weight.Bold))
        self.vehicle_text.setPos(-8, -12)
        self.vehicle_text.setAcceptedMouseButtons(Qt.MouseButton.NoButton)

    def _build_controls(self) -> None:
        self.tabs = QTabWidget()
        self.tabs.addTab(self._build_single_panel(), "单段路径")
        self.tabs.addTab(self._build_mission_panel(), "完整任务链")
        self.tabs.currentChanged.connect(self._change_mode)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(self.tabs)
        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(scroll)
        splitter.addWidget(self.view)
        splitter.setSizes([360, 840])
        self.setCentralWidget(splitter)

    def _build_single_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(12)

        endpoint_form = QFormLayout()
        endpoint_form.addRow("起始节点", self.start_combo)
        endpoint_form.addRow("目标节点", self.goal_combo)
        layout.addLayout(endpoint_form)
        swap = QPushButton("交换起终点")
        swap.clicked.connect(self._swap_endpoints)
        layout.addWidget(swap)

        weight_form = QFormLayout()
        weight_form.addRow("A 路径长度权重", self.distance_weight)
        weight_form.addRow("B 每次转向代价", self.turn_weight)
        weight_form.addRow("C 每次停车代价", self.stop_weight)
        layout.addLayout(weight_form)
        layout.addWidget(QLabel("总成本 = A x 路径长度 + B x 转向单位 + C x 停车次数"))
        layout.addWidget(QLabel("导航边长度：1.0；任务边长度：0.5"))

        restore = QPushButton("恢复全部道路")
        restore.clicked.connect(self._restore_edges)
        layout.addWidget(restore)
        layout.addWidget(self.disabled_label)
        layout.addWidget(QLabel("候选路径（最多 4 条）"))
        layout.addWidget(self.path_list, 1)
        layout.addWidget(self.status_label)
        return panel

    def _build_mission_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(12)

        self.mission_start_combo = QComboBox()
        for node_id in ("START1", "START2"):
            self.mission_start_combo.addItem(display_name(node_id), node_id)
        self.mission_start_combo.currentIndexChanged.connect(self._mission_start_changed)
        self.mission_start_combo.currentIndexChanged.connect(
            lambda: self._invalidate_mission_plan("起始区已变更，请重新生成任务计划")
        )
        form = QFormLayout()
        form.addRow("起始区", self.mission_start_combo)
        layout.addLayout(form)

        self.generate_mission_button = QPushButton("生成任务计划")
        self.generate_mission_button.clicked.connect(self._generate_mission_plan)
        layout.addWidget(self.generate_mission_button)

        self.mission_status_label = QLabel("尚未生成任务计划")
        self.mission_status_label.setWordWrap(True)
        layout.addWidget(self.mission_status_label)
        self.mission_detail_label = QLabel("当前尚无仿真状态")
        self.mission_detail_label.setWordWrap(True)
        layout.addWidget(self.mission_detail_label)
        layout.addWidget(QLabel("任务段（固定 8 段）"))
        self.mission_list = QListWidget()
        layout.addWidget(self.mission_list, 1)

        self.mission_progress = QProgressBar()
        self.mission_progress.setRange(0, 1000)
        self.mission_progress.setValue(0)
        self.mission_progress.setFormat("0%")
        layout.addWidget(self.mission_progress)

        actions = QHBoxLayout()
        self.play_mission_button = QPushButton("播放")
        self.pause_mission_button = QPushButton("暂停")
        self.stop_mission_button = QPushButton("停止并复位")
        self.play_mission_button.clicked.connect(self._play_mission)
        self.pause_mission_button.clicked.connect(self._pause_or_resume_mission)
        self.stop_mission_button.clicked.connect(self._stop_and_reset_mission)
        actions.addWidget(self.play_mission_button)
        actions.addWidget(self.pause_mission_button)
        actions.addWidget(self.stop_mission_button)
        layout.addLayout(actions)

        self.speed_combo = QComboBox()
        for multiplier in (0.5, 1.0, 2.0, 4.0):
            self.speed_combo.addItem(f"{multiplier:g} 倍速", multiplier)
        self.speed_combo.setCurrentIndex(self.speed_combo.findData(1.0))
        self.speed_combo.currentIndexChanged.connect(self._set_mission_speed)
        speed_form = QFormLayout()
        speed_form.addRow("播放速度", self.speed_combo)
        layout.addLayout(speed_form)
        return panel

    def _change_mode(self, index: int) -> None:
        if index == self._SINGLE_TAB:
            self._pause_mission_for_mode_change()
            self._clear_mission_routes()
            self.vehicle_item.setVisible(False)
            self._draw_routes()
        elif index == self._MISSION_TAB:
            self._clear_routes()
            if self.mission_plan is not None:
                self._draw_mission_route()
                self._update_mission_display()

    def _pause_mission_for_mode_change(self) -> None:
        if self.mission_simulator is None:
            return
        snapshot = self.mission_simulator.snapshot()
        if snapshot.running:
            self.mission_simulator.pause()
        self.animation_timer.stop()
        self._last_animation_time = None
        self._refresh_mission_controls()

    def _toggle_edge(self, key: tuple[str, str]) -> None:
        if key in self.blocked_edges:
            self.blocked_edges.remove(key)
        else:
            self.blocked_edges.add(key)
        self.replan()
        self._invalidate_mission_plan("道路配置已变更，请重新生成任务计划")

    def _mission_start_changed(self) -> None:
        self._invalidate_mission_plan("起始启停区已变更，请重新生成任务计划")

    def set_start_node(self, node_id: str) -> None:
        if self.tabs.currentIndex() == self._MISSION_TAB:
            return
        index = self.start_combo.findData(node_id)
        self.start_combo.blockSignals(True)
        self.start_combo.setCurrentIndex(index)
        self.start_combo.blockSignals(False)
        self.replan()

    def set_goal_node(self, node_id: str) -> None:
        if self.tabs.currentIndex() == self._MISSION_TAB:
            return
        index = self.goal_combo.findData(node_id)
        self.goal_combo.blockSignals(True)
        self.goal_combo.setCurrentIndex(index)
        self.goal_combo.blockSignals(False)
        self.replan()

    def _restore_edges(self) -> None:
        self.blocked_edges.clear()
        self.replan()
        self._invalidate_mission_plan("道路配置已恢复，请重新生成任务计划")

    def _swap_endpoints(self) -> None:
        start = self.start_combo.currentData()
        goal = self.goal_combo.currentData()
        self.start_combo.setCurrentIndex(self.start_combo.findData(goal))
        self.goal_combo.setCurrentIndex(self.goal_combo.findData(start))
        self.replan()

    def _replan_after_cost_change(self) -> None:
        self.replan()
        self._invalidate_mission_plan("成本权重已变更，请重新生成任务计划")

    def _select_result(self, row: int) -> None:
        if 0 <= row < len(self.results):
            self.selected_result = row
            if self.tabs.currentIndex() == self._SINGLE_TAB:
                self._draw_routes()

    def replan(self) -> None:
        start_id = self.start_combo.currentData()
        goal_id = self.goal_combo.currentData()
        previous_nodes = (
            self.results[self.selected_result].nodes
            if self.results and self.selected_result < len(self.results)
            else None
        )
        self.results = find_best_paths(
            start_id, goal_id, self.blocked_edges,
            self.distance_weight.value(), self.turn_weight.value(), self.stop_weight.value(), 4,
        )
        self.selected_result = next(
            (index for index, result in enumerate(self.results) if result.nodes == previous_nodes), 0
        )
        self.path_list.blockSignals(True)
        self.path_list.clear()
        for index, result in enumerate(self.results):
            self.path_list.addItem(QListWidgetItem(self._format_result(index, result)))
        if self.results:
            self.path_list.setCurrentRow(self.selected_result)
        self.path_list.blockSignals(False)
        self.disabled_label.setText(f"已禁用道路：{len(self.blocked_edges)} 条")
        self.status_label.setText(
            f"当前：{display_name(start_id)} → {display_name(goal_id)}\n"
            + (f"可用候选路径：{len(self.results)} 条" if self.results else "当前道路配置下不存在可通行路径。")
        )
        self._refresh_edge_colors()
        for node_id, item in self.node_items.items():
            item.set_role(node_id == start_id, node_id == goal_id)
        if self.tabs.currentIndex() == self._SINGLE_TAB:
            self._draw_routes()

    @staticmethod
    def _format_result(index: int, result: PathResult) -> str:
        rank = "最优" if index == 0 else f"次优 {index}"
        path = " → ".join(result.nodes)
        return (
            f"{index + 1}  {rank}  成本 {result.total_cost:.2f}\n"
            f"{path}\n"
            f"长度 {result.distance:.2f} | 转向 {result.quarter_turns} | 停车 {result.stops}"
        )

    def _refresh_edge_colors(self) -> None:
        for key, item in self.edge_items.items():
            color = "#d04444" if key in self.blocked_edges else "#f2f2f2"
            item.setPen(QPen(QColor(color), 9.0))

    def _clear_routes(self) -> None:
        for item in self.route_items:
            self.scene.removeItem(item)
        self.route_items.clear()

    def _draw_routes(self) -> None:
        self._clear_routes()
        for index, result in reversed(list(enumerate(self.results))):
            color = "#31d17c" if index == self.selected_result else "#3f8cff"
            pen = QPen(QColor(color), 5.0)
            pen.setCapStyle(Qt.PenCapStyle.RoundCap)
            path = self._route_path(result.nodes, (index - self.selected_result) * 4.0)
            item = QGraphicsPathItem(path)
            item.setPen(pen)
            item.setZValue(3)
            item.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
            self.scene.addItem(item)
            self.route_items.append(item)
        if self.results:
            self._draw_step_markers(self.results[self.selected_result].nodes)

    def _draw_mission_route(self) -> None:
        self._clear_mission_routes()
        if self.mission_plan is None:
            return
        nodes_in_route = tuple(self.mission_plan.flattened_nodes)
        if len(nodes_in_route) < 2:
            return
        item = QGraphicsPathItem(self._route_path(nodes_in_route, 0.0))
        pen = QPen(QColor("#31d17c"), 6.0)
        pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        item.setPen(pen)
        item.setZValue(3)
        item.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
        self.scene.addItem(item)
        self.mission_route_items.append(item)
        self._draw_step_markers(nodes_in_route, self.mission_route_items)

    @staticmethod
    def _route_path(node_ids: Iterable[str], offset: float) -> QPainterPath:
        node_ids = tuple(node_ids)
        path = QPainterPath()
        for segment, (first, second) in enumerate(zip(node_ids, node_ids[1:])):
            p1, p2 = scene_point(first), scene_point(second)
            dx, dy = p2.x() - p1.x(), p2.y() - p1.y()
            length = max((dx * dx + dy * dy) ** 0.5, 1.0)
            ox, oy = -dy / length * offset, dx / length * offset
            if segment == 0:
                path.moveTo(p1.x() + ox, p1.y() + oy)
            path.lineTo(p2.x() + ox, p2.y() + oy)
        return path

    def _draw_step_markers(
        self, node_ids: Iterable[str], target: list[QGraphicsItem] | None = None
    ) -> None:
        target = self.route_items if target is None else target
        for step, node_id in enumerate(node_ids, 1):
            point = scene_point(node_id)
            marker = QGraphicsEllipseItem(-11, -11, 22, 22)
            marker.setPos(point)
            marker.setBrush(QColor("#31d17c"))
            marker.setPen(QPen(QColor("#ffffff"), 1.0))
            marker.setZValue(6)
            marker.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
            self.scene.addItem(marker)
            target.append(marker)
            label = QGraphicsTextItem(str(step), marker)
            label.setDefaultTextColor(QColor("#101010"))
            label.setFont(QFont("Arial", 8, QFont.Weight.Bold))
            label.setPos(-4, -8)
            label.setAcceptedMouseButtons(Qt.MouseButton.NoButton)

    def _clear_mission_routes(self) -> None:
        for item in self.mission_route_items:
            self.scene.removeItem(item)
        self.mission_route_items.clear()

    def _generate_mission_plan(self) -> None:
        self._pause_mission_for_mode_change()
        start_zone = self.mission_start_combo.currentData()
        try:
            plan = build_fixed_mission_plan(
                start_zone,
                self.blocked_edges,
                self.distance_weight.value(),
                self.turn_weight.value(),
                self.stop_weight.value(),
            )
        except MissionPlanningError as error:
            self._clear_mission_plan(
                f"第 {error.leg_index + 1} 段不可达：{error.start_id} → {error.goal_id}\n"
                f"原因：{error.message}",
                notify=True,
            )
            return
        except ValueError as error:
            self._clear_mission_plan(f"无法生成任务计划：{error}", notify=True)
            return
        if plan is None or not getattr(plan, "legs", ()):
            self._clear_mission_plan("当前道路配置下任务链不可达。", notify=True)
            return
        self.mission_plan = plan
        self.mission_simulator = MissionSimulator(plan, base_speed=1.0)
        self.mission_simulator.set_speed_multiplier(float(self.speed_combo.currentData()))
        self._last_animation_time = None
        self._populate_mission_list()
        self.mission_status_label.setText(f"任务计划已生成：{len(plan.legs)} 段，等待播放")
        self._draw_mission_route()
        self._update_mission_display()
        self._refresh_mission_controls()

    def _populate_mission_list(self) -> None:
        self.mission_list.clear()
        if self.mission_plan is None:
            return
        for index, leg in enumerate(self.mission_plan.legs, 1):
            path = " → ".join(leg.path.nodes)
            self.mission_list.addItem(
                f"{index}. {path}\n"
                f"{leg.goal_stop.action_label} | 长度 {leg.path.distance:.2f} | "
                f"转向 {leg.path.quarter_turns} | 停车 {leg.path.stops} | "
                f"成本 {leg.path.total_cost:.2f}"
            )

    def _clear_mission_plan(self, reason: str, notify: bool = False) -> None:
        self.animation_timer.stop()
        self._last_animation_time = None
        self.mission_plan = None
        self.mission_simulator = None
        self.mission_list.clear()
        self.mission_progress.setValue(0)
        self.mission_progress.setFormat("0%")
        self.mission_status_label.setText(reason)
        self.mission_detail_label.setText("当前尚无仿真状态")
        self.vehicle_item.setVisible(False)
        self._clear_mission_routes()
        self._refresh_mission_controls()
        if notify:
            QMessageBox.warning(self, "任务链不可达", reason)

    def _invalidate_mission_plan(self, reason: str) -> None:
        if self.mission_plan is not None:
            self._clear_mission_plan(reason)

    def _play_mission(self) -> None:
        if self.mission_simulator is None:
            return
        snapshot = self.mission_simulator.snapshot()
        if snapshot.paused:
            self.mission_simulator.resume()
        elif not snapshot.running:
            self.mission_simulator.start()
        self._last_animation_time = time.monotonic()
        self.animation_timer.start()
        self._update_mission_display()
        self._refresh_mission_controls()

    def _pause_or_resume_mission(self) -> None:
        if self.mission_simulator is None:
            return
        if self.mission_simulator.snapshot().running:
            self.mission_simulator.pause()
            self.animation_timer.stop()
            self._last_animation_time = None
        else:
            self.mission_simulator.resume()
            self._last_animation_time = time.monotonic()
            self.animation_timer.start()
        self._update_mission_display()
        self._refresh_mission_controls()

    def _stop_and_reset_mission(self) -> None:
        if self.mission_simulator is None:
            return
        self.animation_timer.stop()
        self.mission_simulator.stop()
        self.mission_simulator.reset()
        self._last_animation_time = None
        self._update_mission_display()
        self._refresh_mission_controls()

    def _set_mission_speed(self) -> None:
        if self.mission_simulator is not None:
            self.mission_simulator.set_speed_multiplier(float(self.speed_combo.currentData()))

    def _advance_mission_animation(self) -> None:
        if self.mission_simulator is None:
            self.animation_timer.stop()
            return
        now = time.monotonic()
        if self._last_animation_time is None:
            self._last_animation_time = now
            return
        self.mission_simulator.tick(max(now - self._last_animation_time, 0.0))
        self._last_animation_time = now
        self._update_mission_display()
        if not self.mission_simulator.snapshot().running:
            self.animation_timer.stop()
            self._last_animation_time = None
        self._refresh_mission_controls()

    def _update_mission_display(self) -> None:
        if self.mission_simulator is None:
            return
        snapshot = self.mission_simulator.snapshot()
        progress = min(max(snapshot.total_progress, 0.0), 1.0)
        self.mission_progress.setValue(round(progress * 1000))
        self.mission_progress.setFormat(f"{progress * 100:.1f}%")
        phase = snapshot.phase
        state_text = {
            SimulationPhase.IDLE: "等待播放",
            SimulationPhase.TRAVEL: "播放中",
            SimulationPhase.DWELL: f"执行任务：{snapshot.current_action_label}",
            SimulationPhase.PAUSED: "已暂停",
            SimulationPhase.FINISHED: "任务已完成",
        }.get(phase, phase)
        leg_index = min(snapshot.leg_index, len(self.mission_plan.legs) - 1)
        self.mission_status_label.setText(
            f"{state_text}，任务段 {leg_index + 1}/{snapshot.leg_count}"
        )
        if (
            snapshot.phase is SimulationPhase.TRAVEL
            and snapshot.leg_index < len(self.mission_plan.legs)
        ):
            action_label = self.mission_plan.legs[snapshot.leg_index].goal_stop.action_label
        else:
            action_label = snapshot.current_action_label
        direction = (
            f"{snapshot.from_node} → {snapshot.to_node}"
            if snapshot.phase is SimulationPhase.TRAVEL
            else "停留"
        )
        self.mission_detail_label.setText(
            f"总进度：{progress * 100:.1f}%\n"
            f"当前路线：{direction}\n"
            f"当前节点：{snapshot.current_node or '-'}\n"
            f"当前动作：{action_label}\n"
            f"状态：{state_text}"
        )
        self.mission_list.setCurrentRow(leg_index)
        self._update_vehicle_position(snapshot)

    def _update_vehicle_position(self, snapshot) -> None:  # type: ignore[no-untyped-def]
        if snapshot.phase in {SimulationPhase.DWELL, SimulationPhase.FINISHED}:
            if snapshot.current_node not in nodes:
                return
            point = scene_point(snapshot.current_node)
        else:
            if snapshot.from_node not in nodes or snapshot.to_node not in nodes:
                return
            start, end = scene_point(snapshot.from_node), scene_point(snapshot.to_node)
            point = QPointF(
                start.x() + (end.x() - start.x()) * snapshot.edge_progress,
                start.y() + (end.y() - start.y()) * snapshot.edge_progress,
            )
        self.vehicle_item.setPos(point)
        self.vehicle_item.setVisible(True)

    def _refresh_mission_controls(self) -> None:
        has_plan = self.mission_simulator is not None
        snapshot = self.mission_simulator.snapshot() if has_plan else None
        self.play_mission_button.setEnabled(has_plan and not snapshot.running)
        self.pause_mission_button.setEnabled(has_plan and (snapshot.running or snapshot.paused))
        self.stop_mission_button.setEnabled(has_plan)
        self.pause_mission_button.setText("继续" if snapshot is not None and snapshot.paused else "暂停")


def main() -> int:
    app = QApplication.instance() or QApplication(sys.argv)
    window = PlannerWindow()
    window.show()
    return app.exec()


__all__ = ["PlannerWindow", "main"]

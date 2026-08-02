"""拓扑路径规划工具的 PySide6 图形界面。"""

from __future__ import annotations

import sys
from typing import Callable

from PySide6.QtCore import QPointF, QRectF, Qt
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
    QPushButton,
    QScrollArea,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from .planner import Edge, PathResult, edge_key, edges, find_best_paths, nodes


SCALE = 180.0
MARGIN = 100.0


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
        self.setToolTip(f"点击切换道路：{edge.node_a} - {edge.node_b}")
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


class PlannerWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("LittleCar2 拓扑路径规划")
        self.resize(1200, 760)
        self.setMinimumSize(1000, 700)
        self.blocked_edges: set[tuple[str, str]] = set()
        self.results: list[PathResult] = []
        self.selected_result = 0
        self.route_items: list[QGraphicsItem] = []

        self.scene = QGraphicsScene(self)
        self.view = QGraphicsView(self.scene)
        self.view.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.view.setBackgroundBrush(QColor("#242424"))
        self.view.setDragMode(QGraphicsView.DragMode.NoDrag)
        self._build_scene()

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
            spin.valueChanged.connect(self.replan)

        self.path_list = QListWidget()
        self.path_list.currentRowChanged.connect(self._select_result)
        self.status_label = QLabel()
        self.status_label.setWordWrap(True)
        self.disabled_label = QLabel()
        self._build_controls()
        self.replan()

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
        self.node_items: dict[str, QGraphicsEllipseItem] = {}
        for node_id, node in nodes.items():
            radius = 28.0 if node.kind == "navigation" else 38.0
            point = scene_point(node_id)
            item = QGraphicsEllipseItem(-radius, -radius, radius * 2, radius * 2)
            item.setPos(point)
            item.setBrush(QColor("#303b46") if node.kind == "navigation" else QColor("#6a4b32"))
            item.setPen(QPen(QColor("#f2f2f2"), 2.0))
            item.setZValue(5)
            item.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
            self.scene.addItem(item)
            self.node_items[node_id] = item
            text = QGraphicsTextItem(node.id if node.kind == "navigation" else node.label, item)
            text.setDefaultTextColor(QColor("#ffffff"))
            text.setFont(QFont("Microsoft YaHei", 9 if node.kind == "navigation" else 8))
            text.setTextWidth(radius * 1.8)
            text.setPos(-radius * 0.9, -text.boundingRect().height() / 2)
            text.setAcceptedMouseButtons(Qt.MouseButton.NoButton)

    def _build_controls(self) -> None:
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
        layout.addWidget(QLabel("总成本 = A×路径长度 + B×转向单位 + C×停车次数"))
        layout.addWidget(QLabel("导航边长度：1.0；任务边长度：0.5"))

        restore = QPushButton("恢复全部道路")
        restore.clicked.connect(self._restore_edges)
        layout.addWidget(restore)
        layout.addWidget(self.disabled_label)
        layout.addWidget(QLabel("候选路径（最多 4 条）"))
        layout.addWidget(self.path_list, 1)
        layout.addWidget(self.status_label)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(panel)
        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(scroll)
        splitter.addWidget(self.view)
        splitter.setSizes([360, 840])
        self.setCentralWidget(splitter)

    def _toggle_edge(self, key: tuple[str, str]) -> None:
        if key in self.blocked_edges:
            self.blocked_edges.remove(key)
        else:
            self.blocked_edges.add(key)
        self.replan()

    def _restore_edges(self) -> None:
        self.blocked_edges.clear()
        self.replan()

    def _swap_endpoints(self) -> None:
        start = self.start_combo.currentData()
        goal = self.goal_combo.currentData()
        self.start_combo.setCurrentIndex(self.start_combo.findData(goal))
        self.goal_combo.setCurrentIndex(self.goal_combo.findData(start))
        self.replan()

    def _select_result(self, row: int) -> None:
        if 0 <= row < len(self.results):
            self.selected_result = row
            self._draw_routes()

    def replan(self) -> None:
        start_id = self.start_combo.currentData()
        goal_id = self.goal_combo.currentData()
        previous_nodes = self.results[self.selected_result].nodes if self.results and self.selected_result < len(self.results) else None
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
            item = QListWidgetItem(self._format_result(index, result))
            self.path_list.addItem(item)
        if self.results:
            self.path_list.setCurrentRow(self.selected_result)
        self.path_list.blockSignals(False)
        self.disabled_label.setText(f"已禁用道路：{len(self.blocked_edges)} 条")
        self.status_label.setText(
            f"当前：{display_name(start_id)} → {display_name(goal_id)}\n"
            + (f"可用候选路径：{len(self.results)} 条" if self.results else "当前道路配置下不存在可通行路径。")
        )
        self._refresh_edge_colors()
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
            path = QPainterPath()
            offset = (index - self.selected_result) * 4.0
            for segment, (first, second) in enumerate(zip(result.nodes, result.nodes[1:])):
                p1, p2 = scene_point(first), scene_point(second)
                dx, dy = p2.x() - p1.x(), p2.y() - p1.y()
                length = max((dx * dx + dy * dy) ** 0.5, 1.0)
                ox, oy = -dy / length * offset, dx / length * offset
                if segment == 0:
                    path.moveTo(p1.x() + ox, p1.y() + oy)
                path.lineTo(p2.x() + ox, p2.y() + oy)
            item = QGraphicsPathItem(path)
            item.setPen(pen)
            item.setZValue(3)
            item.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
            self.scene.addItem(item)
            self.route_items.append(item)
        if not self.results:
            return
        current = self.results[self.selected_result]
        for step, node_id in enumerate(current.nodes, 1):
            point = scene_point(node_id)
            marker = QGraphicsEllipseItem(-11, -11, 22, 22)
            marker.setPos(point)
            marker.setBrush(QColor("#31d17c"))
            marker.setPen(QPen(QColor("#ffffff"), 1.0))
            marker.setZValue(6)
            marker.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
            self.scene.addItem(marker)
            self.route_items.append(marker)
            label = QGraphicsTextItem(str(step), marker)
            label.setDefaultTextColor(QColor("#101010"))
            label.setFont(QFont("Arial", 8, QFont.Weight.Bold))
            label.setPos(-4, -8)
            label.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
            self.route_items.append(label)


def main() -> int:
    app = QApplication.instance() or QApplication(sys.argv)
    window = PlannerWindow()
    window.show()
    return app.exec()


__all__ = ["PlannerWindow", "main"]

# -*- coding: utf-8 -*-
"""HJMB V3.3 spatial trajectory editor."""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from PySide6.QtCore import QPointF, QRectF, Qt, QTimer, Signal
from PySide6.QtGui import QAction, QBrush, QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import (
    QApplication,
    QAbstractItemView,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGraphicsEllipseItem,
    QGraphicsLineItem,
    QGraphicsRectItem,
    QGraphicsScene,
    QGraphicsSimpleTextItem,
    QGraphicsView,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QSplitter,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QToolBar,
    QVBoxLayout,
    QWidget,
)

from path_codec_cli import (
    PathCodec,
    bin_path_traj_id,
    gate_count_from_nodes,
    load_project_dict,
    save_project_json,
)
from path_models import (
    ACTION_FLAG_HOLD_PATH,
    ACTION_FLAG_LOCKED,
    ACTION_FLAG_REQUIRED_AT_END,
    ACTION_GATE_ACCEL,
    ACTION_GATE_UNCONDITIONAL,
    ACTIONS,
    MAX_ACTIONS,
    MAX_EDIT_POINTS,
    MAX_GATES,
    MAX_TRAJ_ID,
    CutInPreviewResult,
    EditPoint,
    MechanicalAction,
    PathProject,
    PlanResult,
    PlanSummary,
    POINT_TYPE_ARRIVAL,
    POINT_TYPE_CUT_IN,
    POINT_TYPE_WAYPOINT,
    POINT_TYPES,
    TRAJ_FLAG_ARRIVAL,
    TRAJ_FLAG_CUT_IN,
    TRAJ_FLAG_END,
    TRAJ_FLAG_GATE,
    TRAJ_FLAG_SCAN,
    TRAJ_FLAG_STOP,
    TRAJ_FLAG_WAYPOINT,
    YAW_UNSPECIFIED_DDEG,
    make_default_project,
)
from trajectory_graphics import (
    ANALYSIS_MODE_ACCEL,
    ANALYSIS_MODE_BETA,
    ANALYSIS_MODE_NORMAL,
    ANALYSIS_MODE_SPEED,
    ANALYSIS_MODE_WZ,
    legend_entries,
    node_hover_text,
    trajectory_segment_color,
)
from trajectory_planner import plan_project

FIELD_W_MM = 4000
FIELD_H_MM = 2000
FIELD_HALF_W_MM = FIELD_W_MM // 2
FIELD_HALF_H_MM = FIELD_H_MM // 2
FIELD_X_MIN_MM = -FIELD_HALF_W_MM
FIELD_X_MAX_MM = FIELD_HALF_W_MM
FIELD_Y_MIN_MM = -FIELD_HALF_H_MM
FIELD_Y_MAX_MM = FIELD_HALF_H_MM
FIELD_SCALE = 0.25
SCENE_MARGIN_PX = 24
YAW_ARROW_LENGTH_MM = 120
FENCE_W_MM = 35

PICKUP_STATIONS = (
    (1, 1800, 500, 210, 300),
    (2, 1500, 0, 210, 300),
    (3, 1800, -500, 210, 300),
)
DROP_STATIONS = (
    (4, -1500, 800, 280, 200),
    (5, -1700, 400, 200, 280),
    (6, -1700, 0, 200, 280),
    (7, -1700, -400, 200, 280),
    (8, -1500, -800, 280, 200),
)
OBSTACLE_CENTERS = ((1000, 0), (-1000, 0))

POINT_TABLE_COLUMNS = (
    "id",
    "type",
    "x_mm",
    "y_mm",
    "yaw_ddeg",
    "max_speed",
    "corner_trim",
    "exact_pass",
    "stop",
    "gate",
    "scan",
    "end",
)
ACTION_TABLE_COLUMNS = (
    "seq",
    "action",
    "unlock_gate",
    "flags",
    "timeout",
    "arm_s",
    "disarm_s",
    "accel_limit",
    "beta_limit",
    "speed_limit",
    "stable_time",
)


def clamp(value: int, lower: int, upper: int) -> int:
    return max(lower, min(upper, int(value)))


def parse_editor_int(text: str, default: int = 0) -> int:
    try:
        return int(str(text).strip(), 0)
    except ValueError:
        return default


def point_colors(point: EditPoint, selected: bool) -> Tuple[QBrush, QPen]:
    if point.type == POINT_TYPE_CUT_IN:
        brush = QColor("#2563eb")
    elif point.type == POINT_TYPE_ARRIVAL and point.is_end:
        brush = QColor("#7c3aed")
    elif point.type == POINT_TYPE_ARRIVAL and point.gate_id != 0xFF:
        brush = QColor("#059669")
    elif point.type == POINT_TYPE_ARRIVAL and point.stop_required:
        brush = QColor("#dc2626")
    elif point.type == POINT_TYPE_ARRIVAL:
        brush = QColor("#10b981")
    else:
        brush = QColor("#f97316")
    pen = QColor("#f59e0b") if selected else brush.darker(170)
    return QBrush(brush), QPen(pen, 2.8 if selected else 1.6)


class DraggablePointItem(QGraphicsEllipseItem):
    def __init__(
        self,
        index: int,
        editor: "FieldView",
        point: EditPoint,
        selected: bool,
    ):
        super().__init__(-7, -7, 14, 14)
        self.index = index
        self.editor = editor
        brush, pen = point_colors(point, selected)
        self.setBrush(brush)
        self.setPen(pen)
        self.setFlags(
            QGraphicsEllipseItem.ItemIsMovable
            | QGraphicsEllipseItem.ItemIsSelectable
            | QGraphicsEllipseItem.ItemSendsGeometryChanges
        )
        self.setZValue(100)

    def itemChange(self, change, value):
        if change == QGraphicsEllipseItem.ItemPositionChange and self.scene() is not None:
            x_mm, y_mm = self.editor.scene_to_world(value)
            return self.editor.world_to_scene(
                clamp(x_mm, FIELD_X_MIN_MM, FIELD_X_MAX_MM),
                clamp(y_mm, FIELD_Y_MIN_MM, FIELD_Y_MAX_MM),
            )
        if (
            change == QGraphicsEllipseItem.ItemPositionHasChanged
            and self.scene() is not None
            and not self.editor.rebuilding
        ):
            x_mm, y_mm = self.editor.scene_to_world(self.pos())
            self.editor.update_point_visuals(self.index, x_mm, y_mm)
            self.editor.point_moved.emit(self.index, x_mm, y_mm)
        return super().itemChange(change, value)

    def mouseReleaseEvent(self, event):
        super().mouseReleaseEvent(event)
        self.editor.point_drag_finished.emit(self.index)


class DraggableYawHandleItem(QGraphicsEllipseItem):
    def __init__(
        self,
        index: int,
        editor: "FieldView",
        center_x_mm: float,
        center_y_mm: float,
    ):
        super().__init__(-5.5, -5.5, 11, 11)
        self.index = index
        self.editor = editor
        self.center_x_mm = center_x_mm
        self.center_y_mm = center_y_mm
        self.ready = False
        self.setBrush(QBrush(QColor("#60a5fa")))
        self.setPen(QPen(QColor("#1d4ed8"), 1.5))
        self.setFlags(
            QGraphicsEllipseItem.ItemIsMovable
            | QGraphicsEllipseItem.ItemSendsGeometryChanges
        )
        self.setCursor(Qt.OpenHandCursor)
        self.setZValue(115)

    def itemChange(self, change, value):
        if change == QGraphicsEllipseItem.ItemPositionChange and self.scene() is not None:
            x_mm, y_mm = self.editor.scene_to_world_float(value)
            dx = x_mm - self.center_x_mm
            dy = y_mm - self.center_y_mm
            length = math.hypot(dx, dy)
            if length < 1e-6:
                return self.pos()
            return self.editor.world_to_scene(
                self.center_x_mm + dx / length * YAW_ARROW_LENGTH_MM,
                self.center_y_mm + dy / length * YAW_ARROW_LENGTH_MM,
            )
        if (
            change == QGraphicsEllipseItem.ItemPositionHasChanged
            and self.scene() is not None
            and self.ready
            and not self.editor.rebuilding
        ):
            self.editor.update_yaw_visuals(self.index, self.pos())
            x_mm, y_mm = self.editor.scene_to_world_float(self.pos())
            yaw = int(
                round(
                    math.degrees(
                        math.atan2(y_mm - self.center_y_mm, x_mm - self.center_x_mm)
                    )
                    * 10.0
                )
            )
            self.editor.yaw_changed.emit(self.index, yaw)
        return super().itemChange(change, value)

    def mouseReleaseEvent(self, event):
        super().mouseReleaseEvent(event)
        self.editor.yaw_drag_finished.emit(self.index)


class FieldView(QGraphicsView):
    add_point_requested = Signal(int, int)
    point_moved = Signal(int, int, int)
    yaw_changed = Signal(int, int)
    point_drag_finished = Signal(int)
    yaw_drag_finished = Signal(int)
    point_selected = Signal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.scene_obj = QGraphicsScene(self)
        self.setScene(self.scene_obj)
        self.setRenderHints(QPainter.Antialiasing | QPainter.TextAntialiasing)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)
        self.setMouseTracking(True)
        self.setDragMode(QGraphicsView.RubberBandDrag)
        self.rebuilding = False
        self.auto_fit = True
        self.project: Optional[PathProject] = None
        self.point_items: Dict[int, DraggablePointItem] = {}
        self.point_labels: Dict[int, QGraphicsSimpleTextItem] = {}
        self.sparse_line_items: List[QGraphicsLineItem] = []
        self.yaw_line_items: Dict[int, QGraphicsLineItem] = {}
        self.yaw_handle_items: Dict[int, DraggableYawHandleItem] = {}
        self.capture_circle = None
        self.preview_line: Optional[QGraphicsLineItem] = None
        self.scene_obj.selectionChanged.connect(self._on_selection_changed)

    def world_to_scene(self, x_mm: float, y_mm: float) -> QPointF:
        return QPointF(
            (x_mm + FIELD_HALF_W_MM) * FIELD_SCALE,
            (FIELD_HALF_H_MM - y_mm) * FIELD_SCALE,
        )

    def scene_to_world_float(self, position: QPointF) -> Tuple[float, float]:
        return (
            position.x() / FIELD_SCALE - FIELD_HALF_W_MM,
            FIELD_HALF_H_MM - position.y() / FIELD_SCALE,
        )

    def scene_to_world(self, position: QPointF) -> Tuple[int, int]:
        x_mm, y_mm = self.scene_to_world_float(position)
        return int(round(x_mm)), int(round(y_mm))

    def add_world_line(
        self, x1: float, y1: float, x2: float, y2: float, pen: QPen
    ) -> QGraphicsLineItem:
        a = self.world_to_scene(x1, y1)
        b = self.world_to_scene(x2, y2)
        return self.scene_obj.addLine(a.x(), a.y(), b.x(), b.y(), pen)

    def add_world_rect(
        self,
        x: float,
        y: float,
        width: float,
        height: float,
        pen: QPen,
        brush: QBrush,
    ) -> QGraphicsRectItem:
        top_left = self.world_to_scene(x, y + height)
        return self.scene_obj.addRect(
            top_left.x(),
            top_left.y(),
            width * FIELD_SCALE,
            height * FIELD_SCALE,
            pen,
            brush,
        )

    def add_world_ellipse(
        self,
        center_x: float,
        center_y: float,
        radius: float,
        pen: QPen,
        brush: QBrush,
    ):
        top_left = self.world_to_scene(center_x - radius, center_y + radius)
        return self.scene_obj.addEllipse(
            top_left.x(),
            top_left.y(),
            radius * 2 * FIELD_SCALE,
            radius * 2 * FIELD_SCALE,
            pen,
            brush,
        )

    def add_world_text(
        self,
        x_mm: float,
        y_mm: float,
        text: str,
        color: QColor = QColor("#202020"),
        size: int = 9,
    ):
        item = self.scene_obj.addSimpleText(text)
        item.setBrush(QBrush(color))
        font = QFont()
        font.setPointSize(size)
        item.setFont(font)
        item.setPos(self.world_to_scene(x_mm, y_mm))
        item.setZValue(80)
        return item

    def draw_field(self):
        self.scene_obj.clear()
        self.scene_obj.setBackgroundBrush(QBrush(QColor("#f5f7fa")))
        self.add_world_rect(
            FIELD_X_MIN_MM - FENCE_W_MM,
            FIELD_Y_MIN_MM - FENCE_W_MM,
            FIELD_W_MM + FENCE_W_MM * 2,
            FIELD_H_MM + FENCE_W_MM * 2,
            QPen(QColor("#965035"), 2),
            QBrush(QColor("#eeb888")),
        )
        self.add_world_rect(
            FIELD_X_MIN_MM,
            FIELD_Y_MIN_MM,
            FIELD_W_MM,
            FIELD_H_MM,
            QPen(QColor("#142d50"), 2),
            QBrush(QColor("#f0f4fa")),
        )
        for x_mm in range(FIELD_X_MIN_MM, FIELD_X_MAX_MM + 1, 250):
            pen = QPen(QColor("#dce1e6"), 1)
            if x_mm % 1000 == 0:
                pen = QPen(QColor("#bec3cd"), 1.5)
            self.add_world_line(x_mm, FIELD_Y_MIN_MM, x_mm, FIELD_Y_MAX_MM, pen)
        for y_mm in range(FIELD_Y_MIN_MM, FIELD_Y_MAX_MM + 1, 250):
            pen = QPen(QColor("#dce1e6"), 1)
            if y_mm % 1000 == 0:
                pen = QPen(QColor("#bec3cd"), 1.5)
            self.add_world_line(FIELD_X_MIN_MM, y_mm, FIELD_X_MAX_MM, y_mm, pen)
        axis_pen = QPen(QColor("#dc2828"), 1.8, Qt.DashLine)
        self.add_world_line(0, FIELD_Y_MIN_MM, 0, FIELD_Y_MAX_MM, axis_pen)
        self.add_world_line(FIELD_X_MIN_MM, 0, FIELD_X_MAX_MM, 0, axis_pen)
        self.add_world_text(35, 45, "中心 (0,0)", QColor("#b91c1c"), 9)
        self.add_world_rect(
            -400,
            -200,
            400,
            400,
            QPen(QColor("#145096"), 2),
            QBrush(QColor(205, 230, 255, 170)),
        )
        self.add_world_text(-360, 230, "起始区域", QColor("#145096"), 9)
        for center_x, center_y in OBSTACLE_CENTERS:
            self.add_world_ellipse(
                center_x,
                center_y,
                51,
                QPen(QColor("#825032"), 2),
                QBrush(QColor(240, 160, 90, 190)),
            )
        for station_id, center_x, center_y, width, height in PICKUP_STATIONS:
            self.add_world_rect(
                center_x - width / 2,
                center_y - height / 2,
                width,
                height,
                QPen(QColor("#146e82"), 2),
                QBrush(QColor(80, 190, 210, 170)),
            )
            self.add_world_text(center_x - 18, center_y + height / 2 + 35, str(station_id))
        for station_id, center_x, center_y, width, height in DROP_STATIONS:
            self.add_world_rect(
                center_x - width / 2,
                center_y - height / 2,
                width,
                height,
                QPen(QColor("#503ca0"), 2),
                QBrush(QColor(180, 170, 230, 165)),
            )
            self.add_world_text(center_x - 18, center_y + height / 2 + 35, str(station_id))
        self.scene_obj.setSceneRect(
            QRectF(
                -FENCE_W_MM * FIELD_SCALE - SCENE_MARGIN_PX,
                -FENCE_W_MM * FIELD_SCALE - SCENE_MARGIN_PX,
                (FIELD_W_MM + 2 * FENCE_W_MM) * FIELD_SCALE
                + 2 * SCENE_MARGIN_PX,
                (FIELD_H_MM + 2 * FENCE_W_MM) * FIELD_SCALE
                + 2 * SCENE_MARGIN_PX,
            )
        )

    def rebuild(
        self,
        project: PathProject,
        plan: Optional[PlanResult],
        selected_index: Optional[int],
        analysis_mode: str,
        overlay_speed_max_mmps: float,
        overlay_accel_max_mmps2: float,
        overlay_wz_max_radps: float,
        overlay_beta_max_radps2: float,
    ):
        self.rebuilding = True
        self.project = project
        self.point_items = {}
        self.point_labels = {}
        self.sparse_line_items = []
        self.yaw_line_items = {}
        self.yaw_handle_items = {}
        self.capture_circle = None
        self.preview_line = None
        self.draw_field()
        if plan is not None:
            for previous, current in zip(plan.nodes[:-1], plan.nodes[1:]):
                color = trajectory_segment_color(
                    analysis_mode,
                    current,
                    overlay_speed_max_mmps,
                    overlay_accel_max_mmps2,
                    overlay_wz_max_radps,
                    overlay_beta_max_radps2,
                )
                line = self.add_world_line(
                    previous.x_mm,
                    previous.y_mm,
                    current.x_mm,
                    current.y_mm,
                    QPen(QColor(color), 3.0),
                )
                line.setZValue(50)
                line.setToolTip(node_hover_text(current))
        if len(project.points) >= 2:
            control_pen = QPen(QColor("#475569"), 1.4, Qt.DashLine)
            for previous, current in zip(project.points[:-1], project.points[1:]):
                line = self.add_world_line(
                    previous.x_mm,
                    previous.y_mm,
                    current.x_mm,
                    current.y_mm,
                    control_pen,
                )
                line.setZValue(72)
                self.sparse_line_items.append(line)

        if project.points:
            cut_in = project.points[0]
            if project.overlay.show_cut_in_capture:
                self.capture_circle = self.add_world_ellipse(
                    cut_in.x_mm,
                    cut_in.y_mm,
                    project.cut_in.capture_radius_mm,
                    QPen(QColor("#2563eb"), 1.8, Qt.DashLine),
                    QBrush(Qt.NoBrush),
                )
                self.capture_circle.setZValue(60)
            preview = project.preview_initial_pose
            if preview.enabled and project.overlay.show_cut_in_preview:
                self.preview_line = self.add_world_line(
                    preview.x_mm,
                    preview.y_mm,
                    cut_in.x_mm,
                    cut_in.y_mm,
                    QPen(QColor("#0891b2"), 2.2, Qt.DashLine),
                )
                self.preview_line.setZValue(65)
                self.add_world_ellipse(
                    preview.x_mm,
                    preview.y_mm,
                    25,
                    QPen(QColor("#0e7490"), 2),
                    QBrush(QColor("#67e8f9")),
                ).setZValue(90)

        for index, point in enumerate(project.points):
            item = DraggablePointItem(index, self, point, index == selected_index)
            item.setPos(self.world_to_scene(point.x_mm, point.y_mm))
            self.scene_obj.addItem(item)
            item.setSelected(index == selected_index)
            self.point_items[index] = item
            labels = [str(index), point.type]
            if point.gate_id != 0xFF:
                labels.append(f"G{point.gate_id}")
            if point.stop_required:
                labels.append("STOP")
            if point.is_end:
                labels.append("END")
            label = QGraphicsSimpleTextItem(" ".join(labels))
            label.setBrush(QBrush(QColor("#111827")))
            label.setFont(QFont("Arial", 9, QFont.Bold))
            label.setPos(item.pos() + QPointF(8, -23))
            label.setZValue(110)
            self.scene_obj.addItem(label)
            self.point_labels[index] = label

            if point.type != POINT_TYPE_WAYPOINT:
                yaw_rad = math.radians(point.yaw_ddeg / 10.0)
                end_x = point.x_mm + math.cos(yaw_rad) * YAW_ARROW_LENGTH_MM
                end_y = point.y_mm + math.sin(yaw_rad) * YAW_ARROW_LENGTH_MM
                yaw_line = self.add_world_line(
                    point.x_mm,
                    point.y_mm,
                    end_x,
                    end_y,
                    QPen(QColor("#1f2937"), 1.7),
                )
                yaw_line.setZValue(105)
                self.yaw_line_items[index] = yaw_line
                handle = DraggableYawHandleItem(
                    index, self, point.x_mm, point.y_mm
                )
                handle.setPos(self.world_to_scene(end_x, end_y))
                self.scene_obj.addItem(handle)
                self.yaw_handle_items[index] = handle
                handle.ready = True
        self.rebuilding = False
        if self.auto_fit:
            self.fit_to_field()

    def _set_line_world(
        self,
        line: QGraphicsLineItem,
        x1_mm: float,
        y1_mm: float,
        x2_mm: float,
        y2_mm: float,
    ):
        start = self.world_to_scene(x1_mm, y1_mm)
        end = self.world_to_scene(x2_mm, y2_mm)
        line.setLine(start.x(), start.y(), end.x(), end.y())

    def update_point_visuals(self, index: int, x_mm: float, y_mm: float):
        if self.project is None or not 0 <= index < len(self.project.points):
            return
        point = self.project.points[index]
        label = self.point_labels.get(index)
        item = self.point_items.get(index)
        if label is not None and item is not None:
            label.setPos(item.pos() + QPointF(8, -23))

        if point.type != POINT_TYPE_WAYPOINT:
            yaw_rad = math.radians(point.yaw_ddeg / 10.0)
            end_x = x_mm + math.cos(yaw_rad) * YAW_ARROW_LENGTH_MM
            end_y = y_mm + math.sin(yaw_rad) * YAW_ARROW_LENGTH_MM
            yaw_line = self.yaw_line_items.get(index)
            if yaw_line is not None:
                self._set_line_world(yaw_line, x_mm, y_mm, end_x, end_y)
            handle = self.yaw_handle_items.get(index)
            if handle is not None:
                handle.ready = False
                handle.center_x_mm = x_mm
                handle.center_y_mm = y_mm
                handle.setPos(self.world_to_scene(end_x, end_y))
                handle.ready = True

        if index > 0 and index - 1 < len(self.sparse_line_items):
            previous = self.project.points[index - 1]
            self._set_line_world(
                self.sparse_line_items[index - 1],
                previous.x_mm,
                previous.y_mm,
                x_mm,
                y_mm,
            )
        if index < len(self.project.points) - 1 and index < len(self.sparse_line_items):
            following = self.project.points[index + 1]
            self._set_line_world(
                self.sparse_line_items[index],
                x_mm,
                y_mm,
                following.x_mm,
                following.y_mm,
            )

        if index == 0:
            if self.capture_circle is not None:
                radius = self.project.cut_in.capture_radius_mm
                top_left = self.world_to_scene(x_mm - radius, y_mm + radius)
                self.capture_circle.setRect(
                    top_left.x(),
                    top_left.y(),
                    radius * 2 * FIELD_SCALE,
                    radius * 2 * FIELD_SCALE,
                )
            if self.preview_line is not None:
                preview = self.project.preview_initial_pose
                self._set_line_world(
                    self.preview_line,
                    preview.x_mm,
                    preview.y_mm,
                    x_mm,
                    y_mm,
                )

    def update_yaw_visuals(self, index: int, handle_position: QPointF):
        if self.project is None or not 0 <= index < len(self.project.points):
            return
        point = self.project.points[index]
        yaw_line = self.yaw_line_items.get(index)
        if yaw_line is None:
            return
        center = self.world_to_scene(point.x_mm, point.y_mm)
        yaw_line.setLine(
            center.x(),
            center.y(),
            handle_position.x(),
            handle_position.y(),
        )

    def set_selected_index(self, selected_index: Optional[int]):
        if self.project is None:
            return
        self.rebuilding = True
        for index, item in self.point_items.items():
            selected = index == selected_index
            item.setSelected(selected)
            brush, pen = point_colors(self.project.points[index], selected)
            item.setBrush(brush)
            item.setPen(pen)
            handle = self.yaw_handle_items.get(index)
            if handle is not None:
                handle.setBrush(
                    QBrush(QColor("#f59e0b") if selected else QColor("#60a5fa"))
                )
        self.rebuilding = False

    def fit_to_field(self):
        self.auto_fit = True
        self.resetTransform()
        self.fitInView(self.scene_obj.sceneRect(), Qt.KeepAspectRatio)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self.auto_fit:
            self.fit_to_field()

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        if delta == 0:
            super().wheelEvent(event)
            return
        factor = 1.15 if delta > 0 else 1.0 / 1.15
        next_scale = self.transform().m11() * factor
        if 0.2 <= next_scale <= 8.0:
            self.auto_fit = False
            self.scale(factor, factor)
        event.accept()

    def mouseDoubleClickEvent(self, event):
        if event.button() == Qt.LeftButton:
            x_mm, y_mm = self.scene_to_world(self.mapToScene(event.pos()))
            if (
                FIELD_X_MIN_MM <= x_mm <= FIELD_X_MAX_MM
                and FIELD_Y_MIN_MM <= y_mm <= FIELD_Y_MAX_MM
            ):
                self.add_point_requested.emit(x_mm, y_mm)
                return
        super().mouseDoubleClickEvent(event)

    def _on_selection_changed(self):
        if self.rebuilding:
            return
        for item in self.scene_obj.selectedItems():
            if isinstance(item, DraggablePointItem):
                self.point_selected.emit(item.index)
                break


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("HJMB 空间轨迹编辑器 V3.3")
        self.resize(1680, 940)
        self.project = make_default_project()
        self.plan_result: Optional[PlanResult] = None
        self.current_json_path: Optional[Path] = None
        self.updating_ui = False
        self.plan_error = ""

        self.field = FieldView(self)
        self.field.add_point_requested.connect(self.add_point_from_canvas)
        self.field.point_moved.connect(self.on_point_moved)
        self.field.yaw_changed.connect(self.on_yaw_changed)
        self.field.point_drag_finished.connect(lambda _index: self.schedule_plan())
        self.field.yaw_drag_finished.connect(lambda _index: self.schedule_plan())
        self.field.point_selected.connect(self.select_point_row)

        self.traj_id_spin = QSpinBox()
        self.traj_id_spin.setRange(0, MAX_TRAJ_ID)
        self.traj_id_spin.valueChanged.connect(self._traj_id_changed)
        self.status_label = QLabel()
        self.status_label.setWordWrap(True)
        self.legend_label = QLabel()
        self.legend_label.setWordWrap(True)

        self.point_table = self._create_table(POINT_TABLE_COLUMNS)
        self.action_table = self._create_table(ACTION_TABLE_COLUMNS)
        self.point_table.itemChanged.connect(self.on_point_item_changed)
        self.point_table.itemSelectionChanged.connect(self.on_point_selection_changed)
        self.action_table.itemChanged.connect(self.on_action_item_changed)

        self.param_widgets: Dict[str, QWidget] = {}
        right_panel = self._build_right_panel()
        right_panel.setMinimumWidth(760)
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self.field)
        splitter.addWidget(right_panel)
        splitter.setStretchFactor(0, 1)
        splitter.setSizes([960, 720])
        self.setCentralWidget(splitter)
        self._build_toolbar()

        self.plan_timer = QTimer(self)
        self.plan_timer.setSingleShot(True)
        self.plan_timer.setInterval(220)
        self.plan_timer.timeout.connect(self.plan_now)
        self.refresh_all()
        self.plan_now()

    def _create_table(self, columns: Tuple[str, ...]) -> QTableWidget:
        table = QTableWidget(0, len(columns))
        table.setHorizontalHeaderLabels(columns)
        table.horizontalHeader().setSectionResizeMode(QHeaderView.Interactive)
        table.horizontalHeader().setMinimumSectionSize(62)
        table.verticalHeader().setVisible(False)
        table.setSelectionBehavior(QAbstractItemView.SelectRows)
        table.setSelectionMode(QAbstractItemView.SingleSelection)
        return table

    def _build_toolbar(self):
        toolbar = QToolBar("工具")
        self.addToolBar(toolbar)
        for text, callback in (
            ("新建", self.new_project),
            ("清空", self.clear_project),
            ("导入配置 JSON", self.open_json),
            ("保存配置 JSON", self.save_json),
            ("导出配置 JSON", self.save_json_as),
            ("导出 BIN", self.export_bin),
            ("打开 BIN", self.open_bin),
            ("适配场地", self.field.fit_to_field),
            ("重新规划", self.plan_now),
            ("校验", self.validate_current_project),
        ):
            action = QAction(text, self)
            action.triggered.connect(callback)
            toolbar.addAction(action)
        toolbar.addSeparator()
        toolbar.addWidget(QLabel("显示模式: "))
        self.analysis_combo = QComboBox()
        self.analysis_combo.addItem("普通路径", ANALYSIS_MODE_NORMAL)
        self.analysis_combo.addItem("速度分区", ANALYSIS_MODE_SPEED)
        self.analysis_combo.addItem("合成线加速度分区", ANALYSIS_MODE_ACCEL)
        self.analysis_combo.addItem("角速度分区", ANALYSIS_MODE_WZ)
        self.analysis_combo.addItem("角加速度分区", ANALYSIS_MODE_BETA)
        self.analysis_combo.currentIndexChanged.connect(self.analysis_mode_changed)
        toolbar.addWidget(self.analysis_combo)

    def _build_right_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        top = QHBoxLayout()
        top.addWidget(QLabel("traj_id:"))
        top.addWidget(self.traj_id_spin)
        top.addStretch(1)
        layout.addLayout(top)
        tabs = QTabWidget()
        tabs.addTab(self._build_point_tab(), "编辑点")
        tabs.addTab(self._build_action_tab(), "机械动作")
        tabs.addTab(self._build_parameter_tab(), "规划参数")
        layout.addWidget(tabs, 1)
        layout.addWidget(self.legend_label)
        layout.addWidget(self.status_label)
        return panel

    def _build_point_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.addWidget(self.point_table, 1)
        for column, width in enumerate(
            (50, 92, 72, 72, 84, 88, 88, 76, 60, 70, 60, 60)
        ):
            self.point_table.setColumnWidth(column, width)
        buttons = QHBoxLayout()
        for text, callback in (
            ("末尾添加", self.add_default_point),
            ("插入", self.insert_point),
            ("删除", self.delete_point),
            ("上移", lambda: self.move_point(-1)),
            ("下移", lambda: self.move_point(1)),
            ("设为终点", self.mark_end),
            ("重编 Gate", self.renumber_gates),
        ):
            button = QPushButton(text)
            button.clicked.connect(callback)
            buttons.addWidget(button)
        layout.addLayout(buttons)
        hint = QLabel(
            "CUT_IN 必须唯一且位于首行；只有 CUT_IN/ARRIVAL 决定 yaw；"
            "WAYPOINT yaw 固定为 0xFF，不显示方向控制点。"
        )
        hint.setWordWrap(True)
        layout.addWidget(hint)
        return tab

    def _build_action_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.addWidget(self.action_table, 1)
        for column, width in enumerate((52, 130, 105, 75, 80, 75, 80, 92, 92, 90, 88)):
            self.action_table.setColumnWidth(column, width)
        row = QHBoxLayout()
        for text, callback in (
            ("添加", self.add_action),
            ("删除", self.delete_action),
            ("上移", lambda: self.move_action(-1)),
            ("下移", lambda: self.move_action(1)),
            ("LOCKED", lambda: self.toggle_action_flag(ACTION_FLAG_LOCKED)),
            ("HOLD_PATH", lambda: self.toggle_action_flag(ACTION_FLAG_HOLD_PATH)),
            (
                "REQUIRED_AT_END",
                lambda: self.toggle_action_flag(ACTION_FLAG_REQUIRED_AT_END),
            ),
        ):
            button = QPushButton(text)
            button.clicked.connect(callback)
            row.addWidget(button)
        layout.addLayout(row)
        hint = QLabel(
            "PREP_STORE_1/2/3 选择暂存仓，随后执行 STORE；"
            "DROP_1/2/3/12/23 无需预备，可使用停车 Gate 或 0xFE 低加速度 Gate。"
        )
        hint.setWordWrap(True)
        layout.addWidget(hint)
        return tab

    def _double_spin(
        self, key: str, minimum: float, maximum: float, decimals: int, suffix: str
    ) -> QDoubleSpinBox:
        widget = QDoubleSpinBox()
        widget.setRange(minimum, maximum)
        widget.setDecimals(decimals)
        widget.setSuffix(suffix)
        widget.valueChanged.connect(self.parameter_changed)
        self.param_widgets[key] = widget
        return widget

    def _int_spin(
        self, key: str, minimum: int, maximum: int, suffix: str = ""
    ) -> QSpinBox:
        widget = QSpinBox()
        widget.setRange(minimum, maximum)
        widget.setSuffix(suffix)
        widget.valueChanged.connect(self.parameter_changed)
        self.param_widgets[key] = widget
        return widget

    def _check(self, key: str) -> QCheckBox:
        widget = QCheckBox()
        widget.toggled.connect(self.parameter_changed)
        self.param_widgets[key] = widget
        return widget

    def _build_parameter_tab(self) -> QWidget:
        content = QWidget()
        layout = QVBoxLayout(content)

        trajectory_group = QGroupBox("轨迹")
        trajectory_form = QFormLayout(trajectory_group)
        trajectory_form.addRow(
            "最大速度",
            self._double_spin("planner.max_speed_mps", 0.05, 5.0, 3, " m/s"),
        )
        trajectory_form.addRow(
            "合成线加速度",
            self._double_spin("planner.linear_accel_mps2", 0.05, 10.0, 3, " m/s²"),
        )
        trajectory_form.addRow(
            "横向加速度",
            self._double_spin("planner.lateral_accel_mps2", 0.0, 10.0, 3, " m/s²"),
        )
        trajectory_form.addRow(
            "最大角速度",
            self._double_spin("planner.max_wz_radps", 0.1, 20.0, 3, " rad/s"),
        )
        trajectory_form.addRow(
            "移动角加速度",
            self._double_spin("planner.angular_accel_moving", 0.1, 20.0, 3, " rad/s²"),
        )
        trajectory_form.addRow(
            "纯旋转角加速度",
            self._double_spin("planner.angular_accel_rotate", 0.1, 30.0, 3, " rad/s²"),
        )
        trajectory_form.addRow(
            "普通采样间距",
            self._int_spin("planner.nominal_spacing_mm", 5, 50, " mm"),
        )
        trajectory_form.addRow(
            "最大节点间距",
            self._int_spin("planner.max_spacing_mm", 5, 50, " mm"),
        )
        trajectory_form.addRow(
            "最大参考超前",
            self._int_spin("planner.max_ref_lead_mm", 1, 500, " mm"),
        )
        scale_mode = QComboBox()
        scale_mode.addItem("速度/角加速度跟随规划上限", "planner")
        scale_mode.addItem("速度 2.0 / 角加速度 2.0", "competition")
        scale_mode.currentIndexChanged.connect(self.parameter_changed)
        self.param_widgets["overlay.scale_mode"] = scale_mode
        trajectory_form.addRow("分档标尺", scale_mode)
        trajectory_form.addRow(
            "启用碰撞检查标志", self._check("project.collision_check_enabled")
        )
        trajectory_form.addRow(
            "启用可达性检查标志", self._check("project.reachability_check_enabled")
        )
        layout.addWidget(trajectory_group)

        cut_in_group = QGroupBox("CUT_IN")
        cut_in_form = QFormLayout(cut_in_group)
        cut_in_form.addRow(
            "捕获半径",
            self._int_spin("cut_in.capture_radius_mm", 1, 1000, " mm"),
        )
        cut_in_form.addRow(
            "交接速度",
            self._double_spin("cut_in.target_speed_mps", 0.01, 5.0, 3, " m/s"),
        )
        cut_in_form.addRow(
            "切入最高速度",
            self._double_spin("cut_in.approach_max_speed_mps", 0.01, 5.0, 3, " m/s"),
        )
        cut_in_form.addRow(
            "首段直线长度",
            self._int_spin("cut_in.straight_length_mm", 1, 5000, " mm"),
        )
        cut_in_form.addRow(
            "yaw 容差",
            self._double_spin("cut_in.yaw_tolerance_deg", 0.0, 180.0, 1, "°"),
        )
        cut_in_form.addRow(
            "切线容差",
            self._double_spin("cut_in.tangent_tolerance_deg", 0.0, 180.0, 1, "°"),
        )
        cut_in_form.addRow("切入时对齐 yaw", self._check("cut_in.align_yaw"))
        cut_in_form.addRow(
            "允许首段投影捕获", self._check("cut_in.allow_first_segment_capture")
        )
        layout.addWidget(cut_in_group)

        vehicle_group = QGroupBox("车辆参数（导出前按实车核对）")
        vehicle_form = QFormLayout(vehicle_group)
        vehicle_form.addRow(
            "轮半径",
            self._double_spin("vehicle.wheel_radius_mm", 0.1, 1000.0, 2, " mm"),
        )
        vehicle_form.addRow(
            "旋转半径",
            self._double_spin("vehicle.rotation_radius_mm", 0.1, 2000.0, 2, " mm"),
        )
        vehicle_form.addRow(
            "轮速规划软限",
            self._int_spin("vehicle.wheel_plan_limit_rpm", 1, 1000, " rpm"),
        )
        vehicle_form.addRow(
            "轮速运行硬限",
            self._int_spin("vehicle.wheel_hard_limit_rpm", 1, 1000, " rpm"),
        )
        convention = QComboBox()
        convention.addItem("X_FL_FR_RL_RR", "X_FL_FR_RL_RR")
        convention.currentIndexChanged.connect(self.parameter_changed)
        self.param_widgets["vehicle.mecanum_convention"] = convention
        vehicle_form.addRow("麦轮约定", convention)
        layout.addWidget(vehicle_group)

        preview_group = QGroupBox("动态切入预览")
        preview_form = QFormLayout(preview_group)
        preview_form.addRow("启用", self._check("preview.enabled"))
        preview_form.addRow(
            "初始 X", self._double_spin("preview.x_mm", -10000, 10000, 1, " mm")
        )
        preview_form.addRow(
            "初始 Y", self._double_spin("preview.y_mm", -10000, 10000, 1, " mm")
        )
        preview_form.addRow(
            "初始 yaw",
            self._double_spin("preview.yaw_deg", -3600, 3600, 1, "°"),
        )
        preview_form.addRow(
            "初始速度",
            self._double_spin("preview.initial_speed_mps", 0, 5, 3, " m/s"),
        )
        layout.addWidget(preview_group)

        mechanism_group = QGroupBox("机械动作预计时间")
        mechanism_form = QFormLayout(mechanism_group)
        for action_code, action_name in ACTIONS.items():
            mechanism_form.addRow(
                action_name,
                self._int_spin(
                    f"mechanism.duration.{action_name}", 0, 60000, " ms"
                ),
            )
        mechanism_form.addRow(
            "DROP 安全余量",
            self._int_spin("mechanism.drop_safety_margin_ms", 0, 60000, " ms"),
        )
        layout.addWidget(mechanism_group)
        layout.addStretch(1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(content)
        tab = QWidget()
        tab_layout = QVBoxLayout(tab)
        tab_layout.addWidget(scroll)
        return tab

    def selected_point_row(self) -> Optional[int]:
        rows = self.point_table.selectionModel().selectedRows()
        return rows[0].row() if rows else None

    def selected_action_row(self) -> Optional[int]:
        rows = self.action_table.selectionModel().selectedRows()
        return rows[0].row() if rows else None

    def _set_item(
        self,
        table: QTableWidget,
        row: int,
        column: int,
        text: str,
        editable: bool = True,
    ):
        item = QTableWidgetItem(text)
        if not editable:
            item.setFlags(item.flags() & ~Qt.ItemIsEditable)
        table.setItem(row, column, item)

    def _set_combo(
        self,
        table: QTableWidget,
        row: int,
        column: int,
        values: List[Tuple[str, object]],
        current,
        callback,
    ):
        combo = QComboBox()
        for text, value in values:
            combo.addItem(text, value)
        if combo.findData(current) < 0:
            combo.addItem(str(current), current)
        combo.setCurrentIndex(combo.findData(current))
        combo.currentIndexChanged.connect(
            lambda _index, row=row, combo=combo: callback(row, combo.currentData())
        )
        table.setCellWidget(row, column, combo)

    def refresh_all(
        self,
        selected_point: Optional[int] = None,
        selected_action: Optional[int] = None,
        rebuild_field: bool = True,
    ):
        self.renumber_points()
        self.renumber_actions()
        self.updating_ui = True
        self.traj_id_spin.setValue(self.project.traj_id)
        self.refresh_point_table(selected_point)
        self.refresh_action_table(selected_action)
        self.refresh_parameter_widgets()
        self.updating_ui = False
        if rebuild_field:
            self.refresh_field(selected_point)
        self.update_status()

    def refresh_point_table(self, selected: Optional[int] = None):
        self.point_table.setRowCount(len(self.project.points))
        gate_values = [("无 Gate", 0xFF)] + [
            (f"Gate {gate_id}", gate_id) for gate_id in range(MAX_GATES)
        ]
        for row, point in enumerate(self.project.points):
            self._set_item(self.point_table, row, 0, str(point.point_id), False)
            self._set_combo(
                self.point_table,
                row,
                1,
                [(name, name) for name in POINT_TYPES],
                point.type,
                self.on_point_type_changed,
            )
            self._set_item(self.point_table, row, 2, f"{point.x_mm:g}")
            self._set_item(self.point_table, row, 3, f"{point.y_mm:g}")
            self._set_item(
                self.point_table,
                row,
                4,
                "0xFF"
                if point.type == POINT_TYPE_WAYPOINT
                else str(point.yaw_ddeg),
                point.type != POINT_TYPE_WAYPOINT,
            )
            self._set_item(self.point_table, row, 5, str(point.max_speed_mmps))
            self._set_item(self.point_table, row, 6, f"{point.corner_trim_mm:g}")
            self._set_item(self.point_table, row, 7, str(int(point.exact_pass)))
            self._set_item(self.point_table, row, 8, str(int(point.stop_required)))
            self._set_combo(
                self.point_table,
                row,
                9,
                gate_values,
                point.gate_id,
                self.on_point_gate_changed,
            )
            self._set_item(self.point_table, row, 10, str(int(point.scan)))
            self._set_item(self.point_table, row, 11, str(int(point.is_end)))
        if selected is not None and 0 <= selected < len(self.project.points):
            self.point_table.selectRow(selected)

    def refresh_action_table(self, selected: Optional[int] = None):
        self.action_table.setRowCount(len(self.project.actions))
        gates = [("无条件 0xFF", ACTION_GATE_UNCONDITIONAL), ("加速度 0xFE", ACTION_GATE_ACCEL)]
        gates.extend(
            (f"Gate {gate_id}", gate_id)
            for gate_id in range(
                len(
                    [
                        point
                        for point in self.project.points
                        if point.gate_id != 0xFF
                    ]
                )
            )
        )
        for row, action in enumerate(self.project.actions):
            self._set_item(self.action_table, row, 0, str(action.action_seq), False)
            self._set_combo(
                self.action_table,
                row,
                1,
                [(name, code) for code, name in ACTIONS.items()],
                action.action,
                self.on_action_code_changed,
            )
            self._set_combo(
                self.action_table,
                row,
                2,
                gates,
                action.unlock_gate_id,
                self.on_action_gate_changed,
            )
            values = (
                action.flags,
                action.timeout_ms,
                action.arm_s_mm,
                action.disarm_s_mm,
                action.accel_limit_mmps2,
                action.beta_limit_ddegps2,
                action.speed_limit_mmps,
                action.stable_time_ms,
            )
            for column, value in enumerate(values, start=3):
                self._set_item(self.action_table, row, column, str(value))
        if selected is not None and 0 <= selected < len(self.project.actions):
            self.action_table.selectRow(selected)

    def refresh_parameter_widgets(self):
        p = self.project
        values = {
            "planner.max_speed_mps": p.planner.max_speed_mmps / 1000.0,
            "planner.linear_accel_mps2": p.planner.linear_accel_mmps2 / 1000.0,
            "planner.lateral_accel_mps2": p.planner.lateral_accel_mmps2 / 1000.0,
            "planner.max_wz_radps": p.planner.max_wz_radps,
            "planner.angular_accel_moving": p.planner.angular_accel_moving_radps2,
            "planner.angular_accel_rotate": p.planner.angular_accel_rotate_radps2,
            "planner.nominal_spacing_mm": p.planner.nominal_spacing_mm,
            "planner.max_spacing_mm": p.planner.max_spacing_mm,
            "planner.max_ref_lead_mm": p.planner.max_ref_lead_mm,
            "overlay.scale_mode": p.overlay.scale_mode,
            "project.collision_check_enabled": p.collision_check_enabled,
            "project.reachability_check_enabled": p.reachability_check_enabled,
            "cut_in.capture_radius_mm": p.cut_in.capture_radius_mm,
            "cut_in.target_speed_mps": p.cut_in.target_speed_mmps / 1000.0,
            "cut_in.approach_max_speed_mps": p.cut_in.approach_max_speed_mmps / 1000.0,
            "cut_in.straight_length_mm": p.cut_in.straight_length_mm,
            "cut_in.yaw_tolerance_deg": p.cut_in.yaw_tolerance_ddeg / 10.0,
            "cut_in.tangent_tolerance_deg": p.cut_in.tangent_tolerance_ddeg / 10.0,
            "cut_in.align_yaw": p.cut_in.align_yaw,
            "cut_in.allow_first_segment_capture": p.cut_in.allow_first_segment_capture,
            "vehicle.wheel_radius_mm": p.vehicle_profile.wheel_radius_mm,
            "vehicle.rotation_radius_mm": p.vehicle_profile.rotation_radius_mm,
            "vehicle.wheel_plan_limit_rpm": p.vehicle_profile.wheel_plan_limit_rpm,
            "vehicle.wheel_hard_limit_rpm": p.vehicle_profile.wheel_hard_limit_rpm,
            "vehicle.mecanum_convention": p.vehicle_profile.mecanum_convention,
            "preview.enabled": p.preview_initial_pose.enabled,
            "preview.x_mm": p.preview_initial_pose.x_mm,
            "preview.y_mm": p.preview_initial_pose.y_mm,
            "preview.yaw_deg": p.preview_initial_pose.yaw_ddeg / 10.0,
            "preview.initial_speed_mps": p.preview_initial_pose.initial_speed_mmps / 1000.0,
            "mechanism.drop_safety_margin_ms": (
                p.mechanism_profile.drop_safety_margin_ms
            ),
        }
        for name, duration in p.mechanism_profile.action_duration_ms.items():
            values[f"mechanism.duration.{name}"] = duration
        for key, value in values.items():
            widget = self.param_widgets.get(key)
            if isinstance(widget, (QSpinBox, QDoubleSpinBox)):
                widget.setValue(value)
            elif isinstance(widget, QCheckBox):
                widget.setChecked(bool(value))
            elif isinstance(widget, QComboBox):
                widget.setCurrentIndex(widget.findData(value))

    def apply_parameter_widgets(self):
        def value(key: str):
            widget = self.param_widgets[key]
            if isinstance(widget, (QSpinBox, QDoubleSpinBox)):
                return widget.value()
            if isinstance(widget, QCheckBox):
                return widget.isChecked()
            if isinstance(widget, QComboBox):
                return widget.currentData()
            raise TypeError(key)

        p = self.project
        p.planner.max_speed_mmps = int(round(value("planner.max_speed_mps") * 1000))
        p.planner.linear_accel_mmps2 = int(
            round(value("planner.linear_accel_mps2") * 1000)
        )
        p.planner.lateral_accel_mmps2 = int(
            round(value("planner.lateral_accel_mps2") * 1000)
        )
        p.planner.max_wz_radps = value("planner.max_wz_radps")
        p.planner.angular_accel_moving_radps2 = value(
            "planner.angular_accel_moving"
        )
        p.planner.angular_accel_rotate_radps2 = value(
            "planner.angular_accel_rotate"
        )
        p.planner.nominal_spacing_mm = value("planner.nominal_spacing_mm")
        p.planner.max_spacing_mm = value("planner.max_spacing_mm")
        p.planner.max_ref_lead_mm = value("planner.max_ref_lead_mm")
        p.overlay.scale_mode = value("overlay.scale_mode")
        p.collision_check_enabled = value("project.collision_check_enabled")
        p.reachability_check_enabled = value("project.reachability_check_enabled")
        p.cut_in.capture_radius_mm = value("cut_in.capture_radius_mm")
        p.cut_in.target_speed_mmps = int(
            round(value("cut_in.target_speed_mps") * 1000)
        )
        p.cut_in.approach_max_speed_mmps = int(
            round(value("cut_in.approach_max_speed_mps") * 1000)
        )
        p.cut_in.straight_length_mm = value("cut_in.straight_length_mm")
        p.cut_in.yaw_tolerance_ddeg = int(
            round(value("cut_in.yaw_tolerance_deg") * 10)
        )
        p.cut_in.tangent_tolerance_ddeg = int(
            round(value("cut_in.tangent_tolerance_deg") * 10)
        )
        p.cut_in.align_yaw = value("cut_in.align_yaw")
        p.cut_in.allow_first_segment_capture = value(
            "cut_in.allow_first_segment_capture"
        )
        p.vehicle_profile.wheel_radius_mm = value("vehicle.wheel_radius_mm")
        p.vehicle_profile.rotation_radius_mm = value("vehicle.rotation_radius_mm")
        p.vehicle_profile.wheel_plan_limit_rpm = value(
            "vehicle.wheel_plan_limit_rpm"
        )
        p.vehicle_profile.wheel_hard_limit_rpm = value(
            "vehicle.wheel_hard_limit_rpm"
        )
        p.vehicle_profile.mecanum_convention = value(
            "vehicle.mecanum_convention"
        )
        p.preview_initial_pose.enabled = value("preview.enabled")
        p.preview_initial_pose.x_mm = value("preview.x_mm")
        p.preview_initial_pose.y_mm = value("preview.y_mm")
        p.preview_initial_pose.yaw_ddeg = int(round(value("preview.yaw_deg") * 10))
        p.preview_initial_pose.initial_speed_mmps = (
            value("preview.initial_speed_mps") * 1000
        )
        p.mechanism_profile.drop_safety_margin_ms = value(
            "mechanism.drop_safety_margin_ms"
        )
        for action_name in ACTIONS.values():
            p.mechanism_profile.action_duration_ms[action_name] = value(
                f"mechanism.duration.{action_name}"
            )

    def refresh_field(self, selected: Optional[int] = None):
        if self.project.overlay.scale_mode == "competition":
            speed_max = 2000
            beta_max = 2.0
        else:
            speed_max = self.project.planner.max_speed_mmps
            beta_max = self.project.planner.angular_accel_moving_radps2
        accel_max = 1600
        wz_max = 6.0
        self.field.rebuild(
            self.project,
            self.plan_result,
            selected,
            self.analysis_combo.currentData()
            if hasattr(self, "analysis_combo")
            else ANALYSIS_MODE_NORMAL,
            speed_max,
            accel_max,
            wz_max,
            beta_max,
        )
        self.update_legend()

    def update_legend(self):
        mode = (
            self.analysis_combo.currentData()
            if hasattr(self, "analysis_combo")
            else ANALYSIS_MODE_NORMAL
        )
        if self.project.overlay.scale_mode == "competition":
            speed_max = 2000
            beta_max = 2.0
        else:
            speed_max = self.project.planner.max_speed_mmps
            beta_max = self.project.planner.angular_accel_moving_radps2
        entries = legend_entries(mode, speed_max, 1600, 6.0, beta_max)
        self.legend_label.setText(
            "图例: "
            + "  ".join(
                f"<span style='color:{color}; font-weight:bold'>■</span> {label}"
                for label, color in entries
            )
        )

    def update_status(self, message: str = ""):
        prefix = (
            f"编辑点 {len(self.project.points)}/{MAX_EDIT_POINTS} | "
            f"动作 {len(self.project.actions)}/{MAX_ACTIONS}"
        )
        if self.plan_result is None:
            details = f"规划失败: {self.plan_error}" if self.plan_error else "等待规划"
        else:
            summary = self.plan_result.summary
            details = (
                f"节点 {len(self.plan_result.nodes)} | "
                f"Gate {gate_count_from_nodes(self.plan_result.nodes)}/{MAX_GATES} | "
                f"长度 {summary.total_length_mm:.0f} mm | "
                f"正式 {summary.formal_time_ms / 1000:.2f} s | "
                f"总预计 {summary.estimated_total_time_ms / 1000:.2f} s | "
                f"最大轮速 {summary.max_wheel_rpm:.1f} rpm"
            )
        self.status_label.setText(
            f"{message}\n{prefix}\n{details}" if message else f"{prefix}\n{details}"
        )

    def schedule_plan(self):
        self.plan_timer.start()
        self.update_status("参数已修改，等待重新规划")

    def plan_now(self):
        self.plan_timer.stop()
        self.renumber_points()
        self.renumber_actions()
        if not self.project.points:
            self.plan_result = None
            self.plan_error = ""
            self.refresh_field()
            self.update_status("等待添加路径点")
            return
        try:
            self.plan_result = plan_project(self.project)
            self.plan_error = ""
            selected_action = self.selected_action_row()
            self.updating_ui = True
            self.refresh_action_table(selected_action)
            self.updating_ui = False
            self.refresh_field(self.selected_point_row())
            self.update_status("规划与 V3.3 静态校验通过")
        except Exception as exc:
            self.plan_result = None
            self.plan_error = str(exc)
            self.refresh_field(self.selected_point_row())
            self.update_status()

    def renumber_points(self):
        for index, point in enumerate(self.project.points):
            point.point_id = index

    def renumber_actions(self):
        for index, action in enumerate(self.project.actions):
            action.action_seq = index

    def _traj_id_changed(self, value: int):
        if self.updating_ui:
            return
        self.project.traj_id = value
        self.update_status()

    def parameter_changed(self, *_args):
        if self.updating_ui:
            return
        self.apply_parameter_widgets()
        self.schedule_plan()

    def analysis_mode_changed(self, _index: int):
        if self.updating_ui:
            return
        self.project.overlay.selected_analysis_mode = self.analysis_combo.currentData()
        self.refresh_field(self.selected_point_row())

    def select_point_row(self, index: int):
        if 0 <= index < self.point_table.rowCount():
            self.point_table.selectRow(index)

    def on_point_selection_changed(self):
        if not self.updating_ui:
            self.field.set_selected_index(self.selected_point_row())

    def add_point_from_canvas(self, x_mm: int, y_mm: int):
        if len(self.project.points) >= MAX_EDIT_POINTS:
            QMessageBox.warning(self, "点数已满", f"最多 {MAX_EDIT_POINTS} 个编辑点")
            return
        point_type = POINT_TYPE_CUT_IN if not self.project.points else POINT_TYPE_WAYPOINT
        point = EditPoint(
            point_id=len(self.project.points),
            type=point_type,
            x_mm=x_mm,
            y_mm=y_mm,
            yaw_ddeg=0 if point_type == POINT_TYPE_CUT_IN else YAW_UNSPECIFIED_DDEG,
            exact_pass=point_type == POINT_TYPE_CUT_IN,
        )
        self.project.points.append(point)
        self.refresh_all(selected_point=len(self.project.points) - 1)
        self.schedule_plan()

    def add_default_point(self):
        if self.project.points:
            x_mm = clamp(
                int(self.project.points[-1].x_mm) + 200,
                FIELD_X_MIN_MM,
                FIELD_X_MAX_MM,
            )
            y_mm = int(self.project.points[-1].y_mm)
        else:
            x_mm, y_mm = 0, 0
        self.add_point_from_canvas(x_mm, y_mm)

    def insert_point(self):
        row = self.selected_point_row()
        if row is None:
            row = len(self.project.points)
        if len(self.project.points) >= MAX_EDIT_POINTS:
            return
        base = self.project.points[row] if row < len(self.project.points) else None
        point = EditPoint(
            x_mm=base.x_mm if base else 0,
            y_mm=base.y_mm if base else 0,
            yaw_ddeg=YAW_UNSPECIFIED_DDEG,
        )
        self.project.points.insert(row, point)
        self.refresh_all(selected_point=row)
        self.schedule_plan()

    def delete_point(self):
        row = self.selected_point_row()
        if row is None:
            return
        del self.project.points[row]
        self.refresh_all(selected_point=min(row, len(self.project.points) - 1))
        self.schedule_plan()

    def move_point(self, offset: int):
        row = self.selected_point_row()
        target = row + offset if row is not None else -1
        if row is None or not 0 <= target < len(self.project.points):
            return
        self.project.points[row], self.project.points[target] = (
            self.project.points[target],
            self.project.points[row],
        )
        self.refresh_all(selected_point=target)
        self.schedule_plan()

    def mark_end(self):
        row = self.selected_point_row()
        if row is None:
            return
        for point in self.project.points:
            point.is_end = False
        point = self.project.points[row]
        point.type = POINT_TYPE_ARRIVAL
        if point.yaw_ddeg == YAW_UNSPECIFIED_DDEG:
            point.yaw_ddeg = 0
        point.is_end = True
        point.stop_required = True
        self.refresh_all(selected_point=row)
        self.schedule_plan()

    def renumber_gates(self):
        mapping: Dict[int, int] = {}
        next_gate = 0
        for point in self.project.points:
            if point.gate_id == 0xFF:
                continue
            mapping.setdefault(point.gate_id, next_gate)
            point.gate_id = next_gate
            next_gate += 1
        for action in self.project.actions:
            if action.unlock_gate_id in mapping:
                action.unlock_gate_id = mapping[action.unlock_gate_id]
        self.refresh_all(selected_point=self.selected_point_row())
        self.schedule_plan()

    def on_point_moved(self, index: int, x_mm: int, y_mm: int):
        if not 0 <= index < len(self.project.points):
            return
        self.project.points[index].x_mm = x_mm
        self.project.points[index].y_mm = y_mm
        self.updating_ui = True
        self.point_table.item(index, 2).setText(str(x_mm))
        self.point_table.item(index, 3).setText(str(y_mm))
        self.updating_ui = False
        self.update_status("正在移动编辑点，松开后重新规划")

    def on_yaw_changed(self, index: int, yaw_ddeg: int):
        if not 0 <= index < len(self.project.points):
            return
        if self.project.points[index].type == POINT_TYPE_WAYPOINT:
            return
        self.project.points[index].yaw_ddeg = yaw_ddeg
        self.updating_ui = True
        self.point_table.item(index, 4).setText(str(yaw_ddeg))
        self.updating_ui = False
        self.update_status("正在调整 yaw，松开后重新规划")

    def on_point_type_changed(self, row: int, value):
        if self.updating_ui or not 0 <= row < len(self.project.points):
            return
        point = self.project.points[row]
        point.type = str(value)
        if point.type == POINT_TYPE_WAYPOINT:
            point.yaw_ddeg = YAW_UNSPECIFIED_DDEG
        elif point.yaw_ddeg == YAW_UNSPECIFIED_DDEG:
            point.yaw_ddeg = 0
        self.updating_ui = True
        self.refresh_point_table(row)
        self.updating_ui = False
        self.schedule_plan()
        self.refresh_field(row)

    def on_point_gate_changed(self, row: int, value):
        if self.updating_ui or not 0 <= row < len(self.project.points):
            return
        self.project.points[row].gate_id = int(value)
        self.schedule_plan()
        self.refresh_field(row)

    def on_point_item_changed(self, item: QTableWidgetItem):
        if self.updating_ui:
            return
        row, column = item.row(), item.column()
        if not 0 <= row < len(self.project.points):
            return
        point = self.project.points[row]
        text = item.text()
        try:
            if column == 2:
                point.x_mm = float(text)
            elif column == 3:
                point.y_mm = float(text)
            elif column == 4:
                if point.type == POINT_TYPE_WAYPOINT:
                    point.yaw_ddeg = YAW_UNSPECIFIED_DDEG
                else:
                    point.yaw_ddeg = parse_editor_int(text)
            elif column == 5:
                point.max_speed_mmps = max(0, parse_editor_int(text))
            elif column == 6:
                point.corner_trim_mm = max(0.0, float(text))
            elif column == 7:
                point.exact_pass = bool(parse_editor_int(text))
            elif column == 8:
                point.stop_required = bool(parse_editor_int(text))
            elif column == 10:
                point.scan = bool(parse_editor_int(text))
            elif column == 11:
                point.is_end = bool(parse_editor_int(text))
            else:
                return
        except ValueError:
            self.refresh_all(selected_point=row)
            return
        self.refresh_field(row)
        self.schedule_plan()

    def add_action(self):
        if len(self.project.actions) >= MAX_ACTIONS:
            return
        self.project.actions.append(
            MechanicalAction(action_seq=len(self.project.actions))
        )
        self.refresh_all(selected_action=len(self.project.actions) - 1)
        self.schedule_plan()

    def delete_action(self):
        row = self.selected_action_row()
        if row is None:
            return
        del self.project.actions[row]
        self.refresh_all(selected_action=min(row, len(self.project.actions) - 1))
        self.schedule_plan()

    def move_action(self, offset: int):
        row = self.selected_action_row()
        target = row + offset if row is not None else -1
        if row is None or not 0 <= target < len(self.project.actions):
            return
        self.project.actions[row], self.project.actions[target] = (
            self.project.actions[target],
            self.project.actions[row],
        )
        self.refresh_all(selected_action=target)
        self.schedule_plan()

    def toggle_action_flag(self, flag: int):
        row = self.selected_action_row()
        if row is None:
            return
        self.project.actions[row].flags ^= flag
        self.refresh_all(selected_action=row)
        self.schedule_plan()

    def on_action_code_changed(self, row: int, value):
        if self.updating_ui or not 0 <= row < len(self.project.actions):
            return
        self.project.actions[row].action = int(value)
        self.schedule_plan()

    def on_action_gate_changed(self, row: int, value):
        if self.updating_ui or not 0 <= row < len(self.project.actions):
            return
        self.project.actions[row].unlock_gate_id = int(value)
        self.schedule_plan()

    def on_action_item_changed(self, item: QTableWidgetItem):
        if self.updating_ui:
            return
        row, column = item.row(), item.column()
        if not 0 <= row < len(self.project.actions) or column < 3:
            return
        action = self.project.actions[row]
        value = parse_editor_int(item.text())
        fields = (
            "flags",
            "timeout_ms",
            "arm_s_mm",
            "disarm_s_mm",
            "accel_limit_mmps2",
            "beta_limit_ddegps2",
            "speed_limit_mmps",
            "stable_time_ms",
        )
        setattr(action, fields[column - 3], clamp(value, 0, 0xFFFF))
        self.schedule_plan()

    def new_project(self):
        if (
            QMessageBox.question(self, "新建", "确定新建 V3.3 工程？")
            != QMessageBox.Yes
        ):
            return
        self.project = make_default_project()
        self.plan_result = None
        self.current_json_path = None
        self.refresh_all()
        self.plan_now()

    def clear_project(self):
        if (
            QMessageBox.question(
                self,
                "清空",
                "确定清空全部编辑点和机械动作？\n规划参数和车辆参数将保留。",
            )
            != QMessageBox.Yes
        ):
            return
        self.plan_timer.stop()
        self.project.points.clear()
        self.project.actions.clear()
        self.plan_result = None
        self.plan_error = ""
        self.current_json_path = None
        self.refresh_all()
        self.update_status("已清空路径内容")

    def open_json(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "导入 V3.3 配置 JSON", "", "JSON (*.json)"
        )
        if not path:
            return
        try:
            self.project = load_project_dict(
                json.loads(Path(path).read_text(encoding="utf-8"))
            )
            self.plan_result = None
            self.current_json_path = Path(path)
            self.analysis_combo.setCurrentIndex(
                max(
                    0,
                    self.analysis_combo.findData(
                        self.project.overlay.selected_analysis_mode
                    ),
                )
            )
            self.refresh_all(selected_point=0 if self.project.points else None)
            self.plan_now()
        except Exception as exc:
            QMessageBox.critical(self, "打开失败", str(exc))

    def save_json(self):
        if self.current_json_path is None:
            self.save_json_as()
            return
        try:
            save_project_json(self.project, self.current_json_path)
            self.update_status(f"已保存 {self.current_json_path}")
        except Exception as exc:
            QMessageBox.critical(self, "保存失败", str(exc))

    def save_json_as(self):
        default = f"P{self.project.traj_id:04d}.json"
        path, _ = QFileDialog.getSaveFileName(
            self, "导出 V3.3 配置 JSON", default, "JSON (*.json)"
        )
        if not path:
            return
        self.current_json_path = Path(path)
        self.save_json()

    def export_bin(self):
        self.plan_now()
        if self.plan_result is None:
            QMessageBox.critical(self, "导出失败", self.plan_error)
            return
        default = f"P{self.project.traj_id:04d}.BIN"
        path, _ = QFileDialog.getSaveFileName(
            self, "导出 HJMB V3.3 BIN", default, "BIN (*.BIN *.bin)"
        )
        if not path:
            return
        try:
            bin_path = Path(path)
            if bin_path_traj_id(bin_path) != self.project.traj_id:
                raise ValueError(
                    f"文件名必须与 traj_id 一致，应为 P{self.project.traj_id:04d}.BIN"
                )
            data = PathCodec.build_bin(self.project, self.plan_result)
            bin_path.write_bytes(data)
            PathCodec.parse_bin(data, self.project.traj_id)
            QMessageBox.information(
                self, "导出成功", f"V3.3 BIN 已导出并回读校验：\n{path}"
            )
            self.update_status(f"已导出并校验 {path}")
        except Exception as exc:
            QMessageBox.critical(self, "导出失败", str(exc))

    def open_bin(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "打开 HJMB V3.3 BIN", "", "BIN (*.BIN *.bin)"
        )
        if not path:
            return
        try:
            bin_path = Path(path)
            parsed = PathCodec.parse_bin(
                bin_path.read_bytes(), bin_path_traj_id(bin_path)
            )
            project = PathProject()
            project.traj_id = parsed.header.traj_id
            project.field.width_mm = parsed.header.field_width_mm
            project.field.height_mm = parsed.header.field_height_mm
            project.planner.nominal_spacing_mm = parsed.header.nominal_spacing_mm
            project.cut_in.capture_radius_mm = (
                parsed.header.cut_in_capture_radius_mm
            )
            project.cut_in.target_speed_mmps = parsed.header.cut_in_speed_mmps
            project.cut_in.approach_max_speed_mmps = (
                parsed.header.approach_max_speed_mmps
            )
            project.cut_in.straight_length_mm = (
                parsed.header.cut_in_straight_length_mm
            )
            project.cut_in.yaw_tolerance_ddeg = (
                parsed.header.cut_in_yaw_tolerance_ddeg
            )
            project.cut_in.tangent_tolerance_ddeg = (
                parsed.header.cut_in_tangent_tolerance_ddeg
            )
            for node in parsed.nodes:
                point_type = None
                if node.flags & TRAJ_FLAG_CUT_IN:
                    point_type = POINT_TYPE_CUT_IN
                elif node.flags & TRAJ_FLAG_ARRIVAL:
                    point_type = POINT_TYPE_ARRIVAL
                elif node.flags & TRAJ_FLAG_WAYPOINT:
                    point_type = POINT_TYPE_WAYPOINT
                if point_type is None:
                    continue
                project.points.append(
                    EditPoint(
                        point_id=len(project.points),
                        type=point_type,
                        x_mm=node.x_mm,
                        y_mm=node.y_mm,
                        yaw_ddeg=(
                            YAW_UNSPECIFIED_DDEG
                            if point_type == POINT_TYPE_WAYPOINT
                            else int(round(math.degrees(node.yaw_rad) * 10))
                        ),
                        exact_pass=point_type != POINT_TYPE_ARRIVAL,
                        stop_required=bool(node.flags & TRAJ_FLAG_STOP),
                        gate_id=node.gate_id,
                        scan=bool(node.flags & TRAJ_FLAG_SCAN),
                        is_end=bool(node.flags & TRAJ_FLAG_END),
                    )
                )
            project.actions = parsed.actions
            summary = PlanSummary(
                total_length_mm=parsed.header.total_length_mm,
                formal_time_ms=parsed.header.planned_time_ms,
                max_speed_mmps=max(node.speed_mmps for node in parsed.nodes),
            )
            self.project = project
            self.plan_result = PlanResult(
                nodes=parsed.nodes,
                actions=parsed.actions,
                summary=summary,
                cut_in_preview=CutInPreviewResult(),
                warnings=["BIN 仅含稠密轨迹；普通塑形 WAYPOINT 无法无损恢复"],
            )
            self.current_json_path = None
            self.plan_error = ""
            self.refresh_all(selected_point=0, rebuild_field=False)
            self.refresh_field(0)
            self.update_status("已打开 BIN 进行检查；重新编辑后会按恢复出的精确点重新规划")
        except Exception as exc:
            QMessageBox.critical(self, "打开 BIN 失败", str(exc))

    def validate_current_project(self):
        self.plan_now()
        if self.plan_result is None:
            QMessageBox.warning(self, "校验失败", self.plan_error)
            return
        try:
            data = PathCodec.build_bin(self.project, self.plan_result)
            parsed = PathCodec.parse_bin(data, self.project.traj_id)
            QMessageBox.information(
                self,
                "校验通过",
                "\n".join(
                    (
                        f"traj_id={parsed.header.traj_id}",
                        f"node_count={parsed.header.node_count}",
                        f"action_count={parsed.header.action_count}",
                        f"gate_count={parsed.header.gate_count}",
                        f"planned_time_ms={parsed.header.planned_time_ms}",
                        f"CRC32=0x{parsed.header.file_crc32:08X}",
                    )
                ),
            )
        except Exception as exc:
            QMessageBox.warning(self, "校验失败", str(exc))


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

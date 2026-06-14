# -*- coding: utf-8 -*-
"""HJMB 搬运豆子机器人 V2.5 路径、Gate 和机械动作 FIFO 编辑器。"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QAction, QBrush, QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import (
    QApplication,
    QAbstractItemView,
    QComboBox,
    QFileDialog,
    QGraphicsEllipseItem,
    QGraphicsLineItem,
    QGraphicsRectItem,
    QGraphicsScene,
    QGraphicsSimpleTextItem,
    QGraphicsView,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
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
    ACTION_FLAG_HOLD_PATH,
    ACTION_FLAG_LOCKED,
    ACTION_FLAG_REQUIRED_AT_END,
    ACTION_FLAGS,
    ACTIONS,
    MAX_ACTIONS,
    MAX_GATES,
    MAX_POINTS,
    MAX_TRAJ_ID,
    PATH_FLAG_END,
    PATH_FLAG_SKIP_SCAN,
    PATH_FLAG_SLOW_ZONE,
    PATH_POINT_ARRIVE_SCAN,
    PATH_POINT_PASS,
    POINT_TYPES,
    MechanicalAction,
    PathCodec,
    PathPoint,
    bin_path_traj_id,
    flags_to_text,
    gate_count_from_points,
    hex8,
    load_project_dict,
    project_to_dict,
    validate_project,
)

# 场地尺寸按 2026 物流技术创意赛图 3 和正文描述绘制。
FIELD_W_MM = 4000
FIELD_H_MM = 2000
FIELD_HALF_W_MM = FIELD_W_MM // 2
FIELD_HALF_H_MM = FIELD_H_MM // 2
FIELD_X_MIN_MM = -FIELD_HALF_W_MM
FIELD_X_MAX_MM = FIELD_HALF_W_MM
FIELD_Y_MIN_MM = -FIELD_HALF_H_MM
FIELD_Y_MAX_MM = FIELD_HALF_H_MM
FENCE_W_MM = 35
FENCE_H_MM = 100
FIELD_SCALE = 0.25  # 1 mm -> 0.25 px，4000mm 显示为 1000px
SCENE_MARGIN_PX = 24
YAW_ARROW_LENGTH_MM = 120

A4_PAPER_W_MM = 210
A4_PAPER_H_MM = 300
CARGO_BOX_W_MM = 280
CARGO_BOX_H_MM = 200
OBSTACLE_D_MM = 102
START_AREA_SIZE_MM = 400

PICKUP_STATIONS = (
    (1, 1800, 500, A4_PAPER_W_MM, A4_PAPER_H_MM),
    (2, 1500, 0, A4_PAPER_W_MM, A4_PAPER_H_MM),
    (3, 1800, -500, A4_PAPER_W_MM, A4_PAPER_H_MM),
)

DROP_STATIONS = (
    (4, -1500, 800, CARGO_BOX_W_MM, CARGO_BOX_H_MM),
    (5, -1700, 400, CARGO_BOX_H_MM, CARGO_BOX_W_MM),
    (6, -1700, 0, CARGO_BOX_H_MM, CARGO_BOX_W_MM),
    (7, -1700, -400, CARGO_BOX_H_MM, CARGO_BOX_W_MM),
    (8, -1500, -800, CARGO_BOX_W_MM, CARGO_BOX_H_MM),
)

OBSTACLE_CENTERS = ((1000, 0), (-1000, 0))

PATH_FLAGS: Dict[int, str] = {
    PATH_FLAG_SKIP_SCAN: "SKIP_SCAN",
    PATH_FLAG_SLOW_ZONE: "SLOW_ZONE",
    PATH_FLAG_END: "END",
}


def point_type_brush_pen(point_type: int, selected: bool = False) -> Tuple[QBrush, QPen]:
    if point_type == PATH_POINT_ARRIVE_SCAN:
        brush_color = QColor(40, 175, 105)
        pen_color = QColor(10, 95, 55)
    elif point_type == PATH_POINT_PASS:
        brush_color = QColor(255, 70, 70)
        pen_color = QColor(120, 0, 0)
    else:
        brush_color = QColor(150, 150, 150)
        pen_color = QColor(70, 70, 70)

    if selected:
        pen_color = QColor(245, 165, 0)
    return QBrush(brush_color), QPen(pen_color, 2.6 if selected else 1.5)


def parse_editor_int(value, default: int = 0) -> int:
    """Parse an editable table value without making a partial edit crash the UI."""
    s = str(value).strip()
    if not s:
        return default
    try:
        return int(s.split()[0], 0)
    except ValueError:
        return default


def clamp(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, int(v)))


class DraggablePointItem(QGraphicsEllipseItem):
    """可拖动的路径点图元。"""

    def __init__(
        self,
        index: int,
        editor: "FieldView",
        point_type: int = PATH_POINT_PASS,
        selected: bool = False,
        radius_px: float = 7.0,
    ):
        super().__init__(-radius_px, -radius_px, radius_px * 2, radius_px * 2)
        self.index = index
        self.editor = editor
        brush, pen = point_type_brush_pen(point_type, selected)
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
            pos = value
            x_mm, y_mm = self.editor.scene_to_world(pos)
            x_mm = clamp(x_mm, FIELD_X_MIN_MM, FIELD_X_MAX_MM)
            y_mm = clamp(y_mm, FIELD_Y_MIN_MM, FIELD_Y_MAX_MM)
            return self.editor.world_to_scene(x_mm, y_mm)
        if change == QGraphicsEllipseItem.ItemPositionHasChanged and self.scene() is not None:
            pos = self.pos()
            x_mm, y_mm = self.editor.scene_to_world(pos)
            self.editor.on_point_item_moved(self.index, x_mm, y_mm)
            self.editor.point_moved.emit(self.index, x_mm, y_mm)
        return super().itemChange(change, value)


class DraggableYawHandleItem(QGraphicsEllipseItem):
    """拖动路径点方向箭头末端来修改 yaw。"""

    def __init__(
        self,
        index: int,
        editor: "FieldView",
        center_x_mm: int,
        center_y_mm: int,
        line_item: QGraphicsLineItem,
        radius_px: float = 5.5,
    ):
        super().__init__(-radius_px, -radius_px, radius_px * 2, radius_px * 2)
        self.index = index
        self.editor = editor
        self.center_x_mm = center_x_mm
        self.center_y_mm = center_y_mm
        self.line_item = line_item
        self._ready = False
        self.setBrush(QBrush(QColor(55, 130, 255)))
        self.setPen(QPen(QColor(15, 65, 150), 1.5))
        self.setFlags(QGraphicsEllipseItem.ItemIsMovable | QGraphicsEllipseItem.ItemSendsGeometryChanges)
        self.setAcceptedMouseButtons(Qt.LeftButton)
        self.setCursor(Qt.OpenHandCursor)
        self.setToolTip("拖动调整车头方向 yaw")
        self.setZValue(115)

    def set_ready(self, ready: bool):
        self._ready = ready

    def _snapped_scene_pos(self, scene_pos: QPointF) -> QPointF:
        end_x_mm, end_y_mm = self.editor.scene_to_world_float(scene_pos)
        dx = end_x_mm - self.center_x_mm
        dy = end_y_mm - self.center_y_mm
        distance = math.hypot(dx, dy)
        if distance < 1e-6:
            return self.pos()

        end_x_mm = self.center_x_mm + dx / distance * YAW_ARROW_LENGTH_MM
        end_y_mm = self.center_y_mm + dy / distance * YAW_ARROW_LENGTH_MM
        return self.editor.world_to_scene(end_x_mm, end_y_mm)

    def itemChange(self, change, value):
        if change == QGraphicsEllipseItem.ItemPositionChange and self.scene() is not None:
            return self._snapped_scene_pos(value)
        if change == QGraphicsEllipseItem.ItemPositionHasChanged and self.scene() is not None and self._ready:
            self.editor.on_yaw_handle_moved(self.index, self.center_x_mm, self.center_y_mm, self.pos(), self.line_item)
        return super().itemChange(change, value)


class FieldView(QGraphicsView):
    """场地绘制和交互。"""

    add_point_requested = Signal(int, int)
    point_moved = Signal(int, int, int)
    yaw_changed = Signal(int, int)
    point_selected = Signal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.scene_obj = QGraphicsScene(self)
        self.setScene(self.scene_obj)
        self.setRenderHints(QPainter.Antialiasing | QPainter.TextAntialiasing)
        self.setDragMode(QGraphicsView.RubberBandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)
        self.setMouseTracking(True)
        self.points: List[PathPoint] = []
        self.selected_index: Optional[int] = None
        self.point_items: Dict[int, DraggablePointItem] = {}
        self.point_labels: Dict[int, QGraphicsSimpleTextItem] = {}
        self.path_line_items: List[QGraphicsLineItem] = []
        self.yaw_line_items: Dict[int, QGraphicsLineItem] = {}
        self.yaw_handle_items: Dict[int, DraggableYawHandleItem] = {}
        self._is_rebuilding = False
        self._auto_fit = True
        self.scene_obj.selectionChanged.connect(self._on_selection_changed)
        self.rebuild([])

    def world_to_scene(self, x_mm: float, y_mm: float) -> QPointF:
        return QPointF((x_mm + FIELD_HALF_W_MM) * FIELD_SCALE, (FIELD_HALF_H_MM - y_mm) * FIELD_SCALE)

    def scene_to_world_float(self, pos: QPointF) -> Tuple[float, float]:
        x_mm = pos.x() / FIELD_SCALE - FIELD_HALF_W_MM
        y_mm = FIELD_HALF_H_MM - pos.y() / FIELD_SCALE
        return x_mm, y_mm

    def scene_to_world(self, pos: QPointF) -> Tuple[int, int]:
        x_mm, y_mm = self.scene_to_world_float(pos)
        x_mm = int(round(x_mm))
        y_mm = int(round(y_mm))
        return x_mm, y_mm

    def add_world_rect(self, x: float, y: float, w: float, h: float, pen: QPen, brush: QBrush) -> QGraphicsRectItem:
        top_left = self.world_to_scene(x, y + h)
        item = self.scene_obj.addRect(top_left.x(), top_left.y(), w * FIELD_SCALE, h * FIELD_SCALE, pen, brush)
        return item

    def add_world_center_rect(
        self, cx: float, cy: float, w: float, h: float, pen: QPen, brush: QBrush
    ) -> QGraphicsRectItem:
        return self.add_world_rect(cx - w / 2, cy - h / 2, w, h, pen, brush)

    def add_world_ellipse(self, cx: float, cy: float, r: float, pen: QPen, brush: QBrush):
        pos = self.world_to_scene(cx - r, cy + r)
        return self.scene_obj.addEllipse(pos.x(), pos.y(), 2 * r * FIELD_SCALE, 2 * r * FIELD_SCALE, pen, brush)

    def add_world_line(self, x1: float, y1: float, x2: float, y2: float, pen: QPen) -> QGraphicsLineItem:
        p1 = self.world_to_scene(x1, y1)
        p2 = self.world_to_scene(x2, y2)
        return self.scene_obj.addLine(p1.x(), p1.y(), p2.x(), p2.y(), pen)

    def add_world_text(self, x: float, y: float, text: str, color: QColor = QColor(20, 20, 20), size: int = 10):
        pos = self.world_to_scene(x, y)
        item = self.scene_obj.addSimpleText(text)
        item.setBrush(QBrush(color))
        font = QFont()
        font.setPointSize(size)
        item.setFont(font)
        item.setPos(pos)
        item.setZValue(80)
        return item

    def draw_field(self):
        self.scene_obj.clear()
        self.scene_obj.setBackgroundBrush(QBrush(QColor(245, 247, 250)))

        # 围栏俯视宽度 35mm，场地坐标只以蓝色内场地 4000x2000mm 为准。
        fence_pen = QPen(QColor(150, 70, 50), 2)
        inner_pen = QPen(QColor(20, 45, 80), 2)
        self.add_world_rect(
            FIELD_X_MIN_MM - FENCE_W_MM,
            FIELD_Y_MIN_MM - FENCE_W_MM,
            FIELD_W_MM + FENCE_W_MM * 2,
            FIELD_H_MM + FENCE_W_MM * 2,
            fence_pen,
            QBrush(QColor(238, 184, 136)),
        )
        self.add_world_rect(
            FIELD_X_MIN_MM,
            FIELD_Y_MIN_MM,
            FIELD_W_MM,
            FIELD_H_MM,
            inner_pen,
            QBrush(QColor(240, 244, 250)),
        )

        # 网格，每 250 mm 一格，1000 mm 加粗。
        for x in range(FIELD_X_MIN_MM, FIELD_X_MAX_MM + 1, 250):
            pen = QPen(QColor(220, 225, 230), 1)
            if x % 1000 == 0:
                pen = QPen(QColor(190, 195, 205), 1.5)
            self.add_world_line(x, FIELD_Y_MIN_MM, x, FIELD_Y_MAX_MM, pen)
        for y in range(FIELD_Y_MIN_MM, FIELD_Y_MAX_MM + 1, 250):
            pen = QPen(QColor(220, 225, 230), 1)
            if y % 1000 == 0:
                pen = QPen(QColor(190, 195, 205), 1.5)
            self.add_world_line(FIELD_X_MIN_MM, y, FIELD_X_MAX_MM, y, pen)

        axis_pen = QPen(QColor(220, 40, 40), 2)
        axis_pen.setStyle(Qt.DashLine)
        self.add_world_line(0, FIELD_Y_MIN_MM, 0, FIELD_Y_MAX_MM, axis_pen)
        self.add_world_line(FIELD_X_MIN_MM, 0, FIELD_X_MAX_MM, 0, axis_pen)
        self.add_world_text(45, 45, "中心 (0,0)", QColor(190, 30, 30), 10)
        self.add_world_text(1510, -930, "+x 取豆区/前向", QColor(20, 120, 70), 11)
        self.add_world_text(-1840, -930, "放置区", QColor(20, 120, 70), 11)
        self.add_world_text(60, 865, "+y 左向", QColor(190, 30, 30), 10)

        path_pen = QPen(QColor(180, 100, 30), 2)
        path_pen.setStyle(Qt.DashLine)
        self.add_world_line(500, 0, 1000, 0, path_pen)
        self.add_world_line(-1000, 0, -500, 0, path_pen)

        self.add_world_rect(
            -START_AREA_SIZE_MM,
            -START_AREA_SIZE_MM / 2,
            START_AREA_SIZE_MM,
            START_AREA_SIZE_MM,
            QPen(QColor(20, 80, 150), 2),
            QBrush(QColor(205, 230, 255, 170)),
        )
        self.add_world_text(-350, 220, "起始区域", QColor(20, 80, 150), 9)

        obs_pen = QPen(QColor(130, 80, 50), 2)
        obs_brush = QBrush(QColor(240, 160, 90, 190))
        for cx, cy in OBSTACLE_CENTERS:
            self.add_world_ellipse(cx, cy, OBSTACLE_D_MM / 2, obs_pen, obs_brush)
            self.add_world_text(cx - 70, cy - 120, "障碍", QColor(80, 60, 40), 9)

        pickup_pen = QPen(QColor(20, 110, 130), 2)
        pickup_brush = QBrush(QColor(80, 190, 210, 170))
        for station_id, cx, cy, w, h in PICKUP_STATIONS:
            self.add_world_center_rect(cx, cy, w, h, pickup_pen, pickup_brush)
            self.add_world_text(cx - 20, cy + h / 2 + 42, str(station_id), QColor(0, 90, 110), 12)
        self.add_world_text(1630, 730, "取豆位 1-3", QColor(0, 90, 110), 10)

        drop_pen = QPen(QColor(80, 60, 160), 2)
        drop_brush = QBrush(QColor(180, 170, 230, 165))
        for station_id, cx, cy, w, h in DROP_STATIONS:
            self.add_world_center_rect(cx, cy, w, h, drop_pen, drop_brush)
            self.add_world_text(cx - 18, cy + h / 2 + 38, str(station_id), QColor(50, 40, 130), 12)
        self.add_world_text(-1820, 940, "放置位 4-8", QColor(50, 40, 130), 10)

        self.add_world_text(
            FIELD_X_MIN_MM,
            FIELD_Y_MIN_MM - 95,
            "坐标：中心为 (0,0)，+x 指向取豆区，+y 为面向取豆区时的左侧，单位 mm；内场地 4000x2000，围栏宽 35",
            QColor(90, 90, 90),
            9,
        )

        rect = QRectF(
            -FENCE_W_MM * FIELD_SCALE - SCENE_MARGIN_PX,
            -FENCE_W_MM * FIELD_SCALE - SCENE_MARGIN_PX,
            (FIELD_W_MM + FENCE_W_MM * 2) * FIELD_SCALE + SCENE_MARGIN_PX * 2,
            (FIELD_H_MM + FENCE_W_MM * 2) * FIELD_SCALE + SCENE_MARGIN_PX * 2,
        )
        self.scene_obj.setSceneRect(rect)

    def rebuild(self, points: List[PathPoint], selected_index: Optional[int] = None):
        self._is_rebuilding = True
        self.points = points
        self.selected_index = selected_index
        self.point_items = {}
        self.point_labels = {}
        self.path_line_items = []
        self.yaw_line_items = {}
        self.yaw_handle_items = {}
        self.draw_field()

        # 路径连线
        if len(points) >= 2:
            line_pen = QPen(QColor(240, 80, 40), 2.5)
            for a, b in zip(points[:-1], points[1:]):
                line_item = self.add_world_line(a.x_mm, a.y_mm, b.x_mm, b.y_mm, line_pen)
                self.path_line_items.append(line_item)

        # 路径点、点号、yaw箭头
        for i, p in enumerate(points):
            item = DraggablePointItem(i, self, p.type, i == selected_index)
            item.setPos(self.world_to_scene(p.x_mm, p.y_mm))
            self.scene_obj.addItem(item)
            self.point_items[i] = item

            label_text = str(p.point_id)
            if p.gate_id != 0xFF:
                label_text += f"  G{p.gate_id}"
            label = QGraphicsSimpleTextItem(label_text)
            label.setBrush(QBrush(QColor(130, 35, 160) if p.gate_id != 0xFF else QColor(0, 0, 0)))
            label.setFont(QFont("Arial", 10, QFont.Bold))
            label.setPos(item.pos() + QPointF(8, -22))
            label.setZValue(110)
            self.scene_obj.addItem(label)
            self.point_labels[i] = label

            # yaw 箭头：0度朝 +x，90度朝 +y。
            yaw_deg = p.yaw_ddeg / 10.0
            yaw_rad = math.radians(yaw_deg)
            end_x = p.x_mm + math.cos(yaw_rad) * YAW_ARROW_LENGTH_MM
            end_y = p.y_mm + math.sin(yaw_rad) * YAW_ARROW_LENGTH_MM
            arrow_pen = QPen(QColor(30, 30, 30), 2)
            arrow_line = self.add_world_line(p.x_mm, p.y_mm, end_x, end_y, arrow_pen)
            arrow_line.setZValue(105)
            self.yaw_line_items[i] = arrow_line

            yaw_handle = DraggableYawHandleItem(i, self, p.x_mm, p.y_mm, arrow_line)
            if i == selected_index:
                yaw_handle.setBrush(QBrush(QColor(255, 180, 60)))
                yaw_handle.setPen(QPen(QColor(155, 90, 0), 1.8))
            yaw_handle.setPos(self.world_to_scene(end_x, end_y))
            self.scene_obj.addItem(yaw_handle)
            self.yaw_handle_items[i] = yaw_handle
            yaw_handle.set_ready(True)

        self._is_rebuilding = False
        if self._auto_fit:
            self.fit_to_field()

    def _set_line_world(self, line_item: QGraphicsLineItem, x1: float, y1: float, x2: float, y2: float):
        p1 = self.world_to_scene(x1, y1)
        p2 = self.world_to_scene(x2, y2)
        line_item.setLine(p1.x(), p1.y(), p2.x(), p2.y())

    def _yaw_end_world(self, point: PathPoint) -> Tuple[float, float]:
        yaw_rad = math.radians(point.yaw_ddeg / 10.0)
        end_x = point.x_mm + math.cos(yaw_rad) * YAW_ARROW_LENGTH_MM
        end_y = point.y_mm + math.sin(yaw_rad) * YAW_ARROW_LENGTH_MM
        return end_x, end_y

    def _update_path_lines_for_point(self, index: int):
        if index > 0 and index - 1 < len(self.path_line_items):
            prev_point = self.points[index - 1]
            point = self.points[index]
            self._set_line_world(
                self.path_line_items[index - 1],
                prev_point.x_mm,
                prev_point.y_mm,
                point.x_mm,
                point.y_mm,
            )

        if index < len(self.points) - 1 and index < len(self.path_line_items):
            point = self.points[index]
            next_point = self.points[index + 1]
            self._set_line_world(
                self.path_line_items[index],
                point.x_mm,
                point.y_mm,
                next_point.x_mm,
                next_point.y_mm,
            )

    def _update_yaw_items_for_point(self, index: int):
        if not (0 <= index < len(self.points)):
            return

        point = self.points[index]
        end_x, end_y = self._yaw_end_world(point)
        yaw_line = self.yaw_line_items.get(index)
        if yaw_line is not None:
            self._set_line_world(yaw_line, point.x_mm, point.y_mm, end_x, end_y)

        yaw_handle = self.yaw_handle_items.get(index)
        if yaw_handle is not None:
            yaw_handle.set_ready(False)
            yaw_handle.center_x_mm = point.x_mm
            yaw_handle.center_y_mm = point.y_mm
            if yaw_line is not None:
                yaw_handle.line_item = yaw_line
            yaw_handle.setPos(self.world_to_scene(end_x, end_y))
            yaw_handle.set_ready(True)

    def on_point_item_moved(self, index: int, x_mm: int, y_mm: int):
        if not (0 <= index < len(self.points)):
            return

        self.points[index].x_mm = clamp(x_mm, FIELD_X_MIN_MM, FIELD_X_MAX_MM)
        self.points[index].y_mm = clamp(y_mm, FIELD_Y_MIN_MM, FIELD_Y_MAX_MM)

        label = self.point_labels.get(index)
        point_item = self.point_items.get(index)
        if label is not None and point_item is not None:
            label.setPos(point_item.pos() + QPointF(8, -22))

        self._update_path_lines_for_point(index)
        self._update_yaw_items_for_point(index)

    def on_yaw_handle_moved(
        self,
        index: int,
        center_x_mm: int,
        center_y_mm: int,
        handle_pos: QPointF,
        line_item: QGraphicsLineItem,
    ):
        center_pos = self.world_to_scene(center_x_mm, center_y_mm)
        line_item.setLine(center_pos.x(), center_pos.y(), handle_pos.x(), handle_pos.y())

        end_x_mm, end_y_mm = self.scene_to_world_float(handle_pos)
        yaw_rad = math.atan2(end_y_mm - center_y_mm, end_x_mm - center_x_mm)
        yaw_ddeg = int(round(math.degrees(yaw_rad) * 10)) % 3600
        if 0 <= index < len(self.points):
            self.points[index].yaw_ddeg = yaw_ddeg
        self.yaw_changed.emit(index, yaw_ddeg)

    def fit_to_field(self):
        self._auto_fit = True
        self.resetTransform()
        self.fitInView(self.scene_obj.sceneRect(), Qt.KeepAspectRatio)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self._auto_fit:
            self.fit_to_field()

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        if delta == 0:
            super().wheelEvent(event)
            return

        factor = 1.15 if delta > 0 else 1 / 1.15
        next_scale = self.transform().m11() * factor
        if not (0.2 <= next_scale <= 8.0):
            event.accept()
            return

        self._auto_fit = False
        self.scale(factor, factor)
        event.accept()

    def mouseDoubleClickEvent(self, event):
        if event.button() == Qt.LeftButton:
            scene_pos = self.mapToScene(event.pos())
            x, y = self.scene_to_world(scene_pos)
            if FIELD_X_MIN_MM <= x <= FIELD_X_MAX_MM and FIELD_Y_MIN_MM <= y <= FIELD_Y_MAX_MM:
                self.add_point_requested.emit(x, y)
                return
        super().mouseDoubleClickEvent(event)

    def mousePressEvent(self, event):
        # 单击空白处添加点会误触拖动，故采用双击添加点。
        super().mousePressEvent(event)

    def _on_selection_changed(self):
        if self._is_rebuilding:
            return
        selected = self.scene_obj.selectedItems()
        for item in selected:
            if isinstance(item, DraggablePointItem):
                self.point_selected.emit(item.index)
                return


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("HJMB 路径编辑器 V2.5")
        self.resize(1600, 900)

        self.points: List[PathPoint] = []
        self.actions: List[MechanicalAction] = []
        self.current_json_path: Optional[Path] = None
        self._updating_tables = False

        self.field = FieldView(self)
        self.field.add_point_requested.connect(self.add_point_from_canvas)
        self.field.point_moved.connect(self.on_point_moved)
        self.field.yaw_changed.connect(self.on_yaw_changed)
        self.field.point_selected.connect(self.select_point_row)

        self.traj_id_spin = QSpinBox()
        self.traj_id_spin.setRange(0, MAX_TRAJ_ID)
        self.traj_id_spin.setValue(0)
        self.traj_id_spin.valueChanged.connect(lambda _value: self.update_status())
        self.status_label = QLabel()
        self.status_label.setWordWrap(True)

        self.point_table = self._create_point_table()
        self.action_table = self._create_action_table()

        right_panel = self._build_right_panel()
        right_panel.setMinimumWidth(620)
        right_panel.setMaximumWidth(820)
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self.field)
        splitter.addWidget(right_panel)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 0)
        splitter.setSizes([930, 670])
        self.setCentralWidget(splitter)
        self._build_toolbar()
        self.refresh_all()

    def _create_point_table(self) -> QTableWidget:
        table = QTableWidget(0, 8)
        table.setHorizontalHeaderLabels(
            ["point_id", "x_mm", "y_mm", "yaw_ddeg", "点类型", "gate_id", "marker_id", "flags"]
        )
        table.horizontalHeader().setSectionResizeMode(QHeaderView.Interactive)
        table.horizontalHeader().setMinimumSectionSize(54)
        for column, width in enumerate((64, 72, 72, 84, 180, 115, 82, 150)):
            table.setColumnWidth(column, width)
        table.verticalHeader().setVisible(False)
        table.setSelectionBehavior(QAbstractItemView.SelectRows)
        table.setSelectionMode(QAbstractItemView.SingleSelection)
        table.itemChanged.connect(self.on_point_item_changed)
        table.itemSelectionChanged.connect(self.on_point_selection_changed)
        return table

    def _create_action_table(self) -> QTableWidget:
        table = QTableWidget(0, 5)
        table.setHorizontalHeaderLabels(
            ["action_seq", "action", "unlock_gate_id", "flags", "timeout_ms"]
        )
        table.horizontalHeader().setSectionResizeMode(QHeaderView.Interactive)
        table.horizontalHeader().setMinimumSectionSize(70)
        for column, width in enumerate((82, 270, 145, 220, 95)):
            table.setColumnWidth(column, width)
        table.verticalHeader().setVisible(False)
        table.setSelectionBehavior(QAbstractItemView.SelectRows)
        table.setSelectionMode(QAbstractItemView.SingleSelection)
        table.itemChanged.connect(self.on_action_item_changed)
        return table

    def _build_toolbar(self):
        toolbar = QToolBar("工具")
        self.addToolBar(toolbar)

        actions = (
            ("新建", self.new_project),
            ("打开JSON", self.open_json),
            ("保存JSON", self.save_json),
            ("另存JSON", self.save_json_as),
        )
        for text, callback in actions:
            action = QAction(text, self)
            action.triggered.connect(callback)
            toolbar.addAction(action)

        toolbar.addSeparator()
        for text, callback in (("导出BIN", self.export_bin), ("打开BIN", self.open_bin)):
            action = QAction(text, self)
            action.triggered.connect(callback)
            toolbar.addAction(action)

        toolbar.addSeparator()
        fit_action = QAction("适配场地", self)
        fit_action.triggered.connect(self.field.fit_to_field)
        toolbar.addAction(fit_action)

        toolbar.addSeparator()
        validate_action = QAction("校验", self)
        validate_action.triggered.connect(self.validate_current_project)
        toolbar.addAction(validate_action)

    def _build_right_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)

        top_line = QHBoxLayout()
        top_line.addWidget(QLabel("traj_id:"))
        top_line.addWidget(self.traj_id_spin)
        top_line.addStretch(1)
        layout.addLayout(top_line)

        tabs = QTabWidget()
        tabs.addTab(self._build_point_tab(), "路径点")
        tabs.addTab(self._build_action_tab(), "机械动作")
        layout.addWidget(tabs, 1)
        layout.addWidget(self.status_label)
        return panel

    def _build_point_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.addWidget(self.point_table, 1)

        row1 = QHBoxLayout()
        for text, callback in (
            ("末尾添加", self.add_default_point),
            ("插入到选中前", self.insert_point_before_selected),
            ("删除选中", self.delete_selected_point),
            ("上移", self.move_selected_point_up),
            ("下移", self.move_selected_point_down),
        ):
            button = QPushButton(text)
            button.clicked.connect(callback)
            row1.addWidget(button)
        layout.addLayout(row1)

        row2 = QHBoxLayout()
        for text, callback in (
            ("重编号点", self.renumber_points),
            ("设为Gate", self.set_selected_point_gate),
            ("清除Gate", self.clear_selected_point_gate),
            ("按路径重编号Gate", self.renumber_gates),
            ("设为终点", self.mark_selected_as_end),
        ):
            button = QPushButton(text)
            button.clicked.connect(callback)
            row2.addWidget(button)
        layout.addLayout(row2)

        row3 = QHBoxLayout()
        scan_button = QPushButton("切换跳过扫码")
        slow_button = QPushButton("切换低速区")
        scan_button.clicked.connect(lambda: self.toggle_selected_point_flag(PATH_FLAG_SKIP_SCAN))
        slow_button.clicked.connect(lambda: self.toggle_selected_point_flag(PATH_FLAG_SLOW_ZONE))
        row3.addWidget(scan_button)
        row3.addWidget(slow_button)
        row3.addStretch(1)
        layout.addLayout(row3)

        hint = QLabel(
            "Gate 点会在画布点号旁显示 G0、G1。Gate 必须按路径顺序连续编号；"
            "V2.5 点 flags 的 bit0 已废除。"
        )
        hint.setWordWrap(True)
        hint.setStyleSheet("color: #555; padding: 4px;")
        layout.addWidget(hint)
        return tab

    def _build_action_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.addWidget(self.action_table, 1)

        row1 = QHBoxLayout()
        for text, callback in (
            ("末尾添加", self.add_default_action),
            ("插入到选中前", self.insert_action_before_selected),
            ("删除选中", self.delete_selected_action),
            ("上移", self.move_selected_action_up),
            ("下移", self.move_selected_action_down),
            ("重编号", self.renumber_actions),
        ):
            button = QPushButton(text)
            button.clicked.connect(callback)
            row1.addWidget(button)
        layout.addLayout(row1)

        row2 = QHBoxLayout()
        for text, flag in (
            ("切换 LOCKED", ACTION_FLAG_LOCKED),
            ("切换 HOLD_PATH", ACTION_FLAG_HOLD_PATH),
            ("切换 REQUIRED_AT_END", ACTION_FLAG_REQUIRED_AT_END),
        ):
            button = QPushButton(text)
            button.clicked.connect(lambda _checked=False, flag=flag: self.toggle_selected_action_flag(flag))
            row2.addWidget(button)
        layout.addLayout(row2)

        hint = QLabel(
            "PICK 和 DROP 必须设置 LOCKED|HOLD_PATH，并引用 ARRIVE_SCAN Gate。"
            "动作严格按 action_seq 进入 FIFO，0xFF 表示轮到队首即可执行。"
        )
        hint.setWordWrap(True)
        hint.setStyleSheet("color: #555; padding: 4px;")
        layout.addWidget(hint)
        return tab

    # -----------------------------
    # 数据刷新
    # -----------------------------
    def refresh_all(
        self,
        selected_point: Optional[int] = None,
        selected_action: Optional[int] = None,
    ):
        self.renumber_points(refresh=False)
        self.renumber_actions(refresh=False)
        self.refresh_point_table(selected_point)
        self.refresh_action_table(selected_action)
        self.field.rebuild(self.points, selected_point)
        self.update_status()

    def refresh_point_table(self, selected: Optional[int] = None):
        self._updating_tables = True
        self.point_table.setRowCount(len(self.points))
        gate_options = {0xFF: "无 Gate (0xFF)"}
        gate_options.update({gate_id: f"Gate {gate_id}" for gate_id in range(MAX_GATES)})
        for row, point in enumerate(self.points):
            self._set_table_item(self.point_table, row, 0, str(point.point_id), editable=False)
            self._set_table_item(self.point_table, row, 1, str(point.x_mm))
            self._set_table_item(self.point_table, row, 2, str(point.y_mm))
            self._set_table_item(self.point_table, row, 3, str(point.yaw_ddeg))
            self._set_combo(
                self.point_table,
                row,
                4,
                POINT_TYPES,
                point.type,
                self.on_point_combo_changed,
                minimum_width=175,
                popup_width=235,
            )
            self._set_combo(
                self.point_table,
                row,
                5,
                gate_options,
                point.gate_id,
                self.on_point_combo_changed,
                minimum_width=110,
                popup_width=145,
            )
            self._set_table_item(self.point_table, row, 6, hex8(point.marker_id))
            self._set_table_item(
                self.point_table,
                row,
                7,
                flags_to_text(point.flags, PATH_FLAGS),
            )
        self._updating_tables = False
        if selected is not None and 0 <= selected < self.point_table.rowCount():
            self.point_table.selectRow(selected)

    def refresh_action_table(self, selected: Optional[int] = None):
        self._updating_tables = True
        self.action_table.setRowCount(len(self.actions))
        gate_options = {0xFF: "立即执行 (0xFF)"}
        for gate_id in sorted({point.gate_id for point in self.points if point.gate_id != 0xFF}):
            gate_options[gate_id] = f"Gate {gate_id}"

        for row, action_item in enumerate(self.actions):
            self._set_table_item(
                self.action_table,
                row,
                0,
                str(action_item.action_seq),
                editable=False,
            )
            self._set_combo(
                self.action_table,
                row,
                1,
                ACTIONS,
                action_item.action,
                self.on_action_combo_changed,
                minimum_width=260,
                popup_width=390,
            )
            self._set_combo(
                self.action_table,
                row,
                2,
                gate_options,
                action_item.unlock_gate_id,
                self.on_action_combo_changed,
                minimum_width=135,
                popup_width=175,
            )
            self._set_table_item(
                self.action_table,
                row,
                3,
                flags_to_text(action_item.flags, ACTION_FLAGS),
            )
            self._set_table_item(self.action_table, row, 4, str(action_item.timeout_ms))
        self._updating_tables = False
        if selected is not None and 0 <= selected < self.action_table.rowCount():
            self.action_table.selectRow(selected)

    def _set_table_item(
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
        options: Dict[int, str],
        current: int,
        callback,
        minimum_width: int,
        popup_width: int,
    ):
        item = QTableWidgetItem(options.get(current, hex8(current)))
        item.setFlags(item.flags() & ~Qt.ItemIsEditable)
        table.setItem(row, column, item)

        combo = QComboBox()
        for code, text in options.items():
            combo.addItem(text, code)
        if combo.findData(current) < 0:
            combo.addItem(f"{hex8(current)} 未知/未引用", current)
        combo.setCurrentIndex(combo.findData(current))
        combo.setMinimumWidth(minimum_width)
        combo.view().setMinimumWidth(popup_width)
        combo.currentIndexChanged.connect(
            lambda _index, row=row, column=column, combo=combo: callback(
                row, column, combo.currentData()
            )
        )
        table.setCellWidget(row, column, combo)

    def selected_point_row(self) -> Optional[int]:
        rows = self.point_table.selectionModel().selectedRows()
        return rows[0].row() if rows else None

    def selected_action_row(self) -> Optional[int]:
        rows = self.action_table.selectionModel().selectedRows()
        return rows[0].row() if rows else None

    def select_point_row(self, index: int):
        if 0 <= index < len(self.points):
            self.point_table.selectRow(index)

    def update_status(self, message: str = ""):
        counts = (
            f"点数 {len(self.points)}/{MAX_POINTS}，"
            f"Gate {gate_count_from_points(self.points)}/{MAX_GATES}，"
            f"动作 {len(self.actions)}/{MAX_ACTIONS}"
        )
        self.status_label.setText(f"{message}\n{counts}" if message else counts)

    # -----------------------------
    # 路径点编辑
    # -----------------------------
    def add_point_from_canvas(self, x_mm: int, y_mm: int):
        if len(self.points) >= MAX_POINTS:
            QMessageBox.warning(self, "点数已满", f"最多只能有 {MAX_POINTS} 个点")
            return
        point = PathPoint(
            x_mm=clamp(x_mm, FIELD_X_MIN_MM, FIELD_X_MAX_MM),
            y_mm=clamp(y_mm, FIELD_Y_MIN_MM, FIELD_Y_MAX_MM),
            point_id=len(self.points),
        )
        self.points.append(point)
        self.refresh_all(selected_point=len(self.points) - 1)
        self.update_status(f"已添加点 {point.point_id}: x={point.x_mm}, y={point.y_mm}")

    def add_default_point(self):
        x_mm, y_mm = 0, 0
        if self.points:
            x_mm = clamp(self.points[-1].x_mm + 200, FIELD_X_MIN_MM, FIELD_X_MAX_MM)
            y_mm = self.points[-1].y_mm
        self.add_point_from_canvas(x_mm, y_mm)

    def insert_point_before_selected(self):
        row = self.selected_point_row()
        if row is None:
            row = len(self.points)
        if len(self.points) >= MAX_POINTS:
            QMessageBox.warning(self, "点数已满", f"最多只能有 {MAX_POINTS} 个点")
            return
        if 0 <= row < len(self.points):
            base = self.points[row]
            point = PathPoint(x_mm=base.x_mm, y_mm=base.y_mm, yaw_ddeg=base.yaw_ddeg)
        else:
            point = PathPoint()
        self.points.insert(row, point)
        self.refresh_all(selected_point=row)

    def delete_selected_point(self):
        row = self.selected_point_row()
        if row is None:
            return
        del self.points[row]
        selected = min(row, len(self.points) - 1) if self.points else None
        self.refresh_all(selected_point=selected)

    def move_selected_point_up(self):
        row = self.selected_point_row()
        if row is None or row <= 0:
            return
        self.points[row - 1], self.points[row] = self.points[row], self.points[row - 1]
        self.refresh_all(selected_point=row - 1)

    def move_selected_point_down(self):
        row = self.selected_point_row()
        if row is None or row >= len(self.points) - 1:
            return
        self.points[row + 1], self.points[row] = self.points[row], self.points[row + 1]
        self.refresh_all(selected_point=row + 1)

    def renumber_points(self, refresh: bool = True):
        for index, point in enumerate(self.points):
            point.point_id = index
        if refresh:
            self.refresh_all(selected_point=self.selected_point_row())

    def set_selected_point_gate(self):
        row = self.selected_point_row()
        if row is None:
            return
        if self.points[row].gate_id == 0xFF:
            used = {point.gate_id for point in self.points if point.gate_id != 0xFF}
            candidate = next((gate_id for gate_id in range(MAX_GATES) if gate_id not in used), None)
            if candidate is None:
                QMessageBox.warning(self, "Gate 已满", f"最多只能有 {MAX_GATES} 个 Gate")
                return
            self.points[row].gate_id = candidate
        self.refresh_all(selected_point=row)

    def clear_selected_point_gate(self):
        row = self.selected_point_row()
        if row is None:
            return
        self.points[row].gate_id = 0xFF
        self.refresh_all(selected_point=row)

    def renumber_gates(self):
        selected = self.selected_point_row()
        old_to_new: Dict[int, int] = {}
        duplicate_ids = set()
        next_gate_id = 0
        for point in self.points:
            if point.gate_id == 0xFF:
                continue
            old_gate_id = point.gate_id
            if old_gate_id in old_to_new:
                duplicate_ids.add(old_gate_id)
            else:
                old_to_new[old_gate_id] = next_gate_id
            point.gate_id = next_gate_id
            next_gate_id += 1

        for action_item in self.actions:
            if action_item.unlock_gate_id in old_to_new:
                action_item.unlock_gate_id = old_to_new[action_item.unlock_gate_id]

        message = f"已按路径顺序重编号 {next_gate_id} 个 Gate"
        if duplicate_ids:
            message += f"；原重复 Gate {sorted(duplicate_ids)} 的动作引用映射到首次出现位置，请复核"
        self.refresh_all(selected_point=selected)
        self.update_status(message)

    def mark_selected_as_end(self):
        row = self.selected_point_row()
        if row is None:
            return
        for point in self.points:
            point.flags &= ~PATH_FLAG_END
        self.points[row].flags |= PATH_FLAG_END
        self.refresh_all(selected_point=row)

    def toggle_selected_point_flag(self, flag: int):
        row = self.selected_point_row()
        if row is None:
            return
        self.points[row].flags ^= flag
        self.refresh_all(selected_point=row)

    def on_point_moved(self, index: int, x_mm: int, y_mm: int):
        if not (0 <= index < len(self.points)):
            return
        x_mm = clamp(x_mm, FIELD_X_MIN_MM, FIELD_X_MAX_MM)
        y_mm = clamp(y_mm, FIELD_Y_MIN_MM, FIELD_Y_MAX_MM)
        self.points[index].x_mm = x_mm
        self.points[index].y_mm = y_mm

        self._updating_tables = True
        x_item = self.point_table.item(index, 1)
        y_item = self.point_table.item(index, 2)
        if x_item is not None:
            x_item.setText(str(x_mm))
        if y_item is not None:
            y_item.setText(str(y_mm))
        self._updating_tables = False
        self.update_status(f"点 {index} 已移动：x={x_mm}, y={y_mm}")

    def on_yaw_changed(self, index: int, yaw_ddeg: int):
        if not (0 <= index < len(self.points)):
            return
        yaw_ddeg = clamp(yaw_ddeg, 0, 3599)
        self.points[index].yaw_ddeg = yaw_ddeg
        self._updating_tables = True
        item = self.point_table.item(index, 3)
        if item is not None:
            item.setText(str(yaw_ddeg))
        self._updating_tables = False
        self.update_status(f"点 {index} 方向已调整：yaw_ddeg={yaw_ddeg}")

    def on_point_selection_changed(self):
        if self._updating_tables:
            return
        row = self.selected_point_row()
        if row is not None:
            self.field.rebuild(self.points, row)

    def on_point_combo_changed(self, row: int, column: int, value: int):
        if self._updating_tables or not (0 <= row < len(self.points)):
            return
        if column == 4:
            self.points[row].type = int(value) & 0xFF
        elif column == 5:
            self.points[row].gate_id = int(value) & 0xFF
        else:
            return
        self.refresh_all(selected_point=row)

    def on_point_item_changed(self, item: QTableWidgetItem):
        if self._updating_tables:
            return
        row, column = item.row(), item.column()
        if not (0 <= row < len(self.points)):
            return
        point = self.points[row]
        value = parse_editor_int(item.text())
        if column == 1:
            point.x_mm = clamp(value, -32768, 32767)
        elif column == 2:
            point.y_mm = clamp(value, -32768, 32767)
        elif column == 3:
            point.yaw_ddeg = clamp(value, 0, 3599)
        elif column == 6:
            point.marker_id = value & 0xFF
        elif column == 7:
            point.flags = value & 0xFF
        else:
            return
        self.refresh_all(selected_point=row)

    # -----------------------------
    # 机械动作编辑
    # -----------------------------
    def add_default_action(self):
        if len(self.actions) >= MAX_ACTIONS:
            QMessageBox.warning(self, "动作已满", f"最多只能有 {MAX_ACTIONS} 个机械动作")
            return
        self.actions.append(MechanicalAction(action_seq=len(self.actions)))
        self.refresh_all(selected_action=len(self.actions) - 1)

    def insert_action_before_selected(self):
        if len(self.actions) >= MAX_ACTIONS:
            QMessageBox.warning(self, "动作已满", f"最多只能有 {MAX_ACTIONS} 个机械动作")
            return
        row = self.selected_action_row()
        if row is None:
            row = len(self.actions)
        self.actions.insert(row, MechanicalAction())
        self.refresh_all(selected_action=row)

    def delete_selected_action(self):
        row = self.selected_action_row()
        if row is None:
            return
        del self.actions[row]
        selected = min(row, len(self.actions) - 1) if self.actions else None
        self.refresh_all(selected_action=selected)

    def move_selected_action_up(self):
        row = self.selected_action_row()
        if row is None or row <= 0:
            return
        self.actions[row - 1], self.actions[row] = self.actions[row], self.actions[row - 1]
        self.refresh_all(selected_action=row - 1)

    def move_selected_action_down(self):
        row = self.selected_action_row()
        if row is None or row >= len(self.actions) - 1:
            return
        self.actions[row + 1], self.actions[row] = self.actions[row], self.actions[row + 1]
        self.refresh_all(selected_action=row + 1)

    def renumber_actions(self, refresh: bool = True):
        for index, action_item in enumerate(self.actions):
            action_item.action_seq = index
        if refresh:
            self.refresh_all(selected_action=self.selected_action_row())

    def toggle_selected_action_flag(self, flag: int):
        row = self.selected_action_row()
        if row is None:
            return
        self.actions[row].flags ^= flag
        self.refresh_all(selected_action=row)

    def on_action_combo_changed(self, row: int, column: int, value: int):
        if self._updating_tables or not (0 <= row < len(self.actions)):
            return
        action_item = self.actions[row]
        if column == 1:
            action_item.action = int(value) & 0xFF
            if action_item.action == 0x20 or action_item.action >= 0x41:
                self.update_status("提示：PICK/DROP 需要手动设置 LOCKED|HOLD_PATH 并选择 ARRIVE_SCAN Gate")
        elif column == 2:
            action_item.unlock_gate_id = int(value) & 0xFF
        else:
            return
        self.refresh_all(selected_action=row)

    def on_action_item_changed(self, item: QTableWidgetItem):
        if self._updating_tables:
            return
        row, column = item.row(), item.column()
        if not (0 <= row < len(self.actions)):
            return
        action_item = self.actions[row]
        value = parse_editor_int(item.text())
        if column == 3:
            action_item.flags = value & 0xFF
        elif column == 4:
            action_item.timeout_ms = clamp(value, 0, 0xFFFF)
        else:
            return
        self.refresh_all(selected_action=row)

    # -----------------------------
    # 文件操作
    # -----------------------------
    def new_project(self):
        if QMessageBox.question(self, "新建", "确定清空当前路径和机械动作？") != QMessageBox.Yes:
            return
        self.points = []
        self.actions = []
        self.current_json_path = None
        self.traj_id_spin.setValue(0)
        self.refresh_all()

    def current_project_dict(self) -> dict:
        field = {
            "width_mm": FIELD_W_MM,
            "height_mm": FIELD_H_MM,
            "origin": "center",
            "x_positive": "field_center_to_pickup",
            "y_positive": "left_when_x_positive_is_forward",
        }
        return project_to_dict(
            self.traj_id_spin.value(),
            self.points,
            self.actions,
            field=field,
        )

    def _load_project_result(self, project, source_description: str):
        self.traj_id_spin.setValue(project.traj_id)
        self.points = project.points
        self.actions = project.actions
        self.refresh_all(
            selected_point=0 if self.points else None,
            selected_action=0 if self.actions else None,
        )
        if project.migrated_from_v2:
            self.current_json_path = None
            errors = validate_project(project.traj_id, project.points, project.actions)
            details = project.migration_summary
            if errors:
                details += "\n\n转换内容仍需手动修正：\n" + "\n".join(errors)
            QMessageBox.information(self, "V2.0 已转换为 V2.5", details)
            self.update_status(f"{source_description} 已转换；保存时必须另存为 V2.5 JSON")
        else:
            self.update_status(f"已打开 {source_description}")

    def open_json(self):
        path, _ = QFileDialog.getOpenFileName(self, "打开 JSON 工程", "", "JSON (*.json)")
        if not path:
            return
        try:
            project = load_project_dict(json.loads(Path(path).read_text(encoding="utf-8")))
            self._load_project_result(project, path)
            self.current_json_path = None if project.migrated_from_v2 else Path(path)
        except Exception as exc:
            QMessageBox.critical(self, "打开失败", str(exc))

    def save_json(self):
        if self.current_json_path is None:
            self.save_json_as()
            return
        try:
            self.current_json_path.write_text(
                json.dumps(self.current_project_dict(), ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
            self.update_status(f"已保存 {self.current_json_path}")
        except Exception as exc:
            QMessageBox.critical(self, "保存失败", str(exc))

    def save_json_as(self):
        default = f"P{self.traj_id_spin.value():04d}.json"
        path, _ = QFileDialog.getSaveFileName(self, "另存 V2.5 JSON 工程", default, "JSON (*.json)")
        if not path:
            return
        self.current_json_path = Path(path)
        self.save_json()

    def export_bin(self):
        traj_id = self.traj_id_spin.value()
        default = f"P{traj_id:04d}.BIN"
        path, _ = QFileDialog.getSaveFileName(self, "导出 HJMB V2.5 BIN", default, "BIN (*.BIN *.bin)")
        if not path:
            return
        try:
            bin_path = Path(path)
            if bin_path_traj_id(bin_path) != traj_id:
                raise ValueError(f"文件名编号必须与 traj_id 一致，应为 P{traj_id:04d}.BIN")
            data = PathCodec.build_bin(traj_id, self.points, self.actions)
            bin_path.write_bytes(data)
            PathCodec.parse_bin(bin_path.read_bytes(), expected_traj_id=traj_id)
            self.update_status(f"已导出并校验通过：{path}")
            QMessageBox.information(self, "导出成功", f"已导出 V2.5 BIN 并校验通过：\n{path}")
        except Exception as exc:
            QMessageBox.critical(self, "导出失败", str(exc))

    def open_bin(self):
        path, _ = QFileDialog.getOpenFileName(self, "打开 HJMB BIN", "", "BIN (*.BIN *.bin)")
        if not path:
            return
        try:
            bin_path = Path(path)
            project = PathCodec.load_bin(
                bin_path.read_bytes(),
                expected_traj_id=bin_path_traj_id(bin_path),
            )
            self.current_json_path = None
            self._load_project_result(project, path)
        except Exception as exc:
            QMessageBox.critical(self, "打开 BIN 失败", str(exc))

    def validate_current_project(self):
        errors = validate_project(
            self.traj_id_spin.value(),
            self.points,
            self.actions,
        )
        if errors:
            QMessageBox.warning(self, "校验失败", "\n".join(errors))
            return
        QMessageBox.information(
            self,
            "校验通过",
            "\n".join(
                (
                    f"traj_id = {self.traj_id_spin.value()}",
                    f"point_count = {len(self.points)}",
                    f"action_count = {len(self.actions)}",
                    f"gate_count = {gate_count_from_points(self.points)}",
                    "V2.5 字段、Gate 和动作 FIFO 校验通过。",
                )
            ),
        )


def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

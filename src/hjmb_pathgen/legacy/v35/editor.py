# -*- coding: utf-8 -*-
"""HJMB V3.5 spatial trajectory editor."""
from __future__ import annotations

import json
import math
import sys
from dataclasses import replace
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

from .path_codec_cli import (
    PathCodec,
    arrival_count_from_nodes,
    bin_path_traj_id,
    load_project_dict,
    save_project_json,
)
from .path_models import (
    ACTION_MODE_ASYNC,
    ACTION_MODE_KINEMATIC,
    ACTION_MODE_NAMES,
    ACTION_MODE_STOP_AND_WAIT,
    ACTIONS,
    MAX_ACTIONS,
    MAX_EDIT_POINTS,
    MAX_TRAJ_ID,
    EditPoint,
    FixedSite,
    MechanicalAction,
    PATH_MODE_FIXED_8,
    PATH_MODE_FREE,
    PATH_MODES,
    PathProject,
    PlanResult,
    PlanSummary,
    POINT_TYPE_ARRIVAL,
    POINT_TYPE_START,
    POINT_TYPE_WAYPOINT,
    POINT_TYPES,
    SITE_ID_FREE,
    TRAJ_FLAG_ARRIVAL,
    TRAJ_FLAG_END,
    TRAJ_FLAG_START,
    TRAJ_FLAG_WAYPOINT,
    YAW_UNSPECIFIED_DDEG,
    YAW_ROTATION_POLICIES,
    fixed_site_key_allows_yaw_override,
    make_default_project,
)
from .trajectory_graphics import (
    ANALYSIS_MODE_ACCEL,
    ANALYSIS_MODE_BETA,
    ANALYSIS_MODE_NORMAL,
    ANALYSIS_MODE_SPEED,
    ANALYSIS_MODE_WZ,
    legend_entries,
    node_hover_text,
    trajectory_segment_color,
)
from .trajectory_planner import plan_project

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
    "site",
    "x_mm",
    "y_mm",
    "yaw_ddeg",
    "max_speed",
    "corner_trim",
    "exact_pass",
)
ACTION_TABLE_COLUMNS = (
    "seq",
    "action",
    "mode",
    "arrival",
    "timeout_ms",
    "post_wait_ms",
    "accel_limit",
    "beta_limit",
    "wz_limit",
    "speed_limit",
    "stable_time",
    "auto_check_start_s",
    "auto_execution_hint",
)
FIXED_SITE_TABLE_COLUMNS = ("site_id", "site_key", "x_mm", "y_mm", "yaw_ddeg")

TABLE_COLUMN_TOOLTIPS = {
    "id": "编辑点序号。0 号点固定为 START，START 也只能出现在 0 号点。",
    "type": "点类型。0 号点只能是 START；其余点只能是 WAYPOINT 或 ARRIVAL。",
    "site": "固定点索引。FREE 模式直接使用 x/y/yaw，site 灰显；FIXED_8 模式下 START 绑定 0，ARRIVAL 绑定 1-7。",
    "x_mm": "场地坐标 X，单位 mm。FIXED_8 的 START/ARRIVAL 会从固定点表读取。",
    "y_mm": "场地坐标 Y，单位 mm。FIXED_8 的 START/ARRIVAL 会从固定点表读取。",
    "yaw_ddeg": "航向角，单位 0.1 度；WAYPOINT 固定为 0xFF，表示不约束航向。",
    "max_speed": "从该编辑点之后开始生效的局部速度上限，单位 mm/s；START 不使用。",
    "corner_trim": "WAYPOINT 的圆角过渡裁切距离，单位 mm；越大转角越圆。START 不使用。",
    "exact_pass": "仅 WAYPOINT 使用。1 表示精确经过该点，0 表示允许按 corner_trim 圆角通过。",
    "seq": "机械动作序号，导出时保持 FIFO 顺序。",
    "action": "机械动作码，例如 PREP_PICK/PICK/DROP/STORE。",
    "mode": "STOP_AND_WAIT=到 ARRIVAL 停车执行并等待 DONE；ASYNC=成为 FIFO 队首后立即请求启动；KINEMATIC=自动 check_start 后满足运动条件再启动。",
    "arrival": "仅 STOP_AND_WAIT 使用，填写 ARRIVAL point_id。",
    "timeout_ms": "机械模块 ACCEPTED 后等待 DONE 的超时时间，单位 ms。",
    "post_wait_ms": "机械模块返回 DONE 后的附加硬等待，单位 ms；0 表示不附加等待。",
    "accel_limit": "KINEMATIC 条件：合成线加速度上限，单位 mm/s^2；0 表示不限制。",
    "beta_limit": "KINEMATIC 条件：角加速度上限，单位 0.1 deg/s^2；0 表示不限制。",
    "wz_limit": "KINEMATIC 条件：角速度上限，单位 0.1 deg/s；0 表示不限制。",
    "speed_limit": "KINEMATIC 条件：速度上限，单位 mm/s；0 表示不限制。",
    "stable_time": "KINEMATIC 启动前实际运动条件需连续满足的时间，单位 ms。",
    "auto_check_start_s": "只读。KINEMATIC 自动计算的最早开始检查位置，写入 BIN，不写入 JSON。",
    "auto_execution_hint": "只读。显示预计路中执行或 ARRIVAL 兜底。",
    "site_id": "固定点编号。0 固定为 START，1-7 可给 ARRIVAL 使用。",
    "site_key": "固定点语义名。当前为 P_START、P_PICK_1、P_PICK_2L、P_PICK_2R、P_PICK_3、P_DROP_1、P_DROP_2、P_DROP_3。",
}


def clamp(value: int, lower: int, upper: int) -> int:
    return max(lower, min(upper, int(value)))


def parse_editor_int(text: str, default: int = 0) -> int:
    try:
        return int(str(text).strip(), 0)
    except ValueError:
        return default


def parse_fixed_site_yaw_text(text: str) -> int:
    value = str(text).strip()
    if value.lower() in {"x", "×", "0xff"}:
        return YAW_UNSPECIFIED_DDEG
    return parse_editor_int(value)


def point_colors(point: EditPoint, selected: bool) -> Tuple[QBrush, QPen]:
    if point.type == POINT_TYPE_START:
        brush = QColor("#2563eb")
    elif point.type == POINT_TYPE_ARRIVAL:
        brush = QColor("#10b981")
    else:
        brush = QColor("#f97316")
    pen = QColor("#f59e0b") if selected else brush.darker(170)
    return QBrush(brush), QPen(pen, 2.8 if selected else 1.6)


def fixed_site_for_point(project: PathProject, point: EditPoint) -> Optional[FixedSite]:
    if project.path_mode != PATH_MODE_FIXED_8 or point.site_id == SITE_ID_FREE:
        return None
    return next(
        (site for site in project.fixed_sites if site.site_id == point.site_id),
        None,
    )


def point_uses_fixed_position(project: PathProject, point: EditPoint) -> bool:
    return (
        point.type in (POINT_TYPE_START, POINT_TYPE_ARRIVAL)
        and fixed_site_for_point(project, point) is not None
    )


def point_uses_fixed_yaw(project: PathProject, point: EditPoint) -> bool:
    site = fixed_site_for_point(project, point)
    return (
        point.type in (POINT_TYPE_START, POINT_TYPE_ARRIVAL)
        and site is not None
        and site.yaw_ddeg != YAW_UNSPECIFIED_DDEG
    )


def display_edit_points(project: PathProject) -> List[EditPoint]:
    points: List[EditPoint] = []
    for point in project.points:
        resolved = replace(point)
        if point.type == POINT_TYPE_WAYPOINT:
            resolved.site_id = SITE_ID_FREE
            resolved.yaw_ddeg = YAW_UNSPECIFIED_DDEG
            points.append(resolved)
            continue
        site = fixed_site_for_point(project, point)
        if site is not None:
            resolved.x_mm = site.x_mm
            resolved.y_mm = site.y_mm
            if site.yaw_ddeg == YAW_UNSPECIFIED_DDEG:
                if resolved.yaw_ddeg == YAW_UNSPECIFIED_DDEG:
                    resolved.yaw_ddeg = 0
            else:
                resolved.yaw_ddeg = site.yaw_ddeg
        points.append(resolved)
    return points


class DraggablePointItem(QGraphicsEllipseItem):
    def __init__(
        self,
        index: int,
        editor: "FieldView",
        point: EditPoint,
        selected: bool,
        movable: bool = True,
    ):
        super().__init__(-7, -7, 14, 14)
        self.index = index
        self.editor = editor
        brush, pen = point_colors(point, selected)
        self.setBrush(brush)
        self.setPen(pen)
        flags = (
            QGraphicsEllipseItem.ItemIsSelectable
            | QGraphicsEllipseItem.ItemSendsGeometryChanges
        )
        if movable:
            flags |= QGraphicsEllipseItem.ItemIsMovable
        self.setFlags(flags)
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
        movable: bool = True,
    ):
        super().__init__(-5.5, -5.5, 11, 11)
        self.index = index
        self.editor = editor
        self.center_x_mm = center_x_mm
        self.center_y_mm = center_y_mm
        self.ready = False
        self.setBrush(QBrush(QColor("#60a5fa")))
        self.setPen(QPen(QColor("#1d4ed8"), 1.5))
        flags = QGraphicsEllipseItem.ItemSendsGeometryChanges
        if movable:
            flags |= QGraphicsEllipseItem.ItemIsMovable
            self.setCursor(Qt.OpenHandCursor)
        else:
            self.setBrush(QBrush(QColor("#cbd5e1")))
            self.setPen(QPen(QColor("#64748b"), 1.2))
        self.setFlags(flags)
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


class DraggableFixedSiteYawHandleItem(QGraphicsEllipseItem):
    def __init__(
        self,
        site_id: int,
        editor: "FieldView",
        center_x_mm: float,
        center_y_mm: float,
    ):
        super().__init__(-5.5, -5.5, 11, 11)
        self.site_id = site_id
        self.editor = editor
        self.center_x_mm = center_x_mm
        self.center_y_mm = center_y_mm
        self.ready = False
        self.setBrush(QBrush(QColor("#fde68a")))
        self.setPen(QPen(QColor("#b45309"), 1.5))
        self.setFlags(
            QGraphicsEllipseItem.ItemIsMovable
            | QGraphicsEllipseItem.ItemSendsGeometryChanges
        )
        self.setCursor(Qt.OpenHandCursor)
        self.setZValue(116)

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
            self.editor.update_fixed_site_yaw_visuals(self.site_id, self.pos())
            x_mm, y_mm = self.editor.scene_to_world_float(self.pos())
            yaw = int(
                round(
                    math.degrees(
                        math.atan2(y_mm - self.center_y_mm, x_mm - self.center_x_mm)
                    )
                    * 10.0
                )
            )
            self.editor.fixed_site_yaw_changed.emit(self.site_id, yaw)
        return super().itemChange(change, value)

    def mouseReleaseEvent(self, event):
        super().mouseReleaseEvent(event)
        self.editor.fixed_site_yaw_drag_finished.emit(self.site_id)


class FieldView(QGraphicsView):
    add_point_requested = Signal(int, int)
    point_moved = Signal(int, int, int)
    yaw_changed = Signal(int, int)
    fixed_site_yaw_changed = Signal(int, int)
    point_drag_finished = Signal(int)
    yaw_drag_finished = Signal(int)
    fixed_site_yaw_drag_finished = Signal(int)
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
        self.fixed_site_yaw_line_items: Dict[int, QGraphicsLineItem] = {}
        self.fixed_site_yaw_handle_items: Dict[int, DraggableFixedSiteYawHandleItem] = {}
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

    def draw_fixed_sites(
        self,
        project: PathProject,
        selected_fixed_site_id: Optional[int],
        fixed_site_yaw_edit_enabled: bool,
    ):
        if project.path_mode != PATH_MODE_FIXED_8 and not fixed_site_yaw_edit_enabled:
            return
        used_site_ids = {
            point.site_id
            for point in project.points
            if point.type in (POINT_TYPE_START, POINT_TYPE_ARRIVAL)
            and point.site_id != SITE_ID_FREE
        }
        for site in project.fixed_sites:
            selected = site.site_id == selected_fixed_site_id
            color = QColor("#f59e0b" if selected else "#16a34a" if site.site_id in used_site_ids else "#64748b")
            pen = QPen(color, 2.4 if selected else 1.6)
            size = 48 if selected else 36
            yaw_text = "×" if site.yaw_ddeg == YAW_UNSPECIFIED_DDEG else str(site.yaw_ddeg)
            tooltip = (
                f"{site.site_id} {site.site_key}\n"
                f"x={site.x_mm:g}, y={site.y_mm:g}, yaw={yaw_text}"
            )
            if site.yaw_ddeg == YAW_UNSPECIFIED_DDEG:
                line_a = self.add_world_line(
                    site.x_mm - size,
                    site.y_mm - size,
                    site.x_mm + size,
                    site.y_mm + size,
                    pen,
                )
                line_b = self.add_world_line(
                    site.x_mm - size,
                    site.y_mm + size,
                    site.x_mm + size,
                    site.y_mm - size,
                    pen,
                )
                for line in (line_a, line_b):
                    line.setZValue(92)
                    line.setToolTip(tooltip)
                marker = self.add_world_text(
                    site.x_mm - 10,
                    site.y_mm + 12,
                    "×",
                    color,
                    11,
                )
                marker.setZValue(94)
                marker.setToolTip(tooltip)
            else:
                center = self.add_world_ellipse(
                    site.x_mm,
                    site.y_mm,
                    18 if selected else 14,
                    pen,
                    QBrush(QColor(255, 251, 235, 220) if selected else QColor(241, 245, 249, 210)),
                )
                center.setZValue(93)
                center.setToolTip(tooltip)
                yaw_rad = math.radians(site.yaw_ddeg / 10.0)
                end_x = site.x_mm + math.cos(yaw_rad) * YAW_ARROW_LENGTH_MM
                end_y = site.y_mm + math.sin(yaw_rad) * YAW_ARROW_LENGTH_MM
                yaw_line = self.add_world_line(
                    site.x_mm,
                    site.y_mm,
                    end_x,
                    end_y,
                    QPen(color, 2.0 if selected else 1.5),
                )
                yaw_line.setZValue(94)
                yaw_line.setToolTip(tooltip)
                self.fixed_site_yaw_line_items[site.site_id] = yaw_line
                editable = selected and fixed_site_yaw_edit_enabled
                if editable:
                    handle = DraggableFixedSiteYawHandleItem(
                        site.site_id,
                        self,
                        site.x_mm,
                        site.y_mm,
                    )
                    handle.setToolTip(tooltip)
                    handle.setPos(self.world_to_scene(end_x, end_y))
                    self.scene_obj.addItem(handle)
                    self.fixed_site_yaw_handle_items[site.site_id] = handle
                    handle.ready = True
                else:
                    endpoint = self.add_world_ellipse(
                        end_x,
                        end_y,
                        9 if selected else 7,
                        QPen(color, 1.4),
                        QBrush(color),
                    )
                    endpoint.setZValue(95)
                    endpoint.setToolTip(tooltip)
            label = self.add_world_text(
                site.x_mm + size + 10,
                site.y_mm + size + 10,
                f"S{site.site_id} {site.site_key}",
                color,
                8,
            )
            label.setZValue(93)
            label.setToolTip(tooltip)

    def rebuild(
        self,
        project: PathProject,
        plan: Optional[PlanResult],
        selected_index: Optional[int],
        selected_fixed_site_id: Optional[int],
        fixed_site_yaw_edit_enabled: bool,
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
        self.fixed_site_yaw_line_items = {}
        self.fixed_site_yaw_handle_items = {}
        self.capture_circle = None
        self.preview_line = None
        self.draw_field()
        self.draw_fixed_sites(
            project,
            selected_fixed_site_id,
            fixed_site_yaw_edit_enabled,
        )
        display_points = display_edit_points(project)
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
        if len(display_points) >= 2:
            control_pen = QPen(QColor("#475569"), 1.4, Qt.DashLine)
            for previous, current in zip(display_points[:-1], display_points[1:]):
                line = self.add_world_line(
                    previous.x_mm,
                    previous.y_mm,
                    current.x_mm,
                    current.y_mm,
                    control_pen,
                )
                line.setZValue(72)
                self.sparse_line_items.append(line)

        for index, point in enumerate(display_points):
            source_point = project.points[index]
            movable = not point_uses_fixed_position(project, source_point)
            item = DraggablePointItem(index, self, point, index == selected_index, movable)
            item.setPos(self.world_to_scene(point.x_mm, point.y_mm))
            self.scene_obj.addItem(item)
            item.setSelected(index == selected_index)
            self.point_items[index] = item
            labels = [str(index), point.type]
            if source_point.site_id != SITE_ID_FREE:
                labels.append(f"S{source_point.site_id}")
            if point.type == POINT_TYPE_ARRIVAL and index == len(display_points) - 1:
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
                yaw_movable = not point_uses_fixed_yaw(project, source_point)
                handle = DraggableYawHandleItem(
                    index, self, point.x_mm, point.y_mm, yaw_movable
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
        display_points = display_edit_points(self.project)
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
            previous = display_points[index - 1]
            self._set_line_world(
                self.sparse_line_items[index - 1],
                previous.x_mm,
                previous.y_mm,
                x_mm,
                y_mm,
            )
        if index < len(self.project.points) - 1 and index < len(self.sparse_line_items):
            following = display_points[index + 1]
            self._set_line_world(
                self.sparse_line_items[index],
                x_mm,
                y_mm,
                following.x_mm,
                following.y_mm,
            )

    def update_yaw_visuals(self, index: int, handle_position: QPointF):
        if self.project is None or not 0 <= index < len(self.project.points):
            return
        point = display_edit_points(self.project)[index]
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

    def update_fixed_site_yaw_visuals(self, site_id: int, handle_position: QPointF):
        if self.project is None:
            return
        site = next(
            (site for site in self.project.fixed_sites if site.site_id == site_id),
            None,
        )
        if site is None:
            return
        yaw_line = self.fixed_site_yaw_line_items.get(site_id)
        if yaw_line is None:
            return
        center = self.world_to_scene(site.x_mm, site.y_mm)
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
        self.setWindowTitle("HJMB 空间轨迹编辑器 V3.5")
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
        self.field.fixed_site_yaw_changed.connect(self.on_fixed_site_yaw_changed)
        self.field.point_drag_finished.connect(lambda _index: self.schedule_plan())
        self.field.yaw_drag_finished.connect(lambda _index: self.schedule_plan())
        self.field.fixed_site_yaw_drag_finished.connect(
            self.on_fixed_site_yaw_drag_finished
        )
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
        self.fixed_site_table = self._create_table(FIXED_SITE_TABLE_COLUMNS)
        self.point_table.itemChanged.connect(self.on_point_item_changed)
        self.point_table.itemSelectionChanged.connect(self.on_point_selection_changed)
        self.action_table.itemChanged.connect(self.on_action_item_changed)
        self.fixed_site_table.itemChanged.connect(self.on_fixed_site_item_changed)
        self.fixed_site_table.itemSelectionChanged.connect(
            self.on_fixed_site_selection_changed
        )

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
        for column, name in enumerate(columns):
            tooltip = TABLE_COLUMN_TOOLTIPS.get(name)
            header_item = table.horizontalHeaderItem(column)
            if tooltip and header_item is not None:
                header_item.setToolTip(tooltip)
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
        self.right_tabs = tabs
        self.point_tab = self._build_point_tab()
        self.action_tab = self._build_action_tab()
        self.fixed_site_tab = self._build_fixed_site_tab()
        self.parameter_tab = self._build_parameter_tab()
        tabs.addTab(self.point_tab, "路径点")
        tabs.addTab(self.action_tab, "机械动作")
        tabs.addTab(self.fixed_site_tab, "固定 8 点 / 批量")
        tabs.addTab(self.parameter_tab, "规划参数")
        tabs.currentChanged.connect(self.on_right_tab_changed)
        layout.addWidget(tabs, 1)
        layout.addWidget(self.legend_label)
        layout.addWidget(self.status_label)
        return panel

    def _build_point_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        mode_row = QHBoxLayout()
        mode_row.addWidget(QLabel("点位模式:"))
        self.path_mode_combo = QComboBox()
        self.path_mode_combo.addItem("自由点位 FREE", PATH_MODE_FREE)
        self.path_mode_combo.addItem("固定 8 点 FIXED_8", PATH_MODE_FIXED_8)
        self.path_mode_combo.setToolTip(
            "FREE: 每个点直接使用 x/y/yaw，site 不生效并灰显；"
            "FIXED_8: START/ARRIVAL 引用固定 8 点表，site 可选。"
        )
        self.path_mode_combo.currentIndexChanged.connect(self.path_mode_changed)
        mode_row.addWidget(self.path_mode_combo)
        mode_row.addStretch(1)
        layout.addLayout(mode_row)
        layout.addWidget(self.point_table, 1)
        for column, width in enumerate(
            (50, 92, 92, 72, 72, 84, 88, 88, 76)
        ):
            self.point_table.setColumnWidth(column, width)
        buttons = QHBoxLayout()
        for text, callback in (
            ("末尾添加", self.add_default_point),
            ("插入", self.insert_point),
            ("删除", self.delete_point),
            ("上移", lambda: self.move_point(-1)),
            ("下移", lambda: self.move_point(1)),
        ):
            button = QPushButton(text)
            button.clicked.connect(callback)
            buttons.addWidget(button)
        layout.addLayout(buttons)
        hint = QLabel(
            "0 号点固定为 START，START 不能出现在其它行；最后一行 ARRIVAL 自动为 END。"
            "site 仅 FIXED_8 使用，FREE 模式会灰显；WAYPOINT yaw 固定为 0xFF。"
        )
        hint.setWordWrap(True)
        layout.addWidget(hint)
        return tab

    def _build_action_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.addWidget(self.action_table, 1)
        for column, width in enumerate((52, 130, 120, 90, 88, 96, 92, 92, 92, 90, 88, 126, 150)):
            self.action_table.setColumnWidth(column, width)
        row = QHBoxLayout()
        for text, callback in (
            ("添加", self.add_action),
            ("删除", self.delete_action),
            ("上移", lambda: self.move_action(-1)),
            ("下移", lambda: self.move_action(1)),
        ):
            button = QPushButton(text)
            button.clicked.connect(callback)
            row.addWidget(button)
        layout.addLayout(row)
        hint = QLabel(
            "mode: STOP_AND_WAIT 到 ARRIVAL 停车等待 DONE；ASYNC 成为 FIFO 队首后立即启动；"
            "KINEMATIC 的 check_start_s 由规划自动计算，JSON 不保存人工触发/窗口。"
        )
        hint.setWordWrap(True)
        layout.addWidget(hint)
        return tab

    def _build_fixed_site_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.addWidget(self.fixed_site_table)
        for column, width in enumerate((72, 110, 96, 96, 96)):
            self.fixed_site_table.setColumnWidth(column, width)
        buttons = QHBoxLayout()
        for text, callback in (
            ("导入固定点 JSON", self.import_fixed_sites),
            ("导出固定点 JSON", self.export_fixed_sites),
        ):
            button = QPushButton(text)
            button.clicked.connect(callback)
            buttons.addWidget(button)
        buttons.addStretch(1)
        layout.addLayout(buttons)
        hint = QLabel(
            "P0000~P0359 的 route case 映射需要集中导入/维护；当前未提供映射表，"
            "不会在代码中猜测编号含义。"
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
        yaw_policy = QComboBox()
        for policy in YAW_ROTATION_POLICIES:
            yaw_policy.addItem(policy, policy)
        yaw_policy.currentIndexChanged.connect(self.parameter_changed)
        self.param_widgets["planner.yaw_rotation_policy"] = yaw_policy
        trajectory_form.addRow("yaw 旋转策略", yaw_policy)
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

        start_check_group = QGroupBox("起点检查")
        start_check_form = QFormLayout(start_check_group)
        start_check_form.addRow(
            "位置容差",
            self._int_spin("start_check.position_tolerance_mm", 0, 1000, " mm"),
        )
        start_check_form.addRow(
            "yaw 容差",
            self._double_spin("start_check.yaw_tolerance_deg", 0.0, 180.0, 1, "°"),
        )
        start_check_form.addRow(
            "稳定时间",
            self._int_spin("start_check.stable_time_ms", 0, 10000, " ms"),
        )
        layout.addWidget(start_check_group)

        arrival_check_group = QGroupBox("ARRIVAL 到达检查")
        arrival_check_form = QFormLayout(arrival_check_group)
        arrival_check_form.addRow(
            "位置容差",
            self._int_spin("arrival_check.position_tolerance_mm", 0, 1000, " mm"),
        )
        arrival_check_form.addRow(
            "yaw 容差",
            self._double_spin("arrival_check.yaw_tolerance_deg", 0.0, 180.0, 1, "°"),
        )
        arrival_check_form.addRow(
            "速度容差",
            self._int_spin("arrival_check.speed_tolerance_mmps", 0, 5000, " mm/s"),
        )
        arrival_check_form.addRow(
            "角速度容差",
            self._int_spin("arrival_check.wz_tolerance_ddegps", 0, 5000, " ddeg/s"),
        )
        arrival_check_form.addRow(
            "稳定时间",
            self._int_spin("arrival_check.stable_time_ms", 0, 10000, " ms"),
        )
        layout.addWidget(arrival_check_group)

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

    def selected_fixed_site_row(self) -> Optional[int]:
        rows = self.fixed_site_table.selectionModel().selectedRows()
        return rows[0].row() if rows else None

    def selected_fixed_site_id(self) -> Optional[int]:
        row = self.selected_fixed_site_row()
        if row is None or not 0 <= row < len(self.project.fixed_sites):
            return None
        return self.project.fixed_sites[row].site_id

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
        return combo

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
        if hasattr(self, "path_mode_combo"):
            self.path_mode_combo.setCurrentIndex(
                max(0, self.path_mode_combo.findData(self.project.path_mode))
            )
        self.refresh_point_table(selected_point)
        self.refresh_action_table(selected_action)
        self.refresh_fixed_site_table()
        self.refresh_parameter_widgets()
        self.updating_ui = False
        if rebuild_field:
            self.refresh_field(selected_point)
        self.update_status()

    def _point_type_values(self, row: int) -> List[Tuple[str, str]]:
        if row == 0:
            return [(POINT_TYPE_START, POINT_TYPE_START)]
        return [
            (POINT_TYPE_WAYPOINT, POINT_TYPE_WAYPOINT),
            (POINT_TYPE_ARRIVAL, POINT_TYPE_ARRIVAL),
        ]

    def _enforce_start_point_rules(self):
        if not self.project.points:
            return
        first = self.project.points[0]
        first.type = POINT_TYPE_START
        first.site_id = 0 if self.project.path_mode == PATH_MODE_FIXED_8 else SITE_ID_FREE
        if first.yaw_ddeg == YAW_UNSPECIFIED_DDEG:
            first.yaw_ddeg = 0
        first.max_speed_mmps = 0
        first.exact_pass = True
        first.corner_trim_mm = 0

        for point in self.project.points[1:]:
            if point.type != POINT_TYPE_START:
                continue
            point.type = POINT_TYPE_WAYPOINT
            point.site_id = SITE_ID_FREE
            point.yaw_ddeg = YAW_UNSPECIFIED_DDEG
            point.exact_pass = False
            if point.corner_trim_mm <= 0:
                point.corner_trim_mm = 200

    def refresh_point_table(self, selected: Optional[int] = None):
        self.point_table.setRowCount(len(self.project.points))
        display_points = display_edit_points(self.project)
        for row, point in enumerate(self.project.points):
            display = display_points[row]
            self._set_item(self.point_table, row, 0, str(point.point_id), False)
            type_combo = self._set_combo(
                self.point_table,
                row,
                1,
                self._point_type_values(row),
                point.type,
                self.on_point_type_changed,
            )
            type_combo.setToolTip(TABLE_COLUMN_TOOLTIPS["type"])
            if self.project.path_mode == PATH_MODE_FIXED_8 and point.type == POINT_TYPE_START:
                site_values = [
                    (f"{self.project.fixed_sites[0].site_id} {self.project.fixed_sites[0].site_key}", 0)
                ]
            elif self.project.path_mode == PATH_MODE_FIXED_8 and point.type == POINT_TYPE_ARRIVAL:
                site_values = [
                    ("0xFF UNSET", SITE_ID_FREE),
                ]
                site_values.extend(
                    (f"{site.site_id} {site.site_key}", site.site_id)
                    for site in self.project.fixed_sites[1:]
                )
            else:
                site_values = [("FREE 0xFF", SITE_ID_FREE)]
                site_values.extend(
                    (f"{site.site_id} {site.site_key}", site.site_id)
                    for site in self.project.fixed_sites
                )
            site_editable = (
                self.project.path_mode == PATH_MODE_FIXED_8
                and point.type == POINT_TYPE_ARRIVAL
            )
            site_combo = self._set_combo(
                self.point_table,
                row,
                2,
                site_values,
                point.site_id,
                self.on_point_site_changed,
            )
            site_combo.setToolTip(TABLE_COLUMN_TOOLTIPS["site"])
            if not site_editable:
                site_combo.setEnabled(False)
            fixed_position = point_uses_fixed_position(self.project, point)
            fixed_yaw = point_uses_fixed_yaw(self.project, point)
            self._set_item(self.point_table, row, 3, f"{display.x_mm:g}", not fixed_position)
            self._set_item(self.point_table, row, 4, f"{display.y_mm:g}", not fixed_position)
            self._set_item(
                self.point_table,
                row,
                5,
                "0xFF"
                if point.type == POINT_TYPE_WAYPOINT
                else str(display.yaw_ddeg),
                point.type != POINT_TYPE_WAYPOINT and not fixed_yaw,
            )
            self._set_item(self.point_table, row, 6, str(point.max_speed_mmps), point.type != POINT_TYPE_START)
            self._set_item(self.point_table, row, 7, f"{point.corner_trim_mm:g}", point.type != POINT_TYPE_START)
            self._set_item(self.point_table, row, 8, str(int(point.exact_pass)), point.type == POINT_TYPE_WAYPOINT)
        if selected is not None and 0 <= selected < len(self.project.points):
            self.point_table.selectRow(selected)

    def _format_point_ref(self, point_id: Optional[int], offset_mm: int = 0) -> str:
        if point_id is None:
            return ""
        if offset_mm == 0:
            return str(point_id)
        sign = "+" if offset_mm >= 0 else ""
        return f"{point_id}{sign}{offset_mm}"

    def _parse_point_ref(self, text: str) -> Tuple[Optional[int], int]:
        text = text.strip()
        if not text:
            return None, 0
        for separator in ("+", "-"):
            if separator in text[1:]:
                index = text[1:].find(separator) + 1
                point_id = parse_editor_int(text[:index])
                offset = parse_editor_int(text[index:])
                return point_id, offset
        return parse_editor_int(text), 0

    def _resolved_action_by_seq(self) -> Dict[int, object]:
        if self.plan_result is None:
            return {}
        return {action.action_seq: action for action in self.plan_result.actions}

    def _action_execution_hint_text(self, resolved) -> str:
        if resolved is None:
            return "未计算"
        if resolved.execution_hint == "MOVING":
            return "预计路中执行"
        if resolved.execution_hint == "ARRIVAL_FALLBACK":
            return f"预计在 ARRIVAL {resolved.fallback_arrival_id} 停车兜底"
        if resolved.execution_hint == "FIFO_HEAD":
            return "队首立即启动"
        if resolved.execution_hint == "ARRIVAL_STOP":
            return f"ARRIVAL {resolved.arrival_id} 停车执行"
        return resolved.execution_hint or "-"

    def refresh_action_table(self, selected: Optional[int] = None):
        self.action_table.setRowCount(len(self.project.actions))
        resolved_by_seq = self._resolved_action_by_seq()
        for row, action in enumerate(self.project.actions):
            self._set_item(self.action_table, row, 0, str(action.action_seq), False)
            self._set_combo(
                self.action_table,
                row,
                1,
                [(name, code) for code, name in ACTIONS.items()],
                action.action,
                self.on_action_code_changed,
            ).setToolTip(TABLE_COLUMN_TOOLTIPS["action"])
            self._set_combo(
                self.action_table,
                row,
                2,
                [(mode, mode) for mode in ACTION_MODE_NAMES],
                action.mode,
                self.on_action_mode_changed,
            ).setToolTip(TABLE_COLUMN_TOOLTIPS["mode"])
            if action.mode == ACTION_MODE_STOP_AND_WAIT:
                primary = "" if action.arrival_point_id is None else str(action.arrival_point_id)
            else:
                primary = ""
            resolved = resolved_by_seq.get(action.action_seq)
            auto_start = (
                "未计算"
                if resolved is None
                else ("-" if resolved.check_start_s_mm == 0xFFFF else str(resolved.check_start_s_mm))
            )
            values = (
                primary,
                action.timeout_ms,
                action.post_wait_ms,
                action.accel_limit_mmps2,
                action.beta_limit_ddegps2,
                action.wz_limit_ddegps,
                action.speed_limit_mmps,
                action.stable_time_ms,
                auto_start,
                self._action_execution_hint_text(resolved),
            )
            for column, value in enumerate(values, start=3):
                editable = column in (4, 5)
                if column == 3:
                    editable = action.mode == ACTION_MODE_STOP_AND_WAIT
                elif column in (6, 7, 8, 9, 10):
                    editable = action.mode == ACTION_MODE_KINEMATIC
                elif column in (11, 12):
                    editable = False
                self._set_item(self.action_table, row, column, str(value), editable)
        if selected is not None and 0 <= selected < len(self.project.actions):
            self.action_table.selectRow(selected)

    def refresh_fixed_site_table(self):
        self.fixed_site_table.setRowCount(len(self.project.fixed_sites))
        for row, site in enumerate(self.project.fixed_sites):
            self._set_item(self.fixed_site_table, row, 0, str(site.site_id), False)
            self._set_item(self.fixed_site_table, row, 1, site.site_key, False)
            self._set_item(self.fixed_site_table, row, 2, f"{site.x_mm:g}")
            self._set_item(self.fixed_site_table, row, 3, f"{site.y_mm:g}")
            self._set_item(
                self.fixed_site_table,
                row,
                4,
                "×" if site.yaw_ddeg == YAW_UNSPECIFIED_DDEG else str(site.yaw_ddeg),
            )

    def refresh_parameter_widgets(self):
        p = self.project
        values = {
            "planner.max_speed_mps": p.planner.max_speed_mmps / 1000.0,
            "planner.linear_accel_mps2": p.planner.linear_accel_mmps2 / 1000.0,
            "planner.lateral_accel_mps2": p.planner.lateral_accel_mmps2 / 1000.0,
            "planner.max_wz_radps": p.planner.max_wz_radps,
            "planner.angular_accel_moving": p.planner.angular_accel_moving_radps2,
            "planner.angular_accel_rotate": p.planner.angular_accel_rotate_radps2,
            "planner.yaw_rotation_policy": p.planner.yaw_rotation_policy,
            "planner.nominal_spacing_mm": p.planner.nominal_spacing_mm,
            "planner.max_spacing_mm": p.planner.max_spacing_mm,
            "planner.max_ref_lead_mm": p.planner.max_ref_lead_mm,
            "overlay.scale_mode": p.overlay.scale_mode,
            "project.collision_check_enabled": p.collision_check_enabled,
            "project.reachability_check_enabled": p.reachability_check_enabled,
            "start_check.position_tolerance_mm": p.start_check.position_tolerance_mm,
            "start_check.yaw_tolerance_deg": p.start_check.yaw_tolerance_ddeg / 10.0,
            "start_check.stable_time_ms": p.start_check.stable_time_ms,
            "arrival_check.position_tolerance_mm": p.arrival_check.position_tolerance_mm,
            "arrival_check.yaw_tolerance_deg": p.arrival_check.yaw_tolerance_ddeg / 10.0,
            "arrival_check.speed_tolerance_mmps": p.arrival_check.speed_tolerance_mmps,
            "arrival_check.wz_tolerance_ddegps": p.arrival_check.wz_tolerance_ddegps,
            "arrival_check.stable_time_ms": p.arrival_check.stable_time_ms,
            "vehicle.wheel_radius_mm": p.vehicle_profile.wheel_radius_mm,
            "vehicle.rotation_radius_mm": p.vehicle_profile.rotation_radius_mm,
            "vehicle.wheel_plan_limit_rpm": p.vehicle_profile.wheel_plan_limit_rpm,
            "vehicle.wheel_hard_limit_rpm": p.vehicle_profile.wheel_hard_limit_rpm,
            "vehicle.mecanum_convention": p.vehicle_profile.mecanum_convention,
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
        p.planner.yaw_rotation_policy = value("planner.yaw_rotation_policy")
        p.planner.nominal_spacing_mm = value("planner.nominal_spacing_mm")
        p.planner.max_spacing_mm = value("planner.max_spacing_mm")
        p.planner.max_ref_lead_mm = value("planner.max_ref_lead_mm")
        p.overlay.scale_mode = value("overlay.scale_mode")
        p.collision_check_enabled = value("project.collision_check_enabled")
        p.reachability_check_enabled = value("project.reachability_check_enabled")
        p.start_check.position_tolerance_mm = value(
            "start_check.position_tolerance_mm"
        )
        p.start_check.yaw_tolerance_ddeg = int(
            round(value("start_check.yaw_tolerance_deg") * 10)
        )
        p.start_check.stable_time_ms = value("start_check.stable_time_ms")
        p.arrival_check.position_tolerance_mm = value(
            "arrival_check.position_tolerance_mm"
        )
        p.arrival_check.yaw_tolerance_ddeg = int(
            round(value("arrival_check.yaw_tolerance_deg") * 10)
        )
        p.arrival_check.speed_tolerance_mmps = value(
            "arrival_check.speed_tolerance_mmps"
        )
        p.arrival_check.wz_tolerance_ddegps = value(
            "arrival_check.wz_tolerance_ddegps"
        )
        p.arrival_check.stable_time_ms = value("arrival_check.stable_time_ms")
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
            self.selected_fixed_site_id(),
            self._fixed_site_tab_active(),
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
                f"ARRIVAL {arrival_count_from_nodes(self.plan_result.nodes)} | "
                f"长度 {summary.total_length_mm:.0f} mm | "
                f"底盘 {summary.formal_time_ms / 1000:.2f} s | "
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
            self.update_status("规划与 V3.5 静态校验通过")
        except Exception as exc:
            self.plan_result = None
            self.plan_error = str(exc)
            self.refresh_field(self.selected_point_row())
            self.update_status()

    def renumber_points(self):
        for index, point in enumerate(self.project.points):
            point.point_id = index
        self._enforce_start_point_rules()

    def renumber_actions(self):
        for index, action in enumerate(self.project.actions):
            action.action_seq = index

    def _traj_id_changed(self, value: int):
        if self.updating_ui:
            return
        self.project.traj_id = value
        self.update_status()

    def _copy_point_to_fixed_site(self, point: EditPoint, site_id: int):
        if not 0 <= site_id < len(self.project.fixed_sites):
            return
        site = self.project.fixed_sites[site_id]
        site.x_mm = point.x_mm
        site.y_mm = point.y_mm
        if point.yaw_ddeg != YAW_UNSPECIFIED_DDEG:
            site.yaw_ddeg = point.yaw_ddeg

    def _apply_fixed_site_to_point(self, point: EditPoint, site_id: int):
        site = next(
            (site for site in self.project.fixed_sites if site.site_id == site_id),
            None,
        )
        if site is None:
            point.site_id = SITE_ID_FREE
            return
        point.site_id = site.site_id
        point.x_mm = site.x_mm
        point.y_mm = site.y_mm
        if site.yaw_ddeg == YAW_UNSPECIFIED_DDEG:
            if point.yaw_ddeg == YAW_UNSPECIFIED_DDEG:
                point.yaw_ddeg = 0
        else:
            point.yaw_ddeg = site.yaw_ddeg

    def _sync_points_from_fixed_site(self, site_id: int):
        site = next(
            (site for site in self.project.fixed_sites if site.site_id == site_id),
            None,
        )
        if site is None:
            return
        for point in self.project.points:
            if point.site_id != site.site_id:
                continue
            if point.type not in (POINT_TYPE_START, POINT_TYPE_ARRIVAL):
                continue
            point.x_mm = site.x_mm
            point.y_mm = site.y_mm
            if site.yaw_ddeg == YAW_UNSPECIFIED_DDEG:
                if point.yaw_ddeg == YAW_UNSPECIFIED_DDEG:
                    point.yaw_ddeg = 0
            else:
                point.yaw_ddeg = site.yaw_ddeg

    def _materialize_current_points_to_fixed_sites(self):
        arrival_site_id = 1
        used_arrival_sites: set[int] = set()
        for point in self.project.points:
            if point.type == POINT_TYPE_START:
                point.site_id = 0
                self._copy_point_to_fixed_site(point, 0)
            elif point.type == POINT_TYPE_ARRIVAL:
                if (
                    point.site_id == SITE_ID_FREE
                    or point.site_id == 0
                    or point.site_id in used_arrival_sites
                ):
                    while (
                        arrival_site_id < len(self.project.fixed_sites)
                        and arrival_site_id in used_arrival_sites
                    ):
                        arrival_site_id += 1
                    point.site_id = min(arrival_site_id, len(self.project.fixed_sites) - 1)
                used_arrival_sites.add(point.site_id)
                self._copy_point_to_fixed_site(point, point.site_id)
            elif point.type == POINT_TYPE_WAYPOINT:
                point.site_id = SITE_ID_FREE

    def path_mode_changed(self, _index: int):
        if self.updating_ui:
            return
        self.project.path_mode = self.path_mode_combo.currentData()
        if self.project.path_mode == PATH_MODE_FIXED_8:
            self._materialize_current_points_to_fixed_sites()
        else:
            for point in self.project.points:
                point.site_id = SITE_ID_FREE
        self.refresh_all(selected_point=self.selected_point_row())
        self.schedule_plan()

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

    def on_fixed_site_selection_changed(self):
        if not self.updating_ui:
            self.refresh_field(self.selected_point_row())

    def on_right_tab_changed(self, _index: int):
        if not self.updating_ui:
            self.refresh_field(self.selected_point_row())

    def _fixed_site_tab_active(self) -> bool:
        return (
            hasattr(self, "right_tabs")
            and hasattr(self, "fixed_site_tab")
            and self.right_tabs.currentWidget() is self.fixed_site_tab
        )

    def _fixed_site_row_by_id(self, site_id: int) -> Optional[int]:
        for row, site in enumerate(self.project.fixed_sites):
            if site.site_id == site_id:
                return row
        return None

    def on_fixed_site_yaw_changed(self, site_id: int, yaw_ddeg: int):
        row = self._fixed_site_row_by_id(site_id)
        if row is None:
            return
        site = self.project.fixed_sites[row]
        site.yaw_ddeg = yaw_ddeg
        self._sync_points_from_fixed_site(site.site_id)
        selected_point = self.selected_point_row()
        self.updating_ui = True
        self.refresh_fixed_site_table()
        self.refresh_point_table(selected_point)
        self.fixed_site_table.selectRow(row)
        self.updating_ui = False
        self.update_status("Adjusting fixed site yaw")

    def on_fixed_site_yaw_drag_finished(self, site_id: int):
        row = self._fixed_site_row_by_id(site_id)
        if row is not None:
            self.updating_ui = True
            self.fixed_site_table.selectRow(row)
            self.updating_ui = False
        self.refresh_field(self.selected_point_row())
        self.schedule_plan()

    def move_selected_fixed_site_to(self, x_mm: int, y_mm: int):
        row = self.selected_fixed_site_row()
        if row is None or not 0 <= row < len(self.project.fixed_sites):
            self.update_status("Select one fixed site row first")
            return
        site = self.project.fixed_sites[row]
        site.x_mm = x_mm
        site.y_mm = y_mm
        self._sync_points_from_fixed_site(site.site_id)
        self.updating_ui = True
        self.refresh_fixed_site_table()
        self.refresh_point_table(self.selected_point_row())
        self.fixed_site_table.selectRow(row)
        self.updating_ui = False
        self.refresh_field(self.selected_point_row())
        self.schedule_plan()

    def add_point_from_canvas(self, x_mm: int, y_mm: int):
        if self._fixed_site_tab_active():
            self.move_selected_fixed_site_to(x_mm, y_mm)
            return
        if len(self.project.points) >= MAX_EDIT_POINTS:
            QMessageBox.warning(self, "点数已满", f"最多 {MAX_EDIT_POINTS} 个编辑点")
            return
        point_type = POINT_TYPE_START if not self.project.points else POINT_TYPE_WAYPOINT
        site_id = 0 if self.project.path_mode == PATH_MODE_FIXED_8 and point_type == POINT_TYPE_START else SITE_ID_FREE
        if site_id != SITE_ID_FREE:
            site = self.project.fixed_sites[site_id]
            x_mm = int(round(site.x_mm))
            y_mm = int(round(site.y_mm))
            yaw_ddeg = site.yaw_ddeg
        else:
            yaw_ddeg = 0 if point_type == POINT_TYPE_START else YAW_UNSPECIFIED_DDEG
        point = EditPoint(
            point_id=len(self.project.points),
            type=point_type,
            site_id=site_id,
            x_mm=x_mm,
            y_mm=y_mm,
            yaw_ddeg=yaw_ddeg,
            exact_pass=point_type == POINT_TYPE_START,
            corner_trim_mm=0 if point_type == POINT_TYPE_START else 200,
        )
        self.project.points.append(point)
        self.refresh_all(selected_point=len(self.project.points) - 1)
        self.schedule_plan()

    def add_default_point(self):
        if self.project.points:
            display_points = display_edit_points(self.project)
            last_point = display_points[-1]
            x_mm = clamp(
                int(last_point.x_mm) + 200,
                FIELD_X_MIN_MM,
                FIELD_X_MAX_MM,
            )
            y_mm = int(last_point.y_mm)
        else:
            x_mm, y_mm = 0, 0
        self.add_point_from_canvas(x_mm, y_mm)

    def insert_point(self):
        row = self.selected_point_row()
        if row is None:
            row = len(self.project.points)
        if len(self.project.points) >= MAX_EDIT_POINTS:
            return
        if not self.project.points:
            self.add_point_from_canvas(0, 0)
            return
        if row <= 0:
            row = 1
        display_points = display_edit_points(self.project)
        base = display_points[row] if row < len(display_points) else None
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
        if row == 0:
            self.update_status("0 号点固定为 START，不能删除")
            return
        del self.project.points[row]
        self.refresh_all(selected_point=min(row, len(self.project.points) - 1))
        self.schedule_plan()

    def move_point(self, offset: int):
        row = self.selected_point_row()
        target = row + offset if row is not None else -1
        if row is None or not 0 <= target < len(self.project.points):
            return
        if row == 0 or target == 0:
            self.update_status("0 号点固定为 START，不能移动")
            return
        self.project.points[row], self.project.points[target] = (
            self.project.points[target],
            self.project.points[row],
        )
        self.refresh_all(selected_point=target)
        self.schedule_plan()

    def on_point_moved(self, index: int, x_mm: int, y_mm: int):
        if not 0 <= index < len(self.project.points):
            return
        point = self.project.points[index]
        if point_uses_fixed_position(self.project, point):
            self.update_status("FIXED_8 的 START/ARRIVAL 请在固定 8 点页编辑")
            return
        self.project.points[index].x_mm = x_mm
        self.project.points[index].y_mm = y_mm
        self.updating_ui = True
        self.point_table.item(index, 3).setText(str(x_mm))
        self.point_table.item(index, 4).setText(str(y_mm))
        self.updating_ui = False
        self.update_status("正在移动编辑点，松开后重新规划")

    def on_yaw_changed(self, index: int, yaw_ddeg: int):
        if not 0 <= index < len(self.project.points):
            return
        if self.project.points[index].type == POINT_TYPE_WAYPOINT:
            return
        if point_uses_fixed_yaw(self.project, self.project.points[index]):
            self.update_status("FIXED_8 的 START/ARRIVAL yaw 请在固定 8 点页编辑")
            return
        self.project.points[index].yaw_ddeg = yaw_ddeg
        self.updating_ui = True
        self.point_table.item(index, 5).setText(str(yaw_ddeg))
        self.updating_ui = False
        self.update_status("正在调整 yaw，松开后重新规划")

    def on_point_type_changed(self, row: int, value):
        if self.updating_ui or not 0 <= row < len(self.project.points):
            return
        point = self.project.points[row]
        new_type = str(value)
        if row == 0 and new_type != POINT_TYPE_START:
            self.refresh_point_table(row)
            self.update_status("0 号点只能是 START")
            return
        if row != 0 and new_type == POINT_TYPE_START:
            self.refresh_point_table(row)
            self.update_status("START 只能放在 0 号点")
            return
        point.type = new_type
        if point.type == POINT_TYPE_START:
            point.site_id = 0 if self.project.path_mode == PATH_MODE_FIXED_8 else SITE_ID_FREE
            point.yaw_ddeg = 0 if point.yaw_ddeg == YAW_UNSPECIFIED_DDEG else point.yaw_ddeg
            point.max_speed_mmps = 0
            point.exact_pass = True
            point.corner_trim_mm = 0
        if point.type == POINT_TYPE_WAYPOINT:
            point.site_id = SITE_ID_FREE
            point.yaw_ddeg = YAW_UNSPECIFIED_DDEG
        elif point.yaw_ddeg == YAW_UNSPECIFIED_DDEG:
            point.yaw_ddeg = 0
        if point.type == POINT_TYPE_ARRIVAL and self.project.path_mode == PATH_MODE_FIXED_8:
            if point.site_id == 0:
                point.site_id = SITE_ID_FREE
            if point.yaw_ddeg == YAW_UNSPECIFIED_DDEG:
                point.yaw_ddeg = 0
        if point.type == POINT_TYPE_START and self.project.path_mode == PATH_MODE_FIXED_8:
            self._apply_fixed_site_to_point(point, 0)
        self.updating_ui = True
        self.refresh_point_table(row)
        self.updating_ui = False
        self.schedule_plan()
        self.refresh_field(row)

    def on_point_site_changed(self, row: int, value):
        if self.updating_ui or not 0 <= row < len(self.project.points):
            return
        point = self.project.points[row]
        if point.type == POINT_TYPE_WAYPOINT:
            point.site_id = SITE_ID_FREE
        elif point.type == POINT_TYPE_START:
            point.site_id = 0
            if self.project.path_mode == PATH_MODE_FIXED_8:
                self._apply_fixed_site_to_point(point, 0)
        elif self.project.path_mode == PATH_MODE_FIXED_8 and point.type == POINT_TYPE_ARRIVAL:
            site_id = int(value)
            if site_id == SITE_ID_FREE:
                point.site_id = SITE_ID_FREE
                if point.yaw_ddeg == YAW_UNSPECIFIED_DDEG:
                    point.yaw_ddeg = 0
            elif site_id == 0:
                point.site_id = SITE_ID_FREE
            else:
                self._apply_fixed_site_to_point(point, site_id)
        else:
            point.site_id = SITE_ID_FREE
        self.updating_ui = True
        self.refresh_point_table(row)
        self.updating_ui = False
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
        fixed_position = point_uses_fixed_position(self.project, point)
        fixed_yaw = point_uses_fixed_yaw(self.project, point)
        try:
            if column == 3 and not fixed_position:
                point.x_mm = float(text)
            elif column == 4 and not fixed_position:
                point.y_mm = float(text)
            elif column == 5 and not fixed_yaw:
                if point.type == POINT_TYPE_WAYPOINT:
                    point.yaw_ddeg = YAW_UNSPECIFIED_DDEG
                else:
                    point.yaw_ddeg = parse_editor_int(text)
            elif column == 6:
                point.max_speed_mmps = max(0, parse_editor_int(text))
            elif column == 7:
                point.corner_trim_mm = max(0.0, float(text))
            elif column == 8:
                point.exact_pass = bool(parse_editor_int(text))
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
        arrival_point_id = next(
            (point.point_id for point in self.project.points if point.type == POINT_TYPE_ARRIVAL),
            None,
        )
        self.project.actions.append(
            MechanicalAction(
                action_seq=len(self.project.actions),
                arrival_point_id=arrival_point_id,
            )
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

    def on_action_code_changed(self, row: int, value):
        if self.updating_ui or not 0 <= row < len(self.project.actions):
            return
        self.project.actions[row].action = int(value)
        self.schedule_plan()

    def on_action_mode_changed(self, row: int, value):
        if self.updating_ui or not 0 <= row < len(self.project.actions):
            return
        action = self.project.actions[row]
        action.mode = str(value)
        if action.mode == ACTION_MODE_STOP_AND_WAIT:
            action.accel_limit_mmps2 = 0
            action.beta_limit_ddegps2 = 0
            action.wz_limit_ddegps = 0
            action.speed_limit_mmps = 0
            action.stable_time_ms = 0
        elif action.mode == ACTION_MODE_ASYNC:
            action.arrival_point_id = None
            action.accel_limit_mmps2 = 0
            action.beta_limit_ddegps2 = 0
            action.wz_limit_ddegps = 0
            action.speed_limit_mmps = 0
            action.stable_time_ms = 0
        elif action.mode == ACTION_MODE_KINEMATIC:
            action.arrival_point_id = None
        self.refresh_all(selected_action=row)
        self.schedule_plan()

    def on_action_item_changed(self, item: QTableWidgetItem):
        if self.updating_ui:
            return
        row, column = item.row(), item.column()
        if not 0 <= row < len(self.project.actions) or column < 3:
            return
        action = self.project.actions[row]
        text = item.text()
        if column == 3:
            if action.mode == ACTION_MODE_STOP_AND_WAIT:
                point_id, _offset = self._parse_point_ref(text)
                action.arrival_point_id = point_id
            self.schedule_plan()
            return
        if column == 4:
            action.timeout_ms = clamp(parse_editor_int(text), 0, 0xFFFF)
        elif column == 5:
            action.post_wait_ms = clamp(parse_editor_int(text), 0, 0xFFFF)
        elif column == 6:
            action.accel_limit_mmps2 = clamp(parse_editor_int(text), 0, 0xFFFF)
        elif column == 7:
            action.beta_limit_ddegps2 = clamp(parse_editor_int(text), 0, 0xFFFF)
        elif column == 8:
            action.wz_limit_ddegps = clamp(parse_editor_int(text), 0, 0xFFFF)
        elif column == 9:
            action.speed_limit_mmps = clamp(parse_editor_int(text), 0, 0xFFFF)
        elif column == 10:
            action.stable_time_ms = clamp(parse_editor_int(text), 0, 0xFFFF)
        else:
            return
        self.schedule_plan()

    def on_fixed_site_item_changed(self, item: QTableWidgetItem):
        if self.updating_ui:
            return
        row, column = item.row(), item.column()
        if not 0 <= row < len(self.project.fixed_sites) or column < 2:
            return
        site = self.project.fixed_sites[row]
        text = item.text()
        try:
            if column == 2:
                site.x_mm = float(text)
            elif column == 3:
                site.y_mm = float(text)
            elif column == 4:
                yaw_ddeg = parse_fixed_site_yaw_text(text)
                if (
                    yaw_ddeg == YAW_UNSPECIFIED_DDEG
                    and not fixed_site_key_allows_yaw_override(site.site_key)
                ):
                    self.update_status("Only DROP fixed sites may use yaw=× / 0xFF")
                    self.updating_ui = True
                    self.refresh_fixed_site_table()
                    self.fixed_site_table.selectRow(row)
                    self.updating_ui = False
                    return
                site.yaw_ddeg = yaw_ddeg
        except ValueError:
            self.updating_ui = True
            self.refresh_fixed_site_table()
            self.fixed_site_table.selectRow(row)
            self.updating_ui = False
            return
        self._sync_points_from_fixed_site(site.site_id)
        selected_point = self.selected_point_row()
        self.updating_ui = True
        self.refresh_fixed_site_table()
        self.refresh_point_table(selected_point)
        self.fixed_site_table.selectRow(row)
        self.updating_ui = False
        self.refresh_field(self.selected_point_row())
        self.schedule_plan()

    def import_fixed_sites(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "导入固定 8 点 JSON", "", "JSON (*.json)"
        )
        if not path:
            return
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
            sites_data = data.get("fixed_sites", data if isinstance(data, list) else [])
            self.project.fixed_sites = [FixedSite.from_dict(item) for item in sites_data]
            self.refresh_all(selected_point=self.selected_point_row())
            self.schedule_plan()
        except Exception as exc:
            QMessageBox.critical(self, "导入固定点失败", str(exc))

    def export_fixed_sites(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "导出固定 8 点 JSON", "fixed_sites_v35.json", "JSON (*.json)"
        )
        if not path:
            return
        data = {"fixed_sites": [site.__dict__ for site in self.project.fixed_sites]}
        try:
            Path(path).write_text(
                json.dumps(data, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
            self.update_status(f"已导出固定点 {path}")
        except Exception as exc:
            QMessageBox.critical(self, "导出固定点失败", str(exc))

    def new_project(self):
        if (
            QMessageBox.question(self, "新建", "确定新建 V3.5 工程？")
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
            self, "导入 V3.5 配置 JSON", "", "JSON (*.json)"
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
            self, "导出 V3.5 配置 JSON", default, "JSON (*.json)"
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
            self, "导出 HJMB V3.5 BIN", default, "BIN (*.BIN *.bin)"
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
                self, "导出成功", f"V3.5 BIN 已导出并回读校验：\n{path}"
            )
            self.update_status(f"已导出并校验 {path}")
        except Exception as exc:
            QMessageBox.critical(self, "导出失败", str(exc))

    def open_bin(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "打开 HJMB V3.5 BIN", "", "BIN (*.BIN *.bin)"
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
            project.start_check.position_tolerance_mm = (
                parsed.header.start_pos_tolerance_mm
            )
            project.start_check.yaw_tolerance_ddeg = (
                parsed.header.start_yaw_tolerance_ddeg
            )
            project.start_check.stable_time_ms = parsed.header.start_stable_time_ms
            project.arrival_check.position_tolerance_mm = (
                parsed.header.arrival_pos_tolerance_mm
            )
            project.arrival_check.yaw_tolerance_ddeg = (
                parsed.header.arrival_yaw_tolerance_ddeg
            )
            project.arrival_check.speed_tolerance_mmps = (
                parsed.header.arrival_speed_tolerance_mmps
            )
            project.arrival_check.wz_tolerance_ddegps = (
                parsed.header.arrival_wz_tolerance_ddegps
            )
            project.arrival_check.stable_time_ms = (
                parsed.header.arrival_stable_time_ms
            )
            arrival_point_id_by_arrival_id: Dict[int, int] = {}
            for node in parsed.nodes:
                point_type = None
                if node.flags & TRAJ_FLAG_START:
                    point_type = POINT_TYPE_START
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
                        site_id=SITE_ID_FREE,
                        x_mm=node.x_mm,
                        y_mm=node.y_mm,
                        yaw_ddeg=(
                            YAW_UNSPECIFIED_DDEG
                            if point_type == POINT_TYPE_WAYPOINT
                            else int(round(math.degrees(node.yaw_rad) * 10))
                        ),
                        exact_pass=point_type != POINT_TYPE_ARRIVAL,
                    )
                )
                if point_type == POINT_TYPE_ARRIVAL:
                    arrival_point_id_by_arrival_id[node.arrival_id] = project.points[-1].point_id
            recovered_actions: List[MechanicalAction] = []
            skipped_actions = 0
            for action in parsed.actions:
                if action.mode == ACTION_MODE_STOP_AND_WAIT:
                    recovered_actions.append(
                        MechanicalAction(
                            action_seq=len(recovered_actions),
                            action=action.action,
                            mode=action.mode,
                            timeout_ms=action.timeout_ms,
                            post_wait_ms=action.post_wait_ms,
                            arrival_point_id=arrival_point_id_by_arrival_id.get(
                                action.arrival_id
                            ),
                        )
                    )
                else:
                    skipped_actions += 1
            project.actions = recovered_actions
            summary = PlanSummary(
                total_length_mm=parsed.header.total_length_mm,
                formal_time_ms=parsed.header.planned_motion_time_ms,
                max_speed_mmps=max(node.speed_mmps for node in parsed.nodes),
            )
            self.project = project
            self.plan_result = PlanResult(
                nodes=parsed.nodes,
                actions=parsed.actions,
                summary=summary,
                warnings=[
                    "BIN 仅含稠密轨迹；普通塑形 WAYPOINT 无法无损恢复",
                    f"{skipped_actions} 个 ASYNC/KINEMATIC 动作因缺少原始 point-relative 引用未恢复",
                ],
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
                        f"arrival_count={parsed.header.arrival_count}",
                        f"planned_motion_time_ms={parsed.header.planned_motion_time_ms}",
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

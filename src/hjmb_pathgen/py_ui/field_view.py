"""V4 field graphics view using the proven V3.5 field canvas style."""

from __future__ import annotations

import math
from collections import Counter
from typing import Any

from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QBrush, QColor, QFont, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import (
    QGraphicsEllipseItem,
    QGraphicsItemGroup,
    QGraphicsLineItem,
    QGraphicsPathItem,
    QGraphicsPolygonItem,
    QGraphicsRectItem,
    QGraphicsScene,
    QGraphicsSimpleTextItem,
    QGraphicsView,
)

from hjmb_pathgen.py_domain.leg import LegV40
from hjmb_pathgen.py_domain.project import ProjectV40

from .graphics_items import (
    YAW_HANDLE_RADIUS_MM,
    DragCommit,
    EditablePointItem,
    YawCommit,
    YawHandleItem,
    yaw_line,
)
from .ui_state import ManualPointDraft, SITE_KEYS, site_has_yaw, site_label

DEFAULT_FIELD_LENGTH_MM = 4000
DEFAULT_FIELD_WIDTH_MM = 2000
FIELD_SCALE = 0.25
SCENE_MARGIN_PX = 24
FENCE_W_MM = 35
GRID_STEP_MM = 250

SITE_LABELS = {
    "P_START": "START",
    "P_PICK_1": "P1",
    "P_PICK_2L": "P2L",
    "P_PICK_2R": "P2R",
    "P_PICK_3": "P3",
    "F_DROP_4": "D4",
    "F_DROP_5": "D5",
    "F_DROP_6": "D6",
    "F_DROP_7": "D7",
    "F_DROP_8": "D8",
}

SITE_LABEL_OFFSETS = {
    "P_START": (-180, 95),
    "P_PICK_1": (85, 90),
    "P_PICK_2L": (85, 35),
    "P_PICK_2R": (85, -35),
    "P_PICK_3": (85, -90),
    "F_DROP_4": (85, 90),
    "F_DROP_5": (85, 45),
    "F_DROP_6": (85, 0),
    "F_DROP_7": (85, -45),
    "F_DROP_8": (85, -90),
}


class V4FieldView(QGraphicsView):
    worldMouseMoved = Signal(float, float)
    zoomChanged = Signal(float)
    backgroundDoubleClicked = Signal(float, float)
    backgroundClicked = Signal(float, float)
    siteSelected = Signal(str)
    sitePositionPreview = Signal(str, int, int)
    sitePositionCommitted = Signal(object)
    siteYawPreview = Signal(str, int)
    siteYawCommitted = Signal(object)
    manualPointSelected = Signal(int)
    manualPointPositionPreview = Signal(int, int, int)
    manualPointPositionCommitted = Signal(object)
    manualPointYawPreview = Signal(int, int)
    manualPointYawCommitted = Signal(object)

    def __init__(self, *, mode: str = "sites", parent=None) -> None:
        super().__init__(parent)
        self.mode = mode
        self.scene_obj = QGraphicsScene(self)
        self.setScene(self.scene_obj)
        self.setObjectName(f"V4FieldView_{mode}")
        self.setRenderHints(QPainter.Antialiasing | QPainter.TextAntialiasing)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)
        self.setMouseTracking(True)
        self.setDragMode(QGraphicsView.RubberBandDrag)
        self.rebuilding = False
        self.project: ProjectV40 | None = None
        self.manual_points: list[ManualPointDraft] = []
        self.leg: LegV40 | None = None
        self.preview_xy: tuple[tuple[float, float], ...] = ()
        self.topology_gates_override: tuple[dict[str, Any], ...] | None = None
        self.editable = True
        self.selected_site: str | None = None
        self.selected_manual_index: int | None = None
        self.layers: dict[str, bool] = {
            "GRID": True,
            "FIELD_OBJECTS": True,
            "SITES": True,
            "TOPOLOGY_GATES": True,
            "SPARSE_PATH": True,
            "DENSE_PATH": True,
            "CONTROL_POINTS": True,
            "ROBOT_POSE": True,
            "COLLISION_FOOTPRINT": True,
            "COLLISION_MARKERS": True,
            "SPEED_OVERLAY": True,
            "LABELS": True,
        }
        self.site_items: dict[str, EditablePointItem] = {}
        self.site_yaw_items: dict[str, YawHandleItem] = {}
        self.manual_items: dict[int, EditablePointItem] = {}
        self.manual_yaw_items: dict[int, YawHandleItem] = {}
        self._zoom_factor = 1.0
        self._auto_fit = True
        self._space_panning = False
        self._panning = False
        self._last_pan_pos = QPointF()
        self.refresh()

    def set_project(self, project: ProjectV40 | None) -> None:
        self.project = project
        self.refresh()

    def set_manual_points(self, points: list[ManualPointDraft]) -> None:
        self.manual_points = points
        self.refresh()

    def set_leg(self, leg: LegV40 | None) -> None:
        self.leg = leg
        self.refresh()

    def set_preview_xy(self, points: tuple[tuple[float, float], ...]) -> None:
        self.preview_xy = tuple((float(x), float(y)) for x, y in points)
        self.refresh()

    def set_topology_gates_override(self, gates: tuple[dict[str, Any], ...] | None) -> None:
        self.topology_gates_override = None if gates is None else tuple(dict(item) for item in gates)
        self.refresh()

    def set_editable(self, editable: bool) -> None:
        self.editable = bool(editable)
        self.refresh()

    def set_selected_site(self, site_key: str | None, *, center: bool = False) -> None:
        self.selected_site = site_key
        for key, item in self.site_items.items():
            item.set_highlighted(key == site_key)
        if center and site_key in self.site_items:
            self.centerOn(self.site_items[site_key])

    def set_selected_manual_index(
        self, index: int | None, *, center: bool = False
    ) -> None:
        self.selected_manual_index = index
        for item_index, item in self.manual_items.items():
            item.set_highlighted(item_index == index)
        if center and index is not None and index in self.manual_items:
            self.centerOn(self.manual_items[index])

    def set_layer_visible(self, layer: str, visible: bool) -> None:
        self.layers[layer] = visible
        self.refresh()

    def refresh(self) -> None:
        self.rebuilding = True
        self.scene_obj.clear()
        self.site_items.clear()
        self.site_yaw_items.clear()
        self.manual_items.clear()
        self.manual_yaw_items.clear()
        self.scene_obj.setSceneRect(self._scene_rect())
        self._draw_base_field()
        if self.project is not None and self.layers.get("FIELD_OBJECTS", True):
            self._draw_field_objects(self.project)
        if self.project is not None and self.layers.get("TOPOLOGY_GATES", True):
            self._draw_topology_gates(self.project)
        if (
            self.mode in {"sites", "route", "template"}
            and self.project is not None
            and self.layers.get("SITES", True)
        ):
            self._draw_sites(self.project)
        if self.mode in {"manual", "template"}:
            self._draw_manual_points()
        if self.preview_xy:
            self._draw_preview_curve()
        if self.leg is not None:
            self._draw_leg(self.leg)
        self.rebuilding = False
        self.set_selected_site(self.selected_site)
        self.set_selected_manual_index(self.selected_manual_index)
        if self._auto_fit:
            self.fit_to_field()

    def fit_to_field(self) -> None:
        self._auto_fit = True
        self.resetTransform()
        self.fitInView(self.scene_obj.sceneRect(), Qt.KeepAspectRatio)
        self._zoom_factor = self.transform().m11()
        self.zoomChanged.emit(self._zoom_factor)

    def reset_zoom(self) -> None:
        self._auto_fit = False
        self.resetTransform()
        self._zoom_factor = self.transform().m11()
        self.zoomChanged.emit(self._zoom_factor)

    def scene_dump(self) -> dict[str, Any]:
        counts = Counter(
            str(item.data(0))
            for item in self.scene_obj.items()
            if item.data(0) is not None
        )
        return {
            "mode": self.mode,
            "scene_item_count": len(self.scene_obj.items()),
            "field_boundary_count": counts.get("field_boundary", 0),
            "grid_line_count": counts.get("grid_line", 0),
            "site_count": counts.get("site", 0),
            "site_yaw_handle_count": counts.get("site_yaw_handle", 0),
            "manual_point_count": counts.get("manual_point", 0),
            "manual_yaw_handle_count": counts.get("manual_yaw_handle", 0),
            "sparse_path_count": counts.get("sparse_path", 0),
            "cylinder_count": counts.get("cylinder", 0),
            "pickup_box_count": counts.get("pickup_box", 0),
            "drop_box_count": counts.get("drop_box", 0),
            "dense_path_count": counts.get("dense_path", 0),
            "preview_curve_count": counts.get("preview_curve", 0),
            "control_point_count": counts.get("control_point", 0),
            "topology_gate_count": counts.get("topology_gate", 0),
            "collision_footprint_count": counts.get("collision_footprint", 0),
            "speed_overlay_count": counts.get("speed_overlay", 0),
        }

    def world_to_scene(self, x_mm: float, y_mm: float) -> QPointF:
        half_length, half_width = self._field_half_size()
        return QPointF(
            (float(x_mm) + half_length) * FIELD_SCALE,
            (half_width - float(y_mm)) * FIELD_SCALE,
        )

    def scene_to_world_float(self, position: QPointF) -> tuple[float, float]:
        half_length, half_width = self._field_half_size()
        return (
            position.x() / FIELD_SCALE - half_length,
            half_width - position.y() / FIELD_SCALE,
        )

    def scene_to_world_int(self, position: QPointF) -> tuple[int, int]:
        x_mm, y_mm = self.scene_to_world_float(position)
        return int(round(x_mm)), int(round(y_mm))

    def clamp_scene_point(self, point: QPointF) -> QPointF:
        x_mm, y_mm = self.scene_to_world_float(point)
        x_min, x_max, y_min, y_max = self._field_limits()
        return self.world_to_scene(
            max(x_min, min(x_max, x_mm)),
            max(y_min, min(y_max, y_mm)),
        )

    def yaw_handle_scene_point(
        self, center_x_mm: int, center_y_mm: int, yaw_ddeg: int
    ) -> QPointF:
        yaw_rad = math.radians(yaw_ddeg / 10.0)
        return self.world_to_scene(
            center_x_mm + math.cos(yaw_rad) * YAW_HANDLE_RADIUS_MM,
            center_y_mm + math.sin(yaw_rad) * YAW_HANDLE_RADIUS_MM,
        )

    def yaw_for_key(self, key: str | int) -> int:
        if isinstance(key, str) and self.project is not None:
            return int(self.project.sites[key].get("yaw_ddeg", 0))
        if isinstance(key, int) and 0 <= key < len(self.manual_points):
            return int(self.manual_points[key].yaw_ddeg or 0)
        return 0

    def point_position_preview(self, key: str | int, x_mm: int, y_mm: int) -> None:
        if isinstance(key, str):
            self.sitePositionPreview.emit(key, x_mm, y_mm)
        else:
            self.manualPointPositionPreview.emit(int(key), x_mm, y_mm)

    def point_position_committed(self, commit: DragCommit) -> None:
        if isinstance(commit.key, str):
            self.sitePositionCommitted.emit(commit)
        else:
            self.manualPointPositionCommitted.emit(commit)

    def yaw_preview(self, key: str | int, yaw_ddeg: int) -> None:
        if isinstance(key, str):
            self.siteYawPreview.emit(key, yaw_ddeg)
        else:
            self.manualPointYawPreview.emit(int(key), yaw_ddeg)

    def yaw_committed(self, commit: YawCommit) -> None:
        if isinstance(commit.key, str):
            self.siteYawCommitted.emit(commit)
        else:
            self.manualPointYawCommitted.emit(commit)

    def point_item_selected(self, key: str | int) -> None:
        if isinstance(key, str):
            self.siteSelected.emit(key)
        else:
            self.manualPointSelected.emit(int(key))

    def mouseMoveEvent(self, event):  # type: ignore[override]
        if self._panning:
            delta = event.position() - self._last_pan_pos
            self._last_pan_pos = event.position()
            self.horizontalScrollBar().setValue(
                self.horizontalScrollBar().value() - int(delta.x())
            )
            self.verticalScrollBar().setValue(
                self.verticalScrollBar().value() - int(delta.y())
            )
        world = self.scene_to_world_float(self.mapToScene(event.position().toPoint()))
        self.worldMouseMoved.emit(world[0], world[1])
        super().mouseMoveEvent(event)

    def mousePressEvent(self, event):  # type: ignore[override]
        if event.button() == Qt.MiddleButton or (
            self._space_panning and event.button() == Qt.LeftButton
        ):
            self._panning = True
            self._last_pan_pos = event.position()
            self.setCursor(Qt.ClosedHandCursor)
            event.accept()
            return
        if event.button() == Qt.LeftButton and self.mode == "template" and self.editable:
            item = self.itemAt(event.position().toPoint())
            kind = None if item is None else str(item.data(0) or "")
            if kind in {"", "outside_field", "field_boundary", "grid_line", "start_zone"}:
                x_mm, y_mm = self.scene_to_world_float(self.mapToScene(event.position().toPoint()))
                if self._contains_world_point(x_mm, y_mm):
                    self.backgroundClicked.emit(x_mm, y_mm)
                    event.accept()
                    return
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):  # type: ignore[override]
        if self._panning and event.button() in {Qt.MiddleButton, Qt.LeftButton}:
            self._panning = False
            self.setCursor(Qt.ArrowCursor)
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def mouseDoubleClickEvent(self, event):  # type: ignore[override]
        if event.button() == Qt.LeftButton:
            x_mm, y_mm = self.scene_to_world_float(
                self.mapToScene(event.position().toPoint())
            )
            if self._contains_world_point(x_mm, y_mm):
                self.backgroundDoubleClicked.emit(x_mm, y_mm)
                event.accept()
                return
        super().mouseDoubleClickEvent(event)

    def wheelEvent(self, event):  # type: ignore[override]
        delta = event.angleDelta().y()
        if delta == 0:
            super().wheelEvent(event)
            return
        factor = 1.15 if delta > 0 else 1.0 / 1.15
        next_zoom = self.transform().m11() * factor
        if 0.2 <= next_zoom <= 8.0:
            self._auto_fit = False
            self.scale(factor, factor)
            self._zoom_factor = self.transform().m11()
            self.zoomChanged.emit(self._zoom_factor)
        event.accept()

    def resizeEvent(self, event):  # type: ignore[override]
        super().resizeEvent(event)
        if self._auto_fit:
            self.fit_to_field()

    def keyPressEvent(self, event):  # type: ignore[override]
        if event.key() == Qt.Key_Space:
            self._space_panning = True
            self.setCursor(Qt.OpenHandCursor)
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):  # type: ignore[override]
        if event.key() == Qt.Key_Space:
            self._space_panning = False
            self.setCursor(Qt.ArrowCursor)
        super().keyReleaseEvent(event)

    def _draw_base_field(self) -> None:
        if not self.layers.get("GRID", True):
            return
        length, width = self._field_size()
        x_min, x_max, y_min, y_max = self._field_limits()
        self.scene_obj.setBackgroundBrush(QBrush(QColor("#f5f7fa")))
        outside = QGraphicsRectItem(self._scene_rect())
        outside.setBrush(QBrush(QColor("#f5f7fa")))
        outside.setPen(QPen(Qt.NoPen))
        outside.setZValue(-220)
        outside.setData(0, "outside_field")
        self.scene_obj.addItem(outside)

        fence = self._add_world_rect(
            x_min - FENCE_W_MM,
            y_min - FENCE_W_MM,
            length + FENCE_W_MM * 2,
            width + FENCE_W_MM * 2,
            QPen(QColor("#965035"), 2.0),
            QBrush(QColor("#eeb888")),
        )
        fence.setZValue(-180)
        fence.setData(0, "field_fence")

        boundary = self._add_world_rect(
            x_min,
            y_min,
            length,
            width,
            QPen(QColor("#142d50"), 2.0),
            QBrush(QColor("#f0f4fa")),
        )
        boundary.setZValue(-170)
        boundary.setData(0, "field_boundary")

        for x_mm in range(int(x_min), int(x_max) + 1, GRID_STEP_MM):
            pen = QPen(QColor("#dce1e6"), 1.0)
            if x_mm % 1000 == 0:
                pen = QPen(QColor("#bec3cd"), 1.5)
            item = self._add_world_line(x_mm, y_min, x_mm, y_max, pen)
            item.setData(0, "grid_line")
            item.setZValue(-150)
        for y_mm in range(int(y_min), int(y_max) + 1, GRID_STEP_MM):
            pen = QPen(QColor("#dce1e6"), 1.0)
            if y_mm % 1000 == 0:
                pen = QPen(QColor("#bec3cd"), 1.5)
            item = self._add_world_line(x_min, y_mm, x_max, y_mm, pen)
            item.setData(0, "grid_line")
            item.setZValue(-150)

        axis_pen = QPen(QColor("#dc2828"), 1.8, Qt.DashLine)
        for item in (
            self._add_world_line(0, y_min, 0, y_max, axis_pen),
            self._add_world_line(x_min, 0, x_max, 0, axis_pen),
        ):
            item.setData(0, "grid_line")
            item.setZValue(-140)

        self._add_label("中心 (0,0)", 35, 45, "#b91c1c", "label", -120)
        self._add_label(
            f"{int(length)} x {int(width)} mm",
            x_min + 30,
            y_max - 45,
            "#475569",
            "label",
            -120,
        )
        start_zone = self._add_world_rect(
            -400,
            -200,
            400,
            400,
            QPen(QColor("#145096"), 2.0),
            QBrush(QColor(205, 230, 255, 170)),
        )
        start_zone.setData(0, "start_zone")
        start_zone.setZValue(-130)
        self._add_label("起始区域", -360, 230, "#145096", "label", -120)

    def _draw_field_objects(self, project: ProjectV40) -> None:
        for item in project.field_objects.get("cylinders", []):
            x_mm = float(item["center_x_mm"])
            y_mm = float(item["center_y_mm"])
            radius_mm = float(item["radius_mm"])
            radius_px = radius_mm * FIELD_SCALE
            center = self.world_to_scene(x_mm, y_mm)
            circle = self.scene_obj.addEllipse(
                center.x() - radius_px,
                center.y() - radius_px,
                radius_px * 2,
                radius_px * 2,
                self._object_pen(item),
                QBrush(QColor(240, 160, 90, 190)),
            )
            circle.setToolTip(_dict_tooltip(item))
            circle.setData(0, "cylinder")
            circle.setZValue(10)
            self._add_label(
                _obstacle_label(item, "cylinder"),
                x_mm + radius_mm + 28,
                y_mm,
                "#825032",
                "label",
                20,
            )
        for item in project.field_objects.get("pickup_boxes", []):
            self._draw_box(item, "pickup_box", QColor(80, 190, 210, 170), "#146e82")
        for item in project.field_objects.get("drop_boxes", []):
            self._draw_box(item, "drop_box", QColor(180, 170, 230, 165), "#503ca0")

    def _draw_box(
        self, data: dict[str, Any], kind: str, brush_color: QColor, label_color: str
    ) -> None:
        group = QGraphicsItemGroup()
        width_mm = float(data["length_mm"])
        height_mm = float(data["width_mm"])
        rect = QGraphicsRectItem(
            -width_mm * FIELD_SCALE / 2.0,
            -height_mm * FIELD_SCALE / 2.0,
            width_mm * FIELD_SCALE,
            height_mm * FIELD_SCALE,
        )
        rect.setPen(self._object_pen(data))
        rect.setBrush(QBrush(brush_color))
        rect.setToolTip(_dict_tooltip(data))
        rect.setData(0, f"{kind}_shape")
        group.addToGroup(rect)
        group.setPos(
            self.world_to_scene(float(data["center_x_mm"]), float(data["center_y_mm"]))
        )
        group.setRotation(-float(data.get("yaw_ddeg", 0)) / 10.0)
        group.setZValue(12)
        group.setData(0, kind)
        self.scene_obj.addItem(group)
        self._add_label(
            _obstacle_label(data, kind),
            float(data["center_x_mm"]) + width_mm / 2.0 + 35,
            float(data["center_y_mm"]),
            label_color,
            "label",
            22,
        )

    def _draw_sites(self, project: ProjectV40) -> None:
        for key in SITE_KEYS:
            site = project.sites[key]
            color = "#f59e0b" if site_has_yaw(key) else "#10b981"
            selected = "#fef3c7" if site_has_yaw(key) else "#dcfce7"
            item = EditablePointItem(
                self, key, radius=12.0, color=color, selected_color=selected,
                movable=self.editable and self.mode != "template",
            )
            item.setPos(self.world_to_scene(int(site["x_mm"]), int(site["y_mm"])))
            item.setToolTip(f"{key}\nx={site['x_mm']} y={site['y_mm']}")
            item.setData(0, "site")
            item.setData(1, key)
            self.scene_obj.addItem(item)
            self.site_items[key] = item
            offset_x, offset_y = SITE_LABEL_OFFSETS.get(key, (70, 70))
            self._add_label(
                SITE_LABELS.get(key, site_label(key)),
                int(site["x_mm"]) + offset_x,
                int(site["y_mm"]) + offset_y,
                "#111827",
                "site_label",
                130,
                size=8,
            )
            if site_has_yaw(key):
                yaw = int(site.get("yaw_ddeg", 0))
                line = yaw_line(
                    self,
                    int(site["x_mm"]),
                    int(site["y_mm"]),
                    yaw,
                    QPen(QColor("#b45309"), 2.0),
                )
                line.setData(0, "site_yaw_line")
                self.scene_obj.addItem(line)
                handle = YawHandleItem(
                    self,
                    key,
                    center_x_mm=int(site["x_mm"]),
                    center_y_mm=int(site["y_mm"]),
                    yaw_ddeg=yaw,
                    color="#fde68a",
                )
                handle.setPos(
                    self.yaw_handle_scene_point(
                        int(site["x_mm"]), int(site["y_mm"]), yaw
                    )
                )
                handle.setData(0, "site_yaw_handle")
                handle.setEnabled(self.editable and self.mode != "template")
                self.scene_obj.addItem(handle)
                self.site_yaw_items[key] = handle

    def _draw_manual_points(self) -> None:
        if self.layers.get("SPARSE_PATH", True) and len(self.manual_points) > 1:
            pen = QPen(QColor("#0f766e"), 2.0, Qt.DashLine)
            for left, right in zip(self.manual_points, self.manual_points[1:]):
                line = self._add_world_line(
                    left.x_mm, left.y_mm, right.x_mm, right.y_mm, pen
                )
                line.setData(0, "sparse_path")
                line.setZValue(40)
        for index, point in enumerate(self.manual_points):
            color = {
                "START": "#22c55e",
                "WAYPOINT": "#0ea5e9",
                "ARRIVAL": "#f97316",
                "TASK_ANCHOR": "#a855f7",
            }.get(point.point_type, "#94a3b8")
            item = EditablePointItem(
                self,
                index,
                radius=11.0,
                color=color,
                selected_color="#fef9c3",
                movable=self.editable,
            )
            item.setPos(self.world_to_scene(point.x_mm, point.y_mm))
            label = point.point_id or f"{index}:{point.point_type}"
            item.setToolTip(f"{label}\nx={point.x_mm} y={point.y_mm}")
            item.setData(0, "manual_point")
            item.setData(1, index)
            self.scene_obj.addItem(item)
            self.manual_items[index] = item
            self._add_label(
                label,
                point.x_mm + 70,
                point.y_mm + 70,
                "#111827",
                "manual_label",
                130,
                size=8,
            )
            if point.has_yaw():
                yaw = int(point.yaw_ddeg or 0)
                line = yaw_line(
                    self,
                    point.x_mm,
                    point.y_mm,
                    yaw,
                    QPen(QColor("#0f172a"), 2.0),
                )
                line.setData(0, "manual_yaw_line")
                self.scene_obj.addItem(line)
                handle = YawHandleItem(
                    self,
                    index,
                    center_x_mm=point.x_mm,
                    center_y_mm=point.y_mm,
                    yaw_ddeg=yaw,
                    color="#93c5fd",
                )
                handle.setPos(self.yaw_handle_scene_point(point.x_mm, point.y_mm, yaw))
                handle.setData(0, "manual_yaw_handle")
                handle.setEnabled(self.editable)
                self.scene_obj.addItem(handle)
                self.manual_yaw_items[index] = handle

    def _draw_leg(self, leg: LegV40) -> None:
        nodes = [node for node in leg.nodes if "x_mm" in node and "y_mm" in node]
        if len(nodes) >= 2 and self.layers.get("DENSE_PATH", True):
            path = QPainterPath(
                self.world_to_scene(float(nodes[0]["x_mm"]), float(nodes[0]["y_mm"]))
            )
            for node in nodes[1:]:
                path.lineTo(
                    self.world_to_scene(float(node["x_mm"]), float(node["y_mm"]))
                )
            item = QGraphicsPathItem(path)
            item.setPen(QPen(QColor("#7c3aed"), 4.0))
            item.setZValue(60)
            item.setData(0, "dense_path")
            item.setToolTip(leg.leg_id)
            self.scene_obj.addItem(item)
            if self.layers.get("SPEED_OVERLAY", True):
                self._draw_speed_overlay(nodes)
        if self.layers.get("CONTROL_POINTS", True):
            for cp in leg.control_points:
                if "x_mm" not in cp or "y_mm" not in cp:
                    continue
                scene = self.world_to_scene(float(cp["x_mm"]), float(cp["y_mm"]))
                item = self.scene_obj.addRect(
                    scene.x() - 6,
                    scene.y() - 6,
                    12,
                    12,
                    QPen(QColor("#0f172a"), 1.5),
                    QBrush(QColor("#fde047")),
                )
                item.setData(0, "control_point")
                item.setZValue(80)
        if nodes and self.layers.get("COLLISION_FOOTPRINT", True):
            node = nodes[-1]
            self._draw_collision_footprint(
                float(node["x_mm"]), float(node["y_mm"]), int(node.get("yaw_ddeg", 0))
            )

    def _draw_preview_curve(self) -> None:
        if len(self.preview_xy) < 2:
            return
        path = QPainterPath(self.world_to_scene(*self.preview_xy[0]))
        for point in self.preview_xy[1:]:
            path.lineTo(self.world_to_scene(*point))
        item = QGraphicsPathItem(path)
        item.setPen(QPen(QColor("#0891b2"), 2.5, Qt.DashLine))
        item.setZValue(55)
        item.setData(0, "preview_curve")
        item.setToolTip("轻量预览（未严格验证）")
        self.scene_obj.addItem(item)

    def _draw_speed_overlay(self, nodes: list[dict[str, Any]]) -> None:
        for left, right in zip(nodes, nodes[1:]):
            speed = math.hypot(
                float(right.get("vx_mmps", 0)), float(right.get("vy_mmps", 0))
            )
            color = (
                QColor("#22c55e")
                if speed < 300
                else QColor("#eab308")
                if speed < 800
                else QColor("#ef4444")
            )
            line = self._add_world_line(
                float(left["x_mm"]),
                float(left["y_mm"]),
                float(right["x_mm"]),
                float(right["y_mm"]),
                QPen(color, 2.0),
            )
            line.setData(0, "speed_overlay")
            line.setZValue(65)

    def _draw_collision_footprint(
        self, x_mm: float, y_mm: float, yaw_ddeg: int
    ) -> None:
        if self.project is None:
            return
        footprint = self.project.vehicle.get("footprint", {})
        r_large = float(footprint.get("r_large_mm", 120))
        r_small = float(footprint.get("r_small_mm", 70))
        center = self.world_to_scene(x_mm, y_mm)
        r_large_px = r_large * FIELD_SCALE
        r_small_px = r_small * FIELD_SCALE
        large = self.scene_obj.addEllipse(
            center.x() - r_large_px,
            center.y() - r_large_px,
            r_large_px * 2,
            r_large_px * 2,
            QPen(QColor("#475569"), 1.5),
            QBrush(QColor(71, 85, 105, 25)),
        )
        large.setData(0, "collision_footprint")
        large.setZValue(30)
        small = self.scene_obj.addEllipse(
            center.x() - r_small_px,
            center.y() - r_small_px,
            r_small_px * 2,
            r_small_px * 2,
            QPen(QColor("#16a34a"), 1.0, Qt.DashLine),
            QBrush(Qt.NoBrush),
        )
        small.setData(0, "collision_footprint")
        small.setZValue(31)
        polygon = _pickup_clipped_disk_polygon(r_large_px, r_small_px)
        item = QGraphicsPolygonItem(polygon)
        item.setPen(QPen(QColor("#dc2626"), 1.5))
        item.setBrush(QBrush(QColor(220, 38, 38, 28)))
        item.setPos(center)
        item.setRotation(-yaw_ddeg / 10.0)
        item.setData(0, "collision_footprint")
        item.setZValue(32)
        self.scene_obj.addItem(item)

    def _draw_topology_gates(self, project: ProjectV40) -> None:
        profiles: list[dict[str, Any]]
        if self.topology_gates_override is not None:
            profiles = [{"gates": list(self.topology_gates_override)}]
        else:
            profiles = [item for item in project.topology_profiles.values() if isinstance(item, dict)]
        for profile in profiles:
            gates = profile.get("gates", [])
            for index, gate in enumerate(gates):
                try:
                    start = gate.get("a", gate.get("start", {}))
                    end = gate.get("b", gate.get("end", {}))
                    x1 = float(gate.get("x1_mm", start.get("x_mm")))
                    y1 = float(gate.get("y1_mm", start.get("y_mm")))
                    x2 = float(gate.get("x2_mm", end.get("x_mm")))
                    y2 = float(gate.get("y2_mm", end.get("y_mm")))
                except (AttributeError, TypeError, ValueError):
                    continue
                item = self._add_world_line(
                    x1, y1, x2, y2, QPen(QColor("#a855f7"), 2.0, Qt.DashLine)
                )
                item.setData(0, "topology_gate")
                item.setToolTip(f"gate {index}")
                item.setZValue(35)

    def _add_world_line(
        self, x1: float, y1: float, x2: float, y2: float, pen: QPen
    ) -> QGraphicsLineItem:
        a = self.world_to_scene(x1, y1)
        b = self.world_to_scene(x2, y2)
        return self.scene_obj.addLine(a.x(), a.y(), b.x(), b.y(), pen)

    def _add_world_rect(
        self,
        x_mm: float,
        y_mm: float,
        width_mm: float,
        height_mm: float,
        pen: QPen,
        brush: QBrush,
    ) -> QGraphicsRectItem:
        top_left = self.world_to_scene(x_mm, y_mm + height_mm)
        return self.scene_obj.addRect(
            top_left.x(),
            top_left.y(),
            width_mm * FIELD_SCALE,
            height_mm * FIELD_SCALE,
            pen,
            brush,
        )

    def _add_label(
        self,
        text: str,
        x_mm: float,
        y_mm: float,
        color: str,
        kind: str,
        z: float,
        *,
        size: int = 9,
    ) -> None:
        if not self.layers.get("LABELS", True) and kind.endswith("label"):
            return
        scene = self.world_to_scene(x_mm, y_mm)
        item = QGraphicsSimpleTextItem(text)
        item.setBrush(QBrush(QColor(color)))
        font = QFont("Arial")
        font.setPointSize(size)
        item.setFont(font)
        item.setPos(scene)
        item.setZValue(z)
        item.setData(0, kind)
        self.scene_obj.addItem(item)

    def _object_pen(self, item: dict[str, Any]) -> QPen:
        color = QColor("#0f172a") if item.get("enabled", True) else QColor("#94a3b8")
        style = Qt.SolidLine if item.get("configured", False) else Qt.DashLine
        return QPen(color, 1.7, style)

    def _field_size(self) -> tuple[int, int]:
        if self.project is not None:
            field = self.project.nominal_field
            return (
                int(field.get("length_mm", DEFAULT_FIELD_LENGTH_MM)),
                int(field.get("width_mm", DEFAULT_FIELD_WIDTH_MM)),
            )
        return DEFAULT_FIELD_LENGTH_MM, DEFAULT_FIELD_WIDTH_MM

    def _field_half_size(self) -> tuple[float, float]:
        length, width = self._field_size()
        return length / 2.0, width / 2.0

    def _field_limits(self) -> tuple[float, float, float, float]:
        half_length, half_width = self._field_half_size()
        return -half_length, half_length, -half_width, half_width

    def _field_rect(self) -> QRectF:
        x_min, x_max, _y_min, y_max = self._field_limits()
        top_left = self.world_to_scene(x_min, y_max)
        length, width = self._field_size()
        return QRectF(top_left.x(), top_left.y(), length * FIELD_SCALE, width * FIELD_SCALE)

    def _scene_rect(self) -> QRectF:
        x_min, x_max, y_min, y_max = self._field_limits()
        top_left = self.world_to_scene(x_min - FENCE_W_MM, y_max + FENCE_W_MM)
        bottom_right = self.world_to_scene(x_max + FENCE_W_MM, y_min - FENCE_W_MM)
        return QRectF(
            top_left.x() - SCENE_MARGIN_PX,
            top_left.y() - SCENE_MARGIN_PX,
            bottom_right.x() - top_left.x() + 2 * SCENE_MARGIN_PX,
            bottom_right.y() - top_left.y() + 2 * SCENE_MARGIN_PX,
        )

    def _contains_world_point(self, x_mm: float, y_mm: float) -> bool:
        x_min, x_max, y_min, y_max = self._field_limits()
        return x_min <= x_mm <= x_max and y_min <= y_mm <= y_max


def _pickup_clipped_disk_polygon(r_large_px: float, r_small_px: float):
    from PySide6.QtGui import QPolygonF

    points: list[QPointF] = []
    if r_large_px <= 0:
        return QPolygonF(points)
    r_small_px = max(0.0, min(r_small_px, r_large_px))
    chord_y = math.sqrt(max(0.0, r_large_px * r_large_px - r_small_px * r_small_px))
    alpha = math.acos(r_small_px / r_large_px)
    steps = 40
    for step in range(steps + 1):
        theta = alpha + (2.0 * math.pi - 2.0 * alpha) * step / steps
        points.append(
            QPointF(math.cos(theta) * r_large_px, -math.sin(theta) * r_large_px)
        )
    points.append(QPointF(r_small_px, -chord_y))
    return QPolygonF(points)


def _obstacle_label(data: dict[str, Any], kind: str) -> str:
    if kind == "pickup_box":
        site = str(data.get("physical_pick_site", data.get("obstacle_id", "")))
        return site.replace("PICK_", "P")
    if kind == "drop_box":
        site = str(data.get("physical_drop_site", data.get("obstacle_id", "")))
        return site.replace("F_DROP_", "D")
    obstacle_id = str(data.get("obstacle_id", "C"))
    if "CYLINDER_" in obstacle_id:
        return obstacle_id.replace("CYLINDER_", "C")
    return obstacle_id


def _dict_tooltip(data: dict[str, Any]) -> str:
    return "\n".join(f"{key}: {value}" for key, value in data.items())

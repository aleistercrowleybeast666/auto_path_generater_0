"""Graphics items used by the V4 field editor."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from PySide6.QtCore import QPointF, Qt
from PySide6.QtGui import QBrush, QColor, QPen
from PySide6.QtWidgets import QApplication, QGraphicsEllipseItem, QGraphicsItem, QGraphicsLineItem

YAW_HANDLE_RADIUS_MM = 135.0


@dataclass(frozen=True)
class DragCommit:
    key: str | int
    old_x_mm: int
    old_y_mm: int
    new_x_mm: int
    new_y_mm: int


@dataclass(frozen=True)
class YawCommit:
    key: str | int
    old_yaw_ddeg: int
    new_yaw_ddeg: int


class EditablePointItem(QGraphicsEllipseItem):
    def __init__(
        self,
        owner: Any,
        key: str | int,
        *,
        radius: float,
        color: str,
        selected_color: str,
        movable: bool = True,
    ) -> None:
        super().__init__(-radius, -radius, radius * 2.0, radius * 2.0)
        self.owner = owner
        self.key = key
        self.color = color
        self.selected_color = selected_color
        self._press_world: tuple[int, int] | None = None
        self.setBrush(QBrush(QColor(color)))
        self.setPen(QPen(QColor("#1f2937"), 2.0))
        flags = QGraphicsItem.ItemIsSelectable | QGraphicsItem.ItemSendsGeometryChanges
        if movable:
            flags |= QGraphicsItem.ItemIsMovable
            self.setCursor(Qt.OpenHandCursor)
        self.setFlags(flags)
        self.setZValue(100)
        self.setData(0, "point")
        self.setData(1, key)

    def set_highlighted(self, highlighted: bool) -> None:
        self.setBrush(QBrush(QColor(self.selected_color if highlighted else self.color)))
        self.setPen(QPen(QColor("#111827" if highlighted else "#1f2937"), 3.0 if highlighted else 2.0))

    def mousePressEvent(self, event):  # type: ignore[override]
        self._press_world = self.owner.scene_to_world_int(self.pos())
        super().mousePressEvent(event)

    def itemChange(self, change, value):  # type: ignore[override]
        if change == QGraphicsItem.ItemPositionChange and self.scene() is not None:
            return self.owner.clamp_scene_point(value)
        if change == QGraphicsItem.ItemPositionHasChanged and self.scene() is not None and not self.owner.rebuilding:
            x_mm, y_mm = self.owner.scene_to_world_int(self.pos())
            self.owner.point_position_preview(self.key, x_mm, y_mm)
        if change == QGraphicsItem.ItemSelectedHasChanged and self.scene() is not None and not self.owner.rebuilding:
            if bool(value):
                self.owner.point_item_selected(self.key)
        return super().itemChange(change, value)

    def mouseReleaseEvent(self, event):  # type: ignore[override]
        super().mouseReleaseEvent(event)
        old = self._press_world
        if old is not None:
            x_mm, y_mm = self.owner.scene_to_world_int(self.pos())
            self.owner.point_position_committed(DragCommit(self.key, old[0], old[1], x_mm, y_mm))
        self._press_world = None


class YawHandleItem(QGraphicsEllipseItem):
    def __init__(
        self,
        owner: Any,
        key: str | int,
        *,
        center_x_mm: int,
        center_y_mm: int,
        yaw_ddeg: int,
        color: str,
    ) -> None:
        super().__init__(-7.0, -7.0, 14.0, 14.0)
        self.owner = owner
        self.key = key
        self.center_x_mm = center_x_mm
        self.center_y_mm = center_y_mm
        self._press_yaw = yaw_ddeg
        self.setBrush(QBrush(QColor(color)))
        self.setPen(QPen(QColor("#111827"), 1.7))
        self.setFlags(QGraphicsItem.ItemIsMovable | QGraphicsItem.ItemSendsGeometryChanges)
        self.setCursor(Qt.OpenHandCursor)
        self.setZValue(120)
        self.setData(0, "yaw_handle")
        self.setData(1, key)

    def mousePressEvent(self, event):  # type: ignore[override]
        self._press_yaw = self.owner.yaw_for_key(self.key)
        super().mousePressEvent(event)

    def itemChange(self, change, value):  # type: ignore[override]
        if change == QGraphicsItem.ItemPositionChange and self.scene() is not None:
            yaw = _snap_yaw(_yaw_from_scene(self.owner, self.center_x_mm, self.center_y_mm, value))
            return self.owner.yaw_handle_scene_point(self.center_x_mm, self.center_y_mm, yaw)
        if change == QGraphicsItem.ItemPositionHasChanged and self.scene() is not None and not self.owner.rebuilding:
            yaw = _snap_yaw(_yaw_from_scene(self.owner, self.center_x_mm, self.center_y_mm, self.pos()))
            self.owner.yaw_preview(self.key, yaw)
        return super().itemChange(change, value)

    def mouseReleaseEvent(self, event):  # type: ignore[override]
        super().mouseReleaseEvent(event)
        yaw = _snap_yaw(_yaw_from_scene(self.owner, self.center_x_mm, self.center_y_mm, self.pos()))
        self.owner.yaw_committed(YawCommit(self.key, self._press_yaw, yaw))


def yaw_line(owner: Any, center_x_mm: int, center_y_mm: int, yaw_ddeg: int, pen: QPen) -> QGraphicsLineItem:
    start = owner.world_to_scene(center_x_mm, center_y_mm)
    end = owner.yaw_handle_scene_point(center_x_mm, center_y_mm, yaw_ddeg)
    item = QGraphicsLineItem(start.x(), start.y(), end.x(), end.y())
    item.setPen(pen)
    item.setZValue(110)
    item.setData(0, "yaw_line")
    return item


def _yaw_from_scene(owner: Any, center_x_mm: int, center_y_mm: int, scene_point: QPointF) -> int:
    x_mm, y_mm = owner.scene_to_world_float(scene_point)
    return int(round(math.degrees(math.atan2(y_mm - center_y_mm, x_mm - center_x_mm)) * 10.0))


def _snap_yaw(yaw_ddeg: int) -> int:
    modifiers = QApplication.keyboardModifiers()
    if modifiers & Qt.ShiftModifier:
        return int(round(yaw_ddeg / 150.0) * 150)
    if modifiers & Qt.ControlModifier:
        return int(round(yaw_ddeg / 10.0) * 10)
    return yaw_ddeg

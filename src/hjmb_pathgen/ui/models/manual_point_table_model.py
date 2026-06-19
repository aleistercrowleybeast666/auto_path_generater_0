"""Table model for editable MANUAL_FREE sparse points."""

from __future__ import annotations

from PySide6.QtCore import QAbstractTableModel, QModelIndex, Qt, Signal

from ..ui_state import ManualPointDraft


class ManualPointTableModel(QAbstractTableModel):
    pointEdited = Signal(int)

    HEADERS = ("seq", "type", "x", "y", "yaw", "exact pass", "action count", "status")

    def __init__(self) -> None:
        super().__init__()
        self.points: list[ManualPointDraft] = []

    def set_points(self, points: list[ManualPointDraft]) -> None:
        self.beginResetModel()
        self.points = points
        self.endResetModel()

    def rowCount(self, parent=QModelIndex()) -> int:  # noqa: N802
        return 0 if parent.isValid() else len(self.points)

    def columnCount(self, parent=QModelIndex()) -> int:  # noqa: N802
        return 0 if parent.isValid() else len(self.HEADERS)

    def data(self, index: QModelIndex, role=Qt.DisplayRole):  # noqa: ANN001
        if not index.isValid():
            return None
        point = self.points[index.row()]
        col = index.column()
        if role in (Qt.DisplayRole, Qt.EditRole):
            if col == 0:
                return index.row()
            if col == 1:
                return point.point_type
            if col == 2:
                return point.x_mm
            if col == 3:
                return point.y_mm
            if col == 4:
                return point.yaw_ddeg if point.has_yaw() else "—"
            if col == 5:
                return point.exact_pass
            if col == 6:
                return 0
            if col == 7:
                return "可编辑"
        if role == Qt.CheckStateRole and col == 5:
            return Qt.Checked if point.exact_pass else Qt.Unchecked
        if role == Qt.TextAlignmentRole:
            return Qt.AlignCenter
        return None

    def setData(self, index: QModelIndex, value, role=Qt.EditRole):  # noqa: ANN001, N802
        if not index.isValid():
            return False
        row = index.row()
        point = self.points[row]
        col = index.column()
        try:
            if role == Qt.EditRole and col == 2:
                point.x_mm = int(round(float(value)))
            elif role == Qt.EditRole and col == 3:
                point.y_mm = int(round(float(value)))
            elif role == Qt.EditRole and col == 4 and point.has_yaw():
                point.yaw_ddeg = int(round(float(value)))
            elif role == Qt.CheckStateRole and col == 5:
                point.exact_pass = value == Qt.Checked
            else:
                return False
        except (TypeError, ValueError):
            return False
        self.dataChanged.emit(self.index(row, 0), self.index(row, len(self.HEADERS) - 1))
        self.pointEdited.emit(row)
        return True

    def flags(self, index: QModelIndex):  # noqa: ANN001
        base = Qt.ItemIsSelectable | Qt.ItemIsEnabled
        if not index.isValid():
            return base
        col = index.column()
        point = self.points[index.row()]
        if col in {2, 3} or (col == 4 and point.has_yaw()):
            return base | Qt.ItemIsEditable
        if col == 5:
            return base | Qt.ItemIsUserCheckable
        return base

    def headerData(self, section: int, orientation: Qt.Orientation, role=Qt.DisplayRole):  # noqa: ANN001, N802
        if role == Qt.DisplayRole and orientation == Qt.Horizontal:
            return self.HEADERS[section]
        return None

    def refresh_row(self, row: int) -> None:
        if 0 <= row < len(self.points):
            self.dataChanged.emit(self.index(row, 0), self.index(row, len(self.HEADERS) - 1))

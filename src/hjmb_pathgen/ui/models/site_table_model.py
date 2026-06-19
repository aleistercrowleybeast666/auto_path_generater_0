"""Table model for the ten V4 fixed project sites."""

from __future__ import annotations

from PySide6.QtCore import QAbstractTableModel, QModelIndex, Qt, Signal

from hjmb_pathgen.models.project import ProjectV40

from ..ui_state import SITE_KEYS, site_has_yaw, site_kind


class SiteTableModel(QAbstractTableModel):
    siteEdited = Signal(str)

    HEADERS = ("ID", "类型", "configured", "x_mm", "y_mm", "yaw_ddeg", "状态", "引用数", "stale影响")

    def __init__(self) -> None:
        super().__init__()
        self.project: ProjectV40 | None = None

    def set_project(self, project: ProjectV40 | None) -> None:
        self.beginResetModel()
        self.project = project
        self.endResetModel()

    def rowCount(self, parent=QModelIndex()) -> int:  # noqa: N802
        return 0 if parent.isValid() or self.project is None else len(SITE_KEYS)

    def columnCount(self, parent=QModelIndex()) -> int:  # noqa: N802
        return 0 if parent.isValid() else len(self.HEADERS)

    def data(self, index: QModelIndex, role=Qt.DisplayRole):  # noqa: ANN001
        if not index.isValid() or self.project is None:
            return None
        key = SITE_KEYS[index.row()]
        site = self.project.sites[key]
        col = index.column()
        if role in (Qt.DisplayRole, Qt.EditRole):
            if col == 0:
                return key
            if col == 1:
                return "取货/起点姿态" if site_has_yaw(key) else "放货物理点"
            if col == 2:
                return bool(site["configured"])
            if col == 3:
                return int(site["x_mm"])
            if col == 4:
                return int(site["y_mm"])
            if col == 5:
                return int(site["yaw_ddeg"]) if site_has_yaw(key) else "—"
            if col == 6:
                return "已配置" if site["configured"] else "未配置"
            if col == 7:
                return 0
            if col == 8:
                return "site_config_hash"
        if role == Qt.ToolTipRole:
            return f"{key}\n{site_kind(key)}\n编辑只标记 STALE，不自动规划"
        if role == Qt.TextAlignmentRole:
            return Qt.AlignCenter
        if role == Qt.CheckStateRole and col == 2:
            return Qt.Checked if site["configured"] else Qt.Unchecked
        return None

    def setData(self, index: QModelIndex, value, role=Qt.EditRole):  # noqa: ANN001, N802
        if not index.isValid() or self.project is None:
            return False
        key = SITE_KEYS[index.row()]
        site = self.project.sites[key]
        col = index.column()
        try:
            if role == Qt.CheckStateRole and col == 2:
                site["configured"] = value == Qt.Checked
            elif role == Qt.EditRole and col == 3:
                site["x_mm"] = int(round(float(value)))
            elif role == Qt.EditRole and col == 4:
                site["y_mm"] = int(round(float(value)))
            elif role == Qt.EditRole and col == 5 and site_has_yaw(key):
                site["yaw_ddeg"] = int(round(float(value)))
            else:
                return False
        except (TypeError, ValueError):
            return False
        self.dataChanged.emit(self.index(index.row(), 0), self.index(index.row(), len(self.HEADERS) - 1))
        self.siteEdited.emit(key)
        return True

    def flags(self, index: QModelIndex):  # noqa: ANN001
        base = Qt.ItemIsSelectable | Qt.ItemIsEnabled
        if not index.isValid() or self.project is None:
            return base
        col = index.column()
        if col in {3, 4} or (col == 5 and site_has_yaw(SITE_KEYS[index.row()])):
            return base | Qt.ItemIsEditable
        if col == 2:
            return base | Qt.ItemIsUserCheckable
        return base

    def headerData(self, section: int, orientation: Qt.Orientation, role=Qt.DisplayRole):  # noqa: ANN001, N802
        if role == Qt.DisplayRole and orientation == Qt.Horizontal:
            return self.HEADERS[section]
        return None

    def row_for_site(self, site_key: str) -> int:
        return SITE_KEYS.index(site_key)

    def refresh_site(self, site_key: str) -> None:
        row = self.row_for_site(site_key)
        self.dataChanged.emit(self.index(row, 0), self.index(row, len(self.HEADERS) - 1))

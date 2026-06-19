"""Planner profile editor tab."""

from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QLabel, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget

from ..ui_state import LoadedProjectState


class PlanningTab(QWidget):
    dirtyChanged = Signal(bool, str)

    def __init__(self) -> None:
        super().__init__()
        self.state: LoadedProjectState | None = None
        self.table = QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(("profile", "max_spacing_mm", "max_yaw_step_ddeg", "algorithm"))
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("规划参数修改只标记相关 leg STALE，不自动优化。"))
        layout.addWidget(self.table, 1)
        self.table.itemChanged.connect(self._item_changed)

    def set_state(self, state: LoadedProjectState | None) -> None:
        self.state = state
        self.table.blockSignals(True)
        profiles = state.project.planner_profiles if state is not None else {}
        self.table.setRowCount(len(profiles))
        for row, key in enumerate(sorted(profiles)):
            profile = profiles[key]
            values = (key, profile.get("max_spacing_mm", ""), profile.get("max_yaw_step_ddeg", ""), profile.get("planner_algorithm_version", "PHASE6_LEG_OPTIMIZER_V1"))
            for col, value in enumerate(values):
                self.table.setItem(row, col, QTableWidgetItem(str(value)))
        self.table.blockSignals(False)

    def _item_changed(self, item: QTableWidgetItem) -> None:
        if self.state is None or item.column() == 0:
            return
        key = self.table.item(item.row(), 0).text()
        profile = self.state.project.planner_profiles.setdefault(key, {})
        try:
            if item.column() == 1:
                profile["max_spacing_mm"] = int(float(item.text()))
            elif item.column() == 2:
                profile["max_yaw_step_ddeg"] = int(float(item.text()))
            elif item.column() == 3:
                profile["planner_algorithm_version"] = item.text()
        except ValueError:
            return
        self.dirtyChanged.emit(True, "planner profile 已修改；相关 leg 标记 STALE")

"""Leg Library tab."""

from __future__ import annotations

from PySide6.QtCore import QSortFilterProxyModel, Qt, Signal
from PySide6.QtWidgets import QHBoxLayout, QLabel, QLineEdit, QPushButton, QTableView, QVBoxLayout, QWidget

from ..models.leg_table_model import LegTableModel
from ..ui_state import LoadedProjectState


class LegLibraryTab(QWidget):
    statusMessage = Signal(str)
    openLegRequested = Signal(str)
    workerRequested = Signal(str, dict)

    def __init__(self) -> None:
        super().__init__()
        self.model = LegTableModel()
        self.proxy = QSortFilterProxyModel(self)
        self.proxy.setSourceModel(self.model)
        self.proxy.setFilterCaseSensitivity(Qt.CaseInsensitive)
        self.table = QTableView()
        self.table.setModel(self.proxy)
        self.search = QLineEdit()
        self.search.setPlaceholderText("搜索 leg_id/from/to")
        self._build_ui()
        self._connect()

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        top = QHBoxLayout()
        top.addWidget(QLabel("Leg Library"))
        top.addWidget(self.search, 1)
        optimize = QPushButton("Optimize Missing")
        optimize.clicked.connect(lambda: self.workerRequested.emit("optimize-missing-legs", {"include_stale": True}))
        validate = QPushButton("Validate All")
        validate.clicked.connect(lambda: self.workerRequested.emit("validate-all", {}))
        top.addWidget(optimize)
        top.addWidget(validate)
        layout.addLayout(top)
        layout.addWidget(self.table, 1)

    def _connect(self) -> None:
        self.search.textChanged.connect(self.proxy.setFilterFixedString)
        self.table.doubleClicked.connect(self._open_selected)

    def set_state(self, state: LoadedProjectState | None) -> None:
        self.model.set_library(state.leg_library if state is not None else None)
        self.table.resizeColumnsToContents()

    def _open_selected(self, index) -> None:  # noqa: ANN001
        source_index = self.proxy.mapToSource(index)
        leg = self.model.leg_at(source_index.row())
        if leg is not None:
            self.openLegRequested.emit(leg.leg_id)

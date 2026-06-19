"""Task/360 case table and worker controls."""

from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QHBoxLayout, QLabel, QPushButton, QSpinBox, QTableView, QVBoxLayout, QWidget

from ..models.task_case_table_model import TaskCaseTableModel
from ..ui_state import LoadedProjectState


class TaskCasesTab(QWidget):
    statusMessage = Signal(str)
    workerRequested = Signal(str, dict)
    openCaseRequested = Signal(int)

    def __init__(self) -> None:
        super().__init__()
        self.model = TaskCaseTableModel()
        self.table = QTableView()
        self.table.setModel(self.model)
        self.traj_spin = QSpinBox()
        self.traj_spin.setRange(0, 359)
        self._build_ui()
        self._connect()

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        top = QHBoxLayout()
        top.addWidget(QLabel("traj_id"))
        top.addWidget(self.traj_spin)
        generate_one = QPushButton("Generate One")
        generate_one.clicked.connect(lambda: self.workerRequested.emit("generate-one", {"traj_id": self.traj_spin.value()}))
        generate_all = QPushButton("Generate 360")
        generate_all.clicked.connect(lambda: self.workerRequested.emit("generate-all", {}))
        validate_all = QPushButton("Validate All")
        validate_all.clicked.connect(lambda: self.workerRequested.emit("validate-all", {}))
        optimize = QPushButton("Optimize Missing Legs")
        optimize.clicked.connect(lambda: self.workerRequested.emit("optimize-missing-legs", {"include_stale": True}))
        for button in (generate_one, generate_all, validate_all, optimize):
            top.addWidget(button)
        top.addStretch(1)
        layout.addLayout(top)
        layout.addWidget(self.table, 1)

    def _connect(self) -> None:
        self.table.clicked.connect(self._row_clicked)
        self.table.doubleClicked.connect(self._row_double_clicked)

    def set_state(self, state: LoadedProjectState | None) -> None:
        self.model.set_data(state.route_table if state is not None else None, state.task_cases if state is not None else {})
        self.table.resizeColumnsToContents()

    def _row_clicked(self, index) -> None:  # noqa: ANN001
        traj_id = self.model.traj_id_at(index.row())
        if traj_id is not None:
            self.traj_spin.setValue(traj_id)

    def _row_double_clicked(self, index) -> None:  # noqa: ANN001
        traj_id = self.model.traj_id_at(index.row())
        if traj_id is not None:
            self.openCaseRequested.emit(traj_id)

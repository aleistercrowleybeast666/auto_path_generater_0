"""Reports and final export tab."""

from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QComboBox, QFormLayout, QHBoxLayout, QLabel, QPushButton, QSpinBox, QTableView, QVBoxLayout, QWidget

from hjmb_pathgen.models.enums import PathSource

from ..models.report_table_model import ReportTableModel
from ..ui_state import LoadedProjectState


class ReportsFinalTab(QWidget):
    workerRequested = Signal(str, dict)
    statusMessage = Signal(str)

    def __init__(self) -> None:
        super().__init__()
        self.model = ReportTableModel()
        self.table = QTableView()
        self.table.setModel(self.model)
        self.traj_spin = QSpinBox()
        self.traj_spin.setRange(0, 359)
        self.source_combo = QComboBox()
        for source in PathSource:
            self.source_combo.addItem(source.value, source.value)
        layout = QVBoxLayout(self)
        form = QFormLayout()
        form.addRow("traj_id", self.traj_spin)
        form.addRow("source", self.source_combo)
        layout.addLayout(form)
        buttons = QHBoxLayout()
        export = QPushButton("Export Final BIN")
        export.clicked.connect(self._export_final)
        buttons.addWidget(export)
        buttons.addStretch(1)
        layout.addLayout(buttons)
        layout.addWidget(QLabel("Final 导出会重新走 export guard；有 STALE/FAILED 或未批准会被阻止。"))
        layout.addWidget(self.table, 1)

    def set_state(self, state: LoadedProjectState | None) -> None:
        self.model.set_data(state.reports if state is not None else [], state.final_bins if state is not None else {})
        self.table.resizeColumnsToContents()

    def _export_final(self) -> None:
        self.workerRequested.emit(
            "export-final",
            {"traj_id": self.traj_spin.value(), "path_source": str(self.source_combo.currentData())},
        )

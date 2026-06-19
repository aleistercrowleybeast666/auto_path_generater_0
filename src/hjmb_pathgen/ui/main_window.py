"""Phase 8 V4 workflow UI."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QFileDialog,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QPlainTextEdit,
    QProgressBar,
    QSpinBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from hjmb_pathgen.models.enums import PathSource
from hjmb_pathgen.services.project_service import ProjectLayout
from hjmb_pathgen.services.worker_process import WorkerJobHandle, start_worker_job


class V4MainWindow(QMainWindow):
    def __init__(self, project_root: str | Path | None = None) -> None:
        super().__init__()
        self.setWindowTitle("HJMB V4.0 Path Generator")
        self.resize(1180, 760)
        self._worker: WorkerJobHandle | None = None
        self._project_root = Path(project_root).resolve(strict=False) if project_root else Path.cwd()
        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(200)
        self._poll_timer.timeout.connect(self._poll_worker)

        self.project_edit = QLineEdit(str(self._project_root))
        self.status_label = QLabel("IDLE")
        self.progress = QProgressBar()
        self.progress.setRange(0, 0)
        self.progress.hide()
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.task_traj_spin = QSpinBox()
        self.task_traj_spin.setRange(0, 359)
        self.final_traj_spin = QSpinBox()
        self.final_traj_spin.setRange(0, 359)
        self.source_combo = QComboBox()
        for source in PathSource:
            self.source_combo.addItem(source.value, source.value)

        root = QWidget()
        layout = QVBoxLayout(root)
        layout.addLayout(self._project_bar())
        tabs = QTabWidget()
        tabs.addTab(self._project_tab(), "Project/Sites")
        tabs.addTab(self._vehicle_tab(), "Vehicle/Collision")
        tabs.addTab(self._current_leg_tab(), "Route/Leg")
        tabs.addTab(self._actions_tab(), "Actions")
        tabs.addTab(self._leg_library_tab(), "Leg Library")
        tabs.addTab(self._task_tab(), "Task/360")
        tabs.addTab(self._manual_tab(), "Manual Free")
        tabs.addTab(self._planning_tab(), "Planning")
        tabs.addTab(self._reports_tab(), "Reports/Final")
        layout.addWidget(tabs, 1)
        layout.addLayout(self._worker_bar())
        layout.addWidget(self.log, 1)
        self.setCentralWidget(root)

    def _project_bar(self) -> QHBoxLayout:
        layout = QHBoxLayout()
        browse = QPushButton("Open")
        browse.clicked.connect(self._choose_project)
        refresh = QPushButton("Status")
        refresh.clicked.connect(self._show_status)
        layout.addWidget(QLabel("Project"))
        layout.addWidget(self.project_edit, 1)
        layout.addWidget(browse)
        layout.addWidget(refresh)
        return layout

    def _worker_bar(self) -> QHBoxLayout:
        layout = QHBoxLayout()
        cancel = QPushButton("Cancel")
        cancel.clicked.connect(self._cancel_worker)
        layout.addWidget(self.status_label)
        layout.addWidget(self.progress)
        layout.addWidget(cancel)
        return layout

    def _project_tab(self) -> QWidget:
        return _form_tab(("project.json", "route_case_table.json", "site presets"))

    def _vehicle_tab(self) -> QWidget:
        return _form_tab(("vehicle footprint", "collision world", "continuous validation"))

    def _current_leg_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        row = QHBoxLayout()
        optimize = QPushButton("Optimize Missing Legs")
        optimize.clicked.connect(lambda: self._start_job("optimize-missing-legs", {"include_stale": True}))
        row.addWidget(optimize)
        layout.addLayout(row)
        return tab

    def _actions_tab(self) -> QWidget:
        return _form_tab(("source actions", "compiled actions", "KINEMATIC scan"))

    def _leg_library_tab(self) -> QWidget:
        return _form_tab(("leg_library.json", "approve/lock/clear", "dependency audit"))

    def _task_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        form = QFormLayout()
        form.addRow("traj_id", self.task_traj_spin)
        layout.addLayout(form)
        generate_one = QPushButton("Generate Current Case")
        generate_one.clicked.connect(lambda: self._start_job("generate-one", {"traj_id": self.task_traj_spin.value()}))
        generate_all = QPushButton("Generate 360")
        generate_all.clicked.connect(lambda: self._start_job("generate-all", {}))
        validate = QPushButton("Validate All")
        validate.clicked.connect(lambda: self._start_job("validate-all", {}))
        for button in (generate_one, generate_all, validate):
            layout.addWidget(button)
        layout.addStretch(1)
        return tab

    def _manual_tab(self) -> QWidget:
        return _form_tab(("manual_free/Pxxxx.json", "manual BIN", "detached override"))

    def _planning_tab(self) -> QWidget:
        return _form_tab(("speed limits", "wheel rpm", "topology gates"))

    def _reports_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        form = QFormLayout()
        form.addRow("traj_id", self.final_traj_spin)
        form.addRow("source", self.source_combo)
        layout.addLayout(form)
        export = QPushButton("Export Final BIN")
        export.clicked.connect(
            lambda: self._start_job(
                "export-final",
                {"traj_id": self.final_traj_spin.value(), "path_source": str(self.source_combo.currentData())},
            )
        )
        layout.addWidget(export)
        layout.addStretch(1)
        return tab

    def _choose_project(self) -> None:
        path = QFileDialog.getExistingDirectory(self, "Open V4 Project", self.project_edit.text())
        if path:
            self.project_edit.setText(path)
            self._project_root = Path(path)
            self._show_status()

    def _show_status(self) -> None:
        try:
            report = ProjectLayout.open(self.project_edit.text()).status()
            self._append({"status": report.status.value, "reasons": list(report.reasons)})
        except Exception as exc:  # noqa: BLE001 - UI boundary.
            self._append({"status": "FAILED", "error": str(exc)})

    def _start_job(self, job: str, params: dict[str, Any]) -> None:
        if self._worker is not None and self._worker.is_alive():
            QMessageBox.warning(self, "Worker Busy", "A worker job is already running.")
            return
        self._worker = start_worker_job(self.project_edit.text(), job, params)
        self.status_label.setText(job)
        self.progress.show()
        self._poll_timer.start()

    def _cancel_worker(self) -> None:
        if self._worker is not None:
            self._worker.cancel()
            self.status_label.setText("CANCEL REQUESTED")

    def _poll_worker(self) -> None:
        if self._worker is None:
            self._poll_timer.stop()
            self.progress.hide()
            return
        for message in self._worker.poll():
            self._append({"kind": message.kind, **message.payload})
            if message.kind in {"result", "error", "cancelled"}:
                self.status_label.setText(message.kind.upper())
        if not self._worker.is_alive():
            self._worker.join(0)
            self._poll_timer.stop()
            self.progress.hide()

    def _append(self, value: dict[str, Any]) -> None:
        self.log.appendPlainText(str(value))


def _form_tab(rows: tuple[str, ...]) -> QWidget:
    tab = QWidget()
    layout = QFormLayout(tab)
    for row in rows:
        layout.addRow(row, QLabel(""))
    return tab


def main() -> int:
    app = QApplication.instance() or QApplication([])
    window = V4MainWindow()
    window.show()
    return app.exec()

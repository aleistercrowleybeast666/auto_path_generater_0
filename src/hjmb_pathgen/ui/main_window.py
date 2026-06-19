"""V4 workflow GUI with real field editors and model-backed tabs."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from PySide6.QtCore import QTimer, Qt
from PySide6.QtWidgets import (
    QApplication,
    QDockWidget,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QStatusBar,
    QTabWidget,
    QToolBar,
    QVBoxLayout,
    QWidget,
)

from hjmb_pathgen.codec.json_codec import save_project
from hjmb_pathgen.services.project_service import ProjectLayout
from hjmb_pathgen.services.worker_process import WorkerJobHandle, start_worker_job

from .tabs.actions_tab import ActionsTab
from .tabs.leg_library_tab import LegLibraryTab
from .tabs.manual_free_tab import ManualFreeTab
from .tabs.planning_tab import PlanningTab
from .tabs.project_sites_tab import ProjectSitesTab
from .tabs.reports_final_tab import ReportsFinalTab
from .tabs.route_leg_tab import RouteLegTab
from .tabs.task_cases_tab import TaskCasesTab
from .tabs.vehicle_collision_tab import VehicleCollisionTab
from .ui_state import LoadedProjectState, project_summary


class V4MainWindow(QMainWindow):
    def __init__(self, project_root: str | Path | None = None) -> None:
        super().__init__()
        self.setWindowTitle("HJMB V4.0 路径生成器")
        self.resize(1680, 940)
        self.setMinimumSize(1280, 760)
        self._worker: WorkerJobHandle | None = None
        self._state: LoadedProjectState | None = None
        self._dirty = False
        self._stale = False
        self._project_root = Path(project_root).resolve(strict=False) if project_root else Path.cwd()
        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(200)
        self._poll_timer.timeout.connect(self._poll_worker)

        self.project_edit = QLineEdit(str(self._project_root))
        self.project_edit.setMinimumWidth(520)
        self.project_status = QLabel("未加载")
        self.worker_status = QLabel("IDLE")
        self.worker_status_bar = QLabel("worker: IDLE")
        self.progress = QProgressBar()
        self.progress.setRange(0, 0)
        self.progress.hide()
        self.cancel_button = QPushButton("Cancel")
        self.cancel_button.setEnabled(False)
        self.cancel_button.clicked.connect(self._cancel_worker)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(1200)
        self.tabs = QTabWidget()

        self.project_sites_tab = ProjectSitesTab()
        self.vehicle_tab = VehicleCollisionTab()
        self.route_leg_tab = RouteLegTab()
        self.actions_tab = ActionsTab()
        self.leg_library_tab = LegLibraryTab()
        self.task_cases_tab = TaskCasesTab()
        self.manual_free_tab = ManualFreeTab()
        self.planning_tab = PlanningTab()
        self.reports_final_tab = ReportsFinalTab()

        self._build_ui()
        self._connect_tabs()
        if project_root is not None:
            self.load_project_path(project_root)

    def _build_ui(self) -> None:
        toolbar = QToolBar("项目")
        toolbar.setMovable(False)
        open_button = QPushButton("打开项目")
        open_button.clicked.connect(self._choose_project)
        save_button = QPushButton("保存 project.json")
        save_button.clicked.connect(self.save_project)
        status_button = QPushButton("刷新状态")
        status_button.clicked.connect(self.show_project_status)
        toolbar.addWidget(QLabel("项目路径"))
        toolbar.addWidget(self.project_edit)
        toolbar.addWidget(open_button)
        toolbar.addWidget(save_button)
        toolbar.addWidget(status_button)
        toolbar.addSeparator()
        toolbar.addWidget(QLabel("项目状态"))
        toolbar.addWidget(self.project_status)
        self.addToolBar(toolbar)

        self.tabs.addTab(self.project_sites_tab, "Project/Sites")
        self.tabs.addTab(self.vehicle_tab, "Vehicle/Collision")
        self.tabs.addTab(self.route_leg_tab, "Route/Leg")
        self.tabs.addTab(self.actions_tab, "Actions")
        self.tabs.addTab(self.leg_library_tab, "Leg Library")
        self.tabs.addTab(self.task_cases_tab, "Task/360")
        self.tabs.addTab(self.manual_free_tab, "Manual Free")
        self.tabs.addTab(self.planning_tab, "Planning")
        self.tabs.addTab(self.reports_final_tab, "Reports/Final")

        root = QWidget()
        layout = QVBoxLayout(root)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)
        worker_row = QHBoxLayout()
        worker_row.addWidget(QLabel("Worker"))
        worker_row.addWidget(self.worker_status)
        worker_row.addWidget(self.progress)
        worker_row.addWidget(self.cancel_button)
        worker_row.addStretch(1)
        layout.addLayout(worker_row)
        layout.addWidget(self.tabs, 1)
        self.setCentralWidget(root)

        dock = QDockWidget("日志 / Worker 输出", self)
        dock.setWidget(self.log)
        dock.setAllowedAreas(Qt.BottomDockWidgetArea | Qt.TopDockWidgetArea)
        self.addDockWidget(Qt.BottomDockWidgetArea, dock)
        dock.resize(900, 120)
        dock.hide()
        self.log_dock = dock

        status = QStatusBar()
        self.setStatusBar(status)
        self.path_status = QLabel("项目: 未加载")
        self.mode_status = QLabel("模式: Project/Sites")
        self.dirty_status = QLabel("clean")
        self.coord_status = QLabel("x=— y=—")
        self.zoom_status = QLabel("zoom=—")
        for widget in (self.path_status, self.mode_status, self.dirty_status, self.worker_status_bar, self.coord_status, self.zoom_status):
            status.addPermanentWidget(widget)

    def _connect_tabs(self) -> None:
        for tab in (self.project_sites_tab, self.vehicle_tab, self.manual_free_tab, self.planning_tab, self.route_leg_tab):
            if hasattr(tab, "dirtyChanged"):
                tab.dirtyChanged.connect(self._mark_dirty)
        for tab in (
            self.project_sites_tab,
            self.vehicle_tab,
            self.route_leg_tab,
            self.leg_library_tab,
            self.task_cases_tab,
            self.manual_free_tab,
            self.reports_final_tab,
        ):
            if hasattr(tab, "statusMessage"):
                tab.statusMessage.connect(self._append_text)
            if hasattr(tab, "workerRequested"):
                tab.workerRequested.connect(self._start_job)
        self.leg_library_tab.openLegRequested.connect(self._open_leg)
        self.route_leg_tab.reloadRequested.connect(self._reload_current_project)
        self.task_cases_tab.openCaseRequested.connect(lambda traj_id: self._append_text(f"打开 Case P{traj_id:04d}"))
        self.tabs.currentChanged.connect(lambda index: self.mode_status.setText(f"模式: {self.tabs.tabText(index)}"))
        for view in (self.project_sites_tab.field_view, self.manual_free_tab.field_view, self.route_leg_tab.field_view, self.vehicle_tab.field_view):
            view.worldMouseMoved.connect(lambda x, y: self.coord_status.setText(f"x={x:.0f} y={y:.0f}"))
            view.zoomChanged.connect(lambda zoom: self.zoom_status.setText(f"zoom={zoom:.2f}"))

    def load_project_path(self, root: str | Path) -> bool:
        old_state = self._state
        try:
            state = LoadedProjectState.load(root)
        except Exception as exc:  # noqa: BLE001 - UI boundary keeps previous state.
            self._append({"status": "FAILED", "error": str(exc)})
            if old_state is None:
                self.project_status.setText("加载失败")
            QMessageBox.warning(self, "打开项目失败", str(exc))
            return False
        self._state = state
        self._project_root = state.layout.root
        self.project_edit.setText(str(state.layout.root))
        self._dirty = False
        self._stale = False
        self._apply_state_to_tabs()
        self.show_project_status()
        summary = project_summary(state.project)
        self._append({"status": "LOADED", **summary, "warnings": state.warnings})
        for warning in state.warnings:
            self._append_text(f"警告: {warning}")
        return True

    def save_project(self) -> None:
        if self._state is None:
            self._append_text("没有已加载项目")
            return
        try:
            save_project(self._state.layout.project_json, self._state.project)
        except Exception as exc:  # noqa: BLE001
            QMessageBox.warning(self, "保存失败", str(exc))
            self._append({"status": "SAVE_FAILED", "error": str(exc)})
            return
        self._dirty = False
        self._refresh_dirty_status()
        self._append_text("project.json 已保存；没有自动规划")

    def show_project_status(self) -> None:
        try:
            report = ProjectLayout.open(self.project_edit.text()).status()
            self.project_status.setText(report.status.value)
            self.path_status.setText(f"项目: {Path(self.project_edit.text()).name}")
            self._append({"status": report.status.value, "reasons": list(report.reasons)})
        except Exception as exc:  # noqa: BLE001
            self.project_status.setText("INVALID")
            self._append({"status": "FAILED", "error": str(exc)})

    def scene_dumps(self) -> dict[str, dict[str, Any]]:
        return {
            "project_sites": self.project_sites_tab.field_view.scene_dump(),
            "manual_free": self.manual_free_tab.field_view.scene_dump(),
            "route_leg": self.route_leg_tab.field_view.scene_dump(),
            "vehicle_collision": self.vehicle_tab.field_view.scene_dump(),
        }

    def _apply_state_to_tabs(self) -> None:
        for tab in (
            self.project_sites_tab,
            self.vehicle_tab,
            self.route_leg_tab,
            self.actions_tab,
            self.leg_library_tab,
            self.task_cases_tab,
            self.manual_free_tab,
            self.planning_tab,
            self.reports_final_tab,
        ):
            tab.set_state(self._state)
        self._refresh_dirty_status()

    def _choose_project(self) -> None:
        path = QFileDialog.getExistingDirectory(self, "打开 V4 项目", self.project_edit.text())
        if path:
            self.load_project_path(path)

    def _reload_current_project(self) -> None:
        if self._state is not None:
            self.load_project_path(self._state.layout.root)

    def _open_leg(self, leg_id: str) -> None:
        self.tabs.setCurrentWidget(self.route_leg_tab)
        self.route_leg_tab.open_leg(leg_id)

    def _mark_dirty(self, dirty: bool, reason: str) -> None:
        self._dirty = self._dirty or dirty
        self._stale = True
        self._refresh_dirty_status()
        self._append_text(reason)

    def _refresh_dirty_status(self) -> None:
        if self._dirty:
            self.dirty_status.setText("dirty / STALE")
        elif self._stale:
            self.dirty_status.setText("clean / STALE")
        else:
            self.dirty_status.setText("clean")

    def _start_job(self, job: str, params: dict[str, Any]) -> None:
        if self._state is None:
            QMessageBox.warning(self, "未加载项目", "请先打开 V4 项目。")
            return
        if self._worker is not None and self._worker.is_alive():
            QMessageBox.warning(self, "Worker Busy", "已有任务正在运行。")
            return
        self._worker = start_worker_job(self._state.layout.root, job, params)
        self.worker_status.setText(job)
        self.worker_status_bar.setText(f"worker: {job}")
        self.progress.show()
        self.cancel_button.setEnabled(True)
        self._poll_timer.start()
        self._append({"kind": "worker-start", "job": job, "params": params})

    def _cancel_worker(self) -> None:
        if self._worker is None or not self._worker.is_alive():
            self.cancel_button.setEnabled(False)
            return
        if QMessageBox.question(self, "取消任务", "确认取消当前 worker 任务？") != QMessageBox.Yes:
            return
        self._worker.cancel()
        self.worker_status.setText("CANCEL REQUESTED")

    def _poll_worker(self) -> None:
        if self._worker is None:
            self._worker_idle()
            return
        for message in self._worker.poll():
            self._append({"kind": message.kind, **message.payload})
            if message.kind in {"result", "error", "cancelled"}:
                self.worker_status.setText(message.kind.upper())
                self.worker_status_bar.setText(f"worker: {message.kind.upper()}")
                if message.kind == "error":
                    self.log_dock.show()
        if not self._worker.is_alive():
            self._worker.join(0)
            self._worker = None
            self._worker_idle()
            self._reload_current_project()

    def _worker_idle(self) -> None:
        self._poll_timer.stop()
        self.progress.hide()
        self.cancel_button.setEnabled(False)
        if self.worker_status.text() not in {"RESULT", "ERROR", "CANCELLED"}:
            self.worker_status.setText("IDLE")
            self.worker_status_bar.setText("worker: IDLE")

    def _append(self, value: dict[str, Any]) -> None:
        self.log.appendPlainText(str(value))

    def _append_text(self, text: str) -> None:
        self.log.appendPlainText(text)
        self.statusBar().showMessage(text, 5000)


def main() -> int:
    app = QApplication.instance() or QApplication([])
    window = V4MainWindow()
    window.show()
    return app.exec()

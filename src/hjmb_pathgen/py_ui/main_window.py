"""Final two-page V4 GUI composed around an explicit AppContext."""

from __future__ import annotations

import shutil
from pathlib import Path
from typing import Any

from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QAction
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QDockWidget,
    QFileDialog,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QSpinBox,
    QStatusBar,
    QTabWidget,
    QToolBar,
)

from hjmb_pathgen.py_app.app_context import AppContext
from hjmb_pathgen.py_domain.enums import GenerationMode
from hjmb_pathgen.py_io.codecs.json_codec import load_case, save_case, save_project
from hjmb_pathgen.py_io.layout.project_layout import ProjectLayout
from hjmb_pathgen.py_io.migration.old_v40_layout_migration import migrate_old_v40_layout
from hjmb_pathgen.py_io.migration.v35_case_migration import migrate_v35_to_manual, migrate_v35_to_semi_auto
from hjmb_pathgen.py_services.leg_clear_service import clear_optimized_leg_result
from hjmb_pathgen.py_services.mode_case_service import convert_full_auto_to_semi_auto
from hjmb_pathgen.py_services.phase9_delivery_service import write_json_report
from hjmb_pathgen.py_workers.worker_process import WorkerJobHandle, start_worker_job

from .pages.optimization_batch_page import OptimizationBatchPage
from .pages.path_editor_page import PathEditorPage


class V4MainWindow(QMainWindow):
    def __init__(self, project_root: str | Path | None = None) -> None:
        super().__init__()
        root = Path(project_root).resolve(strict=False) if project_root else Path.cwd()
        self.context = AppContext(root)
        self._worker: WorkerJobHandle | None = None
        self._dirty = False
        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(200)
        self._poll_timer.timeout.connect(self._poll_worker)
        self.setWindowTitle("HJMB V4.0 三模式路径生成器")
        self.resize(1720, 960)
        self.setMinimumSize(1280, 760)

        self.tabs = QTabWidget()
        self.path_editor_page = PathEditorPage()
        self.optimization_batch_page = OptimizationBatchPage()
        self.tabs.addTab(self.path_editor_page, "路径编辑")
        self.tabs.addTab(self.optimization_batch_page, "最优路段与批量生成")
        self.setCentralWidget(self.tabs)

        self.mode_combo = QComboBox()
        self.mode_combo.addItem("手动模式", GenerationMode.MANUAL.value)
        self.mode_combo.addItem("半自动模式", GenerationMode.SEMI_AUTO.value)
        self.mode_combo.addItem("全自动模式", GenerationMode.FULL_AUTO.value)
        self.traj_spin = QSpinBox()
        self.traj_spin.setRange(0, 359)
        self.project_label = QLabel(str(root))
        self.worker_progress = QProgressBar()
        self.worker_progress.setRange(0, 100)
        self.worker_progress.setValue(0)
        self.cancel_button = QPushButton("停止")
        self.cancel_button.setEnabled(False)
        self.cancel_button.clicked.connect(self._cancel_worker)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(2000)
        self._build_toolbar()
        self._build_menu()
        self._build_docks_and_status()
        self._connect_pages()
        if project_root is not None:
            self.load_project_path(project_root)

    def _build_toolbar(self) -> None:
        toolbar = QToolBar("V4 主工具栏")
        toolbar.setMovable(False)
        actions = (
            ("新建V4项目", self.new_project),
            ("打开V4项目", self.choose_project),
            ("保存项目配置", self.save_project_config),
            ("打开Case", self.open_case),
            ("保存当前Case JSON", self.save_current_case),
        )
        for text, callback in actions:
            button = QPushButton(text)
            button.clicked.connect(callback)
            toolbar.addWidget(button)
        toolbar.addSeparator()
        toolbar.addWidget(QLabel("模式"))
        toolbar.addWidget(self.mode_combo)
        toolbar.addWidget(QLabel("traj_id"))
        toolbar.addWidget(self.traj_spin)
        for text, callback in (
            ("生成/更新当前路径", self.generate_current),
            ("验证当前路径", self.validate_current),
            ("导出当前BIN", self.export_current_bin),
            ("设为最终版本", self.export_final),
            ("撤销", self.path_editor_page.undo_stack.undo),
            ("重做", self.path_editor_page.undo_stack.redo),
        ):
            button = QPushButton(text)
            button.clicked.connect(callback)
            toolbar.addWidget(button)
        toolbar.addWidget(self.cancel_button)
        self.addToolBar(toolbar)

    def _build_menu(self) -> None:
        project_menu = self.menuBar().addMenu("项目")
        import_v35 = QAction("导入旧V3.5工程", self)
        import_v35.triggered.connect(self.import_v35_project)
        migrate_layout = QAction("迁移旧V4输出目录", self)
        migrate_layout.triggered.connect(self.migrate_layout)
        project_menu.addAction(import_v35)
        project_menu.addAction(migrate_layout)
        view_menu = self.menuBar().addMenu("视图")
        fit = QAction("适应场地", self)
        fit.triggered.connect(self.path_editor_page.field_view.fit_to_field)
        view_menu.addAction(fit)

    def _build_docks_and_status(self) -> None:
        log_dock = QDockWidget("日志 / Worker", self)
        log_dock.setWidget(self.log)
        self.addDockWidget(Qt.BottomDockWidgetArea, log_dock)
        log_dock.hide()
        self.log_dock = log_dock
        status = QStatusBar()
        self.setStatusBar(status)
        self.dirty_status = QLabel("clean")
        self.mode_status = QLabel("MANUAL")
        self.worker_status = QLabel("worker: IDLE")
        self.coord_status = QLabel("x=— y=—")
        self.zoom_status = QLabel("zoom=—")
        for widget in (self.dirty_status, self.mode_status, self.worker_status, self.worker_progress, self.coord_status, self.zoom_status):
            status.addPermanentWidget(widget)

    def _connect_pages(self) -> None:
        self.mode_combo.currentIndexChanged.connect(self._mode_changed)
        self.traj_spin.valueChanged.connect(self._traj_changed)
        self.path_editor_page.dirtyChanged.connect(self._mark_dirty)
        self.path_editor_page.statusMessage.connect(self._status)
        self.path_editor_page.conversionRequested.connect(self.convert_to_semi_auto)
        self.path_editor_page.generationRequested.connect(lambda mode, traj: self._start_mode_generation(GenerationMode(mode), traj))
        self.optimization_batch_page.workerRequested.connect(self._page_worker_request)
        self.optimization_batch_page.clearLegRequested.connect(self.clear_leg)
        self.optimization_batch_page.statusMessage.connect(self._status)
        view = self.path_editor_page.field_view
        view.worldMouseMoved.connect(lambda x, y: self.coord_status.setText(f"x={x:.0f} y={y:.0f}"))
        view.zoomChanged.connect(lambda zoom: self.zoom_status.setText(f"zoom={zoom:.2f}"))

    def load_project_path(self, root: str | Path) -> bool:
        try:
            state = self.context.load(root)
        except Exception as exc:  # noqa: BLE001
            QMessageBox.warning(self, "打开项目失败", str(exc))
            self._status(f"打开项目失败: {exc}")
            return False
        self.project_label.setText(str(state.layout.root))
        self._dirty = False
        self.path_editor_page.set_state(state)
        self.optimization_batch_page.set_state(state)
        self._sync_mode_and_traj()
        for warning in state.warnings:
            self._status(f"警告: {warning}")
        return True

    def new_project(self) -> None:
        path = QFileDialog.getExistingDirectory(self, "新建 V4 项目目录", str(self.context.project_root))
        if not path:
            return
        if self.context.state is None:
            QMessageBox.information(self, "新建项目", "请先打开一个 V4 项目作为公共配置模板。")
            return
        target = Path(path)
        if any(target.iterdir()):
            QMessageBox.warning(self, "新建项目", "目标目录必须为空。")
            return
        layout = ProjectLayout.create(target, self.context.state.project)
        source_csv = self.context.state.layout.traj_id_csv
        if source_csv.exists():
            shutil.copy2(source_csv, layout.traj_id_csv)
        self.load_project_path(target)

    def choose_project(self) -> None:
        path = QFileDialog.getExistingDirectory(self, "打开 V4 项目", str(self.context.project_root))
        if path:
            self.load_project_path(path)

    def save_project_config(self) -> None:
        if self.context.state is None:
            return
        save_project(self.context.state.layout.project_json, self.context.state.project)
        self._status("project.json 已原子保存；未自动规划")

    def open_case(self) -> None:
        if self.context.state is None:
            return
        path, _ = QFileDialog.getOpenFileName(self, "打开 V4 Case", str(self.context.state.layout.cases_dir), "JSON (*.json)")
        if not path:
            return
        try:
            case = load_case(path, enforce_filename=False)
        except Exception as exc:  # noqa: BLE001
            QMessageBox.warning(self, "打开 Case 失败", str(exc))
            return
        cases = {
            GenerationMode.MANUAL: self.context.state.manual_cases,
            GenerationMode.SEMI_AUTO: self.context.state.semi_auto_cases,
            GenerationMode.FULL_AUTO: self.context.state.full_auto_cases,
        }[case.generation_mode]
        cases[case.traj_id] = case
        self._set_mode_combo(case.generation_mode)
        self.traj_spin.setValue(case.traj_id)
        self._sync_mode_and_traj()

    def save_current_case(self) -> None:
        if self.context.state is None:
            return
        try:
            case = self.path_editor_page.case_for_save()
            path = self.context.state.layout.case_json_path_for_mode(case.traj_id, case.generation_mode)
            save_case(path, case)
        except Exception as exc:  # noqa: BLE001
            QMessageBox.warning(self, "保存 Case 失败", str(exc))
            return
        cases = {
            GenerationMode.MANUAL: self.context.state.manual_cases,
            GenerationMode.SEMI_AUTO: self.context.state.semi_auto_cases,
            GenerationMode.FULL_AUTO: self.context.state.full_auto_cases,
        }[case.generation_mode]
        cases[case.traj_id] = case
        self.path_editor_page.mark_saved(case)
        self._dirty = False
        self.dirty_status.setText("clean / STALE" if case.review.get("state") == "STALE" else "clean")
        self.optimization_batch_page.refresh()

    def generate_current(self) -> None:
        mode = self.current_mode()
        if mode != GenerationMode.FULL_AUTO:
            self.save_current_case()
            if self._dirty:
                return
        self._start_mode_generation(mode, self.traj_spin.value())

    def validate_current(self) -> None:
        self._start_job("validate-current", {"traj_id": self.traj_spin.value(), "generation_mode": self.current_mode().value})

    def export_current_bin(self) -> None:
        self.generate_current()

    def export_final(self) -> None:
        if self.context.state is None:
            return
        mode = self.current_mode()
        traj_id = self.traj_spin.value()
        case = self.context.state.current_case(traj_id, mode)
        if case is None:
            QMessageBox.warning(self, "设为最终版本", "当前模式没有可发布的 Case。")
            return
        if str(case.review.get("state", "STALE")) == "STALE":
            QMessageBox.warning(self, "设为最终版本", "Case 为 STALE，请先生成并验证。")
            return
        if not case.review.get("approved", False):
            if QMessageBox.question(
                self,
                "审批并发布",
                "设为最终版本会将当前已验证 Case 标记为 approved。确认审批并继续？",
            ) != QMessageBox.Yes:
                return
        target = self.context.state.layout.final_bin_path(self.traj_spin.value())
        if target.exists() and QMessageBox.question(
            self,
            "覆盖最终版本",
            f"{target.name} 已存在。完整验证通过后是否覆盖？",
        ) != QMessageBox.Yes:
            return
        self._start_job(
            "export-final",
            {"traj_id": traj_id, "generation_mode": mode.value, "approve": True},
        )

    def convert_to_semi_auto(self, traj_id: int) -> None:
        if self.context.state is None:
            return
        if QMessageBox.question(self, "转为半自动", "全自动结果不能直接修改，是否转为半自动副本？") != QMessageBox.Yes:
            return
        try:
            converted = convert_full_auto_to_semi_auto(self.context.state.layout, traj_id)
        except Exception as exc:  # noqa: BLE001
            QMessageBox.warning(self, "转换失败", str(exc))
            return
        self.context.state.semi_auto_cases[traj_id] = converted
        self._set_mode_combo(GenerationMode.SEMI_AUTO)
        self._sync_mode_and_traj()

    def clear_leg(self, leg_id: str) -> None:
        if self.context.state is None or self.context.state.leg_library is None:
            return
        leg = next((item for item in self.context.state.leg_library.legs if item.leg_id == leg_id), None)
        if leg is None:
            return
        refs = sum(
            str(ref.get("leg_id")) == leg_id
            for cases in (self.context.state.semi_auto_cases, self.context.state.full_auto_cases)
            for case in cases.values()
            for ref in case.leg_refs
        )
        guarded = leg.review.get("approved") or leg.review.get("locked") or leg.state.value in {"APPROVED", "LOCKED"}
        text = f"清除 {leg_id} 的最优结果？引用 Case 数={refs}。依赖 Case 将标记 STALE，且不会自动重算。"
        if guarded:
            text += "\n该 leg 已 approved/locked，需要加强确认。"
        if QMessageBox.question(self, "清除最优路段", text) != QMessageBox.Yes:
            return
        try:
            clear_optimized_leg_result(self.context.state.layout, leg_id, confirm_leg_id=leg_id if guarded else None)
        except Exception as exc:  # noqa: BLE001
            QMessageBox.warning(self, "清除失败", str(exc))
            return
        self._reload_project()
        if QMessageBox.question(
            self,
            "立即优化",
            "路段已清除，依赖 Case 已标记 STALE。是否立即重新优化该 leg？",
        ) == QMessageBox.Yes:
            self._start_job(
                "reoptimize-current-leg",
                {"leg_id": leg_id, "generation_mode": self.current_mode().value},
            )

    def import_v35_project(self) -> None:
        from hjmb_pathgen.py_legacy.v35_import.legacy_json_reader import load_v35_project

        if self.context.state is None:
            QMessageBox.warning(self, "导入 V3.5", "请先打开目标 V4 项目。")
            return
        path, _ = QFileDialog.getOpenFileName(self, "导入旧 V3.5 工程", "", "JSON (*.json)")
        if not path:
            return
        choice = QMessageBox.question(self, "迁移目标", "选择 Yes 转为 MANUAL；选择 No 转为 SEMI_AUTO。", QMessageBox.Yes | QMessageBox.No | QMessageBox.Cancel)
        if choice == QMessageBox.Cancel:
            return
        try:
            source = load_v35_project(path)
            result = migrate_v35_to_manual(source) if choice == QMessageBox.Yes else migrate_v35_to_semi_auto(source)
            target = self.context.state.layout.case_json_path_for_mode(result.case.traj_id, result.case.generation_mode)
            if target.exists():
                raise FileExistsError(f"目标已存在，拒绝覆盖: {target}")
            save_case(target, result.case)
            report_path = self.context.state.layout.reports_dir / f"v35_migration_P{result.case.traj_id:04d}.json"
            write_json_report(
                report_path,
                {
                    "format": "HJMB_V35_CASE_MIGRATION_REPORT",
                    "source": str(path),
                    "target": str(target),
                    "generation_mode": result.case.generation_mode.value,
                    "migrated_action_count": result.migrated_action_count,
                    "warnings": list(result.warnings),
                    "unsupported_actions": list(result.unsupported_actions),
                },
            )
        except Exception as exc:  # noqa: BLE001
            QMessageBox.warning(self, "V3.5 迁移失败", str(exc))
            return
        QMessageBox.information(
            self,
            "V3.5 迁移",
            f"已迁移 {result.migrated_action_count} 个动作；警告 {len(result.warnings)} 条；"
            f"不支持动作 {len(result.unsupported_actions)} 个。报告：{report_path}。Case 保持 STALE，需人工审核。",
        )
        self._reload_project()

    def migrate_layout(self) -> None:
        if self.context.state is None:
            return
        preview = migrate_old_v40_layout(self.context.state.layout, dry_run=True)
        if preview.conflict_count or preview.unresolved_count:
            QMessageBox.warning(self, "旧目录迁移", f"预览发现 conflict={preview.conflict_count}, unresolved={preview.unresolved_count}；详见报告，未修改文件。")
            return
        if QMessageBox.question(self, "旧目录迁移", f"将迁移 {len(preview.items)} 个文件且不覆盖目标，是否继续？") != QMessageBox.Yes:
            return
        migrate_old_v40_layout(self.context.state.layout, dry_run=False)
        self._reload_project()

    def current_mode(self) -> GenerationMode:
        return GenerationMode(str(self.mode_combo.currentData()))

    def scene_dumps(self) -> dict[str, dict[str, Any]]:
        return {"path_editor": self.path_editor_page.field_view.scene_dump()}

    def _mode_changed(self) -> None:
        if self._dirty and QMessageBox.question(self, "切换模式", "当前修改未保存，确定切换模式？") != QMessageBox.Yes:
            self._set_mode_combo(self.context.generation_mode)
            return
        self.context.generation_mode = self.current_mode()
        self._dirty = False
        self._sync_mode_and_traj()

    def _traj_changed(self, value: int) -> None:
        self.context.traj_id = value
        self._dirty = False
        self._sync_mode_and_traj()

    def _set_mode_combo(self, mode: GenerationMode) -> None:
        index = self.mode_combo.findData(mode.value)
        self.mode_combo.blockSignals(True)
        self.mode_combo.setCurrentIndex(index)
        self.mode_combo.blockSignals(False)
        self.context.generation_mode = mode

    def _sync_mode_and_traj(self) -> None:
        mode = self.context.generation_mode = self.current_mode()
        traj_id = self.context.traj_id = self.traj_spin.value()
        self.path_editor_page.set_mode_and_traj(mode, traj_id)
        self.optimization_batch_page.set_mode_and_traj(mode, traj_id)
        self.mode_status.setText(mode.value)
        self.dirty_status.setText("clean")

    def _mark_dirty(self, dirty: bool, reason: str) -> None:
        self._dirty = self._dirty or dirty
        self.dirty_status.setText("dirty / STALE" if self._dirty else "clean / STALE")
        self._status(reason)

    def _start_mode_generation(self, mode: GenerationMode, traj_id: int) -> None:
        jobs = {GenerationMode.MANUAL: "generate-manual", GenerationMode.SEMI_AUTO: "generate-semi-auto", GenerationMode.FULL_AUTO: "generate-full-auto-one"}
        profile = "default" if mode == GenerationMode.MANUAL else "STANDARD"
        self._start_job(jobs[mode], {"traj_id": traj_id, "generation_mode": mode.value, "profile": profile})

    def _page_worker_request(self, job: str, params: dict[str, Any]) -> None:
        if job == "cancel":
            self._cancel_worker()
        else:
            self._start_job(job, params)

    def _start_job(self, job: str, params: dict[str, Any]) -> None:
        if self.context.state is None:
            QMessageBox.warning(self, "未加载项目", "请先打开 V4 项目。")
            return
        if self._worker is not None and self._worker.is_alive():
            QMessageBox.warning(self, "任务繁忙", "已有 worker 正在运行。")
            return
        self._worker = start_worker_job(self.context.state.layout.root, job, params)
        self.worker_status.setText(f"worker: {job}")
        self.cancel_button.setEnabled(True)
        self._poll_timer.start()
        self._status(f"启动 {job}: {params}")

    def _cancel_worker(self) -> None:
        if self._worker is not None and self._worker.is_alive():
            self._worker.cancel()
            self.worker_status.setText("worker: CANCEL REQUESTED")

    def _poll_worker(self) -> None:
        if self._worker is None:
            return
        for message in self._worker.poll():
            self.log.appendPlainText(f"{message.kind}: {message.payload}")
            if message.kind == "progress":
                payload = dict(message.payload)
                percent = int(payload.get("percent", self.worker_progress.value()))
                completed = int(payload.get("completed_count", 0))
                total = int(payload.get("total_count", 0))
                elapsed = int(payload.get("elapsed_ms", 0))
                if completed > 0 and total >= completed:
                    payload["eta_ms"] = round(elapsed * (total - completed) / completed)
                self.worker_progress.setValue(percent)
                self.optimization_batch_page.set_progress(percent, str(payload.get("message", "")), payload)
            if message.kind == "error":
                self.log_dock.show()
        if not self._worker.is_alive():
            exit_code = self._worker.join(0)
            self._worker = None
            self._poll_timer.stop()
            self.cancel_button.setEnabled(False)
            self.worker_status.setText(f"worker: DONE ({exit_code})")
            self._reload_project()

    def _reload_project(self) -> None:
        if self.context.state is not None:
            self.load_project_path(self.context.state.layout.root)

    def _status(self, text: str) -> None:
        self.log.appendPlainText(text)
        self.statusBar().showMessage(text, 5000)


def main() -> int:
    app = QApplication.instance() or QApplication([])
    window = V4MainWindow()
    window.show()
    return app.exec()

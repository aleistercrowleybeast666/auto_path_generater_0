"""V4 launcher shell that keeps the proven legacy field editor UI."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (
    QFileDialog,
    QComboBox,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from hjmb_pathgen.models.enums import PathSource
from hjmb_pathgen.models.route_case import CaseManifestV40
from hjmb_pathgen.services.mode_output_service import write_manual_free_outputs
from hjmb_pathgen.services.project_service import ProjectLayout
from hjmb_pathgen.services.worker_process import WorkerJobHandle, start_worker_job

from hjmb_pathgen.legacy.v35 import editor as legacy


class LegacyV4MainWindow(legacy.MainWindow):
    """Old field/path editor UI with explicit V4 project actions added as tabs."""

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("HJMB V4.0 路径生成器 - 旧版场地编辑界面")
        self._v4_worker: WorkerJobHandle | None = None
        self._v4_poll_timer = QTimer(self)
        self._v4_poll_timer.setInterval(200)
        self._v4_poll_timer.timeout.connect(self._v4_poll_worker)
        self._v4_project_root = Path.cwd()

        self.v4_project_edit = QLineEdit(str(self._v4_project_root))
        self.v4_status_label = QLabel("未检查")
        self.v4_status_label.setWordWrap(True)
        self.v4_project_log = QPlainTextEdit()
        self.v4_project_log.setReadOnly(True)
        self.v4_project_log.setMaximumBlockCount(1200)
        self.v4_batch_log = QPlainTextEdit()
        self.v4_batch_log.setReadOnly(True)
        self.v4_batch_log.setMaximumBlockCount(1200)
        self.v4_progress = QProgressBar()
        self.v4_progress.setRange(0, 0)
        self.v4_progress.hide()
        self.v4_traj_spin = QSpinBox()
        self.v4_traj_spin.setRange(0, 359)
        self.v4_final_source = QComboBox()
        self.v4_final_source.addItem("任务编译 TASK_COMPILED", PathSource.TASK_COMPILED.value)
        self.v4_final_source.addItem("手动自由 MANUAL_FREE", PathSource.MANUAL_FREE.value)
        self.v4_profile_combo = QComboBox()
        for profile in ("default", "quick", "standard", "final"):
            self.v4_profile_combo.addItem(profile, profile)

        self._add_v4_tabs_to_legacy_panel()

    def _add_v4_tabs_to_legacy_panel(self) -> None:
        if not hasattr(self, "right_tabs"):
            return
        self.right_tabs.addTab(self._build_v4_project_tab(), "V4 项目/手动输出")
        self.right_tabs.addTab(self._build_v4_batch_tab(), "V4 生成/360")

    def export_bin(self) -> None:
        self._v4_write_manual_free()

    def validate_current_project(self) -> None:
        try:
            layout = self._v4_layout()
            case = self._v4_case_from_legacy_points()
            result = write_manual_free_outputs(
                layout,
                case,
                profile_name=str(self.v4_profile_combo.currentData()),
                write_case_json=False,
                write_bin=False,
                write_report=False,
                dry_run=True,
            )
        except Exception as exc:  # noqa: BLE001
            QMessageBox.warning(self, "V4 校验失败", str(exc))
            self._v4_append(f"V4 校验失败: {exc}")
            return
        self._v4_append(
            f"V4 校验通过 P{result.traj_id:04d}: byte_size={result.byte_size}, CRC={result.hashes.get('bin_crc32')}"
        )

    def open_bin(self) -> None:
        QMessageBox.information(
            self,
            "V4 BIN 回读",
            "旧版 V3.5 BIN 回读入口已禁用。V4.0 BIN 请使用 V4 生成/360 页的验证按钮或 path_codec_cli.py check。",
        )

    def _build_v4_project_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)

        path_row = QHBoxLayout()
        path_row.addWidget(QLabel("V4 项目目录:"))
        path_row.addWidget(self.v4_project_edit, 1)
        browse = QPushButton("选择")
        browse.clicked.connect(self._v4_choose_project)
        check = QPushButton("检查状态")
        check.clicked.connect(self._v4_show_status)
        path_row.addWidget(browse)
        path_row.addWidget(check)
        layout.addLayout(path_row)

        layout.addWidget(self.v4_status_label)

        manual_group = QWidget()
        manual_form = QFormLayout(manual_group)
        manual_form.addRow("traj_id", self.v4_traj_spin)
        manual_form.addRow("速度参数 profile", self.v4_profile_combo)
        layout.addWidget(manual_group)

        row = QHBoxLayout()
        preview = QPushButton("检查当前点能否转 V4")
        preview.clicked.connect(self._v4_preview_manual_case)
        write_manual = QPushButton("生成 MANUAL_FREE JSON/BIN")
        write_manual.clicked.connect(self._v4_write_manual_free)
        row.addWidget(preview)
        row.addWidget(write_manual)
        layout.addLayout(row)

        hint = QLabel(
            "左侧场地和路径点表保持旧版逻辑：自由模式双击加点、末尾追加、插入、删除、上移、下移都按原 UI 使用。"
            "本页只在点击按钮时把当前点位显式转换为 V4.0 MANUAL_FREE 输出，不会自动规划。"
        )
        hint.setWordWrap(True)
        layout.addWidget(hint)
        layout.addWidget(self.v4_project_log, 1)
        return tab

    def _build_v4_batch_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)

        form = QFormLayout()
        form.addRow("traj_id", self.v4_traj_spin)
        form.addRow("final 来源", self.v4_final_source)
        layout.addLayout(form)

        row1 = QHBoxLayout()
        for text, job, params in (
            ("生成当前 Case", "generate-one", lambda: {"traj_id": self.v4_traj_spin.value()}),
            ("优化缺失 Leg", "optimize-missing-legs", lambda: {"profile": "STANDARD"}),
            ("生成全部 360", "generate-all", lambda: {}),
            ("验证全部", "validate-all", lambda: {}),
        ):
            button = QPushButton(text)
            button.clicked.connect(lambda _checked=False, j=job, p=params: self._v4_start_job(j, p()))
            row1.addWidget(button)
        layout.addLayout(row1)

        row2 = QHBoxLayout()
        export_final = QPushButton("导出 final BIN")
        export_final.clicked.connect(
            lambda: self._v4_start_job(
                "export-final",
                {
                    "traj_id": self.v4_traj_spin.value(),
                    "path_source": str(self.v4_final_source.currentData()),
                    "profile": str(self.v4_profile_combo.currentData()),
                },
            )
        )
        cancel = QPushButton("取消 V4 任务")
        cancel.clicked.connect(self._v4_cancel_worker)
        row2.addWidget(export_final)
        row2.addWidget(cancel)
        row2.addWidget(self.v4_progress)
        layout.addLayout(row2)

        hint = QLabel(
            "这些按钮调用 V4.0 底层 worker：任务编译、Leg 库、360 生成和 final 导出都走新协议；"
            "编辑旧 UI 点位不会自动触发这些耗时操作。"
        )
        hint.setWordWrap(True)
        layout.addWidget(hint)
        layout.addWidget(self.v4_batch_log, 1)
        return tab

    def _v4_layout(self) -> ProjectLayout:
        return ProjectLayout.open(self.v4_project_edit.text())

    def _v4_choose_project(self) -> None:
        path = QFileDialog.getExistingDirectory(self, "选择 V4 项目目录", self.v4_project_edit.text())
        if path:
            self.v4_project_edit.setText(path)
            self._v4_project_root = Path(path)
            self._v4_show_status()

    def _v4_show_status(self) -> None:
        try:
            report = self._v4_layout().status()
        except Exception as exc:  # noqa: BLE001 - UI boundary.
            self.v4_status_label.setText(f"INVALID: {exc}")
            self._v4_append(f"项目状态检查失败: {exc}")
            return
        text = f"{report.status.value}: " + ("; ".join(report.reasons) if report.reasons else "OK")
        self.v4_status_label.setText(text)
        self._v4_append(f"项目状态: {text}")

    def _v4_preview_manual_case(self) -> None:
        try:
            case = self._v4_case_from_legacy_points()
        except Exception as exc:  # noqa: BLE001
            QMessageBox.warning(self, "V4 手动路径检查失败", str(exc))
            self._v4_append(f"手动路径检查失败: {exc}")
            return
        self._v4_append(
            f"P{case.traj_id:04d} MANUAL_FREE 可转换，点数={len(case.manual_path['points']) if case.manual_path else 0}"
        )

    def _v4_write_manual_free(self) -> None:
        try:
            layout = self._v4_layout()
            case = self._v4_case_from_legacy_points()
            result = write_manual_free_outputs(
                layout,
                case,
                profile_name=str(self.v4_profile_combo.currentData()),
            )
        except Exception as exc:  # noqa: BLE001
            QMessageBox.warning(self, "生成 MANUAL_FREE 失败", str(exc))
            self._v4_append(f"生成 MANUAL_FREE 失败: {exc}")
            return
        self._v4_append(
            f"已生成 P{result.traj_id:04d}: case={result.case_path}, bin={result.bin_path}, CRC={result.hashes.get('bin_crc32')}"
        )

    def _v4_case_from_legacy_points(self) -> CaseManifestV40:
        points = legacy.display_edit_points(self.project)
        if len(points) < 2:
            raise ValueError("至少需要 START 和 ARRIVAL 两个点")
        if points[0].type != legacy.POINT_TYPE_START:
            raise ValueError("第 0 个点必须是 START")
        if points[-1].type != legacy.POINT_TYPE_ARRIVAL:
            raise ValueError("最后一个点必须是 ARRIVAL")

        manual_points: list[dict[str, Any]] = []
        for point in points:
            item: dict[str, Any] = {
                "type": point.type,
                "x_mm": int(round(point.x_mm)),
                "y_mm": int(round(point.y_mm)),
            }
            if point.type in (legacy.POINT_TYPE_START, legacy.POINT_TYPE_ARRIVAL):
                yaw = 0 if point.yaw_ddeg == legacy.YAW_UNSPECIFIED_DDEG else int(point.yaw_ddeg)
                item["yaw_ddeg"] = yaw
                item["exact_pass"] = True
            elif point.type == legacy.POINT_TYPE_WAYPOINT:
                item["exact_pass"] = bool(point.exact_pass)
                if int(point.max_speed_mmps) > 0:
                    item["max_speed_mmps"] = int(point.max_speed_mmps)
            else:
                raise ValueError(f"不支持的点类型: {point.type}")
            manual_points.append(item)

        traj_id = int(self.v4_traj_spin.value())
        data = {
            "format": "HJMB_ROUTE_CASE_JSON_V40",
            "storage_mode": "REFERENCED",
            "path_source": "MANUAL_FREE",
            "traj_id": traj_id,
            "bean_code": 0,
            "drop_code": 0,
            "source_mapping": {"manual_from_legacy_ui": True},
            "selected_plan": {
                "route_family": "MANUAL_FREE",
                "vehicle_bin_assignment": {},
                "drop_targets": [],
                "unload_sequence": [],
                "yaw_direction": "SHORTEST",
                "locked_by_user": True,
            },
            "manual_path": {"points": manual_points},
            "arrival_states": [],
            "leg_refs": [],
            "actions": {"source": [], "compiled": []},
            "finish": {"mode": "AT_FINAL_DROP"},
            "estimates": {},
            "hashes": {},
            "review": {
                "detached_from_library": True,
                "manual_override": True,
                "approved": False,
                "override_reason": "legacy UI manual free path",
            },
        }
        return CaseManifestV40.from_dict(data)

    def _v4_start_job(self, job: str, params: dict[str, Any]) -> None:
        if self._v4_worker is not None and self._v4_worker.is_alive():
            QMessageBox.warning(self, "V4 任务繁忙", "已有 V4 worker 正在运行")
            return
        try:
            layout = self._v4_layout()
        except Exception as exc:  # noqa: BLE001
            QMessageBox.warning(self, "V4 项目无效", str(exc))
            return
        self._v4_worker = start_worker_job(layout.root, job, params)
        self.v4_progress.show()
        self._v4_poll_timer.start()
        self._v4_append(f"启动 V4 任务: {job} {params}")

    def _v4_cancel_worker(self) -> None:
        if self._v4_worker is None or not self._v4_worker.is_alive():
            self.v4_progress.hide()
            return
        self._v4_worker.cancel()
        self._v4_append("已请求取消 V4 worker")

    def _v4_poll_worker(self) -> None:
        if self._v4_worker is None:
            self._v4_worker_idle()
            return
        for message in self._v4_worker.poll():
            self._v4_append(f"{message.kind}: {message.payload}")
        if not self._v4_worker.is_alive():
            exitcode = self._v4_worker.join(0)
            self._v4_append(f"V4 worker 结束，exitcode={exitcode}")
            self._v4_worker = None
            self._v4_worker_idle()

    def _v4_worker_idle(self) -> None:
        self._v4_poll_timer.stop()
        self.v4_progress.hide()

    def _v4_append(self, text: str) -> None:
        self.v4_project_log.appendPlainText(text)
        self.v4_batch_log.appendPlainText(text)
        self.update_status(text)


def main() -> int:
    app = legacy.QApplication.instance() or legacy.QApplication([])
    window = LegacyV4MainWindow()
    window.show()
    return app.exec()

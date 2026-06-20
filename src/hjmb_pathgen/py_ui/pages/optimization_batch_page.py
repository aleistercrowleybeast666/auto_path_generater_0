"""Best-leg inspection and batch generation page."""

from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QProgressBar,
    QPushButton,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from hjmb_pathgen.py_domain.enums import GenerationMode

from ..ui_state import LoadedProjectState


class OptimizationBatchPage(QWidget):
    workerRequested = Signal(str, dict)
    clearLegRequested = Signal(str)
    statusMessage = Signal(str)

    def __init__(self) -> None:
        super().__init__()
        self.state: LoadedProjectState | None = None
        self.mode = GenerationMode.MANUAL
        self.traj_id = 0
        self.summary_labels = {key: QLabel("—") for key in ("mode", "traj_id", "state", "candidate", "time", "validation", "outputs")}
        self.leg_table = QTableWidget(0, 9)
        self.leg_table.setHorizontalHeaderLabels(("leg_id", "from", "to", "state", "time_ms", "clearance", "max_rpm", "refs", "review"))
        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        self.progress_label = QLabel("IDLE")
        self.leg_buttons: list[QPushButton] = []
        self._build_ui()

    def _build_ui(self) -> None:
        summary = QWidget()
        form = QFormLayout(summary)
        labels = {"mode": "mode", "traj_id": "traj_id", "state": "state", "candidate": "candidate", "time": "total time", "validation": "validation", "outputs": "output paths"}
        for key, label in labels.items():
            form.addRow(label, self.summary_labels[key])

        controls = QWidget()
        controls_layout = QVBoxLayout(controls)
        leg_row = QHBoxLayout()
        for text, job in (("优化当前 leg", "optimize-current-leg"), ("重算当前 leg", "reoptimize-current-leg"), ("优化当前 Case 缺失", "generate-semi-auto"), ("优化全部缺失", "optimize-missing-legs")):
            button = QPushButton(text)
            button.clicked.connect(lambda _checked=False, value=job: self._request(value))
            leg_row.addWidget(button)
            self.leg_buttons.append(button)
        clear = QPushButton("清除当前最优 leg")
        clear.clicked.connect(self._clear_selected_leg)
        leg_row.addWidget(clear)
        self.leg_buttons.append(clear)
        controls_layout.addLayout(leg_row)

        validate_row = QHBoxLayout()
        for text, job in (("验证当前", "validate-current"), ("验证全部", "validate-all")):
            button = QPushButton(text)
            button.clicked.connect(lambda _checked=False, value=job: self._request(value))
            validate_row.addWidget(button)
        validate_row.addStretch(1)
        controls_layout.addLayout(validate_row)

        batch_row = QHBoxLayout()
        current = QPushButton("全自动生成当前 ID")
        current.clicked.connect(lambda: self._request("generate-full-auto-one"))
        all_cases = QPushButton("全自动生成全部 360")
        all_cases.clicked.connect(lambda: self._request("generate-full-auto-all"))
        stop = QPushButton("停止")
        stop.clicked.connect(lambda: self.workerRequested.emit("cancel", {}))
        batch_row.addWidget(current)
        batch_row.addWidget(all_cases)
        batch_row.addWidget(stop)
        controls_layout.addLayout(batch_row)
        controls_layout.addWidget(self.progress_label)
        controls_layout.addWidget(self.progress)

        splitter = QSplitter()
        splitter.addWidget(summary)
        splitter.addWidget(controls)
        splitter.setSizes((500, 1000))
        layout = QVBoxLayout(self)
        layout.addWidget(splitter)
        layout.addWidget(QLabel("最优路段库"))
        layout.addWidget(self.leg_table, 1)

    def set_state(self, state: LoadedProjectState | None) -> None:
        self.state = state
        self.refresh()

    def set_mode_and_traj(self, mode: GenerationMode, traj_id: int) -> None:
        self.mode = mode
        self.traj_id = traj_id
        self.refresh()

    def refresh(self) -> None:
        case = self.state.current_case(self.traj_id, self.mode) if self.state else None
        self.summary_labels["mode"].setText(self.mode.value)
        self.summary_labels["traj_id"].setText(str(self.traj_id))
        self.summary_labels["state"].setText(str(case.review.get("state", "MISSING")) if case else "MISSING")
        self.summary_labels["candidate"].setText(str(case.selected_plan.get("candidate_id", "—")) if case else "—")
        self.summary_labels["time"].setText(str(case.estimates.get("planned_total_estimate_ms", "—")) if case else "—")
        self.summary_labels["validation"].setText(str(case.review.get("collision_validation", {}).get("status", "NOT_CHECKED")) if case else "—")
        if self.state:
            layout = self.state.layout
            self.summary_labels["outputs"].setText(
                f"{layout.case_json_path_for_mode(self.traj_id, self.mode)} | {layout.bin_path_for_mode(self.traj_id, self.mode)}"
            )
        else:
            self.summary_labels["outputs"].setText("—")
        enabled = self.mode != GenerationMode.MANUAL
        for button in self.leg_buttons:
            button.setEnabled(enabled)
        self._refresh_legs()

    def set_progress(self, percent: int, text: str, details: dict | None = None) -> None:
        self.progress.setValue(max(0, min(100, percent)))
        details = details or {}
        counters = []
        for key in ("current_item", "reused_count", "optimized_count", "generated_count", "failed_count", "eta_ms"):
            if key in details:
                counters.append(f"{key}={details[key]}")
        self.progress_label.setText(" | ".join((text, *counters)))

    def selected_leg_id(self) -> str | None:
        row = self.leg_table.currentRow()
        if row < 0 or self.leg_table.item(row, 0) is None:
            return None
        return self.leg_table.item(row, 0).text()

    def _refresh_legs(self) -> None:
        legs = list(self.state.leg_library.legs) if self.state and self.state.leg_library else []
        refs: dict[str, int] = {}
        if self.state:
            for cases in (self.state.semi_auto_cases, self.state.full_auto_cases):
                for case in cases.values():
                    for ref in case.leg_refs:
                        leg_id = str(ref.get("leg_id", ""))
                        refs[leg_id] = refs.get(leg_id, 0) + 1
        self.leg_table.setRowCount(len(legs))
        for row, leg in enumerate(legs):
            values = (
                leg.leg_id,
                leg.key.get("from_state_key", leg.key.get("from_state_id", "")),
                leg.key.get("to_state_key", leg.key.get("to_state_id", "")),
                leg.state.value,
                leg.analysis.get("planned_time_ms", ""),
                leg.analysis.get("min_clearance_mm", ""),
                leg.analysis.get("max_wheel_rpm", leg.analysis.get("max_metrics", {}).get("max_wheel_rpm", "")),
                refs.get(leg.leg_id, 0),
                "LOCKED" if leg.review.get("locked") else "APPROVED" if leg.review.get("approved") else "",
            )
            for column, value in enumerate(values):
                self.leg_table.setItem(row, column, QTableWidgetItem(str(value)))

    def _request(self, job: str) -> None:
        if job == "generate-semi-auto" and self.mode == GenerationMode.FULL_AUTO:
            job = "generate-full-auto-one"
        params = {"traj_id": self.traj_id, "generation_mode": self.mode.value}
        if job in {"optimize-current-leg", "reoptimize-current-leg"}:
            leg_id = self.selected_leg_id()
            if leg_id is None:
                self.statusMessage.emit("请先选择 leg")
                return
            params["leg_id"] = leg_id
        self.workerRequested.emit(job, params)

    def _clear_selected_leg(self) -> None:
        leg_id = self.selected_leg_id()
        if leg_id is None:
            self.statusMessage.emit("请先选择 leg")
            return
        self.clearLegRequested.emit(leg_id)

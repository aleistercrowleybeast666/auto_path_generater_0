"""Actions tab showing source and compiled actions."""

from __future__ import annotations

from PySide6.QtWidgets import QLabel, QSplitter, QTableView, QVBoxLayout, QWidget

from hjmb_pathgen.models.enums import PathSource

from ..models.action_table_model import ActionTableModel
from ..ui_state import LoadedProjectState


class ActionsTab(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.source_model = ActionTableModel(source_key="source")
        self.compiled_model = ActionTableModel(source_key="compiled")
        self.source_table = QTableView()
        self.source_table.setModel(self.source_model)
        self.compiled_table = QTableView()
        self.compiled_table.setModel(self.compiled_model)
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("source actions 可编辑语义来自 Case；compiled actions 只读，用于检查 KINEMATIC 和 final binding。"))
        splitter = QSplitter()
        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.addWidget(QLabel("Source"))
        left_layout.addWidget(self.source_table)
        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.addWidget(QLabel("Compiled"))
        right_layout.addWidget(self.compiled_table)
        splitter.addWidget(left)
        splitter.addWidget(right)
        layout.addWidget(splitter, 1)
        layout.addWidget(QLabel("完赛链：最后放货 ARRIVAL → STOP_AND_WAIT DROP_* → DONE → post_wait → FIFO empty → complete。无 SAFE_END/FINISH_CLEAR 设置。"))

    def set_state(self, state: LoadedProjectState | None) -> None:
        case = state.current_case(source=PathSource.TASK_COMPILED) if state is not None else None
        self.source_model.set_case(case)
        self.compiled_model.set_case(case)
        self.source_table.resizeColumnsToContents()
        self.compiled_table.resizeColumnsToContents()

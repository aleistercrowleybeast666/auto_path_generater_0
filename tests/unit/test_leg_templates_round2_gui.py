from __future__ import annotations

import os
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QApplication

from hjmb_pathgen.py_domain.leg_template import (
    LegTemplateState,
    LegTemplateValidationEntryV40,
    LegTemplateValidationReportV40,
)
from hjmb_pathgen.py_io.codecs.json_codec import (
    load_leg_templates,
    load_project,
    save_leg_template_instances,
    save_leg_template_validation_report,
    save_leg_templates,
    save_project,
)
from hjmb_pathgen.py_io.layout.project_layout import ProjectLayout
from hjmb_pathgen.py_services.competition_task_config_service import (
    load_competition_task_config,
    save_competition_task_config,
)
from hjmb_pathgen.py_services.leg_template_service import (
    expand_leg_template_instances,
    sync_leg_templates_for_layout,
    validate_leg_template_for_layout,
)
from hjmb_pathgen.py_ui.pages.leg_template_page import LegTemplatePage
from hjmb_pathgen.py_ui.v35_exact_main_window import V35ExactV4MainWindow
from hjmb_pathgen.py_workers import worker_process


ROOT = Path(__file__).resolve().parents[2]


class _NeverCancelled:
    def is_set(self) -> bool:
        return False


class LegTemplateRound2GuiTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.layout = ProjectLayout.open(self.temp.name, create_dirs=True)
        self.project = load_project(ROOT / "project.json")
        self.config = load_competition_task_config(ROOT / "task_config" / "competition_task_config.json")
        save_project(self.layout.project_json, self.project)
        save_competition_task_config(self.layout.competition_task_config_json, self.config)
        sync_leg_templates_for_layout(self.layout)
        self.page = LegTemplatePage()
        self.page.load_layout(self.layout)

    def tearDown(self) -> None:
        self.page.close()
        self.temp.cleanup()

    def _first_id(self) -> str:
        return str(self.page.template_table.item(0, 1).data(Qt.UserRole))

    def _enable(self, template_id: str) -> None:
        self.page._persist_template_change(template_id, enabled=True)  # noqa: SLF001

    def test_fourth_tab_is_leg_templates_and_original_pages_stay_ordered(self) -> None:
        window = V35ExactV4MainWindow()
        try:
            self.assertEqual(window.right_tabs.indexOf(window.leg_template_page), 3)
            self.assertEqual(
                [window.right_tabs.tabText(index) for index in range(5)],
                ["路径点", "机械动作", "固定8点 / 最优路段 / 批量", "Leg 模板", "规划参数"],
            )
            self.assertEqual(window.leg_template_page.scroll.horizontalScrollBarPolicy(), Qt.ScrollBarAsNeeded)
            self.assertEqual(window.leg_template_page.scroll.verticalScrollBarPolicy(), Qt.ScrollBarAsNeeded)
        finally:
            window.close()

    def test_project_load_shows_sixteen_reachable_slots_and_enable_writes_source_json(self) -> None:
        self.assertEqual(self.page.template_table.rowCount(), 16)
        template_id = self._first_id()
        self._enable(template_id)
        saved = next(item for item in load_leg_templates(self.layout.leg_templates_json).templates if item.template_id == template_id)
        self.assertTrue(saved.enabled)
        self.assertNotEqual(saved.state, LegTemplateState.PASSED)

    def test_waypoint_add_drag_reorder_delete_and_template_switch_keep_draft(self) -> None:
        first_id = self._first_id()
        second_id = str(self.page.template_table.item(1, 1).data(Qt.UserRole))
        self.page.select_template(first_id)
        self.page.add_waypoint(100.0, 200.0)
        self.page.add_waypoint(300.0, 400.0)
        self.page._waypoint_drag_preview(0, 120, 220)  # noqa: SLF001
        self.page._waypoint_drag_committed(SimpleNamespace(key=0, new_x_mm=120, new_y_mm=220))  # noqa: SLF001
        self.page.move_waypoint(0, 1)
        self.assertEqual((self.page.waypoint_draft[1].x_mm, self.page.waypoint_draft[1].y_mm), (120.0, 220.0))
        self.assertEqual(self.page.field_view.scene_dump()["manual_point_count"], 2)
        self.page.delete_waypoint(0)
        self.page.select_template(second_id)
        saved = next(item for item in load_leg_templates(self.layout.leg_templates_json).templates if item.template_id == first_id)
        self.assertEqual([(item.x_mm, item.y_mm) for item in saved.waypoints], [(120.0, 220.0)])
        self.page.select_template(first_id)
        self.assertEqual(len(self.page.waypoint_draft), 1)
        self.assertGreaterEqual(self.page.field_view.scene_dump()["preview_curve_count"], 1)

    def test_validate_button_only_emits_existing_worker_service_job_and_checking_state(self) -> None:
        template_id = self._first_id()
        self._enable(template_id)
        requested = []
        self.page.validationRequested.connect(lambda *args: requested.append(args))
        self.page.validate_template(template_id)
        self.assertEqual(len(requested), 1)
        job, params, token, revision = requested[0]
        self.assertEqual(job, "validate-leg-template")
        self.assertEqual(params["template_id"], template_id)
        self.assertEqual(params["job_token"], token)
        self.assertEqual(params["revision"], revision)
        self.assertIn(template_id, self.page.checking_template_ids)
        status = next(self.page.template_table.item(row, 7).text() for row in range(self.page.template_table.rowCount()) if self.page.template_table.item(row, 1).data(Qt.UserRole) == template_id)
        self.assertEqual(status, "CHECKING")

    def test_worker_dispatch_reuses_round1_validate_one_service(self) -> None:
        template_id = self._first_id()
        self._enable(template_id)
        template = next(item for item in self.page.templates.templates if item.template_id == template_id)
        report = LegTemplateValidationEntryV40(template_id, LegTemplateState.PASSED, {"total": 1, "passed": 1, "failed": 0}, (), (), ())
        fake = SimpleNamespace(template=replace(template, state=LegTemplateState.PASSED), report=report)
        params = {
            "template_id": template_id,
            "template_hash": template.template_hash,
            "dependency_hashes": template.dependency_hashes,
            "job_token": "token",
            "revision": 7,
        }
        with patch.object(worker_process, "validate_leg_template_for_layout", return_value=fake) as validate:
            payload = worker_process._run_job(self.layout, "validate-leg-template", params, _NeverCancelled(), lambda *_args, **_kwargs: None)  # noqa: SLF001
        validate.assert_called_once()
        self.assertEqual(payload["job_token"], "token")
        self.assertEqual(payload["revision"], 7)

    def test_status_refresh_and_old_job_revision_guard(self) -> None:
        template_id = self._first_id()
        self._enable(template_id)
        self.page.select_template(template_id)
        self.page._start_validation("validate-leg-template", {}, {template_id})  # noqa: SLF001
        token = self.page.active_job_token
        revision = self.page.revision
        self.assertFalse(self.page.accept_worker_result({}, token, revision + 1, str(self.layout.root)))
        self.assertIn(template_id, self.page.checking_template_ids)

        document = load_leg_templates(self.layout.leg_templates_json)
        updated = tuple(replace(item, state=LegTemplateState.PARTIAL) if item.template_id == template_id else item for item in document.templates)
        save_leg_templates(self.layout.leg_templates_json, replace(document, templates=updated))
        self.assertTrue(self.page.accept_worker_result({}, token, revision, str(self.layout.root)))
        status = next(self.page.template_table.item(row, 7).text() for row in range(self.page.template_table.rowCount()) if self.page.template_table.item(row, 1).data(Qt.UserRole) == template_id)
        self.assertEqual(status, "PARTIAL")

        self.page._start_validation("validate-leg-template", {}, {template_id})  # noqa: SLF001
        current = self.page.active_job_token
        self.page.accept_worker_failure(current, "cancelled")
        self.assertFalse(self.page.checking_template_ids)

    def test_passed_partial_failed_statuses_and_instance_rows_refresh(self) -> None:
        template_id = self._first_id()
        self._enable(template_id)
        template = next(item for item in self.page.templates.templates if item.template_id == template_id)
        expanded = expand_leg_template_instances(self.project, self.config, template)
        save_leg_template_instances(
            self.layout.leg_template_instances_json,
            replace(self.page.instances, dependency_hashes=self.page.templates.dependency_hashes, instances=expanded.instances),
        )
        for state in (LegTemplateState.PASSED, LegTemplateState.PARTIAL, LegTemplateState.FAILED):
            document = load_leg_templates(self.layout.leg_templates_json)
            save_leg_templates(
                self.layout.leg_templates_json,
                replace(document, templates=tuple(replace(item, state=state) if item.template_id == template_id else item for item in document.templates)),
            )
            self.page.reload_from_project()
            self.page.select_template(template_id)
            status = next(self.page.template_table.item(row, 7).text() for row in range(self.page.template_table.rowCount()) if self.page.template_table.item(row, 1).data(Qt.UserRole) == template_id)
            self.assertEqual(status, state.value)
            self.assertGreaterEqual(self.page.instance_table.rowCount(), 1)

    def test_stale_worker_expected_hash_cannot_overwrite_new_waypoint_file(self) -> None:
        template_id = self._first_id()
        self._enable(template_id)
        old = next(item for item in self.page.templates.templates if item.template_id == template_id)
        self.page.select_template(template_id)
        self.page.add_waypoint(321.0, 123.0)
        self.page.save_current_draft()
        with self.assertRaisesRegex(RuntimeError, "superseded"):
            validate_leg_template_for_layout(
                self.layout,
                template_id,
                expected_template_hash=old.template_hash,
                expected_dependency_hashes=old.dependency_hashes,
            )
        saved = next(item for item in load_leg_templates(self.layout.leg_templates_json).templates if item.template_id == template_id)
        self.assertEqual([(item.x_mm, item.y_mm) for item in saved.waypoints], [(321.0, 123.0)])

    def test_three_json_exports_are_exact_copies_and_do_not_plan(self) -> None:
        template = self.page.templates.templates[0]
        expanded = expand_leg_template_instances(self.project, self.config, template)
        save_leg_template_instances(
            self.layout.leg_template_instances_json,
            replace(self.page.instances, dependency_hashes=self.page.templates.dependency_hashes, instances=expanded.instances),
        )
        entry = LegTemplateValidationEntryV40(
            template.template_id, LegTemplateState.STALE,
            {"total": len(expanded.instances), "passed": 0, "failed": 0, "stale": len(expanded.instances)},
            expanded.missing_profiles, expanded.errors, (),
        )
        save_leg_template_validation_report(
            self.layout.leg_template_validation_report_json,
            LegTemplateValidationReportV40(self.project.project_id, self.page.templates.dependency_hashes, (entry,)),
        )
        self.page.reload_from_project()
        with tempfile.TemporaryDirectory() as output:
            with patch("hjmb_pathgen.py_planning.optimization.leg_optimizer.optimize_leg", side_effect=AssertionError("must not plan")):
                paths = self.page.export_all(output)
            sources = (
                self.layout.leg_templates_json,
                self.layout.leg_template_instances_json,
                self.layout.leg_template_validation_report_json,
            )
            self.assertEqual([path.name for path in paths], ["leg_templates.json", "leg_template_instances.json", "leg_template_validation_report.json"])
            for source, exported in zip(sources, paths):
                self.assertEqual(exported.read_bytes(), source.read_bytes())

    def test_empty_instance_or_report_export_is_rejected_without_fake_pass(self) -> None:
        with tempfile.TemporaryDirectory() as output:
            with self.assertRaisesRegex(RuntimeError, "没有模板实例"):
                self.page.export_document("instances", Path(output) / "instances.json")
            self.assertFalse((Path(output) / "instances.json").exists())

    def test_fixed_pose_change_preserves_waypoint_and_marks_stale(self) -> None:
        template_id = self._first_id()
        self._enable(template_id)
        self.page.select_template(template_id)
        self.page.add_waypoint(111.0, 222.0)
        self.page.save_current_draft()
        sites = {key: dict(value) for key, value in self.project.sites.items()}
        template = next(item for item in self.page.templates.templates if item.template_id == template_id)
        sites[template.from_site]["x_mm"] += 5
        save_project(self.layout.project_json, replace(self.project, sites=sites))
        self.page.sync_from_project()
        changed = next(item for item in self.page.templates.templates if item.template_id == template_id)
        self.assertEqual(changed.waypoints[0].x_mm, 111.0)
        self.assertEqual(changed.state, LegTemplateState.STALE)

    def test_template_operations_do_not_change_traj_id(self) -> None:
        window = V35ExactV4MainWindow()
        try:
            window.traj_id_combo.setEditText("127")
            window._commit_traj_id_selection()  # noqa: SLF001
            window.leg_template_page.load_layout(self.layout)
            window.right_tabs.setCurrentIndex(3)
            window.leg_template_page.sync_from_project()
            with tempfile.TemporaryDirectory() as output:
                window.leg_template_page.export_document("templates", Path(output) / "leg_templates.json")
            self.assertEqual(window.project.traj_id, 127)
            self.assertEqual(window.traj_id_combo.currentText(), "P0127")
        finally:
            window.close()

    def test_switching_tabs_during_validation_keeps_active_job_revision(self) -> None:
        window = V35ExactV4MainWindow()
        try:
            window._v4_state = SimpleNamespace(layout=self.layout)  # noqa: SLF001
            window.leg_template_page.load_layout(self.layout)
            window.leg_template_page.active_job_token = "active-token"
            revision = window.leg_template_page.revision
            window.right_tabs.setCurrentIndex(2)
            window.right_tabs.setCurrentIndex(3)
            self.assertEqual(window.leg_template_page.active_job_token, "active-token")
            self.assertEqual(window.leg_template_page.revision, revision)
        finally:
            window.leg_template_page.active_job_token = ""
            window.close()


if __name__ == "__main__":
    unittest.main()

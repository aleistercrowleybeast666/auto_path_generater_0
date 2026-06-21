from __future__ import annotations

import json
import os
import tempfile
import unittest
import sys
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from hjmb_pathgen.py_domain.leg_optimization import LegOptimizationProfileName
from hjmb_pathgen.py_domain.leg_template import (
    LegTemplateInstanceState,
    LegTemplateInstancesV40,
    LegTemplateState,
    LegTemplateValidationReportV40,
    LegTemplateWaypointV40,
)
from hjmb_pathgen.py_io.codecs.json_codec import (
    dump_leg_template_instances_bytes,
    dump_leg_templates_bytes,
    dump_leg_template_validation_report_bytes,
    load_leg_template_instances,
    load_leg_templates,
    load_leg_template_validation_report,
    load_project,
    parse_leg_template_instances_bytes,
    parse_leg_templates_bytes,
    parse_leg_template_validation_report_bytes,
    save_leg_template_instances,
    save_leg_templates,
    save_leg_template_validation_report,
    save_project,
)
from hjmb_pathgen.py_io.layout.project_layout import ProjectLayout
from hjmb_pathgen.py_planning.geometry.topology_gates import validate_ordered_topology_gates
from hjmb_pathgen.py_domain.topology import TopologyGate
from hjmb_pathgen.py_services.competition_task_config_service import load_competition_task_config
from hjmb_pathgen.py_services.competition_task_config_service import save_competition_task_config
from hjmb_pathgen.py_services.leg_template_service import (
    aggregate_leg_template_state,
    expand_leg_template_instances,
    expected_leg_template_slots,
    leg_template_id,
    leg_template_instance_is_current_and_passed,
    sync_leg_templates,
    sync_leg_templates_for_layout,
    validate_leg_template,
)


ROOT = Path(__file__).resolve().parents[2]


class LegTemplateJsonTests(unittest.TestCase):
    def setUp(self) -> None:
        self.project = load_project(ROOT / "project.json")
        self.config = load_competition_task_config(ROOT / "task_config" / "competition_task_config.json")
        self.templates = sync_leg_templates(self.project, self.config)
        self.instances = LegTemplateInstancesV40.empty(self.project.project_id)
        self.report = LegTemplateValidationReportV40.empty(self.project.project_id)

    def test_three_documents_are_stable_strict_utf8_round_trips(self) -> None:
        cases = (
            (self.templates, dump_leg_templates_bytes, parse_leg_templates_bytes),
            (self.instances, dump_leg_template_instances_bytes, parse_leg_template_instances_bytes),
            (self.report, dump_leg_template_validation_report_bytes, parse_leg_template_validation_report_bytes),
        )
        for model, dump, parse in cases:
            with self.subTest(model=type(model).__name__):
                data = dump(model)
                self.assertFalse(data.startswith(b"\xef\xbb\xbf"))
                self.assertEqual(parse(data), model)
                self.assertEqual(dump(parse(data)), data)
                raw = json.loads(data)
                raw["unknown"] = True
                with self.assertRaises(Exception):
                    parse((json.dumps(raw) + "\n").encode())
                with self.assertRaises(Exception):
                    parse(b"\xef\xbb\xbf" + data)

    def test_three_document_saves_are_atomic_and_write_back_checked(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            triples = (
                (root / "templates.json", self.templates, save_leg_templates, load_leg_templates),
                (root / "instances.json", self.instances, save_leg_template_instances, load_leg_template_instances),
                (root / "report.json", self.report, save_leg_template_validation_report, load_leg_template_validation_report),
            )
            for path, model, save, load in triples:
                with self.subTest(path=path.name):
                    save(path, model)
                    self.assertEqual(load(path), model)
                    original = path.read_bytes()
                    with patch("hjmb_pathgen.py_io.persistence.atomic_writer.os.replace", side_effect=OSError("blocked")):
                        with self.assertRaises(Exception):
                            save(path, model)
                    self.assertEqual(path.read_bytes(), original)
                    self.assertEqual(list(root.glob(f".{path.name}.*.tmp")), [])

    def test_project_layout_exposes_all_template_paths(self) -> None:
        layout = ProjectLayout.open(ROOT)
        self.assertEqual(layout.leg_templates_json.name, "leg_templates.json")
        self.assertEqual(layout.leg_template_instances_json.name, "leg_template_instances.json")
        self.assertEqual(layout.leg_template_validation_report_json.parent.name, "reports")

    def test_first_sync_creates_all_three_documents_without_planning(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            layout = ProjectLayout.open(temp, create_dirs=True)
            save_project(layout.project_json, self.project)
            save_competition_task_config(layout.competition_task_config_json, self.config)
            synchronized = sync_leg_templates_for_layout(layout)
            self.assertEqual(len(synchronized.templates), 16)
            self.assertEqual(load_leg_template_instances(layout.leg_template_instances_json).instances, ())
            self.assertEqual(load_leg_template_validation_report(layout.leg_template_validation_report_json).template_reports, ())


class LegTemplateSynchronizationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.project = load_project(ROOT / "project.json")
        self.config = load_competition_task_config(ROOT / "task_config" / "competition_task_config.json")

    def test_two_routes_produce_only_sixteen_reachable_directional_slots(self) -> None:
        slots = expected_leg_template_slots(self.config)
        first = sync_leg_templates(self.project, self.config)
        second = sync_leg_templates(self.project, self.config, first)
        self.assertEqual(len(slots), 16)
        self.assertEqual(len(first.templates), 16)
        self.assertEqual(len({item.template_id for item in first.templates}), 16)
        self.assertEqual(first, second)
        ids = {item.template_id for item in first.templates}
        self.assertIn(leg_template_id("PICK_1_TO_3", "P_DROP_3", "P_DROP_1"), ids)
        self.assertNotIn(leg_template_id("PICK_1_TO_3", "P_DROP_1", "P_DROP_3"), ids)
        self.assertIn(leg_template_id("PICK_3_TO_1", "P_DROP_1", "P_DROP_3"), ids)
        self.assertNotIn(leg_template_id("PICK_1_TO_3", "P_PICK_3", "P_DROP_1"), ids)
        self.assertNotIn(leg_template_id("PICK_3_TO_1", "P_PICK_1", "P_DROP_3"), ids)
        self.assertIn(leg_template_id("PICK_1_TO_3", "P_PICK_3", "P_DROP_2"), ids)
        self.assertIn(leg_template_id("PICK_3_TO_1", "P_PICK_1", "P_DROP_2"), ids)

    def test_pose_change_preserves_enabled_waypoints_and_marks_stale(self) -> None:
        original = sync_leg_templates(self.project, self.config)
        target = original.templates[0]
        edited = replace(target, enabled=True, state=LegTemplateState.DRAFT,
                         waypoints=(LegTemplateWaypointV40(123.0, 456.0),))
        existing = replace(original, templates=(edited,) + original.templates[1:])
        sites = {key: dict(value) for key, value in self.project.sites.items()}
        sites[edited.from_site]["x_mm"] += 1
        changed_project = replace(self.project, sites=sites)
        synchronized = sync_leg_templates(changed_project, self.config, existing)
        actual = next(item for item in synchronized.templates if item.template_id == edited.template_id)
        self.assertTrue(actual.enabled)
        self.assertEqual(actual.waypoints, edited.waypoints)
        self.assertEqual(actual.state, LegTemplateState.STALE)
        self.assertEqual(actual.last_validated_hash, "")

    def test_removed_slot_is_preserved_as_auditable_orphan(self) -> None:
        original = sync_leg_templates(self.project, self.config)
        orphan = replace(original.templates[0], template_id="LT_REMOVED_SLOT", enabled=True)
        synchronized = sync_leg_templates(self.project, self.config, replace(original, templates=original.templates + (orphan,)))
        saved = next(item for item in synchronized.templates if item.template_id == "LT_REMOVED_SLOT")
        self.assertTrue(saved.orphaned)
        self.assertFalse(saved.enabled)
        self.assertEqual(saved.state, LegTemplateState.DISABLED)
        self.assertTrue(saved.audit_messages)

    def test_drop_profiles_expand_exactly_and_report_unconfigured_profiles(self) -> None:
        templates = sync_leg_templates(self.project, self.config)
        pickup_to_drop = next(item for item in templates.templates
                              if item.route_family.value == "PICK_1_TO_3" and item.from_site == "P_PICK_3" and item.to_site == "P_DROP_2")
        expanded = expand_leg_template_instances(self.project, self.config, pickup_to_drop)
        self.assertEqual(len(expanded.instances), 3)
        self.assertTrue(all(item.to_unload_pose_profile_id for item in expanded.instances))
        moved_sites = {key: dict(value) for key, value in self.project.sites.items()}
        moved_sites["P_DROP_2"]["x_mm"] += 25
        moved_project = replace(self.project, sites=moved_sites)
        moved = expand_leg_template_instances(moved_project, self.config, pickup_to_drop)
        self.assertEqual(
            [item.instance_id for item in expanded.instances],
            [item.instance_id for item in moved.instances],
        )
        drop_to_drop = next(item for item in templates.templates
                            if item.route_family.value == "PICK_1_TO_3" and item.from_site == "P_DROP_3" and item.to_site == "P_DROP_1")
        self.assertEqual(len(expand_leg_template_instances(self.project, self.config, drop_to_drop).instances), 16)
        profiles = {key: dict(value) for key, value in self.project.unload_pose_profiles.items()}
        profiles["DROP_F6_BIN_3"]["configured"] = False
        changed = replace(self.project, unload_pose_profiles=profiles)
        missing = expand_leg_template_instances(changed, self.config, pickup_to_drop)
        self.assertEqual(len(missing.instances), 2)
        self.assertIn("DROP_F6_BIN_3", missing.missing_profiles)

    def test_aggregation_is_passed_partial_or_failed(self) -> None:
        self.assertEqual(aggregate_leg_template_state([LegTemplateInstanceState.PASSED]), LegTemplateState.PASSED)
        self.assertEqual(aggregate_leg_template_state([LegTemplateInstanceState.PASSED, LegTemplateInstanceState.FAILED]), LegTemplateState.PARTIAL)
        self.assertEqual(aggregate_leg_template_state([LegTemplateInstanceState.FAILED]), LegTemplateState.FAILED)
        self.assertEqual(aggregate_leg_template_state([]), LegTemplateState.FAILED)


class LegTemplateStrictValidationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.project = load_project(ROOT / "project.json")
        self.config = load_competition_task_config(ROOT / "task_config" / "competition_task_config.json")

    def _local_template(self, project):
        document = sync_leg_templates(project, self.config)
        item = next(template for template in document.templates
                    if template.route_family.value == "PICK_1_TO_3" and template.from_site == "P_START" and template.to_site == "P_PICK_1")
        return replace(item, enabled=True, state=LegTemplateState.DRAFT)

    def test_strict_service_passes_clear_geometry_and_fails_blocked_geometry(self) -> None:
        clear_objects = {key: value for key, value in self.project.field_objects.items()}
        clear_objects["cylinders"] = [dict(item, enabled=False) for item in clear_objects["cylinders"]]
        clear_objects["pickup_boxes"] = [dict(item, enabled=False) for item in clear_objects["pickup_boxes"]]
        clear_objects["drop_boxes"] = [dict(item, enabled=False) for item in clear_objects["drop_boxes"]]
        clear_project = replace(self.project, field_objects=clear_objects)
        clear_template = self._local_template(clear_project)
        passed = validate_leg_template(clear_project, self.config, clear_template,
                                       profile_name=LegOptimizationProfileName.STANDARD)
        self.assertEqual(passed.template.state, LegTemplateState.PASSED)
        self.assertTrue(passed.instances[0].compiled_leg)
        self.assertTrue(
            leg_template_instance_is_current_and_passed(
                passed.instances[0], passed.template, passed.template.dependency_hashes,
            )
        )
        self.assertFalse(
            leg_template_instance_is_current_and_passed(
                passed.instances[0], passed.template, {**passed.template.dependency_hashes, "planning_hash": "changed"},
            )
        )

        blocked_objects = {key: value for key, value in clear_objects.items()}
        midpoint_x = (self.project.sites["P_START"]["x_mm"] + self.project.sites["P_PICK_1"]["x_mm"]) / 2
        midpoint_y = (self.project.sites["P_START"]["y_mm"] + self.project.sites["P_PICK_1"]["y_mm"]) / 2
        blocked_objects["cylinders"] = [
            dict(clear_objects["cylinders"][0], center_x_mm=midpoint_x, center_y_mm=midpoint_y,
                 radius_mm=2500, configured=True, enabled=True),
            dict(clear_objects["cylinders"][1], enabled=False),
        ]
        blocked_project = replace(self.project, field_objects=blocked_objects)
        failed = validate_leg_template(blocked_project, self.config, self._local_template(blocked_project),
                                       profile_name=LegOptimizationProfileName.STANDARD)
        self.assertEqual(failed.template.state, LegTemplateState.FAILED)
        self.assertTrue(failed.instances[0].failures)

    def test_wrong_virtual_gate_order_fails_without_collision_dependency(self) -> None:
        gates = (
            TopologyGate.from_dict({"gate_id": "LATE", "a": {"x_mm": 8, "y_mm": -5}, "b": {"x_mm": 8, "y_mm": 5}, "direction": "ANY"}),
            TopologyGate.from_dict({"gate_id": "EARLY", "a": {"x_mm": 2, "y_mm": -5}, "b": {"x_mm": 2, "y_mm": 5}, "direction": "ANY"}),
        )
        samples = ({"x_mm": 0, "y_mm": 0}, {"x_mm": 5, "y_mm": 0}, {"x_mm": 10, "y_mm": 0})
        result = validate_ordered_topology_gates(samples, gates)
        self.assertFalse(result.success)

    def test_pure_service_import_does_not_load_pyside6(self) -> None:
        import subprocess
        import sys
        command = [sys.executable, "-c", "import sys; import hjmb_pathgen.py_services.leg_template_service; print(any(x.startswith('PySide6') for x in sys.modules))"]
        environment = dict(os.environ, PYTHONPATH=str(ROOT / "src"))
        completed = subprocess.run(command, cwd=ROOT, env=environment, capture_output=True, text=True, check=True)
        self.assertEqual(completed.stdout.strip(), "False")


if __name__ == "__main__":
    unittest.main()

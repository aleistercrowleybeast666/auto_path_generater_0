from __future__ import annotations

import json
import ast
import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from hjmb_pathgen.py_domain.enums import GenerationMode
from hjmb_pathgen.py_domain.project import ProjectV40
from hjmb_pathgen.py_domain.route_case import CaseManifestV40
from hjmb_pathgen.py_io.codecs.json_codec import load_case, save_case
from hjmb_pathgen.py_io.layout.project_layout import ProjectLayout
from hjmb_pathgen.py_io.migration.old_v40_layout_migration import migrate_old_v40_layout
from hjmb_pathgen.py_io.migration.v35_case_migration import (
    migrate_v35_to_manual,
    migrate_v35_to_semi_auto,
)
from hjmb_pathgen.py_legacy.v35_import.legacy_json_reader import load_v35_project
from hjmb_pathgen.py_services.manual_path_service import plan_manual_case
from hjmb_pathgen.py_services.mode_case_service import convert_full_auto_to_semi_auto

from tests.unit.test_phase4_manual_path import manual_case_dict


ROOT = Path(__file__).resolve().parents[2]
FIXTURES = ROOT / "tests" / "fixtures" / "v40"


class ThreeModeArchitectureTest(unittest.TestCase):
    def test_root_has_only_thin_gui_python_entry_and_no_old_runtime_packages(self):
        self.assertEqual([path.name for path in ROOT.glob("*.py")], ["hjmb_path_editor.py"])
        entry = (ROOT / "hjmb_path_editor.py").read_text(encoding="utf-8")
        self.assertIn("hjmb_pathgen.py_app.gui_main", entry)
        self.assertNotIn("py_legacy", entry)
        for old_name in ("app", "cli", "codec", "collision", "geometry", "legacy", "models", "planning", "services", "ui", "utils"):
            old_dir = ROOT / "src" / "hjmb_pathgen" / old_name
            self.assertEqual(list(old_dir.glob("*.py")), [], old_name)

    def test_py_package_import_graph_has_no_cycles(self):
        package_root = ROOT / "src" / "hjmb_pathgen"
        modules: dict[str, Path] = {}
        for path in package_root.rglob("*.py"):
            relative = path.relative_to(package_root)
            parts = relative.with_suffix("").parts
            name = ".".join(("hjmb_pathgen", *parts[:-1])) if parts[-1] == "__init__" else ".".join(("hjmb_pathgen", *parts))
            modules[name] = path
        graph = {name: set() for name in modules}
        for name, path in modules.items():
            package = name if path.name == "__init__.py" else name.rsplit(".", 1)[0]
            for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"), filename=str(path))):
                targets: list[str] = []
                if isinstance(node, ast.Import):
                    targets.extend(alias.name for alias in node.names)
                elif isinstance(node, ast.ImportFrom):
                    if node.level:
                        target = importlib.util.resolve_name("." * node.level + (node.module or ""), package)
                    else:
                        target = node.module or ""
                    targets.append(target)
                for target in targets:
                    if target in modules and target != name:
                        graph[name].add(target)
        visiting: list[str] = []
        complete: set[str] = set()

        def visit(name: str) -> None:
            if name in visiting:
                cycle = visiting[visiting.index(name):] + [name]
                self.fail("circular import: " + " -> ".join(cycle))
            if name in complete:
                return
            visiting.append(name)
            for target in graph[name]:
                visit(target)
            visiting.pop()
            complete.add(name)

        for name in graph:
            visit(name)

    def test_project_layout_has_three_independent_modes(self):
        project = ProjectV40.from_dict(json.loads((FIXTURES / "minimal_project.json").read_text(encoding="utf-8")))
        with tempfile.TemporaryDirectory() as tmp:
            layout = ProjectLayout.create(Path(tmp) / "project", project)
            for mode, expected in (
                (GenerationMode.MANUAL, "manual"),
                (GenerationMode.SEMI_AUTO, "semi_auto"),
                (GenerationMode.FULL_AUTO, "full_auto"),
            ):
                self.assertEqual(layout.case_json_path_for_mode(7, mode).parent.name, expected)
                self.assertEqual(layout.bin_path_for_mode(7, mode).parent.name, expected)
                self.assertEqual(layout.portable_path_for_mode(7, mode).parent.name, expected)

    def test_same_traj_id_modes_coexist_and_full_to_semi_does_not_overwrite(self):
        project = ProjectV40.from_dict(json.loads((FIXTURES / "minimal_project.json").read_text(encoding="utf-8")))
        full = CaseManifestV40.from_dict(json.loads((FIXTURES / "minimal_case.json").read_text(encoding="utf-8")))
        manual = CaseManifestV40.from_dict(manual_case_dict())
        with tempfile.TemporaryDirectory() as tmp:
            layout = ProjectLayout.create(Path(tmp) / "project", project)
            full_path = layout.case_json_path_for_mode(0, GenerationMode.FULL_AUTO)
            manual_path = layout.case_json_path_for_mode(0, GenerationMode.MANUAL)
            save_case(full_path, full)
            save_case(manual_path, manual)
            before = full_path.read_bytes()
            semi = convert_full_auto_to_semi_auto(layout, 0)
            self.assertEqual(semi.generation_mode, GenerationMode.SEMI_AUTO)
            self.assertEqual(full_path.read_bytes(), before)
            self.assertTrue(manual_path.exists())
            self.assertTrue(layout.case_json_path_for_mode(0, GenerationMode.SEMI_AUTO).exists())

    def test_semi_auto_accepts_only_the_three_auxiliary_point_policies(self):
        data = json.loads((FIXTURES / "minimal_case.json").read_text(encoding="utf-8"))
        data["generation_mode"] = GenerationMode.SEMI_AUTO.value
        data["auxiliary_points"] = [
            {"x_mm": 10, "y_mm": 20, "policy": policy}
            for policy in ("LOCKED_PASS", "INITIAL_GUESS", "OPTIMIZABLE")
        ]
        self.assertEqual(len(CaseManifestV40.from_dict(data).auxiliary_points), 3)
        data["auxiliary_points"][0]["policy"] = "AUTO_MOVE_ANCHOR"
        with self.assertRaisesRegex(Exception, "unsupported semi-auto auxiliary"):
            CaseManifestV40.from_dict(data)

    def test_manual_planning_never_calls_leg_optimizer(self):
        project = ProjectV40.from_dict(json.loads((FIXTURES / "minimal_project.json").read_text(encoding="utf-8")))
        case = CaseManifestV40.from_dict(manual_case_dict())
        with patch(
            "hjmb_pathgen.py_planning.optimization.leg_optimizer.optimize_leg",
            side_effect=AssertionError("MANUAL must not optimize geometry"),
        ):
            result = plan_manual_case(case, project)
        self.assertTrue(result.timing.success)
        self.assertEqual(result.trajectory.nodes[0].x_mm, 0)
        self.assertEqual(result.trajectory.nodes[-1].x_mm, 1000)

    def test_v35_migration_preserves_expressible_actions_and_never_creates_full_auto(self):
        legacy = {
            "format": "HJMB_PATH_EDITOR_JSON_V35",
            "traj_id": 7,
            "points": [
                {"point_id": 0, "type": "START", "site_id": 0, "x_mm": 0, "y_mm": 0, "yaw_ddeg": 0},
                {"point_id": 1, "type": "ARRIVAL", "site_id": 1, "x_mm": 100, "y_mm": 0, "yaw_ddeg": 0},
            ],
            "fixed_sites": [
                {"site_id": index, "site_key": key, "x_mm": index * 100, "y_mm": 0, "yaw_ddeg": 0}
                for index, key in enumerate(("P_START", "P_PICK_1", "P_PICK_2L", "P_PICK_2R", "P_PICK_3", "P_DROP_1", "P_DROP_2", "P_DROP_3"))
            ],
            "actions": [
                {"action_seq": 0, "action": "DROP_1", "mode": "STOP_AND_WAIT", "arrival_point_id": 1, "timeout_ms": 1000, "post_wait_ms": 100}
            ],
        }
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "legacy.json"
            path.write_text(json.dumps(legacy), encoding="utf-8")
            source = load_v35_project(path)
        manual = migrate_v35_to_manual(source)
        semi = migrate_v35_to_semi_auto(source)
        self.assertEqual(manual.case.generation_mode, GenerationMode.MANUAL)
        self.assertEqual(semi.case.generation_mode, GenerationMode.SEMI_AUTO)
        self.assertEqual(manual.migrated_action_count, 1)
        self.assertEqual(semi.migrated_action_count, 1)
        self.assertEqual(len(semi.case.logical_points), 8)

    def test_v35_reader_uses_adjacent_fixed_sites_file_when_project_omits_it(self):
        keys = ("P_START", "P_PICK_1", "P_PICK_2L", "P_PICK_2R", "P_PICK_3", "P_DROP_1", "P_DROP_2", "P_DROP_3")
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "legacy.json").write_text(
                json.dumps({"format": "HJMB_PATH_EDITOR_JSON_V35", "traj_id": 0, "points": [], "actions": []}),
                encoding="utf-8",
            )
            (root / "fixed_sites_v35.json").write_text(
                json.dumps([{"site_id": index, "site_key": key, "x_mm": 0, "y_mm": 0, "yaw_ddeg": 0} for index, key in enumerate(keys)]),
                encoding="utf-8",
            )
            source = load_v35_project(root / "legacy.json")
        self.assertEqual(tuple(item["site_key"] for item in source.fixed_sites), keys)

    def test_flat_layout_migration_is_explicit_and_conflict_safe(self):
        project = ProjectV40.from_dict(json.loads((FIXTURES / "minimal_project.json").read_text(encoding="utf-8")))
        manual = CaseManifestV40.from_dict(manual_case_dict())
        with tempfile.TemporaryDirectory() as tmp:
            layout = ProjectLayout.create(Path(tmp) / "project", project)
            flat = layout.legacy_flat_case_json_path(0)
            save_case(flat, manual)
            preview = migrate_old_v40_layout(layout, dry_run=True)
            self.assertTrue(any(item.status == "PLANNED" for item in preview.items))
            self.assertTrue(flat.exists())
            migrate_old_v40_layout(layout, dry_run=False)
            target = layout.case_json_path_for_mode(0, GenerationMode.MANUAL)
            self.assertTrue(target.exists())
            self.assertFalse(flat.exists())
            self.assertEqual(load_case(target).generation_mode, GenerationMode.MANUAL)


if __name__ == "__main__":
    unittest.main()

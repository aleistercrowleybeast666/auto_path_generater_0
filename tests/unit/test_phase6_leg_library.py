from __future__ import annotations

import json
import sys
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from hjmb_pathgen.codec.json_codec import load_leg_library
from hjmb_pathgen.models.enums import LegState
from hjmb_pathgen.models.leg_optimization import LegOptimizationProfileName, LegOptimizationRequest, Pose2D
from hjmb_pathgen.models.project import ProjectV40
from hjmb_pathgen.planning.leg_optimizer import optimize_leg
from hjmb_pathgen.services.leg_library_service import approve_leg, load_or_create_leg_library, save_leg_library_checked, show_leg, upsert_leg
from hjmb_pathgen.services.leg_stale_service import mark_stale_legs

FIXTURE_ROOT = Path(__file__).resolve().parents[1] / "fixtures" / "v40"


def optimized_leg():
    project = ProjectV40.from_dict(json.loads((FIXTURE_ROOT / "minimal_project.json").read_text(encoding="utf-8")))
    result = optimize_leg(
        LegOptimizationRequest(
            project=project,
            from_state_id="A",
            to_state_id="B",
            from_pose=Pose2D(0, 0, 0),
            to_pose=Pose2D(100, 0, 0),
            route_family="MANUAL_FREE",
            topology_profile="NONE",
            profile_name=LegOptimizationProfileName.STANDARD,
        )
    )
    assert result.leg is not None
    return project, result.leg


class Phase6LegLibraryTest(unittest.TestCase):
    def test_atomic_save_round_trip_and_approved_guard(self):
        project, leg = optimized_leg()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "leg_library.json"
            library = load_or_create_leg_library(path, project)
            library = upsert_leg(library, leg)
            save_leg_library_checked(path, library)
            loaded = load_leg_library(path)
            self.assertEqual(show_leg(loaded, leg.leg_id).hashes["validity_hash"], leg.hashes["validity_hash"])

            approved = approve_leg(loaded, leg.leg_id, notes="checked")
            approved_leg = show_leg(approved, leg.leg_id)
            self.assertEqual(approved_leg.state, LegState.VALID)
            self.assertTrue(approved_leg.review["approved"])
            with self.assertRaisesRegex(Exception, "approved/locked"):
                upsert_leg(approved, leg, replace_existing=True)

    def test_stale_marking_uses_dependency_hashes_and_preserves_review_flags(self):
        project, leg = optimized_leg()
        stale_leg = replace(leg, hashes={**leg.hashes, "dependency_hashes": {"planner_config_hash": "old"}})
        library = upsert_leg(load_or_create_leg_library(Path("missing.json"), project), stale_leg)
        marked = mark_stale_legs(library, project)
        self.assertEqual(show_leg(marked, leg.leg_id).state, LegState.STALE)

        approved = approve_leg(library, leg.leg_id)
        marked_approved = mark_stale_legs(approved, project)
        marked_leg = show_leg(marked_approved, leg.leg_id)
        self.assertEqual(marked_leg.state, LegState.STALE)
        self.assertTrue(marked_leg.review["approved"])
        self.assertEqual(marked_leg.review["state"], "STALE")


if __name__ == "__main__":
    unittest.main()

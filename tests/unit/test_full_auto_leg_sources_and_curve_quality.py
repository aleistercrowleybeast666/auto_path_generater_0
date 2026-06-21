from __future__ import annotations

from hjmb_pathgen.py_domain.enums import LegState
from hjmb_pathgen.py_domain.leg import LegV40
from hjmb_pathgen.py_planning.geometry.bezier import BezierPath, Point2D
from hjmb_pathgen.py_services.full_auto_leg_source_service import (
    FullAutoLegSourcePolicy,
    ManualTemplateLeg,
    choose_effective_leg,
)


def _leg(leg_id: str, time_ms: int, *, source: str) -> LegV40:
    return LegV40(
        leg_id=leg_id,
        key={},
        state=LegState.VALID,
        source=source,
        topology_profile="TEST",
        control_points=(),
        yaw_profile={},
        nodes=({}, {}),
        analysis={"planned_time_ms": time_ms},
        hashes={"self_hash32": f"0x{time_ms:08X}"},
        review={"state": "VALID"},
    )


def test_bezier_is_c2_over_chord_parameter_at_unequal_join() -> None:
    path = BezierPath.from_waypoints(
        (Point2D(0, 0), Point2D(80, 20), Point2D(900, 500), Point2D(980, 520))
    )
    chords = (
        ((80.0**2 + 20.0**2) ** 0.5),
        ((820.0**2 + 480.0**2) ** 0.5),
        ((80.0**2 + 20.0**2) ** 0.5),
    )
    left = path.segments[0].derivative(1.0)
    right = path.segments[1].derivative(0.0)
    assert abs(left[0] / chords[0] - right[0] / chords[1]) < 1e-9
    assert abs(left[1] / chords[0] - right[1] / chords[1]) < 1e-9
    left = path.segments[1].derivative(1.0)
    right = path.segments[2].derivative(0.0)
    assert abs(left[0] / chords[1] - right[0] / chords[2]) < 1e-9
    assert abs(left[1] / chords[1] - right[1] / chords[2]) < 1e-9
    assert abs(path.segments[0].curvature(1.0) - path.segments[1].curvature(0.0)) < 1e-9
    assert abs(path.segments[1].curvature(1.0) - path.segments[2].curvature(0.0)) < 1e-9


def test_full_auto_leg_source_policies_are_explicit_and_deterministic() -> None:
    automatic = _leg("LEG_A", 4400, source="PHASE6_OPTIMIZER")
    manual = ManualTemplateLeg(
        leg=_leg("LEG_A", 3900, source="MANUAL_TEMPLATE"),
        template_id="TPL_A",
        instance_id="INST_A",
    )

    auto_only = choose_effective_leg(
        FullAutoLegSourcePolicy.AUTO_ONLY,
        automatic_leg=automatic,
        automatic_reusable=True,
        manual_leg=manual,
        manual_reusable=True,
    )
    assert auto_only.leg is automatic
    assert auto_only.selected_source == "AUTOMATIC"

    manual_only = choose_effective_leg(
        FullAutoLegSourcePolicy.MANUAL_ONLY,
        automatic_leg=automatic,
        automatic_reusable=True,
        manual_leg=manual,
        manual_reusable=True,
    )
    assert manual_only.leg is manual.leg
    assert manual_only.selected_source == "MANUAL_TEMPLATE"

    best = choose_effective_leg(
        FullAutoLegSourcePolicy.BEST_AVAILABLE,
        automatic_leg=automatic,
        automatic_reusable=True,
        manual_leg=manual,
        manual_reusable=True,
    )
    assert best.leg is manual.leg
    assert best.selection_reason == "FASTER_OR_EQUAL_THAN_AUTOMATIC"
    assert best.to_ref_metadata()["template_instance_id"] == "INST_A"


def test_manual_only_never_silently_falls_back_to_automatic() -> None:
    automatic = _leg("LEG_A", 3000, source="PHASE6_OPTIMIZER")
    selected = choose_effective_leg(
        FullAutoLegSourcePolicy.MANUAL_ONLY,
        automatic_leg=automatic,
        automatic_reusable=True,
        manual_leg=None,
        manual_reusable=False,
    )
    assert selected.leg is None
    assert selected.selected_source == "MISSING"
    assert selected.selection_reason == "MANUAL_TEMPLATE_UNAVAILABLE"


def test_validate_full_auto_case_rehydrates_manual_template_library(monkeypatch, tmp_path) -> None:
    from pathlib import Path
    from types import SimpleNamespace

    from hjmb_pathgen.py_io.layout.project_layout import ProjectLayout
    from hjmb_pathgen.py_services import phase7_generation_service as phase7

    project = SimpleNamespace()
    base_library = object()
    effective_library = object()
    case = SimpleNamespace(leg_refs=({"selected_source": "MANUAL_TEMPLATE"},))
    trajectory = SimpleNamespace(
        nodes=(SimpleNamespace(flags=0),),
        segments=(SimpleNamespace(flags=0),),
        actions=(),
        header=SimpleNamespace(planned_motion_time_ms=1234, finish_mode=1),
    )
    seen: dict[str, object] = {}

    monkeypatch.setattr(phase7, "load_project", lambda _path: project)
    monkeypatch.setattr(phase7, "load_leg_library", lambda _path: base_library)
    monkeypatch.setattr(phase7, "_existing_case_path", lambda *_args, **_kwargs: Path("P0000.json"))
    monkeypatch.setattr(phase7, "load_case", lambda _path: case)

    def fake_effective(layout, loaded_project, loaded_library, loaded_case):
        assert loaded_project is project
        assert loaded_library is base_library
        assert loaded_case is case
        return effective_library

    monkeypatch.setattr(phase7, "effective_library_for_case_refs", fake_effective)
    monkeypatch.setattr(phase7, "_case_dependency_failures", lambda _case, _project, library, **_kwargs: [] if library is effective_library else ["wrong library"])

    def fake_compile(request):
        seen["library"] = request.leg_library
        return trajectory

    monkeypatch.setattr(phase7, "compile_case_to_trajectory", fake_compile)
    monkeypatch.setattr(phase7, "encode_trajectory", lambda _trajectory: b"bin")
    monkeypatch.setattr(phase7, "decode_trajectory", lambda _data: trajectory)
    monkeypatch.setattr(phase7, "check_formal_export_guard", lambda *_args, **_kwargs: SimpleNamespace(allowed=True, reasons=()))

    result = phase7.validate_one(ProjectLayout(tmp_path), 0)
    assert result["valid"] is True
    assert seen["library"] is effective_library


def test_quantized_duplicate_xy_nodes_are_collapsed_for_revalidation() -> None:
    from hjmb_pathgen.py_services.leg_optimization_service import _motion_points_from_nodes

    nodes = (
        {"x_mm": 0, "y_mm": 0, "yaw_ddeg": 0},
        {"x_mm": 100, "y_mm": 20, "yaw_ddeg": 10},
        {"x_mm": 100, "y_mm": 20, "yaw_ddeg": 25},
        {"x_mm": 200, "y_mm": 50, "yaw_ddeg": 40},
    )
    points = _motion_points_from_nodes(nodes)
    assert points == ((0.0, 0.0, 0.0), (100.0, 20.0, 25.0), (200.0, 50.0, 40.0))

    start_duplicate = (
        {"x_mm": 0, "y_mm": 0, "yaw_ddeg": 0},
        {"x_mm": 0, "y_mm": 0, "yaw_ddeg": 15},
        {"x_mm": 50, "y_mm": 10, "yaw_ddeg": 20},
    )
    assert _motion_points_from_nodes(start_duplicate)[0] == (0.0, 0.0, 0.0)


def test_optimizer_local_nodes_never_keep_consecutive_quantized_xy_duplicates() -> None:
    from types import SimpleNamespace

    from hjmb_pathgen.py_planning.optimization.leg_optimizer import _local_nodes_from_time_samples

    samples = (
        SimpleNamespace(s_mm=0.0, x_mm=0.0, y_mm=0.0, yaw_ddeg=0.0, speed_mmps=0.0, vx_mmps=0.0, vy_mmps=0.0, wz_ddegps=0.0, flags=1, arrival_state_id=""),
        SimpleNamespace(s_mm=24.4, x_mm=10.2, y_mm=5.2, yaw_ddeg=10.0, speed_mmps=100.0, vx_mmps=90.0, vy_mmps=40.0, wz_ddegps=5.0, flags=0, arrival_state_id=""),
        SimpleNamespace(s_mm=25.1, x_mm=10.4, y_mm=5.4, yaw_ddeg=12.0, speed_mmps=105.0, vx_mmps=94.0, vy_mmps=42.0, wz_ddegps=6.0, flags=0, arrival_state_id=""),
        SimpleNamespace(s_mm=50.0, x_mm=30.0, y_mm=15.0, yaw_ddeg=20.0, speed_mmps=0.0, vx_mmps=0.0, vy_mmps=0.0, wz_ddegps=0.0, flags=2, arrival_state_id="DROP_STEP_1"),
    )
    nodes = _local_nodes_from_time_samples(samples)
    assert len(nodes) == 3
    assert all(
        (left["x_mm"], left["y_mm"]) != (right["x_mm"], right["y_mm"])
        for left, right in zip(nodes, nodes[1:])
    )
    assert nodes[-1]["arrival_state_id"] == "DROP_STEP_1"

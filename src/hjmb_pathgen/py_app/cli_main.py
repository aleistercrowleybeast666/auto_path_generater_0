"""Minimal command line wrappers for V4 service APIs."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

from hjmb_pathgen.py_io.codecs.csv_codec import load_traj_id_csv
from hjmb_pathgen.py_io.codecs.json_codec import load_case, load_leg_library, load_project
from hjmb_pathgen.py_services.collision_config_service import validate_collision_config
from hjmb_pathgen.py_domain.enums import GenerationMode
from hjmb_pathgen.py_domain.leg_optimization import LegOptimizationProfileName
from hjmb_pathgen.py_services.example_project_service import create_synthetic_example_project
from hjmb_pathgen.py_services.leg_clear_service import clear_optimized_leg_result
from hjmb_pathgen.py_services.manual_path_service import plan_manual_case, retime_case
from hjmb_pathgen.py_services.mode_output_service import export_final_bin, write_manual_outputs
from hjmb_pathgen.py_services.phase9_delivery_service import (
    final_drop_audit_from_bin,
    generate_golden_manifest,
    output_layout_report,
    performance_profile,
    protocol_conformance_report,
    release_manifest,
    write_json_report,
)
from hjmb_pathgen.py_services.case_draft_service import generate_all_case_drafts, generate_case_draft
from hjmb_pathgen.py_services.leg_library_service import show_leg, upsert_leg
from hjmb_pathgen.py_services.leg_optimization_service import (
    approve_leg,
    lock_leg,
    optimize_current_case_leg,
    retime_leg,
    show_leg_from_layout,
    unlock_leg,
    validate_leg,
)
from hjmb_pathgen.py_services.path_validation_service import validate_case_collision, validate_leg_collision
from hjmb_pathgen.py_services.phase7_generation_service import (
    audit_phase6,
    collect_unique_legs,
    evaluate_case_candidates,
    export_portable,
    generate_all,
    generate_one,
    optimize_missing_legs,
    show_batch_report,
    show_leg_status,
    validate_all,
    validate_one,
)
from hjmb_pathgen.py_services.plan_lock_service import list_candidates, lock_candidate, unlock_candidate
from hjmb_pathgen.py_services.project_config_service import validate_project_site_configuration
from hjmb_pathgen.py_io.layout.project_layout import ProjectLayout
from hjmb_pathgen.py_services.route_assembler import transition_requirements_from_case
from hjmb_pathgen.py_services.site_preset_service import apply_site_pose_preset, export_site_pose_preset, import_site_pose_preset_preview
from hjmb_pathgen.py_services.traj_table_service import write_route_case_table


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="hjmb-pathgen")
    subparsers = parser.add_subparsers(dest="command", required=True)

    _project_parser(subparsers.add_parser("validate-traj-table"))
    _project_parser(subparsers.add_parser("validate-project-config"))
    _project_parser(subparsers.add_parser("validate-collision-config"))
    validate_case_collision_parser = _project_parser(subparsers.add_parser("validate-current-case-collision"))
    validate_case_collision_parser.add_argument("--case", required=True)
    validate_case_collision_parser.add_argument("--report")
    validate_case_collision_parser.add_argument("--preview", action="store_true")
    validate_leg_collision_parser = _project_parser(subparsers.add_parser("validate-leg-collision"))
    validate_leg_collision_parser.add_argument("--leg-id", required=True)
    validate_leg_collision_parser.add_argument("--report")
    show_collision_report_parser = subparsers.add_parser("show-collision-report")
    show_collision_report_parser.add_argument("--report", required=True)
    protocol_report_parser = subparsers.add_parser("phase9-protocol-report")
    protocol_report_parser.add_argument("--protocol", default="HJMB_path_file_protocol_v4.0.txt")
    protocol_report_parser.add_argument("--output")
    create_example_parser = subparsers.add_parser("create-example-project")
    create_example_parser.add_argument("--output", required=True)
    create_example_parser.add_argument("--source-traj", default="traj_id.csv")
    create_example_parser.add_argument("--generate-outputs", action="store_true")
    release_manifest_parser = subparsers.add_parser("phase9-release-manifest")
    release_manifest_parser.add_argument("--root", default=".")
    release_manifest_parser.add_argument("--output")
    _project_parser(subparsers.add_parser("build-route-case-table"))
    export_preset_parser = _project_parser(subparsers.add_parser("export-site-preset"))
    export_preset_parser.add_argument("--name", required=True)
    export_preset_parser.add_argument("--notes", default="")
    import_preset_parser = _project_parser(subparsers.add_parser("import-site-preset"))
    import_preset_parser.add_argument("--preset", required=True)
    import_preset_parser.add_argument("--preview", action="store_true")
    apply_preset_parser = _project_parser(subparsers.add_parser("apply-site-preset"))
    apply_preset_parser.add_argument("--preset", required=True)
    plan_manual_parser = _project_parser(subparsers.add_parser("plan-manual-case"))
    plan_manual_parser.add_argument("--case", required=True)
    plan_manual_parser.add_argument("--profile", default="default")
    retime_parser = _project_parser(subparsers.add_parser("retime-case"))
    retime_parser.add_argument("--case", required=True)
    retime_parser.add_argument("--profile", default="default")
    list_parser = _project_parser(subparsers.add_parser("list-candidates"))
    list_parser.add_argument("--traj-id", type=int, required=True)
    single_parser = _project_parser(subparsers.add_parser("generate-case-draft"))
    single_parser.add_argument("--traj-id", type=int, required=True)
    _project_parser(subparsers.add_parser("generate-all-case-drafts"))
    lock_parser = _project_parser(subparsers.add_parser("lock-plan"))
    lock_parser.add_argument("--traj-id", type=int, required=True)
    lock_parser.add_argument("--candidate-id", required=True)
    unlock_parser = _project_parser(subparsers.add_parser("unlock-plan"))
    unlock_parser.add_argument("--traj-id", type=int, required=True)
    list_transitions_parser = _project_parser(subparsers.add_parser("list-transition-requirements"))
    list_transitions_parser.add_argument("--case", required=True)
    optimize_leg_parser = _project_parser(subparsers.add_parser("optimize-leg"))
    _leg_optimize_parser(optimize_leg_parser)
    optimize_current_parser = _project_parser(subparsers.add_parser("optimize-current-case-leg"))
    _leg_optimize_parser(optimize_current_parser)
    retime_leg_parser = _project_parser(subparsers.add_parser("retime-leg"))
    retime_leg_parser.add_argument("--leg-id", required=True)
    retime_leg_parser.add_argument("--profile", default="default")
    retime_leg_parser.add_argument("--write", action="store_true")
    retime_leg_parser.add_argument("--force", action="store_true")
    validate_leg_parser = _project_parser(subparsers.add_parser("validate-leg"))
    validate_leg_parser.add_argument("--leg-id", required=True)
    approve_leg_parser = _project_parser(subparsers.add_parser("approve-leg"))
    approve_leg_parser.add_argument("--leg-id", required=True)
    approve_leg_parser.add_argument("--notes", default="")
    lock_leg_parser = _project_parser(subparsers.add_parser("lock-leg"))
    lock_leg_parser.add_argument("--leg-id", required=True)
    lock_leg_parser.add_argument("--notes", default="")
    unlock_leg_parser = _project_parser(subparsers.add_parser("unlock-leg"))
    unlock_leg_parser.add_argument("--leg-id", required=True)
    show_leg_parser = _project_parser(subparsers.add_parser("show-leg"))
    show_leg_parser.add_argument("--leg-id", required=True)
    _project_parser(subparsers.add_parser("audit-phase6"))
    _project_parser(subparsers.add_parser("collect-unique-legs"))
    _project_parser(subparsers.add_parser("show-leg-status"))
    optimize_missing_parser = _project_parser(subparsers.add_parser("optimize-missing-legs"))
    optimize_missing_parser.add_argument("--profile", default=LegOptimizationProfileName.STANDARD.value)
    optimize_missing_parser.add_argument("--seed", type=int, default=0)
    optimize_missing_parser.add_argument("--max-count", type=int)
    optimize_missing_parser.add_argument("--no-stale", action="store_true")
    optimize_missing_parser.add_argument("--force", action="store_true")
    evaluate_parser = _project_parser(subparsers.add_parser("evaluate-case-candidates"))
    evaluate_parser.add_argument("--traj-id", type=int, required=True)
    generate_one_parser = _project_parser(subparsers.add_parser("generate-one"))
    generate_one_parser.add_argument("--traj-id", type=int, required=True)
    generate_one_parser.add_argument("--portable", action="store_true")
    generate_one_parser.add_argument("--dry-run", action="store_true")
    generate_all_parser = _project_parser(subparsers.add_parser("generate-all"))
    generate_all_parser.add_argument("--portable", action="store_true")
    generate_all_parser.add_argument("--dry-run", action="store_true")
    validate_one_parser = _project_parser(subparsers.add_parser("validate-one"))
    validate_one_parser.add_argument("--traj-id", type=int, required=True)
    _project_parser(subparsers.add_parser("validate-all"))
    export_portable_parser = _project_parser(subparsers.add_parser("export-portable"))
    export_portable_parser.add_argument("--traj-id", type=int, required=True)
    write_manual_parser = _project_parser(subparsers.add_parser("write-manual"))
    write_manual_parser.add_argument("--case", required=True)
    write_manual_parser.add_argument("--profile", default="default")
    write_manual_parser.add_argument("--dry-run", action="store_true")
    export_final_parser = _project_parser(subparsers.add_parser("export-final"))
    export_final_parser.add_argument("--traj-id", type=int, required=True)
    export_final_parser.add_argument("--source", choices=[item.value for item in GenerationMode], required=True)
    export_final_parser.add_argument("--profile", default="default")
    export_final_parser.add_argument("--dry-run", action="store_true")
    clear_leg_parser = _project_parser(subparsers.add_parser("clear-leg-result"))
    clear_leg_parser.add_argument("--leg-id", required=True)
    clear_leg_parser.add_argument("--confirm-leg-id")
    output_layout_parser = _project_parser(subparsers.add_parser("phase9-output-layout"))
    output_layout_parser.add_argument("--output")
    golden_parser = _project_parser(subparsers.add_parser("phase9-golden-manifest"))
    golden_parser.add_argument("--output")
    perf_parser = _project_parser(subparsers.add_parser("phase9-performance"))
    perf_parser.add_argument("--operation", choices=["output-layout", "golden-manifest"], default="golden-manifest")
    perf_parser.add_argument("--output")
    final_drop_parser = subparsers.add_parser("phase9-final-drop-audit")
    final_drop_parser.add_argument("--bin", required=True)
    final_drop_parser.add_argument("--output")
    _project_parser(subparsers.add_parser("show-batch-report"))

    args = parser.parse_args(argv)
    try:
        result = _dispatch(args)
    except Exception as exc:  # noqa: BLE001 - CLI boundary reports a concise failure.
        _print_json({"status": "FAILED", "error": str(exc)})
        return 1
    _print_json(_stdout_payload(args, result))
    return 0


def _project_parser(parser: argparse.ArgumentParser) -> argparse.ArgumentParser:
    parser.add_argument("--project", required=True, help="V4 project directory")
    return parser


def _leg_optimize_parser(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--case", required=True)
    parser.add_argument("--transition-id", required=True)
    parser.add_argument("--profile", default=LegOptimizationProfileName.STANDARD.value)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--replace", action="store_true")


def _dispatch(args: argparse.Namespace) -> dict[str, Any]:
    if args.command == "show-collision-report":
        return json.loads(Path(args.report).read_text(encoding="utf-8"))
    if args.command == "phase9-protocol-report":
        report = protocol_conformance_report(args.protocol)
        if args.output:
            write_json_report(args.output, report)
        return report
    if args.command == "create-example-project":
        return create_synthetic_example_project(
            args.output,
            source_traj_csv=args.source_traj,
            generate_outputs=args.generate_outputs,
        )
    if args.command == "phase9-release-manifest":
        report = release_manifest(args.root)
        if args.output:
            write_json_report(args.output, report)
        return report
    if args.command == "phase9-final-drop-audit":
        report = final_drop_audit_from_bin(args.bin)
        if args.output:
            write_json_report(args.output, report)
        return report
    layout = ProjectLayout.open(Path(args.project), create_dirs=False)
    if args.command == "validate-traj-table":
        table = load_traj_id_csv(layout.traj_id_csv)
        return {"row_count": len(table.rows), "source_csv_sha256": table.source_csv_sha256}
    if args.command == "validate-project-config":
        report = validate_project_site_configuration(load_project(layout.project_json))
        return report.to_dict()
    if args.command == "validate-collision-config":
        report = validate_collision_config(load_project(layout.project_json))
        return report.to_dict()
    if args.command == "validate-current-case-collision":
        case = load_case(args.case, enforce_filename=False)
        project = load_project(layout.project_json)
        report_path = Path(args.report) if args.report else None
        result = validate_case_collision(case, project, strict=not args.preview, report_path=report_path)
        return result.to_dict()
    if args.command == "validate-leg-collision":
        library = load_leg_library(layout.leg_library_json)
        leg = next((item for item in library.legs if item.leg_id == args.leg_id), None)
        if leg is None:
            raise ValueError(f"leg not found: {args.leg_id}")
        project = load_project(layout.project_json)
        report_path = Path(args.report) if args.report else None
        result = validate_leg_collision(leg, project, report_path=report_path)
        return result.to_dict()
    if args.command == "build-route-case-table":
        result = write_route_case_table(layout)
        return {"output_path": str(result.output_path), "case_count": len(result.route_case_table.cases)}
    if args.command == "export-site-preset":
        path = export_site_pose_preset(layout, args.name, notes=args.notes)
        return {"output_path": str(path)}
    if args.command == "import-site-preset":
        preview = import_site_pose_preset_preview(layout, args.preset)
        return preview.to_dict()
    if args.command == "apply-site-preset":
        result = apply_site_pose_preset(layout, args.preset)
        return result.to_dict()
    if args.command == "plan-manual-case":
        case = load_case(args.case, enforce_filename=False)
        project = load_project(layout.project_json)
        result = plan_manual_case(case, project, profile_name=args.profile)
        return result.to_dict()
    if args.command == "retime-case":
        case = load_case(args.case, enforce_filename=False)
        project = load_project(layout.project_json)
        result = retime_case(case, project, profile_name=args.profile)
        return result.to_dict()
    if args.command == "list-candidates":
        candidates = list_candidates(layout, args.traj_id)
        return {"traj_id": args.traj_id, "candidate_count": len(candidates), "candidate_ids": [candidate.candidate_id for candidate in candidates]}
    if args.command == "generate-case-draft":
        result = generate_case_draft(layout, args.traj_id)
        return {"traj_id": result.traj_id, "case_path": str(result.case_path), "selected_candidate_id": result.selected_candidate_id}
    if args.command == "generate-all-case-drafts":
        result = generate_all_case_drafts(layout)
        return {
            "case_draft_count": len(result.results),
            "failure_count": len(result.failures),
            "summary_csv_path": str(result.summary_csv_path),
            "report_json_path": str(result.report_json_path),
            "unique_transition_requirements_path": str(result.unique_transition_requirements_path),
            "unique_transition_requirement_count": result.unique_transition_requirement_count,
        }
    if args.command == "lock-plan":
        result = lock_candidate(layout, args.traj_id, args.candidate_id)
        return {"traj_id": result.traj_id, "case_path": str(result.case_path), "selected_candidate_id": result.selected_candidate_id, "locked_by_user": True}
    if args.command == "unlock-plan":
        result = unlock_candidate(layout, args.traj_id)
        return {"traj_id": result.traj_id, "case_path": str(result.case_path), "selected_candidate_id": result.selected_candidate_id, "locked_by_user": False}
    if args.command == "list-transition-requirements":
        project = load_project(layout.project_json)
        case = load_case(args.case, enforce_filename=False)
        requirements = transition_requirements_from_case(case, project)
        return {"transition_count": len(requirements), "transitions": [item.to_dict() for item in requirements]}
    if args.command in {"optimize-leg", "optimize-current-case-leg"}:
        profile = LegOptimizationProfileName(str(args.profile))
        result = optimize_current_case_leg(
            layout,
            args.case,
            args.transition_id,
            profile_name=profile,
            seed=args.seed,
            replace_existing=args.replace,
        )
        return result.to_dict()
    if args.command == "retime-leg":
        project = load_project(layout.project_json)
        library = load_leg_library(layout.leg_library_json)
        leg = show_leg(library, args.leg_id)
        updated = retime_leg(project, leg, profile_name=args.profile)
        if args.write:
            library = upsert_leg(library, updated, replace_existing=True, force=args.force)
            from hjmb_pathgen.py_io.codecs.json_codec import save_leg_library

            save_leg_library(layout.leg_library_json, library)
        return updated.to_dict()
    if args.command == "validate-leg":
        project = load_project(layout.project_json)
        leg = show_leg_from_layout(layout, args.leg_id)
        return validate_leg(project, leg)
    if args.command == "approve-leg":
        library = approve_leg(layout, args.leg_id, notes=args.notes)
        return {"leg_id": args.leg_id, "state": show_leg(library, args.leg_id).state.value}
    if args.command == "lock-leg":
        library = lock_leg(layout, args.leg_id, notes=args.notes)
        return {"leg_id": args.leg_id, "state": show_leg(library, args.leg_id).state.value}
    if args.command == "unlock-leg":
        library = unlock_leg(layout, args.leg_id)
        return {"leg_id": args.leg_id, "state": show_leg(library, args.leg_id).state.value}
    if args.command == "show-leg":
        return show_leg_from_layout(layout, args.leg_id).to_dict()
    if args.command == "audit-phase6":
        return audit_phase6(layout)
    if args.command == "collect-unique-legs":
        result = collect_unique_legs(layout)
        return {"requirement_count": len(result.requirements), "counts_by_status": result.counts_by_status, "report_path": str(result.report_path)}
    if args.command == "show-leg-status":
        return show_leg_status(layout)
    if args.command == "optimize-missing-legs":
        result = optimize_missing_legs(
            layout,
            profile_name=LegOptimizationProfileName(str(args.profile)),
            seed=args.seed,
            include_stale=not args.no_stale,
            max_count=args.max_count,
            force=args.force,
        )
        return result.to_dict()
    if args.command == "evaluate-case-candidates":
        return evaluate_case_candidates(layout, args.traj_id).to_dict()
    if args.command == "generate-one":
        optimization = None if args.dry_run else optimize_missing_legs(
            layout, profile_name=LegOptimizationProfileName.STANDARD, traj_id=args.traj_id
        )
        generation = generate_one(
            layout,
            args.traj_id,
            write_portable=args.portable,
            dry_run=args.dry_run,
        )
        return {"optimization": optimization.to_dict() if optimization else {"dry_run": True}, "generation": generation.to_dict()}
    if args.command == "generate-all":
        optimization = None if args.dry_run else optimize_missing_legs(
            layout, profile_name=LegOptimizationProfileName.STANDARD
        )
        generation = generate_all(layout, write_portable=args.portable, dry_run=args.dry_run)
        return {"optimization": optimization.to_dict() if optimization else {"dry_run": True}, "generation": generation.to_dict()}
    if args.command == "validate-one":
        return validate_one(layout, args.traj_id)
    if args.command == "validate-all":
        return validate_all(layout)
    if args.command == "export-portable":
        return export_portable(layout, args.traj_id).to_dict()
    if args.command == "write-manual":
        case = load_case(args.case, enforce_filename=False)
        return write_manual_outputs(layout, case, profile_name=args.profile, dry_run=args.dry_run).to_dict()
    if args.command == "export-final":
        return export_final_bin(
            layout,
            args.traj_id,
            generation_mode=GenerationMode(args.source),
            profile_name=args.profile,
            dry_run=args.dry_run,
        ).to_dict()
    if args.command == "clear-leg-result":
        return clear_optimized_leg_result(layout, args.leg_id, confirm_leg_id=args.confirm_leg_id).to_dict()
    if args.command == "phase9-output-layout":
        report = output_layout_report(layout)
        if args.output:
            write_json_report(args.output, report)
        return report
    if args.command == "phase9-golden-manifest":
        report = generate_golden_manifest(layout)
        if args.output:
            write_json_report(args.output, report)
        return report
    if args.command == "phase9-performance":
        operation = args.operation
        func = (lambda: output_layout_report(layout)) if operation == "output-layout" else (lambda: generate_golden_manifest(layout))
        report = performance_profile(operation, func)
        if args.output:
            write_json_report(args.output, report)
        return report
    if args.command == "show-batch-report":
        return show_batch_report(layout)
    raise ValueError(f"unsupported command: {args.command}")


def _print_json(value: dict[str, Any]) -> None:
    print(json.dumps(value, ensure_ascii=False, sort_keys=True))


def _stdout_payload(args: argparse.Namespace, result: dict[str, Any]) -> dict[str, Any]:
    if args.command == "create-example-project":
        payload: dict[str, Any] = {
            "status": "OK",
            "root": result.get("root"),
            "route_case_count": result.get("route_case_count"),
            "unique_leg_count": result.get("unique_leg_count"),
            "generated_outputs": result.get("generated_outputs"),
        }
        generation = result.get("generation")
        if isinstance(generation, dict):
            payload["generation_case_count"] = generation.get("case_count")
            payload["generation_failure_count"] = generation.get("failure_count")
            payload["generation_report_json_path"] = generation.get("report_json_path")
            payload["generation_summary_csv_path"] = generation.get("summary_csv_path")
        return payload
    if args.command == "validate-all":
        results = result.get("results", [])
        invalid_count = sum(1 for item in results if isinstance(item, dict) and not item.get("valid", False))
        return {
            "status": "OK",
            "case_count": result.get("case_count"),
            "failure_count": result.get("failure_count"),
            "invalid_count": invalid_count,
        }
    if args.command in {
        "phase9-protocol-report",
        "phase9-release-manifest",
        "phase9-final-drop-audit",
        "phase9-output-layout",
        "phase9-golden-manifest",
        "phase9-performance",
    }:
        payload = {"status": "OK", **_report_summary(result)}
        output = getattr(args, "output", None)
        if output:
            payload["output_path"] = str(Path(output))
        return payload
    return {"status": "OK", **result}


def _report_summary(report: dict[str, Any]) -> dict[str, Any]:
    summary: dict[str, Any] = {}
    if "status" in report:
        summary["report_status"] = report.get("status")
    for key in (
        "passed",
        "format",
        "protocol_version",
        "case_count",
        "final_bin_count",
        "file_count",
        "manifest_sha256",
        "sha256",
        "elapsed_ms",
        "peak_memory_bytes",
        "operation",
        "bin_path",
        "final_drop",
    ):
        if key in report:
            summary[key] = report[key]
    for key in ("errors", "warnings", "missing_protocol_fragments"):
        value = report.get(key)
        if isinstance(value, list):
            summary[f"{key}_count"] = len(value)
    if "result" in report and isinstance(report["result"], dict):
        summary["result_case_count"] = report["result"].get("case_count")
        summary["result_manifest_sha256"] = report["result"].get("manifest_sha256")
        summary["result_status"] = report["result"].get("status")
    return summary


if __name__ == "__main__":
    sys.exit(main())

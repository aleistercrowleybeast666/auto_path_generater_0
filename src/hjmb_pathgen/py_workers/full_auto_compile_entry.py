"""Isolated FULL_AUTO Case evaluation and assembly entry point.

This helper intentionally runs in a completely new Python interpreter.  Some
native numerical backends retain process-global state after leg optimization;
using a subprocess for final evaluation/assembly avoids inheriting that state.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from hjmb_pathgen.py_io.layout.project_layout import ProjectLayout
from hjmb_pathgen.py_services.phase7_generation_service import (
    evaluate_case_candidates,
    generate_one,
)


def _compile(project_root: str, traj_id: int, *, write_portable: bool, dry_run: bool) -> dict[str, Any]:
    layout = ProjectLayout.open(project_root)
    evaluation = evaluate_case_candidates(layout, traj_id)
    if not any(item.complete for item in evaluation.timings):
        details = " | ".join(
            f"{item.candidate_id}: missing {', '.join(item.missing_leg_ids) or 'none'}"
            for item in evaluation.timings
        )
        raise RuntimeError(
            f"P{traj_id:04d} has no complete candidate after automatic optimization: {details}"
        )
    generation = generate_one(
        layout,
        traj_id,
        write_portable=write_portable,
        dry_run=dry_run,
    ).to_dict()
    return {
        "phase": "GENERATED",
        "candidate_evaluation": evaluation.to_dict(),
        "generation": generation,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--project-root", required=True)
    parser.add_argument("--traj-id", required=True, type=int)
    parser.add_argument("--result-file", required=True)
    parser.add_argument("--write-portable", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)

    result_path = Path(args.result_file)
    result_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        result = _compile(
            args.project_root,
            args.traj_id,
            write_portable=bool(args.write_portable),
            dry_run=bool(args.dry_run),
        )
        payload: dict[str, Any] = {"ok": True, "result": result}
        exit_code = 0
    except Exception as exc:  # noqa: BLE001 - isolated process boundary.
        payload = {"ok": False, "error": str(exc)}
        exit_code = 1
    result_path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())

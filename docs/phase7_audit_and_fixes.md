# Phase 7 Audit And Fixes

Phase 8 re-audited the Phase 7 generation path before adding the UI workflow.

## Findings

- Phase 7 generated into flat `cases/`, `bin/`, and `portable/` directories. This could collide with MANUAL_FREE files sharing the same `traj_id`.
- `_best_complete_case()` selected the fastest complete candidate even when a user-locked candidate was still feasible.
- Generated task cases set `review.approved=true`. Approval is now reserved for explicit user review before final export.
- Leg reuse depended mostly on `leg.state`; generation now actively checks stale dependencies, planner version, leg hash, collision, topology, and dynamics validation.
- KINEMATIC source actions could previously carry or inherit `check_start_s_mm`. Source actions now reject that field, and compiled check starts are generated after final trajectory assembly.
- `validate_one/all` only compiled the case. Validation now also reports dependency audit, BIN round trip, reserved finish bits, and final export guard status.

## Current Fixes

- TASK_COMPILED generation writes mode-scoped files under `cases/task_compiled/` and `bin/task_compiled/`.
- Legacy flat MANUAL_FREE files block headless task generation unless `--replace-manual` is passed.
- Locked candidates are preserved when complete and semantically unchanged. Invalid locks fail with `LOCK_CONFLICT`.
- Auto-generated task cases remain unapproved and therefore cannot be exported to `bin/final/`.
- Leg clearing is an explicit service operation on `leg_library.json`; it does not mutate `project.json`.

## Remaining Risk

The Phase 8 UI is a workflow shell over the corrected services. Rich in-table editing of every V4 field remains a later UI refinement task.

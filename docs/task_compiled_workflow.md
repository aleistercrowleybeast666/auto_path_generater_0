# Task Compiled Workflow

TASK_COMPILED is generated from `traj_id.csv`, `route_case_table.json`, `project.json`, and `leg_library.json`.

1. Collect unique directed legs.
2. Optimize missing or stale legs explicitly.
3. Evaluate candidates for a `traj_id`.
4. Preserve a locked candidate if it is complete and its semantic hash still matches.
5. Generate the working case and BIN into `cases/task_compiled/` and `bin/task_compiled/`.
6. Validate before approval.
7. Export to `bin/final/` only after explicit approval.

Automatic generation never sets `review.approved=true`.

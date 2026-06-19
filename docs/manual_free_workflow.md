# Manual Free Workflow

MANUAL_FREE cases are detached from the leg library. They require:

- `path_source=MANUAL_FREE`
- `selected_plan.route_family=MANUAL_FREE`
- `review.detached_from_library=true`
- `review.manual_override=true`
- non-empty `review.override_reason`
- a valid `manual_path`

Manual working outputs are written to `cases/manual_free/` and `bin/manual_free/`.

Manual and task outputs may share the same `traj_id` because their directories are separate.

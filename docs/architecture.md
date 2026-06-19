# Architecture After Phase 3

Phase 1 introduced a `src` package layout while keeping thin root compatibility
wrappers for the current V3.5 GUI, CLI, and tests. Phase 2 adds V4.0 JSON/BIN
IO, project layout, atomic writes, portable Case export, and unified single /
partial batch output services. Phase 3 adds strict `traj_id.csv` parsing, task
candidate compilation, source action generation, plan locking, 360 Case draft
generation, and deterministic reports.

## Package Layout

```text
src/hjmb_pathgen/
  app/
    main.py
  codec/
    bin_codec.py
    binary_layout.py
    canonical_json.py
    csv_codec.py
    crc32.py
    fixtures.py
    json_codec.py
    legacy_rejection.py
    validators.py
  legacy/
    v35/
      editor.py
      path_codec_cli.py
      path_models.py
      path_geometry.py
      trajectory_planner.py
      trajectory_graphics.py
      batch_models.py
      batch_generator.py
      test_utils.py
  models/
    action.py
    compiled.py
    enums.py
    errors.py
    leg.py
    project.py
    protocol.py
    route_case.py
    task_mapping.py
    task_plan.py
  planning/
  services/
    action_source_compiler.py
    atomic_writer.py
    batch_service.py
    case_draft_service.py
    case_compiler.py
    output_service.py
    path_naming.py
    plan_lock_service.py
    portable_service.py
    project_service.py
    task_compiler.py
    traj_table_service.py
  ui/
  utils/
    yaw_unwrap.py
  cli/
```

Root modules such as `hjmb_path_editor.py`, `path_codec_cli.py`, and
`path_models.py` are compatibility wrappers. They add local `src/` to
`sys.path`, re-export the V3.5 API, and keep old commands/tests working.

## Dependency Direction

- `models` is pure Python and does not depend on PySide6, UI, or planning.
- `codec` depends on `models` and standard-library modules for pure encode/decode
  logic. Public save helpers delegate to the atomic writer.
- `services` contains pure-Python project IO, output orchestration, Phase 3 task
  compilation, source action generation, plan locking, and draft reports. It
  does not depend on PySide6.
- `legacy/v35` contains the existing V3.5 implementation and may depend on
  PySide6 for the GUI.
- `app/main.py` points to the current V3.5 GUI entry point.
- `planning` and `ui` remain placeholders for later V4 workflows.

## V4 And Legacy Boundary

V4.0 code lives in `hjmb_pathgen.models` and `hjmb_pathgen.codec`. V3.5 code
lives in `hjmb_pathgen.legacy.v35`. The old V3.5 models are not reused as V4
models, and the V3.5 UI does not write V4 files.

## Test Layout

```text
tests/
  fixtures/
    legacy/
    v40/
  integration/
  unit/
```

Current root `test_*_v35.py` files remain as V3.5 regression tests and continue
to use compatibility wrappers. New Phase 1 tests live under `tests/unit` and
`tests/integration`.

## Phase Boundary

Phase 3 implements reviewable 360 Case drafts only. It does not implement path
optimization, collision checking, worker processes, new UI tabs, optimized legs,
dense nodes, or formal 360 BIN output.

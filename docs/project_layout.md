# V4.0 Project Layout

Phase 2 adds project directory services under `hjmb_pathgen.services`.
The service creates and opens the V4.0 project skeleton without pretending that
Phase 3 `traj_id.csv` compilation has happened.

## Standard Tree

```text
HJMB_Path_Project/
  project.json
  route_case_table.json        # absent until Phase 3 mapping is produced
  traj_id.csv                  # input authority for Phase 3
  leg_library.json
  cases/
    P0007.json
  bin/
    P0007.BIN
  portable/
    P0007.portable.json
  reports/
    validation_report.json
    batch_summary.csv
    optimization_log/
  cache/
```

`ProjectLayout.create(root, project)` creates the directories, writes
`project.json`, and writes a valid empty `leg_library.json`. It does not create
a fake `route_case_table.json`.

## Path Rules

All generated paths are resolved through `ProjectLayout`; absolute
project-relative paths and `..` traversal are rejected. Case paths are built
only through strict helpers:

```text
case_json_name(7) -> P0007.json
bin_name(7) -> P0007.BIN
portable_name(7) -> P0007.portable.json
```

Only `traj_id` values `0..359` are accepted. Non-canonical names such as
`P1.BIN`, `P00001.BIN`, and `p0007.bin` are rejected.

## Status Model

Phase 2 defines:

```text
INITIALIZED
INCOMPLETE_MAPPING
INCOMPLETE_LIBRARY
READY_FOR_SINGLE_CASE
READY_FOR_BATCH
INVALID
```

A freshly initialized Phase 2 project with `project.json`, directories, and an
empty leg library is `INCOMPLETE_MAPPING` because `route_case_table.json` is not
present. `READY_FOR_BATCH` is reserved for future full 360-case readiness.

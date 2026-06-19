# AGENTS.md

## Purpose

This repository builds the HJMB bean-carrying robot V4.0 offline path editor, optimizer, batch compiler, and BIN encoder.
Codex must treat this file as persistent project instructions and use `TARGET.md` as the phased delivery plan.

## Read first

Before changing code, read in this order:

1. `AGENTS.md`
2. `TARGET.md`
3. `HJMB_path_file_protocol_v4.0.txt`
4. current tests and repository README
5. only then inspect implementation files

The V4.0 protocol is the source of truth. Do not silently change protocol fields, sizes, enum values, JSON formats, hashes, or semantics.
If a protocol inconsistency is found, stop and report it before changing code.

## Scope

Implement the desktop path-generation software only.
Do not implement STM32 firmware in this repository.
Firmware field-size compensation is only an interface/documentation concern here.

## Baseline reality

The uploaded/current codebase may still be V3.3 even when newer design documents exist.
Verify actual code before assuming a feature is implemented.
Do not claim V4.0 support until tests prove it.

## Non-negotiable behavior

- V4.0 is incompatible with V3.x JSON/BIN.
- Reject legacy formats clearly; do not silently migrate them.
- No CUT_IN runtime path.
- START velocity is zero.
- Every ARRIVAL is a full stop.
- BIN `vx/vy/wz` are exact feedforward targets, never speed limits.
- `ASYNC` actions have no trigger position.
- `KINEMATIC` actions have no user window or expiry; only generated `check_start_s_mm` and `stable_time_ms`.
- Mechanical actions are strict FIFO and complete by module `DONE` plus optional `post_wait_ms`.
- Competition completion occurs at the final drop only: the last drop ARRIVAL is the unique `FINISH_ARM` point, and completion is emitted only after the final `DROP_*` STOP_AND_WAIT action reports `DONE`, its `post_wait_ms` has elapsed, the action FIFO is empty, and the chassis remains stopped.
- Formal V4.0 competition output must not generate `SAFE_END`, `FINISH_CLEAR`, half-plane crossing, or a post-drop clearing segment.
- Editing must never auto-run heavy planning. Mark results `STALE` and wait for an explicit user command.
- A high candidate speed is not a global planning failure. Reduce speed, propagate constraints, subdivide, and retry.
- `traj_id.csv` is the only authority for the 360 mappings.
- Single-case and batch generation must use the same compiler and produce byte-identical BIN for the same case and inputs.

## Protocol constants

Do not change without an explicit protocol revision:

- Header V40: 104 bytes
- Node V40: 16 bytes
- Segment V40: 24 bytes
- Action V40: 22 bytes
- BIN version: 40
- field nominal size: 4000 x 2000 mm
- competition traj_id: 0..359

Finish-related packed fields remain in place for binary-layout compatibility:

- the only valid formal completion mode is `FINISH_MODE_AT_FINAL_DROP`
- the old `SAFE_END` node flag is reserved and must be zero
- the old `FINISH_CLEAR` segment flag is reserved and must be zero
- legacy half-plane/braking finish fields remain packed but must be zero in formal V4.0 output

Use the exact `struct` formats from the protocol and assert `struct.calcsize` in tests.

## Project architecture target

Prefer a package layout with separate concerns:

- models
- codec
- task compilation
- geometry
- collision
- planning
- batch/cache/reporting
- providers
- UI/workers

Do not keep adding unrelated behavior to one large Qt window module.
Keep pure computation independent of PySide6 wherever possible.

## Data ownership

- `project.json`: shared manual configuration
- `route_case_table.json`: normalized CSV mapping
- `leg_library.json`: optimized reusable directed legs
- `cases/Pxxxx.json`: compiled case manifests
- `bin/Pxxxx.BIN`: final executable path

Do not duplicate optimized dense legs into all referenced Case JSON files.
Portable single-case JSON may embed resolved legs.

## Geometry and collision

Support all three footprint models:

1. large circle for cylinders
2. small circle for drop boxes
3. truncated large circle for pickup boxes: `u^2 + v^2 <= R_large^2` and `u <= R_small`

Collision checking must cover continuous segments, not only sampled nodes.
S-shaped obstacle routing is enforced by ordered virtual topology gates.
Virtual gates never appear in the BIN.

## Route/task constraints

Automatic pickup route families are limited to:

- `PICK_1_TO_3`: START -> PICK_1 -> PICK_2L -> PICK_3
- `PICK_3_TO_1`: START -> PICK_3 -> PICK_2R -> PICK_1

Drop planning must account for physical drop site, vehicle bin mask, single/dual unload, and continuous yaw.
Allowed unload masks: BIN_1, BIN_2, BIN_3, BIN_12, BIN_23.
Never generate BIN_13 or BIN_123.

The final task action must be one of `DROP_1`, `DROP_2`, `DROP_3`, `DROP_12`, or `DROP_23`, must use `STOP_AND_WAIT`, and must be bound to the final `FINISH_ARM` ARRIVAL. No motion segment or task action may follow it.

## Planning rules

- Optimize time, not distance or energy.
- Reuse identical directed legs.
- Keep START/ARRIVAL boundary velocities zero.
- Prefer yaw rotation in low-speed windows, but collision and true total time decide the result.
- Speed parameterization should operate on `z = v^2` with forward reachable and backward controllable propagation.
- Final verification must check speed, total acceleration, lateral acceleration, angular velocity, angular acceleration, wheel rpm, topology, and collision.

## UI and long-running jobs

- No 220 ms or similar automatic replanning timer.
- Long planning must run outside the Qt main thread, preferably in a worker process.
- Provide progress, stage text, current item, best time, elapsed time, warnings, errors, and cancellation.
- Never leave partially written JSON/BIN files.
- Use temporary files and atomic replace after validation.

## Testing requirements

For each phase:

- add or update tests before claiming completion
- run the relevant tests
- report exact commands and results
- keep existing passing behavior unless the phase intentionally replaces it

Mandatory eventual coverage:

- all V40 struct sizes
- CRC vector
- JSON round trips
- BIN round trips
- legacy rejection
- 360 CSV mapping validation
- single vs batch byte identity
- speed-planner no-false-failure properties
- continuous collision cases
- final-drop completion ordering and blocking behavior
- rejection of nonzero reserved `SAFE_END` / `FINISH_CLEAR` bits and legacy finish modes
- cancellation and atomic write behavior

## Repository hygiene

- Do not commit `.venv`, `__pycache__`, generated BIN/JSON batches, caches, logs, or build outputs.
- Use official Windows x64 CPython 3.14 as the primary development environment; Python 3.13 compatibility is desirable. Do not use MSYS2/MinGW Python for the PySide6 environment.
- Keep dependencies minimal and declared.
- Do not vendor third-party packages.
- Preserve UTF-8 without BOM for JSON/Markdown/Python source unless an external input requires BOM handling.

## Working method

Follow `TARGET.md` phase order.
Do not attempt all phases in one unreviewable rewrite.
At the start of a phase:

1. inspect current implementation
2. state the files to change
3. identify protocol/test implications
4. implement the smallest coherent slice
5. run tests
6. summarize completed, incomplete, and risks

When a requirement is ambiguous, do not invent a hidden rule. Surface the ambiguity and use an explicit configuration or enum.

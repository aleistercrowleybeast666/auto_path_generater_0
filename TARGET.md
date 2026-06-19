# TARGET.md — HJMB Path Generator V4.0

## Mission

Transform the current path editor into a reliable V4.0 offline shortest-time path generation system for the bean-carrying robot.
The deliverable must support manual single-case work, reusable optimized directed legs, exact compilation of all 360 competition cases, and validated V4.0 BIN output.

The authoritative protocol is:

- `HJMB_path_file_protocol_v4.0.txt`

Persistent Codex rules are:

- `AGENTS.md`

Do not start a later phase while the current phase acceptance criteria are failing, unless the user explicitly authorizes parallel work.


## Target skills

Codex should apply these capabilities deliberately:

1. **Repository archaeology** — distinguish actual V3.3 code from V3.5/V4 design documents.
2. **Protocol engineering** — packed binary structures, endian rules, offsets, CRC, version rejection, deterministic encoding.
3. **Typed data modeling** — clear separation among project, route table, leg library, Case, portable Case, and compiled BIN.
4. **Numerical geometry** — splines, arc-length sampling, curvature, continuous yaw profiles, robust finite checks.
5. **Collision detection** — circles, rounded rectangles, convex truncated-circle footprint, SAT/GJK, continuous subdivision.
6. **Time-optimal parameterization** — `z=v^2`, forward reachability, backward controllability, wheel/yaw/acceleration constraints.
7. **Optimization engineering** — deterministic seeds, multi-start, cancellation, progress, caching, stale invalidation.
8. **Task compilation** — exact traj_id CSV mapping, two pickup route families, vehicle-bin assignment, single/dual drops.
9. **Qt architecture** — responsive UI, worker processes, explicit planning commands, stale states.
10. **Testing and reproducibility** — fixtures, property tests, 360-case regression, atomic outputs, byte identity.


## Product invariants

- V4.0 does not silently accept V3.x.
- START and every ARRIVAL are zero-speed boundaries.
- `vx/vy/wz` in BIN are exact feedforward.
- Editing does not trigger heavy planning.
- `ASYNC` has no path trigger.
- `KINEMATIC` has only generated `check_start_s_mm`, no user window or expiry.
- The 360 mapping comes only from `traj_id.csv`.
- Pickup route families are only `PICK_1_TO_3` and `PICK_3_TO_1`.
- Allowed unload masks are BIN_1, BIN_2, BIN_3, BIN_12, BIN_23.
- Repeated directed legs are optimized once and reused.
- Single and batch compilation of the same Case produce byte-identical BIN.
- Nominal paths use 4000×2000 mm; actual field scaling remains a later firmware concern.
- The competition finishes when the final drop action is complete. The last drop ARRIVAL is the unique `FINISH_ARM` point; formal V4.0 output has no `SAFE_END`, no `FINISH_CLEAR`, no half-plane finish, and no post-drop clearing motion.


# Phase 0 — Freeze and clean the baseline

## Goal

Create a reproducible baseline before architectural changes.

## Tasks

- Inspect the real repository version and document it.
- Remove or ignore `.venv`, `__pycache__`, generated outputs, caches, and unrelated large files.
- Add `.gitignore` entries.
- Record the current test inventory and results.
- Add dependency metadata (`pyproject.toml` preferred).
- Use official Windows x64 CPython 3.14 as the primary environment; keep Python 3.13 compatibility where practical, and do not use MSYS2/MinGW Python for the PySide6 environment.
- Place `AGENTS.md`, `TARGET.md`, and the V4.0 protocol at repository root.

## Acceptance

- Fresh environment can install dependencies.
- Baseline tests run with documented commands.
- Repository is clean and does not track a virtual environment.
- No feature claim is made from design documents alone.


# Phase 1 — Lock V4.0 models and protocol tests

## Goal

Make the V4 data contract executable before implementing advanced planning.

## Tasks

- Implement constants/enums for all V40 JSON and BIN formats.
- Add packed structure formats:
  - Header 104
  - Node 16
  - Segment 24
  - Action 22
- Add `struct.calcsize` assertions.
- Implement canonical JSON hashing helper.
- Implement CRC-32/IEEE helper and known vector.
- Define typed models for:
  - project
  - route case table
  - leg library
  - Case manifest
  - portable Case
  - compiled trajectory
- Reject all V3.x formats and deleted fields with clear errors.

## Acceptance

- Exact size tests pass.
- CRC vector passes.
- Minimal valid JSON fixtures load and round-trip.
- Minimal valid BIN fixture encodes, decodes, and re-encodes identically.
- Legacy inputs fail with precise messages.


# Phase 2 — JSON codecs, BIN codec, and project directories

## Goal

Establish the complete V4 data pipeline without advanced optimization.

## Tasks

- Implement read/write for project/table/library/Case/portable Case.
- Implement V40 BIN encode/decode.
- Validate offsets, file size, IDs, flags, reserved fields, and CRC.
- Implement atomic file writes.
- Implement immediate write-then-read validation.
- Implement project directory creation.
- Implement single-case output and empty batch skeleton output.
- Ensure single and batch code paths call the same compiler/encoder functions.

## Acceptance

- A synthetic Case can produce a valid BIN.
- Portable Case can regenerate the same BIN without external leg library.
- Interrupted/failed writes do not leave final files.
- `Pxxxx` filename and traj_id mismatch is rejected.


# Phase 3 — traj_id table and task compiler

## Goal

Convert `traj_id.csv` into deterministic, reviewable task plans.

## Tasks

- Parse BOM/non-BOM UTF-8 CSV safely.
- Validate exactly 360 rows and all 6×60 combinations.
- Generate `route_case_table.json`.
- Model bean types, labels 1–5, physical sites F_DROP_4–8, and target rank 1–3.
- Implement the two pickup route families.
- Implement vehicle-bin assignments.
- Enumerate legal single/dual unload plans.
- Use manually configured BIN_12/BIN_23 yaw values.
- Generate continuous unwrapped yaw sequences.
- Generate source mechanical FIFO actions.
- Allow user locking of a selected plan.
- Produce 360 Case drafts with no optimized legs yet.

## Acceptance

- Every CSV row is reversible from the normalized table.
- All 360 traj_id values are unique and complete.
- PICK_2L/PICK_2R choice matches route family.
- No BIN_13/BIN_123 is generated.
- Empty boxes 4/5 do not alter target semantics.
- Every Case has at least one legal candidate or a specific reason it does not.


# Phase 4 — Robust time parameterization

## Goal

Replace the current false-failure speed planner before adding automatic geometry optimization.

## Tasks

- Remove all nonzero CUT_IN boundary behavior.
- Use START/ARRIVAL zero-speed boundaries.
- Implement `z=v^2` forward reachable and backward controllable propagation.
- Enforce:
  - max speed
  - lateral acceleration
  - total acceleration
  - braking
  - yaw rate
  - yaw acceleration
  - four-wheel rpm
- Add adaptive subdivision where interval constraints change quickly.
- Add local repair and monotonic speed-envelope tightening.
- Separate structural infeasibility from “candidate speed too high”.
- Add final quantized-node verification.

## Acceptance

- Finite ordinary paths do not fail merely because an initial candidate is too fast.
- Sharp curves automatically slow down.
- START/ARRIVAL are exactly zero speed.
- Final quantized nodes satisfy all limits.
- Property tests cover random curves and extreme but valid limits.


# Phase 5 — Collision models and manual path verification

## Goal

Make collision checking real before automatic optimization.

## Tasks

- Implement field rectangles and cylinders.
- Implement large-circle, small-circle, and truncated-circle vehicle footprints.
- Rotate the pickup footprint by yaw.
- Implement convex collision using SAT or GJK.
- Implement target-box approach semantics.
- Implement field-boundary checking.
- Implement continuous segment subdivision.
- Compute and display minimum clearance and first collision location.
- Add ordered virtual topology gates for the S route.

## Acceptance

- All three footprint diagrams match the intended geometry.
- Changing yaw changes pickup-box collision correctly.
- A collision between safe endpoint samples is detected.
- Tangency is numerically stable and treated according to configured tolerance.
- Topology violations are reported separately from geometric collision.


# Phase 6 — Directed leg optimizer and library

## Goal

Optimize a directed leg inside a fixed topology and cache it safely.

## Tasks

- Implement cubic B-spline or piecewise cubic Bézier geometry.
- Generate initial geometry from manual points/templates.
- Implement topology-gate constraints.
- Implement the two-low-speed-window monotonic yaw model.
- Alternate XY control-point optimization, yaw optimization, and speed parameterization.
- Use multiple deterministic initializations.
- Optimize planned motion time.
- Store control points, yaw parameters, dense nodes, diagnostics, hashes, seed, and review state in `leg_library.json`.
- Invalidate stale legs when dependent project hashes change.
- Support quick, standard, and final optimization profiles.

## Acceptance

- Optimized leg preserves exact endpoint pose and zero boundary speed.
- It obeys topology and collision constraints.
- Yaw is continuous and does not reverse against the selected policy.
- It is no slower than its accepted initial template, or reports why.
- Same seed/config produces reproducible results.
- A changed dependency makes the leg STALE, not silently reusable.


# Phase 7 — Phase 6 fixes, unique-leg collection, and full Case assembly

## Goal

Repair any Phase 6 defects that block trustworthy reuse, then compile all Cases by optimizing only unique missing legs.

## Tasks

- Audit and repair Phase 6 correctness issues before batch reuse:
  - ordered topology-gate progression
  - real XY/yaw search
  - seed/profile behavior
  - cancellation/timeout with best valid result
  - stale handling for approved/locked legs
  - consistent review state
  - complete retime/validate/hash refresh
  - safe warm-start restoration
- Compile all legal candidates for all 360 Cases, not only the Phase 3 default selection.
- Collect complete directed state-transition keys.
- Deduplicate transitions across all candidate plans.
- Optimize only missing/stale/quality-insufficient unique legs.
- Evaluate candidates by real motion time plus mechanism time and post-wait.
- Preserve valid user-locked candidates; otherwise select the fastest feasible candidate deterministically.
- Assemble global nodes with duplicate boundary removal.
- Recompute global `s`.
- Generate the Segment table.
- Renumber ARRIVAL and actions.
- Recompute KINEMATIC `check_start_s_mm`.
- Mark the final drop ARRIVAL as the unique `FINISH_ARM`.
- Require the final blocking action to be a `DROP_*` STOP_AND_WAIT bound to that ARRIVAL.
- Emit completion only after final drop `DONE`, `post_wait_ms`, empty FIFO, and stopped chassis.
- Do not generate `SAFE_END`, `FINISH_CLEAR`, half-plane finish, or post-drop motion.
- Keep old finish-related packed bits/fields reserved and zero.
- Generate referenced Case JSON, optional portable JSON, and BIN.
- Generate batch reports.

## Acceptance

- Phase 6 audit defects are fixed and covered by regression tests.
- No invalid, stale, preview-only, or hash-mismatched leg is reused.
- Shared legs are byte-for-byte reused where intended.
- Every assembled Case passes topology, collision, dynamics, quantization, action, finish, protocol, and hash validation.
- Every formal Case has exactly one `FINISH_ARM`, located at the final drop ARRIVAL.
- The final action is a legal `DROP_*` STOP_AND_WAIT action bound to that ARRIVAL.
- No Segment or task action follows the final drop.
- `SAFE_END` and `FINISH_CLEAR` reserved bits are zero, and legacy finish modes are rejected.
- Single and batch generation of the same Case produce identical BIN.
- All 360 BIN files read back and re-encode identically.
- Failures identify the exact Case, candidate, leg, node, Segment, or action.


# Phase 8 — UI workflow, progress, and cancellation

## Goal

Make the software usable without blocking or accidental recomputation.

## Tasks

- Remove automatic planning timers.
- Add explicit STALE/PLANNING/VALID/FAILED states.
- Disable export for STALE/FAILED results.
- Split the UI into focused tabs:
  - field/manual sites
  - vehicle/collision
  - current leg/path
  - mechanical actions
  - leg library
  - 360 Cases/batch
  - planning parameters
- Run long jobs in a worker process.
- Add stage progress, current item, best time, elapsed time, ETA, warnings, errors, and cancel.
- Add double-click navigation from batch errors to Case/leg.
- Add commands for current Case, selected legs, all missing legs, and all 360 outputs.

## Acceptance

- Editing never starts heavy planning.
- UI remains responsive during final optimization.
- Cancellation works and leaves no partial final files.
- Progress is meaningful and monotonic.
- User can inspect and manually approve or override any Case/leg.


# Phase 9 — Final-drop completion validation, packaging, and delivery

## Goal

Complete the V4 desktop product, validate the final-drop completion contract end to end, and deliver reproducible outputs.

## Tasks

- Finalize Case-side `FINISH_MODE_AT_FINAL_DROP` compilation.
- Validate that the unique `FINISH_ARM` is the final drop ARRIVAL.
- Validate that completion waits for final `DROP_*` DONE, `post_wait_ms`, empty FIFO, and stopped chassis.
- Reject legacy `AT_SAFE_END` / half-plane finish modes.
- Reject nonzero reserved `SAFE_END` and `FINISH_CLEAR` bits.
- Confirm all legacy half-plane/braking packed fields are zero in formal V4.0 output.
- Add all-project validation and a 360-case golden manifest.
- Profile performance and memory.
- Test disk errors, cancellation, malformed inputs, and stale hashes.
- Write README and user operation guide.
- Provide an example V4 project.
- Package with PyInstaller `onedir`.
- Ensure protocol copy in repository matches implementation constants.

## Acceptance

- Full automated test suite passes.
- A clean machine can run the packaged application.
- 360 Case JSON/BIN outputs are reproducible from the example project.
- Reports expose all warnings and manual overrides.
- No hidden V3.3/V3.5 behavior remains in the V4 execution path.


## Completion definition

The project is complete only when:

1. V4.0 protocol tests and codecs are stable.
2. `traj_id.csv` produces exactly 360 traceable Cases.
3. unique directed legs are optimized, cached, invalidated, and reviewed correctly.
4. every generated BIN passes round-trip and full dynamic/collision validation.
5. single and batch workflows are consistent.
6. heavy planning is explicit, cancellable, and visible.
7. the application can be packaged and used without the development environment.
8. every competition Case completes at the final drop with no SAFE_END, FINISH_CLEAR, half-plane crossing, or post-drop motion.

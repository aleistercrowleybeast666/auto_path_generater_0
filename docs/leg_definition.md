# Phase 6 Leg Definition

A directed leg is the reusable motion unit between two task states. Its key is
directional and includes endpoint poses, route family, topology profile,
topology gates, yaw policy, planner algorithm version, and functional
dependency hashes.

Phase 6 writes optimized legs into `leg_library.json`. The BIN layout is not
changed. Virtual topology gates and Bezier control handles remain library-side
planning metadata and never become BIN nodes.

Reusable states are:

```text
VALID
APPROVED
LOCKED
```

`QUICK_PREVIEW` optimization creates `PREVIEW_VALID`. Preview legs are useful
for inspection but are not approved for final compilation.

Local leg nodes store:

```text
local_s_mm, x_mm, y_mm, yaw_ddeg, vx_mmps, vy_mmps, wz_ddegps, flags
```

They do not store global `arrival_id`. Case assembly renumbers arrivals later.


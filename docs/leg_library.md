# Phase 6 Leg Library

`leg_library.json` remains the V4.0 reusable directed-leg store. Phase 6 adds
optimizer-produced metadata inside existing V4.0 leg fields:

```text
key
control_points
yaw_profile
nodes
analysis
hashes
review
```

The library service writes atomically through the existing JSON codec and
round-trip validation.

Review semantics:

```text
APPROVED or LOCKED legs are never overwritten silently.
PREVIEW_VALID legs cannot be approved as final reusable legs.
STALE marks dependency or planner-version mismatch.
```

The `hashes` dictionary contains:

```text
validity_hash          SHA-256 semantic validity hash
self_hash32            CRC32 compatibility hash used by existing compiler refs
dependency_hashes      functional project dependency hashes
planner_algorithm_version
```


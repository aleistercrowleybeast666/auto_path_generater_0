# Leg Clear And Regenerate

Phase 8 adds an explicit clear operation for optimized leg results.

Clearing a leg:

- edits only `leg_library.json`
- changes the leg state to `MISSING`
- removes control points, yaw profile, dense nodes, analysis, and validity hashes
- clears approval and lock review flags

Approved or locked legs require a matching confirmation token:

```bash
python -m hjmb_pathgen.cli clear-leg-result --project <project> --leg-id <LEG_ID> --confirm-leg-id <LEG_ID>
```

After clearing, run missing-leg optimization explicitly. Clearing does not automatically replan cases.

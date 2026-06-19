# GUI Route/Leg

The `Route/Leg` tab reads `leg_library.json` and draws the selected leg on a
V4 field canvas.

Visible overlays include:

- dense leg path from `leg.nodes`
- control points when present
- speed-colored path overlay
- robot collision footprints at the selected/last pose
- topology gates from `project.json.topology_profiles` when configured
- project obstacles and fixed sites

Buttons are explicit commands. Validation and optimization are not triggered by
editing or selection. Clearing a leg uses the Phase 8 clear service, edits only
`leg_library.json`, and does not modify `project.json`.

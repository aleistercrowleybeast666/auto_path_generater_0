# Collision Configuration

Phase 5 makes collision configuration explicit in `project.json`.

`vehicle.footprint` requires:

```json
{
  "r_large_mm": 120,
  "r_small_mm": 70,
  "collision_resolution_mm": 10,
  "strict_validation_resolution_mm": 5,
  "numerical_epsilon_mm": 0.000001,
  "pickup_arc_segments": 64,
  "field_boundary_footprint_profile": "LARGE_CIRCLE"
}
```

`field_objects` requires:

- exactly two `cylinders`
- exactly three physical `pickup_boxes`: `PICK_1`, `PICK_2`, `PICK_3`
- exactly five `drop_boxes`: `F_DROP_4` through `F_DROP_8`
- one nominal `field_boundary`

`P_PICK_2L` and `P_PICK_2R` are arrival states. They both reference the same
physical `PICK_2` obstacle; no second middle pickup box is created.

Functional collision hashes:

- `collision_config_hash`: footprint and boundary validation parameters.
- `obstacle_geometry_hash`: obstacle positions, sizes, orientations, enable
  flags, and configured flags.

Changing notes or UI-only metadata does not change those hashes.

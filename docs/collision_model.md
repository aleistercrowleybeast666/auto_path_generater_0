# Phase 5 Collision Model

Phase 5 adds collision validation only. It does not move waypoints, optimize
geometry, search yaw profiles, generate public legs, or change BIN structures.

## Footprints

- `LARGE_CIRCLE`: `u^2 + v^2 <= R_large^2`.
- `SMALL_CIRCLE`: `u^2 + v^2 <= R_small^2`.
- `PICKUP_CLIPPED_DISK`: `u^2 + v^2 <= R_large^2` and `u <= R_small`.

The clipped disk chord is `u = R_small`, with endpoints
`(R_small, +/-sqrt(R_large^2 - R_small^2))`. The front cap is removed, so a
pickup box may enter that missing cap. The retained side/back part is still
protected and rotates with robot yaw.

## Mapping

- Cylinders use `LARGE_CIRCLE`.
- Drop boxes use `SMALL_CIRCLE`.
- Pickup boxes use `PICKUP_CLIPPED_DISK`.
- The nominal field boundary uses `LARGE_CIRCLE`.

Target boxes are not skipped. A target is allowed to be close only because the
correct footprint is used for that obstacle type.

## Clearance

All checks use the same classification:

- `CLEAR`: clearance is greater than epsilon.
- `TOUCHING`: clearance is inside `[-epsilon, epsilon]` and is legal.
- `PENETRATING`: clearance is less than `-epsilon`.

`numerical_epsilon_mm` is floating-point tolerance only. Physical safety margin
must already be included in radii or obstacle dimensions.

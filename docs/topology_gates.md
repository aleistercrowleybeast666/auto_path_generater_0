# Phase 6 Topology Gates

Topology gates are ordered virtual line segments used to force a candidate XY
curve through the intended obstacle topology. They are planning constraints,
not physical path nodes.

Supported gate shape in Phase 6:

```json
{
  "gate_id": "G1",
  "a": {"x_mm": 50, "y_mm": -20},
  "b": {"x_mm": 50, "y_mm": 20},
  "direction": "NEGATIVE"
}
```

`direction` may be `ANY`, `POSITIVE`, or `NEGATIVE`. Direction is computed from
the sign of `cross(gate_vector, path_segment_vector)`.

The validator checks that gates are crossed in order. Gates are not emitted to
BIN and do not appear as arrival or action triggers.


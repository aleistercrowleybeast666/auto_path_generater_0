# Phase 4 Time Parameterization

V4 uses `hjmb_pathgen.py_planning.dynamics.time_parameterization` for finite geometry
that is already defined by manual points or later optimizers.

The module does not optimize geometry and does not perform collision checking.
It operates on `z = v^2`, runs forward reachable and backward controllable
passes, then verifies quantized V40 nodes.

Enforced limits:

- max speed
- lateral acceleration
- total acceleration and braking
- yaw rate
- moving yaw acceleration
- four-wheel mecanum rpm

Failure categories are explicit:

- `INVALID_INPUT`
- `STRUCTURAL_GEOMETRY_ERROR`
- `INVALID_LIMITS`
- `NUMERICAL_FAILURE`
- `QUANTIZATION_RANGE_ERROR`
- `NO_FINITE_TIME_SOLUTION`

A high candidate speed is treated as a reducible envelope problem, not a global
path failure.

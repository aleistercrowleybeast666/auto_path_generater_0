# Phase 6 Leg Optimizer

The public optimizer entry point is:

```python
from hjmb_pathgen.py_planning.optimization.leg_optimizer import optimize_leg
```

The service entry points are:

```python
from hjmb_pathgen.py_services.leg_optimization_service import (
    leg_request_from_transition,
    optimize_transition_leg,
    optimize_current_case_leg,
    retime_leg,
    validate_leg,
    approve_leg,
    lock_leg,
    unlock_leg,
)
```

Candidate evaluation pipeline:

```text
initial XY guess
-> piecewise cubic Bezier
-> ordered topology gate validation
-> TWO_LOW_SPEED_WINDOWS yaw samples
-> Phase 5 continuous collision validation
-> Phase 4 z=v^2 time parameterization
-> quantized local nodes and metrics
```

The objective is planned motion time. Collision, topology, malformed geometry,
and dynamics/time-parameterization failures are hard candidate failures. Curvature
quality is used as a safety guard and deterministic tie-break, not as a reason to
prefer a longer route with an unnecessarily large radius. Near-cusp paths receive
a strong penalty; otherwise the actual time-parameterized result remains dominant.

For pickup-to-drop transfers, ordered virtual gates are crossing intervals rather
than compulsory centre points. AUTOMATIC evaluates a primary seed through the
nearest legal points on the ordered gates and retains the gate-centre S seed as a
conservative fallback. Both candidates still pass the same ordered-gate,
continuous-collision, curvature, wheel-speed, and time-parameterization checks.

Profiles:

```text
QUICK_PREVIEW
STANDARD
FINAL
```

`QUICK_PREVIEW` produces `PREVIEW_VALID`; `STANDARD` and `FINAL` produce
`VALID`.

# Phase 6 Leg Optimizer

The public optimizer entry point is:

```python
from hjmb_pathgen.planning.leg_optimizer import optimize_leg
```

The service entry points are:

```python
from hjmb_pathgen.services.leg_optimization_service import (
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
and dynamics/time-parameterization failures are hard candidate failures.

Profiles:

```text
QUICK_PREVIEW
STANDARD
FINAL
```

`QUICK_PREVIEW` produces `PREVIEW_VALID`; `STANDARD` and `FINAL` produce
`VALID`.


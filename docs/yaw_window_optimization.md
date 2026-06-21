# V4.0 Continuous Yaw Distribution

New paths no longer concentrate rotation into two low-speed windows.  The leg-library metadata uses the V4.0-reserved `MONOTONIC_BSPLINE` model with a degree-1 monotonic spline, which is exactly uniform in arclength.

For every START/ARRIVAL stop-to-stop leg, the resolved continuous yaw change is
distributed uniformly over the complete XY arclength:

```text
yaw(s) = yaw_start + delta_yaw * s / total_length
q = d(yaw)/ds = constant
q_prime = 0
```

Because chassis speed is zero at START and ARRIVAL, `wz = q * v` is also zero at
both ends.  During motion, angular speed follows the same single acceleration and
braking envelope as translational speed, avoiding repeated yaw acceleration and
deceleration.

The time parameterizer records `wz` and `beta` for diagnostics but does not use
them as independent speed caps.  Rotation feasibility is enforced through the
combined four-wheel RPM calculation; translational speed is reduced only when an
actual wheel would exceed `wheel.plan_limit_rpm`.

Yaw policies remain `CW_ONLY`, `CCW_ONLY`, and `SHORTEST`.

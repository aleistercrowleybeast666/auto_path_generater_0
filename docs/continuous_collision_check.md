# Continuous Collision Check

Node-only collision checks are not valid for Phase 5.

For adjacent poses:

```text
P0 = (x0, y0, yaw0)
P1 = (x1, y1, yaw1)
```

the checker uses:

```text
motion_bound =
  hypot(x1 - x0, y1 - y0)
  + R_large * abs(delta_yaw_rad)
```

If `motion_bound` is larger than the validation resolution, the interval is
subdivided and the midpoint is checked recursively. The yaw difference is the
continuous unwrapped difference, not a shortest-wrap value.

Limits:

- maximum recursion depth
- maximum checked poses
- minimum degenerate interval guard

If a limit is exceeded, validation returns `NUMERICAL_ERROR`. It never passes a
path merely because recursion stopped.

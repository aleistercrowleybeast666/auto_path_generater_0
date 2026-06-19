# Phase 6 Yaw Window Optimization

Yaw is generated with `TWO_LOW_SPEED_WINDOWS`.

The model splits total yaw change by `alpha`:

```text
start window: 0 .. start_window_end_s_ratio
middle:       constant yaw
finish window: finish_window_start_s_ratio .. 1
```

Each window uses quintic smoothstep, so yaw rate is zero at the beginning and
end of each low-speed window. The sampled geometry includes:

```text
yaw_ddeg
yaw_ddeg_per_mm
yaw_ddeg_per_mm2
```

Policies:

```text
CW_ONLY      resolved delta is non-positive
CCW_ONLY     resolved delta is non-negative
SHORTEST     resolved delta is in [-1800, 1800)
```


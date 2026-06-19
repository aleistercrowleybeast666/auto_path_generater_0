# Path Validation

Phase 5 exposes pure service APIs:

```python
validate_spatial_path_collision(samples, project)
validate_time_parameterized_trajectory(samples_or_result, project)
validate_case_collision(case, project)
validate_leg_collision(leg, project)
```

Results include:

- status
- checked collision config hash
- checked path hash
- validation resolution
- minimum clearance and pose
- closest obstacle
- first collision
- collision count
- checked pose count
- subdivision count
- warnings and errors

Formal export guard blocks any case whose collision status is not `PASSED`.
`FAILED`, `NOT_CHECKED`, `STALE`, `NUMERICAL_ERROR`, and `NO_GEOMETRY` are not
formal competition export states.

Manual override does not bypass collision validation.

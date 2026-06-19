# Phase 3 Task Compiler

The Phase 3 task compiler converts one `RouteCaseRowV40` into deterministic
task candidates. It does not perform path geometry optimization, speed
planning, collision checking, or BIN generation.

## Numbering Systems

The compiler keeps these identifiers separate:

```text
PICK_1/2/3       CSV pickup positions
P_PICK_2L/2R     robot arrival states for pickup position 2
label 1/2/3      target bean labels: YELLOW/GREEN/WHITE
F_DROP_4..8      physical drop sites
DROP_TARGET_1..3 left-to-right effective target ranks for this Case
BIN_1/2/3        vehicle storage bins
```

Labels 4 and 5 are empty boxes. They remain in raw mapping traceability but do
not enter `DROP_TARGET_1..3`.

## Route Families

Only two automatic route families are generated:

```text
PICK_1_TO_3:
  P_START -> P_PICK_1 -> P_PICK_2L -> P_PICK_3
  drop target order: TARGET_3 -> TARGET_2 -> TARGET_1
  yaw direction: CW_ONLY

PICK_3_TO_1:
  P_START -> P_PICK_3 -> P_PICK_2R -> P_PICK_1
  drop target order: TARGET_1 -> TARGET_2 -> TARGET_3
  yaw direction: CCW_ONLY
```

No third pickup order is generated in Phase 3.

## Vehicle Bin Assignment

The default assignment is bound to the route family so the unload sweep rotates
in one direction. It is a bean-to-bin bijection:

```text
PICK_1_TO_3:
  TARGET_3 bean -> BIN_1
  TARGET_2 bean -> BIN_2
  TARGET_1 bean -> BIN_3

PICK_3_TO_1:
  TARGET_1 bean -> BIN_3
  TARGET_2 bean -> BIN_2
  TARGET_3 bean -> BIN_1
```

Source actions simulate the storage ledger: the bean picked from a pickup slot
is stored into the assigned vehicle bin, and the later DROP action must unload
the same bean from that same bin.

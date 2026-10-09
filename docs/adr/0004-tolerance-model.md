# ADR-0004: Tolerance model

## Status
Accepted (2025-10-07)

## Context
CAD tolerances are a first-class citizen. Treating them as global
constants leads to silent corruption when models cross domains (e.g.
importing a STEP defined in microns into a millimeter project, or
converting an SDF field evaluated in `float` to a B-Rep stored in
`double`).

## Decision

### 1. No global epsilon
Each operation that needs a tolerance receives a `Tolerance` value
explicitly. Default tolerances (`kDefaultLinear`, `kDefaultAngular`)
are exposed as constants but the operation's signature does not have
a default argument.

### 2. Tolerance kind is part of the type
A `Tolerance` is `{ToleranceKind, double}` where `ToleranceKind` is
`Linear | Angular | Relative`. Mixing `Linear` and `Angular`
tolerances is a logic error.

### 3. Accumulation is tracked
The `ToleranceAccumulator` struct aggregates linear and angular
errors as they accumulate across operations. Every conversion
between representations (SDF ↔ B-Rep, B-Rep ↔ mesh, etc.) pushes
its declared error into the accumulator.

### 4. Exposure to the user
The feature tree displays the accumulated tolerance next to each
shape. The Python API exposes `Shape::tolerance_summary()` that
returns a `ToleranceAccumulator` for any shape in the tree.

## Defaults
| Quantity | Default value | Rationale |
|----------|---------------|-----------|
| Linear   | 1e-6 mm       | Matches STEP default precision (`LENGTH_UNIT` 1e-6 m, expressed in mm) |
| Angular  | 1e-7 rad      | Tighter than STEP's default 1e-5 rad because we need it for ray-SDF intersections |
| Relative | 1e-9          | Used for ratio comparisons (length/length) |

## Consequences
- Every boolean operation requires the caller to declare the tolerance.
- The Python convenience API picks `kDefaultLinear` if omitted, but
  emits a `DeprecationWarning` if the user relies on this default.
- Cross-unit operations (mm + inch) are forbidden at compile time
  in C++; Python raises `ValueError`.

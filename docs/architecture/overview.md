# Architecture Overview

cadforge is a **dual-representation CAD kernel** — both SDF (signed distance fields) and B-Rep (boundary representation) are first-class peers. There is no "primary" representation that the other is converted into; the user works in whichever domain fits the design problem.

This document is the entry point to the architecture. Deeper topics are covered in dedicated files under `docs/architecture/` and `docs/adr/`.

## Top-level diagram

```
+----------------------------------+
|        Python bindings          |  ← cadforge.sdf, cadforge.geometry, ...
|        (nanobind, Phase A)      |
+----------------------------------+
                |
                v
+----------------------------------+
|        core/ (C++20)            |
|                                  |
|  +------------+  +------------+ |
|  | geometry/  |  | math/      | |
|  | Shape      |  | Vec3f, Mat4 | |
|  | ShapeId    |  | Predicates  | |
|  +-----+------+  +------+-----+ |
|        |                |       |
|        v                v       |
|  +--------------------------+  |
|  | sdf/                     |  |
|  | field.hpp (SDFBody)      |  |
|  | primitives/operators     |  |
|  | evaluate (xsimd + TBB)   |  |
|  +--------------------------+  |
|                                 |
|  +--------------------------+  |
|  | concurrency/             |  |
|  | thread_pool (TBB)        |  |
|  | gil_guard (RAII)         |  |
|  | parallel_for, task_graph |  |
|  +--------------------------+  |
|                                 |
|  +--------------------------+  |
|  | brep/ (Phase D)          |  |
|  | topology (radial-edge)   |  |
|  | analytic surfaces        |  |
|  | boolean ops              |  |
|  +--------------------------+  |
|                                 |
|  +--------------------------+  |
|  | bridge/ (Phase E)        |  |
|  | sdf_to_brep, brep_to_sdf |  |
|  +--------------------------+  |
+----------------------------------+
                |
                v
+----------------------------------+
|        UI (PySide6, Phase C)    |
|        Viewport + feature tree  |
+----------------------------------+
```

## The Shape type

`Shape` is the unified CAD entity. Its payload is a `std::variant`:

```cpp
using BodyVariant = std::variant<
    std::unique_ptr<sdf::SDFBody>,    // SDF-native
    std::unique_ptr<brep::BRepBody>,  // B-Rep-native
    HybridBody                          // Both, with bridge tolerance
>;
```

This is a **correction** from the original proposal's `optional<SDF>, optional<BRep>` design — see [ADR-0002](../adr/0002-dual-kernel-native-sdf.md) for the rationale.

## Native SDF kernel

The SDF kernel (`core/sdf/`) is **native** — not a converter from B-Rep.

- `field.hpp` — `SDFBody` (immutable tree root), `SDFNode` (abstract leaf/operator)
- `primitives.hpp` — sphere, box, cylinder, torus, cone, capsule, plane
- `operators.hpp` — union, intersect, subtract, smooth_min (CSG)
- `transforms.hpp` — translate, rotate, scale, twist
- `evaluate.hpp` — `EvalBackend` abstraction + `CpuEvalBackend` (xsimd + TBB) + `GpuEvalBackend` (stub)

Every primitive declares its `lipschitz()` constant, used by the ray marcher and the mesh extractor for adaptive step sizing.

## Concurrency model

- **oneTBB** for the thread pool and the feature-tree task graph (see [ADR-0009](../adr/0009-threading-model.md)).
- **RAII GIL guard** (`gil_release`, `gil_acquire`) for safe Python ↔ C++ threading.
- **parallel_for** with adaptive grain size (serial for < 1024 elements, parallel beyond).

## Persistent naming

`ShapeId` is composed of `FeatureId + SubshapeRef + Generation` — see [ADR-0005](../adr/0005-persistent-naming-early.md). **No geometric signature is used for naming** — this is the critical correction from the original proposal, which would have broken on fillet/chamfer and parameter edits.

## Tolerance model

Tolerances are tracked explicitly. Each conversion between representations pushes its declared error into a `ToleranceAccumulator`, exposed in the feature tree UI. See [ADR-0004](../adr/0004-tolerance-model.md).

## Threading safety with Python

Every binding function that does heavy work uses:

```cpp
{
    cadforge::concurrency::gil_release no_gil;
    backend.evaluate(body, points, out);  // pure C++
}  // GIL reacquired before returning to Python
```

This is enforced by code review.

## Phase plan

- **Phase A** (current) — Math, geometry layer, native SDF, bindings, foundations
- **Phase B** — SDF mesh extraction (marching cubes + dual contouring)
- **Phase C** — Viewport (PySide6 + OpenGL 4.5 + SDF raymarching)
- **Phase D** — B-Rep MVP (topology + analytic + boolean)
- **Phase E** — Bridge SDF ↔ B-Rep
- **Phase F** — Modeling parametrico (sketch + feature tree)
- **Phase G** — B-Rep advanced (fillet, chamfer, shell, NURBS booleans)
- **Phase H** — Full IO suite + native format versioning

See the original plan document (proposal) for the full breakdown.

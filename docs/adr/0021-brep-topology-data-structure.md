# ADR-0021: B-Rep topology data structure

## Status
Accepted (2026-10-14) — Phase D, step (a)

## Context
ADR-0012 chose the **radial-edge** structure (Weiler 1986) for the B-Rep
kernel, rejecting the half-edge data structure because it cannot represent
non-manifold solids (3+ faces around one edge, wire bodies, sheet bodies,
boolean intermediate states).

ADR-0012 is a *direction* ADR: it states the structural family but does
not pin down:

- the entity set (`Vertex`/`Edge`/`CoEdge`/`Loop`/`Face`/`Shell`/`Solid`) and
  the responsibilities of each class;
- the ID scheme and its scope;
- the ownership rules between entities (who owns the `CoEdge*` pointers
  stored radially on an `Edge`);
- the **Euler invariant** that every closed shell must satisfy and that the
  test suite must enforce as a regression guard;
- how the radial edge gets populated (in-place during construction, or via
  a post-pass `link_radial_edges()`);
- how this connects to ADR-0005 persistent naming (`ShapeId`/`FeatureId`
  composition) — without dictating Phase F history mechanics.

This ADR closes those gaps. It does **not** introduce boolean operations,
curves beyond the analytic ones already in `curve.hpp`, or any rendering
hook — those are Phase D steps (b) and (c) and get their own ADRs.

## Decision

### 1. Entity set and responsibilities

| Entity    | Owns                                    | Owned by   | Notes |
|-----------|-----------------------------------------|------------|-------|
| `Vertex`  | `math::Vec3f position`, `VertexId id`   | `BRepBody` | Pure 3D point + stable id. |
| `Edge`    | `VertexId v[2]`, `EdgeId id`, `std::vector<CoEdge*> coedges_radial_` | `BRepBody` | The radial list is **non-owning**. Each entry points into the `Loop` that uses this edge. |
| `CoEdge`  | `EdgeId edge`, `bool reversed`          | `Loop`     | Oriented use of an edge inside one loop. `reversed` flips the edge's intrinsic `v[0] → v[1]` direction. |
| `Loop`    | `std::vector<CoEdge> coedges_`, `bool is_outer_` | `Face`     | Closed cycle of co-edges bounding a face. The first loop is the outer boundary; subsequent loops are holes. |
| `Face`    | `std::shared_ptr<SurfaceGeometry> surface_`, `std::vector<Loop> loops_`, `FaceId id`, `bool forward_` | `Shell` | Bounded region of a surface. `forward=false` flips the surface normal. |
| `Shell`   | `std::vector<Face> faces_`, `bool is_closed_` | `Solid` (or `BRepBody` directly for open shells) | Connected set of faces. `is_closed=true` is the manifold-closed contract. |
| `Solid`   | `Shell shell_`                          | `BRepBody` | Closed shell enclosing a volume. |
| `BRepBody`| `Solid solid`, `std::vector<Vertex> vertices`, `std::vector<Edge> edges` | user code | Public-facing aggregate. The single owner of all `Vertex` and `Edge` storage. |

The existing classes in `core/brep/include/CAD_0/brep/topology.hpp`
already implement this layout. This ADR ratifies that layout as the
approved design (it was de facto before; now it is de jure).

### 2. ID scheme

- `VertexId`, `EdgeId`, `FaceId`, `ShellId` are `std::uint64_t`.
- IDs are **scoped per `BRepBody`**, not global. Two bodies may both have
  a vertex with id 0; the disambiguation is the body that owns the id.
- IDs are assigned **sequentially** by factories (`make_box`, `make_sphere`,
  …) in construction order, starting from 0.
- IDs are **not** the persistent names of ADR-0005. `ShapeId{FeatureId,
  SubshapeRef, Generation}` is the persistent layer; the per-body
  `VertexId`/`EdgeId`/… are the *runtime* layer. A future history-aware
  build will translate between the two. This separation is what ADR-0005
  calls "subshape tracing, not signature matching": at rebuild time, the
  new subshapes inherit the `ShapeId` of the originating operation, not
  of a coincident geometric vertex.
- The `id()` getters and `set_id()` setters are public because factories
  need to assign them; user code is expected to treat them as read-only
  after construction.

### 3. Ownership of radial-edge pointers

The radial-edge list `Edge::coedges_radial_` stores **raw non-owning
pointers** to `CoEdge` instances owned by `Loop` instances.

Two consequences:

1. The pointers are valid only as long as the owning `Loop` is not
   reallocated. A `Loop` that grows its `coedges_` vector past capacity
   invalidates every `CoEdge*` registered radially. **Factories must
   finish adding co-edges to every loop before calling
   `link_radial_edges(body)`.**
2. There is no automatic unlinking on body mutation. Removing a face
   from a shell does not remove the orphaned `CoEdge*` from the
   radial list. The next `link_radial_edges(body)` rebuilds the radial
   lists from scratch — this is the only sanctioned way to mutate the
   topology after construction.

This is intentionally simple for the MVP. A future ADR may switch to
`std::unique_ptr<CoEdge>` in `Loop` (stable addresses, removes the
reallocation hazard) or to indirect indices `(FaceId, LoopIndex,
CoEdgeIndex)` — both are non-breaking at the API level.

### 4. Radial-edge population

Factories do **not** register `CoEdge*` pointers in `Edge::coedges_radial_`
during construction. Instead, every factory ends with a call to:

```cpp
namespace CAD_0::brep {
    void link_radial_edges(BRepBody& body);
}
```

This function iterates over every face of every shell, and for every
`CoEdge` in every loop, appends `&coedge` to `body.edges[coedge.edge()].coedges_radial_`.

Rationale:

- Keeps factory code linear and easy to read (no need to thread edge
  mutation through every face construction).
- Provides a single place where the radial-edge invariant is enforced.
- Allows future body mutations (face removal, edge swap) to rebuild the
  radial lists by re-running `link_radial_edges(body)`.

### 5. Euler invariant

For a `BRepBody` whose single shell is closed and manifold (every edge
has exactly two co-edges, every loop is closed, every face has a single
outer loop), the Euler characteristic is:

```
χ = V − E + F = 2
```

where `V = body.vertices.size()`, `E = body.edges.size()`,
`F = body.face_count()`. The function:

```cpp
namespace CAD_0::brep {
    std::int64_t euler_characteristic(const BRepBody& body) noexcept;
}
```

returns `V − E + F` as a signed integer. For `make_box({1,1,1})` the
expected return is `2`. For a torus body (single-face periodic surface,
no edges), it returns `0 + 0 + 1 = 1`, which does **not** equal `2`
because a torus represented as one periodic face is not a closed shell
in the Euler sense — this is a known limitation that the next ADR
(Phase D step (b)) will address by giving the torus a proper wireframe
of seams.

The test suite enforces `χ == 2` on every closed-solid factory output.

### 6. Loop closure invariant

Every loop in a face must be **closed**: for a loop with co-edges
`c_0, c_1, …, c_{n-1}`, the end vertex of `c_i` must equal the start
vertex of `c_{(i+1) mod n}`.

The test suite checks this on every face of every factory output. A loop
that fails this check has a bug in the factory's edge orientation — the
fix is always in the factory, never in the data structure.

### 7. What is explicitly NOT in this ADR

- **Boolean operations** (union/intersect/subtract). They are Phase D
  step (c) and require an ADR for the intersection graph + classification
  algorithm.
- **Surface-surface intersection curves**. Same as above.
- **Persistent naming** (ADR-0005 mechanics: `ShapeId` propagation
  through boolean ops). Phase F.
- **Tessellation**. Already covered by `tessellate.hpp`; not touched.
- **Bindings to Python**. Phase D step (b) will add the nanobind
  wrappers once the C++ topology is stable and tested.
- **Rendering hooks**. The viewport stays SDF-only until Phase E
  (SDF ↔ B-Rep bridge).

## Consequences

- `core/brep/include/CAD_0/brep/topology.hpp` is the canonical
  declaration; `core/brep/include/CAD_0/brep/factory.hpp` is the only
  sanctioned place to build `BRepBody` instances in MVP code.
- `link_radial_edges(BRepBody&)` and `euler_characteristic(const BRepBody&)`
  are added to a new translation unit `core/brep/src/topology.cpp` and
  declared in `topology.hpp`.
- `core/brep/CMakeLists.txt` adds `src/topology.cpp` to
  `CAD_0_BREP_SOURCES`.
- `tests/unit/brep/test_topology.cpp` gains new test cases for:
  - radial-edge population (`make_box` → every edge has exactly 2 co-edges);
  - loop closure (`make_box` → every loop's co-edges form a closed cycle);
  - Euler characteristic (`make_box` → `euler_characteristic == 2`).
- The existing `make_box` factory is cleaned up: dead code in Face 3
  (the `loop = Loop(); face = Face(...)` reassignment mid-block) is
  removed, the "actually" / "wait" comments are deleted, and the
  function ends with `link_radial_edges(*body)`.
- No new third-party dependency is introduced (ADR-0003 honored).
- Floating-point determinism (ADR-0014) is untouched: the topology layer
  is integer ID arithmetic + raw pointer bookkeeping. No `float` is
  evaluated by anything in this ADR.

## References
- ADR-0012 — non-manifold topology (radial-edge, the parent decision).
- ADR-0005 — persistent naming by genealogy (the future history layer
  that will sit on top of these runtime IDs).
- ADR-0004 — tolerance model (every geometric query against this
  topology will receive a `Tolerance` explicitly; the MVP does not yet
  issue such queries).
- ADR-0003 — third-party policy (no CAD kernel dependency).
- ADR-0014 — floating-point determinism (the topology layer is
  integer-only and does not perturb FP evaluation).
- Weiler, K. (1986). *Topological Structures for Geometric Modeling*.
  PhD thesis, Rensselaer Polytechnic Institute.

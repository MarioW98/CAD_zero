# ADR-0022: B-Rep topology query operators

## Status
Accepted (2026-10-14) — Phase D, step (b)

## Context
ADR-0021 pinned down the topology data structure (entity set, ID scheme,
radial-edge ownership, Euler invariant, loop closure). It deliberately
stopped at `link_radial_edges`, `euler_characteristic`, and
`loops_are_closed` — the bare minimum to validate the box factory.

Phase D step (b) needs more: the test suite must be able to **walk** the
topology graph to verify invariants that V/E/F counting cannot reach:

- Is every vertex of the cube really incident on 3 edges? (manifold
  solid vertex fan)
- For every edge, are the two radial co-edges oppositely oriented?
  (consistent shell orientation)
- Given a face, can I enumerate its neighbours via shared edges?
- Given a vertex, can I enumerate its incident edge ring?
- Is the body really a closed manifold (every edge has 2 co-edges AND
  every loop is closed AND `shell.is_closed()`)?

These are pure queries over the topology graph. They do not evaluate
any floating-point quantity; they only walk ID + pointer bookkeeping.

ADR-0021 §5 also called out a known limitation: `make_sphere` and
`make_torus` produce χ ≠ 2 because they represent the periodic surface
as a single face with no seam wire. This ADR does **not** fix that —
the seam wire is a separate concern (periodic surface parametrization)
that belongs to Phase D step (c) (analytic surfaces + boolean on
convex solids). What this ADR does is make the limitation **machine-
checkable**: `is_closed_manifold(make_sphere(...))` returns false,
and the test suite enforces it as a regression guard so a future
refactor of `make_sphere` cannot silently start returning true.

## Decision

### 1. Query operators are free functions, not methods

All new operators are **free functions** in `namespace CAD_0::brep`,
taking `const BRepBody&` (or `const BRepBody&, EdgeId` etc.). None of
them is a member of `Vertex`/`Edge`/`Face`/…

Rationale:

- Keeps the entity classes dumb data holders. Future ADRs may move to
  a different layout (`unique_ptr<CoEdge>`, indexed references, etc.)
  without rewriting query call sites.
- Mirrors the STL convention (`std::size`, `std::begin`) where
  algorithms are decoupled from containers.
- Avoids header bloat: the query operators live in `topology.cpp`, not
  in the inline entity declarations.

### 2. The operator set

| Operator | Signature | Returns |
|---|---|---|
| `faces_around_edge` | `(const BRepBody&, EdgeId) → std::vector<FaceId>` | The faces that use this edge, in radial order. Empty if `edge_id >= body.edges.size()`. For a manifold solid, always size 2. |
| `edges_around_vertex` | `(const BRepBody&, VertexId) → std::vector<EdgeId>` | The edges incident on this vertex, in no particular order (radial sort is Phase D step (c) — needs curve tangents). |
| `faces_around_vertex` | `(const BRepBody&, VertexId) → std::vector<FaceId>` | The faces touching this vertex, deduplicated. |
| `opposite_coedge` | `(const BRepBody&, const CoEdge*) → const CoEdge*` | For a manifold edge, the other co-edge on the same edge. Returns `nullptr` if the edge is non-manifold or the input is not registered radially. |
| `is_closed_manifold` | `(const BRepBody&) → bool` | `shell.is_closed() && loops_are_closed(body) && every edge has exactly 2 coedges`. Strict enough to reject the periodic-surface sphere/torus. |
| `wireframe_edges` | `(const BRepBody&) → std::vector<std::pair<VertexId, VertexId>>` | One `(v0, v1)` pair per edge, with `v0 < v1` (canonical form). Useful for future wireframe rendering; pure topology here. |
| `vertex_valence` | `(const BRepBody&, VertexId) → std::size_t` | Number of edges incident on the vertex. Convenience wrapper around `edges_around_vertex(body, v).size()`. |

### 3. Return types

All return `std::vector` **by value**. The cost of the copy is trivial
for the MVP scale (cube: 12 entries; tetrahedron: 6). When the kernel
starts handling real CAD bodies (10⁴+ edges) the return type can be
switched to `std::span<const EdgeId>` (C++26) without breaking callers.

The exception is `opposite_coedge`, which returns a raw `const CoEdge*`.
The pointer is non-owning and valid for the lifetime of the body (same
contract as the radial-edge list itself, ADR-0021 §3).

### 4. `opposite_coedge` lookup contract

`opposite_coedge(body, ce)` finds the edge that owns `ce` (via
`ce->edge()`), iterates that edge's radial list, and returns the
**first** entry that is not `ce` itself. For a manifold edge (2
co-edges) this is unambiguous.

For a non-manifold edge (3+ co-edges, e.g. a boolean union along a
coincident face) the function returns the first non-`ce` entry in
radial order. This is the entry immediately adjacent to `ce` in the
radial list. Downstream code that needs the full radial ring must call
`body.edges[ce->edge()].coedges()` directly — `opposite_coedge` is the
**convenience** accessor for the common manifold case, not a complete
radial walker.

### 5. `make_tetrahedron`

Adds a second factory case to ADR-0021's `make_box`. A regular
tetrahedron has:

- 4 vertices, 6 edges, 4 triangular faces, 1 shell.
- χ = 4 − 6 + 4 = 2.
- Every vertex has valence 3.
- Every edge is shared by exactly 2 faces.

It is the smallest non-trivial closed manifold and the cheapest
stress test for the query operators above. The factory uses the
canonical vertex layout:

```
       v0 = ( 1,  1,  1)
       v1 = (-1, -1,  1)
       v2 = (-1,  1, -1)
       v3 = ( 1, -1, -1)
```

(edge length `2√2`, centroid at origin). Face normals are oriented
outward; each face is a 3-coedge loop (not 4 like the box).

### 6. What is explicitly NOT in this ADR

- **Boolean operations** — Phase D step (c), separate ADR.
- **Surface-surface intersection curves** — same.
- **Persistent naming** (`ShapeId` propagation) — Phase F.
- **Python bindings** — added in a later step once the C++ surface is
  stable.
- **Rendering / viewport hooks** — viewport stays SDF-only until
  Phase E (SDF ↔ B-Rep bridge).
- **Radial ordering of `edges_around_vertex`** — needs curve tangents
  and a proper fan walker; deferred to Phase D step (c).
- **Seam wires for periodic surfaces** (sphere, torus) — Phase D step
  (c) when we tackle analytic surfaces properly.

## Consequences

- Six new free functions declared in `topology.hpp`, implemented in
  `topology.cpp`. Pure integer + pointer bookkeeping — no FP.
- New `make_tetrahedron` factory in `factory.hpp`. Uses `PlaneSurface`
  for all 4 faces (each face is a triangle on a plane).
- `tests/unit/brep/test_topology.cpp` gains ~10 new test cases
  exercising every operator on both `make_box` and `make_tetrahedron`.
- `is_closed_manifold(make_sphere(...))` returns false — a documented
  regression guard against silently "fixing" the sphere into a closed
  shell without proper seam wires.
- No new third-party dependency (ADR-0003 honored).
- Floating-point determinism (ADR-0014) untouched: every new function
  is integer ID arithmetic + raw pointer bookkeeping. No `float` is
  evaluated by anything in this ADR.

## References
- ADR-0021 — B-Rep topology data structure (the parent ADR).
- ADR-0012 — non-manifold topology (radial-edge, the grandparent).
- ADR-0005 — persistent naming (the future history layer that will
  use these query operators to trace subshape genealogy).
- ADR-0003 — third-party policy.
- ADR-0014 — floating-point determinism.
- Weiler, K. (1986). *Topological Structures for Geometric Modeling*.

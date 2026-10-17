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
| `is_closed_manifold` | `(const BRepBody&) → bool` | Strict closed-orientable-manifold contract (4 invariants, see §7). |
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

### 7. `is_closed_manifold` — strict closed-orientable-manifold contract

> **Amendment (2026-10-18, ADR-0027 D6 — Phase F, accepted).**
> Invariant 4 below turned out to be an over-promise of *spherical
> topology* — correct for every body this repository had produced until
> a boolean feature existed: a pierced box (genus 1, χ = 0) is a
> perfectly closed orientable manifold that invariant 4 rejects. The
> contract is now SPLIT: **`is_closed_orientable`** enforces invariants
> 0–3 (the assembly detectors), χ is **reported, never asserted**
> (`euler_characteristic` + `genus_from_chi`: genus = (2 − χ)/2 derived
> only when χ ≤ 2 and even — χ > 2 reads as multi-shell, "genus n/d",
> components counter deferred), and `is_closed_manifold` remains as a
> **compatibility wrapper with its exact historical meaning**
> (`is_closed_orientable && χ == 2`) so no existing call site changes
> semantics — nothing that passed starts failing, nothing that failed
> starts passing. The §7 analysis below — the χ-invariance of a
> duplicated interface ring, and the conclusion that only invariant 2
> catches it — is UNAFFECTED, and becomes what it always actually was:
> the structural invariants are the assembly-defect detectors; χ is the
> input's topology. The "non-connected shells (χ ≠ 2)" rejection entry
> below is re-read accordingly: multi-shell is a CLASSIFICATION
> (χ > 2 → genus n/d), not an assembly defect.

Updated in ADR-0025 FASE 3 to enforce 4 invariants:

1. **Manifoldness**: every edge has exactly 2 co-edges (extends
   `Edge::is_manifold()` to the whole body).
2. **Orientability (XOR)**: for every edge, exactly one of the two
   co-edges is `reversed()` (logical XOR). This is the invariant that
   distinguishes an orientable closed solid from two faces glued
   back-to-back. A duplicated interface ring (same orientation on both
   co-edges) fails this check immediately.
3. **Loop closure**: `loops_are_closed(body)` — every loop's co-edges
   form a cycle.
4. **Euler characteristic**: `euler_characteristic(body) == 2` —
   single closed shell (topological sphere). A body with χ ≠ 2 is
   either multi-shell or has a topology defect.

**χ-invariance of interface ring duplication**: a duplicated interface
ring adds +4V and +4E (the ring vertices and edges are counted twice).
Since χ = V − E + F, the +4V and +4E cancel and χ remains unchanged.
This means invariant 4 (χ=2) CANNOT detect a duplicated interface
ring — only invariant 2 (XOR) catches it. This is the structural
justification for invariant 2: without it, the test suite would pass
χ=2 on a body with an unwelded interface ring, hiding the bug.

Strict enough to reject:
- periodic-surface sphere/torus (no seam wire → V=E=0)
- two faces glued back-to-back (co-edges same orientation → fails XOR)
- non-connected shells (χ ≠ 2)
- any topology defect from a broken boolean op

## References
- ADR-0021 — B-Rep topology data structure (the parent ADR).
- ADR-0012 — non-manifold topology (radial-edge, the grandparent).
- ADR-0005 — persistent naming (the future history layer that will
  use these query operators to trace subshape genealogy).
- ADR-0003 — third-party policy.
- ADR-0014 — floating-point determinism.
- Weiler, K. (1986). *Topological Structures for Geometric Modeling*.

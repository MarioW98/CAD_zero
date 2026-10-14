# ADR-0025: Proper Greiner-Hormann polygon clipping for boolean ops

## Status
Accepted (2026-10-14) — Phase D, step (c+ fix)

## Context

ADR-0024 shipped `bool_union` and `bool_subtract` via a documented
MVP simplification: fan-triangulation + centroid classification. The
ADR's §2 explicitly said Greiner-Hormann was the proper algorithm and
the fan-triangulation was a "documented MVP simplification".

The simplification had **three correctness bugs** that the test suite
exposed:

1. `bool_union(A, A)` returned an empty body. Every triangle of A had
   its centroid inside B = A, so `clip_outside_convex` discarded all
   of them. The correct result is A itself.

2. Results of `bool_union` / `bool_subtract` on overlapping boxes were
   **not closed manifolds**. Fan-triangulation produced diagonal edges
   that were used by exactly one face (the triangle), violating the
   "every edge has 2 co-edges" contract.

3. The fan-triangulation approach **destroyed the polygon structure**.
   Each quad face became 2 triangles with a diagonal edge, doubling
   the face count and breaking the geometric intent.

These bugs were documented in the test suite as "known limitations"
rather than fixed. That is technical debt, not a finished feature.

This ADR closes the debt by implementing Greiner-Hormann properly.

## Decision

### 1. Algorithm: Greiner-Hormann (1998)

For each pair of faces (F_A from A, F_B from B) whose planes intersect:

1. **Compute the intersection line** of the two face planes. If the
   planes are parallel (no intersection line), skip — the faces don't
   intersect.

2. **Find intersection points**: walk each polygon's edges and find
   where the intersection line crosses each edge. Each crossing
   becomes a new "intersection vertex" inserted into the polygon's
   vertex list at the correct position.

3. **Classify each intersection vertex** as "entry" or "exit":
   - Walk the polygon's edges in order.
   - An intersection vertex is an **entry** if the polygon is going
     from outside to inside the other polygon at that vertex.
   - An intersection vertex is an **exit** if the polygon is going
     from inside to outside.

4. **Walk the result**: starting from an entry vertex on F_A, walk
   F_A's vertices until an exit vertex is reached. At the exit, switch
   to the corresponding intersection vertex on F_B and walk F_B's
   vertices until an entry is reached. Switch back to F_A. Continue
   until the walk returns to the starting vertex. Each closed walk
   produces one result polygon.

5. **Repeat** for each unvisited entry vertex until all intersections
   are processed.

This produces a list of result polygons (convex or non-convex) that
are the fragments of F_A inside F_B, or vice versa.

### 2. Three operations on top of Greiner-Hormann

```
bool_intersect(A, B):
  for each face F_A of A:
    for each face F_B of B that intersects F_A's plane:
      fragments = greiner_hormann(F_A, F_B)
      for each fragment: classify inside/outside B → keep inside
  [same for B's faces against A]
  weld + assemble

bool_union(A, B):
  for each face F_A of A:
    fragments_of_F_A_inside_B = greiner_hormann(F_A, every face of B)
    outer_part = F_A - fragments_of_F_A_inside_B   # polygon subtraction
    add outer_part as a face (with F_A's normal)
  [same for B's faces against A]
  weld + assemble

bool_subtract(A, B):
  for each face F_A of A:
    fragments_of_F_A_inside_B = greiner_hormann(F_A, every face of B)
    outer_part = F_A - fragments_of_F_A_inside_B
    add outer_part as a face (with F_A's normal)
  for each face F_B of B:
    fragments_of_F_B_inside_A = greiner_hormann(F_B, every face of A)
    for each fragment: add as a face with INVERTED normal
  weld + assemble
```

The polygon subtraction `F_A - fragments` is done by Greiner-Hormann
itself: the algorithm produces both the "inside" fragments AND the
"outside" fragments in one pass. The outside fragments are what we
want for union/subtract.

### 3. Special cases

#### Identical bodies (A == B)

`bool_union(A, A)` and `bool_intersect(A, A)` must return A itself.

Detection: compare vertex positions via snapped keys. If all vertices
match (within `kWeldGrid`), the bodies are identical — return a copy
of A.

#### Disjoint bodies

`bool_union(A, B)` for disjoint A and B should ideally return a body
with two separate shells. Our `BRepBody` model has a single shell, so
we **return A unchanged** (the first operand) and document this as a
limitation. A proper fix requires multi-shell bodies (deferred to a
later ADR).

`bool_subtract(A, B)` for disjoint A and B returns A unchanged (B
doesn't cut anything). This is correct and doesn't need special handling.

`bool_intersect(A, B)` for disjoint A and B returns an empty body.
This is correct.

#### B contains A

`bool_subtract(A, B)` where B fully contains A returns an empty body.
Correct.

#### Coplanar faces

Two faces that are coplanar but not coincident: Greiner-Hormann
produces no intersection vertices (the planes don't define an
intersection line — they're the same plane). Each face survives
intact, which is the correct behavior for union (both contribute to
the boundary) and subtract (B's coplanar face becomes an inverted
cap inside A).

Two faces that are coplanar AND coincident (same plane, same boundary):
the deduplication by `SnappedKeySet` from ADR-0023 catches them and
keeps only one.

### 4. Determinism

Greiner-Hormann is deterministic by construction:
- Plane intersection: `dot`, `cross`, `normalize` — deterministic
  under `-ffp-contract=off` (ADR-0014).
- Intersection point computation: `lerp(curr, next, t)` where
  `t = d_curr / (d_curr - d_next)` — deterministic.
- Entry/exit classification: comparisons against `kWeldGrid` (integer
  key after snapping) — deterministic.
- Vertex welding: grid-snapped `SnappedKey` map — order-independent.

### 5. What is explicitly NOT in this ADR

- **Multi-shell bodies** for disjoint unions — deferred.
- **Non-convex polyhedra** — Greiner-Hormann handles non-convex
  polygons, but the point-in-polyhedron test still requires convexity
  (or ray casting for non-convex). The convexity constraint from
  ADR-0023/0024 is kept.
- **Surface geometry derived from topology** (fixing the stale origin
  problem of `get_face_plane`) — separate ADR.
- **Persistent naming** (ADR-0005) — Phase F.
- **Python bindings** — later step.

## Consequences

- `bool_union` and `bool_subtract` are reimplemented using
  Greiner-Hormann. The fan-triangulation helpers (`point_inside_convex`,
  `clip_outside_convex`) are removed.
- `bool_intersect` is also reimplemented to use Greiner-Hormann
  instead of the Sutherland-Hodgman sequential clipping. The result is
  the same for convex inputs, but the code path is unified.
- `clip_polygon` with `keep_inside=false` is removed (it was
  semantically misleading — see ADR-0024 critique).
- Tests are restored to assert `is_closed_manifold` and `χ == 2` on
  union/subtract results.
- The "known limitation" tests for `bool_union(A, A)` are inverted to
  assert the correct behavior (returns A).
- No new third-party dependency (ADR-0003 honored).
- Floating-point determinism (ADR-0014) untouched.

## References
- ADR-0024 — the parent ADR (boolean union/subtract, MVP with
  documented limitations).
- ADR-0023 — analytic seam wires + bool_intersect.
- ADR-0014 — floating-point determinism.
- Greiner, G., & Hormann, K. (1998). "Efficient clipping of arbitrary
  polygons." *ACM Transactions on Graphics*, 17(2), 71–83.
- Vatti, B. R. (1992). "A generic solution to polygon clipping."
  *Communications of the ACM*, 35(7), 56–63. (Alternative algorithm;
  not used but referenced for completeness.)

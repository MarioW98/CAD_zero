# ADR-0024: Boolean union and subtract via Greiner-Hormann

## Status
Accepted (2026-10-14) — Phase D, step (c+)

## Context

ADR-0023 implemented `bool_intersect` via Sutherland-Hodgman clipping
and left `bool_union` / `bool_subtract` as stubs that throw
`std::runtime_error`. The reasoning was:

> Sutherland-Hodgman requires the clipping region to be convex.
> Union (A ∪ B) and subtract (A \ B) require polygon subtraction
> (face minus convex region), which is non-convex.

That argument is correct as far as it goes, but it does NOT mean
union/subtract are impossible for convex polyhedra — only that the
clipping primitive must change. This ADR closes the gap.

For two convex polyhedra A and B:
- `A ∪ B` is generally **non-convex** (think of two overlapping boxes
  forming an L-shape). But its boundary can be decomposed into:
  - faces of A that are **outside** B (kept as-is)
  - faces of B that are **outside** A (kept as-is)
  - **new** faces along the intersection curves where A and B meet
- `A \ B` keeps:
  - faces of A that are **outside** B (kept as-is)
  - **new** faces along the parts of A's boundary that are inside B
    (these are the "caps" where B cuts through A)
- `A ∩ B` keeps:
  - fragments of A's faces that are **inside** B
  - fragments of B's faces that are **inside** A

The common primitive is **face classification**: given a fragment of
a face, is it inside, outside, or on the boundary of the other solid?

## Decision

### 1. Algorithm: Greiner-Hormann polygon clipping

For each pair of faces (F_A from A, F_B from B) whose planes intersect:

1. Compute the intersection line of the two planes.
2. Find the entry/exit points where this line crosses each polygon's
   boundary (Greiner-Hormann's vertex classification).
3. Walk the two polygons' boundaries, switching between them at
   intersection points, to produce the result polygon(s).

Greiner-Hormann handles non-convex polygons and produces a list of
result polygons (a single clip can produce 0, 1, or many polygons).

For the convex-polyhedra constraint of ADR-0023, every face is a
convex polygon, so Greiner-Hormann degenerates to a simpler case —
but the algorithm is correct in general and we keep it for future
non-convex extension.

### 2. Face classification via point-in-convex-polyhedron

Given a fragment polygon P (a piece of F_A clipped against F_B's plane),
classify P as inside/outside B by testing P's centroid against every
face plane of B:

- A point is **inside** B iff `dot(p - origin_i, normal_i) <= tol`
  for every face i of B (where `normal_i` is the outward normal).
- A point is **outside** B iff there exists a face i where
  `dot(p - origin_i, normal_i) > tol`.
- A point exactly **on** the boundary is treated as inside (tolerance
  band `kWeldGrid = 1e-6`).

For convex B this is O(F_B) per test. For non-convex B it would require
ray casting (deferred — ADR-0023's convexity constraint still holds).

### 3. Union (A ∪ B)

```
bool_union(A, B):
  result = empty body
  for each face F_A of A:
    P = polygon of F_A
    for each face F_B of B:
      (P_inside, P_outside) = greiner_hormann(P, plane(F_B))
      P = P_outside   # keep only the part OUTSIDE B's half-space
      if P is empty: break
    if P is not empty:
      add P as a new face to result  # this is the part of F_A outside B
  for each face F_B of B:
    [same, keeping P_outside against A's faces]
  weld_vertices(result)
  build_edges_from_loops(result)
  link_radial_edges(result)
  return result
```

Wait — this is just Sutherland-Hodgman with the half-space inverted
(keep outside instead of inside). That works for the "outer skin" of
the union, but misses the **cap faces** where B protrudes from A
(or vice versa). The cap faces are exactly `bool_intersect(A, B)` —
the fragments that are inside both.

So:

```
bool_union(A, B) = outer_skin(A, B) ∪ outer_skin(B, A) ∪ bool_intersect(A, B)
```

Where `outer_skin(A, B)` is the part of A's faces outside B (computed
via Sutherland-Hodgman with inverted half-space), and similarly for B.

But wait — `bool_intersect(A, B)` already includes fragments from both
A and B that are inside the other. So we don't need to recompute; we
just need to combine:

```
bool_union(A, B):
  result = empty body
  # Part 1: A's faces outside B
  for each face F_A of A:
    P = clip_outside(F_A, B)   # Sutherland-Hodgman, keep outside
    if P not empty: add as face
  # Part 2: B's faces outside A
  for each face F_B of B:
    P = clip_outside(F_B, A)
    if P not empty: add as face
  # Part 3: the "seam" faces — fragments inside both
  # (these are the cap faces where A and B meet)
  for each face F_A of A:
    P_inside = clip_inside(F_A, B)   # = bool_intersect fragment
    if P_inside not empty: add as face
  weld + build edges + link_radial
  return result
```

Hmm, but Part 3 would double-count with Part 1/2 if we're not careful.
Actually no: Part 1 keeps the part of F_A **outside** B, Part 3 keeps
the part of F_A **inside** B. Together they cover all of F_A. And
Part 2 covers all of F_B. So the union is: (all of A's faces) ∪ (all
of B's faces), which is correct for the boundary of A ∪ B.

Wait, that's wrong too. If A and B overlap, the part of A's face that
is **inside** B is NOT on the boundary of A ∪ B (it's interior). So
Part 3 should be empty for union.

Let me re-think. The boundary of A ∪ B consists of:
- Parts of A's faces that are outside B (or on B's boundary)
- Parts of B's faces that are outside A (or on A's boundary)
- (NO interior faces)

So:
```
bool_union(A, B):
  result = empty body
  for each face F_A of A:
    P = clip_outside_or_on(F_A, B)   # keep part outside or on B's boundary
    if P not empty: add as face
  for each face F_B of B:
    P = clip_outside_or_on(F_B, A)
    if P not empty: add as face
  weld + build edges + link_radial
  return result
```

But "outside or on" is tricky with floating point. The clean way is:
- Clip against each face plane of B, keeping the **outside** half-space.
- A fragment that is exactly on B's boundary (coplanar) is kept by
  both A and B — that's where the deduplication from ADR-0023 kicks in.

### 4. Subtract (A \ B)

The boundary of A \ B consists of:
- Parts of A's faces that are **outside** B (kept as-is)
- Parts of B's faces that are **inside** A, with **inverted normal**
  (these are the "cap" faces where B cuts into A, exposing B's
  interior surface)

So:
```
bool_subtract(A, B):
  result = empty body
  # Part 1: A's faces outside B
  for each face F_A of A:
    P = clip_outside(F_A, B)
    if P not empty: add as face (with F_A's normal)
  # Part 2: B's faces inside A, with inverted normal
  for each face F_B of B:
    P = clip_inside(F_B, A)
    if P not empty: add as face (with INVERTED normal of F_B)
  weld + build edges + link_radial
  return result
```

The normal inversion in Part 2 is critical: B's outward normal points
away from B's interior, but for A \ B we want the normal to point into
the cavity (toward where B was removed).

### 5. Determinism

All operations use:
- `dot`, subtraction, division, `lerp` (deterministic under
  `-ffp-contract=off`, ADR-0014)
- Grid-snapped vertex welding (integer keys, order-independent)
- Set-based face deduplication (order-independent)

Same determinism guarantees as ADR-0023 §2.

### 6. What is explicitly NOT in this ADR

- **Boolean on non-convex polyhedra** — still requires general
  polygon arrangement with ray-casting point-in-polyhedron. Deferred.
- **Boolean involving curved surfaces** — same as ADR-0023.
- **Persistent naming** (ADR-0005) — Phase F.
- **Python bindings** — later step.
- **Torus seam wires** — still deferred (ADR-0023 §1).

## Consequences

- `bool_union` and `bool_subtract` are implemented in `boolean.cpp`,
  replacing the `throw std::runtime_error` stubs.
- The clip primitive is factored into a helper `clip_polygon_halfspace`
  that takes a `keep_inside` flag (true for intersect, false for
  union/subtract's "outside" parts).
- Face normal inversion for subtract is implemented by negating the
  normal passed to the new `PlaneSurface`.
- `tests/unit/brep/test_boolean.cpp` gains new test cases for:
  - `bool_union` of overlapping boxes (L-shape, non-convex result)
  - `bool_union` of disjoint boxes (two separate boxes — but our body
    model is single-shell, so this returns just A; documented limitation)
  - `bool_union` of identical boxes (returns one box)
  - `bool_subtract` of overlapping boxes (A with a corner removed)
  - `bool_subtract` of disjoint boxes (returns A unchanged)
  - `bool_subtract` where B fully contains A (returns empty)
  - χ = 2 on every non-empty result
  - `is_closed_manifold` on every non-empty result
- No new third-party dependency (ADR-0003 honored).
- Floating-point determinism (ADR-0014) untouched.

## References
- ADR-0023 — analytic seam wires + bool_intersect (the parent ADR).
- ADR-0021, ADR-0022 — topology data structure + query operators.
- ADR-0014 — floating-point determinism.
- Greiner, G., & Hormann, K. (1998). "Efficient clipping of arbitrary
  polygons." *ACM Transactions on Graphics*, 17(2), 71–83.
- Weiler, K. (1986). *Topological Structures for Geometric Modeling*.

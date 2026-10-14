# ADR-0025: Proper Greiner-Hormann with collinear edge handling

## Status
Accepted (2026-10-14) — Phase D, step (c+ fix v2)

## Context

The first version of this ADR promised Greiner-Hormann (1998) for
boolean ops, but the implementation had two runtime bugs that surfaced
only when tests ran:

1. **`bool_union(A, A)` returned empty.** Every triangle of A had its
   centroid inside B = A, so all were discarded.
2. **Crashes on partially overlapping boxes.** SIGSEGV in `walk_results`
   when polygon edges were collinear.

The root cause for both was the same: **Greiner-Hormann does not handle
collinear edges**. When two polygon edges lie on the same line, the
segment-intersection test (`denom = dx1*dy2 - dy1*dx2`) returns 0 →
"parallel, no intersection" — but the edges may still touch at a point
or overlap along a sub-segment.

This is a known limitation documented in the GH paper itself (§5 of
Greiner & Hormann 1998) and in the survey by Foster et al. (2019).
Three remedies exist in the literature:

- **Symbolic perturbation (SOS)**: perturb vertices by infinitesimal
  symbolic amounts to break collinearity. Deterministic but complex.
- **On-boundary classification**: extend `point_in_polygon` to return
  three states (inside / outside / on-boundary) and handle the
  on-boundary case explicitly.
- **Weiler-Hormann with edge subdivision (Liu et al. 2007)**: pre-pass
  that splits collinear edges at common endpoints.

This ADR picks **on-boundary classification + identical-body shortcut
+ coplanar-face special case**. Rationale:

- The identical-body shortcut (`bodies_identical()`) handles the most
  common collinear case: A == B → all 6 face pairs are coplanar and
  coincident.
- The coplanar-face special case skips GH for coplanar face pairs and
  handles them by direct set comparison (deduplicate if coincident,
  skip if disjoint, both kept if partially overlapping).
- For non-coplanar faces, GH with on-boundary classification handles
  vertex-on-edge and edge-touching cases correctly.

## Decision

### 1. Three-state point-in-polygon

`point_in_polygon` returns an enum:

```cpp
enum class PointClass { Inside, Outside, OnBoundary };
PointClass classify_point(Vec2 p, const Polygon2D& poly);
```

A point is `OnBoundary` if it lies on any edge of the polygon (within
`kWeldGrid` tolerance). Inside/outside uses the even-odd ray-casting
rule for points strictly inside or strictly outside.

### 2. Collinear edge handling in segment_intersection

`segment_intersection` returns the intersection **type**, not just a
boolean:

```cpp
enum class IntersectionType { None, Point, Overlap };
IntersectionType segment_intersection(
    Vec2 p1, Vec2 p2, Vec2 p3, Vec2 p4,
    float& t, float& u
);
```

- `None`: parallel and non-collinear, or non-intersecting.
- `Point`: classic single-point intersection (the common case).
- `Overlap`: collinear edges that share a sub-segment. Returns the
  range `[t_min, t_max]` along the first edge where the overlap occurs.

For the `Overlap` case, GH inserts **two** intersection vertices (the
endpoints of the overlap range) instead of one. This correctly
partitions the polygon at the start and end of the collinear overlap.

### 3. Coplanar face special case

Before running GH on a face pair `(F_A, F_B)`, check if they are
coplanar (normals parallel, vertex of B on A's plane). If yes:

- **Coincident (same vertex set)**: skip entirely — the face is a
  duplicate and will be deduplicated by `assemble_body`'s set
  comparison.
- **Disjoint (bounding boxes don't intersect)**: skip entirely — the
  faces don't contribute to the boolean result.
- **Partially overlapping (bounding boxes intersect but vertex sets
  differ)**: this is the hard case. For MVP we keep both faces intact
  and let the deduplication catch exact duplicates. Partial overlap is
  a documented limitation that requires true edge subdivision
  (deferred).

### 4. Identical-body shortcut (kept from previous ADR version)

`bodies_identical(A, B)` compares vertex sets via `SnappedKeySet`. If
identical:
- `bool_union(A, A)` returns a copy of A.
- `bool_subtract(A, A)` returns empty.
- `bool_intersect(A, A)` returns a copy of A.

### 5. What is explicitly NOT in this ADR

- **Symbolic perturbation (SOS)** — too complex for MVP, on-boundary
  classification suffices for the test cases.
- **Full edge subdivision for partial overlaps** — deferred.
- **Non-convex polyhedra** — still requires general point-in-polyhedron
  (ray casting). Convexity constraint kept.
- **Surface geometry derived from topology** — separate ADR.

## Consequences

- `point_in_polygon` is replaced by `classify_point` (three-state).
- `segment_intersection` returns `IntersectionType` (None/Point/Overlap)
  and handles the `Overlap` case by inserting two intersection vertices.
- `greiner_hormann_2d` is reactivated (no longer `[[maybe_unused]]`)
  and used by all three boolean operations.
- `bool_intersect` uses GH for non-coplanar face pairs, falls back to
  Sutherland-Hodgman sequential clipping for the general case (which
  is equivalent for convex inputs but simpler).
- `bool_union` and `bool_subtract` use GH with the coplanar-face
  special case.
- Tests are strengthened: `is_closed_manifold` and `χ == 2` are
  asserted on union/subtract results for overlapping, identical, and
  nested box cases.

## References
- ADR-0024 — boolean union/subtract (parent).
- ADR-0023 — analytic seam wires + bool_intersect.
- ADR-0014 — floating-point determinism.
- Greiner, G., & Hormann, K. (1998). "Efficient clipping of arbitrary
  polygons." *ACM TOG*, 17(2).
- Foster, P., Hormann, K., & Popa, R. T. (2019). "Survey: Clipping
  of arbitrary polygons by a rectangular window." *Visual Computer*.
- Liu, Y. K., Wang, X. Q., Bao, S. Z., Gomboši, M., & Žalik, B.
  (2007). "An algorithm for polygon clipping, and for determining
  polygon intersections and unions." *Computers & Geosciences*, 33(8).

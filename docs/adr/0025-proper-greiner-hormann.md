# ADR-0025: Proper Greiner-Hormann with collinear edge handling

## Status
Accepted (2026-10-14) — Phase D, step (c+ fix v3)

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
+ coplanar-face special case + polygon_subtract_3d for bool_union**. Rationale:

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

### 5. polygon_subtract_3d — the bool_union primitive

The key insight for `bool_union`: the boundary of A ∪ B consists of
the parts of A's faces that are OUTSIDE B, plus the parts of B's faces
that are OUTSIDE A.

For each face F_A:
1. `inside = clip_inside_3d(B, F_A)` — Sutherland-Hodgman, convex-safe.
2. If `inside` is empty → F_A survives intact (entirely outside B).
3. If `inside == F_A` (vertex sets equal) → F_A skipped (entirely inside B).
4. Otherwise → `outside = polygon_subtract_3d(F_A, inside, normal)`
   via Greiner-Hormann in 2D (projected onto F_A's plane).

`polygon_subtract_3d` uses `greiner_hormann_2d` and returns the
`outside` fragments. The 2D projection + GH + 3D unprojection pipeline
handles collinear edges via the `IntersectionType::Overlap` case.

### 6. IntersectionTable — single source of truth for interface points

**Problem**: in partial-overlap booleans, the interface ring vertices
(points where B's face planes intersect A's edges, and vice versa) are
computed by TWO different FP paths:
- Part 1: `clip_inside_3d(B, F_A)` — Sutherland-Hodgman clips A's face
  against B's planes.
- Part 2: `clip_inside_3d(A, F_B)` — Sutherland-Hodgman clips B's face
  against A's planes.

Both paths compute the same geometric points (e.g. the point at x=0.3
where B's boundary clips A), but with ~ULP FP divergence due to
different operation orders. When the position straddles a grid cell
boundary (e.g. x=0.3 where 0.3/1e-6 = 299999.9999...), the
`SnappedKey` differs and welding fails.

**Solution**: `IntersectionTable` pre-computes each interface point
ONCE. Both Part 1 and Part 2 consume the SAME `Vec3f` instance from
the table — bit-identical by construction. The welding in
`assemble_body` degenerates to an identity lookup.

**Architecture**:
- `SharedVertexPool`: maps `SnappedKey` → canonical `Vec3f`. First
  insertion wins; subsequent lookups for the same cell return the
  stored `Vec3f`.
- `IntersectionTable`: wraps `SharedVertexPool`. `get_or_compute(v0,
  v1, plane_origin, plane_normal)` computes the edge-plane intersection
  point ONCE and canonicalizes it through the pool.
- `clip_polygon_inside`: accepts an optional `IntersectionTable*`.
  When non-null, intersection points are read from the table instead
  of computed locally.
- `clip_inside_3d`: accepts the table and passes it through.
- `bool_union`/`bool_subtract`: create ONE `IntersectionTable` and
  pass it to ALL `clip_inside_3d` calls. Both Part 1 and Part 2 share
  the same table → same `Vec3f` instances.
- `assemble_body`: canonicalizes all clipped face polygons through a
  `SharedVertexPool` before welding. Applied uniformly to all boolean
  ops (behavior-neutral for original vertices, which are already
  bit-identical from source meshes).

**Why not just increase kWeldGrid?** That would mask the bug, not fix
it. The root cause is dual-computation of the same geometric point by
different FP paths. The fix is architectural: compute once, share
everywhere. kWeldGrid stays at 1e-6.

**Known limitation**: `bool_subtract` does not yet generate "cut faces"
— the new faces created where B's clipping plane intersects A's
interior (e.g. the face at x=0.3 when subtracting a shifted box).
Part 1 discards A's faces that are entirely inside B. Part 2 adds B's
faces that are inside A as caps with inverted normals. But the face
at the cut plane is neither — it's a NEW face. Implementing cut-face
generation is the next step (ADR-0025 v4 TODO).

### 7. What is explicitly NOT in this ADR

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

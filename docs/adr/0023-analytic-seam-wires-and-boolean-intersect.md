# ADR-0023: Analytic seam wires + boolean intersect on convex polyhedra

## Status
Accepted (2026-10-14) — Phase D, step (c)

## Context

ADR-0021 and ADR-0022 shipped a topology data structure and query
operators that pass all tests on the box and tetrahedron factories.
However, two problems were carried forward and left as documented
limitations:

1. **`make_sphere` and `make_cylinder` lie.** Both call
   `shell.set_closed(true)`, but neither is a closed manifold — they
   represent periodic surfaces as a single face with no seam wire,
   zero vertices, and zero edges. ADR-0022 §5 added an
   `is_closed_manifold` regression guard that returns `false` for
   these bodies, which protected the invariant but did not fix the
   root cause. The factories still claim `is_closed = true` in the
   shell flag, which is a lie that downstream code could trust.

2. **No boolean operations.** The kernel can build and query topology
   but cannot combine two bodies. The dual-representation vision
   (ADR-0002: SDF and B-Rep as peers) requires that B-Rep eventually
   support the same CSG operations that SDF already has. Phase D step
   (c) is the first step toward that.

## Decision

### 1. Seam wires for analytic surfaces

A periodic surface (sphere, cylinder, torus) becomes a valid closed
manifold by introducing a **seam wire**: a set of vertices and edges
that "cut" the periodic parametrization, turning the single face into
a proper topological disk whose boundary is a closed loop of co-edges.

#### Sphere (2V / 1E / 1F, χ = 2)

```
V = {v_north = (0, +R, 0), v_south = (0, -R, 0)}
E = {e_seam: v_north → v_south}   (one meridian edge)
F = {f_sphere: SphereSurface}
Loop of f_sphere = [e_seam forward, e_seam reversed]
```

The seam edge is used twice by the same face — once forward (north→south)
and once reversed (south→north). Both co-edges are on the same face;
`Edge::coedges().size() == 2`, so `is_manifold()` returns `true`.
`faces_around_edge` returns the same face twice for this edge — a
documented consequence of seam edges that downstream code must handle
(by deduplicating if it expects two distinct faces).

#### Cylinder (4V / 5E / 3F, χ = 2)

```
V = {v_top1 = (+R, +h/2, 0), v_top2 = (-R, +h/2, 0),
     v_bot1 = (+R, -h/2, 0), v_bot2 = (-R, -h/2, 0)}
E = {e_top1: v_top1→v_top2,  e_top2: v_top2→v_top1,   (top rim arcs)
     e_bot1: v_bot1→v_bot2,  e_bot2: v_bot2→v_bot1,     (bottom rim arcs)
     e_seam: v_bot1→v_top1}                              (vertical seam)
F = {f_lateral: CylinderSurface,
     f_top:     PlaneSurface (normal +Y),
     f_bot:     PlaneSurface (normal -Y)}
```

- Lateral loop (6 co-edges):
  `[e_bot1 rev, e_bot2 rev, e_seam fwd, e_top2 rev, e_top1 rev, e_seam rev]`
- Top cap loop: `[e_top1 fwd, e_top2 fwd]`
- Bottom cap loop: `[e_bot1 fwd, e_bot2 fwd]`

Every edge has exactly 2 co-edges; every edge has opposite coedge
orientations. The seam edge `e_seam` has both co-edges on the lateral
face (same pattern as the sphere seam).

#### Torus — deferred

The torus requires **two** seam wires (one around the major loop, one
around the minor loop) and a more complex vertex layout. It is
deferred to a later step; `make_torus` (if it exists in the future)
will continue to be a non-closed-manifold until then.

### 2. Boolean intersect on convex polyhedra

#### Algorithm: Sutherland-Hodgman clipping

For two convex polyhedra A and B, the intersection `A ∩ B` is computed
by clipping each face of A against the half-spaces of B (and vice
versa):

```
bool_intersect(A, B):
  result = empty body
  for each face F_A of A:
    polygon P = vertices of F_A's outer loop
    for each face F_B of B:
      P = clip(P, plane of F_B)   # keep vertices inside F_B's half-space
      if P is empty: break
    if P is not empty:
      add P as a new face to result (with F_A's surface geometry)
  for each face F_B of B:
    [same, clipping against A's faces]
  weld_vertices(result)
  build_edges_from_loops(result)
  link_radial_edges(result)
  return result
```

The clip function is standard Sutherland-Hodgman:
- For each edge of the polygon, classify both endpoints against the
  plane (inside / outside / on).
- If the current vertex is inside, keep it.
- If the current and next vertices are on different sides, compute the
  intersection point and keep it.

This produces convex polygon fragments. Each fragment becomes a face
of the result body.

#### Convexity requirement

Sutherland-Hodgman works **only when the clipping region is convex**
(B must be convex). It also requires the input polygon to be convex
(each face of A must be convex). Both box and tetrahedron satisfy
this. For non-convex inputs, the algorithm produces incorrect results
and must throw `std::invalid_argument`.

#### Vertex welding — grid snapping for determinism

Intersection points are computed via `lerp(curr, next, t)` where
`t = d_curr / (d_curr - d_next)`. The same physical point computed
from two different faces can have slightly different floating-point
results. Exact comparison would fail to weld them.

**Solution: grid snapping.** Round each coordinate to the nearest
multiple of `weld_tolerance` (default 1e-6), then use the snapped
triple as a key in `std::map`. This is:
- Deterministic (same inputs → same snapped keys, same weld result).
- Transitive (if A snaps to the same grid point as B, and B snaps to
  the same grid point as C, then A and C snap to the same grid point).
- Independent of insertion order (the map key is the snapped position,
  not the insertion index).

With `-ffp-contract=off` (ADR-0014), the FP operations are
reproducible across platforms, so the welding is reproducible.

#### What is explicitly NOT implemented

- **`bool_union` and `bool_subtract`** — declared in `boolean.hpp`
  but throw `std::runtime_error("not implemented")`. These require
  polygon subtraction (face minus convex region), which is
  non-convex and cannot use Sutherland-Hodgman. A proper
  implementation needs a polygon arrangement algorithm, which is a
  separate ADR (deferred to Phase D step (c+) or Phase E).

- **Boolean on non-convex polyhedra** — Sutherland-Hodgman requires
  convex inputs. Non-convex boolean needs a general arrangement
  (Bentley-Ottmann or similar), deferred.

- **Boolean involving curved surfaces** (sphere ∩ box, cylinder ∪
  tetrahedron) — the clip function operates on planar polygons only.
  Curved-surface boolean needs surface-surface intersection curves,
  deferred to a much later step.

- **Persistent naming** (ADR-0005 `ShapeId` propagation through
  boolean ops) — Phase F.

- **Python bindings** — added in a later step.

## Consequences

- `make_sphere` is rewritten: 2V / 1E / 1F, seam edge used twice by
  the single face. `is_closed_manifold(make_sphere(R))` now returns
  `true`. The ADR-0022 regression guard test
  `is_closed_manifold(sphere) == false` is **inverted** to `== true`.

- `make_cylinder` is rewritten: 4V / 5E / 3F, seam edge used twice
  by the lateral face. `is_closed_manifold(make_cylinder(R, H))` now
  returns `true`. The ADR-0022 regression guard test
  `is_closed_manifold(cylinder) == false` is **inverted** to `== true`.

- New file `core/brep/include/CAD_0/brep/boolean.hpp` declares
  `bool_intersect`, `bool_union`, `bool_subtract`.

- New file `core/brep/src/boolean.cpp` implements `bool_intersect`
  via Sutherland-Hodgman. `bool_union` and `bool_subtract` throw
  `std::runtime_error`.

- `core/brep/CMakeLists.txt` adds `src/boolean.cpp`.

- New test file `tests/unit/brep/test_boolean.cpp` exercises:
  - box ∩ box (overlapping, disjoint, nested)
  - tetrahedron ∩ box
  - χ = 2 on every result
  - `is_closed_manifold` on every result
  - `bool_union` / `bool_subtract` throw as documented

- Floating-point determinism (ADR-0014) is honored: the clip
  computation uses only `dot`, subtraction, division, and `lerp` —
  all deterministic under `-ffp-contract=off`. The grid snapping for
  welding is integer-keyed (deterministic).

- No new third-party dependency (ADR-0003 honored).

## References

- ADR-0021 — B-Rep topology data structure.
- ADR-0022 — B-Rep topology query operators.
- ADR-0012 — non-manifold topology (radial-edge).
- ADR-0014 — floating-point determinism.
- ADR-0016 — vertex welding via canonical edge keys (the mesh
  equivalent of the grid snapping used here).
- Sutherland, I. E., & Hodgman, G. W. (1974). "Reentrant Polygon
  Clipping." *Communications of the ACM*, 17(1).
- Weiler, K. (1986). *Topological Structures for Geometric Modeling*.

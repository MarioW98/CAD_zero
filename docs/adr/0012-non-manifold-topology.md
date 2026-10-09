# ADR-0012: Non-manifold B-Rep topology (radial-edge)

## Status
Accepted (2025-10-07)

## Context
The original proposal listed `core/brep/halfedge.hpp`. Half-edge
data structures assume 2-manifold solids: every edge has exactly
two incident faces. This is too restrictive for real CAD:

- Boolean operations on coincident faces create non-manifold edges
  (3+ faces around one edge).
- Wire bodies (curves without faces) appear in imported STEP files.
- Sheet bodies (single-face walls) are common in sheet metal.
- Fillet/chamfer operations can momentarily produce non-manifold
  intermediate states that need to be healed before the result is
  a clean solid.

## Decision
The B-Rep kernel uses a **radial-edge** structure inspired by
Weiler (1986) and used by ACIS. Every edge carries a list of
co-edges, one per incident face; the list is radially ordered.

```cpp
struct Edge {
    VertexId v[2];
    std::vector<CoEdge*> coedges_radial_;  // can be > 2
    CurveGeomId geom;
    Orientation orient;
};
```

### Why not TopoDS-style?
OpenCascade's `TopoDS_Shape` uses a single inheritance hierarchy with
orientation flags. It is more memory-efficient but harder to reason
about (orientation is implicit in the cast direction). The
radial-edge structure is more verbose but explicit.

## Consequences
- `core/brep/halfedge.hpp` is renamed to `core/brep/topology.hpp` and
  does NOT use half-edge. The original filename was a misnomer.
- Memory cost: a typical manifold solid pays ~16 bytes per edge extra
  (one vector header) vs a half-edge structure. Acceptable for MVP.
- Phase D (B-Rep MVP) ships with this structure; no migration path
  from a half-edge predecessor is needed.

## References
- Weiler, K. (1986). *Topological Structures for Geometric Modeling*.
  PhD thesis, Rensselaer Polytechnic Institute.

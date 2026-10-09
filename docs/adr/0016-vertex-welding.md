# ADR-0016: Vertex welding via canonical edge keys

## Status
Accepted (2025-10-07) — Phase B.5

## Context
Phase B emitted 3 vertices per triangle without deduplication. Each edge
in the marching cubes grid is shared by up to 4 neighbouring cells, so
Phase B's output had ~4x the number of vertices actually needed:

  Phase B  → 285k vertices for a 64³ lattice ball mesh (99k triangles)
  Phase B.5 →  77k vertices for the same mesh (74% reduction)

The waste has downstream effects:
- Larger STL/OBJ/3MF files (~4x)
- Slower consumer-side processing (slicer, viewer, FEM mesher)
- Higher memory footprint for in-memory mesh storage

## Decision
**Canonical edge keys**: each of the 12 marching-cubes edges is associated
with a *canonical (cell, edge_idx) representation*. The canonical cell is
the one whose origin (corner 0) coincides with the smaller-coordinate
endpoint of the edge.

### Edge anchor table
For each of the 12 cube edges, we precompute:
- The anchor cell offset (dx, dy, dz) relative to the cell generating
  the triangle.
- The edge index in that anchor cell.

```
Edge 0 (along +X, corner 0-1)     → anchored at (cx,    cy, cz), edge 0
Edge 1 (along +Y, corner 1-2)     → anchored at (cx,    cy, cz), edge 1
Edge 2 (along +X, corner 2-3)     → anchored at (cx-1,  cy, cz), edge 1
Edge 3 (along +Y, corner 3-0)     → anchored at (cx,    cy, cz), edge 3
Edge 4 (along +X, corner 4-5)     → anchored at (cx,    cy, cz), edge 4
Edge 5 (along +Y, corner 5-6)     → anchored at (cx,    cy, cz), edge 5
Edge 6 (along +X, corner 6-7)     → anchored at (cx-1,  cy, cz), edge 5
Edge 7 (along +Y, corner 7-4)     → anchored at (cx,    cy, cz), edge 7
Edge 8 (along +Z, corner 0-4)     → anchored at (cx,    cy, cz), edge 8
Edge 9 (along +Z, corner 1-5)     → anchored at (cx,    cy, cz), edge 9
Edge 10 (along +Z, corner 2-6)    → anchored at (cx, cy-1, cz), edge 9
Edge 11 (along +Z, corner 3-7)    → anchored at (cx,    cy, cz), edge 11
```

### Hash key
We pack `(acx, acy, acz, edge_idx)` into a 64-bit key:
- 16 bits per coordinate (signed range ±32k, enough for resolutions up to 32k)
- 4 bits for edge_idx

The hash table maps this key to the vertex index assigned on first
encounter. Subsequent encounters return the cached index without
allocating a new vertex.

### Fallback for negative-coord anchor cells
When the original cell is at `(0, *, *)` and the edge anchor is at
`(-1, *, *)`, we can't read grid values from the negative cell (out of
bounds). Instead, we compute the vertex using the original cell's local
edge — the physical edge is the same, just indexed differently.

## Implementation

```cpp
auto get_vertex = [&](cx, cy, cz, local_edge) -> std::uint32_t {
    const EdgeAnchor& a = kEdgeAnchors[local_edge];
    const auto acx = cx + a.dx, acy = cy + a.dy, acz = cz + a.dz;
    const auto aedge = a.edge_idx;
    const auto key = (acx & 0xFFFF) | ((acy & 0xFFFF) << 16)
                   | ((acz & 0xFFFF) << 32) | (aedge << 48);

    if (auto it = edge_cache.find(key); it != edge_cache.end()) {
        return it->second;  // already created
    }

    // Compute the vertex position. If the anchor cell == original cell,
    // use the anchor edge. Otherwise fall back to the local edge
    // (same physical edge, different (cell, edge_idx) representation).
    math::Vec3f pos = (a.dx == 0 && a.dy == 0 && a.dz == 0)
        ? interp_edge(g, cx, cy, cz, kEdgeCorners[aedge].first, kEdgeCorners[aedge].second)
        : interp_edge(g, cx, cy, cz, kEdgeCorners[local_edge].first, kEdgeCorners[local_edge].second);

    const auto vid = static_cast<std::uint32_t>(mesh.positions.size());
    mesh.positions.push_back(pos);
    if (compute_normals) mesh.normals.push_back(/* interpolated gradient */);
    edge_cache[key] = vid;
    return vid;
};
```

## Performance
Measured on the `examples/cpp/sphere_to_stl.cpp` lattice ball (sphere r=2
minus 3 orthogonal cylinders), resolution 64:

| Metric | Phase B | Phase B.5 | Change |
|---|---|---|---|
| Vertices | 297,345 | 76,653 | -74% |
| Triangles | 99,115 | 95,186 | -4% (narrow-band culling skips more cells) |
| Extraction time | ~80 ms | 44 ms | -45% |
| STL file size | 4.6 MB | 4.6 MB | unchanged (STL is per-triangle, not per-vertex) |
| OBJ file size | ~24 MB | 9.9 MB | -59% |
| 3MF file size | ~28 MB | 11 MB | -61% |

The OBJ and 3MF formats benefit the most because they explicitly store
per-vertex normals (when `compute_normals=true`); fewer vertices →
fewer normals → smaller files.

## Consequences
- `MarchingCubesOptions.weld_vertices` defaults to `true`.
- The public `weld_vertices(mesh, tolerance)` utility can be called on
  any existing mesh (e.g. loaded from a non-welded source).
- The edge anchor table is fixed (12 entries) — no runtime overhead
  beyond one hash table lookup per vertex.

## Future work (Phase C+)
- **Dual Contouring** for sharp feature preservation (deferred)
- **GPU marching cubes** with shared-memory vertex deduplication
- **Multi-threaded cell traversal** — currently single-threaded, but
  the per-cell work is now O(1) thanks to the hash lookup

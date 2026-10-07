# ADR-0015: Marching Cubes for SDF mesh extraction

## Status
Accepted (2025-10-07) — Phase B.

## Context
We need to extract a triangle mesh from an SDF body for:
- Visualization (ray-marching is good for live preview, but a static mesh
  is needed for export and offline rendering)
- Export to STL/3MF/OBJ for 3D printing and CAM
- As input to the future SDF ↔ B-Rep bridge (Phase E)

The standard algorithms are:
1. **Marching Cubes** (Lorensen & Cline 1987) — uniform grid, 256-entry
   edge table, linear interpolation along edges
2. **Dual Contouring** (Ju et al. 2002) — adaptive, preserves sharp
   features by placing vertices inside cells at the intersection of the
   surface and edge crossing points
3. **SurfaceNets** — similar to dual contouring but smoother and simpler

## Decision
**Marching Cubes** is the Phase B mesh extractor. We use the standard
256-entry edge table with linear vertex interpolation along edges.

### Implementation notes
- **Grid evaluation**: we evaluate the SDF at all grid points first using
  `CpuEvalBackend::evaluate` (which uses TBB parallelism if available),
  then run marching cubes single-threaded over the resulting scalar field.
  For very large grids, the cell traversal could also be parallelized, but
  we leave that to Phase B.5.
- **Vertex deduplication**: in Phase B each cell emits its own vertices
  (no deduplication across cells). A typical 64^3 extraction produces
  ~285k vertices, of which ~90% are duplicates of vertices from
  neighbouring cells. A spatial-hash based welder will be added in
  Phase B.5 to deduplicate.
- **Normal computation**: per-vertex normals are computed by interpolating
  the SDF gradient (∇f) along edges. This is more accurate than computing
  per-face normals and averaging, because the gradient is the true
  surface normal for SDF fields.
- **Sharp feature preservation**: NOT supported in Phase B. Dual Contouring
  would be required to preserve sharp features (e.g. cube edges). This is
  deferred to Phase B.5 (post-MVP enhancement).

### Resolution guidance
- `res=16`: rough preview, ~1k triangles per primitive
- `res=32`: smooth visualization, ~6k triangles
- `res=64`: 3D-print quality, ~30k triangles
- `res=128`: high-detail export, ~120k triangles
- Memory cost: `(res+1)^3` evaluations × 16 bytes (position + value +
  gradient) = ~30 MB at res=128

## Consequences
- Phase B ships `marching_cubes()` returning a `TriangleMesh`.
- The STL exporter (`core/io/stl.hpp`) consumes `TriangleMesh` directly.
- Phase E (bridge to B-Rep) will use the same `TriangleMesh` as input
  to the `sdf_to_brep` converter.
- Phase B.5 will add vertex deduplication and possibly Dual Contouring.

## Future work (Phase B.5 / post-MVP)
1. **Vertex deduplication** via spatial hashing — reduce vertex count by ~90%
2. **Adaptive resolution** — coarse far from the surface, fine near it
3. **Dual Contouring** — preserve sharp features for CAD-grade geometry
4. **GPU marching cubes** — via the `GpuEvalBackend` stub

## References
- Lorensen, W. & Cline, H. (1987). *Marching Cubes: A High Resolution
  3D Surface Construction Algorithm*. SIGGRAPH.
- Ju, T., Losasso, F., Schaefer, S., & Warren, J. (2002). *Dual
  Contouring of Hermite Data*. SIGGRAPH.
- Inigo Quilez — https://iquilezles.org/articles/marchingcubes/

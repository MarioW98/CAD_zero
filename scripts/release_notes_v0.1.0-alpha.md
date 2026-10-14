# CAD_0 v0.1.0-alpha

First tagged release of CAD_0 — a dual-representation CAD kernel (native SDF + B-Rep) in C++20 and Python, with no third-party CAD kernel dependency.

## Scope of this alpha

Phases A → D (steps a, b, c) complete. This release is a developer preview: the SDF kernel, marching-cubes mesh extraction, native `.CAD_0` project format (v0.2.0), STL/OBJ/3MF I/O, the PySide6 desktop app with viewport picking, and the B-Rep topology kernel with radial-edge structure, query operators, seam-wired analytic surfaces, and boolean intersect on convex polyhedra are all working and tested.

**What is NOT in this release:**
- `bool_union` and `bool_subtract` (declared, throw `std::runtime_error` — require polygon arrangement, deferred to Phase D step (c+)).
- Boolean on non-convex polyhedra or curved surfaces (deferred).
- STEP import/export (ADR-0010 — Phase H).
- Persistent naming for subshape genealogy (ADR-0005 — Phase F).
- SDF ↔ B-Rep bridge (Phase E).
- Python bindings for the B-Rep topology layer (added once the C++ surface is stable).

## Highlights

### SDF kernel (Phases A, B)
- **Native SDF** — primitives (sphere/box/cylinder/torus via unified builder registry), CSG booleans, smooth blends, transforms, batched SIMD evaluation (xsimd, multi-threaded).
- **Marching cubes** (ADR-0015) with vertex welding (ADR-0016).
- **Native `.CAD_0` format v0.2.0** (ADR-0007/0007a) — JSON feature tree + binary blob container, atomic save (tmp + fsync + rename), transactional load, migrator chain (0.1.0 → 0.2.0), bit-exact round-trip.
- **STL/OBJ import** (welded, first-class mesh nodes) + **STL/OBJ/3MF export**.

### Desktop app (Phase C+)
- **PySide6 app** — in-window menubar, maximized startup, GL viewport (orbit/pan/zoom), feature tree, parametric property panel, undo/redo (CommandStack: create/delete/edit), viewport picking with amber 3D highlight (depth LEQUAL), mesh-node picking via AABB.
- **Determinism contract** (ADR-0014): `-ffp-contract=off`, bit-identical round-trip tests, byte-identical golden files.
- **Headless-first testing** (ADR-0017): the full UI suite runs under Xvfb in CI.
- **Toolchain contract**: warning audits reproduce only against gcc 11.4 / clang 14, the CI compilers.

### B-Rep kernel (Phase D, steps a–c)
- **ADR-0021 — topology data structure**: radial-edge entity set (Vertex/Edge/CoEdge/Loop/Face/Shell/Solid/BRepBody), per-body ID scheme, non-owning radial-edge ownership, `link_radial_edges()` post-pass, Euler invariant (χ = V − E + F = 2), loop closure check.
- **ADR-0022 — query operators**: `faces_around_edge`, `edges_around_vertex`, `faces_around_vertex`, `opposite_coedge`, `is_closed_manifold`, `wireframe_edges`, `vertex_valence`. Plus `make_tetrahedron` as the second canonical closed-solid case. Factories now produce consistently oriented shells (every edge has opposite coedge orientations).
- **ADR-0023 — seam wires + boolean intersect**: `make_sphere` (2V/1E/1F, seam edge used twice by the single face) and `make_cylinder` (4V/5E/3F, seam edge on the lateral face) are now genuine closed manifolds. `bool_intersect` on convex polyhedra via Sutherland-Hodgman clipping with deterministic grid-snapped vertex welding (1e-6 resolution, integer keys, order-independent).

## CI matrix (all green on `main`)

- GCC 11 debug with AddressSanitizer (C++ only — ASan extension module can't be imported by a plain `python3`)
- GCC 11 release
- Clang 14

`fail-fast` is off: one red job never hides the others. The matrix is the diagnosis tool.

## Known limitations

- `bool_union` / `bool_subtract` throw `std::runtime_error` (Phase D step (c+)).
- Torus B-Rep factory not yet implemented (requires two seam wires — deferred).
- Surface-surface intersection curves not implemented (deferred to a later step).
- Software GL only in Codespaces (`LIBGL_ALWAYS_SOFTWARE=1`); hardware GL works locally.
- GIF demo not yet embedded in README — the `scripts/record_demo.py` recorder is included in this release; the GIF itself lands with the first Codespaces run.

## Architecture decisions recorded (ADR corpus)

ADR-0001 through ADR-0023 in `docs/adr/`. See README for the full table.

## How to try it

Open in GitHub Codespaces and run `bash scripts/start_codespaces.sh`. Or build locally — see README.

```bash
git clone https://github.com/MarioW98/CAD_zero.git
cd CAD_zero
cmake --preset linux-gcc-release
cmake --build --preset linux-gcc-release
ctest --preset linux-gcc-release
```

## How to record the demo GIF

```bash
# After starting the Codespace app (bash scripts/start_codespaces.sh),
# in a separate terminal:
python3 scripts/record_demo.py
# Produces docs/media/demo.gif (30–60s, ≤5 MB)
```

## What's next

- **Phase D step (c+)** — `bool_union` / `bool_subtract` via polygon arrangement.
- **Phase E** — SDF ↔ B-Rep bridge (SDF CSG results converted to B-Rep topology).
- **Phase F** — modeling: feature history, sketcher, persistent naming (ADR-0005).
- **Phase H** — STEP AP203 MVP (ADR-0010).

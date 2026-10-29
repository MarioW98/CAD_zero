# CAD_0

[![CI](https://github.com/MarioW98/CAD_zero/actions/workflows/ci-linux.yml/badge.svg)](https://github.com/MarioW98/CAD_zero/actions/workflows/ci-linux.yml)
[![License: Apache-2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

**Dual-representation CAD kernel — native SDF + B-Rep, in C++20 and Python.**

CAD_0 is a from-scratch CAD kernel where **SDF (signed distance fields) and B-Rep (boundary representation) are first-class peers**. There is no "primary" representation: every operation has a native implementation in both domains, and the system dispatches on operand types.

The long-term goal is an **open-source CAD system built without any third-party CAD kernel** ([ADR-0003](docs/adr/0003-third-party-policy.md)), so every geometric decision lives in this repository and can be tested deterministically ([ADR-0014](docs/adr/0014-floating-point-determinism.md)).

> **Status.** A dual-representation CAD kernel, no third-party CAD kernels involved. The **SDF side**: primitives, CSG booleans, smooth blends, MC33 extraction with vertex welding, and the **twist warp under an explicit contract** ([ADR-0029](docs/adr/0029-twist-warp-contract.md), implemented: cylinder-AABB bounds, the σ(k, r) Lipschitz closed form, the full chain-rule gradient, construction-time validation on every node, and a property closure with twisted bridge extractions). The **B-Rep side**: radial-edge topology with query operators, analytic seam wires, and **boolean operations on convex polyhedra** (`bool_union`/`bool_subtract`/`bool_intersect`) — every non-empty result validated against exact topological contracts (V/E/F counts, closed loops, and the amended χ contract: `is_closed_orientable` for the structural invariants, χ and genus **reported** — derived only for single closed-orientable shells — with `is_closed_manifold` kept as the exact χ=2 compatibility wrapper). The **SDF → B-Rep bridge** ([ADR-0026](docs/adr/0026-sdf-brep-bridge.md)) extracts native SDF bodies into B-Rep solids — exact analytic primitives (sphere, box, cylinder) or MC33 faceted — under the same hard contracts, pinned V/E/F per grid, genus-1 extractions included (the pierced box: χ=0, genus 1, bit-identical re-runs); the **inverse direction (B-Rep → SDF) is an approximate, documented stub** (primitive detection + mesh SDF), deferred to a future ADR. The PySide6 desktop app adds parametric shape creation, undo/redo, viewport picking with 3D highlight, STL/OBJ import, STL/OBJ/3MF export — and **boolean operations as tree features** ([ADR-0027](docs/adr/0027-phase-f-boolean-feature-slice.md)): Edit → Subtract builds an operation node whose operands stay referenced by FeatureId (auto-hidden, editable, world-anchored placement), undo/redo is id-stable, and a deleted operand leaves a visible dangling state — never a silent rebind — all persisted by the versioned native format (`.CAD_0` v0.4.0: recipes are the source of truth, bodies the cache; chained migrators from v0.1.0) with atomic save. The **sketcher MVP** ([ADR-0030](docs/adr/0030-sketcher-mvp.md), implemented) adds parametric 2D profiles: **Create → Sketch** opens a canvas where mouse gestures draw typed entities (segments, circles, arcs — sketch toolbar, shortcuts 1–4), **Feature → Extrude** turns the closed profile into a solid prism, and **Edit → Subtract** cuts it from a body — then edit any entity parameter and the whole chain (sketch → extrude → subtract) regenerates from the recipe, deterministically, one undo entry per modification. The Debug menu reports topology, mass and provenance for any selected body (the report grew with G1: χ with its inner-loop term, genus, the mass properties with the exact/approximated flag, and the per-kind provenance tallies — through `CAD_0.report`, the single consumer of the mass API and the body query). **Roadmap: Phase F (parametric modeling) in progress — the boolean feature slice and the sketcher MVP are in. The great fork is decided ([ADR-0031](docs/adr/0031-exact-brep-kernel.md) accepted: the exact B-Rep kernel will be built in house, forever; the license whitelist is pinned in ADR-0003), and its first rung is complete ([ADR-0032](docs/adr/0032-g1-topology-and-exact-primitives.md), implemented 2026-10-26: G1 units 1–6 landed and click-tested — **G1 complete and closed: the D7.5 table read in the report, the volume verified by hand, the radius edited live** — unit 1, the data model: the sketch kernel's 2D types reused as the p-curve vocabulary on co-edges, Layer-2 tolerance + provenance on every vertex/edge/face with the writers' honesty, and the extended χ = V−E+F−H with its backward compatibility proven on every golden; unit 2, the primitives with true trimming: box/sphere/cylinder/tetrahedron faces are now regions of domains — every co-edge carries its typed p-curve (the wall's rectangle walk in (θ,z), the caps' π-sweep arc pairs, the sphere's periodic seam), the cylinder's parametrization corrected to the surfaces' one right-handed rule, and the face query (family, loops, p-curves, provenance) exposed through the bindings as plain values; unit 3, the exact extrude: the sketch-born geometry becomes exact — the profile chain maps to the prism's true faces per the ADR's table, the caps' outer trimming loop is the validated chain itself (entity for entity, hole rims as Circle2 trim curves), and the extrude node materializes the exact B-Rep BESIDE its SDF as the HybridBody ADR-0002 always meant (bridge tolerance 0, the same solid twice — the SDF still feeds display, the exact B-Rep reads through the bindings' query, and out-of-class profiles keep their SDF with the refusal recorded); unit 4, the exact subtract: the exact chain crosses its first curved-surface intersection — the plane∩cylinder SSI entry in the three orientations (⊥ the circle with its Line2 on the cylinder domain, ∥ the line pair with the tangent and empty cases, oblique refused naming the angle), the pinned through-hole topology (pierced faces gaining inner loops split at the seam pair, the surviving wall the patch between the circles with its seam's two co-edges in the one loop, the guide case 12V/17E/7F/H=2 χ=0 genus 1 — same topology as the faceted report, better geometry — and V+2n/E+3n/F+n for every tool profile), the degenerates with their pinned dispositions (coplanar absorbed, tangential/crossing/blind refused with the remedies named, the miss the exact copy), and the subtract node owning both materializations — the v0.4 golden materializes the exact subtract on load while the v0.3 golden stays honestly faceted (its tool is the Y-axis primitive cylinder, not a Z-axis prism — the refusal recorded); unit 5, the exact mass properties: the B-Rep’s first quantitative interrogation — volume, centroid and the inertia tensor by the divergence theorem over the exact faces (V = (1/3)∮x·n and the moment kernels, the 1/5 ansatz constant derived), the planar families by Green’s theorem in the face’s own domain (the winding sign plus the positional outer-minus-inner — one geometric rule for every authored walk), the cylindrical patches by the (θ,z) rectangle integrals, the full sphere by its closed form; all_exact is the honesty flag (a faceted body answers the polyhedral value, never silently mixed), out-of-class faces refuse actionably, and the classical formulas ARE the tests — the guide case’s hand-checkable table abc − πr²h and I_zz = abc(a²+b²)/12 − πr⁴h/2, readable from Python through CAD_0.brep.mass_properties; unit 6, the report and the guide case end-to-end: the Debug report grows mass + provenance beside χ (label honest: V−E+F−H) and genus, the property panel shows the numbers read-only (Volume, Centroid, Inertia, the exact/approximated flag — QLabel rows structurally incapable of emitting parameter_changed, the UI-bug lesson made structural; a faceted body says approximated with its polyhedral numbers, never an exact in disguise), the body-level query `CAD_0.brep.body_query` answers counts/χ/genus/provenance as plain values, and the guide-case example prints the D7.5 table side by side — analytic vs reported, 12/17/7/2, χ=0, genus 1, abc − πr²h, I_zz = abc(a²+b²)/12 − πr⁴h/2, all exact — every row PASS or the difference named, the radius edit bit-identical through ONE undo entry, the example running in the CI as a suite member; the v0.4 golden pinned with the table’s numbers, the v0.3 golden honestly faceted.** The G1 fork follow-up is **accepted and G2 is under way** ([ADR-0033](docs/adr/0033-g2-exact-booleans-and-first-fillet.md), accepted 2026-10-26: the analytic SSI table, the exact booleans, the first fillet — **the reserved table-boundary decision is taken: option (i), the table is the wall** (out-of-table pairs refused actionably, the ceiling explicit and queryable; option (ii) — declared-tolerance subdivision — registered as a priced candidate with its entry criterion: a real case demanded by an author body, decided at the next internal checkpoint with table and fillet data on the table); one correction carried on the record — the drafted unequal-radii limit of the cylinder pair missed the typed transverse interior, corrected where it stood; **units 1–2 of 6 LANDED in the companion commits — unit 1, the table** — the four typed entries (plane×sphere's circle √(R²−d²) at the foot of the perpendicular, sphere×sphere's radical-plane circle, cylinder×sphere coaxial's two circles at ±√(R²−r²), cylinder×cylinder parallel equal-radii's mid-plane line pair), each a derivation whose golden IS the classical formula, the p-curves typed in BOTH domains, the re-frame rule landed (the framed SphereSurface beside the framed CircleCurve, the seam re-wired in the ADR-0023 pattern, the frame budget derived from the face's own record — one axis family per face, the incompatible family finding the wall), the degenerates pinned (tangencies, the empty classes, the co-radial coincident), and the whole table QUERYABLE from Python through `CAD_0.brep.ssi_query` BEFORE any boolean consumes it (curves, p-curves in both domains, provenance exact/0, the refusals as returned records naming the pair, the class and the remedies — the Viviani quartic, the Steinmetz sinusoids, the corrected unequal-radii gate, the landed pairs pointed to their owners, the cone's no-writer registration, the fillet's toroidal rows) — with `CAD_0.brep.translate_body` exposing the exact side's own placement so the author bodies can be positioned for the query; **unit 2 of 6 LANDED in the companion commit: the extended booleans** — the operations consume the table (sphere×sphere union/lens/bitten, the bead/rivet/core, the truncation family, the two-holes pierced-target guide case 16/22/8/4 χ=−2 genus 2 with the volume abc − 2πr²c, the frame budget applied, the non-annular refusals naming the offset-plane disposition, the trimmed-sphere mass family with the classical formulas as the tests — all of it from Python through CAD_0.brep.exact_union / exact_intersect / exact_subtract, bit-identical, the historical goldens and the v0.3/v0.4 projects untouched); the ADR-0031 internal checkpoint is recorded: the estimates stand). The CI matrix (GCC 11 debug + release, Clang 14) is green on `main`.

<!-- TODO(badge GIF): record a 15–25s noVNC demo (create box → sketch →
     draw a circle → extrude → subtract → double-click the circle and edit
     the radius live, geometry regenerating) and embed it here:
     <p align="center"><img src="docs/media/demo.gif" width="640" alt="CAD_0 desktop app demo"></p> -->

## Highlights

- **Native SDF kernel** — primitives, CSG booleans, smooth blends, transforms, batched vectorized evaluation (xsimd, multi-threaded) — no mesh in sight until you ask for one.
- **Marching Cubes mesh extraction** ([ADR-0015](docs/adr/0015-marching-cubes.md)) with **vertex welding** via canonical edge keys ([ADR-0016](docs/adr/0016-vertex-welding.md)).
- **Versioned native format** (`.CAD_0`, [ADR-0007](docs/adr/0007-native-format-versioning.md)): JSON feature tree + binary blob container ([ADR-0007a](docs/adr/0007a-binary-blob-container.md)) for embedded meshes, atomic save, transactional load, explicit migration chain (v0.1.0 → v0.2.0 → v0.3.0 → v0.4.0).
- **STL/OBJ import** (welded, first-class mesh scene nodes) and **STL/OBJ/3MF export**.
- **Desktop app** (PySide6, [ADR-0006](docs/adr/0006-ui-primary-pyside6.md)): registry-driven Create menu, feature tree, parametric property panel, undo/redo, picking with amber 3D highlight.
- **Parametric sketcher** ([ADR-0030](docs/adr/0030-sketcher-mvp.md)): typed 2D profiles drawn by gesture on the sketch canvas, extruded into solid prisms, subtracted from bodies — edit any parameter and the DAG regenerates the chain.
- **Deterministic by construction** ([ADR-0014](docs/adr/0014-floating-point-determinism.md)): `-ffp-contract=off`, bit-identical round-trip tests, byte-identical golden files.
- **Headless-first testing** ([ADR-0017](docs/adr/0017-viewport-architecture.md)): the whole UI suite runs offscreen under Xvfb in CI — including GL viewport, picking and navigation.
- **C++20 core, zero UI dependencies**; nanobind bindings ([ADR-0011](docs/adr/0011-nanobind-vs-pybind11.md)).

## Quick start

### Try it in GitHub Codespaces (one click)

Open this repo in a Codespace and run:

```bash
bash scripts/start_codespaces.sh
```

This builds the kernel + Python package, starts an Xvfb + fluxbox desktop with a **noVNC preview** (:6080), and launches the app — the same software-GL stack the CI UI tests use. See [CODESPACES.md](CODESPACES.md) for details and troubleshooting.

### Build locally

Prerequisites: C++20 compiler (GCC ≥ 11, Clang ≥ 14, or MSVC ≥ 17.10), CMake ≥ 3.22, Ninja, Python ≥ 3.10 with dev headers (for the bindings).

```bash
git clone https://github.com/MarioW98/CAD_zero.git
cd CAD_0

# Kernel + tests, no Python bindings
cmake -B build -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCAD_0_BUILD_PYTHON=OFF \
    -DCAD_0_BUILD_TESTING=ON
cmake --build build
ctest --test-dir build
```

Preset users: `cmake --preset linux-gcc-release` (or `linux-clang`, `linux-gcc-debug` with ASan). CI parity is reproducible with the pinned `linux-gcc11-ci` preset — see the *Toolchain contract* in [docs/audits](docs/audits/).

### Build with Python bindings

The bindings need Python dev headers + nanobind. If you use a venv (e.g. uv-managed), point CMake at the right interpreter:

```bash
pip install cmake ninja nanobind scikit-build-core

cmake -B build -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCAD_0_BUILD_PYTHON=ON \
    -DCAD_0_BUILD_TESTING=ON \
    -DPython3_EXECUTABLE=$(which python3)
cmake --build build

# Smoke-test the extension
cd build/python
python3 -c "
import sys; sys.path.insert(0, '.')
import CAD_0
sph = CAD_0.sdf.sphere(radius=2.0)
print(sph.describe())
mesh = CAD_0.sdf.marching_cubes(sph, resolution=16)
print(f'Vertices: {mesh.vertex_count()}, Triangles: {mesh.triangle_count()}')
"
```

<details>
<summary><b>Troubleshooting: “Could NOT find Python3 (missing: Development.Module)”</b></summary>

CMake found an interpreter but not its development headers.

1. **Debian/Ubuntu (system Python):** `sudo apt install python3-dev`
2. **Venv (uv- or pip-managed):** the venv must have its own dev headers; pass `-DPython3_EXECUTABLE=/path/to/venv/bin/python3` so CMake picks the right interpreter. uv-managed Pythons ship headers under `<uv-cache>/python/cpython-3.XX-*/include/python3.XX/Python.h`.
3. **Multiple Python versions:** CMake may find Python 3.13 (with headers) when your venv uses 3.12 (without). Always pass `-DPython3_EXECUTABLE=` explicitly.

</details>

## Use the kernel from Python

```python
import CAD_0
import numpy as np

# Build a sphere SDF — this is a *native* representation, not a mesh.
sph = CAD_0.sdf.sphere(radius=2.0)

# Boolean CSG — all native to the SDF domain.
cyl = CAD_0.sdf.cylinder(radius=0.5, height=10.0)
carved = CAD_0.sdf.subtract(sph, cyl)

# Smooth blend — equation-driven, mesh-free.
b = CAD_0.sdf.translate(CAD_0.sdf.sphere(radius=1.0), (3.0, 0.0, 0.0))
blended = CAD_0.sdf.smooth_union(sph, b, k=0.6)

# Batched evaluation — vectorized, multi-threaded.
pts = np.random.uniform(-5, 5, (1024, 3)).astype(np.float32)
result = CAD_0.sdf.evaluate(blended, pts)
print(result.values[:8])

# Mesh extraction + export.
mesh = CAD_0.sdf.marching_cubes(blended, resolution=64)
CAD_0.io.export('output.stl', mesh)  # format auto-detected from the extension

# Native project format — save the whole scene (+ camera) atomically.
scene = CAD_0.scene.Scene()
scene.add_node("part", blended)
CAD_0.io.save("project.CAD_0", scene,
              camera={"target": (0, 0, 0), "distance": 8.0,
                      "yaw": 0.5, "pitch": 0.3, "fov_deg": 45.0})

# Transactional load: a corrupted or newer-version file raises with an
# explicit message and leaves the target scene untouched.
scene2 = CAD_0.scene.Scene()
cam = CAD_0.io.load("project.CAD_0", scene2)
```

## Use the kernel from C++

```cpp
#include <CAD_0/sdf/primitives.hpp>
#include <CAD_0/sdf/operators.hpp>
#include <CAD_0/sdf/transforms.hpp>
#include <CAD_0/sdf/evaluate.hpp>

using namespace CAD_0;

int main() {
    auto sph  = sdf::make_sphere(2.0f);
    auto cyl  = sdf::make_cylinder(0.5f, 10.0f);
    auto cut  = sdf::sdf_subtract(std::move(sph), std::move(cyl));

    sdf::CpuEvalBackend backend;
    std::vector<math::Vec3f> pts = {{0, 0, 0}, {1, 1, 1}};
    sdf::EvalResult out;
    backend.evaluate(cut, pts, out);
    return 0;
}
```

## The desktop app (Phase C+)

The PySide6 app is the primary UI ([ADR-0006](docs/adr/0006-ui-primary-pyside6.md)):

- **Create** menu driven by the kernel's builder registry — adding a builder in C++ automatically adds it to the UI (single source of truth: `core/scene/builders.hpp`).
- **Feature tree + parametric property panel** — edit radius/height/etc. and the body rebuilds; every edit is undoable (create/delete/parameter commands).
- **Edit → Subtract (boolean as a tree feature)** — multi-select two SDF bodies and subtract: an operation node joins the tree (type `CSG`), the operands are hidden but stay editable, the placement offset is world-anchored, and edits propagate through the DAG bit-identically. Undo/redo restores the **same node ids**; deleting an operand leaves a visible `CSG!` dangling state with an actionable message instead of a silent rebind.
- **The parametric hole (sketcher MVP, [ADR-0030](docs/adr/0030-sketcher-mvp.md))** — the canonical CAD use case as a user path: **Create → Sketch** (a third node kind; the canvas is a view of the node) → pick a tool on the sketch toolbar (**1** select, **2** segment, **3** circle, **4** arc) and draw — a circle is two clicks (center, then rim), and every gesture writes typed parameters into the sketch node's recipe → **Feature → Extrude…** turns the closed profile into a prism → **Edit → Subtract** cuts it from the box. Then select the circle — or double-click it — and edit the radius in the property panel: the extrude and the subtract regenerate live, with one undo entry per modification. `examples/sketch_hole.py` walks the same chain from Python, genus-1 report included.
- **Viewport picking** — click a shape in the 3D view (SDF or imported mesh) to select it: the tree, the property panel and an amber highlight in the viewport all follow, whatever the selection origin.
- **Import STL/OBJ** as first-class mesh nodes (vertices welded, counts reported); **export STL/OBJ/3MF**.
- **Open… / Save / Save As** (`Ctrl+O`, `Ctrl+S`, `Ctrl+Shift+S`) in the native `.CAD_0` format (v0.4.0 — [ADR-0007](docs/adr/0007-native-format-versioning.md) / [ADR-0007a](docs/adr/0007a-binary-blob-container.md) / [ADR-0027](docs/adr/0027-phase-f-boolean-feature-slice.md) / [ADR-0030](docs/adr/0030-sketcher-mvp.md)):
  - **Atomic save** — writes `<file>.tmp` then renames; a crash never destroys the previous file.
  - **Transactional load** — the document is fully validated before the scene is touched; a corrupted file raises an explicit error.
  - **Bit-exact round-trip** — node parameters, builder tags, transforms, visibility and the viewport camera survive save/load unchanged; legacy v0.1.0 pure-JSON files keep opening through the migrator chain.
  - **Dirty tracking** — any scene change flags the project; closing prompts to save.

## Repository layout

```
CAD_0/
├── core/                # C++20 kernel (zero UI deps)
│   ├── math/            # vec, mat, quat, bbox, tolerance, robust predicates
│   ├── geometry/        # Shape (std::variant), ShapeId, Transform, Visitor
│   ├── sdf/             # Native SDF: primitives, operators, transforms, evaluate
│   ├── brep/            # B-Rep kernel (tessellation now; topology = Phase D)
│   ├── bridge/          # (Phase E) SDF ↔ B-Rep conversion
│   ├── concurrency/     # TBB thread pool, GIL guard, task graph
│   ├── scene/           # Scene graph, SceneNode (Sdf|Mesh), builder registry
│   ├── viewport/        # Headless-first renderers (ADR-0017)
│   └── io/              # STL/OBJ/3MF, native .CAD_0 format
├── bindings/python/     # nanobind bindings + Python package
├── app/                 # PySide6 desktop app
├── scripts/             # Codespaces launcher (Xvfb + fluxbox + noVNC)
├── docs/                # architecture, ADRs, warning audits
├── examples/            # Python usage examples
├── tests/               # doctest unit + regression suites
└── third_party/         # Vendored Shewchuk predicates (public domain)
```

See [docs/architecture/overview.md](docs/architecture/overview.md) for the full layout and design rationale.

## Roadmap

| Phase | Scope | Status |
|-------|-------|--------|
| A | Core math, geometry, tolerance model | ✅ done |
| B | SDF kernel, marching cubes, vertex welding | ✅ done |
| C | Viewport architecture, PySide6 desktop app | ✅ done |
| C+ | Native format + migrators, STL/OBJ import, picking, CI matrix, toolchain contract | ✅ done |
| **D** | **Radial-edge B-Rep topology (ADR-0012), query operators, seam wires, convex booleans (union/subtract/intersect) with slab decomposition, T-junction subdivision** | ✅ done ([ADR-0021](docs/adr/0021-brep-topology-data-structure.md)–[0025](docs/adr/0025-proper-greiner-hormann.md)) |
| E | SDF → B-Rep extraction under hard topology contracts ([ADR-0026](docs/adr/0026-sdf-brep-bridge.md)); inverse bridge: documented stub, future ADR | ✅ done |
| **F** | **Modeling: boolean feature slice in the tree + `.CAD_0` v0.3 operation recipes ([ADR-0027](docs/adr/0027-phase-f-boolean-feature-slice.md)); sketcher MVP — typed 2D profiles, extrude as a feature, `.CAD_0` v0.4 ([ADR-0030](docs/adr/0030-sketcher-mvp.md))** | ▶️ in progress |
| **G** | **The exact B-Rep kernel — road (a), all in house ([ADR-0031](docs/adr/0031-exact-brep-kernel.md), accepted; ADR-0003 amended: the license whitelist): G1 exact primitives + exact mass properties ([ADR-0032](docs/adr/0032-g1-topology-and-exact-primitives.md), implemented), G2 exact booleans + the first fillet ([ADR-0033](docs/adr/0033-g2-exact-booleans-and-first-fillet.md) **accepted 2026-10-26: D2 = option (i), the table is the wall — option (ii) registered priced with its entry criterion, decided at the next checkpoint**), G3 free surfaces** | ▶️ in progress — **G1 COMPLETE AND CLICK-TESTED (2026-10-26): ADR-0032 Implemented (the flip is the click-test's own act, the ADR-0030 precedent), the ADR-0031 internal checkpoint recorded (the estimates stand), ADR-0033 accepted with the table-boundary decision taken — D2 = option (i), the wall; the review carried one correction to the record (the cylinder pair's unequal-radii limit missed the typed transverse interior, corrected where it stood); G2 units 1–2 of 6 LANDED — unit 1, the table**: the four typed entries, one derivation each (the classical formulas ARE the goldens: √(R²−d²), the radical plane, ±√(R²−r²), the mid-plane lines at ±√(r²−d²/4)), the p-curves typed in BOTH domains (the framed SphereSurface/CircleCurve, the latitude Line2s, the full-period convention), the re-frame rule with its derived frame budget (one axis family per face — the incompatible family finds the wall, option (i) coherent to the end), the degenerates with pinned dispositions (the tangent point/circle/double line, the empty classes named, the co-radial coincident — the dedup rule's trigger), the wall as returned records (the Viviani quartic, the Steinmetz sinusoids in both domains, the corrected unequal-radii gate, plane×cylinder and plane×plane pointed to their landed owners, the cone registered with no writer, the toroidal rows registered for the fillet's unit, the provenance gate on approximated operands) — all of it QUERYABLE from Python (`CAD_0.brep.ssi_query`, with `translate_body` for positioning) before any boolean consumes it, bit-identical across re-runs (hex-digest pinned); **unit 2 of 6 LANDED in the companion commit: the extended booleans — the operations CONSUME the table** (union/intersect/subtract exact exactly where every curved pair of the operands sits in the typed table, the class check part of the ENTRY contract): the sphere×sphere family (the classical union 4/4/2 χ=2, the lens, the bitten sphere — the radical-plane disk, the co-radial dedup, the tangency and nested/separate dispositions pinned), the sphere±prism family (the BEAD 4/6/2 χ=0 genus 1 — the equatorial zone + the cavity wall band; the rivet 8/11/5; the core 6/7/3; the truncation family: the far/near caps, the dome with the pierced tool face gaining the latitude inner loop, the two-shell slab), and the prism-pair subtract's PIERCED-TARGET class — the two-holes guide case: 16/22/8/4, χ=−2, genus 2, volume abc − 2πr²c, TWO distinct CylinderZ walls, the pierced faces carrying two inner loops each; the frame budget APPLIED (the trimmed survivor's one family — the incompatible later cut finds the wall, the multi-cut cascade honestly refused), the non-annular refusals naming the dispositions (the unequal-radii crossing carries the offset plane x=(d²+r₁²−r₂²)/(2d) and its two typed lines — typable, on the record, never dismissed), the worst-of provenance chain, the blind/tangent/empty/Viviani degenerates, and the TRIMMED-SPHERE MASS family (the latitude-bounded windows integrated by the bivariate trig recurrences — the classical cap/lens/bead/rivet formulas ARE the tests, readable from Python through CAD_0.brep.exact_union / exact_intersect / exact_subtract with the bodies queryable and everything bit-identical across re-runs); the G1 amendments at the two sites they stood (the sphere target and the sphere-tool node now materialize the bead — the unit's own act, documented in the cases); no fillet moved, the format did not move, the display did not change — units 3–6 follow (the fillet, the degenerates' corpus, the UI, the guide case end-to-end); G1 units 1–6 all landed — unit 1, the data model (p-curves on co-edges, Layer-2 provenance, χ = V−E+F−H); unit 2, the primitives with true trimming + the face query in the bindings; unit 3, the exact extrude (the HybridBody on the node); unit 4, the exact subtract (the plane∩cylinder SSI, the pinned through-hole topology, the honest refusals); unit 5, the exact mass properties (the divergence theorem over the exact faces, all_exact the honesty flag, the classical formulas as the tests); unit 6, the report and the guide case end-to-end — the Debug report grew mass + provenance beside χ (with the H term in the label) and genus through `CAD_0.report` (the single consumer of the mass API and the body query, no second opinion), the property panel shows the numbers READ-ONLY (QLabels that cannot emit parameter_changed — the UI-bug lesson made structural, the exact/approximated flag visible, a faceted body's polyhedral numbers never disguised as exact), and the guide-case example prints the D7.5 table side by side (12/17/7/2, χ=0, genus 1, abc − πr²h, I_zz = abc(a²+b²)/12 − πr⁴h/2 — every row PASS) with the radius edit bit-identical through ONE undo entry, running in the CI as a suite member; the v0.4 golden pinned with the table's numbers, the v0.3 golden honestly faceted |
| H | STEP AP203 MVP (ADR-0010) | planned |

**Platform vision** — the kernel as the geometric core of a broader CAE platform: a preprocessor's exact geometry, parametric vehicle geometry (G3, declared), and vortex/panel analysis consuming coherent surface meshes — [docs/architecture/platform-vision.md](docs/architecture/platform-vision.md). Orientation, not scope: the horizon never widens a rung.

## Key architectural decisions

| ADR | Topic |
|-----|-------|
| [0001](docs/adr/0001-use-cpp20.md) | C++20 (subset) for the core kernel |
| [0002](docs/adr/0002-dual-kernel-native-sdf.md) | **SDF is a first-class native representation; `Shape` uses `std::variant`** |
| [0003](docs/adr/0003-third-party-policy.md) | No third-party CAD kernels |
| [0004](docs/adr/0004-tolerance-model.md) | Tolerance is a first-class citizen; no global epsilon |
| [0005](docs/adr/0005-persistent-naming-early.md) | **Persistent naming by genealogy + subshape tracing (NOT geometric signature)** |
| [0006](docs/adr/0006-ui-primary-pyside6.md) | PySide6 (Python) is the primary UI; Qt6 C++ removed |
| [0007](docs/adr/0007-native-format-versioning.md) | Native format with explicit versioning + migration chain |
| [0007a](docs/adr/0007a-binary-blob-container.md) | Binary blob container for embedded meshes (v0.2.0, "raw" encoding; zstd deferred) |
| [0008](docs/adr/0008-simd-abstraction.md) | xsimd as SIMD abstraction; GPU backend stubbed from Phase A |
| [0009](docs/adr/0009-threading-model.md) | oneTBB + RAII GIL guard |
| [0010](docs/adr/0010-step-scope-mvp.md) | STEP MVP scope: AP203 ed. 2, analytic-only |
| [0011](docs/adr/0011-nanobind-vs-pybind11.md) | nanobind (not pybind11) for Python bindings |
| [0012](docs/adr/0012-non-manifold-topology.md) | Radial-edge B-Rep topology (not half-edge) |
| [0013](docs/adr/0013-license-compatibility.md) | Apache-2.0; CGAL GPL modules forbidden |
| [0014](docs/adr/0014-floating-point-determinism.md) | Cross-platform FP determinism via `-ffp-contract=off` |
| [0015](docs/adr/0015-marching-cubes.md) | Marching Cubes for SDF mesh extraction (Phase B) |
| [0016](docs/adr/0016-vertex-welding.md) | Vertex welding via canonical edge keys (Phase B.5) |
| [0017](docs/adr/0017-viewport-architecture.md) | Headless-first viewport architecture (Phase C) |
| [0018](docs/adr/0018-pyside6-ui-integration.md) | PySide6 UI integration (Phase C.6-C.8) |
| [0019](docs/adr/0019-project-rename.md) | Project rename cadforge → CAD_0 |
| [0020](docs/adr/0020-python-bindings-build.md) | Python bindings build configuration |
| [0021](docs/adr/0021-brep-topology-data-structure.md) | B-Rep topology data structure — radial-edge entity set, ID scheme, ownership, Euler invariant (Phase D step (a)) |
| [0022](docs/adr/0022-brep-topology-query-operators.md) | B-Rep topology query operators (faces_around_edge, edges_around_vertex, opposite_coedge, is_closed_manifold, wireframe_edges, vertex_valence) + make_tetrahedron (Phase D step (b)) |
| [0023](docs/adr/0023-analytic-seam-wires-and-boolean-intersect.md) | Analytic seam wires (sphere: 2V/1E/1F, cylinder: 4V/5E/3F — both now closed manifolds) + bool_intersect on convex polyhedra via Sutherland-Hodgman (Phase D step (c)) |
| [0024](docs/adr/0024-boolean-union-and-subtract.md) | bool_union and bool_subtract via centroid classification + identical-body shortcut (MVP; partial-overlap faces kept intact when their centroid is outside the other solid) (Phase D step (c+)) |
| [0025](docs/adr/0025-proper-greiner-hormann.md) | Greiner-Hormann 2D polygon clipping infrastructure (kept for future use; current bool_union/subtract use the simpler centroid-classification approach with identical-body shortcut, ADR-0024 §3) (Phase D step (c+) fix) |
| [0026](docs/adr/0026-sdf-brep-bridge.md) | SDF → B-Rep bridge under hard topology contracts (pinned V/E/F per grid, χ/genus reported and derived only where sound; the inverse direction is a documented stub) |
| [0027](docs/adr/0027-phase-f-boolean-feature-slice.md) | Phase F boolean feature slice: operation nodes in the tree, FeatureId-referenced operands, id-stable undo/redo, `.CAD_0` v0.3 operation recipes |
| [0028](docs/adr/0028-sdf-bounds-smooth-blend-dilation.md) | SmoothMinSDF bounds() dilates by k/8 per blend level (the k/16 field dip, conservatively converted); the stratified positivity property closes the thin-shell blind spot |
| [0029](docs/adr/0029-twist-warp-contract.md) | **The twist warp contract (implemented)**: cylinder-AABB bounds (k-blind proof, √2 on-axis cost), the σ(k, r) Lipschitz closed form, the full chain-rule gradient with the y shear; construction-time validation across the 16-node table; property closure with anti-revolution liveness guards, the σ k-sweep, and twisted bridge extractions under the full contract set |
| [0030](docs/adr/0030-sketcher-mvp.md) | **The sketcher MVP (implemented)**: typed 2D entities (segments, circles, arcs) as a third node kind, extrude as an operation node with the subtract's genealogy, the 2D field semantics derived to the ADR-0029 standard (max-composition: sign and zero-set exact, Lipschitz 1 proved, the conservative gap named), `.CAD_0` v0.4 — the constraint solver explicitly deferred with its entry criterion |
| [0031](docs/adr/0031-exact-brep-kernel.md) | **The exact B-Rep kernel — the great fork (accepted: road (a), all in house, forever)**: build / OCCT / hybrid priced honestly side by side with failure modes named; the fork closed at acceptance — the G2 gate an internal checkpoint, not a fork; the license whitelist pinned into ADR-0003 in the same act; the data model, the three-layer tolerance discipline, and Euler-Poincaré as kernel invariant bind every future geometry ADR; the G1/G2/G3 ladder with the migration (SDF positioned, not killed; `.CAD_0` extends, never rewrites) — **the G1 internal checkpoint recorded 2026-10-26: G1 measured (six units, three sessions, one CI-caught defect closed same day, zero user-visible), the estimates stand** |
| [0032](docs/adr/0032-g1-topology-and-exact-primitives.md) | **G1 — topology and exact primitives (implemented 2026-10-26 — six units in three sessions, closed by the user's click-test)**: the first rung — box/sphere/cylinder with true trimming (p-curves on co-edges, the sketch kernel's 2D types reused as the vocabulary — now WRITTEN by the factories, with the analytic frames' right-handed rule corrected on the cylinder), exact mass properties from the B-Rep via the divergence theorem (volume/centroid/inertia, closed forms per analytic face — LANDED: the kernels, the Green/rectangle/sphere families, the all_exact honesty flag, the classical formulas as the tests), the plane∩cylinder entry (⊥ and ∥ exact; the oblique case honestly out — the sinusoid p-curve needs G3's splines), the exact extrude and subtract (the subtract landed: the SSI entry, the pinned through-hole topology, the degenerates’ dispositions, the HybridBody on the subtract node), the Layer-1 invariant hard-fail with the derived inner-loop term **χ = V−E+F−H = 2(S−G)** (the guide case's pierced faces force it), the provenance contract ("is this edge exact?" always answerable — now a face query in the bindings), six delivery units, all landed; no promise on G2 |
| [0033](docs/adr/0033-g2-exact-booleans-and-first-fillet.md) | **G2 — the analytic SSI table, exact booleans, and the first fillet (accepted 2026-10-26 — D2 decided at review: option (i), the table is the wall)**: the typable-pair table with one derivation per entry under the both-domain admission criterion (typed 3D curve AND typed p-curves in BOTH parameter domains — the G1 sinusoid lesson made a rule; ADR-0032 D5.3's unlanded Ellipse2 parenthetical corrected on the record), the deterministic re-frame rule for revolution surfaces (one axis family per face — the second incompatible cut is out-of-table), **the table boundary DECIDED: exact-where-the-table-reaches with actionable refusals (option i) — option (ii), declared-tolerance subdivision, registered as a priced candidate with its entry criterion (a real case demanded by an author body; decided at the next internal checkpoint, empirically — the user's motivation recorded verbatim: the wall is always honest; the hedge, once planted, must be watched forever; first the wall, the hedge when a real case asks for it)**; the degree > 2 pairs (the Viviani-class quartics, the spiric sections of Perseus) are not typable, no silent third way; one correction carried at acceptance — the drafted unequal-radii limit of the cylinder pair missed the typed transverse interior (two lines in the offset plane, admitted by the criterion), corrected where it stood; the rolling-ball fillet on the derived class (the box edge's cylindrical flank; the hole rim's toroidal flank — the G1 guide body's own rim, the counts 14/20/8/2 and the volume with the torus term derived hand-checkable), the degenerate dispositions (radius-too-large errors, the corner collision IS the vertex blend → refused with the remedy, tangencies tie-broken golden-pinned), the six-unit ladder and the no-promises list (vertex blends unscheduled, G3 untouched, no new dependencies, the format moves only with the fillet unit's v0.5 companion per ADR-0007) |

## Testing, determinism, CI

- Every push runs the **full matrix on ubuntu-22.04**: GCC 11 debug (ASan), GCC 11 release, Clang 14 — C++ doctest suite, Python bindings tests, and headless UI tests under Xvfb (`fail-fast` is off: one red job never hides the others).
- **Determinism is a contract, not a hope** ([ADR-0014](docs/adr/0014-floating-point-determinism.md)): `-ffp-contract=off`, round-trip tests that must be *bit-identical*, golden files that must be *byte-identical*. Fixes may not change evaluation values.
- **Toolchain contract**: warning audits and any `-Werror` change must be verified against the CI compilers (GCC 11.4, Clang 14) before push — reproducible locally via the `linux-gcc11-ci` preset. See the audits in [docs/audits/](docs/audits/).

## Contributing

The project follows an **ADR-first culture**: architectural decisions are written, reviewed and recorded in `docs/adr/` before the code that implements them; regressions get a failing test before the fix. Open a Codespace (everything needed to build, run and test ships in the dev container), branch from `main`, and keep commits descriptive.

## License

Apache-2.0. See [LICENSE](LICENSE).

The vendored Shewchuk robust predicates under `third_party/robust_predicates/` are public domain; all other dependencies keep their original licenses (MIT, BSD-3, Apache-2.0). The `[ui]` extra pulls in PySide6, the only copyleft (LGPL-3.0) in the tree — dynamic import, replaceable, notices required for closed-source derivatives. The [NOTICE](NOTICE) file lists every third-party component and its license (the ADR-0013 release notice, due since the `v0.1.0-alpha` tag). Full inventory and per-dependency flags: [docs/audits/2026-10-16-third-party-license-audit.md](docs/audits/2026-10-16-third-party-license-audit.md).
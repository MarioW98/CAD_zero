# CAD_0

[![CI](https://github.com/MarioW98/CAD_zero/actions/workflows/ci-linux.yml/badge.svg)](https://github.com/MarioW98/CAD_zero/actions/workflows/ci-linux.yml)
[![License: Apache-2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

**Dual-representation CAD kernel — native SDF + B-Rep, in C++20 and Python.**

CAD_0 is a from-scratch CAD kernel where **SDF (signed distance fields) and B-Rep (boundary representation) are first-class peers**. There is no "primary" representation: every operation has a native implementation in both domains, and the system dispatches on operand types.

The long-term goal is an **open-source CAD system built without any third-party CAD kernel** ([ADR-0003](docs/adr/0003-third-party-policy.md)), so every geometric decision lives in this repository and can be tested deterministically ([ADR-0014](docs/adr/0014-floating-point-determinism.md)).

> **Status — the truth, in three lines.** What exists, and the CI testifies: the **exact primitives** with true trimming, the **booleans with gates** (input contracts validated, out-of-table pairs refused actionably, every result checked against exact topological contracts), and **persistent naming** — the derived name beside the address ([ADR-0036](docs/adr/0036-persistent-naming-and-shape-history.md)); beside them the full SDF kernel, the sketch → extrude → subtract chain, exact mass properties, the desktop app. What does **not** exist yet — said, not hidden: the **fillet** (G2 units 2–6), **patterns**, **STEP** (Phase H), G3 free surfaces, the sketcher's constraint solver. The full record, with the roadmap and the cronaca: [docs/status/](docs/status/README.md).

## Highlights

- **Native SDF kernel** — primitives, CSG booleans, smooth blends, transforms, batched vectorized evaluation (xsimd; the batch runs serially — oneTBB is an off-by-default opt-in, [ADR-0009](docs/adr/0009-threading-model.md)) — no mesh in sight until you ask for one.
- **Marching Cubes mesh extraction** ([ADR-0015](docs/adr/0015-marching-cubes.md)) with **vertex welding** via canonical edge keys ([ADR-0016](docs/adr/0016-vertex-welding.md)).
- **Versioned native format** (`.CAD_0`, [ADR-0007](docs/adr/0007-native-format-versioning.md)): JSON feature tree + binary blob container ([ADR-0007a](docs/adr/0007a-binary-blob-container.md)) for embedded meshes, atomic save, transactional load, explicit migration chain (v0.1.0 → v0.2.0 → v0.3.0 → v0.4.0). That v0.x is the **format** axis — independent of the software version; see *The two version axes* in [CONTRIBUTING](CONTRIBUTING.md).
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

This builds the kernel + Python package, starts an Xvfb + fluxbox desktop with a **noVNC preview** (:6080), and launches the app — the same software-GL stack the CI UI tests use. See [docs/CODESPACES.md](docs/CODESPACES.md) for details and troubleshooting.

### Build locally

Prerequisites: C++20 compiler — the CI-tested columns are GCC 14 and Clang 19 (the `linux-*` presets pin `g++-14` / `clang++-19`; older compilers build via a plain `cmake -B build` but are not CI-tested; Windows/MSVC is registered, not CI-tested — see [docs/audits](docs/audits/2026-11-01-ci-runner-asan-and-attestations.md)), CMake ≥ 3.22, Ninja, Python ≥ 3.10 with dev headers (for the bindings).

```bash
git clone https://github.com/MarioW98/CAD_zero.git
cd CAD_zero

# Kernel + tests, no Python bindings
cmake -B build -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCAD_0_BUILD_PYTHON=OFF \
    -DCAD_0_BUILD_TESTING=ON
cmake --build build
ctest --test-dir build
```

Preset users: `cmake --preset linux-gcc-release` (or `linux-clang`, `linux-gcc-debug` with ASan). The presets pin the CI's compilers themselves (`g++-14` / `clang++-19`): a preset build IS the toolchain the CI judges — see the *Toolchain contract* in [docs/audits](docs/audits/).

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

### The command line — optional `[cli]` extra

The `CAD_0` command evaluates SDF scripts, exports meshes and prints body metadata — headless, no Qt involved. It rides on `typer`/`rich`, which are **not** base dependencies: install the `[cli]` extra and it runs; install the base package only and the command answers with the hint that names the extra instead of a traceback. The package is not on PyPI — install from a clone:

```bash
pip install ".[cli]"

CAD_0 info examples/sdf_sphere.py                    # metadata table
CAD_0 primitives                                     # the primitive/operator list
CAD_0 eval-sdf examples/sdf_sphere.py --output sphere.stl
CAD_0 view examples/sdf_lattice_ball.py              # opens a viewer window
CAD_0 slice examples/sdf_lattice_ball.py --plane z --at 0.0
```

`eval-sdf` writes STL/OBJ/3MF (format detected from the extension); `view`/`slice` open matplotlib windows and need a display.

## Use the kernel from Python

```python
import CAD_0
import numpy as np

# Build a sphere SDF — this is a *native* representation, not a mesh.
sph = CAD_0.sdf.sphere(radius=2.0)

# Boolean CSG — all native to the SDF domain.
cyl = CAD_0.sdf.cylinder(radius=0.5, height=10.0)
carved = CAD_0.sdf.subtract(sph, cyl)

# Smooth blend — equation-driven, mesh-free. (Offsets are Vec3f,
# not tuples: the bindings do not auto-convert.)
b = CAD_0.sdf.translate(CAD_0.sdf.sphere(radius=1.0),
                        CAD_0.math.Vec3f(3.0, 0.0, 0.0))
blended = CAD_0.sdf.smooth_union(sph, b, k=0.6)

# Batched evaluation — vectorized (serial: oneTBB is an opt-in).
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
│   ├── brep/            # The exact B-Rep kernel: topology, primitives, booleans, naming
│   ├── bridge/          # SDF ↔ B-Rep conversion (ADR-0026)
│   ├── concurrency/     # Thread pool + parallel_for (oneTBB opt-in, serial fallback)
│   ├── scene/           # Scene graph, SceneNode (Sdf|Mesh), builder registry
│   ├── viewport/        # Headless-first renderers (ADR-0017)
│   └── io/              # STL/OBJ/3MF, native .CAD_0 format
├── bindings/python/     # nanobind bindings + Python package (CLI included)
├── app/                 # PySide6 desktop app
├── scripts/             # Tooling: subject checker, suite launcher, build helper, Codespaces launcher
├── docs/                # The register: adr/, audits/, status/, history/, architecture/, manuals
├── examples/            # Python usage examples — the sdf_*.py are CLI-ready SDF scripts
├── tests/               # doctest unit + property suites, golden files
└── third_party/         # Vendored: the authentic Shewchuk robust.c (public domain, ADR-0034)
```

See [docs/architecture/overview.md](docs/architecture/overview.md) for the full layout and design rationale. Where every artifact lives — and why — is pinned in *Where things live* in [CONTRIBUTING](CONTRIBUTING.md); the dependency pins and the pin-vs-vendor decisions live in [docs/dependencies.md](docs/dependencies.md).

## Roadmap

| Phase | Scope | Status |
|-------|-------|--------|
| A | Core math, geometry, tolerance model | ✅ done |
| B | SDF kernel, marching cubes, vertex welding | ✅ done |
| C | Viewport architecture, PySide6 desktop app | ✅ done |
| C+ | Native format + migrators, STL/OBJ import, picking, CI matrix, toolchain contract | ✅ done |
| D | Radial-edge B-Rep topology, query operators, seam wires, convex booleans — the aligned coplanar union sewn by the overlay (H2-u5, [ADR-0037](docs/adr/0037-the-coplanar-class.md)) | ✅ done ([ADR-0021](docs/adr/0021-brep-topology-data-structure.md)–[0025](docs/adr/0025-proper-greiner-hormann.md), [0037](docs/adr/0037-the-coplanar-class.md)) |
| E | SDF → B-Rep extraction under hard contracts; the inverse bridge's sign is the true generalized winding number, watertight-only (open meshes refused by name), evaluated hierarchically over the ε_w-bounded BVH | ✅ done ([ADR-0026](docs/adr/0026-sdf-brep-bridge.md), the sign and its hierarchy: [ADR-0035 D2.4/D2.5](docs/adr/0035-h2-numerical-taxonomy.md)) |
| F | Modeling: boolean feature slice, sketcher MVP | ▶️ in progress — both named slices are in ([ADR-0027](docs/adr/0027-phase-f-boolean-feature-slice.md), [ADR-0030](docs/adr/0030-sketcher-mvp.md)); the constraint solver deferred with its entry criterion |
| G | The exact B-Rep kernel — road (a), all in house | ▶️ in progress — G1 complete and closed ([ADR-0032](docs/adr/0032-g1-topology-and-exact-primitives.md)); persistent naming landed ([ADR-0036](docs/adr/0036-persistent-naming-and-shape-history.md)); G2 unit 1 of 6 landed ([ADR-0033](docs/adr/0033-g2-exact-booleans-and-first-fillet.md)) |
| H | STEP AP203 MVP (ADR-0010) | planned |

**Platform vision** — the kernel as the geometric core of a broader CAE platform: a preprocessor's exact geometry, parametric vehicle geometry (G3, declared), and vortex/panel analysis consuming coherent surface meshes — [docs/architecture/platform-vision.md](docs/architecture/platform-vision.md). Orientation, not scope: the horizon never widens a rung.

The full record — the roadmap with its history and the cronaca of every rung — lives in [docs/status/](docs/status/README.md).

## Key architectural decisions

| ADR | Topic |
|-----|-------|
| [0001](docs/adr/0001-use-cpp20.md) | C++20 (subset) for the core kernel |
| [0002](docs/adr/0002-dual-kernel-native-sdf.md) | SDF is a first-class native representation; `Shape` uses `std::variant` |
| [0003](docs/adr/0003-third-party-policy.md) | No third-party CAD kernels; the license whitelist |
| [0004](docs/adr/0004-tolerance-model.md) | Tolerance is a first-class citizen; no global epsilon |
| [0005](docs/adr/0005-persistent-naming-early.md) | Persistent naming by genealogy + subshape tracing (NOT geometric signature) |
| [0006](docs/adr/0006-ui-primary-pyside6.md) | PySide6 (Python) is the primary UI; Qt6 C++ removed |
| [0007](docs/adr/0007-native-format-versioning.md) | Native format with explicit versioning + migration chain |
| [0007a](docs/adr/0007a-binary-blob-container.md) | Binary blob container for embedded meshes (v0.2.0, "raw" encoding; zstd deferred) |
| [0008](docs/adr/0008-simd-abstraction.md) | xsimd as SIMD abstraction; GPU backend stubbed from Phase A |
| [0009](docs/adr/0009-threading-model.md) | oneTBB + RAII GIL guard — OFF by default, serial fallback shipped (2026-11-01 amendment) |
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
| [0021](docs/adr/0021-brep-topology-data-structure.md) | B-Rep topology data structure — radial-edge entity set, ID scheme, ownership (Phase D (a)) |
| [0022](docs/adr/0022-brep-topology-query-operators.md) | B-Rep topology query operators + make_tetrahedron (Phase D (b)) |
| [0023](docs/adr/0023-analytic-seam-wires-and-boolean-intersect.md) | Analytic seam wires + bool_intersect on convex polyhedra (Phase D (c)) |
| [0024](docs/adr/0024-boolean-union-and-subtract.md) | bool_union / bool_subtract, centroid classification + identical-body shortcut (Phase D (c+)) |
| [0025](docs/adr/0025-proper-greiner-hormann.md) | Greiner-Hormann clipping infrastructure, kept for future use (Phase D (c+) fix) |
| [0026](docs/adr/0026-sdf-brep-bridge.md) | SDF → B-Rep bridge under hard topology contracts; the inverse's sign: the generalized winding number, watertight-only, hierarchical over the ε_w-bounded BVH (H2-u4) |
| [0027](docs/adr/0027-phase-f-boolean-feature-slice.md) | Phase F boolean feature slice: operation nodes, FeatureId operands, id-stable undo, v0.3 recipes |
| [0028](docs/adr/0028-sdf-bounds-smooth-blend-dilation.md) | SmoothMinSDF bounds() dilates by k/8 per blend level |
| [0029](docs/adr/0029-twist-warp-contract.md) | The twist warp contract (implemented): bounds, σ(k, r) Lipschitz, chain-rule gradient, validation |
| [0030](docs/adr/0030-sketcher-mvp.md) | The sketcher MVP (implemented): typed entities, extrude as a feature, v0.4; solver deferred |
| [0031](docs/adr/0031-exact-brep-kernel.md) | The exact B-Rep kernel — the great fork (accepted: road (a), all in house) |
| [0032](docs/adr/0032-g1-topology-and-exact-primitives.md) | G1 (implemented): true trimming, exact extrude/subtract, exact mass properties, χ = V−E+F−H |
| [0033](docs/adr/0033-g2-exact-booleans-and-first-fillet.md) | G2 (accepted): the analytic SSI table — the table is the wall (option i); option (ii) priced, registered |
| [0034](docs/adr/0034-h1-arithmetic-boundary.md) | H1: the arithmetic boundary — the authentic adaptive Shewchuk predicates vendored |
| [0035](docs/adr/0035-h2-numerical-taxonomy.md) | H2: the numerical taxonomy — typed tolerances, the derived determinant band, p-curves in double |
| [0036](docs/adr/0036-persistent-naming-and-shape-history.md) | Persistent naming and shape history (implemented): the derived name, the descent table, the resolver |
| [0037](docs/adr/0037-the-coplanar-class.md) | The coplanar class (accepted, implemented): the contact taxonomy, the composite ε_C gate, the aligned union sewn by the overlay |

## Testing, determinism, CI

- Every push runs the **full matrix on ubuntu-24.04**: GCC 14 debug (ASan/UBSan — C++ suite **and** the Python bindings under the preloaded sanitizer), GCC 14 release, Clang 19 — the C++ doctest suite, the bindings suite as a ctest member, and headless UI tests under Xvfb (`fail-fast` is off: one red job never hides the others). A dedicated job byte-compiles the tree under Python 3.10 (the syntax floor).
- **Determinism is a contract, not a hope** ([ADR-0014](docs/adr/0014-floating-point-determinism.md)): `-ffp-contract=off`, round-trip tests that must be *bit-identical*, golden files that must be *byte-identical*. Fixes may not change evaluation values.
- **Toolchain contract**: warning audits and any `-Werror` change must be verified against the CI compilers (GCC 14, Clang 19) before push — the presets pin them (`g++-14` / `clang++-19`), so a preset build IS the verification. See the audits in [docs/audits/](docs/audits/).

## Contributing

The project follows an **ADR-first culture**: architectural decisions are written, reviewed and recorded in `docs/adr/` before the code that implements them; regressions get a failing test before the fix. Open a Codespace (everything needed to build, run and test ships in the dev container), branch from `main`, and keep commits descriptive. The conventions — subjects, the pre-delivery rite, where every artifact lives — are in [CONTRIBUTING](CONTRIBUTING.md).

## License

Apache-2.0. See [LICENSE](LICENSE) and [NOTICE](NOTICE) — the NOTICE lists every third-party component and its license; the `[ui]` extra pulls in PySide6, the only copyleft (LGPL-3.0) in the tree, dynamically imported and replaceable. The full inventory, the per-dependency flags for closed-source derivatives, and the H1 provenance correction for the vendored predicates: [docs/audits/2026-10-16-third-party-license-audit.md](docs/audits/2026-10-16-third-party-license-audit.md).

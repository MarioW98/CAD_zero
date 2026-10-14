# CAD_0

[![CI](https://github.com/MarioW98/CAD_zero/actions/workflows/ci-linux.yml/badge.svg)](https://github.com/MarioW98/CAD_zero/actions/workflows/ci-linux.yml)
[![License: Apache-2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

**Dual-representation CAD kernel — native SDF + B-Rep, in C++20 and Python.**

CAD_0 is a from-scratch CAD kernel where **SDF (signed distance fields) and B-Rep (boundary representation) are first-class peers**, inspired by Siemens NX *Convergent Modeling*. There is no "primary" representation: every operation has a native implementation in both domains, and the system dispatches on operand types.

The long-term goal is an **open-source CAD system that can stand next to commercial modelers** — built without any third-party CAD kernel (no OpenCascade, no Parasolid, no ACIS — [ADR-0003](docs/adr/0003-third-party-policy.md)), so every geometric decision lives in this repository and can be tested deterministically ([ADR-0014](docs/adr/0014-floating-point-determinism.md)).

> **Status — Phase D step (c+ fix v3) complete.** An interactive desktop app (PySide6) on top of the kernel: parametric shape creation, undo/redo, viewport picking with 3D highlight, STL/OBJ import with vertex welding, STL/OBJ/3MF export, and a versioned native project format (`.CAD_0` v0.2.0) with a migration chain. **Phase D steps (a)–(c)** landed the B-Rep topology data structure ([ADR-0021](docs/adr/0021-brep-topology-data-structure.md)), query operators ([ADR-0022](docs/adr/0022-brep-topology-query-operators.md)), seam wires + `bool_intersect` ([ADR-0023](docs/adr/0023-analytic-seam-wires-and-boolean-intersect.md)). **Phase D step (c+)** ([ADR-0024](docs/adr/0024-boolean-union-and-subtract.md)) added `bool_union`/`bool_subtract`. **Phase D step (c+ fix v3)** ([ADR-0025](docs/adr/0025-proper-greiner-hormann.md)) rewrites `bool_union` AND `bool_subtract` with the correct algorithm: `polygon_subtract_3d` via Greiner-Hormann with collinear-edge handling (`IntersectionType::Overlap`), three-state `classify_point` (Inside/Outside/OnBoundary), bounded walks to prevent SIGSEGV, identical-body shortcut. Both operations now produce correct face fragments for partial overlaps, corner notches, and B-contains-A cases. Next up: **Phase E** — SDF ↔ B-Rep bridge. The CI matrix (GCC 11 debug + release, Clang 14) is green on `main`.

<!-- TODO(badge GIF): record a 30–60s noVNC demo (create sphere → edit radius →
     pick in viewport → import STL → save/reopen) and embed it here:
     <p align="center"><img src="docs/media/demo.gif" width="640" alt="CAD_0 desktop app demo"></p> -->

## Highlights

- **Native SDF kernel** — primitives, CSG booleans, smooth blends, transforms, batched vectorized evaluation (xsimd, multi-threaded) — no mesh in sight until you ask for one.
- **Marching Cubes mesh extraction** ([ADR-0015](docs/adr/0015-marching-cubes.md)) with **vertex welding** via canonical edge keys ([ADR-0016](docs/adr/0016-vertex-welding.md)).
- **Versioned native format** (`.CAD_0`, [ADR-0007](docs/adr/0007-native-format-versioning.md)): JSON feature tree + binary blob container ([ADR-0007a](docs/adr/0007a-binary-blob-container.md)) for embedded meshes, atomic save, transactional load, explicit migration chain (v0.1.0 → v0.2.0).
- **STL/OBJ import** (welded, first-class mesh scene nodes) and **STL/OBJ/3MF export**.
- **Desktop app** (PySide6, [ADR-0006](docs/adr/0006-ui-primary-pyside6.md)): registry-driven Create menu, feature tree, parametric property panel, undo/redo, picking with amber 3D highlight.
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
- **Viewport picking** — click a shape in the 3D view (SDF or imported mesh) to select it: the tree, the property panel and an amber highlight in the viewport all follow, whatever the selection origin.
- **Import STL/OBJ** as first-class mesh nodes (vertices welded, counts reported); **export STL/OBJ/3MF**.
- **Open… / Save / Save As** (`Ctrl+O`, `Ctrl+S`, `Ctrl+Shift+S`) in the native `.CAD_0` format (v0.2.0 — [ADR-0007](docs/adr/0007-native-format-versioning.md) / [ADR-0007a](docs/adr/0007a-binary-blob-container.md)):
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
| **D** | **Radial-edge B-Rep topology (ADR-0012) — ADR first, then pure-topology MVP** | ▶️ steps (a)+(b)+(c)+(c+) done ([ADR-0021](docs/adr/0021-brep-topology-data-structure.md), [0022](docs/adr/0022-brep-topology-query-operators.md), [0023](docs/adr/0023-analytic-seam-wires-and-boolean-intersect.md), [0024](docs/adr/0024-boolean-union-and-subtract.md), [0025](docs/adr/0025-proper-greiner-hormann.md)) |
| E | SDF ↔ B-Rep bridge | planned |
| F | Modeling: feature history, sketcher | planned |
| H | STEP AP203 MVP (ADR-0010) | planned |

## Key architectural decisions

| ADR | Topic |
|-----|-------|
| [0001](docs/adr/0001-use-cpp20.md) | C++20 (subset) for the core kernel |
| [0002](docs/adr/0002-dual-kernel-native-sdf.md) | **SDF is a first-class native representation; `Shape` uses `std::variant`** |
| [0003](docs/adr/0003-third-party-policy.md) | No third-party CAD kernels (OpenCascade, Parasolid, ACIS) |
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

## Testing, determinism, CI

- Every push runs the **full matrix on ubuntu-22.04**: GCC 11 debug (ASan), GCC 11 release, Clang 14 — C++ doctest suite, Python bindings tests, and headless UI tests under Xvfb (`fail-fast` is off: one red job never hides the others).
- **Determinism is a contract, not a hope** ([ADR-0014](docs/adr/0014-floating-point-determinism.md)): `-ffp-contract=off`, round-trip tests that must be *bit-identical*, golden files that must be *byte-identical*. Fixes may not change evaluation values.
- **Toolchain contract**: warning audits and any `-Werror` change must be verified against the CI compilers (GCC 11.4, Clang 14) before push — reproducible locally via the `linux-gcc11-ci` preset. See the audits in [docs/audits/](docs/audits/).

## Contributing

The project follows an **ADR-first culture**: architectural decisions are written, reviewed and recorded in `docs/adr/` before the code that implements them; regressions get a failing test before the fix. Open a Codespace (everything needed to build, run and test ships in the dev container), branch from `main`, and keep commits descriptive.

## License

Apache-2.0. See [LICENSE](LICENSE).

The vendored Shewchuk robust predicates under `third_party/robust_predicates/` are public domain; all other dependencies keep their original licenses (MIT, BSD-3, Apache-2.0, Boost).
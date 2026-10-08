# CAD_0

**Dual-representation CAD kernel — native SDF + B-Rep, in C++20 and Python.**

CAD_0 is a from-scratch CAD kernel where **SDF (signed distance fields) and B-Rep (boundary representation) are first-class peers**, inspired by Siemens NX *Convergent Modeling*. There is no "primary" representation: every operation has native implementations in both domains, and the system dispatches based on operand types.

> **Status**: Phase C (viewport + UI) completed. See `docs/adr/` for the architectural decisions, and `docs/architecture/` for the high-level overview. 165/165 tests passing.

---

## Quick start

### Prerequisites

- C++20 compiler: GCC ≥ 11, Clang ≥ 14, or MSVC ≥ 17.10
- CMake ≥ 3.22 (or install via `pip install cmake`)
- Ninja (recommended, install via `pip install ninja`)
- Python ≥ 3.10 with dev headers (for Python bindings)
- For Python bindings: `pip install nanobind scikit-build-core`

### Build (Linux + GCC, no Python bindings)

```bash
git clone https://github.com/your-org/CAD_0.git
cd CAD_0
cmake -B build -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCAD_0_BUILD_PYTHON=OFF \
    -DCAD_0_BUILD_TESTING=ON
cmake --build build
ctest --test-dir build
```

### Build with Python bindings

The Python bindings require Python dev headers + nanobind. If you use a
venv (e.g. uv-managed), make sure to point CMake at the right interpreter:

```bash
# Install build dependencies in your venv
pip install cmake ninja nanobind scikit-build-core

# Configure — pass Python3_EXECUTABLE so CMake finds the right Python
cmake -B build -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCAD_0_BUILD_PYTHON=ON \
    -DCAD_0_BUILD_TESTING=ON \
    -DPython3_EXECUTABLE=$(which python3)

cmake --build build

# Test the Python extension
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

### Troubleshooting: "Could NOT find Python3 (missing: Development.Module)"

This means CMake found a Python interpreter but not its development headers.
Fixes:

1. **Debian/Ubuntu (system Python):**
   ```bash
   sudo apt install python3-dev
   ```

2. **Venv (uv-managed or pip-managed):**
   - The venv must have its own dev headers. uv-managed Pythons ship them
     under `<uv-cache>/python/cpython-3.XX-*/include/python3.XX/Python.h`.
   - Pass `-DPython3_EXECUTABLE=/path/to/venv/bin/python3` to CMake so it
     picks the right interpreter.

3. **Multiple Python versions:**
   - CMake might find Python 3.13 (with dev headers) when your venv uses
     3.12 (without). Always pass `-DPython3_EXECUTABLE=` explicitly.

### Use the kernel from Python

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
CAD_0.io.export('output.stl', mesh)  # auto-detects format from extension
```

### Use the kernel from C++

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

---

## Repository layout

```
CAD_0/
├── core/                # C++20 kernel (zero UI deps)
│   ├── math/            # vec, mat, quat, bbox, tolerance, robust predicates
│   ├── geometry/        # Shape (std::variant), ShapeId, Transform, Visitor
│   ├── sdf/            # Native SDF kernel: primitives, operators, transforms, evaluate
│   ├── concurrency/    # TBB thread pool, GIL guard, task graph
│   ├── brep/            # (Phase D) B-Rep kernel
│   ├── bridge/          # (Phase E) SDF ↔ B-Rep conversion
│   ├── modeling/        # (Phase F) feature tree, sketch
│   └── io/              # (Phase H) STEP, 3MF, STL, native format
├── bindings/python/    # nanobind bindings + Python package
├── app/                # PySide6 desktop app resources (Phase C)
├── docs/               # architecture + ADRs
├── examples/           # Python usage examples
├── tests/              # doctest unit + regression + fuzz
└── third_party/        # Vendored Shewchuk predicates (public domain)
```

See `docs/architecture/overview.md` for the full layout and design rationale.

---

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

---

## License

Apache-2.0. See [LICENSE](LICENSE).

The vendored Shewchuk robust predicates under `third_party/robust_predicates/` are public domain; all other dependencies keep their original licenses (MIT, BSD-3, Apache-2.0, Boost).

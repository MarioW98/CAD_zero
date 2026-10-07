# CAD _test _from zero

**Dual-representation CAD kernel — native SDF + B-Rep, in C++20 and Python.**

cadforge is a from-scratch CAD kernel where **SDF (signed distance fields) and B-Rep (boundary representation) are first-class peers**, inspired by Siemens NX *Convergent Modeling*. There is no "primary" representation: every operation has native implementations in both domains, and the system dispatches based on operand types.

> **Status**: Phase A (foundations) is in progress. See `docs/adr/` for the architectural decisions, and `docs/architecture/` for the high-level overview.

---

## Quick start

### Build (Linux + GCC)

```bash
git clone https://github.com/your-org/cadforge.git
cd cadforge
cmake --preset linux-gcc-release
cmake --build --preset linux-gcc-release
ctest --preset linux-gcc-release
```

### Install the Python package (editable)

```bash
pip install -e .[dev]
python examples/sdf_sphere.py
```

### Use the kernel from Python

```python
import cadforge
import numpy as np

# Build a sphere SDF — this is a *native* representation, not a mesh.
sph = cadforge.sdf.sphere(radius=2.0)

# Boolean CSG — all native to the SDF domain.
cyl = cadforge.sdf.cylinder(radius=0.5, height=10.0)
carved = cadforge.sdf.subtract(sph, cyl)

# Smooth blend — equation-driven, mesh-free.
b = cadforge.sdf.translate(cadforge.sdf.sphere(radius=1.0), (3.0, 0.0, 0.0))
blended = cadforge.sdf.smooth_union(sph, b, k=0.6)

# Batched evaluation — vectorized, multi-threaded.
pts = np.random.uniform(-5, 5, (1024, 3)).astype(np.float32)
result = cadforge.sdf.evaluate(blended, pts)
print(result.values[:8])
```

### Use the kernel from C++

```cpp
#include <cadforge/sdf/primitives.hpp>
#include <cadforge/sdf/operators.hpp>
#include <cadforge/sdf/transforms.hpp>
#include <cadforge/sdf/evaluate.hpp>

using namespace cadforge;

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
cadforge/
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

---

## License

Apache-2.0. See [LICENSE](LICENSE).

The vendored Shewchuk robust predicates under `third_party/robust_predicates/` are public domain; all other dependencies keep their original licenses (MIT, BSD-3, Apache-2.0, Boost).

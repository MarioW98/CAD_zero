# ADR-0020: Python bindings build configuration

## Status
Accepted (2025-10-07)

## Context
The CMake configuration for Python bindings required several fixes to
support common build environments, including system Python installs,
uv-managed venvs, and conda environments. The issues encountered:

1. `find_package(Python3 COMPONENTS Development.Module REQUIRED)` was
   called inside `bindings/python/CMakeLists.txt`, but `core/concurrency/`
   also called it independently. This caused inconsistent state when
   CMake's `Development.Module` component couldn't be found.

2. The `nanobind-static` target created by `nanobind_add_module` did
   not inherit `Python3_INCLUDE_DIRS`. When Python dev headers were
   in a non-standard location (e.g. uv-managed venvs), compilation
   of `nb_internals.cpp` failed with `fatal error: Python.h: No such
   file or directory`.

3. The `_CAD_0` target was not linked against `CAD_0_io` and
   `CAD_0_viewport`. Python-side calls to `CAD_0.io.export_*`
   failed at runtime with `undefined symbol`.

4. The `bind_mesh()` function created its own `sdf` submodule inside
   `io`, causing a name conflict. `CAD_0.sdf.marching_cubes` was
   therefore unavailable (`AttributeError`).

5. Missing `<nanobind/stl/string.h>` include in binding files.
   `std::string` return values (e.g. `SDFBody::describe()`) could
   not be converted to Python `str`.

6. Member-function pointers passed directly to nanobind `def()` for
   overloaded C++ operators (e.g. `Vec3f::operator+`) failed overload
   resolution.

## Decision

### 1. Centralize `find_package(Python3)` at top level

`find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)`
is called once in the top-level `CMakeLists.txt`, before
`add_subdirectory(core)`. The `Python3::Module` imported target is
then available to all subdirectories.

### 2. Add `Python3_INCLUDE_DIRS` to nanobind-static after creation

```cmake
nanobind_add_module(_CAD_0 ...)
if(TARGET nanobind-static)
    target_include_directories(nanobind-static PRIVATE ${Python3_INCLUDE_DIRS})
endif()
```

This must be called after `nanobind_add_module` because that function
creates the `nanobind-static` target.

### 3. Link all CAD_0 libraries to `_CAD_0`

```cmake
target_link_libraries(_CAD_0 PRIVATE
    CAD_0_math
    CAD_0_geometry
    CAD_0_sdf
    CAD_0_concurrency
    CAD_0_io
    CAD_0_viewport
    CAD_0_brep
)
```

### 4. Separate `bind_mesh` (SDF module) from `bind_io` (IO submodule)

```cpp
bind_math(math_mod);
bind_geometry(geometry_mod);
bind_sdf(sdf_mod);
bind_mesh(sdf_mod);  // TriangleMesh, MarchingCubesOptions, marching_cubes, weld_vertices
bind_io(m);          // creates CAD_0.io submodule with export_stl_binary/ascii/obj/3mf
```

### 5. Include `<nanobind/stl/string.h>` in binding files

Required whenever a bound function returns `std::string`. Examples:
`SDFBody::describe()`, `ShapeId::to_string()`.

### 6. Use lambda wrappers for overloaded operators

```cpp
// Ambiguous overload resolution (does not compile):
.def("__add__", &Vec3f::operator+)

// Explicit lambda (compiles):
.def("__add__", [](const Vec3f& a, const Vec3f& b) { return a + b; })
```

## Required Python packages

The build requires the following packages in the Python environment
used by CMake:

- `cmake` (≥ 3.22)
- `ninja`
- `nanobind` (≥ 2.2)
- `scikit-build-core` (for `pip install` integration)

Install in the venv:

```bash
pip install cmake ninja nanobind scikit-build-core
```

## Required system packages

- `python3-dev` (Debian/Ubuntu) or `python3-devel` (Fedora/RHEL)
- Or a Python distribution that ships dev headers (uv-managed, conda, pyenv)

## Build invocation

Pass `Python3_EXECUTABLE` explicitly when using a non-default Python:

```bash
cmake -B build -G Ninja \
    -DCAD_0_BUILD_PYTHON=ON \
    -DPython3_EXECUTABLE=$(which python3) \
    ...
```

Alternatively, use the convenience script:

```bash
./scripts/build.sh --python --test --clean
```

## Verification

After these fixes, the Python bindings work end-to-end:

```
Test 1: SDF primitives                       PASS
Test 2: Batched evaluation                   PASS
Test 3: Mesh extraction (3810 vertices)      PASS
Test 4: Export STL (224 KB)                  PASS
Test 5: Boolean operations                    PASS
Test 6: Export all formats (OBJ + 3MF)      PASS
```

C++ tests still pass: 165/165.

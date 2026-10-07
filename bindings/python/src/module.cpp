// bindings/python/src/module.cpp
//
// Top-level nanobind module for CAD_0.
// Submodules: math, sdf, geometry, io (Phase B.5+).
//
// Build: see bindings/python/CMakeLists.txt
//
#include <nanobind/nanobind.h>

namespace nb = nanobind;

void bind_math(nb::module_&);
void bind_geometry(nb::module_&);
void bind_sdf(nb::module_&);
void bind_mesh(nb::module_&);

NB_MODULE(_CAD_0, m) {
    m.doc() = "CAD_0 — dual-representation CAD kernel (native SDF + B-Rep)";

    auto math_mod     = m.def_submodule("math",     "Vector / matrix / quaternion math");
    auto geometry_mod = m.def_submodule("geometry", "Shape, ShapeId, Transform, DatumCS");
    auto sdf_mod      = m.def_submodule("sdf",      "Native SDF kernel + mesh extraction");
    auto io_mod       = m.def_submodule("io",       "Mesh export (STL / OBJ / 3MF)");

    bind_math(math_mod);
    bind_geometry(geometry_mod);
    bind_sdf(sdf_mod);
    bind_mesh(io_mod);  // bind_mesh() registers both mesh extraction and IO
    (void)io_mod;
}

// bindings/python/src/module.cpp
//
// Top-level nanobind module for CAD_0.
//
#include <nanobind/nanobind.h>

namespace nb = nanobind;

void bind_math(nb::module_&);
void bind_geometry(nb::module_&);
void bind_sdf(nb::module_&);
void bind_mesh(nb::module_&);
void bind_io(nb::module_&);
void bind_scene(nb::module_&);

NB_MODULE(_CAD_0, m) {
    m.doc() = "CAD_0 - dual-representation CAD kernel (native SDF + B-Rep)";

    auto math_mod     = m.def_submodule("math",     "Vector / matrix / quaternion math");
    auto geometry_mod = m.def_submodule("geometry", "Shape, ShapeId, Transform, DatumCS");
    auto sdf_mod      = m.def_submodule("sdf",      "Native SDF kernel + mesh extraction");

    bind_math(math_mod);
    bind_geometry(geometry_mod);
    bind_sdf(sdf_mod);
    bind_mesh(sdf_mod);
    bind_io(m);
    bind_scene(m);
}

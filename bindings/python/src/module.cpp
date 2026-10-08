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
void bind_mesh(nb::module_&);   // binds TriangleMesh + marching_cubes to SDF module
void bind_io(nb::module_&);    // binds STL/OBJ/3MF export to a new io submodule

NB_MODULE(_CAD_0, m) {
    m.doc() = "CAD_0 - dual-representation CAD kernel (native SDF + B-Rep)";

    auto math_mod     = m.def_submodule("math",     "Vector / matrix / quaternion math");
    auto geometry_mod = m.def_submodule("geometry", "Shape, ShapeId, Transform, DatumCS");
    auto sdf_mod      = m.def_submodule("sdf",      "Native SDF kernel + mesh extraction");

    bind_math(math_mod);
    bind_geometry(geometry_mod);
    bind_sdf(sdf_mod);
    bind_mesh(sdf_mod);  // TriangleMesh, MarchingCubesOptions, marching_cubes, weld_vertices
    bind_io(m);          // creates CAD_0.io submodule with export_stl_binary/ascii/obj/3mf
}

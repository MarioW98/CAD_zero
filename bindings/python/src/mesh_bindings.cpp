// bindings/python/src/mesh_bindings.cpp
//
// Python bindings for the SDF mesh extraction + IO module.
//
// Exposes:
//   * CAD_0.sdf.TriangleMesh          — mesh result type
//   * CAD_0.sdf.MarchingCubesOptions  — extraction options
//   * CAD_0.sdf.marching_cubes(...)   — extraction (multiple overloads)
//   * CAD_0.sdf.weld_vertices(...)    — vertex welding utility
//   * CAD_0.io.export_stl_binary(...)
//   * CAD_0.io.export_stl_ascii(...)
//   * CAD_0.io.export_obj(...)
//   * CAD_0.io.export_3mf(...)
//
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/string_view.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/vector.h>

#include "CAD_0/sdf/field.hpp"
#include "CAD_0/sdf/mesh_extract.hpp"
#include "CAD_0/io/stl.hpp"
#include "CAD_0/io/obj.hpp"
#include "CAD_0/io/threemf.hpp"

#include "CAD_0/math/vec.hpp"
#include "CAD_0/math/bbox.hpp"

namespace nb = nanobind;
using namespace CAD_0;

void bind_mesh(nb::module_& m) {
    auto sdf_mod = m.def_submodule("sdf", "SDF mesh extraction (already exists if sdf module bound)");

    // TriangleMesh — return as numpy arrays + counts.
    nb::class_<sdf::TriangleMesh>(sdf_mod, "TriangleMesh")
        .def_ro("positions", [](const sdf::TriangleMesh& mesh) {
            return nb::ndarray<nb::numpy, float, nb::ndim<2>>(
                const_cast<float*>(reinterpret_cast<const float*>(mesh.positions.data())),
                {mesh.positions.size(), std::size_t(3)},
                nb::handle());
        })
        .def_ro("normals", [](const sdf::TriangleMesh& mesh) {
            if (mesh.normals.empty()) {
                return nb::ndarray<nb::numpy, float, nb::ndim<2>>();
            }
            return nb::ndarray<nb::numpy, float, nb::ndim<2>>(
                const_cast<float*>(reinterpret_cast<const float*>(mesh.normals.data())),
                {mesh.normals.size(), std::size_t(3)},
                nb::handle());
        })
        .def_ro("indices", [](const sdf::TriangleMesh& mesh) {
            return nb::ndarray<nb::numpy, std::uint32_t, nb::ndim<1>>(
                const_cast<std::uint32_t*>(mesh.indices.data()),
                {mesh.indices.size()},
                nb::handle());
        })
        .def("vertex_count",   &sdf::TriangleMesh::vertex_count)
        .def("triangle_count", &sdf::TriangleMesh::triangle_count);

    // MarchingCubesOptions.
    nb::class_<sdf::MarchingCubesOptions>(sdf_mod, "MarchingCubesOptions")
        .def(nb::init<>())
        .def_rw("resolution",            &sdf::MarchingCubesOptions::resolution)
        .def_rw("compute_normals",      &sdf::MarchingCubesOptions::compute_normals)
        .def_rw("weld_vertices",         &sdf::MarchingCubesOptions::weld_vertices)
        .def_rw("weld_tolerance_scale", &sdf::MarchingCubesOptions::weld_tolerance_scale);

    // marching_cubes(body, resolution=64, compute_normals=True)
    sdf_mod.def("marching_cubes",
        [](const sdf::SDFBody& body, std::uint32_t resolution, bool compute_normals) {
            return sdf::marching_cubes(body, resolution, compute_normals);
        },
        nb::arg("body"), nb::arg("resolution") = 64, nb::arg("compute_normals") = true);

    // marching_cubes(body, bounds, opts)
    sdf_mod.def("marching_cubes",
        [](const sdf::SDFBody& body, const math::Bboxf& bounds, const sdf::MarchingCubesOptions& opts) {
            return sdf::marching_cubes(body, bounds, opts);
        },
        nb::arg("body"), nb::arg("bounds"), nb::arg("opts"));

    // weld_vertices(mesh, tolerance)
    sdf_mod.def("weld_vertices",
        [](sdf::TriangleMesh& mesh, float tolerance) {
            return sdf::weld_vertices(mesh, tolerance);
        },
        nb::arg("mesh"), nb::arg("tolerance"),
        nb::rv_policy::reference_internal);

    // IO module
    auto io_mod = m.def_submodule("io", "Mesh export (STL / OBJ / 3MF)");

    io_mod.def("export_stl_binary",
        [](const std::string& path, const sdf::TriangleMesh& mesh,
           std::string_view name) {
            return io::export_stl_binary(path, mesh, name);
        },
        nb::arg("path"), nb::arg("mesh"), nb::arg("name") = "CAD_0");

    io_mod.def("export_stl_ascii",
        [](const std::string& path, const sdf::TriangleMesh& mesh,
           std::string_view name) {
            return io::export_stl_ascii(path, mesh, name);
        },
        nb::arg("path"), nb::arg("mesh"), nb::arg("name") = "CAD_0");

    io_mod.def("export_obj",
        [](const std::string& path, const sdf::TriangleMesh& mesh,
           std::string_view name) {
            return io::export_obj(path, mesh, name);
        },
        nb::arg("path"), nb::arg("mesh"), nb::arg("name") = "CAD_0");

    io_mod.def("export_3mf",
        [](const std::string& path, const sdf::TriangleMesh& mesh,
           std::string_view name, std::string_view application) {
            return io::export_3mf(path, mesh, name, application);
        },
        nb::arg("path"), nb::arg("mesh"),
        nb::arg("name") = "CAD_0", nb::arg("application") = "CAD_0");
}

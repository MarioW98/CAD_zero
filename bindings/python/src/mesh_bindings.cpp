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
#include "CAD_0/scene/scene.hpp"
#include "CAD_0/io/native.hpp"
#include "CAD_0/io/stl.hpp"
#include "CAD_0/io/obj.hpp"
#include "CAD_0/io/threemf.hpp"

#include "CAD_0/math/vec.hpp"
#include "CAD_0/math/bbox.hpp"

namespace nb = nanobind;
using namespace CAD_0;
namespace native = CAD_0::io::native;

void bind_mesh(nb::module_& m) {
    // TriangleMesh and marching_cubes are exposed on the SDF submodule
    // passed in by the caller (typically the existing `CAD_0.sdf` module).
    nb::module_& sdf_mod = m;

    // TriangleMesh — return as numpy arrays + counts.
    //
    // Bug 4 fix (lifetime safety): the ndarray views used to be created with
    // an empty `nb::handle()` owner — numpy arrays kept pointing at the C++
    // vectors WITHOUT keeping the TriangleMesh alive:
    //
    //     arr = scene.node_at(0).cached_mesh.positions
    //     del scene          # C++ mesh freed -> arr dangles (use-after-free)
    //
    // `nb::find(&mesh)` returns the Python wrapper of the mesh the property
    // was called on, so the ndarray holds a reference to it and the C++
    // memory stays alive as long as the numpy array exists.
    nb::class_<sdf::TriangleMesh>(sdf_mod, "TriangleMesh")
        .def_prop_ro("positions", [](const sdf::TriangleMesh& mesh) {
            return nb::ndarray<nb::numpy, float, nb::ndim<2>>(
                const_cast<float*>(reinterpret_cast<const float*>(mesh.positions.data())),
                {mesh.positions.size(), std::size_t(3)},
                nb::find(&mesh));
        })
        .def_prop_ro("normals", [](const sdf::TriangleMesh& mesh) {
            if (mesh.normals.empty()) {
                return nb::ndarray<nb::numpy, float, nb::ndim<2>>();
            }
            return nb::ndarray<nb::numpy, float, nb::ndim<2>>(
                const_cast<float*>(reinterpret_cast<const float*>(mesh.normals.data())),
                {mesh.normals.size(), std::size_t(3)},
                nb::find(&mesh));
        })
        .def_prop_ro("indices", [](const sdf::TriangleMesh& mesh) {
            return nb::ndarray<nb::numpy, std::uint32_t, nb::ndim<1>>(
                const_cast<std::uint32_t*>(mesh.indices.data()),
                {mesh.indices.size()},
                nb::find(&mesh));
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

    // weld_vertices(mesh, tolerance) — returns the number of vertices
    // after welding (std::size_t). Bug 4 cleanup: the binding carried a
    // leftover `rv_policy::reference_internal`, which is meaningless for an
    // arithmetic return type (likely a leftover from a mesh-returning API).
    sdf_mod.def("weld_vertices",
        [](sdf::TriangleMesh& mesh, float tolerance) {
            return sdf::weld_vertices(mesh, tolerance);
        },
        nb::arg("mesh"), nb::arg("tolerance"));
}

// ---- Native-format camera helpers (dict <-> NativeCamera) -------------------
// Must be called with the GIL held (they touch Python objects).

static native::NativeCamera native_camera_from_dict(const nb::dict& d) {
    native::NativeCamera cam{};
    auto get_number = [&](const char* key, float& out) {
        if (!d.contains(key))
            throw std::runtime_error(std::string("save_native: camera dict is "
                                                 "missing key '") + key + "'");
        const double v = nb::cast<double>(d[key]);
        out = static_cast<float>(v);
    };
    if (!d.contains("target"))
        throw std::runtime_error("save_native: camera dict is missing key 'target'");
    nb::object tgt = d["target"];
    if (!nb::isinstance<nb::tuple>(tgt) && !PySequence_Check(tgt.ptr()))
        throw std::runtime_error("save_native: camera 'target' must be a 3-sequence");
    if (nb::len(tgt) != 3)
        throw std::runtime_error("save_native: camera 'target' must have 3 elements");
    cam.target = math::Vec3f{
        static_cast<float>(nb::cast<double>(tgt[0])),
        static_cast<float>(nb::cast<double>(tgt[1])),
        static_cast<float>(nb::cast<double>(tgt[2])),
    };
    get_number("distance", cam.distance);
    get_number("yaw", cam.yaw);
    get_number("pitch", cam.pitch);
    get_number("fov_deg", cam.fov_deg);
    return cam;
}

static nb::dict native_camera_to_dict(const native::NativeCamera& cam) {
    nb::dict out;
    out["target"] = nb::make_tuple(cam.target.x, cam.target.y, cam.target.z);
    out["distance"] = cam.distance;
    out["yaw"] = cam.yaw;
    out["pitch"] = cam.pitch;
    out["fov_deg"] = cam.fov_deg;
    return out;
}

// IO bindings — exported to a separate io submodule.
void bind_io(nb::module_& m) {
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

    // ---- Native project format (.CAD_0, ADR-0007) --------------------------
    //
    // save_native(path, scene, camera=None)
    //   Serializes scene + camera to the native JSON format and writes it
    //   ATOMICALLY (<path>.tmp + rename). Raises RuntimeError with an
    //   explicit message on failure; the previous file is untouched.
    //
    // load_native(path, scene)
    //   Transactional load: the document is fully parsed and validated
    //   FIRST (with the GIL released), and the scene is only mutated if
    //   everything succeeded — a failed load leaves the scene untouched.
    //   NodeAdded/SceneCleared callbacks fire during install (GIL held).
    //   Returns the saved camera as a dict:
    //     {"target": (x, y, z), "distance": d, "yaw": y, "pitch": p,
    //      "fov_deg": f}
    //
    // The camera dict mirrors the desktop app's orbit camera; fov is in
    // degrees. Do NOT mutate the scene from other Python threads while a
    // save is in flight (the GIL is released for the C++ write).

    io_mod.attr("NATIVE_VERSION") = "0.1.0";

    io_mod.def("save_native",
        [](const std::string& path, const scene::Scene& s, nb::object camera) {
            // Convert the camera dict with the GIL held, then release it
            // for the pure-C++ serialize + atomic write.
            native::NativeCamera cam{};
            if (!camera.is_none()) {
                if (!nb::isinstance<nb::dict>(camera))
                    throw std::runtime_error(
                        "save_native: 'camera' must be a dict or None");
                const nb::dict d = nb::cast<nb::dict>(camera);
                cam = native_camera_from_dict(d);
            }
            nb::gil_scoped_release release;
            native::save_file(path, s, cam);
        },
        nb::arg("path"), nb::arg("scene"), nb::arg("camera") = nb::none());

    io_mod.def("load_native",
        [](const std::string& path, scene::Scene& s) {
            native::NativeCamera cam;
            {
                // Phase 1 — parse + validate (pure C++, GIL released).
                // Throws with an explicit message on any error; the
                // scene is NOT touched in that case.
                nb::gil_scoped_release release;
                native::ParsedDocument doc = native::load_file(path);
                cam = doc.camera;
                // Phase 2 — re-acquire the GIL and commit (callbacks may
                // call back into Python here, so the GIL must be held).
                nb::gil_scoped_acquire acquire;
                native::install_document(s, std::move(doc));
            }
            return native_camera_to_dict(cam);
        },
        nb::arg("path"), nb::arg("scene"));
}

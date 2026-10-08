// bindings/python/src/sdf_bindings.cpp
//
// Exposes the native SDF kernel to Python. The public API mirrors the
// C++ helpers in core/sdf/primitives.hpp and operators.hpp, plus the
// batched evaluation API in core/sdf/evaluate.hpp.
//
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/string.h>

#include "CAD_0/sdf/field.hpp"
#include "CAD_0/sdf/primitives.hpp"
#include "CAD_0/sdf/operators.hpp"
#include "CAD_0/sdf/transforms.hpp"
#include "CAD_0/sdf/evaluate.hpp"

#include "CAD_0/math/vec.hpp"
#include "CAD_0/math/quat.hpp"

#include <memory>

namespace nb = nanobind;
using namespace CAD_0::sdf;
using namespace CAD_0::math;

void bind_sdf(nb::module_& m) {
    // SDFBody — opaque handle to an SDF tree root.
    nb::class_<SDFBody>(m, "SDFBody")
        .def(nb::init<>())
        .def("value",     &SDFBody::value)
        .def("sample",    [](const SDFBody& b, const Vec3f& p) { return b.sample(p); })
        .def("lipschitz",  &SDFBody::lipschitz)
        .def("bounds",     &SDFBody::bounds)
        .def("describe",   &SDFBody::describe)
        .def("__repr__",   [](const SDFBody& b) {
            return "<SDFBody " + b.describe() + ">";
        });

    // Primitives
    m.def("sphere",   &make_sphere,   nb::arg("radius"));
    m.def("box",      [](Vec3f e)     { return make_box(e); }, nb::arg("extent"));
    m.def("cylinder", &make_cylinder, nb::arg("radius"), nb::arg("height"));
    m.def("torus",    &make_torus,    nb::arg("R"), nb::arg("r"));
    m.def("cone",     &make_cone,     nb::arg("radius"), nb::arg("height"));
    m.def("capsule",  &make_capsule,  nb::arg("a"), nb::arg("b"), nb::arg("radius"));
    m.def("plane",    &make_plane);

    // Operators
    m.def("union",          &sdf_union,          nb::arg("a"), nb::arg("b"));
    m.def("intersect",      &sdf_intersect,      nb::arg("a"), nb::arg("b"));
    m.def("subtract",       &sdf_subtract,       nb::arg("a"), nb::arg("b"));
    m.def("smooth_union",   &sdf_smooth_union,   nb::arg("a"), nb::arg("b"), nb::arg("k"));

    // Transforms
    m.def("translate", &sdf_translate, nb::arg("body"), nb::arg("offset"));
    m.def("rotate",    &sdf_rotate,    nb::arg("body"), nb::arg("q"));
    m.def("scale",     &sdf_scale,     nb::arg("body"), nb::arg("s"));
    m.def("twist",     &sdf_twist,     nb::arg("body"), nb::arg("k"));

    // Batched evaluation
    nb::class_<EvalResult>(m, "EvalResult")
        .def_ro("values",    &EvalResult::values)
        .def_ro("gradients", &EvalResult::gradients);

    // Accept either a list of Vec3f or a numpy (N,3) float32 array.
    m.def("evaluate", [](const SDFBody& body, nb::ndarray<float, nb::ndim<2>> points) {
        if (points.shape(1) != 3) {
            throw std::invalid_argument("points must have shape (N, 3)");
        }
        const auto n = points.shape(0);
        std::vector<Vec3f> pts(n);
        for (std::size_t i = 0; i < n; ++i) {
            pts[i] = Vec3f{points(i, 0), points(i, 1), points(i, 2)};
        }
        EvalResult out;
        CpuEvalBackend backend;
        backend.evaluate(body, std::span<const Vec3f>(pts.data(), n), out);
        return out;
    }, nb::arg("body"), nb::arg("points").noconvert());

    // EvalBackend (for documentation / introspection)
    nb::class_<CpuEvalBackend>(m, "CpuEvalBackend")
        .def(nb::init<unsigned>(), nb::arg("num_threads") = 0)
        .def("evaluate",
             [](CpuEvalBackend& b, const SDFBody& body,
                nb::ndarray<float, nb::ndim<2>> points) {
            if (points.shape(1) != 3)
                throw std::invalid_argument("points must have shape (N, 3)");
            const auto n = points.shape(0);
            std::vector<Vec3f> pts(n);
            for (std::size_t i = 0; i < n; ++i) {
                pts[i] = Vec3f{points(i, 0), points(i, 1), points(i, 2)};
            }
            EvalResult out;
            b.evaluate(body, std::span<const Vec3f>(pts.data(), n), out);
            return out;
        });
}

// bindings/python/src/math_bindings.cpp
#include <nanobind/nanobind.h>
#include <nanobind/stl/array.h>

#include "cadforge/math/vec.hpp"
#include "cadforge/math/mat.hpp"
#include "cadforge/math/quat.hpp"
#include "cadforge/math/bbox.hpp"
#include "cadforge/math/tolerance.hpp"

namespace nb = nanobind;
using namespace cadforge::math;

void bind_math(nb::module_& m) {
    // Vec3f — the workhorse for SDF evaluation
    nb::class_<Vec3f>(m, "Vec3f")
        .def(nb::init<>())
        .def(nb::init<float, float, float>(), nb::arg("x"), nb::arg("y"), nb::arg("z"))
        .def_rw("x", &Vec3f::x)
        .def_rw("y", &Vec3f::y)
        .def_rw("z", &Vec3f::z)
        .def("__getitem__", [](const Vec3f& v, std::size_t i) {
            if (i >= 3) throw std::out_of_range("Vec3f index");
            return v[i];
        })
        .def("__setitem__", [](Vec3f& v, std::size_t i, float x) {
            if (i >= 3) throw std::out_of_range("Vec3f index");
            v[i] = x;
        })
        .def("__add__", &Vec3f::operator+)
        .def("__sub__", &Vec3f::operator-)
        .def("__mul__", [](const Vec3f& v, float s) { return v * s; })
        .def("__rmul__", [](const Vec3f& v, float s) { return v * s; })
        .def("__neg__", [](const Vec3f& v) { return -v; })
        .def("length",      &Vec3f::length)
        .def("length_sq",   &Vec3f::length_sq)
        .def("normalized",  &Vec3f::normalized)
        .def("dot",          &Vec3f::dot)
        .def("cross",       &Vec3f::cross)
        .def("__repr__", [](const Vec3f& v) {
            return "<Vec3f " + std::to_string(v.x) + ", " +
                   std::to_string(v.y) + ", " + std::to_string(v.z) + ">";
        });

    m.def("dot",     [](const Vec3f& a, const Vec3f& b) { return a.dot(b); });
    m.def("cross",   [](const Vec3f& a, const Vec3f& b) { return a.cross(b); });
    m.def("length",  [](const Vec3f& v) { return v.length(); });
    m.def("normalize", [](const Vec3f& v) { return v.normalized(); });

    // Mat4f
    nb::class_<Mat4f>(m, "Mat4f")
        .def(nb::init<>())
        .def_static("identity",   &Mat4f::identity)
        .def_static("translation", &Mat4f::translation, nb::arg("t"))
        .def_static("scaling",    &Mat4f::scaling,      nb::arg("s"))
        .def_static("rotation_x", &Mat4f::rotation_x,    nb::arg("rad"))
        .def_static("rotation_y", &Mat4f::rotation_y,    nb::arg("rad"))
        .def_static("rotation_z", &Mat4f::rotation_z,    nb::arg("rad"))
        .def("transform_point", &Mat4f::transform_point)
        .def("transform_dir",   &Mat4f::transform_dir)
        .def("__mul__",         &Mat4f::operator*)
        .def("inverse",         &Mat4f::inverse)
        .def("inverse_orthonormal", &Mat4f::inverse_orthonormal);

    // Quatf
    nb::class_<Quatf>(m, "Quatf")
        .def(nb::init<>())
        .def_static("identity",     &Quatf::identity)
        .def_static("from_axis_angle", &Quatf::from_axis_angle,
                    nb::arg("axis"), nb::arg("rad"))
        .def("rotate",      &Quatf::rotate)
        .def("to_matrix",  &Quatf::to_matrix)
        .def("conjugate",  &Quatf::conjugate)
        .def("normalized", &Quatf::normalized)
        .def("__mul__",    &Quatf::operator*);

    // Bboxf
    nb::class_<Bboxf>(m, "Bboxf")
        .def(nb::init<>())
        .def_rw("min", &Bboxf::min)
        .def_rw("max", &Bboxf::max)
        .def("empty",  &Bboxf::empty)
        .def("center", &Bboxf::center)
        .def("extent", &Bboxf::extent)
        .def("radius", &Bboxf::radius)
        .def("contains", &Bboxf::contains)
        .def("distance_to", &Bboxf::distance_to);

    // Tolerance
    nb::class_<Tolerance>(m, "Tolerance")
        .def(nb::init<>())
        .def(nb::init<ToleranceKind, double>())
        .def_static("linear",   &Tolerance::linear,   nb::arg("mm") = 1e-6)
        .def_static("angular",   &Tolerance::angular, nb::arg("rad") = 1e-7)
        .def_static("relative", &Tolerance::relative, nb::arg("r")  = 1e-9)
        .def("describe", &Tolerance::describe);

    nb::enum_<ToleranceKind>(m, "ToleranceKind")
        .value("Linear",   ToleranceKind::Linear)
        .value("Angular",  ToleranceKind::Angular)
        .value("Relative", ToleranceKind::Relative);
}

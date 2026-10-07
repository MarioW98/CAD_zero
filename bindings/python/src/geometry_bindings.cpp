// bindings/python/src/geometry_bindings.cpp
//
// NOTE: We deliberately do NOT expose BRepBody or HybridBody in Phase A —
// B-Rep lands in Phase D, hybrid bodies land in Phase E.
// Shape is bound as a "place-holder" wrapper that lets Python create
// SDF-only Shapes and read back their representation.
//
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/unique_ptr.h>

#include "cadforge/geometry/shape.hpp"
#include "cadforge/geometry/shape_bodies.hpp"
#include "cadforge/geometry/transform.hpp"
#include "cadforge/sdf/field.hpp"

namespace nb = nanobind;
using namespace cadforge::geometry;
using namespace cadforge::sdf;

void bind_geometry(nb::module_& m) {
    // FeatureId / SubshapeRef / Generation / ShapeId
    nb::class_<FeatureId>(m, "FeatureId")
        .def(nb::init<>())
        .def(nb::init<std::uint64_t>(), nb::arg("value"))
        .def_ro("value", &FeatureId::value)
        .def("__repr__", [](const FeatureId& f) {
            return "<FeatureId " + std::to_string(f.value) + ">";
        });

    nb::class_<SubshapeRef>(m, "SubshapeRef")
        .def(nb::init<>());

    nb::class_<Generation>(m, "Generation")
        .def(nb::init<>())
        .def_ro("value", &Generation::value);

    nb::class_<ShapeId>(m, "ShapeId")
        .def(nb::init<>())
        .def_ro("feature",    &ShapeId::feature)
        .def_ro("subshape",   &ShapeId::subshape)
        .def_ro("generation", &ShapeId::generation)
        .def("to_string",    &ShapeId::to_string)
        .def("__repr__",     [](const ShapeId& s) { return "<ShapeId " + s.to_string() + ">"; });

    // DatumCS, Transform
    nb::class_<DatumCS>(m, "DatumCS")
        .def(nb::init<>())
        .def_rw("origin",  &DatumCS::origin)
        .def_rw("x_axis",  &DatumCS::x_axis)
        .def_rw("y_axis",  &DatumCS::y_axis)
        .def_rw("z_axis",  &DatumCS::z_axis);

    nb::class_<Transform>(m, "Transform")
        .def(nb::init<>())
        .def_rw("translation", &Transform::translation)
        .def_rw("rotation",    &Transform::rotation)
        .def_rw("scale",       &Transform::scale)
        .def_static("identity", &Transform::identity)
        .def("inverse",        &Transform::inverse)
        .def("to_matrix",      &Transform::to_matrix);

    // Representation enum
    nb::enum_<Representation>(m, "Representation")
        .value("SDF",    Representation::SDF)
        .value("BRep",   Representation::BRep)
        .value("Hybrid", Representation::Hybrid);

    nb::enum_<Units>(m, "Units")
        .value("Millimeter", Units::Millimeter)
        .value("Centimeter", Units::Centimeter)
        .value("Meter",     Units::Meter)
        .value("Inch",      Units::Inch)
        .value("Foot",      Units::Foot);

    // Metadata
    nb::class_<Metadata>(m, "Metadata")
        .def(nb::init<>())
        .def_rw("name",     &Metadata::name)
        .def_rw("color",    &Metadata::color)
        .def_rw("layer",    &Metadata::layer)
        .def_rw("material", &Metadata::material);

    // Shape
    nb::class_<Shape>(m, "Shape")
        .def(nb::init<>())
        .def("__init__", [](Shape* s, SDFBody& body) {
            new (s) Shape(BodyVariant(std::make_unique<SDFBody>(std::move(body))));
        }, nb::arg("body"))
        .def("id",            [](const Shape& sh) { return sh.id(); }, nb::rv_policy::copy)
        .def("set_id",        &Shape::set_id)
        .def("transform",     [](const Shape& sh) { return sh.transform(); }, nb::rv_policy::copy)
        .def("set_transform", &Shape::set_transform)
        .def("datum",         [](const Shape& sh) { return sh.datum(); }, nb::rv_policy::copy)
        .def("set_datum",     &Shape::set_datum)
        .def("units",         [](const Shape& sh) { return sh.units(); })
        .def("set_units",     &Shape::set_units)
        .def("representation", [](const Shape& sh) { return sh.representation(); })
        .def("describe_metadata",
             [](const Shape& sh) { return sh.metadata().name; });
}

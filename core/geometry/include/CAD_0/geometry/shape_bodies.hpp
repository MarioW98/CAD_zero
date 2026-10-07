// core/geometry/include/CAD_0/geometry/shape_bodies.hpp
//
// Forward declarations of body types + HybridBody definition.
//
// The actual implementations of SDFBody and BRepBody live in core/sdf
// and core/brep respectively. This file provides the *concrete type
// list* for std::variant<SDFBody, BRepBody, HybridBody> so that
// Shape can be a value type.
//
#pragma once

#include "CAD_0/math/tolerance.hpp"
#include "CAD_0/math/bbox.hpp"

#include <memory>
#include <variant>

namespace CAD_0::sdf       { class SDFBody; }  // core/sdf/field.hpp
// Phase A: include the brep/topology.hpp stub so that BRepBody is a
// complete type. This is required because std::variant<unique_ptr<T>>
// needs T to be complete at the point where the variant is instantiated
// (e.g. in Shape's move constructor).
//
// Phase D will replace this stub with the real radial-edge topology
// (see ADR-0012) — the include directive itself will not change.
#include "CAD_0/brep/topology.hpp"

namespace CAD_0::geometry {

// HybridBody holds *both* representations simultaneously, plus a bridge
// tolerance that documents the conversion error between them.
//
// Roles:
//   BRepWithSDFShell  — B-Rep solid with an SDF envelope (e.g. for lattice fill)
//   SDFWithBRepFeats  — SDF field with B-Rep reference features (e.g. bolt holes)
//   BridgedUnion      — Result of a hybrid boolean operation; the union of
//                        an SDF region and a B-Rep region glued together
enum class HybridRole : std::uint8_t {
    BRepWithSDFShell,
    SDFWithBRepFeats,
    BridgedUnion,
};

struct HybridBody {
    std::unique_ptr<sdf::SDFBody>   sdf;
    std::unique_ptr<brep::BRepBody> brep;
    HybridRole                       role{HybridRole::BRepWithSDFShell};
    math::ToleranceAccumulator      bridge_tolerance;

    HybridBody() = default;
    HybridBody(std::unique_ptr<sdf::SDFBody>,
               std::unique_ptr<brep::BRepBody>,
               HybridRole,
               math::ToleranceAccumulator);

    // Destructor declared here, defined in core/geometry/src/shape.cpp.
    // The definition is a no-op in Phase A (sdf::SDFBody and brep::BRepBody
    // are forward-declared). The real destructor will be provided in Phase D
    // once B-Rep lands — see shape.cpp for details.
    ~HybridBody();

    HybridBody(const HybridBody&) = delete;
    HybridBody& operator=(const HybridBody&) = delete;
    HybridBody(HybridBody&&) noexcept = default;
    HybridBody& operator=(HybridBody&&) noexcept = default;
};

// Variant of body payloads. SDFBody and BRepBody are forward-declared;
// HybridBody is fully defined here. We use std::variant for type-safe
// dispatch via std::visit (see Shape::accept() in shape.hpp).
using BodyVariant = std::variant<
    std::unique_ptr<sdf::SDFBody>,
    std::unique_ptr<brep::BRepBody>,
    HybridBody
>;

} // namespace CAD_0::geometry

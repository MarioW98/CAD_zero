// core/geometry/src/shape.cpp
//
// NOTE: We include the full sdf/field.hpp here (not just the forward
// declaration in shape_bodies.hpp) so that the HybridBody destructor
// can call `delete` on the unique_ptr<sdf::SDFBody> member.
//
// This is the standard C++ idiom for breaking cycles with PIMPL-style
// forward-declared members: declare the destructor in the header, define
// it in a TU that sees the full type definition.

#include "cadforge/geometry/shape.hpp"
#include "cadforge/geometry/shape_bodies.hpp"
#include "cadforge/sdf/field.hpp"        // IWYU pragma: keep — needed for ~HybridBody
#include "cadforge/brep/topology.hpp"    // IWYU pragma: keep — Phase D stub; complete type for ~HybridBody

#include <sstream>

namespace cadforge::geometry {

const sdf::SDFBody* Shape::as_sdf() const noexcept {
    if (auto* p = std::get_if<std::unique_ptr<sdf::SDFBody>>(&payload_)) {
        return p->get();
    }
    if (auto* h = std::get_if<HybridBody>(&payload_)) {
        return h->sdf.get();
    }
    return nullptr;
}

const brep::BRepBody* Shape::as_brep() const noexcept {
    if (auto* p = std::get_if<std::unique_ptr<brep::BRepBody>>(&payload_)) {
        return p->get();
    }
    if (auto* h = std::get_if<HybridBody>(&payload_)) {
        return h->brep.get();
    }
    return nullptr;
}

const HybridBody* Shape::as_hybrid() const noexcept {
    return std::get_if<HybridBody>(&payload_);
}

// HybridBody constructor — defined here so that the unique_ptr to
// sdf::SDFBody (now fully defined thanks to the include above) can
// be initialized properly.
HybridBody::HybridBody(std::unique_ptr<sdf::SDFBody> s,
                       std::unique_ptr<brep::BRepBody> b,
                       HybridRole r,
                       math::ToleranceAccumulator t)
    : sdf(std::move(s)), brep(std::move(b)), role(r), bridge_tolerance(std::move(t)) {}

// HybridBody destructor — defined here so unique_ptr<sdf::SDFBody>::~unique_ptr
// can call delete on a fully-defined type. The brep::BRepBody member is still
// forward-declared; in Phase A it is never non-null, so this is safe.
// Phase D will replace this file's include with both sdf/field.hpp and
// brep/topology.hpp.
HybridBody::~HybridBody() = default;

std::string ShapeId::to_string() const {
    std::ostringstream ss;
    ss << "F" << feature.value << "/S";
    for (std::uint8_t i = 0; i < subshape.depth; ++i) {
        ss << (i ? "." : "") << subshape.path[i];
    }
    ss << "@G" << generation.value;
    return ss.str();
}

} // namespace cadforge::geometry

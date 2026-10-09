// core/geometry/include/CAD_0/geometry/visitor.hpp
//
// Visitor pattern for Shape bodies. Dispatches on the std::variant
// payload of Shape — see ADR-0002.
//
#pragma once

#include "CAD_0/geometry/shape.hpp"
#include "CAD_0/geometry/shape_bodies.hpp"

namespace CAD_0::sdf  { class SDFBody; }
namespace CAD_0::brep { class BRepBody; }

namespace CAD_0::geometry {

// Interface-based visitor for code that prefers virtual dispatch over
// std::visit (e.g. for plugins). Std::visit is preferred inside the
// core library because it lets the compiler optimize the dispatch.
class ShapeVisitor {
public:
    virtual ~ShapeVisitor() = default;
    virtual void visit_sdf   (const sdf::SDFBody&)    {}
    virtual void visit_brep  (const brep::BRepBody&)   {}
    virtual void visit_hybrid(const HybridBody&)      {}
};

} // namespace CAD_0::geometry

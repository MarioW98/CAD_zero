// core/geometry/include/cadforge/geometry/visitor.hpp
//
// Visitor pattern for Shape bodies. Dispatches on the std::variant
// payload of Shape — see ADR-0002.
//
#pragma once

#include "cadforge/geometry/shape.hpp"
#include "cadforge/geometry/shape_bodies.hpp"

namespace cadforge::sdf  { class SDFBody; }
namespace cadforge::brep { class BRepBody; }

namespace cadforge::geometry {

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

} // namespace cadforge::geometry

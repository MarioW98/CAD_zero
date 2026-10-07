// core/brep/include/CAD_0/brep/topology.hpp
//
// B-Rep kernel — Phase D module (stub for Phase A).
//
// In Phase A this file exists only so that the HybridBody destructor
// can call delete on `std::unique_ptr<brep::BRepBody>` without a
// complete type. The class is fully defined as an empty struct;
// Phase D will replace this with the real radial-edge topology
// (see ADR-0012).
//
#pragma once

namespace CAD_0::brep {

// STUB — Phase D will replace this with the real BRepBody class
// (vertex, edge, coedge, loop, face, shell, solid).
class BRepBody {
public:
    BRepBody() = default;
    ~BRepBody() = default;
};

} // namespace CAD_0::brep

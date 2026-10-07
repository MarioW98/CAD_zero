// core/geometry/include/cadforge/geometry/representation.hpp
//
// Enumeration of first-class shape representations in cadforge.
// See ADR-0002 — "Dual-representation kernel with native SDF".
//
#pragma once

#include <cstdint>
#include <string_view>

namespace cadforge::geometry {

enum class Representation : std::uint8_t {
    SDF,        // Implicit field — equation-driven, volumetric
    BRep,       // Boundary representation — exact topology
    Hybrid,     // Both representations live together, coordinated
};

constexpr std::string_view to_string(Representation r) noexcept {
    switch (r) {
        case Representation::SDF:    return "SDF";
        case Representation::BRep:   return "BRep";
        case Representation::Hybrid: return "Hybrid";
    }
    return "Unknown";
}

} // namespace cadforge::geometry

// core/geometry/include/CAD_0/geometry/shape.hpp
//
// Shape — the unified CAD entity.
//
// CRITICAL CORRECTION vs. original proposal:
//   Original  : payload { sdf: optional<SDF>, brep: optional<BRep> }
//   Problem   : ambiguous when both are non-null; no clear hybrid semantics;
//               visitor pattern becomes a 4-way switch instead of 3-way.
//   Corrected: payload is std::variant<SDFBody, BRepBody, HybridBody>.
//               Hybrid is a first-class type, not an emergent property
//               of two optionals. std::visit() is the dispatch primitive.
//
// A Shape is:
//   * identified  — ShapeId is genealogy-based, stable across rebuilds
//   * placed      — Transform relative to DatumCS
//   * unit-aware  — declared in mm, m, inch, ...
//   * visible     — metadata: name, color, layer, material
//
#pragma once

#include "CAD_0/geometry/representation.hpp"
#include "CAD_0/geometry/shape_id.hpp"
#include "CAD_0/geometry/transform.hpp"
#include "CAD_0/geometry/shape_bodies.hpp"

#include <CAD_0/config.h>

#include <cstdint>
#include <optional>
#include <string>
#include <variant>

namespace CAD_0::geometry {

enum class Units : std::uint8_t {
    Millimeter,
    Centimeter,
    Meter,
    Inch,
    Foot,
};

constexpr std::string_view to_string(Units u) noexcept {
    switch (u) {
        case Units::Millimeter: return "mm";
        case Units::Centimeter: return "cm";
        case Units::Meter:     return "m";
        case Units::Inch:      return "in";
        case Units::Foot:      return "ft";
    }
    return "?";
}

// Per-shape metadata.
struct Metadata {
    std::string name;
    std::uint32_t color{0xFFFFFFFF};   // RGBA, white opaque default
    std::string layer;
    std::string material;
};

class Shape {
public:
    Shape() = default;

    // Construct from an existing body (move-only — bodies own substantial state).
    explicit Shape(BodyVariant body, ShapeId id = {}) noexcept
        : id_(id), payload_(std::move(body)) {}

    // Movable but not copyable (BRepBody can own many megabytes of topology).
    Shape(Shape&&) noexcept = default;
    Shape& operator=(Shape&&) noexcept = default;
    Shape(const Shape&) = delete;
    Shape& operator=(const Shape&) = delete;

    // ----- Identity --------------------------------------------------------
    const ShapeId& id() const noexcept { return id_; }
    void set_id(ShapeId id) noexcept { id_ = id; }

    // ----- Placement -------------------------------------------------------
    const Transform&  transform()  const noexcept { return transform_; }
    const DatumCS&    datum()      const noexcept { return datum_; }
    Units             units()      const noexcept { return units_; }
    void set_transform(Transform t) noexcept { transform_ = t; }
    void set_datum(DatumCS d)       noexcept { datum_ = d; }
    void set_units(Units u)         noexcept { units_ = u; }

    // ----- Metadata --------------------------------------------------------
    const Metadata& metadata()             const noexcept { return metadata_; }
    void set_metadata(Metadata m)                 noexcept { metadata_ = std::move(m); }

    // ----- Payload access --------------------------------------------------
    Representation representation() const noexcept {
        // Index 0 = SDF, 1 = BRep, 2 = Hybrid
        switch (payload_.index()) {
            case 0: return Representation::SDF;
            case 1: return Representation::BRep;
            case 2: return Representation::Hybrid;
        }
        return Representation::SDF; // unreachable
    }

    const BodyVariant& body() const noexcept { return payload_; }
    BodyVariant&       body()       noexcept { return payload_; }

    template<typename Visitor>
    decltype(auto) accept(Visitor&& v) const {
        return std::visit(std::forward<Visitor>(v), payload_);
    }

    template<typename Visitor>
    decltype(auto) accept(Visitor&& v) {
        return std::visit(std::forward<Visitor>(v), payload_);
    }

    // Convenience type-safe accessors (return nullptr if wrong type).
    const sdf::SDFBody*    as_sdf()    const noexcept;
    const brep::BRepBody*   as_brep()   const noexcept;
    const HybridBody*      as_hybrid() const noexcept;

private:
    ShapeId      id_{};
    Transform    transform_{};
    DatumCS      datum_{};
    Units        units_{Units::Millimeter};
    Metadata     metadata_{};
    BodyVariant  payload_;
};

} // namespace CAD_0::geometry

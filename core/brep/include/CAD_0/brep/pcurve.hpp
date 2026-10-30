// core/brep/include/CAD_0/brep/pcurve.hpp
//
// The p-curve vocabulary — ADR-0032 D2.1, as amended by ADR-0035
// D4/D6 (H2 unit 3): ONE OPINION PER WORLD.
//
// The BREP parametric domain's OWN typed 2D entity set, in DOUBLE —
// the exact path's arithmetic world, carried to the payload ADR-0034
// D1 exempted (the demotions at the authoring sites are REMOVED,
// not converted; the double→float direction is gone from the exact
// path entirely):
//
//     Line2   — the segment  (a → b, region side interior_left)
//     Circle2 — the circle   (center, radius, disc | complement)
//     Arc2    — the arc      (center, radius, CCW span, region side)
//     PCurve  — the typed chain entity itself (the Kind variant)
//
// THE MIRROR, DECLARED (D4 "one opinion PER WORLD"): the semantics
// are the sketch kernel's — the same construction-time validation
// rules (finite endpoints, p0 ≠ p1 for the segment; finite center,
// r > 0 for the circle and the arc; the CCW sweep in (0, 2π) for
// the arc), the same region-side convention (interior_left / hole:
// the entity's REGION is where the FACE'S MATERIAL lies in the
// domain), the same CCW-store discipline (arcs sweep CCW only; the
// full turn is the Circle2, never a 2π Arc2). This is NOT a second
// implementation of the sketch kernel's validation in the SAME
// world — it is the BREP domain's one vocabulary, in its own
// arithmetic. The sketch kernel's float set stays the sketch
// world's one vocabulary (the canvas, the UI, the snapping, the
// recipe serialization — ADR-0002's domain split, untouched).
// The anti-fork rule survives in its essence: per world, one
// opinion; the per-world static_asserts below (and the test-side
// pair in test_g1_data_model.cpp) stop a silent reunification or
// a silent re-fork at compile time.
//
// THE SKETCH FRONTIER (D4): where loops are born from sketch
// entities — the extrude's caps, whose outer loop IS the chain
// itself, entity for entity — the conversion is float→double:
// EXACT (every float32 is exactly a double — no rounding, no band,
// no quantizer; ADR-0014 needs no amendment), ONE direction
// (pcurve_from_sketch below is the only bridge), DECLARED at the
// site. The reverse bridge does not exist: the lossy direction
// (double→float) died with the demotions, and no code path may
// reintroduce it silently — a payload that must shrink back to the
// sketch world is the sketcher's own authoring, never the exact
// path's answer.
//
// Semantics pinned for the writers (G1 units 2–4; ADR-0032 D2.2):
//
//   * DOMAIN — the curve lives in the parameter domain (u, v) of the
//     face whose loop owns the co-edge that carries it. The same 3D
//     edge carries DIFFERENT p-curves in different faces' domains:
//     that pair (3D space reference, 2D p-curve) is the defining
//     structure of the exact B-Rep (D2.2).
//   * DIRECTION — the p-curve is stored in the CO-EDGE'S TRAVERSAL
//     direction: its start is the image of the co-edge's start
//     vertex, its end the image of the end vertex. A loop's p-curves
//     therefore chain endpoint-to-endpoint in the domain, the same
//     connectedness discipline the sketch chain already enforces
//     (ADR-0030 D2).
//   * REGION SIDE — the entity's orientation bit (interior_left /
//     hole) keeps its sketch meaning: the entity's REGION is where
//     the FACE'S MATERIAL lies in the domain. Loops authored per the
//     sketch discipline (material on the left of traversal) read CCW
//     as outer loops and CW as inner loops — one convention from the
//     sketcher to the trimming loop, with no translation step to get
//     wrong.
//   * FACETED WORLD — bridge-extracted faces (plane-per-triangle,
//     ADR-0026) carry NO p-curves: the field stays empty and the
//     Layer-2 provenance field (topology.hpp, D2.4) says
//     approximated(tol). The invariant is per-world, never fudged.
//
// THE COHERENCE CONTRACT (morso 3's pin, tested in
// test_h2_u3_pcurve_double.cpp): a p-curve is not free-floating
// domain data — its image under the owning surface IS the 3D
// trimming curve. Sampled, evaluated, and pulled back onto the
// domain, it must return within the ADR-declared parameter
// tolerance of the family (D2.2: ε_P derived per family through the
// family's Jacobian — the plane's is 1, the cylinder's u is r, the
// sphere's are R·sin v and R); at the endpoints its image must
// coincide with the co-edge's vertex records within the linear
// tolerance (ε_L, D2.1). The goldens pin both directions, and the
// discriminating case — an extended domain where the float payload
// breaks the endpoint coherence at the declared ε_L and the double
// payload holds — proves the compromise ADR-0035 D4 closed was
// real.
#pragma once

#include "CAD_0/math/vec.hpp"
#include "CAD_0/sdf/sketch_entities.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace CAD_0::brep {

// --- the payloads (the BREP parametric domain's double world) -------------
//
// Field-for-field the sketch entities' mirror (the types' contract,
// above): every coordinate double, every angle double — the domain
// joins the exact path's arithmetic (ADR-0034 D1's principle, no
// exemption left).

struct Line2 {
    math::Vec2d a{}, b{};
    bool interior_left{};  // the material lies on the LEFT of a->b
};

struct Circle2 {
    math::Vec2d center{};
    double radius{};
    bool hole{};           // false: region = disc; true: region = complement
};

struct Arc2 {
    math::Vec2d center{};
    double radius{};
    bool hole{};           // false: convex (disc side); true: concave notch
    double start_angle{};  // radians, CCW start of the active span
    double sweep{};        // radians, CCW, in (0, 2π)
};

struct PCurve {
    enum class Kind : std::uint8_t { Segment, Circle, Arc };
    Kind kind{Kind::Segment};
    Line2 segment{};
    Circle2 circle{};
    Arc2 arc{};
};

// --- the factories (the mirrored construction-time rules, loud) -----------

// The periodic domain's constants, DOUBLE — they feed the p-curve
// payloads, the BREP parametric domain's own world since morso 3
// (the sketch kernel keeps its float constants for its own world;
// mathematical constants, not mirrored layouts).
inline constexpr double kPi = 3.14159265358979;
inline constexpr double kTwoPi = 6.28318530717958647692;

inline PCurve pcurve_segment(math::Vec2d a, math::Vec2d b,
                             bool interior_left) {
    if (!std::isfinite(a.x) || !std::isfinite(a.y) ||
        !std::isfinite(b.x) || !std::isfinite(b.y)) {
        throw std::invalid_argument(
            "pcurve_segment: endpoints a and b must be finite, got a=(" +
            std::to_string(a.x) + ", " + std::to_string(a.y) + ") b=(" +
            std::to_string(b.x) + ", " + std::to_string(b.y) +
            ") — a non-finite endpoint puts the boundary line off the "
            "number line and the region side with it. Remedy: author "
            "the segment with finite endpoints.");
    }
    if (!(a != b)) {
        throw std::invalid_argument(
            "pcurve_segment: endpoints must satisfy a != b (the segment "
            "must have non-zero length), got a == b == (" +
            std::to_string(a.x) + ", " + std::to_string(a.y) +
            ") — a degenerate segment has no side to orient (the "
            "mirrored Halfplane2D rule, ADR-0030 D3). Remedy: give the "
            "segment a non-zero extent.");
    }
    PCurve e;
    e.kind = PCurve::Kind::Segment;
    e.segment = Line2{a, b, interior_left};
    return e;
}

inline PCurve pcurve_circle(math::Vec2d center, double radius, bool hole) {
    if (!std::isfinite(center.x) || !std::isfinite(center.y) ||
        !std::isfinite(radius) || radius <= 0.0) {
        throw std::invalid_argument(
            "pcurve_circle: center must be finite and radius must be "
            "finite and > 0, got center=(" + std::to_string(center.x) +
            ", " + std::to_string(center.y) + ") radius=" +
            std::to_string(radius) +
            " — a non-positive or non-finite radius inverts the "
            "region's sign class (the mirrored Disc2D rule, ADR-0030 "
            "D3). Remedy: author the circle with radius > 0.");
    }
    PCurve e;
    e.kind = PCurve::Kind::Circle;
    e.circle = Circle2{center, radius, hole};
    return e;
}

inline PCurve pcurve_arc(math::Vec2d center, double radius, bool hole,
                         double start_angle, double sweep) {
    if (!std::isfinite(center.x) || !std::isfinite(center.y) ||
        !std::isfinite(radius) || radius <= 0.0) {
        throw std::invalid_argument(
            "pcurve_arc: center must be finite and radius must be "
            "finite and > 0, got center=(" + std::to_string(center.x) +
            ", " + std::to_string(center.y) + ") radius=" +
            std::to_string(radius) +
            " — the mirrored Disc2D rule (ADR-0030 D3). Remedy: author "
            "the arc with radius > 0.");
    }
    if (!std::isfinite(start_angle) || !std::isfinite(sweep) ||
        sweep <= 0.0 || sweep >= kTwoPi) {
        throw std::invalid_argument(
            "pcurve_arc: arc sweep must be finite and in (0, 2π), got "
            "sweep=" + std::to_string(sweep) +
            " — the vocabulary stores CCW sweeps only: a full turn is "
            "the Circle2 (direction-free), and a reversed walk is the "
            "chain's order, never a negative span (the mirrored "
            "validate_arc_sweep rule, sketch_profile.hpp). Remedy: "
            "author the span as a CCW sweep in (0, 2π), or a Circle2 "
            "for the full turn.");
    }
    PCurve e;
    e.kind = PCurve::Kind::Arc;
    e.arc = Arc2{center, radius, hole, start_angle, sweep};
    return e;
}

// --- the derived endpoints (the mirror of the sketch kernel's own) --------
//
// Computed, not stored — the same discipline as sdf::arc_start_point
// (the recipe owns center/radius/start/sweep, the joints are
// derived), here in the double world's arithmetic. One opinion per
// world: the BREP's readers call THESE for BREP payloads, the
// sketcher's call the sketch kernel's for sketch entities.

inline math::Vec2d pcurve_arc_start_point(const Arc2& a) noexcept {
    return a.center + math::Vec2d{a.radius * std::cos(a.start_angle),
                                  a.radius * std::sin(a.start_angle)};
}

inline math::Vec2d pcurve_arc_end_point(const Arc2& a) noexcept {
    const double end_angle = a.start_angle + a.sweep;
    return a.center + math::Vec2d{a.radius * std::cos(end_angle),
                                  a.radius * std::sin(end_angle)};
}

// --- THE SKETCH FRONTIER (D4): the one-direction bridge -------------------
//
// The extrude caps' authoring site: the sketch chain's entities
// enter the BREP parametric domain here, and ONLY here. Every field
// crosses as an EXPLICIT promotion — float32 → double is exact by
// construction (every float IS a double), so the payload carries
// the sketch's own numbers bit-for-bit: no rounding, no band, no
// quantizer. The declared direction is one-way; the reverse bridge
// does not exist (see the header contract).

inline PCurve pcurve_from_sketch(const sdf::SketchEntity& e) {
    switch (e.kind) {
    case sdf::SketchEntity::Kind::Segment:
        return pcurve_segment(
            math::Vec2d{static_cast<double>(e.segment.a.x),
                        static_cast<double>(e.segment.a.y)},
            math::Vec2d{static_cast<double>(e.segment.b.x),
                        static_cast<double>(e.segment.b.y)},
            e.segment.interior_left);
    case sdf::SketchEntity::Kind::Circle:
        return pcurve_circle(
            math::Vec2d{static_cast<double>(e.circle.center.x),
                        static_cast<double>(e.circle.center.y)},
            static_cast<double>(e.circle.radius), e.circle.hole);
    case sdf::SketchEntity::Kind::Arc:
        return pcurve_arc(
            math::Vec2d{static_cast<double>(e.arc.center.x),
                        static_cast<double>(e.arc.center.y)},
            static_cast<double>(e.arc.radius), e.arc.hole,
            static_cast<double>(e.arc.start_angle),
            static_cast<double>(e.arc.sweep));
    }
    throw std::invalid_argument(
        "pcurve_from_sketch: unknown sketch entity kind — the frontier "
        "converts Segment/Circle/Arc only (the mirrored set; a new "
        "kind is a kernel extension, not a silent skip). Remedy: "
        "report the caller.");
}

// --- the per-world compile-time pins --------------------------------------
//
// The BREP parametric domain's vocabulary is DOUBLE, all of it
// (D4): coordinates, radii, angles. A field that quietly goes back
// to float stops the build here; the test-side pair (these types
// are NOT the sketch's, and the sketch's stay float) lives in
// test_g1_data_model.cpp.
static_assert(std::is_same_v<decltype(Line2::a), math::Vec2d> &&
                  std::is_same_v<decltype(Line2::b), math::Vec2d>,
              "D4: Line2's endpoints are the double world's Vec2d");
static_assert(std::is_same_v<decltype(Circle2::center), math::Vec2d> &&
                  std::is_same_v<decltype(Circle2::radius), double>,
              "D4: Circle2's payload is the double world's");
static_assert(std::is_same_v<decltype(Arc2::start_angle), double> &&
                  std::is_same_v<decltype(Arc2::sweep), double> &&
                  std::is_same_v<decltype(Arc2::radius), double>,
              "D4: Arc2's angles and radius are double");
static_assert(!std::is_same_v<Line2, sdf::SketchSegment> &&
                  !std::is_same_v<Circle2, sdf::SketchCircle> &&
                  !std::is_same_v<Arc2, sdf::SketchArc> &&
                  !std::is_same_v<PCurve, sdf::SketchEntity>,
              "D4: the BREP domain's vocabulary is its OWN double set — "
              "the float sketch entities and these types are two worlds "
              "bridged ONLY by pcurve_from_sketch");

} // namespace CAD_0::brep

// core/sdf/include/CAD_0/sdf/sketch_entities.hpp
//
// The 2D entity fields of the sketcher kernel — ADR-0030 D3, MVP unit 1.
//
// Each entity carries the EXACT signed field of its ORIENTED REGION —
// the 2D solid whose boundary the entity's contour side delimits:
//
//   * Segment -> half-plane    f(P) = s·((P − p0)·n̂)
//   * Circle  -> disc or complement, by orientation
//   * Arc     -> the SAME disc/complement field as the circle it
//                belongs to — the "curved half-plane" (a convex fillet
//                contributes the disc, a concave notch the complement).
//                NOT the angularly-clamped sdArc distance-to-the-curve:
//                that measures the CURVE, the profile needs the REGION
//                (lie #1 of ADR-0030 D3, closed in the ADR text; the
//                clamped formula remains the honest picking/snapping
//                measurer of D5, never the profile's field).
//
// The validation-table rows (ADR-0030 D3) live here, verbatim:
//
// | Node        | Field         | Exact regime | Understated | L | Gradient      | Bounds             | Construction checks       |
// |-------------|---------------|--------------|-------------|---|---------------|--------------------|---------------------------|
// | Halfplane2D | s·((P−p0)·n̂) | everywhere   | —           | 1 | s·n̂ constant | unbounded half-box | endpoints finite, p0≠p1  |
// | Disc2D      | ±(|P−C|−r)    | everywhere   | —           | 1 | ±(P−C)/|P−C| | [C±r]² / universe | C finite, r > 0          |
//
// (ProfileSDF / SketchEntitySet / SlabZ / ExtrudeSDF rows live in
// sketch_profile.hpp and sketch_extrude.hpp.)
//
// Honesty notes, stated once each at their site:
//   * Halfplane2D bounds: the half-box is exact only when the normal
//     is axis-aligned; a diagonal half-plane's region reaches every
//     axis extreme, so its box degenerates to the universe (an
//     over-estimate, contract-true). ProfileSDF closes the gap with
//     the contour hull — see sketch_profile.hpp.
//   * Disc2D bounds: the table's [C±r]² is the OUTER orientation's
//     box (region = disc). The HOLE orientation's region is the
//     unbounded complement, so its contract-true box is the universe —
//     the same unbounded class as the half-plane's half-box. Returning
//     [C±r]² for a hole would be UNSOUND (a ring profile's annulus
//     escapes the hole's box); the profile-level composition needs the
//     universe box to intersect correctly.
//
// Everything here is 2D on the world XY plane (ADR-0030 D2: world XY
// and only world XY for the MVP — the L = 1 declarations hold in the
// frame the profile is authored in, with no transform in the composite;
// arbitrary work planes are deferred with their entry criterion).
#pragma once

#include "CAD_0/math/vec.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace CAD_0::sdf {

// --- 2D value companions of the 3D field.hpp vocabulary ---------------

struct SDFSample2D {
    float value{};          // signed field (negative inside the region)
    math::Vec2f gradient{}; // ∇f, magnitude ≤ 1 (unit Lipschitz fields)
};

// Axis-aligned 2D box. Same conventions as math::Bboxf: an empty box by
// default (min = +inf, max = -inf), expand()/intersect() compose, and
// infinite components express unbounded sides (the half-plane's
// half-box, the hole disc's universe). Local to the sketch kernel; if a
// second consumer appears it can graduate to core/math.
struct Bbox2f {
    math::Vec2f min{ std::numeric_limits<float>::infinity(),
                     std::numeric_limits<float>::infinity()};
    math::Vec2f max{-std::numeric_limits<float>::infinity(),
                    -std::numeric_limits<float>::infinity()};

    bool empty() const noexcept { return min.x > max.x || min.y > max.y; }

    // Finite on both axes (and non-empty): usable as extraction bounds.
    bool bounded() const noexcept {
        return !empty() && std::isfinite(min.x) && std::isfinite(min.y) &&
               std::isfinite(max.x) && std::isfinite(max.y);
    }

    void expand(const math::Vec2f& p) noexcept {
        min.x = std::min(min.x, p.x); min.y = std::min(min.y, p.y);
        max.x = std::max(max.x, p.x); max.y = std::max(max.y, p.y);
    }
    void expand(const Bbox2f& o) noexcept {
        if (o.empty()) return;
        expand(o.min);
        expand(o.max);
    }

    Bbox2f intersect(const Bbox2f& o) const noexcept {
        Bbox2f r;
        if (empty() || o.empty()) return r;
        r.min = math::Vec2f{std::max(min.x, o.min.x), std::max(min.y, o.min.y)};
        r.max = math::Vec2f{std::min(max.x, o.max.x), std::min(max.y, o.max.y)};
        if (r.empty()) return Bbox2f{};
        return r;
    }

    static Bbox2f universe() noexcept {
        Bbox2f b;
        b.min = math::Vec2f{-std::numeric_limits<float>::infinity(),
                            -std::numeric_limits<float>::infinity()};
        b.max = math::Vec2f{ std::numeric_limits<float>::infinity(),
                             std::numeric_limits<float>::infinity()};
        return b;
    }
};

// --- Recipe-level entity payloads --------------------------------------
//
// These are the INPUT structures the caller (today: tests; tomorrow:
// the sketch node's recipe, ADR-0030 D2/D5) fills in. The orientation
// bit of each entity is EXPLICIT recipe data — the kernel derives the
// field from it and then certifies the composed result (the certificate
// catches a wrong orientation loudly; it never guesses one).

struct SketchSegment {
    math::Vec2f a{}, b{};
    bool interior_left{}; // the profile interior is on the LEFT of a->b
};

struct SketchCircle {
    math::Vec2f center{};
    float radius{};
    bool hole{};          // false: region = disc; true: region = complement
};

struct SketchArc {
    math::Vec2f center{};
    float radius{};
    bool hole{};          // false: convex fillet (disc); true: concave notch (complement)
    float start_angle{};  // radians, CCW start of the active span
    float sweep{};        // radians, CCW, in (0, 2π) — validated by the entity set
};

// The typed entity of a chain (ADR-0030 D2: an ordered chain of typed
// entities connected endpoint-to-endpoint).
struct SketchEntity {
    enum class Kind : std::uint8_t { Segment, Circle, Arc };
    Kind kind{Kind::Segment};
    SketchSegment segment{};
    SketchCircle circle{};
    SketchArc arc{};
};

inline SketchEntity sketch_segment(math::Vec2f a, math::Vec2f b, bool interior_left) {
    SketchEntity e;
    e.kind = SketchEntity::Kind::Segment;
    e.segment = SketchSegment{a, b, interior_left};
    return e;
}
inline SketchEntity sketch_circle(math::Vec2f center, float radius, bool hole) {
    SketchEntity e;
    e.kind = SketchEntity::Kind::Circle;
    e.circle = SketchCircle{center, radius, hole};
    return e;
}
inline SketchEntity sketch_arc(math::Vec2f center, float radius, bool hole,
                               float start_angle, float sweep) {
    SketchEntity e;
    e.kind = SketchEntity::Kind::Arc;
    e.arc = SketchArc{center, radius, hole, start_angle, sweep};
    return e;
}

// Arc endpoints (the chain's connectivity material). Computed, not
// stored: the recipe owns (center, radius, start_angle, sweep) and the
// joints are derived — a 1-ulp libm difference lands inside the joint
// coincidence tolerance, never on the other side of a check.
inline math::Vec2f arc_start_point(const SketchArc& a) noexcept {
    return a.center + math::Vec2f{a.radius * std::cos(a.start_angle),
                                   a.radius * std::sin(a.start_angle)};
}
inline math::Vec2f arc_end_point(const SketchArc& a) noexcept {
    const float end_angle = a.start_angle + a.sweep;
    return a.center + math::Vec2f{a.radius * std::cos(end_angle),
                                   a.radius * std::sin(end_angle)};
}

// --- Halfplane2D (segment) ---------------------------------------------
//
// Table row: f(P) = s·((P − p0)·n̂), exact everywhere, L = 1, ∇f = s·n̂
// constant, bounds = unbounded half-box, construction: endpoints
// finite, p0 ≠ p1.
//
// Stored form: p0 = the segment's start a, n = s·n̂ = the unit normal
// pointing OUT of the interior (f = (P − p0)·n < 0 inside, ∇f = n).
// The generating segment (a, b) is kept for describe() and for the
// entity set's certificate (the pairwise check works on CURVES).
class Halfplane2D {
public:
    Halfplane2D(math::Vec2f a, math::Vec2f b, bool interior_left)
        : a_(a), b_(b) {
        const math::Vec2f d = b - a;
        const float len_sq = d.length_sq();
        if (!std::isfinite(a.x) || !std::isfinite(a.y) ||
            !std::isfinite(b.x) || !std::isfinite(b.y))
            throw std::invalid_argument(
                "Halfplane2D: endpoints a and b must be finite, got a=(" +
                std::to_string(a.x) + ", " + std::to_string(a.y) + ") b=(" +
                std::to_string(b.x) + ", " + std::to_string(b.y) +
                ") — a non-finite endpoint puts the boundary line off the "
                "number line: the field and the normal are NaN for every "
                "query. Remedy: construct with finite endpoints.");
        if (!(len_sq > 0.0f))
            throw std::invalid_argument(
                "Halfplane2D: endpoints must satisfy p0 != p1 (the segment "
                "must have non-zero length), got a == b == (" +
                std::to_string(a.x) + ", " + std::to_string(a.y) +
                ") — a degenerate segment has no normal to orient: the "
                "half-plane it should delimit is undefined. Remedy: give "
                "the segment a non-zero extent.");
        const float inv_len = 1.0f / std::sqrt(len_sq);
        // Left normal of a->b: perp(d) = (-dy, dx). Interior on the left
        // => the outward normal is -perp(d); interior on the right => +perp(d).
        const math::Vec2f left_normal{-d.y * inv_len, d.x * inv_len};
        n_ = interior_left ? left_normal * -1.0f : left_normal;
    }

    SDFSample2D sample(const math::Vec2f& p) const noexcept {
        const math::Vec2f u = p - a_;
        return SDFSample2D{u.dot(n_), n_};
    }

    float lipschitz() const noexcept { return 1.0f; }

    // The box of the region {f < 0}. Exact (a true half-box) only when
    // the normal is axis-aligned: then the constrained axis is clipped
    // at the boundary and the free axis is infinite. A diagonal
    // half-plane's region reaches every extreme of both axes, so its
    // box degenerates to the universe — an over-estimate, contract-true
    // (the profile-level bounds close this gap with the contour hull;
    // see sketch_profile.hpp).
    Bbox2f bounds() const noexcept {
        Bbox2f b = Bbox2f::universe();
        if (n_.x == 0.0f && n_.y != 0.0f) {
            if (n_.y > 0.0f) b.max.y = a_.y; // region y < a.y
            else             b.min.y = a_.y; // region y > a.y
        } else if (n_.y == 0.0f && n_.x != 0.0f) {
            if (n_.x > 0.0f) b.max.x = a_.x; // region x < a.x
            else             b.min.x = a_.x; // region x > a.x
        }
        return b;
    }

    std::string describe() const {
        return "segment(a=(" + std::to_string(a_.x) + ", " + std::to_string(a_.y) +
               "), b=(" + std::to_string(b_.x) + ", " + std::to_string(b_.y) + "))";
    }

    const math::Vec2f& point() const noexcept { return a_; }        // p0
    const math::Vec2f& outward_normal() const noexcept { return n_; } // s·n̂
    const math::Vec2f& start() const noexcept { return a_; }
    const math::Vec2f& end() const noexcept { return b_; }

private:
    math::Vec2f a_, b_;
    math::Vec2f n_;
};

// --- Disc2D (circle / arc) ----------------------------------------------
//
// Table row: f(P) = ±(|P − C| − r), exact everywhere, L = 1,
// ∇f = ±(P−C)/|P−C|, bounds = [C±r]² (outer) / universe (hole),
// construction: C finite, r > 0.
//
// The angular span of an ARC is recipe data (SketchArc): it takes part
// in the entity set's pairwise certificate (arcs participate as their
// circle's angular span) but NOT in the field — the region is bounded
// by the whole circle, cut to the span by the neighbouring entities'
// regions (ADR-0030 D3, the curved half-plane).
class Disc2D {
public:
    Disc2D(math::Vec2f center, float radius, bool hole)
        : c_(center), r_(radius), hole_(hole) {
        if (!std::isfinite(center.x) || !std::isfinite(center.y) ||
            !std::isfinite(radius) || radius <= 0.0f)
            throw std::invalid_argument(
                "Disc2D: center must be finite and radius must be finite "
                "and > 0, got center=(" + std::to_string(center.x) + ", " +
                std::to_string(center.y) + ") radius=" + std::to_string(radius) +
                " — a non-positive or non-finite radius inverts the field's "
                "sign class (the region becomes the unbounded complement "
                "or NaN). Remedy: construct with radius > 0.");
    }

    SDFSample2D sample(const math::Vec2f& p) const noexcept {
        const math::Vec2f u = p - c_;
        const float d = u.length();
        const float f = hole_ ? (r_ - d) : (d - r_);
        // Gradient: ±u/|u|. At the exact center the gradient is
        // undefined; normalized() of the zero vector returns zero — the
        // SphereSDF precedent (the surface-extraction code never asks
        // for normals there: the center is at field depth r, the
        // deepest point of the region).
        const math::Vec2f g = hole_ ? (u * -1.0f).normalized() : u.normalized();
        return SDFSample2D{f, g};
    }

    float lipschitz() const noexcept { return 1.0f; }

    // Outer: the disc's box. Hole: the region is the unbounded
    // complement, so the contract-true box is the universe (see the
    // honesty note in the header comment — [C±r]² there would be
    // unsound for ring-like profiles).
    Bbox2f bounds() const noexcept {
        if (hole_) return Bbox2f::universe();
        Bbox2f b;
        b.expand(math::Vec2f{c_.x - r_, c_.y - r_});
        b.expand(math::Vec2f{c_.x + r_, c_.y + r_});
        return b;
    }

    std::string describe() const {
        return std::string(hole_ ? "hole-disc(" : "disc(") +
               "c=(" + std::to_string(c_.x) + ", " + std::to_string(c_.y) +
               "), r=" + std::to_string(r_) + ")";
    }

    const math::Vec2f& center() const noexcept { return c_; }
    float radius() const noexcept { return r_; }
    bool hole() const noexcept { return hole_; }

private:
    math::Vec2f c_;
    float r_;
    bool hole_;
};

} // namespace CAD_0::sdf

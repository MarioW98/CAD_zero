// core/brep/include/CAD_0/brep/curve.hpp
//
// Analytic curves: Line, Circle, Ellipse.
//
// Each curve is parametrized by t and provides:
//   * evaluate(t) → 3D point
//   * tangent(t) → unit tangent vector
//   * length() → arc length (or infinity for lines)
//
#pragma once

#include "CAD_0/math/vec.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace CAD_0::brep {

// Line: from point `p0` in direction `dir` (normalized).
class LineCurve {
public:
    LineCurve(math::Vec3f p0, math::Vec3f dir)
        : p0_(p0), dir_(dir.normalized()) {}

    math::Vec3f evaluate(float t) const noexcept {
        return p0_ + dir_ * t;
    }

    math::Vec3f tangent(float) const noexcept { return dir_; }

    float length() const noexcept { return std::numeric_limits<float>::infinity(); }

    math::Vec3f origin() const noexcept { return p0_; }
    math::Vec3f direction() const noexcept { return dir_; }

private:
    math::Vec3f p0_;
    math::Vec3f dir_;
};

// Circle: radius R in a plane, centered at `center`.
//
// THE FRAME (G2 unit 1, ADR-0033 D1.1): the circle's plane is spanned
// by the orthonormal (u_ref, v_ref) —
//
//     evaluate(t) = center + R·(cos t·u_ref + sin t·v_ref),
//     tangent(t)  = −sin t·u_ref + cos t·v_ref.
//
// The HISTORICAL frame is the default and is untouched: (X, Z), the
// in-XZ-plane-around-Y parametrization every pre-G2 pin reads (the
// constructor without a frame keeps it, so no golden moves). The
// table's entries derive their own frames from the operands (the cut
// plane's own (u, v), the radical plane's, the cylinder's u=0
// meridian) — the frame is DERIVED state serving the geometry, never
// the reverse (the ADR-0002/0031 doctrine, the re-frame rule's own
// premise).
class CircleCurve {
public:
    CircleCurve(float radius, math::Vec3f center = {})
        : radius_(radius), center_(center),
          u_ref_{1, 0, 0}, v_ref_{0, 0, 1} {}

    // The derived frame: (u_ref, v_ref) must be perpendicular unit-ish
    // directions (each is normalized here — the DIRECTIONS are the
    // frame; a cross-product-derived pair is unit up to fp). The
    // PERPENDICULARITY is validated loudly: a skewed pair has no
    // circle in its span, and accepting it would bend every
    // evaluated point out of the intended plane.
    CircleCurve(float radius, math::Vec3f center,
                math::Vec3f u_ref, math::Vec3f v_ref)
        : radius_(radius), center_(center),
          u_ref_(u_ref.normalized()), v_ref_(v_ref.normalized()) {
        const float du = u_ref_.length();
        const float dv = v_ref_.length();
        const float dot = u_ref_.dot(v_ref_);
        if (!(du > 0.0f) || !(dv > 0.0f) || !std::isfinite(du) ||
            !std::isfinite(dv) || std::abs(dot) > 1e-6f) {
            throw std::invalid_argument(
                "CircleCurve: (u_ref, v_ref) must be non-zero and "
                "perpendicular (an orthonormal in-plane frame), got "
                "|u_ref|=" + std::to_string(du) + " |v_ref|=" +
                std::to_string(dv) + " u_ref·v_ref=" + std::to_string(dot) +
                " — a skewed frame has no circle in its span: every "
                "evaluated point would leave the intended plane. "
                "Remedy: pass the derived frame (the cut plane's own "
                "axes, or the entry's documented derivation).");
        }
    }

    math::Vec3f evaluate(float t) const noexcept {
        return center_ + u_ref_ * (radius_ * std::cos(t)) +
               v_ref_ * (radius_ * std::sin(t));
    }

    math::Vec3f tangent(float t) const noexcept {
        return u_ref_ * (-std::sin(t)) + v_ref_ * std::cos(t);
    }

    float length() const noexcept { return 2.0f * 3.14159265358979f * radius_; }

    float radius() const noexcept { return radius_; }
    math::Vec3f center() const noexcept { return center_; }
    const math::Vec3f& u_ref() const noexcept { return u_ref_; }
    const math::Vec3f& v_ref() const noexcept { return v_ref_; }

private:
    float radius_;
    math::Vec3f center_;
    math::Vec3f u_ref_;
    math::Vec3f v_ref_;
};

// Ellipse: semi-axes a (X) and b (Z), in the XZ plane, centered at origin.
class EllipseCurve {
public:
    EllipseCurve(float a, float b, math::Vec3f center = {})
        : a_(a), b_(b), center_(center) {}

    math::Vec3f evaluate(float t) const noexcept {
        return {center_.x + a_ * std::cos(t),
                center_.y,
                center_.z + b_ * std::sin(t)};
    }

    math::Vec3f tangent(float t) const noexcept {
        const float tx = -a_ * std::sin(t);
        const float tz = b_ * std::cos(t);
        const float len = std::sqrt(tx * tx + tz * tz);
        if (len < 1e-9f) return {0, 0, 1};
        return {tx / len, 0.0f, tz / len};
    }

    // Approximate arc length (Ramanujan's formula)
    float length() const noexcept {
        const float h = static_cast<float>(std::pow((a_ - b_) / (a_ + b_), 2));
        return 3.14159265358979f * (a_ + b_) * (1.0f + 3.0f * h / (10.0f + std::sqrt(4.0f - 3.0f * h)));
    }

private:
    float a_, b_;
    math::Vec3f center_;
};

} // namespace CAD_0::brep

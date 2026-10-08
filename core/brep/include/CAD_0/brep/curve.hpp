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

// Circle: radius R, in the XZ plane (Y up), centered at origin.
class CircleCurve {
public:
    CircleCurve(float radius, math::Vec3f center = {})
        : radius_(radius), center_(center) {}

    math::Vec3f evaluate(float t) const noexcept {
        return {center_.x + radius_ * std::cos(t),
                center_.y,
                center_.z + radius_ * std::sin(t)};
    }

    math::Vec3f tangent(float t) const noexcept {
        return {-std::sin(t), 0.0f, std::cos(t)};
    }

    float length() const noexcept { return 2.0f * 3.14159265358979f * radius_; }

    float radius() const noexcept { return radius_; }
    math::Vec3f center() const noexcept { return center_; }

private:
    float radius_;
    math::Vec3f center_;
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
        const float h = std::pow((a_ - b_) / (a_ + b_), 2);
        return 3.14159265358979f * (a_ + b_) * (1.0f + 3.0f * h / (10.0f + std::sqrt(4.0f - 3.0f * h)));
    }

private:
    float a_, b_;
    math::Vec3f center_;
};

} // namespace CAD_0::brep

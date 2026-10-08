// core/brep/include/CAD_0/brep/analytic.hpp
//
// Analytic surfaces: Plane, Cylinder, Cone, Sphere, Torus.
//
// Each surface is parametrized by (u, v) and provides:
//   * evaluate(u, v) → 3D point on the surface
//   * normal(u, v) → unit normal at that point
//   * bounds() → bounding box
//
#pragma once

#include "CAD_0/brep/topology.hpp"
#include "CAD_0/math/vec.hpp"
#include "CAD_0/math/mat.hpp"

#include <cmath>

namespace CAD_0::brep {

// Plane: a flat surface defined by origin + normal.
// Parametrization: u along X-axis, v along Y-axis (in local frame).
class PlaneSurface : public SurfaceGeometry {
public:
    PlaneSurface(math::Vec3f origin, math::Vec3f normal)
        : origin_(origin), normal_(normal.normalized()) {
        // Build local frame: u-axis perpendicular to normal
        if (std::abs(normal_.y) > 0.9f) {
            u_axis_ = normal_.cross(math::Vec3f{1, 0, 0}).normalized();
        } else {
            u_axis_ = normal_.cross(math::Vec3f{0, 1, 0}).normalized();
        }
        v_axis_ = normal_.cross(u_axis_).normalized();
    }

    math::Vec3f evaluate(float u, float v) const override {
        return origin_ + u_axis_ * u + v_axis_ * v;
    }

    math::Vec3f normal(float, float) const override { return normal_; }
    const char* type_name() const override { return "plane"; }
    math::Bboxf bounds() const override { return math::Bboxf::universe(); }

    math::Vec3f origin() const noexcept { return origin_; }
    math::Vec3f normal_vec() const noexcept { return normal_; }

private:
    math::Vec3f origin_;
    math::Vec3f normal_;
    math::Vec3f u_axis_;
    math::Vec3f v_axis_;
};

// Cylinder: radius R, axis along Y, centered at origin.
// u = angle [0, 2π), v = height along axis.
class CylinderSurface : public SurfaceGeometry {
public:
    CylinderSurface(float radius, float height, math::Vec3f origin = {})
        : radius_(radius), half_height_(height * 0.5f), origin_(origin) {}

    math::Vec3f evaluate(float u, float v) const override {
        const float c = std::cos(u), s = std::sin(u);
        return {origin_.x + radius_ * c,
                origin_.y + v,
                origin_.z + radius_ * s};
    }

    math::Vec3f normal(float u, float) const override {
        return {std::cos(u), 0.0f, std::sin(u)};
    }

    const char* type_name() const override { return "cylinder"; }
    math::Bboxf bounds() const override {
        return {{-radius_, -half_height_, -radius_},
                { radius_,  half_height_,  radius_}};
    }

    float radius() const noexcept { return radius_; }
    float height() const noexcept { return half_height_ * 2.0f; }

private:
    float radius_;
    float half_height_;
    math::Vec3f origin_;
};

// Sphere: radius R centered at origin.
// u = azimuth [0, 2π), v = polar angle [0, π].
class SphereSurface : public SurfaceGeometry {
public:
    SphereSurface(float radius, math::Vec3f origin = {})
        : radius_(radius), origin_(origin) {}

    math::Vec3f evaluate(float u, float v) const override {
        const float sv = std::sin(v);
        return {origin_.x + radius_ * sv * std::cos(u),
                origin_.y + radius_ * std::cos(v),
                origin_.z + radius_ * sv * std::sin(u)};
    }

    math::Vec3f normal(float u, float v) const override {
        const float sv = std::sin(v);
        return {sv * std::cos(u), std::cos(v), sv * std::sin(u)};
    }

    const char* type_name() const override { return "sphere"; }
    math::Bboxf bounds() const override {
        return {{-radius_ + origin_.x, -radius_ + origin_.y, -radius_ + origin_.z},
                { radius_ + origin_.x,  radius_ + origin_.y,  radius_ + origin_.z}};
    }

    float radius() const noexcept { return radius_; }

private:
    float radius_;
    math::Vec3f origin_;
};

// Cone: base radius r at y=0, apex at y=h, centered at origin.
// u = angle [0, 2π), v = height [0, h].
class ConeSurface : public SurfaceGeometry {
public:
    ConeSurface(float base_radius, float height, math::Vec3f origin = {})
        : base_radius_(base_radius), height_(height), origin_(origin) {}

    math::Vec3f evaluate(float u, float v) const override {
        const float r = base_radius_ * (1.0f - v / height_);
        return {origin_.x + r * std::cos(u),
                origin_.y + v,
                origin_.z + r * std::sin(u)};
    }

    math::Vec3f normal(float u, float v) const override {
        // Normal points outward; slope = base_radius / height
        const float slope = base_radius_ / height_;
        const float c = std::cos(u), s = std::sin(u);
        const float n = 1.0f / std::sqrt(slope * slope + 1.0f);
        return {c * n, slope * n, s * n};
    }

    const char* type_name() const override { return "cone"; }
    math::Bboxf bounds() const override {
        return {{-base_radius_, 0, -base_radius_},
                { base_radius_, height_, base_radius_}};
    }

    float base_radius() const noexcept { return base_radius_; }
    float height() const noexcept { return height_; }

private:
    float base_radius_;
    float height_;
    math::Vec3f origin_;
};

// Torus: major radius R (ring), minor radius r (tube), in XZ plane.
// u = angle around ring [0, 2π), v = angle around tube [0, 2π).
class TorusSurface : public SurfaceGeometry {
public:
    TorusSurface(float R, float r, math::Vec3f origin = {})
        : R_(R), r_(r), origin_(origin) {}

    math::Vec3f evaluate(float u, float v) const override {
        const float cu = std::cos(u), su = std::sin(u);
        const float cv = std::cos(v), sv = std::sin(v);
        return {origin_.x + (R_ + r_ * cv) * cu,
                origin_.y + r_ * sv,
                origin_.z + (R_ + r_ * cv) * su};
    }

    math::Vec3f normal(float u, float v) const override {
        const float cv = std::cos(v), sv = std::sin(v);
        const float cu = std::cos(u), su = std::sin(u);
        return {cv * cu, sv, cv * su};
    }

    const char* type_name() const override { return "torus"; }
    math::Bboxf bounds() const override {
        const float e = R_ + r_;
        return {{-e + origin_.x, -r_ + origin_.y, -e + origin_.z},
                { e + origin_.x,  r_ + origin_.y,  e + origin_.z}};
    }

    float major_radius() const noexcept { return R_; }
    float minor_radius() const noexcept { return r_; }

private:
    float R_, r_;
    math::Vec3f origin_;
};

} // namespace CAD_0::brep

// core/sdf/include/CAD_0/sdf/primitives.hpp
//
// SDF primitive nodes — the leaves of an SDF expression tree.
// All primitives are centered at the origin in local space; use
// Transform nodes (see transforms.hpp) to place them.
//
// References:
//   * Inigo Quilez — https://iquilezles.org/articles/distfunctions/
//   * Siemens NX Implicit Modeling — design philosophy
//
#pragma once

#include "CAD_0/sdf/field.hpp"

namespace CAD_0::sdf {

// Sphere of given radius centered at the origin.
class SphereSDF final : public SDFNode {
public:
    explicit SphereSDF(float radius) noexcept : radius_(radius) {}

    SDFSample sample(const math::Vec3f& p) const noexcept override {
        const math::Vec3f g = p / radius_;
        const float d = p.length() - radius_;
        return SDFSample{d, g.normalized()};
    }
    float lipschitz()      const noexcept override { return 1.0f; }
    math::Bboxf bounds()   const noexcept override {
        return { {-radius_, -radius_, -radius_}, {radius_, radius_, radius_} };
    }
    std::string describe() const override { return "sphere(r=" + std::to_string(radius_) + ")"; }
    std::unique_ptr<SDFNode> clone() const override { return std::make_unique<SphereSDF>(*this); }
    float radius() const noexcept { return radius_; }

private:
    float radius_;
};

// Axis-aligned box centered at the origin with half-extent `extent`.
class BoxSDF final : public SDFNode {
public:
    explicit BoxSDF(math::Vec3f extent) noexcept : extent_(extent) {}

    SDFSample sample(const math::Vec3f& p) const noexcept override {
        const math::Vec3f q{std::abs(p.x) - extent_.x,
                            std::abs(p.y) - extent_.y,
                            std::abs(p.z) - extent_.z};
        const math::Vec3f outer{std::max(q.x, 0.0f),
                                std::max(q.y, 0.0f),
                                std::max(q.z, 0.0f)};
        const float d = outer.length() + std::min(std::max({q.x, q.y, q.z}), 0.0f);
        // Gradient = sign(p) * sign(qmax)
        math::Vec3f g{
            (q.x > q.y && q.x > q.z) ? (p.x > 0 ? 1.0f : -1.0f) : 0.0f,
            (q.y > q.x && q.y > q.z) ? (p.y > 0 ? 1.0f : -1.0f) : 0.0f,
            (q.z > q.x && q.z > q.y) ? (p.z > 0 ? 1.0f : -1.0f) : 0.0f,
        };
        return SDFSample{d, g};
    }
    float lipschitz()      const noexcept override { return 1.0f; }
    math::Bboxf bounds()   const noexcept override {
        return { -extent_, extent_ };
    }
    std::string describe() const override {
        return "box(e=" + std::to_string(extent_.x) + "," +
                         std::to_string(extent_.y) + "," +
                         std::to_string(extent_.z) + ")";
    }
    std::unique_ptr<SDFNode> clone() const override { return std::make_unique<BoxSDF>(*this); }
    math::Vec3f extent() const noexcept { return extent_; }

private:
    math::Vec3f extent_;
};

// Cylinder of given radius and height, aligned with the Y axis.
// Centered at origin.
class CylinderSDF final : public SDFNode {
public:
    CylinderSDF(float radius, float height) noexcept
        : radius_(radius), half_height_(height * 0.5f) {}

    SDFSample sample(const math::Vec3f& p) const noexcept override {
        const math::Vec2f d{std::sqrt(p.x * p.x + p.z * p.z) - radius_,
                            std::abs(p.y) - half_height_};
        const float dist = std::min(std::max(d.x, d.y), 0.0f) +
                           math::Vec2f{std::max(d.x, 0.0f), std::max(d.y, 0.0f)}.length();
        math::Vec3f g{p.x, 0.0f, p.z};
        const float rg = g.length();
        if (rg > 1e-9f) g = g * (1.0f / rg);
        g.y = (p.y > 0) ? 1.0f : -1.0f;
        return SDFSample{dist, g};
    }
    float lipschitz()      const noexcept override { return 1.0f; }
    math::Bboxf bounds()   const noexcept override {
        return { {-radius_, -half_height_, -radius_}, {radius_, half_height_, radius_} };
    }
    std::string describe() const override {
        return "cylinder(r=" + std::to_string(radius_) + ",h=" + std::to_string(half_height_ * 2.0f) + ")";
    }
    std::unique_ptr<SDFNode> clone() const override { return std::make_unique<CylinderSDF>(*this); }

private:
    float radius_;
    float half_height_;
};

// Torus with major radius R and minor radius r, in the XZ plane.
class TorusSDF final : public SDFNode {
public:
    TorusSDF(float R, float r) noexcept : R_(R), r_(r) {}

    SDFSample sample(const math::Vec3f& p) const noexcept override {
        const math::Vec2f q{std::sqrt(p.x * p.x + p.z * p.z) - R_, p.y};
        const float d = q.length() - r_;
        math::Vec2f g = q;
        const float ql = q.length();
        if (ql > 1e-9f) g = g * (1.0f / ql);
        math::Vec3f g3{p.x, 0.0f, p.z};
        const float g3l = g3.length();
        if (g3l > 1e-9f) g3 = g3 * (1.0f / g3l);
        return SDFSample{d, math::Vec3f{g3.x * g.x, g.y, g3.z * g.x}};
    }
    float lipschitz()      const noexcept override { return 1.0f; }
    math::Bboxf bounds()   const noexcept override {
        const float e = R_ + r_;
        return { {-e, -r_, -e}, {e, r_, e} };
    }
    std::string describe() const override {
        return "torus(R=" + std::to_string(R_) + ",r=" + std::to_string(r_) + ")";
    }
    std::unique_ptr<SDFNode> clone() const override { return std::make_unique<TorusSDF>(*this); }

private:
    float R_;
    float r_;
};

// Cone along +Y, base radius `r` at y = 0, apex at y = h.
class ConeSDF final : public SDFNode {
public:
    ConeSDF(float r, float h) noexcept : r_(r), h_(h) {}

    SDFSample sample(const math::Vec3f& p) const noexcept override {
        const math::Vec2f q{r_ * (p.y / h_), h_ - p.y};
        const float d = (p.x * p.x + p.z * p.z) > 0.0f
            ? math::Vec2f{std::sqrt(p.x * p.x + p.z * p.z), p.y}.length() - r_ * (1.0f - p.y / h_)
            : (q.x - q.y);
        (void)q;
        return SDFSample{d, math::Vec3f{p.x, 0.0f, p.z}.normalized()};
    }
    float lipschitz()      const noexcept override { return 1.0f; }
    math::Bboxf bounds()   const noexcept override {
        return { {-r_, 0.0f, -r_}, {r_, h_, r_} };
    }
    std::string describe() const override {
        return "cone(r=" + std::to_string(r_) + ",h=" + std::to_string(h_) + ")";
    }
    std::unique_ptr<SDFNode> clone() const override { return std::make_unique<ConeSDF>(*this); }

private:
    float r_;
    float h_;
};

// Capsule (line + radius) from point a to b.
class CapsuleSDF final : public SDFNode {
public:
    CapsuleSDF(math::Vec3f a, math::Vec3f b, float r) noexcept
        : a_(a), b_(b), r_(r) {
        // Precompute the segment direction and length.
        const math::Vec3f d = b_ - a_;
        length_sq_ = d.length_sq();
    }

    SDFSample sample(const math::Vec3f& p) const noexcept override {
        math::Vec3f pa = p - a_;
        const math::Vec3f ba = b_ - a_;
        float t = pa.dot(ba) / std::max(length_sq_, 1e-12f);
        t = std::clamp(t, 0.0f, 1.0f);
        const math::Vec3f closest = a_ + ba * t;
        const math::Vec3f diff = p - closest;
        const float dist = diff.length() - r_;
        return SDFSample{dist, diff.normalized()};
    }
    float lipschitz()      const noexcept override { return 1.0f; }
    math::Bboxf bounds()   const noexcept override {
        math::Bboxf b;
        b.expand(a_ + math::Vec3f{r_, r_, r_});
        b.expand(a_ - math::Vec3f{r_, r_, r_});
        b.expand(b_ + math::Vec3f{r_, r_, r_});
        b.expand(b_ - math::Vec3f{r_, r_, r_});
        return b;
    }
    std::string describe() const override {
        return "capsule(r=" + std::to_string(r_) + ")";
    }
    std::unique_ptr<SDFNode> clone() const override { return std::make_unique<CapsuleSDF>(*this); }

private:
    math::Vec3f a_;
    math::Vec3f b_;
    float r_;
    float length_sq_{1.0f};
};

// Infinite plane at y = 0 (normal points +Y). Half-space y > 0 is inside.
class PlaneSDF final : public SDFNode {
public:
    SDFSample sample(const math::Vec3f& p) const noexcept override {
        // Inside (y > 0) → negative distance (so negate y).
        // Outside (y < 0) → positive distance.
        return SDFSample{-p.y, math::Vec3f{0.0f, -1.0f, 0.0f}};
    }
    float lipschitz()    const noexcept override { return 1.0f; }
    math::Bboxf bounds() const noexcept override {
        return math::Bboxf::universe();
    }
    std::string describe() const override { return "plane(y=0)"; }
    std::unique_ptr<SDFNode> clone() const override { return std::make_unique<PlaneSDF>(*this); }
};

// ---- Free helper constructors (used by Python bindings) -------------------
inline SDFBody make_sphere(float r) { return SDFBody(std::make_unique<SphereSDF>(r)); }
inline SDFBody make_box(math::Vec3f e) { return SDFBody(std::make_unique<BoxSDF>(e)); }
inline SDFBody make_cylinder(float r, float h) { return SDFBody(std::make_unique<CylinderSDF>(r, h)); }
inline SDFBody make_torus(float R, float r) { return SDFBody(std::make_unique<TorusSDF>(R, r)); }
inline SDFBody make_cone(float r, float h) { return SDFBody(std::make_unique<ConeSDF>(r, h)); }
inline SDFBody make_capsule(math::Vec3f a, math::Vec3f b, float r) {
    return SDFBody(std::make_unique<CapsuleSDF>(a, b, r));
}
inline SDFBody make_plane() { return SDFBody(std::make_unique<PlaneSDF>()); }

} // namespace CAD_0::sdf

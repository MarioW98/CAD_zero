// core/sdf/include/CAD_0/sdf/primitives.hpp
//
// SDF primitive nodes — the leaves of an SDF expression tree.
// All primitives are centered at the origin in local space; use
// Transform nodes (see transforms.hpp) to place them.
//
// References:
//   * Inigo Quilez — https://iquilezles.org/articles/distfunctions/
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
//
// The exact SDF is the standard "2D cross-section" formulation: d is the
// length of (max(d_radial, 0), max(d_axial, 0)) plus the min of the
// (signed) inside-distance. The gradient must be the outward unit
// normal of whichever surface element is closest:
//   * on the lateral surface (rho = r, |y| < half_h): (x, 0, z)/rho
//   * on the top cap (y = +half_h, rho < r):  (0, +1, 0)
//   * on the bottom cap (y = -half_h, rho < r): (0, -1, 0)
//   * on the rim edges (rho = r AND |y| = half_h): discontinuous;
//     we pick the radial direction (the lateral normal), which is the
//     convention used by the surface-extraction code.
//
// Crucially the returned gradient MUST have magnitude 1 everywhere on
// the surface (and ≤ 1 in the interior). The previous implementation
// left `g.y` set to ±1 *and* the radial component non-zero, producing
// |g| = sqrt(2) on the rim — this would break any downstream consumer
// using the analytic gradient (e.g. GPU shading, narrow-band MC).
class CylinderSDF final : public SDFNode {
public:
    CylinderSDF(float radius, float height) noexcept
        : radius_(radius), half_height_(height * 0.5f) {}

    SDFSample sample(const math::Vec3f& p) const noexcept override {
        // 2D cross-section: d = (rho - r, |y| - half_h)
        const float rho = std::sqrt(p.x * p.x + p.z * p.z);
        const float dx = rho - radius_;
        const float dy = std::abs(p.y) - half_height_;

        // Outside distance (positive): only the components that exceed 0.
        const float ox = std::max(dx, 0.0f);
        const float oy = std::max(dy, 0.0f);
        const float d_out = std::sqrt(ox * ox + oy * oy);

        // Inside distance (negative): max of the (negative) signed
        // distances. If we're outside on both axes this is ≤ 0 so the
        // min(d_out, ...) branch picks d_out.
        const float d_in = std::min(std::max(dx, dy), 0.0f);

        const float dist = d_out + d_in;

        // ----- Gradient (normalized) -----
        // The closest surface element is determined by which of dx, dy
        // is the *positive* maximum (i.e. which one we are "exiting"
        // through). If both are negative (inside) we pick the one
        // closest to zero (i.e. the max).
        math::Vec3f g{};
        if (dy >= dx) {
            // Closest to a cap. Determine which one by the sign of p.y.
            g = math::Vec3f{0.0f, (p.y >= 0.0f) ? 1.0f : -1.0f, 0.0f};
        } else {
            // Closest to the lateral surface. Outward normal in 3D is
            // the radial unit vector (x, 0, z)/rho.
            if (rho > 1e-9f) {
                const float inv_rho = 1.0f / rho;
                g = math::Vec3f{p.x * inv_rho, 0.0f, p.z * inv_rho};
            } else {
                // On the axis: the lateral surface can't be the closest
                // (we're inside the cylinder), so this branch shouldn't
                // trigger. Fall back to +Y as a safe default.
                g = math::Vec3f{0.0f, 1.0f, 0.0f};
            }
        }
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

    // Native-format serialization accessors.
    float radius() const noexcept { return radius_; }
    float height() const noexcept { return half_height_ * 2.0f; } // full height, as constructed

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

    // Native-format serialization accessors.
    float major_radius() const noexcept { return R_; }
    float minor_radius() const noexcept { return r_; }

private:
    float R_;
    float r_;
};


// Cone along +Y, base radius `r` at y = 0, apex at y = h.
//
// SDF = max(ww, -y) where:
//   ww = signed distance to the infinite lateral line (outward positive)
//   -y = signed distance to the base plane (upward positive)
//
// Both ww and -y are 1-Lipschitz, so max(ww, -y) is 1-Lipschitz.
// The VALUE is C0-continuous everywhere. The gradient has a discontinuity
// at the rim (sharp geometric edge where the lateral meets the base),
// but |f(p)-f(q)| <= |p-q| always holds.
//
// This is a CONSERVATIVE SDF: for points past the apex, ww underestimates
// the true distance to the finite cone. This is safe for ray marching
// (takes more steps but never overshoots).
class ConeSDF final : public SDFNode {
public:
    ConeSDF(float r, float h) noexcept : r_(r), h_(h) {}

    SDFSample sample(const math::Vec3f& p) const noexcept override {
        const float rho = std::sqrt(p.x * p.x + p.z * p.z);
        const float y = p.y;

        // Outward normal of the lateral line: (h, r) / L.
        const float L = std::sqrt(r_ * r_ + h_ * h_);
        const float cos_theta = (L > 1e-9f) ? h_ / L : 1.0f;
        const float sin_theta = (L > 1e-9f) ? r_ / L : 0.0f;

        // Signed distance to the infinite lateral line.
        // pa = (rho - r, y), outward normal = (cos_theta, sin_theta).
        // ww = dot(pa, outward_normal).
        const float ww = (rho - r_) * cos_theta + y * sin_theta;

        // d = max(ww, -y).
        const float d_base = -y;
        const float d = std::max(ww, d_base);

        // Gradient.
        math::Vec3f g{};
        if (d_base >= ww) {
            g = math::Vec3f{0.0f, -1.0f, 0.0f};
        } else if (rho > 1e-9f) {
            const float inv_rho = 1.0f / rho;
            g = math::Vec3f{cos_theta * p.x * inv_rho,
                            sin_theta,
                            cos_theta * p.z * inv_rho};
        } else {
            g = math::Vec3f{0.0f, sin_theta, 0.0f};
        }
        return SDFSample{d, g};
    }
    float lipschitz()      const noexcept override { return 1.0f; }
    math::Bboxf bounds()   const noexcept override {
        return { {-r_, 0.0f, -r_}, {r_, h_, r_} };
    }
    std::string describe() const override {
        return "cone(r=" + std::to_string(r_) + ",h=" + std::to_string(h_) + ")";
    }
    std::unique_ptr<SDFNode> clone() const override { return std::make_unique<ConeSDF>(*this); }

    // Native-format serialization accessors.
    float base_radius() const noexcept { return r_; }
    float height() const noexcept { return h_; }

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

    // Native-format serialization accessors.
    const math::Vec3f& point_a() const noexcept { return a_; }
    const math::Vec3f& point_b() const noexcept { return b_; }
    float radius() const noexcept { return r_; }

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

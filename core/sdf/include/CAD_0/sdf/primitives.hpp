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
#include "CAD_0/sdf/detail/validate.hpp"

namespace CAD_0::sdf {

// NOTE (ADR-0029 D5): every constructor below validates its inputs
// and throws std::invalid_argument (bridge-style message: operator,
// parameter, value, supported class, diagnosis, remedy). Constructors
// are therefore NOT noexcept; sample() and the other query methods
// keep theirs — no hot-path change.

// Sphere of given radius centered at the origin.
class SphereSDF final : public SDFNode {
public:
    explicit SphereSDF(float radius) : radius_(radius) {
        detail::require_positive(radius, "SphereSDF", "radius",
            "a non-positive or non-finite radius inverts the field's sign "
            "class (the 'inside' becomes the unbounded complement) and the "
            "gradient divides by it (p / radius)",
            "construct with radius > 0");
    }

    SDFSample sample(const math::Vec3f& p) const noexcept override;
    // PERF 3: the xsimd kernel (core/sdf/src/batch_kernels.cpp) —
    // bit-identical to sample() by the per-class corpus pin.
    void sample_batch(std::span<const math::Vec3f> points, float* values,
                      math::Vec3f* gradients) const noexcept override;
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
    explicit BoxSDF(math::Vec3f extent) : extent_(extent) {
        detail::require_positive_vec3(extent, "BoxSDF", "extent",
            "a non-positive or non-finite half-extent makes the box "
            "degenerate (empty or sign-inverted) and its bounds() empty",
            "construct with a positive half-extent per axis (a full-size "
            "box uses extent = full/2)");
    }

    SDFSample sample(const math::Vec3f& p) const noexcept override;
    // PERF 3: the xsimd kernel — bit-identical, per-class pin.
    void sample_batch(std::span<const math::Vec3f> points, float* values,
                      math::Vec3f* gradients) const noexcept override;
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
    CylinderSDF(float radius, float height)
        : radius_(radius), half_height_(height * 0.5f) {
        detail::require_positive(radius, "CylinderSDF", "radius",
            "a non-positive or non-finite radius inverts the field's sign "
            "class and the gradient divides by it ((x,0,z)/rho)",
            "construct with radius > 0");
        detail::require_positive(height, "CylinderSDF", "height",
            "a non-positive or non-finite height makes the cylinder "
            "degenerate (empty slab) and its bounds() flat",
            "construct with height > 0");
    }

    SDFSample sample(const math::Vec3f& p) const noexcept override;
    // PERF 3: the xsimd kernel — bit-identical, per-class pin.
    void sample_batch(std::span<const math::Vec3f> points, float* values,
                      math::Vec3f* gradients) const noexcept override;
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
    TorusSDF(float R, float r) : R_(R), r_(r) {
        // ADR-0029 D5, minimal-list principle: r > R (a spindle torus)
        // is LEGAL and deliberately unchecked — the offset field and
        // its box remain valid for it. Only the contract-breaking
        // class (non-finite, non-positive) is rejected.
        detail::require_positive(R, "TorusSDF", "R",
            "a non-positive or non-finite major radius inverts the field's "
            "sign class (the tube sweeps the mirror complement)",
            "construct with R > 0 and r > 0 — a spindle torus (r > R) is "
            "legal and stays supported");
        detail::require_positive(r, "TorusSDF", "r",
            "a non-positive or non-finite minor radius makes the tube "
            "empty and the gradient divide by it",
            "construct with R > 0 and r > 0 — a spindle torus (r > R) is "
            "legal and stays supported");
    }

    SDFSample sample(const math::Vec3f& p) const noexcept override;
    // PERF 3: the xsimd kernel — bit-identical, per-class pin.
    void sample_batch(std::span<const math::Vec3f> points, float* values,
                      math::Vec3f* gradients) const noexcept override;
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
    ConeSDF(float r, float h) : r_(r), h_(h) {
        detail::require_positive(r, "ConeSDF", "r",
            "a non-positive or non-finite base radius inverts the field's "
            "sign class",
            "construct with r > 0");
        detail::require_positive(h, "ConeSDF", "h",
            "a non-positive or non-finite height makes the cone degenerate "
            "(empty or flat) and bounds() empty",
            "construct with h > 0");
    }

    SDFSample sample(const math::Vec3f& p) const noexcept override;
    // PERF 3: the xsimd kernel — bit-identical, per-class pin.
    void sample_batch(std::span<const math::Vec3f> points, float* values,
                      math::Vec3f* gradients) const noexcept override;
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
    CapsuleSDF(math::Vec3f a, math::Vec3f b, float r)
        : a_(a), b_(b), r_(r) {
        // Precompute the segment direction and length.
        const math::Vec3f d = b_ - a_;
        length_sq_ = d.length_sq();
        // ADR-0029 D5: a = b is LEGAL (degenerates to a sphere behind
        // the explicit std::max(length_sq_, 1e-12) guard in sample()) —
        // do not legislate taste; reject only the contract-breaking
        // class (non-finite endpoints, non-positive radius).
        detail::require_finite_vec3(a, "CapsuleSDF", "a",
            "a non-finite endpoint puts the segment off the number line: "
            "sample(), bounds() and the gradient are NaN/garbage for "
            "every query",
            "construct with finite endpoints (a = b is allowed — it "
            "degenerates to a sphere behind an explicit guard)");
        detail::require_finite_vec3(b, "CapsuleSDF", "b",
            "a non-finite endpoint puts the segment off the number line: "
            "sample(), bounds() and the gradient are NaN/garbage for "
            "every query",
            "construct with finite endpoints (a = b is allowed — it "
            "degenerates to a sphere behind an explicit guard)");
        detail::require_positive(r, "CapsuleSDF", "r",
            "a non-positive or non-finite radius inverts the field's sign "
            "class (the 'inside' becomes the unbounded complement)",
            "construct with radius > 0");
    }

    SDFSample sample(const math::Vec3f& p) const noexcept override;
    // PERF 3: the xsimd kernel — bit-identical, per-class pin.
    void sample_batch(std::span<const math::Vec3f> points, float* values,
                      math::Vec3f* gradients) const noexcept override;
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
    SDFSample sample(const math::Vec3f& p) const noexcept override;
    // PERF 3: the xsimd kernel — bit-identical, per-class pin.
    void sample_batch(std::span<const math::Vec3f> points, float* values,
                      math::Vec3f* gradients) const noexcept override;
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

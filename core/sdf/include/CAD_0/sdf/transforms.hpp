// core/sdf/include/CAD_0/sdf/transforms.hpp
//
// Spatial transforms applied to an SDF tree. These wrap a child node
// and re-express queries in the child's local coordinate system.
//
#pragma once

#include "CAD_0/sdf/field.hpp"
#include "CAD_0/sdf/detail/validate.hpp"
#include "CAD_0/math/mat.hpp"
#include "CAD_0/math/quat.hpp"

#include <cmath>

namespace CAD_0::sdf {

// Translate — shift the child field by a fixed offset.
class TranslateSDF final : public SDFNode {
public:
    TranslateSDF(std::unique_ptr<SDFNode> child, math::Vec3f offset)
        : child_(std::move(child)), offset_(offset) {
        detail::require_non_null(child_, "TranslateSDF", "child",
            "the wrapper dereferences it on every sample() (undefined "
            "behavior, not an empty field)",
            "construct the child first; an empty SDFBody has no root to "
            "wrap");
        detail::require_finite_vec3(offset, "TranslateSDF", "offset",
            "a non-finite offset shifts the field off the number line: "
            "sample() and bounds() return NaN for every query",
            "translate by a finite vector");
    }

    SDFSample sample(const math::Vec3f& p) const noexcept override {
        return child_->sample(p - offset_);
    }
    float lipschitz()    const noexcept override { return child_->lipschitz(); }
    math::Bboxf bounds() const noexcept override {
        // ADR-0028 bounds-contract audit: IMMUNE — the negative region
        // shifts rigidly with the field, and the box shifts with it.
        auto b = child_->bounds();
        return {b.min + offset_, b.max + offset_};
    }
    std::string describe() const override {
        return "translate(" + child_->describe() + ",[" +
               std::to_string(offset_.x) + "," +
               std::to_string(offset_.y) + "," +
               std::to_string(offset_.z) + "])";
    }
    std::unique_ptr<SDFNode> clone() const override {
        return std::make_unique<TranslateSDF>(child_->clone(), offset_);
    }

    // Native-format serialization accessors.
    const SDFNode* wrapped() const noexcept { return child_.get(); }
    const math::Vec3f& offset() const noexcept { return offset_; }

private:
    std::unique_ptr<SDFNode> child_;
    math::Vec3f offset_;
};

// Rotate — rotate the child field by a quaternion.
class RotateSDF final : public SDFNode {
public:
    RotateSDF(std::unique_ptr<SDFNode> child, math::Quatf q)
        : child_(std::move(child)), q_(q), inv_q_(q.conjugate()),
          qmat_(q.to_matrix()) {
        detail::require_non_null(child_, "RotateSDF", "child",
            "the wrapper dereferences it on every sample() (undefined "
            "behavior, not an empty field)",
            "construct the child first; an empty SDFBody has no root to "
            "wrap");
        // ADR-0029 D5: unit norm within 1e-6 (serialization round-trips
        // at ~1e-9). A non-unit quaternion rotates-AND-scales the field
        // by |q|^2, breaking the rigid-rotation Lipschitz, gradient and
        // bounds() contracts; NaN components fail the norm check.
        detail::require_unit_quat(q_, "RotateSDF",
            "a non-unit quaternion rotates AND scales the field by |q|^2, "
            "breaking the rigid-rotation Lipschitz, gradient and bounds() "
            "contracts (NaN components fail the norm check too)",
            "normalize the quaternion first (Quatf::normalized()) or build "
            "it with from_axis_angle(unit_axis, angle)");
    }

    SDFSample sample(const math::Vec3f& p) const noexcept override {
        const math::Vec3f local = inv_q_.rotate(p);
        auto s = child_->sample(local);
        s.gradient = qmat_.transform_dir(s.gradient);
        return s;
    }
    float lipschitz()    const noexcept override { return child_->lipschitz(); }
    math::Bboxf bounds() const noexcept override {
        // ADR-0028 bounds-contract audit: IMMUNE — a rigid rotation
        // maps the child's negative region exactly, and the corner
        // hull of the rotated child box contains it.
        auto b = child_->bounds();
        math::Bboxf out;
        // Rotate the 8 corners of the child's bbox.
        const math::Vec3f corners[8] = {
            {b.min.x, b.min.y, b.min.z}, {b.max.x, b.min.y, b.min.z},
            {b.min.x, b.max.y, b.min.z}, {b.max.x, b.max.y, b.min.z},
            {b.min.x, b.min.y, b.max.z}, {b.max.x, b.min.y, b.max.z},
            {b.min.x, b.max.y, b.max.z}, {b.max.x, b.max.y, b.max.z},
        };
        for (const auto& c : corners) out.expand(qmat_.transform_point(c));
        return out;
    }
    std::string describe() const override {
        return "rotate(" + child_->describe() + ")";
    }
    std::unique_ptr<SDFNode> clone() const override {
        return std::make_unique<RotateSDF>(child_->clone(), q_);
    }

    // Native-format serialization accessors.
    const SDFNode* wrapped() const noexcept { return child_.get(); }
    const math::Quatf& quat() const noexcept { return q_; }

private:
    std::unique_ptr<SDFNode> child_;
    math::Quatf q_;
    math::Quatf inv_q_;
    math::Mat4f qmat_;
};

// Scale — uniform scale.
//
// Math: f(x) = scale · g(x / scale)
//
// Gradient: ∇f(x) = scale · ∇g(x/scale) · (1/scale) = ∇g(x/scale).
//   So |∇f| = |∇g| ≤ lipschitz(g). For a unit-Lipschitz SDF g (e.g.
//   a primitive), this means |∇f| = 1 everywhere — the scaled field is
//   *also* a unit-Lipschitz SDF.
//
// Lipschitz constant: lipschitz(f) = lipschitz(g) (NOT scale · lipschitz(g)).
//   The previous implementation reported `scale · lipschitz(g)`, which is
//   wrong for scale > 1 (over-reports) AND wrong for scale < 1 (under-reports
//   when the child has Lipschitz > 1, but the scaled field is still bounded
//   by lipschitz(g), not by scale · lipschitz(g)).
//
// The gradient and the Lipschitz constant must be coherent: both are
// bounded by lipschitz(g), not by scale · lipschitz(g).
class ScaleSDF final : public SDFNode {
public:
    ScaleSDF(std::unique_ptr<SDFNode> child, float s)
        : child_(std::move(child)), inv_s_(1.0f / s), scale_(s) {
        detail::require_non_null(child_, "ScaleSDF", "child",
            "the wrapper dereferences it on every sample() (undefined "
            "behavior, not an empty field)",
            "construct the child first; an empty SDFBody has no root to "
            "wrap");
        // ADR-0029 D5 — the motivating rejection. The member init above
        // computes 1/s before this check runs; for out-of-class s that
        // is a fleeting inf/NaN inside an object being abandoned — the
        // throw guarantees it never escapes into a query.
        detail::require_positive(s, "ScaleSDF", "scale factor s",
            "s = 0 computes 1/s = inf (NaN fields on every query) and "
            "s < 0 flips the field's sign, making the negative region "
            "the unbounded complement of the mirrored child, which no "
            "bounds() box can describe",
            "scale by s > 0 — reflection is not a supported transform in "
            "this kernel");
    }

    SDFSample sample(const math::Vec3f& p) const noexcept override {
        auto r = child_->sample(p * inv_s_);
        // f(x) = scale · g(x/scale). The value scales linearly.
        r.value *= scale_;
        // ∇f(x) = ∇g(x/scale) — the gradient is inherited unchanged.
        // |∇f| = |∇g| ≤ lipschitz(g) = lipschitz(f), so the Lipschitz
        // invariant is preserved without any modification.
        return r;
    }
    float lipschitz()    const noexcept override { return child_->lipschitz(); }
    math::Bboxf bounds() const noexcept override {
        // ADR-0028 bounds-contract audit: IMMUNE for s > 0 — f is
        // negative exactly where g is (the sign of the positive scale
        // factor never flips it), so the negative region is the
        // child's scaled by s and the box scales identically. s <= 0
        // is REJECTED at construction (ADR-0029 D5): it is outside the
        // supported input class (the field's sign semantics would
        // invert), not a bounds() bug.
        auto b = child_->bounds();
        return {b.min * scale_, b.max * scale_};
    }
    std::string describe() const override {
        return "scale(" + child_->describe() + "," + std::to_string(scale_) + ")";
    }
    std::unique_ptr<SDFNode> clone() const override {
        return std::make_unique<ScaleSDF>(child_->clone(), scale_);
    }

    // Native-format serialization accessors.
    const SDFNode* wrapped() const noexcept { return child_.get(); }
    float scale() const noexcept { return scale_; }

private:
    std::unique_ptr<SDFNode> child_;
    float inv_s_;
    float scale_;
};

// Twist — non-rigid warp along the Y axis. Useful for organic shapes.
//
// The warp rotates each horizontal slice by θ(y) = y·k around Y. It is
// NOT rigid — the rotation angle depends on the height — and the three
// contract methods treat it accordingly (ADR-0029, the twist warp
// contract): bounds() carries the child's axis-cylinder, lipschitz()
// declares the closed-form σ(k, r), and sample() returns the full
// chain-rule gradient including the shear term. The pre-contract code
// treated the warp as rigid on all three fronts: bounds() returned the
// child box unchanged (violated off-axis), lipschitz() declared the
// dimensionally blind 1 + 2|k|, and the gradient omitted the y shear.
class TwistSDF final : public SDFNode {
public:
    TwistSDF(std::unique_ptr<SDFNode> child, float twist_per_unit_y)
        : child_(std::move(child)), k_(twist_per_unit_y) {
        detail::require_non_null(child_, "TwistSDF", "child",
            "the wrapper dereferences it on every sample() (undefined "
            "behavior, not an empty field)",
            "construct the child first; an empty SDFBody has no root to "
            "wrap");
        // ADR-0029 D5: any finite k (k = 0 is the identity warp and
        // stays legal; the bounds() passthrough and the σ(k, r) closed
        // form are both k-blind).
        detail::require_finite(k_, "TwistSDF", "twist rate k",
            "a non-finite twist rate makes every slice's rotation angle "
            "NaN: the field, the gradient, bounds() and lipschitz() all "
            "return NaN",
            "pass a finite radians-per-unit-y rate (k = 0 keeps the "
            "child untouched)");
    }

    SDFSample sample(const math::Vec3f& p) const noexcept override {
        const float ang = p.y * k_;
        const float c = std::cos(ang), s = std::sin(ang);
        // Inverse rotation: rotate point back into child's local frame.
        const math::Vec3f local{
            p.x * c + p.z * s,
            p.y,
            -p.x * s + p.z * c,
        };
        auto r = child_->sample(local);
        // Gradient — the full chain rule (ADR-0029 D2). The inverse map
        // u(p) rotates p's (x, z) by −k·p.y, so its Jacobian carries the
        // shear column ∂u/∂p.y = (k·u.z, 1, −k·u.x): the rotation angle
        // itself depends on p.y, with the point's radius as lever arm.
        // The y-partial therefore gains, on top of the child's g.y, the
        // shear term
        //   ∂f/∂p.y = g.y + k · (u.z·g.x − u.x·g.z),
        // computed HERE in the child's frame (the local point and the
        // child gradient, before the forward xz rotation below), while
        // the x/z partials are the child gradient's xz rotated forward:
        //   ∂f/∂p.x = g.x·c − g.z·s,   ∂f/∂p.z = g.x·s + g.z·c.
        // Returning g.y alone (the pre-ADR-0029 code) is wrong whenever
        // the child gradient has a tangential component at an off-axis
        // point — and the error is invisible to a norm-only property
        // (the xz rotation is an isometry), which is why the regression
        // pin checks the gradient against finite differences
        // component-wise.
        const float gy = r.gradient.y
                       + k_ * (local.z * r.gradient.x - local.x * r.gradient.z);
        math::Vec3f g{
            r.gradient.x * c - r.gradient.z * s,
            gy,
            r.gradient.x * s + r.gradient.z * c,
        };
        r.gradient = g;
        return r;
    }
    float lipschitz() const noexcept override {
        // ADR-0029 D2 — the warp is not a rigid rotation, and the field
        // slope pays for it. For f = g ∘ u with u the slice rotation by
        // −k·p.y, the Jacobian J of u satisfies
        //   JᵀJ:  λ_max = 1 + (t + √(t² + 4t))/2,   t = (k·r)²,
        // with r the evaluation point's distance from the axis, so the
        // honest local factor is the closed form
        //   σ(k, r) = √(1 + (t + √(t² + 4t))/2),   t = (k·r)²,
        // sandwiched — each direction one line, the proof device and
        // the sanity envelope — by
        //   √(1 + (k·r)²)  ≤  σ(k, r)  ≤  1 + |k·r|.
        // σ grows without bound as r → ∞: the twisted field is not
        // globally Lipschitz, so the declaration is regional, over the
        // D1 bounds() box — r_box = √2·R, the corner distance of the
        // new AABB, covers every in-box consumer (the MC grid and its
        // 1%-of-extent pad included). The residual — marching far
        // off-axis, outside the box, where the local σ exceeds the
        // declared value — is inherent to any non-globally-Lipschitz
        // warp and is documented in ADR-0029 D2.
        // Conviction of the old declaration, L_child·(1 + 2|k|): k is
        // radians per unit length, so no function of k alone can bound
        // a factor that depends on the dimensionless k·r — it failed
        // as an upper bound for k·r ≳ 0.2 (r ≳ 4 at k = 0.05; the
        // true slope runs +16% over it at k = 0.05, r = 10), and the
        // old comment's √(1 + (k·R)²) was only the sandwich's LOWER
        // edge: it bounds the y-partial, not the gradient norm (at
        // k·r = 1 it says 1.414 where the true factor is 1.618).
        const float r_box = std::sqrt(2.0f) * cylinder_radius(child_->bounds());
        const float t = (k_ * r_box) * (k_ * r_box);
        const float sigma = std::sqrt(1.0f + 0.5f * (t + std::sqrt(t * t + 4.0f * t)));
        return child_->lipschitz() * sigma;
    }
    // ADR-0029 D1 — the bounds contract (this block was the audit's
    // KNOWN GAP; it is the contract now): the AABB of the cylinder
    // that carries the child's box,
    //   [−R, R] × [y_min, y_max] × [−R, R],
    // with R the maximum distance from the Y axis over the child box's
    // corners. Proof, four steps: (1) sample() computes f = g ∘ u
    // exactly, so f(p) < 0 ⟺ p ∈ φ(neg g), the forward per-slice
    // rotation of the child's negative region — no approximation
    // anywhere in the chain; (2) neg g ⊆ B, the child's own bounds()
    // contract; (3) φ preserves Y (slices map to slices) and radial
    // distance (a slice rotation is an isometry of the (x, z) plane),
    // so φ(B) stays inside the cylinder {y ∈ [y_min, y_max],
    // |p.xz| ≤ R}, with R attained at a box corner because x² + z² is
    // convex on the rectangle; (4) the cylinder's AABB is the box
    // above. Corollaries: MONOTONE — the new box always contains the
    // old one, the fix can only widen a reported box; K-BLIND — the
    // proof uses nothing about θ(y) = y·k beyond its being a rotation,
    // so the bound holds for every finite k, any sign, any magnitude,
    // with no angle-range analysis. The cost, stated plainly (review
    // condition 1): an on-axis child box does NOT almost coincide with
    // its cylinder — R is the cross-section's half-diagonal, so the
    // footprint dilates by √2 on the diagonal (a centered square pays
    // 2× footprint and cell volume at the extractor's fixed cell
    // count); conservativity is provable, monotone, k-blind — and
    // never free. The tighter slice-tight hull is deferred with its
    // entry criterion in ADR-0029 D1.
    math::Bboxf bounds() const noexcept override {
        const math::Bboxf b = child_->bounds();
        // k = 0: the identity warp returns the child's box verbatim.
        if (k_ == 0.0f) return b;
        const float R = cylinder_radius(b);
        return {{-R, b.min.y, -R}, {R, b.max.y, R}};
    }
    std::string describe() const override {
        return "twist(" + child_->describe() + ",k=" + std::to_string(k_) + ")";
    }
    std::unique_ptr<SDFNode> clone() const override {
        return std::make_unique<TwistSDF>(child_->clone(), k_);
    }

    // Native-format serialization accessors.
    const SDFNode* wrapped() const noexcept { return child_.get(); }
    float twist_k() const noexcept { return k_; }

private:
    // R = the maximum distance from the Y axis over the child box's
    // corners: x² + z² is convex on the rectangle and monotone in
    // |x|, |z|, so the max sits at the (max |x|, max |z|) vertex.
    static float cylinder_radius(const math::Bboxf& b) noexcept {
        const float mx = std::max(std::abs(b.min.x), std::abs(b.max.x));
        const float mz = std::max(std::abs(b.min.z), std::abs(b.max.z));
        return std::sqrt(mx * mx + mz * mz);
    }

    std::unique_ptr<SDFNode> child_;
    float k_;
};

// --- Free helpers ----------------------------------------------------------
inline SDFBody sdf_translate(SDFBody body, math::Vec3f offset) {
    return SDFBody(std::make_unique<TranslateSDF>(body.release_root(), offset));
}
inline SDFBody sdf_rotate(SDFBody body, math::Quatf q) {
    return SDFBody(std::make_unique<RotateSDF>(body.release_root(), q));
}
inline SDFBody sdf_scale(SDFBody body, float s) {
    return SDFBody(std::make_unique<ScaleSDF>(body.release_root(), s));
}
inline SDFBody sdf_twist(SDFBody body, float k) {
    return SDFBody(std::make_unique<TwistSDF>(body.release_root(), k));
}

} // namespace CAD_0::sdf

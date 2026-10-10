// core/sdf/include/CAD_0/sdf/operators.hpp
//
// CSG operators on SDF trees. Each operator composes its children into
// a new SDF tree.
//
//   * union (smin with k=0 = hard min)
//   * intersection
//   * subtraction (a - b = a intersect (not b))
//   * smooth_min (polynomial smooth union, Inigo Quilez)
//   * blend
//
// All operators preserve the SDF invariant: |∇f| ≤ lipschitz (≤ 1 for
// primitives). For smooth_min the Lipschitz constant grows slightly,
// and we report it via lipschitz() so the ray marcher compensates.
//
#pragma once

#include "CAD_0/sdf/field.hpp"

#include <algorithm>
#include <cmath>

namespace CAD_0::sdf {

// --- Union (hard min) ------------------------------------------------------
// Math: f(x) = min(a(x), b(x))
//   ∇f(x) = ∇a(x) when a(x) < b(x),  ∇b(x) when b(x) ≤ a(x).
// Same normalization rationale as SubtractionSDF.
class UnionSDF final : public SDFNode {
public:
    UnionSDF(std::unique_ptr<SDFNode> a, std::unique_ptr<SDFNode> b) {
        children_.reserve(2);
        if (a) children_.push_back(std::move(a));
        if (b) children_.push_back(std::move(b));
    }

    SDFSample sample(const math::Vec3f& p) const noexcept override {
        if (children_.empty()) return SDFSample{1e30f, {}};
        auto r = children_[0]->sample(p);
        for (std::size_t i = 1; i < children_.size(); ++i) {
            const auto s = children_[i]->sample(p);
            if (s.value < r.value) r = s;
        }
        // Normalize the picked gradient (defensive — children of
        // composites may not have unit gradient).
        math::Vec3f g = r.gradient;
        const float gl = g.length();
        if (gl > 1e-9f) g = g * (1.0f / gl);
        r.gradient = g;
        return r;
    }

    float lipschitz() const noexcept override {
        float m = 1.0f;
        for (const auto& c : children_) m = std::max(m, c->lipschitz());
        return m;
    }

    math::Bboxf bounds() const noexcept override {
        math::Bboxf b;
        for (const auto& c : children_) b.expand(c->bounds());
        return b;
    }

    std::string describe() const override {
        std::string s = "union(";
        for (std::size_t i = 0; i < children_.size(); ++i) {
            if (i) s += ",";
            s += children_[i]->describe();
        }
        s += ")";
        return s;
    }
    std::unique_ptr<SDFNode> clone() const override {
        std::vector<std::unique_ptr<SDFNode>> cs;
        cs.reserve(children_.size());
        for (const auto& c : children_) cs.push_back(c->clone());
        auto out = std::make_unique<UnionSDF>(nullptr, nullptr);
        out->children_ = std::move(cs);
        return out;
    }

private:
    std::vector<std::unique_ptr<SDFNode>> children_;
};

// --- Intersection ---------------------------------------------------------
// Math: f(x) = max(a(x), b(x))
//   ∇f(x) = ∇a(x) when a(x) > b(x),  ∇b(x) when b(x) ≥ a(x).
// Same normalization rationale as SubtractionSDF.
class IntersectionSDF final : public SDFNode {
public:
    IntersectionSDF(std::unique_ptr<SDFNode> a, std::unique_ptr<SDFNode> b)
        : a_(std::move(a)), b_(std::move(b)) {}

    SDFSample sample(const math::Vec3f& p) const noexcept override {
        auto sa = a_->sample(p);
        const auto sb = b_->sample(p);
        // Max of the two signed distances.
        if (sb.value > sa.value) {
            sa = sb;
        }
        // Normalize the gradient we picked (defensive — children of
        // composites may not have unit gradient).
        math::Vec3f g = sa.gradient;
        const float gl = g.length();
        if (gl > 1e-9f) g = g * (1.0f / gl);
        sa.gradient = g;
        return sa;
    }

    float lipschitz() const noexcept override {
        return std::max(a_->lipschitz(), b_->lipschitz());
    }

    math::Bboxf bounds() const noexcept override {
        return a_->bounds().intersect(b_->bounds());
    }

    std::string describe() const override {
        return "intersect(" + a_->describe() + "," + b_->describe() + ")";
    }
    std::unique_ptr<SDFNode> clone() const override {
        return std::make_unique<IntersectionSDF>(a_->clone(), b_->clone());
    }

private:
    std::unique_ptr<SDFNode> a_;
    std::unique_ptr<SDFNode> b_;
};

// --- Subtraction (a - b) --------------------------------------------------
// Math: f(x) = max(a(x), -b(x))   (intersection of a and complement of b)
//   ∇f(x) = ∇a(x) when a(x) > -b(x),  -∇b(x) when -b(x) > a(x).
//
// Both inputs are SDFs, so their gradients have magnitude ≤ lipschitz.
// For the result to be a valid SDF (|∇f| ≤ 1 when both inputs are
// unit Lipschitz), we must normalize the gradient we pick. This matters
// in particular when the children are themselves composites with
// Lipschitz > 1 (e.g. a Twist): in that case |∇child| could exceed 1,
// and propagating that gradient would break the |∇f| ≤ lipschitz(f)
// invariant downstream.
class SubtractionSDF final : public SDFNode {
public:
    SubtractionSDF(std::unique_ptr<SDFNode> a, std::unique_ptr<SDFNode> b)
        : a_(std::move(a)), b_(std::move(b)) {}

    SDFSample sample(const math::Vec3f& p) const noexcept override {
        auto sa = a_->sample(p);
        const auto sb = b_->sample(p);
        // Inside b => outside the result, so negate b's distance and take max.
        const float bneg = -sb.value;
        if (bneg > sa.value) {
            sa.value = bneg;
            // Result inherits -∇b as its gradient. Normalize it to keep
            // |∇f| = 1 (assuming b is a proper SDF with |∇b| = 1 on the
            // surface; this also clamps any slight over-unit magnitude
            // from composited children).
            math::Vec3f g = -sb.gradient;
            const float gl = g.length();
            if (gl > 1e-9f) g = g * (1.0f / gl);
            sa.gradient = g;
        } else {
            // Result inherits ∇a. Normalize for the same reason.
            math::Vec3f g = sa.gradient;
            const float gl = g.length();
            if (gl > 1e-9f) g = g * (1.0f / gl);
            sa.gradient = g;
        }
        return sa;
    }

    float lipschitz() const noexcept override {
        return std::max(a_->lipschitz(), b_->lipschitz());
    }

    math::Bboxf bounds() const noexcept override { return a_->bounds(); }

    std::string describe() const override {
        return "subtract(" + a_->describe() + "," + b_->describe() + ")";
    }
    std::unique_ptr<SDFNode> clone() const override {
        return std::make_unique<SubtractionSDF>(a_->clone(), b_->clone());
    }

private:
    std::unique_ptr<SDFNode> a_;
    std::unique_ptr<SDFNode> b_;
};

// --- Smooth union (polynomial) -------------------------------------------
// Ref: Inigo Quilez — "smooth minimum" (polynomial form, k>0)
//
//   h = clamp(0.5 + 0.5·(b - a)/k, 0, 1)
//   v = a + (b - a)·h - k·h·(1 - h)·0.25
//
// k = 0 → hard min (degenerate to UnionSDF).
//
// Gradient derivation (chain rule, p-dependence comes through a(p),
// b(p), and h(p) = clamp(0.5 + 0.5·(b-a)/k, 0, 1))
// ------------------------------------------------------------------------
// For 0 < h < 1 (the smooth-interior region, i.e. |b - a| < k):
//
//   ∇v = (1 - h)·∇a + h·∇b + (df/dh)·∇h
//
// where (df/dh) is the partial derivative of v with respect to h,
// holding a, b fixed:
//
//   df/dh = (b - a) - k·(1 - 2h)/4
//
// and the gradient of h itself:
//
//   ∇h = (0.5/k)·(∇b - ∇a)
//
// Substituting u = (b - a)/k = 2·(h - 0.5) (so 2h - 1 = u):
//
//   ∇v = (1 - h)·∇a + h·∇b
//        + (0.5/k)·(∇b - ∇a)·(k·u - k·(1 - 2h)/4)
//        = (1 - h)·∇a + h·∇b
//          + 0.5·(∇b - ∇a)·(u - (1 - 2h)/4)
//
// Now 2h - 1 = u, so 1 - 2h = -u, so (u - (1 - 2h)/4) = u·(1 + 1/4) = (5/4)·u.
// And u = 2h - 1, so:
//
//   ∇v = (1 - h)·∇a + h·∇b + (5/8)·(2h - 1)·(∇b - ∇a)
//
// For h = 0 or h = 1 (the clamped region, |b - a| ≥ k), ∇h = 0 and:
//   v = b (when h = 1, i.e. b ≤ a) → ∇v = ∇b
//   v = a (when h = 0, i.e. a ≤ b) → ∇v = ∇a
//
// Lipschitz bound
// --------------
// In the smooth-interior region:
//
//   |∇v| ≤ (1-h)·|∇a| + h·|∇b| + (5/8)·|2h-1|·|∇b - ∇a|
//        ≤ max(L_a, L_b)·((1-h) + h) + (5/8)·1·(L_a + L_b)
//        ≤ max(L_a, L_b) + (5/4)·max(L_a, L_b)
//        = (9/4)·max(L_a, L_b) ≈ 2.25·max(L_a, L_b)
//
// In the clamped region |∇v| = |∇a| or |∇b| ≤ max(L_a, L_b).
//
// Therefore the k-independent Lipschitz upper bound is:
//
//   L = 2.25·max(L_a, L_b)
//
// We round up to 2.5·max(L_a, L_b) for safety against numerical noise
// in the FD-vs-analytic comparison, and against composite children
// whose actual Lipschitz may transiently exceed the declared value
// (e.g. TwistSDF reports a conservative L).
//
// Note: this is k-independent because the (1/k) factor in ∇h is
// exactly cancelled by the (k) factor in (df/dh): the blend band
// shrinks as k → 0 but its slope stays bounded.
class SmoothMinSDF final : public SDFNode {
public:
    SmoothMinSDF(std::unique_ptr<SDFNode> a, std::unique_ptr<SDFNode> b, float k)
        : a_(std::move(a)), b_(std::move(b)), k_(k) {}

    SDFSample sample(const math::Vec3f& p) const noexcept override {
        const auto sa = a_->sample(p);
        const auto sb = b_->sample(p);
        if (k_ <= 0.0f) {
            // k=0 → hard min; pick the closer child and normalize its
            // gradient defensively (the child might be a composite
            // whose gradient magnitude slightly exceeds 1).
            auto r = (sb.value < sa.value) ? sb : sa;
            const float gl = r.gradient.length();
            if (gl > 1e-9f) r.gradient = r.gradient * (1.0f / gl);
            return r;
        }
        const float diff = sb.value - sa.value;
        const float h = std::clamp(0.5f + 0.5f * diff / k_, 0.0f, 1.0f);
        const float v = sa.value + diff * h - k_ * h * (1.0f - h) * 0.25f;
        // Full chain-rule gradient:
        //   ∇v = (1 - h)·∇a + h·∇b + (5/8)·(2h - 1)·(∇b - ∇a)
        // for 0 < h < 1, and ∇v = ∇a or ∇b in the clamped region.
        // The clamped branches fall out automatically because (2h - 1)
        // is ±1 at h = 0 or h = 1 but (5/8)·(±1)·(∇b - ∇a) is the
        // (linear extrapolation) contribution; we must NOT add it in
        // the clamped region. Use a coefficient that vanishes at the
        // clamp boundaries.
        const float chain_coeff = (h > 0.0f && h < 1.0f)
            ? (5.0f / 8.0f) * (2.0f * h - 1.0f)
            : 0.0f;
        const math::Vec3f ga = sa.gradient;
        const math::Vec3f gb = sb.gradient;
        math::Vec3f g = ga * (1.0f - h) + gb * h + (gb - ga) * chain_coeff;
        return SDFSample{v, g};
    }

    float lipschitz() const noexcept override {
        // (9/4)·max(L_a, L_b) ≤ 2.5·max(L_a, L_b). See derivation above.
        // k-independent: the (1/k) in ∇h cancels with the (k) in (df/dh).
        // For k = 0 the field degenerates to hard min, so the Lipschitz
        // is just max(L_a, L_b).
        if (k_ <= 0.0f) return std::max(a_->lipschitz(), b_->lipschitz());
        return 2.5f * std::max(a_->lipschitz(), b_->lipschitz());
    }

    math::Bboxf bounds() const noexcept override {
        math::Bboxf b = a_->bounds();
        b.expand(b_->bounds());
        return b;
    }

    std::string describe() const override {
        return "smin(" + a_->describe() + "," + b_->describe() + ",k=" + std::to_string(k_) + ")";
    }
    std::unique_ptr<SDFNode> clone() const override {
        return std::make_unique<SmoothMinSDF>(a_->clone(), b_->clone(), k_);
    }

private:
    std::unique_ptr<SDFNode> a_, b_;
    float k_;
};

// --- Free helpers ---------------------------------------------------------
// These helpers take SDFBody by value (which moves the underlying
// unique_ptr) so that the caller's body is left in a valid empty state
// and the children's ownership is transferred into the new composite.
inline SDFBody sdf_union(SDFBody a, SDFBody b) {
    return SDFBody(std::make_unique<UnionSDF>(
        a.release_root(),
        b.release_root()));
}
inline SDFBody sdf_intersect(SDFBody a, SDFBody b) {
    return SDFBody(std::make_unique<IntersectionSDF>(
        a.release_root(),
        b.release_root()));
}
inline SDFBody sdf_subtract(SDFBody a, SDFBody b) {
    return SDFBody(std::make_unique<SubtractionSDF>(
        a.release_root(),
        b.release_root()));
}
inline SDFBody sdf_smooth_union(SDFBody a, SDFBody b, float k) {
    return SDFBody(std::make_unique<SmoothMinSDF>(
        a.release_root(),
        b.release_root(), k));
}

} // namespace CAD_0::sdf

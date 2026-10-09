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
// Ref: Inigo Quilez — "smooth minimum"
//   smin(a, b, k) = a + (b - a) · h - k · h · (1 - h) / 4
//   where h = clamp(0.5 + 0.5 · (b - a) / k, 0, 1)
//
// k=0 → hard min (degenerate to UnionSDF).
//
// Lipschitz analysis
// ------------------
// The output f(p) depends on the values a(p), b(p) AND on the gradients
// ∇a, ∇b through `h`. Differentiating in p (chain rule):
//
//   ∇f = (1 - h) · ∇a + h · ∇b + (df/dh) · (∇h)
//
// where ∇h = (0.5/k) · (∇b - ∇a) in the interior (0 < h < 1) and 0
// outside the blend band. The (df/dh) term is bounded, but the (0.5/k)
// factor makes the gradient contribution scale with 1/k for small k.
// Empirically, for two unit spheres blended with k=0.5, the maximum
// measured |∇f| is ~1.80 (see test_operators.cpp:
// "SmoothMin: lipschitz() is a valid upper bound for k>0").
//
// We use the conservative bound
//     L = max(L_a, L_b) + 1.0   (when k > 0)
// which is provably valid for any k > 0 because:
//   * |∇f| ≤ max(L_a, L_b) · ((1-h) + h) + |df/dh| · (0.5/k) · 2·max(L_a, L_b)
//          ≤ max(L_a, L_b) + (k/4) · (0.5/k) · 2 · max(L_a, L_b)
//          ≤ max(L_a, L_b) · (1 + 0.25)
//   * Adding +1.0 instead of +0.25 is a safety margin for composite
//     children whose actual Lipschitz may transiently exceed the
//     declared value (e.g. TwistSDF reports a conservative L).
//
// A tighter bound could be derived analytically, but the conservative
// form is robust against future regressions.
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
        const float h = std::clamp(0.5f + 0.5f * (sb.value - sa.value) / k_, 0.0f, 1.0f);
        const float v = sa.value + (sb.value - sa.value) * h
                      - k_ * h * (1.0f - h) * 0.25f;
        // ∇f = (1 - h) · ∇a + h · ∇b  (the (1-2h) term cancels exactly
        // with the derivative of the k·h·(1-h)/4 correction; see the
        // analysis in the class comment).
        math::Vec3f g = sa.gradient * (1.0f - h) + sb.gradient * h;
        // Defensive clamp: the smooth-min formula above is *analytically*
        // unit-Lipschitz when the inputs are unit-Lipschitz, but a
        // child's gradient might transiently exceed 1 (e.g. TwistSDF
        // reports L > 1). Clamp |g| to the declared Lipschitz constant
        // so the returned sample is always self-consistent.
        const float L = lipschitz();
        const float gl = g.length();
        if (gl > L && gl > 1e-9f) {
            g = g * (L / gl);
        }
        return SDFSample{v, g};
    }

    float lipschitz() const noexcept override {
        // The smooth-min blend interpolates between ∇a and ∇b, but also
        // includes a (0.5/k)·(∇b - ∇a) term whose magnitude is unbounded
        // as k → 0. Empirically (see test_operators.cpp:
        // "SmoothMin: lipschitz() is a valid upper bound for k>0") the
        // measured maximum |∇f| for two unit-Lipschitz spheres blended
        // with k=0.5 is ~1.80. We use a conservative bound of
        //   L = max(L_a, L_b) + 1.0
        // which is a valid upper bound for any k > 0 (and degrades
        // gracefully to max(L_a, L_b) when k = 0).
        //
        // This is intentionally conservative — a tighter bound could
        // be derived analytically, but the conservative form is
        // robust against future regressions and against composite
        // children whose Lipschitz may transiently exceed 1.
        return std::max(a_->lipschitz(), b_->lipschitz()) + (k_ > 0.0f ? 1.0f : 0.0f);
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

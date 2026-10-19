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
#include "CAD_0/sdf/detail/validate.hpp"

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
        // ADR-0029 D5: the silent null-drop — sdf_union(empty_body, x)
        // used to evaluate as x, losing an operand without a trace — is
        // now the loud error it always should have been.
        detail::require_non_null(a, "UnionSDF", "operand a",
            "the two-operand union used to drop null children silently, "
            "so an empty operand side went missing without a trace",
            "pass two constructed bodies; an empty SDFBody has no root "
            "to union");
        detail::require_non_null(b, "UnionSDF", "operand b",
            "the two-operand union used to drop null children silently, "
            "so an empty operand side went missing without a trace",
            "pass two constructed bodies; an empty SDFBody has no root "
            "to union");
        children_.reserve(2);
        children_.push_back(std::move(a));
        children_.push_back(std::move(b));
    }

    // N-ary construction (used by native-format deserialization to rebuild
    // a flattened union exactly as it was saved; min-union is associative
    // so this is semantically identical to folding pairs). ADR-0029 D5:
    // non-empty and all non-null — the min over an empty list has no
    // field to evaluate and a null child is dereferenced on sample().
    explicit UnionSDF(std::vector<std::unique_ptr<SDFNode>> children)
        : children_(std::move(children)) {
        if (children_.empty())
            detail::throw_invalid("UnionSDF", "children",
                "be a non-empty list of SDF children", "0 children",
                "an empty union has no field to evaluate — the min over "
                "nothing is undefined",
                "pass at least one constructed child; fold pairwise unions "
                "when building incrementally");
        for (std::size_t i = 0; i < children_.size(); ++i)
            detail::require_non_null(children_[i], "UnionSDF",
                "children[" + std::to_string(i) + "]",
                "a null child is dereferenced on sample() (undefined "
                "behavior, not an empty field)",
                "pass constructed children only");
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
        // ADR-0028 bounds-contract audit: IMMUNE, no dilation needed.
        // min over the children is < 0 iff some child field is < 0, so
        // the negative region is exactly the UNION of the children's
        // negative regions — contained in the hull of their (already
        // correct) bounds. The hard min never dips below any child.
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
        return std::make_unique<UnionSDF>(std::move(cs)); // deep copy via N-ary ctor
    }

    // Native-format serialization accessors.
    const std::vector<std::unique_ptr<SDFNode>>& children() const noexcept { return children_; }

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
        : a_(std::move(a)), b_(std::move(b)) {
        // Validate the MEMBERS: the init list above has already moved
        // the parameters, so `a`/`b` here would be moved-from nulls.
        detail::require_non_null(a_, "IntersectionSDF", "operand a",
            "sample() dereferences both operands on every query — a null "
            "operand is undefined behavior, not an empty intersection",
            "pass two constructed bodies");
        detail::require_non_null(b_, "IntersectionSDF", "operand b",
            "sample() dereferences both operands on every query — a null "
            "operand is undefined behavior, not an empty intersection",
            "pass two constructed bodies");
    }

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
        // ADR-0028 bounds-contract audit: IMMUNE, no dilation needed.
        // max(a, b) < 0 iff a < 0 AND b < 0, so the negative region is
        // a subset of EACH child's — contained in the intersection of
        // the children's bounds. The hard max never dips below either
        // child (the smooth-max analogue of the smin dip would need a
        // smooth_max operator, which this kernel does not have).
        return a_->bounds().intersect(b_->bounds());
    }

    std::string describe() const override {
        return "intersect(" + a_->describe() + "," + b_->describe() + ")";
    }
    std::unique_ptr<SDFNode> clone() const override {
        return std::make_unique<IntersectionSDF>(a_->clone(), b_->clone());
    }

    // Native-format serialization accessors.
    const SDFNode* child_a() const noexcept { return a_.get(); }
    const SDFNode* child_b() const noexcept { return b_.get(); }

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
        : a_(std::move(a)), b_(std::move(b)) {
        // Validate the MEMBERS (the init list already moved the params).
        detail::require_non_null(a_, "SubtractionSDF", "operand a",
            "sample() dereferences both operands on every query — a null "
            "operand is undefined behavior, not an empty difference",
            "pass two constructed bodies (place the tool with translate)");
        detail::require_non_null(b_, "SubtractionSDF", "operand b",
            "sample() dereferences both operands on every query — a null "
            "operand is undefined behavior, not an empty difference",
            "pass two constructed bodies (place the tool with translate)");
    }

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

    // ADR-0028 bounds-contract audit: IMMUNE, no dilation needed.
    // max(a, -b) < 0 iff a < 0 and b > 0, so the negative region is a
    // subset of a's negative region — contained in a's bounds alone.
    // (b's bounds are irrelevant: the tool only ever REMOVES material,
    // and a hard max never dips below either operand.)
    math::Bboxf bounds() const noexcept override { return a_->bounds(); }

    std::string describe() const override {
        return "subtract(" + a_->describe() + "," + b_->describe() + ")";
    }
    std::unique_ptr<SDFNode> clone() const override {
        return std::make_unique<SubtractionSDF>(a_->clone(), b_->clone());
    }

    // Native-format serialization accessors.
    const SDFNode* child_a() const noexcept { return a_.get(); }
    const SDFNode* child_b() const noexcept { return b_.get(); }

private:
    std::unique_ptr<SDFNode> a_;
    std::unique_ptr<SDFNode> b_;
};

// --- Smooth union (polynomial) -------------------------------------------
// Ref: Inigo Quilez — "smooth minimum" (polynomial form, k>0)
//
//   h = clamp(0.5 + 0.5·(a - b)/k, 0, 1)
//   v = a + (b - a)·h - k·h·(1 - h)·0.25
//
// k = 0 → hard min (degenerate to UnionSDF).
//
// Bug fix (2026-10-16, found while pinning the ADR-0026 smooth-union
// follow-up tests): the clamp argument was written with (b - a), which
// makes h = 1 when b ≥ a + k — so v picked the LARGER child in the
// clamped regions (a smooth MAX, i.e. an inflated intersection)
// instead of the smaller (min = union), contradicting both the
// derivation below and the k = 0 branch. Symptom: the centre of a
// constituent sphere evaluated OUTSIDE its own "smooth union". The
// sign flip also changes the chain-rule coefficient:
// (5/8)·(2h-1) → (3/8)·(2h-1) (rederived below); the value formula
// and the endpoint cases are unchanged.
//
// Gradient derivation (chain rule, p-dependence comes through a(p),
// b(p), and h(p) = clamp(0.5 + 0.5·(a-b)/k, 0, 1))
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
//   ∇h = (-0.5/k)·(∇b - ∇a)
//
// Substituting u = (b - a)/k = 1 - 2h (so 2h - 1 = -u):
//
//   ∇v = (1 - h)·∇a + h·∇b
//        + (-0.5/k)·(∇b - ∇a)·(k·u - k·(1 - 2h)/4)
//        = (1 - h)·∇a + h·∇b
//          - 0.5·(∇b - ∇a)·(u - (1 - 2h)/4)
//
// Now 1 - 2h = u, so (u - (1 - 2h)/4) = u·(1 - 1/4) = (3/4)·u.
// And u = -(2h - 1), so:
//
//   ∇v = (1 - h)·∇a + h·∇b + (3/8)·(2h - 1)·(∇b - ∇a)
//
// For h = 0 or h = 1 (the clamped region, |b - a| ≥ k), ∇h = 0 and:
//   v = b (when h = 1, i.e. b ≤ a) → ∇v = ∇b
//   v = a (when h = 0, i.e. a ≤ b) → ∇v = ∇a
//
// Lipschitz bound
// --------------
// In the smooth-interior region:
//
//   |∇v| ≤ (1-h)·|∇a| + h·|∇b| + (3/8)·|2h-1|·|∇b - ∇a|
//        ≤ max(L_a, L_b)·((1-h) + h) + (3/8)·1·(L_a + L_b)
//        ≤ max(L_a, L_b) + (3/4)·max(L_a, L_b)
//        = (7/4)·max(L_a, L_b) ≈ 1.75·max(L_a, L_b)
//
// In the clamped region |∇v| = |∇a| or |∇b| ≤ max(L_a, L_b).
//
// Therefore the k-independent Lipschitz upper bound is:
//
//   L = 1.75·max(L_a, L_b)
//
// We round up to 2.5·max(L_a, L_b) for safety against numerical noise
// in the FD-vs-analytic comparison, and against composite children
// whose actual Lipschitz may transiently exceed the declared value.
// [Corrected 2026-10-20, ADR-0029 D2 / consequence 6: this note once
// cited "TwistSDF reports a conservative L" as its example — false in
// the wide-radius regime, where the twist's then-declared 1 + 2|k|
// UNDER-reported the true slope for k·r ≳ 0.2 (r ≳ 4 already at
// k = 0.05; +16% at k = 0.05, r = 10), and this very margin
// (2.5/1.75 = 1.429× absorption) was what silently covered the lie,
// up to r ≈ 19 at k = 0.05. TwistSDF now declares the closed form
// σ(k, √2·R) over its bounds box (ADR-0029); the 2.5× rounding stays
// as headroom, now on no child's example.]
//
// Note: this is k-independent because the (1/k) factor in ∇h is
// exactly cancelled by the (k) factor in (df/dh): the blend band
// shrinks as k → 0 but its slope stays bounded.
class SmoothMinSDF final : public SDFNode {
public:
    SmoothMinSDF(std::unique_ptr<SDFNode> a, std::unique_ptr<SDFNode> b, float k)
        : a_(std::move(a)), b_(std::move(b)), k_(k) {
        // Validate the MEMBERS (the init list already moved the params).
        detail::require_non_null(a_, "SmoothMinSDF", "operand a",
            "sample() dereferences both operands on every query — a null "
            "operand is undefined behavior, not an empty blend",
            "pass two constructed bodies");
        detail::require_non_null(b_, "SmoothMinSDF", "operand b",
            "sample() dereferences both operands on every query — a null "
            "operand is undefined behavior, not an empty blend",
            "pass two constructed bodies");
        // ADR-0029 D5: k >= 0. k = 0 is the documented hard min (the
        // degenerate branch in sample() is deliberate and pinned); k < 0
        // is rejected because the polynomial derivation assumes k > 0 —
        // it used to silently coerce to the hard-min branch, a different
        // operator than requested.
        detail::require_non_negative(k, "SmoothMinSDF", "k",
            "the polynomial blend derivation assumes k > 0: k < 0 silently "
            "coerced to the hard-min branch (a different operator than "
            "requested) and NaN k flows into the polynomial producing "
            "NaN fields",
            "k = 0 is the documented hard min (use it deliberately); "
            "k > 0 is a smooth blend");
    }

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
        // h = 1 when b ≤ a - k (v follows b, the SMALLER child — see
        // the bug-fix note in the class comment); the sign of the
        // clamp argument is what selects min vs max semantics.
        const float h = std::clamp(0.5f - 0.5f * diff / k_, 0.0f, 1.0f);
        const float v = sa.value + diff * h - k_ * h * (1.0f - h) * 0.25f;
        // Full chain-rule gradient:
        //   ∇v = (1 - h)·∇a + h·∇b + (3/8)·(2h - 1)·(∇b - ∇a)
        // for 0 < h < 1, and ∇v = ∇a or ∇b in the clamped region.
        // The clamped branches fall out automatically because (2h - 1)
        // is ±1 at h = 0 or h = 1 but (3/8)·(±1)·(∇b - ∇a) is the
        // (linear extrapolation) contribution; we must NOT add it in
        // the clamped region. Use a coefficient that vanishes at the
        // clamp boundaries.
        const float chain_coeff = (h > 0.0f && h < 1.0f)
            ? (3.0f / 8.0f) * (2.0f * h - 1.0f)
            : 0.0f;
        const math::Vec3f ga = sa.gradient;
        const math::Vec3f gb = sb.gradient;
        math::Vec3f g = ga * (1.0f - h) + gb * h + (gb - ga) * chain_coeff;
        return SDFSample{v, g};
    }

    float lipschitz() const noexcept override {
        // (7/4)·max(L_a, L_b) ≤ 2.5·max(L_a, L_b). See derivation above.
        // k-independent: the (1/k) in ∇h cancels with the (k) in (df/dh).
        // For k = 0 the field degenerates to hard min, so the Lipschitz
        // is just max(L_a, L_b).
        if (k_ <= 0.0f) return std::max(a_->lipschitz(), b_->lipschitz());
        return 2.5f * std::max(a_->lipschitz(), b_->lipschitz());
    }

    math::Bboxf bounds() const noexcept override {
        // ADR-0028: field.hpp declares bounds() as the box of the region
        // where the field can be NEGATIVE. The polynomial blend dips
        // BELOW min(f_a, f_b) by up to k/16 (phi(u) = (8u - 7u^2 - 1)/16,
        // minimum -1/16 at u = 0), so the negative region escapes the
        // children's plain hull wherever the two operand fields stay
        // within k of each other up to the hull boundary (the pinned
        // probe: concentric unit spheres, blend k — zero crossing at
        // exactly 1 + k/16, outside the old [-1,1]^3 hull; the escape
        // exceeds the extractor's 1%-of-extent pad for k >~ 0.32 at
        // extent 2).
        //
        // Fix: dilate the hull by k/8 — 2x the k/16 field dip. The
        // distance conversion is sound because a point p with
        // smin(p) < 0 satisfies min(f_a, f_b)(p) < k/16, hence
        // dist(p, neg-child) <= (k/16)/L_child, and every node in
        // this library keeps L >= 1 (primitives are unit-Lipschitz;
        // min/max/translate/rotate/scale preserve L; smin multiplies
        // by 2.5; twist by σ(k, √2·R) >= 1, the ADR-0029 closed form
        // over its bounds box — the (1 + 2|k|) it replaced was NOT an
        // upper bound in the wide-radius regime, k·r ≳ 0.2) — so the
        // escape is at most k/16 in distance and k/8 covers it with
        // headroom.
        //
        // Composition along nested blends is by construction: the
        // parent takes the hull of the children's ALREADY-dilated
        // boxes and adds its own k/8. The parent's dip applies to the
        // FIELDS (<= k_p/16 below the min of the child fields), never
        // to the boxes, so the child dilation is never re-applied.
        //
        // k = 0 (hard min): no dip below either child, the plain hull
        // is exact — no dilation (UnionSDF-equivalent layout).
        math::Bboxf b = a_->bounds();
        b.expand(b_->bounds());
        if (k_ > 0.0f) {
            const math::Vec3f dil{k_ * 0.125f, k_ * 0.125f, k_ * 0.125f};
            b.min = b.min - dil;
            b.max = b.max + dil;
        }
        return b;
    }

    std::string describe() const override {
        return "smin(" + a_->describe() + "," + b_->describe() + ",k=" + std::to_string(k_) + ")";
    }
    std::unique_ptr<SDFNode> clone() const override {
        return std::make_unique<SmoothMinSDF>(a_->clone(), b_->clone(), k_);
    }

    // Native-format serialization accessors.
    const SDFNode* child_a() const noexcept { return a_.get(); }
    const SDFNode* child_b() const noexcept { return b_.get(); }
    float blend_k() const noexcept { return k_; }

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

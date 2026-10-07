// core/sdf/include/cadforge/sdf/operators.hpp
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

#include "cadforge/sdf/field.hpp"

#include <algorithm>
#include <cmath>

namespace cadforge::sdf {

// --- Union (hard min) ------------------------------------------------------
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
class IntersectionSDF final : public SDFNode {
public:
    IntersectionSDF(std::unique_ptr<SDFNode> a, std::unique_ptr<SDFNode> b)
        : a_(std::move(a)), b_(std::move(b)) {}

    SDFSample sample(const math::Vec3f& p) const noexcept override {
        auto sa = a_->sample(p);
        const auto sb = b_->sample(p);
        // Max of the two signed distances.
        if (sb.value > sa.value) sa = sb;
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
            sa.gradient = -sb.gradient;
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
// smin(a, b, k) = max(a, b) - h*h*k*0.25   where h = clamp(0.5 + 0.5*(b-a)/k, 0, 1)
//
// k=0 => hard min, equivalent to UnionSDF.
class SmoothMinSDF final : public SDFNode {
public:
    SmoothMinSDF(std::unique_ptr<SDFNode> a, std::unique_ptr<SDFNode> b, float k)
        : a_(std::move(a)), b_(std::move(b)), k_(k) {}

    SDFSample sample(const math::Vec3f& p) const noexcept override {
        const auto sa = a_->sample(p);
        const auto sb = b_->sample(p);
        if (k_ <= 0.0f) {
            return (sb.value < sa.value) ? sb : sa;
        }
        const float h = std::clamp(0.5f + 0.5f * (sb.value - sa.value) / k_, 0.0f, 1.0f);
        const float v = sa.value + (sb.value - sa.value) * h
                      - k_ * h * (1.0f - h) * 0.25f;
        // Gradient = mix(b_grad, a_grad, h) — opposite to interpolation side.
        math::Vec3f g = sa.gradient * (1.0f - h) + sb.gradient * h;
        return SDFSample{v, g};
    }

    float lipschitz() const noexcept override {
        // Smooth union has Lipschitz constant 1 + (k_lipschitz_factor).
        // Approximation: max of children Lipschitz, with a small bump for k>0.
        return std::max(a_->lipschitz(), b_->lipschitz()) + (k_ > 0.0f ? 0.25f : 0.0f);
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

} // namespace cadforge::sdf

// core/sdf/include/CAD_0/sdf/transforms.hpp
//
// Spatial transforms applied to an SDF tree. These wrap a child node
// and re-express queries in the child's local coordinate system.
//
#pragma once

#include "CAD_0/sdf/field.hpp"
#include "CAD_0/math/mat.hpp"
#include "CAD_0/math/quat.hpp"

#include <cmath>

namespace CAD_0::sdf {

// Translate — shift the child field by a fixed offset.
class TranslateSDF final : public SDFNode {
public:
    TranslateSDF(std::unique_ptr<SDFNode> child, math::Vec3f offset)
        : child_(std::move(child)), offset_(offset) {}

    SDFSample sample(const math::Vec3f& p) const noexcept override {
        return child_->sample(p - offset_);
    }
    float lipschitz()    const noexcept override { return child_->lipschitz(); }
    math::Bboxf bounds() const noexcept override {
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

private:
    std::unique_ptr<SDFNode> child_;
    math::Vec3f offset_;
};

// Rotate — rotate the child field by a quaternion.
class RotateSDF final : public SDFNode {
public:
    RotateSDF(std::unique_ptr<SDFNode> child, math::Quatf q)
        : child_(std::move(child)), q_(q), inv_q_(q.conjugate()),
          qmat_(q.to_matrix()) {}

    SDFSample sample(const math::Vec3f& p) const noexcept override {
        const math::Vec3f local = inv_q_.rotate(p);
        auto s = child_->sample(local);
        s.gradient = qmat_.transform_dir(s.gradient);
        return s;
    }
    float lipschitz()    const noexcept override { return child_->lipschitz(); }
    math::Bboxf bounds() const noexcept override {
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
        : child_(std::move(child)), inv_s_(1.0f / s), scale_(s) {}

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
        auto b = child_->bounds();
        return {b.min * scale_, b.max * scale_};
    }
    std::string describe() const override {
        return "scale(" + child_->describe() + "," + std::to_string(scale_) + ")";
    }
    std::unique_ptr<SDFNode> clone() const override {
        return std::make_unique<ScaleSDF>(child_->clone(), scale_);
    }

private:
    std::unique_ptr<SDFNode> child_;
    float inv_s_;
    float scale_;
};

// Twist — non-rigid warp along the Y axis. Useful for organic shapes.
class TwistSDF final : public SDFNode {
public:
    TwistSDF(std::unique_ptr<SDFNode> child, float twist_per_unit_y)
        : child_(std::move(child)), k_(twist_per_unit_y) {}

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
        // Gradient must be rotated forward to match the warp.
        math::Vec3f g{
            r.gradient.x * c - r.gradient.z * s,
            r.gradient.y,
            r.gradient.x * s + r.gradient.z * c,
        };
        r.gradient = g;
        return r;
    }
    float lipschitz()    const noexcept override {
        // Twist increases the Lipschitz constant by sqrt(1 + (k*R)^2)
        // where R is the radius of the child's bounds. Conservative bound:
        return child_->lipschitz() * (1.0f + std::abs(k_) * 2.0f);
    }
    math::Bboxf bounds() const noexcept override { return child_->bounds(); }
    std::string describe() const override {
        return "twist(" + child_->describe() + ",k=" + std::to_string(k_) + ")";
    }
    std::unique_ptr<SDFNode> clone() const override {
        return std::make_unique<TwistSDF>(child_->clone(), k_);
    }

private:
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

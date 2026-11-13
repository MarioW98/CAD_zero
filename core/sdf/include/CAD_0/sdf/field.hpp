// core/sdf/include/CAD_0/sdf/field.hpp
//
// SDFBody — the native implicit representation of a solid.
//
// An SDFBody is an immutable tree of nodes. Each node is one of:
//   * Primitive — sphere, box, cylinder, torus, cone, capsule, plane,
//                  slab-z (the extrusion slab)
//   * Operator  — union, intersection, subtraction, smooth_min, blend
//   * Transform — translate, rotate, scale, twist, bend, mirror
//   * Domain    — repeat, symmetry, warp
//
// Evaluation:
//   * Each node implements `evaluate(p) -> float` returning the signed
//     distance (negative inside the solid).
//   * Each node also declares a Lipschitz constant for the field, used
//     by the ray-marcher to size step lengths and by the mesh extractor
//     to bound marching cubes refinement.
//   * Each node implements `sample(p)` (one point, the scalar form) and
//     may override `sample_batch(...)` (many points, the batch form).
//     PERF 3: the batch form is the evaluator's real road — the leaf
//     classes with xsimd kernels (the seven primitives + SlabZ, in
//     core/sdf/src/batch_kernels.cpp) compute `width` points per pass
//     with bit-identical arithmetic to sample() (ADR-0014; pinned per
//     class by test_perf3_evaluator.cpp); the interior classes compose
//     their children's batches; the classes WITHOUT an override (twist,
//     extrude — the register's declared seats) ride the scalar default.
//     The dispatch witness (detail/batch_stats.hpp) counts the points
//     each path serves.
//
// The body is the *root* of an SDF expression tree. The body itself
// is the root node.
//
#pragma once

#include "CAD_0/math/vec.hpp"
#include "CAD_0/math/bbox.hpp"
#include "CAD_0/sdf/detail/batch_stats.hpp"

#include <cstddef>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace CAD_0::sdf {

// Single-point evaluation result. We carry both the value and an
// estimated gradient (used by normal computation in mesh extraction).
struct SDFSample {
    float value{};          // signed distance
    math::Vec3f gradient{}; // ∇field, magnitude ≤ lipschitz
};

// Abstract base for all SDF nodes.
class SDFNode {
public:
    virtual ~SDFNode() = default;

    // Evaluate the field at a single point in *local* coordinates.
    virtual SDFSample sample(const math::Vec3f& p) const noexcept = 0;

    // Evaluate the field at a BATCH of points (AoS in, SoA out: one
    // value per point in `values`, one gradient per point in
    // `gradients`; both arrays are `points.size()` long, both are
    // written entirely).
    //
    // PERF 3 — the batch road. The default is the scalar fallback
    // (one virtual sample() per point; the witness counts every point
    // scalar). A class whose field arithmetic is lane-vectorizable
    // overrides this with an xsimd kernel that MUST produce bits
    // identical to sample() for every point of every input — the
    // per-class bit-identity corpora of test_perf3_evaluator.cpp are
    // the contract's enforcement, ADR-0014 the law. The interior
    // classes (operators, rigid transforms) override it to compose
    // their children's batches with the SAME per-point expressions
    // their sample() uses — bit-identity by construction.
    virtual void sample_batch(std::span<const math::Vec3f> points,
                              float* values,
                              math::Vec3f* gradients) const noexcept {
        for (std::size_t i = 0; i < points.size(); ++i) {
            const SDFSample s = sample(points[i]);
            values[i] = s.value;
            gradients[i] = s.gradient;
        }
        detail::batch_witness().scalar_points.fetch_add(
            points.size(), std::memory_order_relaxed);
    }

    // Lipschitz constant for the field. Used by:
    //   * Ray marchers — step size = |value| / lipschitz
    //   * Mesh extractors — narrow-band width
    //   * CSG operators — composed as max of children's lipschitz.
    virtual float lipschitz() const noexcept = 0;

    // Local-space bounding box of the region where the field can be
    // negative (inside the solid). Used to cull evaluation outside the
    // solid of interest.
    virtual math::Bboxf bounds() const noexcept = 0;

    // Human-readable description for the feature tree.
    virtual std::string describe() const = 0;

    // Polymorphic clone (deep copy of the subtree).
    virtual std::unique_ptr<SDFNode> clone() const = 0;
};

// SDFBody — the public-facing, owning wrapper around an SDF tree.
class SDFBody {
public:
    SDFBody() = default;
    explicit SDFBody(std::unique_ptr<SDFNode> root) noexcept : root_(std::move(root)) {}

    SDFBody(SDFBody&&) noexcept = default;
    SDFBody& operator=(SDFBody&&) noexcept = default;
    SDFBody(const SDFBody& other) : root_(other.root_ ? other.root_->clone() : nullptr) {}
    SDFBody& operator=(const SDFBody& other) {
        if (this != &other) {
            root_ = other.root_ ? other.root_->clone() : nullptr;
        }
        return *this;
    }

    SDFSample sample(const math::Vec3f& p) const noexcept {
        return root_ ? root_->sample(p) : SDFSample{};
    }

    float value(const math::Vec3f& p) const noexcept {
        return sample(p).value;
    }

    float lipschitz() const noexcept {
        return root_ ? root_->lipschitz() : 1.0f;
    }

    math::Bboxf bounds() const noexcept {
        return root_ ? root_->bounds() : math::Bboxf{};
    }

    const SDFNode* root() const noexcept { return root_.get(); }

    // Release ownership of the root node. After this call, the body is
    // in an empty state. Used by helper constructors (e.g. sdf_union) to
    // take ownership without cloning.
    std::unique_ptr<SDFNode> release_root() noexcept { return std::move(root_); }

    std::string describe() const {
        return root_ ? root_->describe() : "<empty SDFBody>";
    }

private:
    std::unique_ptr<SDFNode> root_;
};

} // namespace CAD_0::sdf

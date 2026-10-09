// core/sdf/include/CAD_0/sdf/field.hpp
//
// SDFBody — the native implicit representation of a solid.
//
// An SDFBody is an immutable tree of nodes. Each node is one of:
//   * Primitive — sphere, box, cylinder, torus, cone, capsule, plane
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
//
// The body is the *root* of an SDF expression tree. The body itself
// is the root node.
//
#pragma once

#include "CAD_0/math/vec.hpp"
#include "CAD_0/math/bbox.hpp"

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

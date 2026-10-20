// core/sdf/include/CAD_0/sdf/sketch_extrude.hpp
//
// The extrusion — ADR-0030 D3, MVP unit 1 (third coherent group: the
// ESTRUSIONE). f_solid(x, y, z) = max(f_profile(x, y), slab(z)) — the
// intersection of the infinite profile prism with the extrusion slab.
//
// The validation-table rows (ADR-0030 D3) live here, verbatim:
//
// | Node       | Field               | Exact regime     | Understated            | L | Gradient                     | Bounds               | Construction checks |
// |------------|---------------------|------------------|------------------------|---|------------------------------|----------------------|---------------------|
// | SlabZ      | |z−d/2|−d/2         | everywhere       | —                      | 1 | (0,0,±1)                    | z-range              | d > 0 finite        |
// | ExtrudeSDF | max(profile, slab)  | interior; active faces | horizontal edges ≤ √2 | 1 | winning branch, normalized  | bounds₂D × z-range  | via children        |
//
// Sign and zero-set exact (the ADR): inside exact — near a prism edge
// the max of two negatives is the nearer face's true depth; outside,
// past the horizontal edges (lateral face meeting the cap at 90°), the
// field understates by up to √2 — the same 1/sin(θ/2) derivation with
// θ = π/2 (lie #4 closed: max(profile, slab) is the prism's FIELD —
// conservative, sign-true, not the metric distance near the edges; the
// exact corner-aware prism SDF exists in the literature and is NOT
// used: it costs branches per sample for a benefit only measurement
// would see, and measurement uses the entity formulas).
//
// Gradient (lie #3's convention, 3D edition): (∇f_2D, 0) where the
// profile branch wins, (0, 0, ±1) where the slab wins, winning branch
// + normalization at the seam — the same max-seam semantics already
// declared for IntersectionSDF/SubtractionSDF/BoxSDF. At ties the
// profile branch wins (replace only on strictly greater — the
// IntersectionSDF precedent). Lipschitz: the max of two 1-Lipschitz
// fields is 1-Lipschitz (the two-line proof in sketch_profile.hpp) —
// the ray-marcher and the extractor consume it unchanged.
#pragma once

#include "CAD_0/sdf/field.hpp"
#include "CAD_0/sdf/sketch_profile.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace CAD_0::sdf {

// --- SlabZ ----------------------------------------------------------------
//
// The extrusion slab: slab(z) = |z − dir·d/2| − d/2 for direction +1
// (the solid spans z ∈ [0, d], measured from the sketch plane z = 0),
// mirrored for −1 (z ∈ [−d, 0]). An SDFNode: infinite in x/y, the
// z-range in bounds. Used by ExtrudeSDF; standalone-testable per its
// table row.
class SlabZ final : public SDFNode {
public:
    SlabZ(float distance, int direction)
        : distance_(distance), direction_(direction) {
        if (!std::isfinite(distance) || distance <= 0.0f)
            throw std::invalid_argument(
                "SlabZ: distance must be finite and > 0, got " +
                std::to_string(distance) +
                " — a non-positive or non-finite extrusion distance "
                "degenerates the slab (empty or NaN: no solid to "
                "extrude). Remedy: extrude with a positive distance.");
        if (direction != 1 && direction != -1)
            throw std::invalid_argument(
                "SlabZ: direction must be +1 or -1 (±Z), got " +
                std::to_string(direction) +
                " — the MVP extrudes along the world Z axis only "
                "(ADR-0030 D2: arbitrary work planes are deferred with "
                "their entry criterion). Remedy: pass +1 or -1.");
    }

    SDFSample sample(const math::Vec3f& p) const noexcept override {
        const float zc = center_z();
        const float f = std::abs(p.z - zc) - distance_ * 0.5f;
        // Gradient: (0, 0, sign(z − zc)) — at the mirror plane the +Z
        // branch wins by convention (the field there is at its deepest,
        // −d/2: no surface consumer asks for the normal there).
        const float gz = (p.z >= zc) ? 1.0f : -1.0f;
        return SDFSample{f, math::Vec3f{0.0f, 0.0f, gz}};
    }

    float lipschitz() const noexcept override { return 1.0f; }

    math::Bboxf bounds() const noexcept override {
        math::Bboxf b = math::Bboxf::universe();
        if (direction_ > 0) {
            b.min.z = 0.0f;
            b.max.z = distance_;
        } else {
            b.min.z = -distance_;
            b.max.z = 0.0f;
        }
        return b;
    }

    std::string describe() const override {
        return "slab-z(d=" + std::to_string(distance_) + ", dir=" +
               (direction_ > 0 ? std::string("+Z") : std::string("-Z")) + ")";
    }
    std::unique_ptr<SDFNode> clone() const override {
        return std::make_unique<SlabZ>(*this);
    }

    float distance() const noexcept { return distance_; }
    int direction() const noexcept { return direction_; }
    float center_z() const noexcept {
        return static_cast<float>(direction_) * distance_ * 0.5f;
    }

private:
    float distance_;
    int direction_;
};

// --- ExtrudeSDF -----------------------------------------------------------
//
// The extruded profile as an SDFNode: max(profile(x, y), slab(z)),
// bounds = bounds₂D × z-range, gradient winning-branch, L = 1. The
// construction checks are "via children": the ProfileSDF certificate
// (samples + pairwise + bounded result) runs in the profile's
// constructor, the slab validates its own distance/direction — the
// ADR-0029 D5 family through and through.
class ExtrudeSDF final : public SDFNode {
public:
    ExtrudeSDF(ProfileSDF profile, float distance, int direction)
        : profile_(std::move(profile)), slab_(distance, direction) {}

    // Convenience: build the profile from chains (the certificate runs
    // inside — a crossed or open chain refuses HERE, at construction).
    ExtrudeSDF(std::vector<SketchChain> chains, float distance, int direction)
        : profile_(std::move(chains)), slab_(distance, direction) {}

    SDFSample sample(const math::Vec3f& p) const noexcept override {
        const SDFSample2D s2 = profile_.sample(math::Vec2f{p.x, p.y});
        const float slab = std::abs(p.z - slab_.center_z()) - slab_.distance() * 0.5f;
        if (slab > s2.value) {
            return SDFSample{slab, math::Vec3f{0.0f, 0.0f, (p.z >= slab_.center_z())
                                                                   ? 1.0f : -1.0f}};
        }
        // Profile branch wins (also at ties — the declared convention).
        // The 2D entity gradients are unit, so (g2, 0) is unit in 3D;
        // the defensive normalization matches IntersectionSDF.
        math::Vec3f g{s2.gradient.x, s2.gradient.y, 0.0f};
        const float gl = g.length();
        if (gl > 1e-9f) g = g * (1.0f / gl);
        return SDFSample{s2.value, g};
    }

    float lipschitz() const noexcept override { return 1.0f; }

    math::Bboxf bounds() const noexcept override {
        const Bbox2f& b2 = profile_.bounds();
        math::Bboxf b;
        b.min = math::Vec3f{b2.min.x, b2.min.y, slab_.bounds().min.z};
        b.max = math::Vec3f{b2.max.x, b2.max.y, slab_.bounds().max.z};
        return b;
    }

    std::string describe() const override {
        return "extrude(" + profile_.describe() + ", " + slab_.describe() + ")";
    }
    std::unique_ptr<SDFNode> clone() const override {
        return std::make_unique<ExtrudeSDF>(profile_, slab_.distance(),
                                             slab_.direction());
    }

    const ProfileSDF& profile() const noexcept { return profile_; }
    float distance() const noexcept { return slab_.distance(); }
    int direction() const noexcept { return slab_.direction(); }

private:
    ProfileSDF profile_;
    SlabZ slab_;
};

} // namespace CAD_0::sdf

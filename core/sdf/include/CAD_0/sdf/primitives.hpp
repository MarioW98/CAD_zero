// core/sdf/include/CAD_0/sdf/primitives.hpp
//
// SDF primitive nodes — the leaves of an SDF expression tree.
// All primitives are centered at the origin in local space; use
// Transform nodes (see transforms.hpp) to place them.
//
// References:
//   * Inigo Quilez — https://iquilezles.org/articles/distfunctions/
//   * Siemens NX Implicit Modeling — design philosophy
//
#pragma once

#include "CAD_0/sdf/field.hpp"

namespace CAD_0::sdf {

// Sphere of given radius centered at the origin.
class SphereSDF final : public SDFNode {
public:
    explicit SphereSDF(float radius) noexcept : radius_(radius) {}

    SDFSample sample(const math::Vec3f& p) const noexcept override {
        const math::Vec3f g = p / radius_;
        const float d = p.length() - radius_;
        return SDFSample{d, g.normalized()};
    }
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
    explicit BoxSDF(math::Vec3f extent) noexcept : extent_(extent) {}

    SDFSample sample(const math::Vec3f& p) const noexcept override {
        const math::Vec3f q{std::abs(p.x) - extent_.x,
                            std::abs(p.y) - extent_.y,
                            std::abs(p.z) - extent_.z};
        const math::Vec3f outer{std::max(q.x, 0.0f),
                                std::max(q.y, 0.0f),
                                std::max(q.z, 0.0f)};
        const float d = outer.length() + std::min(std::max({q.x, q.y, q.z}), 0.0f);
        // Gradient = sign(p) * sign(qmax)
        math::Vec3f g{
            (q.x > q.y && q.x > q.z) ? (p.x > 0 ? 1.0f : -1.0f) : 0.0f,
            (q.y > q.x && q.y > q.z) ? (p.y > 0 ? 1.0f : -1.0f) : 0.0f,
            (q.z > q.x && q.z > q.y) ? (p.z > 0 ? 1.0f : -1.0f) : 0.0f,
        };
        return SDFSample{d, g};
    }
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
    CylinderSDF(float radius, float height) noexcept
        : radius_(radius), half_height_(height * 0.5f) {}

    SDFSample sample(const math::Vec3f& p) const noexcept override {
        // 2D cross-section: d = (rho - r, |y| - half_h)
        const float rho = std::sqrt(p.x * p.x + p.z * p.z);
        const float dx = rho - radius_;
        const float dy = std::abs(p.y) - half_height_;

        // Outside distance (positive): only the components that exceed 0.
        const float ox = std::max(dx, 0.0f);
        const float oy = std::max(dy, 0.0f);
        const float d_out = std::sqrt(ox * ox + oy * oy);

        // Inside distance (negative): max of the (negative) signed
        // distances. If we're outside on both axes this is ≤ 0 so the
        // min(d_out, ...) branch picks d_out.
        const float d_in = std::min(std::max(dx, dy), 0.0f);

        const float dist = d_out + d_in;

        // ----- Gradient (normalized) -----
        // The closest surface element is determined by which of dx, dy
        // is the *positive* maximum (i.e. which one we are "exiting"
        // through). If both are negative (inside) we pick the one
        // closest to zero (i.e. the max).
        math::Vec3f g{};
        if (dy >= dx) {
            // Closest to a cap. Determine which one by the sign of p.y.
            g = math::Vec3f{0.0f, (p.y >= 0.0f) ? 1.0f : -1.0f, 0.0f};
        } else {
            // Closest to the lateral surface. Outward normal in 3D is
            // the radial unit vector (x, 0, z)/rho.
            if (rho > 1e-9f) {
                const float inv_rho = 1.0f / rho;
                g = math::Vec3f{p.x * inv_rho, 0.0f, p.z * inv_rho};
            } else {
                // On the axis: the lateral surface can't be the closest
                // (we're inside the cylinder), so this branch shouldn't
                // trigger. Fall back to +Y as a safe default.
                g = math::Vec3f{0.0f, 1.0f, 0.0f};
            }
        }
        return SDFSample{dist, g};
    }
    float lipschitz()      const noexcept override { return 1.0f; }
    math::Bboxf bounds()   const noexcept override {
        return { {-radius_, -half_height_, -radius_}, {radius_, half_height_, radius_} };
    }
    std::string describe() const override {
        return "cylinder(r=" + std::to_string(radius_) + ",h=" + std::to_string(half_height_ * 2.0f) + ")";
    }
    std::unique_ptr<SDFNode> clone() const override { return std::make_unique<CylinderSDF>(*this); }

private:
    float radius_;
    float half_height_;
};

// Torus with major radius R and minor radius r, in the XZ plane.
class TorusSDF final : public SDFNode {
public:
    TorusSDF(float R, float r) noexcept : R_(R), r_(r) {}

    SDFSample sample(const math::Vec3f& p) const noexcept override {
        const math::Vec2f q{std::sqrt(p.x * p.x + p.z * p.z) - R_, p.y};
        const float d = q.length() - r_;
        math::Vec2f g = q;
        const float ql = q.length();
        if (ql > 1e-9f) g = g * (1.0f / ql);
        math::Vec3f g3{p.x, 0.0f, p.z};
        const float g3l = g3.length();
        if (g3l > 1e-9f) g3 = g3 * (1.0f / g3l);
        return SDFSample{d, math::Vec3f{g3.x * g.x, g.y, g3.z * g.x}};
    }
    float lipschitz()      const noexcept override { return 1.0f; }
    math::Bboxf bounds()   const noexcept override {
        const float e = R_ + r_;
        return { {-e, -r_, -e}, {e, r_, e} };
    }
    std::string describe() const override {
        return "torus(R=" + std::to_string(R_) + ",r=" + std::to_string(r_) + ")";
    }
    std::unique_ptr<SDFNode> clone() const override { return std::make_unique<TorusSDF>(*this); }

private:
    float R_;
    float r_;
};

// Cone along +Y, base radius `r` at y = 0, apex at y = h.
//
// This is the closed-form SDF of a *finite* cone (capped base disk +
// lateral surface). The formula is the standard one from Inigo Quilez,
// "distance functions" — finite cone:
//
//   Let θ be the cone half-angle, sin θ = r/L, cos θ = h/L, where
//   L = sqrt(r² + h²) is the slant height. The lateral edge runs from
//   the base rim point q = (r, 0) to the apex q' = (0, h) in the
//   (ρ, y) plane, where ρ = sqrt(x² + z²).
//
//   For a query point q = (ρ, y):
//     w  = q - q_base_rim = (ρ - r, y)
//     // along-edge coordinate (0 at rim, L at apex):
//     dd = dot(w, edge_dir)        with edge_dir = (-r, h)/L
//     // outward (signed) distance to the *infinite* lateral line:
//     //   dot(w, outward_normal)  with outward_normal = (h, r)/L = (cos θ, sin θ)
//     ww = dot(w, outward_normal) = (ρ - r)·cos θ + y·sin θ
//
//   The exact signed distance to the *finite* cone (intersection of the
//   infinite-line half-space "below the lateral" with the half-space
//   y ≥ 0) is then:
//
//       d = max( ww - clamp(dd, 0, L)·0,   // lateral contribution
//                -y )                       // base cap contribution
//
//   Here `ww` already carries the correct sign (negative inside the
//   infinite half-space of the lateral, positive outside), and `max(ww, -y)`
//   is the SDF of the intersection. The clamp(dd, 0, L) term is *not*
//   needed for the distance itself (the SDF max formulation handles it),
//   but it IS needed for the gradient computation, to decide whether
//   the closest point is on the lateral surface, the apex cap, or the
//   base cap.
//
//   The formula above reduces to: d = max(ww, -y). But wait — that's
//   not quite right either. The lateral "infinite-line half-space" treats
//   the line as infinite, so points above the apex (along the line's
//   extension) would still get a "below the line" classification. We
//   need to clamp the lateral contribution to a signed distance only
//   when the foot of the perpendicular lies inside the segment.
//
//   The correct closed form is:
//       d_lateral_signed = (dd < 0) ? +length(w)         // before rim
//                          : (dd > L) ? +length(w)       // past apex
//                          :           ww                 // on segment
//   (We use `+length(w)` in the off-segment cases because there is no
//   "inside" along the off-segment portion — the perpendicular foot is
//   outside, so the closest point is the rim or apex, and we are by
//   definition outside the lateral surface in those regions.)
//
//   Wait — but if the rim is *inside* the solid (i.e. (ρ < r, y = 0)
//   is the base interior, which is inside the cone), then `length(w)`
//   would give a positive distance when it should be negative.
//
//   The cleanest fix is to compute the unsigned distance to the rim and
//   apex, and let the `max(ww, -y)` formulation determine the sign of
//   the lateral contribution. Specifically:
//
//       d_rim   = sqrt((ρ - r)² + y²)            // unsigned
//       d_apex  = sqrt(ρ² + (y - h)²)            // unsigned
//       d_lat   = (dd < 0) ? d_rim :
//                 (dd > L) ? d_apex :
//                            ww                    // signed (negative inside)
//
//   Then the result is:
//       d = max(d_lat, -y)
//
//   This is correct because:
//     * When the perpendicular foot is on the segment, d_lat = ww
//       (signed, negative inside) — this dominates -y for points above
//       the rim.
//     * When the foot is before the rim (dd < 0), the closest point on
//       the lateral surface is the rim itself; d_lat = d_rim (always
//       non-negative). max(d_rim, -y) gives the correct positive
//       distance to the closest surface element (rim or base cap).
//     * When the foot is past the apex (dd > L), the closest point is
//       the apex; d_lat = d_apex (always non-negative). max(d_apex, -y)
//       gives the correct positive distance.
//
//   Gradient: pick the gradient of whichever surface element dominates
//   the max:
//     * if d_lat > -y → lateral: outward normal in (ρ, y) plane is
//       (cos θ, sin θ); in 3D: (cos θ · x/ρ, sin θ, cos θ · z/ρ).
//       (For points off-segment, the gradient points from the closest
//       endpoint toward the query; we approximate by the lateral normal
//       when |ww| > ε and by the radial direction otherwise.)
//     * if -y > d_lat → base cap: outward normal = (0, -1, 0).
//
class ConeSDF final : public SDFNode {
public:
    ConeSDF(float r, float h) noexcept : r_(r), h_(h) {
        const float edge_dx = -r_;
        const float edge_dy =  h_;
        const float edge_len_sq = edge_dx * edge_dx + edge_dy * edge_dy;
        const float edge_len = std::sqrt(edge_len_sq);
        if (edge_len < 1e-9f) {
            sin_theta_ = 0.0f;
            cos_theta_ = 1.0f;
            inv_edge_len_ = 0.0f;
        } else {
            sin_theta_ = r_ / edge_len;
            cos_theta_ = h_ / edge_len;
            inv_edge_len_ = 1.0f / edge_len;
        }
        edge_len_ = edge_len;
    }

    SDFSample sample(const math::Vec3f& p) const noexcept override {
        const float rho = std::sqrt(p.x * p.x + p.z * p.z);
        const float y = p.y;

        // w = (ρ - r, y - 0)
        const float wx = rho - r_;
        const float wy = y;

        // Project on edge direction (-r, h)/L → along-edge coordinate.
        const float dd = (wx * (-r_) + wy * h_) * inv_edge_len_;

        // Signed distance to infinite lateral line (outward positive).
        const float ww = wx * cos_theta_ + wy * sin_theta_;

        // Unsigned distances to rim (r, 0) and apex (0, h).
        const float d_rim  = std::sqrt(wx * wx + wy * wy);
        const float d_apex = std::sqrt(rho * rho + (y - h_) * (y - h_));

        // Lateral contribution.
        // When the perpendicular foot falls outside the segment [rim, apex],
        // we need to decide whether to use the unsigned distance to the
        // endpoint (rim/apex) or the signed distance to the infinite
        // lateral line (ww). The rule:
        //   - If ww < 0 (inside the lateral half-space), use ww (signed,
        //     negative) — the point is "inside" the lateral, and the
        //     finite-segment correction doesn't make it "outside".
        //   - If ww >= 0 (outside the lateral), use the unsigned distance
        //     to the closest endpoint (rim or apex) — the point is
        //     genuinely outside the solid, and the closest surface
        //     element is the endpoint.
        float d_lat;
        if (dd < 0.0f) {
            d_lat = (ww < 0.0f) ? ww : d_rim;
        } else if (dd > edge_len_) {
            d_lat = (ww < 0.0f) ? ww : d_apex;
        } else {
            d_lat = ww;
        }

        // Base cap contribution: signed distance to the y = 0 half-plane
        // (negative inside i.e. y > 0, positive outside i.e. y < 0).
        const float d_base = -y;

        // Final SDF: max of the two contributions (intersection of the
        // two half-spaces "below lateral" and "above base cap").
        const float d = std::max(d_lat, d_base);

        // ----- Gradient (normalized, unit length) -----
        math::Vec3f g{};
        if (d_base >= d_lat) {
            // Closest to base cap → outward normal points down (-Y).
            g = math::Vec3f{0.0f, -1.0f, 0.0f};
        } else if (dd < 0.0f) {
            // Closest to rim → gradient from rim toward query.
            if (d_rim > 1e-9f) {
                const float inv = 1.0f / d_rim;
                const float rho_hat_x = (rho > 1e-9f) ? p.x / rho : 0.0f;
                const float rho_hat_z = (rho > 1e-9f) ? p.z / rho : 0.0f;
                g = math::Vec3f{wx * inv * rho_hat_x,
                                wy * inv,
                                wx * inv * rho_hat_z};
            } else {
                // Exactly on the rim: pick the lateral outward normal
                // (which is also the cap's edge tangent direction).
                g = math::Vec3f{cos_theta_, sin_theta_, 0.0f};
            }
        } else if (dd > edge_len_) {
            // Closest to apex → gradient from apex toward query.
            if (d_apex > 1e-9f) {
                const float inv = 1.0f / d_apex;
                const float rho_hat_x = (rho > 1e-9f) ? p.x / rho : 0.0f;
                const float rho_hat_z = (rho > 1e-9f) ? p.z / rho : 0.0f;
                g = math::Vec3f{rho * inv * rho_hat_x,
                                (y - h_) * inv,
                                rho * inv * rho_hat_z};
            } else {
                // Exactly at the apex: pick +Y (the cone axis direction
                // above the apex, which is "outside").
                g = math::Vec3f{0.0f, 1.0f, 0.0f};
            }
        } else {
            // Closest to lateral surface → outward normal in (ρ, y) plane.
            // 3D: cos_θ · (x, 0, z)/ρ + sin_θ · (0, 1, 0).
            if (rho > 1e-9f) {
                const float inv_rho = 1.0f / rho;
                g = math::Vec3f{
                    cos_theta_ * p.x * inv_rho,
                    sin_theta_,
                    cos_theta_ * p.z * inv_rho
                };
            } else {
                // On the axis: pick +Y as the outward direction (we are
                // either inside the cone or above the apex).
                g = math::Vec3f{0.0f, 1.0f, 0.0f};
            }
        }
        return SDFSample{d, g};
    }
    float lipschitz()      const noexcept override { return 1.0f; }
    math::Bboxf bounds()   const noexcept override {
        return { {-r_, 0.0f, -r_}, {r_, h_, r_} };
    }
    std::string describe() const override {
        return "cone(r=" + std::to_string(r_) + ",h=" + std::to_string(h_) + ")";
    }
    std::unique_ptr<SDFNode> clone() const override { return std::make_unique<ConeSDF>(*this); }

private:
    float r_;
    float h_;
    float sin_theta_;    // r / sqrt(r² + h²)
    float cos_theta_;    // h / sqrt(r² + h²)
    float inv_edge_len_; // 1 / sqrt(r² + h²)
    float edge_len_;     // sqrt(r² + h²)
};

// Capsule (line + radius) from point a to b.
class CapsuleSDF final : public SDFNode {
public:
    CapsuleSDF(math::Vec3f a, math::Vec3f b, float r) noexcept
        : a_(a), b_(b), r_(r) {
        // Precompute the segment direction and length.
        const math::Vec3f d = b_ - a_;
        length_sq_ = d.length_sq();
    }

    SDFSample sample(const math::Vec3f& p) const noexcept override {
        math::Vec3f pa = p - a_;
        const math::Vec3f ba = b_ - a_;
        float t = pa.dot(ba) / std::max(length_sq_, 1e-12f);
        t = std::clamp(t, 0.0f, 1.0f);
        const math::Vec3f closest = a_ + ba * t;
        const math::Vec3f diff = p - closest;
        const float dist = diff.length() - r_;
        return SDFSample{dist, diff.normalized()};
    }
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

private:
    math::Vec3f a_;
    math::Vec3f b_;
    float r_;
    float length_sq_{1.0f};
};

// Infinite plane at y = 0 (normal points +Y). Half-space y > 0 is inside.
class PlaneSDF final : public SDFNode {
public:
    SDFSample sample(const math::Vec3f& p) const noexcept override {
        // Inside (y > 0) → negative distance (so negate y).
        // Outside (y < 0) → positive distance.
        return SDFSample{-p.y, math::Vec3f{0.0f, -1.0f, 0.0f}};
    }
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

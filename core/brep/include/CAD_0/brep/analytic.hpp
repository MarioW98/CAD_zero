// core/brep/include/CAD_0/brep/analytic.hpp
//
// Analytic surfaces: Plane, Cylinder, Cone, Sphere, Torus.
//
// Each surface is parametrized by (u, v) and provides:
//   * evaluate(u, v) → 3D point on the surface
//   * normal(u, v) → unit normal at that point
//   * bounds() → bounding box
//
// THE FRAME RULE (G1 unit 2's convention correction, landed with
// the p-curve writers): every analytic surface's (u, v) frame is
// RIGHT-HANDED around the OUTWARD normal — numerically,
// (∂u × ∂v) · n_outward > 0 — so "CCW in the domain" and "CCW
// around the outward normal in 3D" are the same statement for every
// surface, and a face's material-on-left loop (the pcurve.hpp
// convention the factories author) reads consistently everywhere.
//
// WHY THE CORRECTION WAS FORCED (the first real use of the pinned
// conventions — the code said so): CylinderSurface was born
// LEFT-handed (∂u×∂v pointed INWARD: evaluate(u,v) = (R cos u, v,
// R sin u) has ∂u×∂v = −(radial)), while PlaneSurface (u_axis×v_axis
// = normal, by the constructor's cross products) and SphereSurface
// (∂u×∂v = +R sin v · radial) were right-handed. Writing the
// cylinder's caps and wall all material-on-left then walked the
// shared rim edges in the SAME direction on both sides — breaking
// the radial-edge orientation-XOR (ADR-0022 §7 invariant 2) — and no
// loop rearrangement fixes that with both faces reading their
// regions honestly: the two domains disagree about handedness. The
// correction flips the cylinder's u direction (u = −azimuth: u=0
// stays the seam meridian +X); the existing loop structure of all
// three factories then IS the material-on-left walk, and the top
// cap's loop — which under the old frame was CW around its outward
// normal — becomes CCW like every other factory face. The two
// evaluate() pins that encoded the old handedness are amended in
// test_analytic_surfaces.cpp, which also pins the rule itself as a
// regression guard.
//
// KNOWN LEFTOVERS, registered for the units that will write their
// p-curves (pre-existing, out of G1 unit 2's class — no factory
// produces them today, so no p-curve writer exercises them):
// ConeSurface and TorusSurface are still left-handed. Their frames
// flip the same one-line way the day their first trimming loop is
// authored, with their own discriminating pins — not before (an
// untested flip of surfaces nobody trims would be exactly the
// silent drift the house forbids).
//
// G1 UNIT 3 ADDS the extrude's own cylinder (below): the sketch
// plane is the world XY, the extrusion runs along ±Z, so the arc
// lateral faces live on cylinders whose axis is Z — a different
// placement from the primitive factory's axis-Y CylinderSurface
// (whose goldens and pins stay untouched; G1 does not unify them).
#pragma once

#include "CAD_0/brep/topology.hpp"
#include "CAD_0/math/vec.hpp"
#include "CAD_0/math/mat.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace CAD_0::brep {

// Plane: a flat surface defined by origin + normal.
// Parametrization: u along X-axis, v along Y-axis (in local frame).
class PlaneSurface : public SurfaceGeometry {
public:
    PlaneSurface(math::Vec3f origin, math::Vec3f normal)
        : origin_(origin), normal_(normal.normalized()) {
        // Build local frame: u-axis perpendicular to normal
        if (std::abs(normal_.y) > 0.9f) {
            u_axis_ = normal_.cross(math::Vec3f{1, 0, 0}).normalized();
        } else {
            u_axis_ = normal_.cross(math::Vec3f{0, 1, 0}).normalized();
        }
        v_axis_ = normal_.cross(u_axis_).normalized();
    }

    math::Vec3f evaluate(float u, float v) const override {
        return origin_ + u_axis_ * u + v_axis_ * v;
    }

    math::Vec3f normal(float, float) const override { return normal_; }
    const char* type_name() const override { return "plane"; }
    math::Bboxf bounds() const override { return math::Bboxf::universe(); }

    math::Vec3f origin() const noexcept { return origin_; }
    math::Vec3f normal_vec() const noexcept { return normal_; }
    // The domain frame, for the p-curve writers and the readers: a
    // point's domain coordinates are ((P−origin)·u_axis,
    // (P−origin)·v_axis) — the inverse of evaluate(). Right-handed
    // around the normal by construction (u_axis × v_axis = normal).
    const math::Vec3f& u_axis() const noexcept { return u_axis_; }
    const math::Vec3f& v_axis() const noexcept { return v_axis_; }

private:
    math::Vec3f origin_;
    math::Vec3f normal_;
    math::Vec3f u_axis_;
    math::Vec3f v_axis_;
};

// Cylinder: radius R, axis along Y, centered at origin.
// u = angle [0, 2π), v = height along axis. RIGHT-HANDED around the
// outward normal since G1 unit 2 (see the frame rule in the file
// header): evaluate(u, v) = (R cos u, v, −R sin u) — u runs the
// opposite azimuth from the historical frame, u = 0 is the seam
// meridian (+X side), and ∂u×∂v = +radial.
class CylinderSurface : public SurfaceGeometry {
public:
    CylinderSurface(float radius, float height, math::Vec3f origin = {})
        : radius_(radius), half_height_(height * 0.5f), origin_(origin) {}

    math::Vec3f evaluate(float u, float v) const override {
        const float c = std::cos(u), s = std::sin(u);
        return {origin_.x + radius_ * c,
                origin_.y + v,
                origin_.z - radius_ * s};
    }

    math::Vec3f normal(float u, float) const override {
        return {std::cos(u), 0.0f, -std::sin(u)};
    }

    const char* type_name() const override { return "cylinder"; }
    math::Bboxf bounds() const override {
        return {{-radius_, -half_height_, -radius_},
                { radius_,  half_height_,  radius_}};
    }

    float radius() const noexcept { return radius_; }
    float height() const noexcept { return half_height_ * 2.0f; }
    // The placement origin (G1 unit 4: translate_body re-anchors the
    // analytic surfaces through their own accessors — one table, no
    // second frame copy).
    const math::Vec3f& origin() const noexcept { return origin_; }

private:
    float radius_;
    float half_height_;
    math::Vec3f origin_;
};

// CylinderZ — the EXTRUDE's cylinder (G1 unit 3): axis = world Z
// through (cx, cy) — the sketch plane's own normal, the direction the
// prism grows in. u = the azimuth AROUND THE AXIS signed by u_sign,
// v = the height above z_base (evaluate(u, v) = (cx + R cos(s·u),
// cy + R sin(s·u), z_base + v)); ∂u×∂v = s·radial — right-handed
// around the outward normal BY CONSTRUCTION, both signs:
//
//   * s = +1 (u = the CCW azimuth): the SOLID side — the fillet arcs
//     (hole = false, the disc material) and the solid circle's wall,
//     whose outward normal is +radial;
//   * s = -1 (u = the reversed azimuth): the HOLE side — the notch
//     faces and the hole walls, whose outward normal is -radial (the
//     material lies outside the cylinder). The frame rule forces the
//     flip: (∂u×∂v)·n_outward > 0 admits no other sign, and the
//     wall's material-on-left window walk then runs CCW in (u, v)
//     exactly as the solid's does (the domain rectangle "Δθ × h" of
//     ADR-0032 D6, θ measured in the walk's own azimuth direction).
//
// The seam meridian is u = 0 ≡ u ∓ 2π = the +X direction from the
// axis (the discipline CylinderSurface's u = 0 follows on its own
// frame). The window handedness, the u-sign table and the seam
// placement are pinned by tests/unit/brep/test_g1_unit3_extrude.cpp.
class CylinderZSurface : public SurfaceGeometry {
public:
    CylinderZSurface(math::Vec2f axis_center, float radius, float z_base,
                     int u_sign)
        : center_(axis_center), radius_(radius), z_base_(z_base),
          u_sign_(u_sign == 1 ? 1.0f : -1.0f) {}

    math::Vec3f evaluate(float u, float v) const override {
        const float phi = u_sign_ * u;
        const float c = std::cos(phi), s = std::sin(phi);
        return {center_.x + radius_ * c,
                center_.y + radius_ * s,
                z_base_ + v};
    }

    math::Vec3f normal(float u, float) const override {
        // s·radial(phi): the SAME sign as ∂u×∂v — the outward normal
        // the frame rule pins (+radial on the solid side, −radial on
        // the hole side, where "outward" points INTO the hole, away
        // from the material).
        const float phi = u_sign_ * u;
        return {u_sign_ * std::cos(phi), u_sign_ * std::sin(phi), 0.0f};
    }

    const char* type_name() const override { return "cylinder"; }
    math::Bboxf bounds() const override {
        // The full-cylinder box (the patch trim lives in the loops,
        // not the surface): z unbounded above z_base — the extrude's
        // span is carried by the trimming window [0, h].
        math::Bboxf b = math::Bboxf::universe();
        b.min.x = center_.x - radius_;
        b.max.x = center_.x + radius_;
        b.min.y = center_.y - radius_;
        b.max.y = center_.y + radius_;
        b.min.z = z_base_;
        return b;
    }

    math::Vec2f axis_center() const noexcept { return center_; }
    float radius() const noexcept { return radius_; }
    float z_base() const noexcept { return z_base_; }
    // The u direction's sign (+1 CCW azimuth, -1 reversed) — the
    // reader-facing declaration of the frame-rule flip above.
    float u_sign() const noexcept { return u_sign_; }

private:
    math::Vec2f center_;
    float radius_;
    float z_base_;
    float u_sign_;
};

// Sphere: radius R centered at origin, in a (pole, azimuth) frame.
//
// THE FRAME (G2 unit 1's re-frame rule, ADR-0033 D1.1): the
// parametrization is
//
//   evaluate(u, v) = C + R·[ sin v·(cos u·e₀ + sin u·e₁) + cos v·n̂ ],
//   e₁ = e₀ × n̂,
//
// with n̂ the POLE (v = 0) and e₀ the AZIMUTH reference (u = 0 is the
// meridian through +e₀). The HISTORICAL frame is the default and is
// untouched: n̂ = +Y, e₀ = +X — the (u, v) every factory authoring and
// every pre-G2 golden reads (the constructor without a frame keeps
// it, so nothing moves). The re-frame derives a new (n̂, e₀) from the
// OPERATION's canonical axis (the cut plane's normal, the center
// line, the cylinder's axis) — the frame serves the geometry, never
// the reverse, and the re-framed surface is the SAME mathematical
// sphere (C, R unchanged). Right-handedness HOLDS BY CONSTRUCTION in
// every frame: ∂u×∂v = +R·sin v·radial for any (e₀, n̂) with
// e₁ = e₀×n̂ (the derivation in ADR-0033 D1.1; the frame rule's
// (∂u×∂v)·n_outward > 0 read on the OUTER normal — the radial).
class SphereSurface : public SurfaceGeometry {
public:
    // The historical frame: pole +Y, azimuth +X.
    SphereSurface(float radius, math::Vec3f origin = {})
        : radius_(radius), origin_(origin),
          pole_{0, 1, 0}, azimuth_{1, 0, 0} {
        precompute();
    }

    // The derived frame (the re-frame rule): `pole` is the new v = 0
    // axis, `azimuth` the new u = 0 meridian direction. Perpendicular
    // unit-ish inputs (each normalized; cross-product-derived pairs
    // are unit up to fp); the PERPENDICULARITY is validated loudly —
    // a skewed frame has no sphere parametrization in its span.
    SphereSurface(float radius, math::Vec3f origin,
                  math::Vec3f pole, math::Vec3f azimuth)
        : radius_(radius), origin_(origin),
          pole_(pole.normalized()), azimuth_(azimuth.normalized()) {
        const float dot = pole_.dot(azimuth_);
        if (!std::isfinite(dot) || std::abs(dot) > 1e-6f ||
            !(pole_.length() > 0.0f) || !(azimuth_.length() > 0.0f)) {
            throw std::invalid_argument(
                "SphereSurface: (pole, azimuth) must be non-zero and "
                "perpendicular (the re-frame's derived frame), got "
                "pole·azimuth=" + std::to_string(dot) +
                " — a skewed frame bends the parametrization off the "
                "sphere's meridians and every authored p-curve with it. "
                "Remedy: pass the operation's derived axis and a "
                "perpendicular azimuth reference (the entries' own "
                "derivations).");
        }
        precompute();
    }

    math::Vec3f evaluate(float u, float v) const override {
        const float sv = std::sin(v);
        return origin_ + e0_ * (radius_ * sv * std::cos(u)) +
               e1_ * (radius_ * sv * std::sin(u)) +
               pole_ * (radius_ * std::cos(v));
    }

    math::Vec3f normal(float u, float v) const override {
        const float sv = std::sin(v);
        return e0_ * (sv * std::cos(u)) + e1_ * (sv * std::sin(u)) +
               pole_ * std::cos(v);
    }

    const char* type_name() const override { return "sphere"; }
    math::Bboxf bounds() const override {
        return {{-radius_ + origin_.x, -radius_ + origin_.y, -radius_ + origin_.z},
                { radius_ + origin_.x,  radius_ + origin_.y,  radius_ + origin_.z}};
    }

    float radius() const noexcept { return radius_; }
    // The placement origin (G1 unit 4: translate_body's table).
    const math::Vec3f& origin() const noexcept { return origin_; }
    // The frame (G2 unit 1): the pole n̂ (v = 0) and the azimuth
    // reference e₀ (u = 0) — readable so translate_body and the table
    // can carry the DERIVED frame through without a second copy.
    const math::Vec3f& pole() const noexcept { return pole_; }
    const math::Vec3f& azimuth() const noexcept { return azimuth_; }

private:
    void precompute() {
        e0_ = azimuth_;
        e1_ = e0_.cross(pole_);  // e₁ = e₀ × n̂ (the historical X×Y = Z)
    }

    float radius_;
    math::Vec3f origin_;
    math::Vec3f pole_;
    math::Vec3f azimuth_;
    math::Vec3f e0_;
    math::Vec3f e1_;
};

// Cone: base radius r at y=0, apex at y=h, centered at origin.
// u = angle [0, 2π), v = height [0, h].
class ConeSurface : public SurfaceGeometry {
public:
    ConeSurface(float base_radius, float height, math::Vec3f origin = {})
        : base_radius_(base_radius), height_(height), origin_(origin) {}

    math::Vec3f evaluate(float u, float v) const override {
        const float r = base_radius_ * (1.0f - v / height_);
        return {origin_.x + r * std::cos(u),
                origin_.y + v,
                origin_.z + r * std::sin(u)};
    }

    math::Vec3f normal(float u, float v) const override {
        // Normal points outward; slope = base_radius / height
        const float slope = base_radius_ / height_;
        const float c = std::cos(u), s = std::sin(u);
        const float n = 1.0f / std::sqrt(slope * slope + 1.0f);
        return {c * n, slope * n, s * n};
    }

    const char* type_name() const override { return "cone"; }
    math::Bboxf bounds() const override {
        return {{-base_radius_, 0, -base_radius_},
                { base_radius_, height_, base_radius_}};
    }

    float base_radius() const noexcept { return base_radius_; }
    float height() const noexcept { return height_; }
    // The placement origin (G1 unit 4: translate_body's table).
    const math::Vec3f& origin() const noexcept { return origin_; }

private:
    float base_radius_;
    float height_;
    math::Vec3f origin_;
};

// Torus: major radius R (ring), minor radius r (tube), in XZ plane.
// u = angle around ring [0, 2π), v = angle around tube [0, 2π).
class TorusSurface : public SurfaceGeometry {
public:
    TorusSurface(float R, float r, math::Vec3f origin = {})
        : R_(R), r_(r), origin_(origin) {}

    math::Vec3f evaluate(float u, float v) const override {
        const float cu = std::cos(u), su = std::sin(u);
        const float cv = std::cos(v), sv = std::sin(v);
        return {origin_.x + (R_ + r_ * cv) * cu,
                origin_.y + r_ * sv,
                origin_.z + (R_ + r_ * cv) * su};
    }

    math::Vec3f normal(float u, float v) const override {
        const float cv = std::cos(v), sv = std::sin(v);
        const float cu = std::cos(u), su = std::sin(u);
        return {cv * cu, sv, cv * su};
    }

    const char* type_name() const override { return "torus"; }
    math::Bboxf bounds() const override {
        const float e = R_ + r_;
        return {{-e + origin_.x, -r_ + origin_.y, -e + origin_.z},
                { e + origin_.x,  r_ + origin_.y,  e + origin_.z}};
    }

    float major_radius() const noexcept { return R_; }
    float minor_radius() const noexcept { return r_; }
    // The placement origin (G1 unit 4: translate_body's table).
    const math::Vec3f& origin() const noexcept { return origin_; }

private:
    float R_, r_;
    math::Vec3f origin_;
};

} // namespace CAD_0::brep

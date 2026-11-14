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
// THE LEFTOVER PAIR FLIPPED (2026-11-04, G2-u3 m3, ADR-0039 D1.2 —
// the promise kept on its own day): the torus's first trimming loop
// is authored in this unit (the fillet's toroidal flank), and the
// registered left-handed pair — TorusSurface and ConeSurface — turned
// right-handed in the same act, the same one-line u = -azimuth flip
// the cylinder received, WITH THEIR DISCRIMINATING PINS (the frame
// rule pointwise, tests/unit/brep/test_analytic_surfaces.cpp: the
// guard that FAILS on the left-handed frame). The cone RIDES with
// the family (the register's economy: the correction made ONCE, the
// pins on both — no cone writer exists yet; its first trimming loop
// remains the gate its own unit will pay). The family's frames are
// ALL right-handed now; the record lives in the m3 register and in
// ADR-0032's amendment, pointing here.
//
// H1 (ADR-0034): every surface's parameters, arithmetic and answers
// are DOUBLE — the exact path's domain. FLOAT appears only at the
// named one-direction boundaries (the p-curve vocabulary, the mesh).
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
    PlaneSurface(math::Vec3d origin, math::Vec3d normal)
        : origin_(origin), normal_(normal.normalized()) {
        // Build local frame: u-axis perpendicular to normal
        if (std::abs(normal_.y) > 0.9) {
            u_axis_ = normal_.cross(math::Vec3d{1, 0, 0}).normalized();
        } else {
            u_axis_ = normal_.cross(math::Vec3d{0, 1, 0}).normalized();
        }
        v_axis_ = normal_.cross(u_axis_).normalized();
    }

    math::Vec3d evaluate(double u, double v) const override {
        return origin_ + u_axis_ * u + v_axis_ * v;
    }

    math::Vec3d normal(double, double) const override { return normal_; }
    SurfaceFamily family() const override { return SurfaceFamily::Plane; }
    void accept(SurfaceVisitor& v) const override { v.visit(*this); }
    math::Bboxd bounds() const override { return math::Bboxd::universe(); }

    math::Vec3d origin() const noexcept { return origin_; }
    math::Vec3d normal_vec() const noexcept { return normal_; }
    // The domain frame, for the p-curve writers and the readers: a
    // point's domain coordinates are ((P−origin)·u_axis,
    // (P−origin)·v_axis) — the inverse of evaluate(). Right-handed
    // around the normal by construction (u_axis × v_axis = normal).
    const math::Vec3d& u_axis() const noexcept { return u_axis_; }
    const math::Vec3d& v_axis() const noexcept { return v_axis_; }

private:
    math::Vec3d origin_;
    math::Vec3d normal_;
    math::Vec3d u_axis_;
    math::Vec3d v_axis_;
};

// Cylinder: radius R, axis along Y, centered at origin.
// u = angle [0, 2π), v = height along axis. RIGHT-HANDED around the
// outward normal since G1 unit 2 (see the frame rule in the file
// header): evaluate(u, v) = (R cos u, v, −R sin u) — u runs the
// opposite azimuth from the historical frame, u = 0 is the seam
// meridian (+X side), and ∂u×∂v = +radial.
class CylinderSurface : public SurfaceGeometry {
public:
    CylinderSurface(double radius, double height, math::Vec3d origin = {})
        : radius_(radius), half_height_(height * 0.5), origin_(origin) {}

    math::Vec3d evaluate(double u, double v) const override {
        const double c = std::cos(u), s = std::sin(u);
        return {origin_.x + radius_ * c,
                origin_.y + v,
                origin_.z - radius_ * s};
    }

    math::Vec3d normal(double u, double) const override {
        return {std::cos(u), 0.0, -std::sin(u)};
    }

    SurfaceFamily family() const override { return SurfaceFamily::Cylinder; }
    void accept(SurfaceVisitor& v) const override { v.visit(*this); }
    math::Bboxd bounds() const override {
        return {{-radius_, -half_height_, -radius_},
                { radius_,  half_height_,  radius_}};
    }

    double radius() const noexcept { return radius_; }
    double height() const noexcept { return half_height_ * 2.0; }
    // The placement origin (G1 unit 4: translate_body re-anchors the
    // analytic surfaces through their own accessors — one table, no
    // second frame copy).
    const math::Vec3d& origin() const noexcept { return origin_; }

private:
    double radius_;
    double half_height_;
    math::Vec3d origin_;
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
    CylinderZSurface(math::Vec2d axis_center, double radius, double z_base,
                     int u_sign)
        : center_(axis_center), radius_(radius), z_base_(z_base),
          u_sign_(u_sign == 1 ? 1.0 : -1.0) {}

    math::Vec3d evaluate(double u, double v) const override {
        const double phi = u_sign_ * u;
        const double c = std::cos(phi), s = std::sin(phi);
        return {center_.x + radius_ * c,
                center_.y + radius_ * s,
                z_base_ + v};
    }

    math::Vec3d normal(double u, double) const override {
        // s·radial(phi): the SAME sign as ∂u×∂v — the outward normal
        // the frame rule pins (+radial on the solid side, −radial on
        // the hole side, where "outward" points INTO the hole, away
        // from the material).
        const double phi = u_sign_ * u;
        return {u_sign_ * std::cos(phi), u_sign_ * std::sin(phi), 0.0};
    }

    SurfaceFamily family() const override { return SurfaceFamily::Cylinder; }
    void accept(SurfaceVisitor& v) const override { v.visit(*this); }
    math::Bboxd bounds() const override {
        // The full-cylinder box (the patch trim lives in the loops,
        // not the surface): z unbounded above z_base — the extrude's
        // span is carried by the trimming window [0, h].
        math::Bboxd b = math::Bboxd::universe();
        b.min.x = center_.x - radius_;
        b.max.x = center_.x + radius_;
        b.min.y = center_.y - radius_;
        b.max.y = center_.y + radius_;
        b.min.z = z_base_;
        return b;
    }

    math::Vec2d axis_center() const noexcept { return center_; }
    double radius() const noexcept { return radius_; }
    double z_base() const noexcept { return z_base_; }
    // The u direction's sign (+1 CCW azimuth, -1 reversed) — the
    // reader-facing declaration of the frame-rule flip above.
    double u_sign() const noexcept { return u_sign_; }

private:
    math::Vec2d center_;
    double radius_;
    double z_base_;
    double u_sign_;
};

// CylinderX / CylinderY — the general fillet's RING FLANKS (G5 m1,
// ADR-0043 D2.1): the cylinder family's X and Y placements, the same
// mathematics one axis under CylinderZSurface. The ring edges of the
// cube (the horizontal 90° joints between two planes) roll the same
// inscribed ball — the center on the bisector at r·cot(π/4) = r from
// the edge — and the swept envelope is a cylinder whose axis is
// PARALLEL TO THE EDGE: world X or Y, the two classes below (the
// TorusZ precedent: the family class CylinderSurface stays the
// factory's own, the writer classes anchor at the flank's frame).
//
// THE FRAME (the ADR's own derivation, one axis under the Z
// discipline): u = the azimuth AROUND the axis measured from the
// DECLARED reference ê₀ (the seam meridian u = 0, frozen by the
// goldens: +Z for CylinderX, +X for CylinderY — the factories'
// reference planes), v = the height along the axis above the base.
// The second azimuth axis is DERIVED IN THE CONSTRUCTOR from the
// product, ê₁ = â × ê₀:
//
//   evaluate(u, v) = P₀ + R·(cos u·ê₀ + sin u·ê₁) + v·â,
//   ∂u×∂v = +R·(cos u·ê₀ + sin u·ê₁) = +R·radial
//
// — right-handed around the outward normal BY CONSTRUCTION for any
// (â, ê₀) with ê₁ = â × ê₀ (the (ê₀, ê₁, â) triplet: ê₀×ê₁ = â), the
// SAME sign as the returned normal, and the chirality is NOT A KNOB:
// a left-handed X/Y flank is unrepresentable in the class (the
// linear-plane law, one family over — the counterproof in
// test_g5_m1_cylinder_xy.cpp pins the discrimination). No u_sign
// parameter exists: the flank's outward normal is +radial (the
// material inside the eroded body, the wedge removed outside), the
// hole-side flip has no consumer here.
//
// THE SEAM: the slit-rectangle pattern of ADR-0023 exactly as the Z
// placement's — u = 0 ≡ u ∓ 2π the declared reference meridian, the
// seam-wired window walks material-on-left in (u, v).
class CylinderXSurface : public SurfaceGeometry {
public:
    CylinderXSurface(double radius, double x_base, double axis_y,
                     double axis_z)
        : radius_(radius), x_base_(x_base), axis_y_(axis_y),
          axis_z_(axis_z) {
        if (!(radius_ > 0.0) || !std::isfinite(radius_)) {
            throw std::invalid_argument(
                "CylinderXSurface: the radius must be finite and "
                "positive, got " +
                std::to_string(radius_) +
                " — a non-positive cylinder has no flank to sweep "
                "(the rolling ball's own radius, ADR-0043 D2.1). "
                "Remedy: construct with the fillet's positive "
                "radius.");
        }
        // The second azimuth axis DERIVED from the product — the
        // right-handedness's source, never a hand-written constant.
        e1_ = a_.cross(e0_);
    }

    math::Vec3d evaluate(double u, double v) const override {
        const double c = std::cos(u), s = std::sin(u);
        return {x_base_ + v,
                axis_y_ + radius_ * (c * e0_.y + s * e1_.y),
                axis_z_ + radius_ * (c * e0_.z + s * e1_.z)};
    }
    math::Vec3d normal(double u, double) const override {
        // +radial(u): the SAME direction ∂u×∂v points (the frame
        // rule's pin — the flank's outward normal, the material
        // inside the eroded body).
        const double c = std::cos(u), s = std::sin(u);
        return {0.0, c * e0_.y + s * e1_.y, c * e0_.z + s * e1_.z};
    }

    SurfaceFamily family() const override { return SurfaceFamily::Cylinder; }
    void accept(SurfaceVisitor& v) const override { v.visit(*this); }
    math::Bboxd bounds() const override {
        // The full-cylinder box (the patch trim lives in the loops,
        // not the surface): x unbounded above x_base — the flank's
        // span is carried by the trimming window, the Z placement's
        // own discipline one axis under.
        math::Bboxd b = math::Bboxd::universe();
        b.min.x = x_base_;
        b.min.y = axis_y_ - radius_;
        b.max.y = axis_y_ + radius_;
        b.min.z = axis_z_ - radius_;
        b.max.z = axis_z_ + radius_;
        return b;
    }

    double radius() const noexcept { return radius_; }
    // The placement accessors (G1 unit 4's table: translate_body
    // re-anchors through the family's own accessors).
    double x_base() const noexcept { return x_base_; }
    double axis_y() const noexcept { return axis_y_; }
    double axis_z() const noexcept { return axis_z_; }

private:
    double radius_;
    double x_base_;
    double axis_y_;
    double axis_z_;
    // The frame, derived once: â = +x̂, ê₀ = +ẑ (the seam meridian,
    // frozen by the goldens), ê₁ = â × ê₀ = −ŷ — the constructor's
    // own cross product, the right-handedness's source.
    math::Vec3d a_{1.0, 0.0, 0.0};
    math::Vec3d e0_{0.0, 0.0, 1.0};
    math::Vec3d e1_{0.0, -1.0, 0.0};
};

class CylinderYSurface : public SurfaceGeometry {
public:
    CylinderYSurface(double radius, double y_base, double axis_x,
                     double axis_z)
        : radius_(radius), y_base_(y_base), axis_x_(axis_x),
          axis_z_(axis_z) {
        if (!(radius_ > 0.0) || !std::isfinite(radius_)) {
            throw std::invalid_argument(
                "CylinderYSurface: the radius must be finite and "
                "positive, got " +
                std::to_string(radius_) +
                " — a non-positive cylinder has no flank to sweep "
                "(the rolling ball's own radius, ADR-0043 D2.1). "
                "Remedy: construct with the fillet's positive "
                "radius.");
        }
        e1_ = a_.cross(e0_);
    }

    math::Vec3d evaluate(double u, double v) const override {
        const double c = std::cos(u), s = std::sin(u);
        return {axis_x_ + radius_ * (c * e0_.x + s * e1_.x),
                y_base_ + v,
                axis_z_ + radius_ * (c * e0_.z + s * e1_.z)};
    }
    math::Vec3d normal(double u, double) const override {
        const double c = std::cos(u), s = std::sin(u);
        return {c * e0_.x + s * e1_.x, 0.0, c * e0_.z + s * e1_.z};
    }

    SurfaceFamily family() const override { return SurfaceFamily::Cylinder; }
    void accept(SurfaceVisitor& v) const override { v.visit(*this); }
    math::Bboxd bounds() const override {
        math::Bboxd b = math::Bboxd::universe();
        b.min.y = y_base_;
        b.min.x = axis_x_ - radius_;
        b.max.x = axis_x_ + radius_;
        b.min.z = axis_z_ - radius_;
        b.max.z = axis_z_ + radius_;
        return b;
    }

    double radius() const noexcept { return radius_; }
    double y_base() const noexcept { return y_base_; }
    double axis_x() const noexcept { return axis_x_; }
    double axis_z() const noexcept { return axis_z_; }

private:
    double radius_;
    double y_base_;
    double axis_x_;
    double axis_z_;
    // The frame, derived once: â = +ŷ, ê₀ = +x̂ (the seam meridian,
    // frozen by the goldens), ê₁ = â × ê₀ = −ẑ.
    math::Vec3d a_{0.0, 1.0, 0.0};
    math::Vec3d e0_{1.0, 0.0, 0.0};
    math::Vec3d e1_{0.0, 0.0, -1.0};
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
    SphereSurface(double radius, math::Vec3d origin = {})
        : radius_(radius), origin_(origin),
          pole_{0, 1, 0}, azimuth_{1, 0, 0} {
        precompute();
    }

    // The derived frame (the re-frame rule): `pole` is the new v = 0
    // axis, `azimuth` the new u = 0 meridian direction. Perpendicular
    // unit-ish inputs (each normalized; cross-product-derived pairs
    // are unit up to fp); the PERPENDICULARITY is validated loudly —
    // a skewed frame has no sphere parametrization in its span.
    SphereSurface(double radius, math::Vec3d origin,
                  math::Vec3d pole, math::Vec3d azimuth)
        : radius_(radius), origin_(origin),
          pole_(pole.normalized()), azimuth_(azimuth.normalized()) {
        const double dot = pole_.dot(azimuth_);
        if (!std::isfinite(dot) || std::abs(dot) > 1e-6 ||
            !(pole_.length() > 0.0) || !(azimuth_.length() > 0.0)) {
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

    math::Vec3d evaluate(double u, double v) const override {
        const double sv = std::sin(v);
        return origin_ + e0_ * (radius_ * sv * std::cos(u)) +
               e1_ * (radius_ * sv * std::sin(u)) +
               pole_ * (radius_ * std::cos(v));
    }

    math::Vec3d normal(double u, double v) const override {
        const double sv = std::sin(v);
        return e0_ * (sv * std::cos(u)) + e1_ * (sv * std::sin(u)) +
               pole_ * std::cos(v);
    }

    SurfaceFamily family() const override { return SurfaceFamily::Sphere; }
    void accept(SurfaceVisitor& v) const override { v.visit(*this); }
    math::Bboxd bounds() const override {
        return {{-radius_ + origin_.x, -radius_ + origin_.y, -radius_ + origin_.z},
                { radius_ + origin_.x,  radius_ + origin_.y,  radius_ + origin_.z}};
    }

    double radius() const noexcept { return radius_; }
    // The placement origin (G1 unit 4: translate_body's table).
    const math::Vec3d& origin() const noexcept { return origin_; }
    // The frame (G2 unit 1): the pole n̂ (v = 0) and the azimuth
    // reference e₀ (u = 0) — readable so translate_body and the table
    // can carry the DERIVED frame through without a second copy.
    const math::Vec3d& pole() const noexcept { return pole_; }
    const math::Vec3d& azimuth() const noexcept { return azimuth_; }

private:
    void precompute() {
        e0_ = azimuth_;
        e1_ = e0_.cross(pole_);  // e₁ = e₀ × n̂ (the historical X×Y = Z)
    }

    double radius_;
    math::Vec3d origin_;
    math::Vec3d pole_;
    math::Vec3d azimuth_;
    math::Vec3d e0_;
    math::Vec3d e1_;
};

// Cone: base radius r at y=0, apex at y=h, centered at origin.
// u = angle [0, 2π), v = height [0, h].
class ConeSurface : public SurfaceGeometry {
public:
    ConeSurface(double base_radius, double height, math::Vec3d origin = {})
        : base_radius_(base_radius), height_(height), origin_(origin) {}

    math::Vec3d evaluate(double u, double v) const override {
        // The one-line flip (G2-u3 m3, ADR-0039 D1.2): u = -azimuth,
        // the third component negated — du x dv turns from the
        // left-handed frame to the right-handed one, like every
        // analytic surface.
        const double r = base_radius_ * (1.0 - v / height_);
        return {origin_.x + r * std::cos(u),
                origin_.y + v,
                origin_.z - r * std::sin(u)};
    }

    math::Vec3d normal(double u, double v) const override {
        // Normal points outward; slope = base_radius / height
        const double slope = base_radius_ / height_;
        const double c = std::cos(u), s = std::sin(u);
        const double n = 1.0 / std::sqrt(slope * slope + 1.0);
        return {c * n, slope * n, -s * n};
    }

    SurfaceFamily family() const override { return SurfaceFamily::Cone; }
    void accept(SurfaceVisitor& v) const override { v.visit(*this); }
    math::Bboxd bounds() const override {
        return {{-base_radius_, 0, -base_radius_},
                { base_radius_, height_, base_radius_}};
    }

    double base_radius() const noexcept { return base_radius_; }
    double height() const noexcept { return height_; }
    // The placement origin (G1 unit 4: translate_body's table).
    const math::Vec3d& origin() const noexcept { return origin_; }

private:
    double base_radius_;
    double height_;
    math::Vec3d origin_;
};

// Torus: major radius R (ring), minor radius r (tube), in XZ plane.
// u = angle around ring [0, 2π), v = angle around tube [0, 2π).
class TorusSurface : public SurfaceGeometry {
public:
    TorusSurface(double R, double r, math::Vec3d origin = {})
        : R_(R), r_(r), origin_(origin) {}

    math::Vec3d evaluate(double u, double v) const override {
        // The one-line flip (G2-u3 m3, ADR-0039 D1.2): u = -azimuth,
        // the third component negated — du x dv = +(R + r cos v)
        // . r . n_out (left-handed before, verified; the tube's outer
        // radial, the CylinderZSurface precedent's own one-liner).
        const double cu = std::cos(u), su = std::sin(u);
        const double cv = std::cos(v), sv = std::sin(v);
        return {origin_.x + (R_ + r_ * cv) * cu,
                origin_.y + r_ * sv,
                origin_.z - (R_ + r_ * cv) * su};
    }

    math::Vec3d normal(double u, double v) const override {
        const double cv = std::cos(v), sv = std::sin(v);
        const double cu = std::cos(u), su = std::sin(u);
        return {cv * cu, sv, -cv * su};
    }

    SurfaceFamily family() const override { return SurfaceFamily::Torus; }
    void accept(SurfaceVisitor& v) const override { v.visit(*this); }
    math::Bboxd bounds() const override {
        const double e = R_ + r_;
        return {{-e + origin_.x, -r_ + origin_.y, -e + origin_.z},
                { e + origin_.x,  r_ + origin_.y,  e + origin_.z}};
    }

    double major_radius() const noexcept { return R_; }
    double minor_radius() const noexcept { return r_; }
    // The placement origin (G1 unit 4: translate_body's table).
    const math::Vec3d& origin() const noexcept { return origin_; }

private:
    double R_, r_;
    math::Vec3d origin_;
};

// TorusZ — THE FILLET'S OWN WRITER (G2-u3 m3, ADR-0039 D1.2): the
// Z-axis torus, the CylinderZSurface precedent read one family over.
// The tube's center circle rides the ring at (cx, cy) around the
// world Z axis at quota z_c; u = the azimuth (u = 0 the seam
// meridian +X, the ADR-0023 discipline the wall's own seam follows),
// v = the meridian angle (v = 0 the OUTER equator). The frame is
// RIGHT-HANDED FROM BIRTH:
//
//   evaluate(u, v) = ((R + r cos v) cos u + cx,
//                     (R + r cos v) sin u + cy,
//                      z_c + r sin v),
//   du x dv = (R + r cos v) . r . n_out
//
// with n_out the tube's OUTER radial (cos v cos u, cos v sin u,
// sin v) — verified pointwise over the flank's window by the m3
// tests. The fillet's flank is the QUARTER patch v in [pi/2, pi]
// (v = pi/2 the tube's top — the plane contact; v = pi the inner
// equator — the wall contact), u the full revolution: the periodic
// seam rides the trimming loop TWICE (the wall's own pattern,
// ADR-0023). A future carried torus with a flipped material side
// reopens the u_sign knob question — not v1's, and registered
// (ADR-0039 D1.2's own clause).
class TorusZSurface : public SurfaceGeometry {
public:
    TorusZSurface(math::Vec2d axis_center, double R, double r,
                  double z_center)
        : center_(axis_center), R_(R), r_(r), z_c_(z_center) {}

    math::Vec3d evaluate(double u, double v) const override {
        const double w = R_ + r_ * std::cos(v);
        return {center_.x + w * std::cos(u),
                center_.y + w * std::sin(u),
                z_c_ + r_ * std::sin(v)};
    }

    math::Vec3d normal(double u, double v) const override {
        // The tube's OUTER radial: the outward of the fillet's flank
        // (the material inside the tube, the removed wedge beyond) —
        // the SAME sign as du x dv, the frame rule's own pin.
        const double cv = std::cos(v), sv = std::sin(v);
        return {cv * std::cos(u), cv * std::sin(u), sv};
    }

    SurfaceFamily family() const override { return SurfaceFamily::Torus; }
    void accept(SurfaceVisitor& v) const override { v.visit(*this); }
    math::Bboxd bounds() const override {
        const double e = R_ + r_;
        return {{-e + center_.x, -e + center_.y, z_c_ - r_},
                { e + center_.x,  e + center_.y, z_c_ + r_}};
    }

    math::Vec2d axis_center() const noexcept { return center_; }
    double major_radius() const noexcept { return R_; }
    double minor_radius() const noexcept { return r_; }
    double z_center() const noexcept { return z_c_; }

private:
    math::Vec2d center_;
    double R_;
    double r_;
    double z_c_;
};

// Ellipsoid — THE PROBE'S FAMILY (K1 m3, ADR-0042): the ellipsoid of
// revolution (the spheroid), the sphere's one-parameter generalization
// — the first family to enter under ADR-0041's protocol, through the
// six declared rows the ADR derives in the open. The sphere's own
// (pole, azimuth) form, one parameter more:
//
//   evaluate(u, v) = C + e0·(a·sin v·cos u) + e1·(a·sin v·sin u)
//                      + n̂·(c·cos v),      e1 = e0 × n̂,
//
// with a the EQUATORIAL semiaxis (the u-circle's radius, v = π/2 the
// equator) and c the POLAR semiaxis (the pole distance; v = 0 and
// v = π the poles — the parametrization's degenerate points, the
// sphere's own pole discipline: the u-circle collapses there, the
// pointwise frame checks sample off them). The frame rule holds BY
// CONSTRUCTION, in every frame, pointwise over the open domain
// (ADR-0042 D2's derivation):
//
//   ∂u×∂v = sin v·(a·c·sin v·cos u·e0 + a·c·sin v·sin u·e1
//                  + a²·cos v·n̂),
//   (∂u×∂v)·N̂ = sin v·(c·sin²v + (a²/c)·cos²v)  >  0,
//
// with N̂ the unnormalized outward gradient (sin v·cos u / a,
// sin v·sin u / a, cos v / c) in the frame basis — positive for every
// a, c > 0 and v ∈ (0, π), so the class is right-handed the only way
// it can be authored: a left-handed spheroid is UNREPRESENTABLE (no
// knob exists), the m3 pattern's cleanest case.
//
// THE RE-FRAME RULE'S BOUNDARY, read honestly (ADR-0042 D1/D2): the
// sphere's full re-frame freedom (any derived axis serves — a
// symmetry property) is a freedom the spheroid LACKS — the pole n̂ is
// INTRINSIC (the revolution axis), the azimuth e0 a free choice (the
// seam meridian u = 0 through +e0, the ADR-0023 discipline). The SSI
// entries type in the STORED frame when the pole serves — the
// sphere's own clause, the spheroid's permanent condition, never a
// silent re-frame.
//
// The v1 class carries NO second equatorial knob: the triaxial
// ellipsoid is unrepresentable in it (ADR-0042's Context: the
// family's next member, entering through the same six rows when its
// consumer arrives).
class EllipsoidSurface : public SurfaceGeometry {
public:
    // The default frame the world reads: pole +Z (the revolution axis
    // vertical — the CylinderZ/TorusZ precedent of the axis the
    // world's primitives stand on), azimuth +X (the seam meridian in
    // the +X half-plane).
    EllipsoidSurface(double equatorial, double polar,
                     math::Vec3d origin = {})
        : a_(equatorial), c_(polar), origin_(origin),
          pole_{0, 0, 1}, azimuth_{1, 0, 0} {
        validate();
        precompute();
    }

    // The explicit frame (the SphereSurface re-frame constructor's own
    // shape): `pole` IS the revolution axis, `azimuth` the seam
    // meridian's reference — perpendicularity validated LOUDLY (a
    // skewed frame has no spheroid parametrization in its span).
    EllipsoidSurface(double equatorial, double polar, math::Vec3d origin,
                     math::Vec3d pole, math::Vec3d azimuth)
        : a_(equatorial), c_(polar), origin_(origin),
          pole_(pole.normalized()), azimuth_(azimuth.normalized()) {
        validate();
        const double dot = pole_.dot(azimuth_);
        if (!std::isfinite(dot) || std::abs(dot) > 1e-6) {
            throw std::invalid_argument(
                "EllipsoidSurface: (pole, azimuth) must be perpendicular "
                "(the revolution axis and the seam meridian's reference), "
                "got pole·azimuth=" + std::to_string(dot) +
                " — a skewed frame bends the parametrization off the "
                "meridians and every authored p-curve with it (the "
                "SphereSurface re-frame rule's own form). Remedy: pass "
                "the revolution axis and a perpendicular azimuth "
                "reference (the entries' own derivations).");
        }
        precompute();
    }

    math::Vec3d evaluate(double u, double v) const override {
        const double sv = std::sin(v);
        return origin_ + e0_ * (a_ * sv * std::cos(u)) +
               e1_ * (a_ * sv * std::sin(u)) +
               pole_ * (c_ * std::cos(v));
    }

    math::Vec3d normal(double u, double v) const override {
        // The outward gradient (x/a², y/a², z/c²) read in the frame
        // basis, normalized — the SAME direction ∂u×∂v points (the
        // frame rule's own pin, ADR-0042 D2).
        const double sv = std::sin(v);
        const math::Vec3d g = e0_ * (sv * std::cos(u) / a_) +
                              e1_ * (sv * std::sin(u) / a_) +
                              pole_ * (std::cos(v) / c_);
        return g.normalized();
    }

    SurfaceFamily family() const override {
        return SurfaceFamily::Ellipsoid;
    }
    void accept(SurfaceVisitor& v) const override { v.visit(*this); }
    math::Bboxd bounds() const override {
        // The frame's own box: the principal extents carried through
        // the frame's components (conservative for a tilted frame,
        // EXACT for the default axes-aligned one — the honest v1; the
        // trim lives in the loops, not the surface).
        const auto ax = [](const math::Vec3d& e, double k) {
            return math::Vec3d{std::abs(e.x) * k, std::abs(e.y) * k,
                               std::abs(e.z) * k};
        };
        const math::Vec3d e = ax(e0_, a_) + ax(e1_, a_) + ax(pole_, c_);
        return {{origin_.x - e.x, origin_.y - e.y, origin_.z - e.z},
                {origin_.x + e.x, origin_.y + e.y, origin_.z + e.z}};
    }

    double equatorial_semiaxis() const noexcept { return a_; }
    double polar_semiaxis() const noexcept { return c_; }
    // The placement origin (G1 unit 4's table: translate_body re-anchors
    // through the family's own accessors).
    const math::Vec3d& origin() const noexcept { return origin_; }
    // The intrinsic frame (K1 m3): the pole n̂ (v = 0, the revolution
    // axis) and the azimuth reference e0 (u = 0, the seam meridian) —
    // readable so translate_body and the SSI entries carry the frame
    // through without a second copy (the sphere's own discipline).
    const math::Vec3d& pole() const noexcept { return pole_; }
    const math::Vec3d& azimuth() const noexcept { return azimuth_; }

private:
    void validate() const {
        if (!(std::isfinite(a_) && a_ > 0.0) ||
            !(std::isfinite(c_) && c_ > 0.0)) {
            throw std::invalid_argument(
                "EllipsoidSurface: the semiaxes must be finite and > 0 "
                "(equatorial=" + std::to_string(a_) + ", polar=" +
                std::to_string(c_) +
                ") — a non-positive or non-finite semiaxis degenerates "
                "the surface (the ADR-0029 family: the 'inside' becomes "
                "empty or the complement). Remedy: construct with "
                "positive finite semiaxes.");
        }
    }
    void precompute() {
        e0_ = azimuth_;
        e1_ = e0_.cross(pole_);  // e1 = e0 × n̂ (the sphere's own form)
    }

    double a_;
    double c_;
    math::Vec3d origin_;
    math::Vec3d pole_;
    math::Vec3d azimuth_;
    math::Vec3d e0_;
    math::Vec3d e1_;
};

} // namespace CAD_0::brep

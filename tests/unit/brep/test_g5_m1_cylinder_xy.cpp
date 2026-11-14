// tests/unit/brep/test_g5_m1_cylinder_xy.cpp
//
// G5 m1 — I FIANCHI CylinderX/CylinderY (ADR-0043, D2.1/D8): the
// cylinder family's X and Y placements land as first-class members
// of the existing analytic machine — the same mathematics one axis
// under CylinderZSurface, the flank writer's own classes (the
// TorusZ precedent). This file pins:
//
//   * THE FRAME RULE POINTWISE on both classes (the sampled
//     (∂u×∂v)·n > 0 guard, the analytic set's own law) with the
//     seam meridians the ADR froze: u = 0 ≡ +Z for CylinderX,
//     +X for CylinderY;
//   * THE CHIRALITY COUNTERPROOF (the m3 pattern): the naive
//     left-handed authoring (the flipped second axis ê₁ = +ŷ/+ẑ)
//     fails the rule at EVERY sample — red on the naive, green on
//     the landed class, the pair read together;
//   * THE QUARTER-CYLINDER SOLID — a hand-authored closed body (6
//     vertices, 9 edges, 5 faces, χ = 2) whose wall rides the new
//     class, the mass machinery's unified cylinder frame one
//     placement richer: V = (πr²/4)h and the centroid's classical
//     forms at 1e-12, the ORACLE — the test's own midpoint product
//     quadrature of the divergence kernels over the five faces'
//     parametrizations, ZERO code shared with the kernel — at the
//     declared 2e-6, the wall's own flux (the flank's contribution
//     to the divergence theorem, x·(∂u×∂v) = r² CONSTANT on the
//     wall) at its closed form, and the naive counterproofs;
//   * TRANSLATE_BODY's re-anchoring through the class's own
//     placement accessors (the table's prescribed extension).
//
// The guide numbers (r = 0.5, h = 1.5, every one verified against
// an independent volume quadrature at authoring time):
//   V = 0.09375π = 0.2945243112740431
//   centroid (X) = (0.75, −4r/3π, +4r/3π) = (0.75, −0.2122065907891938, +0.2122065907891938)
//   wall flux = πr²h/2 = 0.5890486225480862
#include <doctest/doctest.h>

#include "CAD_0/brep/analytic.hpp"
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/mass_properties.hpp"
#include "CAD_0/brep/pcurve.hpp"
#include "CAD_0/brep/subtract.hpp"

#include <cmath>
#include <functional>

namespace {

using CAD_0::brep::BRepBody;
using CAD_0::brep::CoEdge;
using CAD_0::brep::Edge;
using CAD_0::brep::Face;
using CAD_0::brep::Loop;
using CAD_0::brep::Vertex;
using CAD_0::math::Vec2d;
using CAD_0::math::Vec3d;

constexpr double kR = 0.5;
constexpr double kH = 1.5;
constexpr double kHalfPi = CAD_0::brep::kPi / 2.0;

// ---------------------------------------------------------------------------
// The hand-authored quarter-cylinder solid (the counts derived in the
// file header): 6 vertices, 9 edges, 5 faces (wall + 2 flats + 2 caps),
// every edge exactly two OPPOSITE co-edges, χ = 6 − 9 + 5 = 2.
//
// THE X BODY: the wall on CylinderXSurface(kR, 0, 0, 0) — axis x̂
// through (y, z) = (0, 0), the window u ∈ [0, π/2] × v ∈ [0, kH], the
// quarter y ≤ 0, z ≥ 0. The walk derivations (material on the left in
// every face's own domain, CCW around the outward normal):
//   wall   : (0,0)→(π/2,0)→(π/2,h)→(0,h) — 4 Line2, the mass window;
//   flat y=0 (outward +ŷ): v4→v0→v2→v5, the domain (u,v) = (−z, −x);
//   flat z=0 (outward −ẑ): v4→v5→v3→v1, the domain (u,v) = (x, −y);
//   cap x=0 (outward −x̂): v4→v1→arc→v0, the domain (u,v) = (−z, −y);
//   cap x=h (outward +x̂): v5→v2→arc→v3, the domain (u,v) = (z, −y).
// ---------------------------------------------------------------------------
std::unique_ptr<BRepBody> quarter_cylinder_x() {
    auto body = std::make_unique<BRepBody>();
    auto& shell = body->solid.shell_mut();
    shell.set_closed(true);

    body->vertices = {
        Vertex{{0.0, 0.0, kR}},   // v0: the seam meridian's foot (u=0, v=0)
        Vertex{{0.0, -kR, 0.0}},  // v1: the u=π/2 foot
        Vertex{{kH, 0.0, kR}},    // v2: the seam meridian's top (u=0, v=h)
        Vertex{{kH, -kR, 0.0}},   // v3: the u=π/2 top
        Vertex{{0.0, 0.0, 0.0}},  // v4: the axis endpoint at x=0
        Vertex{{kH, 0.0, 0.0}},   // v5: the axis endpoint at x=h
    };
    for (std::size_t i = 0; i < body->vertices.size(); ++i)
        body->vertices[i].set_id(static_cast<CAD_0::brep::VertexId>(i));

    body->edges = {
        Edge{0, 2},  // e0: the θ=0 wall line (wall + flat y=0)
        Edge{1, 3},  // e1: the θ=π/2 wall line (wall + flat z=0)
        Edge{0, 1},  // e2: the v=0 wall arc (wall + cap x=0)
        Edge{2, 3},  // e3: the v=h wall arc (wall + cap x=h)
        Edge{4, 0},  // e4: the x=0 line on the y=0 flat (flat y=0 + cap x=0)
        Edge{5, 2},  // e5: the x=h line on the y=0 flat (flat y=0 + cap x=h)
        Edge{4, 1},  // e6: the x=0 line on the z=0 flat (flat z=0 + cap x=0)
        Edge{5, 3},  // e7: the x=h line on the z=0 flat (flat z=0 + cap x=h)
        Edge{4, 5},  // e8: the axis line (flat y=0 + flat z=0)
    };
    for (std::size_t i = 0; i < body->edges.size(); ++i)
        body->edges[i].set_id(static_cast<CAD_0::brep::EdgeId>(i));

    const auto seg = [](Vec2d a, Vec2d b) {
        return CAD_0::brep::pcurve_segment(a, b, /*interior_left=*/true);
    };

    // Face 0 — the wall: CylinderXSurface, the Line2 rectangle.
    {
        CAD_0::brep::Face face(
            std::make_shared<CAD_0::brep::CylinderXSurface>(kR, 0.0, 0.0, 0.0));
        Loop loop;
        CoEdge e2_fwd{2, false};
        e2_fwd.set_p_curve(seg({0.0, 0.0}, {kHalfPi, 0.0}));
        CoEdge e1_fwd{1, false};
        e1_fwd.set_p_curve(seg({kHalfPi, 0.0}, {kHalfPi, kH}));
        CoEdge e3_rev{3, true};
        e3_rev.set_p_curve(seg({kHalfPi, kH}, {0.0, kH}));
        CoEdge e0_rev{0, true};
        e0_rev.set_p_curve(seg({0.0, kH}, {0.0, 0.0}));
        loop.add_coedge(e2_fwd);
        loop.add_coedge(e1_fwd);
        loop.add_coedge(e3_rev);
        loop.add_coedge(e0_rev);
        face.add_loop(std::move(loop));
        face.set_id(0);
        shell.add_face(std::move(face));
    }

    // Face 1 — the flat y=0 (outward +ŷ): the domain (u,v) = (−z, −x).
    {
        CAD_0::brep::Face face(std::make_shared<CAD_0::brep::PlaneSurface>(
            Vec3d{0, 0, 0}, Vec3d{0, 1, 0}));
        Loop loop;
        CoEdge e4_fwd{4, false};
        e4_fwd.set_p_curve(seg({0.0, 0.0}, {-kR, 0.0}));
        CoEdge e0_fwd{0, false};
        e0_fwd.set_p_curve(seg({-kR, 0.0}, {-kR, -kH}));
        CoEdge e5_rev{5, true};
        e5_rev.set_p_curve(seg({-kR, -kH}, {0.0, -kH}));
        CoEdge e8_rev{8, true};
        e8_rev.set_p_curve(seg({0.0, -kH}, {0.0, 0.0}));
        loop.add_coedge(e4_fwd);
        loop.add_coedge(e0_fwd);
        loop.add_coedge(e5_rev);
        loop.add_coedge(e8_rev);
        face.add_loop(std::move(loop));
        face.set_id(1);
        shell.add_face(std::move(face));
    }

    // Face 2 — the flat z=0 (outward −ẑ): the domain (u,v) = (x, −y).
    {
        CAD_0::brep::Face face(std::make_shared<CAD_0::brep::PlaneSurface>(
            Vec3d{0, 0, 0}, Vec3d{0, 0, -1}));
        Loop loop;
        CoEdge e8_fwd{8, false};
        e8_fwd.set_p_curve(seg({0.0, 0.0}, {kH, 0.0}));
        CoEdge e7_fwd{7, false};
        e7_fwd.set_p_curve(seg({kH, 0.0}, {kH, kR}));
        CoEdge e1_rev{1, true};
        e1_rev.set_p_curve(seg({kH, kR}, {0.0, kR}));
        CoEdge e6_rev{6, true};
        e6_rev.set_p_curve(seg({0.0, kR}, {0.0, 0.0}));
        loop.add_coedge(e8_fwd);
        loop.add_coedge(e7_fwd);
        loop.add_coedge(e1_rev);
        loop.add_coedge(e6_rev);
        face.add_loop(std::move(loop));
        face.set_id(2);
        shell.add_face(std::move(face));
    }

    // Face 3 — the cap x=0 (outward −x̂): the domain (u,v) = (−z, −y);
    // the boundary is 2 Line2 + 1 Arc2 (the quarter arc CCW, the
    // make_cylinder caps' own vocabulary).
    {
        CAD_0::brep::Face face(std::make_shared<CAD_0::brep::PlaneSurface>(
            Vec3d{0, 0, 0}, Vec3d{-1, 0, 0}));
        Loop loop;
        CoEdge e6_fwd{6, false};
        e6_fwd.set_p_curve(seg({0.0, 0.0}, {0.0, kR}));
        CoEdge e2_rev{2, true};
        e2_rev.set_p_curve(CAD_0::brep::pcurve_arc(
            Vec2d{0, 0}, kR, /*hole=*/false, kHalfPi, kHalfPi));
        CoEdge e4_rev{4, true};
        e4_rev.set_p_curve(seg({-kR, 0.0}, {0.0, 0.0}));
        loop.add_coedge(e6_fwd);
        loop.add_coedge(e2_rev);
        loop.add_coedge(e4_rev);
        face.add_loop(std::move(loop));
        face.set_id(3);
        shell.add_face(std::move(face));
    }

    // Face 4 — the cap x=h (outward +x̂): the domain (u,v) = (z, −y).
    {
        CAD_0::brep::Face face(std::make_shared<CAD_0::brep::PlaneSurface>(
            Vec3d{kH, 0, 0}, Vec3d{1, 0, 0}));
        Loop loop;
        CoEdge e5_fwd{5, false};
        e5_fwd.set_p_curve(seg({0.0, 0.0}, {kR, 0.0}));
        CoEdge e3_fwd{3, false};
        e3_fwd.set_p_curve(CAD_0::brep::pcurve_arc(
            Vec2d{0, 0}, kR, /*hole=*/false, 0.0, kHalfPi));
        CoEdge e7_rev{7, true};
        e7_rev.set_p_curve(seg({0.0, kR}, {0.0, 0.0}));
        loop.add_coedge(e5_fwd);
        loop.add_coedge(e3_fwd);
        loop.add_coedge(e7_rev);
        face.add_loop(std::move(loop));
        face.set_id(4);
        shell.add_face(std::move(face));
    }

    CAD_0::brep::link_radial_edges(*body);
    return body;
}

// ---------------------------------------------------------------------------
// THE Y BODY — the X body's image under the orientation-preserving
// cyclic map (x,y,z) ↦ (z,x,y) (x̂ ↦ ŷ, ẑ ↦ x̂ — the permutation that
// carries the X flank's frame onto the Y flank's: â = x̂ ↦ ŷ, the seam
// ê₀ = ẑ ↦ x̂). The same incidence table, the walks one axis over, the
// wall on CylinderYSurface(kR, 0, 0, 0), the window u ∈ [0, π/2] ×
// v ∈ [0, kH], the quarter x ≥ 0, z ≤ 0.
// ---------------------------------------------------------------------------
std::unique_ptr<BRepBody> quarter_cylinder_y() {
    auto body = std::make_unique<BRepBody>();
    auto& shell = body->solid.shell_mut();
    shell.set_closed(true);

    body->vertices = {
        Vertex{{kR, 0.0, 0.0}},   // v0: the seam meridian's foot (u=0, v=0)
        Vertex{{0.0, 0.0, -kR}},  // v1: the u=π/2 foot
        Vertex{{kR, kH, 0.0}},    // v2: the seam meridian's top
        Vertex{{0.0, kH, -kR}},   // v3: the u=π/2 top
        Vertex{{0.0, 0.0, 0.0}},  // v4: the axis endpoint at y=0
        Vertex{{0.0, kH, 0.0}},   // v5: the axis endpoint at y=h
    };
    for (std::size_t i = 0; i < body->vertices.size(); ++i)
        body->vertices[i].set_id(static_cast<CAD_0::brep::VertexId>(i));

    body->edges = {
        Edge{0, 2},  // e0: the θ=0 wall line (wall + flat z=0)
        Edge{1, 3},  // e1: the θ=π/2 wall line (wall + flat x=0)
        Edge{0, 1},  // e2: the v=0 wall arc (wall + cap y=0)
        Edge{2, 3},  // e3: the v=h wall arc (wall + cap y=h)
        Edge{4, 0},  // e4: the y=0 line on the z=0 flat (flat z=0 + cap y=0)
        Edge{5, 2},  // e5: the y=h line on the z=0 flat (flat z=0 + cap y=h)
        Edge{4, 1},  // e6: the y=0 line on the x=0 flat (flat x=0 + cap y=0)
        Edge{5, 3},  // e7: the y=h line on the x=0 flat (flat x=0 + cap y=h)
        Edge{4, 5},  // e8: the axis line (flat z=0 + flat x=0)
    };
    for (std::size_t i = 0; i < body->edges.size(); ++i)
        body->edges[i].set_id(static_cast<CAD_0::brep::EdgeId>(i));

    const auto seg = [](Vec2d a, Vec2d b) {
        return CAD_0::brep::pcurve_segment(a, b, /*interior_left=*/true);
    };

    // Face 0 — the wall: CylinderYSurface, the same Line2 rectangle.
    {
        CAD_0::brep::Face face(
            std::make_shared<CAD_0::brep::CylinderYSurface>(kR, 0.0, 0.0, 0.0));
        Loop loop;
        CoEdge e2_fwd{2, false};
        e2_fwd.set_p_curve(seg({0.0, 0.0}, {kHalfPi, 0.0}));
        CoEdge e1_fwd{1, false};
        e1_fwd.set_p_curve(seg({kHalfPi, 0.0}, {kHalfPi, kH}));
        CoEdge e3_rev{3, true};
        e3_rev.set_p_curve(seg({kHalfPi, kH}, {0.0, kH}));
        CoEdge e0_rev{0, true};
        e0_rev.set_p_curve(seg({0.0, kH}, {0.0, 0.0}));
        loop.add_coedge(e2_fwd);
        loop.add_coedge(e1_fwd);
        loop.add_coedge(e3_rev);
        loop.add_coedge(e0_rev);
        face.add_loop(std::move(loop));
        face.set_id(0);
        shell.add_face(std::move(face));
    }

    // Face 1 — the flat z=0 (outward +ẑ): the domain (u,v) = (−x, −y).
    {
        CAD_0::brep::Face face(std::make_shared<CAD_0::brep::PlaneSurface>(
            Vec3d{0, 0, 0}, Vec3d{0, 0, 1}));
        Loop loop;
        CoEdge e4_fwd{4, false};
        e4_fwd.set_p_curve(seg({0.0, 0.0}, {-kR, 0.0}));
        CoEdge e0_fwd{0, false};
        e0_fwd.set_p_curve(seg({-kR, 0.0}, {-kR, -kH}));
        CoEdge e5_rev{5, true};
        e5_rev.set_p_curve(seg({-kR, -kH}, {0.0, -kH}));
        CoEdge e8_rev{8, true};
        e8_rev.set_p_curve(seg({0.0, -kH}, {0.0, 0.0}));
        loop.add_coedge(e4_fwd);
        loop.add_coedge(e0_fwd);
        loop.add_coedge(e5_rev);
        loop.add_coedge(e8_rev);
        face.add_loop(std::move(loop));
        face.set_id(1);
        shell.add_face(std::move(face));
    }

    // Face 2 — the flat x=0 (outward −x̂): the domain (u,v) = (−z, −y).
    {
        CAD_0::brep::Face face(std::make_shared<CAD_0::brep::PlaneSurface>(
            Vec3d{0, 0, 0}, Vec3d{-1, 0, 0}));
        Loop loop;
        CoEdge e8_fwd{8, false};
        e8_fwd.set_p_curve(seg({0.0, 0.0}, {0.0, -kH}));
        CoEdge e7_fwd{7, false};
        e7_fwd.set_p_curve(seg({0.0, -kH}, {kR, -kH}));
        CoEdge e1_rev{1, true};
        e1_rev.set_p_curve(seg({kR, -kH}, {kR, 0.0}));
        CoEdge e6_rev{6, true};
        e6_rev.set_p_curve(seg({kR, 0.0}, {0.0, 0.0}));
        loop.add_coedge(e8_fwd);
        loop.add_coedge(e7_fwd);
        loop.add_coedge(e1_rev);
        loop.add_coedge(e6_rev);
        face.add_loop(std::move(loop));
        face.set_id(2);
        shell.add_face(std::move(face));
    }

    // Face 3 — the cap y=0 (outward −ŷ): the domain (u,v) = (z, −x);
    // the arc from (−R, 0) CCW to (0, −R) (start π, sweep π/2).
    {
        CAD_0::brep::Face face(std::make_shared<CAD_0::brep::PlaneSurface>(
            Vec3d{0, 0, 0}, Vec3d{0, -1, 0}));
        Loop loop;
        CoEdge e6_fwd{6, false};
        e6_fwd.set_p_curve(seg({0.0, 0.0}, {-kR, 0.0}));
        CoEdge e2_rev{2, true};
        e2_rev.set_p_curve(CAD_0::brep::pcurve_arc(
            Vec2d{0, 0}, kR, /*hole=*/false, CAD_0::brep::kPi, kHalfPi));
        CoEdge e4_rev{4, true};
        e4_rev.set_p_curve(seg({0.0, -kR}, {0.0, 0.0}));
        loop.add_coedge(e6_fwd);
        loop.add_coedge(e2_rev);
        loop.add_coedge(e4_rev);
        face.add_loop(std::move(loop));
        face.set_id(3);
        shell.add_face(std::move(face));
    }

    // Face 4 — the cap y=h (outward +ŷ): the domain (u,v) = (−z, −x);
    // the arc from (0, −R) CCW to (R, 0) (start 3π/2, sweep π/2).
    {
        CAD_0::brep::Face face(std::make_shared<CAD_0::brep::PlaneSurface>(
            Vec3d{0, kH, 0}, Vec3d{0, 1, 0}));
        Loop loop;
        CoEdge e5_fwd{5, false};
        e5_fwd.set_p_curve(seg({0.0, 0.0}, {0.0, -kR}));
        CoEdge e3_fwd{3, false};
        e3_fwd.set_p_curve(CAD_0::brep::pcurve_arc(
            Vec2d{0, 0}, kR, /*hole=*/false,
            3.0 * CAD_0::brep::kPi / 2.0, kHalfPi));
        CoEdge e7_rev{7, true};
        e7_rev.set_p_curve(seg({kR, 0.0}, {0.0, 0.0}));
        loop.add_coedge(e5_fwd);
        loop.add_coedge(e3_fwd);
        loop.add_coedge(e7_rev);
        face.add_loop(std::move(loop));
        face.set_id(4);
        shell.add_face(std::move(face));
    }

    CAD_0::brep::link_radial_edges(*body);
    return body;
}

// ---------------------------------------------------------------------------
// THE INDEPENDENT ORACLE (the m3 precedent's form, D1.5): the (u,v)
// midpoint product quadrature of the divergence kernels over each
// face's OWN parametrization — the TEST'S loops, ZERO code shared
// with the kernel's CylFrame machinery or its closed forms. The
// kernels (the divergence theorem's own algebra, general for any
// closed surface):
//     F0  = Σ ∫∫ x·(∂u×∂v) dudv          = 3V
//     Fij = Σ ∫∫ xᵢxⱼ·[x·(∂u×∂v)] dudv   = 5·S_ij   (div(xᵢxⱼx⃗) = 5xᵢxⱼ)
//     Fᵢ  = Σ ∫∫ xᵢ·[x·(∂u×∂v)] dudv     = 4·V·cᵢ   (div(xᵢx⃗) = 4xᵢ)
// The grid declared: 1600 × 800 per face; the derivatives are the
// parametrization's own central differences (h = 1e-6, the error two
// orders below the quadrature's own O(h²)); the tolerance 2e-6
// relative (the m3 precedent's number for this class of kernels).
// ---------------------------------------------------------------------------
struct QuadFlux {
    double F0{0};
    double F1{0}, F2{0}, F3{0};        // the centroid kernels
    double F11{0}, F22{0}, F33{0};     // the second moments
    double F12{0}, F13{0}, F23{0};
};

QuadFlux integrate_face(const std::function<Vec3d(double, double)>& P,
                        double u0, double u1, double v0, double v1,
                        int NU, int NV) {
    QuadFlux q;
    const double du = (u1 - u0) / NU, dv = (v1 - v0) / NV;
    const double h = 1e-6;
    for (int i = 0; i < NU; ++i) {
        const double u = u0 + (i + 0.5) * du;
        for (int j = 0; j < NV; ++j) {
            const double v = v0 + (j + 0.5) * dv;
            const Vec3d pu = P(u, v);
            const Vec3d dpu = (P(u + h, v) - P(u - h, v)) / (2.0 * h);
            const Vec3d dpv = (P(u, v + h) - P(u, v - h)) / (2.0 * h);
            const Vec3d n = dpu.cross(dpv);
            const double w = du * dv;
            const double k = pu.dot(n) * w;
            q.F0 += k;
            q.F1 += pu.x * k;
            q.F2 += pu.y * k;
            q.F3 += pu.z * k;
            q.F11 += pu.x * pu.x * k;
            q.F22 += pu.y * pu.y * k;
            q.F33 += pu.z * pu.z * k;
            q.F12 += pu.x * pu.y * k;
            q.F13 += pu.x * pu.z * k;
            q.F23 += pu.y * pu.z * k;
        }
    }
    return q;
}

// The X body's five faces' parametrizations (each authored so
// ∂u×∂v points along the OUTWARD normal — the walk discipline read
// on the oracle's own geometry):
QuadFlux oracle_x() {
    QuadFlux q, f;
    // The wall: the class's own parametrization; x·(∂u×∂v) = r² (const).
    f = integrate_face([](double u, double v) {
            return Vec3d{v, -kR * std::sin(u), kR * std::cos(u)};
        }, 0.0, kHalfPi, 0.0, kH, 1600, 800);
    q = f;
    // The flat y=0 (outward +ŷ).
    f = integrate_face([](double u, double v) {
            return Vec3d{v, 0.0, u};
        }, 0.0, kR, 0.0, kH, 1600, 800);
    q.F0 += f.F0; q.F1 += f.F1; q.F2 += f.F2; q.F3 += f.F3;
    q.F11 += f.F11; q.F22 += f.F22; q.F33 += f.F33;
    q.F12 += f.F12; q.F13 += f.F13; q.F23 += f.F23;
    // The flat z=0 (outward −ẑ).
    f = integrate_face([](double u, double v) {
            return Vec3d{v, u, 0.0};
        }, -kR, 0.0, 0.0, kH, 1600, 800);
    q.F0 += f.F0; q.F1 += f.F1; q.F2 += f.F2; q.F3 += f.F3;
    q.F11 += f.F11; q.F22 += f.F22; q.F33 += f.F33;
    q.F12 += f.F12; q.F13 += f.F13; q.F23 += f.F23;
    // The cap x=0 (outward −x̂), the polar quarter-disc.
    f = integrate_face([](double u, double v) {
            return Vec3d{0.0, -v * std::sin(u), v * std::cos(u)};
        }, 0.0, kHalfPi, 0.0, kR, 1600, 800);
    q.F0 += f.F0; q.F1 += f.F1; q.F2 += f.F2; q.F3 += f.F3;
    q.F11 += f.F11; q.F22 += f.F22; q.F33 += f.F33;
    q.F12 += f.F12; q.F13 += f.F13; q.F23 += f.F23;
    // The cap x=h (outward +x̂).
    f = integrate_face([](double u, double v) {
            return Vec3d{kH, -v * std::cos(u), v * std::sin(u)};
        }, 0.0, kHalfPi, 0.0, kR, 1600, 800);
    q.F0 += f.F0; q.F1 += f.F1; q.F2 += f.F2; q.F3 += f.F3;
    q.F11 += f.F11; q.F22 += f.F22; q.F33 += f.F33;
    q.F12 += f.F12; q.F13 += f.F13; q.F23 += f.F23;
    return q;
}

// The Y body's five faces — the X oracle's images under the cyclic
// map (x,y,z) ↦ (z,x,y).
QuadFlux oracle_y() {
    QuadFlux q, f;
    f = integrate_face([](double u, double v) {
            return Vec3d{kR * std::cos(u), v, -kR * std::sin(u)};
        }, 0.0, kHalfPi, 0.0, kH, 1600, 800);
    q = f;
    f = integrate_face([](double u, double v) {
            return Vec3d{u, v, 0.0};
        }, 0.0, kR, 0.0, kH, 1600, 800);
    q.F0 += f.F0; q.F1 += f.F1; q.F2 += f.F2; q.F3 += f.F3;
    q.F11 += f.F11; q.F22 += f.F22; q.F33 += f.F33;
    q.F12 += f.F12; q.F13 += f.F13; q.F23 += f.F23;
    f = integrate_face([](double u, double v) {
            return Vec3d{0.0, v, u};
        }, -kR, 0.0, 0.0, kH, 1600, 800);
    q.F0 += f.F0; q.F1 += f.F1; q.F2 += f.F2; q.F3 += f.F3;
    q.F11 += f.F11; q.F22 += f.F22; q.F33 += f.F33;
    q.F12 += f.F12; q.F13 += f.F13; q.F23 += f.F23;
    f = integrate_face([](double u, double v) {
            return Vec3d{v * std::cos(u), 0.0, -v * std::sin(u)};
        }, 0.0, kHalfPi, 0.0, kR, 1600, 800);
    q.F0 += f.F0; q.F1 += f.F1; q.F2 += f.F2; q.F3 += f.F3;
    q.F11 += f.F11; q.F22 += f.F22; q.F33 += f.F33;
    q.F12 += f.F12; q.F13 += f.F13; q.F23 += f.F23;
    f = integrate_face([](double u, double v) {
            return Vec3d{v * std::sin(u), kH, -v * std::cos(u)};
        }, 0.0, kHalfPi, 0.0, kR, 1600, 800);
    q.F0 += f.F0; q.F1 += f.F1; q.F2 += f.F2; q.F3 += f.F3;
    q.F11 += f.F11; q.F22 += f.F22; q.F33 += f.F33;
    q.F12 += f.F12; q.F13 += f.F13; q.F23 += f.F23;
    return q;
}

} // namespace

// ===========================================================================
// THE FRAME RULE POINTWISE + THE SEAM/EVALUATE/NORMAL/BOUNDS PINS
// ===========================================================================
TEST_CASE("G5 m1 the frame rule pointwise: CylinderX and CylinderY — "
          "right-handed by construction, the seam meridians the ADR "
          "froze (+Z and +X)") {
    const auto frame_rule = [](const auto& surf, double u0, double u1,
                               double v0, double v1) {
        for (int i = 0; i <= 12; ++i) {
            const double u = u0 + (u1 - u0) * i / 12.0;
            for (int j = 0; j <= 12; ++j) {
                const double v = v0 + (v1 - v0) * j / 12.0;
                const double h = 1e-7;
                const Vec3d pu = surf.evaluate(u, v);
                const Vec3d du = (surf.evaluate(u + h, v) - pu) / h;
                const Vec3d dv = (surf.evaluate(u, v + h) - pu) / h;
                CHECK(du.cross(dv).dot(surf.normal(u, v)) > 0.0);
            }
        }
    };

    SUBCASE("CylinderX: the seam u = 0 at +Z, the window's sweep") {
        const CAD_0::brep::CylinderXSurface s(kR, 0.25, -0.4, 0.7);
        frame_rule(s, 0.0, CAD_0::brep::kTwoPi, -1.0, 1.0);
        // The seam meridian: u = 0 is the +Z direction from the axis.
        auto p = s.evaluate(0.0, 0.3);
        CHECK(p.x == doctest::Approx(0.25 + 0.3).epsilon(1e-12));
        CHECK(p.y == doctest::Approx(-0.4).epsilon(1e-12));
        CHECK(p.z == doctest::Approx(0.7 + kR).epsilon(1e-12));
        // The quarter sweep: u = π/2 is the −Y direction.
        p = s.evaluate(kHalfPi, 0.0);
        CHECK(p.x == doctest::Approx(0.25).epsilon(1e-12));
        CHECK(p.y == doctest::Approx(-0.4 - kR).epsilon(1e-12));
        CHECK(p.z == doctest::Approx(0.7).epsilon(1e-12));
        // All points at distance R from the axis line; the normal is
        // the radial unit vector (0, −sin u, cos u).
        for (int i = 0; i <= 24; ++i) {
            const double u = CAD_0::brep::kTwoPi * i / 24.0;
            const Vec3d q = s.evaluate(u, 0.37);
            CHECK(std::hypot(q.y + 0.4, q.z - 0.7) ==
                  doctest::Approx(kR).epsilon(1e-12));
            const Vec3d n = s.normal(u, 0.37);
            CHECK(n.x == doctest::Approx(0.0).epsilon(1e-12));
            CHECK(n.y == doctest::Approx(-std::sin(u)).epsilon(1e-12));
            CHECK(n.z == doctest::Approx(std::cos(u)).epsilon(1e-12));
        }
        CHECK(s.family() == CAD_0::brep::SurfaceFamily::Cylinder);
        CHECK(s.radius() == doctest::Approx(kR));
        CHECK(s.x_base() == doctest::Approx(0.25));
        CHECK(s.axis_y() == doctest::Approx(-0.4));
        CHECK(s.axis_z() == doctest::Approx(0.7));
    }

    SUBCASE("CylinderX: the bounds — the full-cylinder box, v "
            "unbounded above the base (the Z placement's own)") {
        const CAD_0::brep::CylinderXSurface s(kR, 1.0, 2.0, 3.0);
        const auto b = s.bounds();
        CHECK(b.min.x == doctest::Approx(1.0).epsilon(1e-12));
        CHECK(b.min.y == doctest::Approx(2.0 - kR).epsilon(1e-12));
        CHECK(b.max.y == doctest::Approx(2.0 + kR).epsilon(1e-12));
        CHECK(b.min.z == doctest::Approx(3.0 - kR).epsilon(1e-12));
        CHECK(b.max.z == doctest::Approx(3.0 + kR).epsilon(1e-12));
    }

    SUBCASE("CylinderY: the seam u = 0 at +X, the window's sweep") {
        const CAD_0::brep::CylinderYSurface s(kR, -0.3, 0.6, 0.2);
        frame_rule(s, 0.0, CAD_0::brep::kTwoPi, -1.0, 1.0);
        auto p = s.evaluate(0.0, 0.45);
        CHECK(p.x == doctest::Approx(0.6 + kR).epsilon(1e-12));
        CHECK(p.y == doctest::Approx(-0.3 + 0.45).epsilon(1e-12));
        CHECK(p.z == doctest::Approx(0.2).epsilon(1e-12));
        // The quarter sweep: u = π/2 is the −Z direction.
        p = s.evaluate(kHalfPi, 0.0);
        CHECK(p.x == doctest::Approx(0.6).epsilon(1e-12));
        CHECK(p.y == doctest::Approx(-0.3).epsilon(1e-12));
        CHECK(p.z == doctest::Approx(0.2 - kR).epsilon(1e-12));
        for (int i = 0; i <= 24; ++i) {
            const double u = CAD_0::brep::kTwoPi * i / 24.0;
            const Vec3d q = s.evaluate(u, -0.11);
            CHECK(std::hypot(q.x - 0.6, q.z - 0.2) ==
                  doctest::Approx(kR).epsilon(1e-12));
            const Vec3d n = s.normal(u, -0.11);
            CHECK(n.x == doctest::Approx(std::cos(u)).epsilon(1e-12));
            CHECK(n.y == doctest::Approx(0.0).epsilon(1e-12));
            CHECK(n.z == doctest::Approx(-std::sin(u)).epsilon(1e-12));
        }
        CHECK(s.family() == CAD_0::brep::SurfaceFamily::Cylinder);
        CHECK(s.radius() == doctest::Approx(kR));
        CHECK(s.y_base() == doctest::Approx(-0.3));
        CHECK(s.axis_x() == doctest::Approx(0.6));
        CHECK(s.axis_z() == doctest::Approx(0.2));
    }

    SUBCASE("CylinderY: the bounds") {
        const CAD_0::brep::CylinderYSurface s(kR, 1.0, 2.0, 3.0);
        const auto b = s.bounds();
        CHECK(b.min.y == doctest::Approx(1.0).epsilon(1e-12));
        CHECK(b.min.x == doctest::Approx(2.0 - kR).epsilon(1e-12));
        CHECK(b.max.x == doctest::Approx(2.0 + kR).epsilon(1e-12));
        CHECK(b.min.z == doctest::Approx(3.0 - kR).epsilon(1e-12));
        CHECK(b.max.z == doctest::Approx(3.0 + kR).epsilon(1e-12));
    }

    SUBCASE("The honest refusals: a non-positive radius is not a "
            "flank (the diagnosis + remedy culture)") {
        bool threw = false;
        try {
            CAD_0::brep::CylinderXSurface bad(0.0, 0, 0, 0);
            (void)bad;
        } catch (const std::invalid_argument& e) {
            threw = true;
            CHECK(std::string(e.what()).find("radius") !=
                  std::string::npos);
            CHECK(std::string(e.what()).find("Remedy") !=
                  std::string::npos);
        }
        CHECK(threw);
        threw = false;
        try {
            CAD_0::brep::CylinderYSurface bad(-1.0, 0, 0, 0);
            (void)bad;
        } catch (const std::invalid_argument& e) {
            threw = true;
            CHECK(std::string(e.what()).find("radius") !=
                  std::string::npos);
        }
        CHECK(threw);
    }
}

// ===========================================================================
// THE CHIRALITY COUNTERPROOF — the naive left-handed authoring, the
// guard RED on it (the m3 pattern, one family over: the flipped
// second axis ê₁ = +ŷ/+ẑ — the historical left-handed cylinder's own
// mistake, the X/Y placements' shape)
// ===========================================================================
TEST_CASE("G5 m1 the counterproof: the naive flipped-ê₁ authoring is "
          "LEFT-handed — the frame rule RED where the classes are "
          "green (the discriminating pin, no knob exists)") {
    // THE NAIVE AUTHORING: ê₁ = +ŷ for the X placement (ê₁ = â×ê₀
    // NEGATED) — evaluate (x₀+v, y_c + R sin u, z_c + R cos u). It is
    // NOT the landed class (a left-handed X flank is unrepresentable
    // in it — the constructor derives ê₁ from the product); this
    // local authoring exists so the guard's discrimination is pinned.
    struct NLX {
        Vec3d evaluate(double u, double v) const {
            return {v, kR * std::sin(u), kR * std::cos(u)};
        }
        Vec3d normal(double u, double) const {
            return {0.0, std::sin(u), std::cos(u)};
        }
    };
    struct NLY {
        Vec3d evaluate(double u, double v) const {
            return {kR * std::cos(u), v, kR * std::sin(u)};
        }
        Vec3d normal(double u, double) const {
            return {std::cos(u), 0.0, std::sin(u)};
        }
    };
    const auto sample_red = [](const auto& naive, int& red, int& total) {
        red = 0;
        total = 0;
        for (int i = 0; i <= 12; ++i) {
            const double u = CAD_0::brep::kTwoPi * i / 12.0;
            for (int j = 0; j <= 12; ++j) {
                const double v = -1.0 + 2.0 * j / 12.0;
                const double h = 1e-7;
                const Vec3d pu = naive.evaluate(u, v);
                const Vec3d du = (naive.evaluate(u + h, v) - pu) / h;
                const Vec3d dv = (naive.evaluate(u, v + h) - pu) / h;
                ++total;
                if (du.cross(dv).dot(naive.normal(u, v)) <= 0.0) ++red;
            }
        }
    };

    SUBCASE("The X naive: red at EVERY sample; the landed class green "
            "on the same grid") {
        const NLX naive;
        int red = 0, total = 0;
        sample_red(naive, red, total);
        CHECK(red == total);
        CHECK(total == 169);
        const CAD_0::brep::CylinderXSurface landed(kR, 0.0, 0.0, 0.0);
        int green = 0;
        for (int i = 0; i <= 12; ++i) {
            const double u = CAD_0::brep::kTwoPi * i / 12.0;
            for (int j = 0; j <= 12; ++j) {
                const double v = -1.0 + 2.0 * j / 12.0;
                const double h = 1e-7;
                const Vec3d pu = landed.evaluate(u, v);
                const Vec3d du = (landed.evaluate(u + h, v) - pu) / h;
                const Vec3d dv = (landed.evaluate(u, v + h) - pu) / h;
                if (du.cross(dv).dot(landed.normal(u, v)) > 0.0) ++green;
            }
        }
        CHECK(green == 169);
    }

    SUBCASE("The Y naive: red at EVERY sample; the landed class green "
            "on the same grid") {
        const NLY naive;
        int red = 0, total = 0;
        sample_red(naive, red, total);
        CHECK(red == total);
        CHECK(total == 169);
        const CAD_0::brep::CylinderYSurface landed(kR, 0.0, 0.0, 0.0);
        int green = 0;
        for (int i = 0; i <= 12; ++i) {
            const double u = CAD_0::brep::kTwoPi * i / 12.0;
            for (int j = 0; j <= 12; ++j) {
                const double v = -1.0 + 2.0 * j / 12.0;
                const double h = 1e-7;
                const Vec3d pu = landed.evaluate(u, v);
                const Vec3d du = (landed.evaluate(u + h, v) - pu) / h;
                const Vec3d dv = (landed.evaluate(u, v + h) - pu) / h;
                if (du.cross(dv).dot(landed.normal(u, v)) > 0.0) ++green;
            }
        }
        CHECK(green == 169);
    }
}

// ===========================================================================
// THE QUARTER-CYLINDER SOLID ON CylinderXSurface — the counts, the
// classical forms, the oracle (three roads, zero shared code)
// ===========================================================================
TEST_CASE("G5 m1 the mass family: the X quarter-cylinder solid — the "
          "counts 6/9/5 (χ = 2), V = (πr²/4)h, the centroid's "
          "classical forms, the oracle's three roads") {
    auto body = quarter_cylinder_x();

    // THE COUNTS (the topology of the hand-authored solid, verified
    // structurally: the closed-orientable contract is the gate the
    // mass machinery itself demands).
    CHECK(body->vertices.size() == 6);
    CHECK(body->edges.size() == 9);
    CHECK(body->solid.shell().faces().size() == 5);
    const int chi = static_cast<int>(body->vertices.size()) -
                    static_cast<int>(body->edges.size()) +
                    static_cast<int>(body->solid.shell().faces().size());
    CHECK(chi == 2);
    CHECK(CAD_0::brep::is_closed_orientable(*body));

    const CAD_0::brep::MassProperties mp = CAD_0::brep::mass_properties(
        *body, CAD_0::math::Tolerance::linear(1e-9));

    // THE CLASSICAL FORMS (the closed road, 1e-12 — the house pin).
    const double V = CAD_0::brep::kPi * kR * kR * kH / 4.0;
    CHECK(mp.all_exact);
    CHECK(mp.volume == doctest::Approx(0.2945243112740431).epsilon(1e-12));
    CHECK(mp.volume == doctest::Approx(V).epsilon(1e-12));
    CHECK(mp.centroid.x == doctest::Approx(kH / 2).epsilon(1e-12));
    CHECK(mp.centroid.y ==
          doctest::Approx(-4.0 * kR / (3.0 * CAD_0::brep::kPi)).epsilon(1e-12));
    CHECK(mp.centroid.z ==
          doctest::Approx(+4.0 * kR / (3.0 * CAD_0::brep::kPi)).epsilon(1e-12));
    CHECK(mp.centroid.y == doctest::Approx(-0.2122065907891938).epsilon(1e-12));

    // THE SECOND ROAD — the test's own closed forms for the origin
    // moments (the quarter-disc's own algebra, verified against an
    // independent volume quadrature at authoring time):
    //   S_xx = (πr²/4)(h³/3), S_yy = S_zz = πr⁴h/16,
    //   S_yz = −r⁴h/8, S_xy = −h²r³/6, S_xz = +h²r³/6,
    // then I^C = (tr(S)δ − S) − V(δ|c|² − c⊗c) read on them.
    const double Sxx = CAD_0::brep::kPi * kR * kR * kH * kH * kH / 12.0;
    const double Syy = CAD_0::brep::kPi * kR * kR * kR * kR * kH / 16.0;
    const double Szz = Syy;
    const double Syz = -kR * kR * kR * kR * kH / 8.0;
    const double Sxy = -kH * kH * kR * kR * kR / 6.0;
    const double Sxz = +kH * kH * kR * kR * kR / 6.0;
    const double trS = Sxx + Syy + Szz;
    const Vec3d c = mp.centroid;
    const double c2 = c.dot(c);
    // I_ij = tr(S)δ_ij − S_ij − V(δ_ij|c|² − cᵢcⱼ), the test's own
    // algebra (the parallel-axis theorem read on the closed forms).
    const auto I_of = [&](int i, int j) {
        const double Sij[3][3] = {{Sxx, Sxy, Sxz},
                                  {Sxy, Syy, Syz},
                                  {Sxz, Syz, Szz}};
        const double ci[3] = {c.x, c.y, c.z};
        const double d = (i == j) ? 1.0 : 0.0;
        return trS * d - Sij[i][j] - V * (d * c2 - ci[i] * ci[j]);
    };
    CHECK(mp.inertia_about_centroid(0, 0) ==
          doctest::Approx(I_of(0, 0)).epsilon(1e-12));
    CHECK(mp.inertia_about_centroid(1, 1) ==
          doctest::Approx(I_of(1, 1)).epsilon(1e-12));
    CHECK(mp.inertia_about_centroid(2, 2) ==
          doctest::Approx(I_of(2, 2)).epsilon(1e-12));
    CHECK(mp.inertia_about_centroid(0, 1) ==
          doctest::Approx(I_of(0, 1)).epsilon(1e-12));
    CHECK(mp.inertia_about_centroid(0, 2) ==
          doctest::Approx(I_of(0, 2)).epsilon(1e-12));
    CHECK(mp.inertia_about_centroid(1, 2) ==
          doctest::Approx(I_of(1, 2)).epsilon(1e-12));

    // THE THIRD ROAD — the oracle: the test's own midpoint product
    // quadrature of the divergence kernels over the five faces'
    // parametrizations (zero code shared with the kernel).
    const QuadFlux q = oracle_x();
    CHECK(mp.volume == doctest::Approx(q.F0 / 3.0).epsilon(2e-6));
    CHECK(mp.centroid.x == doctest::Approx(q.F1 / (4.0 * V)).epsilon(2e-6));
    CHECK(mp.centroid.y == doctest::Approx(q.F2 / (4.0 * V)).epsilon(2e-6));
    CHECK(mp.centroid.z == doctest::Approx(q.F3 / (4.0 * V)).epsilon(2e-6));
    const double oSxx = q.F11 / 5.0, oSyy = q.F22 / 5.0, oSzz = q.F33 / 5.0;
    const double oSxy = q.F12 / 5.0, oSxz = q.F13 / 5.0, oSyz = q.F23 / 5.0;
    const double oTr = oSxx + oSyy + oSzz;
    const auto oI_of = [&](int i, int j) {
        const double Sij[3][3] = {{oSxx, oSxy, oSxz},
                                  {oSxy, oSyy, oSyz},
                                  {oSxz, oSyz, oSzz}};
        const double ci[3] = {c.x, c.y, c.z};
        const double d = (i == j) ? 1.0 : 0.0;
        return oTr * d - Sij[i][j] - V * (d * c2 - ci[i] * ci[j]);
    };
    CHECK(mp.inertia_about_centroid(0, 0) ==
          doctest::Approx(oI_of(0, 0)).epsilon(2e-6));
    CHECK(mp.inertia_about_centroid(1, 1) ==
          doctest::Approx(oI_of(1, 1)).epsilon(2e-6));
    CHECK(mp.inertia_about_centroid(2, 2) ==
          doctest::Approx(oI_of(2, 2)).epsilon(2e-6));
    CHECK(mp.inertia_about_centroid(1, 2) ==
          doctest::Approx(oI_of(1, 2)).epsilon(2e-6));

    // THE WALL'S OWN FLUX — the flank's contribution to the divergence
    // theorem, named: x·(∂u×∂v) = r² (CONSTANT on the wall — the
    // parametrization's own algebra: the point sits at distance r
    // from the axis and the axis passes through the origin), so the
    // wall's F0 = r²·(π/2)·h in EXACT arithmetic and the oracle's
    // midpoint rule reproduces it to the derivative's rounding.
    const double wall_flux = kR * kR * kHalfPi * kH;
    CHECK(wall_flux == doctest::Approx(0.5890486225480862).epsilon(1e-12));
    // The wall's share, measured by the oracle on the wall face alone
    // (the first integrate_face call of oracle_x, the constant
    // integrand's exactness under the midpoint rule).
    const QuadFlux wall_only = integrate_face(
        [](double u, double v) {
            return Vec3d{v, -kR * std::sin(u), kR * std::cos(u)};
        }, 0.0, kHalfPi, 0.0, kH, 1600, 800);
    CHECK(wall_only.F0 == doctest::Approx(wall_flux).epsilon(1e-9));
    // The closure's share is the rest: 3V − wall.
    CHECK(q.F0 - wall_only.F0 ==
          doctest::Approx(3.0 * V - wall_flux).epsilon(2e-6));

    // THE NAIVE COUNTERPROOFS (the m3 pattern): the half-cylinder
    // form and the r²h box form are NOT the quarter's volume.
    const double naive_half = CAD_0::brep::kPi * kR * kR * kH;
    CHECK(naive_half > 1.1);  // grossly off (4x)
    CHECK_FALSE(mp.volume == doctest::Approx(naive_half).epsilon(1e-3));
    const double naive_box = kR * kR * kH;
    CHECK_FALSE(mp.volume == doctest::Approx(naive_box).epsilon(1e-3));
}

// ===========================================================================
// THE Y BODY — the same mathematics one axis over: the placement's
// own CylFrame filling, verified by the classical forms and the oracle
// ===========================================================================
TEST_CASE("G5 m1 the mass family: the Y quarter-cylinder solid — the "
          "placement's own frame, the classical forms, the oracle") {
    auto body = quarter_cylinder_y();

    CHECK(body->vertices.size() == 6);
    CHECK(body->edges.size() == 9);
    CHECK(body->solid.shell().faces().size() == 5);
    CHECK(CAD_0::brep::is_closed_orientable(*body));

    const CAD_0::brep::MassProperties mp = CAD_0::brep::mass_properties(
        *body, CAD_0::math::Tolerance::linear(1e-9));

    const double V = CAD_0::brep::kPi * kR * kR * kH / 4.0;
    CHECK(mp.all_exact);
    CHECK(mp.volume == doctest::Approx(V).epsilon(1e-12));
    // The centroid: the X body's image under (x,y,z) ↦ (z,x,y).
    CHECK(mp.centroid.x ==
          doctest::Approx(+4.0 * kR / (3.0 * CAD_0::brep::kPi)).epsilon(1e-12));
    CHECK(mp.centroid.y == doctest::Approx(kH / 2).epsilon(1e-12));
    CHECK(mp.centroid.z ==
          doctest::Approx(-4.0 * kR / (3.0 * CAD_0::brep::kPi)).epsilon(1e-12));

    // The oracle, one axis over.
    const QuadFlux q = oracle_y();
    CHECK(mp.volume == doctest::Approx(q.F0 / 3.0).epsilon(2e-6));
    CHECK(mp.centroid.x == doctest::Approx(q.F1 / (4.0 * V)).epsilon(2e-6));
    CHECK(mp.centroid.y == doctest::Approx(q.F2 / (4.0 * V)).epsilon(2e-6));
    CHECK(mp.centroid.z == doctest::Approx(q.F3 / (4.0 * V)).epsilon(2e-6));

    // The inertia about the centroid: the test's own closed forms in
    // the Y frame (the X body's S carried by the cyclic permutation,
    // verified against an independent volume quadrature at authoring
    // time): S_xx = S_zz = πr⁴h/16, S_yy = πr²h³/12, S_xz = −r⁴h/8,
    // S_xy = +h²r³/6, S_yz = −h²r³/6 — plus the oracle's own
    // parallel-axis read.
    const double Sxx = CAD_0::brep::kPi * kR * kR * kH * kH * kH / 12.0;
    const double Sd = CAD_0::brep::kPi * kR * kR * kR * kR * kH / 16.0;
    const double Sxz = -kR * kR * kR * kR * kH / 8.0;
    const double Sxy = +kH * kH * kR * kR * kR / 6.0;
    const double Syz = -kH * kH * kR * kR * kR / 6.0;
    const double trS = Sxx + 2.0 * Sd;
    const Vec3d c = mp.centroid;
    const double c2 = c.dot(c);
    const auto I_of = [&](int i, int j) {
        const double Sij[3][3] = {{Sd, Sxy, Sxz},
                                  {Sxy, Sxx, Syz},
                                  {Sxz, Syz, Sd}};
        const double ci[3] = {c.x, c.y, c.z};
        const double d = (i == j) ? 1.0 : 0.0;
        return trS * d - Sij[i][j] - V * (d * c2 - ci[i] * ci[j]);
    };
    CHECK(mp.inertia_about_centroid(0, 0) ==
          doctest::Approx(I_of(0, 0)).epsilon(1e-12));
    CHECK(mp.inertia_about_centroid(1, 1) ==
          doctest::Approx(I_of(1, 1)).epsilon(1e-12));
    CHECK(mp.inertia_about_centroid(2, 2) ==
          doctest::Approx(I_of(2, 2)).epsilon(1e-12));
    CHECK(mp.inertia_about_centroid(0, 2) ==
          doctest::Approx(I_of(0, 2)).epsilon(1e-12));
    CHECK(mp.inertia_about_centroid(0, 1) ==
          doctest::Approx(I_of(0, 1)).epsilon(1e-12));
    CHECK(mp.inertia_about_centroid(1, 2) ==
          doctest::Approx(I_of(1, 2)).epsilon(1e-12));

    // The oracle's own parallel-axis read (the quadrature's S, the
    // same shift) — the second road, one axis over.
    const double oSxx = q.F11 / 5.0, oSyy = q.F22 / 5.0, oSzz = q.F33 / 5.0;
    const double oSxy = q.F12 / 5.0, oSxz = q.F13 / 5.0, oSyz = q.F23 / 5.0;
    const double oTr = oSxx + oSyy + oSzz;
    const auto oI_of = [&](int i, int j) {
        const double Sij[3][3] = {{oSxx, oSxy, oSxz},
                                  {oSxy, oSyy, oSyz},
                                  {oSxz, oSyz, oSzz}};
        const double ci[3] = {c.x, c.y, c.z};
        const double d = (i == j) ? 1.0 : 0.0;
        return oTr * d - Sij[i][j] - V * (d * c2 - ci[i] * ci[j]);
    };
    CHECK(mp.inertia_about_centroid(0, 0) ==
          doctest::Approx(oI_of(0, 0)).epsilon(2e-6));
    CHECK(mp.inertia_about_centroid(1, 1) ==
          doctest::Approx(oI_of(1, 1)).epsilon(2e-6));
    CHECK(mp.inertia_about_centroid(1, 2) ==
          doctest::Approx(oI_of(1, 2)).epsilon(2e-6));
}

// ===========================================================================
// TRANSLATE_BODY — the table's prescribed extension: the placement
// accessors, the frame unchanged, the authored p-curves valid verbatim
// ===========================================================================
TEST_CASE("G5 m1 translate_body: the X flank's body re-anchors "
          "through its own accessors — the mass follows, the centroid "
          "moves, the inertia about the centroid is invariant") {
    auto body = quarter_cylinder_x();
    const CAD_0::brep::MassProperties mp0 = CAD_0::brep::mass_properties(
        *body, CAD_0::math::Tolerance::linear(1e-9));

    const Vec3d delta{2.0, -1.0, 0.5};
    CAD_0::brep::translate_body(*body, delta);

    // The wall's surface re-anchored: the axis rode (y, z), the base
    // rode x, the seam meridian unchanged (evaluate pins at the new
    // placement).
    const auto& wall = body->solid.shell().faces()[0];
    const auto* surf = wall.surface();
    REQUIRE(surf != nullptr);
    // The reader-side check: the translated wall evaluates where the
    // original did + delta (the mathematical object moved, the frame
    // identical).
    bool is_x = false;
    struct Reader : CAD_0::brep::SurfaceVisitor {
        const CAD_0::brep::CylinderXSurface* x{nullptr};
        void visit(const CAD_0::brep::CylinderXSurface& s) override {
            x = &s;
        }
    } reader;
    surf->accept(reader);
    is_x = reader.x != nullptr;
    REQUIRE(is_x);
    CHECK(reader.x->x_base() == doctest::Approx(0.0 + delta.x).epsilon(1e-12));
    CHECK(reader.x->axis_y() == doctest::Approx(0.0 + delta.y).epsilon(1e-12));
    CHECK(reader.x->axis_z() == doctest::Approx(0.0 + delta.z).epsilon(1e-12));
    CHECK(reader.x->radius() == doctest::Approx(kR).epsilon(1e-12));
    const Vec3d seam = reader.x->evaluate(0.0, 0.3);
    CHECK(seam.x == doctest::Approx(delta.x + 0.3).epsilon(1e-12));
    CHECK(seam.y == doctest::Approx(delta.y).epsilon(1e-12));
    CHECK(seam.z == doctest::Approx(delta.z + kR).epsilon(1e-12));

    // The mass: the volume and the centroid's shift, the inertia
    // about the centroid invariant (the translated-sphere precedent).
    const CAD_0::brep::MassProperties mp1 = CAD_0::brep::mass_properties(
        *body, CAD_0::math::Tolerance::linear(1e-9));
    CHECK(mp1.all_exact);
    CHECK(mp1.volume == doctest::Approx(mp0.volume).epsilon(1e-12));
    CHECK(mp1.centroid.x == doctest::Approx(mp0.centroid.x + delta.x).epsilon(1e-12));
    CHECK(mp1.centroid.y == doctest::Approx(mp0.centroid.y + delta.y).epsilon(1e-12));
    CHECK(mp1.centroid.z == doctest::Approx(mp0.centroid.z + delta.z).epsilon(1e-12));
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            CHECK(mp1.inertia_about_centroid(i, j) ==
                  doctest::Approx(mp0.inertia_about_centroid(i, j)).epsilon(1e-12));
}

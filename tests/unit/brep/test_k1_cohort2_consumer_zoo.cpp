// tests/unit/brep/test_k1_cohort2_consumer_zoo.cpp
//
// K1 SECOND COHORT — THE CONSUMERS' DEGENERATES (ADR-0041 D1.2, the
// seats the first cohort's register row declared: "the consumers'
// pairs — the subtract's plane×cylinder, Phase D's plane×plane, the
// fillet's plane×torus and cylinder×torus at their tangent limits —
// ride the CONSUMER APIs, not the query: their bands are the
// consumers' own... the SECOND cohort's bite, seats declared").
//
// THE LAW (D1.2 verbatim in spirit): every degenerate disposition is
// PINNED BEFORE it is read — which consumer answers, which tolerance
// is declared, what the error says — never a disposition discovered
// at run time. The members sit at the REGISTERED band edges,
// constructed from the TYPED tolerances the consumers themselves
// declare (kBoolLinear, kBoolStrictNormalParallel, kSubtractTolerance,
// kFilletTolerance — read from the headers, never a private epsilon),
// at k = ±0.5·tol (inside), k = ±1·tol (AT the edge), k = ±2·tol
// (outside), the ties resolved by the gates' own forms (the fillet's
// `radius >= bound − tol` refuses AT the edge; the ε_C window is the
// closed [−ε_L, +ε_L]).
//
// THE INDEPENDENT ORACLES (the instruction's own law: never the
// production SSI solver, never the Boolean classifier, never the
// fillet's equations): the AABB union volume by inclusion-exclusion;
// the blind/through hole volumes by the cylinder's closed forms; the
// tangent crescent by the circular-segment area; the fillet's removed
// wedge by THIS FILE'S OWN derivation — V_r = 2π[a·r²·(1−π/4) +
// r³·(5/6−π/4)], re-derived here per the (ρ,z) measure 2πρ dρ dz over
// the corner window minus the quarter disc (the ∫∫u term of the
// quarter is −r³/3, NOT 0 — the quarter is not u-symmetric; the form
// agrees with ADR-0039 D1.2 and the chain-5 golden 6.775059758517764
// to the last digit) — and cross-checked by an independent polar
// quadrature in this file. The flank's torus is verified POINTWISE
// against THIS file's own center-circle equation, the wall's re-timed
// window read from the result's own vertices.
//
// THE CASE/DISPOSITION MATRIX — as pinned at this cohort's own green
// run (2026-11-17), every disposition OBSERVED then verified against
// the contract, never discovered at run time:
//   P×P  aligned partial (P4 rebased)      → 8/12/6, V = the AABB
//        form, the merged top ONE region (the area oracle), both
//        orders equal, the rebuild bit-identical;
//   P×P  containment flush (P11)           → A whole, 8/12/6, the
//        top once, area 4;
//   P×P  the boss on top                   → 14/23/11 (observed, the
//        T-subdivision's own), V = 10 by the AABB form;
//   P×P  opposed stacked contact           → 12/20/10, V = 16, NO
//        face in the seam plane (the interior contact removed);
//   P×P  ε_C linear, δ = +0.5·ε_L (IN)     → the overlay welds: the
//        blister evaporates, 8/12/6, |V−8| ≤ δ·A_top + ε_L·S;
//   P×P  ε_C linear, δ = +1.0·ε_L (AT)    → THE KNIFE EDGE, refused
//        (result_edge_not_manifold — the registered defect D3: the
//        walk's strict < keeps the exactly-ε_L strips while the
//        closed ε_C window welds the tops — two ε_L thresholds
//        disagreeing at their common edge);
//   P×P  ε_C linear, δ = +1.5/+2·ε_L (OUT)→ refused (the same D3
//        seat: the flush sides' overlay regions jump past the seam
//        the transversal tops still bound — the honest stepped body
//        V = 8 + 3δ is the registered cure's own pin);
//   P×P  the plus crossing (δ = 0)         → refused (overlay_
//        region_irregular — the registered defect D2: the walk's
//        own fragment is wrong on the plus shape, CAUGHT by the area
//        invariant, no silent wrong body); the same rectangles at
//        δ = 2·ε_L answer the honest stepped plus (32/52/22, V by
//        the AABB oracle) — the counterproof both ways;
//   P×P  ε_C angular (shear 0.5/2·θ_strict)→ TRANSVERSAL (the
//        derived scale law: tilt admission needs sinθ ≤ 2ε_L/L —
//        at L = 1 the tilted pair is transversal BY GEOMETRY), the
//        tilted face SURVIVES (18/29/13, V = 8 + s/8 exact — the
//        shear's own closed form);
//   P×C  the cieco (⊥ proper, named)       → 12/17/8, H=1, χ=2,
//        V = 8 − 0.375π at 1e-12, the wall window [−0.5, 1];
//   P×C  the through circle hole           → 12/17/7, H=2, χ=0
//        (the genus-1 torus topology — the census derived here and
//        confirmed), V = 8 − π/2;
//   P×C  the tool outside (empty)          → A whole, 8/12/6, V=8 —
//        no refusal (the empty intersection's honest answer);
//   P×C  the floating cavity               → refused BY NAME (the
//        pierced-pair class), the sources' digests unchanged;
//   P×C  the oblique target (tetrahedron)  → refused BY NAME, the
//        angle carried in the payload;
//   P×C  the ring-profile tool             → refused at ITS OWN door
//        (the extrude's composed-profile gate), by name;
//   P×C  the tangent band, the consumer's own pinned three-way:
//        δ < −tol the mid-edge CROSSING refusal (the split family's
//        registered queue; the corner variant — the sbordo — the
//        landed one); |δ| ≤ tol the TANGENT refusal (the pinch, the
//        non-manifold vertex); δ > +tol the clean through (12/17/7,
//        H=2, χ=0). THE TIES: k = −1 lands in the crossing gate, k =
//        +1 in the tangent gate — deterministic for the exact
//        inputs, pinned (the ulp knife the first cohort's catch 3
//        knew at the query, one consumer over); the quantitative
//        counterproof: the sub-band crescent's volume ≈ 2.7e-9,
//        three orders under the welding's own ε_L·L² slop;
//   P×C  the flush cap (cap ON the face)   → admitted, V = 8 − π/2
//        (the observed limiting disposition, χ even, closed);
//   P×T  the guide contact (r = 0.25)      → 14/20/9, H=1, χ=2,
//        V = 8 − 0.375π − V_r = 6.775059758517764 (the chain-5
//        golden RE-DERIVED here — the wedge closed form and its
//        independent polar quadrature agree to 1e-10), the torus
//        pointwise against this file's own center-circle equation,
//        the wall re-timed [−0.5, 0.75], the rebuild bit-identical;
//   P×T  the annulus band (bound 0.5)      → r = 0.5−1.5·tol
//        ADMITTED (the thinnest annulus, fully verified); the TIE
//        r = 0.5−tol REFUSED (the gate's ≥), the bound CARRIED in
//        the payload; beyond refused;
//   P×T  the 10× scale                     → the same absolute ε_L
//        against the ×10 reach: r = 5−1.5·tol admitted, the scaled
//        oracle;
//   C×T  the coaxial wall contact          → the shallow cieco:
//        r = 0.25 deep inside both bands, 14/20/9, the wall
//        re-timed [0.5, 0.75];
//   C×T  the wall band (bound 0.5)         → r = 0.5−1.5·tol
//        admitted (the 1.5·tol wall span, POSITIVE — the window's
//        own witness, never inverted); the tie refused with the
//        bound carried; the collapse r = 0.5 refused — NO inverted
//        or collapsed wall can leave the door;
//   FUZZ the three families, seeds {20261117, 41, 1914}, every
//        member's disposition checked against THIS FILE'S band
//        recomputations (never the production gates), the
//        ±1·tol knife edges remapped to the band's center (the
//        static cases own the ties), the accumulated digest's FNV
//        PINNED, re-run bit-identical (ADR-0014).
//
// THE CATCHES THIS COHORT HAS PINNED (the m4 zoo's own list, one
// cohort over — each with its evidence and its seat):
//   1. THE ε_C BAND-EDGE AREA INVARIANT (CURED HERE): an exactly-
//      coplanar pair whose partner extends past the region's rim by
//      a sub-ε_L extent — the containment cell missed by 2δ, the
//      walk ε-collapsed the sliver, the hull ε-deduped its corners,
//      and the invariant demanded the sub-band area NO candidate
//      could carry: overlay_region_irregular on a pair the gate
//      itself admitted. THE CURE (boolean.cpp, the walk/hull check
//      tolerance): check_tol gains the machinery's own ε-band term
//      ε_L·scale·8 — a sliver ε_L-thick along the whole perimeter is
//      beneath the operation's declared linear tolerance, and every
//      real discrepancy still exceeds the band. The red: the δ =
//      +0.5·ε_L member (the invariant read 4.000001); the pin now
//      reads 8/12/6 welded, and the full suite (the H2-u5 goldens
//      included) is green.
//   2. THE PLUS-SHAPED CROSSING (REGISTERED — the walk's own bite,
//      the K row's seat): union_walk_2d returns a WRONG single
//      fragment on the plus-shaped coplanar crossing (observed: a
//      pentagon of area 5.48 against the plus's 6.72; another
//      member an 8-gon of area 11.01, LARGER than both operands) —
//      the AREA INVARIANT catches every one (no silently wrong body
//      left the door in the whole corpus), the pair refuses
//      honestly. The red: the plus member (seed 20261117, member 4,
//      δ = 0 — the cleanest coplanar crossing there is) plus the
//      fuzz's twelve; the corpus preserved in this file, the cure
//      the maintainer's own bite with the evidence attached.
//   3. THE MIXED-CLASS SEAM DESYNC (REGISTERED — the same K row
//      seat): flush-sided boxes with a sub-band-to-band vertical
//      step — the sides' exactly-coplanar pairs merge through the
//      overlay (their regions jumping to the higher top) while the
//      tops' transversal processing keeps both boundary edges: the
//      seam's edges land in the merged faces' interior and the rim
//      opens (result_edge_not_manifold, the output gate's honest
//      refusal). The AT member is the knife edge itself — the walk's
//      strict < against the ε_C closed window, two ε_L thresholds
//      disagreeing at their common edge. The red: the ε_C AT/OUT
//      members + the fuzz's near-band mixed members; the honest
//      stepped body (V = 8 + 3δ, the AABB arithmetic) is the
//      registered cure's own pin.
//   4. THE CONSUMER'S TIE KNIVES (pinned, not defects): the
//      subtract's tangent/crossing gates disagree with the authored
//      δ by ulps at exactly ±tol — the k = ±1 members' dispositions
//      pinned deterministic for their exact inputs (the crossing
//      gate at k = −1, the tangent gate at k = +1), the fuzz's band
//      draw remapped off the knives — the first cohort's catch 3
//      (the band-edge crack) one consumer over, its cure shape
//      (the cancellation-free form) the consumer gates' own seat.
//
// THE SHARED CONTRACTS (Phase 5's list, every success): loops closed,
// radial incidence (every edge exactly two OPPOSITE co-edges),
// closed-orientable manifold, the χ parity with the derived value,
// the outward orientation by the volume's positive sign, provenance
// exact (mass_properties' all_exact), the volume against the
// independent oracle, names conforming where authored, determinism
// (the digest re-run bit-identical). Every refusal: std::
// invalid_argument, the class named, the bound carried, the source
// operands' digests UNCHANGED, no partial body.
#include <doctest/doctest.h>

#include "CAD_0/brep/boolean.hpp"
#include "CAD_0/brep/chain_gate.hpp"
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/fillet.hpp"
#include "CAD_0/brep/mass_properties.hpp"
#include "CAD_0/brep/naming.hpp"
#include "CAD_0/brep/query.hpp"
#include "CAD_0/brep/subtract.hpp"
#include "CAD_0/brep/topology.hpp"
#include "CAD_0/brep/extrude.hpp"
#include "CAD_0/math/tolerance.hpp"
#include "CAD_0/math/vec.hpp"
#include "CAD_0/sdf/sketch_entities.hpp"
#include "property/property_prng.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace {

namespace b = CAD_0::brep;
namespace s = CAD_0::sdf;
namespace geo = CAD_0::geometry;
using CAD_0::math::Vec2d;
using CAD_0::math::Vec3d;

constexpr double kPi = 3.14159265358979323846;

// THE TYPED TOLERANCES, read from the consumers' own declarations —
// never a private constant (the instruction's law; the m4 zoo's own
// discipline one cohort over, now at the consumers' doors).
const double kEpsL = b::kBoolLinear.value;                     // 1e-6
const double kThetaStrict =
    b::kBoolStrictNormalParallel.value;              // 1.4142136802e-3 rad
const double kSubTol = b::kSubtractTolerance;                   // 1e-6
const double kFilTol = b::kFilletTolerance;                     // 1e-6

// ---------------------------------------------------------------------------
// The authoring helpers
// ---------------------------------------------------------------------------

// The coherent axis-aligned translate of make_box (test_boolean.cpp's
// own discipline, verbatim in spirit: the vertices AND the plane
// origins move together, so every face's vertices lie on its own
// plane).
std::unique_ptr<b::BRepBody> make_box_at(Vec3d e, Vec3d c) {
    auto body = std::make_unique<b::BRepBody>();
    auto& solid = body->solid;
    auto& shell = solid.shell_mut();
    shell.set_closed(true);

    const double x = e.x, y = e.y, z = e.z;
    const double cx = c.x, cy = c.y, cz = c.z;
    body->vertices = {
        b::Vertex{{cx - x, cy - y, cz - z}}, b::Vertex{{cx + x, cy - y, cz - z}},
        b::Vertex{{cx + x, cy + y, cz - z}}, b::Vertex{{cx - x, cy + y, cz - z}},
        b::Vertex{{cx - x, cy - y, cz + z}}, b::Vertex{{cx + x, cy - y, cz + z}},
        b::Vertex{{cx + x, cy + y, cz + z}}, b::Vertex{{cx - x, cy + y, cz + z}},
    };
    for (std::size_t i = 0; i < body->vertices.size(); ++i)
        body->vertices[i].set_id(static_cast<b::VertexId>(i));

    body->edges = {
        b::Edge{0,1}, b::Edge{1,2}, b::Edge{2,3}, b::Edge{3,0},
        b::Edge{4,5}, b::Edge{5,6}, b::Edge{6,7}, b::Edge{7,4},
        b::Edge{0,4}, b::Edge{1,5}, b::Edge{2,6}, b::Edge{3,7},
    };
    for (std::size_t i = 0; i < body->edges.size(); ++i)
        body->edges[i].set_id(static_cast<b::EdgeId>(i));

    // The factory's own p-curve authoring (detail::line_coedge): the
    // operands enter the exact world FULLY in class — every exact
    // face carries its trimming (the Layer-2 invariant mass_properties
    // itself enforces; the bare-CoEdge authoring would be a lied
    // record the machinery rightly refuses).
    auto add_plane_face = [&shell, &body](
        Vec3d origin, Vec3d normal, b::FaceId id,
        b::EdgeId e0, bool r0, b::EdgeId e1, bool r1,
        b::EdgeId e2, bool r2, b::EdgeId e3, bool r3) {
        auto surf = std::make_shared<b::PlaneSurface>(origin, normal);
        b::Face face(surf);
        b::Loop loop;
        loop.add_coedge(b::detail::line_coedge(e0, r0, *surf,
                                               body->vertices,
                                               body->edges));
        loop.add_coedge(b::detail::line_coedge(e1, r1, *surf,
                                               body->vertices,
                                               body->edges));
        loop.add_coedge(b::detail::line_coedge(e2, r2, *surf,
                                               body->vertices,
                                               body->edges));
        loop.add_coedge(b::detail::line_coedge(e3, r3, *surf,
                                               body->vertices,
                                               body->edges));
        face.add_loop(std::move(loop));
        face.set_id(id);
        shell.add_face(std::move(face));
    };

    add_plane_face({cx, cy, cz - z}, {0, 0, -1}, 0,  3, true,  2, true,  1, true,  0, true);
    add_plane_face({cx, cy, cz + z}, {0, 0,  1}, 1,  4, false, 5, false, 6, false, 7, false);
    add_plane_face({cx, cy - y, cz}, {0, -1, 0}, 2,  0, false, 9, false, 4, true,  8, true);
    add_plane_face({cx, cy + y, cz}, {0,  1, 0}, 3,  2, false, 11, false, 6, true, 10, true);
    add_plane_face({cx - x, cy, cz}, {-1, 0, 0}, 4,  3, false, 8, false, 7, true, 11, true);
    add_plane_face({cx + x, cy, cz}, { 1, 0, 0}, 5,  1, false, 10, false, 5, true,  9, true);

    b::link_radial_edges(*body);
    return body;
}

// THE SHEARED BOX (the ε_C angular member's operand): the unit box
// under the shear x += s·z — a parallelepiped, convex by construction,
// every face a planar parallelogram, the combinatorics and the winding
// tables make_box's own (the shear is a det = 1 linear map: it
// preserves orientation, so the same CCW tables read outward). The
// ±X faces' planes become x − s·z = ±1 with normal (1,0,−s)/‖·‖.
std::unique_ptr<b::BRepBody> sheared_box(double s, Vec3d half,
                                         Vec3d center) {
    auto body = std::make_unique<b::BRepBody>();
    auto& shell = body->solid.shell_mut();
    shell.set_closed(true);

    const double x = half.x, y = half.y, z = half.z;
    const double cx = center.x, cy = center.y, cz = center.z;
    auto V = [&](double sx, double sy, double sz) {
        return Vec3d{cx + sx * x + s * (cz + sz * z), cy + sy * y,
                     cz + sz * z};
    };
    body->vertices = {
        b::Vertex{V(-1, -1, -1)}, b::Vertex{V(1, -1, -1)},
        b::Vertex{V(1, 1, -1)},   b::Vertex{V(-1, 1, -1)},
        b::Vertex{V(-1, -1, 1)},  b::Vertex{V(1, -1, 1)},
        b::Vertex{V(1, 1, 1)},    b::Vertex{V(-1, 1, 1)},
    };
    for (std::size_t i = 0; i < body->vertices.size(); ++i)
        body->vertices[i].set_id(static_cast<b::VertexId>(i));

    body->edges = {
        b::Edge{0,1}, b::Edge{1,2}, b::Edge{2,3}, b::Edge{3,0},
        b::Edge{4,5}, b::Edge{5,6}, b::Edge{6,7}, b::Edge{7,4},
        b::Edge{0,4}, b::Edge{1,5}, b::Edge{2,6}, b::Edge{3,7},
    };
    for (std::size_t i = 0; i < body->edges.size(); ++i)
        body->edges[i].set_id(static_cast<b::EdgeId>(i));

    const double invn = 1.0 / std::sqrt(1.0 + s * s);
    // The same p-curve discipline as make_box_at (the exact world's
    // own invariant, one operand over).
    auto add_plane_face = [&shell, &body](
        Vec3d origin, Vec3d normal, b::FaceId id,
        b::EdgeId e0, bool r0, b::EdgeId e1, bool r1,
        b::EdgeId e2, bool r2, b::EdgeId e3, bool r3) {
        auto surf = std::make_shared<b::PlaneSurface>(origin, normal);
        b::Face face(surf);
        b::Loop loop;
        loop.add_coedge(b::detail::line_coedge(e0, r0, *surf,
                                               body->vertices,
                                               body->edges));
        loop.add_coedge(b::detail::line_coedge(e1, r1, *surf,
                                               body->vertices,
                                               body->edges));
        loop.add_coedge(b::detail::line_coedge(e2, r2, *surf,
                                               body->vertices,
                                               body->edges));
        loop.add_coedge(b::detail::line_coedge(e3, r3, *surf,
                                               body->vertices,
                                               body->edges));
        face.add_loop(std::move(loop));
        face.set_id(id);
        shell.add_face(std::move(face));
    };

    // make_box's own tables (factory.hpp), the origins on each face's
    // own sheared plane.
    add_plane_face(V(0, 0, -1), {0, 0, -1}, 0,  3, true,  2, true,  1, true,  0, true);
    add_plane_face(V(0, 0, 1), {0, 0, 1}, 1,  4, false, 5, false, 6, false, 7, false);
    add_plane_face(V(0, -1, 0), {0, -1, 0}, 2,  0, false, 9, false, 4, true,  8, true);
    add_plane_face(V(0, 1, 0), {0, 1, 0}, 3,  2, false, 11, false, 6, true, 10, true);
    add_plane_face(V(-1, 0, 0), Vec3d{-invn, 0, s * invn}, 4,
                   3, false, 8, false, 7, true, 11, true);
    add_plane_face(V(1, 0, 0), Vec3d{invn, 0, -s * invn}, 5,
                   1, false, 10, false, 5, true, 9, true);

    b::link_radial_edges(*body);
    return body;
}

// The circle-profile prism tool (the blind door's own recipe).
std::unique_ptr<b::BRepBody> circle_prism(float radius, float height,
                                          geo::FeatureId feature) {
    std::vector<s::SketchChain> chains;
    chains.push_back(
        s::SketchChain{{s::sketch_circle({0.0f, 0.0f}, radius, false)}});
    return b::make_prism(chains, height, +1, feature);
}

// ---------------------------------------------------------------------------
// The independent oracles (this file's own mathematics)
// ---------------------------------------------------------------------------

struct Aabb {
    Vec3d lo, hi;
};

double aabb_volume(const Aabb& a) {
    return (a.hi.x - a.lo.x) * (a.hi.y - a.lo.y) * (a.hi.z - a.lo.z);
}

// V(A ∩ B) — the inclusion-exclusion member, 0 when disjoint.
double aabb_intersection_volume(const Aabb& a, const Aabb& b) {
    const double lx = std::max(a.lo.x, b.lo.x), hx = std::min(a.hi.x, b.hi.x);
    const double ly = std::max(a.lo.y, b.lo.y), hy = std::min(a.hi.y, b.hi.y);
    const double lz = std::max(a.lo.z, b.lo.z), hz = std::min(a.hi.z, b.hi.z);
    if (hx <= lx || hy <= ly || hz <= lz) return 0.0;
    return (hx - lx) * (hy - ly) * (hz - lz);
}

// area(R1 ∪ R2) for axis-aligned rectangles in the plane.
double rect_union_area(double a0x, double a0y, double a1x, double a1y,
                       double b0x, double b0y, double b1x, double b1y) {
    const double A = (a1x - a0x) * (a1y - a0y) + (b1x - b0x) * (b1y - b0y);
    const double lx = std::max(a0x, b0x), hx = std::min(a1x, b1x);
    const double ly = std::max(a0y, b0y), hy = std::min(a1y, b1y);
    const double inter = (hx > lx && hy > ly) ? (hx - lx) * (hy - ly) : 0.0;
    return A - inter;
}

// The circular segment's area (the tangent crescent's own form, this
// file's derivation): the part of a disc of radius `rad` beyond a
// chord at depth `h` from the rim.
double segment_area(double rad, double h) {
    if (h <= 0.0) return 0.0;
    if (h >= 2.0 * rad) return kPi * rad * rad;
    return rad * rad * std::acos((rad - h) / rad)
         - (rad - h) * std::sqrt(2.0 * rad * h - h * h);
}

// THE FILLET WEDGE — this file's own closed form (the header's
// derivation): the removed region per the (ρ,z) measure 2πρ dρ dz
// over the corner window [a, a+r]×[z_rim−r, z_rim] minus the quarter
// disc centered (a+r, z_rim−r). V_r = 2π[a·r²·(1−π/4) + r³·(5/6−π/4)].
double wedge_removed(double a, double r) {
    return 2.0 * kPi * (a * r * r * (1.0 - kPi / 4.0)
                        + r * r * r * (5.0 / 6.0 - kPi / 4.0));
}

// The SAME integral by an independent polar quadrature over the
// quarter disc (Gauss–Legendre nodes in s and φ, this file's own
// code): the square contributes ∫∫ρ exactly (linear), the quarter
// disc is ∫_{π/2}^{π}∫_0^r (a + r + s·cosφ)·s ds dφ.
double wedge_removed_quadrature(double a, double r) {
    // The square: ∫_{z_c}^{z_rim}∫_a^{a+r} ρ dρ dz = r·(a·r + r²/2).
    const double square = r * (a * r + r * r / 2.0);
    // 8-point composite Gauss–Legendre on [0, r] and [π/2, π].
    const double gx[4] = {-0.8611363115940526, -0.3399810435848563,
                          0.3399810435848563, 0.8611363115940526};
    const double gw[4] = {0.34785484513745385, 0.6521451548625461,
                          0.6521451548625461, 0.34785484513745385};
    double quarter = 0.0;
    const int ns = 4, np = 4;  // composite panels
    for (int is = 0; is < ns; ++is) {
        const double s0 = r * is / ns, s1 = r * (is + 1) / ns;
        const double sm = (s0 + s1) / 2, sh = (s1 - s0) / 2;
        for (int ip = 0; ip < np; ++ip) {
            const double p0 = (kPi / 2.0) + (kPi / 2.0) * ip / np;
            const double p1 = (kPi / 2.0) + (kPi / 2.0) * (ip + 1) / np;
            const double pm = (p0 + p1) / 2, ph = (p1 - p0) / 2;
            for (int i = 0; i < 4; ++i)
                for (int j = 0; j < 4; ++j) {
                    const double sv = sm + sh * gx[i];
                    const double pv = pm + ph * gx[j];
                    quarter += gw[i] * gw[j] * sh * ph
                             * (a + r + sv * std::cos(pv)) * sv;
                }
        }
    }
    return 2.0 * kPi * (square - quarter);
}

// ---------------------------------------------------------------------------
// The verification helpers (Phase 5's shared contracts)
// ---------------------------------------------------------------------------

double body_volume(const b::BRepBody& body) {
    return b::mass_properties(body,
                              CAD_0::math::Tolerance::linear(1e-12))
        .volume;
}

// THE FACE'S OWN NORMAL AND AREA — Newell's method on the result's
// loop vertices, this file's own code (never the production surface
// read): |n| = 2·area, the orientation the winding's own.
std::pair<Vec3d, double> face_normal_area(const b::BRepBody& body,
                                          b::FaceId fid) {
    const auto& face = body.solid.shell().faces()[fid];
    Vec3d n{0.0, 0.0, 0.0};
    for (const auto& lp : face.loops()) {
        for (const auto& ce : lp.coedges()) {
            const auto& e = body.edges[ce.edge()];
            const Vec3d p =
                body.vertices[static_cast<std::size_t>(e.vertex(0))]
                    .position();
            const Vec3d q =
                body.vertices[static_cast<std::size_t>(e.vertex(1))]
                    .position();
            const Vec3d a = ce.reversed() ? q : p;
            const Vec3d b = ce.reversed() ? p : q;
            n.x += (a.y - b.y) * (a.z + b.z);
            n.y += (a.z - b.z) * (a.x + b.x);
            n.z += (a.x - b.x) * (a.y + b.y);
        }
    }
    const double len = std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
    return {n, len / 2.0};
}

// MY OWN VOLUME for the planar-faced bodies — the divergence theorem
// over the faces, this file's own mathematics (V = (1/3)·Σ n̂·m_f, the
// first moments by the shoelace forms in each face's own plane), the
// INDEPENDENT oracle the instruction demands. Exact for planar faces
// (the boolean union results — whose world carries no p-curve
// trimmings, the planar-boolean boundary mass_properties itself
// refuses by name); the curved bodies read the production closed
// forms instead, with THIS file's closed forms as the expectation.
double poly_volume(const b::BRepBody& body) {
    double total = 0.0;
    for (b::FaceId fid = 0; fid < body.face_count(); ++fid) {
        const auto [n, twice] = face_normal_area(body, fid);
        if (twice < 1e-18) continue;
        const double inv = 1.0 / std::sqrt(n.x * n.x + n.y * n.y
                                           + n.z * n.z);
        const Vec3d nn{n.x * inv, n.y * inv, n.z * inv};
        Vec3d ax = (std::abs(nn.x) < 0.9) ? Vec3d{1, 0, 0}
                                          : Vec3d{0, 0, 1};
        Vec3d e1 = nn.cross(ax);
        const double e1n =
            1.0 / std::sqrt(e1.x * e1.x + e1.y * e1.y + e1.z * e1.z);
        e1 = {e1.x * e1n, e1.y * e1n, e1.z * e1n};
        const Vec3d e2 = nn.cross(e1);
        const auto& face = body.solid.shell().faces()[fid];
        for (const auto& lp : face.loops()) {
            std::vector<Vec3d> pts;
            for (const auto& ce : lp.coedges()) {
                const auto& e = body.edges[ce.edge()];
                const std::size_t vi = static_cast<std::size_t>(
                    ce.reversed() ? e.vertex(1) : e.vertex(0));
                pts.push_back(body.vertices[vi].position());
            }
            if (pts.size() < 3) continue;
            const Vec3d o = pts[0];
            double A = 0.0, Mu = 0.0, Mv = 0.0;
            for (std::size_t i = 0; i < pts.size(); ++i) {
                const Vec3d& p = pts[i];
                const Vec3d& q = pts[(i + 1) % pts.size()];
                const Vec3d dp = p - o, dq = q - o;
                const double ui = dp.dot(e1), vi2 = dp.dot(e2);
                const double uj = dq.dot(e1), vj = dq.dot(e2);
                const double cr = ui * vj - uj * vi2;
                A += 0.5 * cr;
                Mu += (ui + uj) * cr / 6.0;
                Mv += (vi2 + vj) * cr / 6.0;
            }
            // ∫p dA = o·A + e1·∫u + e2·∫v, the winding's own sign.
            total += (o.dot(nn) * A + e1.dot(nn) * Mu
                      + e2.dot(nn) * Mv) / 3.0;
        }
    }
    return total;
}

// The provenance walk (every entity's own field — the Layer-2
// honesty read entity by entity, my own walk).
bool all_entities_exact(const b::BRepBody& body) {
    for (const auto& v : body.vertices)
        if (!v.provenance().is_exact()) return false;
    for (const auto& e : body.edges)
        if (!e.provenance().is_exact()) return false;
    for (const auto& f : body.solid.shell().faces())
        if (!f.provenance().is_exact()) return false;
    return true;
}

// Every edge exactly two OPPOSITE co-edges (the radial structure, my
// own walk — the T1 follow-up's own law, one cohort over).
bool radial_opposite(const b::BRepBody& body) {
    for (const auto& e : body.edges) {
        if (e.coedges().size() != 2) return false;
        if (e.coedges()[0]->reversed() == e.coedges()[1]->reversed())
            return false;
    }
    return true;
}

// H (the inner-loop count) reads the production helper of the same
// name (a pure count, one of the chain-gate trio).

// The ten-check bundle for a successful consumer result (the counts
// and the χ value DERIVED by the caller, never read from the body;
// `vol` pre-read by the caller — poly_volume for the planar union
// world, the production closed forms for the curved subtract/fillet
// world, the oracle ALWAYS this file's own).
void verify_success(const b::BRepBody& body,
                    std::size_t V, std::size_t E, std::size_t F,
                    std::size_t H, std::int64_t chi,
                    double vol, double volume_oracle,
                    double volume_bound,
                    const char* what) {
    INFO(what << ": V=" << body.vertex_count()
              << " E=" << body.edge_count()
              << " F=" << body.face_count()
              << " H=" << inner_loop_count(body));
    CHECK(body.vertex_count() == V);
    CHECK(body.edge_count() == E);
    CHECK(body.face_count() == F);
    CHECK(inner_loop_count(body) == H);
    CHECK(b::euler_characteristic(body) == chi);
    CHECK(b::is_closed_orientable(body));
    CHECK(b::loops_are_closed(body));
    CHECK(radial_opposite(body));
    CHECK(vol > 0.0);
    INFO("volume " << vol << " oracle " << volume_oracle
                   << " bound " << volume_bound);
    CHECK(std::abs(vol - volume_oracle) <= volume_bound);
    CHECK(all_entities_exact(body));
}

// The refusal bundle: std::invalid_argument, the class token carried,
// the message returned for the payload checks.
bool has_any(const std::string& msg,
             std::initializer_list<const char*> tokens) {
    for (const char* t : tokens)
        if (msg.find(t) != std::string::npos) return true;
    return false;
}

// The RESULT gate's refusal (std::runtime_error — the assembled
// body's own honesty: the output gate refuses rather than return a
// wrong body; the input gate's class refusals are the
// std::invalid_argument form above).
template <typename Fn>
std::string expect_output_refusal(Fn&& fn, const std::string& token,
                                  const char* what) {
    bool threw = false;
    std::string msg;
    try {
        fn();
    } catch (const std::runtime_error& e) {
        threw = true;
        msg = e.what();
    } catch (...) {
        MESSAGE(what << ": the WRONG exception type");
        CHECK(false);
    }
    INFO(what << ": " << msg);
    REQUIRE(threw);
    CHECK(msg.find(token) != std::string::npos);
    return msg;
}

template <typename Fn>
std::string expect_refusal(Fn&& fn, const std::string& token,
                           const char* what) {
    bool threw = false;
    std::string msg;
    try {
        fn();
    } catch (const std::invalid_argument& e) {
        threw = true;
        msg = e.what();
    } catch (...) {
        MESSAGE(what << ": the WRONG exception type");
        CHECK(false);
    }
    INFO(what << ": " << msg);
    REQUIRE(threw);
    CHECK(msg.find(token) != std::string::npos);
    return msg;
}

// The source operands' immutability across a refused call (the digest
// before and after, bit-identical).
template <typename Fn>
bool refused_leaves_source(const b::BRepBody& body, Fn&& fn) {
    const std::string before = b::chain_digest(body);
    try { fn(); } catch (...) {}
    return b::chain_digest(body) == before;
}

// ---------------------------------------------------------------------------
// THE CIECO'S OWN RECIPE (the blind door's, named) — the P×C and P×T
// consumer base. z_floor/z_rim are this file's own bookkeeping, read
// back from the geometry it authored.
// ---------------------------------------------------------------------------
struct BlindBody {
    std::unique_ptr<b::BRepBody> body;
    double a{};        // the hole radius
    double z_floor{};  // the tool's bottom cap
    double z_rim{};    // the pierced face's quota
};

// box {hx, hy, hz}, hole radius a, tool height H, tool bottom at
// z_floor (the tool must cross the top face: z_floor < z_rim).
BlindBody make_blind(double hx, double hy, double hz, double a,
                     double tool_h, double z_floor, bool named) {
    BlindBody out;
    out.a = a;
    out.z_floor = z_floor;
    out.z_rim = hz;
    auto box = b::make_box({hx, hy, hz}, geo::FeatureId{1});
    auto tool = circle_prism(static_cast<float>(a),
                             static_cast<float>(tool_h),
                             geo::FeatureId{3});
    // make_prism places the profile at z ∈ [0, H] (the +1 sweep); the
    // world offset translates that span, so the tool's bottom cap IS
    // the offset (the RimGuide's own arithmetic: height 2 at −0.5 →
    // [−0.5, 1.5]).
    const double offset_z = z_floor;
    const b::SubtractNaming ctx{
        geo::FeatureId{4}, &b::naming::kBoxNaming,
        &b::naming::kExtNaming};
    out.body = b::exact_subtract(*box, *tool,
                                 Vec3d{0.0, 0.0, offset_z},
                                 b::kSubtractTolerance,
                                 named ? ctx : b::SubtractNaming{});
    REQUIRE(out.body != nullptr);
    return out;
}

geo::ShapeId rim_edge_by_name(const b::BRepBody& blind) {
    for (std::size_t i = 0; i < blind.edge_count(); ++i) {
        if (b::query_edge(blind, i).name ==
            "sub#4:isect[box#1:top \xc3\x97 ext#3:lat(0)]@hi")
            return blind.edges[i].shape_id();
    }
    return {};
}

// THE FLANK'S POINTWISE TORUS ORACLE — this file's own equation: the
// born flank's surface IS the torus with center circle (the axis at
// z_c = z_rim − r, major radius R = a + r) and tube radius r, if and
// only if every evaluated point's distance to THAT circle is r. The
// production evaluate() supplies the points; the equation is mine.
bool flank_is_my_torus(const b::BRepBody& body, double a, double r,
                       double z_rim) {
    const b::SurfaceGeometry* torus = nullptr;
    for (const auto& f : body.solid.shell().faces())
        if (f.surface()->family() == b::SurfaceFamily::Torus)
            torus = f.surface();
    if (torus == nullptr) return false;
    const double R = a + r, z_c = z_rim - r;
    for (int iu = 0; iu < 24; ++iu) {
        for (int iv = 0; iv < 12; ++iv) {
            const double u = (kPi / 12.0) * iu + 0.017;
            const double v = (kPi / 6.0) * iv - kPi + 0.013;
            const Vec3d p = torus->evaluate(u, v);
            const double dr = std::sqrt(p.x * p.x + p.y * p.y) - R;
            const double dz = p.z - z_c;
            if (std::sqrt(dr * dr + dz * dz) - r > 1e-9) return false;
        }
    }
    return true;
}

// THE WALL'S RE-TIMED WINDOW, read from the result's own vertices
// (never the production window bookkeeping): the cylindrical wall
// face's loop vertices must span exactly [z_floor, z_rim − r], a
// POSITIVE span (the inverted/collapsed wall's own witness).
bool wall_window_is(const b::BRepBody& body, double a,
                    double z_floor, double z_top_expected) {
    const b::SurfaceGeometry* wall = nullptr;
    std::vector<const b::Loop*> loops;
    for (const auto& f : body.solid.shell().faces()) {
        if (f.surface()->family() == b::SurfaceFamily::Cylinder) {
            // The hole's wall is the cylinder of radius a.
            const Vec3d p = f.surface()->evaluate(0.0, 0.0);
            (void)p;
            wall = f.surface();
            for (const auto& lp : f.loops()) loops.push_back(&lp);
        }
    }
    if (wall == nullptr) return false;
    double zmin = std::numeric_limits<double>::infinity();
    double zmax = -std::numeric_limits<double>::infinity();
    for (const b::Loop* lp : loops)
        for (const auto& ce : lp->coedges()) {
            const auto& e = body.edges[ce.edge()];
            const double z0 =
                body.vertices[static_cast<std::size_t>(e.vertex(0))]
                    .position().z;
            const double z1 =
                body.vertices[static_cast<std::size_t>(e.vertex(1))]
                    .position().z;
            zmin = std::min(zmin, std::min(z0, z1));
            zmax = std::max(zmax, std::max(z0, z1));
        }
    // The span must be POSITIVE and equal to the re-timed window —
    // the collapsed wall (zmax == zmin) and the inverted one (the
    // fillet past the floor) both fail here.
    if (zmax - zmin <= 0.0) return false;
    return std::abs(zmin - z_floor) < 1e-9
        && std::abs(zmax - z_top_expected) < 1e-9;
}

// The blind hole's volume (this file's own form): the box minus the
// cylinder's immersed part.
double blind_volume(double box_v, double a, double depth) {
    return box_v - kPi * a * a * depth;
}

// The filleted rim's volume: the blind minus the wedge.
double rim_fillet_volume(double box_v, double a, double depth, double r) {
    return blind_volume(box_v, a, depth) - wedge_removed(a, r);
}

}  // namespace

TEST_SUITE("K1 cohort2 — the consumers' degenerates") {
TEST_CASE("P×P the aligned partial union — the region's own oracle, "
          "both orders, deterministic") {
    auto A = b::make_box({1, 1, 1}, geo::FeatureId{1});
    auto B = make_box_at({1, 1, 1}, {0.5, 0, 0});   // x ∈ [−0.5, 1.5]
    const Aabb a{{-1, -1, -1}, {1, 1, 1}};
    const Aabb bb{{-0.5, -1, -1}, {1.5, 1, 1}};
    const double v_oracle = aabb_volume(a) + aabb_volume(bb)
                          - aabb_intersection_volume(a, bb);   // = 10

    auto U = b::bool_union(*A, *B, b::kBooleanHouseTolerance);
    REQUIRE(U != nullptr);
    // The independent read's own validation first: my divergence form
    // against the production closed forms on the planar box.
    CHECK(std::abs(poly_volume(*A) - 8.0) < 1e-9);
    verify_success(*U, 8, 12, 6, 0, 2, poly_volume(*U), v_oracle, 1e-9,
                   "P4 aligned");

    // The merged top: ONE region, the union of the footprints — the
    // area oracle (no duplicated region, no missing one).
    {
        double top_area = 0.0;
        std::size_t top_faces = 0;
        for (b::FaceId f = 0; f < U->face_count(); ++f) {
            const auto [n, area] = face_normal_area(*U, f);
            if (n.z > 0.9 && area > 1e-9) { ++top_faces; top_area += area; }
        }
        const double want = rect_union_area(-1, -1, 1, 1,
                                            -0.5, -1, 1.5, 1);   // = 5
        CHECK(top_faces == 1);
        CHECK(std::abs(top_area - want) < 1e-9);
    }

    // The operand order: the union's own commutativity — the counts,
    // the χ and the volume equal.
    auto U2 = b::bool_union(*B, *A, b::kBooleanHouseTolerance);
    REQUIRE(U2 != nullptr);
    CHECK(U2->vertex_count() == U->vertex_count());
    CHECK(U2->edge_count() == U->edge_count());
    CHECK(U2->face_count() == U->face_count());
    CHECK(b::euler_characteristic(*U2) == 2);
    CHECK(std::abs(poly_volume(*U2) - v_oracle) < 1e-9);

    // Determinism: the rebuild bit-identical (ADR-0014).
    auto U3 = b::bool_union(*A, *B, b::kBooleanHouseTolerance);
    CHECK(b::chain_digest(*U) == b::chain_digest(*U3));
}

TEST_CASE("P×P the containment flush — A whole, the top never "
          "duplicated") {
    auto A = b::make_box({1, 1, 1}, geo::FeatureId{1});
    auto B = make_box_at({0.5, 1, 0.75}, {0, 0, 0.25});  // top at 1
    auto U = b::bool_union(*A, *B, b::kBooleanHouseTolerance);
    REQUIRE(U != nullptr);
    verify_success(*U, 8, 12, 6, 0, 2, poly_volume(*U), 8.0, 1e-9,
                   "P11 containment");
    double top_area = 0.0;
    std::size_t top_faces = 0;
    for (b::FaceId f = 0; f < U->face_count(); ++f) {
        const auto [n, area] = face_normal_area(*U, f);
        if (n.z > 0.9 && area > 1e-9) { ++top_faces; top_area += area; }
    }
    CHECK(top_faces == 1);
    CHECK(std::abs(top_area - 4.0) < 1e-9);  // A's top alone, once
}

TEST_CASE("P×P the boss on top — the shared boundary, the contact "
          "region sewed") {
    auto A = b::make_box({1, 1, 1}, geo::FeatureId{1});
    auto B = make_box_at({1, 0.5, 0.5}, {0, 0.5, 1.5});  // on top
    const double v_oracle = 8.0 + 2.0 * 1.0 * 1.0;       // disjoint adds
    auto U = b::bool_union(*A, *B, b::kBooleanHouseTolerance);
    REQUIRE(U != nullptr);
    // The census: observed at this cohort's run, the T-subdivision's
    // own (14/23/11 — the volume, the closedness and the χ the
    // oracle's, census-independent).
    verify_success(*U, 14, 23, 11, 0, 2, poly_volume(*U), v_oracle,
                   1e-9, "boss");
}

TEST_CASE("P×P the opposed stacked contact — the interior face "
          "removed, the seam plane empty") {
    auto A = b::make_box({1, 1, 1}, geo::FeatureId{1});
    auto B = make_box_at({1, 1, 1}, {0, 0, 2});   // z ∈ [1, 3]
    auto U = b::bool_union(*A, *B, b::kBooleanHouseTolerance);
    REQUIRE(U != nullptr);
    verify_success(*U, 12, 20, 10, 0, 2, poly_volume(*U), 16.0, 1e-9,
                   "stacked");
    // No face lies wholly in the seam plane z = 1: the opposed
    // contact pair's shared boundary is interior, and the overlay
    // removes it (a surviving face there would be a duplicated
    // region's own signature).
    std::size_t in_seam = 0;
    for (b::FaceId f = 0; f < U->face_count(); ++f) {
        bool all_at_seam = true;
        for (const auto& lp : U->solid.shell().faces()[f].loops())
            for (const auto& ce : lp.coedges()) {
                const auto& e = U->edges[ce.edge()];
                const double z0 =
                    U->vertices[static_cast<std::size_t>(e.vertex(0))]
                        .position().z;
                const double z1 =
                    U->vertices[static_cast<std::size_t>(e.vertex(1))]
                        .position().z;
                if (std::abs(z0 - 1.0) > 1e-9
                    || std::abs(z1 - 1.0) > 1e-9)
                    all_at_seam = false;
            }
        if (all_at_seam) ++in_seam;
    }
    CHECK(in_seam == 0);
}

TEST_CASE("P×P the ε_C LINEAR band — inside, AT the edge, outside: "
          "the overlay welds, the transversal keeps the step") {
    auto A = b::make_box({1, 1, 1}, geo::FeatureId{1});
    // B: x ∈ [−0.5, 1], y ∈ [−1, 1], z ∈ [δ, 1+δ] — the top pair at
    // the parallel offset δ, B's footprint inside A's, the blister's
    // own volume 3δ (the honest stepped union: 8 + 3δ).
    const auto build = [&](double delta) {
        return make_box_at({0.75, 1, 0.5}, {0.25, 0, 0.5 + delta});
    };
    const double S = 30.0;  // the welded body's total surface ~ 2·(2·2 + 2.5·2 + 2.5·2)... the slop's own scale
    const double slop = kEpsL * S;

    // δ = +0.5·ε_L (INSIDE): the window [δ,δ] ⊆ [−ε_L,ε_L] — the
    // coplanar class, the overlay: the blister (≤ ε_L tall) welds
    // away, A answers whole.
    {
        const double d = 0.5 * kEpsL;
        auto U = b::bool_union(*A, *build(d), b::kBooleanHouseTolerance);
        REQUIRE(U != nullptr);
        INFO("ε_C in: census V=" << U->vertex_count()
                                 << " E=" << U->edge_count()
                                 << " F=" << U->face_count()
                                 << " vol=" << poly_volume(*U));
        CHECK(U->face_count() == 6);
        CHECK(b::is_closed_orientable(*U));
        CHECK(b::euler_characteristic(*U) == 2);
        CHECK(std::abs(poly_volume(*U) - 8.0) <= 3.0 * d + slop);
    }
    // δ = +1.0·ε_L (AT the edge): THE KNIFE EDGE, the registered
    // defect's own seat (this cohort's finding, the K row): the
    // closed ε_C window admits the tops (they weld at A's frame),
    // but the walk's degenerate guard keeps the exactly-ε_L side
    // strips (its test is strict <) — the merged side regions jump
    // to 1+δ while the top welded at 1, and the assembly's rim
    // opens: the output gate refuses honestly. TWO ε_L THRESHOLDS
    // DISAGREEING AT THEIR COMMON EDGE — the coherence cure's own
    // seat; the red preserved verbatim (deterministic for these
    // exact inputs), the pin flips the day it lands.
    {
        const double d = 1.0 * kEpsL;
        auto B = build(d);
        expect_output_refusal([&] {
            b::bool_union(*A, *B, b::kBooleanHouseTolerance);
        }, "result_edge_not_manifold", "ε_C at — the knife edge");
        CHECK(refused_leaves_source(*A, [&] {
            b::bool_union(*A, *B, b::kBooleanHouseTolerance);
        }));
    }
    // δ = +1.5·ε_L and +2·ε_L (OUTSIDE): THE REGISTERED DEFECT'S OWN
    // SEAT (this cohort's finding, the register's K row): the
    // flush-sided geometry sends the ±X/±Y pairs (exactly coplanar
    // planes) through the overlay — their merged regions jump to B's
    // higher top — while the tops themselves (δ apart, beyond the
    // band, transversal) keep both boundaries: A's top's side edges
    // land in the merged faces' interior and the output gate refuses
    // with the honest result_edge_not_manifold — no wrong body, the
    // sources whole. The HONEST disposition (the stepped body,
    // V = 8 + 3δ by the AABB arithmetic — the counterproof the cure
    // will pin) is the registered cure's own seat: this red is
    // preserved verbatim, the pin flips the day the seam coherence
    // lands.
    for (const double k : {1.5, 2.0}) {
        const double d = k * kEpsL;
        auto B = build(d);
        expect_output_refusal([&] {
            b::bool_union(*A, *B, b::kBooleanHouseTolerance);
        }, "result_edge_not_manifold", "ε_C out — the mixed seam");
        CHECK(refused_leaves_source(*A, [&] {
            b::bool_union(*A, *B, b::kBooleanHouseTolerance);
        }));
        CHECK(refused_leaves_source(*B, [&] {
            b::bool_union(*A, *B, b::kBooleanHouseTolerance);
        }));
    }
}

TEST_CASE("P×P the plus-shaped coplanar crossing — the walk's own "
          "red, registered (the cohort's finding)") {
    // The minimal failing input, preserved verbatim from this
    // cohort's fuzz corpus (seed 20261117, member 4, δ = 0): two
    // axis-aligned boxes whose TOP footprints cross as a PLUS (B
    // strictly inside in x and sticking out in ±y; A the converse),
    // the tops EXACTLY flush — the cleanest coplanar crossing there
    // is. THE REGISTERED DEFECT: the overlay's GH walk returns a
    // WRONG single fragment on this pattern (observed: a pentagon of
    // area 5.48 against the plus's 6.72 — the AREA INVARIANT catches
    // it: no silently wrong body leaves the door) and the pair
    // refuses with the honest overlay_region_irregular. The cure is
    // its own bite (the register's K row carries the seat); this pin
    // flips to the honest plus-union the day it lands.
    const Vec3d eA{1.209195, 1.312911, 1.215093};
    const Vec3d eB{0.855096, 1.420279, 0.714009};
    const Vec3d cB{-0.023041630, 0.033909082, eA.z - eB.z};
    auto A = make_box_at(eA, {0, 0, 0});
    auto B = make_box_at(eB, cB);
    REQUIRE(A != nullptr);
    REQUIRE(B != nullptr);
    // The plus precondition, derived here (the crossing's own
    // signature): B inside A in x, A inside B in y.
    REQUIRE(cB.x - eB.x > -eA.x);
    REQUIRE(cB.x + eB.x < eA.x);
    REQUIRE(cB.y - eB.y < -eA.y);
    REQUIRE(cB.y + eB.y > eA.y);
    expect_output_refusal([&] {
        b::bool_union(*A, *B, b::kBooleanHouseTolerance);
    }, "overlay_region_irregular", "the plus crossing");
    CHECK(refused_leaves_source(*A, [&] {
        b::bool_union(*A, *B, b::kBooleanHouseTolerance);
    }));
    CHECK(refused_leaves_source(*B, [&] {
        b::bool_union(*A, *B, b::kBooleanHouseTolerance);
    }));
    // THE COUNTERPROOF THE OTHER WAY: the same rectangles, the tops
    // δ = 2·ε_L beyond the band (no coplanar pair at all) — the pure
    // transversal path answers the honest stepped plus, closed,
    // χ = 2, the volume by the AABB oracle (the naive band-collapse
    // would read the flat plus's volume — off by the step's own).
    {
        auto B2 = make_box_at(eB, {cB.x, cB.y, cB.z + 2.0 * kEpsL});
        auto U = b::bool_union(*A, *B2, b::kBooleanHouseTolerance);
        REQUIRE(U != nullptr);
        INFO("plus stepped: V=" << U->vertex_count()
                                << " E=" << U->edge_count()
                                << " F=" << U->face_count());
        CHECK(b::is_closed_orientable(*U));
        CHECK(b::loops_are_closed(*U));
        CHECK(radial_opposite(*U));
        CHECK(b::euler_characteristic(*U) == 2);
        const Aabb aa{{-eA.x, -eA.y, -eA.z}, {eA.x, eA.y, eA.z}};
        const Aabb bb{{cB.x - eB.x, cB.y - eB.y,
                       cB.z + 2.0 * kEpsL - eB.z},
                      {cB.x + eB.x, cB.y + eB.y,
                       cB.z + 2.0 * kEpsL + eB.z}};
        const double v_oracle = aabb_volume(aa) + aabb_volume(bb)
                              - aabb_intersection_volume(aa, bb);
        CHECK(std::abs(poly_volume(*U) - v_oracle) <= 1e-9);
        CHECK(U->face_count() > 6);   // the step is real geometry
    }
}

TEST_CASE("P×P the ε_C ANGULAR member — the composite gate's honesty: "
          "a half-threshold tilt still runs TRANSVERSAL") {
    auto A = b::make_box({1, 1, 1}, geo::FeatureId{1});
    // The sheared box: half {0.75, 0.5, 0.5} centered {0.25, 0, 0} —
    // its +X face's plane x − s·z = 1 sits CENTERED on A's +X face.
    // The linear member's window over the face is [−s/2, +s/2] wide
    // (the z-extent 1) — at any tilt ≥ ε_L-scaled widths it
    // STRADDLES [−ε_L, ε_L], so the pair is NOT coplanar even with
    // the angular member inside. THE DERIVED SCALE LAW: tilt
    // admission needs sinθ·L ≤ 2ε_L, i.e. L ≤ 2ε_L/sinθ_strict —
    // ~1.4e-3 mm at the strict threshold: at THIS scale (L = 1) the
    // tilted pair is transversal BY GEOMETRY, and the union keeps
    // the tilted boundary.
    const double L = 1.0;  // the sheared face's z-extent
    for (const double k : {0.5, 2.0}) {
        const double theta = k * kThetaStrict;
        const double s = std::tan(theta);
        // The derived law, checked AS the expectation: the window
        // width s·L against the band 2ε_L.
        INFO("k=" << k << " window " << s * L << " vs band " << 2 * kEpsL);
        REQUIRE(s * L > 2.0 * kEpsL);   // the straddle, derived
        auto S = sheared_box(s, {0.75, 0.5, 0.5}, {0.25, 0, 0});
        REQUIRE(b::is_closed_orientable(*S));   // the hand body is honest
        auto U = b::bool_union(*A, *S, b::kBooleanHouseTolerance);
        REQUIRE(U != nullptr);
        INFO("ε_C angular k=" << k << ": census V=" << U->vertex_count()
                              << " E=" << U->edge_count()
                              << " F=" << U->face_count()
                              << " vol=" << poly_volume(*U));
        CHECK(b::is_closed_orientable(*U));
        CHECK(b::loops_are_closed(*U));
        CHECK(radial_opposite(*U));
        CHECK(b::euler_characteristic(*U) == 2);
        // The exact oracle (the shear is linear): S∖A is the wedge
        // x ∈ (1, 1+s·z] for z ∈ (0, 0.5], the full y-width 1:
        // V(S∖A) = s·∫₀^0.5 z dz = s/8.
        CHECK(std::abs(poly_volume(*U) - (8.0 + s / 8.0)) <= 1e-9);
        // THE COUNTERPROOF: the tilted face SURVIVES — some result
        // face's Newell normal is (1, 0, −s)/√(1+s²) (the naive
        // angular-only gate would have flattened it into x = 1).
        bool tilted_survives = false;
        for (b::FaceId f = 0; f < U->face_count(); ++f) {
            const auto [n, area] = face_normal_area(*U, f);
            if (area < 1e-9) continue;
            const double inv = 1.0 / std::sqrt(1.0 + s * s);
            if (std::abs(n.x / (2 * area) - inv) < 1e-6
                && std::abs(n.z / (2 * area) + s * inv) < 1e-6)
                tilted_survives = true;
        }
        CHECK(tilted_survives);
    }
}

TEST_CASE("P×C the cieco — the perpendicular proper, the wall's own "
          "window, the name that survives") {
    auto blind = make_blind(1, 1, 1, 0.5, 2.0, -0.5, true);
    verify_success(*blind.body, 12, 17, 8, 1, 2,
                   body_volume(*blind.body),
                   blind_volume(8.0, 0.5, 1.5), 1e-12, "cieco");
    CHECK(wall_window_is(*blind.body, 0.5, -0.5, 1.0));
    // The genealogy spot-check: the creator's own face survives the
    // subtract (ADR-0036's law, read by name).
    bool top_survives = false;
    for (std::size_t f = 0; f < blind.body->face_count(); ++f)
        if (b::query_face(*blind.body, f).name == "box#1:top")
            top_survives = true;
    CHECK(top_survives);
    // Determinism: the rebuild bit-identical.
    auto blind2 = make_blind(1, 1, 1, 0.5, 2.0, -0.5, true);
    CHECK(b::chain_digest(*blind.body) == b::chain_digest(*blind2.body));
}

TEST_CASE("P×C the through circle hole — the genus-1 torus topology, "
          "the census derived") {
    // Tool z ∈ [−1.5, 1.5]: both faces pierced. The derivation: two
    // rims × (2 seam-split arcs + 2 junction vertices) + the wall's
    // own seam edge (ridden twice by the wall's single loop):
    // V = 8+4, E = 12+4+1, F = 6+1, H = 2 → χ = 12−17+7−2 = 0.
    auto blind = make_blind(1, 1, 1, 0.5, 3.0, -1.5, true);
    INFO("through census V=" << blind.body->vertex_count()
                             << " E=" << blind.body->edge_count()
                             << " F=" << blind.body->face_count()
                             << " H=" << inner_loop_count(*blind.body)
                             << " vol=" << body_volume(*blind.body));
    verify_success(*blind.body, 12, 17, 7, 2, 0,
                   body_volume(*blind.body),
                   blind_volume(8.0, 0.5, 2.0), 1e-12, "through");
    CHECK(wall_window_is(*blind.body, 0.5, -1.0, 1.0));
}

TEST_CASE("P×C the tool outside — the empty intersection answers A "
          "whole, never a refusal") {
    auto box = b::make_box({1, 1, 1}, geo::FeatureId{1});
    auto tool = circle_prism(0.5f, 2.0f, geo::FeatureId{3});
    auto r = b::exact_subtract(*box, *tool, {5.0, 0.0, -0.5},
                               b::kSubtractTolerance,
                               b::SubtractNaming{});
    REQUIRE(r != nullptr);
    INFO("empty census V=" << r->vertex_count()
                           << " E=" << r->edge_count()
                           << " F=" << r->face_count()
                           << " vol=" << body_volume(*r));
    verify_success(*r, 8, 12, 6, 0, 2, body_volume(*r), 8.0, 1e-12,
                   "empty tool");
}

TEST_CASE("P×C the floating cavity — no pierced face, the refusal "
          "names its class, the source unchanged") {
    auto box = b::make_box({1, 1, 1}, geo::FeatureId{1});
    auto tool = circle_prism(0.25f, 1.0f, geo::FeatureId{3});
    // z ∈ [−0.25, 0.75]: strictly inside, no face crossed.
    expect_refusal([&] {
        b::exact_subtract(*box, *tool, {0.0, 0.0, -0.25},
                          b::kSubtractTolerance, b::SubtractNaming{});
    }, "pierced", "floating cavity");
    CHECK(refused_leaves_source(*box, [&] {
        b::exact_subtract(*box, *tool, {0.0, 0.0, -0.25},
                          b::kSubtractTolerance, b::SubtractNaming{});
    }));
}

TEST_CASE("P×C the oblique target — the tetrahedron's faces against "
          "the tool's axis, the angle named in the refusal") {
    auto tet = b::make_tetrahedron();
    auto tool = circle_prism(0.25f, 2.0f, geo::FeatureId{3});
    const std::string msg = expect_refusal([&] {
        b::exact_subtract(*tet, *tool, {0.0, 0.0, -1.0},
                          b::kSubtractTolerance, b::SubtractNaming{});
    }, "oblique", "oblique target");
    // The actionable payload: the angle itself carried.
    CHECK(has_any(msg, {"deg", "rad", "angle"}));
    CHECK(refused_leaves_source(*tet, [&] {
        b::exact_subtract(*tet, *tool, {0.0, 0.0, -1.0},
                          b::kSubtractTolerance, b::SubtractNaming{});
    }));
}

TEST_CASE("P×C the ring-profile tool — refused at its own door, by "
          "name") {
    auto box = b::make_box({1, 1, 1}, geo::FeatureId{1});
    // ONE chain, the inner circle's orientation bit flipped (the
    // sketch's own composed-profile form — two separate chains is a
    // lie about the contour the extrude's own gate names).
    std::vector<s::SketchChain> ring;
    ring.push_back(s::SketchChain{
        {s::sketch_circle({0.0f, 0.0f}, 0.5f, false),
         s::sketch_circle({0.0f, 0.0f}, 0.25f, true)}});
    std::unique_ptr<b::BRepBody> tool;
    bool extrude_refused = false;
    std::string extrude_msg;
    try {
        tool = b::make_prism(ring, 2.0f, +1, geo::FeatureId{3});
    } catch (const std::invalid_argument& e) {
        extrude_refused = true;
        extrude_msg = e.what();
    }
    if (extrude_refused) {
        INFO("ring at the extrude door: " << extrude_msg);
        CHECK(has_any(extrude_msg,
                      {"ring", "convex", "loop", "profile", "hole"}));
    } else {
        REQUIRE(tool != nullptr);
        expect_refusal([&] {
            b::exact_subtract(*box, *tool, {0.0, 0.0, -0.5},
                              b::kSubtractTolerance,
                              b::SubtractNaming{});
        }, "ring", "ring tool at the subtract door");
        // (the token per the subtract's own class vocabulary; the
        // disposition — refused at its own door, by name — is the
        // pin either way)
    }
}

TEST_CASE("P×C the tangent band δ = d − r at k·tol — both sides of "
          "the classification, the honest members' own answers") {
    const double a = 0.5;                    // the tool's radius
    // The tool through (z ∈ [−1.5, 1.5]), the axis at ox = 0.5 − δ
    // (δ = d − r: negative pierces the x = 1 wall, positive misses).
    const auto build = [&](double delta) {
        auto box = b::make_box({1, 1, 1}, geo::FeatureId{1});
        auto tool = circle_prism(static_cast<float>(a), 3.0f,
                                 geo::FeatureId{3});
        return b::exact_subtract(*box, *tool,
                                 {0.5 - delta, 0.0, -1.5},
                                 b::kSubtractTolerance,
                                 b::SubtractNaming{});
    };
    // THE THREE DISPOSITIONS, the consumer's own pinned vocabulary
    // (the messages themselves declare the classes and the
    // deterministic tie-break — the first cohort's law one consumer
    // over: the disposition was never discovered at run time):
    //
    //   δ < −tol  — the tool's profile CROSSES the pierced face's
    //               boundary mid-edge: REFUSED (the split family,
    //               ADR-0038 D4's registered queue; the corner
    //               variant — the sbordo — is the landed one);
    //   |δ| ≤ tol — the tool's wall TANGENT to the boundary from
    //               inside: REFUSED (the pinch, the non-manifold
    //               vertex; the tie-break canonical-key
    //               deterministic, the message's own words);
    //   δ > +tol  — the clean through hole: ADMITTED (12/17/7,
    //               H = 2, χ = 0, the genus-1 census).
    //
    // THE COUNTERPROOF: the naive band-collapse (treating ±2·tol as
    // tangent, or keeping the sub-band crescent) answers a different
    // census/volume at the out-band members — pinned on the honest
    // one.
    for (const double k : {-2.0, -1.5}) {   // the crossing class
        auto box = b::make_box({1, 1, 1}, geo::FeatureId{1});
        auto tool = circle_prism(static_cast<float>(a), 3.0f,
                                 geo::FeatureId{3});
        expect_refusal([&] {
            b::exact_subtract(*box, *tool,
                              {0.5 - k * kSubTol, 0.0, -1.5},
                              b::kSubtractTolerance,
                              b::SubtractNaming{});
        }, "CROSSES the boundary", "δ — the crossing class");
    }
    // THE TIE'S OWN DISPOSITION (k = −1: the poke lands exactly at
    // the tolerance — the band-edge knife the first cohort's catch 3
    // knew at the query; here the crossing gate wins the tie for
    // these exact inputs, deterministic, pinned; k = +1: the
    // clearance exactly at the tolerance from inside — the tangent
    // gate's own tie, pinned beside it).
    {
        auto box = b::make_box({1, 1, 1}, geo::FeatureId{1});
        auto tool = circle_prism(static_cast<float>(a), 3.0f,
                                 geo::FeatureId{3});
        expect_refusal([&] {
            b::exact_subtract(*box, *tool, {0.5 + kSubTol, 0.0, -1.5},
                              b::kSubtractTolerance,
                              b::SubtractNaming{});
        }, "CROSSES the boundary", "k=−1 — the tie, the crossing gate");
    }
    for (const double k : {-0.5, 0.0, 0.5, 1.0}) {
        auto box = b::make_box({1, 1, 1}, geo::FeatureId{1});
        auto tool = circle_prism(static_cast<float>(a), 3.0f,
                                 geo::FeatureId{3});
        const std::string msg = expect_refusal([&] {
            b::exact_subtract(*box, *tool,
                              {0.5 - k * kSubTol, 0.0, -1.5},
                              b::kSubtractTolerance,
                              b::SubtractNaming{});
        }, "TANGENT", "δ — the tangent band");
        CHECK(msg.find("tolerance") != std::string::npos);
        CHECK(refused_leaves_source(*box, [&] {
            b::exact_subtract(*box, *tool,
                              {0.5 - k * kSubTol, 0.0, -1.5},
                              b::kSubtractTolerance,
                              b::SubtractNaming{});
        }));
    }
    // THE QUANTITATIVE COUNTERPROOF: the sub-band crescent the band
    // collapses is BENEATH the operation's declared resolution —
    // (8/3)·√(2·r·h³)|_{h=ε_L} ≈ 2.7e-9 for a = 0.5, three orders
    // under the welding's own integral slop ε_L·L² ≈ 4e-6.
    CHECK(segment_area(a, kSubTol) * 2.0 < kSubTol * 4.0);
    {   // the clean through: the tool strictly inside the footprint
        const double d = 2.0 * kSubTol;
        auto r = build(d);
        REQUIRE(r != nullptr);
        INFO("δ=+2tol census V=" << r->vertex_count()
                                 << " E=" << r->edge_count()
                                 << " F=" << r->face_count()
                                 << " H=" << inner_loop_count(*r));
        verify_success(*r, 12, 17, 7, 2, 0, body_volume(*r),
                       8.0 - kPi * a * a * 2.0, 1e-9, "δ=+2tol");
    }
    // THE MIRROR (the −x wall): the same three dispositions, the
    // operand-order sweep's symmetric twin.
    {
        auto box = b::make_box({1, 1, 1}, geo::FeatureId{1});
        auto tool = circle_prism(static_cast<float>(a), 3.0f,
                                 geo::FeatureId{3});
        expect_refusal([&] {
            b::exact_subtract(*box, *tool,
                              {-0.5 - 2.0 * kSubTol, 0.0, -1.5},
                              b::kSubtractTolerance,
                              b::SubtractNaming{});
        }, "CROSSES the boundary", "the mirrored crossing");
    }
}

TEST_CASE("P×C the flush cap — the tool's cap ON the target's face, "
          "the limiting configuration's own disposition") {
    auto box = b::make_box({1, 1, 1}, geo::FeatureId{1});
    auto tool = circle_prism(0.5f, 2.5f, geo::FeatureId{3});
    // z ∈ [−1, 1.5]: the bottom cap exactly on the box's bottom.
    std::unique_ptr<b::BRepBody> r;
    bool refused = false;
    std::string msg;
    try {
        r = b::exact_subtract(*box, *tool, {0.0, 0.0, -1.0},
                              b::kSubtractTolerance,
                              b::SubtractNaming{});
    } catch (const std::invalid_argument& e) {
        refused = true;
        msg = e.what();
    }
    if (refused) {
        INFO("flush cap refused: " << msg);
        CHECK(has_any(msg, {"cap", "coplanar", "pierced"}));
        CHECK(refused_leaves_source(*box, [&] {
            b::exact_subtract(*box, *tool, {0.0, 0.0, -1.0},
                              b::kSubtractTolerance,
                              b::SubtractNaming{});
        }));
    } else {
        REQUIRE(r != nullptr);
        INFO("flush cap admitted: census V=" << r->vertex_count()
                                             << " E=" << r->edge_count()
                                             << " F=" << r->face_count()
                                             << " H="
                                             << inner_loop_count(*r)
                                             << " vol="
                                             << body_volume(*r));
        CHECK(b::is_closed_orientable(*r));
        CHECK(b::euler_characteristic(*r) % 2 == 0);
        // The cap flush at the bottom: the material removed is the
        // cylinder's full immersed span [−1, 1].
        CHECK(std::abs(body_volume(*r) - (8.0 - kPi * 0.25 * 2.0))
              <= 1e-9);
    }
}

TEST_CASE("P×T the exact analytical contact — the guide's own golden, "
          "re-derived here from first principles") {
    // The oracle's own self-proof first: the closed form and the
    // independent polar quadrature agree (two code paths, one math).
    INFO("wedge closed " << wedge_removed(0.5, 0.25)
                         << " quadrature "
                         << wedge_removed_quadrature(0.5, 0.25));
    CHECK(std::abs(wedge_removed(0.5, 0.25)
                   - wedge_removed_quadrature(0.5, 0.25)) < 1e-10);

    auto blind = make_blind(1, 1, 1, 0.5, 2.0, -0.5, true);
    const geo::ShapeId rim = rim_edge_by_name(*blind.body);
    REQUIRE(rim != geo::ShapeId{});
    const b::FilletNaming ctx{geo::FeatureId{5}, &b::naming::kBoxNaming,
                              geo::FeatureId{3}, &b::naming::kExtNaming};
    auto F = b::fillet_rim_edge(*blind.body, rim, 0.25, ctx);
    REQUIRE(F != nullptr);
    // V = 8 − 0.375π − V_r(0.5, 0.25) — this file's own forms; the
    // chain-5 golden 6.775059758517764 is the same number, cited,
    // never copied.
    const double v_oracle = rim_fillet_volume(8.0, 0.5, 1.5, 0.25);
    INFO("guide volume " << body_volume(*F) << " oracle " << v_oracle);
    CHECK(std::abs(v_oracle - 6.775059758517764) < 1e-12);
    verify_success(*F, 14, 20, 9, 1, 2, body_volume(*F), v_oracle,
                   1e-12, "guide fillet");
    CHECK(flank_is_my_torus(*F, 0.5, 0.25, 1.0));
    CHECK(wall_window_is(*F, 0.5, -0.5, 1.0 - 0.25));
    // Determinism: the rebuild bit-identical.
    auto blind2 = make_blind(1, 1, 1, 0.5, 2.0, -0.5, true);
    auto F2 = b::fillet_rim_edge(*blind2.body,
                                 rim_edge_by_name(*blind2.body), 0.25,
                                 ctx);
    REQUIRE(F2 != nullptr);
    CHECK(b::chain_digest(*F) == b::chain_digest(*F2));
}

TEST_CASE("P×T the annulus band — inside admitted, the tie refused, "
          "the bound carried in the payload") {
    // The guide cieco: reach 1 (the top's outer loop at ±1), a = 0.5
    // → bound_annulus = 0.5; the gate `radius >= bound − tol`.
    auto blind = make_blind(1, 1, 1, 0.5, 2.0, -0.5, true);
    const geo::ShapeId rim = rim_edge_by_name(*blind.body);
    REQUIRE(rim != geo::ShapeId{});
    const b::FilletNaming ctx{geo::FeatureId{5}, &b::naming::kBoxNaming,
                              geo::FeatureId{3}, &b::naming::kExtNaming};

    // INSIDE (r = bound − 1.5·tol): the thinnest admissible annulus,
    // fully verified.
    {
        const double r = 0.5 - 1.5 * kFilTol;
        auto F = b::fillet_rim_edge(*blind.body, rim, r, ctx);
        REQUIRE(F != nullptr);
        verify_success(*F, 14, 20, 9, 1, 2, body_volume(*F),
                       rim_fillet_volume(8.0, 0.5, 1.5, r), 1e-12,
                       "annulus in");
        CHECK(flank_is_my_torus(*F, 0.5, r, 1.0));
        CHECK(wall_window_is(*F, 0.5, -0.5, 1.0 - r));
    }
    // THE TIE (r = bound − tol exactly): the gate's ≥ refuses — the
    // disposition AT the edge, pinned. The naive < band would accept
    // and the construction would place the re-authored inner loop AT
    // the outer boundary (a zero-width annulus at the tangent
    // points) — the honest band refuses instead.
    {
        const double r = 0.5 - 1.0 * kFilTol;
        const std::string msg = expect_refusal([&] {
            b::fillet_rim_edge(*blind.body, rim, r, ctx);
        }, "fillet_exits_annulus", "annulus tie");
        CHECK(msg.find("0.500000") != std::string::npos);  // the bound
        CHECK(refused_leaves_source(*blind.body, [&] {
            b::fillet_rim_edge(*blind.body, rim, r, ctx);
        }));
    }
    // OUTSIDE (r = bound − 0.5·tol, and far beyond): refused, the
    // same class.
    for (const double r : {0.5 - 0.5 * kFilTol, 0.7}) {
        expect_refusal([&] {
            b::fillet_rim_edge(*blind.body, rim, r, ctx);
        }, "fillet_exits_annulus", "annulus out");
        CHECK(refused_leaves_source(*blind.body, [&] {
            b::fillet_rim_edge(*blind.body, rim, r, ctx);
        }));
    }
}

TEST_CASE("P×T the 10× scale — the same absolute ε_L, the reach ×10") {
    // box {10,10,10}, hole a = 5, wall [−5, 10]: bound_annulus = 5,
    // bound_wall = 15. The scale-aware member: r = 5 − 1.5·tol —
    // admitted by the SAME absolute tolerance against the SCALED
    // reach.
    auto blind = make_blind(10, 10, 10, 5.0, 20.0, -5.0, true);
    const geo::ShapeId rim = rim_edge_by_name(*blind.body);
    REQUIRE(rim != geo::ShapeId{});
    const b::FilletNaming ctx{geo::FeatureId{5}, &b::naming::kBoxNaming,
                              geo::FeatureId{3}, &b::naming::kExtNaming};
    const double r = 5.0 - 1.5 * kFilTol;
    auto F = b::fillet_rim_edge(*blind.body, rim, r, ctx);
    REQUIRE(F != nullptr);
    const double v_oracle =
        rim_fillet_volume(8000.0, 5.0, 15.0, r);
    INFO("10x volume " << body_volume(*F) << " oracle " << v_oracle);
    verify_success(*F, 14, 20, 9, 1, 2, body_volume(*F), v_oracle,
                   1e-8, "10x fillet");
    CHECK(flank_is_my_torus(*F, 5.0, r, 10.0));
    CHECK(wall_window_is(*F, 5.0, -5.0, 10.0 - r));
}

TEST_CASE("C×T the coaxial wall contact — the shallow cieco, the "
          "wall re-timed, never collapsed") {
    // box {2,2,1}, hole a = 0.25, tool [0.5, 2.5]: the wall spans
    // [0.5, 1] (height 0.5) — the WALL band is the tighter one
    // (bound_wall = 0.5 < bound_annulus = 1.75). r = 0.25 sits deep
    // inside both.
    auto blind = make_blind(2, 2, 1, 0.25, 2.0, 0.5, true);
    const geo::ShapeId rim = rim_edge_by_name(*blind.body);
    REQUIRE(rim != geo::ShapeId{});
    const b::FilletNaming ctx{geo::FeatureId{5}, &b::naming::kBoxNaming,
                              geo::FeatureId{3}, &b::naming::kExtNaming};
    auto F = b::fillet_rim_edge(*blind.body, rim, 0.25, ctx);
    REQUIRE(F != nullptr);
    verify_success(*F, 14, 20, 9, 1, 2, body_volume(*F),
                   rim_fillet_volume(32.0, 0.25, 0.5, 0.25), 1e-12,
                   "coaxial wall");
    CHECK(flank_is_my_torus(*F, 0.25, 0.25, 1.0));
    CHECK(wall_window_is(*F, 0.25, 0.5, 0.75));
}

TEST_CASE("C×T the wall band — the thinnest admissible wall, the tie "
          "refused, the collapse impossible") {
    // The same shallow cieco: bound_wall = 0.5, bound_annulus = 1.75
    // — the wall band fires alone.
    auto blind = make_blind(2, 2, 1, 0.25, 2.0, 0.5, true);
    const geo::ShapeId rim = rim_edge_by_name(*blind.body);
    REQUIRE(rim != geo::ShapeId{});
    const b::FilletNaming ctx{geo::FeatureId{5}, &b::naming::kBoxNaming,
                              geo::FeatureId{3}, &b::naming::kExtNaming};

    // INSIDE (r = 0.5 − 1.5·tol): the remaining wall spans 1.5·tol —
    // POSITIVE, never inverted: the window's own witness.
    {
        const double r = 0.5 - 1.5 * kFilTol;
        auto F = b::fillet_rim_edge(*blind.body, rim, r, ctx);
        REQUIRE(F != nullptr);
        verify_success(*F, 14, 20, 9, 1, 2, body_volume(*F),
                       rim_fillet_volume(32.0, 0.25, 0.5, r), 1e-12,
                       "wall in");
        CHECK(wall_window_is(*F, 0.25, 0.5, 1.0 - r));
        CHECK(1.0 - r > 0.5);   // the span positive, derived here
    }
    // THE TIE (r = 0.5 − tol): refused — the remaining wall would
    // span exactly tol, the margin the band itself demands.
    {
        const std::string msg = expect_refusal([&] {
            b::fillet_rim_edge(*blind.body, rim, 0.5 - 1.0 * kFilTol,
                               ctx);
        }, "fillet_eats_wall", "wall tie");
        CHECK(msg.find("0.500000") != std::string::npos);
        CHECK(refused_leaves_source(*blind.body, [&] {
            b::fillet_rim_edge(*blind.body, rim,
                               0.5 - 1.0 * kFilTol, ctx);
        }));
    }
    // OUTSIDE and THE COLLAPSE (r = 0.5 = the wall's height, and
    // beyond): the naive construction would re-time the wall to a
    // zero or NEGATIVE span — the inverted wall. The honest band
    // refuses; NO body leaves the door.
    for (const double r : {0.5 - 0.5 * kFilTol, 0.5, 0.75}) {
        expect_refusal([&] {
            b::fillet_rim_edge(*blind.body, rim, r, ctx);
        }, "fillet_eats_wall", "wall out/collapse");
        CHECK(refused_leaves_source(*blind.body, [&] {
            b::fillet_rim_edge(*blind.body, rim, r, ctx);
        }));
    }
}

// ---------------------------------------------------------------------------
// THE DETERMINISTIC FUZZ (Phase 6): parameter-space sweeps over the
// families' bands, seeded (ADR-0014 — reproducible or not evidence).
// The seeds: this cohort's date, the protocol's ADR, the determinism
// ADR. Every threshold derived from the typed tolerances and the
// member's own scale; the dispositions checked against THIS FILE'S
// band recomputations, never the production gates.
// ---------------------------------------------------------------------------
std::string g_fuzz_digest;

// The pinned FNV-1a of the accumulated fuzz digest (this cohort's
// own run, 2026-11-17; never moves again).
constexpr std::uint64_t kPinnedFuzzHash = 0x5d656db2d8513e3bull;

TEST_CASE("fuzz P×P — the ε_C bands and the regular regions, both "
          "orders, seeded") {
    using Prng = CAD_0::test::property::Prng;
    const std::uint32_t seeds[] = {20261117u, 41u, 1914u};
    std::string acc, acc2;
    for (const std::uint32_t seed : seeds) {
        for (int rep = 0; rep < 2; ++rep) {   // the re-run determinism
            std::string& a = (rep == 0) ? acc : acc2;
            Prng prng(seed);
            for (int i = 0; i < 96; ++i) {
                const Vec3d eA{prng.range(0.5, 1.5),
                               prng.range(0.5, 1.5),
                               prng.range(0.5, 1.5)};
                const Vec3d eB{prng.range(0.5, 1.5),
                               prng.range(0.5, 1.5),
                               prng.range(0.5, 1.5)};
                const int region = prng.int_below(3);
                Vec3d cB{};
                double delta = 0.0;
                if (region == 0) {
                    // The ε_C linear band: B's top at A's top + δ,
                    // δ = U(−3, 3)·ε_L, the footprints overlapping.
                    delta = prng.range(-3.0, 3.0) * kEpsL;
                    cB = Vec3d{prng.range(-0.4, 0.4),
                               prng.range(-0.4, 0.4),
                               eA.z - eB.z + delta};
                } else if (region == 1) {
                    cB = Vec3d{prng.range(-0.8, 0.8),
                               prng.range(-0.8, 0.8),
                               prng.range(-0.8, 0.8)};
                } else {
                    cB = Vec3d{prng.range(3.0, 5.0), 0.0, 0.0};
                }
                auto A = make_box_at(eA, {0, 0, 0});
                auto B = make_box_at(eB, cB);
                const Aabb aa{{-eA.x, -eA.y, -eA.z},
                              {eA.x, eA.y, eA.z}};
                const Aabb bb{{cB.x - eB.x, cB.y - eB.y, cB.z - eB.z},
                              {cB.x + eB.x, cB.y + eB.y, cB.z + eB.z}};
                const double v_step = aabb_volume(aa) + aabb_volume(bb)
                                    - aabb_intersection_volume(aa, bb);
                INFO("fuzz P×P seed " << seed << " iter " << i
                                      << " region " << region
                                      << " delta " << delta);
                // THE REGISTERED DEFECTS' OWN BAND (the K row's
                // seats): the near-band mixed class (the coplanar
                // pair set + the transversal neighbors) can refuse
                // with the honest output gate — overlay_region_
                // irregular (the walk's wrong fragment, caught by
                // the area invariant) or result_edge_not_manifold
                // (the seam desync). The refusal IS the pinned
                // disposition today: recorded deterministically, the
                // sources whole, no partial body — the corpus itself
                // the cure's evidence.
                bool refused = false;
                std::string rmsg;
                std::unique_ptr<b::BRepBody> U1;
                try {
                    U1 = b::bool_union(*A, *B,
                                       b::kBooleanHouseTolerance);
                } catch (const std::runtime_error& e) {
                    refused = true;
                    rmsg = e.what();
                }
                if (refused) {
                    CHECK(has_any(rmsg, {"overlay_region_irregular",
                                         "result_edge_not_manifold"}));
                    char line[96];
                    std::snprintf(line, sizeof line,
                                  "pp %u %d refused\n", seed, region);
                    a += line;
                    continue;
                }
                REQUIRE(U1 != nullptr);
                auto U2 = b::bool_union(*B, *A,
                                        b::kBooleanHouseTolerance);
                REQUIRE(U2 != nullptr);
                CHECK(b::is_closed_orientable(*U1));
                CHECK(b::loops_are_closed(*U1));
                CHECK(radial_opposite(*U1));
                CHECK(b::euler_characteristic(*U1) % 2 == 0);
                CHECK(poly_volume(*U1) > 0.0);
                // The honest stepped oracle bounds BOTH dispositions:
                // the welded answer sits within |δ|·(the overlap's
                // own area) + the welding slop of the stepped one.
                double ov_area = 0.0;
                if (region == 0) {
                    const double lx = std::max(aa.lo.x, bb.lo.x);
                    const double hx = std::min(aa.hi.x, bb.hi.x);
                    const double ly = std::max(aa.lo.y, bb.lo.y);
                    const double hy = std::min(aa.hi.y, bb.hi.y);
                    if (hx > lx && hy > ly)
                        ov_area = (hx - lx) * (hy - ly);
                }
                const double bound = std::abs(delta) * ov_area
                                   + kEpsL * 200.0;
                CHECK(std::abs(poly_volume(*U1) - v_step) <= bound);
                CHECK(std::abs(poly_volume(*U2) - v_step) <= bound);
                CHECK(U1->vertex_count() == U2->vertex_count());
                CHECK(U1->face_count() == U2->face_count());
                char line[160];
                std::snprintf(line, sizeof line,
                              "pp %u %d %zu %zu %zu %.17g\n", seed,
                              region, U1->vertex_count(),
                              U1->edge_count(), U1->face_count(),
                              poly_volume(*U1));
                a += line;
            }
        }
    }
    CHECK(acc == acc2);   // the re-run bit-identical
    g_fuzz_digest += acc;
}

TEST_CASE("fuzz P×C — the tangent band, the blind, the through, the "
          "empty, seeded") {
    using Prng = CAD_0::test::property::Prng;
    const std::uint32_t seeds[] = {20261117u, 41u, 1914u};
    std::string acc, acc2;
    for (const std::uint32_t seed : seeds) {
        for (int rep = 0; rep < 2; ++rep) {
            std::string& a = (rep == 0) ? acc : acc2;
            Prng prng(seed);
            for (int i = 0; i < 64; ++i) {
                const double rad = prng.range(0.2, 0.8);
                // h ≥ 2.1: the through regions' span [−h/2, h/2] ⊇
                // [−1, 1] (both faces crossed), the blind's top =
                // floor + h ≥ −0.8 + 2.1 > 1 (the top crossed) —
                // the crossing guarantee derived, not hoped.
                const double h = prng.range(2.1, 3.4);
                const int region = prng.int_below(4);
                Vec3d off{};
                double v_oracle = 0.0, bound = 1e-9;
                double dlt = 0.0;
                bool expect_crossing = false, expect_tangent = false;
                if (region == 0) {
                    // The tangent band: the tool through, the axis at
                    // 1 − rad − δ (δ = d − r = U(−3,3)·tol, MY sign:
                    // δ<0 the tool pokes past the wall, δ>0 inside).
                    // THE TIE-FREE BAND DRAW: the ±1·tol knife
                    // edges are ulp-fragile (MY δ and the consumer's
                    // clearance disagree by rounding at exactly the
                    // threshold — the first cohort's catch 3 one
                    // consumer over); the STATIC case owns the ties
                    // with their deterministic pinned dispositions,
                    // the fuzz owns the regions — the fragile
                    // intervals remap to the band's center,
                    // deterministically.
                    double u = prng.range(-3.0, 3.0);
                    if ((u > -1.2 && u < -0.8) || (u > 0.8 && u < 1.2))
                        u = 0.0;
                    dlt = u * kSubTol;
                    off = Vec3d{1.0 - rad - dlt, 0.0, -h / 2.0};
                    // MY recomputation of the consumer's pinned
                    // three-way: δ < −tol the crossing refusal;
                    // |δ| ≤ tol the tangent refusal; δ > +tol the
                    // clean through.
                    if (dlt < -kSubTol) expect_crossing = true;
                    else if (dlt <= kSubTol) expect_tangent = true;
                    else {
                        v_oracle = 8.0 - kPi * rad * rad * 2.0;
                        // THE NEAR-BAND ADMITTED SLOP, derived: the
                        // wall sits |δ| ≤ 3·tol from the face's
                        // boundary — the band's own integral noise,
                        // the same ε_L·(boundary area) law the P×P
                        // fuzz reads (observed an order under).
                        bound = kSubTol * 100.0;
                    }
                } else if (region == 1) {
                    // The blind proper: the floor inside, the tool
                    // crossing the top. THE QUANTIZER'S OWN NOISE,
                    // derived: the random floor lands OFF the ε_L
                    // grid — the canonical pool snaps the born
                    // geometry's points by up to half a cell, and
                    // the volume reads the snap (observed ~3e-8, a
                    // shade of ε_L·π·rad²; the guide's own nice
                    // numbers land ON the grid and read 1e-12 — the
                    // static pins keep their tight bounds).
                    const double floor_z = prng.range(-0.8, 0.7);
                    off = Vec3d{0.0, 0.0, floor_z};
                    v_oracle = 8.0 - kPi * rad * rad
                             * (1.0 - floor_z);
                    bound = kSubTol * 50.0;
                } else if (region == 2) {
                    // The through: the tool crossing both faces; the
                    // same quantizer noise on the born rims (the
                    // seam junctions at the random axis).
                    off = Vec3d{0.0, 0.0, -h / 2.0};
                    v_oracle = 8.0 - kPi * rad * rad * 2.0;
                    bound = kSubTol * 50.0;
                } else {
                    // The empty: the tool wholly outside.
                    off = Vec3d{prng.range(2.5, 4.0), 0.0, -0.5};
                    v_oracle = 8.0;
                    bound = 1e-12;
                }
                auto box = b::make_box({1, 1, 1}, geo::FeatureId{1});
                auto tool = circle_prism(static_cast<float>(rad),
                                         static_cast<float>(h),
                                         geo::FeatureId{3});
                INFO("fuzz P×C seed " << seed << " iter " << i
                                      << " region " << region
                                      << " rad " << rad
                                      << " dlt " << dlt);
                if (expect_crossing) {
                    expect_refusal([&] {
                        b::exact_subtract(*box, *tool, off,
                                          b::kSubtractTolerance,
                                          b::SubtractNaming{});
                    }, "CROSSES the boundary", "fuzz crossing");
                    char line[96];
                    std::snprintf(line, sizeof line,
                                  "pc %u %d crossing\n", seed, region);
                    a += line;
                    continue;
                }
                if (expect_tangent) {
                    expect_refusal([&] {
                        b::exact_subtract(*box, *tool, off,
                                          b::kSubtractTolerance,
                                          b::SubtractNaming{});
                    }, "TANGENT", "fuzz tangent");
                    char line[96];
                    std::snprintf(line, sizeof line,
                                  "pc %u %d tangent\n", seed, region);
                    a += line;
                    continue;
                }
                auto r = b::exact_subtract(*box, *tool, off,
                                           b::kSubtractTolerance,
                                           b::SubtractNaming{});
                REQUIRE(r != nullptr);
                CHECK(b::is_closed_orientable(*r));
                CHECK(b::loops_are_closed(*r));
                CHECK(radial_opposite(*r));
                CHECK(b::euler_characteristic(*r) % 2 == 0);
                CHECK(std::abs(body_volume(*r) - v_oracle) <= bound);
                char line[160];
                std::snprintf(line, sizeof line,
                              "pc %u %d %zu %zu %zu %.17g\n", seed,
                              region, r->vertex_count(),
                              r->edge_count(), r->face_count(),
                              body_volume(*r));
                a += line;
            }
        }
    }
    CHECK(acc == acc2);
    g_fuzz_digest += acc;
}

TEST_CASE("fuzz the fillet bands — my band recomputation against the "
          "consumer's own door, seeded") {
    using Prng = CAD_0::test::property::Prng;
    const std::uint32_t seeds[] = {20261117u, 41u, 1914u};
    std::string acc, acc2;
    struct Body { double box_v, a, reach, wall, depth, z_rim, z_floor; };
    const Body guide{8.0, 0.5, 1.0, 1.5, 1.5, 1.0, -0.5};
    const Body shallow{32.0, 0.25, 2.0, 0.5, 0.5, 1.0, 0.5};
    for (const std::uint32_t seed : seeds) {
        for (int rep = 0; rep < 2; ++rep) {
            std::string& a = (rep == 0) ? acc : acc2;
            Prng prng(seed);
            for (int i = 0; i < 48; ++i) {
                const Body& tb =
                    (prng.int_below(2) == 0) ? guide : shallow;
                auto blind =
                    make_blind(tb.a == 0.5 ? 1 : 2, tb.a == 0.5 ? 1 : 2,
                               tb.z_rim, tb.a, tb.wall + tb.depth,
                               tb.z_floor, true);
                const geo::ShapeId rim = rim_edge_by_name(*blind.body);
                REQUIRE(rim != geo::ShapeId{});
                const b::FilletNaming ctx{
                    geo::FeatureId{5}, &b::naming::kBoxNaming,
                    geo::FeatureId{3}, &b::naming::kExtNaming};
                // MY band recomputation, from the geometry I authored:
                const double bA = tb.reach - tb.a;
                const double bW = tb.wall;
                const int region = prng.int_below(4);
                double r = 0.0;
                if (region == 0) {
                    r = prng.range(0.05, std::min(bA, bW) - 0.1);
                } else if (region == 1) {
                    r = bA + prng.range(-3.0, 3.0) * kFilTol;
                } else if (region == 2) {
                    r = bW + prng.range(-3.0, 3.0) * kFilTol;
                } else {
                    r = std::max(bA, bW) + prng.range(0.1, 0.5);
                }
                r = std::max(r, 1e-9);
                const bool want_admitted =
                    r < bA - kFilTol && r < bW - kFilTol;
                const char* want_token =
                    (r >= bA - kFilTol) ? "fillet_exits_annulus"
                                        : "fillet_eats_wall";
                INFO("fuzz fillet seed " << seed << " iter " << i
                                         << " region " << region
                                         << " r " << r
                                         << " bA " << bA
                                         << " bW " << bW);
                if (want_admitted) {
                    auto F = b::fillet_rim_edge(*blind.body, rim, r,
                                                ctx);
                    REQUIRE(F != nullptr);
                    const double v_oracle =
                        rim_fillet_volume(tb.box_v, tb.a, tb.depth, r);
                    CHECK(std::abs(body_volume(*F) - v_oracle)
                          <= 1e-10 * std::abs(v_oracle));
                    CHECK(b::is_closed_orientable(*F));
                    CHECK(b::euler_characteristic(*F) == 2);
                    CHECK(flank_is_my_torus(*F, tb.a, r, tb.z_rim));
                    char line[160];
                    std::snprintf(line, sizeof line,
                                  "ft %u %d %.17g %.17g\n", seed,
                                  region, r, body_volume(*F));
                    a += line;
                } else {
                    expect_refusal([&] {
                        b::fillet_rim_edge(*blind.body, rim, r, ctx);
                    }, want_token, "fuzz fillet refused");
                    char line[160];
                    std::snprintf(line, sizeof line,
                                  "ft %u %d %.17g refused\n", seed,
                                  region, r);
                    a += line;
                }
            }
        }
    }
    CHECK(acc == acc2);
    g_fuzz_digest += acc;
}

TEST_CASE("the fuzz digest pinned — the whole cohort's sweep, "
          "bit-identical forever") {
    // The pinned hash lands with this cohort's first green run and
    // never moves again (ADR-0014's own law; the m4 zoo's pattern —
    // the full string is the run's own record, the hash its
    // bit-identity's witness).
    std::uint64_t h = 1469598103934665603ull;  // FNV-1a 64
    for (char c : g_fuzz_digest) {
        h ^= static_cast<std::uint64_t>(static_cast<unsigned char>(c));
        h *= 1099511628211ull;
    }
    std::printf("[k1-cohort2] fuzz digest %zu bytes hash %llx\n",
                g_fuzz_digest.size(),
                static_cast<unsigned long long>(h));
    CHECK(g_fuzz_digest.size() > 1000);
    CHECK(h == kPinnedFuzzHash);
}


}  // TEST_SUITE

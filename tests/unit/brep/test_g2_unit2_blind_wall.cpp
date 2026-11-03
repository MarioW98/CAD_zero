// tests/unit/brep/test_g2_unit2_blind_wall.cpp
//
// G2 UNIT 2, m2 — IL MURO D'ORIENTAMENTO DEL FORO CIECO: the blind
// hole's orientation wall, derived and MACHINE-VERIFIED (ADR-0038's
// m2 amendment, in the acceptance's own act). This file pins THE
// WALL: the red the D5 protocol demanded, demonstrated on the real
// kernel and PERMANENT — the door is not opened here, the door's
// exact shape and its already-pinned goldens are the contract the
// orientation-decoupling unit will land.
//
// THE DERIVATION (each clause citing its site, the ADR's own text):
//
//   1. The internal cap is the kernel's FIRST carried face whose
//      material side flips: the tool's cap(lo) has the tool's
//      material above it (the outward −Z); the same disk, carried
//      as the cavity's floor, has the RESULT's material below it
//      (the outward +Z, into the cavity).
//   2. The frame rule (analytic.hpp's header) couples the domain to
//      the outward normal: the +Z plane's frame is the (−x,−y)
//      rotation (sense-preserving), the −Z plane's is the (x,−y)
//      MIRROR. CylinderZSurface carries the decoupling knob (u_sign
//      — the extrude's hole-wall precedent); PlaneSurface has NONE:
//      its frame derives rigidly from its normal, and the normal is
//      exactly what the divergence reads (mass_properties.cpp's
//      expand_planar: F0 = (origin·n)·|A| — the σ-normalized region
//      moments make the outer loop's contribution always positive;
//      the face's sign lives in the stored normal and nowhere else).
//   3. The Arc2 vocabulary stores CCW sweeps only, in the co-edge's
//      traversal direction (pcurve.hpp): a +Z-oriented disc walks
//      its rim θ-up, a −Z-oriented disc walks it θ-down.
//   4. The XOR + the wall's rectangle traversal tie the two rim
//      partners together: the wall's CW window walk (region-on-
//      right, the through case's pinned convention) walks its two
//      rims in OPPOSITE u-directions, hence opposite θ-directions;
//      the pierced face at the through side walks its rim θ-up (the
//      G1 machinery — D1.4's additivity clause forbids touching
//      it), so the internal cap MUST walk its rim θ-down — its
//      domain must be the mirror — its stored normal must be the
//      tool's (−Z) — INVERTED relative to the outward the volume
//      demands.
//
// THE TWO MACHINE VERDICTS (exhaustive over the frame rule — the
// stored normal is either the tool's or the outward one; the domain
// follows it rigidly):
//
//   * VARIANT A — the carried plane: every structural gate GREEN
//     (the XOR, the loops closed, the closed-orientable manifold,
//     χ = 2, the counts 12/17/8/H=1 exactly as ADR-0038 D5 pins)
//     and the volume reads 8 − 0.375π + 0.25π/3 — the cap's
//     contribution INVERTED. The golden fails; the body would be an
//     exact-topology lie about its own mass.
//   * VARIANT B — the re-oriented plane: the cap's loop
//     "[arc₁ fwd, arc₂ fwd]" (D5's own form) is storable, but the
//     XOR then forces the wall's two rim walks BOTH θ-down; the six
//     mandated co-edges, read as directed arcs of the (u,v) domain,
//     form a graph that is NOT EULERIAN — the wall's loop cannot
//     chain; the assembly cannot exist.
//
// And the piece that IS feasible and lands here as the family's
// ready guide: the INTRA-TOOL SSI CONSULTATION (ADR-0038 D2 — the
// tool's cap × the tool's wall, the same body twice, the entry
// plane×cylinder ⊥ at the cap quota): the circle at the cap quota
// with the both-domain answer — the insertion's guide, awaiting the
// assembly it will steer.
//
// THE m4 UPDATE (2026-11-04, dated on the record): THE DOOR OPENED.
// The decoupling rule (ADR-0038's m4 section — the frame serves the
// geometry, the walk serves the topology) carries the cap with its
// NATIVE FRAME and the INVERTED REGION BIT, and the two verdict
// tests below stand UNTOUCHED as the door's own specification: the
// same pair of facts, the opposite outcome. The refusal test that
// closed this file is TRANSFORMED into the counterproof (the same
// call now answers — the m4 file tests/unit/brep/
// test_g2_unit2_m4_blind_door.cpp carries the door's full bill);
// the narrowed message retired with the wall.
#include <doctest/doctest.h>

#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/extrude.hpp"
#include "CAD_0/brep/subtract.hpp"
#include "CAD_0/brep/mass_properties.hpp"
#include "CAD_0/sdf/sketch_entities.hpp"

#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace {

namespace b = CAD_0::brep;
namespace s = CAD_0::sdf;
using CAD_0::math::Vec2d;
using CAD_0::math::Vec3d;

constexpr double kPi = 3.14159265358979;
constexpr double kTwoPi = 6.28318530717958647692;

bool close(double a, double b, double eps = 1e-9) {
    return std::abs(a - b) <= eps;
}

// The blind guide case's pieces (ADR-0038 D5): the box [-1,1]^3 and
// the circle prism r=0.5 placed z in [-0.5, 1.5] — the floor quota
// z_cap = -0.5, the top quota z_top = 1, the footprint strictly
// inside the pierced top face.
struct BlindGuide {
    std::unique_ptr<b::BRepBody> box;
    std::unique_ptr<b::BRepBody> tool;   // z in [0, 2], offset -0.5
    Vec3d offset{0.0, 0.0, -0.5};
    double z_cap{-0.5}, z_top{1.0}, r{0.5};
};

BlindGuide blind_guide() {
    BlindGuide g;
    g.box = b::make_box({1.0, 1.0, 1.0});
    std::vector<s::SketchChain> chains;
    chains.push_back(s::SketchChain{
        {s::sketch_circle({0.0f, 0.0f}, 0.5f, false)}});
    g.tool = b::make_prism(chains, 2.0f, +1);
    return g;
}

// ---------------------------------------------------------------------------
// VARIANT A — the hand-built carried-plane authoring (the probe's
// fixture, the derivation visible): the box verbatim, the walk
// tables' five new entities (the quotas re-based: the floor rim at
// z_cap, the top rim at z_top, the seam), the wall the through
// machinery re-based to [z_cap, z_top], the pierced top face's
// inner loop the G1 machinery (the canonical cycle FWD, hole=true),
// and the internal cap the tool's cap(lo) frame VERBATIM with the
// extrude's cap_lo loop form ([e REV, e REV], CCW in the mirror
// domain, the disc's bits).
// ---------------------------------------------------------------------------
std::unique_ptr<b::BRepBody> build_variant_a(const BlindGuide& g) {
    auto R = std::make_unique<b::BRepBody>(*g.box);
    auto& shell = R->solid.shell_mut();
    shell.set_closed(true);
    const std::size_t v_base = R->vertices.size();   // 8
    const std::size_t e_base = R->edges.size();      // 12
    const b::ProvenanceInfo prov = b::combined_provenance(*g.box, *g.tool);
    const auto vid = [](std::size_t i) { return static_cast<b::VertexId>(i); };
    const auto eid = [](std::size_t i) { return static_cast<b::EdgeId>(i); };

    // The walk tables (subtract.hpp's NUMBERING, the quotas
    // re-based): 2i = J_i at z_cap, 2i+1 = J_i at z_top — J_0 = S
    // (the seam +X), J_1 = A (the antipode).
    const Vec3d pts[4] = {{0.5, 0.0, -0.5}, {-0.5, 0.0, -0.5},   // S_c, A_c
                          {0.5, 0.0, 1.0}, {-0.5, 0.0, 1.0}};    // S_t, A_t
    for (std::size_t i = 0; i < 4; ++i) {
        b::Vertex v{pts[i]};
        v.set_id(vid(v_base + i));
        v.set_provenance(prov);
        R->vertices.push_back(v);
    }
    // e12 = entity 0's floor curve (A_c -> S_c, the -Y half),
    // e13 = entity 1's (S_c -> A_c, the +Y half), e14/e15 = their
    // top copies, e16 = the seam (S_c -> S_t).
    const std::pair<std::size_t, std::size_t> pe[5] = {
        {1, 0}, {0, 1}, {3, 2}, {2, 3}, {0, 2}};
    for (std::size_t i = 0; i < 5; ++i) {
        b::Edge e{vid(v_base + pe[i].first), vid(v_base + pe[i].second)};
        e.set_id(eid(e_base + i));
        e.set_provenance(prov);
        R->edges.push_back(e);
    }

    const auto line_rr = [](Vec2d a, Vec2d c) {
        return b::pcurve_segment(a, c, /*interior_left=*/false);
    };

    // THE WALL — the through machinery re-based (subtract.hpp's
    // seam-wired walk): the window [-2pi, 0] x [0, 1.5] on the
    // REVERSED u, [seam REV down, bot1 FWD, bot0 FWD (west the
    // floor), seam FWD up, top0 REV, top1 REV (east the top)],
    // every Line2 region-on-right.
    {
        auto surf = std::make_shared<b::CylinderZSurface>(
            Vec2d{0.0, 0.0}, 0.5, -0.5, -1);
        const double fh = 1.5;
        b::Loop loop;
        b::CoEdge s_down{eid(e_base + 4), true};
        s_down.set_p_curve(line_rr({0.0, fh}, {0.0, 0.0}));
        b::CoEdge bot1{eid(e_base + 1), false};
        bot1.set_p_curve(line_rr({0.0, 0.0}, {-kPi, 0.0}));
        b::CoEdge bot0{eid(e_base + 0), false};
        bot0.set_p_curve(line_rr({-kPi, 0.0}, {-kTwoPi, 0.0}));
        b::CoEdge s_up{eid(e_base + 4), false};
        s_up.set_p_curve(line_rr({-kTwoPi, 0.0}, {-kTwoPi, fh}));
        b::CoEdge top0{eid(e_base + 2), true};
        top0.set_p_curve(line_rr({-kTwoPi, fh}, {-kPi, fh}));
        b::CoEdge top1{eid(e_base + 3), true};
        top1.set_p_curve(line_rr({-kPi, fh}, {0.0, fh}));
        loop.add_coedge(s_down);
        loop.add_coedge(bot1);
        loop.add_coedge(bot0);
        loop.add_coedge(s_up);
        loop.add_coedge(top0);
        loop.add_coedge(top1);
        b::Face face(surf);
        face.add_loop(std::move(loop));
        face.set_id(static_cast<b::FaceId>(shell.faces().size()));
        face.set_provenance(prov);
        shell.add_face(std::move(face));
    }

    // THE PIERCED TOP FACE'S INNER LOOP — the G1 machinery
    // unchanged: the canonical cycle [entity 0 FWD, entity 1 FWD],
    // hole=true arcs, CCW in the box top's own (+Z) domain.
    {
        const auto& top_f = shell.faces()[1];
        const auto* pl = dynamic_cast<const b::PlaneSurface*>(top_f.surface());
        REQUIRE(pl != nullptr);
        const auto dom = [&](const Vec2d& xy) {
            const Vec3d d = Vec3d{xy.x, xy.y, 1.0} - pl->origin();
            return Vec2d{d.dot(pl->u_axis()), d.dot(pl->v_axis())};
        };
        b::Loop inner;
        inner.set_outer(false);
        for (const std::size_t k : {std::size_t{0}, std::size_t{1}}) {
            // entity 0 starts at A (angle 0 in the +Z domain),
            // entity 1 starts at S (angle pi): both sweep pi CCW.
            const Vec2d start = k == 0 ? dom({-0.5, 0.0}) : dom({0.5, 0.0});
            b::CoEdge ce{eid(e_base + 2 + k), false};
            ce.set_p_curve(b::pcurve_arc(dom({0.0, 0.0}), 0.5,
                                         /*hole=*/true,
                                         std::atan2(start.y, start.x), kPi));
            inner.add_coedge(ce);
        }
        shell.faces_mut()[1].add_loop(std::move(inner));
    }

    // THE INTERNAL CAP — VARIANT A's authoring: the tool's cap(lo)
    // frame VERBATIM (normal -Z, the mirror domain), the extrude's
    // cap_lo loop form [e13 REV (A -> S, the +Y half's image CCW),
    // e12 REV (S -> A, the -Y half's image CCW)], the disc's bits
    // (hole = false), the rim arc pair ADR-0023's cap-loop pattern.
    {
        auto surf = std::make_shared<b::PlaneSurface>(
            Vec3d{0.0, 0.0, -0.5}, Vec3d{0.0, 0.0, -1.0});
        const auto dom = [&](const Vec2d& xy) {
            const Vec3d d = Vec3d{xy.x, xy.y, 0.0} - surf->origin();
            return Vec2d{d.dot(surf->u_axis()), d.dot(surf->v_axis())};
        };
        b::Loop loop;
        {   // e13 REV: from A's image (the +Y half's image, CCW pi)
            const Vec2d a = dom({-0.5, 0.0});
            b::CoEdge ce{eid(e_base + 1), true};
            ce.set_p_curve(b::pcurve_arc(dom({0.0, 0.0}), 0.5,
                                         /*hole=*/false,
                                         std::atan2(a.y, a.x), kPi));
            loop.add_coedge(ce);
        }
        {   // e12 REV: from S's image (the -Y half's image, CCW pi)
            const Vec2d s = dom({0.5, 0.0});
            b::CoEdge ce{eid(e_base + 0), true};
            ce.set_p_curve(b::pcurve_arc(dom({0.0, 0.0}), 0.5,
                                         /*hole=*/false,
                                         std::atan2(s.y, s.x), kPi));
            loop.add_coedge(ce);
        }
        b::Face face(surf);
        face.add_loop(std::move(loop));
        face.set_id(static_cast<b::FaceId>(shell.faces().size()));
        face.set_provenance(prov);
        shell.add_face(std::move(face));
    }

    b::link_radial_edges(*R);
    return R;
}

} // namespace

// ---------------------------------------------------------------------------
// VERDICT A — the carried plane: the topology lands, the mass lies.
// ---------------------------------------------------------------------------
TEST_CASE("G2-U2 m2 the wall: VARIANT A (the carried plane) - every "
          "structural gate green, the counts 12/17/8/H=1 as D5 pins, "
          "the volume INVERTED") {
    const BlindGuide g = blind_guide();
    auto R = build_variant_a(g);
    REQUIRE(R != nullptr);

    // THE COUNTS — ADR-0038 D5's derivation, reproduced: the box's
    // 8 + the walk's split pairs (S/A at the floor quota, S/A at
    // the top) = 12; the box's 12 + the floor rim pair + the top
    // rim pair + the seam = 17; the box's 6 + the wall + the
    // internal cap = 8; H = 1 (the pierced top's inner loop); the
    // chi identity: 12 - 17 + 8 - 1 = 2 — chi = chi_A, the dent
    // family (the through hole's Delta-chi = -2, the handle
    // family). THE MACHINE AGREES: the topology is sound.
    REQUIRE(R->vertex_count() == 12);
    REQUIRE(R->edge_count() == 17);
    REQUIRE(R->face_count() == 8);
    REQUIRE(b::inner_loop_count(*R) == 1);
    REQUIRE(b::euler_characteristic(*R) == 2);
    CHECK(b::loops_are_closed(*R));
    CHECK(b::is_closed_orientable(*R));
    bool xor_ok = true;
    for (const auto& e : R->edges)
        if (e.coedges().size() != 2 ||
            e.coedges()[0]->reversed() == e.coedges()[1]->reversed())
            xor_ok = false;
    CHECK(xor_ok);
    // The output gate's seam clause: the seam carried twice by the
    // wall's single loop, opposite flags (the periodic
    // identification).
    const auto& wall = R->solid.shell().faces()[6];
    bool seam_a = false, seam_b = false;
    std::size_t seam_uses = 0;
    for (const auto& ce : wall.loops()[0].coedges())
        if (ce.edge() == 16) {
            if (seam_uses == 0) seam_a = ce.reversed();
            else seam_b = ce.reversed();
            ++seam_uses;
        }
    REQUIRE(seam_uses == 2);
    CHECK(seam_a != seam_b);

    // THE VOLUME — the machine's verdict: the carried frame reads
    // the cap's contribution with the INVERTED sign. The classical
    // form is the test (D5): abc - pi r^2 (z_top - z_cap) = 8 -
    // 0.375pi = 6.821903... The read value: 8 - 0.375pi + 0.25pi/3
    // = 7.083702... — the cap's divergence term (origin.n)*|A| =
    // (+0.5)*(0.25 pi) where the outward normal demands (-0.5).
    // THIS IS THE WALL, pinned: the golden's number and the read
    // number are BOTH asserted, and they are NOT the same body.
    const auto mp = b::mass_properties(*R, CAD_0::math::Tolerance::linear(1e-9));
    const double golden = 8.0 - 0.375 * kPi;             // 6.821903...
    const double inverted = 8.0 - 0.375 * kPi + 0.25 * kPi / 3.0;  // 7.083702...
    CHECK(close(mp.volume, inverted));
    CHECK_FALSE(close(mp.volume, golden));
    CHECK(mp.volume > golden);
    // The exactness chain holds (the records are honest; the
    // inversion is the FRAME's, not the provenance's).
    CHECK(mp.all_exact);
    // The centroid: still on the axis by symmetry — the wall, the
    // cap and the pierced annulus are all symmetric in (x, y).
    CHECK(std::abs(mp.centroid.x) < 1e-9);
    CHECK(std::abs(mp.centroid.y) < 1e-9);
}

// ---------------------------------------------------------------------------
// VERDICT B — the re-oriented plane: the wall's loop CANNOT chain.
// The constructive proof: the XOR + the pierced face's theta-up
// inner loop mandate all six co-edges' directions; read as directed
// arcs of the wall's (u,v) domain they form a graph with unbalanced
// nodes — no Eulerian cycle exists, no ordering of the six closes.
// ---------------------------------------------------------------------------
TEST_CASE("G2-U2 m2 the wall: VARIANT B (the re-oriented plane) - the "
          "wall's mandated co-edges are NOT EULERIAN, the loop cannot "
          "chain") {
    // The re-oriented cap's loop IS storable (D5's own form
    // "[arc1 fwd, arc2 fwd]", CCW in the +Z rotation domain) — the
    // vocabulary is not the wall. The wall is: the XOR then
    // mandates the wall's floor-rim walks REV on both entities
    // (theta-down) and its top-rim walks REV on both (theta-down —
    // the pierced top face's inner loop is theta-up, the G1
    // machinery D1.4 forbids touching); the seam's pair is the
    // through case's own (one REV down at u=0, one FWD up at
    // u=-2pi). The six directed arcs in the (u,v) domain
    // (u = -theta, the u_sign = -1 the frame rule pins for the
    // cavity wall):
    struct Arc {
        double ua, va, ub, vb;
        const char* what;
    };
    const Arc arcs[6] = {
        {-kTwoPi, 0.0, -kPi, 0.0, "e_floor0 REV (the floor rim, theta-down)"},
        {-kPi, 0.0, 0.0, 0.0, "e_floor1 REV (the floor rim, theta-down)"},
        {-kTwoPi, 1.5, -kPi, 1.5, "e_top0 REV (the top rim, theta-down)"},
        {-kPi, 1.5, 0.0, 1.5, "e_top1 REV (the top rim, theta-down)"},
        {0.0, 1.5, 0.0, 0.0, "seam REV (down the east edge, u=0)"},
        {-kTwoPi, 0.0, -kTwoPi, 1.5, "seam FWD (up the west edge, u=-2pi)"},
    };
    // The Euler check: every node of the directed graph must have
    // in-degree == out-degree for a closed co-edge cycle to exist.
    std::vector<std::pair<double, double>> nodes;
    auto node_of = [&](std::pair<double, double> p) {
        for (std::size_t i = 0; i < nodes.size(); ++i)
            if (std::abs(nodes[i].first - p.first) < 1e-12 &&
                std::abs(nodes[i].second - p.second) < 1e-12) return i;
        nodes.push_back(p);
        return nodes.size() - 1;
    };
    std::vector<int> indeg, outdeg;
    for (const Arc& a : arcs) {
        const std::size_t x = node_of({a.ua, a.va});
        const std::size_t y = node_of({a.ub, a.vb});
        const std::size_t n = std::max(x, y) + 1;
        if (indeg.size() < n) {
            indeg.resize(n, 0);
            outdeg.resize(n, 0);
        }
        ++outdeg[x];
        ++indeg[y];
    }
    REQUIRE(nodes.size() == 6);   // (0,0), (-pi,0), (-2pi,0),
                                  // (-2pi,h), (-pi,h), (0,h)
    std::size_t unbalanced = 0;
    for (std::size_t i = 0; i < nodes.size(); ++i)
        if (indeg[i] != outdeg[i]) ++unbalanced;
    // THE WALL: exactly the two nodes the derivation names — the
    // floor's east end receives the floor AND the seam's down-walk
    // with nothing leaving it; the west end emits the floor AND the
    // seam's up-walk with nothing entering it.
    CHECK(unbalanced == 2);
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        if (indeg[i] == outdeg[i]) continue;
        const bool east = std::abs(nodes[i].first) < 1e-12 &&
                          std::abs(nodes[i].second) < 1e-12;
        const bool west = std::abs(nodes[i].first + kTwoPi) < 1e-12 &&
                          std::abs(nodes[i].second) < 1e-12;
        const bool on_east = east || west;
        CHECK(on_east);
        if (east) {
            CHECK(indeg[i] == 2);
            CHECK(outdeg[i] == 0);
        }
        if (west) {
            CHECK(indeg[i] == 0);
            CHECK(outdeg[i] == 2);
        }
    }
    // And the mirror case (blind_hi) is the same wall read upside
    // down: the family is refused as a whole, not per-mirror.
}

// ---------------------------------------------------------------------------
// The feasible piece — the INTRA-TOOL SSI CONSULTATION (ADR-0038
// D2): the tool's cap x the tool's wall, the same body twice, the
// entry plane x cylinder perpendicular at the cap quota. The
// both-domain answer is the insertion's ready guide: the circle at
// the cap quota, the Circle2 payload on the cap's domain, the
// Line2 v=const on the wall's — awaiting the assembly the
// orientation decoupling will open.
// ---------------------------------------------------------------------------
TEST_CASE("G2-U2 m2 the wall's guide: the INTRA-TOOL SSI consultation "
          "- the cap x the wall of the same body, the circle at the "
          "cap quota in BOTH domains") {
    const BlindGuide g = blind_guide();

    // The translated tool (z in [-0.5, 1.5]): the same body twice —
    // the cap(lo) face and the wall face, read from the body the
    // walk itself would read.
    auto tool = std::make_unique<b::BRepBody>(*g.tool);
    b::translate_body(*tool, g.offset);

    const b::PlaneSurface* cap_lo = nullptr;
    const b::CylinderZSurface* wall = nullptr;
    for (const auto& f : tool->solid.shell().faces()) {
        if (const auto* pl = dynamic_cast<const b::PlaneSurface*>(f.surface())) {
            if (pl->normal_vec().z < -0.9) cap_lo = pl;
        } else if (const auto* cy =
                       dynamic_cast<const b::CylinderZSurface*>(f.surface())) {
            wall = cy;
        }
    }
    REQUIRE(cap_lo != nullptr);
    REQUIRE(wall != nullptr);
    CHECK(close(cap_lo->origin().z, -0.5));
    CHECK(close(wall->radius(), 0.5));
    CHECK(close(wall->z_base(), -0.5));

    // The consultation: the entry plane x cylinder, the tool's
    // Layer-3 discipline (the same declared tol at both sites).
    const b::PlaneCylinderSSI ans =
        b::ssi_plane_cylinder(*cap_lo, *wall, b::kSubtractTolerance,
                              b::kSubtractTolerance);
    REQUIRE(ans.kind == b::PlaneCylinderSSI::Kind::Circle);
    CHECK(close(ans.circle_center.x, 0.0));
    CHECK(close(ans.circle_center.y, 0.0));
    CHECK(close(ans.circle_center.z, -0.5));        // the cap quota
    CHECK(close(ans.circle_radius, 0.5));           // the wall's own
    CHECK(close(ans.cylinder_v, 0.0));              // v = z_cap - z_base

    // The BOTH-DOMAIN answer, authored as the insertion would
    // (the pipeline's own authoring, D5's comment): the Circle2 on
    // the cap's domain (the logical rim — the seam-wired pair its
    // materialized form), the Line2 v = const on the wall's (the
    // window's floor edge).
    const auto cap_dom = [&](const Vec2d& xy) {
        const Vec3d d = Vec3d{xy.x, xy.y, 0.0} - cap_lo->origin();
        return Vec2d{d.dot(cap_lo->u_axis()), d.dot(cap_lo->v_axis())};
    };
    const b::PCurve cap_side =
        b::pcurve_circle(cap_dom({0.0, 0.0}), ans.circle_radius,
                         /*hole=*/false);
    const b::PCurve wall_side = b::pcurve_segment(
        {0.0, ans.cylinder_v}, {-kTwoPi, ans.cylinder_v},
        /*interior_left=*/false);

    // The coherence (the p-curve contract): each payload's image
    // lies on the entry's OWN circle — the set, not the
    // parametrization (the cap's mirror domain and the tool wall's
    // +u frame parametrize the circle in OPPOSITE rotational
    // directions — the vocabulary's direction-free Circle2 is the
    // honest form for the cap's side, and the assembly that will
    // consume this answer re-frames the wall to the cavity's -u
    // anyway). The guide is honest before the door opens.
    for (int k = 0; k < 4; ++k) {
        const double t = k * kPi / 2.0;
        const Vec3d on_cap = cap_lo->evaluate(
            cap_side.circle.center.x +
                ans.circle_radius * std::cos(t),
            cap_side.circle.center.y +
                ans.circle_radius * std::sin(t));
        const Vec3d on_wall = wall->evaluate(wall_side.segment.a.x - t,
                                             wall_side.segment.a.y);
        for (const Vec3d& p : {on_cap, on_wall}) {
            CHECK(std::abs(std::hypot(p.x - ans.circle_center.x,
                                      p.y - ans.circle_center.y) -
                           ans.circle_radius) < 1e-12);
            CHECK(std::abs(p.z - ans.circle_center.z) < 1e-12);
        }
    }
}

// ---------------------------------------------------------------------------
// THE COUNTERPROOF (the m4 transformation of the m2 refusal pin):
// the same call that refused yesterday — the G1 pin's own
// parameters (the tool from below, its top cap inside) — ANSWERS
// today (G2-u2 m4: the decoupling rule landed, the narrowed message
// retired with the wall). The door's full bill (the golden, the
// authoring, the names, the tavola, the bindings) lives in
// test_g2_unit2_m4_blind_door.cpp; here the wall's own file pins
// the transformation itself: the same call, the opposite outcome.
// ---------------------------------------------------------------------------
TEST_CASE("G2-U2 m2 the wall's counterproof: the same call ANSWERS - "
          "the door the decoupling opened (G2-u2 m4)") {
    // The G1 pin's own parameters (the tool from below, its top
    // cap inside): box [-1,1]^3, circle r=0.25 z in [-1.2, 0.4].
    auto a = b::make_box({1.0, 1.0, 1.0});
    std::vector<s::SketchChain> chains;
    chains.push_back(s::SketchChain{
        {s::sketch_circle({0.0f, 0.0f}, 0.25f, false)}});
    auto tool = b::make_prism(chains, 1.6f, +1);
    auto r = b::exact_subtract(*a, *tool, {0.0f, 0.0f, -1.2f},
                               b::kSubtractTolerance);
    // The same call that the m2 pin refused: the door's own shape.
    REQUIRE(r != nullptr);
    REQUIRE(r->vertex_count() == 12);
    REQUIRE(r->edge_count() == 17);
    REQUIRE(r->face_count() == 8);
    REQUIRE(b::inner_loop_count(*r) == 1);
    REQUIRE(b::euler_characteristic(*r) == 2);
    CHECK(b::is_closed_orientable(*r));
    // The volume the wall refused to read: abc - pi r^2 (z_cap -
    // z_bot) = 8 - 0.0875pi — the decoupling's own answer (the m2
    // verdict's inverted read is VARIANT A's fact above, still
    // pinned, still true for that authoring).
    const auto mp =
        b::mass_properties(*r, CAD_0::math::Tolerance::linear(1e-9));
    // The cap's own quota is the FLOAT-EXTRUDED one (double(1.6f)
    // offset by the float -1.2f) — the honest number: abc - pi r^2 (z_cap +
    // 1) with z_cap = double(1.6f) - 1.2.
    const double z_floor = static_cast<double>(1.6f) -
                           static_cast<double>(1.2f);
    CHECK(close(mp.volume, 8.0 - 0.0625 * kPi * (1.0 + z_floor)));
    CHECK(mp.all_exact);
}

// ---------------------------------------------------------------------------
// THE ADDITIVITY — the through family beside the wall: the G1
// machine untouched by the narrowing (the counts, the gate and the
// golden all answer exactly as the unit-4 tests pin them — the
// sanity oracle the two verdicts ride on).
// ---------------------------------------------------------------------------
TEST_CASE("G2-U2 m2 the wall's oracle: the THROUGH case answers the "
          "landed golden - the machine the verdicts are read on") {
    // The tool SPANS the box (z in [-1.5, 1.5] against [-1, 1] — the
    // probe's own sanity call): the through family, untouched by
    // the narrowing, answering the landed golden.
    auto a = b::make_box({1.0f, 1.0f, 1.0f});
    std::vector<s::SketchChain> chains;
    chains.push_back(s::SketchChain{
        {s::sketch_circle({0.0f, 0.0f}, 0.5f, false)}});
    auto tool = b::make_prism(chains, 3.0f, +1);
    auto r = b::exact_subtract(*a, *tool, {0.0f, 0.0f, -1.5f},
                                b::kSubtractTolerance);
    // The tool spans: THROUGH (z_lo <= z_bot + tol, z_hi >= z_top - tol).
    REQUIRE(r != nullptr);
    REQUIRE(r->vertex_count() == 12);
    REQUIRE(r->edge_count() == 17);
    REQUIRE(r->face_count() == 7);
    REQUIRE(b::inner_loop_count(*r) == 2);
    REQUIRE(b::euler_characteristic(*r) == 0);
    CHECK(b::is_closed_orientable(*r));
    const auto mp =
        b::mass_properties(*r, CAD_0::math::Tolerance::linear(1e-9));
    CHECK(close(mp.volume, 8.0 - 0.5 * kPi));   // abc - pi r^2 c
    CHECK(mp.all_exact);
}

// tests/unit/brep/test_g2_unit2_m4_blind_door.cpp
//
// G2 UNIT 2, m4 (FINALE DELL'UNITÀ) — IL DECOUPLING D'ORIENTAMENTO:
// the blind door opens. The wall the m2 verdicts pinned becomes the
// door's own specification: the same pair of facts, the opposite
// outcome — the carried cap keeps its NATIVE FRAME and gains the
// INVERTED REGION BIT (the rim arcs' hole = true, "the material now
// outside the profile"); the walk serves the topology (the chain,
// the XOR, the volume — the mass machinery reads the outer loop's
// region side, ADR-0038's m4 rule). The wall's two tests
// (test_g2_unit2_blind_wall.cpp's VERDICT A and VERDICT B) run
// BESIDE this file UNTOUCHED: the inverted-volume pin and the
// unchainable-walk pin are the facts THIS door clears — the m2
// probe's VARIANT A authoring (hole = false) still reads inverted
// under the same machinery, and that IS the discrimination: the bit,
// not the frame, is the sign's master.
//
// THE GUIDE CASE (D5, the numbers standing unmoved): the box
// [-1,1]^3, the tool the circle prism r = 0.5, axis Z, footprint
// strictly inside, z in [-0.5, 1.5] — the floor quota z_cap = -0.5
// (the tool's cap(lo), carried), the pierced quota z_top = 1.
//
// THE DERIVED COUNTS, reproduced as the test's own body:
//
//   * VERTICES: the box's 8 + the walk's split pairs (S/A at the
//     floor quota, S/A at the pierced quota): V = 12.
//   * EDGES: the box's 12 + the floor rim's half-arc pair + the
//     pierced rim's half-arc pair + the seam: E = 17.
//   * FACES: the box's 6 (the top absorbed — its inner loop) + the
//     wall (absorbed, the window re-timed to [-0.5, 1]) + the
//     internal cap (absorbed, carried): F = 8.
//   * H: 1 (the pierced top's inner loop); the floor's rim is its
//     own outer loop.
//   * THE CHI IDENTITY: chi = 12 - 17 + 8 - 1 = 2 = the box's own
//     chi; Delta-chi = +4 - 5 + 2 - 1 = 0 — the DENT family (the
//     through hole's -2 is the handle family).
//   * THE VOLUME: abc - pi r^2 (z_top - z_cap) = 8 - 0.375pi.
//
// THE NUMBERING (the through case's circle layout, the quotas
// re-based): vertices 8..11 = S_c, A_c, S_t, A_t (S = the seam +X
// meridian, A the antipode); edges 12 = the -Y half at the floor
// (A_c -> S_c), 13 = the +Y half (S_c -> A_c), 14/15 = their
// pierced copies, 16 = the seam (S_c -> S_t); faces 6 = the wall,
// 7 = the internal cap.
//
// THE LOOPS (the decoupling's authoring, every flag pinned):
//
//   wall (6):  [16 REV, 13 FWD, 12 FWD, 16 FWD, 14 REV, 15 REV]
//              — the through machinery's seam-wired walk re-based,
//              every Line2 region-on-right.
//   top (1):   the box's own outer loop + the inner [14 FWD, 15
//              FWD] — the G1 machinery, the arcs hole = true.
//   cap (7):   [13 REV, 12 REV] — the cap(lo)'s OWN walk (the
//              reversed cycle — the caps' frame parity), CCW in the
//              native mirror domain, the arcs hole = TRUE (the
//              inverted region bit: THE DECOUPLING).
//
// THE NAMES (D3): the floor rim sub#4:isect[ext#3:cap(lo) ×
// ext#3:lat(0)] — the grammar's FIRST INTRA-BODY row (the same body
// twice, the level absent: the cap's plane IS the level); the
// pierced rim sub#4:isect[box#1:top × ext#3:lat(0)]@hi and the seam
// sub#4:seam[ext#3:lat(0)] in the G1 forms; the cap ABSORBED, keeping
// ext#3:cap(lo) (identity is where a thing was born).
//
// THE TAVOLA (D3): 47 rows — 37 result (8 kept + 4 born vertices;
// 12 kept + 5 born edges; 5 kept + 3 absorbed faces) + 10 vanished
// (the tool's split pair and rims/seam re-authored, the far cap cut
// away), the two-sided closure green.
//
// THE BORDERS: the both-caps-inside cavity refused with ITS OWN
// NAME (the two-shell body, chi = 2*2 — D4's queue intact); the
// CHAIN blind refused with its own name (the per-entity rim
// projection, the walk table's queued row); the tangential border
// byte-identical; the mid-edge crossed and the composite keep their
// words; the through family and the m3 corner answer their landed
// goldens beside (the additivity without a re-pin).
#include <doctest/doctest.h>

#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/extrude.hpp"
#include "CAD_0/brep/subtract.hpp"
#include "CAD_0/brep/descent.hpp"
#include "CAD_0/brep/mass_properties.hpp"
#include "CAD_0/brep/query.hpp"
#include "CAD_0/brep/naming.hpp"
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

// The guide case's chain (the D1 chain the m3/m4 tests practice):
// box#1 -> ext#3 -> sub#4. The tool: the circle r = 0.5, prism
// height 2 (z in [0, 2]), offset (0, 0, -0.5) -> z in [-0.5, 1.5].
struct BlindGuide {
    std::unique_ptr<b::BRepBody> box;
    std::unique_ptr<b::BRepBody> tool;
    Vec3d offset{0.0, 0.0, -0.5};
    b::SubtractNaming named{};
};

BlindGuide blind_guide() {
    BlindGuide g;
    g.box = b::make_box({1.0, 1.0, 1.0},
                        CAD_0::geometry::FeatureId{1});
    std::vector<s::SketchChain> chains;
    chains.push_back(s::SketchChain{
        {s::sketch_circle({0.0f, 0.0f}, 0.5f, false)}});
    g.tool = b::make_prism(chains, 2.0f, +1,
                           CAD_0::geometry::FeatureId{3});
    g.named = b::SubtractNaming{
        CAD_0::geometry::FeatureId{4},
        &b::naming::kBoxNaming,
        &b::naming::kExtNaming};
    return g;
}

std::string name_digest(const b::BRepBody& body) {
    std::string out;
    for (b::FaceId i = 0; i < body.face_count(); ++i) {
        out += std::to_string(i);
        out += '|';
        out += b::query_face(body, i).name;
        out += '\n';
    }
    return out;
}

std::string edge_name_digest(const b::BRepBody& body) {
    std::string out;
    for (b::EdgeId i = 0; i < body.edge_count(); ++i) {
        out += std::to_string(i);
        out += '|';
        out += b::query_edge(body, i).name;
        out += '\n';
    }
    return out;
}

bool pcurves_chain(const b::BRepBody& body, const b::Face& face,
                   std::size_t loop_index) {
    const auto& loop = face.loops()[loop_index];
    const std::size_t n = loop.coedges().size();
    for (std::size_t i = 0; i < n; ++i) {
        const auto* pc = loop.coedges()[i].p_curve();
        if (pc == nullptr) return false;
        const auto* next = loop.coedges()[(i + 1) % n].p_curve();
        if (next == nullptr) return false;
        Vec2d end{}, next_start{};
        if (pc->kind == b::PCurve::Kind::Segment) {
            end = pc->segment.b;
        } else if (pc->kind == b::PCurve::Kind::Arc) {
            end = b::pcurve_arc_end_point(pc->arc);
        } else {
            return false;
        }
        if (next->kind == b::PCurve::Kind::Segment) {
            next_start = next->segment.a;
        } else if (next->kind == b::PCurve::Kind::Arc) {
            next_start = b::pcurve_arc_start_point(next->arc);
        } else {
            return false;
        }
        if (std::abs(end.x - next_start.x) > 1e-9 ||
            std::abs(end.y - next_start.y) > 1e-9)
            return false;
    }
    return true;
}

struct ExpectedRow {
    b::Descent relation{};
    b::EntityKind entity{};
    std::uint64_t address{};
    bool has_operand{};
    std::uint8_t operand{};
    b::EntityKind operand_kind{};
    std::uint64_t operand_address{};
    const char* name{};
    const char* cause{};
};

bool row_matches(const b::DescentRow& row, const ExpectedRow& want) {
    if (row.relation != want.relation) return false;
    if (row.entity != want.entity) return false;
    if (row.address != want.address) return false;
    if (row.name != want.name) return false;
    if (want.cause == nullptr) {
        if (!row.cause.empty()) return false;
    } else if (row.cause != want.cause) {
        return false;
    }
    if (want.has_operand) {
        if (row.operands.size() != 1) return false;
        if (row.operands[0].operand != want.operand) return false;
        if (row.operands[0].kind != want.operand_kind) return false;
        if (row.operands[0].address != want.operand_address) return false;
    } else if (!row.operands.empty()) {
        return false;
    }
    return row.from.empty();
}

constexpr const char* kCutAwayCause =
    "cut away outside the target";
constexpr const char* kReAuthoredCause =
    "re-authored with the window";

} // namespace

// ---------------------------------------------------------------------------
// THE GOLDEN — the counts, the chi identity, the volume (the
// decoupling's own answer), the gate, the numbering tables.
// ---------------------------------------------------------------------------
TEST_CASE("G2-U2 m4 the golden: the blind door - 12/17/8/H=1/chi=2, "
          "the volume 8-0.375pi, all exact, the output gate green") {
    const BlindGuide g = blind_guide();
    auto r = b::exact_subtract(*g.box, *g.tool, g.offset,
                               b::kSubtractTolerance, g.named);
    REQUIRE(r != nullptr);

    // THE COUNTS — D5's derivation reproduced: the box's 8 + the
    // walk's split pairs = 12; the box's 12 + the floor pair + the
    // pierced pair + the seam = 17; the box's 6 + the wall + the
    // carried cap = 8; H = 1 (ONE pierced face — the blind pierces
    // one, the through pierces two).
    REQUIRE(r->vertex_count() == 12);
    REQUIRE(r->edge_count() == 17);
    REQUIRE(r->face_count() == 8);
    REQUIRE(b::inner_loop_count(*r) == 1);
    // THE CHI IDENTITY, visible: 12 - 17 + 8 - 1 = 2 = the box's
    // own chi; Delta-chi = +4 - 5 + 2 - 1 = 0 — the DENT family
    // (the m2 verdict's proof: the counts land; the volume was the
    // sole wall, and the decoupling opened it).
    REQUIRE(b::euler_characteristic(*r) == 2);
    CHECK(b::euler_characteristic(*r) == b::euler_characteristic(*g.box));
    CHECK(b::is_closed_orientable(*r));
    CHECK(b::loops_are_closed(*r));
    // The output gate's XOR: every edge exactly two opposite
    // co-edges.
    for (const auto& e : r->edges) {
        REQUIRE(e.coedges().size() == 2);
        CHECK(e.coedges()[0]->reversed() != e.coedges()[1]->reversed());
    }
    // The seam clause: the seam carried twice by the wall's single
    // loop, opposite flags (the periodic identification).
    {
        const auto& wall = r->solid.shell().faces()[6];
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
    }
    // The id discipline (the factories' own).
    for (std::size_t i = 0; i < r->vertices.size(); ++i)
        CHECK(r->vertices[i].id() == static_cast<b::VertexId>(i));
    for (std::size_t i = 0; i < r->edges.size(); ++i)
        CHECK(r->edges[i].id() == static_cast<b::EdgeId>(i));

    // THE NUMBERING TABLE — the through case's circle layout, the
    // quotas re-based (2i/2i+1 = J_i at the window's low/high end):
    // 8 = S_c, 9 = S_t, 10 = A_c, 11 = A_t (S = the seam +X
    // meridian, A the antipode).
    const Vec3d pv[4] = {{0.5, 0.0, -0.5}, {0.5, 0.0, 1.0},
                         {-0.5, 0.0, -0.5}, {-0.5, 0.0, 1.0}};
    for (std::size_t i = 0; i < 4; ++i) {
        CAPTURE(i);
        const Vec3d p = r->vertices[8 + i].position();
        CHECK(std::abs(p.x - pv[i].x) < 1e-12);
        CHECK(std::abs(p.y - pv[i].y) < 1e-12);
        CHECK(std::abs(p.z - pv[i].z) < 1e-12);
    }
    // 12 = the -Y half at the floor (A_c -> S_c); 13 = the +Y half;
    // 14/15 = their pierced copies; 16 = the seam (S_c -> S_t).
    const std::pair<std::size_t, std::size_t> pe[5] = {
        {10, 8}, {8, 10}, {11, 9}, {9, 11}, {8, 9}};
    for (std::size_t i = 0; i < 5; ++i) {
        CAPTURE(i);
        CHECK(r->edges[12 + i].vertex(0) == pe[i].first);
        CHECK(r->edges[12 + i].vertex(1) == pe[i].second);
    }

    // THE VOLUME — the decoupling's own answer, the classical form
    // the test: abc - pi r^2 (z_top - z_cap) = 8 - 0.375pi. The m2
    // wall's inverted read (8 - 0.375pi + 0.25pi/3) is the OTHER
    // authoring's fact, pinned untouched in the wall's own file.
    const auto mp =
        b::mass_properties(*r, CAD_0::math::Tolerance::linear(1e-9));
    CHECK(close(mp.volume, 8.0 - 0.375 * kPi));
    CHECK_FALSE(close(mp.volume, 8.0 - 0.375 * kPi + 0.25 * kPi / 3.0));
    CHECK(mp.all_exact);
    // The centroid: on the axis by symmetry; the cavity's own
    // centroid at z = (z_cap + z_top)/2 = 0.25 removed.
    CHECK(std::abs(mp.centroid.x) < 1e-9);
    CHECK(std::abs(mp.centroid.y) < 1e-9);
    CHECK(close(mp.centroid.z,
                -(0.375 * kPi * 0.25) / (8.0 - 0.375 * kPi)));

    // The provenance: exact throughout (the operands are exact; the
    // carried cap's record is combined, its exactness inherited).
    for (const auto& v : r->vertices) CHECK(v.provenance().is_exact());
    for (const auto& e : r->edges) CHECK(e.provenance().is_exact());
    for (const auto& f : r->solid.shell().faces())
        CHECK(f.provenance().is_exact());
}

// ---------------------------------------------------------------------------
// THE DECOUPLING — the authoring pinned: the cap's NATIVE FRAME,
// its OWN walk, the INVERTED REGION BIT; the wall re-based; the
// pierced inner loop the G1 machinery. The bit is the only
// difference from the m2 probe's VARIANT A — and it is the sign's
// master.
// ---------------------------------------------------------------------------
TEST_CASE("G2-U2 m4 the decoupling: the carried cap's native frame + "
          "the inverted region bit, the wall re-based, the pierced "
          "inner loop - the co-edge tables and the p-curves pinned") {
    const BlindGuide g = blind_guide();
    auto r = b::exact_subtract(*g.box, *g.tool, g.offset,
                               b::kSubtractTolerance, g.named);
    REQUIRE(r != nullptr);
    const auto& faces = r->solid.shell().faces();

    // THE WALL (face 6): the through machinery's seam-wired walk
    // re-based to [z_cap, z_top] — [seam REV down, bot1 FWD, bot0
    // FWD, seam FWD up, top0 REV, top1 REV], the CylinderZSurface on
    // the cavity's u_sign = -1, z_base = z_cap, every Line2
    // region-on-right.
    {
        const auto* cyl = dynamic_cast<const b::CylinderZSurface*>(
            faces[6].surface());
        REQUIRE(cyl != nullptr);
        CHECK(close(cyl->radius(), 0.5));
        CHECK(close(cyl->z_base(), -0.5));
        CHECK(cyl->u_sign() < 0.0);
        const std::pair<b::EdgeId, bool> want[6] = {
            {16, true}, {13, false}, {12, false},
            {16, false}, {14, true}, {15, true}};
        const auto& ces = faces[6].loops()[0].coedges();
        REQUIRE(ces.size() == 6);
        for (std::size_t i = 0; i < 6; ++i) {
            CAPTURE(i);
            CHECK(ces[i].edge() == want[i].first);
            CHECK(ces[i].reversed() == want[i].second);
        }
        CHECK(pcurves_chain(*r, faces[6], 0));
        // The window payloads: [-2pi, 0] x [0, 1.5], every Line2
        // region-on-right (interior_left = false).
        const std::pair<Vec2d, Vec2d> seg[6] = {
            {{0.0, 1.5}, {0.0, 0.0}},
            {{0.0, 0.0}, {-kPi, 0.0}},
            {{-kPi, 0.0}, {-kTwoPi, 0.0}},
            {{-kTwoPi, 0.0}, {-kTwoPi, 1.5}},
            {{-kTwoPi, 1.5}, {-kPi, 1.5}},
            {{-kPi, 1.5}, {0.0, 1.5}}};
        for (std::size_t i = 0; i < 6; ++i) {
            CAPTURE(i);
            const auto* pc = ces[i].p_curve();
            REQUIRE(pc != nullptr);
            CHECK(pc->kind == b::PCurve::Kind::Segment);
            CHECK(std::abs(pc->segment.a.x - seg[i].first.x) < 1e-12);
            CHECK(std::abs(pc->segment.a.y - seg[i].first.y) < 1e-12);
            CHECK(std::abs(pc->segment.b.x - seg[i].second.x) < 1e-12);
            CHECK(std::abs(pc->segment.b.y - seg[i].second.y) < 1e-12);
            CHECK(pc->segment.interior_left == false);
        }
    }

    // THE PIERCED TOP FACE (face 1): the box's own outer loop
    // verbatim + the inner loop [14 FWD, 15 FWD] — the G1 machinery,
    // the arcs hole = true (the material outside the profile), CCW
    // in the +Z rotation domain.
    {
        REQUIRE(faces[1].loops().size() == 2);
        CHECK(faces[1].loops()[0].is_outer());
        CHECK(!faces[1].loops()[1].is_outer());
        const auto& ces = faces[1].loops()[1].coedges();
        REQUIRE(ces.size() == 2);
        CHECK(ces[0].edge() == 14);
        CHECK(ces[0].reversed() == false);
        CHECK(ces[1].edge() == 15);
        CHECK(ces[1].reversed() == false);
        CHECK(pcurves_chain(*r, faces[1], 1));
        for (const auto& ce : ces) {
            const auto* pc = ce.p_curve();
            REQUIRE(pc != nullptr);
            CHECK(pc->kind == b::PCurve::Kind::Arc);
            CHECK(pc->arc.hole == true);
            CHECK(close(pc->arc.radius, 0.5));
        }
    }

    // THE INTERNAL CAP (face 7) — THE DECOUPLING'S OWN AUTHORING:
    // the tool's cap(lo) surface carried BIT-IDENTICAL (the NATIVE
    // FRAME: the plane z = -0.5 with the tool's own -Z normal, the
    // MIRROR domain), the loop the cap's OWN walk ([13 REV, 12 REV]
    // — the reversed cycle, the caps' frame parity), and the region
    // bit INVERTED: the rim arcs carry hole = TRUE. The m2 probe's
    // VARIANT A is this exact authoring with hole = false — the bit
    // is the sole difference, and the volume's master.
    {
        const auto* pl = dynamic_cast<const b::PlaneSurface*>(
            faces[7].surface());
        REQUIRE(pl != nullptr);
        CHECK(std::abs(pl->origin().z - (-0.5)) < 1e-12);
        CHECK(std::abs(pl->origin().x) < 1e-12);
        CHECK(std::abs(pl->origin().y) < 1e-12);
        CHECK(std::abs(pl->normal_vec().z - (-1.0)) < 1e-12);
        // The mirror frame: u = +x, v = -y (the -Z plane's own).
        CHECK(std::abs(pl->u_axis().x - 1.0) < 1e-12);
        CHECK(std::abs(pl->v_axis().y + 1.0) < 1e-12);
        REQUIRE(faces[7].loops().size() == 1);
        const auto& ces = faces[7].loops()[0].coedges();
        REQUIRE(ces.size() == 2);
        CHECK(ces[0].edge() == 13);
        CHECK(ces[0].reversed() == true);
        CHECK(ces[1].edge() == 12);
        CHECK(ces[1].reversed() == true);
        CHECK(pcurves_chain(*r, faces[7], 0));
        // The rim arcs: the consultation's circle (the center the
        // axis's own image, the radius the wall's own), the starts
        // at the traversal's own starts (A's image then S's image),
        // the sweeps pi — and THE BIT: hole = true.
        const auto* a0 = ces[0].p_curve();
        const auto* a1 = ces[1].p_curve();
        REQUIRE(a0 != nullptr);
        REQUIRE(a1 != nullptr);
        CHECK(a0->kind == b::PCurve::Kind::Arc);
        CHECK(a1->kind == b::PCurve::Kind::Arc);
        CHECK(a0->arc.hole == true);
        CHECK(a1->arc.hole == true);
        CHECK(close(a0->arc.radius, 0.5));
        CHECK(close(a1->arc.radius, 0.5));
        CHECK(std::abs(a0->arc.center.x) < 1e-12);
        CHECK(std::abs(a0->arc.center.y) < 1e-12);
        CHECK(std::abs(a1->arc.center.x) < 1e-12);
        CHECK(std::abs(a1->arc.center.y) < 1e-12);
        // The reversed cycle's starts: the co-edge's traversal
        // start's own image — A's image (the antipode -X) first,
        // then S's (the seam +X); the sweeps pi each. (The raw
        // start ANGLE of A's image sits on the atan2(-0.0, -r) side
        // — minus pi, not pi — the same point on the circle; the
        // point is the honest pin, never the angle's branch.)
        const Vec2d a0_start = b::pcurve_arc_start_point(a0->arc);
        const Vec2d a1_start = b::pcurve_arc_start_point(a1->arc);
        CHECK(std::abs(a0_start.x - (-0.5)) < 1e-12);
        CHECK(std::abs(a0_start.y) < 1e-12);
        CHECK(std::abs(a1_start.x - 0.5) < 1e-12);
        CHECK(std::abs(a1_start.y) < 1e-12);
        CHECK(close(a0->arc.sweep, kPi));
        CHECK(close(a1->arc.sweep, kPi));
    }

    // THE FACE QUARTET'S SURVEY — the address plane: 0..5 the box's
    // own (the top pierced), 6 the wall, 7 the cap.
    CHECK(faces[6].loops().size() == 1);
    CHECK(faces[7].loops().size() == 1);
    for (std::size_t f : {std::size_t{0}, std::size_t{2},
                          std::size_t{3}, std::size_t{4},
                          std::size_t{5}})
        CHECK(faces[f].loops().size() == 1);
}

// ---------------------------------------------------------------------------
// THE NAMES (D3) — the intra-body row's first instance, the G1
// forms, the inheritances; the structural by-name query; the honest
// unnamed mode.
// ---------------------------------------------------------------------------
TEST_CASE("G2-U2 m4 the names: the floor rim's INTRA-BODY row, the "
          "pierced rim and the seam in the G1 forms, the cap "
          "absorbed keeping its own - the D3 table on the blind") {
    const BlindGuide g = blind_guide();
    auto r = b::exact_subtract(*g.box, *g.tool, g.offset,
                               b::kSubtractTolerance, g.named);
    REQUIRE(r != nullptr);

    // THE FLOOR RIM — the grammar's FIRST INTRA-BODY row: the
    // carried cap x the wall of the SAME body (both legs ext#3), the
    // level ABSENT (the cap's plane IS the level). Both halves of
    // the seam-wired pair answer to the ONE logical name.
    CHECK(b::query_edge(*r, 12).name ==
          "sub#4:isect[ext#3:cap(lo) × ext#3:lat(0)]");
    CHECK(b::query_edge(*r, 13).name ==
          "sub#4:isect[ext#3:cap(lo) × ext#3:lat(0)]");
    // THE PIERCED RIM and THE SEAM — the G1 forms, unchanged.
    CHECK(b::query_edge(*r, 14).name ==
          "sub#4:isect[box#1:top × ext#3:lat(0)]@hi");
    CHECK(b::query_edge(*r, 15).name ==
          "sub#4:isect[box#1:top × ext#3:lat(0)]@hi");
    CHECK(b::query_edge(*r, 16).name == "sub#4:seam[ext#3:lat(0)]");
    // The inheritances: the wall is the extrude's lateral; the cap
    // ABSORBED keeps its own name (identity is where a thing was
    // born); the pierced face keeps the factory's.
    CHECK(b::query_face(*r, 6).name == "ext#3:lat(0)");
    CHECK(b::query_face(*r, 7).name == "ext#3:cap(lo)");
    CHECK(b::query_face(*r, 1).name == "box#1:top");
    CHECK(b::query_face(*r, 0).name == "box#1:bottom");
    // The box's kept edges keep their names.
    CHECK(b::query_edge(*r, 0).name == "box#1:bottom0");
    CHECK(b::query_edge(*r, 4).name == "box#1:top0");

    // THE STRUCTURAL BY-NAME QUERY — the cap interrogated with its
    // own name (ext#3, the path [kExtCapLoRole]): the absorbed face
    // found alive in the vivo; the floor rim's logical ShapeId (the
    // intra-body row's path) finds BOTH halves.
    {
        CAD_0::geometry::SubshapeRef cap_ref{};
        cap_ref = cap_ref.push(b::naming::kExtCapLoRole);
        const CAD_0::geometry::ShapeId cap_want{
            CAD_0::geometry::FeatureId{3}, cap_ref, {}};
        const std::vector<b::FaceId> cap_found =
            b::query_face_by_name(*r, cap_want);
        REQUIRE(cap_found.size() == 1);
        CHECK(cap_found[0] == 7);
    }
    {
        CAD_0::geometry::SubshapeRef rim_ref{};
        rim_ref = rim_ref.push(b::naming::kSubIsectIntraRole)
                      .push(b::naming::kExtCapLoRole)
                      .push(b::naming::kExtLatRole)
                      .push(0);
        const CAD_0::geometry::ShapeId rim_want{
            CAD_0::geometry::FeatureId{4}, rim_ref, {}};
        const std::vector<b::EdgeId> rim_found =
            b::query_edge_by_name(*r, rim_want);
        REQUIRE(rim_found.size() == 2);
        CHECK(rim_found[0] == 12);
        CHECK(rim_found[1] == 13);
    }

    // The honest boundary: the unnamed mode — the born rims and the
    // seam read honestly EMPTY; the wall's and the cap's
    // inheritances flow regardless (stamp copies, no table
    // involved); the address plane untouched.
    {
        auto r2 = b::exact_subtract(*g.box, *g.tool, g.offset,
                                    b::kSubtractTolerance);
        REQUIRE(r2 != nullptr);
        for (const b::EdgeId i : {b::EdgeId{12}, b::EdgeId{13},
                                  b::EdgeId{14}, b::EdgeId{15},
                                  b::EdgeId{16}}) {
            CAPTURE(static_cast<unsigned>(i));
            CHECK(b::query_edge(*r2, i).name.empty());
        }
        CHECK(b::query_face(*r2, 6).name == "ext#3:lat(0)");
        CHECK(b::query_face(*r2, 7).name == "ext#3:cap(lo)");
        CHECK(r2->vertex_count() == 12);
        CHECK(r2->edge_count() == 17);
        CHECK(r2->face_count() == 8);
    }
}

// ---------------------------------------------------------------------------
// THE TAVOLA PIENA — the 47 rows, row by row (D3's counts: 37
// result + 10 vanished; the two-sided closure green).
// ---------------------------------------------------------------------------
TEST_CASE("G2-U2 m4 the tavola piena: the blind case's 47 rows, row "
          "by row, the closure green") {
    const BlindGuide g = blind_guide();
    auto r = b::exact_subtract(*g.box, *g.tool, g.offset,
                               b::kSubtractTolerance, g.named);
    REQUIRE(r != nullptr);
    const b::DescentTable t = b::subtract_descent(*r, *g.box, *g.tool);

    // D3's own count, verified: 37 out + 10 vanished = 47.
    REQUIRE(t.rows.size() == 47);
    CHECK(t.count(b::Descent::Kept) == 25);      // 8 V + 12 E + 5 F
    CHECK(t.count(b::Descent::Absorbed) == 3);   // the pierced top,
                                                 // the wall, the cap
    CHECK(t.count(b::Descent::Born) == 9);       // 4 V + 5 E
    CHECK(t.count(b::Descent::Vanished) == 10);  // 4 V + 5 E + 1 F
    CHECK(t.count(b::Descent::Split) == 0);
    CHECK(t.count(b::Descent::Merged) == 0);

    const std::vector<ExpectedRow> expected = {
        // ---- the result's vertices: 8 kept + 4 born
        {b::Descent::Kept, b::EntityKind::Vertex, 0,
         true, 0, b::EntityKind::Vertex, 0, "", nullptr},
        {b::Descent::Kept, b::EntityKind::Vertex, 1,
         true, 0, b::EntityKind::Vertex, 1, "", nullptr},
        {b::Descent::Kept, b::EntityKind::Vertex, 2,
         true, 0, b::EntityKind::Vertex, 2, "", nullptr},
        {b::Descent::Kept, b::EntityKind::Vertex, 3,
         true, 0, b::EntityKind::Vertex, 3, "", nullptr},
        {b::Descent::Kept, b::EntityKind::Vertex, 4,
         true, 0, b::EntityKind::Vertex, 4, "", nullptr},
        {b::Descent::Kept, b::EntityKind::Vertex, 5,
         true, 0, b::EntityKind::Vertex, 5, "", nullptr},
        {b::Descent::Kept, b::EntityKind::Vertex, 6,
         true, 0, b::EntityKind::Vertex, 6, "", nullptr},
        {b::Descent::Kept, b::EntityKind::Vertex, 7,
         true, 0, b::EntityKind::Vertex, 7, "", nullptr},
        {b::Descent::Born, b::EntityKind::Vertex, 8,
         false, 0, b::EntityKind::Vertex, 0, "", nullptr},
        {b::Descent::Born, b::EntityKind::Vertex, 9,
         false, 0, b::EntityKind::Vertex, 0, "", nullptr},
        {b::Descent::Born, b::EntityKind::Vertex, 10,
         false, 0, b::EntityKind::Vertex, 0, "", nullptr},
        {b::Descent::Born, b::EntityKind::Vertex, 11,
         false, 0, b::EntityKind::Vertex, 0, "", nullptr},
        // ---- the result's edges: 12 kept + 5 born
        {b::Descent::Kept, b::EntityKind::Edge, 0,
         true, 0, b::EntityKind::Edge, 0, "box#1:bottom0", nullptr},
        {b::Descent::Kept, b::EntityKind::Edge, 1,
         true, 0, b::EntityKind::Edge, 1, "box#1:bottom1", nullptr},
        {b::Descent::Kept, b::EntityKind::Edge, 2,
         true, 0, b::EntityKind::Edge, 2, "box#1:bottom2", nullptr},
        {b::Descent::Kept, b::EntityKind::Edge, 3,
         true, 0, b::EntityKind::Edge, 3, "box#1:bottom3", nullptr},
        {b::Descent::Kept, b::EntityKind::Edge, 4,
         true, 0, b::EntityKind::Edge, 4, "box#1:top0", nullptr},
        {b::Descent::Kept, b::EntityKind::Edge, 5,
         true, 0, b::EntityKind::Edge, 5, "box#1:top1", nullptr},
        {b::Descent::Kept, b::EntityKind::Edge, 6,
         true, 0, b::EntityKind::Edge, 6, "box#1:top2", nullptr},
        {b::Descent::Kept, b::EntityKind::Edge, 7,
         true, 0, b::EntityKind::Edge, 7, "box#1:top3", nullptr},
        {b::Descent::Kept, b::EntityKind::Edge, 8,
         true, 0, b::EntityKind::Edge, 8, "box#1:vertical0", nullptr},
        {b::Descent::Kept, b::EntityKind::Edge, 9,
         true, 0, b::EntityKind::Edge, 9, "box#1:vertical1", nullptr},
        {b::Descent::Kept, b::EntityKind::Edge, 10,
         true, 0, b::EntityKind::Edge, 10, "box#1:vertical2", nullptr},
        {b::Descent::Kept, b::EntityKind::Edge, 11,
         true, 0, b::EntityKind::Edge, 11, "box#1:vertical3", nullptr},
        // THE FLOOR RIM's pair — the intra-body row's first
        // instances (both halves the one logical name).
        {b::Descent::Born, b::EntityKind::Edge, 12, false, 0,
         b::EntityKind::Edge, 0,
         "sub#4:isect[ext#3:cap(lo) × ext#3:lat(0)]", nullptr},
        {b::Descent::Born, b::EntityKind::Edge, 13, false, 0,
         b::EntityKind::Edge, 0,
         "sub#4:isect[ext#3:cap(lo) × ext#3:lat(0)]", nullptr},
        {b::Descent::Born, b::EntityKind::Edge, 14, false, 0,
         b::EntityKind::Edge, 0,
         "sub#4:isect[box#1:top × ext#3:lat(0)]@hi", nullptr},
        {b::Descent::Born, b::EntityKind::Edge, 15, false, 0,
         b::EntityKind::Edge, 0,
         "sub#4:isect[box#1:top × ext#3:lat(0)]@hi", nullptr},
        {b::Descent::Born, b::EntityKind::Edge, 16, false, 0,
         b::EntityKind::Edge, 0, "sub#4:seam[ext#3:lat(0)]", nullptr},
        // ---- the result's faces: 5 kept + 3 absorbed
        {b::Descent::Kept, b::EntityKind::Face, 0,
         true, 0, b::EntityKind::Face, 0, "box#1:bottom", nullptr},
        {b::Descent::Absorbed, b::EntityKind::Face, 1,
         true, 0, b::EntityKind::Face, 1, "box#1:top", nullptr},
        {b::Descent::Kept, b::EntityKind::Face, 2,
         true, 0, b::EntityKind::Face, 2, "box#1:front", nullptr},
        {b::Descent::Kept, b::EntityKind::Face, 3,
         true, 0, b::EntityKind::Face, 3, "box#1:back", nullptr},
        {b::Descent::Kept, b::EntityKind::Face, 4,
         true, 0, b::EntityKind::Face, 4, "box#1:left", nullptr},
        {b::Descent::Kept, b::EntityKind::Face, 5,
         true, 0, b::EntityKind::Face, 5, "box#1:right", nullptr},
        // THE WALL — the tool's own lateral, absorbed.
        {b::Descent::Absorbed, b::EntityKind::Face, 6,
         true, 1, b::EntityKind::Face, 0, "ext#3:lat(0)", nullptr},
        // THE INTERNAL CAP — the tool's cap(lo), ABSORBED (the
        // carried face: one face in, one face out).
        {b::Descent::Absorbed, b::EntityKind::Face, 7,
         true, 1, b::EntityKind::Face, 1, "ext#3:cap(lo)", nullptr},
        // ---- the vanished tail: the tool's own order — the split
        // pair, the rims and the seam re-authored with the window.
        {b::Descent::Vanished, b::EntityKind::Vertex, 0,
         true, 1, b::EntityKind::Vertex, 0, "", kReAuthoredCause},
        {b::Descent::Vanished, b::EntityKind::Vertex, 1,
         true, 1, b::EntityKind::Vertex, 1, "", kReAuthoredCause},
        {b::Descent::Vanished, b::EntityKind::Vertex, 2,
         true, 1, b::EntityKind::Vertex, 2, "", kReAuthoredCause},
        {b::Descent::Vanished, b::EntityKind::Vertex, 3,
         true, 1, b::EntityKind::Vertex, 3, "", kReAuthoredCause},
        {b::Descent::Vanished, b::EntityKind::Edge, 0,
         true, 1, b::EntityKind::Edge, 0, "", kReAuthoredCause},
        {b::Descent::Vanished, b::EntityKind::Edge, 1,
         true, 1, b::EntityKind::Edge, 1, "", kReAuthoredCause},
        {b::Descent::Vanished, b::EntityKind::Edge, 2,
         true, 1, b::EntityKind::Edge, 2, "", kReAuthoredCause},
        {b::Descent::Vanished, b::EntityKind::Edge, 3,
         true, 1, b::EntityKind::Edge, 3, "", kReAuthoredCause},
        {b::Descent::Vanished, b::EntityKind::Edge, 4,
         true, 1, b::EntityKind::Edge, 4, "ext#3:seam",
         kReAuthoredCause},
        // The FAR cap (cap(hi), the tool's face 2) cut away outside.
        {b::Descent::Vanished, b::EntityKind::Face, 2,
         true, 1, b::EntityKind::Face, 2, "ext#3:cap(hi)",
         kCutAwayCause},
    };
    REQUIRE(expected.size() == 47);
    for (std::size_t i = 0; i < expected.size(); ++i) {
        CAPTURE(i);
        CHECK(row_matches(t.rows[i], expected[i]));
    }

    // THE CLOSURE, both sides, hard (the m4 activation runs it in
    // debug builds inside; here the explicit pins).
    std::string why;
    CHECK(b::descent_closed(*r, t, &why));
    CHECK(b::old_generation_closed(*g.box, *g.tool, t, &why));
}

// ---------------------------------------------------------------------------
// THE DIGEST + THE DETERMINISM — the blind's names beside the G1
// ones, N=10 rebuilds bit-identical.
// ---------------------------------------------------------------------------
TEST_CASE("G2-U2 m4 the digest and the determinism: N=10 rebuilds, "
          "one digest, bit for bit") {
    const BlindGuide g = blind_guide();
    auto r0 = b::exact_subtract(*g.box, *g.tool, g.offset,
                                b::kSubtractTolerance, g.named);
    REQUIRE(r0 != nullptr);
    const std::string fd = name_digest(*r0);
    const std::string ed = edge_name_digest(*r0);

    // The digest's shape: the blind's own rows — the intra-body
    // floor rim beside the G1 forms, the cap's inherited name beside
    // the wall's, the box's own carried.
    REQUIRE(ed.size() > 0);
    CHECK(ed.find("12|sub#4:isect[ext#3:cap(lo) × ext#3:lat(0)]"
                  "\n") != std::string::npos);
    CHECK(ed.find("14|sub#4:isect[box#1:top × ext#3:lat(0)]@hi"
                  "\n") != std::string::npos);
    CHECK(ed.find("16|sub#4:seam[ext#3:lat(0)]\n") !=
          std::string::npos);
    CHECK(fd.find("7|ext#3:cap(lo)\n") != std::string::npos);
    CHECK(fd.find("6|ext#3:lat(0)\n") != std::string::npos);
    CHECK(fd.find("1|box#1:top\n") != std::string::npos);
    CHECK(ed.find("0|box#1:bottom0\n") != std::string::npos);

    // N=10 rebuilds: the same recipe, the same digests, bit for bit.
    for (int i = 0; i < 10; ++i) {
        BlindGuide g2 = blind_guide();
        auto r = b::exact_subtract(*g2.box, *g2.tool, g2.offset,
                                   b::kSubtractTolerance, g2.named);
        REQUIRE(r != nullptr);
        CHECK(name_digest(*r) == fd);
        CHECK(edge_name_digest(*r) == ed);
    }
}

// ---------------------------------------------------------------------------
// LA CONTROPROFA + THE BORDERS — the pin :973's own call answers
// (the mirror blind_hi), the cavity and the chain blind keep honest
// refusals with their OWN names, the other doors green beside.
// ---------------------------------------------------------------------------
TEST_CASE("G2-U2 m4 the counterproof and the borders: the G1 pin's "
          "call answers, the cavity and the chain blind refuse with "
          "their own names, the families green beside") {
    // THE COUNTERPROOF — the pin :973's own parameters (the m2
    // wall's refusal case, the same call): the tool from below, its
    // top cap inside (z in [-1.2, 0.4], the mirror blind_hi). The
    // same call that refused yesterday ANSWERS today: the counts,
    // the chi, the volume, and the mirror's own names (the floor
    // rim's intra-body row on cap(hi), the pierced rim @lo on the
    // bottom face).
    {
        auto a = b::make_box({1.0, 1.0, 1.0},
                             CAD_0::geometry::FeatureId{1});
        std::vector<s::SketchChain> chains;
        chains.push_back(s::SketchChain{
            {s::sketch_circle({0.0f, 0.0f}, 0.25f, false)}});
        auto tool = b::make_prism(chains, 1.6f, +1,
                                  CAD_0::geometry::FeatureId{3});
        const b::SubtractNaming named{
            CAD_0::geometry::FeatureId{4},
            &b::naming::kBoxNaming,
            &b::naming::kExtNaming};
        auto r = b::exact_subtract(*a, *tool, {0.0, 0.0, -1.2},
                                   b::kSubtractTolerance, named);
        REQUIRE(r != nullptr);
        REQUIRE(r->vertex_count() == 12);
        REQUIRE(r->edge_count() == 17);
        REQUIRE(r->face_count() == 8);
        REQUIRE(b::inner_loop_count(*r) == 1);
        REQUIRE(b::euler_characteristic(*r) == 2);
        CHECK(b::is_closed_orientable(*r));
        const auto mp =
            b::mass_properties(*r,
                               CAD_0::math::Tolerance::linear(1e-9));
        // abc - pi r^2 (z_cap - z_bot): the cap's own quota is the
        // FLOAT-EXTRUDED one (1.6f promoted, the offset applied) —
        // the honest number, not the decimal sketch: z_cap =
        // double(1.6f) - 1.2, the cavity's height z_cap + 1.
        const double z_floor = static_cast<double>(1.6f) - 1.2;
        CHECK(close(mp.volume, 8.0 - 0.0625 * kPi * (1.0 + z_floor)));
        CHECK(mp.all_exact);
        // THE MIRROR'S NAMES — the same rule read on the other
        // parity: the floor rim's intra-body row on cap(hi), the
        // pierced rim @lo on box#1:bottom, the seam the G1 form.
        CHECK(b::query_edge(*r, 14).name ==
              "sub#4:isect[ext#3:cap(hi) × ext#3:lat(0)]");
        CHECK(b::query_edge(*r, 15).name ==
              "sub#4:isect[ext#3:cap(hi) × ext#3:lat(0)]");
        CHECK(b::query_edge(*r, 12).name ==
              "sub#4:isect[box#1:bottom × ext#3:lat(0)]@lo");
        CHECK(b::query_edge(*r, 16).name == "sub#4:seam[ext#3:lat(0)]");
        CHECK(b::query_face(*r, 7).name == "ext#3:cap(hi)");
        // The mirror's cap: the +Z rotation domain, the CANONICAL
        // cycle (the caps' own walks — the other parity).
        const auto& ces =
            r->solid.shell().faces()[7].loops()[0].coedges();
        REQUIRE(ces.size() == 2);
        CHECK(ces[0].edge() == 14);
        CHECK(ces[0].reversed() == false);
        CHECK(ces[1].edge() == 15);
        CHECK(ces[1].reversed() == false);
        for (const auto& ce : ces) {
            const auto* pc = ce.p_curve();
            REQUIRE(pc != nullptr);
            CHECK(pc->kind == b::PCurve::Kind::Arc);
            CHECK(pc->arc.hole == true);
        }
        // The mirror's tavola: the same 47 rows, the operands
        // mirrored (the near cap cap(hi), the far cap(lo) cut away).
        const b::DescentTable t =
            b::subtract_descent(*r, *a, *tool);
        REQUIRE(t.rows.size() == 47);
        CHECK(t.count(b::Descent::Kept) == 25);
        CHECK(t.count(b::Descent::Absorbed) == 3);
        CHECK(t.count(b::Descent::Born) == 9);
        CHECK(t.count(b::Descent::Vanished) == 10);
        std::string why;
        CHECK(b::descent_closed(*r, t, &why));
        CHECK(b::old_generation_closed(*a, *tool, t, &why));
    }

    auto box = [] {
        return b::make_box({1.0, 1.0, 1.0});
    };
    const auto try_subtract = [](const b::BRepBody& a,
                                 const b::BRepBody& tool,
                                 Vec3d offset,
                                 std::string& msg) {
        try {
            auto r = b::exact_subtract(a, tool, offset,
                                       b::kSubtractTolerance);
            (void)r;
            return false;
        } catch (const std::invalid_argument& e) {
            msg = e.what();
            return true;
        }
    };

    // THE BOTH-CAPS-INSIDE CAVITY — refused with ITS OWN NAME: the
    // two-shell body, chi = 2*2 (D4's queue intact).
    {
        std::vector<s::SketchChain> chains;
        chains.push_back(s::SketchChain{
            {s::sketch_circle({0.0f, 0.0f}, 0.25f, false)}});
        auto tool = b::make_prism(chains, 0.8f, +1);
        std::string msg;
        REQUIRE(try_subtract(*box(), *tool, {0.0, 0.0, -0.4}, msg));
        CHECK(msg.find("BLIND") != std::string::npos);
        CHECK(msg.find("TWO-SHELL") != std::string::npos);
        CHECK(msg.find("Remedy") != std::string::npos);
    }

    // THE CHAIN BLIND — refused with its own name: the per-entity
    // rim projection is the walk table's queued row (the same
    // decoupling, the table's row away).
    {
        std::vector<s::SketchChain> chains;
        chains.push_back(s::SketchChain{{
            s::sketch_segment({-0.3f, -0.3f}, {0.3f, -0.3f}, true),
            s::sketch_segment({0.3f, -0.3f}, {0.3f, 0.3f}, true),
            s::sketch_segment({0.3f, 0.3f}, {-0.3f, 0.3f}, true),
            s::sketch_segment({-0.3f, 0.3f}, {-0.3f, -0.3f}, true),
        }});
        auto tool = b::make_prism(chains, 1.6f, +1);
        std::string msg;
        REQUIRE(try_subtract(*box(), *tool, {0.0, 0.0, -1.2}, msg));
        CHECK(msg.find("BLIND") != std::string::npos);
        CHECK(msg.find("CHAIN") != std::string::npos);
        CHECK(msg.find("Remedy") != std::string::npos);
    }

    // THE TANGENTIAL BORDER — LA PARETE's own pin (:914): the
    // tangential family keeps its refusal BYTE-IDENTICAL.
    {
        std::vector<s::SketchChain> chains;
        chains.push_back(s::SketchChain{
            {s::sketch_circle({0.6f, 0.0f}, 0.4f, false)}});
        auto tool = b::make_prism(chains, 3.0f, +1);
        std::string msg;
        REQUIRE(try_subtract(*box(), *tool, {0.0, 0.0, -1.5}, msg));
        CHECK(msg.find("TANGENT") != std::string::npos);
        CHECK(msg.find("DETERMINISTIC") != std::string::npos);
        CHECK(msg.find("Remedy") != std::string::npos);
    }

    // THE MID-EDGE CROSSED — the pin's own circle, still refused.
    {
        std::vector<s::SketchChain> chains;
        chains.push_back(s::SketchChain{
            {s::sketch_circle({0.9f, 0.0f}, 0.35f, false)}});
        auto tool = b::make_prism(chains, 3.0f, +1);
        std::string msg;
        REQUIRE(try_subtract(*box(), *tool, {0.0, 0.0, -1.5}, msg));
        CHECK(msg.find("CROSSES") != std::string::npos);
        CHECK(msg.find("Remedy") != std::string::npos);
    }

    // THE BLIND-CROSSED COMPOSITE — the sharpened reason: the
    // L-shaped cap needs the CHAIN cap insertion (the m4 door is the
    // circle's).
    {
        std::vector<s::SketchChain> chains;
        chains.push_back(s::SketchChain{{
            s::sketch_segment({0.5f, 0.5f}, {1.5f, 0.5f}, true),
            s::sketch_segment({1.5f, 0.5f}, {1.5f, 1.5f}, true),
            s::sketch_segment({1.5f, 1.5f}, {0.5f, 1.5f}, true),
            s::sketch_segment({0.5f, 1.5f}, {0.5f, 0.5f}, true),
        }});
        auto tool = b::make_prism(chains, 2.0f, +1);
        std::string msg;
        REQUIRE(try_subtract(*box(), *tool, {0.0, 0.0, -0.5}, msg));
        CHECK(msg.find("COMPOSITE") != std::string::npos);
        CHECK(msg.find("CHAIN") != std::string::npos);
        CHECK(msg.find("Remedy") != std::string::npos);
    }

    // THE ADDITIVITY — the through family and the m3 corner answer
    // their landed goldens beside the blind door (one assertion
    // each; the suite carries the rest, without a re-pin).
    {
        std::vector<s::SketchChain> chains;
        chains.push_back(s::SketchChain{
            {s::sketch_circle({0.0f, 0.0f}, 0.5f, false)}});
        auto tool = b::make_prism(chains, 3.0f, +1);
        auto r = b::exact_subtract(*box(), *tool, {0.0, 0.0, -1.5},
                                   b::kSubtractTolerance);
        REQUIRE(r != nullptr);
        REQUIRE(r->face_count() == 7);
        REQUIRE(b::euler_characteristic(*r) == 0);
        const auto mp =
            b::mass_properties(*r,
                               CAD_0::math::Tolerance::linear(1e-9));
        CHECK(close(mp.volume, 8.0 - 0.5 * kPi));
    }
    {
        std::vector<s::SketchChain> chains;
        chains.push_back(s::SketchChain{{
            s::sketch_segment({0.5f, 0.5f}, {1.5f, 0.5f}, true),
            s::sketch_segment({1.5f, 0.5f}, {1.5f, 1.5f}, true),
            s::sketch_segment({1.5f, 1.5f}, {0.5f, 1.5f}, true),
            s::sketch_segment({0.5f, 1.5f}, {0.5f, 0.5f}, true),
        }});
        auto tool = b::make_prism(chains, 3.0f, +1);
        auto r = b::exact_subtract(*box(), *tool, {0.0, 0.0, -1.5},
                                   b::kSubtractTolerance);
        REQUIRE(r != nullptr);
        REQUIRE(r->vertex_count() == 12);
        REQUIRE(r->edge_count() == 18);
        REQUIRE(r->face_count() == 8);
        REQUIRE(b::euler_characteristic(*r) == 2);
    }
}

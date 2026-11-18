// tests/unit/brep/test_g6_m2_the_chain.cpp
//
// G6 m2 — LA CATENA K=2 (ADR-0045 D8.1/D8.2, the m6ter stamp): the
// second chamfer on a box ALREADY chamfered, on a NON-ADJACENT
// vertical (the corners disjoint: the second's endpoints never
// touch the first's bevel). The admission WIDENS in a controlled
// way — the m1 signature's frozen triple (the factory box alone)
// declared widened IN THE TEST (the old predicate asserted false
// where it stood, the new admits: never a silent loosening) — and
// the machine is THE SAME re-executed on the chain's state.
//
// THE WIDENING'S WALLS, each with its name:
//   * the counts' chain formula 8+2k / 12+3k / 6+k (k = the
//     bevel-stamped faces), chi = 2, closed orientable, all exact,
//     single-loop, ALL PLANAR;
//   * THE BEVEL-STAMP WALL: the chain's faces are all planes, so
//     the counts alone admit the ALL-PLANAR IMPOSTORS — the
//     HEXAGONAL PRISM is 12/18/8, every plane — the extras must be
//     the chain's own bevels BY STAMP (the identity is where a
//     thing was born), the stamped count and the counts' k
//     agreeing;
//   * a curved face anywhere (a FILLET CHAIN is 12/18/8 with two
//     cylinders) refuses by family — the mixed corner's world,
//     D5.3;
//   * THE STAGE BAND d < 1 (D5.1's derived wall, ALIVE with the
//     multi-edge state): 1.01 refused with the TWO READINGS in the
//     payload (the faces' current reach — the m1 band, satisfied —
//     and the stage's 1, the binding one); the single edge's own
//     band stays the m1 tests' (a lone 1.5 within the reach is the
//     single-edge mode's geometry, its tests IMMOBILE in this
//     suite's own m1 file);
//   * THE STRIP LAW (D1.3, now live for the first time): the two
//     retreat strips on the SHARED face must not meet (d1 + d2 <
//     the face's extent) — enforced by the m1 band machinery
//     reading the CURRENT face's reach (the first strip's retreat
//     TIGHTENS it: 2 - d1), pinned red on the thin box where the
//     sum fires alone (0.5 + 0.5 against the 0.4-extent face, the
//     bound 0.3 carried).
//
// The guide pair (the brief's own): vertical2 (1,1) then vertical1
// (1,-1) — the SAME FACE x=1 carrying both strips. This file pins:
// the golden 12/18/8 / chi=2 / H=0 with V = 8 - 2d^2 = 7.5 all
// exact at 1e-12; the nested names (che#3:bevel[box#1:front x
// box#1:right] on the CURRENT operands — the first chamfer's
// SHRINK FACE referenced as a face operand, the box's stamp kept
// through the absorption, the eternally-antecedent leg) and the
// first link's names kept verbatim (the stamp held); the tavola of
// the SECOND link (41 rows, the first bevel a KEPT face row under
// its own name) beside the first link's own 35; the oracle (the m3
// law: the volume re-derived with THIS FILE's own equations — the
// two removed prisms, the DISJOINTNESS checked with the written
// geometry, the 2e6-midpoint rule — never from the production;
// the doctored reads refused); the chain gate at the FIVE
// predicates AFTER EACH STEP; the digests bit-identical at the
// re-run; the translated box (the m1 lesson holding the SECOND
// application); the walls (the dead reference, the first wire's
// 45-degree dihedral, the impostors).
#include <doctest/doctest.h>

#include "CAD_0/brep/analytic.hpp"
#include "CAD_0/brep/chain_gate.hpp"
#include "CAD_0/brep/chamfer.hpp"
#include "CAD_0/brep/descent.hpp"
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/fillet.hpp"
#include "CAD_0/brep/mass_properties.hpp"
#include "CAD_0/brep/naming.hpp"
#include "CAD_0/brep/pcurve.hpp"
#include "CAD_0/brep/query.hpp"
#include "CAD_0/brep/subtract.hpp"
#include "CAD_0/brep/topology.hpp"
#include "CAD_0/sdf/sketch_profile.hpp"
#include "CAD_0/geometry/shape_id.hpp"

#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace {

namespace b = CAD_0::brep;
namespace n = CAD_0::brep::naming;
namespace s = CAD_0::sdf;
using CAD_0::geometry::FeatureId;
using CAD_0::geometry::ShapeId;
using CAD_0::geometry::SubshapeRef;
using Vec3d = CAD_0::math::Vec3d;

constexpr double kPi = 3.14159265358979323846;
constexpr double kD = 0.5;

bool close(double a, double b, double eps = 1e-9) {
    return std::abs(a - b) <= eps;
}

// The chain's construction: box#1 -> che#2 (vertical2, the (+X,+Y)
// corner, edge 10) -> che#3 (vertical1, the (+X,-Y) corner, edge 9
// — the NON-ADJACENT vertical sharing the RIGHT face x=1: the
// strip pair's own guide). The reference is the KEPT STAMP's
// structural form (the box's own — the stamp survives the first
// link, the ADR-0036 D3 MATCH class through the chain); the naming
// contexts both bind the ROOT's table (the box's — the adjacent
// faces' stamps are the box's, however deep the chain absorbs
// them: the eternally-antecedent leg).
struct ChainGuide {
    std::unique_ptr<b::BRepBody> box;
    std::unique_ptr<b::BRepBody> first;
    std::unique_ptr<b::BRepBody> result;
    ShapeId dead{};         // box#1:vertical2 (edge 10), dead in che#2
    ShapeId second{};       // box#1:vertical1 (edge 9), kept by che#2
    ShapeId first_wire{};   // che#2's right-face wire (a kept stamp)

    explicit ChainGuide(double d2 = kD, double d1 = kD,
                        bool third = true) {
        box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        dead = ShapeId{FeatureId{1}, SubshapeRef{}.push(10), {}};
        second = ShapeId{FeatureId{1}, SubshapeRef{}.push(9), {}};
        const b::ChamferNaming ctx2{FeatureId{2}, &n::kBoxNaming};
        first = b::chamfer_edge(*box, dead, d1, ctx2);
        // che#2's right-face setback wire (edge 12 of the first
        // body) — referenced structurally below for the
        // 45-degree wall.
        first_wire = first->edges[12].shape_id();
        if (!third) return;
        const b::ChamferNaming ctx3{FeatureId{3}, &n::kBoxNaming};
        result = b::chamfer_edge(*first, second, d2, ctx3);
    }
};

// The digests (the m1 test's own shape, replicated for the chain).
std::uint64_t fnv1a64(const std::string& str) {
    std::uint64_t h = 0xcbf29ce484222325ull;
    for (const char c : str) {
        h ^= static_cast<std::uint64_t>(static_cast<unsigned char>(c));
        h *= 0x100000001b3ull;
    }
    return h;
}

std::string name_digest(const b::BRepBody& body) {
    std::string out;
    for (b::FaceId i = 0; i < body.face_count(); ++i) {
        out += std::to_string(i);
        out += '|';
        out += b::query_face(body, i).name;
        out += '\n';
    }
    for (b::EdgeId i = 0; i < body.edge_count(); ++i) {
        out += std::to_string(i);
        out += '|';
        out += b::query_edge(body, i).name;
        out += '\n';
    }
    return out;
}

std::uint64_t structure_digest(const b::BRepBody& body) {
    std::string out;
    out += std::to_string(body.vertex_count());
    out += '/';
    out += std::to_string(body.edge_count());
    out += '/';
    out += std::to_string(body.face_count());
    for (const auto& v : body.vertices) {
        out += std::to_string(v.position().x) + ',';
        out += std::to_string(v.position().y) + ',';
        out += std::to_string(v.position().z) + ';';
    }
    for (const auto& e : body.edges) {
        out += std::to_string(e.vertex(0)) + '-';
        out += std::to_string(e.vertex(1)) + ';';
    }
    for (const auto& f : body.solid.shell().faces())
        for (const auto& loop : f.loops())
            for (const auto& ce : loop.coedges()) {
                out += std::to_string(ce.edge());
                out += ce.reversed() ? 'R' : 'F';
                const auto* pc = ce.p_curve();
                if (pc != nullptr && pc->kind ==
                                         b::PCurve::Kind::Segment) {
                    out += 'p';
                    out += std::to_string(pc->segment.a.x) + ',' +
                           std::to_string(pc->segment.a.y) + ',' +
                           std::to_string(pc->segment.b.x) + ',' +
                           std::to_string(pc->segment.b.y);
                }
                out += ';';
            }
    return fnv1a64(out);
}

// The regular hexagon chain (the impostor's profile): six segments
// CCW, interior on the left.
std::vector<s::SketchChain> hexagon_chains() {
    const double h = std::sqrt(3.0) / 2.0;
    const std::pair<float, float> pts[6] = {
        {1.0f, 0.0f}, {0.5f, static_cast<float>(h)},
        {-0.5f, static_cast<float>(h)}, {-1.0f, 0.0f},
        {-0.5f, static_cast<float>(-h)}, {0.5f, static_cast<float>(-h)}};
    s::SketchChain chain;
    for (int i = 0; i < 6; ++i) {
        const auto& a = pts[i];
        const auto& b2 = pts[(i + 1) % 6];
        chain.entities.push_back(s::sketch_segment(
            {a.first, a.second}, {b2.first, b2.second}, true));
    }
    return {chain};
}

} // namespace

// ===========================================================================
// THE CHAIN GOLDEN — every number, the same walk one link over
// ===========================================================================
TEST_CASE("G6 m2 the chain golden: the second chamfer consumes the "
          "current state — 12/18/8/H=0/chi=2, the volume 8-2d^2 all "
          "exact, the stamps held, the names nested") {
    const ChainGuide g;
    REQUIRE(g.first != nullptr);
    REQUIRE(g.result != nullptr);

    // THE COUNTS — the walk's own arithmetic one link over (the
    // states of D6.2's formula: 10/15/7 -> 12/18/8; the first two
    // the fillet's own chain goldens' counts, the coherence
    // holding a family under).
    REQUIRE(g.first->vertex_count() == 10);
    REQUIRE(g.first->edge_count() == 15);
    REQUIRE(g.first->face_count() == 7);
    REQUIRE(g.result->vertex_count() == 12);
    REQUIRE(g.result->edge_count() == 18);
    REQUIRE(g.result->face_count() == 8);
    REQUIRE(b::inner_loop_count(*g.result) == 0);
    REQUIRE(b::euler_characteristic(*g.result) == 2);
    CHECK(b::euler_characteristic(*g.result) ==
          b::euler_characteristic(*g.box));
    CHECK(b::is_closed_orientable(*g.result));
    CHECK(b::loops_are_closed(*g.result));

    // THE CENSUS: every edge exactly two OPPOSITE co-edges. The
    // per-face row: bottom 6 + top 6 (one chamfer segment per
    // link), the laterals 4 each (the setback line replaces the
    // dead co-edge per link — the m1 arithmetic holds per link),
    // bevel#1 4, bevel#2 4 — 36 incidences / 2 = 18.
    {
        std::size_t total = 0;
        for (const auto& e : g.result->edges) {
            REQUIRE(e.coedges().size() == 2);
            CHECK(e.coedges()[0]->reversed() !=
                  e.coedges()[1]->reversed());
        }
        for (const auto& f : g.result->solid.shell().faces())
            for (const auto& loop : f.loops())
                total += loop.coedges().size();
        REQUIRE(total == 36);
        const std::size_t per_face[8] = {6, 6, 4, 4, 4, 4, 4, 4};
        for (std::size_t i = 0; i < 8; ++i) {
            CAPTURE(i);
            REQUIRE(g.result->solid.shell().faces()[i]
                        .loops()[0]
                        .coedges()
                        .size() == per_face[i]);
        }
    }

    // The id discipline (the factories' own).
    for (std::size_t i = 0; i < g.result->vertices.size(); ++i)
        CHECK(g.result->vertices[i].id() ==
              static_cast<b::VertexId>(i));
    for (std::size_t i = 0; i < g.result->edges.size(); ++i)
        CHECK(g.result->edges[i].id() ==
              static_cast<b::EdgeId>(i));

    // THE NUMBERING TABLE — the second link's own: the survivors
    // compact (the dead corners (1,-1,+-1) dropped), the born
    // append. Vertices 8..11 = SA_lo, SA_hi, SB_lo, SB_hi (A = the
    // FRONT face — the edge's first co-edge; B = right, the first
    // chamfer's SHRINK FACE referenced as an operand).
    {
        const Vec3d pv[4] = {
            {0.5, -1.0, -1.0}, {0.5, -1.0, 1.0},
            {1.0, -0.5, -1.0}, {1.0, -0.5, 1.0}};
        for (std::size_t i = 0; i < 4; ++i) {
            CAPTURE(i);
            const Vec3d p = g.result->vertices[8 + i].position();
            CHECK(close(p.x, pv[i].x, 1e-12));
            CHECK(close(p.y, pv[i].y, 1e-12));
            CHECK(close(p.z, pv[i].z, 1e-12));
        }
        // The born edges: 14 = e_sb_A (8->9, the front's wire), 15
        // = e_sb_B (10->11, the right's), 16 = e_seg_lo (10->8),
        // 17 = e_seg_hi (11->9) — the deterministic B->A
        // convention per link.
        const std::pair<std::size_t, std::size_t> pe[4] = {
            {8, 9}, {10, 11}, {10, 8}, {11, 9}};
        for (std::size_t i = 0; i < 4; ++i) {
            CAPTURE(i);
            CHECK(g.result->edges[14 + i].vertex(0) == pe[i].first);
            CHECK(g.result->edges[14 + i].vertex(1) == pe[i].second);
        }
        // The re-spanned ring edges — the second link's corners'
        // four, read from the CURRENT stored pairs (the first
        // link's feet the surviving ends): old 0 (bottom x front)
        // (0 -> 8); old 1 (bottom x right, ALREADY re-spanned by
        // che#2) (10 -> 6); old 4 (top x front) (2 -> 9); old 5
        // (top x right, already re-spanned) (11 -> 7).
        const std::size_t rs_idx[4] = {0, 1, 4, 5};
        const std::pair<std::size_t, std::size_t> rs[4] = {
            {0, 8}, {10, 6}, {2, 9}, {11, 7}};
        for (std::size_t i = 0; i < 4; ++i) {
            CAPTURE(i);
            CHECK(g.result->edges[rs_idx[i]].vertex(0) ==
                  rs[i].first);
            CHECK(g.result->edges[rs_idx[i]].vertex(1) ==
                  rs[i].second);
        }
        // The first link's born, remapped and kept: che#2's feet
        // are the result's 4..7 (the survivors' compaction), the
        // wires the result's 10/11, the segments 12/13.
        CHECK(close(g.result->vertices[6].position().x, 1.0, 1e-12));
        CHECK(close(g.result->vertices[6].position().y, 0.5, 1e-12));
    }

    // THE VOLUME — the two disjoint prisms, the closed form per
    // link: V = 8 - 2*d^2 = 7.5 at d = 0.5, all exact.
    {
        const auto mp = b::mass_properties(
            *g.result, CAD_0::math::Tolerance::linear(1e-9));
        CHECK(close(mp.volume, 8.0 - 2.0 * kD * kD, 1e-12));
        CHECK(close(8.0 - mp.volume, 2.0 * (kD * kD), 1e-12));
        CHECK(mp.all_exact);
        // THE NAIVE READS, red by discrimination: the single
        // chamfer's 7.75 (the sibling's volume — what consuming
        // the PRISTINE box would have produced), the square-cut
        // chain (8 - 4*d^2 = 7.0), and the FILLET chain's own
        // 7 + pi/4 (a different geometry, never the flat bevel's).
        CHECK_FALSE(close(mp.volume, 8.0 - kD * kD, 1e-6));
        CHECK_FALSE(close(mp.volume, 8.0 - 4.0 * kD * kD, 1e-6));
        CHECK_FALSE(close(mp.volume, 7.0 + kPi / 4.0, 1e-3));
    }

    // The provenance: exact throughout.
    for (const auto& v : g.result->vertices)
        CHECK(v.provenance().is_exact());
    for (const auto& e : g.result->edges)
        CHECK(e.provenance().is_exact());
    for (const auto& f : g.result->solid.shell().faces())
        CHECK(f.provenance().is_exact());

    // THE NAMES — the second bevel born under che#3 with the
    // ROOT's vocabulary: the operands the CURRENT faces' stamps —
    // the FRONT (untouched) and the RIGHT (the first chamfer's
    // shrink face, absorbed and re-windowed, its box stamp KEPT:
    // the eternally-antecedent leg, ADR-0036 unamended). The
    // first link's names held VERBATIM (the stamp kept, D8.2's own
    // words) — the bevel a kept face under its own name, the
    // wires at their remapped indices.
    {
        const std::string bevel1 =
            "che#2:bevel[box#1:back × box#1:right]";
        const std::string bevel2 =
            "che#3:bevel[box#1:front × box#1:right]";
        CHECK(b::query_face(*g.result, 6).name == bevel1);
        CHECK(b::query_face(*g.result, 7).name == bevel2);
        CHECK(g.result->solid.shell().faces()[7].shape_id()
                  .feature == FeatureId{3});
        // The second link's setback wires: the level absent, the
        // landing faces the CURRENT operands.
        CHECK(b::query_edge(*g.result, 14).name ==
              "che#3:isect[box#1:front × " + bevel2 + "]");
        CHECK(b::query_edge(*g.result, 15).name ==
              "che#3:isect[box#1:right × " + bevel2 + "]");
        // The first link's wires, kept verbatim at the remapped
        // indices 10/11 (the back's and the right's landings).
        CHECK(b::query_edge(*g.result, 10).name ==
              "che#2:isect[box#1:back × " + bevel1 + "]");
        CHECK(b::query_edge(*g.result, 11).name ==
              "che#2:isect[box#1:right × " + bevel1 + "]");
        // The segments honestly unnamed; the shrink faces keep
        // their names (the box's, through BOTH absorptions).
        CHECK(b::query_edge(*g.result, 16).name.empty());
        CHECK(b::query_edge(*g.result, 17).name.empty());
        CHECK(b::query_face(*g.result, 5).name == "box#1:right");
        CHECK(b::query_face(*g.result, 2).name == "box#1:front");
        // The by-name queries answer the addresses (the name
        // plane's door through the chain): the bevel's own stamp,
        // taken from the face record, finds the face.
        const auto& sid =
            g.result->solid.shell().faces()[7].shape_id();
        CHECK(b::query_face_by_name(*g.result, sid) ==
              std::vector<b::FaceId>{7});
    }

    // THE SECOND BEVEL'S SURFACE — the bisector of the CURRENT
    // operand normals: (1,-1,0)/sqrt(2) through (0.75,-0.75,0);
    // the window the rectangle in its own frame; the walk's
    // co-edge table (the mirrored orientation: this bevel's
    // v-axis reads the feet in the OPPOSITE order — the CCW cycle
    // walks the segment REVERSED first: 16R, 15F, 17F, 14R).
    {
        const auto* surf =
            g.result->solid.shell().faces()[7].surface();
        REQUIRE(surf != nullptr);
        CHECK(surf->family() == b::SurfaceFamily::Plane);
        const auto* pl =
            dynamic_cast<const b::PlaneSurface*>(surf);
        REQUIRE(pl != nullptr);
        CHECK(close(pl->normal_vec().x, 1.0 / std::sqrt(2.0), 1e-12));
        CHECK(close(pl->normal_vec().y, -1.0 / std::sqrt(2.0), 1e-12));
        CHECK(close(pl->normal_vec().z, 0.0, 1e-12));
        CHECK(close(pl->origin().x, 0.75, 1e-12));
        CHECK(close(pl->origin().y, -0.75, 1e-12));
        CHECK(close(pl->origin().z, 0.0, 1e-12));
        double u_min = 1e9, u_max = -1e9, v_min = 1e9, v_max = -1e9;
        for (const auto& ce :
             g.result->solid.shell().faces()[7].loops()[0]
                 .coedges()) {
            const auto* pc = ce.p_curve();
            REQUIRE(pc != nullptr);
            REQUIRE(pc->kind == b::PCurve::Kind::Segment);
            u_min = std::min({u_min, pc->segment.a.x,
                              pc->segment.b.x});
            u_max = std::max({u_max, pc->segment.a.x,
                              pc->segment.b.x});
            v_min = std::min({v_min, pc->segment.a.y,
                              pc->segment.b.y});
            v_max = std::max({v_max, pc->segment.a.y,
                              pc->segment.b.y});
        }
        CHECK(close(u_min, -1.0, 1e-12));
        CHECK(close(u_max, 1.0, 1e-12));
        CHECK(close(v_min, -kD / std::sqrt(2.0), 1e-12));
        CHECK(close(v_max, kD / std::sqrt(2.0), 1e-12));
        const auto& ces =
            g.result->solid.shell().faces()[7].loops()[0]
                .coedges();
        REQUIRE(ces.size() == 4);
        CHECK(ces[0].edge() == 16);
        CHECK(ces[0].reversed() == true);
        CHECK(ces[1].edge() == 15);
        CHECK(ces[1].reversed() == false);
        CHECK(ces[2].edge() == 17);
        CHECK(ces[2].reversed() == false);
        CHECK(ces[3].edge() == 14);
        CHECK(ces[3].reversed() == true);
        // ALL LINES everywhere (the flat bevel's signature through
        // the chain): no Arc in the whole body.
        for (std::size_t f = 0; f < 8; ++f)
            for (const auto& loop :
                 g.result->solid.shell().faces()[f].loops())
                for (const auto& ce : loop.coedges()) {
                    REQUIRE(ce.p_curve() != nullptr);
                    CHECK(ce.p_curve()->kind ==
                          b::PCurve::Kind::Segment);
                }
        // The endpoint coherence: every co-edge's p-curve images
        // land on its vertices (the decoupling rule through the
        // second link).
        for (std::size_t f = 0; f < 8; ++f) {
            const auto& face =
                g.result->solid.shell().faces()[f];
            const auto* s2 = face.surface();
            for (const auto& loop : face.loops())
                for (const auto& ce : loop.coedges()) {
                    const auto* pc = ce.p_curve();
                    REQUIRE(pc != nullptr);
                    const auto s0 = static_cast<std::size_t>(
                        ce.start_vertex(g.result->edges));
                    const auto s1 = static_cast<std::size_t>(
                        ce.end_vertex(g.result->edges));
                    const auto q0 = s2->evaluate(
                        pc->segment.a.x, pc->segment.a.y);
                    const auto q1 = s2->evaluate(
                        pc->segment.b.x, pc->segment.b.y);
                    CHECK((q0 - g.result->vertices[s0].position())
                              .length_sq() < 1e-18);
                    CHECK((q1 - g.result->vertices[s1].position())
                              .length_sq() < 1e-18);
                }
        }
    }

    // THE WIDENING DECLARED (the brief's own law): the m1
    // signature's frozen triple REFUSED this carrier — the old
    // predicate, asserted false; the m2 signature admits it —
    // both said, never silent (the deeper declaration lives in the
    // m1 file where the old refusal stood).
    {
        const bool old_signature =
            g.first->vertex_count() == 8 &&
            g.first->edge_count() == 12 &&
            g.first->face_count() == 6;
        CHECK_FALSE(old_signature);
    }
}

// ===========================================================================
// THE ORACLE — the m3 law: the chain's volume re-derived with THIS
// FILE's own equations, the disjointness checked with the written
// geometry, the doctored reads refused
// ===========================================================================
TEST_CASE("G6 m2 the oracle: the two prisms disjoint by the written "
          "geometry, the volume by the file's own 2e6-midpoint rule, "
          "the doctored reads red") {
    const double d1 = kD, d2 = kD;

    // THE DISJOINTNESS, written (D1.3's own condition): the two
    // retreat strips on the shared face x=1 live at opposite ends
    // — che#2's [1-d1, 1] and che#3's [-1, -1+d2] in y; the gap
    // between them (2 - d1 - d2) must be POSITIVE (the strips not
    // meeting), the D1.3 sum law satisfied with the written
    // geometry, never assumed.
    const double gap = (1.0 - d1) - (-1.0 + d2);
    CHECK(gap == 2.0 - d1 - d2);
    CHECK(gap > 0.0);

    // THE FILE'S OWN QUADRATURE (the landed road-3 idiom): each
    // prism's cross-section int_0^d (d-u) du over the arm 2 — two
    // 1-D midpoint rules, 2e6 samples each, the affine integrands.
    const int N = 2'000'000;
    const auto section = [&](double d) {
        const double hstep = d / N;
        double s = 0.0;
        for (int i = 0; i < N; ++i)
            s += (d - (i + 0.5) * hstep) * hstep;
        return s;
    };
    const double removed = 2.0 * (section(d1) + section(d2));
    const double v_quad = 8.0 - removed;

    // Against the closed form at the ADR's declared bar, and the
    // kernel's mass at the exactness pin.
    CHECK(close(v_quad, 8.0 - d1 * d1 - d2 * d2, 2e-6));
    const ChainGuide g;
    REQUIRE(g.result != nullptr);
    const auto mp = b::mass_properties(
        *g.result, CAD_0::math::Tolerance::linear(1e-9));
    CHECK(close(mp.volume, 8.0 - d1 * d1 - d2 * d2, 1e-12));
    CHECK(close(v_quad, mp.volume, 2e-6));

    // THE DOCTORED READS, each refused by the very comparison the
    // golden passes:
    // (a) THE OVERLAP DOCTORED — the merged-strip naive: the two
    //     strips read as ONE prism of depth d1+d2 (the disjointness
    //     forgotten): removed (d1+d2)^2 = 1.0, NOT the truth.
    const double v_merged = 8.0 - (d1 + d2) * (d1 + d2);
    CHECK_FALSE(close(v_merged, mp.volume, 1e-6));
    // (b) THE SIBLING FORGOTTEN — one prism alone.
    CHECK_FALSE(close(8.0 - d1 * d1, mp.volume, 1e-6));
    // (c) THE SQUARE-CUT — each section as a square.
    CHECK_FALSE(close(8.0 - 2.0 * (2.0 * d1 * d1), mp.volume, 1e-6));
    // (d) THE SETBACK ALTERED (+0.01 on the second link's rule).
    CHECK_FALSE(close(8.0 - d1 * d1 - (d2 + 0.01) * (d2 + 0.01),
                      mp.volume, 1e-6));
    // (e) THE OFFSET HALVED — the second bevel's plane read at
    //     d2/2 (the naive offset the m1 counterproof doctored).
    CHECK_FALSE(close(8.0 - d1 * d1 - (0.5 * d2) * (0.5 * d2),
                      mp.volume, 1e-6));
    // (f) THE FILLET'S OWN CHAIN — a different geometry's number.
    CHECK_FALSE(close(7.0 + kPi / 4.0, mp.volume, 1e-3));
}

// ===========================================================================
// THE TAVOLA — the second link's descent (the first bevel a KEPT
// face row under its own name), beside the first link's own
// ===========================================================================
TEST_CASE("G6 m2 the tavola: the second link's 41 rows — the kept "
          "bevel, the absorbed quartet, the born four, the vanished "
          "vertical with its cause; the first link's 35 beside it") {
    const ChainGuide g;
    REQUIRE(g.result != nullptr);

    // THE FIRST LINK's own tavola (the m1 shape, the chain's step
    // evidence): 35 rows.
    const b::DescentTable t1 = b::chamfer_descent(*g.first, *g.box);
    REQUIRE(t1.rows.size() == 35);
    CHECK(t1.count(b::Descent::Vanished) == 3);

    // THE SECOND LINK's tavola: 41 rows — the result's 12 + 18 + 8
    // = 38, plus the vanished tail 1 edge + 2 vertices.
    const b::DescentTable t =
        b::chamfer_descent(*g.result, *g.first);
    REQUIRE(t.rows.size() == 41);
    CHECK(t.count(b::Descent::Kept) == 21);      // 8 V + 10 E + 3 F
    CHECK(t.count(b::Descent::Absorbed) == 8);   // 4 E + 4 F
    CHECK(t.count(b::Descent::Born) == 9);       // 4 V + 4 E + 1 F
    CHECK(t.count(b::Descent::Vanished) == 3);   // 2 V + 1 E
    CHECK(t.count(b::Descent::Split) == 0);
    CHECK(t.count(b::Descent::Merged) == 0);
    {
        std::size_t kept_f = 0, abs_f = 0, born_f = 0;
        std::size_t kept_v = 0, born_v = 0, van_v = 0;
        std::size_t kept_e = 0, abs_e = 0, born_e = 0, van_e = 0;
        for (const auto& row : t.rows) {
            const bool k = row.relation == b::Descent::Kept;
            const bool a = row.relation == b::Descent::Absorbed;
            const bool bo = row.relation == b::Descent::Born;
            const bool va = row.relation == b::Descent::Vanished;
            if (row.entity == b::EntityKind::Vertex) {
                kept_v += k; born_v += bo; van_v += va;
            } else if (row.entity == b::EntityKind::Edge) {
                kept_e += k; abs_e += a; born_e += bo; van_e += va;
            } else {
                kept_f += k; abs_f += a; born_f += bo;
            }
        }
        CHECK(kept_v == 8);  CHECK(born_v == 4);  CHECK(van_v == 2);
        CHECK(kept_e == 10); CHECK(abs_e == 4);
        CHECK(born_e == 4);  CHECK(van_e == 1);
        CHECK(kept_f == 3);  CHECK(abs_f == 4);  CHECK(born_f == 1);
    }

    // The vanished rows' cause, verbatim: "replaced by the bevel"
    // — per link, the same act.
    {
        std::size_t causes = 0;
        for (const auto& row : t.rows) {
            if (row.relation != b::Descent::Vanished) continue;
            CHECK(row.cause == "replaced by the bevel");
            if (row.entity == b::EntityKind::Edge) {
                // The dead vertical1: the FIRST body's edge 9, its
                // box stamp carried by the row.
                CHECK(row.address == 9);
                CHECK(row.shape_id ==
                      g.first->edges[9].shape_id());
            }
            ++causes;
        }
        CHECK(causes == 3);
    }

    // THE FIRST BEVEL A KEPT FACE ROW UNDER ITS OWN NAME (the
    // stamp held — D8.2's "il timbro tenuto"): the second link
    // does not touch it, and the tavola says so by name.
    {
        bool found = false;
        for (const auto& row : t.rows) {
            if (row.relation != b::Descent::Kept ||
                row.entity != b::EntityKind::Face)
                continue;
            if (row.name ==
                "che#2:bevel[box#1:back × box#1:right]") {
                found = true;
                REQUIRE(!row.operands.empty());
                CHECK(row.operands[0].kind ==
                      b::EntityKind::Face);
                CHECK(row.operands[0].address == 6);  // the first
                                                      // body's bevel
            }
        }
        CHECK(found);
    }

    // The born rows' names: the second bevel and its two wires.
    for (const auto& row : t.rows) {
        if (row.relation != b::Descent::Born) continue;
        if (row.entity == b::EntityKind::Face)
            CHECK(row.name ==
                  "che#3:bevel[box#1:front × box#1:right]");
        if (row.entity == b::EntityKind::Edge && !row.name.empty())
            CHECK(row.name.rfind("che#3:isect[", 0) == 0);
    }

    // THE CLOSURE, both sides green (the hard invariant, per
    // link).
    std::string why;
    CHECK(b::descent_closed(*g.result, t, &why));
    {
        const b::BRepBody no_second{};
        CHECK(b::old_generation_closed(*g.first, no_second, t,
                                       &why));
    }

    // Determinism of the derivation itself.
    const b::DescentTable t2 =
        b::chamfer_descent(*g.result, *g.first);
    REQUIRE(t2.rows.size() == t.rows.size());
    for (std::size_t i = 0; i < t.rows.size(); ++i) {
        CHECK(t.rows[i].relation == t2.rows[i].relation);
        CHECK(t.rows[i].entity == t2.rows[i].entity);
        CHECK(t.rows[i].address == t2.rows[i].address);
        CHECK(t.rows[i].name == t2.rows[i].name);
        CHECK(t.rows[i].cause == t2.rows[i].cause);
        CHECK(t.rows[i].operands == t2.rows[i].operands);
        CHECK(t.rows[i].shape_id == t2.rows[i].shape_id);
    }
}

// ===========================================================================
// THE GATE — the chain gate's five predicates AFTER EACH STEP
// ===========================================================================
TEST_CASE("G6 m2 the gate: the five predicates green after EVERY "
          "step — the census by name with the held references, the "
          "drift zero at the re-run's digest") {
    const ChainGuide g;
    REQUIRE(g.first != nullptr);
    REQUIRE(g.result != nullptr);
    const b::DescentTable t1 = b::chamfer_descent(*g.first, *g.box);
    const b::DescentTable t2 =
        b::chamfer_descent(*g.result, *g.first);

    // The re-run (the drift's own subject): the whole chain from
    // the recipes, fresh bodies.
    const ChainGuide rerun;
    REQUIRE(rerun.result != nullptr);

    // STEP 1 — the box to the first chamfer (the m1 step, gated
    // again as the chain's own first step).
    {
        b::ChainStep step;
        step.body = g.first.get();
        step.tavola = &t1;
        step.previous = g.box.get();
        step.held.push_back(
            {ShapeId{FeatureId{1}, SubshapeRef{}.push(10), {}},
             b::Resolution::Kind::Vanished});
        step.held.push_back(
            {ShapeId{FeatureId{1}, SubshapeRef{}.push(11), {}},
             b::Resolution::Kind::Resolved});
        step.rerun_digest = b::chain_digest(*rerun.first);
        const b::ChainGateVerdict v = b::chain_gate(step);
        CHECK(v.pass);
        CHECK(v.failures.empty());
        CHECK(v.radial_two_opposite);
        CHECK(v.loops_closed);
        CHECK(v.chi_even_orientable);
        CHECK(v.census_by_name);
        CHECK(v.drift_zero);
        CHECK(v.not_applicable.empty());
        CHECK(v.digest == b::chain_digest(*g.first));
    }

    // STEP 2 — the first chamfer to the chain: the dead vertical1
    // VANISHED, the surviving vertical3 RESOLVED, and che#2's
    // right-face wire — a born name of the FIRST link — held
    // across the second (the stamp kept, RESOLVED).
    {
        b::ChainStep step;
        step.body = g.result.get();
        step.tavola = &t2;
        step.previous = g.first.get();
        step.held.push_back(
            {ShapeId{FeatureId{1}, SubshapeRef{}.push(9), {}},
             b::Resolution::Kind::Vanished});
        step.held.push_back(
            {ShapeId{FeatureId{1}, SubshapeRef{}.push(11), {}},
             b::Resolution::Kind::Resolved});
        step.held.push_back(
            {g.first_wire, b::Resolution::Kind::Resolved});
        step.rerun_digest = b::chain_digest(*rerun.result);
        const b::ChainGateVerdict v = b::chain_gate(step);
        CHECK(v.pass);
        CHECK(v.failures.empty());
        CHECK(v.radial_two_opposite);
        CHECK(v.loops_closed);
        CHECK(v.chi_even_orientable);
        CHECK(v.census_by_name);
        CHECK(v.drift_zero);
        CHECK(v.not_applicable.empty());
        CHECK(v.v == 12);
        CHECK(v.e == 18);
        CHECK(v.f == 8);
        CHECK(v.h == 0);
        CHECK(v.chi == 2);
        CHECK(v.digest == b::chain_digest(*g.result));
    }

    // THE DIGEST'S PURITY: the names a pure function of the recipe
    // — another instance of the SAME chain, the same names byte
    // for byte; the structure digest bit-identical at the re-run.
    CHECK(name_digest(*rerun.result) == name_digest(*g.result));
    CHECK(structure_digest(*rerun.result) ==
          structure_digest(*g.result));
    for (int i = 0; i < 3; ++i) {
        const ChainGuide again;
        REQUIRE(again.result != nullptr);
        CHECK(structure_digest(*again.result) ==
              structure_digest(*g.result));
    }
}

// ===========================================================================
// THE STRIP LAW — D1.3's condition and D5.1's stage band, both
// alive for the first time (the m1 single-edge band's own tests
// IMMOBILE in the m1 file: a lone 1.5 still admits there)
// ===========================================================================
TEST_CASE("G6 m2 the strip law: the stage band d < 1 with the two "
          "readings in the payload, the confine 0.99 admitted, the "
          "sum law red on the thin box with the tightened bound") {
    // (1.01 as the SECOND, 0.5 the first): the faces' CURRENT
    // reach would admit it (the right face's tightened 1.5 — the
    // m1 band, satisfied), but the STAGE band refuses (the strips
    // of the opposing chamfers must not meet, 2d < 2; at d = 1 the
    // six planes die to points) — BOTH READINGS in the payload,
    // the wall's name the same chamfer_setback_exceeds_face.
    {
        const ChainGuide g(1.01, 0.5, /*third=*/false);
        REQUIRE(g.first != nullptr);
        bool threw = false;
        try {
            const b::ChamferNaming ctx3{FeatureId{3},
                                        &n::kBoxNaming};
            auto r = b::chamfer_edge(*g.first, g.second, 1.01,
                                     ctx3);
            (void)r;
        } catch (const std::invalid_argument& e) {
            threw = true;
            const std::string what = e.what();
            CHECK(what.find("chamfer_setback_exceeds_face") !=
                  std::string::npos);
            // THE TWO READINGS: the asked setback, the stage's
            // bound 1, and the faces' current reach 1.5 (the
            // first strip's retreat — the m1 band's own reading,
            // SATISFIED, said in the same breath).
            CHECK(what.find("1.010000") != std::string::npos);
            CHECK(what.find("1.000000") != std::string::npos);
            CHECK(what.find("1.500000") != std::string::npos);
        }
        REQUIRE(threw);
    }

    // THE CONFINE (0.99, 0.99): both walls just passed — the stage
    // band (0.99 < 1 - eps) and the sum law (0.99 + 0.99 = 1.98 <
    // 2, the shared face's extent) — the chain lands, the counts
    // the formula's own, the volume the closed form's.
    {
        const ChainGuide g(0.99, 0.99);
        REQUIRE(g.result != nullptr);
        REQUIRE(g.result->vertex_count() == 12);
        REQUIRE(g.result->edge_count() == 18);
        REQUIRE(g.result->face_count() == 8);
        CHECK(b::euler_characteristic(*g.result) == 2);
        const auto mp = b::mass_properties(
            *g.result, CAD_0::math::Tolerance::linear(1e-9));
        CHECK(close(mp.volume, 8.0 - 2.0 * 0.99 * 0.99, 1e-9));
        CHECK(mp.all_exact);
    }

    // THE SUM LAW ALONE (D1.3's own condition, the stage band
    // silent): the THIN box {1, 0.4, 1} — the shared face x=1
    // spans y in [-0.4, 0.4], the extent 0.8: the strips of
    // (0.5, 0.5) MEET (0.5 + 0.5 >= 0.8), and the second chamfer's
    // band reads the CURRENT face's reach — the first strip's
    // retreat TIGHTENED it to 0.8 - 0.5 = 0.3 — the bound carried,
    // the refusal the m1 machinery's own (the wall that needed NO
    // new code: the reach is read from the payload).
    {
        auto thin = b::make_box({1.0, 0.4, 1.0}, FeatureId{1});
        const ShapeId v2{FeatureId{1}, SubshapeRef{}.push(10), {}};
        const ShapeId v1{FeatureId{1}, SubshapeRef{}.push(9), {}};
        const b::ChamferNaming ctx2{FeatureId{2},
                                    &n::kBoxNaming};
        auto first = b::chamfer_edge(*thin, v2, 0.5, ctx2);
        REQUIRE(first != nullptr);
        bool threw = false;
        try {
            const b::ChamferNaming ctx3{FeatureId{3},
                                        &n::kBoxNaming};
            auto r = b::chamfer_edge(*first, v1, 0.5, ctx3);
            (void)r;
        } catch (const std::invalid_argument& e) {
            threw = true;
            const std::string what = e.what();
            CHECK(what.find("chamfer_setback_exceeds_face") !=
                  std::string::npos);
            CHECK(what.find("0.500000") != std::string::npos);
            CHECK(what.find("0.300000") != std::string::npos);
        }
        REQUIRE(threw);
        // In-band on the same thin face: (0.3, 0.3) — the sum 0.6
        // < 0.8 — the chain lands, the volume the two thin
        // prisms' closed form (the box's volume 2*0.8*2 = 3.2,
        // each prism d^2/2 * the arm 2).
        auto first2 = b::chamfer_edge(*thin, v2, 0.3, ctx2);
        REQUIRE(first2 != nullptr);
        const b::ChamferNaming ctx3{FeatureId{3},
                                    &n::kBoxNaming};
        auto ok = b::chamfer_edge(*first2, v1, 0.3, ctx3);
        REQUIRE(ok != nullptr);
        REQUIRE(ok->vertex_count() == 12);
        REQUIRE(ok->edge_count() == 18);
        REQUIRE(ok->face_count() == 8);
        const auto mp = b::mass_properties(
            *ok, CAD_0::math::Tolerance::linear(1e-9));
        CHECK(close(mp.volume, 3.2 - 2.0 * (0.3 * 0.3), 1e-12));
        CHECK(mp.all_exact);
    }

    // The m1 single-edge band's own world, untouched by the stage
    // law: a LONE chamfer at 1.5 on the pristine box admits (the
    // m1 test's own pin, restated here as the chain's boundary —
    // the stage band binds the MULTI-EDGE state only).
    {
        auto box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        const ShapeId v2{FeatureId{1}, SubshapeRef{}.push(10), {}};
        const b::ChamferNaming ctx{FeatureId{2},
                                   &n::kBoxNaming};
        auto lone = b::chamfer_edge(*box, v2, 1.5, ctx);
        REQUIRE(lone != nullptr);
        REQUIRE(lone->vertex_count() == 10);
    }
}

// ===========================================================================
// THE WALLS — the dead reference, the 45-degree dihedral, the
// impostors (the hexagonal prism, the fillet chain, the sphere,
// the blind body)
// ===========================================================================
TEST_CASE("G6 m2 the walls: the dead reference, the first wire's "
          "45-degree dihedral, the hexagonal prism's bevel-stamp "
          "refusal, the fillet chain by family") {
    const ChainGuide g(kD, kD, /*third=*/false);
    REQUIRE(g.first != nullptr);
    const b::ChamferNaming ctx{FeatureId{3}, &n::kBoxNaming};

    // THE DEAD REFERENCE (the VANISHED class at the kernel): che#2
    // consumed vertical2 — the structural find against the CURRENT
    // state's stamps answers nothing.
    {
        bool threw = false;
        try {
            auto r = b::chamfer_edge(*g.first, g.dead, 0.25, ctx);
            (void)r;
        } catch (const std::invalid_argument& e) {
            threw = true;
            CHECK(std::string(e.what()).find(
                      "names no edge of this body") !=
                  std::string::npos);
        }
        REQUIRE(threw);
    }

    // THE 45-DEGREE WALL (the chamfer's own family wall — the
    // fillet's m6ter had the curved flank, the flat bevel's
    // adjacent is a PLANE at 45 degrees): che#2's right-face
    // setback wire IS vertical (its endpoints share (x, y)) and
    // both its adjacents are PLANES — but the bevel's bisector
    // normal is at 45 degrees to the face's own, and the class's
    // exact dihedral arithmetic refuses (the general-dihedral
    // family, registered — the stage 2 seat, never smuggled).
    {
        bool threw = false;
        try {
            auto r = b::chamfer_edge(*g.first, g.first_wire, 0.25,
                                     ctx);
            (void)r;
        } catch (const std::invalid_argument& e) {
            threw = true;
            const std::string what = e.what();
            CHECK(what.find("dihedral is not the class's 90") !=
                  std::string::npos);
        }
        REQUIRE(threw);
    }

    // THE HEXAGONAL PRISM (the all-planar impostor): 12/18/8,
    // every plane, every count MATCHING the chain's k = 2 — the
    // bevel-stamp wall refuses (0 bevels where the counts declare
    // 2; the identity is where a thing was born).
    {
        auto prism =
            b::make_prism(hexagon_chains(), 2.0f, +1, FeatureId{3});
        REQUIRE(prism != nullptr);
        REQUIRE(prism->vertex_count() == 12);
        REQUIRE(prism->edge_count() == 18);
        REQUIRE(prism->face_count() == 8);
        bool threw = false;
        try {
            auto r = b::chamfer_edge(
                *prism,
                ShapeId{FeatureId{3}, SubshapeRef{}.push(0), {}},
                0.1, ctx);
            (void)r;
        } catch (const std::invalid_argument& e) {
            threw = true;
            const std::string what = e.what();
            CHECK(what.find(
                      "bevel-stamped faces where the chain's "
                      "counts declare") != std::string::npos);
            CHECK(what.find("HEXAGONAL PRISM") !=
                  std::string::npos);
        }
        REQUIRE(threw);
    }

    // THE FILLET CHAIN (the mixed corner's world): two fillets
    // answer the SAME counts 12/18/8 — but the flanks are
    // CYLINDERS, and the planarity wall refuses by family (D5.3,
    // the crease merge's seat).
    {
        auto box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        const ShapeId v2{FeatureId{1}, SubshapeRef{}.push(10), {}};
        const ShapeId v0{FeatureId{1}, SubshapeRef{}.push(8), {}};
        const b::FilletNaming fctx2{FeatureId{2},
                                    &n::kBoxNaming};
        auto f1 = b::fillet_edge(*box, v2, 0.5, fctx2);
        REQUIRE(f1 != nullptr);
        const b::FilletNaming fctx3{FeatureId{3},
                                    &n::kBoxNaming};
        auto f2 = b::fillet_edge(*f1, v0, 0.5, fctx3);
        REQUIRE(f2 != nullptr);
        REQUIRE(f2->vertex_count() == 12);
        REQUIRE(f2->edge_count() == 18);
        REQUIRE(f2->face_count() == 8);
        bool threw = false;
        try {
            auto r = b::chamfer_edge(
                *f2, ShapeId{FeatureId{1},
                             SubshapeRef{}.push(9), {}},
                0.25, ctx);
            (void)r;
        } catch (const std::invalid_argument& e) {
            threw = true;
            CHECK(std::string(e.what()).find(
                      "outside the class families") !=
                  std::string::npos);
        }
        REQUIRE(threw);
    }

    // The sphere (the counts) and the blind body (12/17/8 — the
    // edge count off by the formula's own arithmetic).
    {
        const auto sphere = b::make_sphere(1.0, {}, FeatureId{1});
        try {
            auto r = b::chamfer_edge(
                *sphere, ShapeId{FeatureId{1},
                                 SubshapeRef{}.push(0), {}},
                0.5, ctx);
            (void)r;
            FAIL("the sphere must refuse");
        } catch (const std::invalid_argument& e) {
            CHECK(std::string(e.what()).find(
                      "outside the class signature") !=
                  std::string::npos);
        }
        std::vector<s::SketchChain> chains;
        chains.push_back(s::SketchChain{
            {s::sketch_circle({0.0f, 0.0f}, 0.25f, false)}});
        auto tool = b::make_prism(chains, 3.0f, +1, FeatureId{3});
        const b::SubtractNaming sub_ctx{
            FeatureId{4}, &n::kBoxNaming, &n::kExtNaming};
        auto box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        auto blind = b::exact_subtract(*box, *tool,
                                       {0.0, 0.0, -1.5},
                                       b::kSubtractTolerance,
                                       sub_ctx);
        REQUIRE(blind != nullptr);
        try {
            auto r = b::chamfer_edge(
                *blind,
                ShapeId{FeatureId{1}, SubshapeRef{}.push(10), {}},
                0.1, ctx);
            (void)r;
            FAIL("the blind body must refuse");
        } catch (const std::invalid_argument& e) {
            CHECK(std::string(e.what()).find(
                      "outside the class signature") !=
                  std::string::npos);
        }
    }
}

// ===========================================================================
// THE FRAME FROM THE PAYLOAD — the m1 lesson holding the SECOND
// application (the chain on the translated box)
// ===========================================================================
TEST_CASE("G6 m2 the frame from the payload: the chain on the "
          "translated box — the same closed forms, the same names, "
          "the world's origin never consulted") {
    auto box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
    b::translate_body(*box, {3.0, -2.0, 5.0});
    const ShapeId v2{FeatureId{1}, SubshapeRef{}.push(10), {}};
    const ShapeId v1{FeatureId{1}, SubshapeRef{}.push(9), {}};
    const b::ChamferNaming ctx2{FeatureId{2}, &n::kBoxNaming};
    auto first = b::chamfer_edge(*box, v2, 0.5, ctx2);
    REQUIRE(first != nullptr);
    const b::ChamferNaming ctx3{FeatureId{3}, &n::kBoxNaming};
    auto result = b::chamfer_edge(*first, v1, 0.5, ctx3);
    REQUIRE(result != nullptr);
    REQUIRE(result->vertex_count() == 12);
    REQUIRE(result->edge_count() == 18);
    REQUIRE(result->face_count() == 8);
    CHECK(b::euler_characteristic(*result) == 2);
    CHECK(b::is_closed_orientable(*result));
    CHECK(b::loops_are_closed(*result));
    // The second link's born feet: the TRANSLATED corner's own —
    // (3.5, -3, 4) and (4, -2.5, 4) at the low end — the payload's
    // reads, never the world's (1,-1)-corner.
    CHECK(close(result->vertices[8].position().x, 3.5, 1e-12));
    CHECK(close(result->vertices[8].position().y, -3.0, 1e-12));
    CHECK(close(result->vertices[8].position().z, 4.0, 1e-12));
    CHECK(close(result->vertices[10].position().x, 4.0, 1e-12));
    CHECK(close(result->vertices[10].position().y, -2.5, 1e-12));
    // The names identical (the recipe's pure function, the chain
    // two links deep on a translated frame).
    CHECK(b::query_face(*result, 7).name ==
          "che#3:bevel[box#1:front × box#1:right]");
    CHECK(b::query_face(*result, 6).name ==
          "che#2:bevel[box#1:back × box#1:right]");
    // The volume the golden's own.
    const auto mp = b::mass_properties(
        *result, CAD_0::math::Tolerance::linear(1e-9));
    CHECK(close(mp.volume, 8.0 - 2.0 * 0.25, 1e-12));
    CHECK(mp.all_exact);
    // The tavola closes on the translated chain too.
    const b::DescentTable t =
        b::chamfer_descent(*result, *first);
    std::string why;
    CHECK(b::descent_closed(*result, t, &why));
    CHECK(t.count(b::Descent::Vanished) == 3);
    // The strip law holds the translated frame: the stage band's
    // refusal carries the same readings (the payload's own
    // numbers, the tightened reach 1.5).
    {
        auto first2 = b::chamfer_edge(*box, v2, 0.5, ctx2);
        bool threw = false;
        try {
            auto r = b::chamfer_edge(*first2, v1, 1.01, ctx3);
            (void)r;
        } catch (const std::invalid_argument& e) {
            threw = true;
            const std::string what = e.what();
            CHECK(what.find("chamfer_setback_exceeds_face") !=
                  std::string::npos);
            CHECK(what.find("1.500000") != std::string::npos);
        }
        REQUIRE(threw);
    }
}
